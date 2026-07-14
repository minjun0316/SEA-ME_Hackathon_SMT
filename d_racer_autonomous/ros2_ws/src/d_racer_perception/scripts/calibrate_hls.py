#!/usr/bin/env python3
"""@file calibrate_hls.py
@brief 사진에서 '차선 유력 후보' 픽셀을 자동 추출 → 그 HLS 분포로 yellow/white 임계(lo/hi)를 역산.

@details
lane_detect의 색 마스크 임계(`yellow_lo/hi`, `white_lo/hi`)를 손으로 찍지 않고,
캡처한 프레임(예: `save_frames.py` 결과)에서 데이터로 계산한다. 조명이 바뀌면
그 조명에서 찍은 사진을 다시 넣기만 하면 새 임계가 나온다.

파이프라인(프레임마다):
  1. ROI 크롭(기본 하단 65% — 차선이 있는 영역, 상단 배경/사람/차량 배제).
  2. BGR→HLS.
  3. '유력 후보' 추출:
     - YELLOW: 노란 hue 밴드 안에서 **Otsu로 밝기(L) 이봉분포를 자동 분리**한다
       (노란 조명 먹은 어두운 바닥 vs 밝은 노란 페인트 → 밝은 쪽만 채택).
       추가로 채도 하한(Otsu 또는 절대하한 중 큰 값)으로 저채도(흰선 warm-tint) 컷.
     - WHITE: 저채도(S 낮음) 픽셀 중 Otsu로 밝은 집단만 채택(밝은 흰 선).
     - 둘 다 morphological open + 연결요소 면적필터로 흩뿌린 노이즈 제거(선형 구조만 남김).
  4. 여러 프레임의 후보 HLS를 모아 백분위수로 lo/hi를 정한다
     (lo = 낮은 분위 - 여유, hi = 높은 분위 + 여유; H,L,S 각각. 클램프).

혼합 폴더(노랑·흰 프레임 섞임)도 OK: 프레임별 후보 픽셀 수로 자동 판별해 각 색 표본에만 넣는다.

@par 사용
```bash
python3 calibrate_hls.py ~/bev_frames                    # 폴더 전체
python3 calibrate_hls.py f1.jpg f2.jpg --roi 0.4         # 특정 파일, ROI 상단40%부터
python3 calibrate_hls.py ~/bev_frames --bev config/lane.yaml   # lane.yaml의 bev_matrix로 BEV 변환 후 샘플(검출기와 동일 좌표)
python3 calibrate_hls.py ~/bev_frames --lo-pct 3 --hi-pct 99 --margin 5 --save-preview out/
```
출력: lane.yaml에 그대로 붙일 `yellow_lo/hi`, `white_lo/hi` (+ 진단 통계).

@note 색 값은 원근변환(BEV)에 거의 불변이라 raw 프레임 샘플로 충분하지만, 검출기와
      완전히 동일한 픽셀을 보려면 `--bev`로 같은 호모그래피를 적용해 샘플할 수 있다.
"""
from __future__ import annotations

import argparse
import glob
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml


# ---------------------------------------------------------------------------- #
def load_frames(paths: List[str]) -> List[Tuple[str, np.ndarray]]:
    """@brief 인자(파일/폴더)를 실제 이미지 목록으로 확장해 로드."""
    files: List[str] = []
    for p in paths:
        if os.path.isdir(p):
            for ext in ('*.jpg', '*.jpeg', '*.png'):
                files += glob.glob(os.path.join(p, ext))
        else:
            files.append(p)
    files = sorted(set(files))
    out = []
    for f in files:
        img = cv2.imread(f)
        if img is not None:
            out.append((os.path.basename(f), img))
    return out


def load_bev_matrix(yaml_path: str) -> Optional[np.ndarray]:
    """@brief lane.yaml에서 bev_matrix(9값)를 읽어 3x3 반환. 없으면 None."""
    with open(yaml_path, 'r', encoding='utf-8') as fh:
        doc = yaml.safe_load(fh) or {}
    # lane.yaml 형식: {lane_detect_node: {ros__parameters: {bev_matrix: [...]}}}
    params = (doc.get('lane_detect_node', {}) or {}).get('ros__parameters', {}) or {}
    m = params.get('bev_matrix')
    if not m or len(m) != 9 or not any(abs(float(x)) > 1e-12 for x in m):
        return None
    return np.array(m, dtype=np.float32).reshape(3, 3)


def otsu_threshold(values: np.ndarray) -> float:
    """@brief 1차원 값 집합의 Otsu 임계(이봉분포 분리점). 값이 부족하면 중앙값."""
    if values.size < 16:
        return float(np.median(values)) if values.size else 0.0
    v = values.astype(np.uint8).reshape(-1, 1)
    thr, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return float(thr)


def clean_mask(mask: np.ndarray, min_area: int, open_ksize: int) -> np.ndarray:
    """@brief morphological open 후 면적이 작은 연결요소 제거(흩뿌린 노이즈 컷, 선형만 유지)."""
    if open_ksize > 0:
        k = np.ones((open_ksize, open_ksize), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    if min_area > 0:
        n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = np.zeros_like(mask)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                keep[lab == i] = 255
        mask = keep
    return mask


# ---------------------------------------------------------------------------- #
def yellow_candidates(hls: np.ndarray, hue_band: Tuple[int, int],
                      s_abs_floor: int, min_area: int, open_ksize: int) -> np.ndarray:
    """@brief 노란 페인트 '유력 후보' 마스크.

    노란 hue 밴드 안에서 L을 Otsu로 이봉분리(어두운 바닥 vs 밝은 페인트)해 밝은 쪽만,
    채도는 (Otsu, 절대하한) 중 큰 값 이상만 채택 → 노란빛 바닥·저채도 warm-tint를 배제.
    """
    H, L, S = hls[:, :, 0], hls[:, :, 1], hls[:, :, 2]
    hue = (H >= hue_band[0]) & (H <= hue_band[1])
    if int(hue.sum()) < 50:
        return np.zeros(H.shape, np.uint8)
    l_thr = otsu_threshold(L[hue])                 # 바닥/페인트 밝기 분리점
    s_thr = max(float(s_abs_floor), otsu_threshold(S[hue]))
    cand = hue & (L >= l_thr) & (S >= s_thr)
    return clean_mask((cand.astype(np.uint8) * 255), min_area, open_ksize)


def white_candidates(hls: np.ndarray, s_white_max: int, bright_frac: float,
                     min_area: int, open_ksize: int) -> np.ndarray:
    """@brief 흰 페인트 '유력 후보' 마스크.

    흰 선은 '저채도 + 프레임 내 최고 밝기' 구조다. 저채도(S<=s_white_max) 픽셀의
    거의-최대 밝기(p99)를 '흰 선 밝기 기준'으로 잡고, 그 **bright_frac 배 이상**만
    채택한다. 저채도 파란/회색 바닥(중간 밝기)은 이 문턱 아래로 빠지고 진짜 흰 선만 남는다.

    분포 백분위(Otsu 포함)로는 바닥이 저채도 픽셀의 대다수라 문턱이 바닥 안쪽에 갇혀
    바닥을 물었다. '최대 밝기의 비율'은 표본 크기와 무관하게 '가장 밝은 것=흰 선'을
    기준으로 삼아 소수의 밝은 선을 안정적으로 분리한다(작은 클래스에 강건).
    """
    H, L, S = hls[:, :, 0], hls[:, :, 1], hls[:, :, 2]
    low_sat = S <= s_white_max
    if int(low_sat.sum()) < 50:
        return np.zeros(H.shape, np.uint8)
    l_ref = float(np.percentile(L[low_sat], 99))   # 저채도 최고밝기 ≈ 흰 선 레벨
    l_thr = bright_frac * l_ref
    cand = low_sat & (L >= l_thr)
    return clean_mask((cand.astype(np.uint8) * 255), min_area, open_ksize)


# ---------------------------------------------------------------------------- #
def bounds_from_samples(samples: np.ndarray, lo_pct: float, hi_pct: float,
                        margin: int) -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
    """@brief 후보 HLS 표본(N,3)에서 (lo, hi) 임계를 백분위수로 역산.

    @return ((H_lo,L_lo,S_lo), (H_hi,L_hi,S_hi)). H는 [0,180], L·S는 [0,255] 클램프.
    """
    def clamp(v, hi):
        return int(max(0, min(hi, round(v))))

    hmax = 180
    lo = []
    hi = []
    for ch, cap in ((0, hmax), (1, 255), (2, 255)):
        a = samples[:, ch]
        lo.append(clamp(np.percentile(a, lo_pct) - margin, cap))
        hi.append(clamp(np.percentile(a, hi_pct) + margin, cap))
    return (tuple(lo), tuple(hi))


def summarize(name: str, samples: np.ndarray) -> None:
    """@brief 표본 HLS 분포를 진단 출력."""
    print(f"\n[{name}] 후보 픽셀 {len(samples)}개")
    for i, ch in enumerate('HLS'):
        a = samples[:, i]
        print(f"  {ch}: p1={np.percentile(a,1):.0f} p5={np.percentile(a,5):.0f} "
              f"p50={np.percentile(a,50):.0f} p95={np.percentile(a,95):.0f} "
              f"p99={np.percentile(a,99):.0f}")


# ---------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='이미지 파일들 또는 폴더')
    ap.add_argument('--roi', type=float, default=0.35,
                    help='ROI 시작 y 비율(이 값부터 하단까지 샘플). 0=전체. 기본 0.35')
    ap.add_argument('--hue-band', type=int, nargs=2, default=(10, 45),
                    help='노랑 hue 탐색 밴드(넓게). 기본 10 45')
    ap.add_argument('--s-yellow-floor', type=int, default=45,
                    help='노랑 채도 절대하한(Otsu와 이 값 중 큰 값). 기본 45')
    ap.add_argument('--s-white-max', type=int, default=90,
                    help='흰색 후보 최대 채도(무채색만). 기본 90')
    ap.add_argument('--white-bright-frac', type=float, default=0.75,
                    help='흰 선 밝기 문턱 = 저채도 최고밝기(p99)×이 비율. 바닥 물면 ↑(0.8~0.85), 흐린흰선 놓치면 ↓(0.65). 기본 0.75')
    ap.add_argument('--lo-pct', type=float, default=3.0, help='lo 임계 백분위. 기본 3')
    ap.add_argument('--hi-pct', type=float, default=99.0, help='hi 임계 백분위. 기본 99')
    ap.add_argument('--margin', type=int, default=4, help='lo/hi 여유(±). 기본 4')
    ap.add_argument('--min-area', type=int, default=30,
                    help='연결요소 최소 면적[px](노이즈 컷). 기본 30')
    ap.add_argument('--open-ksize', type=int, default=3, help='open 커널. 0=off. 기본 3')
    ap.add_argument('--min-cand', type=int, default=200,
                    help='프레임을 해당 색 표본에 넣을 최소 후보 픽셀 수. 기본 200')
    ap.add_argument('--bev', type=str, default=None,
                    help='lane.yaml 경로. 주면 bev_matrix로 BEV 변환 후 샘플(검출기와 동일 픽셀)')
    ap.add_argument('--save-preview', type=str, default=None,
                    help='후보 마스크 오버레이 저장 폴더(진단용)')
    args = ap.parse_args()

    frames = load_frames(args.paths)
    if not frames:
        print('이미지를 찾지 못했습니다.')
        return
    print(f'프레임 {len(frames)}장 로드')

    M = None
    if args.bev:
        M = load_bev_matrix(args.bev)
        print(f'BEV 행렬 {"로드됨" if M is not None else "없음(raw 샘플)"}: {args.bev}')

    if args.save_preview:
        os.makedirs(args.save_preview, exist_ok=True)

    y_samples: List[np.ndarray] = []
    w_samples: List[np.ndarray] = []
    n_y = n_w = 0

    for fname, img in frames:
        if M is not None:
            h, w = img.shape[:2]
            img = cv2.warpPerspective(img, M, (w, h))
        h = img.shape[0]
        y0 = int(h * args.roi)
        roi = img[y0:]
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)

        ymask = yellow_candidates(hls, tuple(args.hue_band), args.s_yellow_floor,
                                  args.min_area, args.open_ksize)
        wmask = white_candidates(hls, args.s_white_max, args.white_bright_frac,
                                 args.min_area, args.open_ksize)

        yn, wn = int((ymask > 0).sum()), int((wmask > 0).sum())
        if yn >= args.min_cand:
            idx = ymask > 0
            y_samples.append(hls[idx]); n_y += 1
        if wn >= args.min_cand:
            idx = wmask > 0
            w_samples.append(hls[idx]); n_w += 1

        if args.save_preview:
            ov = roi.copy()
            ov[ymask > 0] = (255, 0, 255)   # 노랑 후보 → 마젠타
            ov[wmask > 0] = (0, 255, 0)     # 흰 후보 → 초록
            out = cv2.addWeighted(ov, 0.55, roi, 0.45, 0)
            cv2.imwrite(os.path.join(args.save_preview, f'cand_{fname}'), out)

    print(f'\n노랑 후보 프레임 {n_y}장 / 흰 후보 프레임 {n_w}장')

    print('\n' + '=' * 60)
    print('# lane.yaml 붙여넣기용 (아래는 데이터로 역산된 값)')
    print('=' * 60)

    if y_samples:
        Y = np.concatenate(y_samples, axis=0)
        summarize('YELLOW', Y)
        lo, hi = bounds_from_samples(Y, args.lo_pct, args.hi_pct, args.margin)
        print(f'\n    yellow_lo: [{lo[0]}, {lo[1]}, {lo[2]}]')
        print(f'    yellow_hi: [{hi[0]}, {hi[1]}, {hi[2]}]')
    else:
        print('\n(노랑 후보 표본 없음 — --s-yellow-floor / --min-cand 낮춰보세요)')

    if w_samples:
        W = np.concatenate(w_samples, axis=0)
        summarize('WHITE', W)
        lo, hi = bounds_from_samples(W, args.lo_pct, args.hi_pct, args.margin)
        # 흰색 관례: H 전체 허용, S는 상한만 의미(하한 0). L은 하한만(상한 255).
        print(f'\n    white_lo: [0, {lo[1]}, 0]')
        print(f'    white_hi: [180, 255, {hi[2]}]')
    else:
        print('\n(흰 후보 표본 없음 — --s-white-max 높여보세요)')


if __name__ == '__main__':
    main()
