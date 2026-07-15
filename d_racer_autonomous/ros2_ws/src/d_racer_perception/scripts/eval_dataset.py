#!/usr/bin/env python3
"""@file eval_dataset.py
@brief 데이터셋 폴더를 **실차와 동일한 YOLO 경로**로 태워 결과를 채점 (오프라인, ROS 불필요).

@details
`core.perception.traffic_light_detect.TrafficLightDetector` 를 그대로 써서 추론+게이트를
실차와 동일하게 수행한다. 모델은 config의 `yolo_model_path`(기본 NCNN 디렉토리), 파라미터는
`config/mission_cues.yaml` 을 읽어 `TrafficCueConfig` 에 주입 → 여기서 나온 결과 = 차에서
나올 결과.

@par 왜 `yolo val`(mAP) 이 아니라 이 스크립트인가
차가 소비하는 건 bbox가 아니라 `light`(none/red/green)/`direction`(none/left/right) 이산
신호뿐이다. 게다가 검출돼도 게이트(red_min_conf, sign_min_box_h_frac ...)에서 탈락하면
차에선 신호가 안 뜬다. mAP는 이 탈락을 못 본다 → **gated 와 miss 를 분리**해 보여준다.
  - gated : 모델은 찾았다. YAML 임계값 문제. → config 튜닝
  - miss  : 모델이 못 찾았다. → 재학습/데이터 문제

@par 사용법
  python3 scripts/eval_dataset.py --data /media/usb/dataset --split train
  python3 scripts/eval_dataset.py --data /media/usb/dataset --no-gates     # 게이트 끄고 raw 모델 성능만
  python3 scripts/eval_dataset.py --data DIR --save-vis /tmp/vis --vis-fails-only
  python3 scripts/eval_dataset.py --images DIR --labels DIR --csv out.csv

@warning h_frac 게이트는 **프레임 높이 기준**이다. 데이터셋 이미지가 차량 카메라 framing이
         아니면(크롭/다른 카메라/다른 종횡비) gated 통계는 실주행과 무관하다 → `--no-gates`
         결과만 신뢰할 것.
@warning 학습에 쓴 데이터(train)로 돌리면 결과는 낙관적이다. correct 비율이 아니라 여기서도
         뜨는 **miss/wrong** 이 유의미한 신호다.
"""
from __future__ import annotations

import argparse
import csv as csv_mod
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
_PKG = _HERE.parents[1]                 # .../src/d_racer_perception
_REPO = _HERE.parents[4]                # .../d_racer_autonomous
sys.path.insert(0, str(_REPO))

from core.perception.traffic_light_detect import (  # noqa: E402
    SIGN_LEFT, SIGN_NONE, SIGN_RIGHT, TL_GREEN, TL_NONE, TL_RED,
    TrafficCueConfig, TrafficLightDetector,
)

IMG_EXT = {'.jpg', '.jpeg', '.png', '.bmp'}
LIGHT_NAME = {TL_NONE: 'none', TL_RED: 'red', TL_GREEN: 'green'}
SIGN_NAME = {SIGN_NONE: 'none', SIGN_LEFT: 'left', SIGN_RIGHT: 'right'}


def load_yaml(path: Path) -> dict:
    import yaml
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def cfg_from_mission_cues(path: Path, models_dir: Path) -> TrafficCueConfig:
    """@brief mission_cues.yaml → TrafficCueConfig (노드가 하는 매핑을 그대로 재현)."""
    p = load_yaml(path).get('mission_cues_node', {}).get('ros__parameters', {})
    model = str(p.get('yolo_model_path', 'best_ncnn_model'))
    mp = Path(model)
    if not mp.is_absolute():
        mp = models_dir / model
    return TrafficCueConfig(
        model_path=str(mp),
        conf=float(p.get('yolo_conf', 0.35)),
        imgsz=int(p.get('yolo_imgsz', 320)),
        class_green=int(p.get('tl_class_green', 0)),
        class_red=int(p.get('tl_class_red', 2)),
        class_left=int(p.get('tl_class_left', 1)),
        class_right=int(p.get('tl_class_right', 3)),
        prefer_red=bool(p.get('tl_prefer_red', False)),
        cv_threads=int(p.get('yolo_cv_threads', 1)),
        torch_threads=int(p.get('yolo_torch_threads', 2)),
        sign_min_box_h_frac=float(p.get('sign_min_box_h_frac', 0.0)),
        sign_min_box_aspect=float(p.get('sign_min_box_aspect', 0.0)),
        red_min_box_h_frac=float(p.get('red_min_box_h_frac', 0.0)),
        red_min_conf=float(p.get('red_min_conf', 0.0)),
    )


def find_pairs(data: Path | None, images: Path | None, labels: Path | None,
               split: str) -> list[tuple[Path, Path | None]]:
    """@brief 이미지↔라벨 페어링. YOLO 표준 레이아웃 2종(images/train, train/images) 자동 인식."""
    if images is not None:
        img_dirs = [images]
        lbl_of = (lambda d: labels) if labels else (lambda d: None)
    else:
        cands: list[Path] = []
        splits = ['train', 'val', 'valid', 'test'] if split == 'all' else [split]
        for s in splits:
            for c in (data / 'images' / s, data / s / 'images'):
                if c.is_dir():
                    cands.append(c)
        if not cands:                       # 평평한 폴더(images/ 만, 또는 이미지 직접)
            for c in (data / 'images', data):
                if c.is_dir() and any(f.suffix.lower() in IMG_EXT for f in c.iterdir()):
                    cands.append(c)
                    break
        img_dirs = cands

        def lbl_of(d: Path) -> Path | None:
            parts = list(d.parts)
            for i in range(len(parts) - 1, -1, -1):
                if parts[i] == 'images':
                    parts[i] = 'labels'
                    cand = Path(*parts)
                    if cand.is_dir():
                        return cand
            cand = d.parent / 'labels'
            return cand if cand.is_dir() else None

    pairs: list[tuple[Path, Path | None]] = []
    for d in img_dirs:
        ld = lbl_of(d)
        for f in sorted(d.iterdir()):
            if f.suffix.lower() not in IMG_EXT:
                continue
            lf = (ld / (f.stem + '.txt')) if ld else None
            pairs.append((f, lf if (lf and lf.exists()) else None))
    return pairs


def read_label(path: Path | None) -> list[tuple[int, float, float, float, float]]:
    """@brief YOLO 라벨(cls cx cy w h, 정규화) 파싱."""
    if path is None:
        return []
    out = []
    for line in path.read_text().splitlines():
        t = line.split()
        if len(t) >= 5:
            out.append((int(float(t[0])), *(float(v) for v in t[1:5])))
    return out


def pct(v: float, tot: int) -> str:
    return f'{100.0 * v / tot:.1f}%' if tot else '  -  '


def quantiles(vals: list[float]) -> str:
    if not vals:
        return 'n/a'
    a = np.array(vals)
    return (f'p10={np.percentile(a, 10):.3f} p50={np.percentile(a, 50):.3f} '
            f'p90={np.percentile(a, 90):.3f} (n={len(a)})')


def main() -> int:
    ap = argparse.ArgumentParser(description='데이터셋을 실차와 동일한 YOLO 경로로 평가')
    ap.add_argument('--data', type=Path, help='YOLO 데이터셋 루트(images/labels 자동 페어링)')
    ap.add_argument('--images', type=Path, help='이미지 폴더 직접 지정')
    ap.add_argument('--labels', type=Path, help='라벨 폴더 직접 지정(없으면 GT 비교 생략)')
    ap.add_argument('--split', default='train', choices=['train', 'val', 'valid', 'test', 'all'])
    ap.add_argument('--config', type=Path, default=_PKG / 'config' / 'mission_cues.yaml')
    ap.add_argument('--model', type=str, help='config의 yolo_model_path 오버라이드')
    ap.add_argument('--conf', type=float, help='config의 yolo_conf 오버라이드')
    ap.add_argument('--imgsz', type=int, help='config의 yolo_imgsz 오버라이드')
    ap.add_argument('--no-gates', action='store_true', help='게이트 전부 끄고 raw 모델 성능만')
    ap.add_argument('--save-vis', type=Path, help='result.plot() 오버레이 저장 폴더')
    ap.add_argument('--vis-fails-only', action='store_true', help='틀린 이미지만 오버레이 저장')
    ap.add_argument('--csv', type=Path, help='이미지별 결과 CSV')
    ap.add_argument('--limit', type=int, default=0, help='앞에서 N장만')
    ap.add_argument('--max-fail-dump', type=int, default=40, help='실패 raw 박스 덤프 최대 개수')
    args = ap.parse_args()

    if not args.data and not args.images:
        ap.error('--data 또는 --images 중 하나는 필요합니다')

    import cv2

    cfg = cfg_from_mission_cues(args.config, _PKG / 'models')
    if args.model:
        mp = Path(args.model)
        cfg.model_path = str(mp if mp.is_absolute() else _PKG / 'models' / args.model)
    if args.conf is not None:
        cfg.conf = args.conf
    if args.imgsz is not None:
        cfg.imgsz = args.imgsz
    if args.no_gates:
        cfg.red_min_conf = 0.0
        cfg.red_min_box_h_frac = 0.0
        cfg.sign_min_box_h_frac = 0.0
        cfg.sign_min_box_aspect = 0.0

    if not Path(cfg.model_path).exists():
        print(f'[ERR] 모델 없음: {cfg.model_path}', file=sys.stderr)
        return 1

    id2name = {cfg.class_green: 'green', cfg.class_left: 'left',
               cfg.class_red: 'red', cfg.class_right: 'right'}

    print('=== 설정 (실차와 동일 경로) ===')
    print(f'config : {args.config}')
    print(f'         ⚠ 차는 install/ 사본으로 돕니다. 이 값이 실차와 같으려면 colcon build 필요.')
    print(f'model  : {cfg.model_path}')
    print(f'infer  : conf={cfg.conf} imgsz={cfg.imgsz} prefer_red={cfg.prefer_red}')
    g = ('OFF (--no-gates)' if args.no_gates else
         f'red_min_conf={cfg.red_min_conf or "off"} '
         f'red_min_box_h_frac={cfg.red_min_box_h_frac or "off"} '
         f'sign_min_box_h_frac={cfg.sign_min_box_h_frac or "off"} '
         f'sign_min_box_aspect={cfg.sign_min_box_aspect or "off"}')
    print(f'gates  : {g}')
    print(f'classes: ' + ' '.join(f'{i}={n}' for i, n in sorted(id2name.items())))

    # 데이터셋 data.yaml 의 클래스 순서와 config 클래스 id가 어긋나면 채점이 통째로 틀린다.
    if args.data:
        for dy in (args.data / 'data.yaml', args.data / 'dataset.yaml'):
            if dy.exists():
                names = load_yaml(dy).get('names')
                if isinstance(names, list):
                    names = dict(enumerate(names))
                if isinstance(names, dict):
                    ds = {int(k): str(v) for k, v in names.items()}
                    print(f'dataset names: {ds}')
                    bad = [i for i, n in id2name.items() if ds.get(i) != n]
                    if bad:
                        print(f'  ⚠⚠ 클래스 id 불일치! config {id2name} vs dataset {ds}')
                        print(f'     → 채점이 무의미합니다. mission_cues.yaml 의 tl_class_* 를 맞추세요.')
                break

    pairs = find_pairs(args.data, args.images, args.labels, args.split)
    if args.limit:
        pairs = pairs[:args.limit]
    if not pairs:
        print('[ERR] 이미지를 못 찾았습니다. --data 레이아웃을 확인하세요.', file=sys.stderr)
        return 1
    n_lbl = sum(1 for _, l in pairs if l)
    print(f'\n=== 이미지 {len(pairs)}장 (split={args.split}, 라벨 {n_lbl}장) ===')
    if n_lbl == 0:
        print('라벨이 없어 GT 비교 없이 검출 결과만 나열합니다.')

    print('모델 로드 중...')
    det = TrafficLightDetector(cfg)
    if args.save_vis:
        args.save_vis.mkdir(parents=True, exist_ok=True)

    # 클래스별 집계: GT 있음 / 게이트 통과 검출 / 게이트 탈락 / 완전 미검출 / FP
    stat = {n: dict(gt=0, hit=0, gated=0, miss=0, fp=0) for n in ('green', 'red', 'left', 'right')}
    light_buckets = dict(correct=0, gated=0, miss=0, wrong=0, fp=0)
    sign_buckets = dict(correct=0, gated=0, miss=0, wrong=0, fp=0)
    red_h_all: list[float] = []
    red_conf_gt: list[float] = []
    sign_h_gt: list[float] = []
    sign_asp_gt: list[float] = []
    fails: list[tuple[Path, str]] = []
    rows: list[dict] = []

    for i, (img_path, lbl_path) in enumerate(pairs):
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f'  [skip] 디코드 실패: {img_path.name}')
            continue

        want_vis = bool(args.save_vis) and not args.vis_fails_only
        r = det.detect(frame, want_debug=want_vis)

        gt = read_label(lbl_path)
        gt_ids = {c for c, *_ in gt}
        gt_names = {id2name.get(c) for c in gt_ids if c in id2name}

        pred_conf = {'green': r.green_conf, 'red': r.red_conf,
                     'left': r.left_conf, 'right': r.right_conf}
        gated_of = {'green': False, 'red': r.red_gated,
                    'left': r.sign_gated, 'right': r.sign_gated}

        # GT light/direction (둘 다 있으면 red/한쪽 우선 없이 집합으로 비교)
        gt_light = ('red' if 'red' in gt_names else 'green' if 'green' in gt_names else 'none')
        gt_sign = ('left' if 'left' in gt_names else 'right' if 'right' in gt_names else 'none')
        pd_light = LIGHT_NAME[r.light]
        pd_sign = SIGN_NAME[r.direction]

        if r.red_h_frac > 0:
            red_h_all.append(r.red_h_frac)
        if 'red' in gt_names:
            red_conf_gt.append(max(r.red_conf, 0.0))
        if gt_names & {'left', 'right'}:
            if r.sign_h_frac > 0:
                sign_h_gt.append(r.sign_h_frac)
                sign_asp_gt.append(r.sign_aspect)

        fail_reason = ''
        if lbl_path:
            for n in stat:
                has_gt = n in gt_names
                hit = pred_conf[n] > 0.0
                if has_gt:
                    stat[n]['gt'] += 1
                    if hit:
                        stat[n]['hit'] += 1
                    elif gated_of[n]:
                        stat[n]['gated'] += 1
                    else:
                        stat[n]['miss'] += 1
                elif hit:
                    stat[n]['fp'] += 1

            for buckets, gtv, pdv, gated in (
                    (light_buckets, gt_light, pd_light, r.red_gated),
                    (sign_buckets, gt_sign, pd_sign, r.sign_gated)):
                if gtv == pdv:
                    buckets['correct'] += 1
                elif gtv == 'none':
                    buckets['fp'] += 1
                elif pdv == 'none':
                    buckets['gated' if gated else 'miss'] += 1
                else:
                    buckets['wrong'] += 1

            if gt_light != pd_light:
                fail_reason += f'light GT={gt_light} pred={pd_light} '
            if gt_sign != pd_sign:
                fail_reason += f'sign GT={gt_sign} pred={pd_sign} '
            if fail_reason:
                fails.append((img_path, fail_reason.strip()))

        rows.append(dict(
            image=img_path.name, gt_light=gt_light, pred_light=pd_light,
            gt_sign=gt_sign, pred_sign=pd_sign,
            green_conf=f'{r.green_conf:.3f}', red_conf=f'{r.red_conf:.3f}',
            left_conf=f'{r.left_conf:.3f}', right_conf=f'{r.right_conf:.3f}',
            red_h_frac=f'{r.red_h_frac:.3f}', red_gated=int(r.red_gated),
            sign_h_frac=f'{r.sign_h_frac:.3f}', sign_aspect=f'{r.sign_aspect:.3f}',
            sign_gated=int(r.sign_gated), num_boxes=r.num_boxes, ok=int(not fail_reason),
        ))

        if args.save_vis and (not args.vis_fails_only or fail_reason):
            vis = r.debug_image
            if vis is None:
                vis = det.detect(frame, want_debug=True).debug_image
            if vis is not None:
                tag = 'FAIL_' if fail_reason else ''
                cv2.imwrite(str(args.save_vis / f'{tag}{img_path.stem}.jpg'), vis)

        if (i + 1) % 50 == 0:
            print(f'  {i + 1}/{len(pairs)} ...')

    n = len(rows)
    if n_lbl:
        print('\n--- 클래스별 검출 (게이트 통과 기준) ---')
        print(f'{"class":<7}{"GT":>6}{"detected":>10}{"gated":>8}{"missed":>8}{"FP":>6}')
        for name in ('green', 'red', 'left', 'right'):
            s = stat[name]
            print(f'{name:<7}{s["gt"]:>6}{s["hit"]:>10}{s["gated"]:>8}{s["miss"]:>8}{s["fp"]:>6}'
                  f'   detect={pct(s["hit"], s["gt"])}')

        print('\n--- 신호 판정 (차가 실제로 소비하는 값) ---')
        for label, b in (('light    ', light_buckets), ('direction', sign_buckets)):
            tot = sum(b.values())
            print(f'{label}: correct {b["correct"]}/{tot} ({pct(b["correct"], tot)})  '
                  f'gated {b["gated"]}  miss {b["miss"]}  wrong {b["wrong"]}  FP {b["fp"]}')
        print('  gated = 모델은 찾았으나 게이트 탈락 → YAML 튜닝 문제')
        print('  miss  = 모델이 못 찾음 → 재학습/데이터 문제')

    print('\n--- 게이트 실측 분포 ---')
    print(f'red_h_frac (빨강 검출된 이미지): {quantiles(red_h_all)}')
    print(f'   임계 red_min_box_h_frac={cfg.red_min_box_h_frac or "off"}')
    if red_conf_gt:
        print(f'red conf (빨강 GT 이미지)   : {quantiles([c for c in red_conf_gt if c > 0])}')
        if cfg.red_min_conf > 0.0:
            below = sum(1 for c in red_conf_gt if 0 < c < cfg.red_min_conf)
            print(f'   임계 red_min_conf={cfg.red_min_conf} → 미만 {below}장(=게이트에 걸려 종료 못 함)')
        else:
            print('   임계 red_min_conf=off')
    print(f'sign_h_frac (팻말 GT 이미지) : {quantiles(sign_h_gt)}')
    print(f'   임계 sign_min_box_h_frac={cfg.sign_min_box_h_frac or "off"}')
    print(f'sign_aspect                 : {quantiles(sign_asp_gt)}')
    print(f'   임계 sign_min_box_aspect={cfg.sign_min_box_aspect or "off"}')

    if fails:
        print(f'\n--- 실패 이미지 {len(fails)}장 (raw 박스 덤프, 최대 {args.max_fail_dump}) ---')
        print('※ detect()는 클래스별 최고 conf만 주므로, 실패분만 raw 추론 1회 더 돌려 개별 박스를 봅니다.')
        for img_path, reason in fails[:args.max_fail_dump]:
            frame = cv2.imread(str(img_path))
            res = det.model(frame, conf=cfg.conf, imgsz=cfg.imgsz, verbose=False)[0]
            fh = float(frame.shape[0])
            print(f'\n{img_path.name}  {reason}')
            b = res.boxes
            if b is None or len(b) == 0:
                print('    (박스 0개 — 모델이 아무것도 못 찾음)')
                continue
            for c, cf, xyxy in zip(b.cls.tolist(), b.conf.tolist(), b.xyxy.tolist()):
                x1, y1, x2, y2 = xyxy
                bh, bw = max(0.0, y2 - y1), max(0.0, x2 - x1)
                hf = bh / fh if fh > 1e-6 else 0.0
                asp = bw / bh if bh > 1e-6 else 0.0
                name = id2name.get(int(c), f'id{int(c)}')
                why = []
                if name == 'red':
                    if cfg.red_min_conf > 0 and cf < cfg.red_min_conf:
                        why.append(f'conf<{cfg.red_min_conf}')
                    if cfg.red_min_box_h_frac > 0 and hf < cfg.red_min_box_h_frac:
                        why.append(f'h_frac<{cfg.red_min_box_h_frac}')
                elif name in ('left', 'right'):
                    if cfg.sign_min_box_h_frac > 0 and hf < cfg.sign_min_box_h_frac:
                        why.append(f'h_frac<{cfg.sign_min_box_h_frac}')
                    if cfg.sign_min_box_aspect > 0 and asp < cfg.sign_min_box_aspect:
                        why.append(f'aspect<{cfg.sign_min_box_aspect}')
                tail = ('  ← 게이트 탈락: ' + ', '.join(why)) if why else ''
                print(f'    {name:<6} conf={cf:.3f} h_frac={hf:.3f} aspect={asp:.2f}{tail}')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv_mod.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f'\nCSV: {args.csv}')
    if args.save_vis:
        print(f'오버레이: {args.save_vis}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
