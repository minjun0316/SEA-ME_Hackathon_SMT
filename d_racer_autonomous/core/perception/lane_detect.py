"""@file lane_detect.py
@brief ROS-free 차선 인지 순수 로직(OpenCV). `lane_only_detect.cpp` 알고리즘의 파이썬 포팅.

@details
계약(docs/interfaces.md §4.2/4.3)에 맞춰 결과를 낸다:
- `lane_path`: 차선 중심선 점열, **base_link 미터 좌표**(+x 전방, +y 좌측), near→far.
- `LaneResult`(= racer_msgs/LaneStatus 미러): lane_detected/confidence/num_points/
  lateral_offset[m,+좌]/heading_error[rad]/stop_line/stop_line_dist[m] + 노랑·흰 검출.

핵심 파이프라인(cpp와 동일): BEV 원근변환 → HLS 노랑/흰 마스크 → (노랑 우세면 노랑만) →
Canny 에지 → 슬라이딩 윈도우로 좌/우 차선 추적 → 창별 중심점으로 centerline(px) →
**픽셀→미터 변환**(base_link) → offset/heading/정지선거리 산출.

@note 픽셀→미터 스케일(`m_per_px_*`, `x_near_m`)은 **캘리브 전 잠정값**이다. BEV가
      실제 지면 미터로 캘리된 게 아니므로 트랙에서 실측·튜닝해야 정확하다(계약대로
      lane_path는 미터여야 controller Pure Pursuit가 맞다).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class LaneCalib:
    """@brief 차선 인지 캘리브/임계값(전부 파라미터, 하드코딩 금지)."""

    # --- BEV 원근변환 src(이미지 크기 대비 비율). dst는 전체 사각형 ---
    bev_top_y: float = 0.4      ##< 상단 라인 y 비율(작을수록 멀리까지)
    bev_top_x: float = 0.2      ##< 상단 좌/우 x 비율(좌=x, 우=1-x)
    # 체커보드 IPM으로 실측한 BEV 호모그래피(이미지→BEV, 3x3 행렬을 행우선 9값).
    # 지정되면 4점(bev_top_*) 대신 이 행렬을 그대로 쓴다. None이면 기존 4점 폴백.
    bev_matrix: Optional[Tuple[float, ...]] = None

    # --- HLS 색 임계 (H, L, S) ---
    yellow_lo: Tuple[int, int, int] = (15, 80, 70)
    yellow_hi: Tuple[int, int, int] = (35, 255, 255)
    white_lo: Tuple[int, int, int] = (0, 160, 0)   ##< white_adaptive=False 폴백 하한
    white_hi: Tuple[int, int, int] = (180, 255, 100)  ##< 〃 상한
    # 흰색 마스크 방식. 기본 False = 원본 C++ 고정 임계(white_lo/hi = inRange((0,200,0),
    # (180,255,70)))를 그대로 사용 — 밝은 흰색은 L≥200이면 무조건 통과(원본 검증된 로직).
    # True = 적응형(L 하한 = k×L상위퍼센타일, 조명 불변). 적응형은 회색오검·저조도 깜빡임을
    # 없애지만, 프레임에 흰 선보다 더 밝은 것(tops글레어)이 있으면 기준이 올라가 '밝은 흰색을
    # 놓치는' 부작용이 있어 기본은 원본 고정으로 둔다. 조명 변동이 심하면 True로 실험.
    white_adaptive: bool = False        ##< False=원본 C++ 고정(white_lo/hi), True=적응형
    # L 하한80 k × (L 상위 white_pct 퍼센타일). 낮출수록 완화(더 어두운 흰색까지 통과).
    # 0.80→0.72(07-10): 흰 선보다 더 밝은 것(글레어/반사)이 프레임에 있으면 p99가 그쪽으로
    # 올라가 L_lo가 높아지고 정작 흰 선이 그 밑으로 빠져 '밝은 흰색이 안 잡히던' 문제 완화.
    # k=흰 선이 최고밝기의 이 비율 이상이면 통과 → 0.72면 72%까지 허용(글레어 여유↑).
    # 여전히 밝은 흰색을 놓치면 0.68→0.65로, 회색 바닥까지 잡혀 노이즈 생기면 0.75~0.78로.
    white_adaptive_k: float = 0.65
    white_pct: float = 99.0             ##< 밝기 기준 퍼센타일(흰 선=최고밝기 추종)
    white_l_floor: int = 90             ##< L 하한 최소(칠흑 프레임서 노이즈 폭주 방지)
    white_l_cap: int = 250              ##< L 하한 최대(과도한 상승 방지)
    white_s_hi: int = 75               ##< 흰색 최대 채도 S(무채색만 통과, 유채색 배제)
    yellow_pixel_threshold: int = 30   ##< 노랑 픽셀 이 이상이면 노랑 우세 후보(절대 하한)
    # 노랑 '우세' 판정에 절대 픽셀수만 쓰면, 흰 차선의 warm-tint 조각(수백 px)이
    # 임계(30)를 넘겨 '노랑 우세'로 오판→흰 차선을 통째로 버린다(=흰선 인식 실패,
    # edges 뚝뚝 끊김). 그래서 상대 조건을 추가: 노랑이 흰색의 이 비율 이상일 때만
    # 노랑-only. 1.0=노랑이 흰색보다 많아야 우세(실제 노란 차선이면 성립, 흰선 조각은
    # 미달→노랑∪흰 유지). 노란 차선을 자꾸 놓치면 ↓(0.6), 흰선이 노랑에 밀리면 ↑.
    yellow_over_white_ratio: float = 1.0

    # --- 에지(Canny) 입력/임계 ---
    # 원래는 (마스킹 BGR→gray→blur→Canny). 마스크가 이미 이진 차선영역이라, blur한
    # gray 대신 '정리한 마스크'에 Canny를 걸면 차선 윤곽이 연속·깨끗해진다(끊김↓).
    edge_from_mask: bool = True        ##< True=정리한 lane_mask에 Canny(권장), False=옛 gray/blur
    edge_close_ksize: int = 3          ##< MORPH_CLOSE/dilate 커널 크기(작은 구멍 메우기)
    edge_dilate_iter: int = 1          ##< dilate 반복(차선 두껍게→창별 minpix 안정). 0=off
    canny_lo: int = 30                 ##< Canny 하한(옛 50→30, 흰선 민감도↑). 노이즈면 ↑
    canny_hi: int = 90                 ##< Canny 상한(옛 150→90). lo의 ~3배 유지 권장

    # --- 슬라이딩 윈도우 ---
    nwindows: int = 9
    margin: int = 30
    minpix: int = 5
    # seed(탐색 시작 x): 직전 프레임 confidence가 이 값 이상이면 '추종 중(lock)'으로
    # 보고 직전 피팅 near-x를 그대로 시드로 쓴다(search-around-poly) → 곡선 연속추종.
    # 미만이면 'lost'로 보고 하단 히스토그램으로 재획득. midpoint 고정분할 히스토그램은
    # 곡선에서 양 차선이 한쪽으로 쏠릴 때 좌우를 뒤바꾸므로 lost일 때만 최소로 쓴다.
    seed_lock_conf: float = 0.40
    # lost 재획득 시 하단 히스토그램 peak을 '실제 차선'으로 인정할 최소 에지 비율.
    # peak 열의 에지량이 (스캔행수×255×이 값) 미만이면 그쪽 차선이 안 보이는 것으로
    # 보고 base를 화면 25%/75% 기본위치로 고정한다(argmax가 0/노이즈를 잡는 것 방지).
    seed_min_fill: float = 0.05
    # lost 재획득 시 argmax peak를 직전 seed와 섞는 히스토그램 가중치(0~1).
    # base = prev*(1-b) + peak*b. 0.5=반반(기존 하드코딩). 1.0=peak만, 0.0=prev만.
    seed_reacquire_blend: float = 0.5
    # lock(추종 중) 상태에서도 매 프레임 히스토그램 peak를 seed에 소폭 섞는 가중치(0~1).
    # 0.0=off(순수 search-around-poly, 기존 동작). 0.2면 seed=prev*0.8 + peak*0.2.
    # 단순 EMA가 아니라: prev seed ±seed_hist_band_px 밴드 안에서만 peak를 찾고
    # (윈도우 탐색 → 반대 차선/노이즈로의 점프 배제), 그 밴드 에지량이 seed_min_fill
    # 이상일 때만 섞는다(garbage 프레임은 유지). 커브에서 lock seed가 뒤처지는 지연을
    # 줄이는 용도. 켜면 지연↓·추종성↑, 과하면 노이즈로 seed가 떨릴 수 있어 0.1~0.3 권장.
    seed_lock_hist_blend: float = 0.0
    seed_hist_band_px: float = 60.0     ##< lock 블렌딩 시 prev 주변 peak 탐색 밴드(±px)
    # 한쪽 차선이 화면 밖으로 나갔을 때(커브) 복원용 차선폭[BEV px]. 두 선이 다
    # 보이는 프레임에서 자동 학습하며, 이 값은 학습 전/한번도 못 본 경우의 초기값.
    lane_width_px: float = 180.0
    # 곡선에서 안쪽 차선이 프레임 밖으로 나가면 좌/우 두 탐색창이 남은 바깥선 하나에
    # 모두 달라붙는다. 두 피팅선 간격이 이 비율×lane_w보다 좁으면 '같은 선을 중복
    # 검출'로 보고 단일 차선으로 강등한다(→ 곡선방향 기반 안쪽 복원).
    lane_collapse_frac: float = 0.5
    # 중심선 노이즈 완화: 경로점을 다항식(y~x)으로 피팅해 매끄럽게. 슬라이딩윈도우
    # 창별 흔들림이 조향 휘청임으로 이어지는 걸 방지(직선·커브 공통). order 2면 곡선까지.
    path_smooth: bool = True
    path_smooth_order: int = 2

    # --- 정지선(Hough) ---
    stopline_len_threshold: float = 150.0

    # --- 픽셀→미터(BEV 기준). 캘리브 전 잠정값 → 트랙 튜닝 ---
    m_per_px_forward: float = 0.005    ##< BEV 세로 1px 당 전방 거리[m]
    m_per_px_lateral: float = 0.005    ##< BEV 가로 1px 당 횡 거리[m]
    x_near_m: float = 0.15             ##< base_link(뒷차축)→BEV 최하단행 전방거리[m]

    # --- 색 신뢰도 정규화(이 픽셀수면 confidence=1.0) ---
    color_conf_pixels: float = 2000.0


@dataclass
class LaneResult:
    """@brief racer_msgs/LaneStatus 미러 + lane_path(미터). @see docs/interfaces.md §4.3."""

    lane_detected: bool = False
    confidence: float = 0.0
    num_points: int = 0
    lateral_offset: float = 0.0    ##< [m], +좌
    heading_error: float = 0.0     ##< [rad]
    stop_line: bool = False
    stop_line_dist: float = -1.0   ##< [m], 미검출 -1.0
    yellow_detected: bool = False
    yellow_confidence: float = 0.0
    white_detected: bool = False
    white_confidence: float = 0.0
    # lane_path: (x,y) 미터, base_link, near→far
    lane_path: List[Tuple[float, float]] = field(default_factory=list)
    debug_image: Optional[np.ndarray] = None
    # 중간 파이프라인 단계(이름→이미지). want_debug일 때만 채움. 예: {'bev':..., 'edges':...}
    debug_stages: dict = field(default_factory=dict)


class LaneDetector:
    """@brief 프레임(BGR) → LaneResult. 상태(직전 좌/우 x, BEV 행렬)를 내부에 유지."""

    def __init__(self, calib: Optional[LaneCalib] = None):
        self.calib = calib or LaneCalib()
        self._M = None
        self._M_size: Optional[Tuple[int, int]] = None
        self._last_leftx = 0.0
        self._last_rightx = 0.0
        self._last_conf = 0.0        ##< 직전 프레임 confidence. seed lock/lost 판정용.
        self._lane_width_px = None   ##< 두 선 다 보일 때 학습한 차선폭[px]. None이면 config 기본값.

    # ------------------------------------------------------------------ #
    def detect(self, frame: np.ndarray, want_debug: bool = False) -> LaneResult:
        """@brief 한 프레임 처리. @param frame BGR 이미지. @return LaneResult."""
        res = LaneResult()
        if frame is None or getattr(frame, 'size', 0) == 0:
            return res

        c = self.calib
        h, w = frame.shape[:2]
        midpoint = w // 2

        bev = self._to_bev(frame)
        hls = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)

        yellow = cv2.inRange(hls, np.array(c.yellow_lo), np.array(c.yellow_hi))
        white = self._white_mask(hls)
        yellow_px = int(cv2.countNonZero(yellow))
        white_px = int(cv2.countNonZero(white))

        res.yellow_detected = yellow_px > c.yellow_pixel_threshold
        res.white_detected = white_px > c.yellow_pixel_threshold
        res.yellow_confidence = min(1.0, yellow_px / c.color_conf_pixels)
        res.white_confidence = min(1.0, white_px / c.color_conf_pixels)

        # 노랑 '우세'면 노랑만, 아니면 노랑∪흰. 우세 = 절대 하한 초과 AND 흰색 대비
        # 비율 조건(흰선 조각이 임계만 넘겨 흰 차선을 버리는 오판 방지). @see LaneCalib.
        yellow_dominant = (yellow_px > c.yellow_pixel_threshold
                           and yellow_px >= white_px * c.yellow_over_white_ratio)
        lane_mask = yellow if yellow_dominant else cv2.bitwise_or(yellow, white)

        edges = self._edges_from_mask(bev, lane_mask)

        debug = bev.copy() if want_debug else None

        # --- 중심선 점열(BEV px, near→far) + confidence ---
        centerline_px, confidence = self._sliding_window(edges, debug)
        res.confidence = confidence
        res.lane_detected = len(centerline_px) >= 2 and confidence > 0.0

        if res.lane_detected:
            path_m = [self._px_to_m(cx, cy, h, midpoint) for (cx, cy) in centerline_px]
            path_m = self._smooth_path(path_m)            # 노이즈 완화(다항식 피팅)
            res.lane_path = path_m
            res.num_points = len(path_m)
            xn, yn = path_m[0]
            xf, yf = path_m[-1]
            res.lateral_offset = yn                       # 최근접점 횡오차[m], +좌
            res.heading_error = math.atan2(yf - yn, xf - xn)  # 접선 방향 vs 전방
        else:
            # 미검출: 경로 비움(controller watchdog가 정지). 계약: num_points=0.
            res.lane_path = []
            res.num_points = 0

        # --- 정지선: 검출 + 거리[m] ---
        detected, row_px = self._stopline(bev, debug)
        res.stop_line = detected
        if detected and row_px >= 0.0:
            res.stop_line_dist = c.x_near_m + (h - row_px) * c.m_per_px_forward
        else:
            res.stop_line_dist = -1.0

        if debug is not None:
            cv2.line(debug, (midpoint, 0), (midpoint, h), (255, 255, 0), 1)  # 중앙 기준선
            # lane_path(발행되는 중심선) 시각화: 미터 경로를 BEV 픽셀로 역변환해 초록
            # 폴리라인+점으로 그린다. near(바닥)은 주황 원으로 강조. 모니터 "Lane" 판에
            # 그대로 나온다(디버그 전용, 제어 동작에는 영향 없음).
            if res.lane_path:
                pts = [self._m_to_px(x, y, h, midpoint) for (x, y) in res.lane_path]
                for a, b in zip(pts[:-1], pts[1:]):
                    cv2.line(debug, a, b, (0, 255, 0), 2)
                for p in pts:
                    cv2.circle(debug, p, 3, (0, 255, 0), -1)
                cv2.circle(debug, pts[0], 5, (0, 128, 255), -1)  # near(최근접점) 강조
        res.debug_image = debug

        # 중간 단계 노출(모니터 디버그 화면용). BEV=원근변환, edges=Canny 결과.
        if want_debug:
            res.debug_stages = {'bev': bev, 'edges': edges}

        return res

    # ------------------------------------------------------------------ #
    def _white_mask(self, hls: np.ndarray) -> np.ndarray:
        """@brief 흰색 마스크. 적응형이면 프레임 밝기에 상대적인 L 하한을 쓴다.

        @param hls BEV 이미지의 HLS 변환(H,L,S).
        @return uint8 마스크(흰=255).

        @details 흰 차선은 항상 프레임 내 최고 밝기다. L 하한을 'L 상위 퍼센타일 ×
        비율'로 잡으면 조명이 밝든 어둡든 흰 선만 안정적으로 남아 깜빡임이 사라지고,
        상대적으로 어두운 회색 바닥·매트는 자동 배제된다(고정 임계의 밝음=회색오검 /
        어두움=흰선실종 문제를 동시 해결). S 상한으로 노랑 등 유채색을 배제한다.
        white_adaptive=False면 기존 고정 임계(white_lo/hi)로 폴백한다.
        """
        c = self.calib
        if not c.white_adaptive:
            return cv2.inRange(hls, np.array(c.white_lo), np.array(c.white_hi))
        L = hls[:, :, 1]
        S = hls[:, :, 2]
        l_lo = int(round(c.white_adaptive_k * float(np.percentile(L, c.white_pct))))
        l_lo = int(np.clip(l_lo, c.white_l_floor, c.white_l_cap))
        return ((L >= l_lo) & (S <= c.white_s_hi)).astype(np.uint8) * 255

    # ------------------------------------------------------------------ #
    def _edges_from_mask(self, bev: np.ndarray, lane_mask: np.ndarray) -> np.ndarray:
        """@brief 차선 마스크 → Canny 에지.

        @param bev       BEV 이미지(edge_from_mask=False 폴백 경로에서만 사용).
        @param lane_mask 노랑/흰 차선 이진 마스크(uint8, 차선=255).
        @return uint8 에지 이미지.

        @details 기본(edge_from_mask): 마스크에 MORPH_CLOSE로 작은 구멍을 메우고
        dilate로 살짝 두껍게 한 뒤 Canny → 차선 윤곽이 연속·깨끗해져 슬라이딩 윈도우
        창별 minpix가 안정된다(원인: 얇은 마스크+높은 Canny 임계면 윤곽이 끊겨
        '인식했다 말았다'). edge_from_mask=False면 옛 경로(BGR 마스킹→gray→blur→Canny).
        """
        c = self.calib
        if c.edge_from_mask:
            k = np.ones((max(1, c.edge_close_ksize),) * 2, np.uint8)
            m = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, k, iterations=1)
            if c.edge_dilate_iter > 0:
                m = cv2.dilate(m, k, iterations=c.edge_dilate_iter)
            return cv2.Canny(m, c.canny_lo, c.canny_hi)
        masked = cv2.bitwise_and(bev, bev, mask=lane_mask)
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.Canny(blur, c.canny_lo, c.canny_hi)

    # ------------------------------------------------------------------ #
    def _to_bev(self, frame: np.ndarray) -> np.ndarray:
        """@brief 원근변환(BEV). 이미지 크기 바뀌면 행렬 재계산."""
        h, w = frame.shape[:2]
        if self._M is None or self._M_size != (w, h):
            c = self.calib
            if c.bev_matrix is not None and len(c.bev_matrix) == 9:
                # 체커보드 IPM 실측 행렬 직접 사용(이미지→BEV, 행우선 3x3).
                self._M = np.array(c.bev_matrix, dtype=np.float32).reshape(3, 3)
            else:
                src = np.float32([
                    [w * c.bev_top_x, h * c.bev_top_y],
                    [w * (1.0 - c.bev_top_x), h * c.bev_top_y],
                    [w, h],
                    [0, h],
                ])
                dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
                self._M = cv2.getPerspectiveTransform(src, dst)
            self._M_size = (w, h)
            self._last_leftx = w * 0.25
            self._last_rightx = w * 0.75
        return cv2.warpPerspective(frame, self._M, (w, h))

    def _reacquire_base(self, hist: np.ndarray, w: int, h: int) -> Tuple[int, int]:
        """@brief lost 상태에서 좌/우 탐색 시작 base x를 재획득한다.

        @param hist 하단 영역 열별 에지 합(길이 w).
        @param w,h  프레임 크기[px].
        @return (leftx, rightx) 시작 base.

        @details 각 반쪽 히스토그램 peak의 에지량이 충분하면(=차선이 보이면)
        argmax를 직전값과 블렌딩해 채택한다. 임계 미만이면 그쪽 차선이 '안 보이는'
        것으로 보고 base를 화면 25%/75% 기본위치로 고정한다 — argmax는 에지가 없어도
        무조건 인덱스를 반환(전부 0이면 0=맨왼쪽)하므로 엉뚱한 점을 잡는 것을 막는다.
        """
        c = self.calib
        midpoint = w // 2
        n_rows = h - int(h * 0.6)                        # 히스토그램 스캔 행 수
        min_mass = 255.0 * n_rows * c.seed_min_fill      # 실제 차선 인정 최소 에지량
        left_hist = hist[:midpoint]
        right_hist = hist[midpoint:]
        b = c.seed_reacquire_blend                        # 히스토그램 가중치(prev와 블렌딩)
        if left_hist.size and float(left_hist.max()) >= min_mass:
            cur_left = int(np.argmax(left_hist))
            leftx = int(self._last_leftx * (1.0 - b) + cur_left * b)
        else:
            leftx = int(w * 0.25)                         # 좌측 차선 미검출 → 기본 25%
        if right_hist.size and float(right_hist.max()) >= min_mass:
            cur_right = int(np.argmax(right_hist) + midpoint)
            rightx = int(self._last_rightx * (1.0 - b) + cur_right * b)
        else:
            rightx = int(w * 0.75)                        # 우측 차선 미검출 → 기본 75%
        return leftx, rightx

    def _blend_lock_seed(self, prev_x: float, hist: np.ndarray, w: int, h: int) -> int:
        """@brief lock 상태 seed를 히스토그램 peak 쪽으로 소폭 당긴다(gated+windowed).

        @param prev_x 직전 seed x(=lock 시작점).
        @param hist   하단 영역 열별 에지 합(길이 w).
        @param w,h    프레임 크기[px].
        @return 블렌딩된 seed x.

        @details prev_x ±seed_hist_band_px 밴드 **안에서만** peak를 찾는다(윈도우
        탐색 → 반대 차선/노이즈로 seed가 점프하는 것을 원천 배제). 밴드 내 최대
        에지량이 seed_min_fill 기준 미만이면 그 프레임엔 신뢰할 peak가 없다고 보고
        prev_x를 그대로 유지한다(garbage 프레임 미반영). 기준을 넘으면
        prev*(1-b) + peak*b 로 섞어 커브에서 seed가 뒤처지는 지연만 줄인다.
        seed_lock_hist_blend=0이면 호출되지 않는다.
        """
        c = self.calib
        n_rows = h - int(h * 0.6)
        min_mass = 255.0 * n_rows * c.seed_min_fill
        lo = max(0, int(prev_x - c.seed_hist_band_px))
        hi = min(w, int(prev_x + c.seed_hist_band_px))
        if hi - lo < 1:
            return int(prev_x)
        band = hist[lo:hi]
        if float(band.max()) < min_mass:
            return int(prev_x)                        # 밴드 내 실제 에지 없음 → 유지
        peak = lo + int(np.argmax(band))
        b = c.seed_lock_hist_blend
        return int(prev_x * (1.0 - b) + peak * b)

    def _sliding_window(self, edges: np.ndarray, debug):
        """@brief 좌/우 차선 슬라이딩 윈도우. @return (centerline_px[near→far], confidence)."""
        c = self.calib
        h, w = edges.shape[:2]
        midpoint = w // 2

        # 시작 x seed 결정: lock(추종 중)이면 직전 피팅 near-x를 그대로 써서 곡선을
        # 연속 추종(search-around-poly). lost면 하단 히스토그램으로 좌/우 재획득.
        # midpoint 고정분할은 곡선에서 좌우를 뒤바꾸므로 lost일 때만 경로에 탄다.
        if self._last_conf >= c.seed_lock_conf:
            leftx = int(self._last_leftx)
            rightx = int(self._last_rightx)
            if c.seed_lock_hist_blend > 0.0:
                # 옵션: lock 중에도 히스토그램 peak를 소폭 섞어 커브 지연을 줄인다.
                hist = np.sum(edges[int(h * 0.6):, :], axis=0)
                leftx = self._blend_lock_seed(self._last_leftx, hist, w, h)
                rightx = self._blend_lock_seed(self._last_rightx, hist, w, h)
        else:
            hist = np.sum(edges[int(h * 0.6):, :], axis=0)
            leftx, rightx = self._reacquire_base(hist, w, h)

        nz = cv2.findNonZero(edges)
        centerline: List[Tuple[float, float]] = []
        if nz is None or len(nz) == 0:
            self._last_conf = 0.0   # 에지 없음 → lost. 다음 프레임 히스토그램 재획득.
            return centerline, 0.0
        # findNonZero는 OpenCV 버전에 따라 (N,1,2) 또는 (N,2) → 항상 (N,2)로.
        pts = np.asarray(nz).reshape(-1, 2)
        nzx = pts[:, 0]
        nzy = pts[:, 1]

        window_height = max(1, h // c.nwindows)

        # --- 1단계: 좌/우 각각 추적(창별 위치 + 검출여부 기록) ---
        lxs: List[float] = []
        rxs: List[float] = []
        cys: List[float] = []
        lfound: List[bool] = []
        rfound: List[bool] = []
        for win in range(c.nwindows):
            y_low = h - (win + 1) * window_height
            y_high = h - win * window_height
            lx_low, lx_high = leftx - c.margin, leftx + c.margin
            rx_low, rx_high = rightx - c.margin, rightx + c.margin

            in_y = (nzy >= y_low) & (nzy < y_high)
            good_left = in_y & (nzx >= lx_low) & (nzx < lx_high)
            good_right = in_y & (nzx >= rx_low) & (nzx < rx_high)

            lf = int(good_left.sum()) > c.minpix
            rf = int(good_right.sum()) > c.minpix
            if lf:
                leftx = int(nzx[good_left].mean())
            if rf:
                rightx = int(nzx[good_right].mean())

            lxs.append(leftx); rxs.append(rightx); cys.append((y_low + y_high) / 2.0)
            lfound.append(lf); rfound.append(rf)

            if debug is not None:
                cv2.rectangle(debug, (lx_low, y_low), (lx_high, y_high), (255, 0, 0), 2)
                cv2.rectangle(debug, (rx_low, y_low), (rx_high, y_high), (0, 0, 255), 2)

        # --- 2단계: 유효 선 판정 + 곡선 피팅(far까지 연장) ---
        # 잡힌 점들을 x=f(y) 다항식으로 피팅해 놓친(위쪽) 창까지 곡선을 연장한다
        # → 커브에서 선이 프레임 위로 빠져도 중심선이 계속 휘어 커브를 완주.
        # min_track = 피팅에 필요한 최소 창 수. 실제 트랙엔 얼룩이 없어 느슨히 2로 둠
        # (2 미만은 피팅 불가). 얼룩 있는 환경이면 ↑로 얼룩 오검을 컷할 수 있음.
        min_track = 2
        cys_arr = np.array(cys, dtype=np.float64)

        def fit_line(found_flag, xs_list):
            ys = np.array([cys[i] for i in range(c.nwindows) if found_flag[i]],
                          dtype=np.float64)
            xs = np.array([xs_list[i] for i in range(c.nwindows) if found_flag[i]],
                          dtype=np.float64)
            if len(ys) < min_track or float(ys.max() - ys.min()) < 1e-3:
                return None
            order = 2 if len(ys) >= 3 else 1   # 3점 이상이면 곡선(2차)
            try:
                coef = np.polyfit(ys, xs, order)
            except (np.linalg.LinAlgError, ValueError):
                return None
            return np.polyval(coef, cys_arr)   # 모든 창 y에서의 x(연장 포함)

        left_line = fit_line(lfound, lxs)
        right_line = fit_line(rfound, rxs)
        left_ok = left_line is not None
        right_ok = right_line is not None

        # 차선폭 학습(양쪽 다 피팅될 때, 피팅선 간격 평균).
        lane_w = self._lane_width_px if self._lane_width_px is not None \
            else float(c.lane_width_px)
        if left_ok and right_ok:
            wobs = float(np.mean(right_line - left_line))
            if wobs > 0.3 * lane_w:
                lane_w = lane_w * 0.7 + wobs * 0.3

        # --- 붕괴(collapse) 방지 ---
        # 노이즈/한쪽 선 끊김으로 좌·우 두 탐색창이 같은 실선 하나에 달라붙으면
        # (두 피팅선 간격 < lane_collapse_frac × lane_w) 중심선이 반차선 튀고, seed도
        # 같은 선에 얹혀 lock인 채 못 빠져나온다(→ 원래 차선 상실·이탈). 이때 단일
        # 차선으로 강등: 붕괴 위치가 직전 좌/우 seed 중 어디에 더 가깝냐로 어느 실선인지
        # 판정하고 반대편을 lane_w로 복원한다 → 중심선 정상화 + 다음 seed 분리(복귀).
        if left_ok and right_ok and \
                float(np.mean(right_line - left_line)) < c.lane_collapse_frac * lane_w:
            merged = (left_line + right_line) / 2.0            # 붙어버린 실선(창별)
            if abs(float(merged[0]) - self._last_leftx) <= \
                    abs(float(merged[0]) - self._last_rightx):
                left_line, right_line = merged, merged + lane_w    # 왼쪽 선에 붙음
            else:
                left_line, right_line = merged - lane_w, merged    # 오른쪽 선에 붙음

        # --- 3단계: 모드별 중심선(피팅선 사용 → 곡선 연장) ---
        centerline = []
        prev_cx = None
        for i in range(c.nwindows):
            if left_ok and right_ok:
                cx = (left_line[i] + right_line[i]) / 2.0
            elif left_ok:                       # 왼쪽선만: 오른쪽을 차선폭으로 복원
                cx = left_line[i] + lane_w / 2.0
            elif right_ok:                      # 오른쪽선만: 왼쪽 복원
                cx = right_line[i] - lane_w / 2.0
            else:
                cx = prev_cx if prev_cx is not None else float(midpoint)
            centerline.append((float(cx), cys[i]))
            prev_cx = cx

        # confidence: 실제로 픽셀이 잡힌 창 비율(피팅 연장분 제외).
        found_any = sum(1 for i in range(c.nwindows) if lfound[i] or rfound[i])
        confidence = found_any / float(c.nwindows)

        # 다음 프레임 seed: near(바닥=cys[0]) 쪽 피팅값 우선. 한쪽만 잡힌 경우
        # 놓친 쪽 seed를 (잡힌 선 ± lane_w)로 앵커한다 → 곡선에서 한쪽만 보다가
        # 반대편 차선이 돌아올 때 탐색창이 이미 예측 위치에 있어 즉시 재획득한다.
        # (앵커 없이 leftx/rightx를 그대로 두면 놓친 쪽 seed가 히스토그램 노이즈로
        #  표류해, 복귀 프레임에서 진짜 선을 margin 밖에 두고 몇 프레임간 못 잡음.)
        if left_ok and right_ok:
            self._last_leftx = float(left_line[0])
            self._last_rightx = float(right_line[0])
        elif left_ok:
            self._last_leftx = float(left_line[0])
            self._last_rightx = float(left_line[0] + lane_w)
        elif right_ok:
            self._last_rightx = float(right_line[0])
            self._last_leftx = float(right_line[0] - lane_w)
        else:
            self._last_leftx = leftx
            self._last_rightx = rightx
        if lane_w > 1.0:
            self._lane_width_px = lane_w   # 다음 프레임으로 학습 폭 이월.
        self._last_conf = confidence       # 다음 프레임 seed lock/lost 판정에 사용.
        return centerline, confidence

    def _smooth_path(self, path_m: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """@brief 중심선(x,y)을 다항식(y~x)으로 피팅해 창별 노이즈 제거.

        @details 슬라이딩 윈도우 중심은 창마다 흔들려 조향 휘청임을 유발한다.
        near→far로 단조증가하는 x에 대해 y를 저차 다항식으로 피팅해 매끄러운 경로를
        만든다. 점이 부족하거나 x폭이 없으면 원본 유지(안전).
        """
        c = self.calib
        if not c.path_smooth or len(path_m) < c.path_smooth_order + 1:
            return path_m
        xs = np.array([p[0] for p in path_m], dtype=np.float64)
        ys = np.array([p[1] for p in path_m], dtype=np.float64)
        if float(xs.max() - xs.min()) < 1e-3:             # x가 거의 동일 → 피팅 불가
            return path_m
        try:
            coef = np.polyfit(xs, ys, c.path_smooth_order)
            ys_fit = np.polyval(coef, xs)
        except (np.linalg.LinAlgError, ValueError):
            return path_m
        return [(float(x), float(y)) for x, y in zip(xs, ys_fit)]

    def _px_to_m(self, col: float, row: float, h: int, midpoint: int) -> Tuple[float, float]:
        """@brief BEV 픽셀(col,row) → base_link 미터(+x 전방, +y 좌). 잠정 스케일."""
        c = self.calib
        x = c.x_near_m + (h - row) * c.m_per_px_forward
        y = (midpoint - col) * c.m_per_px_lateral
        return (x, y)

    def _m_to_px(self, x: float, y: float, h: int, midpoint: int) -> Tuple[int, int]:
        """@brief base_link 미터(x,y) → BEV 픽셀(col,row). `_px_to_m`의 역변환(디버그 그리기용)."""
        c = self.calib
        col = midpoint - y / c.m_per_px_lateral if c.m_per_px_lateral else midpoint
        row = h - (x - c.x_near_m) / c.m_per_px_forward if c.m_per_px_forward else h
        return (int(round(col)), int(round(row)))

    def _stopline(self, bev: np.ndarray, debug):
        """@brief 정지선 검출 + 최근접 정지선의 BEV 행(row). @return (detected, row|-1)."""
        c = self.calib
        h, w = bev.shape[:2]
        y0 = max(0, h - 80)
        roi = bev[y0:h, :]

        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        white_mask = cv2.inRange(hls, np.array((0, 200, 0)), np.array((180, 255, 255)))
        edges = cv2.Canny(white_mask, 100, 200)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 40,
                                minLineLength=40, maxLineGap=5)
        total_len = 0.0
        nearest_row = -1.0
        if lines is not None and len(lines) > 0:
            # HoughLinesP도 버전에 따라 (N,1,4)/(N,4) → (N,4)로.
            for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
                if abs(angle) < 10 or abs(angle) > 170:
                    total_len += math.hypot(x2 - x1, y2 - y1)
                    y_abs = (y1 + y2) * 0.5 + y0
                    # 바닥에 가장 가까운(=row 큰) 수평선 유지
                    if nearest_row < 0 or y_abs > nearest_row:
                        nearest_row = float(y_abs)
                    if debug is not None:
                        cv2.line(debug, (x1, y1 + y0), (x2, y2 + y0), (0, 255, 255), 2)

        detected = total_len > c.stopline_len_threshold
        return (detected, nearest_row if detected else -1.0)
