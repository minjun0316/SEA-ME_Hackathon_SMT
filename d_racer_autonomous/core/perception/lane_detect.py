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

    # --- HLS 색 임계 (H, L, S) ---
    yellow_lo: Tuple[int, int, int] = (15, 80, 70)
    yellow_hi: Tuple[int, int, int] = (35, 255, 255)
    white_lo: Tuple[int, int, int] = (0, 200, 0)
    white_hi: Tuple[int, int, int] = (180, 255, 70)
    yellow_pixel_threshold: int = 30   ##< 노랑 픽셀 이 이상이면 노랑 우세(노랑만 추종)

    # --- 슬라이딩 윈도우 ---
    nwindows: int = 9
    margin: int = 30
    minpix: int = 5
    # 한쪽 차선이 화면 밖으로 나갔을 때(커브) 복원용 차선폭[BEV px]. 두 선이 다
    # 보이는 프레임에서 자동 학습하며, 이 값은 학습 전/한번도 못 본 경우의 초기값.
    lane_width_px: float = 180.0
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
        white = cv2.inRange(hls, np.array(c.white_lo), np.array(c.white_hi))
        yellow_px = int(cv2.countNonZero(yellow))
        white_px = int(cv2.countNonZero(white))

        res.yellow_detected = yellow_px > c.yellow_pixel_threshold
        res.white_detected = white_px > c.yellow_pixel_threshold
        res.yellow_confidence = min(1.0, yellow_px / c.color_conf_pixels)
        res.white_confidence = min(1.0, white_px / c.color_conf_pixels)

        # 노랑 우세면 노랑만, 아니면 노랑∪흰
        lane_mask = yellow if yellow_px > c.yellow_pixel_threshold \
            else cv2.bitwise_or(yellow, white)

        masked = cv2.bitwise_and(bev, bev, mask=lane_mask)
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

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
            cv2.line(debug, (midpoint, 0), (midpoint, h), (255, 255, 0), 1)
            if res.lane_path:
                cx0 = int(centerline_px[0][0])
                cv2.circle(debug, (cx0, h - 40), 5, (0, 255, 0), -1)
        res.debug_image = debug

        # 중간 단계 노출(모니터 디버그 화면용). BEV=원근변환, edges=Canny 결과.
        if want_debug:
            res.debug_stages = {'bev': bev, 'edges': edges}

        return res

    # ------------------------------------------------------------------ #
    def _to_bev(self, frame: np.ndarray) -> np.ndarray:
        """@brief 원근변환(BEV). 이미지 크기 바뀌면 행렬 재계산."""
        h, w = frame.shape[:2]
        if self._M is None or self._M_size != (w, h):
            c = self.calib
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

    def _sliding_window(self, edges: np.ndarray, debug):
        """@brief 좌/우 차선 슬라이딩 윈도우. @return (centerline_px[near→far], confidence)."""
        c = self.calib
        h, w = edges.shape[:2]
        midpoint = w // 2

        # 하단 40% 히스토그램으로 시작 x(직전값과 EMA)
        hist = np.sum(edges[int(h * 0.6):, :], axis=0)
        cur_left = int(np.argmax(hist[:midpoint])) if midpoint > 0 else 0
        cur_right = int(np.argmax(hist[midpoint:]) + midpoint) if w > midpoint else midpoint
        leftx = int(self._last_leftx * 0.8 + cur_left * 0.2)
        rightx = int(self._last_rightx * 0.8 + cur_right * 0.2)

        nz = cv2.findNonZero(edges)
        centerline: List[Tuple[float, float]] = []
        if nz is None or len(nz) == 0:
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

        # 다음 프레임 seed: near(바닥=cys[0]) 쪽 피팅값 우선.
        self._last_leftx = float(left_line[0]) if left_ok else leftx
        self._last_rightx = float(right_line[0]) if right_ok else rightx
        if lane_w > 1.0:
            self._lane_width_px = lane_w   # 다음 프레임으로 학습 폭 이월.
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
