"""@file aruco_detect.py
@brief ArUco 마커 검출 (ROS-free) — 미션 M4 장애물 구역 정지/재출발 신호.

@details
`cv2.aruco`로 프레임에서 ArUco 마커를 찾아 `aruco_present`(bool)를 낸다. YOLO 학습이
필요 없는 고전 CV라 대회 전 바로 쓸 수 있다. 로직은 여기(core)에 있고 ROS 노드는
구독/디코드/발행만 한다(시뮬↔실차 일원화, 하드코딩 금지·파라미터 YAML).

@par 계약
- 미션(`core.planning.mission`)은 `MissionObservation.aruco_present`만 소비한다
  (장애물 구역에서 마커가 보이는 동안 STOP, 사라지면 재출발).
- 마커는 **하단 ROI**에서 본다(mission_fsm: roi_mode LOWER_ARUCO). 배경 오검 방지.

@note OpenCV 4.7+/5.x 신 API(`ArucoDetector`) 기준. 구버전(`detectMarkers` 함수형)도
      폴백 지원. 사전(dictionary)·ROI·최소크기·대상 ID는 전부 config로 조정.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class ArucoConfig:
    """@brief ArUco 검출 파라미터(전부 YAML/CLI 조정). 값은 대회 마커 확정 후 튜닝."""
    dictionary: str = "DICT_6X6_50"   ##< cv2.aruco.DICT_* 이름. 대회 마커 확정(07-08): 6X6_50.
    roi_bottom_frac: float = 1.0      ##< 하단에서 볼 세로 비율(1.0=전체, 0.5=아래 절반).
    min_perimeter_px: float = 0.0     ##< 이 둘레(px) 미만 마커 무시(먼 오검 컷). 0=끔.
    target_ids: Tuple[int, ...] = ()  ##< 이 ID만 present로 인정. 비면 아무 마커나 인정.

    # --- 검출기 파라미터(cv2.aruco.DetectorParameters 미러) — 07-15 ---
    # 기본값 = OpenCV 스톡값(= 건드리기 전과 동일 동작). 실차 튜닝은 mission_cues.yaml에서.
    # 심판이 마커를 **비스듬히/화면 가장자리에서** 보여줘도 잡아야 한다는 요구(07-15)에
    # 맞춰 노출한 것들이다. 스톡값은 '정면·중앙' 기준이라 그 조건에서 recall이 떨어진다.
    adaptive_thresh_win_size_min: int = 3        ##< 적응임계 윈도 최소[px].
    adaptive_thresh_win_size_max: int = 23       ##< 적응임계 윈도 최대[px].
    adaptive_thresh_win_size_step: int = 10      ##< 윈도 증가 폭. ↓면 시도 창 수↑(recall↑, CPU↑). 3~23 step10=창 3개, step4=창 6개.
    polygonal_approx_accuracy_rate: float = 0.03 ##< 사각형 근사 허용오차. 기울거나 렌즈왜곡(화면 가장자리)이면 변이 휘어 근사 실패 → ↑(0.05~0.08).
    error_correction_rate: float = 0.6           ##< 비트오류 정정 허용 비율. 비스듬=비트오류 → ↑(0.8~1.0). target_ids로 거르므로 오검 위험은 제한적.
    perspective_remove_ignored_margin_per_cell: float = 0.13  ##< 셀 샘플링 시 가장자리 무시 여백. 기울면 셀 경계가 번져 옆 셀을 샘플링 → ↑(0.2~0.25).
    perspective_remove_pixel_per_cell: int = 4   ##< 셀당 샘플 픽셀. ↑면 비트판독 안정(CPU↑).
    min_corner_distance_rate: float = 0.05       ##< 코너 간 최소거리 비율. 극단 각도면 마커가 납작해져 코너가 붙음 → ↓(0.02).
    min_marker_perimeter_rate: float = 0.03      ##< 최소 마커 둘레 비율(이미지 최대변 대비). 멀거나 극단 각도면 둘레가 작아짐 → ↓(0.01).


@dataclass
class ArucoResult:
    """@brief 한 프레임 검출 결과."""
    present: bool                       ##< 유효 마커 검출 여부(→ mission_cues.aruco_present).
    ids: List[int] = field(default_factory=list)  ##< 검출·필터 통과한 마커 ID들.
    num_markers: int = 0                ##< 필터 통과 마커 개수.
    debug_image: Optional[np.ndarray] = None  ##< 오버레이(want_debug=True일 때).
    # --- 실측치(튜닝 근거) — 07-15 ---
    # 로그로 뽑아 "어느 거리/위치에서 깨지는가"를 재는 용도. 빨강 게이트를 r=/red_h= 실측으로
    # 잡았던 것과 같은 방식 — 이게 없으면 임계 튜닝이 전부 추측이 된다.
    max_perimeter_px: float = 0.0       ##< 통과 마커 중 최대 둘레[px]. 거리 지표(가까울수록 큼).
    cx_frac: float = -1.0               ##< 최대 마커 중심의 화면 가로 위치(0=좌끝, 0.5=중앙, 1=우끝). -1=검출 없음.
    cy_frac: float = -1.0               ##< 같은 마커의 세로 위치(0=위, 1=아래). -1=검출 없음.
    num_raw: int = 0                    ##< ID/크기 필터 **전** 검출 수. num_markers와 벌어지면 필터가 버리는 중.


# cv2.aruco.DICT_* 이름 → 상수. 존재하는 것만 매핑(버전차 안전).
def _resolve_dictionary(name: str):
    ar = cv2.aruco
    const = getattr(ar, name, None)
    if const is None:
        raise ValueError(
            f"ArucoConfig.dictionary='{name}' 은(는) 이 OpenCV(cv2.aruco)에 없습니다. "
            f"예: DICT_4X4_50 / DICT_5X5_100 / DICT_6X6_250")
    if hasattr(ar, "getPredefinedDictionary"):      # 4.7+/5.x
        return ar.getPredefinedDictionary(const)
    return ar.Dictionary_get(const)                  # 구버전 폴백


class ArucoDetector:
    """@brief BGR 프레임 → ArUcoResult. 상태 없음(디바운스는 노드가 담당)."""

    def __init__(self, cfg: Optional[ArucoConfig] = None):
        self.cfg = cfg or ArucoConfig()
        self._dict = _resolve_dictionary(self.cfg.dictionary)
        ar = cv2.aruco
        # 신 API(ArucoDetector) 우선, 없으면 함수형 폴백.
        self._detector = None
        if hasattr(ar, "ArucoDetector"):
            params = ar.DetectorParameters() if hasattr(ar, "DetectorParameters") \
                else ar.DetectorParameters_create()
            self._apply_params(params)
            self._detector = ar.ArucoDetector(self._dict, params)
        else:
            self._params = ar.DetectorParameters_create()
            self._apply_params(self._params)

    def _apply_params(self, params) -> None:
        """@brief ArucoConfig의 검출기 파라미터를 cv2 DetectorParameters에 반영.

        @details 이름이 없는 OpenCV 버전에서도 죽지 않도록 hasattr로 방어한다(있는 것만
        설정). 기본값이 OpenCV 스톡값이라, yaml에서 안 건드리면 동작은 종전과 같다.
        """
        c = self.cfg
        mapping = {
            "adaptiveThreshWinSizeMin": c.adaptive_thresh_win_size_min,
            "adaptiveThreshWinSizeMax": c.adaptive_thresh_win_size_max,
            "adaptiveThreshWinSizeStep": c.adaptive_thresh_win_size_step,
            "polygonalApproxAccuracyRate": c.polygonal_approx_accuracy_rate,
            "errorCorrectionRate": c.error_correction_rate,
            "perspectiveRemoveIgnoredMarginPerCell":
                c.perspective_remove_ignored_margin_per_cell,
            "perspectiveRemovePixelPerCell": c.perspective_remove_pixel_per_cell,
            "minCornerDistanceRate": c.min_corner_distance_rate,
            "minMarkerPerimeterRate": c.min_marker_perimeter_rate,
        }
        for name, value in mapping.items():
            if hasattr(params, name):
                setattr(params, name, value)

    def _detect_markers(self, gray: np.ndarray):
        """@brief 버전차 흡수: (corners, ids) 반환."""
        if self._detector is not None:               # 신 API
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:                                        # 구 API
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._params)
        return corners, ids

    def detect(self, frame_bgr: np.ndarray, want_debug: bool = False) -> ArucoResult:
        """@brief 하단 ROI에서 마커 검출 → 필터(크기/ID) → present 판정."""
        if frame_bgr is None or frame_bgr.size == 0:
            return ArucoResult(present=False)

        h = frame_bgr.shape[0]
        frac = min(max(self.cfg.roi_bottom_frac, 0.05), 1.0)
        y0 = int(round(h * (1.0 - frac)))            # ROI 시작 행(하단만).
        roi = frame_bgr[y0:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        corners, ids = self._detect_markers(gray)
        num_raw = 0 if ids is None else len(ids)   # 필터 전 원검출 수(로그용).

        kept_ids: List[int] = []
        kept_corners = []
        if ids is not None and len(ids) > 0:
            targets = set(self.cfg.target_ids)
            for c, i in zip(corners, ids.flatten().tolist()):
                if targets and i not in targets:
                    continue
                if self.cfg.min_perimeter_px > 0.0:
                    peri = cv2.arcLength(c.reshape(-1, 2).astype(np.float32), True)
                    if peri < self.cfg.min_perimeter_px:
                        continue
                kept_ids.append(int(i))
                kept_corners.append(c)

        present = len(kept_ids) > 0

        # --- 실측치(튜닝 근거): 가장 큰 = 가장 가까운 마커 기준 ---
        # 좌표는 ROI 기준이라 세로는 y0을 더해 전체 프레임으로 환산한다(가로는 ROI가
        # 가로를 안 자르므로 그대로). 화면비로 내보내 해상도가 바뀌어도 임계가 유지된다.
        max_peri = 0.0
        cx_frac = -1.0
        cy_frac = -1.0
        best_pts = None
        for c in kept_corners:
            pts = c.reshape(-1, 2).astype(np.float32)
            peri = float(cv2.arcLength(pts, True))
            if peri > max_peri:
                max_peri = peri
                best_pts = pts
        if best_pts is not None:
            width = float(frame_bgr.shape[1])
            cx_frac = float(best_pts[:, 0].mean()) / max(width, 1.0)
            cy_frac = (float(best_pts[:, 1].mean()) + float(y0)) / max(float(h), 1.0)

        debug = None
        if want_debug:
            debug = frame_bgr.copy()
            # ROI 경계선 표시.
            cv2.line(debug, (0, y0), (debug.shape[1], y0), (0, 200, 255), 1)
            if kept_corners:
                shifted = [c + np.array([0.0, float(y0)], dtype=c.dtype)
                           for c in kept_corners]
                cv2.aruco.drawDetectedMarkers(
                    debug, shifted, np.array(kept_ids).reshape(-1, 1))
            label = f"ARUCO {'PRESENT' if present else '-'} ids={kept_ids}"
            cv2.putText(debug, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0) if present else (0, 0, 255), 2)
            # 실측치도 화면에 — 모니터만 보고도 거리/위치 감을 잡게.
            cv2.putText(debug, f"peri={max_peri:.0f}px cx={cx_frac:.2f} raw={num_raw}",
                        (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        return ArucoResult(present=present, ids=kept_ids,
                           num_markers=len(kept_ids), debug_image=debug,
                           max_perimeter_px=max_peri, cx_frac=cx_frac,
                           cy_frac=cy_frac, num_raw=num_raw)
