"""@file sign_detect.py
@brief 방향 팻말(좌/우) 인지 (ROS-free) — S자 코스 뒤 갈림길 분기 신호.

@details
새 미션 흐름(07-14): S자 흰선 코스가 끝나면 방향 팻말 구간(SIGN_BRANCH)이 나온다.
두 단계로 인지한다.
  1) **OpenCV 색 트리거**(`detect_color`): 팻말 고유색을 HSV로 찾아 "팻말 구간 진입"만
     판정한다. 싸고 무상태라 매 프레임 돌린다. 판단이 이 신호로 SIGN_BRANCH에 진입하고
     팻말 YOLO 게이트(sign_enable)를 켠다.
  2) **팻말 YOLO 방향**(`detect_direction`): 별도 학습 모델(좌/우 클래스)로 좌/우를
     판정한다. 무거우니 SIGN_BRANCH(게이트 ON)에서만 호출한다.

@par 역할 경계 (traffic_light_detect.py / aruco_detect.py 와 동일 관례)
- 이 검출기는 **프레임당 무상태**다. "지금 프레임에 무엇이 보이나"만 판정한다.
- 시간 디바운스/구간 타이머(고정시간 후 복귀 등)는 **노드/판단**이 담당한다.
- 반환 방향값 SIGN_NONE(0)/SIGN_LEFT(1)/SIGN_RIGHT(2)는 racer_msgs/MissionCues의
  SIGN_* 상수, core.planning.SignDirection 과 값이 일치한다(노드가 그대로 매핑).

@note 좌/우 팻말 YOLO 모델은 **학습 중**(2026-07-14)이다. 모델 경로가 없거나 로드에
      실패하면 `detect_direction`은 항상 SIGN_NONE을 낸다(=bias 0, 직진 — 안전 기본값).
      모델이 준비되면 노드가 `model_path`만 넘기면 이 슬롯이 활성화된다. `ultralytics`는
      무거워 **지연 import**한다(색 트리거 경로는 torch 없이 동작).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# MissionCues.SIGN_* / core.planning.SignDirection 와 값 일치(계약).
SIGN_NONE = 0
SIGN_LEFT = 1
SIGN_RIGHT = 2


@dataclass
class SignConfig:
    """@brief 방향 팻말 인지 파라미터(전부 YAML/CLI 조정, 하드코딩 금지)."""
    # --- OpenCV 색 트리거(HSV) — 팻말 고유색. ⚠ 실제 팻말색으로 반드시 캘리브 ---
    # OpenCV BGR2HSV 규약: H 0~179, S/L 0~255. 아래는 '파란 팻말' 잠정 기본값 —
    # 대회 팻말 실측색으로 sign.yaml에서 교체할 것(오검 방지).
    color_lo: Tuple[int, int, int] = (100, 120, 60)   ##< HSV 하한(팻말색).
    color_hi: Tuple[int, int, int] = (130, 255, 255)  ##< HSV 상한.
    min_px: int = 400            ##< 색 픽셀 이 이상이면 팻말 구간 진입 트리거(stray 배제).
    roi_top_frac: float = 0.0    ##< 색 탐색 상단 잘라내기 비율(0=전체, 0.3=상단30% 무시).
    # --- 팻말 방향 YOLO(별도 모델, 학습중) ---
    model_path: str = ""         ##< 좌/우 팻말 모델 경로(노드가 해석). 빈값=방향검출 비활성(NONE).
    conf: float = 0.35           ##< 박스 인정 최소 신뢰도.
    imgsz: int = 320             ##< 추론 입력 크기(정사각).
    class_left: int = 0          ##< 좌측 지시 팻말 클래스 id(모델 확정 시 갱신).
    class_right: int = 1         ##< 우측 지시 팻말 클래스 id(모델 확정 시 갱신).
    cv_threads: int = 1          ##< OpenCV 스레드 상한(코어 경합 완화).
    torch_threads: int = 2       ##< torch 스레드 상한(NMS/후처리).


@dataclass
class SignColorResult:
    """@brief 한 프레임 팻말 색 트리거 결과."""
    present: bool = False   ##< 팻말색 픽셀이 min_px 이상(구간 진입 트리거).
    px: int = 0             ##< 팻말색 마스크 픽셀 수(진단/튜닝).


@dataclass
class SignDirResult:
    """@brief 한 프레임 팻말 방향(YOLO) 결과."""
    direction: int = SIGN_NONE   ##< SIGN_NONE/LEFT/RIGHT(프레임 단위).
    left_conf: float = 0.0       ##< 좌 팻말 최고 신뢰도(없으면 0).
    right_conf: float = 0.0      ##< 우 팻말 최고 신뢰도(없으면 0).
    num_boxes: int = 0           ##< 전체 박스 개수(로깅용).
    debug_image: Optional[np.ndarray] = None  ##< 오버레이(want_debug=True이고 모델 있을 때).


class SignDetector:
    """@brief BGR 프레임 → 팻말 색 트리거 + (모델 있으면) 좌/우 방향. 상태 없음.

    @details 색 트리거(`detect_color`)는 항상 동작한다. 방향(`detect_direction`)은
    생성 시 모델 로드에 성공했을 때만 유효하고, 아니면 SIGN_NONE을 낸다(모델 학습중).
    """

    def __init__(self, cfg: Optional[SignConfig] = None):
        self.cfg = cfg or SignConfig()
        self.model = None

        # 색 트리거는 모델 없이 동작하므로, 모델 로드 실패해도 예외를 삼키고 계속.
        if self.cfg.model_path:
            try:
                import cv2
                cv2.setNumThreads(int(self.cfg.cv_threads))
            except Exception:
                pass
            try:
                import torch
                torch.set_num_threads(int(self.cfg.torch_threads))
            except Exception:
                pass
            try:
                from ultralytics import YOLO  # 지연 import(torch를 여기서만 끌어옴).
                self.model = YOLO(str(self.cfg.model_path))
            except Exception:
                # 모델 미준비/로드 실패 → 방향검출 비활성(색 트리거만). 노드가 경고 로그.
                self.model = None

    # ------------------------------------------------------------------ #
    def detect_color(self, frame_bgr: np.ndarray) -> SignColorResult:
        """@brief 팻말 고유색(HSV)이 프레임에 충분히 보이는지 → 구간 진입 트리거."""
        if frame_bgr is None or frame_bgr.size == 0:
            return SignColorResult()
        import cv2
        h = frame_bgr.shape[0]
        y0 = int(max(0.0, min(0.95, self.cfg.roi_top_frac)) * h)
        roi = frame_bgr[y0:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.cfg.color_lo), np.array(self.cfg.color_hi))
        px = int(cv2.countNonZero(mask))
        return SignColorResult(present=px >= self.cfg.min_px, px=px)

    # ------------------------------------------------------------------ #
    def detect_direction(self, frame_bgr: np.ndarray,
                         want_debug: bool = False) -> SignDirResult:
        """@brief 팻말 YOLO로 좌/우 판정. 모델 없으면 SIGN_NONE(직진 기본값)."""
        if self.model is None or frame_bgr is None or frame_bgr.size == 0:
            return SignDirResult()

        result = self.model(frame_bgr, conf=self.cfg.conf,
                            imgsz=self.cfg.imgsz, verbose=False)[0]
        left_conf = 0.0
        right_conf = 0.0
        boxes = result.boxes
        n = 0 if boxes is None else len(boxes)
        if boxes is not None and n > 0:
            for cls_t, conf_t in zip(boxes.cls.tolist(), boxes.conf.tolist()):
                c = int(cls_t)
                cf = float(conf_t)
                if c == self.cfg.class_left:
                    left_conf = max(left_conf, cf)
                elif c == self.cfg.class_right:
                    right_conf = max(right_conf, cf)

        # 프레임 단위 방향: 좌/우 중 conf 높은 쪽. 둘 다 없으면 NONE.
        if left_conf == 0.0 and right_conf == 0.0:
            direction = SIGN_NONE
        elif right_conf >= left_conf:
            direction = SIGN_RIGHT
        else:
            direction = SIGN_LEFT

        debug = result.plot() if want_debug else None
        return SignDirResult(
            direction=direction,
            left_conf=left_conf,
            right_conf=right_conf,
            num_boxes=n,
            debug_image=debug,
        )
