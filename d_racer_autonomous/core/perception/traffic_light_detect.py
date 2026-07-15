"""@file traffic_light_detect.py
@brief 미션 신호(신호등 + 방향팻말) YOLO 통합 검출 (ROS-free) — 출발/정지/분기 트리거.

@details
학습된 YOLO 통합 모델(`best.pt`, 클래스 0:green 1:left 2:red 3:right — 2026-07-14
계획 개편으로 신호등·팻말을 한 모델로 통합)로 프레임에서 **신호등(초록/빨강)** 과
**방향팻말(좌/우)** 을 한 번의 추론으로 찾아 미션 신호를 낸다. 로직은 여기(core)에
있고 ROS 노드는 구독/디코드/발행만 한다(하드코딩 금지·파라미터 YAML).
클래스 id는 전부 파라미터화(class_green/red/left/right) — 모델 바뀌면 YAML만 갱신.
구 모델의 checker/stop_line 클래스는 폐기(checker 도착판정 폐기·정지선은 색검출).

@par 역할 경계 (aruco_detect.py 와 동일 관례)
- 이 검출기는 **프레임당 무상태**다. "지금 프레임에 무엇이 보이나"만 판정한다.
- 오출발 방지용 시간 디바운스(초록 N초 연속 확정 등)는 **노드**가 담당한다.
- 판단(`core.planning.mission`)은 `MissionObservation.traffic_light`(초록→출발)와
  `checkerboard_detected`(도착)만 소비한다. 이 검출기는 그 계약값을 채운다.

@par 신호등 색 판정 (프레임 단위)
초록/빨강 박스 중 conf 최고를 택하되, **둘 다 보이면 안전하게 RED 우선**(prefer_red).
출발 게이트라 애매하면 대기가 안전하다. 반환 `light` 값은 racer_msgs/MissionCues의
TL_NONE(0)/TL_RED(1)/TL_GREEN(2)와 일치한다(노드가 그대로 매핑).

@note `ultralytics`(YOLO)는 무거워 **지연 import**한다(단순히 이 파일이 있다고
      sim/tests가 torch를 끌어오지 않게). 모델 경로 해석(ament share 등)은 ROS 노드
      몫이라 여기엔 **이미 해석된 절대/상대 경로**를 받는다(core는 ament 비의존).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

# MissionCues.TL_* 와 값 일치(계약). 판단 core.planning.TrafficLight 와도 동일 값.
TL_NONE = 0
TL_RED = 1
TL_GREEN = 2

# MissionCues.SIGN_* / core.planning.SignDirection 와 값 일치(계약). sign_detect.py 와 동일.
SIGN_NONE = 0
SIGN_LEFT = 1
SIGN_RIGHT = 2


@dataclass
class TrafficCueConfig:
    """@brief 신호등 + 방향팻말 통합 YOLO 검출 파라미터(전부 YAML/CLI 조정)."""
    model_path: str = "best.pt"           ##< 노드가 해석해 넘기는 모델 경로(.pt/ncnn 디렉토리).
    conf: float = 0.35                    ##< 박스 인정 최소 신뢰도(이하 무시).
    imgsz: int = 320                      ##< 추론 입력 크기(정사각).
    # --- 통합 모델 클래스 id (기본=best.pt: 0 green/1 left/2 red/3 right). 모델 바뀌면 갱신 ---
    class_green: int = 0                  ##< 초록불 클래스 id.
    class_red: int = 2                    ##< 빨간불 클래스 id.
    class_left: int = 1                   ##< 좌지시 팻말 클래스 id.
    class_right: int = 3                  ##< 우지시 팻말 클래스 id.
    prefer_red: bool = True               ##< 초록·빨강 동시 검출 시 RED로(출발 안전측).
    cv_threads: int = 1                   ##< OpenCV 스레드 상한(코어 경합 완화).
    torch_threads: int = 2                ##< torch 스레드 상한(NMS/후처리).
    # --- 팻말(좌/우) 거리·각도 게이트 (07-14) ---
    # 팻말을 너무 멀리서/비스듬히 잡으면 아직 그 차선이 안 보이는 구간에서 차선전환을
    # 지시해 버린다 → 아래 두 게이트로 "가까이서 정면으로 보일 때만" 방향을 인정한다.
    # 신호등(green/red)에는 적용 안 함(멀리서 봐야 출발 판단 가능).
    sign_min_box_h_frac: float = 0.0      ##< 팻말 박스 높이/프레임높이 하한(=거리 게이트). 0=off. 멀수록 작아짐.
    sign_min_box_aspect: float = 0.0      ##< 팻말 박스 가로/세로(w/h) 하한(=각도 게이트). 0=off. 비스듬할수록 납작(작아짐).
    # --- 팻말 신뢰도 게이트 (07-15) ---
    # 좌/우 오판 하나가 갈림길을 반대로 타게 하므로, 방향은 '확실할 때만' 인정한다.
    # 판단(SIGN_BRANCH)의 진입 트리거가 이제 sign_direction 자체라 이 게이트가 곧 진입
    # 조건이다 — MissionCues에 conf 필드를 두지 않고 여기서 걸러 계약을 유지한다.
    sign_min_conf: float = 0.0            ##< 팻말 방향 인정 최소 신뢰도(0=off → 전역 conf 사용). 좌/우 오판 방지.
    # --- 빨간불 게이트 (07-15) ---
    # 빨강 오검출 하나가 코스를 끝내버려(red_confirm_count=1, 즉시정지) 방어가 필요하다.
    # 종료 신호등은 '가까이서' 보므로 거리(박스 크기)·신뢰도로 거른다.
    # ⚠ 초록엔 적용 안 함 — 출발 신호등은 멀리서 봐야 하므로 게이트를 걸면 출발을 못 한다.
    red_min_box_h_frac: float = 0.0       ##< 빨강 박스 높이/프레임높이 하한(=거리). 0=off. 멀리서 뜨는 오검출 컷.
    red_min_conf: float = 0.0             ##< 빨강 전용 최소 신뢰도(0=off → 전역 conf 사용). 초록보다 엄격하게.


@dataclass
class TrafficCueResult:
    """@brief 한 프레임 통합 검출 결과(신호등 + 방향팻말)."""
    light: int = TL_NONE          ##< 신호등 상태(TL_NONE/RED/GREEN, 프레임 단위 판정).
    green_conf: float = 0.0       ##< 검출된 초록 박스 최고 신뢰도(없으면 0).
    red_conf: float = 0.0         ##< 검출된 빨강 박스 최고 신뢰도(없으면 0).
    direction: int = SIGN_NONE    ##< 방향팻말(SIGN_NONE/LEFT/RIGHT, 프레임 단위 판정).
    left_conf: float = 0.0        ##< 좌 팻말 박스 최고 신뢰도(없으면 0).
    right_conf: float = 0.0       ##< 우 팻말 박스 최고 신뢰도(없으면 0).
    num_boxes: int = 0            ##< 전체 박스 개수(로깅용).
    # 팻말 게이트 튜닝용 관측치: 게이트 통과 여부와 무관하게 '가장 큰 팻말 박스'의 실측값.
    # 로그로 찍어 sign_min_box_h_frac/aspect 임계를 실측에 맞춰 잡는다.
    sign_h_frac: float = 0.0      ##< 최대 팻말 박스 높이/프레임높이(팻말 없으면 0).
    sign_aspect: float = 0.0      ##< 그 박스의 w/h(팻말 없으면 0).
    sign_gated: bool = False      ##< 팻말 박스는 있었으나 거리/각도 게이트에서 걸러짐.
    red_h_frac: float = 0.0       ##< 최대 빨강 박스 높이/프레임높이(빨강 없으면 0). 게이트 튜닝용 실측.
    red_gated: bool = False       ##< 빨강 박스는 있었으나 거리/신뢰도 게이트에서 걸러짐.
    debug_image: Optional[np.ndarray] = None  ##< 오버레이(want_debug=True일 때).


class TrafficLightDetector:
    """@brief BGR 프레임 → TrafficCueResult(신호등+팻말). 상태 없음(디바운스는 노드가 담당).

    @details 생성 시 통합 YOLO 모델을 로드한다(무거우니 노드가 한 번만 만든다). detect()는
    프레임마다 **한 번의 추론**으로 초록/빨강(신호등)과 좌/우(팻말) 박스를 모아 프레임 단위
    신호를 낸다 — 신호등·팻말이 한 모델이라 노드는 이 결과를 양쪽으로 라우팅한다.
    """

    def __init__(self, cfg: Optional[TrafficCueConfig] = None):
        self.cfg = cfg or TrafficCueConfig()

        # 스레드 과다구독 방지: OpenCV(디코드/plot)·torch(NMS) 코어 제한. import 前 설정.
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

        # ultralytics 지연 import(torch를 여기서만 끌어옴).
        from ultralytics import YOLO
        self.model = YOLO(str(self.cfg.model_path))

    def detect(self, frame_bgr: np.ndarray, want_debug: bool = False) -> TrafficCueResult:
        """@brief 프레임에서 신호등(초록/빨강)+팻말(좌/우) 통합 검출 → 프레임 단위 판정."""
        if frame_bgr is None or frame_bgr.size == 0:
            return TrafficCueResult()

        result = self.model(frame_bgr, conf=self.cfg.conf,
                             imgsz=self.cfg.imgsz, verbose=False)[0]

        green_conf = 0.0
        red_conf = 0.0
        left_conf = 0.0
        right_conf = 0.0
        sign_h_frac = 0.0
        sign_aspect = 0.0
        sign_gated = False
        red_h_frac = 0.0
        red_gated = False
        frame_h = float(frame_bgr.shape[0])
        boxes = result.boxes
        n = 0 if boxes is None else len(boxes)
        if boxes is not None and n > 0:
            # model(conf=...)가 이미 임계 이하를 걸러 남은 박스는 전부 유효.
            for cls_t, conf_t, xyxy_t in zip(boxes.cls.tolist(), boxes.conf.tolist(),
                                             boxes.xyxy.tolist()):
                c = int(cls_t)
                cf = float(conf_t)
                if c == self.cfg.class_green:
                    green_conf = max(green_conf, cf)
                elif c == self.cfg.class_red:
                    # 빨강: 거리(박스 높이)·신뢰도 게이트. 오검출 하나가 코스를 끝내므로
                    # 종료 신호등처럼 '가까이서 확실할 때'만 인정. (초록엔 미적용)
                    x1, y1, x2, y2 = (float(v) for v in xyxy_t)
                    rh = max(0.0, y2 - y1) / frame_h if frame_h > 1e-6 else 0.0
                    red_h_frac = max(red_h_frac, rh)
                    too_far = (self.cfg.red_min_box_h_frac > 0.0
                               and rh < self.cfg.red_min_box_h_frac)
                    too_weak = (self.cfg.red_min_conf > 0.0
                                and cf < self.cfg.red_min_conf)
                    if too_far or too_weak:
                        red_gated = True
                        continue
                    red_conf = max(red_conf, cf)
                elif c in (self.cfg.class_left, self.cfg.class_right):
                    # 팻말: 거리(박스 높이)·각도(w/h)·신뢰도 게이트를 통과한 박스만 방향으로 인정.
                    x1, y1, x2, y2 = (float(v) for v in xyxy_t)
                    bw = max(0.0, x2 - x1)
                    bh = max(0.0, y2 - y1)
                    h_frac = (bh / frame_h) if frame_h > 1e-6 else 0.0
                    aspect = (bw / bh) if bh > 1e-6 else 0.0
                    if h_frac > sign_h_frac:      # 로깅/튜닝용 최대 박스 실측치.
                        sign_h_frac, sign_aspect = h_frac, aspect
                    too_far = (self.cfg.sign_min_box_h_frac > 0.0
                               and h_frac < self.cfg.sign_min_box_h_frac)
                    too_skew = (self.cfg.sign_min_box_aspect > 0.0
                                and aspect < self.cfg.sign_min_box_aspect)
                    too_weak = (self.cfg.sign_min_conf > 0.0
                                and cf < self.cfg.sign_min_conf)
                    if too_far or too_skew or too_weak:
                        sign_gated = True         # 멀거나 비스듬하거나 애매 → 방향으로 안 씀.
                        continue
                    if c == self.cfg.class_left:
                        left_conf = max(left_conf, cf)
                    else:
                        right_conf = max(right_conf, cf)

        # 프레임 단위 신호등 판정: 둘 다면 prefer_red(또는 빨강이 더 확신).
        has_green = green_conf > 0.0
        has_red = red_conf > 0.0
        if has_red and has_green:
            light = TL_RED if (self.cfg.prefer_red or red_conf >= green_conf) else TL_GREEN
        elif has_red:
            light = TL_RED
        elif has_green:
            light = TL_GREEN
        else:
            light = TL_NONE

        # 프레임 단위 방향 판정: 좌/우 중 conf 높은 쪽(sign_detect 와 동일 관례). 둘 다 없으면 NONE.
        if left_conf == 0.0 and right_conf == 0.0:
            direction = SIGN_NONE
        elif right_conf >= left_conf:
            direction = SIGN_RIGHT
        else:
            direction = SIGN_LEFT

        debug = result.plot() if want_debug else None

        return TrafficCueResult(
            light=light,
            green_conf=green_conf,
            red_conf=red_conf,
            direction=direction,
            left_conf=left_conf,
            right_conf=right_conf,
            num_boxes=n,
            sign_h_frac=sign_h_frac,
            sign_aspect=sign_aspect,
            sign_gated=sign_gated,
            red_h_frac=red_h_frac,
            red_gated=red_gated,
            debug_image=debug,
        )
