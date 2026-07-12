"""@file traffic_light_detect.py
@brief 미션 신호(신호등·체커보드) YOLO 검출 (ROS-free) — 출발/도착 트리거.

@details
학습된 YOLO 모델(`best_ncnn_model`, 클래스 0:checker 1:green 2:red 3:stop_line)로
프레임에서 **신호등(초록/빨강)** 과 **체커보드**를 찾아 미션 신호를 낸다. 로직은 여기
(core)에 있고 ROS 노드는 구독/디코드/발행만 한다(하드코딩 금지·파라미터 YAML).

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


@dataclass
class TrafficCueConfig:
    """@brief 신호등/체커보드 YOLO 검출 파라미터(전부 YAML/CLI 조정)."""
    model_path: str = "best_ncnn_model"  ##< 노드가 해석해 넘기는 모델 경로(ncnn 디렉토리/.pt).
    conf: float = 0.35                    ##< 박스 인정 최소 신뢰도(이하 무시).
    imgsz: int = 320                      ##< 추론 입력 크기(정사각).
    class_checker: int = 0                ##< 체커보드 클래스 id.
    class_green: int = 1                  ##< 초록불 클래스 id.
    class_red: int = 2                    ##< 빨간불 클래스 id.
    prefer_red: bool = True               ##< 초록·빨강 동시 검출 시 RED로(출발 안전측).
    cv_threads: int = 1                   ##< OpenCV 스레드 상한(코어 경합 완화).
    torch_threads: int = 2                ##< torch 스레드 상한(NMS/후처리).


@dataclass
class TrafficCueResult:
    """@brief 한 프레임 신호 검출 결과."""
    light: int = TL_NONE          ##< 신호등 상태(TL_NONE/RED/GREEN, 프레임 단위 판정).
    green_conf: float = 0.0       ##< 검출된 초록 박스 최고 신뢰도(없으면 0).
    red_conf: float = 0.0         ##< 검출된 빨강 박스 최고 신뢰도(없으면 0).
    checker: bool = False         ##< 체커보드 검출 여부.
    checker_conf: float = 0.0     ##< 체커보드 최고 신뢰도(없으면 0).
    num_boxes: int = 0            ##< 전체 박스 개수(로깅용).
    debug_image: Optional[np.ndarray] = None  ##< 오버레이(want_debug=True일 때).


class TrafficLightDetector:
    """@brief BGR 프레임 → TrafficCueResult. 상태 없음(디바운스는 노드가 담당).

    @details 생성 시 YOLO 모델을 로드한다(무거우니 노드가 한 번만 만든다). detect()는
    프레임마다 초록/빨강/체커보드 박스를 모아 프레임 단위 신호를 낸다.
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
        """@brief 프레임에서 신호등(초록/빨강)+체커보드 검출 → 프레임 단위 신호 판정."""
        if frame_bgr is None or frame_bgr.size == 0:
            return TrafficCueResult()

        result = self.model(frame_bgr, conf=self.cfg.conf,
                             imgsz=self.cfg.imgsz, verbose=False)[0]

        green_conf = 0.0
        red_conf = 0.0
        checker_conf = 0.0
        boxes = result.boxes
        n = 0 if boxes is None else len(boxes)
        if boxes is not None and n > 0:
            # model(conf=...)가 이미 임계 이하를 걸러 남은 박스는 전부 유효.
            for cls_t, conf_t in zip(boxes.cls.tolist(), boxes.conf.tolist()):
                c = int(cls_t)
                cf = float(conf_t)
                if c == self.cfg.class_green:
                    green_conf = max(green_conf, cf)
                elif c == self.cfg.class_red:
                    red_conf = max(red_conf, cf)
                elif c == self.cfg.class_checker:
                    checker_conf = max(checker_conf, cf)

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

        debug = result.plot() if want_debug else None

        return TrafficCueResult(
            light=light,
            green_conf=green_conf,
            red_conf=red_conf,
            checker=checker_conf > 0.0,
            checker_conf=checker_conf,
            num_boxes=n,
            debug_image=debug,
        )
