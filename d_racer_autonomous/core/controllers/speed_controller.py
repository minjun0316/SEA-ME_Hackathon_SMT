"""@file speed_controller.py
@brief 속도 결정(곡률 기반) + 선택적 Speed PID.

@details
"조향과 속도를 분리한다" 원칙에 따라, 조향(Pure Pursuit)과 완전히
독립적으로 동작한다. 두 단계로 구성된다.

1. 목표 속도 결정: 경로 곡률에 따라 직선은 빠르게, 급커브는 느리게.
   @f[ v_{target} = \\operatorname{clip}(v_{max} - K_\\kappa |\\kappa|,\\; v_{min},\\; v_{max}) @f]
2. (선택) Speed PID: 목표 속도와 현재 속도의 오차로 throttle 산출.
   실차에서 throttle↔실제속도가 비선형일 때 의미가 있다. 순수 운동학
   시뮬에서는 보통 비활성(목표 속도를 그대로 모델에 입력).

@par 확장 가능성
State Machine은 상태별로 v_max 등을 바꿔 이 컨트롤러의 동작을 조절한다
(컨트롤러 자체는 모든 상태에서 동일, 파라미터만 변경).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config_schema import SpeedConfig


@dataclass
class SpeedResult:
    """@brief 속도 컨트롤러 출력.

    @var target_speed  곡률 기반 목표 속도.
    @var command       모델/액추에이터에 줄 최종 명령(throttle 또는 속도).
    """

    target_speed: float
    command: float


class SpeedController:
    """@brief 곡률 기반 목표 속도 + 선택적 Speed PID 컨트롤러."""

    def __init__(self, config: SpeedConfig):
        """@param config 속도 튜닝 파라미터."""
        self.cfg = config
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self) -> None:
        """@brief PID 내부 상태 초기화."""
        self._integral = 0.0
        self._prev_error = 0.0

    def target_speed(self, curvature: float) -> float:
        """@brief 곡률로부터 목표 속도를 계산한다.

        @param curvature 경로 곡률 κ [1/m].
        @return 목표 속도 (clip 적용).
        """
        v = self.cfg.v_max - self.cfg.curvature_gain * abs(curvature)
        return float(np.clip(v, self.cfg.v_min, self.cfg.v_max))

    def compute(self, curvature: float, current_speed: float, dt: float) -> SpeedResult:
        """@brief 목표 속도 및 최종 명령을 계산한다.

        @param curvature     경로 곡률 κ.
        @param current_speed 현재 측정 속도(또는 모델 속도).
        @param dt            시간 간격 [s].
        @return SpeedResult.

        @details use_speed_pid=False면 목표 속도를 그대로 명령으로 출력한다
        (운동학 시뮬 / 개루프 throttle). True면 PID로 보정한다.
        """
        target = self.target_speed(curvature)

        if not self.cfg.use_speed_pid:
            return SpeedResult(target_speed=target, command=target)

        error = target - current_speed
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 1e-9 else 0.0
        self._prev_error = error

        command = (self.cfg.kp * error
                   + self.cfg.ki * self._integral
                   + self.cfg.kd * derivative)
        # throttle 출력은 [0, v_max]로 클램프(후진은 미션에서 별도 처리).
        command = float(np.clip(command, 0.0, self.cfg.v_max))
        return SpeedResult(target_speed=target, command=command)
