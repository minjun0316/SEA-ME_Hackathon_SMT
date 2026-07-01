"""@file vehicle_model.py
@brief Stage 2: 운동학적 자전거 모델(Kinematic Bicycle Model).

@details
실차 없이 컨트롤러를 검증하기 위한 가상 차량이다. 후륜축 중심을
기준점으로 하는 표준 자전거 모델을 사용한다.

@par 상태/입력
- 상태: (x, y, yaw)
- 입력: (speed v [m/s], steering δ [rad])

@par 운동 방정식 (후륜축 기준)
@f[
    \\dot{x} = v\\cos\\psi,\\quad
    \\dot{y} = v\\sin\\psi,\\quad
    \\dot{\\psi} = \\frac{v}{L}\\tan\\delta
@f]
여기서 L은 휠베이스. 적분은 단순 오일러 전진법을 쓴다(작은 dt에서 충분).

@par 설계 이유
컨트롤러가 출력하는 것은 [-1, 1] 정규화 명령이지만, 차량 동역학은
물리 단위(rad, m/s)로 계산해야 한다. 정규화↔물리 변환은 컨트롤러가
담당하고, 이 모델은 순수하게 물리 입력만 받는다 (책임 분리).
"""
from __future__ import annotations

import math

from .geometry import Pose2D, normalize_angle


class KinematicBicycleModel:
    """@brief 후륜축 기준 운동학 자전거 모델."""

    def __init__(self, wheelbase: float, max_steer_rad: float,
                 speed_tau: float = 0.0):
        """@brief 모델 초기화.

        @param wheelbase     휠베이스 L [m].
        @param max_steer_rad 물리 조향각 한계 [rad].
        @param speed_tau     1차 속도 지연 시상수 [s]. 0이면 명령 속도가
                             즉시 반영된다. >0이면 SpeedController의 Speed
                             PID가 의미를 갖도록 1차 지연을 모사한다.
        """
        self.wheelbase = float(wheelbase)
        self.max_steer_rad = float(max_steer_rad)
        self.speed_tau = float(speed_tau)

        self.pose = Pose2D()
        self.speed = 0.0  # 현재 실제 속도 [m/s]

    def reset(self, pose: Pose2D, speed: float = 0.0) -> None:
        """@brief 모델 상태를 초기화한다.

        @param pose  초기 자세.
        @param speed 초기 속도 [m/s].
        """
        self.pose = Pose2D(pose.x, pose.y, pose.yaw)
        self.speed = float(speed)

    def step(self, speed_cmd: float, steer_cmd_rad: float, dt: float) -> Pose2D:
        """@brief 한 스텝(dt) 동안 차량 상태를 전진 적분한다.

        @param speed_cmd     명령 속도 [m/s].
        @param steer_cmd_rad 명령 조향각 [rad] (내부에서 한계로 포화).
        @param dt            시간 간격 [s].
        @return 갱신된 자세 Pose2D.
        """
        # 1) 조향 포화 (액추에이터 물리 한계).
        delta = max(-self.max_steer_rad, min(self.max_steer_rad, steer_cmd_rad))

        # 2) 속도 1차 지연 (선택적). tau=0이면 즉시 반영.
        if self.speed_tau > 1e-6:
            alpha = dt / (self.speed_tau + dt)
            self.speed += alpha * (speed_cmd - self.speed)
        else:
            self.speed = speed_cmd

        # 3) 운동학 적분 (오일러).
        v = self.speed
        self.pose.x += v * math.cos(self.pose.yaw) * dt
        self.pose.y += v * math.sin(self.pose.yaw) * dt
        self.pose.yaw = normalize_angle(
            self.pose.yaw + (v / self.wheelbase) * math.tan(delta) * dt)
        return self.pose
