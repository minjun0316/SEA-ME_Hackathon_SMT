"""@file simulator.py
@brief Stage 2/3: 폐루프 시뮬레이션 엔진.

@details
다음 폐루프를 dt 간격으로 반복한다.

@verbatim
  ┌─────────────────────────────────────────────────────────────┐
  │  KinematicBicycleModel.pose                                  │
  │        │                                                     │
  │        ▼                                                     │
  │  SpeedController  ──► target speed (곡률 기반)              │
  │  PurePursuit      ──► steering (정규화)                     │
  │  BatteryCompensator ─► throttle 보정                         │
  │        │                                                     │
  │        ▼                                                     │
  │  KinematicBicycleModel.step(speed, steering, dt)            │
  └─────────────────────────────────────────────────────────────┘
@endverbatim

이 엔진은 ROS가 없어 PC에서 즉시 실행되며, 그대로 두고 컨트롤러만
튜닝(Stage 3)한다. Stage 6에서 동일한 컨트롤러 객체가 ROS2 노드로
감싸진다.

@par 좌표계 주의
시뮬에서는 글로벌 프레임을 쓰며 모델이 ground-truth pose를 제공한다.
실차(카메라 단독)에서는 매 프레임 로컬 경로 + 원점 pose가 들어오지만,
컨트롤러 인터페이스가 동일하므로 코드 변경이 없다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core.config_schema import AppConfig
from core.control import (BatteryCompensator, PurePursuitController,
                              SpeedController)
from core.geometry import Pose2D
from core.path import Path
from core.vehicle_model import KinematicBicycleModel

from .logger import DriveLogger


@dataclass
class SimSummary:
    """@brief 시뮬 종료 후 정량 요약(튜닝 비교용).

    @var reached_goal 경로 끝 도달 여부.
    @var steps        진행한 스텝 수.
    @var sim_time     소요 시뮬 시간 [s].
    @var rms_cte      cross-track error RMS [m] (추종 정확도 핵심 지표).
    @var max_cte      cross-track error 최대 절댓값 [m].
    """

    reached_goal: bool
    steps: int
    sim_time: float
    rms_cte: float
    max_cte: float


class Simulator:
    """@brief 정적 경로를 추종하는 폐루프 시뮬레이터."""

    def __init__(self, config: AppConfig, path: Path,
                 mission_state: str = "LANE_FOLLOW"):
        """@brief 시뮬레이터 구성.

        @param config        전체 설정(AppConfig).
        @param path          추종할 정적 경로.
        @param mission_state 로그에 기록할 미션 상태 라벨(Stage 1~5에선 고정).
        """
        self.cfg = config
        self.path = path
        self.mission_state = mission_state

        self.model = KinematicBicycleModel(
            wheelbase=config.vehicle.wheelbase,
            max_steer_rad=config.vehicle.max_steer_rad,
            speed_tau=config.vehicle.speed_tau,
        )
        self.steering_ctrl = PurePursuitController(config.vehicle, config.pure_pursuit)
        self.speed_ctrl = SpeedController(config.speed)
        self.battery = BatteryCompensator(config.battery)

        self.logger = DriveLogger()

    def _initial_pose(self) -> Pose2D:
        """@brief 경로 시작점/접선에 맞춘 초기 차량 자세."""
        p0 = self.path.points[0]
        yaw0 = self.path.heading_at(0)
        return Pose2D(float(p0[0]), float(p0[1]), yaw0)

    def run(self) -> SimSummary:
        """@brief 시뮬레이션을 끝까지 실행하고 로그/요약을 만든다.

        @return SimSummary 정량 요약.
        """
        self.model.reset(self._initial_pose(), speed=0.0)
        self.steering_ctrl.reset()
        self.speed_ctrl.reset()

        dt = self.cfg.sim.dt
        max_steps = int(self.cfg.sim.max_time / dt)
        goal = self.path.points[-1]
        voltage = self.cfg.sim.battery_voltage

        cte_history = []
        reached = False
        step = 0

        for step in range(max_steps):
            pose = self.model.pose

            # --- 제어 계산 (조향/속도 분리) ---
            steer = self.steering_ctrl.compute(pose, self.path, self.model.speed)
            speed = self.speed_ctrl.compute(steer.curvature, self.model.speed, dt)
            throttle = self.battery.compensate(speed.command, voltage)

            # --- 로깅 (Stage 4 스키마) ---
            # 컨트롤러가 찾은 진행 인덱스를 재사용해 오차를 일관되게 계산.
            cte = self.path.cross_track_error_at(pose.x, pose.y, steer.nearest_index)
            he = self.path.heading_error(pose.yaw, steer.nearest_index)
            cte_history.append(cte)
            self.logger.log(
                time=round(step * dt, 4),
                x=round(pose.x, 5), y=round(pose.y, 5), yaw=round(pose.yaw, 5),
                target_x=round(float(steer.target_point[0]), 5),
                target_y=round(float(steer.target_point[1]), 5),
                lookahead=round(steer.lookahead, 5),
                curvature=round(steer.curvature, 5),
                cross_track_error=round(cte, 5),
                heading_error=round(he, 5),
                steering_cmd=round(steer.steering_norm, 5),
                speed_cmd=round(throttle, 5),
                battery_voltage=round(voltage, 3),
                mission_state=self.mission_state,
            )

            # --- 차량 전진 ---
            # 운동학 시뮬에서 throttle은 목표 속도(정규화 ≈ m/s)로 해석한다.
            self.model.step(throttle, steer.steering_rad, dt)

            # --- 종료 판정: 진행 인덱스가 경로 끝에 도달했을 때만 ---
            # 진행 인덱스(단조 전진)를 쓰므로, 닫힌/자기교차 경로에서
            # 시작점 부근을 지나도 조기 종료하지 않는다.
            if (steer.nearest_index >= len(self.path) - 2
                    and np.hypot(pose.x - goal[0], pose.y - goal[1])
                    < self.cfg.sim.goal_tolerance):
                reached = True
                break

        cte_arr = np.asarray(cte_history) if cte_history else np.zeros(1)
        return SimSummary(
            reached_goal=reached,
            steps=step + 1,
            sim_time=(step + 1) * dt,
            rms_cte=float(np.sqrt(np.mean(cte_arr ** 2))),
            max_cte=float(np.max(np.abs(cte_arr))),
        )
