"""@file geometry.py
@brief 2D 기하 유틸리티 (좌표계 변환, 각도 정규화, 거리).

@details
이 모듈은 시스템 전체에서 공유하는 가장 낮은 수준의 수학 도구만 담는다.
의도적으로 ROS / numpy 외부 의존성을 두지 않으며, 어떤 상위 모듈
(Perception / Planning / Control)도 여기에 자유롭게 의존할 수 있다.

@par 설계 이유
Pure Pursuit는 "차량 로컬 좌표계"에서 lookahead point의 (x, y)가 필요하다.
글로벌 프레임(시뮬레이션)과 로컬 프레임(실차, 카메라 단독)을 동일한
컨트롤러 코드로 다루기 위해, 좌표 변환을 한 곳에 모아둔다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def normalize_angle(angle: float) -> float:
    """@brief 각도를 (-pi, pi] 범위로 정규화한다.

    @param angle 라디안 단위 각도.
    @return (-pi, pi] 범위로 감싼 각도.

    @note heading error 계산 시 +179°와 -179°가 358°가 아니라 2°로
          취급되도록 하기 위해 반드시 필요하다.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Pose2D:
    """@brief 평면상 차량 자세 (위치 + 방향).

    @var x   글로벌(또는 기준) 프레임 X [m].
    @var y   글로벌(또는 기준) 프레임 Y [m].
    @var yaw 진행 방향 [rad], +X축 기준 반시계가 양수.
    """

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_array(self) -> np.ndarray:
        """@brief (x, y) 위치를 numpy 배열로 반환."""
        return np.array([self.x, self.y], dtype=float)


def distance(ax: float, ay: float, bx: float, by: float) -> float:
    """@brief 두 점 사이 유클리드 거리."""
    return math.hypot(bx - ax, by - ay)


def to_local_frame(pose: Pose2D, point: np.ndarray) -> np.ndarray:
    """@brief 글로벌 좌표의 점을 차량 로컬 좌표계로 변환한다.

    @details
    로컬 프레임은 차량을 원점에 두고 +X를 차량 전방, +Y를 좌측으로 한다.
    변환식 (R은 -yaw 회전):
    @f[
        \\begin{bmatrix} x_{local} \\\\ y_{local} \\end{bmatrix}
        = R(-\\psi)\\,
        \\begin{bmatrix} x_g - x_v \\\\ y_g - y_v \\end{bmatrix}
    @f]

    @param pose  글로벌 프레임에서의 차량 자세.
    @param point 변환할 점 [x, y] (글로벌 프레임).
    @return 차량 로컬 프레임에서의 점 [x_local, y_local].

    @note 실차(카메라 단독, 로컬 프레임)에서는 pose가 항상 원점이므로
          이 변환이 항등(identity)이 되어 동일 코드가 그대로 동작한다.
    """
    dx = point[0] - pose.x
    dy = point[1] - pose.y
    cos_y = math.cos(-pose.yaw)
    sin_y = math.sin(-pose.yaw)
    return np.array([cos_y * dx - sin_y * dy,
                     sin_y * dx + cos_y * dy], dtype=float)


def to_global_frame(pose: Pose2D, point: np.ndarray) -> np.ndarray:
    """@brief 차량 로컬 좌표의 점을 글로벌 좌표계로 변환한다 (to_local_frame의 역변환).

    @param pose  글로벌 프레임에서의 차량 자세.
    @param point 차량 로컬 프레임에서의 점 [x_local, y_local].
    @return 글로벌 프레임에서의 점 [x, y].
    """
    cos_y = math.cos(pose.yaw)
    sin_y = math.sin(pose.yaw)
    gx = cos_y * point[0] - sin_y * point[1] + pose.x
    gy = sin_y * point[0] + cos_y * point[1] + pose.y
    return np.array([gx, gy], dtype=float)
