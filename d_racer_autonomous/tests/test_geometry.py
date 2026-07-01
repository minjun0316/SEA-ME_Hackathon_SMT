"""@file test_geometry.py
@brief geometry 모듈 단위 테스트."""
import math

import numpy as np

from core.geometry import (Pose2D, normalize_angle, to_global_frame,
                           to_local_frame)


def test_normalize_angle_wraps_to_pi_range():
    # ±pi는 동일 각도이므로 절댓값으로 비교(atan2 정규화는 경계에서 -pi 반환 가능).
    assert math.isclose(abs(normalize_angle(3 * math.pi)), math.pi, abs_tol=1e-9)
    assert math.isclose(abs(normalize_angle(-3 * math.pi)), math.pi, abs_tol=1e-9)
    assert math.isclose(normalize_angle(0.5), 0.5, abs_tol=1e-9)
    assert math.isclose(normalize_angle(2 * math.pi + 0.3), 0.3, abs_tol=1e-9)


def test_local_frame_identity_at_origin():
    """원점·무회전 pose에서 로컬 변환은 항등(실차 로컬 프레임 가정)."""
    pose = Pose2D(0.0, 0.0, 0.0)
    p = np.array([2.0, 1.0])
    assert np.allclose(to_local_frame(pose, p), p)


def test_local_global_roundtrip():
    """to_local → to_global 왕복은 원래 점을 복원해야 한다."""
    pose = Pose2D(1.5, -2.0, 0.7)
    p = np.array([3.0, 4.0])
    local = to_local_frame(pose, p)
    back = to_global_frame(pose, local)
    assert np.allclose(back, p, atol=1e-9)


def test_point_ahead_is_positive_x_in_local():
    """차량 정면(글로벌 +Y, yaw=90°)의 점은 로컬 +X여야 한다."""
    pose = Pose2D(0.0, 0.0, math.pi / 2)
    p = np.array([0.0, 2.0])  # 차량 바로 앞
    local = to_local_frame(pose, p)
    assert local[0] > 1.9 and abs(local[1]) < 1e-9
