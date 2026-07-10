"""@file test_path.py
@brief path / path_factory 단위 테스트."""
import math

import numpy as np

from core import path_factory
from core.path import Path


def test_straight_path_has_zero_curvature():
    p = path_factory.straight(length=5.0)
    # 끝점은 차분이 불안정하므로 내부 구간만 검사.
    assert np.all(np.abs(p.curvatures[2:-2]) < 1e-6)


def test_circle_curvature_matches_inverse_radius():
    radius = 2.0
    p = path_factory.circle(radius=radius, n=800)
    kappa = np.median(p.curvatures[5:-5])
    assert math.isclose(kappa, 1.0 / radius, rel_tol=0.05)


def test_lookahead_advances_by_arc_length():
    p = path_factory.straight(length=5.0, spacing=0.05)
    target = p.lookahead_point(0, 1.0)
    # 직선이므로 x≈시작점+1.0
    assert math.isclose(target[0], p.points[0, 0] + 1.0, abs_tol=0.06)


def _straight_then_arc(radius=0.5):
    """앞 2m 직선 + 뒤 90° 원호(R) 경로. 진입 전 preview 검증용."""
    xs = np.linspace(0.0, 2.0, 60)
    straight = np.column_stack([xs, np.zeros_like(xs)])
    th = np.linspace(0.0, math.pi / 2, 60)
    arc = np.column_stack([2.0 + radius * np.sin(th), radius - radius * np.cos(th)])
    return Path(np.vstack([straight, arc]))


def test_max_abs_curvature_ahead_sees_curve_before_arrival():
    p = _straight_then_arc(radius=0.5)  # 원호 κ≈2.0
    near0 = p.max_abs_curvature_ahead(0, 0.3)   # 직선 안만 봄 → 거의 0
    ahead = p.max_abs_curvature_ahead(0, 5.0)   # 전방 전체 → 원호 곡률 감지
    assert near0 < 0.5
    assert ahead > 1.0
    assert ahead >= abs(p.curvature_at(0))


def test_max_abs_curvature_ahead_zero_distance_is_pointwise():
    p = path_factory.circle(radius=2.0, n=800)
    assert math.isclose(
        p.max_abs_curvature_ahead(100, 0.0), abs(p.curvature_at(100)), rel_tol=1e-9)


def test_lookahead_clamps_at_path_end():
    p = path_factory.straight(length=2.0)
    target = p.lookahead_point(0, 999.0)
    assert np.allclose(target, p.points[-1])


def test_cross_track_error_sign():
    """직선 경로(+X) 위, 좌측(+Y)에 있는 차량은 CTE가 양수."""
    p = path_factory.straight(length=5.0)
    cte = p.cross_track_error(2.0, 0.3, 0.0)
    assert cte > 0
    cte_right = p.cross_track_error(2.0, -0.3, 0.0)
    assert cte_right < 0


def test_resample_is_uniform():
    p = path_factory.s_curve()
    seg = np.sqrt(np.sum(np.diff(p.points, axis=0) ** 2, axis=1))
    assert np.std(seg) < 0.01  # 거의 균일 간격
