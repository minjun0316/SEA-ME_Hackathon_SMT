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
