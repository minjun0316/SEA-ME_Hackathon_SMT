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


def _arc(radius, left, n=400):
    """전방(+x) 진행하며 좌(+y)/우(−y)로 도는 1/4원 호. left=True면 좌회전."""
    theta = np.linspace(0.0, math.pi / 2, n)
    x = radius * np.sin(theta)
    y = (1.0 if left else -1.0) * radius * (1.0 - np.cos(theta))
    return Path(np.column_stack([x, y]), resample_spacing=0.02)


def test_signed_curvature_sign_left_positive_right_negative():
    """+y=좌측 좌표계: 좌회전(반시계) κ>0, 우회전 κ<0. 피드포워드 방향의 근거."""
    left = _arc(2.0, left=True)
    right = _arc(2.0, left=False)
    kl = np.median(left.signed_curvatures[5:-5])
    kr = np.median(right.signed_curvatures[5:-5])
    assert kl > 0.0 and math.isclose(kl, 0.5, rel_tol=0.1)   # 1/R = 0.5
    assert kr < 0.0 and math.isclose(kr, -0.5, rel_tol=0.1)


def test_signed_curvature_magnitude_matches_abs():
    """부호곡률의 절댓값 = 크기 곡률(curvatures)."""
    p = _arc(2.0, left=False)
    assert np.allclose(np.abs(p.signed_curvatures), p.curvatures)


def test_mean_signed_curvature_ahead_matches_direction():
    """근거리 평균 부호곡률: 좌회전 호에서 +값(피드포워드가 좌조향을 얹도록)."""
    left = _arc(2.0, left=True)
    k = left.mean_signed_curvature_ahead(0, 0.5)
    assert k > 0.0


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
