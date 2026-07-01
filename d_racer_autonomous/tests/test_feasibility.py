"""@file test_feasibility.py
@brief 경로 실현가능성 검사 단위 테스트."""
import math

from core import path_factory
from core.config_schema import VehicleConfig
from core.feasibility import check_path, min_turn_radius


def _vehicle():
    return VehicleConfig(wheelbase=0.26, max_steer_deg=24.0)


def test_min_turn_radius_formula():
    v = _vehicle()
    expected = v.wheelbase / math.tan(v.max_steer_rad)
    assert math.isclose(min_turn_radius(v), expected, rel_tol=1e-9)


def test_default_stress_paths_are_feasible():
    """기본 경로들은 기본 차량으로 추종 가능해야 한다(물리적으로 의미있는 테스트)."""
    v = _vehicle()
    for name in ["straight", "circle", "s_curve", "sharp_s",
                 "figure_eight", "rotary"]:
        report = check_path(path_factory.make_path(name), v)
        assert report.feasible, f"{name} infeasible: {report.summary()}"


def test_too_tight_circle_is_infeasible():
    """차량 최소 회전반경보다 작은 원은 불가능으로 판정돼야 한다."""
    v = _vehicle()
    tight = path_factory.circle(radius=0.2)  # 20cm < 58cm 최소반경
    report = check_path(tight, v)
    assert not report.feasible
    assert report.margin < 1.0
