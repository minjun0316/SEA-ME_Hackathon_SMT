"""@file test_pure_pursuit.py
@brief Pure Pursuit / Speed / Battery 컨트롤러 + 폐루프 시뮬 테스트."""
import math

from core import path_factory
from core.config_schema import (AppConfig, BatteryConfig, PurePursuitConfig,
                                SpeedConfig, VehicleConfig)
from core.controllers import (BatteryCompensator, PurePursuitController,
                              SpeedController)
from core.geometry import Pose2D
from sim.simulator import Simulator


def _vehicle():
    return VehicleConfig(wheelbase=0.26, max_steer_deg=24.0)


def test_straight_centered_gives_zero_steer():
    """직선 경로 위 정중앙·정렬 차량은 조향이 ~0."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(steering_smoothing=1.0))
    path = path_factory.straight(length=5.0)
    res = ctrl.compute(Pose2D(0.0, 0.0, 0.0), path, speed=0.5)
    assert abs(res.steering_norm) < 1e-3


def test_offset_left_steers_right():
    """경로 좌측에 치우친 차량은 우측(음수)으로 조향해 복귀해야 한다."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(steering_smoothing=1.0))
    path = path_factory.straight(length=5.0)
    res = ctrl.compute(Pose2D(0.0, 0.3, 0.0), path, speed=0.5)
    assert res.steering_norm < 0


def test_steering_saturates_within_unit():
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(steering_smoothing=1.0))
    path = path_factory.circle(radius=0.3)  # 매우 급한 곡률
    res = ctrl.compute(Pose2D(0.0, 0.5, 0.0), path, speed=0.5)
    assert -1.0 <= res.steering_norm <= 1.0


def test_adaptive_lookahead_increases_with_speed():
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(
        use_adaptive_lookahead=True, ld_min=0.3, ld_max=2.0, kv=0.4, kc=0.0))
    ld_slow = ctrl.compute_lookahead(speed=0.1, curvature=0.0)
    ld_fast = ctrl.compute_lookahead(speed=1.0, curvature=0.0)
    assert ld_fast > ld_slow


def test_speed_controller_slows_in_curves():
    sc = SpeedController(SpeedConfig(v_max=0.8, v_min=0.3, curvature_gain=0.5))
    assert sc.target_speed(0.0) == 0.8
    assert sc.target_speed(2.0) < 0.8


def test_battery_compensation_boosts_low_voltage():
    bc = BatteryCompensator(BatteryConfig(
        enabled=True, v_nominal=7.4, v_min=6.4, v_max=8.4, max_gain=1.5))
    assert math.isclose(bc.gain(7.4), 1.0, rel_tol=1e-6)
    assert bc.gain(6.4) > 1.0  # 전압 낮으면 boost
    assert bc.gain(6.4) <= 1.5  # 상한 준수


def test_battery_disabled_is_noop():
    bc = BatteryCompensator(BatteryConfig(enabled=False))
    assert bc.compensate(0.5, 6.0) == 0.5


def test_closed_loop_follows_s_curve():
    """폐루프 시뮬: S자 경로를 낮은 CTE로 완주해야 한다(통합 테스트)."""
    cfg = AppConfig(
        vehicle=_vehicle(),
        pure_pursuit=PurePursuitConfig(use_adaptive_lookahead=True),
        speed=SpeedConfig(v_max=0.8, v_min=0.3),
    )
    cfg.sim.path_name = "s_curve"
    path = path_factory.make_path("s_curve")
    summary = Simulator(cfg, path).run()
    assert summary.reached_goal
    assert summary.rms_cte < 0.15  # 15cm 이내 추종
