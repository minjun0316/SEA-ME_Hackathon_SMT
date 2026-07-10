"""@file test_pure_pursuit.py
@brief Pure Pursuit / Speed / Battery 컨트롤러 + 폐루프 시뮬 테스트."""
import math

import numpy as np

from core import path_factory
from core.config_schema import (AppConfig, BatteryConfig, PurePursuitConfig,
                                SpeedConfig, VehicleConfig)
from core.control import (BatteryCompensator, PurePursuitController,
                              SpeedController)
from core.geometry import Pose2D
from core.path import Path
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
    ld_slow = ctrl.compute_lookahead(speed=0.1, kappa_eff=0.0)
    ld_fast = ctrl.compute_lookahead(speed=1.0, kappa_eff=0.0)
    assert ld_fast > ld_slow


def test_adaptive_lookahead_shortens_in_curves():
    """곡선(κ_eff>0)에선 Ld가 짧아진다(코너컷 방지·민첩)."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(
        use_adaptive_lookahead=True, ld_min=0.3, ld_max=2.0, kv=0.2, kc=0.15))
    ld_straight = ctrl.compute_lookahead(speed=0.8, kappa_eff=0.0)
    ld_curve = ctrl.compute_lookahead(speed=0.8, kappa_eff=1.0)
    assert ld_curve < ld_straight


def test_preview_shrinks_lookahead_before_curve():
    """curvature_preview>0면 곡선 진입 전(직선 위)에도 앞의 곡률을 봐 Ld가 짧아진다."""
    # 짧은 직선(0.5m) 뒤 급커브 → preview 1.0m면 진입 전에 곡선을 봄.
    xs = np.linspace(0.0, 0.5, 20)
    straight = np.column_stack([xs, np.zeros_like(xs)])
    th = np.linspace(0.0, math.pi / 2, 60)
    r = 0.5
    arc = np.column_stack([0.5 + r * np.sin(th), r - r * np.cos(th)])
    path = Path(np.vstack([straight, arc]))

    def _ld(preview):
        cfg = PurePursuitConfig(
            use_adaptive_lookahead=True, ld_min=0.45, ld_max=1.2, kv=0.2, kc=0.2,
            curvature_smoothing=1.0, curvature_deadband=0.3, curvature_preview=preview)
        ctrl = PurePursuitController(_vehicle(), cfg)
        return ctrl.compute(Pose2D(0.0, 0.0, 0.0), path, speed=0.8).lookahead

    assert _ld(1.0) < _ld(0.0)  # 앞의 곡선을 보면 Ld 수축(턴인 지연)


def test_smooth_kappa_deadbands_straight_noise():
    """직선 곡률 노이즈(<deadband)는 κ_eff=0으로 격리, 초과분만 유효."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(
        curvature_deadband=0.3, curvature_smoothing=1.0))  # α=1.0 → EMA=|κ| 즉시
    assert ctrl._smooth_kappa(0.0) == 0.0
    ctrl.reset()
    assert ctrl._smooth_kappa(0.2) == 0.0            # 데드밴드 이하 → 격리
    ctrl.reset()
    assert math.isclose(ctrl._smooth_kappa(1.0), 0.7, rel_tol=1e-6)  # 초과분 0.7


def test_curvature_gain_disabled_is_constant():
    """use_curvature_gain=False면 곡률과 무관하게 상수 steering_gain."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(
        use_curvature_gain=False, steering_gain=0.8))
    assert ctrl.compute_gain(0.0) == 0.8
    assert ctrl.compute_gain(2.0) == 0.8


def test_curvature_gain_straight_stays_baseline():
    """κ_eff=0(직선/데드밴드 격리됨)이면 baseline gain 유지."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(
        use_curvature_gain=True, steering_gain=0.62, steering_gain_kappa=0.25,
        steering_gain_max=1.0))
    assert math.isclose(ctrl.compute_gain(0.0), 0.62, rel_tol=1e-6)


def test_curvature_gain_rises_in_curves():
    """κ_eff가 양수면 gain이 baseline 위로 올라간다."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(
        use_curvature_gain=True, steering_gain=0.62, steering_gain_kappa=0.25,
        steering_gain_max=1.0))
    # κ_eff=0.7 → 0.62 + 0.25*0.7 = 0.795.
    assert math.isclose(ctrl.compute_gain(0.7), 0.795, rel_tol=1e-6)


def test_curvature_gain_clamped_to_max():
    """가산 후에도 steering_gain_max를 넘지 않는다."""
    ctrl = PurePursuitController(_vehicle(), PurePursuitConfig(
        use_curvature_gain=True, steering_gain=0.62, steering_gain_kappa=0.25,
        steering_gain_max=1.0))
    assert ctrl.compute_gain(10.0) == 1.0  # 큰 κ_eff → 상한 클램프


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
