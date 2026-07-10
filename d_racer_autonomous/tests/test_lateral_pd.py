"""@file test_lateral_pd.py
@brief 근거리 차선오차 PD(+heading) 횡제어 단위 테스트."""
import math

from core.config_schema import LateralPDConfig, VehicleConfig
from core.control import LateralPDController


def _vehicle(trim=0.2238):
    return VehicleConfig(wheelbase=0.26, max_steer_deg=24.0, steer_trim=trim)


def _ctrl(trim=0.0, **cfg):
    # 트림 0으로 두면 조향 부호/크기를 트림 오프셋 없이 직접 검증할 수 있다.
    base = dict(k_cross=1.2, k_heading=0.8, k_deriv=0.0, steering_smoothing=1.0)
    base.update(cfg)
    return LateralPDController(_vehicle(trim), LateralPDConfig(**base))


def test_centered_and_aligned_gives_trim():
    """오차 0·heading 0이면 조향 = 직진 트림."""
    ctrl = _ctrl(trim=0.2238)
    res = ctrl.compute(0.0, 0.0, dt=0.1)
    assert math.isclose(res.steering_norm, 0.2238, rel_tol=1e-6)


def test_lane_to_left_steers_left():
    """차선중심이 좌측(+offset)이면 좌회전(+조향)해서 복귀."""
    res = _ctrl().compute(0.2, 0.0, dt=0.1)
    assert res.steering_norm > 0.0
    assert math.isclose(res.p_cross, 1.2 * 0.2, rel_tol=1e-6)


def test_lane_to_right_steers_right():
    res = _ctrl().compute(-0.2, 0.0, dt=0.1)
    assert res.steering_norm < 0.0


def test_heading_error_contributes():
    """heading 오차만 있어도 정렬 방향으로 조향."""
    res = _ctrl().compute(0.0, 0.3, dt=0.1)
    assert math.isclose(res.p_heading, 0.8 * 0.3, rel_tol=1e-6)
    assert res.steering_norm > 0.0


def test_steering_sign_flips_direction():
    """steering_sign=-1이면 같은 오차에 반대로 조향(차량 부호 반전 대응)."""
    pos = _ctrl(steering_sign=1.0).compute(0.2, 0.1, dt=0.1).steering_norm
    neg = _ctrl(steering_sign=-1.0).compute(0.2, 0.1, dt=0.1).steering_norm
    assert math.isclose(pos, -neg, abs_tol=1e-9)


def test_max_offset_clamps_spike():
    """오검출로 offset이 튀어도 max_offset으로 클램프되어 과조향 방지."""
    res = _ctrl(k_cross=1.0, max_offset=0.5).compute(10.0, 0.0, dt=0.1)
    assert math.isclose(res.lateral_offset, 0.5, rel_tol=1e-6)


def test_output_saturates_within_unit():
    res = _ctrl(trim=0.2238, k_cross=50.0).compute(0.4, 0.0, dt=0.1)
    assert -1.0 <= res.steering_norm <= 1.0


def test_smoothing_lowpasses_output():
    """β<1이면 첫 스텝 출력이 목표에 못 미친다(저역통과)."""
    ctrl = _ctrl(trim=0.0, k_cross=1.0, steering_smoothing=0.3)
    first = ctrl.compute(0.5, 0.0, dt=0.1).steering_norm  # target=0.5, prev=0 → 0.15
    assert 0.0 < first < 0.5
    assert math.isclose(first, 0.3 * 0.5, rel_tol=1e-6)


def test_derivative_filtered_and_zero_first_step():
    """첫 스텝은 미분 0(이전값 없음). 이후 EMA로 필터된 미분이 반영."""
    ctrl = _ctrl(k_cross=0.0, k_heading=0.0, k_deriv=1.0, deriv_smoothing=1.0)
    r0 = ctrl.compute(0.1, 0.0, dt=0.1)
    assert math.isclose(r0.d_cross, 0.0, abs_tol=1e-9)  # 첫 스텝 미분 없음
    r1 = ctrl.compute(0.2, 0.0, dt=0.1)                 # Δe=0.1, dt=0.1 → de/dt=1.0
    assert r1.d_cross > 0.0
