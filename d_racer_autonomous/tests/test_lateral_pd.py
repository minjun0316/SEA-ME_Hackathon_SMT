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


def test_curvature_feedforward_off_by_default():
    """k_ff=0(기본)이면 κ를 줘도 피드포워드 기여 0 — 기존 순수 PD와 동일."""
    ctrl = _ctrl(k_cross=0.0, k_heading=0.0)  # k_ff 기본 0
    res = ctrl.compute(0.0, 0.0, dt=0.1, curvature=1.0)
    assert math.isclose(res.p_ff, 0.0, abs_tol=1e-9)
    assert math.isclose(res.steering_norm, 0.0, abs_tol=1e-9)


def test_curvature_feedforward_steers_toward_curve():
    """좌커브(+κ)면 +조향(좌), 우커브(−κ)면 −조향 — 오차 없이 커브 유지 조향."""
    left = _ctrl(k_cross=0.0, k_heading=0.0, k_ff=0.5,
                 curvature_smoothing=1.0).compute(0.0, 0.0, dt=0.1, curvature=1.0)
    assert left.p_ff > 0.0 and left.steering_norm > 0.0
    right = _ctrl(k_cross=0.0, k_heading=0.0, k_ff=0.5,
                  curvature_smoothing=1.0).compute(0.0, 0.0, dt=0.1, curvature=-1.0)
    assert right.p_ff < 0.0 and right.steering_norm < 0.0


def test_curvature_deadband_isolates_straight_noise():
    """데드밴드 이하 |κ|는 직선 취급 → ff 0(직선 곡률노이즈 격리)."""
    ctrl = _ctrl(k_cross=0.0, k_heading=0.0, k_ff=1.0,
                 curvature_smoothing=1.0, curvature_deadband=0.3)
    res = ctrl.compute(0.0, 0.0, dt=0.1, curvature=0.2)  # |κ|<deadband
    assert math.isclose(res.curvature, 0.0, abs_tol=1e-9)
    assert math.isclose(res.p_ff, 0.0, abs_tol=1e-9)


def test_k_cross_schedule_off_by_default():
    """k_cross_kappa=0(기본)이면 κ와 무관하게 k_cross 상수 — 기존 동작."""
    ctrl = _ctrl(k_cross=0.4, k_heading=0.0, curvature_smoothing=1.0)
    res = ctrl.compute(0.1, 0.0, dt=0.1, curvature=1.0)
    assert math.isclose(res.k_cross_eff, 0.4, rel_tol=1e-9)
    assert math.isclose(res.p_cross, 0.4 * 0.1, rel_tol=1e-6)


def test_k_cross_schedule_boosts_on_curve_not_straight():
    """직선(κ_eff=0)은 baseline, 커브(κ_eff>0)는 k_cross 부스트."""
    ctrl = _ctrl(k_cross=0.4, k_cross_kappa=0.5, k_cross_max=1.2,
                 k_heading=0.0, curvature_smoothing=1.0, curvature_deadband=0.15)
    straight = ctrl.compute(0.1, 0.0, dt=0.1, curvature=0.0)
    assert math.isclose(straight.k_cross_eff, 0.4, rel_tol=1e-9)  # 직선 baseline
    curve = ctrl.compute(0.1, 0.0, dt=0.1, curvature=1.0)  # κ_eff=1−0.15=0.85
    assert math.isclose(curve.k_cross_eff, 0.4 + 0.5 * 0.85, rel_tol=1e-6)
    assert curve.k_cross_eff > straight.k_cross_eff


def test_k_cross_schedule_clamps_at_max():
    """급커브서도 k_cross_max로 상한 클램프(과조향 방지)."""
    ctrl = _ctrl(k_cross=0.4, k_cross_kappa=2.0, k_cross_max=1.0,
                 k_heading=0.0, curvature_smoothing=1.0, curvature_deadband=0.0)
    res = ctrl.compute(0.1, 0.0, dt=0.1, curvature=5.0)  # 부스트 크지만 max로 컷
    assert math.isclose(res.k_cross_eff, 1.0, rel_tol=1e-9)


def test_curvature_feedforward_respects_steering_sign():
    """steering_sign=-1이면 같은 κ에 피드포워드도 반대 방향으로 조향."""
    pos = _ctrl(k_cross=0.0, k_heading=0.0, k_ff=0.5, steering_sign=1.0,
                curvature_smoothing=1.0).compute(0.0, 0.0, dt=0.1, curvature=1.0)
    neg = _ctrl(k_cross=0.0, k_heading=0.0, k_ff=0.5, steering_sign=-1.0,
                curvature_smoothing=1.0).compute(0.0, 0.0, dt=0.1, curvature=1.0)
    assert math.isclose(pos.steering_norm, -neg.steering_norm, abs_tol=1e-9)


# --- 구간별 게인 프로파일 전환(07-16b) ----------------------------------

def test_set_config_swaps_gains():
    """set_config로 게인 세트를 갈아끼우면 그 스텝부터 새 게인이 적용된다."""
    from core.config_schema import LateralPDConfig
    ctrl = _ctrl(k_heading=0.33, steering_smoothing=1.0)
    before = ctrl.compute(0.0, 0.5, dt=0.1)
    post = LateralPDConfig(k_cross=1.2, k_heading=0.231, k_deriv=0.0,
                           steering_smoothing=1.0)
    ctrl.set_config(post)
    after = ctrl.compute(0.0, 0.5, dt=0.1)
    assert math.isclose(before.p_heading, 0.33 * 0.5, rel_tol=1e-6)
    assert math.isclose(after.p_heading, 0.231 * 0.5, rel_tol=1e-6)


def test_set_config_preserves_internal_state():
    """프로파일 전환이 EMA/직전조향을 리셋하면 그 순간 조향이 튄다 — 이력은 유지된다."""
    from core.config_schema import LateralPDConfig
    same = dict(k_cross=1.2, k_heading=0.8, k_deriv=0.0, steering_smoothing=0.5)
    a = _ctrl(**same)
    b = _ctrl(**same)
    for _ in range(5):                       # 동일 이력을 쌓는다.
        a.compute(0.2, 0.1, dt=0.1)
        b.compute(0.2, 0.1, dt=0.1)
    b.set_config(LateralPDConfig(**same))    # 값이 같은 세트로 교체 = 결과가 같아야 한다.
    assert math.isclose(a.compute(0.2, 0.1, dt=0.1).steering_norm,
                        b.compute(0.2, 0.1, dt=0.1).steering_norm, rel_tol=1e-9)


def test_post_sign_gains_reduce_early_steer():
    """직진 접근(offset≈0)인데 ψ만 큰 상황 = ㄱ자 선반영 → post_sign 세트가 조기조향을 줄인다.

    ψ는 경로 끝점까지의 현 각도라 코너 1m 전부터 커진다. 그때 offset은 아직 0이라
    조향은 사실상 heading 항 단독 = k_heading이 그대로 조기조향 크기가 된다.
    """
    base = _ctrl(k_heading=0.33, steering_smoothing=1.0).compute(0.0, 0.7, dt=0.1)
    post = _ctrl(k_heading=0.231, steering_smoothing=1.0).compute(0.0, 0.7, dt=0.1)
    assert 0.0 < post.steering_norm < base.steering_norm
