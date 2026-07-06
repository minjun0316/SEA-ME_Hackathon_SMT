"""@file test_decision.py
@brief 판단(Decision) 상태기계 단위 테스트."""
from core.config_schema import DecisionConfig
from core.planning import DecisionMaker, DriveState, LaneObservation


def _good(**kw):
    """정상 주행에 충분한 관측(필요 필드만 override)."""
    base = dict(lane_detected=True, confidence=0.9, num_points=20,
                lateral_offset=0.0, heading_error=0.0,
                stop_line=False, stop_line_dist=-1.0, stop_request=False)
    base.update(kw)
    return LaneObservation(**base)


def test_starts_in_init_and_stops():
    dm = DecisionMaker()
    cmd = dm.update(LaneObservation(), dt=0.1)  # 검출 없음
    assert cmd.state == DriveState.INIT
    assert cmd.go is False
    assert cmd.speed_scale == 0.0


def test_init_to_drive_after_recover_grace():
    cfg = DecisionConfig(recover_grace=0.1)
    dm = DecisionMaker(cfg)
    # 첫 유효검출이지만 아직 recover_grace 미달 → 여전히 INIT.
    c1 = dm.update(_good(), dt=0.05)
    assert c1.state == DriveState.INIT
    # 유효검출 누적 0.10s >= recover_grace → DRIVE.
    c2 = dm.update(_good(), dt=0.05)
    assert c2.state == DriveState.DRIVE
    assert c2.go is True
    assert c2.speed_scale == cfg.drive_speed_scale


def _to_drive(dm):
    """상태기계를 DRIVE로 밀어넣는 헬퍼."""
    for _ in range(5):
        dm.update(_good(), dt=0.05)
    assert dm.state == DriveState.DRIVE


def test_low_confidence_goes_slow():
    dm = DecisionMaker()
    _to_drive(dm)
    cmd = dm.update(_good(confidence=0.5), dt=0.05)  # conf_drive(0.6) 미만
    assert cmd.state == DriveState.SLOW
    assert cmd.go is True
    assert cmd.speed_scale == dm.cfg.slow_speed_scale
    assert cmd.lookahead_scale == dm.cfg.slow_lookahead_scale


def test_large_heading_error_goes_slow():
    dm = DecisionMaker()
    _to_drive(dm)
    cmd = dm.update(_good(heading_error=0.5), dt=0.05)  # heading_slow(0.35) 초과
    assert cmd.state == DriveState.SLOW


def test_large_offset_goes_slow():
    dm = DecisionMaker()
    _to_drive(dm)
    cmd = dm.update(_good(lateral_offset=0.2), dt=0.05)  # offset_slow(0.12) 초과
    assert cmd.state == DriveState.SLOW


def test_lost_after_grace_only():
    cfg = DecisionConfig(lost_grace=0.3)
    dm = DecisionMaker(cfg)
    _to_drive(dm)
    # 짧은 끊김(grace 이내)은 직전 상태(DRIVE) 유지.
    c1 = dm.update(LaneObservation(lane_detected=False), dt=0.2)
    assert c1.state == DriveState.DRIVE
    # 누적 0.4s >= lost_grace → LOST, 정지.
    c2 = dm.update(LaneObservation(lane_detected=False), dt=0.2)
    assert c2.state == DriveState.LOST
    assert c2.go is False


def test_recover_from_lost():
    cfg = DecisionConfig(lost_grace=0.1, recover_grace=0.1)
    dm = DecisionMaker(cfg)
    _to_drive(dm)
    dm.update(LaneObservation(lane_detected=False), dt=0.2)  # LOST
    assert dm.state == DriveState.LOST
    dm.update(_good(), dt=0.05)   # 복귀 유예 미달 → 아직 LOST
    assert dm.state == DriveState.LOST
    c = dm.update(_good(), dt=0.05)  # 유예 충족 → 주행 복귀
    assert c.state == DriveState.DRIVE


def test_stop_line_within_trigger_stops():
    dm = DecisionMaker()
    _to_drive(dm)
    cmd = dm.update(_good(stop_line=True, stop_line_dist=0.2), dt=0.05)
    assert cmd.state == DriveState.STOP
    assert cmd.go is False


def test_stop_line_approach_slows():
    dm = DecisionMaker()
    _to_drive(dm)
    cmd = dm.update(_good(stop_line=True, stop_line_dist=0.5), dt=0.05)
    assert cmd.state == DriveState.SLOW


def test_stop_dwell_then_resume_even_if_line_visible():
    cfg = DecisionConfig(stop_dwell=0.2)
    dm = DecisionMaker(cfg)
    _to_drive(dm)
    stop_obs = dict(stop_line=True, stop_line_dist=0.2)
    dm.update(_good(**stop_obs), dt=0.1)   # STOP, dwell 0.1
    assert dm.state == DriveState.STOP
    dm.update(_good(**stop_obs), dt=0.15)  # dwell 0.25 >= 0.2 → 래치
    # 다음 주기: 정지선 계속 보여도 재출발(데드락 방지).
    c = dm.update(_good(**stop_obs), dt=0.05)
    assert c.state != DriveState.STOP
    assert c.go is True


def test_stop_request_overrides_everything():
    dm = DecisionMaker()
    _to_drive(dm)
    cmd = dm.update(_good(stop_request=True), dt=0.05)
    assert cmd.state == DriveState.STOP
    assert cmd.go is False


def test_state_constants_match_contract():
    # 계약 §4.4의 uint8 상수값과 일치해야 한다.
    assert int(DriveState.INIT) == 0
    assert int(DriveState.DRIVE) == 1
    assert int(DriveState.SLOW) == 2
    assert int(DriveState.STOP) == 3
    assert int(DriveState.LOST) == 4
