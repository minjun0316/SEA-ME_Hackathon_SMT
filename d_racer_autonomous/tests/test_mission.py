"""@file test_mission.py
@brief 상위 미션 시퀀스(MissionSequencer, 12-state) 단위 테스트.

로터리 지름길 미션의 페이즈 전이(WAIT_START~FINISH_STOP), 정지선 카운트
(1번째=오른쪽 계속, 2번째=왼쪽 탈출), 진입무시, EXIT_CONNECTOR 흰색 안정,
그리고 인지 지시(follow_color/roi_mode/turn_hint)를 검증한다.
설계: docs/mission_fsm.md
"""
import os

import pytest

from core.config_schema import MissionConfig, load_config
from core.planning import (
    DriveState,
    LaneColor,
    LaneObservation,
    MissionObservation,
    MissionPhase,
    MissionSequencer,
    RoiMode,
    TrafficLight,
    TurnHint,
)

_CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def _cfg(**kw):
    """테스트용: 타이머를 짧게 줄여 전이 검증을 빠르게."""
    base = dict(shortcut_approach_sec=0.1, stopline_ignore_after_entry_sec=0.1,
                stopline_debounce_sec=0.2, continue_right_sec=0.1,
                exit_left_sec=0.1, white_stable_sec=0.1)
    base.update(kw)
    return MissionConfig(**base)


def _lane(**kw):
    base = dict(lane_detected=True, confidence=0.9, num_points=20,
                lateral_offset=0.0, heading_error=0.0,
                stop_line=False, stop_line_dist=-1.0, stop_request=False)
    base.update(kw)
    return LaneObservation(**base)


def _obs(lane=None, **kw):
    return MissionObservation(lane=lane if lane is not None else _lane(), **kw)


def _tick(seq, n=3, dt=0.05, **kw):
    """n주기 update. 마지막 명령 반환."""
    cmd = None
    for _ in range(n):
        cmd = seq.update(_obs(**kw), dt=dt)
    return cmd


# --- 페이즈까지 밀어넣는 헬퍼 --------------------------------------------

def _to_start(seq):
    seq.update(_obs(traffic_light=TrafficLight.GREEN), dt=0.1)
    _tick(seq)  # 흰 주행 DRIVE로.
    assert seq.phase == MissionPhase.START_STRAIGHT


def _to_shortcut(seq):
    _to_start(seq)
    seq.update(_obs(yellow_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.SHORTCUT_APPROACH


def _to_entry(seq):
    _to_shortcut(seq)
    _tick(seq, n=3, yellow_detected=True)  # shortcut_approach_sec(0.1) 경과.
    assert seq.phase == MissionPhase.ROUNDABOUT_ENTRY


def _to_follow(seq):
    _to_entry(seq)
    _tick(seq, n=3, yellow_detected=True)  # 진입무시(0.1) 경과.
    assert seq.phase == MissionPhase.ROUNDABOUT_FOLLOW


def _stopline_obs(**kw):
    return _obs(lane=_lane(stop_line=True, stop_line_dist=0.3), yellow_detected=True, **kw)


# --- WAIT_START / START --------------------------------------------------

def test_starts_waiting_and_stops():
    seq = MissionSequencer(_cfg())
    cmd = seq.update(_obs(), dt=0.1)
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL
    assert cmd.state == DriveState.STOP and cmd.go is False


def test_green_starts_driving_white():
    seq = MissionSequencer(_cfg())
    _to_start(seq)
    cmd = _tick(seq)
    assert cmd.follow_color == LaneColor.WHITE
    assert cmd.roi_mode == RoiMode.LOWER
    assert cmd.go is True


def test_yellow_triggers_shortcut():
    seq = MissionSequencer(_cfg())
    _to_start(seq)
    cmd = seq.update(_obs(yellow_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.SHORTCUT_APPROACH
    assert cmd.follow_color == LaneColor.YELLOW


# --- 로터리 진입/무시시간 -------------------------------------------------

def test_entry_then_follow_via_timers():
    seq = MissionSequencer(_cfg())
    _to_entry(seq)
    assert seq.phase == MissionPhase.ROUNDABOUT_ENTRY
    _tick(seq, n=3, yellow_detected=True)
    assert seq.phase == MissionPhase.ROUNDABOUT_FOLLOW


def test_stopline_ignored_during_entry():
    """진입무시 시간 동안 정지선을 봐도 카운트하지 않는다."""
    seq = MissionSequencer(_cfg())
    _to_entry(seq)
    seq.update(_stopline_obs(), dt=0.02)  # ENTRY 유지(0.02<0.1), 카운트 안 함.
    assert seq.phase == MissionPhase.ROUNDABOUT_ENTRY
    assert seq.stopline_count == 0


# --- 정지선 카운트: 1번째=오른쪽, 2번째=왼쪽 -----------------------------

def test_first_stopline_continues_right():
    seq = MissionSequencer(_cfg())
    _to_follow(seq)
    cmd = seq.update(_stopline_obs(), dt=0.05)
    assert seq.stopline_count == 1
    assert seq.phase == MissionPhase.ROUNDABOUT_CONTINUE_RIGHT
    assert cmd.turn_hint == TurnHint.RIGHT
    assert cmd.roi_mode == RoiMode.RIGHT
    assert cmd.follow_color == LaneColor.YELLOW


def test_continue_right_returns_to_follow():
    seq = MissionSequencer(_cfg())
    _to_follow(seq)
    seq.update(_stopline_obs(), dt=0.05)          # → CONTINUE_RIGHT
    _tick(seq, n=3, yellow_detected=True)          # continue_right_sec(0.1) 경과
    assert seq.phase == MissionPhase.ROUNDABOUT_FOLLOW


def test_second_stopline_exits_left():
    seq = MissionSequencer(_cfg())
    _to_follow(seq)
    # 1번째 정지선 → CONTINUE_RIGHT → 복귀.
    seq.update(_stopline_obs(), dt=0.05)
    seq.update(_obs(yellow_detected=True), dt=0.1)   # 선 사라짐(rising 리셋)
    _tick(seq, n=3, yellow_detected=True)            # CONTINUE_RIGHT 종료 → FOLLOW
    assert seq.phase == MissionPhase.ROUNDABOUT_FOLLOW
    # debounce(0.2) 넘기고 2번째 정지선.
    seq.update(_obs(yellow_detected=True), dt=0.2)
    cmd = seq.update(_stopline_obs(), dt=0.05)
    assert seq.stopline_count == 2
    assert seq.phase == MissionPhase.ROUNDABOUT_EXIT_LEFT
    assert cmd.turn_hint == TurnHint.LEFT
    assert cmd.roi_mode == RoiMode.LEFT
    assert cmd.go is True  # 탈출은 정지 아님.


def test_stopline_debounced():
    """같은 정지선 연속 프레임은 한 번만 카운트(→ CONTINUE_RIGHT 1회)."""
    seq = MissionSequencer(_cfg())
    _to_follow(seq)
    seq.update(_stopline_obs(), dt=0.05)  # count 1 → CONTINUE_RIGHT
    # 계속 정지선 유지해도 rising-edge 아님 → count 그대로.
    for _ in range(4):
        seq.update(_stopline_obs(), dt=0.02)
    assert seq.stopline_count == 1


# --- 탈출 커넥터 → 외곽 ---------------------------------------------------

def _to_exit_left(seq):
    _to_follow(seq)
    seq.update(_stopline_obs(), dt=0.05)             # CONTINUE_RIGHT
    seq.update(_obs(yellow_detected=True), dt=0.1)
    _tick(seq, n=3, yellow_detected=True)            # FOLLOW
    seq.update(_obs(yellow_detected=True), dt=0.2)
    seq.update(_stopline_obs(), dt=0.05)             # EXIT_LEFT
    assert seq.phase == MissionPhase.ROUNDABOUT_EXIT_LEFT


def test_exit_left_to_connector_then_outer():
    seq = MissionSequencer(_cfg())
    _to_exit_left(seq)
    _tick(seq, n=3, yellow_detected=True)            # exit_left_sec(0.1) → CONNECTOR
    assert seq.phase == MissionPhase.EXIT_CONNECTOR
    cmd = seq.update(_obs(yellow_detected=True), dt=0.05)
    assert cmd.follow_color == LaneColor.YELLOW
    assert cmd.roi_mode == RoiMode.LEFT
    assert cmd.go is True  # 점선 구간, 정지 금지.
    # 흰색 안정(white_stable_sec 0.1) 검출 → OUTER.
    _tick(seq, n=3, white_detected=True)
    assert seq.phase == MissionPhase.OUTER_LANE_FOLLOW


def test_connector_creeps_even_if_lane_lost():
    """커넥터에서 차선 소실돼도 정지하지 않고 왼쪽 저속 유지."""
    seq = MissionSequencer(_cfg())
    _to_exit_left(seq)
    _tick(seq, n=3, yellow_detected=True)   # → CONNECTOR
    cmd = seq.update(_obs(lane=LaneObservation(lane_detected=False)), dt=0.05)
    assert cmd.go is True
    assert cmd.turn_hint == TurnHint.LEFT


# --- 외곽 → 장애물 → 도착 -------------------------------------------------

def _to_outer(seq):
    seq2 = _to_exit_left(seq)
    _tick(seq, n=3, yellow_detected=True)
    _tick(seq, n=3, white_detected=True)
    assert seq.phase == MissionPhase.OUTER_LANE_FOLLOW


def test_outer_follows_white():
    seq = MissionSequencer(_cfg())
    _to_outer(seq)
    cmd = _tick(seq, white_detected=True)
    assert cmd.follow_color == LaneColor.WHITE
    assert cmd.roi_mode == RoiMode.FULL


def test_obstacle_zone_stops_on_aruco():
    seq = MissionSequencer(_cfg())
    _to_outer(seq)
    seq.update(_obs(red_zone_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.DYNAMIC_OBSTACLE_ZONE
    # 아루코 보임 → 정지.
    c_stop = seq.update(_obs(red_zone_detected=True, aruco_present=True), dt=0.05)
    assert c_stop.state == DriveState.STOP and c_stop.go is False
    assert c_stop.roi_mode == RoiMode.LOWER_ARUCO
    # 아루코 사라짐 → 재출발.
    c_go = seq.update(_obs(red_zone_detected=True, aruco_present=False), dt=0.05)
    assert c_go.go is True


def test_obstacle_to_finish_then_stop():
    seq = MissionSequencer(_cfg())
    _to_outer(seq)
    seq.update(_obs(red_zone_detected=True), dt=0.05)   # OBSTACLE
    seq.update(_obs(red_zone_detected=False), dt=0.05)  # 벗어남 → FINISH_APPROACH
    assert seq.phase == MissionPhase.FINISH_APPROACH
    cmd = seq.update(_obs(checkerboard_detected=True), dt=0.05)  # 체커보드 → 정지
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.state == DriveState.STOP and cmd.go is False


def test_full_sequence_reaches_finish():
    seq = MissionSequencer(_cfg())
    _to_outer(seq)
    seq.update(_obs(red_zone_detected=True), dt=0.05)
    seq.update(_obs(red_zone_detected=False), dt=0.05)
    cmd = seq.update(_obs(checkerboard_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.go is False


def test_reset_returns_to_wait():
    seq = MissionSequencer(_cfg())
    _to_follow(seq)
    seq.reset()
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL
    assert seq.stopline_count == 0


# --- 설정 ----------------------------------------------------------------

def test_config_rejects_bad_side():
    with pytest.raises(ValueError):
        MissionConfig.from_dict({"roundabout_exit_side": "UP"})


def test_config_side_mapping():
    cfg = MissionConfig.from_dict({"roundabout_continue_side": "RIGHT",
                                   "roundabout_exit_side": "LEFT"})
    assert cfg.continue_side() == TurnHint.RIGHT
    assert cfg.exit_side() == TurnHint.LEFT


def test_mission_yaml_loads():
    app = load_config(os.path.join(_CONFIG_DIR, "mission.yaml"))
    assert app.mission.continue_side() == TurnHint.RIGHT
    assert app.mission.exit_side() == TurnHint.LEFT
