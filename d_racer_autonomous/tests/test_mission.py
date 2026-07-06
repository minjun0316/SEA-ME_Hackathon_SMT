"""@file test_mission.py
@brief 상위 미션 시퀀스(MissionSequencer) 단위 테스트.

각 페이즈 전이(M0~M6)와 미션 지시(follow_color/turn_hint), 그리고 아래층
반응형 SM과의 합성(정지 페이즈 STOP 강제, M2 정지선 마스킹)을 검증한다.
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
    TrafficLight,
    TurnHint,
)

_CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def _lane(**kw):
    """정상 주행에 충분한 차선 관측(필요 필드만 override)."""
    base = dict(lane_detected=True, confidence=0.9, num_points=20,
                lateral_offset=0.0, heading_error=0.0,
                stop_line=False, stop_line_dist=-1.0, stop_request=False)
    base.update(kw)
    return LaneObservation(**base)


def _obs(lane=None, **kw):
    """미션 관측 묶음. lane 미지정 시 정상 주행 차선."""
    return MissionObservation(lane=lane if lane is not None else _lane(), **kw)


# --- 특정 페이즈까지 밀어넣는 헬퍼들 --------------------------------------

def _drive(seq, color=LaneColor.WHITE, n=5, **kw):
    """유효 차선을 n주기 먹여 아래층을 DRIVE로 만든다. 마지막 명령 반환."""
    cmd = None
    for _ in range(n):
        cmd = seq.update(_obs(lane_color=color, **kw), dt=0.05)
    return cmd


def _at_m1(seq):
    seq.update(_obs(traffic_light=TrafficLight.GREEN), dt=0.1)
    _drive(seq)
    assert seq.phase == MissionPhase.M1_LANE_WHITE_1


def _at_m2(seq):
    _at_m1(seq)
    _drive(seq, color=LaneColor.YELLOW)   # 흰→노랑 전환 → M2, 노랑 추종 주행
    assert seq.phase == MissionPhase.M2_CIRCLE


def _at_m3(seq):
    _at_m2(seq)
    _drive(seq, color=LaneColor.WHITE)    # 노랑→흰 전환 → M3
    assert seq.phase == MissionPhase.M3_LANE_WHITE_2


def _at_m4(seq):
    _at_m3(seq)
    seq.update(_obs(red_zone_detected=True), dt=0.05)   # 빨강 구역 진입 → M4
    assert seq.phase == MissionPhase.M4_OBSTACLE


def _at_m5(seq):
    _at_m4(seq)
    seq.update(_obs(red_zone_detected=False), dt=0.05)  # 구역 벗어남 → M5
    _drive(seq)
    assert seq.phase == MissionPhase.M5_LANE_WHITE_3


# --- M0 ------------------------------------------------------------------

def test_starts_in_m0_and_stops():
    seq = MissionSequencer()
    cmd = seq.update(_obs(), dt=0.1)   # 차선 보여도 신호 대기 → STOP
    assert seq.phase == MissionPhase.M0_WAIT_GREEN
    assert cmd.state == DriveState.STOP
    assert cmd.go is False


def test_m0_holds_until_green():
    seq = MissionSequencer()
    seq.update(_obs(traffic_light=TrafficLight.RED), dt=0.1)
    assert seq.phase == MissionPhase.M0_WAIT_GREEN
    seq.update(_obs(traffic_light=TrafficLight.GREEN), dt=0.1)
    assert seq.phase == MissionPhase.M1_LANE_WHITE_1


# --- M1 ------------------------------------------------------------------

def test_m1_follows_white_and_drives():
    seq = MissionSequencer()
    _at_m1(seq)
    cmd = _drive(seq)
    assert cmd.follow_color == LaneColor.WHITE
    assert cmd.turn_hint == TurnHint.NONE
    assert cmd.go is True


def test_m1_to_m2_on_yellow():
    seq = MissionSequencer()
    _at_m1(seq)
    cmd = seq.update(_obs(lane_color=LaneColor.YELLOW), dt=0.05)
    assert seq.phase == MissionPhase.M2_CIRCLE
    assert cmd.follow_color == LaneColor.YELLOW


# --- M2 원 지름길 ---------------------------------------------------------

def test_m2_stop_line_count_sets_turn_hint():
    seq = MissionSequencer()   # 기본: loop=LEFT, exit=RIGHT
    _at_m2(seq)
    # 아직 정지선 전 → 힌트 없음.
    c0 = seq.update(_obs(lane_color=LaneColor.YELLOW), dt=0.05)
    assert c0.turn_hint == TurnHint.NONE
    # 첫 정지선(rising edge) → count 1 → loop 쪽(LEFT).
    c1 = seq.update(_obs(lane_color=LaneColor.YELLOW,
                         lane=_lane(stop_line=True, stop_line_dist=0.3)), dt=0.05)
    assert seq.stopline_count == 1
    assert c1.turn_hint == TurnHint.LEFT
    # 정지선 사라지고 디바운스 시간 경과.
    seq.update(_obs(lane_color=LaneColor.YELLOW), dt=1.0)
    # 두 번째 정지선 → count 2 → exit 쪽(RIGHT).
    c2 = seq.update(_obs(lane_color=LaneColor.YELLOW,
                         lane=_lane(stop_line=True, stop_line_dist=0.3)), dt=0.05)
    assert seq.stopline_count == 2
    assert c2.turn_hint == TurnHint.RIGHT


def test_m2_stop_line_debounced():
    """같은 정지선이 여러 프레임 잡혀도 한 번만 카운트."""
    seq = MissionSequencer()
    _at_m2(seq)
    stop = dict(lane_color=LaneColor.YELLOW,
                lane=_lane(stop_line=True, stop_line_dist=0.3))
    for _ in range(5):   # 같은 정지선 연속 유지
        seq.update(_obs(**stop), dt=0.05)
    assert seq.stopline_count == 1


def test_m2_fork_stop_line_does_not_stop_car():
    """M2 정지선은 갈림길 표식일 뿐 → 차는 멈추지 않고 통과."""
    seq = MissionSequencer()
    _at_m2(seq)
    cmd = seq.update(_obs(lane_color=LaneColor.YELLOW,
                          lane=_lane(stop_line=True, stop_line_dist=0.05)), dt=0.05)
    assert cmd.state != DriveState.STOP
    assert cmd.go is True


def test_m2_to_m3_on_white():
    seq = MissionSequencer()
    _at_m2(seq)
    seq.update(_obs(lane_color=LaneColor.WHITE), dt=0.05)
    assert seq.phase == MissionPhase.M3_LANE_WHITE_2


# --- M3 → M4 빨강 구역 ----------------------------------------------------

def test_m3_to_m4_on_red_zone():
    seq = MissionSequencer()
    _at_m3(seq)
    seq.update(_obs(red_zone_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.M4_OBSTACLE


def test_m4_stops_on_aruco_else_drives():
    seq = MissionSequencer()
    _at_m4(seq)
    # 아루코 보임 → STOP.
    c_stop = seq.update(_obs(red_zone_detected=True, aruco_present=True), dt=0.05)
    assert c_stop.state == DriveState.STOP
    assert c_stop.go is False
    assert seq.phase == MissionPhase.M4_OBSTACLE   # 아직 구역 안
    # 아루코 사라짐 → 그대로 흰 차선 주행.
    c_go = seq.update(_obs(red_zone_detected=True, aruco_present=False), dt=0.05)
    assert c_go.follow_color == LaneColor.WHITE
    assert c_go.go is True


def test_m4_to_m5_on_red_gone():
    seq = MissionSequencer()
    _at_m4(seq)
    seq.update(_obs(red_zone_detected=False), dt=0.05)
    assert seq.phase == MissionPhase.M5_LANE_WHITE_3


# --- M5 → M6 정지 구역 ----------------------------------------------------

def test_m5_to_m6_on_stop_zone_and_stops():
    seq = MissionSequencer()
    _at_m5(seq)
    # 멀리 보이면 아직 M5.
    seq.update(_obs(stop_zone_detected=True, stop_zone_dist=1.0), dt=0.05)
    assert seq.phase == MissionPhase.M5_LANE_WHITE_3
    # 가까우면 M6 진입 + 정지.
    cmd = seq.update(_obs(stop_zone_detected=True, stop_zone_dist=0.1), dt=0.05)
    assert seq.phase == MissionPhase.M6_FINISH
    assert cmd.state == DriveState.STOP
    assert cmd.go is False


def test_full_sequence_reaches_finish():
    seq = MissionSequencer()
    _at_m5(seq)
    cmd = seq.update(_obs(stop_zone_detected=True, stop_zone_dist=0.1), dt=0.05)
    assert seq.phase == MissionPhase.M6_FINISH
    assert cmd.go is False


def test_reset_returns_to_m0():
    seq = MissionSequencer()
    _at_m4(seq)
    seq.reset()
    assert seq.phase == MissionPhase.M0_WAIT_GREEN
    assert seq.stopline_count == 0


# --- 설정 검증 ------------------------------------------------------------

def test_config_rejects_same_side():
    with pytest.raises(ValueError):
        MissionConfig.from_dict({"circle_loop_side": "LEFT",
                                 "circle_exit_side": "LEFT"})


def test_config_side_mapping():
    cfg = MissionConfig.from_dict({"circle_loop_side": "RIGHT",
                                   "circle_exit_side": "LEFT"})
    assert cfg.loop_side() == TurnHint.RIGHT
    assert cfg.exit_side() == TurnHint.LEFT


def test_mission_yaml_loads():
    app = load_config(os.path.join(_CONFIG_DIR, "mission.yaml"))
    assert app.mission.loop_side() != app.mission.exit_side()
