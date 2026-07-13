"""@file test_mission.py
@brief 상위 미션 시퀀스(MissionSequencer) 단위 테스트.

미션 페이즈 전이(WAIT_START → LANE_FOLLOW → OBSTACLE → FINISH)와 각 상태의 정지
게이트·인지 지시(follow_color/roi_mode)를 검증한다. 로터리 회전은 이 FSM이 아니라
제어단 StoplineManeuver가 담당하므로 여기서 다루지 않는다. 설계: docs/mission_fsm.md
"""
import os

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
    return MissionConfig(**kw)


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

def _to_lane_follow(seq):
    seq.update(_obs(traffic_light=TrafficLight.GREEN), dt=0.1)
    _tick(seq)  # 흰 주행 DRIVE로.
    assert seq.phase == MissionPhase.LANE_FOLLOW


def _to_obstacle(seq):
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)   # 아루코=정지구역 진입 트리거.
    assert seq.phase == MissionPhase.DYNAMIC_OBSTACLE_ZONE


# --- WAIT_START / LANE_FOLLOW --------------------------------------------

def test_starts_waiting_and_stops():
    seq = MissionSequencer(_cfg())
    cmd = seq.update(_obs(), dt=0.1)
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL
    assert cmd.state == DriveState.STOP and cmd.go is False


def test_green_starts_driving_white():
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    cmd = _tick(seq)
    assert cmd.follow_color == LaneColor.WHITE
    assert cmd.roi_mode == RoiMode.LOWER
    assert cmd.turn_hint == TurnHint.NONE
    assert cmd.go is True


def test_no_start_without_green():
    seq = MissionSequencer(_cfg())
    _tick(seq, n=5, traffic_light=TrafficLight.RED)
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL


def test_stopline_does_not_branch_in_lane_follow():
    """로터리 카운트 폐기: LANE_FOLLOW에서 정지선을 봐도 페이즈가 갈리지 않는다."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    seq.update(_obs(lane=_lane(stop_line=True, stop_line_dist=0.3)), dt=0.05)
    assert seq.phase == MissionPhase.LANE_FOLLOW


# --- 지름길(노랑 래치) -----------------------------------------------------

def test_yellow_enters_shortcut_and_slows():
    """인지 on_yellow 래치 → SHORTCUT 진입 + 감속."""
    seq = MissionSequencer(_cfg(slow_speed_scale=0.5))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(on_yellow=True), dt=0.05)
    assert seq.phase == MissionPhase.SHORTCUT
    assert cmd.go is True
    assert cmd.speed_scale <= 0.5


def test_shortcut_returns_when_yellow_gone():
    """지름길 통과 후 노랑 래치 해제 → 흰 외곽 주행(LANE_FOLLOW) 복귀."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    seq.update(_obs(on_yellow=True), dt=0.05)          # → SHORTCUT
    assert seq.phase == MissionPhase.SHORTCUT
    seq.update(_obs(on_yellow=False), dt=0.05)         # 노랑 해제 → 복귀
    assert seq.phase == MissionPhase.LANE_FOLLOW


# --- 장애물(아루코) → 도착 -----------------------------------------------

def test_lane_follow_to_obstacle_on_aruco():
    """아루코 마커 보이면(red_zone 무관) 정지 구역 진입."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)
    assert seq.phase == MissionPhase.DYNAMIC_OBSTACLE_ZONE


def test_obstacle_zone_stops_on_aruco():
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)
    # 아루코 보이는 동안 정지 유지.
    c_stop = seq.update(_obs(aruco_present=True), dt=0.05)
    assert c_stop.state == DriveState.STOP and c_stop.go is False
    assert c_stop.roi_mode == RoiMode.LOWER_ARUCO
    # 아루코 치우면 → 도착 접근으로 재출발.
    c_go = seq.update(_obs(aruco_present=False), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_APPROACH
    assert c_go.go is True


def test_obstacle_to_finish_then_stop():
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)
    seq.update(_obs(aruco_present=False), dt=0.05)  # 마커 치움 → FINISH_APPROACH
    assert seq.phase == MissionPhase.FINISH_APPROACH
    cmd = seq.update(_obs(checkerboard_detected=True), dt=0.05)  # 체커보드 → 정지
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.state == DriveState.STOP and cmd.go is False


def test_full_sequence_reaches_finish():
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)   # 정지 구역
    seq.update(_obs(aruco_present=False), dt=0.05)  # 도착 접근
    cmd = seq.update(_obs(checkerboard_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.go is False


def test_reset_returns_to_wait():
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)
    seq.reset()
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL


# --- 설정 ----------------------------------------------------------------

def test_config_defaults():
    cfg = MissionConfig.from_dict({"slow_speed_scale": 0.4})
    assert cfg.slow_speed_scale == 0.4


def test_mission_yaml_loads():
    app = load_config(os.path.join(_CONFIG_DIR, "mission.yaml"))
    assert app.mission.slow_speed_scale == 0.5
    assert app.mission.yolo_gate_enable is True


# --- YOLO 추론 게이트(페이즈별 on/off + 아루코 래치) -----------------------

def test_yolo_gate_on_while_waiting_start():
    """출발 신호등 대기 구간엔 YOLO ON(신호등 봐야 함)."""
    seq = MissionSequencer(_cfg())
    cmd = seq.update(_obs(), dt=0.1)
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL
    assert cmd.yolo_enable is True


def test_yolo_gate_off_while_driving():
    """초록 확인 후 차선주행 구간엔 YOLO OFF(FPS 확보)."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    cmd = _tick(seq)
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.yolo_enable is False


def test_yolo_gate_turns_on_when_aruco_appears():
    """주행 중 OFF → 아루코 마커 보이는 순간 ON(정지구역 진입과 동시)."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    assert _tick(seq).yolo_enable is False           # 주행 중 OFF.
    cmd = seq.update(_obs(aruco_present=True), dt=0.05)
    assert seq.phase == MissionPhase.DYNAMIC_OBSTACLE_ZONE
    assert cmd.yolo_enable is True                    # 아루코 → ON.


def test_yolo_gate_latches_through_finish():
    """아루코로 켜진 YOLO는 도착까지 유지(래치, 마커 사라져도 안 꺼짐)."""
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)                                 # 아루코로 정지구역 + 래치.
    # 마커 치워 FINISH_APPROACH로 넘어가도 ON 유지.
    cmd = seq.update(_obs(aruco_present=False), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_APPROACH
    assert cmd.yolo_enable is True
    cmd = seq.update(_obs(checkerboard_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.yolo_enable is True


def test_yolo_gate_disabled_always_on():
    """yolo_gate_enable=False면 게이트 끔 → 전 구간 항상 ON(기존 동작)."""
    seq = MissionSequencer(_cfg(yolo_gate_enable=False))
    _to_lane_follow(seq)
    cmd = _tick(seq)
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.yolo_enable is True


def test_yolo_gate_resets_with_sequencer():
    """reset() 후 래치 해제 → 다시 대기ON/주행OFF 패턴."""
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)                                 # 아루코 래치 ON.
    seq.reset()
    cmd = seq.update(_obs(), dt=0.1)  # 다시 WAIT_START_SIGNAL.
    assert cmd.yolo_enable is True
    _to_lane_follow(seq)
    assert _tick(seq).yolo_enable is False  # 래치 풀렸으니 주행 OFF.
