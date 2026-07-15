"""@file test_mission.py
@brief 상위 미션 시퀀스(MissionSequencer) 단위 테스트 — 새 6-state FSM(07-14).

미션 페이즈 전이(WAIT_START → LANE_FOLLOW → SIGN_BRANCH → LANE_FOLLOW → OBSTACLE_ZONE
→ FINISH_WATCH → FINISH_STOP)와 각 상태의 정지 게이트·인지 지시(roi_mode/yolo_enable/
sign_enable)·조향 bias를 검증한다. 지름길(노랑)·로터리·체커보드 도착은 폐기. 설계:
docs/mission_fsm.md
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
    SignDirection,
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


def _to_obstacle(seq, dwell=2.5):
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)   # 아루코=정지구역 진입 트리거.
    assert seq.phase == MissionPhase.OBSTACLE_ZONE
    # 오검출 방어(obstacle_min_dwell_sec) 통과: 마커를 든 채 최소 체류시간을 채운다.
    # 이걸 안 채우면 마커를 치웠을 때 FINISH_WATCH가 아니라 LANE_FOLLOW로 되돌아간다.
    seq.update(_obs(aruco_present=True), dt=dwell)


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
    assert cmd.steer_bias == 0.0
    assert cmd.go is True


def test_no_start_without_green():
    seq = MissionSequencer(_cfg())
    _tick(seq, n=5, traffic_light=TrafficLight.RED)
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL


def test_stopline_does_not_branch_in_lane_follow():
    """정지선 STOP은 아래층 반응형 몫 — LANE_FOLLOW 페이즈는 정지선으로 안 갈린다."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    seq.update(_obs(lane=_lane(stop_line=True, stop_line_dist=0.3)), dt=0.05)
    assert seq.phase == MissionPhase.LANE_FOLLOW


# --- 방향 팻말 분기(SIGN_BRANCH) -----------------------------------------

def test_sign_color_enters_branch_and_slows():
    """OpenCV 팻말색(sign_detected) → SIGN_BRANCH 진입 + 감속 + 팻말 YOLO ON."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0, sign_branch_speed_scale=0.5))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(sign_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.go is True
    assert cmd.speed_scale <= 0.5
    assert cmd.sign_enable is True          # 팻말 YOLO 게이트 ON.
    assert cmd.yolo_enable is False         # 신호등 YOLO는 OFF.


def test_sign_direction_right_biases_steering_negative():
    """팻말 우측 지시 → 조향 bias 음수(우), 좌측 지시 → 양수(좌)."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0, steer_bias_value=0.2))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(sign_detected=True, sign_direction=SignDirection.RIGHT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.steer_bias == -0.2

    seq2 = MissionSequencer(_cfg(sign_branch_duration=1.0, steer_bias_value=0.2))
    _to_lane_follow(seq2)
    cmd2 = seq2.update(_obs(sign_detected=True, sign_direction=SignDirection.LEFT), dt=0.05)
    assert cmd2.steer_bias == +0.2


def test_sign_direction_latches_through_momentary_none():
    """분기 중 방향이 잠깐 NONE으로 튀어도 첫 검출 방향을 유지(bias 안 풀림)."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0, steer_bias_value=0.2))
    _to_lane_follow(seq)
    seq.update(_obs(sign_detected=True, sign_direction=SignDirection.RIGHT), dt=0.05)
    cmd = seq.update(_obs(sign_detected=True, sign_direction=SignDirection.NONE), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.steer_bias == -0.2           # 래치 유지.


def test_sign_branch_returns_after_duration_and_latches_done():
    """고정시간 경과 → LANE_FOLLOW 복귀, bias 0, 이후 팻말색 재검출해도 재진입 안 함."""
    seq = MissionSequencer(_cfg(sign_branch_duration=0.1))
    _to_lane_follow(seq)
    seq.update(_obs(sign_detected=True, sign_direction=SignDirection.LEFT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    # 고정시간(0.1s) 넘기면 복귀.
    cmd = seq.update(_obs(sign_detected=True), dt=0.1)
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.steer_bias == 0.0
    assert cmd.sign_enable is False
    # 팻말색 다시 봐도 1회성 래치라 재진입 안 함.
    seq.update(_obs(sign_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.LANE_FOLLOW


def test_aruco_preempts_sign_branch():
    """분기 중 아루코 보이면 정지 구역 우선."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    seq.update(_obs(sign_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    seq.update(_obs(aruco_present=True), dt=0.05)
    assert seq.phase == MissionPhase.OBSTACLE_ZONE


# --- 장애물(아루코) → 빨간불 종료 ----------------------------------------

def test_lane_follow_to_obstacle_on_aruco():
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)
    assert seq.phase == MissionPhase.OBSTACLE_ZONE


def test_obstacle_zone_stops_on_aruco_then_restarts():
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)
    # 아루코 보이는 동안 정지 유지.
    c_stop = seq.update(_obs(aruco_present=True), dt=0.05)
    assert c_stop.state == DriveState.STOP and c_stop.go is False
    assert c_stop.roi_mode == RoiMode.LOWER_ARUCO
    # 아루코 치우면 → 빨간불 감시로 재출발.
    c_go = seq.update(_obs(aruco_present=False), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_WATCH
    assert c_go.go is True


def test_short_aruco_returns_to_lane_follow_not_finish():
    """아루코 오검출(최소 체류 미달) → FINISH_WATCH가 아니라 LANE_FOLLOW 복귀.

    FINISH_WATCH는 편도(돌아올 길 없음)라, 오검출 한 프레임이 팻말 분기를 통째로
    스킵하고 첫 빨간불에 코스를 끝내던 것을 막는다.
    """
    seq = MissionSequencer(_cfg(obstacle_min_dwell_sec=2.0))
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)
    assert seq.phase == MissionPhase.OBSTACLE_ZONE
    seq.update(_obs(aruco_present=False), dt=0.05)   # 체류 0.1s ≪ 2.0s → 오검출 판정.
    assert seq.phase == MissionPhase.LANE_FOLLOW


def test_short_aruco_keeps_sign_branch_available():
    """오검출로 되돌아온 뒤에도 팻말 분기는 살아있다(_sign_done이 안 서야 함)."""
    seq = MissionSequencer(_cfg(obstacle_min_dwell_sec=2.0))
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)    # 오검출.
    seq.update(_obs(aruco_present=False), dt=0.05)   # → LANE_FOLLOW 복귀.
    seq.update(_obs(sign_detected=True), dt=0.05)    # 팻말은 여전히 분기시켜야 한다.
    assert seq.phase == MissionPhase.SIGN_BRANCH


def test_short_aruco_cancels_yolo_relatch():
    """오검출 복귀 시 신호등 YOLO 재점화도 취소 → 주행 구간 FPS 회복."""
    seq = MissionSequencer(_cfg(obstacle_min_dwell_sec=2.0))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(aruco_present=True), dt=0.05)
    assert cmd.yolo_enable is True                    # 아루코 → 래치 ON.
    seq.update(_obs(aruco_present=False), dt=0.05)    # 오검출 판정 → 복귀.
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert _tick(seq).yolo_enable is False            # 래치 취소됨.


def test_long_aruco_still_reaches_finish_watch():
    """진짜 마커(최소 체류 충족)는 종전대로 FINISH_WATCH로 간다 — 방어가 정상 정지를 막지 않음."""
    seq = MissionSequencer(_cfg(obstacle_min_dwell_sec=2.0))
    _to_lane_follow(seq)
    seq.update(_obs(aruco_present=True), dt=0.05)
    seq.update(_obs(aruco_present=True), dt=2.5)      # 심판이 들고 있는 동안.
    seq.update(_obs(aruco_present=False), dt=0.05)    # 치움 → 재출발.
    assert seq.phase == MissionPhase.FINISH_WATCH


def test_finish_watch_stops_on_red():
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)
    seq.update(_obs(aruco_present=False), dt=0.05)  # 마커 치움 → FINISH_WATCH
    assert seq.phase == MissionPhase.FINISH_WATCH
    cmd = seq.update(_obs(traffic_light=TrafficLight.RED), dt=0.05)  # 빨간불 → 정지
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.state == DriveState.STOP and cmd.go is False


def test_full_sequence_reaches_finish():
    seq = MissionSequencer(_cfg(sign_branch_duration=0.1))
    _to_lane_follow(seq)
    seq.update(_obs(sign_detected=True, sign_direction=SignDirection.LEFT), dt=0.05)  # 팻말 분기
    seq.update(_obs(sign_detected=False), dt=0.1)   # 고정시간 경과 → LANE_FOLLOW
    assert seq.phase == MissionPhase.LANE_FOLLOW
    seq.update(_obs(aruco_present=True), dt=0.05)    # 정지 구역
    seq.update(_obs(aruco_present=True), dt=2.5)     # 마커 든 채 최소 체류시간 충족
    seq.update(_obs(aruco_present=False), dt=0.05)   # 재출발 → FINISH_WATCH
    cmd = seq.update(_obs(traffic_light=TrafficLight.RED), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.go is False


def test_reset_returns_to_wait():
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)
    seq.reset()
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL


# --- 설정 ----------------------------------------------------------------

def test_config_defaults():
    cfg = MissionConfig.from_dict({"slow_speed_scale": 0.4, "steer_bias_value": 0.3})
    assert cfg.slow_speed_scale == 0.4
    assert cfg.steer_bias_value == 0.3


def test_mission_yaml_loads():
    app = load_config(os.path.join(_CONFIG_DIR, "mission.yaml"))
    assert app.mission.slow_speed_scale == 0.5
    assert app.mission.yolo_gate_enable is True


# --- YOLO 추론 게이트(페이즈별 on/off + 아루코 래치) -----------------------

def test_yolo_gate_on_while_waiting_start():
    """출발 신호등 대기 구간엔 신호등 YOLO ON."""
    seq = MissionSequencer(_cfg())
    cmd = seq.update(_obs(), dt=0.1)
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL
    assert cmd.yolo_enable is True
    assert cmd.sign_enable is False


def test_yolo_gate_off_while_driving():
    """초록 확인 후 차선주행 구간엔 신호등/팻말 YOLO 둘 다 OFF(FPS 확보)."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    cmd = _tick(seq)
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.yolo_enable is False
    assert cmd.sign_enable is False


def test_yolo_gate_turns_on_when_aruco_appears():
    """주행 중 OFF → 아루코 마커 보이는 순간 ON(정지구역 진입과 동시)."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    assert _tick(seq).yolo_enable is False           # 주행 중 OFF.
    cmd = seq.update(_obs(aruco_present=True), dt=0.05)
    assert seq.phase == MissionPhase.OBSTACLE_ZONE
    assert cmd.yolo_enable is True                    # 아루코 → ON.


def test_yolo_gate_latches_through_finish():
    """아루코로 켜진 신호등 YOLO는 빨간불 종료까지 유지(래치)."""
    seq = MissionSequencer(_cfg())
    _to_obstacle(seq)                                 # 아루코로 정지구역 + 래치.
    cmd = seq.update(_obs(aruco_present=False), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_WATCH
    assert cmd.yolo_enable is True
    cmd = seq.update(_obs(traffic_light=TrafficLight.RED), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.yolo_enable is True


def test_sign_gate_only_in_branch():
    """팻말 YOLO(sign_enable)는 SIGN_BRANCH에서만 ON."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    assert _tick(seq).sign_enable is False
    cmd = seq.update(_obs(sign_detected=True), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.sign_enable is True


def test_yolo_gate_disabled_always_on():
    """yolo_gate_enable=False면 신호등 게이트 끔 → 전 구간 항상 ON(기존 동작)."""
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
