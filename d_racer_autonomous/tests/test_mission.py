"""@file test_mission.py
@brief 상위 미션 시퀀스(MissionSequencer) 단위 테스트 — 새 6-state FSM(07-14).

미션 페이즈 전이(WAIT_START → LANE_FOLLOW → SIGN_BRANCH → LANE_FOLLOW → OBSTACLE_ZONE
→ FINISH_WATCH → FINISH_STOP)와 각 상태의 정지 게이트·인지 지시(roi_mode/yolo_enable/
sign_enable/turn_hint)를 검증한다. 지름길(노랑)·로터리·체커보드 도착은 폐기.
07-15 밤 팻말 재설계: 진입 트리거=팻말 YOLO 방향(색 트리거 폐기), 기동=turn_hint 앵커
차선 지시(고정 steer_bias 폐기 → 상시 0). 설계: docs/mission_fsm.md
"""
import os

from core.config_schema import MissionConfig, load_config
from core.planning import (
    DriveState,
    LaneColor,
    LaneObservation,
    MissionObservation,
    MissionPhase,
    GainProfile,
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

def test_sign_direction_enters_branch_and_slows():
    """팻말 YOLO 방향(sign_direction≠NONE) → SIGN_BRANCH 진입 + 감속 + 팻말 YOLO ON."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0, sign_branch_speed_scale=0.5))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.go is True
    assert cmd.speed_scale <= 0.5
    assert cmd.sign_enable is True          # 팻말 YOLO 게이트 ON.
    assert cmd.yolo_enable is False         # 신호등 YOLO는 OFF.


def test_sign_color_alone_does_not_enter_branch():
    """07-15 밤: OpenCV 팻말색은 더 이상 트리거가 아니다(YOLO 방향만 진입시킨다)."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    seq.update(_obs(sign_detected=True, sign_direction=SignDirection.NONE), dt=0.05)
    assert seq.phase == MissionPhase.LANE_FOLLOW


def test_sign_yolo_on_while_hunting_sign():
    """진입 트리거가 팻말 YOLO라 LANE_FOLLOW에서도 켜져 있어야 한다(논리 순환 방지)."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(), dt=0.05)       # 팻말 아직 안 보임.
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.sign_enable is True


def test_sign_direction_sets_turn_hint_not_steer_bias():
    """팻말 방향 → 인지 앵커 지시(turn_hint). 고정 조향 bias는 폐기(항상 0)."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(sign_direction=SignDirection.RIGHT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.turn_hint == TurnHint.RIGHT
    assert cmd.steer_bias == 0.0

    seq2 = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq2)
    cmd2 = seq2.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)
    assert cmd2.turn_hint == TurnHint.LEFT
    assert cmd2.steer_bias == 0.0


def test_sign_direction_latches_through_momentary_none():
    """분기 중 방향이 잠깐 NONE으로 튀어도 첫 검출 방향을 유지(앵커 지시 안 풀림)."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    seq.update(_obs(sign_direction=SignDirection.RIGHT), dt=0.05)
    cmd = seq.update(_obs(sign_direction=SignDirection.NONE), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.turn_hint == TurnHint.RIGHT  # 래치 유지.


def test_sign_branch_returns_after_duration_and_latches_done():
    """고정시간 경과 → LANE_FOLLOW 복귀, 앵커 지시 해제, 이후 팻말 재검출해도 재진입 안 함."""
    seq = MissionSequencer(_cfg(sign_branch_duration=0.1))
    _to_lane_follow(seq)
    seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    # 고정시간(0.1s) 넘기면 복귀.
    cmd = seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.1)
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.turn_hint == TurnHint.NONE   # 인지는 adaptive 기하학으로 복귀.
    assert cmd.steer_bias == 0.0
    assert cmd.sign_enable is False         # 팻말 끝 → CPU 회수.
    # 팻말 다시 봐도 1회성 래치라 재진입 안 함.
    seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)
    assert seq.phase == MissionPhase.LANE_FOLLOW


# --- 분기 종료 속도 램프 + 곡선 게이트(07-16b) ---------------------------

_RAMP_CFG = dict(sign_branch_duration=0.1, sign_lost_release_sec=0.0,
                 sign_branch_speed_scale=0.3, sign_exit_ramp_sec=1.0,
                 sign_exit_straight_rad=0.2, sign_exit_hold_max_sec=4.0)


def _to_exit_ramp(seq):
    """@brief 팻말 분기를 통과시켜 '종료 램프 활성' 상태(LANE_FOLLOW)로 만든다."""
    _to_lane_follow(seq)
    seq.update(_obs(sign_direction=SignDirection.RIGHT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    seq.update(_obs(sign_direction=SignDirection.RIGHT), dt=0.1)  # duration 경과 → 복귀.
    assert seq.phase == MissionPhase.LANE_FOLLOW


def test_exit_ramp_holds_branch_speed_through_merge_curve():
    """합류 곡선(heading 큼)이 이어지는 동안 램프가 얼어 분기 속도를 유지한다.

    램프가 시계로만 돌면 곡선 한복판에서 만료돼 거기서 속도가 붙었다(07-16 실차).
    램프 시간(1.0s)의 3배를 곡선으로 흘려도 속도가 안 올라야 한다.
    """
    seq = MissionSequencer(_cfg(**_RAMP_CFG))
    _to_exit_ramp(seq)
    cmd = _tick(seq, n=60, dt=0.05, lane=_lane(heading_error=0.5))  # 3.0s 곡선.
    assert cmd.speed_scale == 0.3


def test_exit_ramp_resumes_when_lane_straightens():
    """차선이 펴지면 그때부터 램프가 흘러 속도가 평시로 복귀한다."""
    seq = MissionSequencer(_cfg(**_RAMP_CFG))
    _to_exit_ramp(seq)
    _tick(seq, n=20, dt=0.05, lane=_lane(heading_error=0.5))       # 1.0s 곡선 = 유지.
    mid = _tick(seq, n=10, dt=0.05, lane=_lane(heading_error=0.0))  # 0.5s 직선 = 램프 중.
    assert 0.3 < mid.speed_scale < 1.0
    end = _tick(seq, n=12, dt=0.05, lane=_lane(heading_error=0.0))  # 램프 완료.
    assert end.speed_scale == 1.0                                   # drive_speed_scale 기본값.


def test_exit_ramp_fallback_releases_when_curve_never_ends():
    """heading이 계속 문턱 위여도 hold 상한을 넘으면 램프가 진행된다(영구 저속 방지)."""
    seq = MissionSequencer(_cfg(**{**_RAMP_CFG, 'sign_exit_hold_max_sec': 1.0}))
    _to_exit_ramp(seq)
    held = _tick(seq, n=18, dt=0.05, lane=_lane(heading_error=0.5))  # 0.9s < 상한.
    assert held.speed_scale == 0.3
    # 상한(1.0s) 초과 → 곡선이어도 램프가 흐른다. 곡선이라 SLOW 상한(0.5)까지.
    freed = _tick(seq, n=30, dt=0.05, lane=_lane(heading_error=0.5))
    assert freed.speed_scale == 0.5


def test_exit_ramp_holds_while_lane_lost():
    """차선 미검출이면 램프를 붙든다 — heading_error=0(watchdog)을 직선으로 오독 금지."""
    seq = MissionSequencer(_cfg(**_RAMP_CFG))
    _to_exit_ramp(seq)
    # 미검출은 heading_error=0.0으로 들어온다(mission_node watchdog). 게이트만 보면 '직선'.
    lost = _lane(lane_detected=False, confidence=0.0, num_points=0, heading_error=0.0)
    _tick(seq, n=40, dt=0.05, lane=lost)                          # 2.0s > 램프 1.0s.
    cmd = _tick(seq, n=4, dt=0.05, lane=_lane(heading_error=0.5))  # 곡선서 차선 복구.
    assert cmd.speed_scale == 0.3   # 램프가 안 흘렀다 → 분기 속도 유지.


def test_exit_ramp_gate_off_is_pure_time_ramp():
    """sign_exit_straight_rad=0 = 게이트 끔 → 곡선이어도 시간만으로 복귀(옛 동작)."""
    seq = MissionSequencer(_cfg(**{**_RAMP_CFG, 'sign_exit_straight_rad': 0.0}))
    _to_exit_ramp(seq)
    cmd = _tick(seq, n=30, dt=0.05, lane=_lane(heading_error=0.5))   # 1.5s > 램프 1.0s.
    assert cmd.speed_scale == 0.5   # 램프 완료 → SLOW 상한(곡선이므로).


# --- 구간별 게인 프로파일(gain_profile, 07-16b) -------------------------

def test_gain_profile_default_before_sign():
    """팻말 前(S자)은 DEFAULT — 곡률이 연속이라 heading 선반영이 이득인 구간."""
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    assert _tick(seq).gain_profile == GainProfile.DEFAULT


def test_gain_profile_default_during_branch():
    """분기 중(SIGN_BRANCH)엔 전환 안 함 — 지금 튜닝된 분기 동작 보존."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    cmd = seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.gain_profile == GainProfile.DEFAULT


def test_gain_profile_post_sign_after_branch():
    """분기를 마치면(_sign_done) 직선-ㄱ자 구간이라 POST_SIGN 세트를 지시한다."""
    seq = MissionSequencer(_cfg(**_RAMP_CFG))
    _to_exit_ramp(seq)
    assert _tick(seq).gain_profile == GainProfile.POST_SIGN
    # 장애물 구역을 지나 재출발한 뒤에도 유지(래치라 코스 끝까지 ㄱ자 구간).
    seq.update(_obs(aruco_present=True), dt=0.05)
    seq.update(_obs(aruco_present=True), dt=2.5)
    cmd = seq.update(_obs(aruco_present=False), dt=0.05)
    assert seq.phase == MissionPhase.FINISH_WATCH
    assert cmd.gain_profile == GainProfile.POST_SIGN


def test_gain_profile_disabled_stays_default():
    """post_sign_profile_enable=false면 전 구간 DEFAULT(옛 동작)."""
    seq = MissionSequencer(_cfg(**{**_RAMP_CFG, 'post_sign_profile_enable': False}))
    _to_exit_ramp(seq)
    assert _tick(seq).gain_profile == GainProfile.DEFAULT


def test_aruco_preempts_sign_branch():
    """분기 중 아루코 보이면 정지 구역 우선."""
    seq = MissionSequencer(_cfg(sign_branch_duration=1.0))
    _to_lane_follow(seq)
    seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)
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
    seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)  # 팻말은 여전히 분기시켜야 한다.
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


# --- 빨간불 정지 해제(FINISH_STOP → FINISH_WATCH) — 07-15c -----------------
# 편도 종착 상태라 빨강 오검출 하나가 코스를 복구 불가로 끝내던 것을 막는다.
# 심판은 멈춘 뒤에도 빨강을 계속 들고 있으므로 '빨강이 사라짐' = 오검출이었거나
# 화면 밖으로 나갔거나 뿐 → 어느 쪽이든 복귀가 안전하다. 설계: docs/mission_fsm.md.

def _to_finish_stop(seq):
    """FINISH_STOP까지 밀어넣는다(빨간불 1회 관측)."""
    _to_obstacle(seq)
    seq.update(_obs(aruco_present=False), dt=0.05)                  # → FINISH_WATCH
    seq.update(_obs(traffic_light=TrafficLight.RED), dt=0.05)       # → FINISH_STOP
    assert seq.phase == MissionPhase.FINISH_STOP


def test_finish_stop_releases_after_red_disappears():
    """오검출로 멈춘 경우: 빨강이 사라지면 해제시간 뒤 FINISH_WATCH 복귀 → 미션 계속."""
    seq = MissionSequencer(_cfg(finish_stop_release_sec=10.0))
    _to_finish_stop(seq)
    seq.update(_obs(traffic_light=TrafficLight.NONE), dt=9.9)       # 아직 10s 미만
    assert seq.phase == MissionPhase.FINISH_STOP
    cmd = seq.update(_obs(traffic_light=TrafficLight.NONE), dt=0.2)  # 누적 10.1s
    assert seq.phase == MissionPhase.FINISH_WATCH
    assert cmd.go is True                                            # 주행 재개.


def test_finish_stop_holds_while_red_visible():
    """진짜 빨강(심판이 계속 들고 있음): 타이머가 계속 리셋 → 영구 정지(정상 종료)."""
    seq = MissionSequencer(_cfg(finish_stop_release_sec=10.0))
    _to_finish_stop(seq)
    for _ in range(200):                                            # 20초간 빨강 유지.
        cmd = seq.update(_obs(traffic_light=TrafficLight.RED), dt=0.1)
    assert seq.phase == MissionPhase.FINISH_STOP
    assert cmd.go is False


def test_finish_stop_release_is_minimum_stop_time():
    """해제시간 = 최소 정지시간. 진입 직후 빨강이 사라져도 그 시간은 서 있어야 한다.

    (오버슛으로 정지 후 빨강이 화면 밖으로 나간 경우 = '정지했음' 증명 구간.)
    """
    seq = MissionSequencer(_cfg(finish_stop_release_sec=10.0))
    _to_finish_stop(seq)
    elapsed = 0.0
    while seq.phase == MissionPhase.FINISH_STOP and elapsed < 30.0:
        seq.update(_obs(traffic_light=TrafficLight.NONE), dt=0.1)   # 진입 직후부터 무관측.
        elapsed += 0.1
    assert seq.phase == MissionPhase.FINISH_WATCH
    assert elapsed >= 10.0                                          # 10초는 반드시 정지.


def test_finish_stop_can_release_repeatedly():
    """래치가 아니다: 오검출이 여러 번 나도 그때마다 복귀해 미션이 이어진다."""
    seq = MissionSequencer(_cfg(finish_stop_release_sec=10.0))
    _to_finish_stop(seq)
    for _ in range(3):
        seq.update(_obs(traffic_light=TrafficLight.NONE), dt=10.1)  # 해제 → FINISH_WATCH
        assert seq.phase == MissionPhase.FINISH_WATCH
        seq.update(_obs(traffic_light=TrafficLight.RED), dt=0.05)   # 또 오검출 → 정지
        assert seq.phase == MissionPhase.FINISH_STOP


def test_finish_stop_release_disabled_keeps_terminal():
    """0=끔이면 옛 동작(영구 정지) 그대로 — 되돌리기용 스위치."""
    seq = MissionSequencer(_cfg(finish_stop_release_sec=0.0))
    _to_finish_stop(seq)
    seq.update(_obs(traffic_light=TrafficLight.NONE), dt=60.0)
    assert seq.phase == MissionPhase.FINISH_STOP


def test_full_sequence_reaches_finish():
    seq = MissionSequencer(_cfg(sign_branch_duration=0.1))
    _to_lane_follow(seq)
    seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)  # 팻말 분기
    seq.update(_obs(), dt=0.1)                      # 고정시간 경과 → LANE_FOLLOW
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
    cfg = MissionConfig.from_dict({"slow_speed_scale": 0.4, "sign_branch_duration": 4.0})
    assert cfg.slow_speed_scale == 0.4
    assert cfg.sign_branch_duration == 4.0


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
    """초록 확인 후 차선주행 구간엔 신호등 YOLO OFF(FPS 확보).

    07-15 밤: 팻말 YOLO(sign_enable)는 여기서 OFF가 아니다 — 분기 진입 트리거가
    팻말 YOLO 자체로 바뀌어, 팻말을 아직 안 지났다면 주행 중에도 켜 둬야 한다
    (SIGN_BRANCH 전용이면 영영 진입 못 하는 논리 순환). 팻말을 지나면 그때 OFF.
    """
    seq = MissionSequencer(_cfg())
    _to_lane_follow(seq)
    cmd = _tick(seq)
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.yolo_enable is False
    assert cmd.sign_enable is True


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
    """팻말 YOLO(sign_enable)는 팻말을 찾는 LANE_FOLLOW + SIGN_BRANCH에서 ON, 분기 후 OFF."""
    seq = MissionSequencer(_cfg(sign_branch_duration=0.1))
    _to_lane_follow(seq)
    assert _tick(seq).sign_enable is True    # 아직 팻말 전 → 탐색해야 하므로 ON.
    cmd = seq.update(_obs(sign_direction=SignDirection.LEFT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
    assert cmd.sign_enable is True
    cmd = seq.update(_obs(), dt=0.1)         # 분기 종료 → 1회성 래치 → OFF.
    assert seq.phase == MissionPhase.LANE_FOLLOW
    assert cmd.sign_enable is False


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


# --- 출발 신호등 게이트 우회(테스트 스위치) -------------------------------

def test_traffic_light_start_disabled_starts_driving_immediately():
    """traffic_light_start_enable=False → WAIT_START_SIGNAL 건너뛰고 즉시 주행."""
    seq = MissionSequencer(_cfg(traffic_light_start_enable=False))
    assert seq.phase == MissionPhase.LANE_FOLLOW      # 초록불 없이 바로 주행 페이즈.
    cmd = _tick(seq)                                  # 신호등 관측 없음.
    assert cmd.go is True
    assert seq.phase == MissionPhase.LANE_FOLLOW


def test_traffic_light_start_enabled_still_waits():
    """기본(True)은 종전대로 초록불 대기 — 대회 주행 동작이 안 바뀐다."""
    seq = MissionSequencer(_cfg())
    assert seq.phase == MissionPhase.WAIT_START_SIGNAL
    assert _tick(seq).go is False
    seq.update(_obs(traffic_light=TrafficLight.GREEN), dt=0.05)
    assert seq.phase == MissionPhase.LANE_FOLLOW


def test_traffic_light_start_disabled_still_reaches_sign_branch():
    """우회로 시작해도 팻말 분기는 정상 동작(= 이 스위치로 팻말만 검증 가능)."""
    seq = MissionSequencer(_cfg(traffic_light_start_enable=False,
                                sign_branch_duration=1.0))
    seq.update(_obs(sign_direction=SignDirection.RIGHT), dt=0.05)
    assert seq.phase == MissionPhase.SIGN_BRANCH
