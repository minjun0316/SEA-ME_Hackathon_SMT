"""@file mission.py
@brief 상위 미션 시퀀스(MissionSequencer, 12-state) — ROS-free 순수 로직.

@details
2계층 판단의 **위층**. 로터리 지름길 미션의 페이즈(WAIT_START_SIGNAL~FINISH_STOP)를
정하고, 각 상태에서 **인지에 줄 지시(directive)** 를 낸다:
  - follow_color : 노랑/흰 어느 차선을 따라갈지
  - roi_mode     : 인지가 볼 ROI(full/lower/right/left/lower+aruco)
  - turn_hint    : target 좌/우 bias(원형 계속=오른쪽, 출구=왼쪽)
그리고 제어용 게이트(go/speed_scale/lookahead_scale)도 낸다.

**역할 경계**: ROI 자르기·색 mask·target point 추출·bias 적용·아루코 검출은 모두
**인지(OpenCV/YOLO) 몫**이다. 위층은 "어느 모드로 볼지"만 지시하고, 눈앞 차선에
대한 즉각 반응(DRIVE/SLOW/LOST)은 아래층 DecisionMaker에 위임한다.

@par 로터리 한 바퀴 판정 (핵심)
yaw 누적각이 아니라 **로터리 내부 정지선 검출 횟수**만 사용한다.
정지선은 **노란(점선) 차선이 검출되는 동안(yellow_detected)에만** 센다 — 로터리 안
노란 구간을 벗어나면 정지선 신호가 떠도 무시한다.
  - ROUNDABOUT_FOLLOW에서 1번째 정지선 → count=1 → ROUNDABOUT_CONTINUE_RIGHT
    (왼쪽 출구 무시, 오른쪽 원형 차선으로 계속 회전)
  - 2번째 정지선 → ROUNDABOUT_EXIT_LEFT (오른쪽 버리고 왼쪽 출구로 탈출)
정지선은 debounce + 진입무시시간으로 중복/오검출을 막는다.

@par 탈출 후 (커넥터)
탈출하면 바로 흰색이 아니라 노란 점선 → 노란 연결도로 → 흰색 순서라, EXIT_CONNECTOR
에서 노랑을 계속 따라가다 **흰색이 일정 시간 안정 검출**되면 OUTER_LANE_FOLLOW로 간다.
점선 구간은 검출이 끊겨도 정지하지 않는다(왼쪽으로 저속 크립, target 유지는 인지 몫).

시퀀스 설계: docs/mission_fsm.md, 계약: docs/interfaces.md, 합의: docs/perception_agreement.md.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import IntEnum

from ..config_schema import DecisionConfig, MissionConfig
from .decision import (
    DecisionMaker,
    DriveCommand,
    DriveState,
    LaneColor,
    LaneObservation,
    RoiMode,
    TurnHint,
)


class MissionPhase(IntEnum):
    """@brief 상위 미션 페이즈(12-state). 값=미션 순서."""

    WAIT_START_SIGNAL = 0        ##< 출발점 체커보드에서 초록불 대기.
    START_STRAIGHT = 1           ##< 하단 직선 흰 차선 주행.
    SHORTCUT_APPROACH = 2        ##< 노란 지름길 진입로 추종(감속).
    ROUNDABOUT_ENTRY = 3         ##< 로터리 합류(정지선 무시 시간).
    ROUNDABOUT_FOLLOW = 4        ##< 로터리 일반 추종(정지선 카운트).
    ROUNDABOUT_CONTINUE_RIGHT = 5  ##< 1번째 정지선 후: 오른쪽 원형으로 계속 회전.
    ROUNDABOUT_EXIT_LEFT = 6     ##< 2번째 정지선 후: 왼쪽 출구로 탈출.
    EXIT_CONNECTOR = 7           ##< 탈출 후 노란 점선/연결도로 추종.
    OUTER_LANE_FOLLOW = 8        ##< 외곽 흰 차선 주행.
    DYNAMIC_OBSTACLE_ZONE = 9    ##< 빨강 구역: 아루코 보이면 정지.
    FINISH_APPROACH = 10         ##< 체커보드 도착선 접근.
    FINISH_STOP = 11             ##< 도착 정지(종료).


class TrafficLight(IntEnum):
    """@brief 신호등 인지 결과(YOLO)."""

    NONE = 0
    RED = 1
    GREEN = 2


@dataclass
class MissionObservation:
    """@brief 미션 판단 입력 — 차선 관측 + 미션 신호 묶음.

    @details 아래층이 쓰는 LaneObservation을 품고(=lane), 위층 전환에 필요한 신호를
    더한다. 색은 노랑/흰을 **둘 다 항상** 보고한다(현재 따라가는 색과 무관하게, 다른
    색 등장을 감지해야 하므로). 정지선은 lane.stop_line 재사용. 신호 출처/의미는
    docs/perception_agreement.md 참조.
    """

    lane: LaneObservation = field(default_factory=LaneObservation)  ##< 차선 관측(아래층 입력).
    traffic_light: TrafficLight = TrafficLight.NONE  ##< 신호등(YOLO): 출발/정지.
    yellow_detected: bool = False    ##< 노랑 차선 검출(OpenCV): 지름길 등장/추종.
    yellow_confidence: float = 0.0   ##< 노랑 신뢰도(로깅/튜닝용).
    white_detected: bool = False     ##< 흰 차선 검출(OpenCV): 커넥터→외곽 복귀 판정.
    white_confidence: float = 0.0    ##< 흰 신뢰도(로깅/튜닝용).
    red_zone_detected: bool = False  ##< 빨강 바닥 구역(OpenCV): 장애물 구간.
    aruco_present: bool = False      ##< 아루코 마커(cv2.aruco, 하단 ROI): 정지/재출발.
    checkerboard_detected: bool = False  ##< 체커보드(YOLO): 도착선.


# --- 페이즈 분류(지시/오버레이 결정용) ------------------------------------- #
_YELLOW_PHASES = frozenset({
    MissionPhase.SHORTCUT_APPROACH, MissionPhase.ROUNDABOUT_ENTRY,
    MissionPhase.ROUNDABOUT_FOLLOW, MissionPhase.ROUNDABOUT_CONTINUE_RIGHT,
    MissionPhase.ROUNDABOUT_EXIT_LEFT, MissionPhase.EXIT_CONNECTOR,
})
_ROI_MAP = {
    MissionPhase.START_STRAIGHT: RoiMode.LOWER,
    MissionPhase.SHORTCUT_APPROACH: RoiMode.LOWER,
    MissionPhase.ROUNDABOUT_ENTRY: RoiMode.FULL,
    MissionPhase.ROUNDABOUT_FOLLOW: RoiMode.FULL,
    MissionPhase.ROUNDABOUT_CONTINUE_RIGHT: RoiMode.RIGHT,
    MissionPhase.ROUNDABOUT_EXIT_LEFT: RoiMode.LEFT,
    MissionPhase.EXIT_CONNECTOR: RoiMode.LEFT,
    MissionPhase.OUTER_LANE_FOLLOW: RoiMode.FULL,
    MissionPhase.DYNAMIC_OBSTACLE_ZONE: RoiMode.LOWER_ARUCO,
    MissionPhase.FINISH_APPROACH: RoiMode.LOWER,
}
# 아래층 반응형을 돌리는 주행 페이즈(정지선은 카운트 전용 → 마스킹).
_MASK_STOPLINE_PHASES = frozenset({
    MissionPhase.SHORTCUT_APPROACH, MissionPhase.ROUNDABOUT_ENTRY,
    MissionPhase.ROUNDABOUT_FOLLOW, MissionPhase.ROUNDABOUT_CONTINUE_RIGHT,
})
# 속도 상한(감속) 적용 페이즈.
_SLOW_PHASES = frozenset({
    MissionPhase.SHORTCUT_APPROACH, MissionPhase.ROUNDABOUT_ENTRY,
    MissionPhase.ROUNDABOUT_FOLLOW, MissionPhase.ROUNDABOUT_CONTINUE_RIGHT,
    MissionPhase.DYNAMIC_OBSTACLE_ZONE,
})
# 짧은 lookahead 적용 페이즈.
_ROUNDABOUT_PHASES = frozenset({
    MissionPhase.ROUNDABOUT_ENTRY, MissionPhase.ROUNDABOUT_FOLLOW,
    MissionPhase.ROUNDABOUT_CONTINUE_RIGHT,
})
# 점선/탈출: 검출 끊겨도 정지 금지(저속 크립).
_CREEP_PHASES = frozenset({
    MissionPhase.ROUNDABOUT_EXIT_LEFT, MissionPhase.EXIT_CONNECTOR,
})


class MissionSequencer:
    """@brief 12-state 미션 상태기계(순수 로직). 아래층 DecisionMaker를 감싼다.

    @details 매 주기 update(obs, dt)로 페이즈를 갱신하고 DriveCommand(제어 게이트 +
    인지 지시)를 반환한다. 한 바퀴 판정은 정지선 카운트만 사용한다.
    """

    def __init__(self,
                 config: MissionConfig | None = None,
                 decision_config: DecisionConfig | None = None):
        """@param config 미션 타이머/방향/배율. None이면 기본값.
        @param decision_config 아래층 판단 임계값. None이면 기본값."""
        self.cfg = config or MissionConfig()
        self.decision = DecisionMaker(decision_config)
        self.reset()

    def reset(self) -> None:
        """@brief 페이즈·카운터·타이머·아래층을 모두 초기화한다."""
        self._phase = MissionPhase.WAIT_START_SIGNAL
        self._phase_time = 0.0        ##< 현재 페이즈 지속시간[s].
        self._stopline_count = 0      ##< 로터리 정지선 누적(0→1→탈출).
        self._prev_stop_line = False  ##< 직전 주기 정지선 여부(rising-edge용).
        self._since_count = float("inf")  ##< 마지막 카운트 이후 경과[s](디바운스).
        self._white_time = 0.0        ##< EXIT_CONNECTOR 흰색 연속 검출 누적[s].
        self.decision.reset()

    @property
    def phase(self) -> MissionPhase:
        """@brief 현재 미션 페이즈."""
        return self._phase

    @property
    def stopline_count(self) -> int:
        """@brief 로터리 정지선 누적 카운트."""
        return self._stopline_count

    def update(self, obs: MissionObservation, dt: float) -> DriveCommand:
        """@brief 한 주기: 페이즈 갱신 → DriveCommand(제어 게이트 + 인지 지시) 반환."""
        dt = max(0.0, dt)
        self._phase_time += dt
        self._since_count += dt
        self._advance_phase(obs, dt)
        cmd = self._command(obs, dt)
        self._prev_stop_line = bool(obs.lane.stop_line)  # 매 틱 갱신(rising-edge용).
        return cmd

    # --- 페이즈 전이 -----------------------------------------------------

    def _set_phase(self, p: MissionPhase) -> None:
        """@brief 페이즈 전환 + 관련 타이머/카운터 초기화."""
        self._phase = p
        self._phase_time = 0.0
        if p == MissionPhase.SHORTCUT_APPROACH:
            # 로터리 시도 시작: 정지선 카운트/디바운스 리셋.
            self._stopline_count = 0
            self._since_count = float("inf")
        if p == MissionPhase.EXIT_CONNECTOR:
            self._white_time = 0.0

    def _advance_phase(self, obs: MissionObservation, dt: float) -> None:
        """@brief 현재 페이즈와 관측으로 다음 페이즈를 정한다."""
        p = self._phase
        cfg = self.cfg

        if p == MissionPhase.WAIT_START_SIGNAL:
            if obs.traffic_light == TrafficLight.GREEN:
                self._set_phase(MissionPhase.START_STRAIGHT)

        elif p == MissionPhase.START_STRAIGHT:
            if obs.yellow_detected:  # 노란 지름길 진입로 등장.
                self._set_phase(MissionPhase.SHORTCUT_APPROACH)

        elif p == MissionPhase.SHORTCUT_APPROACH:
            if self._phase_time >= cfg.shortcut_approach_sec:
                self._set_phase(MissionPhase.ROUNDABOUT_ENTRY)

        elif p == MissionPhase.ROUNDABOUT_ENTRY:
            if self._phase_time >= cfg.stopline_ignore_after_entry_sec:
                self._set_phase(MissionPhase.ROUNDABOUT_FOLLOW)

        elif p == MissionPhase.ROUNDABOUT_FOLLOW:
            self._count_stop_lines(obs)  # 카운트 → CONTINUE_RIGHT / EXIT_LEFT 전이.

        elif p == MissionPhase.ROUNDABOUT_CONTINUE_RIGHT:
            if self._phase_time >= cfg.continue_right_sec:
                self._set_phase(MissionPhase.ROUNDABOUT_FOLLOW)

        elif p == MissionPhase.ROUNDABOUT_EXIT_LEFT:
            if self._phase_time >= cfg.exit_left_sec:
                self._set_phase(MissionPhase.EXIT_CONNECTOR)

        elif p == MissionPhase.EXIT_CONNECTOR:
            # 흰색이 일정 시간 연속 검출되면 외곽 복귀.
            self._white_time = self._white_time + dt if obs.white_detected else 0.0
            if self._white_time >= cfg.white_stable_sec:
                self._set_phase(MissionPhase.OUTER_LANE_FOLLOW)

        elif p == MissionPhase.OUTER_LANE_FOLLOW:
            if obs.red_zone_detected:
                self._set_phase(MissionPhase.DYNAMIC_OBSTACLE_ZONE)

        elif p == MissionPhase.DYNAMIC_OBSTACLE_ZONE:
            if not obs.red_zone_detected:
                self._set_phase(MissionPhase.FINISH_APPROACH)

        elif p == MissionPhase.FINISH_APPROACH:
            if obs.checkerboard_detected:
                self._set_phase(MissionPhase.FINISH_STOP)
        # FINISH_STOP: 종료(전이 없음).

    def _count_stop_lines(self, obs: MissionObservation) -> None:
        """@brief 로터리 정지선 카운트(rising-edge + debounce) → 분기 전이.

        @details 1번째(count 0→1) → CONTINUE_RIGHT, 2번째(count→2) → EXIT_LEFT.
        같은 정지선 중복 카운트는 rising-edge + stopline_debounce_sec로 막는다.
        @note 노란(점선) 차선이 검출되는 동안(yellow_detected)에만 카운트한다. 노랑이
        사라지면 정지선 신호가 떠도 무시 — 로터리 안 노란 구간에서만 세기 위함.
        """
        rising = obs.lane.stop_line and not self._prev_stop_line
        if not (rising and obs.yellow_detected
                and self._since_count >= self.cfg.stopline_debounce_sec):
            return
        self._stopline_count += 1
        self._since_count = 0.0
        if self._stopline_count == 1:
            self._set_phase(MissionPhase.ROUNDABOUT_CONTINUE_RIGHT)
        else:  # 2번째(이상) → 탈출.
            self._set_phase(MissionPhase.ROUNDABOUT_EXIT_LEFT)

    # --- 명령/지시 생성 --------------------------------------------------

    def _directives(self):
        """@brief 현재 페이즈의 인지 지시 (follow_color, roi_mode, turn_hint)."""
        p = self._phase
        follow = LaneColor.YELLOW if p in _YELLOW_PHASES else LaneColor.WHITE
        roi = _ROI_MAP.get(p, RoiMode.FULL)
        if p == MissionPhase.ROUNDABOUT_CONTINUE_RIGHT:
            hint = self.cfg.continue_side()
        elif p in (MissionPhase.ROUNDABOUT_EXIT_LEFT, MissionPhase.EXIT_CONNECTOR):
            hint = self.cfg.exit_side()
        else:
            hint = TurnHint.NONE
        return follow, roi, hint

    def _command(self, obs: MissionObservation, dt: float) -> DriveCommand:
        """@brief 현재 페이즈에 맞는 DriveCommand(제어 게이트 + 인지 지시)."""
        p = self._phase
        follow, roi, hint = self._directives()

        # 정지 게이트: 출발 대기 / 도착 / 장애물(아루코).
        if p in (MissionPhase.WAIT_START_SIGNAL, MissionPhase.FINISH_STOP):
            return self._stop_command(follow, roi)
        if p == MissionPhase.DYNAMIC_OBSTACLE_ZONE and obs.aruco_present:
            return self._stop_command(follow, roi)  # obstacle_stop.

        # 점선/탈출: 검출 끊겨도 정지하지 말고 저속으로 왼쪽 크립.
        if p in _CREEP_PHASES:
            return self._creep_command(follow, roi, hint)

        # 나머지 주행: 아래층 반응형 + 미션 지시 오버레이.
        lane = obs.lane
        if p in _MASK_STOPLINE_PHASES:
            # 로터리 정지선은 한 바퀴 카운트용 → 아래층엔 숨겨 통과(정지 방지).
            lane = replace(lane, stop_line=False, stop_line_dist=-1.0)
        cmd = self.decision.update(lane, dt)
        cmd.follow_color = follow
        cmd.roi_mode = roi
        cmd.turn_hint = hint
        if p in _SLOW_PHASES:
            cmd.speed_scale = min(cmd.speed_scale, self.cfg.slow_speed_scale)
        if p in _ROUNDABOUT_PHASES:
            cmd.lookahead_scale = min(cmd.lookahead_scale, self.cfg.roundabout_lookahead_scale)
        return cmd

    def _stop_command(self, follow: LaneColor, roi: RoiMode) -> DriveCommand:
        """@brief 정지 게이트 명령(현재 지시 색/ROI 유지)."""
        return DriveCommand(
            state=DriveState.STOP, go=False, speed_scale=0.0,
            lookahead_scale=1.0, steer_limit=1.0,
            follow_color=follow, turn_hint=TurnHint.NONE, roi_mode=roi)

    def _creep_command(self, follow: LaneColor, roi: RoiMode,
                       hint: TurnHint) -> DriveCommand:
        """@brief 점선 구간 저속 크립(정지 금지, 방향 유지). target 유지는 인지 몫."""
        return DriveCommand(
            state=DriveState.SLOW, go=True, speed_scale=self.cfg.slow_speed_scale,
            lookahead_scale=self.cfg.roundabout_lookahead_scale, steer_limit=1.0,
            follow_color=follow, turn_hint=hint, roi_mode=roi)
