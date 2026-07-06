"""@file mission.py
@brief 상위 미션 시퀀스(MissionSequencer) — ROS-free 순수 로직.

@details
2계층 판단 구조의 **위층**. "지금 미션 어디쯤인지"(M0~M6 페이즈)를 정하고,
각 페이즈에 맞는 지시를 낸다. 눈앞 차선에 대한 즉각 반응(DRIVE/SLOW/LOST)은
아래층 DecisionMaker(decision.py)에 위임하고, 위층은 그 결과 위에 미션 게이트를
덮는다: 정지 페이즈면 STOP 강제, 주행 페이즈면 아래층 결과 + 미션 지시
(follow_color/turn_hint)를 얹는다.

@par 페이즈 전이 (docs/mission_fsm.md)
- M0 WAIT_GREEN  : 신호등 GREEN → M1
- M1 LANE_WHITE_1: 흰→노랑 색전환 → M2
- M2 CIRCLE      : 정지선 rising-edge로 count, count에 따라 turn_hint,
                   노랑→흰 색전환 → M3
- M3 LANE_WHITE_2: 빨강 바닥 구역 검출 → M4
- M4 OBSTACLE    : 빨강 구역 안(흰 차선 추종). 아루코 보이면 STOP, 아니면 주행.
                   빨강 구역 벗어남 → M5
- M5 LANE_WHITE_3: 정지구역 접근(거리 이내) → M6
- M6 FINISH      : STOP(종료)

@par 설계 원칙
- 아래층 재사용: DecisionMaker를 소유·호출(안전 반사 로직 중복 안 함).
- 제어기 튜닝은 안 건드리고 상태·게이트·미션 지시만 낸다(계약 원칙).
- 좌/우·거리 임계값은 트랙마다 달라 전부 MissionConfig(YAML).
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
    TurnHint,
)


class MissionPhase(IntEnum):
    """@brief 상위 미션 페이즈(M0~M6). 값=미션 순서."""

    M0_WAIT_GREEN = 0    ##< 출발점 신호등 대기.
    M1_LANE_WHITE_1 = 1  ##< 첫 흰 차선 직진.
    M2_CIRCLE = 2        ##< 노랑 원형 지름길(한 바퀴 + 갈림길 분기).
    M3_LANE_WHITE_2 = 3  ##< 지름길 탈출 후 흰 직진.
    M4_OBSTACLE = 4      ##< 빨강 장애물 구역(아루코 정지/재출발).
    M5_LANE_WHITE_3 = 5  ##< 장애물 이후 흰 직진.
    M6_FINISH = 6        ##< 정지구역 도착 → 종료.


class TrafficLight(IntEnum):
    """@brief 신호등 인지 결과(YOLO). M0 출발 트리거."""

    NONE = 0    ##< 미검출.
    RED = 1     ##< 빨간불(대기).
    GREEN = 2   ##< 초록불(출발).


@dataclass
class MissionObservation:
    """@brief 미션 판단 입력 — 차선 관측 + 미션 신호 묶음.

    @details 아래층이 쓰는 LaneObservation을 그대로 품고(=lane), 위층 미션 전이에
    필요한 신호를 더한다. ROS 노드가 여러 인지 토픽을 모아 이 객체를 채운다.
    필드 의미는 docs/mission_fsm.md §6(인터페이스 확장) 참조.
    """

    lane: LaneObservation = field(default_factory=LaneObservation)  ##< 차선 관측(아래층 입력).
    traffic_light: TrafficLight = TrafficLight.NONE  ##< 신호등(M0).
    lane_color: LaneColor = LaneColor.WHITE          ##< 현재 인지된 차선 색(M1↔M2 전환).
    red_zone_detected: bool = False   ##< 빨강 바닥 구역 안(M4 진입/탈출).
    aruco_present: bool = False       ##< 아루코 마커 보임(M4 정지/재출발).
    stop_zone_detected: bool = False  ##< 정지구역 검출(M6).
    stop_zone_dist: float = -1.0      ##< 정지구역까지 거리 [m](미검출 -1.0).


class MissionSequencer:
    """@brief 미션 페이즈 상태기계(순수 로직). 아래층 DecisionMaker를 감싼다.

    @details 매 주기 update(obs, dt)를 호출한다. 먼저 페이즈를 갱신(_advance_phase)
    하고, 그 페이즈에 맞는 DriveCommand를 만든다(_command). 상태는 명시적이며
    reset()으로 초기화된다.
    """

    def __init__(self,
                 config: MissionConfig | None = None,
                 decision_config: DecisionConfig | None = None):
        """@param config 미션 임계값(좌/우·거리). None이면 기본값.
        @param decision_config 아래층 판단 임계값. None이면 기본값."""
        self.cfg = config or MissionConfig()
        self.decision = DecisionMaker(decision_config)
        self.reset()

    def reset(self) -> None:
        """@brief 페이즈·카운터·아래층을 모두 초기화한다."""
        self._phase = MissionPhase.M0_WAIT_GREEN
        self._stopline_count = 0
        self._prev_stop_line = False     ##< 직전 주기 정지선 여부(rising-edge용).
        self._since_count = float("inf")  ##< 마지막 카운트 이후 경과[s](디바운스). inf=첫 카운트 허용.
        self.decision.reset()

    @property
    def phase(self) -> MissionPhase:
        """@brief 현재 미션 페이즈."""
        return self._phase

    @property
    def stopline_count(self) -> int:
        """@brief M2 정지선 누적 카운트."""
        return self._stopline_count

    def update(self, obs: MissionObservation, dt: float) -> DriveCommand:
        """@brief 한 주기 미션 판단을 수행하고 DriveCommand를 반환한다.

        @param obs 이번 주기 미션 관측.
        @param dt  직전 호출 이후 경과시간 [s](>=0).
        @return 이번 주기 DriveCommand(아래층 결과 + 미션 지시/게이트).
        """
        dt = max(0.0, dt)
        self._since_count += dt
        self._advance_phase(obs)
        return self._command(obs, dt)

    # --- 페이즈 전이 -----------------------------------------------------

    def _advance_phase(self, obs: MissionObservation) -> None:
        """@brief 현재 페이즈와 관측으로 다음 페이즈를 정한다."""
        p = self._phase
        cfg = self.cfg

        if p == MissionPhase.M0_WAIT_GREEN:
            if obs.traffic_light == TrafficLight.GREEN:
                self._phase = MissionPhase.M1_LANE_WHITE_1

        elif p == MissionPhase.M1_LANE_WHITE_1:
            if obs.lane_color == LaneColor.YELLOW:
                self._enter_circle()

        elif p == MissionPhase.M2_CIRCLE:
            self._count_stop_lines(obs)
            # 노랑→흰 전환 = 지름길 탈출(count는 2가 됐어야 정상).
            if obs.lane_color == LaneColor.WHITE:
                self._phase = MissionPhase.M3_LANE_WHITE_2

        elif p == MissionPhase.M3_LANE_WHITE_2:
            if obs.red_zone_detected:
                self._phase = MissionPhase.M4_OBSTACLE

        elif p == MissionPhase.M4_OBSTACLE:
            if not obs.red_zone_detected:
                self._phase = MissionPhase.M5_LANE_WHITE_3

        elif p == MissionPhase.M5_LANE_WHITE_3:
            if (obs.stop_zone_detected
                    and 0.0 <= obs.stop_zone_dist <= cfg.stop_zone_stop_dist):
                self._phase = MissionPhase.M6_FINISH
        # M6_FINISH: 종료 상태(전이 없음).

    def _enter_circle(self) -> None:
        """@brief M2 진입: 페이즈 전환 + 정지선 카운터 초기화."""
        self._phase = MissionPhase.M2_CIRCLE
        self._stopline_count = 0
        self._prev_stop_line = False
        self._since_count = float("inf")

    def _count_stop_lines(self, obs: MissionObservation) -> None:
        """@brief 갈림길 정지선을 rising-edge로 센다(디바운스 적용).

        @details 정지선은 두꺼워 여러 프레임 잡히므로 False→True 순간에만 +1하고,
        같은 정지선 재검출을 막기 위해 마지막 카운트 후 stop_line_debounce 이상
        지나야 다시 센다.
        """
        rising = obs.lane.stop_line and not self._prev_stop_line
        if rising and self._since_count >= self.cfg.stop_line_debounce:
            self._stopline_count += 1
            self._since_count = 0.0
        self._prev_stop_line = obs.lane.stop_line

    # --- 명령 생성 -------------------------------------------------------

    def _command(self, obs: MissionObservation, dt: float) -> DriveCommand:
        """@brief 현재 페이즈에 맞는 DriveCommand를 만든다.

        @details 정지 페이즈(M0·M6, 그리고 M4에서 아루코 보임)는 STOP 강제.
        주행 페이즈는 아래층 DecisionMaker 결과에 미션 지시를 얹는다. M2 정지선은
        갈림길 표식이라 아래층엔 마스킹해 넘긴다(정지선 STOP 방지).
        """
        p = self._phase

        if p in (MissionPhase.M0_WAIT_GREEN, MissionPhase.M6_FINISH):
            return self._stop_command()
        if p == MissionPhase.M4_OBSTACLE and obs.aruco_present:
            return self._stop_command()

        lane = obs.lane
        if p == MissionPhase.M2_CIRCLE:
            # 갈림길 정지선은 위층이 count로 소비. 아래층엔 숨겨 통과시킨다.
            lane = replace(lane, stop_line=False, stop_line_dist=-1.0)

        cmd = self.decision.update(lane, dt)
        cmd.follow_color = self._follow_color()
        cmd.turn_hint = self._turn_hint()
        return cmd

    def _stop_command(self) -> DriveCommand:
        """@brief 정지 게이트 명령(현재 페이즈의 추종 색 유지)."""
        return DriveCommand(
            state=DriveState.STOP, go=False, speed_scale=0.0,
            lookahead_scale=1.0, steer_limit=1.0,
            follow_color=self._follow_color(), turn_hint=TurnHint.NONE)

    def _follow_color(self) -> LaneColor:
        """@brief 추종할 차선 색. M2만 노랑, 나머지 흰."""
        if self._phase == MissionPhase.M2_CIRCLE:
            return LaneColor.YELLOW
        return LaneColor.WHITE

    def _turn_hint(self) -> TurnHint:
        """@brief 갈림길 조향 힌트. M2에서 count에 따라 loop/exit 쪽."""
        if self._phase != MissionPhase.M2_CIRCLE:
            return TurnHint.NONE
        if self._stopline_count >= 2:
            return self.cfg.exit_side()
        if self._stopline_count == 1:
            return self.cfg.loop_side()
        return TurnHint.NONE
