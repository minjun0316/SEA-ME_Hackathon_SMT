"""@file mission.py
@brief 상위 미션 시퀀스(MissionSequencer) — ROS-free 순수 로직.

@details
2계층 판단의 **위층**. 신호등 출발 대기 → 차선 주행 → 장애물 구역 → 도착 정지의
간단한 순차 상태기계다. 각 상태에서 제어 게이트(go/speed_scale)와 인지 지시
(follow_color/roi_mode)를 내고, 눈앞 차선에 대한 즉각 반응(DRIVE/SLOW/LOST)은
아래층 DecisionMaker에 위임한다.

@par 로터리(원형 지름길) 회전은 여기서 하지 않는다
로터리 진입/탈출 회전은 이 FSM이 아니라 **제어단의 개루프 고정조향 기동**
(core.planning.stopline_maneuver.StoplineManeuver, controller_node)이 정지선
카운트로 처리한다. 따라서 이 FSM은 로터리를 별도 페이즈로 다루지 않고, 로터리
구간에서도 일반 차선 주행(LANE_FOLLOW) 상태로 통과한다. (ROI를 좌/우로 전환해
차선추종으로 도는 옛 방식은 폐기 — 07-13 결정.)

@par 역할 경계
ROI 자르기·색 mask·target point 추출·아루코 검출은 모두 **인지(OpenCV/YOLO) 몫**이다.
위층은 "어느 모드로 볼지(roi_mode/follow_color)"만 지시한다.

시퀀스 설계: docs/mission_fsm.md, 계약: docs/interfaces.md, 합의: docs/perception_agreement.md.
"""
from __future__ import annotations

from dataclasses import dataclass, field
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
    """@brief 상위 미션 페이즈. 값=미션 순서."""

    WAIT_START_SIGNAL = 0        ##< 출발점 체커보드에서 초록불 대기.
    LANE_FOLLOW = 1              ##< 흰 차선 주행(외곽/직진).
    SHORTCUT = 2                 ##< 지름길: 노랑 차선 추종(인지 on_yellow 래치). 로터리 회전은 StoplineManeuver 몫.
    DYNAMIC_OBSTACLE_ZONE = 3    ##< 아루코 마커 정지 구역: 마커 보이면 정지, 치우면 도착 접근(red_zone 폐기 07-13).
    FINISH_APPROACH = 4          ##< 체커보드 도착선 접근.
    FINISH_STOP = 5              ##< 도착 정지(종료).


class TrafficLight(IntEnum):
    """@brief 신호등 인지 결과(YOLO)."""

    NONE = 0
    RED = 1
    GREEN = 2


@dataclass
class MissionObservation:
    """@brief 미션 판단 입력 — 차선 관측 + 미션 신호 묶음.

    @details 아래층이 쓰는 LaneObservation을 품고(=lane), 위층 전환에 필요한 미션
    신호(신호등/빨강구역/아루코/체커보드)를 더한다. 정지선은 lane.stop_line 재사용.
    신호 출처/의미는 docs/perception_agreement.md 참조.
    """

    lane: LaneObservation = field(default_factory=LaneObservation)  ##< 차선 관측(아래층 입력).
    traffic_light: TrafficLight = TrafficLight.NONE  ##< 신호등(YOLO): 출발.
    on_yellow: bool = False          ##< 인지 노랑모드 래치(지름길 노랑선 추종 중): SHORTCUT 전환.
    red_zone_detected: bool = False  ##< 빨강 바닥 구역(OpenCV). ⚠07-13 폐기: 전이에 미사용(필드는 msg 호환 유지). 정지는 aruco_present가 담당.
    aruco_present: bool = False      ##< 아루코 마커(cv2.aruco): 정지 구역 진입/유지 트리거 + YOLO 재점화.
    checkerboard_detected: bool = False  ##< 체커보드(YOLO): 도착선.


# --- 페이즈별 인지 ROI 지시 ------------------------------------------------- #
_ROI_MAP = {
    MissionPhase.LANE_FOLLOW: RoiMode.LOWER,
    MissionPhase.SHORTCUT: RoiMode.LOWER,
    MissionPhase.DYNAMIC_OBSTACLE_ZONE: RoiMode.LOWER_ARUCO,
    MissionPhase.FINISH_APPROACH: RoiMode.LOWER,
}
# 속도 상한(감속) 적용 페이즈(지름길 커브·장애물 구간).
_SLOW_PHASES = frozenset({MissionPhase.SHORTCUT, MissionPhase.DYNAMIC_OBSTACLE_ZONE})


class MissionSequencer:
    """@brief 미션 상태기계(순수 로직). 아래층 DecisionMaker를 감싼다.

    @details 매 주기 update(obs, dt)로 페이즈를 갱신하고 DriveCommand(제어 게이트 +
    인지 지시)를 반환한다. 로터리 회전은 제어단 StoplineManeuver가 담당하므로 여기선
    다루지 않는다.
    """

    def __init__(self,
                 config: MissionConfig | None = None,
                 decision_config: DecisionConfig | None = None):
        """@param config 미션 배율. None이면 기본값.
        @param decision_config 아래층 판단 임계값. None이면 기본값."""
        self.cfg = config or MissionConfig()
        self.decision = DecisionMaker(decision_config)
        self.reset()

    def reset(self) -> None:
        """@brief 페이즈·타이머·아래층을 모두 초기화한다."""
        self._phase = MissionPhase.WAIT_START_SIGNAL
        self._phase_time = 0.0        ##< 현재 페이즈 지속시간[s].
        self._yolo_relatch = False    ##< 장애물구역서 아루코 최초검출 시 True(도착까지 YOLO 재점화 래치).
        self.decision.reset()

    @property
    def phase(self) -> MissionPhase:
        """@brief 현재 미션 페이즈."""
        return self._phase

    def update(self, obs: MissionObservation, dt: float) -> DriveCommand:
        """@brief 한 주기: 페이즈 갱신 → DriveCommand(제어 게이트 + 인지 지시) 반환."""
        dt = max(0.0, dt)
        self._phase_time += dt
        self._advance_phase(obs)
        cmd = self._command(obs, dt)
        cmd.yolo_enable = self._yolo_enable(obs)
        return cmd

    def _yolo_enable(self, obs: MissionObservation) -> bool:
        """@brief 인지 YOLO 추론 게이트 산출(페이즈 + 아루코 래치).

        @details 무거운 YOLO는 신호 검출이 필요한 양 끝단에서만 켠다:
        출발 신호등 대기(WAIT_START_SIGNAL)=ON → 주행중(LANE_FOLLOW/SHORTCUT)=OFF로
        FPS 확보 → 아루코 마커가 처음 보이면(=정지/종료 임박, 구역 무관) ON을 래치해
        도착 체커보드(FINISH_APPROACH/STOP)까지 유지한다. 래치는 한 번 서면 reset()
        전까지 유지(마커 깜빡여도 안 꺼짐). yolo_gate_enable=False면 게이트를 끄고
        항상 ON(기존 동작). @return True=YOLO 추론 ON.
        """
        if not self.cfg.yolo_gate_enable:
            return True
        # 출발 이후 아루코가 보이면(=정지/종료 임박) YOLO 재점화 래치. red_zone stub과
        # 무관하게 동작하도록 페이즈 조건 없이 아루코만으로 건다.
        if obs.aruco_present and self._phase != MissionPhase.WAIT_START_SIGNAL:
            self._yolo_relatch = True
        return (self._phase == MissionPhase.WAIT_START_SIGNAL
                or self._yolo_relatch)

    # --- 페이즈 전이 -----------------------------------------------------

    def _set_phase(self, p: MissionPhase) -> None:
        """@brief 페이즈 전환 + 타이머 초기화."""
        self._phase = p
        self._phase_time = 0.0

    def _advance_phase(self, obs: MissionObservation) -> None:
        """@brief 현재 페이즈와 관측으로 다음 페이즈를 정한다."""
        p = self._phase

        if p == MissionPhase.WAIT_START_SIGNAL:
            if obs.traffic_light == TrafficLight.GREEN:
                self._set_phase(MissionPhase.LANE_FOLLOW)

        elif p == MissionPhase.LANE_FOLLOW:
            # 아루코 마커 = 정지/종료 임박 트리거(red_zone 폐기, 07-13). 마커 보이면
            # 어느 주행 페이즈든 정지구역으로. 없으면 노랑 래치로 지름길 전환.
            if obs.aruco_present:
                self._set_phase(MissionPhase.DYNAMIC_OBSTACLE_ZONE)
            elif obs.on_yellow:
                self._set_phase(MissionPhase.SHORTCUT)

        elif p == MissionPhase.SHORTCUT:
            # 지름길 중에도 아루코 보이면 정지 우선. 아니면 노랑 해제 시 외곽 복귀.
            if obs.aruco_present:
                self._set_phase(MissionPhase.DYNAMIC_OBSTACLE_ZONE)
            elif not obs.on_yellow:
                self._set_phase(MissionPhase.LANE_FOLLOW)

        elif p == MissionPhase.DYNAMIC_OBSTACLE_ZONE:
            # 마커 치우면(재출발) 도착 접근으로. 아루코 도착 근처에만 등장 전제.
            if not obs.aruco_present:
                self._set_phase(MissionPhase.FINISH_APPROACH)

        elif p == MissionPhase.FINISH_APPROACH:
            if obs.checkerboard_detected:
                self._set_phase(MissionPhase.FINISH_STOP)
        # FINISH_STOP: 종료(전이 없음).

    # --- 명령/지시 생성 --------------------------------------------------

    def _command(self, obs: MissionObservation, dt: float) -> DriveCommand:
        """@brief 현재 페이즈에 맞는 DriveCommand(제어 게이트 + 인지 지시)."""
        p = self._phase
        roi = _ROI_MAP.get(p, RoiMode.FULL)
        follow = LaneColor.WHITE  # 로터리 ROI 방식 폐기 → 항상 흰 차선 추종.

        # 정지 게이트: 출발 대기 / 도착 / 장애물(아루코).
        if p in (MissionPhase.WAIT_START_SIGNAL, MissionPhase.FINISH_STOP):
            return self._stop_command(follow, roi)
        if p == MissionPhase.DYNAMIC_OBSTACLE_ZONE and obs.aruco_present:
            return self._stop_command(follow, roi)  # obstacle_stop.

        # 주행: 아래층 반응형 + 미션 지시 오버레이.
        cmd = self.decision.update(obs.lane, dt)
        cmd.follow_color = follow
        cmd.roi_mode = roi
        cmd.turn_hint = TurnHint.NONE
        if p in _SLOW_PHASES:
            cmd.speed_scale = min(cmd.speed_scale, self.cfg.slow_speed_scale)
        return cmd

    def _stop_command(self, follow: LaneColor, roi: RoiMode) -> DriveCommand:
        """@brief 정지 게이트 명령(현재 지시 색/ROI 유지)."""
        return DriveCommand(
            state=DriveState.STOP, go=False, speed_scale=0.0,
            lookahead_scale=1.0, steer_limit=1.0,
            follow_color=follow, turn_hint=TurnHint.NONE, roi_mode=roi)
