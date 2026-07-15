"""@file mission.py
@brief 상위 미션 시퀀스(MissionSequencer) — ROS-free 순수 로직.

@details
2계층 판단의 **위층**. 흰선 폐루프 코스(07-14 재설계)를 6-state 순차 상태기계로 돈다:

  WAIT_START_SIGNAL → LANE_FOLLOW → SIGN_BRANCH → LANE_FOLLOW → OBSTACLE_ZONE
                    → FINISH_WATCH → FINISH_STOP

각 상태에서 제어 게이트(go/speed_scale/steer_bias)와 인지 지시(roi_mode/yolo_enable/
sign_enable)를 내고, 눈앞 차선에 대한 즉각 반응(DRIVE/SLOW/LOST)은 아래층 DecisionMaker에
위임한다. 지름길(노랑 차선추종)·로터리·체커보드 도착은 **폐기**(07-14).

@par 코스 시퀀스
1. 출발점에서 신호등 **초록불** 대기(신호등 YOLO ON) → 초록 확정 시 출발(YOLO OFF).
2. S자 흰선 코스를 **양쪽 흰 차선** 슬라이딩윈도우로 추종(LANE_FOLLOW).
3. S자 끝 **방향 팻말**: OpenCV 팻말색이 잡히면(sign_detected) SIGN_BRANCH 진입 →
   팻말 YOLO(sign_enable) ON → 좌/우 판정 래치 → 흰선 추종은 유지한 채 조향에 bias만
   얹는다 → 고정시간(sign_branch_duration) 경과 시 LANE_FOLLOW 복귀(팻말 1회성 래치).
4. 다시 흰선 기본 추종(LANE_FOLLOW).
5. 동적 장애물 **아루코 마커**: 보이면 정지, 사라지면 재출발(신호등 YOLO 재점화).
6. 재출발 후 **빨간불** 감시(FINISH_WATCH) → 빨간불 보이면 정지(FINISH_STOP=종료).

@par 역할 경계
ROI 자르기·색 mask·target point 추출·아루코/팻말/신호등 검출은 모두 **인지** 몫이다.
위층은 "어느 모드로 볼지(roi_mode)"와 "YOLO 어느 모델을 켤지(yolo_enable/sign_enable)",
"조향 bias/속도배율"만 지시한다.

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

    WAIT_START_SIGNAL = 0        ##< 출발점에서 초록불 대기(신호등 YOLO ON).
    LANE_FOLLOW = 1              ##< 흰 차선 양쪽 추종(S자 코스 + 팻말 후 기본 주행).
    SIGN_BRANCH = 2              ##< 방향 팻말 분기: 팻말 YOLO 좌/우 → 조향 bias(고정시간).
    OBSTACLE_ZONE = 3            ##< 동적 장애물 아루코: 보이면 정지, 치우면 재출발.
    FINISH_WATCH = 4             ##< 재출발 후 빨간불 감시(신호등 YOLO ON).
    FINISH_STOP = 5              ##< 빨간불 → 정지(종료).


class TrafficLight(IntEnum):
    """@brief 신호등 인지 결과(YOLO). 값=MissionCues.TL_* 와 일치."""

    NONE = 0
    RED = 1
    GREEN = 2


class SignDirection(IntEnum):
    """@brief 방향 팻말 인지 결과(팻말 YOLO). 값=MissionCues.SIGN_* 와 일치."""

    NONE = 0
    LEFT = 1
    RIGHT = 2


@dataclass
class MissionObservation:
    """@brief 미션 판단 입력 — 차선 관측 + 미션 신호 묶음.

    @details 아래층이 쓰는 LaneObservation을 품고(=lane), 위층 전환에 필요한 미션
    신호(신호등/아루코/팻말)를 더한다. 신호 출처/의미는 docs/perception_agreement.md 참조.
    """

    lane: LaneObservation = field(default_factory=LaneObservation)  ##< 차선 관측(아래층 입력).
    traffic_light: TrafficLight = TrafficLight.NONE  ##< 신호등(YOLO): 출발(초록)/종료(빨강).
    aruco_present: bool = False          ##< 아루코 마커(cv2.aruco): 정지 구역 진입/유지 + YOLO 재점화.
    sign_detected: bool = False          ##< 방향 팻말색(OpenCV): SIGN_BRANCH 진입 트리거.
    sign_direction: SignDirection = SignDirection.NONE  ##< 방향 팻말 좌/우(팻말 YOLO): 조향 bias 방향.


# --- 페이즈별 인지 ROI 지시 ------------------------------------------------- #
_ROI_MAP = {
    MissionPhase.LANE_FOLLOW: RoiMode.LOWER,
    MissionPhase.SIGN_BRANCH: RoiMode.LOWER,
    MissionPhase.OBSTACLE_ZONE: RoiMode.LOWER_ARUCO,
    MissionPhase.FINISH_WATCH: RoiMode.LOWER,
}


class MissionSequencer:
    """@brief 미션 상태기계(순수 로직). 아래층 DecisionMaker를 감싼다.

    @details 매 주기 update(obs, dt)로 페이즈를 갱신하고 DriveCommand(제어 게이트 +
    인지 지시)를 반환한다.
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
        """@brief 페이즈·타이머·래치·아래층을 모두 초기화한다."""
        self._phase = MissionPhase.WAIT_START_SIGNAL
        self._phase_time = 0.0        ##< 현재 페이즈 지속시간[s].
        self._yolo_relatch = False    ##< 장애물구역서 아루코 최초검출 시 True(빨간불 종료까지 신호등 YOLO 재점화).
        self._sign_done = False       ##< 팻말 분기 완료 1회성 래치(재진입 방지).
        self._sign_dir = SignDirection.NONE  ##< SIGN_BRANCH서 래치한 좌/우(첫 non-NONE 유지).
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
        self._latch_sign_direction(obs)
        cmd = self._command(obs, dt)
        cmd.yolo_enable = self._yolo_enable(obs)
        cmd.sign_enable = self._sign_enable()
        return cmd

    def _yolo_enable(self, obs: MissionObservation) -> bool:
        """@brief 신호등 YOLO 추론 게이트(페이즈 + 아루코 래치).

        @details 무거운 신호등 YOLO는 신호가 필요한 양 끝단에서만 켠다: 출발 신호등
        대기(WAIT_START_SIGNAL)=ON → 주행중 OFF로 FPS 확보 → 아루코 마커가 처음 보이면
        (=정지/종료 임박) ON을 래치해 재출발 후 빨간불 종료 감시(FINISH_WATCH)까지 유지한다.
        래치는 한 번 서면 reset() 전까지 유지(마커 깜빡여도 안 꺼짐). yolo_gate_enable=False면
        게이트를 끄고 항상 ON(기존 동작). @return True=신호등 YOLO 추론 ON.
        """
        if not self.cfg.yolo_gate_enable:
            return True
        if obs.aruco_present and self._phase != MissionPhase.WAIT_START_SIGNAL:
            self._yolo_relatch = True
        return (self._phase == MissionPhase.WAIT_START_SIGNAL
                or self._yolo_relatch)

    def _sign_enable(self) -> bool:
        """@brief 방향 팻말 YOLO(별도 모델) 게이트 — SIGN_BRANCH에서만 ON.

        @details 팻말 모델은 좌/우 판정이 필요한 팻말 분기 구간에서만 켠다(그 외 OFF로
        CPU/FPS 확보). 신호등 게이트(yolo_enable)와 독립. @return True=팻말 YOLO ON.
        """
        return self._phase == MissionPhase.SIGN_BRANCH

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
            # 아루코 = 정지/종료 임박 트리거(최우선). 없으면 팻말색으로 분기(1회만).
            if obs.aruco_present:
                self._set_phase(MissionPhase.OBSTACLE_ZONE)
            elif not self._sign_done and obs.sign_detected:
                self._set_phase(MissionPhase.SIGN_BRANCH)
                self._sign_dir = SignDirection.NONE   # 새 분기 진입 시 방향 래치 초기화.

        elif p == MissionPhase.SIGN_BRANCH:
            # 분기 중에도 아루코 보이면 정지 우선. 아니면 고정시간 경과 시 복귀(1회성 래치).
            if obs.aruco_present:
                self._sign_done = True
                self._set_phase(MissionPhase.OBSTACLE_ZONE)
            elif self._phase_time >= self.cfg.sign_branch_duration:
                self._sign_done = True
                self._set_phase(MissionPhase.LANE_FOLLOW)

        elif p == MissionPhase.OBSTACLE_ZONE:
            # 마커 치우면(재출발) 빨간불 감시로. 아루코는 종료 근처에만 등장 전제.
            if not obs.aruco_present:
                self._set_phase(MissionPhase.FINISH_WATCH)

        elif p == MissionPhase.FINISH_WATCH:
            # 재출발 후 흰선 주행하며 빨간불 감시. 빨간불 = 종료.
            if obs.traffic_light == TrafficLight.RED:
                self._set_phase(MissionPhase.FINISH_STOP)
        # FINISH_STOP: 종료(전이 없음).

    def _latch_sign_direction(self, obs: MissionObservation) -> None:
        """@brief SIGN_BRANCH 중 첫 non-NONE 팻말 방향을 래치(잠깐 NONE에도 안 풀림)."""
        if (self._phase == MissionPhase.SIGN_BRANCH
                and self._sign_dir == SignDirection.NONE
                and obs.sign_direction != SignDirection.NONE):
            self._sign_dir = SignDirection(int(obs.sign_direction))

    # --- 명령/지시 생성 --------------------------------------------------

    def _command(self, obs: MissionObservation, dt: float) -> DriveCommand:
        """@brief 현재 페이즈에 맞는 DriveCommand(제어 게이트 + 인지 지시)."""
        p = self._phase
        roi = _ROI_MAP.get(p, RoiMode.FULL)
        follow = LaneColor.WHITE  # 항상 흰 차선 추종(노랑 폐기).

        # 정지 게이트: 출발 대기 / 종료 정지 / 장애물(아루코).
        if p in (MissionPhase.WAIT_START_SIGNAL, MissionPhase.FINISH_STOP):
            return self._stop_command(follow, roi)
        if p == MissionPhase.OBSTACLE_ZONE and obs.aruco_present:
            return self._stop_command(follow, roi)  # obstacle_stop.

        # 주행: 아래층 반응형 + 미션 지시 오버레이.
        cmd = self.decision.update(obs.lane, dt)
        cmd.follow_color = follow
        cmd.roi_mode = roi
        cmd.turn_hint = TurnHint.NONE

        if p == MissionPhase.SIGN_BRANCH:
            # 흰선 추종은 유지하고 팻말 지시쪽으로 조향 bias만 얹는다 + 감속.
            cmd.speed_scale = min(cmd.speed_scale, self.cfg.sign_branch_speed_scale)
            cmd.steer_bias = self._sign_steer_bias()
        elif p == MissionPhase.OBSTACLE_ZONE:
            cmd.speed_scale = min(cmd.speed_scale, self.cfg.slow_speed_scale)
        return cmd

    def _sign_steer_bias(self) -> float:
        """@brief 래치된 팻말 방향 → 조향 bias(트림 전 raw: +=좌, -=우). NONE=0(직진)."""
        if self._sign_dir == SignDirection.LEFT:
            return +float(self.cfg.steer_bias_value)
        if self._sign_dir == SignDirection.RIGHT:
            return -float(self.cfg.steer_bias_value)
        return 0.0

    def _stop_command(self, follow: LaneColor, roi: RoiMode) -> DriveCommand:
        """@brief 정지 게이트 명령(현재 지시 색/ROI 유지)."""
        return DriveCommand(
            state=DriveState.STOP, go=False, speed_scale=0.0,
            lookahead_scale=1.0, steer_limit=1.0,
            follow_color=follow, turn_hint=TurnHint.NONE, roi_mode=roi)
