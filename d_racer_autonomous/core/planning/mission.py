"""@file mission.py
@brief 상위 미션 시퀀스(MissionSequencer) — ROS-free 순수 로직.

@details
2계층 판단의 **위층**. 흰선 폐루프 코스(07-14 재설계)를 6-state 순차 상태기계로 돈다:

  WAIT_START_SIGNAL → LANE_FOLLOW → SIGN_BRANCH → LANE_FOLLOW → OBSTACLE_ZONE
                    → FINISH_WATCH ⇄ FINISH_STOP
                                   ↖ 빨강 안 보인 지 finish_stop_release_sec 경과 시 복귀

각 상태에서 제어 게이트(go/speed_scale/steer_bias)와 인지 지시(roi_mode/yolo_enable/
sign_enable)를 내고, 눈앞 차선에 대한 즉각 반응(DRIVE/SLOW/LOST)은 아래층 DecisionMaker에
위임한다. 지름길(노랑 차선추종)·로터리·체커보드 도착은 **폐기**(07-14).

@par 코스 시퀀스
1. 출발점에서 신호등 **초록불** 대기(신호등 YOLO ON) → 초록 확정 시 출발(YOLO OFF).
2. S자 흰선 코스를 **양쪽 흰 차선** 슬라이딩윈도우로 추종(LANE_FOLLOW).
3. S자 끝 **방향 팻말**: 팻말 YOLO가 좌/우를 신뢰도 있게 읽으면(sign_direction≠NONE,
   인지의 sign_min_conf=0.8 통과분) SIGN_BRANCH 진입 → 방향 래치 → 인지에 turn_hint로
   지시해 **그쪽 차선 하나만 앵커**로 추종(lane.yaml sign_apply=anchor) + 감속 →
   고정시간(sign_branch_duration) 경과 시 LANE_FOLLOW 복귀(팻말 1회성 래치).
4. 다시 흰선 기본 추종(LANE_FOLLOW).
5. 동적 장애물 **아루코 마커**: 보이면 정지, 사라지면 재출발(신호등 YOLO 재점화).
   단 `obstacle_min_dwell_sec`를 못 채우고 사라지면 오검출로 보고 LANE_FOLLOW 복귀
   (FINISH_WATCH가 편도라 오검출 1프레임이 코스를 끝내던 것을 막음 — 07-15).
6. 재출발 후 **빨간불** 감시(FINISH_WATCH) → 빨간불 보이면 정지(FINISH_STOP=종료).
   FINISH_STOP은 **편도가 아니다**(07-15c): 빨강을 마지막으로 본 지
   `finish_stop_release_sec`(기본 10s)가 지나면 FINISH_WATCH로 복귀해 주행을 재개한다.
   심판이 빨강을 계속 들고 있으면 타이머가 리셋돼 영구 정지(정상 종료)이고, 오검출로
   멈춘 경우엔 10초 뒤 스스로 풀려 미션을 이어간다 — 대회 중엔 동글이 없어 페이즈를
   손으로 되돌릴 수 없으므로, 편도 종착 상태는 오검출 하나에 미션 전체를 잃는다.
   이 값은 동시에 **최소 정지시간**이라 '정지했음'의 증명이 된다(재출발 패널티 없음).

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
    sign_detected: bool = False          ##< (미사용) 팻말색 OpenCV 트리거. 07-15 밤 진입 트리거를 YOLO로 옮겨 소비하지 않는다 — 인지는 계속 채우므로 되돌리기용으로 필드만 유지.
    sign_direction: SignDirection = SignDirection.NONE  ##< 방향 팻말 좌/우(팻말 YOLO, conf≥sign_min_conf): SIGN_BRANCH 진입 트리거 + 앵커 차선 방향.


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
        """@brief 페이즈·타이머·래치·아래층을 모두 초기화한다.

        @details traffic_light_start_enable=False면 WAIT_START_SIGNAL을 건너뛰고
        LANE_FOLLOW에서 시작한다(초록불 없이 즉시 출발 — 팻말/차선 단독 검증용 테스트
        스위치). 대회 주행은 True.
        """
        self._phase = (MissionPhase.WAIT_START_SIGNAL
                       if self.cfg.traffic_light_start_enable
                       else MissionPhase.LANE_FOLLOW)
        self._phase_time = 0.0        ##< 현재 페이즈 지속시간[s].
        self._red_quiet_time = 0.0    ##< 빨강을 마지막으로 본 뒤 흐른 시간[s](FINISH_STOP 해제용).
        self._sign_quiet_time = 0.0   ##< 팻말을 마지막으로 본 뒤 흐른 시간[s](SIGN_BRANCH 해제용).
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
        # 빨강 무관측 시간. 페이즈와 무관하게 누적하고 빨강을 보면 0으로 되돌린다 —
        # FINISH_STOP 진입은 그 틱에 빨강을 봤다는 뜻이라 진입 시점에 항상 0에서 시작한다
        # (= finish_stop_release_sec가 그대로 최소 정지시간이 된다). _advance_phase 前에
        # 갱신해야 진입 틱의 관측이 반영된다.
        if obs.traffic_light == TrafficLight.RED:
            self._red_quiet_time = 0.0
        else:
            self._red_quiet_time += dt
        # 팻말 무관측 시간(빨강과 같은 관례). 접근 중엔 계속 보이므로 0에 머물고, 팻말이
        # 시야를 벗어나는 순간(= 거의 다 왔다)부터 쌓인다 → SIGN_BRANCH 종료 판정에 쓴다.
        # _advance_phase 前에 갱신해야 진입 틱의 관측이 반영된다.
        if obs.sign_direction != SignDirection.NONE:
            self._sign_quiet_time = 0.0
        else:
            self._sign_quiet_time += dt
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
        """@brief 방향 팻말 YOLO 게이트 — LANE_FOLLOW(팻말 탐색) + SIGN_BRANCH(방향 유지).

        @details ⚠ 07-15 밤: SIGN_BRANCH 전용이면 **논리 순환**이다 — 진입 트리거가 이제
        팻말 YOLO의 sign_direction인데(색 트리거 폐기), 그 YOLO가 SIGN_BRANCH에서만 켜지면
        영영 진입하지 못한다. 팻말을 아직 안 지났다면(_sign_done=False) LANE_FOLLOW에서도
        켜 둬야 한다. 팻말 분기를 마치면 다시 OFF로 CPU/FPS를 되찾는다.
        신호등 게이트(yolo_enable)와 독립. @return True=팻말 YOLO ON.
        """
        if self._phase == MissionPhase.SIGN_BRANCH:
            return True
        return self._phase == MissionPhase.LANE_FOLLOW and not self._sign_done

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
            # 아루코 = 정지/종료 임박 트리거(최우선). 없으면 팻말 YOLO 방향으로 분기(1회만).
            # 07-15 밤: 진입 트리거를 OpenCV 팻말색 → **YOLO 방향 자체**로 교체. 색은 대회
            # 팻말 실측색을 못 넣어 잠정값(파랑)이었고, 방향을 이미 신뢰도 있게 읽는 마당에
            # 색이 진입을 막는 단일 실패점이었다. 인지가 conf<sign_min_conf를 NONE으로
            # 걸러주므로 '≠NONE'이 곧 '확실한 팻말'이다.
            if obs.aruco_present:
                self._set_phase(MissionPhase.OBSTACLE_ZONE)
            elif not self._sign_done and obs.sign_direction != SignDirection.NONE:
                self._set_phase(MissionPhase.SIGN_BRANCH)
                # 진입시킨 그 판독이 곧 방향 — 즉시 래치(_latch_sign_direction도 멱등).
                self._sign_dir = SignDirection(int(obs.sign_direction))

        elif p == MissionPhase.SIGN_BRANCH:
            # 분기 중에도 아루코 보이면 정지 우선. 아니면 팻말을 지났거나(무관측 지속)
            # 상한 시간 경과 시 복귀(1회성 래치).
            # [07-16] 종료 조건에 '팻말 무관측'을 추가했다. 고정시간(5.0)만 쓰면 팻말을
            # 지난 뒤에도 offset 9cm를 계속 물고 달려 차선(폭 36.6cm)을 이탈했다(실차).
            # 팻말이 시야에서 사라짐 = 거의 다 왔다는 물리적 사건이라 속도·배터리에 안 흔들린다.
            # 무관측 직후 바로 풀면 아직 팻말 옆이라 들이받으므로 sign_lost_release_sec만큼
            # 더 물고 간다. 고정시간은 이제 **상한(폴백)**으로만 남는다.
            if obs.aruco_present:
                self._sign_done = True
                self._set_phase(MissionPhase.OBSTACLE_ZONE)
            elif (self.cfg.sign_lost_release_sec > 0.0
                  and self._sign_quiet_time >= self.cfg.sign_lost_release_sec):
                self._sign_done = True
                self._set_phase(MissionPhase.LANE_FOLLOW)
            elif self._phase_time >= self.cfg.sign_branch_duration:
                self._sign_done = True
                self._set_phase(MissionPhase.LANE_FOLLOW)

        elif p == MissionPhase.OBSTACLE_ZONE:
            # 마커가 사라졌다 = 재출발. 단 FINISH_WATCH는 **편도**라(LANE_FOLLOW로 돌아올
            # 길이 없어 팻말 분기를 영영 스킵하고 첫 빨간불에 코스가 끝난다) 아루코
            # 오검출 한 프레임이 코스를 통째로 날릴 수 있었다. 최소 체류시간을 못 채웠으면
            # 오검출로 보고 LANE_FOLLOW로 되돌린다 — 인지의 aruco_hold_sec가 present를
            # 늘려주므로, 여기 도달한 _phase_time은 (실제 검출시간 + hold)에 해당한다.
            if not obs.aruco_present:
                if self._phase_time >= self.cfg.obstacle_min_dwell_sec:
                    self._set_phase(MissionPhase.FINISH_WATCH)
                else:
                    # 오검출로 판정 → 진입 前 상태를 그대로 복원(신호등 YOLO 재점화도 취소해
                    # 주행 구간 FPS를 되찾는다). 진짜 마커면 다시 잡혀 재진입한다.
                    self._yolo_relatch = False
                    self._set_phase(MissionPhase.LANE_FOLLOW)

        elif p == MissionPhase.FINISH_WATCH:
            # 재출발 후 흰선 주행하며 빨간불 감시. 빨간불 = 종료.
            if obs.traffic_light == TrafficLight.RED:
                self._set_phase(MissionPhase.FINISH_STOP)

        elif p == MissionPhase.FINISH_STOP:
            # [07-15c] 편도 아님. 빨강이 finish_stop_release_sec 동안 안 보이면 주행 재개.
            # 빨강이 사라졌다 = 오검출이었거나(→ 미션 계속) 정지 후 화면을 벗어났거나
            # (→ 이미 증명 끝남) 둘 중 하나라 어느 쪽이든 복귀가 안전하다. 심판이 계속
            # 들고 있는 진짜 빨강은 타이머가 0으로 리셋되어 여기 도달하지 못한다.
            # 래치가 아니라 몇 번이든 복귀 가능 — 오검출이 여러 번 나도 미션이 이어진다.
            if (self.cfg.finish_stop_release_sec > 0.0
                    and self._red_quiet_time >= self.cfg.finish_stop_release_sec):
                self._set_phase(MissionPhase.FINISH_WATCH)

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
        cmd.turn_hint = self._sign_turn_hint()

        if p == MissionPhase.SIGN_BRANCH:
            # 07-15 밤: 고정 조향 bias 폐기 → **인지에 앵커 차선을 지시**(turn_hint)하고
            # 제어는 평소 차선추종 그대로 둔다. 오픈루프가 아니라 폐루프라 속도·배터리·
            # 노면에 안 흔들린다. 감속만 여기서 얹는다.
            cmd.speed_scale = min(cmd.speed_scale, self.cfg.sign_branch_speed_scale)
        elif p == MissionPhase.OBSTACLE_ZONE:
            cmd.speed_scale = min(cmd.speed_scale, self.cfg.slow_speed_scale)
        return cmd

    def _sign_turn_hint(self) -> TurnHint:
        """@brief SIGN_BRANCH 중 래치된 팻말 방향 → 인지 앵커 차선 지시. 그 외 NONE.

        @details 노드가 LaneMode.turn_bias로 실어 인지(lane_detect)에 넘기면, 인지는
        그쪽 차선 하나만 슬라이딩윈도우로 잡고 차선폭/2를 더해 중심선을 만든다
        (lane.yaml sign_apply=anchor). 커브에선 인지의 커브 게이트가 적용을 보류하고
        직선 분기에 닿아야 발동한다 = 판독(커브 접근로)과 발동(직선 분기)의 분리.
        """
        if self._phase != MissionPhase.SIGN_BRANCH:
            return TurnHint.NONE
        if self._sign_dir == SignDirection.LEFT:
            return TurnHint.LEFT
        if self._sign_dir == SignDirection.RIGHT:
            return TurnHint.RIGHT
        return TurnHint.NONE

    def _stop_command(self, follow: LaneColor, roi: RoiMode) -> DriveCommand:
        """@brief 정지 게이트 명령(현재 지시 색/ROI 유지)."""
        return DriveCommand(
            state=DriveState.STOP, go=False, speed_scale=0.0,
            lookahead_scale=1.0, steer_limit=1.0,
            follow_color=follow, turn_hint=TurnHint.NONE, roi_mode=roi)
