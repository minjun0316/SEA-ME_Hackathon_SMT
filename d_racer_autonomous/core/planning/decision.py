"""@file decision.py
@brief 판단(Decision) 상태기계 — ROS-free 순수 로직.

@details
인지가 준 차선 상태(LaneStatus 상당)를 보고 "지금 어떻게 달릴지"를 정한다.
출력은 계약(docs/interfaces.md §4.4)의 DriveCommand 값 — 상태·정지게이트·
속도배율 등 **배율/게이트만** 정하고 제어기 튜닝값은 건드리지 않는다.

@par 상태
INIT → (첫 유효검출) → DRIVE ⇄ SLOW, 어디서든 정지선/정지요청 → STOP,
유효검출 소실 grace 초과 → LOST(정지, 조향 유지). LOST/INIT는 유효검출이
recover_grace 동안 이어지면 주행상태로 복귀.

@par 설계 원칙
- 안전 우선: 입력 불량/소실이면 fail-safe 정지가 기본.
- ROS 의존성 없음: (LaneObservation, dt) → DriveCommand. 단위 테스트 가능.
- 짧은 신호 끊김(<lost_grace)은 직전 상태를 유지해 떨림을 막는다.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from ..config_schema import DecisionConfig


class DriveState(IntEnum):
    """@brief DriveCommand.state 상수(계약 §4.4와 값 일치)."""

    INIT = 0    ##< 초기화/대기(첫 유효검출 전).
    DRIVE = 1   ##< 정상 주행.
    SLOW = 2    ##< 감속 주행(코너/저신뢰/정지선 접근).
    STOP = 3    ##< 정지(정지선/정지요청).
    LOST = 4    ##< 차선 소실 → 정지, 조향은 마지막값 유지.


class LaneColor(IntEnum):
    """@brief 추종할 차선 색(DriveCommand.follow_color). 인지(OpenCV)에 전달.

    @details 상위 미션 SM이 페이즈별로 대상선을 전환한다(흰↔노랑). 빨강 장애물
    구역은 '따라가는 선'이 아니라 바닥 구역이므로 여기 포함하지 않는다(구역 안에서도
    흰 차선 추종). 하위 반응형 SM은 기본값 WHITE만 낸다.
    """

    WHITE = 0    ##< 흰 차선 추종(일반 구간).
    YELLOW = 1   ##< 노랑 차선 추종(원형 지름길 M2).


class TurnHint(IntEnum):
    """@brief 갈림길 조향/ROI bias 힌트(DriveCommand.turn_hint = LaneMode.turn_bias).

    @details 원 지름길 갈림길은 양쪽이 같은 노랑·같은 곡률이라 카메라로 구분 불가.
    상위 미션 SM이 정지선 카운트에 따라 좌/우를 지정해 인지가 그쪽 갈래를 잡게 한다
    (인지는 이 bias로 target_x를 편향). 좌/우의 물리적 의미는 트랙(대회장) 확정.
    """

    NONE = 0    ##< 힌트 없음(직진/일반).
    LEFT = 1    ##< 좌측 갈래/출구로 bias.
    RIGHT = 2   ##< 우측 원형 차선으로 bias.


class RoiMode(IntEnum):
    """@brief 인지에 줄 ROI 지시(DriveCommand.roi_mode = LaneMode.roi_mode).

    @details 판단이 미션 상태에 따라 "어느 영역을 볼지"를 인지에 지시한다. 실제
    ROI 자르기·mask·target 추출은 인지(OpenCV) 몫이고, 판단은 모드만 낸다.
    """

    FULL = 0         ##< 전체 ROI(로터리 진입/추종, 외곽 복귀).
    LOWER = 1        ##< 하단 ROI(직선/접근 주행).
    RIGHT = 2        ##< 오른쪽 50~60% ROI(원형 계속, 오른쪽 갈래).
    LEFT = 3         ##< 왼쪽 50~60% ROI(출구 탈출/연결도로).
    LOWER_ARUCO = 4  ##< 하단 ROI 포함 + 아루코 검출 우선(장애물 구간).


@dataclass
class LaneObservation:
    """@brief 판단 입력 — racer_msgs/LaneStatus를 core 자료구조로 옮긴 것.

    @details ROS 노드가 LaneStatus(+선택적 외부 정지요청)를 이 객체로 채워
    DecisionMaker.update()에 넘긴다. 필드 의미는 계약 §4.3과 동일.
    """

    lane_detected: bool = False   ##< 유효 차선 검출 여부.
    confidence: float = 0.0       ##< 0.0~1.0 검출 신뢰도.
    num_points: int = 0           ##< lane_path 점 개수(0이면 미검출).
    lateral_offset: float = 0.0   ##< 차선중심 횡오차 [m], +좌측.
    heading_error: float = 0.0    ##< 전방 대비 차선 접선 오차 [rad].
    stop_line: bool = False       ##< 정지선 검출 여부.
    stop_line_dist: float = -1.0  ##< 정지선까지 거리 [m](미검출 -1.0).
    stop_request: bool = False    ##< 외부 정지요청(e-stop/미션). true면 무조건 STOP.


@dataclass
class DriveCommand:
    """@brief 판단 출력 — racer_msgs/DriveCommand에 그대로 매핑.

    @var state           DriveState(디버그/로깅용).
    @var go              false면 제어가 throttle=0(정지). 조향은 유지.
    @var speed_scale     v_max에 곱하는 배율 [0.0~1.0].
    @var lookahead_scale lookahead 배율(기본 1.0).
    @var steer_limit     정규화 조향 상한 [0.0~1.0](기본 1.0).
    @var follow_color    추종할 차선 색(인지에 전달). 하위 SM은 기본 WHITE.
    @var turn_hint       갈림길 조향/ROI bias 힌트. 하위 SM은 기본 NONE.
    @var roi_mode        인지 ROI 지시. 하위 SM은 기본 FULL.
    @var yolo_enable     인지 신호등 YOLO 추론 게이트(True=ON). 하위 반응형 SM은 기본 True.
                         위층 미션 SM만 페이즈에 따라 끄고(주행중) 켠다(출발/빨간불 종료).
    @var sign_enable     인지 팻말 YOLO(별도 모델) 게이트(True=ON). SIGN_BRANCH에서만 True.
    @var steer_bias      제어 조향 offset(트림 전 raw[-1,1], +=좌/-=우). 팻말 분기서만 ≠0.

    @note follow_color/turn_hint/roi_mode/yolo_enable/sign_enable 는 제어가 아니라 **인지 지시**,
    steer_bias 는 **제어 지시**다. ROS 발행 시 DriveCommand(제어)와 LaneMode(인지)로 나눠 실어 보낸다.
    """

    state: DriveState = DriveState.INIT
    go: bool = False
    speed_scale: float = 0.0
    lookahead_scale: float = 1.0
    steer_limit: float = 1.0
    follow_color: LaneColor = LaneColor.WHITE
    turn_hint: TurnHint = TurnHint.NONE
    roi_mode: RoiMode = RoiMode.FULL
    yolo_enable: bool = True
    sign_enable: bool = False
    steer_bias: float = 0.0


class DecisionMaker:
    """@brief 차선상태 → DriveCommand 상태기계(순수 로직).

    @details 매 주기 update(obs, dt)를 호출한다. 내부에 소실/복귀/정지선 dwell
    타이머를 두어 채터링과 데드락을 방지한다. 상태는 명시적이고 reset() 제공.
    """

    def __init__(self, config: DecisionConfig | None = None):
        """@param config 판단 임계값/배율. None이면 기본값."""
        self.cfg = config or DecisionConfig()
        self.reset()

    def reset(self) -> None:
        """@brief 상태와 모든 타이머를 초기화한다."""
        self._state = DriveState.INIT
        self._invalid_time = 0.0    ##< 미검출 지속시간[s].
        self._valid_time = 0.0      ##< 유효검출 지속시간[s].
        self._stop_time = 0.0       ##< STOP 진입 후 경과시간[s](dwell용).
        self._stopline_done = False  ##< 현재 정지선에 대해 dwell 완료(재출발 래치).

    @property
    def state(self) -> DriveState:
        """@brief 현재 상태."""
        return self._state

    def _is_valid(self, obs: LaneObservation) -> bool:
        """@brief 유효 차선 검출인지 판정(점 개수·신뢰도·검출플래그)."""
        return (obs.lane_detected
                and obs.num_points >= self.cfg.min_points
                and obs.confidence >= self.cfg.conf_min)

    def update(self, obs: LaneObservation, dt: float) -> DriveCommand:
        """@brief 한 주기 판단을 수행하고 DriveCommand를 반환한다.

        @param obs 이번 주기 차선상태.
        @param dt  직전 호출 이후 경과시간 [s](>=0).
        @return 이번 주기 DriveCommand.

        @details 우선순위: 외부 정지요청 > 소실(LOST) > 정지선(STOP/접근SLOW)
        > 신뢰도/자세 기반(DRIVE/SLOW). INIT/LOST는 유효검출이 recover_grace
        동안 이어져야 주행으로 복귀한다.
        """
        dt = max(0.0, dt)
        valid = self._is_valid(obs)
        if valid:
            self._valid_time += dt
            self._invalid_time = 0.0
        else:
            self._invalid_time += dt
            self._valid_time = 0.0

        self._state = self._next_state(obs, valid)

        # STOP dwell: 정지선으로 인한 STOP만 시간을 누적한다(정지요청은 계속 유지).
        # dwell 초과 시 이 정지선에 대해 재출발 래치를 세워 데드락을 막는다.
        if self._state == DriveState.STOP and not obs.stop_request:
            self._stop_time += dt
            if self._stop_time >= self.cfg.stop_dwell:
                self._stopline_done = True
        else:
            self._stop_time = 0.0

        return _STATE_COMMAND[self._state](self.cfg)

    def _next_state(self, obs: LaneObservation, valid: bool) -> DriveState:
        """@brief 전이 규칙. 현재 상태와 관측으로 다음 상태를 정한다."""
        cfg = self.cfg

        # 1) 외부 정지요청은 최우선.
        if obs.stop_request:
            return DriveState.STOP

        # 2) 유효검출 소실 처리.
        if not valid:
            # INIT에선 아직 한 번도 못 잡았으므로 대기 유지.
            if self._state == DriveState.INIT:
                return DriveState.INIT
            # 짧은 끊김(grace 이내)은 직전 상태 유지, 초과 시 LOST.
            if self._invalid_time >= cfg.lost_grace:
                return DriveState.LOST
            return self._state

        # 3) 여기부터 유효검출. INIT/LOST는 복귀 유예 후 주행 진입.
        if self._state in (DriveState.INIT, DriveState.LOST):
            if self._valid_time < cfg.recover_grace:
                return self._state

        # 4) 정지선: dwell 래치가 안 걸린 경우에만 정지/접근감속.
        #    커브 게이트: 헤딩오차가 크면(=커브 중) 정지선 신호를 무시한다. 커브서
        #    가로로 눕는 차선을 정지선으로 오인하는 것을 차단(검출단 폭 하한과 겹치는
        #    안전망). 진짜 정지선은 직진 접근이라 heading_error가 작아 게이트가 열려 있다.
        in_curve = (cfg.stopline_heading_gate > 0.0
                    and abs(obs.heading_error) >= cfg.stopline_heading_gate)
        if (obs.stop_line and obs.stop_line_dist >= 0.0
                and not self._stopline_done and not in_curve):
            if obs.stop_line_dist <= cfg.stop_trigger_dist:
                return DriveState.STOP
            if obs.stop_line_dist <= cfg.stop_approach_dist:
                return DriveState.SLOW

        # 정지선이 사라지면 dwell 래치 해제(다음 정지선을 다시 인식).
        if not obs.stop_line:
            self._stopline_done = False

        # 5) 신뢰도/자세 기반 DRIVE vs SLOW.
        if (obs.confidence < cfg.conf_drive
                or abs(obs.heading_error) >= cfg.heading_slow
                or abs(obs.lateral_offset) >= cfg.offset_slow):
            return DriveState.SLOW
        return DriveState.DRIVE


# 상태별 출력 생성기(설정 의존). 순수 함수라 테스트·확장 용이.
_STATE_COMMAND = {
    DriveState.INIT: lambda c: DriveCommand(DriveState.INIT, False, 0.0, 1.0, 1.0),
    DriveState.DRIVE: lambda c: DriveCommand(
        DriveState.DRIVE, True, c.drive_speed_scale, 1.0, 1.0),
    DriveState.SLOW: lambda c: DriveCommand(
        DriveState.SLOW, True, c.slow_speed_scale, c.slow_lookahead_scale, 1.0),
    DriveState.STOP: lambda c: DriveCommand(DriveState.STOP, False, 0.0, 1.0, 1.0),
    DriveState.LOST: lambda c: DriveCommand(DriveState.LOST, False, 0.0, 1.0, 1.0),
}
