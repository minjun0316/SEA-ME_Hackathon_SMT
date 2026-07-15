"""@file decision_node.py
@brief 판단(Decision) ROS2 노드 — ROS-free 코어 DecisionMaker의 얇은 래퍼.

@details
`core.planning.DecisionMaker`(반응형 안전 상태기계)를 그대로 재사용해
`/perception/lane_status`(racer_msgs/LaneStatus)를 보고 `/decision/drive_command`
(racer_msgs/DriveCommand)를 발행한다. 판단 로직은 core에 있고 이 노드는 토픽
구독/변환/발행만 하는 얇은 래퍼다(시뮬↔실차 로직 일원화). 계약: docs/interfaces.md §4.3/4.4.

@par 범위 (이번 단계)
아래층 **반응형 SM만** 래핑한다(DRIVE/SLOW/STOP/LOST). 위층 미션 시퀀스
(MissionSequencer, M0~M6)는 인지로 가는 신호 경로(follow_color/turn_hint 등)가
계약에 아직 없어(현재 그래프 decision→controller뿐) 확장 설계 후 얹는다.
→ 지금은 폐루프 배관(인지→판단→제어)을 먼저 완성하는 것이 목표.

@par 안전 (fail-safe)
- 타이머(rate_hz)마다 최신 lane_status로 판단·발행한다.
- **watchdog**: 마지막 lane_status 이후 `lane_timeout`(기본 0.3s)이 지나면 유효
  차선이 없다고 보고(lane_detected=false 취급) → DecisionMaker가 grace 후 LOST →
  go=false(정지). 인지가 끊겨도 달리지 않는다.
- 아직 시작 전(lane_status 미수신) → INIT → go=false. 실행만으로 구동 안 함.
- `stop_request`(e-stop/미션 정지)는 아직 발행원이 없어 항상 False. 후속 확장.

@par 발행 계약
- state 값은 core.DriveState와 계약 §4.4 STATE_* 상수가 일치(그대로 매핑).
- go=false면 제어가 throttle=0(조향 유지). speed_scale/lookahead_scale/steer_limit는
  제어기 튜닝을 안 건드리는 "배율/게이트"(계약 원칙).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path as _FsPath


# --------------------------------------------------------------------------- #
# core 패키지 경로 해석: controller_node.py 와 동일 방식(하드코딩/복사 없이 단일
# 소스 core/ 를 import). 소스 실행과 colcon install 실행 두 위치 모두 지원.
# --------------------------------------------------------------------------- #
def _find_core_root() -> str:
    """@brief `core`를 import 할 수 있는 프로젝트 루트(d_racer_autonomous)를 찾는다.

    @details 우선순위: 환경변수 `D_RACER_ROOT` → __file__ 상위 디렉토리 탐색.
    @return `core/`와 `config/`를 담은 디렉토리 절대경로.
    @throws RuntimeError 어디서도 찾지 못하면 명확히 실패시킨다.
    """
    env_root = os.environ.get('D_RACER_ROOT')
    candidates = []
    if env_root:
        candidates.append(_FsPath(os.path.expanduser(env_root)))

    here = _FsPath(__file__).resolve()
    for base in [here, *here.parents]:
        candidates.append(base)                      # 소스 위치: base=d_racer_autonomous.
        candidates.append(base / 'd_racer_autonomous')  # install 위치.

    for cand in candidates:
        if (cand / 'core' / '__init__.py').exists() and \
           (cand / 'config' / 'vehicle.yaml').exists():
            return str(cand)

    raise RuntimeError(
        'decision_node: core 패키지를 찾지 못했습니다. '
        '환경변수 D_RACER_ROOT 로 d_racer_autonomous 경로를 지정하세요 '
        '(예: export D_RACER_ROOT=~/SEA-ME_Hackathon_SMT/d_racer_autonomous).'
    )


_CORE_ROOT = _find_core_root()
if _CORE_ROOT not in sys.path:
    sys.path.insert(0, _CORE_ROOT)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from racer_msgs.msg import DriveCommand, LaneStatus, MissionCues, LaneMode  # noqa: E402

from core.config_schema import load_config  # noqa: E402
from core.planning import DecisionMaker, DriveState, LaneObservation  # noqa: E402


class DecisionNode(Node):
    """@brief core DecisionMaker를 감싸는 판단 명령 발행 노드."""

    def __init__(self):
        super().__init__('decision_node')

        # --- 파라미터 (모두 CLI/YAML로 변경 가능, 하드코딩 없음) ---
        self.declare_parameter('lane_status_topic', '/perception/lane_status')
        self.declare_parameter('drive_command_topic', '/decision/drive_command')
        self.declare_parameter('rate_hz', 10.0)
        # 이 시간 넘게 lane_status가 안 오면 유효검출 없음으로 취급(→ LOST, fail-safe).
        self.declare_parameter('lane_timeout', 0.3)
        # core 설정 YAML (비우면 core_root/config/decision.yaml 사용).
        self.declare_parameter('config_files', [''])
        # 미션신호(아루코) 정지 게이트: aruco_present면 stop_request → 최우선 STOP.
        # "언제든 마커 보이면 정지, 사라지면 복귀"(반응층). 미션 M4 페이즈 한정 아님.
        self.declare_parameter('mission_cues_topic', '/perception/mission_cues')
        self.declare_parameter('aruco_stop_enable', True)
        # 이 시간 넘게 mission_cues가 안 오면 아루코 없음으로 취급(주행 유지).
        # 정지신호는 신선할 때만 유효 — 프레임 끊김/노드 죽음에 붙들려 정지 안 함.
        self.declare_parameter('mission_cues_timeout', 0.5)
        # 방향 팻말 → 차선 선택(07-14): mission_cues.sign_direction(YOLO 좌/우)을 래치해
        # /decision/lane_mode.turn_bias(BIAS_LEFT/RIGHT)로 발행 → 인지(lane_detect)가 그
        # 방향 차선을 강제 anchor. 팻말이 잠깐 보이고 지나가므로 마지막 감지 후
        # sign_hold_sec 동안 유지(분기 통과) → 이후 BIAS_NONE(인지 adaptive 복귀). 반대
        # 팻말이 보이면 즉시 전환.
        self.declare_parameter('sign_lane_enable', True)
        self.declare_parameter('lane_mode_topic', '/decision/lane_mode')
        self.declare_parameter('sign_hold_sec', 2.5)
        # [07-15] 방향 디바운스: 실측상 통합 YOLO가 **같은 팻말에서 좌/우를 뒤집는다**
        # (LEFT conf0.93 ↔ RIGHT conf0.88, 심지어 한 팻말에 좌·우 박스 동시 검출 boxes=2).
        # 방향이 뒤집히면 인지의 offset도 좌↔우로 뒤집혀 상쇄 → 결국 가운데 직진 → 팻말 충돌.
        # → 같은 방향이 이만큼 '연속'으로 나와야 확정한다(짧은 뒤집힘 무시).
        # ⚠ 근본 해결은 모델 재학습(좌/우 혼동). 이건 완화책이다.
        # [07-15] 3 → 1: 인지(mission_cues)가 '접근 구간 신뢰도 누적 투표'로 이미 안정화해
        # 발행하므로, 여기서 또 연속 N을 요구하면 지연만 늘고 중복이다. 투표를 끄면(
        # sign_vote_enable=false) 3으로 되돌릴 것.
        self.declare_parameter('sign_confirm_count', 1)
        # --- [07-15] 팻말 회피: 고정 조향(개루프) ---
        # 경로를 ±offset(9cm)으로 미는 방식은 lateral_pd가 부드러운 차선유지용이라 반응이
        # 약했다(9cm 스텝 → 조향 0.042). 대신 방향 확정 시 **정해진 시간만큼 정해진 조향**을
        # 정규화 조향에 그대로 얹는다(DriveCommand.steer_bias → 컨트롤러가 가산).
        # 정지선 기동(controller.yaml stopline_maneuver)과 같은 개루프 방식.
        # 부호 규약: steer_bias + = 좌, − = 우.
        # ⚠ 이걸 쓰면 경로 offset은 꺼야 한다(이중 적용 방지) → lane.yaml sign_lane_offset_m: 0.0
        self.declare_parameter('sign_steer_enable', True)
        self.declare_parameter('sign_steer_value', 0.2)   # 고정 조향 크기(정규화, 트림 전).
        self.declare_parameter('sign_steer_sec', 1.0)     # 유지 시간[s]. [07-15] 2.0→1.0.
        # [07-15] 기동 시점 = 팻말이 **사라지는 순간**(미검출 또는 거리게이트 탈락).
        #   보이자마자 꺾으면 아직 멀어서 일찍 꺾고, 먼 판독(작은 박스)이라 좌/우도 부정확하다.
        #   팻말이 시야에서 빠지는 순간 = ①가장 가까이서 본 판독(가장 크고 신뢰도 높음)이
        #   방금 확보됐고 ②회피해야 할 바로 그 지점이다 → 그때 마지막 방향으로 기동한다.
        #   sign_lost_sec: 이 시간 동안 팻말 판정이 없으면 '사라졌다'로 보고 발동(짧은
        #   검출 끊김에 조기 발동하지 않게 하는 디바운스). 발동이 늦으면 ↓, 깜빡임에 조기
        #   발동하면 ↑.
        self.declare_parameter('sign_lost_sec', 0.3)
        # 기동 후 팻말이 이 시간 이상 안 보여야 재무장(같은 팻말에 중복 기동 방지).
        self.declare_parameter('sign_rearm_sec', 3.0)
        # 신호등 출발 게이트(07-14): 초록불(MissionCues.TL_GREEN)을 볼 때까지 정지 대기.
        # 초록 '연속 확정'(tl_green_confirm_sec)은 인지(mission_cues_node)가 이미 처리하므로
        # 여기선 GREEN을 한 번 보면 출발로 **영구 래치**한다(주행 중 신호등이 시야에서
        # 사라지거나 빨강이 보여도 다시 안 멈춤 — 재출발 데드락 방지).
        # ⚠ YOLO가 초록을 못 잡으면 영영 출발 안 함 → 그땐 False로 끄고 주행.
        self.declare_parameter('traffic_light_start_enable', True)
        # [07-15] 빨간불 종료: 출발(초록) 후 한 바퀴 돌아와 빨간불을 보면 정지(=코스 종료).
        #  1) finish_grace_sec: 출발 후 이 시간 동안 빨강 무시.
        #     [07-15 실차] 10.0 → 0.0: "초반에 초록 본 뒤 빨강으로 다시 안 바뀐다"(실측).
        #     즉 출발선 신호등이 초록→빨강으로 바뀌어 즉시 멈추는 상황이 애초에 없다
        #     → 유예는 방어할 대상이 없고 빨간불 인식을 막기만 했다(테스트서 "빨강 아예
        #     안 먹음"의 원인).
        #     [07-15b] 0.0 → 15.0: 위 "유예 불필요" 판단은 **비용 계산이 빠져 있었다**.
        #     종료 신호등은 코스 **마지막**에 있고 최속 주파가 ~40s다. 즉 출발 후 15s 안에
        #     보이는 빨강은 정의상 진짜 종료 신호일 수 없다(= 전부 오검출).
        #     유예로 잃는 것: 없음(15s < 40s라 진짜 빨강은 유예 밖에서 온다).
        #     유예로 얻는 것: 출발 구간 빨강 오검출 → 즉시 영구종료(복구 불가)를 원천 차단.
        #     비용이 비대칭이라 유예를 넉넉히 두는 게 이득. 코스가 빨라지면 ↓.
        #     ⚠ 유예 중 빨강은 warning 로그로 알린다("빨강 아예 안 먹음" 오해 방지).
        #  2) red_confirm_count 연속일 때만 종료로 본다(오검출 방어).
        #     [07-15] 3 → 1("보자마자 바로 정지") → 3 복귀: 1은 빨강 오검출 한 프레임에
        #     코스가 그 자리서 끝나버려 실차에서 "너무 쉽게" 종료됐다.
        #     [07-15b] 3 → 1: 이 카운터는 **방어가 되지 못한다**(3이 안전해 보이는 건 착각).
        #     "3연속이면 ~0.5s@6Hz"라고 봤지만 틀렸다 — 이 콜백은 mission_cues 수신마다
        #     도는데 그 토픽은 추론(6Hz)이 아니라 **카메라 프레임마다(20Hz)** 발행된다.
        #     즉 추론 1회 결과가 20Hz로 재발행돼 3연속을 0.15s 만에 스스로 채운다
        #     (같은 추론을 3번 센 것이라 독립 증거가 아니다 = 오검출을 전혀 못 거른다).
        #     → 진짜 방어는 인지의 시간 확정(tl_red_confirm_sec)이 담당한다. 여기선 1로 두고
        #     이 카운터에 방어 역할을 기대하지 않는다(올려도 지연만 늘 뿐 안전해지지 않음).
        # 종료는 영구 래치(다시 출발 안 함).
        self.declare_parameter('traffic_light_finish_enable', True)
        self.declare_parameter('finish_grace_sec', 15.0)
        self.declare_parameter('red_confirm_count', 1)

        lane_topic = str(self.get_parameter('lane_status_topic').value)
        cmd_topic = str(self.get_parameter('drive_command_topic').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be greater than 0')
        self.rate_hz = rate_hz
        self.dt_nominal = 1.0 / rate_hz
        self.lane_timeout = float(self.get_parameter('lane_timeout').value)
        self.mission_cues_topic = str(self.get_parameter('mission_cues_topic').value)
        self.aruco_stop_enable = bool(self.get_parameter('aruco_stop_enable').value)
        self.mission_cues_timeout = float(self.get_parameter('mission_cues_timeout').value)
        self.sign_lane_enable = bool(self.get_parameter('sign_lane_enable').value)
        self.lane_mode_topic = str(self.get_parameter('lane_mode_topic').value)
        self.sign_hold_sec = float(self.get_parameter('sign_hold_sec').value)
        self.sign_confirm_count = int(self.get_parameter('sign_confirm_count').value)
        self.sign_steer_enable = bool(self.get_parameter('sign_steer_enable').value)
        self.sign_steer_value = float(self.get_parameter('sign_steer_value').value)
        self.sign_steer_sec = float(self.get_parameter('sign_steer_sec').value)
        self.sign_lost_sec = float(self.get_parameter('sign_lost_sec').value)
        self.sign_rearm_sec = float(self.get_parameter('sign_rearm_sec').value)
        self.tl_start_enable = bool(self.get_parameter('traffic_light_start_enable').value)
        self.tl_finish_enable = bool(self.get_parameter('traffic_light_finish_enable').value)
        self.finish_grace_sec = float(self.get_parameter('finish_grace_sec').value)
        self.red_confirm_count = int(self.get_parameter('red_confirm_count').value)

        # --- core 판단 로직 구성 ---
        config_files = self._resolve_config_files()
        self.config = load_config(*config_files)
        self.decision = DecisionMaker(self.config.decision)

        # --- 수신 상태 ---
        self._last_status = None        ##< 최신 LaneStatus(없으면 None).
        self._last_status_time = None   ##< 최신 수신 시각(rclpy.Time).
        self._last_tick_time = None     ##< 직전 타이머 tick 시각(dt 산출용).
        self._prev_log_state = None
        self._aruco_present = False     ##< 최신 mission_cues.aruco_present.
        self._aruco_time = None         ##< 최신 mission_cues 수신 시각(stale 판정).
        self._sign_dir = MissionCues.SIGN_NONE  ##< 래치된(확정) 팻말 방향(SIGN_LEFT/RIGHT/NONE).
        self._sign_time = None          ##< 마지막 확정 팻말 감지 시각(hold 판정).
        self._sign_vote = MissionCues.SIGN_NONE  ##< 디바운스 후보 방향.
        self._sign_votes = 0            ##< 후보가 연속으로 나온 횟수.
        # 고정 조향 기동 상태(개루프)
        self._steer_start = None        ##< 기동 시작 시각(None=기동 중 아님).
        self._steer_dir = MissionCues.SIGN_NONE  ##< 기동 방향(확정된 팻말 방향).
        self._steer_done = False        ##< 이번 팻말에 대해 기동 완료(재무장 전까지 재기동 금지).
        self._last_dir = MissionCues.SIGN_NONE  ##< 팻말이 보이던 마지막 순간의 방향(=가장 가까웠던 판독).
        self._was_visible = False       ##< 직전 tick의 팻말 가시성(사라지는 edge 검출용).
        self._sign_gone_since = None    ##< 팻말이 안 보이기 시작한 시각(재무장 판정).
        # 신호등 출발 래치: 초록 한 번 보면 True 고정.
        # 게이트가 꺼져 있으면 처음부터 출발 상태로 둔다(신호등 대기 없음).
        self._started = not self.tl_start_enable
        self._started_time = None   ##< 출발 시각(빨간불 유예 계산 기준). None=아직 출발 전.
        self._finished = False      ##< 빨간불 종료 래치(영구 정지).
        self._red_votes = 0         ##< 빨강 연속 검출 수(오검출 방어).

        self.get_logger().info(
            'decision_node 구성:\n'
            f'  core_root={_CORE_ROOT}\n'
            f'  config_files={config_files}\n'
            f'  구독 lane_status={lane_topic}\n'
            f'  발행 drive_command={cmd_topic} rate_hz={self.rate_hz}\n'
            f'  lane_timeout={self.lane_timeout}s (초과 시 LOST, fail-safe)\n'
            f'  신호등출발: enable={self.tl_start_enable} '
            f'({"초록불 볼 때까지 정지 대기(1회 래치)" if self.tl_start_enable else "게이트 OFF → 차선 잡히면 바로 출발"})\n'
            f'  빨간불종료: enable={self.tl_finish_enable} '
            f'(출발 {self.finish_grace_sec:.0f}s 후부터 감시, 빨강 {self.red_confirm_count}연속 → 영구정지)\n'
            f'  팻말차선: enable={self.sign_lane_enable} '
            f'발행={self.lane_mode_topic} hold={self.sign_hold_sec}s '
            '(sign_direction→turn_bias→인지 강제 anchor)\n'
            f'  아루코정지: enable={self.aruco_stop_enable} '
            f'구독={self.mission_cues_topic} timeout={self.mission_cues_timeout}s '
            '(aruco_present→stop_request→최우선 STOP, 사라지면 복귀)\n'
            '  범위: 아래층 반응형 SM + 아루코 정지 게이트.'
        )

        # 계약 §3: lane_status/drive_command 는 reliable, depth 1 (rclpy 기본 reliable).
        self.pub = self.create_publisher(DriveCommand, cmd_topic, 1)
        self.sub = self.create_subscription(
            LaneStatus, lane_topic, self._on_lane_status, 1)
        # 방향 팻말 → 차선 지시 발행(/decision/lane_mode.turn_bias). 인지가 소비.
        self.pub_lane_mode = None
        if self.sign_lane_enable:
            self.pub_lane_mode = self.create_publisher(LaneMode, self.lane_mode_topic, 1)
        # 미션신호(아루코 정지 게이트 + 팻말 방향) 구독 — 둘 중 하나라도 켜지면.
        self.sub_cues = None
        if self.aruco_stop_enable or self.sign_lane_enable:
            self.sub_cues = self.create_subscription(
                MissionCues, self.mission_cues_topic, self._on_mission_cues, 1)
        self.timer = self.create_timer(self.dt_nominal, self._on_timer)

        self._log_period = max(1, int(round(self.rate_hz)))
        self._tick = 0

    # ------------------------------------------------------------------ #
    def _resolve_config_files(self):
        """@brief 사용할 core 설정 YAML 경로 목록. 비면 config/decision.yaml."""
        raw = self.get_parameter('config_files').value
        given = [str(p) for p in (raw or []) if str(p).strip()]
        if given:
            return [os.path.expanduser(p) for p in given]
        return [os.path.join(_CORE_ROOT, 'config', 'decision.yaml')]

    def _on_lane_status(self, msg: LaneStatus):
        """@brief 최신 lane_status 저장(판단은 타이머에서 일괄 수행)."""
        self._last_status = msg
        self._last_status_time = self.get_clock().now()

    def _on_mission_cues(self, msg: MissionCues):
        """@brief 미션신호 저장. aruco_present=정지 게이트, sign_direction=차선 방향 래치."""
        self._aruco_present = bool(msg.aruco_present)
        self._aruco_time = self.get_clock().now()
        # 팻말 방향 래치(+디바운스). NONE은 무시(hold 유지) — 팻말이 잠깐 안 보여도
        # sign_hold_sec 동안 방향을 붙든다.
        # 디바운스: 모델이 같은 팻말에서 좌/우를 뒤집으므로(실측), 같은 방향이
        # sign_confirm_count 연속으로 나올 때만 확정한다. 뒤집힘 한두 프레임은 후보만
        # 리셋되고 확정 방향(_sign_dir)은 유지 → offset이 좌↔우로 상쇄되지 않는다.
        d = int(msg.sign_direction)
        if d in (MissionCues.SIGN_LEFT, MissionCues.SIGN_RIGHT):
            if d == self._sign_vote:
                self._sign_votes += 1
            else:
                self._sign_vote = d
                self._sign_votes = 1
            if self._sign_votes >= self.sign_confirm_count:
                if self._sign_dir != d:
                    self.get_logger().info(
                        f'팻말 방향 확정: '
                        f'{"LEFT" if d == MissionCues.SIGN_LEFT else "RIGHT"} '
                        f'({self._sign_votes}연속)')
                self._sign_dir = d
                self._sign_time = self.get_clock().now()
        now = self.get_clock().now()
        light = int(msg.traffic_light)

        # 신호등 출발: 초록(인지가 tl_green_confirm_sec 연속 확정해 발행)을 보면 영구 래치.
        if not self._started and light == MissionCues.TL_GREEN:
            self._started = True
            self._started_time = now
            self.get_logger().info(
                f'>>> 초록불 확인 → 출발! (빨간불 종료 감시는 '
                f'{self.finish_grace_sec:.0f}s 후부터)')

        # 빨간불 종료: 출발 후 유예시간이 지난 뒤, 빨강이 red_confirm_count 연속이면 종료.
        # 유예는 '출발 신호등이 아직 시야에 있는 동안 빨강으로 바뀌어 즉시 멈추는 것'을 막는다.
        elif (self._started and not self._finished and self.tl_finish_enable
                and self._started_time is not None):
            age = (now - self._started_time).nanoseconds * 1e-9
            if age >= self.finish_grace_sec:
                if light == MissionCues.TL_RED:
                    self._red_votes += 1
                    if self._red_votes >= self.red_confirm_count:
                        self._finished = True
                        self.get_logger().info(
                            f'>>> 빨간불 확인 ({self._red_votes}연속) → 코스 종료, 정지!')
                    else:
                        # 왜 아직 안 멈추는지 보이게(연속 카운트 부족).
                        self.get_logger().info(
                            f'빨강 감지 {self._red_votes}/{self.red_confirm_count}연속 '
                            f'(확정까지 대기)', throttle_duration_sec=0.5)
                else:
                    self._red_votes = 0   # 연속 끊기면 리셋(오검출 방어)
            elif light == MissionCues.TL_RED:
                # 유예 중 빨강 → 의도적으로 무시. 이게 안 보이면 "빨간불이 아예 안 먹는다"로
                # 오해하게 되므로 반드시 로그로 알린다(테스트 중 흔한 함정).
                self.get_logger().warning(
                    f'빨강 감지했으나 출발 후 {age:.1f}s < 유예 {self.finish_grace_sec:.0f}s '
                    f'→ 무시. 지금 테스트하려면 finish_grace_sec:=0 으로 실행.',
                    throttle_duration_sec=1.0)

    def _current_turn_bias(self, now) -> int:
        """@brief 래치된 팻말 방향을 turn_bias로 환산(hold 만료면 NONE=인지 adaptive 복귀).

        @note 고정조향(sign_steer_enable)을 쓰면 인지 경로 offset은 0으로 꺼두므로
              이 turn_bias는 실질 효과가 없다(모니터/호환용으로만 계속 발행).
        """
        if self._sign_dir == MissionCues.SIGN_NONE or self._sign_time is None:
            return LaneMode.BIAS_NONE
        age = (now - self._sign_time).nanoseconds * 1e-9
        if age > self.sign_hold_sec:
            self._sign_dir = MissionCues.SIGN_NONE   # 만료 → 래치 해제.
            return LaneMode.BIAS_NONE
        return (LaneMode.BIAS_LEFT if self._sign_dir == MissionCues.SIGN_LEFT
                else LaneMode.BIAS_RIGHT)

    def _current_steer_bias(self, now) -> float:
        """@brief 팻말 고정조향 기동 → steer_bias[-1,1]. 기동 중이 아니면 0.0.

        @details 방향이 확정되면 sign_steer_sec 동안 sign_steer_value를 그 방향으로
        그대로 얹는다(개루프). 경로 offset(9cm)은 lateral_pd 반응이 약해 늦었기에,
        조향을 직접 주어 즉각 회피한다. 기동은 팻말당 1회 — 팻말이 sign_rearm_sec 이상
        안 보여야 재무장한다(같은 팻말에 반복 기동 방지).
        부호: + = 좌, − = 우 (DriveCommand.steer_bias 규약).
        """
        if not self.sign_steer_enable:
            return 0.0

        # 팻말이 지금 보이는지(신선한 좌/우 판정). sign_lost_sec 안에 판정이 있으면 '보임'.
        # 거리게이트에 걸려도(먼 팻말) 인지가 SIGN_NONE을 내므로 여기선 '안 보임'과 동일하게
        # 취급된다 — 둘 다 '더는 못 읽는 상태'라 기동 트리거로서 같은 의미다.
        visible = (self._sign_dir != MissionCues.SIGN_NONE and self._sign_time is not None
                   and (now - self._sign_time).nanoseconds * 1e-9 <= self.sign_lost_sec)

        # 보이는 동안 마지막 방향을 계속 갱신 → 사라지는 순간의 값이 '가장 가까웠던 판독'.
        if visible:
            self._last_dir = self._sign_dir
            self._sign_gone_since = None
        else:
            if self._sign_gone_since is None:
                self._sign_gone_since = now
            # 재무장: 기동을 끝냈고 팻말이 충분히 오래 안 보이면 다음 팻말 준비.
            elif (self._steer_done and (now - self._sign_gone_since).nanoseconds * 1e-9
                    > self.sign_rearm_sec):
                self._steer_done = False
                self._steer_start = None
                self._steer_dir = MissionCues.SIGN_NONE
                self._last_dir = MissionCues.SIGN_NONE

        # 기동 중이면 시간 만료까지 같은 조향 유지(도중에 팻말이 뒤집혀도 방향 고정).
        if self._steer_start is not None:
            elapsed = (now - self._steer_start).nanoseconds * 1e-9
            if elapsed < self.sign_steer_sec:
                self._was_visible = visible
                return (self.sign_steer_value
                        if self._steer_dir == MissionCues.SIGN_LEFT
                        else -self.sign_steer_value)
            self._steer_start = None          # 기동 종료
            self._steer_done = True
            self.get_logger().info('팻말 고정조향 종료 → 차선추종 복귀')
            self._was_visible = visible
            return 0.0

        # 기동 시작 = 팻말이 **사라지는 순간**(보임→안보임 edge). 그 직전 판독이 가장
        # 가까운(=가장 크고 정확한) 값이고, 위치상으로도 지금이 회피할 지점이다.
        fired = 0.0
        if (self._was_visible and not visible and not self._steer_done
                and self._last_dir != MissionCues.SIGN_NONE):
            self._steer_start = now
            self._steer_dir = self._last_dir
            fired = (self.sign_steer_value if self._steer_dir == MissionCues.SIGN_LEFT
                     else -self.sign_steer_value)
            self.get_logger().info(
                f'>>> 팻말 사라짐(마지막 판독='
                f'{"LEFT" if self._steer_dir == MissionCues.SIGN_LEFT else "RIGHT"}) '
                f'→ 고정조향 {fired:+.2f} 를 {self.sign_steer_sec:.1f}s 적용')
        self._was_visible = visible
        return fired

    def _aruco_stop_active(self, now) -> bool:
        """@brief 아루코 정지신호가 유효한지(활성 + present + 신선)."""
        if not self.aruco_stop_enable or not self._aruco_present \
                or self._aruco_time is None:
            return False
        age = (now - self._aruco_time).nanoseconds * 1e-9
        return age <= self.mission_cues_timeout

    def _obs_from_status(self, msg: LaneStatus) -> LaneObservation:
        """@brief LaneStatus(ROS) → LaneObservation(core) 변환."""
        return LaneObservation(
            lane_detected=bool(msg.lane_detected),
            confidence=float(msg.confidence),
            num_points=int(msg.num_points),
            lateral_offset=float(msg.lateral_offset),
            heading_error=float(msg.heading_error),
            stop_line=bool(msg.stop_line),
            stop_line_dist=float(msg.stop_line_dist),
            stop_request=False,   # 아루코 등 외부 정지는 _on_timer에서 덮어씀.
        )

    def _on_timer(self):
        """@brief 매 주기: 최신(또는 stale) 관측으로 판단 → drive_command 발행."""
        now = self.get_clock().now()

        # 실제 경과 dt(코어 타이머용). 첫 tick은 nominal.
        if self._last_tick_time is None:
            dt = self.dt_nominal
        else:
            dt = (now - self._last_tick_time).nanoseconds * 1e-9
        self._last_tick_time = now

        # watchdog: lane_status가 오래 끊겼거나 아예 없으면 유효검출 없음으로.
        stale = (self._last_status is None or self._last_status_time is None or
                 (now - self._last_status_time).nanoseconds * 1e-9 > self.lane_timeout)
        if stale:
            obs = LaneObservation(lane_detected=False)  # 미검출 → grace 후 LOST.
        else:
            obs = self._obs_from_status(self._last_status)

        # 아루코 정지 게이트: 유효하면 stop_request=True(차선 상태와 무관하게 최우선 STOP).
        aruco_stop = self._aruco_stop_active(now)
        if aruco_stop:
            obs.stop_request = True

        # 신호등 출발 게이트: 초록 래치 전이면 무조건 정지 대기(차선이 보여도 안 나감).
        if not self._started:
            obs.stop_request = True
        # 빨간불 종료: 래치되면 영구 정지(다시 출발 없음).
        if self._finished:
            obs.stop_request = True

        cmd = self.decision.update(obs, dt)

        out = DriveCommand()
        out.header.stamp = now.to_msg()
        out.state = int(cmd.state)          # DriveState 값 = 계약 STATE_* 상수.
        out.go = bool(cmd.go)
        out.speed_scale = float(cmd.speed_scale)
        out.lookahead_scale = float(cmd.lookahead_scale)
        out.steer_limit = float(cmd.steer_limit)
        # 팻말 고정조향 기동: 확정 방향으로 sign_steer_sec 동안 조향을 그대로 얹는다.
        # 컨트롤러가 정규화 조향에 가산(steering + steer_bias).
        steer_bias = self._current_steer_bias(now)
        out.steer_bias = float(steer_bias)
        self.pub.publish(out)

        # 방향 팻말 → 차선 지시(turn_bias) 발행. 인지(lane_detect)가 강제 anchor에 소비.
        turn_bias = LaneMode.BIAS_NONE
        if self.pub_lane_mode is not None:
            turn_bias = self._current_turn_bias(now)
            lm = LaneMode()
            lm.header.stamp = now.to_msg()
            lm.turn_bias = int(turn_bias)
            # ⚠ [07-15 버그수정] LaneMode엔 YOLO 게이트 필드가 같이 실린다. ROS bool 기본값이
            # False라, turn_bias만 채워 발행하면 이 메시지가 mission_cues_node의
            # yolo_enable/sign_enable 게이트를 **False로 꺼버려** 신호등·팻말 검출이 통째로
            # 죽는다(초록·빨강 동시 미검출의 원인이었음). 이 노드는 YOLO 상시 on 전제이므로
            # 두 게이트를 명시적으로 True로 실어 보낸다.
            lm.yolo_enable = True
            lm.sign_enable = True
            self.pub_lane_mode.publish(lm)

        # 로그: 1초마다 또는 상태 전이 시.
        self._tick += 1
        changed = (self._prev_log_state != cmd.state)
        if self._tick % self._log_period == 0 or changed:
            self._prev_log_state = cmd.state
            self.get_logger().info(
                f'state={DriveState(cmd.state).name} go={cmd.go} '
                f'speed_scale={cmd.speed_scale:.2f} '
                f'lookahead_scale={cmd.lookahead_scale:.2f} '
                f'steer_limit={cmd.steer_limit:.2f} '
                f'{"[초록불 대기중]" if not self._started else ""}'
                f'{("[팻말조향 %+.2f]" % steer_bias) if steer_bias else ""}'
                f'{"[빨간불 종료]" if self._finished else ""}'
                f'{"[ARUCO-STOP]" if aruco_stop else ""}'
                f'{("[SIGN→" + {0:"NONE",1:"LEFT",2:"RIGHT"}.get(turn_bias,"?") + "]") if turn_bias else ""}'
                f'{"(stale→watchdog)" if stale else ""}'
            )

    def destroy_node(self):
        """@brief 종료 시 정지 명령(go=false, STOP) 한 번 발행 후 종료."""
        try:
            if hasattr(self, 'pub') and self.pub is not None:
                out = DriveCommand()
                out.header.stamp = self.get_clock().now().to_msg()
                out.state = int(DriveState.STOP)
                out.go = False
                out.speed_scale = 0.0
                out.lookahead_scale = 1.0
                out.steer_limit = 1.0
                self.pub.publish(out)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
