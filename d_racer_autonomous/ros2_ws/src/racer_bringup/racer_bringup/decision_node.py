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

from racer_msgs.msg import DriveCommand, LaneStatus, MissionCues  # noqa: E402

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

        lane_topic = str(self.get_parameter('lane_status_topic').value)
        cmd_topic = str(self.get_parameter('drive_command_topic').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be greater than 0')
        self.rate_hz = rate_hz
        self.dt_nominal = 1.0 / rate_hz
        self.lane_timeout = float(self.get_parameter('lane_timeout').value)

        # --- core 판단 로직 구성 ---
        config_files = self._resolve_config_files()
        self.config = load_config(*config_files)
        self.decision = DecisionMaker(self.config.decision)

        # --- 수신 상태 ---
        self._last_status = None        ##< 최신 LaneStatus(없으면 None).
        self._last_status_time = None   ##< 최신 수신 시각(rclpy.Time).
        self._last_tick_time = None     ##< 직전 타이머 tick 시각(dt 산출용).
        self._prev_log_state = None

        self.get_logger().info(
            'decision_node 구성:\n'
            f'  core_root={_CORE_ROOT}\n'
            f'  config_files={config_files}\n'
            f'  구독 lane_status={lane_topic}\n'
            f'  발행 drive_command={cmd_topic} rate_hz={self.rate_hz}\n'
            f'  lane_timeout={self.lane_timeout}s (초과 시 LOST, fail-safe)\n'
            '  범위: 아래층 반응형 SM만 (미션층은 확장 후). '
            'stop_request 발행원 없음 → 항상 False.'
        )

        # 계약 §3: lane_status/drive_command 는 reliable, depth 1 (rclpy 기본 reliable).
        self.pub = self.create_publisher(DriveCommand, cmd_topic, 1)
        self.sub = self.create_subscription(
            LaneStatus, lane_topic, self._on_lane_status, 1)
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
            stop_request=False,   # 발행원 없음(후속 e-stop 토픽에서 채움).
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

        cmd = self.decision.update(obs, dt)

        out = DriveCommand()
        out.header.stamp = now.to_msg()
        out.state = int(cmd.state)          # DriveState 값 = 계약 STATE_* 상수.
        out.go = bool(cmd.go)
        out.speed_scale = float(cmd.speed_scale)
        out.lookahead_scale = float(cmd.lookahead_scale)
        out.steer_limit = float(cmd.steer_limit)
        self.pub.publish(out)

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
