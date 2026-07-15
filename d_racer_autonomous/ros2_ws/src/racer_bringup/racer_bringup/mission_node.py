"""@file mission_node.py
@brief 미션(Mission) ROS2 노드 — core MissionSequencer(6-state)의 얇은 래퍼.

@details
`core.planning.MissionSequencer`(위층 미션 SM + 내부 반응형 DecisionMaker)를 그대로
재사용해, 인지 토픽을 보고 제어/인지 지시를 발행한다. 판단 로직은 core에 있고 이
노드는 토픽 구독/변환/발행만 하는 얇은 래퍼다(시뮬↔실차 로직 일원화).

@par 구독 → 발행
- 구독 `/perception/lane_status`(racer_msgs/LaneStatus): 차선 기하.
- 구독 `/perception/mission_cues`(racer_msgs/MissionCues): 신호등·아루코·팻말(색/방향).
- 발행 `/decision/drive_command`(racer_msgs/DriveCommand): 제어 게이트(state/go/배율/steer_bias).
- 발행 `/decision/lane_mode`(racer_msgs/LaneMode): **인지 역방향 지시**
  (roi_mode/yolo_enable/sign_enable). 인지가 이 지시대로 ROI/YOLO 게이트를 적용한다.

@par 안전 (fail-safe)
- 타이머(rate_hz)마다 최신 관측으로 판단·발행.
- watchdog: lane_status가 `lane_timeout`(0.3s) 넘게 없으면 유효차선 없음 취급 →
  아래층 LOST → go=false(정지). 인지 끊겨도 안 달림.
- 시작 시 초록불 전까지 WAIT_START_SIGNAL → STOP.
- mission_cues 미수신 시 기본값(모두 미검출/신호없음) → 페이즈 유지(전이 안 함).

계약: docs/interfaces.md, 시퀀스: docs/mission_fsm.md.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path as _FsPath


def _find_core_root() -> str:
    """@brief `core`를 import 할 프로젝트 루트(d_racer_autonomous). controller_node와 동일."""
    env_root = os.environ.get('D_RACER_ROOT')
    candidates = []
    if env_root:
        candidates.append(_FsPath(os.path.expanduser(env_root)))
    here = _FsPath(__file__).resolve()
    for base in [here, *here.parents]:
        candidates.append(base)
        candidates.append(base / 'd_racer_autonomous')
    for cand in candidates:
        if (cand / 'core' / '__init__.py').exists() and \
           (cand / 'config' / 'vehicle.yaml').exists():
            return str(cand)
    raise RuntimeError(
        'mission_node: core 패키지를 찾지 못했습니다. 환경변수 D_RACER_ROOT 로 '
        'd_racer_autonomous 경로를 지정하세요.')


_CORE_ROOT = _find_core_root()
if _CORE_ROOT not in sys.path:
    sys.path.insert(0, _CORE_ROOT)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from racer_msgs.msg import DriveCommand, LaneMode, LaneStatus, MissionCues  # noqa: E402

from core.config_schema import load_config  # noqa: E402
from core.planning import (  # noqa: E402
    LaneObservation,
    MissionObservation,
    MissionPhase,
    MissionSequencer,
    SignDirection,
    TrafficLight,
)


class MissionNode(Node):
    """@brief core MissionSequencer를 감싸는 미션 판단 노드."""

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('lane_status_topic', '/perception/lane_status')
        self.declare_parameter('mission_cues_topic', '/perception/mission_cues')
        self.declare_parameter('drive_command_topic', '/decision/drive_command')
        self.declare_parameter('lane_mode_topic', '/decision/lane_mode')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('lane_timeout', 0.3)
        # 비우면 core_root/config 의 decision.yaml + mission.yaml 병합.
        self.declare_parameter('config_files', [''])

        lane_topic = str(self.get_parameter('lane_status_topic').value)
        cues_topic = str(self.get_parameter('mission_cues_topic').value)
        cmd_topic = str(self.get_parameter('drive_command_topic').value)
        mode_topic = str(self.get_parameter('lane_mode_topic').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be greater than 0')
        self.rate_hz = rate_hz
        self.dt_nominal = 1.0 / rate_hz
        self.lane_timeout = float(self.get_parameter('lane_timeout').value)

        config_files = self._resolve_config_files()
        self.config = load_config(*config_files)
        self.seq = MissionSequencer(self.config.mission, self.config.decision)

        self._last_lane = None
        self._last_lane_time = None
        self._last_cues = None
        self._last_tick_time = None
        self._prev_log_phase = None

        self.get_logger().info(
            'mission_node 구성:\n'
            f'  core_root={_CORE_ROOT}\n'
            f'  config_files={config_files}\n'
            f'  구독 lane_status={lane_topic}, mission_cues={cues_topic}\n'
            f'  발행 drive_command={cmd_topic}, lane_mode={mode_topic}\n'
            f'  rate_hz={self.rate_hz} lane_timeout={self.lane_timeout}s\n'
            '  범위: 6-state 미션 SM(+내부 반응형). ROI/mask/target은 인지 몫.'
        )

        self.pub_cmd = self.create_publisher(DriveCommand, cmd_topic, 1)
        self.pub_mode = self.create_publisher(LaneMode, mode_topic, 1)
        self.create_subscription(LaneStatus, lane_topic, self._on_lane_status, 1)
        self.create_subscription(MissionCues, cues_topic, self._on_mission_cues, 1)
        self.timer = self.create_timer(self.dt_nominal, self._on_timer)

        self._log_period = max(1, int(round(self.rate_hz)))
        self._tick = 0

    def _resolve_config_files(self):
        raw = self.get_parameter('config_files').value
        given = [str(p) for p in (raw or []) if str(p).strip()]
        if given:
            return [os.path.expanduser(p) for p in given]
        cfg_dir = os.path.join(_CORE_ROOT, 'config')
        return [os.path.join(cfg_dir, 'decision.yaml'),
                os.path.join(cfg_dir, 'mission.yaml')]

    def _on_lane_status(self, msg: LaneStatus):
        self._last_lane = msg
        self._last_lane_time = self.get_clock().now()

    def _on_mission_cues(self, msg: MissionCues):
        self._last_cues = msg

    def _build_obs(self, stale: bool) -> MissionObservation:
        """@brief 최신 lane_status + mission_cues → MissionObservation."""
        lane_msg = self._last_lane
        if stale or lane_msg is None:
            lane = LaneObservation(lane_detected=False)   # watchdog: 미검출.
        else:
            lane = LaneObservation(
                lane_detected=bool(lane_msg.lane_detected),
                confidence=float(lane_msg.confidence),
                num_points=int(lane_msg.num_points),
                lateral_offset=float(lane_msg.lateral_offset),
                heading_error=float(lane_msg.heading_error),
                stop_line=bool(lane_msg.stop_line),
                stop_line_dist=float(lane_msg.stop_line_dist),
                stop_request=False,
            )

        c = self._last_cues
        return MissionObservation(
            lane=lane,
            traffic_light=TrafficLight(int(c.traffic_light)) if c else TrafficLight.NONE,
            aruco_present=bool(c.aruco_present) if c else False,
            sign_detected=bool(getattr(c, 'sign_detected', False)) if c else False,
            sign_direction=(SignDirection(int(getattr(c, 'sign_direction', 0)))
                            if c else SignDirection.NONE),
        )

    def _on_timer(self):
        now = self.get_clock().now()
        if self._last_tick_time is None:
            dt = self.dt_nominal
        else:
            dt = (now - self._last_tick_time).nanoseconds * 1e-9
        self._last_tick_time = now

        stale = (self._last_lane is None or self._last_lane_time is None or
                 (now - self._last_lane_time).nanoseconds * 1e-9 > self.lane_timeout)

        obs = self._build_obs(stale)
        cmd = self.seq.update(obs, dt)

        stamp = now.to_msg()
        out = DriveCommand()
        out.header.stamp = stamp
        out.state = int(cmd.state)
        out.go = bool(cmd.go)
        out.speed_scale = float(cmd.speed_scale)
        out.lookahead_scale = float(cmd.lookahead_scale)
        out.steer_limit = float(cmd.steer_limit)
        out.steer_bias = float(cmd.steer_bias)     # 팻말 분기 조향 offset(그 외 0.0).
        self.pub_cmd.publish(out)

        mode = LaneMode()
        mode.header.stamp = stamp
        mode.follow_color = int(cmd.follow_color)
        mode.roi_mode = int(cmd.roi_mode)
        mode.turn_bias = int(cmd.turn_hint)
        mode.yolo_enable = bool(cmd.yolo_enable)   # 신호등 YOLO 게이트(출발/빨간불 종료).
        mode.sign_enable = bool(cmd.sign_enable)   # 팻말 YOLO 게이트(SIGN_BRANCH만).
        self.pub_mode.publish(mode)

        self._tick += 1
        phase = self.seq.phase
        changed = (self._prev_log_phase != phase)
        if self._tick % self._log_period == 0 or changed:
            self._prev_log_phase = phase
            self.get_logger().info(
                f'phase={phase.name} '
                f'go={cmd.go} speed={cmd.speed_scale:.2f} '
                f'roi={cmd.roi_mode.name} steer_bias={cmd.steer_bias:+.2f} '
                f'yolo={"ON" if cmd.yolo_enable else "OFF"} '
                f'sign={"ON" if cmd.sign_enable else "OFF"} '
                f'{"(stale)" if stale else ""}'
            )

    def destroy_node(self):
        """@brief 종료 시 정지 명령 한 번 발행."""
        try:
            if hasattr(self, 'pub_cmd') and self.pub_cmd is not None:
                from core.planning import DriveState
                out = DriveCommand()
                out.header.stamp = self.get_clock().now().to_msg()
                out.state = int(DriveState.STOP)
                out.go = False
                out.speed_scale = 0.0
                out.lookahead_scale = 1.0
                out.steer_limit = 1.0
                self.pub_cmd.publish(out)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
