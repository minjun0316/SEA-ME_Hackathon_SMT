"""@file controller_node.py
@brief Stage 6-b: ROS-free 코어를 감싸는 실차 조향/속도 컨트롤러 노드.

@details
`core`(Pure Pursuit + Speed)를 그대로 재사용해 정적 경로에 대한 조향/속도
명령을 계산하고 `control_msgs/Control`을 발행한다. 시뮬과 **완전히 동일한
코어 코드**를 쓰며(시뮬↔실차 일원화), 이 노드는 얇은 ROS2 래퍼일 뿐이다.

@par 이번 단계의 목표 (perception 이전)
아직 카메라 인지가 없어 차량 pose(로컬 경로)가 없다. 따라서 폐루프 주행은
불가능하다. 대신 **고정 pose(로컬 프레임 원점)** 를 가정하고 정적 경로에
대해 "곡률에 맞는 조향 명령이 나오는지"를 거치대/저속에서 검증한다.
- `path:=straight`  → 조향 ≈ steer_trim(직진 중립).
- `path:=circle`    → 한쪽으로 일정하게 꺾인 조향.
- `path:=sharp_s`   → 곡률이 클수록 큰 조향.
perception이 로컬 경로/pose를 제공하면 이 고정 pose를 실제 값으로 바꾸는
것만으로 폐루프가 완성된다(컨트롤러 코드 변경 없음).

@par 트림 처리 (중요, 07-02 확정)
키트 control_node는 `/control`의 steering을 트림 없이 그대로 서보에 전달한다
(control_node.py:121, STEER_TRIM은 idle/종료 중립값으로만 사용). 따라서 직진
보정은 **우리 쪽에서** 실어야 한다. Pure Pursuit `compute()`가 출력에
`vehicle.steer_trim`(=0.2238)을 이미 더해 반환하므로(pure_pursuit.py:141),
이 노드는 `steering_norm`을 그대로 발행하면 된다. 별도 가산 금지(이중 적용).

@par 안전 설계 (실모터 구동 가능 — calibration_node와 동일 원칙)
- 기본 `enable_drive=False` → throttle은 항상 0. 실행만으로는 바퀴가 안 돈다.
- `enable_drive=True`라도 throttle은 `throttle_limit`(기본 0.15)로 하드 클램프.
- Speed 컨트롤러 출력(target_speed)은 계산·로깅만 한다. 정규화 속도→실제
  throttle 매핑은 아직 캘리브레이션 전(Stage 4)이라 자동 구동에 쓰지 않는다.
- D-Racer joystick e-stop이 control_node에서 최우선 처리되므로 항상 손에 둘 것.

@par 발행
- `control_msgs/Control` on `control_topic`(기본 /control), `rate_hz`로 주기 발행.
- 키트 control_node는 `use_joystick_control:=False`일 때 이 토픽을 따른다.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path as _FsPath


# --------------------------------------------------------------------------- #
# core 패키지 경로 해석: 하드코딩/복사 없이 단일 소스(core/)를 import 한다.
# 소스 실행(d_racer_autonomous/ros2_ws/src/...)과 colcon install 실행
# (install/racer_bringup/lib/...) 두 위치 모두에서, 상위 디렉토리를 거슬러
# `.../d_racer_autonomous`(core/__init__.py + config/vehicle.yaml 보유)를 찾는다.
# --------------------------------------------------------------------------- #
def _find_core_root() -> str:
    """@brief `core`를 import 할 수 있는 프로젝트 루트(d_racer_autonomous)를 찾는다.

    @details 우선순위: 환경변수 `D_RACER_ROOT` → __file__ 상위 디렉토리 탐색.
    @return `core/`와 `config/`를 담은 디렉토리 절대경로.
    @throws RuntimeError 어디서도 찾지 못하면(설치 위치 이상) 명확히 실패시킨다.
    """
    env_root = os.environ.get('D_RACER_ROOT')
    candidates = []
    if env_root:
        candidates.append(_FsPath(os.path.expanduser(env_root)))

    here = _FsPath(__file__).resolve()
    for base in [here, *here.parents]:
        # 소스 위치: base 자체가 d_racer_autonomous.
        candidates.append(base)
        # install 위치: base 하위에 d_racer_autonomous 가 있을 수 있다.
        candidates.append(base / 'd_racer_autonomous')

    for cand in candidates:
        if (cand / 'core' / '__init__.py').exists() and \
           (cand / 'config' / 'vehicle.yaml').exists():
            return str(cand)

    raise RuntimeError(
        'controller_node: core 패키지를 찾지 못했습니다. '
        '환경변수 D_RACER_ROOT 로 d_racer_autonomous 경로를 지정하세요 '
        '(예: export D_RACER_ROOT=~/SEA-ME_Hackathon_SMT/d_racer_autonomous).'
    )


_CORE_ROOT = _find_core_root()
if _CORE_ROOT not in sys.path:
    sys.path.insert(0, _CORE_ROOT)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402

import numpy as np  # noqa: E402

from control_msgs.msg import Control  # noqa: E402
from nav_msgs.msg import Path as PathMsg  # noqa: E402
from racer_msgs.msg import DriveCommand  # noqa: E402

from core import path_factory  # noqa: E402
from core.config_schema import load_config  # noqa: E402
from core.control.pure_pursuit import PurePursuitController  # noqa: E402
from core.control.speed_controller import SpeedController  # noqa: E402
from core.geometry import Pose2D  # noqa: E402
from core.path import Path as CorePath  # noqa: E402
from core.planning import DriveState  # noqa: E402


class ControllerNode(Node):
    """@brief core(Pure Pursuit + Speed)를 감싸는 조향/속도 명령 발행 노드."""

    def __init__(self):
        super().__init__('controller_node')

        # --- 파라미터 (모두 CLI/YAML로 변경 가능, 하드코딩 없음) ---
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('rate_hz', 10.0)
        # 경로 입력원: static(정적경로, 거치대검증) | topic(/perception/lane_path, 폐루프).
        self.declare_parameter('source', 'static')
        # 사용할 정적 경로(source=static): straight | circle | s_curve | sharp_s | figure_eight | rotary
        self.declare_parameter('path', 'straight')
        # 폐루프(topic) 입력 토픽 + watchdog.
        self.declare_parameter('lane_path_topic', '/perception/lane_path')
        self.declare_parameter('drive_command_topic', '/decision/drive_command')
        self.declare_parameter('lane_timeout', 0.3)  # lane_path 끊김 판정[s] → 정지.
        # core 설정 YAML (비우면 core_root/config 의 기본 파일을 순서대로 병합).
        self.declare_parameter('config_files', [''])
        # perception 이전: 가정하는 로컬 pose(원점, 전방 +x). 폐루프 시 실제값으로 대체.
        self.declare_parameter('pose_x', 0.0)
        self.declare_parameter('pose_y', 0.0)
        self.declare_parameter('pose_yaw', 0.0)
        # --- 안전 게이트 ---
        # enable_drive=False면 throttle은 무조건 0 (조향만 검증).
        self.declare_parameter('enable_drive', False)
        # enable_drive=True일 때 발행할 상수 throttle (속도→throttle 캘리브 전 임시).
        self.declare_parameter('drive_throttle', 0.0)
        # throttle 크기 하드 상한 (절대 초과 안 함).
        self.declare_parameter('throttle_limit', 0.15)

        control_topic = str(self.get_parameter('control_topic').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be greater than 0')
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        self.path_name = str(self.get_parameter('path').value)
        self.pose = Pose2D(
            x=float(self.get_parameter('pose_x').value),
            y=float(self.get_parameter('pose_y').value),
            yaw=float(self.get_parameter('pose_yaw').value),
        )
        self.enable_drive = bool(self.get_parameter('enable_drive').value)
        self.drive_throttle = float(self.get_parameter('drive_throttle').value)
        self.throttle_limit = abs(float(self.get_parameter('throttle_limit').value))

        self.source = str(self.get_parameter('source').value).strip().lower()
        if self.source not in ('static', 'topic'):
            raise ValueError("source must be 'static' or 'topic'")
        self.lane_timeout = float(self.get_parameter('lane_timeout').value)
        # 폐루프 입력 상태(topic 모드).
        self._topic_path = None          ##< 최신 lane_path → core.Path(없으면 None).
        self._topic_path_time = None     ##< 최신 lane_path 수신 시각.
        self._drive_cmd = None           ##< 최신 DriveCommand(게이트). 없으면 게이트 통과.
        self._drive_cmd_time = None      ##< 최신 DriveCommand 수신 시각(stale 판정).
        self._last_steer = None          ##< 마지막 발행 조향(정지 시 유지용).

        # --- core 설정 로드 (vehicle.yaml + controller.yaml 병합) ---
        config_files = self._resolve_config_files()
        self.config = load_config(*config_files)

        # --- core 객체 구성 ---
        self.path = path_factory.make_path(self.path_name)
        self.pp = PurePursuitController(self.config.vehicle, self.config.pure_pursuit)
        self.speed = SpeedController(self.config.speed)
        # Adaptive lookahead 초기 속도값(첫 스텝용). 이후 target_speed로 갱신.
        self._last_speed = self.config.speed.v_min

        self.get_logger().info(
            'controller_node 구성:\n'
            f'  core_root={_CORE_ROOT}\n'
            f'  config_files={config_files}\n'
            f'  path={self.path_name} (points={len(self.path)})\n'
            f'  steer_trim={self.config.vehicle.steer_trim} '
            f'(PP 출력에 이미 포함, 재가산 금지)\n'
            f'  max_steer_deg={self.config.vehicle.max_steer_deg}\n'
            f'  wheelbase={self.config.vehicle.wheelbase}\n'
            f'  pose=({self.pose.x}, {self.pose.y}, yaw={self.pose.yaw})\n'
            f'  enable_drive={self.enable_drive} '
            f'drive_throttle={self.drive_throttle} '
            f'throttle_limit={self.throttle_limit}\n'
            f'  control_topic={control_topic} rate_hz={self.rate_hz}'
        )
        if not self.enable_drive:
            self.get_logger().info(
                'enable_drive=False → throttle=0 (조향만 검증). '
                '구동하려면 enable_drive:=True 로 실행할 것 (거치대에서 시작).'
            )

        self.publisher = self.create_publisher(Control, control_topic, 10)

        # 폐루프(topic): 경로/판단 명령 구독.
        if self.source == 'topic':
            lane_topic = str(self.get_parameter('lane_path_topic').value)
            cmd_topic = str(self.get_parameter('drive_command_topic').value)
            # lane_path는 인지가 best-effort로 발행(계약 §3) → 구독도 best-effort로 맞춤.
            # (reliable로 구독하면 QoS 불일치로 메시지를 아예 못 받음.)
            best_effort_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                       history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(PathMsg, lane_topic, self._on_lane_path, best_effort_q)
            # drive_command는 판단이 reliable로 발행 → reliable(기본) 유지.
            self.create_subscription(DriveCommand, cmd_topic, self._on_drive_command, 1)
            self.get_logger().info(
                f'source=topic → 구독 lane_path={lane_topic}, drive_command={cmd_topic}, '
                f'lane_timeout={self.lane_timeout}s (끊기면 정지, 조향 유지)')

        self.timer = self.create_timer(self.dt, self.timer_callback)

        # 로그 heartbeat(약 1초마다) 및 조향값 변화 감지용.
        self._log_period = max(1, int(round(self.rate_hz)))
        self._tick = 0
        self._prev_log_steer = None

    # ------------------------------------------------------------------ #
    def _resolve_config_files(self):
        """@brief 사용할 core 설정 YAML 경로 목록을 정한다.

        @details `config_files` 파라미터가 비어 있으면 core_root/config 의
        vehicle.yaml + controller.yaml 을 순서대로 병합한다(뒤가 앞을 override).
        """
        raw = self.get_parameter('config_files').value
        given = [str(p) for p in (raw or []) if str(p).strip()]
        if given:
            return [os.path.expanduser(p) for p in given]
        cfg_dir = os.path.join(_CORE_ROOT, 'config')
        return [
            os.path.join(cfg_dir, 'vehicle.yaml'),
            os.path.join(cfg_dir, 'controller.yaml'),
        ]

    def _on_lane_path(self, msg: PathMsg):
        """@brief /perception/lane_path(nav_msgs/Path) → core.Path 로 변환·저장.

        @details poses의 (x,y)를 (N,2) 배열로 만들어 core.Path 생성(계약 §4.2).
        점이 2개 미만이면 무효로 보고 저장하지 않는다(→ watchdog가 정지 처리).
        """
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            self._topic_path = None
            return
        try:
            self._topic_path = CorePath(np.asarray(pts, dtype=float))
            self._topic_path_time = self.get_clock().now()
        except Exception as exc:  # noqa: BLE001 - 경로 불량은 정지로 흡수.
            self.get_logger().warn(f'lane_path 변환 실패: {exc}')
            self._topic_path = None

    def _on_drive_command(self, msg: DriveCommand):
        """@brief /decision/drive_command 저장(게이트/배율 적용용)."""
        self._drive_cmd = msg
        self._drive_cmd_time = self.get_clock().now()

    def _select_path(self):
        """@brief 이번 주기 사용할 경로와 정지사유를 반환. @return (path, stop_reason|None)."""
        if self.source == 'static':
            return self.path, None
        # topic 모드: 최신 lane_path + watchdog.
        if self._topic_path is None or self._topic_path_time is None:
            return None, 'no_lane_path'
        age = (self.get_clock().now() - self._topic_path_time).nanoseconds * 1e-9
        if age > self.lane_timeout:
            return None, 'lane_timeout'
        return self._topic_path, None

    def timer_callback(self):
        """@brief 매 주기: 경로 선택(정적/토픽+watchdog) → PP 조향 + 게이트 → /control 발행.

        @details 폐루프(topic)에서 pose는 항상 로컬 원점. 경로가 없거나 끊기면(watchdog)
        또는 drive_command가 정지(go=false/STOP/LOST)면 throttle=0(조향은 마지막값 유지).
        """
        path, stop_reason = self._select_path()

        # 판단 게이트: drive_command가 있으면 정지여부/속도배율 반영(없으면 통과).
        # 명령이 stale(판단 노드 끊김)하면 fail-safe 정지(오래된 go 명령을 붙들지 않음).
        cmd = self._drive_cmd
        gate_stop = False
        gate_reason = None
        speed_scale = 1.0
        steer_limit = 1.0
        if cmd is not None:
            cmd_age = (self.get_clock().now() - self._drive_cmd_time).nanoseconds * 1e-9
            if cmd_age > self.lane_timeout:
                gate_stop = True
                gate_reason = 'cmd_timeout'
            else:
                speed_scale = float(cmd.speed_scale)
                steer_limit = float(cmd.steer_limit)
                if (not cmd.go) or cmd.state in (int(DriveState.STOP), int(DriveState.LOST)):
                    gate_stop = True
                    gate_reason = 'gate'

        stop = (stop_reason is not None) or gate_stop

        if stop or path is None:
            # 정지: throttle=0, 조향은 마지막값 유지(없으면 트림=중립).
            steering = (self._last_steer if self._last_steer is not None
                        else float(self.config.vehicle.steer_trim))
            throttle = 0.0
            pp = None
        else:
            pp = self.pp.compute(self.pose, path, self._last_speed)
            sp = self.speed.compute(pp.curvature, self._last_speed, self.dt)
            self._last_speed = sp.target_speed
            steering = float(pp.steering_norm)
            # steer_limit(정규화 조향 상한) 적용.
            if steer_limit < 1.0:
                steering = max(-steer_limit, min(steer_limit, steering))
            self._last_steer = steering
            # throttle: 안전 게이트 + speed_scale.
            if self.enable_drive:
                throttle = float(max(-self.throttle_limit,
                                     min(self.throttle_limit,
                                         self.drive_throttle * speed_scale)))
            else:
                throttle = 0.0

        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steering = float(steering)
        msg.throttle = float(throttle)
        self.publisher.publish(msg)

        # 로그: 1초마다 또는 상태/조향 변화 시.
        self._tick += 1
        reason = stop_reason or gate_reason or 'drive'
        if self._tick % self._log_period == 0 or (self._prev_log_steer != reason):
            self._prev_log_steer = reason
            if pp is not None:
                self.get_logger().info(
                    f'[{reason}] steer={steering:+.4f} (κ={pp.curvature:+.3f}) '
                    f'throttle={throttle:.3f} scale={speed_scale:.2f} Ld={pp.lookahead:.2f}')
            else:
                self.get_logger().info(
                    f'[STOP:{reason}] steer={steering:+.4f}(유지) throttle=0.0')

    def destroy_node(self):
        """@brief 종료 시 중립(steering=trim, throttle=0) 한 번 발행 후 종료."""
        try:
            if hasattr(self, 'publisher') and self.publisher is not None:
                msg = Control()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.steering = float(self.config.vehicle.steer_trim)
                msg.throttle = 0.0
                self.publisher.publish(msg)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
