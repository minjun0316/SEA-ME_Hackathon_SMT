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
from racer_msgs.msg import DriveCommand, LaneStatus  # noqa: E402

from core import path_factory  # noqa: E402
from core.config_schema import load_config  # noqa: E402
from core.planning.stopline_maneuver import StoplineManeuver  # noqa: E402
from core.control.lateral_pd import LateralPDController  # noqa: E402
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
        # 횡제어 법칙: pure_pursuit(경로 룩어헤드) | lateral_pd(근거리 offset+heading PD).
        # lateral_pd는 lane_path 대신 /perception/lane_status의 lateral_offset·heading_error를 쓴다.
        self.declare_parameter('lateral_controller', 'pure_pursuit')
        # 폐루프(topic) 입력 토픽 + watchdog.
        self.declare_parameter('lane_path_topic', '/perception/lane_path')
        self.declare_parameter('lane_status_topic', '/perception/lane_status')
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
        self.lateral_controller = str(
            self.get_parameter('lateral_controller').value).strip().lower()
        if self.lateral_controller not in ('pure_pursuit', 'lateral_pd'):
            raise ValueError("lateral_controller must be 'pure_pursuit' or 'lateral_pd'")
        self.use_lateral_pd = self.lateral_controller == 'lateral_pd'
        self.lane_timeout = float(self.get_parameter('lane_timeout').value)
        # 폐루프 입력 상태(topic 모드).
        self._topic_path = None          ##< 최신 lane_path → core.Path(없으면 None).
        self._topic_path_time = None     ##< 최신 lane_path 수신 시각.
        self._lane_status = None         ##< 최신 LaneStatus(lateral_pd 모드).
        self._lane_status_time = None    ##< 최신 LaneStatus 수신 시각.
        self._drive_cmd = None           ##< 최신 DriveCommand(게이트). 없으면 게이트 통과.
        self._drive_cmd_time = None      ##< 최신 DriveCommand 수신 시각(stale 판정).
        self._last_steer = None          ##< 마지막 발행 조향(정지 시 유지용).
        self._maneuver_active_prev = False  ##< 직전 스텝 기동 활성(종료 edge서 PD reset).

        # --- core 설정 로드 (vehicle.yaml + controller.yaml 병합) ---
        config_files = self._resolve_config_files()
        self.config = load_config(*config_files)

        # --- lateral_pd 게인 CLI 오버라이드 (pd_* 문자열 파라미터) ---
        # 튜닝 편의: `racer-run pd_k_heading:=0.4`처럼 파일 안 고치고 즉석 스윕.
        # 비워두면(기본 '') YAML 값을 그대로 쓴다. 좋은 값이 나오면 YAML에 박아 영구화.
        lpd = self.config.lateral_pd
        lpd_post = self.config.lateral_pd_post_sign
        for _name in ('steering_sign', 'k_cross', 'k_cross_kappa', 'k_cross_max',
                      'k_heading', 'k_deriv',
                      'deriv_smoothing', 'max_offset', 'steering_smoothing',
                      'k_ff', 'curvature_smoothing', 'curvature_deadband',
                      'curvature_preview'):
            self.declare_parameter(f'pd_{_name}', '')
            _v = str(self.get_parameter(f'pd_{_name}').value).strip()
            if _v:
                # 두 프로파일 **모두**에 건다. CLI는 스윕용 디버그 도구인데 baseline만
                # 바꾸면 팻말 뒤 구간에서 조용히 안 먹혀(= post_sign이 YAML 값을 유지)
                # "왜 안 변하지"로 시간을 태운다. 구간별로 다르게 주고 싶으면 YAML이 맞다.
                setattr(lpd, _name, float(_v))
                setattr(lpd_post, _name, float(_v))
                self.get_logger().info(
                    f'  lateral_pd.{_name} = {float(_v)} (CLI 오버라이드, YAML 무시, 양 프로파일)')
        self.get_logger().info(
            f'  lateral_pd 활성값: sign={lpd.steering_sign} k_cross={lpd.k_cross} '
            f'k_cross_kappa={lpd.k_cross_kappa} k_cross_max={lpd.k_cross_max} '
            f'k_heading={lpd.k_heading} k_deriv={lpd.k_deriv} '
            f'smooth={lpd.steering_smoothing} max_off={lpd.max_offset} '
            f'k_ff={lpd.k_ff} curv_smooth={lpd.curvature_smoothing} '
            f'curv_db={lpd.curvature_deadband} curv_prev={lpd.curvature_preview}')
        # post_sign 프로파일(팻말 분기 후 = 직선-ㄱ자-직선-ㄱ자). baseline과 다른 값만 찍는다.
        _diff = {f: (getattr(lpd, f), getattr(lpd_post, f))
                 for f in lpd.__dataclass_fields__
                 if getattr(lpd, f) != getattr(lpd_post, f)}
        self.get_logger().info(
            '  lateral_pd_post_sign: ' + (
                ' '.join(f'{k}={b}→{p}' for k, (b, p) in _diff.items()) if _diff
                else '(baseline과 동일 — 구간 분기 효과 없음)'))

        # --- 정지선 개루프 고정스티어 기동 (로터리 진입/탈출) ---
        # on/off는 운영 플래그 → ROS 파라미터(런치 인자). 튜닝값은 controller.yaml.
        self.declare_parameter('stopline_maneuver_enable', False)
        self.maneuver_enable = bool(
            self.get_parameter('stopline_maneuver_enable').value)
        self.maneuver = StoplineManeuver(self.config.stopline_maneuver)
        if self.maneuver_enable:
            smc = self.config.stopline_maneuver
            self.get_logger().info(
                f'  stopline_maneuver ON: steer={smc.steer} dur={smc.duration_sec}s '
                f'1st_dir={smc.first_dir}(좌+) 2nd_dir={smc.second_dir}(우-) '
                f'debounce={smc.debounce_sec}s max_count={smc.max_count}')

        # --- core 객체 구성 ---
        self.path = path_factory.make_path(self.path_name)
        self.pp = PurePursuitController(self.config.vehicle, self.config.pure_pursuit)
        self.lat = LateralPDController(self.config.vehicle, self.config.lateral_pd)
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
            cmd_topic = str(self.get_parameter('drive_command_topic').value)
            best_effort_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                       history=HistoryPolicy.KEEP_LAST)
            if self.use_lateral_pd:
                # lateral_pd: 근거리 offset+heading. lane_status는 reliable 발행(계약 §4.3).
                status_topic = str(self.get_parameter('lane_status_topic').value)
                self.create_subscription(
                    LaneStatus, status_topic, self._on_lane_status, 1)
                # 곡률 피드포워드(k_ff>0)용 lane_path도 구독(best-effort, 인지 §3).
                # 없거나 끊겨도 조향은 lane_status로 계속 — κ=0 폴백(순수PD). 조향
                # 입력은 여전히 lane_status라 watchdog은 lane_status 기준(변경 없음).
                lane_topic = str(self.get_parameter('lane_path_topic').value)
                self.create_subscription(
                    PathMsg, lane_topic, self._on_lane_path, best_effort_q)
                self.get_logger().info(
                    f'source=topic, lateral_controller=lateral_pd → 구독 '
                    f'lane_status={status_topic}, lane_path={lane_topic}(κ ff), '
                    f'drive_command={cmd_topic}, lane_timeout={self.lane_timeout}s')
            else:
                # pure_pursuit: lane_path(인지가 best-effort 발행 §3) → best-effort로 맞춤.
                lane_topic = str(self.get_parameter('lane_path_topic').value)
                self.create_subscription(
                    PathMsg, lane_topic, self._on_lane_path, best_effort_q)
                self.get_logger().info(
                    f'source=topic, lateral_controller=pure_pursuit → 구독 '
                    f'lane_path={lane_topic}, drive_command={cmd_topic}, '
                    f'lane_timeout={self.lane_timeout}s (끊기면 정지, 조향 유지)')
            # drive_command는 판단이 reliable로 발행 → reliable(기본) 유지.
            self.create_subscription(DriveCommand, cmd_topic, self._on_drive_command, 1)

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

    def _on_lane_status(self, msg: LaneStatus):
        """@brief /perception/lane_status 저장(lateral_pd 모드 입력)."""
        self._lane_status = msg
        self._lane_status_time = self.get_clock().now()

    def _on_drive_command(self, msg: DriveCommand):
        """@brief /decision/drive_command 저장(게이트/배율 적용용)."""
        self._drive_cmd = msg
        self._drive_cmd_time = self.get_clock().now()

    def _select_lane_status(self):
        """@brief lateral_pd용 최신 LaneStatus + 정지사유. @return (status, stop_reason|None)."""
        if self._lane_status is None or self._lane_status_time is None:
            return None, 'no_lane_status'
        age = (self.get_clock().now() - self._lane_status_time).nanoseconds * 1e-9
        if age > self.lane_timeout:
            return None, 'lane_timeout'
        if not self._lane_status.lane_detected:
            return None, 'lane_lost'
        return self._lane_status, None

    def _lat_cfg_for(self, gain_profile: int):
        """@brief 판단이 지시한 게인 프로파일 → 쓸 lateral_pd 게인 세트(계약 §4.4).

        @details 팻말 뒤 코스(직선-ㄱ자-직선-ㄱ자)는 S자와 곡률 성격이 정반대라 게인 세트를
        가른다. 판단은 구간 성격(gain_profile)만 주고 **수치는 여기(controller.yaml)** 있다.
        모르는 값이면 baseline으로 폴백한다 — 계약이 앞서가도 제어는 안전한 기본으로 돈다.
        @param gain_profile racer_msgs/DriveCommand.PROFILE_* 값.
        @return LateralPDConfig.
        """
        if gain_profile == 1:      # PROFILE_POST_SIGN
            return self.config.lateral_pd_post_sign
        return self.config.lateral_pd

    def _near_field_curvature(self):
        """@brief lateral_pd 곡률 피드포워드용 근거리 부호곡률 κ [1/m].

        @details lane_path(topic)가 있고 신선(<lane_timeout)하면 차량 원점 최근접
        부터 curvature_preview[m] 구간의 평균 부호곡률을 반환. 없거나 끊기면 0.0
        (피드포워드 off → 순수 PD로 안전 degrade). lane_path는 조향 주입력이 아니라
        보조 신호라 여기서 정지판정은 하지 않는다(정지판정은 lane_status가 담당).
        """
        if self._topic_path is None or self._topic_path_time is None:
            return 0.0
        age = (self.get_clock().now() - self._topic_path_time).nanoseconds * 1e-9
        if age > self.lane_timeout:
            return 0.0
        path = self._topic_path
        # 차량은 로컬 원점(뒷차축). 최근접부터 preview 구간 평균 부호곡률.
        idx = path.nearest_index(self.pose.x, self.pose.y)
        return path.mean_signed_curvature_ahead(
            idx, float(self.config.lateral_pd.curvature_preview))

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
        # 입력 선택: 법칙에 따라 lane_status(lateral_pd) 또는 lane_path(pure_pursuit).
        if self.use_lateral_pd:
            control_input, stop_reason = self._select_lane_status()
        else:
            control_input, stop_reason = self._select_path()

        # 판단 게이트: drive_command가 있으면 정지여부/속도배율 반영(없으면 통과).
        # 명령이 stale(판단 노드 끊김)하면 fail-safe 정지(오래된 go 명령을 붙들지 않음).
        cmd = self._drive_cmd
        gate_stop = False
        gate_reason = None
        speed_scale = 1.0
        steer_limit = 1.0
        steer_bias = 0.0     # 팻말 분기(SIGN_BRANCH) 조향 offset. 그 외 0.0.
        gain_profile = 0     # 제어 게인 프로파일(계약 §4.4). 팻말 분기 후만 POST_SIGN=1.
        if cmd is not None:
            cmd_age = (self.get_clock().now() - self._drive_cmd_time).nanoseconds * 1e-9
            if cmd_age > self.lane_timeout:
                gate_stop = True
                gate_reason = 'cmd_timeout'
            else:
                speed_scale = float(cmd.speed_scale)
                steer_limit = float(cmd.steer_limit)
                steer_bias = float(getattr(cmd, 'steer_bias', 0.0))
                # gain_profile: 미지정(=0)이 곧 DEFAULT라 ROS 기본값과 계약 기본값이 일치한다
                # (배율 방식과 달리 '안 채움'이 위험값이 되지 않는다).
                gain_profile = int(getattr(cmd, 'gain_profile', 0))
                if (not cmd.go) or cmd.state in (int(DriveState.STOP), int(DriveState.LOST)):
                    gate_stop = True
                    gate_reason = 'gate'

        # --- 정지선 개루프 고정스티어 기동 (아루코 게이트 정지가 아닐 때만 진행) ---
        # 활성 기동 중이면 차선 소실 watchdog(stop_reason)을 무시하고 정해진 시간만큼
        # 고정 조향으로 회전한다(로터리 진입=좌 / 탈출=우). 정지선 신호는 lateral_pd
        # 모드의 lane_status에서 온다.
        mvr_steer = None
        if self.maneuver_enable and not gate_stop:
            stop_line_now = bool(
                self.use_lateral_pd and control_input is not None
                and control_input.stop_line)
            mvr_steer = self.maneuver.update(stop_line_now, self.dt)
        maneuver_active = mvr_steer is not None
        # 기동 종료 edge(활성→비활성)에서 PD 내부상태 리셋 → 폐루프 복귀 시 조향 튐 방지.
        if self._maneuver_active_prev and not maneuver_active:
            self.lat.reset()
        self._maneuver_active_prev = maneuver_active

        dbg = None  # 주행 중이면 로그용 상세 문자열, 정지면 None.

        if gate_stop:
            # 판단(아루코) 정지: throttle=0, 조향은 마지막값 유지(없으면 트림=중립).
            steering = (self._last_steer if self._last_steer is not None
                        else float(self.config.vehicle.steer_trim))
            throttle = 0.0
        elif maneuver_active:
            # 개루프 고정스티어: 트림 실어 발행(규칙 #6), watchdog 무시하고 주행.
            steering = float(np.clip(
                float(self.config.vehicle.steer_trim) + float(mvr_steer),
                -1.0, 1.0))
            self._last_steer = steering
            if self.enable_drive:
                throttle = float(max(-self.throttle_limit,
                                     min(self.throttle_limit,
                                         self.drive_throttle * speed_scale)))
            else:
                throttle = 0.0
            dbg = (f'steer_raw={float(mvr_steer):+.3f} '
                   f'count={self.maneuver.count}')
        elif (stop_reason is not None) or control_input is None:
            # 정지: throttle=0, 조향은 마지막값 유지(없으면 트림=중립).
            steering = (self._last_steer if self._last_steer is not None
                        else float(self.config.vehicle.steer_trim))
            throttle = 0.0
        else:
            if self.use_lateral_pd:
                # 근거리 PD: lane_status의 offset+heading으로 조향. 곡률 피드포워드는
                # lane_path 근거리 κ(있으면), 없으면 0(순수 PD).
                kappa = self._near_field_curvature()
                # 구간별 게인 세트 선택(계약 §4.4). 내부 상태(EMA·직전 조향)는 유지된다.
                self.lat.set_config(self._lat_cfg_for(gain_profile))
                lat = self.lat.compute(float(control_input.lateral_offset),
                                       float(control_input.heading_error),
                                       self.dt, curvature=kappa)
                steering = float(lat.steering_norm)
                dbg = (f'off={lat.lateral_offset:+.3f}m head={lat.heading_error:+.3f}rad '
                       f'κ={lat.curvature:+.3f} pFF={lat.p_ff:+.3f} '
                       f'kC={lat.k_cross_eff:.2f} pC={lat.p_cross:+.3f} pH={lat.p_heading:+.3f}')
            else:
                pp = self.pp.compute(self.pose, control_input, self._last_speed)
                sp = self.speed.compute(pp.curvature, self._last_speed, self.dt)
                self._last_speed = sp.target_speed
                steering = float(pp.steering_norm)
                dbg = f'κ={pp.curvature:+.3f} Ld={pp.lookahead:.2f}'
            # 팻말 분기(SIGN_BRANCH) 조향 bias: 흰선 추종 위에 좌/우로 살짝 얹는다(+=좌).
            # 정규화 조향 규약(steer_trim 이미 포함)에 그대로 가산. 최종 [-1,1] 클램프.
            if steer_bias != 0.0:
                steering = float(np.clip(steering + steer_bias, -1.0, 1.0))
                if dbg is not None:
                    dbg += f' bias={steer_bias:+.2f}'
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
        if maneuver_active:
            reason = f'maneuver{self.maneuver.count}'
        else:
            reason = stop_reason or gate_reason or 'drive'
        if self._tick % self._log_period == 0 or (self._prev_log_steer != reason):
            self._prev_log_steer = reason
            if dbg is not None:
                self.get_logger().info(
                    f'[{reason}] steer={steering:+.4f} {dbg} '
                    f'throttle={throttle:.3f} scale={speed_scale:.2f}')
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
