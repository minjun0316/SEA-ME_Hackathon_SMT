import os
from pathlib import Path
import re
import shutil

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from battery_msgs.msg import Battery
from control_msgs.msg import Control
from joystick_msgs.msg import Joystick
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import yaml

from .flask_app_factory import FLASK_IMPORT_ERROR, FlaskServerThread, create_app
from .graph_utils import build_graph_snapshot
from .image_utils import extract_jpeg_dimensions
from .monitor_state import MonitorState

PACKAGE_ROOT = Path(__file__).resolve().parent


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/D-Racer/src/config/vehicle_config.yaml'


def resolve_resource_path(filename):
    candidates = []

    try:
        share_dir = Path(get_package_share_directory('monitor'))
        candidates.append(share_dir / 'resource' / filename)
    except PackageNotFoundError:
        pass

    candidates.append(PACKAGE_ROOT.parent / 'resource' / filename)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise FileNotFoundError(f'Unable to find resource file: {filename}')

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        if FLASK_IMPORT_ERROR is not None:
            raise RuntimeError(
                'Flask is not installed. Install "python3-flask" and try again.'
            ) from FLASK_IMPORT_ERROR

        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('battery_topic', 'battery_status')
        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('debug_image',True)
        self.declare_parameter('opencv_grayscale_topic', '/opencv/image/grayscale')
        self.declare_parameter('opencv_blur_topic', '/opencv/image/blur')
        self.declare_parameter('opencv_edge_topic', '/opencv/image/edge')
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('joystick_topic', 'joystick')
        self.declare_parameter('storage_path', '/')
        self.declare_parameter('storage_poll_interval_sec', 1.0)
        self.declare_parameter('web_host', '192.168.0.12')
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('page_title', 'D-Racer Monitor')
        self.declare_parameter('refresh_interval_ms', 1000)
        self.declare_parameter('image_refresh_interval_ms', 100)
        self.declare_parameter('stale_timeout_sec', 3.0)
        self.declare_parameter('image_source_width', 160)
        self.declare_parameter('image_source_height', 120)
        self.declare_parameter('image_display_width', 160)
        self.declare_parameter('image_display_height', 120)
        self.declare_parameter('debug_log', False)

        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        yaml_config = self.load_vehicle_config()

        self.battery_topic = self.get_yaml_or_param_str(yaml_config, 'BATTERY_TOPIC', 'battery_topic')
        self.image_topic = self.get_yaml_or_param_str(yaml_config, 'IMAGE_TOPIC', 'image_topic')
        self.control_topic = self.get_yaml_or_param_str(yaml_config, 'CONTROL_TOPIC', 'control_topic')
        self.debug_image = self.get_yaml_or_param_bool_multi(yaml_config, ('OPENCV_DEBUG_MODE', 'DEBUG_IMAGE'), 'debug_image')
        self.opencv_grayscale_topic = self.get_yaml_or_param_str(yaml_config, 'OPENCV_GRAYSCALE_TOPIC', 'opencv_grayscale_topic')
        self.opencv_blur_topic = self.get_yaml_or_param_str(yaml_config, 'OPENCV_BLUR_TOPIC', 'opencv_blur_topic')
        self.opencv_edge_topic = self.get_yaml_or_param_str(yaml_config, 'OPENCV_EDGE_TOPIC', 'opencv_edge_topic')
        self.joystick_topic = self.get_yaml_or_param_str(yaml_config, 'JOYSTICK_TOPIC', 'joystick_topic')
        if not self.control_topic:
            # Backward compatibility for legacy typo key.
            self.control_topic = str(yaml_config.get('CONTORL_TOPIC', '')).strip()
        if not self.control_topic:
            self.control_topic = str(self.get_parameter('control_topic').value)

        requested_storage_path = Path(
            self.get_yaml_or_param_str(yaml_config, 'STORAGE_PATH', 'storage_path')
        ).expanduser()
        self.storage_poll_interval_sec = float(
            self.get_parameter('storage_poll_interval_sec').value
        )
        self.web_host = self.get_yaml_or_param_str(yaml_config, 'WEB_HOST', 'web_host')
        self.web_port = self.get_yaml_or_param_int(yaml_config, 'WEB_PORT', 'web_port')
        self.page_title = str(self.get_parameter('page_title').value)
        self.refresh_interval_ms = int(self.get_parameter('refresh_interval_ms').value)
        self.image_refresh_interval_ms = int(
            self.get_parameter('image_refresh_interval_ms').value
        )
        self.debug_log = bool(self.get_parameter('debug_log').value)
        stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        image_source_width = int(self.get_parameter('image_source_width').value)
        image_source_height = int(self.get_parameter('image_source_height').value)
        self.image_source_width = image_source_width
        self.image_source_height = image_source_height
        self.image_display_width = self.get_yaml_or_param_int(
            yaml_config, 'IMAGE_DISPLAY_WIDTH', 'image_display_width'
        )
        self.image_display_height = self.get_yaml_or_param_int(
            yaml_config, 'IMAGE_DISPLAY_HEIGHT', 'image_display_height'
        )

        if requested_storage_path.exists():
            self.storage_path = str(requested_storage_path)
        else:
            self.storage_path = '/'
            self.get_logger().warning(
                f'Storage path {requested_storage_path} does not exist. Falling back to /.'
            )

        self.header_logo_path = resolve_resource_path('Telechips-CI-White.png')
        self.telechips_logo_path = resolve_resource_path('Telechips-CI-White.png')
        self.topst_logo_path = resolve_resource_path('TOPST-Logo(White).png')

        # 디버그 화면 채널 목록(config DEBUG_CHANNELS). 없으면 기존 opencv 3종으로 폴백.
        self.debug_channels = self._build_debug_channels(yaml_config)

        self.state = MonitorState(
            stale_timeout_sec,
            image_source_width,
            image_source_height,
            debug_keys=[ch['key'] for ch in self.debug_channels],
        )
        self.app = create_app(
            self.state,
            self.page_title,
            self.battery_topic,
            self.image_topic,
            self.control_topic,
            self.storage_path,
            self.refresh_interval_ms,
            self.image_refresh_interval_ms,
            self.header_logo_path,
            self.telechips_logo_path,
            self.topst_logo_path,
            self.image_display_width,
            self.image_display_height,
            self.debug_image,
            self.debug_channels,
            graph_snapshot_provider=self.get_graph_snapshot,
        )
        self.server_thread = FlaskServerThread(self.app, self.web_host, self.web_port)

        self.create_subscription(
            Battery,
            self.battery_topic,
            self.battery_callback,
            10,
        )
        self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10,
        )
        if self.debug_image:
            for channel in self.debug_channels:
                self.create_subscription(
                    CompressedImage,
                    channel['topic'],
                    self._make_debug_callback(channel['key'], channel['topic']),
                    10,
                )
        self.create_subscription(
            Control,
            self.control_topic,
            self.control_callback,
            10,
        )
        self.create_subscription(
            Joystick,
            self.joystick_topic,
            self.joystick_callback,
            10,
        )
        self.storage_timer = self.create_timer(
            self.storage_poll_interval_sec,
            self.storage_timer_callback,
        )
        self.storage_timer_callback()

        self.server_thread.start()

        display_host = '127.0.0.1' if self.web_host == '0.0.0.0' else self.web_host
        self.get_logger().info(
            f'[Monitor node started] \n'
            f'battery_topic={self.battery_topic} \n'
            f'image_topic={self.image_topic} \n'
            f'debug_image={self.debug_image}, \n'
            f'control_topic={self.control_topic}, \n'
            f'joystick_topic={self.joystick_topic}, \n'
            f'storage_path={self.storage_path}, \n'
            f'web=http://{display_host}:{self.web_port} \n' 
            f'vehicle_config_file={self.vehicle_config_file} \n'
        )

    def get_graph_snapshot(self):
        return build_graph_snapshot(self)

    @staticmethod
    def _slugify(text):
        """@brief 라벨 → URL/상태키로 쓸 슬러그(영숫자+언더스코어)."""
        slug = re.sub(r'[^a-z0-9]+', '_', str(text).lower()).strip('_')
        return slug or 'ch'

    def _build_debug_channels(self, yaml_config):
        """@brief config DEBUG_CHANNELS(label/topic 리스트) → 정규화된 채널 목록.

        @details 각 채널 = {key, label, topic}. key는 URL/상태 딕셔너리 키로 쓰이며
                 중복되면 접미사(_2, _3...)로 유일화한다. DEBUG_CHANNELS가 없으면
                 기존 opencv 3종(grayscale/blur/edge)으로 폴백해 하위호환 유지.
        """
        raw = yaml_config.get('DEBUG_CHANNELS')
        channels = []

        if isinstance(raw, list) and raw:
            for item in raw:
                if not isinstance(item, dict):
                    continue
                topic = str(item.get('topic', '')).strip()
                if not topic:
                    continue
                label = str(item.get('label', topic)).strip() or topic
                key = str(item.get('key', '')).strip() or self._slugify(label)
                channels.append({'key': key, 'label': label, 'topic': topic})
        else:
            legacy = [
                ('grayscale', 'Grayscale', self.opencv_grayscale_topic),
                ('blur', 'Blur', self.opencv_blur_topic),
                ('edge', 'Edge', self.opencv_edge_topic),
            ]
            for key, label, topic in legacy:
                if topic:
                    channels.append({'key': key, 'label': label, 'topic': topic})

        # 키 유일화(중복 라벨/토픽 대비).
        seen = set()
        for channel in channels:
            base = channel['key']
            unique = base
            suffix = 1
            while unique in seen:
                suffix += 1
                unique = f'{base}_{suffix}'
            channel['key'] = unique
            seen.add(unique)

        return channels

    def load_vehicle_config(self):
        if not os.path.exists(self.vehicle_config_file):
            return {}

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                return yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return {}

    def get_yaml_or_param_str(self, yaml_config, yaml_key, param_key):
        raw_value = yaml_config.get(yaml_key)
        if raw_value is not None:
            text_value = str(raw_value).strip()
            if text_value:
                return text_value
        return str(self.get_parameter(param_key).value)

    def get_yaml_or_param_int(self, yaml_config, yaml_key, param_key):
        raw_value = yaml_config.get(yaml_key)
        if raw_value is not None:
            text_value = str(raw_value).strip()
            if text_value:
                return int(raw_value)
        return int(self.get_parameter(param_key).value)

    def get_yaml_or_param_bool(self, yaml_config, yaml_key, param_key):
        raw_value = yaml_config.get(yaml_key)
        if raw_value is not None:
            if isinstance(raw_value, bool):
                return raw_value
            if isinstance(raw_value, str):
                return raw_value.strip().lower() in ('1', 'true', 'yes', 'on')
            return bool(raw_value)
        return bool(self.get_parameter(param_key).value)

    def get_yaml_or_param_bool_multi(self, yaml_config, yaml_keys, param_key):
        for yaml_key in yaml_keys:
            if yaml_key in yaml_config:
                return self.get_yaml_or_param_bool(yaml_config, yaml_key, param_key)
        return bool(self.get_parameter(param_key).value)

    def battery_callback(self, msg):
        self.state.update_battery(msg.battery_status)

        if self.debug_log:
            self.get_logger().info(f'Battery status updated: {msg.battery_status:.1f}%')

    def image_callback(self, msg):
        try:
            frame_bytes = bytes(msg.data)
            width, height = extract_jpeg_dimensions(frame_bytes)
            if width is None or height is None:
                width = self.image_source_width
                height = self.image_source_height

            self.state.update_image(frame_bytes, width, height)
        except Exception as exc:
            self.get_logger().error(f'Failed to process {self.image_topic} frame: {exc}')

    def control_callback(self, msg):
        self.state.update_control(msg.throttle, msg.steering)

        if self.debug_log:
            self.get_logger().info(
                f'Control updated: throttle={msg.throttle:.2f}, steering={msg.steering:.2f}'
            )

    def _debug_image_callback(self, msg, image_key, topic):
        try:
            frame_bytes = bytes(msg.data)
            width, height = extract_jpeg_dimensions(frame_bytes)
            if width is None or height is None:
                width = self.image_source_width
                height = self.image_source_height

            self.state.update_debug_image(image_key, frame_bytes, width, height)
        except Exception as exc:
            self.get_logger().error(f'Failed to process {topic} frame: {exc}')

    def _make_debug_callback(self, image_key, topic):
        """@brief 채널별 디버그 콜백 생성(루프 클로저 캡처 버그 방지)."""

        def _callback(msg):
            self._debug_image_callback(msg, image_key, topic)

        return _callback

    def joystick_callback(self, msg):
        self.state.update_recording(msg.is_recording)

        if self.debug_log:
            self.get_logger().info(f'Recording updated: is_recording={msg.is_recording}')

    def storage_timer_callback(self):
        try:
            usage = shutil.disk_usage(self.storage_path)
            self.state.update_storage(usage.used, usage.total)
        except Exception as exc:
            self.get_logger().error(f'Failed to read storage usage from {self.storage_path}: {exc}')

    def destroy_node(self):
        if hasattr(self, 'server_thread') and self.server_thread is not None:
            self.server_thread.shutdown()
            self.server_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    node = None
    rclpy.init(args=args)

    try:
        node = MonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().info('Shutting down monitor node.')
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
