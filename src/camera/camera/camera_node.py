import os
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
import yaml


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/D-Racer/src/config/vehicle_config.yaml'


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ROS parameters
        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('publish_topic', 'camera/image/compressed')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('usb_camera_device', '/dev/video1')
        self.declare_parameter('mipi_camera_device', '/dev/video0')
        self.declare_parameter('flip_method', 'none')
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('debug_log', True)

        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        publish_topic = str(self.get_parameter('publish_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be greater than 0')
        default_camera_device = str(self.get_parameter('camera_device').value)
        usb_camera_device = str(self.get_parameter('usb_camera_device').value)
        mipi_camera_device = str(self.get_parameter('mipi_camera_device').value)
        flip_method = str(self.get_parameter('flip_method').value)
        jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if not 0 <= jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in range [0, 100]')
        self.debug_log = bool(self.get_parameter('debug_log').value)
        self.publish_hz = publish_hz
        self.jpeg_quality = jpeg_quality

        self.image_width, self.image_height = self.load_image_size()
        # 캡처 해상도는 모델 입력(image_*)과 분리한다. C920은 16:9 센서라
        # 4:3(640x480)로 캡처하면 좌우가 잘려 수평 화각이 좁아진다. 16:9로 캡처해
        # 전체 화각을 확보하고, 출력은 _postprocess에서 image_*로 리사이즈한다.
        self.capture_width, self.capture_height = self.load_capture_size()
        self.usb_cam_enabled, self.mipi_cam_enabled = self.load_camera_source_flags()
        usb_camera_device, mipi_camera_device = self.load_camera_device_overrides(
            usb_camera_device,
            mipi_camera_device,
        )
        if self.usb_cam_enabled:
            self.camera_source = 'usb'
            camera_device = usb_camera_device or default_camera_device
        else:
            self.camera_source = 'mipi'
            camera_device = mipi_camera_device or default_camera_device

        self.camera_device = camera_device
        self.flip_method = flip_method

        # QoS compatible with web_video_server and monitor subscribers.
        self.image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher_ = self.create_publisher(CompressedImage, publish_topic, self.image_qos)
        self.cap = None
        self.pipeline = None
        if not self.open_capture():
            raise RuntimeError(
                'Failed to open camera with GStreamer pipeline '
                f'(source={self.camera_source}, device={camera_device}, '
                f'width={self.image_width}, height={self.image_height})'
            )

        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)
        self.get_logger().info('\n'
            f'[Camera Node] : topic={publish_topic} \n'
            f'[camera source] : {self.camera_source} \n'
            f'[width] : {self.image_width}, [height] : {self.image_height} \n'
            f'[capture] : {self.capture_width}x{self.capture_height} (16:9 화각 확보용) \n'
            f'[camera_device] : {camera_device} \n'
            f'[flip_method] : {flip_method} \n'
            f'[jpeg_quality] : {self.jpeg_quality} \n'
            f'[vehicle_config_file] : {self.vehicle_config_file} \n'
            f'[debug_log] : {self.debug_log} \n'
        )

    def load_image_size(self):
        default_size = (640, 480)
        if not os.path.exists(self.vehicle_config_file):
            return default_size

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                config_data = yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return default_size

        image_width = int(config_data.get('IMAGE_WIDTH', default_size[0]))
        image_height = int(config_data.get('IMAGE_HEIGHT', default_size[1]))
        return image_width, image_height

    def load_capture_size(self):
        # 센서에서 실제로 읽어올 해상도(=화각 결정). 16:9로 두어 C920의 전체 수평
        # 화각을 확보한다. 화각은 640x360이나 1280x720이나 동일하므로 CPU 절약 위해
        # 640x360을 기본값으로 쓴다. config에서 CAPTURE_WIDTH/HEIGHT로 재정의 가능.
        default_size = (640, 360)
        if not os.path.exists(self.vehicle_config_file):
            return default_size

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                config_data = yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return default_size

        capture_width = int(config_data.get('CAPTURE_WIDTH', default_size[0]))
        capture_height = int(config_data.get('CAPTURE_HEIGHT', default_size[1]))
        return capture_width, capture_height

    def load_camera_source_flags(self):
        # Backward-compatible default: MIPI enabled.
        default_usb_cam = False
        default_mipi_cam = True

        if not os.path.exists(self.vehicle_config_file):
            return default_usb_cam, default_mipi_cam

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                config_data = yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return default_usb_cam, default_mipi_cam

        usb_cam = bool(config_data.get('USB_CAM', default_usb_cam))
        mipi_cam = bool(config_data.get('MIPI_CAM', default_mipi_cam))

        if usb_cam and mipi_cam:
            raise ValueError('Only one of USB_CAM or MIPI_CAM can be true.')
        if not usb_cam and not mipi_cam:
            raise ValueError('One of USB_CAM or MIPI_CAM must be true.')

        return usb_cam, mipi_cam

    def build_candidate_pipelines(self, camera_device, flip_method):
        if self.usb_cam_enabled:
            # Many USB webcams expose MJPG by default.
            mjpg_pipeline = (
                f"v4l2src device={camera_device} io-mode=2 ! "
                "image/jpeg,framerate=30/1 ! jpegdec ! "
                f"videoconvert ! videoflip method={flip_method} ! videoscale ! "
                f"video/x-raw,format=BGR,width={self.image_width},height={self.image_height},framerate=30/1 ! "
                "appsink sync=false drop=true max-buffers=1"
            )
            # Fallback for raw USB camera modes.
            raw_pipeline = (
                f"v4l2src device={camera_device} io-mode=2 ! "
                f"videoconvert ! videoflip method={flip_method} ! videoscale ! "
                f"video/x-raw,format=BGR,width={self.image_width},height={self.image_height},framerate=30/1 ! "
                "appsink sync=false drop=true max-buffers=1"
            )
            return [mjpg_pipeline, raw_pipeline]

        mipi_pipeline = (
            f"v4l2src device={camera_device} io-mode=2 ! "
            f"video/x-raw,format=NV12,width={self.image_width},height={self.image_height},framerate=30/1 ! "
            f"videoconvert ! videoflip method={flip_method} ! "
            "video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=1"
        )
        return [mipi_pipeline]

    def open_capture(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            self.cap = None

        self.use_v4l2_direct = False

        for candidate_pipeline in self.build_candidate_pipelines(self.camera_device, self.flip_method):
            cap = cv2.VideoCapture(candidate_pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.cap = cap
                self.pipeline = candidate_pipeline
                self.get_logger().info(f'Camera capture opened with pipeline: {candidate_pipeline}')
                return True

            cap.release()
            self.get_logger().warning(f'Failed to open candidate pipeline: {candidate_pipeline}')

        # --- 폴백: OpenCV에 GStreamer 지원이 없는 빌드(GStreamer:NO)에서는 위 파이프라인이
        #     전부 실패한다. V4L2로 직접 열고 flip/resize는 수동 처리(출력 shape 동일). ---
        cap = self._open_v4l2(self.camera_device)
        if cap is not None:
            self.cap = cap
            self.pipeline = f'V4L2-direct({self.camera_device})'
            self.use_v4l2_direct = True
            self.get_logger().warning(
                f'GStreamer 파이프라인 실패 → V4L2 직접 캡처로 폴백: {self.camera_device} '
                f'(수동 flip={self.flip_method}, resize→{self.image_width}x{self.image_height})')
            return True

        self.cap = None
        self.pipeline = None
        return False

    def _open_v4l2(self, device):
        """@brief GStreamer 없이 V4L2로 직접 연다. @return 성공 시 VideoCapture, 실패 None."""
        candidates = [device]
        # '/dev/videoN' → 인덱스 N 도 시도(백엔드에 따라 문자열/인덱스 선호가 다름).
        try:
            if str(device).startswith('/dev/video'):
                candidates.append(int(str(device).replace('/dev/video', '')))
        except ValueError:
            pass
        for src in candidates:
            cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
            if cap.isOpened():
                try:  # MJPG 우선(USB 대역폭↓). 실패해도 무방.
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                except Exception:
                    pass
                # 16:9 캡처로 열어 좌우 화각 확보(FOURCC 뒤에 설정해야 반영됨).
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
                ok, frame = cap.read()
                if ok and frame is not None:
                    return cap
            cap.release()
        return None

    def _postprocess_v4l2(self, frame):
        """@brief V4L2 폴백 시 GStreamer가 하던 flip+scale을 수동으로(출력 규격 일치)."""
        method = (self.flip_method or '').lower()
        if method in ('rotate-180', '180'):
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif method in ('horizontal', 'horizontal-flip'):
            frame = cv2.flip(frame, 1)
        elif method in ('vertical', 'vertical-flip'):
            frame = cv2.flip(frame, 0)
        if (frame.shape[1], frame.shape[0]) != (self.image_width, self.image_height):
            frame = cv2.resize(frame, (self.image_width, self.image_height))
        return frame

    def load_camera_device_overrides(self, default_usb_camera_device, default_mipi_camera_device):
        if not os.path.exists(self.vehicle_config_file):
            return default_usb_camera_device, default_mipi_camera_device

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                config_data = yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return default_usb_camera_device, default_mipi_camera_device

        usb_camera_device = str(
            config_data.get('USB_CAM_DEVICE', default_usb_camera_device)
        ).strip()
        mipi_camera_device = str(
            config_data.get('MIPI_CAM_DEVICE', default_mipi_camera_device)
        ).strip()
        return usb_camera_device, mipi_camera_device

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warning('Camera capture is not opened')
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warning('Failed to read frame')
            return

        if getattr(self, 'use_v4l2_direct', False):
            frame = self._postprocess_v4l2(frame)

        success, encoded = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not success:
            self.get_logger().warning('Failed to encode frame as JPEG')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        self.publisher_.publish(msg)
        if self.debug_log:
            self.get_logger().info(f'Published frame: {len(msg.data)} bytes')

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                self.cap = None
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
