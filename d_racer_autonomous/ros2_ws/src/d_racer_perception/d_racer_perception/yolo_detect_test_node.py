import os

# 4코어 보드 CPU 경합 완화: NCNN/OpenMP·BLAS 스레드 상한을 import 前에 고정한다.
# (네이티브 스레드풀이 import 시점에 생성되므로 ultralytics/torch import 이전에
#  설정해야 반영된다. launch가 값을 주면 setdefault가 그걸 존중.)
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

from time import perf_counter

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from ultralytics import YOLO

from .yolo_test_common import (
    RateMeter,
    decode_compressed_image,
    encode_jpeg,
    resolve_model_path,
)


class YoloDetectTestNode(Node):
    """YOLO detection-only test node.

    This node is intentionally for performance/visibility checks only. It does
    not publish driving commands or final perception contracts.
    """

    def __init__(self):
        super().__init__('yolo_detect_test_node')

        self.declare_parameter('image_topic', 'camera/image/compressed')
        self.declare_parameter('model_path', 'best_ncnn_model')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_jpeg_quality', 80)
        # 실시간성: 추론 상한 fps(카메라 26fps를 다 처리하지 않고 스킵) + 스레드 상한.
        self.declare_parameter('max_infer_hz', 8.0)
        self.declare_parameter('cv_threads', 1)
        self.declare_parameter('torch_threads', 2)

        image_topic = str(self.get_parameter('image_topic').value)
        model_path = resolve_model_path(self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_jpeg_quality = int(self.get_parameter('debug_jpeg_quality').value)

        # 스레드 과다구독 방지: OpenCV(디코드/plot/인코드)·torch(NMS/후처리) 코어 제한.
        cv2.setNumThreads(int(self.get_parameter('cv_threads').value))
        try:
            import torch
            torch.set_num_threads(int(self.get_parameter('torch_threads').value))
        except Exception:
            pass

        # 입력 스로틀: 최대 max_infer_hz 만 추론. 오래된 큐 프레임 누적/지연 폭증 방지.
        max_infer_hz = float(self.get_parameter('max_infer_hz').value)
        self._min_interval = 1.0 / max_infer_hz if max_infer_hz > 0.0 else 0.0
        self._last_infer = 0.0

        if not model_path.exists():
            raise FileNotFoundError(
                f'Model not found: {model_path}. Place the trained model under '
                f'models/ (e.g. best_ncnn_model/ or best.pt).'
            )
        self.model = YOLO(str(model_path))

        # best-effort + depth1: 항상 최신 프레임만 받는다(오래된 프레임 드롭).
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(
            CompressedImage, image_topic, self.on_image, sensor_qos
        )
        self.debug_pub = self.create_publisher(
            CompressedImage, 'perception/test/yolo_detect/debug/compressed', 10
        )
        self.stats_pub = self.create_publisher(String, 'perception/test/yolo_detect/stats', 10)
        self.rate_meter = RateMeter()

        self.get_logger().info(
            f'YOLO detect test ready: topic={image_topic} model={model_path} '
            f'conf={self.conf} imgsz={self.imgsz} max_infer_hz={max_infer_hz} '
            f'omp={os.environ.get("OMP_NUM_THREADS")} '
            f'cv_threads={self.get_parameter("cv_threads").value}'
        )

    def on_image(self, msg):
        # 스로틀: 직전 추론 후 최소 간격이 안 지났으면 이 프레임은 스킵(최대 max_infer_hz).
        now = perf_counter()
        if self._min_interval > 0.0 and (now - self._last_infer) < self._min_interval:
            return
        self._last_infer = now

        frame = decode_compressed_image(msg)
        if frame is None:
            self.get_logger().warn('Failed to decode compressed image')
            return

        start = perf_counter()
        result = self.model(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        latency_ms = (perf_counter() - start) * 1000.0
        fps = self.rate_meter.tick()

        box_count = 0 if result.boxes is None else len(result.boxes)
        stats = String()
        stats.data = f'boxes={box_count} latency_ms={latency_ms:.1f} fps={fps:.1f}'
        self.stats_pub.publish(stats)

        if self.publish_debug:
            debug = encode_jpeg(result.plot(), msg.header, self.debug_jpeg_quality)
            if debug is not None:
                self.debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
