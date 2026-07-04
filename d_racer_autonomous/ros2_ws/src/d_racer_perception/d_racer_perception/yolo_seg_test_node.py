from time import perf_counter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from ultralytics import YOLO

from .yolo_test_common import (
    RateMeter,
    decode_compressed_image,
    encode_jpeg,
    resolve_model_path,
)


class YoloSegTestNode(Node):
    """YOLO segmentation test node.

    This node is intentionally for performance/visibility checks only. It does
    not publish driving commands or final perception contracts.
    """

    def __init__(self):
        super().__init__('yolo_seg_test_node')

        self.declare_parameter('image_topic', 'camera/image/compressed')
        self.declare_parameter('model_path', 'yolo_seg_test.pt')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_jpeg_quality', 80)

        image_topic = str(self.get_parameter('image_topic').value)
        model_path = resolve_model_path(self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_jpeg_quality = int(self.get_parameter('debug_jpeg_quality').value)

        if not model_path.exists():
            raise FileNotFoundError(
                f'Model not found: {model_path}. Run scripts/download_test_models.sh first.'
            )
        self.model = YOLO(str(model_path))

        self.sub = self.create_subscription(CompressedImage, image_topic, self.on_image, 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, 'perception/test/yolo_seg/debug/compressed', 10
        )
        self.stats_pub = self.create_publisher(String, 'perception/test/yolo_seg/stats', 10)
        self.rate_meter = RateMeter()

        self.get_logger().info(
            f'YOLO seg test ready: topic={image_topic} model={model_path} '
            f'conf={self.conf} imgsz={self.imgsz}'
        )

    def on_image(self, msg):
        frame = decode_compressed_image(msg)
        if frame is None:
            self.get_logger().warn('Failed to decode compressed image')
            return

        start = perf_counter()
        result = self.model(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        latency_ms = (perf_counter() - start) * 1000.0
        fps = self.rate_meter.tick()

        box_count = 0 if result.boxes is None else len(result.boxes)
        mask_count = 0 if result.masks is None else len(result.masks)
        stats = String()
        stats.data = (
            f'boxes={box_count} masks={mask_count} '
            f'latency_ms={latency_ms:.1f} fps={fps:.1f}'
        )
        self.stats_pub.publish(stats)

        if self.publish_debug:
            debug = encode_jpeg(result.plot(), msg.header, self.debug_jpeg_quality)
            if debug is not None:
                self.debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
