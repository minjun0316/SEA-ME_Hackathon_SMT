"""YOLO 디버그 이미지 저장 노드(진단 전용, 주행 로직과 무관).

웹 모니터는 지연이 있어 빨간불 오검출이 "어느 프레임/어느 위치"에서 났는지 보기 어렵다.
이 노드는 이미 발행 중인 YOLO 박스 오버레이(`.../yolo/debug/compressed`)만 구독해서
주행 중 낮은 주기로 jpg를 떨군다.

- **새 YOLO 추론을 돌리지 않는다.** 구독 → 파일 쓰기가 전부다.
- CompressedImage.data는 이미 JPEG 바이트라 **디코딩/재인코딩 없이 그대로** 쓴다
  (cv_bridge·numpy 불필요 → CPU/FPS 영향 최소).

실행:
    ros2 run d_racer_perception yolo_debug_recorder_node
    ros2 run d_racer_perception yolo_debug_recorder_node --ros-args \
        -p save_hz:=2.0 -p max_frames:=400 -p out_dir:=~/yolo_red_debug

주의: 발행측(mission_cues_node)은 publish_debug=true일 때만 디버그 토픽을 낸다.
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

DEFAULT_TOPIC = '/perception/mission_cues/yolo/debug/compressed'
DEFAULT_OUT_DIR = '~/yolo_red_debug'


class YoloDebugRecorder(Node):
    def __init__(self):
        super().__init__('yolo_debug_recorder_node')

        self.declare_parameter('image_topic', DEFAULT_TOPIC)
        self.declare_parameter('out_dir', DEFAULT_OUT_DIR)
        self.declare_parameter('save_hz', 1.0)
        self.declare_parameter('max_frames', 200)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.out_dir = os.path.expanduser(str(self.get_parameter('out_dir').value))
        save_hz = float(self.get_parameter('save_hz').value)
        self.max_frames = int(self.get_parameter('max_frames').value)

        # save_hz<=0이면 스로틀 없음(들어오는 족족 저장). 오타 방어로 음수도 같이 처리.
        self.min_period = (1.0 / save_hz) if save_hz > 0.0 else 0.0
        self.saved = 0
        self.last_save = 0.0
        self.done = False

        os.makedirs(self.out_dir, exist_ok=True)

        # 발행측(mission_cues_node)이 기본 QoS(RELIABLE, depth=1)로 낸다 → 동일하게 맞춘다.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            CompressedImage, self.image_topic, self.on_image, qos)

        self.get_logger().info(
            f'YOLO 디버그 저장 시작: topic={self.image_topic} → dir={self.out_dir} '
            f'(save_hz={save_hz}, max_frames={self.max_frames}) — 추론 없음, 구독만.')

    def on_image(self, msg: CompressedImage):
        if self.done:
            return

        now = time.monotonic()
        if self.min_period > 0.0 and (now - self.last_save) < self.min_period:
            return
        self.last_save = now

        self.saved += 1
        path = os.path.join(self.out_dir, f'frame_{self.saved:06d}.jpg')
        try:
            # msg.data가 곧 JPEG 바이트 → 그대로 쓴다(재인코딩 금지).
            with open(path, 'wb') as f:
                f.write(bytes(msg.data))
        except OSError as e:
            self.saved -= 1
            self.get_logger().error(f'저장 실패 {path}: {e}')
            return

        self.get_logger().info(f'저장 [{self.saved}/{self.max_frames}] {path}')

        if self.max_frames > 0 and self.saved >= self.max_frames:
            self.done = True
            self.destroy_subscription(self.sub)
            self.get_logger().info(
                f'저장 완료: {self.saved}장 → {self.out_dir} '
                '(max_frames 도달, 구독 해제. Ctrl+C로 종료)')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDebugRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'저장 완료: 총 {node.saved}장 → {node.out_dir}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
