#!/usr/bin/env python3
"""@file save_frames.py
@brief 헤드리스 보드용 — CompressedImage 토픽을 구독해 프레임을 jpg로 저장한다.

디스플레이가 없는 보드(SSH)에서 rqt_image_view 대신 쓴다. 저장된 파일을 노트북으로
scp 해서 눈으로 확인한다.

@par 사용
```bash
# 기본: 차선 디버그 오버레이 20장 저장
python3 save_frames.py
# 토픽/개수/폴더 지정
python3 save_frames.py --topic /camera/image/compressed --count 30 --out ~/frames
```
Ctrl-C 로 언제든 중단. 저장 후:  `scp -r topst@<보드IP>:~/lane_debug ./`
"""
import argparse
import os
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


class FrameSaver(Node):
    def __init__(self, topic, out_dir, count):
        super().__init__('frame_saver')
        self.out_dir = os.path.expanduser(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.count = count
        self.saved = 0
        # 인지 디버그/카메라 모두 best-effort로 발행될 수 있어 best-effort 구독.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(CompressedImage, topic, self.on_image, qos)
        self.get_logger().info(f'구독 {topic} → 저장 {self.out_dir} (최대 {count}장)')

    def on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        path = os.path.join(self.out_dir, f'frame_{self.saved:03d}.jpg')
        cv2.imwrite(path, frame)
        self.saved += 1
        self.get_logger().info(f'저장 {path}  ({frame.shape[1]}x{frame.shape[0]})')
        if self.saved >= self.count:
            self.get_logger().info(f'{self.count}장 완료. scp로 가져가세요: {self.out_dir}')
            raise SystemExit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/perception/lane/debug/compressed')
    ap.add_argument('--out', default='~/lane_debug')
    ap.add_argument('--count', type=int, default=20)
    args = ap.parse_args()

    rclpy.init()
    node = FrameSaver(args.topic, args.out, args.count)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
