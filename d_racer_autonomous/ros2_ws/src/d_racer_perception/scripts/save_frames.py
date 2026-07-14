#!/usr/bin/env python3
"""@file save_frames.py
@brief 헤드리스 보드용 — CompressedImage 토픽을 구독해 프레임을 jpg로 저장한다.

디스플레이가 없는 보드(SSH)에서 rqt_image_view 대신 쓴다. 저장된 파일을 노트북으로
scp 해서 눈으로 확인한다(예: BEV 프레임으로 차선 색 HLS 튜닝).

@par 사용
```bash
# 기본: BEV 토픽을 3fps로, 폴더에 이어붙여 계속 저장(Ctrl-C 로 중단)
python3 save_frames.py
# 토픽/fps/최대장수/폴더 지정 (--fps 0 = 매 프레임, --count 0 = 무제한)
python3 save_frames.py --topic /camera/image/compressed --fps 3 --count 100 --out ~/frames
```
- --fps N  : 초당 최대 N장만 저장(기본 3). 나머지 프레임은 버려 중복을 줄인다. 0=매 프레임.
- 이어쓰기 : --out 폴더에 이미 frame_###.jpg 가 있으면 **그 다음 번호부터 이어 저장**한다.
             (주행을 여러 번 나눠 담아도 번호가 겹치지 않고 누적된다.)
- --count  : 이번 실행에서 저장할 최대 장수. 0=무제한(Ctrl-C 로 중단).
Ctrl-C 로 언제든 중단. 저장 후:  `scp -r topst@<보드IP>:~/bev_frames ./`
"""
import argparse
import os
import re
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


def next_start_index(out_dir):
    """out_dir 안의 frame_###.jpg 중 최대 번호+1 을 반환(이어쓰기 시작번호). 없으면 0."""
    mx = -1
    pat = re.compile(r'frame_(\d+)\.jpg$')
    if os.path.isdir(out_dir):
        for name in os.listdir(out_dir):
            m = pat.search(name)
            if m:
                mx = max(mx, int(m.group(1)))
    return mx + 1


class FrameSaver(Node):
    def __init__(self, topic, out_dir, count, fps):
        super().__init__('frame_saver')
        self.out_dir = os.path.expanduser(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.count = count                                  # 이번 실행 최대 저장 수(0=무제한)
        self.min_interval = (1.0 / fps) if fps > 0 else 0.0  # fps 제한 간격[s]
        self.start_index = next_start_index(self.out_dir)   # 이어쓰기 시작 번호
        self.saved = 0                                      # 이번 실행 저장 수
        self._last_save = 0.0
        # 인지 디버그/카메라 모두 best-effort로 발행될 수 있어 best-effort 구독.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(CompressedImage, topic, self.on_image, qos)
        limit = '무제한(Ctrl-C 중단)' if count == 0 else f'{count}장'
        rate = '매 프레임' if self.min_interval == 0.0 else f'{fps}fps'
        self.get_logger().info(
            f'구독 {topic} → 저장 {self.out_dir}\n'
            f'  ({rate}, {limit}, 이어쓰기 시작 frame_{self.start_index:03d})')

    def on_image(self, msg):
        # --- fps 스로틀: min_interval 안 지났으면 이 프레임은 버린다 ---
        now = time.monotonic()
        if self.min_interval > 0.0 and (now - self._last_save) < self.min_interval:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        idx = self.start_index + self.saved
        path = os.path.join(self.out_dir, f'frame_{idx:03d}.jpg')
        cv2.imwrite(path, frame)
        self.saved += 1
        self._last_save = now
        self.get_logger().info(f'저장 {path}  ({frame.shape[1]}x{frame.shape[0]})')
        if self.count > 0 and self.saved >= self.count:
            self.get_logger().info(f'{self.count}장 완료. scp로 가져가세요: {self.out_dir}')
            raise SystemExit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/perception/lane/debug/bev/compressed',
                    help='구독 토픽(기본 BEV 디버그). raw는 /camera/image/compressed')
    ap.add_argument('--out', default='~/bev_frames', help='저장 폴더')
    ap.add_argument('--count', type=int, default=0,
                    help='이번 실행 최대 저장 장수. 0=무제한(Ctrl-C 중단)')
    ap.add_argument('--fps', type=float, default=3.0,
                    help='초당 최대 저장 장수(기본 3). 0=매 프레임 저장')
    args = ap.parse_args()

    rclpy.init()
    node = FrameSaver(args.topic, args.out, args.count, args.fps)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        last = node.start_index + max(0, node.saved - 1)
        node.get_logger().info(
            f'총 {node.saved}장 저장 (frame_{node.start_index:03d} ~ frame_{last:03d})')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
