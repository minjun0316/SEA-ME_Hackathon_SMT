#!/usr/bin/env python3
"""@file capture_dataset.py
@brief YOLO 학습용 데이터 수집 노드 — raw 카메라 프레임을 라벨링용 jpg로 저장한다.

@details
save_frames.py(디버그 저장)와 목적이 다르다: 여기선 **YOLO가 실제 추론하는 raw
카메라 토픽**(camera/image/compressed)을 원본 해상도·고품질로 저장하고, 프레임이
직전 저장분과 너무 비슷하면 건너뛴다(--min-diff). 정지 상태에서 같은 그림이 수백 장
쌓이는 걸 막아 **다양한 학습 데이터**만 모은다. 저장본을 노트북으로 scp → 라벨링(예:
Roboflow/labelImg) → 재학습.

클래스(현 모델): 0:checker 1:green 2:red 3:stop_line — 이 대상들이 다양한 각도·거리·
조명으로 담기게 촬영한다.

@par 사용
```bash
# 기본: raw 카메라, 2fps, 중복 스킵, ~/yolo_dataset 에 이어붙여 저장(Ctrl-C 중단)
python3 capture_dataset.py
# 옵션 지정
python3 capture_dataset.py --out ~/dataset_light --fps 3 --min-diff 8 --count 300
```
- --topic    : 구독 토픽(기본 raw 카메라 /camera/image/compressed).
- --fps      : 초당 최대 저장 수(기본 2). 0=매 프레임.
- --min-diff : 직전 저장 프레임과 평균 픽셀차가 이 값 미만이면 스킵(중복 제거). 0=끔.
- --count    : 이번 실행 최대 저장 수. 0=무제한(Ctrl-C 중단).
- 이어쓰기   : --out 에 img_#####.jpg 가 있으면 그 다음 번호부터 누적 저장.
Ctrl-C 중단. 저장 후:  `scp -r topst@<보드IP>:~/yolo_dataset ./`
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

FNAME_RE = re.compile(r'img_(\d+)\.jpg$')


def next_start_index(out_dir):
    """out_dir 안 img_#####.jpg 중 최대 번호+1(이어쓰기 시작번호). 없으면 0."""
    mx = -1
    if os.path.isdir(out_dir):
        for name in os.listdir(out_dir):
            m = FNAME_RE.search(name)
            if m:
                mx = max(mx, int(m.group(1)))
    return mx + 1


class DatasetCapture(Node):
    def __init__(self, topic, out_dir, count, fps, min_diff, quality):
        super().__init__('dataset_capture')
        self.out_dir = os.path.expanduser(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.count = count
        self.min_interval = (1.0 / fps) if fps > 0 else 0.0
        self.min_diff = float(min_diff)
        self.quality = int(quality)
        self.start_index = next_start_index(self.out_dir)
        self.saved = 0
        self.skipped_dup = 0
        self._last_save_t = 0.0
        self._last_small = None                 # 중복 판정용 직전 저장 프레임(축소·그레이)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(CompressedImage, topic, self.on_image, qos)
        limit = '무제한(Ctrl-C 중단)' if count == 0 else f'{count}장'
        rate = '매 프레임' if self.min_interval == 0.0 else f'{fps}fps'
        dedup = '끔' if self.min_diff <= 0 else f'평균차<{self.min_diff:.0f} 스킵'
        self.get_logger().info(
            f'구독 {topic} → 저장 {self.out_dir}\n'
            f'  ({rate}, {limit}, 중복제거 {dedup}, 이어쓰기 시작 img_{self.start_index:05d})')

    def _is_duplicate(self, frame):
        """직전 저장 프레임과 평균 절대차로 근접중복 판정."""
        if self.min_diff <= 0:
            return False
        small = cv2.cvtColor(cv2.resize(frame, (64, 64)), cv2.COLOR_BGR2GRAY)
        if self._last_small is None:
            self._pending_small = small
            return False
        diff = float(np.mean(cv2.absdiff(small, self._last_small)))
        self._pending_small = small
        return diff < self.min_diff

    def on_image(self, msg):
        # fps 스로틀
        now = time.monotonic()
        if self.min_interval > 0.0 and (now - self._last_save_t) < self.min_interval:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        # 근접중복 스킵(직전 저장분과 거의 동일하면 버림)
        if self._is_duplicate(frame):
            self.skipped_dup += 1
            if self.skipped_dup % 30 == 0:
                self.get_logger().info(f'  (중복 스킵 누적 {self.skipped_dup}장)')
            return
        idx = self.start_index + self.saved
        path = os.path.join(self.out_dir, f'img_{idx:05d}.jpg')
        cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        self.saved += 1
        self._last_save_t = now
        self._last_small = getattr(self, '_pending_small', None)  # 저장한 것만 기준 갱신
        self.get_logger().info(f'저장 {path}  ({frame.shape[1]}x{frame.shape[0]})')
        if self.count > 0 and self.saved >= self.count:
            self.get_logger().info(f'{self.count}장 완료. scp로 가져가세요: {self.out_dir}')
            raise SystemExit


def main():
    ap = argparse.ArgumentParser(description='YOLO 학습용 raw 프레임 수집')
    ap.add_argument('--topic', default='/camera/image/compressed',
                    help='구독 토픽(기본 raw 카메라 — YOLO가 추론하는 원본)')
    ap.add_argument('--out', default='~/yolo_dataset', help='저장 폴더')
    ap.add_argument('--fps', type=float, default=2.0, help='초당 최대 저장 수(기본 2). 0=매 프레임')
    ap.add_argument('--min-diff', type=float, default=6.0,
                    help='직전 저장분과 평균 픽셀차 미만이면 스킵(중복제거). 0=끔')
    ap.add_argument('--count', type=int, default=0, help='최대 저장 수. 0=무제한(Ctrl-C)')
    ap.add_argument('--quality', type=int, default=95, help='JPEG 저장 품질(1~100)')
    args = ap.parse_args()

    rclpy.init()
    node = DatasetCapture(args.topic, args.out, args.count, args.fps,
                          args.min_diff, args.quality)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        last = node.start_index + max(0, node.saved - 1)
        node.get_logger().info(
            f'총 {node.saved}장 저장 (img_{node.start_index:05d} ~ img_{last:05d}), '
            f'중복 스킵 {node.skipped_dup}장')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
