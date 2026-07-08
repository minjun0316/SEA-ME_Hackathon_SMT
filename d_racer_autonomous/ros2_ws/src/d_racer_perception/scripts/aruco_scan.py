#!/usr/bin/env python3
"""@file aruco_scan.py
@brief 카메라 프레임 1장을 모든 ArUco 사전으로 검출 시도 → 어느 사전/ID가 맞는지 찾는 진단툴.

mission_cues_node가 마커를 못 잡을 때 사용. 마커를 카메라에 보여준 채로 실행하면,
검출되는 (사전, ID)를 전부 출력한다. 프레임은 scratchpad에 저장돼 눈으로도 확인 가능.

사용:
  ros2 run 없이 직접:  python3 aruco_scan.py [image_topic] [save_path]
  기본 토픽 camera/image/compressed, 저장 /tmp/aruco_frame.jpg
"""
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

# 시도할 모든 predefined 사전(이 OpenCV에 존재하는 것만).
DICTS = [n for n in [
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
    "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
    "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
    "DICT_ARUCO_ORIGINAL",
    "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11",
] if hasattr(cv2.aruco, n)]


def scan(gray):
    ar = cv2.aruco
    hits = []
    for name in DICTS:
        d = ar.getPredefinedDictionary(getattr(ar, name))
        if hasattr(ar, "ArucoDetector"):
            det = ar.ArucoDetector(d, ar.DetectorParameters())
            corners, ids, _ = det.detectMarkers(gray)
        else:
            corners, ids, _ = ar.detectMarkers(gray, d)
        if ids is not None and len(ids) > 0:
            hits.append((name, ids.flatten().tolist()))
    return hits


class ScanNode(Node):
    def __init__(self, topic, save_path):
        super().__init__('aruco_scan')
        self.save_path = save_path
        self.done = False
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CompressedImage, topic, self.cb, q)
        self.get_logger().info(f'구독 {topic} — 마커를 카메라에 보여주세요. 프레임 대기중...')

    def cb(self, msg):
        if self.done:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('디코드 실패')
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        hits = scan(gray)
        cv2.imwrite(self.save_path, frame)
        print(f'\n=== 프레임 {w}x{h}, 저장: {self.save_path} ===')
        if hits:
            print('✅ 검출된 (사전, ID들):')
            for name, ids in hits:
                print(f'   {name:24s} ids={ids}')
        else:
            print('❌ 어떤 사전으로도 마커 미검출.')
            print('   → 마커가 프레임 안에 있나? 너무 멀거나 작진 않나? 흐릿/역광?')
            print('   → 마커 주변 흰 여백(quiet zone) 있나? 종이가 구겨졌나?')
        self.done = True


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else 'camera/image/compressed'
    save_path = sys.argv[2] if len(sys.argv) > 2 else '/tmp/aruco_frame.jpg'
    rclpy.init()
    node = ScanNode(topic, save_path)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
