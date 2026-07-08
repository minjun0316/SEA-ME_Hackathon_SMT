"""@file lane_detect_node.py
@brief 차선 인지 ROS2 노드 — core `LaneDetector`(ROS-free)의 얇은 래퍼.

@details
카메라 압축영상을 구독해 `core.perception.lane_detect.LaneDetector`로 차선을 뽑아
계약 토픽을 발행한다. 인지 로직은 core에 있고 이 노드는 구독/디코드/발행만 한다
(시뮬↔실차 로직 일원화, 하드코딩 금지·파라미터 YAML).

@par 구독 → 발행
- 구독 `camera/image/compressed`(sensor_msgs/CompressedImage): 키트 camera_node 소스.
- 발행 `/perception/lane_status`(racer_msgs/LaneStatus): 차선 메타(판단 소비). reliable.
- 발행 `/perception/lane_path`(nav_msgs/Path, frame=base_link): 중심선 경로(제어 소비). best-effort.
- (옵션) 발행 `perception/lane/debug/compressed`: 디버그 오버레이.

@note YOLO/미션 신호(mission_cues)와 무관. 차선 추종만 담당한다.
      lane_path는 base_link 미터 좌표 — 픽셀→미터 스케일은 config/lane.yaml에서 튜닝.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path as _FsPath


def _find_core_root() -> str:
    """@brief `core`를 import 할 프로젝트 루트(d_racer_autonomous). controller_node와 동일 규약."""
    env_root = os.environ.get('D_RACER_ROOT')
    candidates = []
    if env_root:
        candidates.append(_FsPath(os.path.expanduser(env_root)))
    here = _FsPath(__file__).resolve()
    for base in [here, *here.parents]:
        candidates.append(base)
        candidates.append(base / 'd_racer_autonomous')
    for cand in candidates:
        if (cand / 'core' / '__init__.py').exists() and \
           (cand / 'config' / 'vehicle.yaml').exists():
            return str(cand)
    raise RuntimeError(
        'lane_detect_node: core 패키지를 찾지 못했습니다. 환경변수 D_RACER_ROOT 로 '
        'd_racer_autonomous 경로를 지정하세요.')


_CORE_ROOT = _find_core_root()
if _CORE_ROOT not in sys.path:
    sys.path.insert(0, _CORE_ROOT)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402

from sensor_msgs.msg import CompressedImage  # noqa: E402
from nav_msgs.msg import Path as PathMsg  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from racer_msgs.msg import LaneStatus  # noqa: E402

from core.perception.lane_detect import LaneDetector, LaneCalib  # noqa: E402


class LaneDetectNode(Node):
    """@brief compressed 영상 → LaneDetector → lane_status + lane_path 발행."""

    def __init__(self):
        super().__init__('lane_detect_node')

        # --- 파라미터(전부 YAML/CLI 변경 가능) ---
        self.declare_parameter('image_topic', 'camera/image/compressed')
        self.declare_parameter('lane_status_topic', '/perception/lane_status')
        self.declare_parameter('lane_path_topic', '/perception/lane_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_jpeg_quality', 80)

        # 캘리브(core.LaneCalib 미러). 기본값=LaneCalib 기본값.
        d = LaneCalib()
        self.declare_parameter('m_per_px_forward', d.m_per_px_forward)
        self.declare_parameter('m_per_px_lateral', d.m_per_px_lateral)
        self.declare_parameter('x_near_m', d.x_near_m)
        self.declare_parameter('bev_top_y', d.bev_top_y)
        self.declare_parameter('bev_top_x', d.bev_top_x)
        self.declare_parameter('yellow_pixel_threshold', d.yellow_pixel_threshold)
        self.declare_parameter('stopline_len_threshold', d.stopline_len_threshold)

        calib = LaneCalib(
            m_per_px_forward=float(self.get_parameter('m_per_px_forward').value),
            m_per_px_lateral=float(self.get_parameter('m_per_px_lateral').value),
            x_near_m=float(self.get_parameter('x_near_m').value),
            bev_top_y=float(self.get_parameter('bev_top_y').value),
            bev_top_x=float(self.get_parameter('bev_top_x').value),
            yellow_pixel_threshold=int(self.get_parameter('yellow_pixel_threshold').value),
            stopline_len_threshold=float(self.get_parameter('stopline_len_threshold').value),
        )
        self.detector = LaneDetector(calib)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_quality = int(self.get_parameter('debug_jpeg_quality').value)

        image_topic = str(self.get_parameter('image_topic').value)
        status_topic = str(self.get_parameter('lane_status_topic').value)
        path_topic = str(self.get_parameter('lane_path_topic').value)

        # QoS: lane_status=reliable depth1, lane_path=best-effort depth1(계약 §3).
        reliable_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        best_effort_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST)

        self.pub_status = self.create_publisher(LaneStatus, status_topic, reliable_q)
        self.pub_path = self.create_publisher(PathMsg, path_topic, best_effort_q)
        self.pub_debug = None
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                CompressedImage, 'perception/lane/debug/compressed', 1)
        # 중간단계(bev/edges 등) 퍼블리셔는 처음 등장할 때 생성(core가 내보내는 키에 맞춤).
        self._stage_pubs: dict = {}

        # 카메라 구독(best-effort로 최신 프레임만).
        self.sub = self.create_subscription(
            CompressedImage, image_topic, self.on_image, best_effort_q)

        self._frames = 0
        self.get_logger().info(
            f'lane_detect_node ready: sub={image_topic} → '
            f'pub {status_topic} + {path_topic} (frame={self.base_frame}). '
            f'YOLO/미션 없음(차선 추종 전용). '
            f'm/px(f,l)=({calib.m_per_px_forward},{calib.m_per_px_lateral}), '
            f'x_near={calib.x_near_m}m [잠정, 트랙 튜닝 필요]')

    def on_image(self, msg: CompressedImage):
        """@brief compressed 디코드 → detect → lane_status + lane_path 발행."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('프레임 디코드 실패', throttle_duration_sec=2.0)
            return

        res = self.detector.detect(frame, want_debug=self.publish_debug)
        stamp = msg.header.stamp  # 촬영시각 승계(watchdog 타이밍 정확).

        # --- lane_status ---
        s = LaneStatus()
        s.header.stamp = stamp
        s.header.frame_id = self.base_frame
        s.lane_detected = bool(res.lane_detected)
        s.confidence = float(res.confidence)
        s.num_points = int(res.num_points)
        s.lateral_offset = float(res.lateral_offset)
        s.heading_error = float(res.heading_error)
        s.stop_line = bool(res.stop_line)
        s.stop_line_dist = float(res.stop_line_dist)
        s.yellow_detected = bool(res.yellow_detected)
        s.yellow_confidence = float(res.yellow_confidence)
        s.white_detected = bool(res.white_detected)
        s.white_confidence = float(res.white_confidence)
        self.pub_status.publish(s)

        # --- lane_path (near→far, base_link 미터) ---
        path = PathMsg()
        path.header.stamp = stamp
        path.header.frame_id = self.base_frame
        for (x, y) in res.lane_path:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.base_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)

        # --- 디버그 오버레이 ---
        if self.pub_debug is not None and res.debug_image is not None:
            self._publish_jpeg(self.pub_debug, res.debug_image, stamp)

        # --- 중간단계 디버그(bev/edges 등): perception/lane/debug/<stage>/compressed ---
        if self.publish_debug and res.debug_stages:
            for name, img in res.debug_stages.items():
                if img is None:
                    continue
                pub = self._stage_pubs.get(name)
                if pub is None:
                    pub = self.create_publisher(
                        CompressedImage, f'perception/lane/debug/{name}/compressed', 1)
                    self._stage_pubs[name] = pub
                self._publish_jpeg(pub, img, stamp)

        self._frames += 1
        if self._frames % 30 == 0:
            self.get_logger().info(
                f'lane={res.lane_detected} conf={res.confidence:.2f} '
                f'off={res.lateral_offset:+.3f}m head={res.heading_error:+.3f}rad '
                f'pts={res.num_points} stop={res.stop_line} '
                f'stopdist={res.stop_line_dist:.2f}m')

    def _publish_jpeg(self, pub, img, stamp):
        """@brief numpy 이미지(BGR 또는 그레이) → JPEG CompressedImage 발행."""
        ok, enc = cv2.imencode(
            '.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.format = 'jpeg'
        msg.data = enc.tobytes()
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
