"""@file mission_cues_node.py
@brief 미션 신호 인지 노드 → `/perception/mission_cues`(racer_msgs/MissionCues) 발행.

@details
차선 기하 외의 미션 페이즈 전환 신호(신호등·체커보드·빨강구역·아루코)를 한 토픽으로
묶어 발행한다(계약: docs/interfaces.md §4.5). 신호는 판단(mission)이 소비.

현재 구현: **ArUco 마커만**(cv2.aruco, 학습 불필요). 나머지(traffic_light/checkerboard/
red_zone)는 **stub**(기본값 발행) — 초록불(색검출)·빨강구역은 후속 추가.

@par 구독 → 발행
- 구독 `camera/image/compressed`(sensor_msgs/CompressedImage): 키트 camera_node.
- 발행 `/perception/mission_cues`(racer_msgs/MissionCues): reliable depth1(계약).
- (옵션) 발행 `perception/mission_cues/aruco/debug/compressed`: 오버레이.

@note 대회 시나리오: 심판이 막대에 붙인 ArUco(ID=3)를 **멀리서** 보여주면 정지,
      마커가 사라지면 재출발. → 하단 ROI 아님, **화면 전체**에서 ID 3만 인정.
      마커가 잠깐 흔들려 놓쳐도 튀지 않게 hold(디바운스)로 present 유지.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path as _FsPath


def _find_core_root() -> str:
    """@brief core import 루트 탐색. lane_detect_node/controller_node와 동일 규약."""
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
        'mission_cues_node: core 패키지를 찾지 못했습니다. 환경변수 D_RACER_ROOT 로 '
        'd_racer_autonomous 경로를 지정하세요.')


_CORE_ROOT = _find_core_root()
if _CORE_ROOT not in sys.path:
    sys.path.insert(0, _CORE_ROOT)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402

from sensor_msgs.msg import CompressedImage  # noqa: E402
from racer_msgs.msg import MissionCues  # noqa: E402

from core.perception.aruco_detect import ArucoDetector, ArucoConfig  # noqa: E402


class MissionCuesNode(Node):
    """@brief compressed 영상 → ArUco 검출(+stub) → MissionCues 발행."""

    def __init__(self):
        super().__init__('mission_cues_node')

        # --- 파라미터(전부 YAML/CLI 조정) ---
        self.declare_parameter('image_topic', 'camera/image/compressed')
        self.declare_parameter('mission_cues_topic', '/perception/mission_cues')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_jpeg_quality', 80)

        # ArUco 검출 파라미터.
        d = ArucoConfig()
        self.declare_parameter('aruco_dictionary', d.dictionary)  # 대회 마커에 맞춤(미확정).
        self.declare_parameter('aruco_roi_bottom_frac', 1.0)      # 막대에 들고 멀리서→전체.
        self.declare_parameter('aruco_min_perimeter_px', d.min_perimeter_px)
        # 정지 트리거로 인정할 마커 ID(대회=[3]). 비었거나 음수 포함이면 '아무 마커나'.
        self.declare_parameter('aruco_target_ids', [3])
        # present 홀드[s]: 마커 잠깐 놓쳐도 이 시간 안엔 유지(흔들림→튐 방지, 재출발 지연).
        self.declare_parameter('aruco_hold_sec', 0.4)

        target_ids = list(self.get_parameter('aruco_target_ids').value)
        # 음수/빈 리스트 = 아무 마커나 인정.
        use_any = (len(target_ids) == 0) or any(int(t) < 0 for t in target_ids)
        cfg = ArucoConfig(
            dictionary=str(self.get_parameter('aruco_dictionary').value),
            roi_bottom_frac=float(self.get_parameter('aruco_roi_bottom_frac').value),
            min_perimeter_px=float(self.get_parameter('aruco_min_perimeter_px').value),
            target_ids=() if use_any else tuple(int(t) for t in target_ids),
        )
        try:
            self.detector = ArucoDetector(cfg)
        except Exception as e:                        # 사전 이름 오타/미지원 등.
            self.get_logger().fatal(f'ArucoDetector 초기화 실패: {e}')
            raise

        self.hold_sec = float(self.get_parameter('aruco_hold_sec').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_quality = int(self.get_parameter('debug_jpeg_quality').value)

        image_topic = str(self.get_parameter('image_topic').value)
        cues_topic = str(self.get_parameter('mission_cues_topic').value)

        reliable_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        best_effort_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST)

        self.pub_cues = self.create_publisher(MissionCues, cues_topic, reliable_q)
        self.pub_debug = None
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                CompressedImage, 'perception/mission_cues/aruco/debug/compressed', 1)

        self.sub = self.create_subscription(
            CompressedImage, image_topic, self.on_image, best_effort_q)

        # 디바운스 상태.
        self._last_seen_time: Time | None = None
        self._held_present = False
        self._frames = 0

        self.get_logger().info(
            f'mission_cues_node ready: sub={image_topic} → pub {cues_topic} (reliable). '
            f'ArUco dict={cfg.dictionary} ids={"ANY" if use_any else list(cfg.target_ids)} '
            f'roi_bottom={cfg.roi_bottom_frac} hold={self.hold_sec}s. '
            f'traffic_light/checkerboard/red_zone=stub(미구현).')

    def on_image(self, msg: CompressedImage):
        """@brief compressed 디코드 → ArUco 검출 → 홀드 → MissionCues 발행."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('프레임 디코드 실패', throttle_duration_sec=2.0)
            return

        res = self.detector.detect(frame, want_debug=self.publish_debug)

        # --- present 홀드(디바운스): 검출되면 즉시 on, 끊겨도 hold_sec 동안 유지 ---
        now = self.get_clock().now()
        if res.present:
            self._last_seen_time = now
            self._held_present = True
        elif self._held_present and self._last_seen_time is not None:
            elapsed = (now - self._last_seen_time).nanoseconds * 1e-9
            if elapsed >= self.hold_sec:
                self._held_present = False

        # --- MissionCues 발행(aruco만 실제값, 나머지 stub) ---
        cue = MissionCues()
        cue.header.stamp = msg.header.stamp
        cue.header.frame_id = self.base_frame
        cue.traffic_light = MissionCues.TL_NONE      # stub(초록불 색검출 후속).
        cue.checkerboard_detected = False            # stub(YOLO 후속).
        cue.red_zone_detected = False                # stub(색검출 후속).
        cue.aruco_present = bool(self._held_present)
        self.pub_cues.publish(cue)

        if self.pub_debug is not None and res.debug_image is not None:
            self._publish_jpeg(self.pub_debug, res.debug_image, msg.header.stamp)

        self._frames += 1
        if res.present or self._frames % 30 == 0:
            self.get_logger().info(
                f'aruco raw={res.present} ids={res.ids} → present(held)={self._held_present}',
                throttle_duration_sec=0.5)

    def _publish_jpeg(self, pub, img, stamp):
        ok, enc = cv2.imencode(
            '.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_quality])
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp = stamp
        out.header.frame_id = self.base_frame
        out.format = 'jpeg'
        out.data = enc.tobytes()
        pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCuesNode()
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
