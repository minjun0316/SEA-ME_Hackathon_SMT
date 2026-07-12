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

# 4코어 보드 CPU 경합 완화: YOLO(NCNN/torch) 네이티브 스레드풀 상한을 ultralytics
# import 前에 고정한다(TrafficLightDetector가 지연 import). aruco 경로엔 영향 없음.
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

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
from core.perception.traffic_light_detect import (  # noqa: E402
    TrafficLightDetector, TrafficCueConfig, TL_NONE, TL_RED, TL_GREEN)

from .yolo_test_common import resolve_model_path  # noqa: E402


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

        # --- YOLO 미션신호(신호등/체커보드) 검출 ---
        # mission_cues_node가 MissionCues 주인이라 신호등·체커보드(둘 다 YOLO)도 여기서
        # 채운다. ultralytics는 무거워 실패해도 aruco는 계속 동작하도록 격리.
        self.declare_parameter('yolo_enable', True)
        self.declare_parameter('yolo_model_path', 'best_ncnn_model')
        self.declare_parameter('yolo_conf', 0.35)
        self.declare_parameter('yolo_imgsz', 320)
        self.declare_parameter('yolo_max_infer_hz', 6.0)     # 추론 상한(카메라 다 안 돌림→CPU 경합↓).
        self.declare_parameter('yolo_cv_threads', 1)
        self.declare_parameter('yolo_torch_threads', 2)
        self.declare_parameter('yolo_stale_sec', 1.0)        # 이 시간 내 추론 없으면 신호 NONE.
        self.declare_parameter('tl_prefer_red', True)        # 초록·빨강 동시 → RED(출발 안전측).
        self.declare_parameter('tl_green_confirm_sec', 0.5)  # 초록 연속 확정 시간(오출발 방지).
        self.declare_parameter('checker_hold_sec', 0.3)      # 체커보드 잠깐 놓쳐도 유지.

        self.yolo_enable = bool(self.get_parameter('yolo_enable').value)
        self.yolo_stale_sec = float(self.get_parameter('yolo_stale_sec').value)
        self.tl_green_confirm_sec = float(self.get_parameter('tl_green_confirm_sec').value)
        self.checker_hold_sec = float(self.get_parameter('checker_hold_sec').value)
        _yolo_hz = float(self.get_parameter('yolo_max_infer_hz').value)
        self._yolo_min_interval = 1.0 / _yolo_hz if _yolo_hz > 0.0 else 0.0

        self.tl_detector = None
        if self.yolo_enable:
            try:
                _model_path = resolve_model_path(
                    self.get_parameter('yolo_model_path').value)
                _tcfg = TrafficCueConfig(
                    model_path=str(_model_path),
                    conf=float(self.get_parameter('yolo_conf').value),
                    imgsz=int(self.get_parameter('yolo_imgsz').value),
                    prefer_red=bool(self.get_parameter('tl_prefer_red').value),
                    cv_threads=int(self.get_parameter('yolo_cv_threads').value),
                    torch_threads=int(self.get_parameter('yolo_torch_threads').value),
                )
                self.tl_detector = TrafficLightDetector(_tcfg)
                self.get_logger().info(
                    f'YOLO 미션신호 ON: model={_model_path} conf={_tcfg.conf} '
                    f'imgsz={_tcfg.imgsz} max_hz={_yolo_hz} prefer_red={_tcfg.prefer_red} '
                    f'green_confirm={self.tl_green_confirm_sec}s stale={self.yolo_stale_sec}s')
            except Exception as e:  # noqa: BLE001 - 모델/torch 미비 시 aruco만 계속.
                self.tl_detector = None
                self.get_logger().warn(
                    f'YOLO 초기화 실패 → 신호등/체커보드 비활성(aruco만 동작): {e}')
        else:
            self.get_logger().info('yolo_enable=False → 신호등/체커보드 stub(aruco만).')

        # YOLO 디버그 오버레이(옵션, aruco와 별도 토픽).
        self.pub_yolo_debug = None
        if self.publish_debug and self.tl_detector is not None:
            self.pub_yolo_debug = self.create_publisher(
                CompressedImage, 'perception/mission_cues/yolo/debug/compressed', 1)

        # YOLO 시간 상태(스로틀·초록 확정·체커 hold).
        self._yolo_last_infer: Time | None = None    ##< 마지막 추론 시각(스로틀).
        self._yolo_result_time: Time | None = None   ##< 마지막 추론 결과 시각(stale 판정).
        self._tl_raw_light = TL_NONE                  ##< 마지막 추론 프레임단위 신호.
        self._green_since: Time | None = None         ##< 초록 연속 시작 시각(확정용).
        self._checker_seen_time: Time | None = None   ##< 체커보드 마지막 검출 시각(hold).

        self.get_logger().info(
            f'mission_cues_node ready: sub={image_topic} → pub {cues_topic} (reliable). '
            f'ArUco dict={cfg.dictionary} ids={"ANY" if use_any else list(cfg.target_ids)} '
            f'roi_bottom={cfg.roi_bottom_frac} hold={self.hold_sec}s. '
            f'traffic_light/checkerboard=YOLO({"ON" if self.tl_detector else "OFF"}), '
            f'red_zone=stub(미구현).')

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

        # --- YOLO 미션신호: 스로틀 추론 → 초록 연속 확정 / 체커 hold 갱신 ---
        if self.tl_detector is not None:
            due = (self._yolo_last_infer is None or
                   (now - self._yolo_last_infer).nanoseconds * 1e-9
                   >= self._yolo_min_interval)
            if due:
                self._yolo_last_infer = now
                tl = self.tl_detector.detect(
                    frame, want_debug=self.pub_yolo_debug is not None)
                self._yolo_result_time = now
                self._tl_raw_light = tl.light
                # 초록이면 연속 시작시각 유지, 아니면 확정 타이머 리셋.
                self._green_since = (
                    (self._green_since or now) if tl.light == TL_GREEN else None)
                if tl.checker:
                    self._checker_seen_time = now
                if self.pub_yolo_debug is not None and tl.debug_image is not None:
                    self._publish_jpeg(self.pub_yolo_debug, tl.debug_image, msg.header.stamp)
                self.get_logger().info(
                    f'yolo light={tl.light} g={tl.green_conf:.2f} r={tl.red_conf:.2f} '
                    f'checker={tl.checker}({tl.checker_conf:.2f}) boxes={tl.num_boxes}',
                    throttle_duration_sec=1.0)

        tl_state, checker_det = self._resolve_cues(now)

        # --- MissionCues 발행(aruco/신호등/체커보드 실제값, red_zone stub) ---
        cue = MissionCues()
        cue.header.stamp = msg.header.stamp
        cue.header.frame_id = self.base_frame
        cue.traffic_light = tl_state                 # YOLO 신호등(초록 확정 후 GREEN).
        cue.checkerboard_detected = checker_det      # YOLO 체커보드(hold).
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

    def _resolve_cues(self, now):
        """@brief 시간 상태로 발행할 traffic_light + checkerboard 계산.

        @details 추론이 stale(오래됨)하거나 없으면 NONE/false로 폴백(끊긴 프레임에 붙들려
        가짜 신호 유지 방지). 초록은 tl_green_confirm_sec 동안 **연속**돼야 TL_GREEN을 낸다
        (한 프레임 가짜 초록 오출발 방지) — 확정 전엔 NONE(대기). 빨강은 즉시(대기라 안전).
        체커보드는 hold로 잠깐 놓쳐도 유지. @return (traffic_light, checkerboard_detected).
        """
        if self.tl_detector is None or self._yolo_result_time is None:
            return MissionCues.TL_NONE, False
        fresh = (now - self._yolo_result_time).nanoseconds * 1e-9 <= self.yolo_stale_sec
        if not fresh:
            return MissionCues.TL_NONE, False

        if self._tl_raw_light == TL_RED:
            light = MissionCues.TL_RED
        elif (self._tl_raw_light == TL_GREEN and self._green_since is not None and
              (now - self._green_since).nanoseconds * 1e-9 >= self.tl_green_confirm_sec):
            light = MissionCues.TL_GREEN
        else:
            light = MissionCues.TL_NONE   # 초록 확정 전/미검출 → 대기.

        checker = (self._checker_seen_time is not None and
                   (now - self._checker_seen_time).nanoseconds * 1e-9 <= self.checker_hold_sec)
        return light, checker

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
