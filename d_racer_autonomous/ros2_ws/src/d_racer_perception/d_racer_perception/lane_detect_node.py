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
from racer_msgs.msg import LaneStatus, LaneMode  # noqa: E402

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
        # 방향 팻말 → 차선 선택(07-14): 판단(/decision/lane_mode.turn_bias)을 구독해
        # BIAS_LEFT/RIGHT면 그 쪽 차선을 강제 anchor(커브·dual 무시), 팻말 방향으로
        # sign_lane_offset_m만큼 더 붙인다. BIAS_NONE이면 adaptive_anchor 자동으로 복귀.
        self.declare_parameter('lane_mode_topic', '/decision/lane_mode')
        self.declare_parameter('sign_lane_offset_m', 0.0)

        # 캘리브(core.LaneCalib 미러). 기본값=LaneCalib 기본값.
        d = LaneCalib()
        self.declare_parameter('m_per_px_forward', d.m_per_px_forward)
        self.declare_parameter('m_per_px_lateral', d.m_per_px_lateral)
        self.declare_parameter('x_near_m', d.x_near_m)
        self.declare_parameter('bev_top_y', d.bev_top_y)
        self.declare_parameter('bev_top_x', d.bev_top_x)
        self.declare_parameter('white_pixel_threshold', d.white_pixel_threshold)
        # 노랑 HLS 임계(H,L,S) 하한/상한 — 차선추종에선 폐기(흰선-only), 이제 정지선
        # (_stopline, stopline_color=yellow) 마스크만 참조. 정지선 노랑이 잘 안 잡히면 S/L 하한 튜닝.
        self.declare_parameter('yellow_lo', [int(v) for v in d.yellow_lo])
        self.declare_parameter('yellow_hi', [int(v) for v in d.yellow_hi])
        # 흰 HLS 임계(H,L,S). white_adaptive=False 기본 경로가 이 값을 씀. 이전엔 노드가
        # 미노출이라 core 기본값에 고정돼 있었음 → 파라미터로 물려 lane.yaml에서 튜닝 가능.
        self.declare_parameter('white_lo', [int(v) for v in d.white_lo])
        self.declare_parameter('white_hi', [int(v) for v in d.white_hi])
        self.declare_parameter('edge_from_mask', d.edge_from_mask)
        self.declare_parameter('edge_close_ksize', d.edge_close_ksize)
        self.declare_parameter('edge_dilate_iter', d.edge_dilate_iter)
        self.declare_parameter('canny_lo', d.canny_lo)
        self.declare_parameter('canny_hi', d.canny_hi)
        self.declare_parameter('stopline_len_threshold', d.stopline_len_threshold)
        # 정지선(행별 색 커버리지 밴드): 색(yellow|white|both)/ROI 높이/행 커버리지/가로폭·세로두께 창.
        self.declare_parameter('stopline_color', d.stopline_color)
        self.declare_parameter('stopline_roi_h', d.stopline_roi_h)
        self.declare_parameter('stopline_row_coverage', d.stopline_row_coverage)
        self.declare_parameter('stopline_max_width_m', d.stopline_max_width_m)
        self.declare_parameter('stopline_min_rows', d.stopline_min_rows)
        # 정지선 검출 방식(coverage|hough) + hough 파라미터(작년 검증 방식 재이식).
        self.declare_parameter('stopline_method', d.stopline_method)
        self.declare_parameter('stopline_canny_lo', d.stopline_canny_lo)
        self.declare_parameter('stopline_canny_hi', d.stopline_canny_hi)
        self.declare_parameter('stopline_hough_thresh', d.stopline_hough_thresh)
        self.declare_parameter('stopline_hough_min_len', d.stopline_hough_min_len)
        self.declare_parameter('stopline_hough_max_gap', d.stopline_hough_max_gap)
        self.declare_parameter('stopline_angle_tol_deg', d.stopline_angle_tol_deg)
        self.declare_parameter('lane_width_px', d.lane_width_px)
        self.declare_parameter('lane_width_learn', d.lane_width_learn)
        # 단일 기준선 고정 추종(single_anchor) — 오른선 하나만. dual 우회(붕괴/정체성 차단).
        self.declare_parameter('single_anchor', d.single_anchor)
        self.declare_parameter('anchor_side', d.anchor_side)
        self.declare_parameter('anchor_hold_frames', d.anchor_hold_frames)
        # 적응형 anchor(3-state 하이브리드): 직선/양선=dual, 커브=바깥선 single 자동 전환.
        self.declare_parameter('adaptive_anchor', d.adaptive_anchor)
        self.declare_parameter('both_enter_frames', d.both_enter_frames)
        self.declare_parameter('both_exit_frames', d.both_exit_frames)
        self.declare_parameter('curve_dx_deadband_px', d.curve_dx_deadband_px)
        self.declare_parameter('sign_force_max_curve_px', d.sign_force_max_curve_px)
        self.declare_parameter('sign_force_straight_frames', d.sign_force_straight_frames)
        self.declare_parameter('sign_apply', d.sign_apply)
        self.declare_parameter('perp_offset', d.perp_offset)
        self.declare_parameter('perp_max_slope', d.perp_max_slope)
        self.declare_parameter('seed_reacquire_blend', d.seed_reacquire_blend)
        self.declare_parameter('seed_lock_hist_blend', d.seed_lock_hist_blend)
        self.declare_parameter('seed_hist_band_px', d.seed_hist_band_px)
        # 체커보드 IPM 실측 BEV 행렬(이미지→BEV, 행우선 9값). 실차 노드는 IPM 전용.
        self.declare_parameter('bev_matrix', [0.0] * 9)

        _bev = list(self.get_parameter('bev_matrix').value or [])
        _bev = tuple(float(x) for x in _bev) \
            if len(_bev) == 9 and any(abs(float(x)) > 1e-12 for x in _bev) else None
        # IPM 강제: bev_matrix 없으면 4점 폴백으로 조용히 넘어가지 않고 즉시 실패.
        # (모니터에서 IPM↔4점이 실행마다 바뀌던 원인 = 임시 params에만 행렬이 있고
        #  정식 lane.yaml엔 없어서였음. 이제 lane.yaml에 행렬이 없으면 노드가 뜨지 않는다.)
        if _bev is None:
            raise RuntimeError(
                'bev_matrix(IPM 호모그래피 9값)가 설정되지 않았습니다. '
                'config/lane.yaml의 bev_matrix를 채우세요. 실차 노드는 IPM 전용입니다(4점 폴백 비활성).')

        calib = LaneCalib(
            m_per_px_forward=float(self.get_parameter('m_per_px_forward').value),
            m_per_px_lateral=float(self.get_parameter('m_per_px_lateral').value),
            x_near_m=float(self.get_parameter('x_near_m').value),
            bev_top_y=float(self.get_parameter('bev_top_y').value),
            bev_top_x=float(self.get_parameter('bev_top_x').value),
            bev_matrix=_bev,
            white_pixel_threshold=int(self.get_parameter('white_pixel_threshold').value),
            yellow_lo=tuple(int(v) for v in self.get_parameter('yellow_lo').value),
            yellow_hi=tuple(int(v) for v in self.get_parameter('yellow_hi').value),
            white_lo=tuple(int(v) for v in self.get_parameter('white_lo').value),
            white_hi=tuple(int(v) for v in self.get_parameter('white_hi').value),
            edge_from_mask=bool(self.get_parameter('edge_from_mask').value),
            edge_close_ksize=int(self.get_parameter('edge_close_ksize').value),
            edge_dilate_iter=int(self.get_parameter('edge_dilate_iter').value),
            canny_lo=int(self.get_parameter('canny_lo').value),
            canny_hi=int(self.get_parameter('canny_hi').value),
            stopline_len_threshold=float(self.get_parameter('stopline_len_threshold').value),
            stopline_color=str(self.get_parameter('stopline_color').value),
            stopline_roi_h=int(self.get_parameter('stopline_roi_h').value),
            stopline_row_coverage=float(self.get_parameter('stopline_row_coverage').value),
            stopline_max_width_m=float(self.get_parameter('stopline_max_width_m').value),
            stopline_min_rows=int(self.get_parameter('stopline_min_rows').value),
            stopline_method=str(self.get_parameter('stopline_method').value),
            stopline_canny_lo=int(self.get_parameter('stopline_canny_lo').value),
            stopline_canny_hi=int(self.get_parameter('stopline_canny_hi').value),
            stopline_hough_thresh=int(self.get_parameter('stopline_hough_thresh').value),
            stopline_hough_min_len=int(self.get_parameter('stopline_hough_min_len').value),
            stopline_hough_max_gap=int(self.get_parameter('stopline_hough_max_gap').value),
            stopline_angle_tol_deg=float(self.get_parameter('stopline_angle_tol_deg').value),
            lane_width_px=float(self.get_parameter('lane_width_px').value),
            lane_width_learn=bool(self.get_parameter('lane_width_learn').value),
            single_anchor=bool(self.get_parameter('single_anchor').value),
            anchor_side=str(self.get_parameter('anchor_side').value),
            anchor_hold_frames=int(self.get_parameter('anchor_hold_frames').value),
            adaptive_anchor=bool(self.get_parameter('adaptive_anchor').value),
            both_enter_frames=int(self.get_parameter('both_enter_frames').value),
            both_exit_frames=int(self.get_parameter('both_exit_frames').value),
            curve_dx_deadband_px=float(self.get_parameter('curve_dx_deadband_px').value),
            sign_force_max_curve_px=float(
                self.get_parameter('sign_force_max_curve_px').value),
            sign_force_straight_frames=int(
                self.get_parameter('sign_force_straight_frames').value),
            sign_apply=str(self.get_parameter('sign_apply').value),
            perp_offset=bool(self.get_parameter('perp_offset').value),
            perp_max_slope=float(self.get_parameter('perp_max_slope').value),
            seed_reacquire_blend=float(self.get_parameter('seed_reacquire_blend').value),
            seed_lock_hist_blend=float(self.get_parameter('seed_lock_hist_blend').value),
            seed_hist_band_px=float(self.get_parameter('seed_hist_band_px').value),
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

        # 방향 팻말 강제 anchor 상태(판단 /decision/lane_mode.turn_bias 구독).
        self._force_side = None       ##< 'left'|'right'|None. None=adaptive 자동.
        # 오프셋[m] → BEV px(팻말 방향 추가 붙임). m_per_px_lateral로 환산(0이면 0).
        _off_m = float(self.get_parameter('sign_lane_offset_m').value)
        _mppl = float(calib.m_per_px_lateral)
        self._sign_offset_px = (abs(_off_m) / _mppl) if _mppl > 1e-9 else 0.0
        lane_mode_topic = str(self.get_parameter('lane_mode_topic').value)
        self.sub_lane_mode = self.create_subscription(
            LaneMode, lane_mode_topic, self.on_lane_mode, reliable_q)

        # 카메라 구독(best-effort로 최신 프레임만).
        self.sub = self.create_subscription(
            CompressedImage, image_topic, self.on_image, best_effort_q)

        self._frames = 0
        self.get_logger().info(
            f'lane_detect_node ready: sub={image_topic} → '
            f'pub {status_topic} + {path_topic} (frame={self.base_frame}). '
            f'팻말 차선지시 구독={lane_mode_topic} '
            f'(offset={_off_m}m={self._sign_offset_px:.0f}px). '
            f'm/px(f,l)=({calib.m_per_px_forward},{calib.m_per_px_lateral}), '
            f'x_near={calib.x_near_m}m [잠정, 트랙 튜닝 필요]')

    def on_lane_mode(self, msg: LaneMode):
        """@brief 판단 역채널: turn_bias(팻말 좌/우) → 강제 anchor 쪽 갱신."""
        bias = int(msg.turn_bias)
        new_side = ('left' if bias == LaneMode.BIAS_LEFT
                    else 'right' if bias == LaneMode.BIAS_RIGHT else None)
        if new_side != self._force_side:
            self.get_logger().info(
                f'팻말 차선지시: force_side={new_side or "NONE(adaptive 복귀)"}')
        self._force_side = new_side

    def on_image(self, msg: CompressedImage):
        """@brief compressed 디코드 → detect → lane_status + lane_path 발행."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('프레임 디코드 실패', throttle_duration_sec=2.0)
            return

        res = self.detector.detect(
            frame, want_debug=self.publish_debug,
            force_side=self._force_side, sign_offset_px=self._sign_offset_px)
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
        s.on_yellow = False  # 지름길(노랑추종) 폐기(07-14). 필드는 계약 호환 유지, 항상 False.
        # 정지선 인식 순간(False→True) 즉시 로그 — 인식 여부를 눈으로 확인.
        if s.stop_line and not getattr(self, '_prev_stopline', False):
            self.get_logger().info(f'>>> STOPLINE 인식! dist={s.stop_line_dist:.2f}m')
        # 근접 로그(튜닝용): 노랑이 ROI에 잡히는데(≥0.15) 검출엔 미달인 순간을 즉시 찍어
        # 커버리지 피크를 놓치지 않게 한다 → 왜 못 잡는지(색? 임계?)를 숫자로 판단.
        elif (not s.stop_line) and res.stopline_cov_max >= 0.15:
            self.get_logger().warn(
                f'stopline 근접(미검출): cov={res.stopline_cov_max:.2f}/'
                f'{self.get_parameter("stopline_row_coverage").value:.2f} '
                f'rows={res.stopline_n_band} '
                f'min_rows={self.get_parameter("stopline_min_rows").value}')
        self._prev_stopline = bool(s.stop_line)
        # 주의: LaneStatus msg엔 yellow/white_detected 필드가 없다(슬림화됨). 여기서
        # s.yellow_detected 등을 set하면 AttributeError로 노드가 죽는다 → set 금지.
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
                f'stopdist={res.stop_line_dist:.2f}m '
                f'slcov={res.stopline_cov_max:.2f}/{self.get_parameter("stopline_row_coverage").value:.2f} '
                f'slthick={res.stopline_n_band}px')

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
    # 스레드 과다구독 방지(4코어 경합 완화): OpenCV 스레드 상한.
    # 기본 1, 필요 시 env LANE_CV_THREADS 로 조정.
    cv2.setNumThreads(int(os.environ.get('LANE_CV_THREADS', '1')))
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
