"""@file pixel_pid_node.py
@brief [실험] 원본 카메라 라인트래커(C++)의 횡방향 픽셀-PID를 1:1 이식한 독립 노드.

@details
`racer-run-ex`(실험) 전용. 팀원 인지(`lane_detect_node`)의 미터 기반 `lane_path`를
쓰지 않고, **카메라 압축영상을 직접 구독해 BEV→HLS 마스크→히스토그램→슬라이딩
윈도우로 차선중심 픽셀 `lane_center`를 뽑고, `error = midpoint - lane_center`(픽셀)에
원본 픽셀 게인 PID(Kp=0.65, Ki=0.008, Kd=0.0008)를 그대로 적용**해 조향을 만든다.
정규화(`/(width/2)`)·출력 포화·조향 오프셋까지 원본과 동일하게 유지한다.

@par 아키텍처상 위치 (의도적 예외)
이 노드는 인지·판단·제어 분리(규칙 #1·#3)와 ROS-free 코어 원칙에서 **의도적으로
벗어난 실험 노드**다. 컨트롤러가 영상처리를 직접 수행하고 `lane_path`를 쓰지 않는다.
정식 주행 스택(`racer-run`, Pure Pursuit)은 전혀 건드리지 않는다.

@par 원본과 다르게 "적응"한 2가지 (차량별 캘리브레이션 — 파라미터로 노출)
원본 픽셀오차→조향 수식/게인은 그대로지만, 아래 둘은 차량마다 다른 값이라 D-Racer에
맞춘 기본값을 쓰되 파라미터로 바꿀 수 있게 했다(원본 그대로 재현하려면 값만 되돌리면 됨):
- `steering_sign`(기본 +1.0): 조향 부호. 원본 차량은 `steer=-pid/(w/2)`(=-1)였으나
  D-Racer는 조향 부호가 반대(+steer=좌회전, Pure Pursuit로 검증). 원본 그대로면 -1.0.
  ⚠ 이 부호가 틀리면 차선과 **반대로** 꺾여 위험 → 저속 거치대에서 방향 먼저 확인.
- `steering_offset`(기본 0.2238=차량 직진 트림): 키트 control_node는 트림을 안 더하므로
  우리가 실어야 함(규칙 #6). 원본은 자기 차량 트림 -0.05였음. 원본 그대로면 -0.05.

@par 구독/발행
- 구독: `camera/image/compressed`(sensor_msgs/CompressedImage, best-effort) — 키트 camera_node.
- 발행: `/control`(control_msgs/Control) — steering=픽셀PID, throttle=안전게이트(enable_drive).

@par 안전
- 기본 `enable_drive=False` → throttle 항상 0(조향만 검증). True라도 `throttle_limit`로 클램프.
- 프레임을 못 받으면 발행하지 않음(control_node가 마지막값 유지) — 거치대에서 e-stop 상시.
"""
from __future__ import annotations

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from control_msgs.msg import Control


class PixelPIDNode(Node):
    """@brief 카메라 픽셀 차선중심 오차 PID 조향 노드(원본 C++ 1:1 이식, 실험용)."""

    def __init__(self):
        super().__init__('pixel_pid_node')

        # --- 토픽 ---
        self.declare_parameter('image_topic', 'camera/image/compressed')
        self.declare_parameter('control_topic', '/control')

        # --- PID 게인 (원본 C++ 픽셀 게인 그대로) ---
        self.declare_parameter('kp', 0.65)
        self.declare_parameter('ki', 0.008)
        self.declare_parameter('kd', 0.0008)
        self.declare_parameter('integral_limit', 500.0)  # 원본 적분 클램프 ±500.

        # --- 차량별 캘리브레이션 (원본과 다른 2가지, 파라미터로 노출) ---
        self.declare_parameter('steering_sign', 1.0)     # D-Racer=+1(+좌회전). 원본 그대로=-1.
        self.declare_parameter('steering_offset', 0.2238)  # 차량 직진 트림. 원본 그대로=-0.05.

        # --- 차선검출 파라미터 (원본 C++ 값) ---
        self.declare_parameter('yellow_pixel_threshold', 30)  # 노랑 우선 인식 임계.
        self.declare_parameter('nwindows', 9)
        self.declare_parameter('margin', 20)
        self.declare_parameter('minpix', 5)

        # --- 안전 게이트 (controller_node와 동일 원칙) ---
        self.declare_parameter('enable_drive', False)
        self.declare_parameter('drive_throttle', 0.0)
        self.declare_parameter('throttle_limit', 0.15)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.integral_limit = float(self.get_parameter('integral_limit').value)
        self.steering_sign = float(self.get_parameter('steering_sign').value)
        self.steering_offset = float(self.get_parameter('steering_offset').value)
        self.yellow_thr = int(self.get_parameter('yellow_pixel_threshold').value)
        self.nwindows = int(self.get_parameter('nwindows').value)
        self.margin = int(self.get_parameter('margin').value)
        self.minpix = int(self.get_parameter('minpix').value)
        self.enable_drive = bool(self.get_parameter('enable_drive').value)
        self.drive_throttle = float(self.get_parameter('drive_throttle').value)
        self.throttle_limit = abs(float(self.get_parameter('throttle_limit').value))

        # --- PID/EMA 상태 (원본 멤버변수와 동일) ---
        self._integral = 0.0
        self._last_error = 0.0
        self._last_leftx = None    # 첫 프레임에서 width 기준으로 초기화(원본 80/240 등가).
        self._last_rightx = None
        self._M = None             # BEV 변환행렬(첫 프레임에서 계산).

        best_effort_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST)
        control_topic = str(self.get_parameter('control_topic').value)
        image_topic = str(self.get_parameter('image_topic').value)
        self.pub = self.create_publisher(Control, control_topic, 10)
        self.sub = self.create_subscription(
            CompressedImage, image_topic, self.on_image, best_effort_q)

        self._tick = 0
        self.get_logger().info(
            'pixel_pid_node(실험) 시작:\n'
            f'  image={image_topic} → control={control_topic}\n'
            f'  PID kp={self.kp} ki={self.ki} kd={self.kd} (원본 픽셀게인)\n'
            f'  steering_sign={self.steering_sign} (D-Racer +1=좌, 원본 -1) '
            f'steering_offset={self.steering_offset} (직진트림, 원본 -0.05)\n'
            f'  enable_drive={self.enable_drive} drive_throttle={self.drive_throttle} '
            f'throttle_limit={self.throttle_limit}')
        if not self.enable_drive:
            self.get_logger().info('enable_drive=False → throttle=0 (조향만). 거치대에서 방향 먼저 확인.')

    # ------------------------------------------------------------------ #
    def _bev(self, frame):
        """@brief 원본과 동일한 4점 BEV 워핑."""
        h, w = frame.shape[:2]
        if self._M is None:
            src = np.float32([[w * 0.2, h * 0.4], [w * 0.8, h * 0.4], [w, h], [0.0, h]])
            dst = np.float32([[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]])
            self._M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(frame, self._M, (w, h))

    def _lane_edges(self, bev):
        """@brief HLS 노랑/흰 마스크(노랑 우선) → Canny 엣지. 원본과 동일."""
        hls = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)
        yellow = cv2.inRange(hls, (15, 80, 70), (35, 255, 255))
        white = cv2.inRange(hls, (0, 200, 0), (180, 255, 70))
        if cv2.countNonZero(yellow) > self.yellow_thr:
            lane_mask = yellow                      # 노랑 충분 → 노랑만.
        else:
            lane_mask = cv2.bitwise_or(yellow, white)
        masked = cv2.bitwise_and(bev, bev, mask=lane_mask)
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.Canny(blur, 50, 150)

    def _lane_center(self, edges):
        """@brief 히스토그램 + 슬라이딩 윈도우로 좌/우 차선 픽셀 → 중심 반환. 원본과 동일.

        @return (lane_center_px, midpoint_px, leftx, rightx).
        """
        h, w = edges.shape[:2]
        midpoint = w // 2
        if self._last_leftx is None:      # 원본 80/240의 width-일반화.
            self._last_leftx = w * 0.25
            self._last_rightx = w * 0.75

        # 히스토그램: 하단 40% ROI의 열별 엣지 합.
        roi = edges[int(h * 0.6):h, :]
        hist = roi.sum(axis=0)
        leftx = int(np.argmax(hist[:midpoint]))
        rightx = int(np.argmax(hist[midpoint:]) + midpoint)
        initial_leftx, initial_rightx = leftx, rightx

        # EMA(원본): 이전 최종값과 히스토그램 검출을 0.8:0.2로 혼합.
        self._last_leftx = self._last_leftx * 0.8 + leftx * 0.2
        self._last_rightx = self._last_rightx * 0.8 + rightx * 0.2
        leftx = int(self._last_leftx)
        rightx = int(self._last_rightx)

        # 슬라이딩 윈도우: 하단→상단, 각 윈도우 내 엣지 평균으로 재중심.
        nz_y, nz_x = np.nonzero(edges)
        window_height = h // self.nwindows
        margin, minpix = self.margin, self.minpix
        for wi in range(self.nwindows):
            y_low = h - (wi + 1) * window_height
            y_high = h - wi * window_height
            in_y = (nz_y >= y_low) & (nz_y < y_high)
            good_left = in_y & (nz_x >= leftx - margin) & (nz_x < leftx + margin)
            good_right = in_y & (nz_x >= rightx - margin) & (nz_x < rightx + margin)
            if np.count_nonzero(good_left) > minpix:
                leftx = int(np.mean(nz_x[good_left]))
            if np.count_nonzero(good_right) > minpix:
                rightx = int(np.mean(nz_x[good_right]))

        # 급격한 코너 무시(원본): 차선 기울기가 75~80° 밴드면 EMA값으로 되돌림.
        vertical = float(self.nwindows * window_height)
        la = 90.0 - abs(np.degrees(np.arctan2(vertical, float(leftx - initial_leftx))))
        ra = 90.0 - abs(np.degrees(np.arctan2(vertical, float(rightx - initial_rightx))))
        if (75.0 < la < 80.0) or (75.0 < ra < 80.0):
            leftx = int(self._last_leftx)
            rightx = int(self._last_rightx)

        # 최종값을 다음 프레임 EMA 기준으로 저장(원본).
        self._last_leftx = float(leftx)
        self._last_rightx = float(rightx)
        return (leftx + rightx) / 2.0, midpoint, leftx, rightx

    def on_image(self, msg: CompressedImage):
        """@brief 프레임마다: 차선중심 픽셀오차 → 픽셀 PID → /control 발행."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('프레임 디코딩 실패')
            return
        w = frame.shape[1]

        bev = self._bev(frame)
        edges = self._lane_edges(bev)
        lane_center, midpoint, leftx, rightx = self._lane_center(edges)

        # --- 픽셀 PID (원본 그대로) ---
        error = midpoint - lane_center
        p_term = self.kp * error
        self._integral += error
        self._integral = float(np.clip(self._integral, -self.integral_limit, self.integral_limit))
        i_term = self.ki * self._integral
        d_term = self.kd * (error - self._last_error)
        self._last_error = error
        pid_output = p_term + i_term + d_term

        # 정규화(/(width/2)) + 부호 + 트림 오프셋 + 포화. (부호/오프셋만 차량 적응)
        steer = self.steering_sign * (pid_output / (w / 2.0))
        steer += self.steering_offset
        steer = float(np.clip(steer, -1.0, 1.0))

        throttle = 0.0
        if self.enable_drive:
            throttle = float(np.clip(self.drive_throttle, -self.throttle_limit, self.throttle_limit))

        out = Control()
        out.header.stamp = self.get_clock().now().to_msg()
        out.steering = steer
        out.throttle = throttle
        self.pub.publish(out)

        self._tick += 1
        if self._tick % 10 == 0:
            self.get_logger().info(
                f'lane_center={lane_center:.1f}(mid={midpoint}) err={error:+.1f}px '
                f'steer={steer:+.4f} throttle={throttle:.3f} L={leftx} R={rightx}')


def main(args=None):
    rclpy.init(args=args)
    node = PixelPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. 종료.')
    finally:
        # 종료 시 중립 한 번 발행(throttle=0, steering=트림).
        try:
            msg = Control()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.steering = float(node.steering_offset)
            msg.throttle = 0.0
            node.pub.publish(msg)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
