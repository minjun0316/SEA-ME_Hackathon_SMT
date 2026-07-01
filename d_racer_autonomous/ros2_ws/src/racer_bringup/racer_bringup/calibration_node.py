"""@file calibration_node.py
@brief Stage 6-a: 실차 액추에이터 브링업 / 캘리브레이션 노드.

@details
실차 연결의 가장 안전한 첫 단계. Pure Pursuit/경로추종을 돌리기 전에
"내 ROS2 노드 → control_node → PCA9685 → 서보/ESC" 배관이 동작하는지
확인하고, 이후 모든 제어가 의존하는 실제 액추에이터 파라미터를 측정한다.

@par 측정 목표 (거치대 위, 바퀴를 띄운 상태에서)
- 서보 조향 중심값(STEER_TRIM): 바퀴가 정확히 직진하는 steering 명령.
- 최대 조향각(max_steer_deg): steering=±1.0일 때 실제 바퀴 각도(각도기 측정).
- throttle 중립/데드존: 바퀴가 막 돌기 시작하는 throttle 크기.

@par 안전 설계 (실모터 구동 — 매우 중요)
- throttle은 항상 `throttle_limit`(기본 0.15)로 하드 클램프된다.
- 기본 모드는 hold(steering=0, throttle=0) — 실행만으로는 아무 일도 안 일어남.
- D-Racer joystick e-stop이 control_node에서 최우선 처리되므로 항상 손에 둘 것.
- 반드시 **차를 거치대에 올려 바퀴를 띄운 상태**에서 시작.

@par 발행
- control_msgs/Control on `control_topic` (기본 /control), `rate_hz`로 주기 발행.
- D-Racer control_node는 use_joystick_control=False일 때 이 토픽을 따른다.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from control_msgs.msg import Control


class CalibrationNode(Node):
    """@brief 액추에이터 캘리브레이션용 Control 명령 발행 노드."""

    def __init__(self):
        super().__init__('calibration_node')

        # --- 파라미터 (모두 YAML/CLI로 변경 가능, 하드코딩 없음) ---
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('rate_hz', 20.0)
        # mode: hold | steer_sweep | throttle_pulse
        self.declare_parameter('mode', 'hold')
        # 안전 한계: throttle 크기는 절대 이 값을 넘지 않음.
        self.declare_parameter('throttle_limit', 0.15)
        self.declare_parameter('steer_limit', 1.0)
        # hold 모드 고정값.
        self.declare_parameter('steering', 0.0)
        self.declare_parameter('throttle', 0.0)
        # steer_sweep: steering = amplitude * sin(2π t / period), throttle=0.
        self.declare_parameter('sweep_amplitude', 1.0)
        self.declare_parameter('sweep_period', 4.0)
        # throttle_pulse: steering=0, throttle를 on/off 반복(데드존 탐색).
        self.declare_parameter('pulse_throttle', 0.10)
        self.declare_parameter('pulse_on', 1.0)
        self.declare_parameter('pulse_off', 2.0)

        self.control_topic = str(self.get_parameter('control_topic').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.mode = str(self.get_parameter('mode').value)
        self.throttle_limit = abs(float(self.get_parameter('throttle_limit').value))
        self.steer_limit = abs(float(self.get_parameter('steer_limit').value))
        self.steering = float(self.get_parameter('steering').value)
        self.throttle = float(self.get_parameter('throttle').value)
        self.sweep_amplitude = float(self.get_parameter('sweep_amplitude').value)
        self.sweep_period = max(0.1, float(self.get_parameter('sweep_period').value))
        self.pulse_throttle = float(self.get_parameter('pulse_throttle').value)
        self.pulse_on = float(self.get_parameter('pulse_on').value)
        self.pulse_off = float(self.get_parameter('pulse_off').value)

        if self.rate_hz <= 0.0:
            raise ValueError('rate_hz must be > 0')

        self.publisher = self.create_publisher(Control, self.control_topic, 10)
        self.elapsed = 0.0
        self.dt = 1.0 / self.rate_hz
        self._last_logged = None  ##< 마지막으로 로그한 (steering, throttle) — 변화 시에만 로그.
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().warn(
            '\n=== CALIBRATION NODE (실모터 구동) ===\n'
            '  ▶ 차를 거치대에 올려 바퀴를 띄웠는지 확인하세요.\n'
            '  ▶ joystick E-STOP을 항상 손에 두세요.\n'
            f'  topic={self.control_topic}  mode={self.mode}  rate={self.rate_hz}Hz\n'
            f'  throttle_limit=±{self.throttle_limit} (하드 클램프)\n'
            '====================================')

    def _clamp_steer(self, v: float) -> float:
        """@brief 조향을 [-steer_limit, +steer_limit]로 제한."""
        return max(-self.steer_limit, min(self.steer_limit, v))

    def _clamp_throttle(self, v: float) -> float:
        """@brief throttle을 안전 한계 [-throttle_limit, +throttle_limit]로 제한."""
        return max(-self.throttle_limit, min(self.throttle_limit, v))

    def compute_command(self) -> tuple[float, float]:
        """@brief 현재 모드/시간에 따른 (steering, throttle) 명령 계산.

        @return (steering, throttle) — 둘 다 안전 한계로 클램프된 값.
        """
        if self.mode == 'steer_sweep':
            # 조향만 천천히 좌우로 흔든다(throttle=0). 중심/최대각 측정용.
            steer = self.sweep_amplitude * math.sin(
                2.0 * math.pi * self.elapsed / self.sweep_period)
            return self._clamp_steer(steer), 0.0

        if self.mode == 'throttle_pulse':
            # 작은 throttle을 on/off 반복(steering=0). 데드존/중립 측정용.
            cycle = self.pulse_on + self.pulse_off
            phase = self.elapsed % cycle if cycle > 0 else 0.0
            thr = self.pulse_throttle if phase < self.pulse_on else 0.0
            return 0.0, self._clamp_throttle(thr)

        # 기본: hold. steering/throttle 파라미터를 매 틱 실시간으로 읽어,
        # 노드 재시작 없이 `ros2 param set` 으로 즉석 캘리브레이션이 가능하다.
        # (throttle은 항상 안전 한계로 클램프되므로 param으로도 한계 초과 불가.)
        steering = float(self.get_parameter('steering').value)
        throttle = float(self.get_parameter('throttle').value)
        return self._clamp_steer(steering), self._clamp_throttle(throttle)

    def timer_callback(self):
        """@brief 주기적으로 Control 명령을 계산해 발행한다."""
        steering, throttle = self.compute_command()

        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.steering = float(steering)
        msg.throttle = float(throttle)
        self.publisher.publish(msg)

        self.elapsed += self.dt
        # 로그 폭주 방지: 명령이 바뀔 때만 출력(0.01 단위로 반올림 비교).
        key = (round(steering, 2), round(throttle, 2))
        if key != self._last_logged:
            self._last_logged = key
            self.get_logger().info(
                f'[{self.mode}] steering={steering:+.3f} throttle={throttle:+.3f}')

    def destroy_node(self):
        """@brief 종료 시 안전하게 정지 명령(0,0)을 한 번 발행."""
        try:
            stop = Control()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.steering = 0.0
            stop.throttle = 0.0
            self.publisher.publish(stop)
            self.get_logger().info('Sent stop command (0, 0).')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — stopping.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
