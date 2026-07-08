import os
from pathlib import Path

import rclpy
from rclpy.node import Node
import yaml

from control_msgs.msg import Control
from joystick_msgs.msg import Joystick
from topst_utils.d3racer import D3Racer


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/D-Racer/src/config/vehicle_config.yaml'


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # ROS parameters
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('pca9685_addr', 0x40)
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('throttle_channel', 1)
        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('use_joystick_control', False)
        self.declare_parameter('joystick_topic', 'joystick')
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('command_hz', 10.0)
        # watchdog: 명령이 이 시간[s] 이상 안 오면 throttle=0(fail-safe 정지).
        self.declare_parameter('cmd_timeout', 0.5)

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        pca9685_addr = int(self.get_parameter('pca9685_addr').value)
        steering_channel = int(self.get_parameter('steering_channel').value)
        throttle_channel = int(self.get_parameter('throttle_channel').value)
        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        self.use_joystick_control = bool(self.get_parameter('use_joystick_control').value)
        joystick_topic = str(self.get_parameter('joystick_topic').value)
        control_topic = str(self.get_parameter('control_topic').value)
        command_hz = float(self.get_parameter('command_hz').value)
        if command_hz <= 0.0:
            raise ValueError('command_hz must be greater than 0')

        self.command_hz = command_hz
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.steer_trim = self.load_steer_trim()

        self.d3_racer = D3Racer(
            i2c_bus=i2c_bus,
            pca9685_addr=pca9685_addr,
            steering_channel=steering_channel,
            throttle_channel=throttle_channel,
        )

        self.get_logger().info(
            'd3_racer configured:\n'
            f'  i2c_bus={i2c_bus}\n'
            f'  pca9685_addr=0x{pca9685_addr:02X}\n'
            f'  steering_channel={steering_channel}\n'
            f'  throttle_channel={throttle_channel}\n'
            f'  steer_trim={self.steer_trim}\n'
            f'  use_joystick_control={self.use_joystick_control}\n'
            f'  joystick_topic={joystick_topic}\n'
            f'  control_topic={control_topic}\n'
            f'  command_hz={self.command_hz}\n'
            f'  cmd_timeout={self.cmd_timeout}s (명령 끊기면 정지)\n'
            f'  vehicle_config_file={self.vehicle_config_file}'
        )

        self.throttle = 0.0
        self.steering = self.steer_trim
        self.e_stop_active = False
        # watchdog 상태: 마지막 명령 수신 시각(ns). None이면 아직 명령 없음→정지 유지.
        self.last_cmd_ns = None
        self._watchdog_tripped = False

        # Control inputs
        self.create_subscription(
            Joystick,
            joystick_topic,
            self.joystick_callback,
            10,
        )
        self.create_subscription(
            Control,
            control_topic,
            self.control_callback,
            10,
        )

        # Command output loop
        self.timer = self.create_timer(1.0 / self.command_hz, self.timer_callback)

    def timer_callback(self):
        if self.e_stop_active:
            self.apply_actuation(self.steering, 0.0)
            return

        # watchdog: 명령 스트림이 cmd_timeout 넘게 끊기면 정지(조향은 유지).
        # 상위 스택이 죽거나 Ctrl+C로 사라져도 마지막 throttle을 물고 달리지 않도록.
        if self._command_stale():
            if not self._watchdog_tripped:
                self._watchdog_tripped = True
                self.get_logger().warning(
                    f'명령 {self.cmd_timeout:.1f}s 이상 끊김 → watchdog 정지(throttle=0)')
            self.apply_actuation(self.steering, 0.0)
            return

        self.apply_actuation(self.steering, self.throttle)

    def _command_stale(self):
        """@brief 마지막 명령 이후 cmd_timeout 초과했는지(=정지해야 하는지)."""
        if self.last_cmd_ns is None:
            return True
        elapsed = (self.get_clock().now().nanoseconds - self.last_cmd_ns) * 1e-9
        return elapsed > self.cmd_timeout

    def _mark_command(self):
        """@brief 유효 주행명령 수신 시각 갱신 + watchdog 해제."""
        self.last_cmd_ns = self.get_clock().now().nanoseconds
        if self._watchdog_tripped:
            self._watchdog_tripped = False
            self.get_logger().info('명령 복구 → watchdog 해제(주행 재개)')

    def apply_actuation(self, steering, throttle):
        self.d3_racer.set_steering_percent(float(steering))
        self.d3_racer.set_throttle_percent(float(throttle))

    def joystick_callback(self, msg: Joystick):
        if bool(msg.e_stop_en):
            self.engage_e_stop()
            return

        if self.e_stop_active or not self.use_joystick_control:
            return

        self.steering = float(msg.control_msg.steering)
        self.throttle = float(msg.control_msg.throttle)
        self._mark_command()

    def control_callback(self, msg: Control):
        if self.e_stop_active or self.use_joystick_control:
            return

        self.steering = float(msg.steering)
        self.throttle = float(msg.throttle)
        self._mark_command()

    def engage_e_stop(self):
        if self.e_stop_active:
            return

        self.e_stop_active = True
        self.throttle = 0.0
        self.apply_actuation(self.steering, 0.0)
        self.get_logger().warning('E-STOP engaged. Ignoring incoming throttle commands.')

    def load_steer_trim(self):
        if not os.path.exists(self.vehicle_config_file):
            return 0.0

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                config_data = yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return 0.0

        return float(config_data.get('STEER_TRIM', 0.0))

    def destroy_node(self):
        try:
            if hasattr(self, 'd3_racer') and self.d3_racer is not None:
                self.apply_actuation(self.steer_trim, 0.0)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
