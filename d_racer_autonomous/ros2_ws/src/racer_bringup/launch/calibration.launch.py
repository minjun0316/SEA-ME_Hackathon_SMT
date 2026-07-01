"""@file calibration.launch.py
@brief 캘리브레이션 노드 실행 런치.

@details
control_node 등 D-Racer 기본 노드는 별도로(manual_driving 계열) 띄워야
하며, 이 런치는 캘리브레이션 노드만 올린다. control_node가
use_joystick_control=False 로 떠 있어야 /control 명령이 반영된다.

@par 사용 예
@code{.sh}
# 기본(hold, steering=0, throttle=0 — 안전)
ros2 launch racer_bringup calibration.launch.py

# 조향 스윕(중심/최대각 측정), throttle=0
ros2 launch racer_bringup calibration.launch.py mode:=steer_sweep

# throttle 데드존 측정(작은 펄스, 하드 클램프 0.15)
ros2 launch racer_bringup calibration.launch.py mode:=throttle_pulse pulse_throttle:=0.12
@endcode
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mode', default_value='hold',
                              description='hold | steer_sweep | throttle_pulse'),
        DeclareLaunchArgument('control_topic', default_value='/control'),
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('throttle_limit', default_value='0.15'),
        DeclareLaunchArgument('steering', default_value='0.0'),
        DeclareLaunchArgument('throttle', default_value='0.0'),
        DeclareLaunchArgument('sweep_amplitude', default_value='1.0'),
        DeclareLaunchArgument('sweep_period', default_value='4.0'),
        DeclareLaunchArgument('pulse_throttle', default_value='0.10'),
    ]

    node = Node(
        package='racer_bringup',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'control_topic': LaunchConfiguration('control_topic'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'throttle_limit': LaunchConfiguration('throttle_limit'),
            'steering': LaunchConfiguration('steering'),
            'throttle': LaunchConfiguration('throttle'),
            'sweep_amplitude': LaunchConfiguration('sweep_amplitude'),
            'sweep_period': LaunchConfiguration('sweep_period'),
            'pulse_throttle': LaunchConfiguration('pulse_throttle'),
        }],
    )

    return LaunchDescription(args + [node])
