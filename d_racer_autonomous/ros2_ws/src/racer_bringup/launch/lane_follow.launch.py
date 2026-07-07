"""@file lane_follow.launch.py
@brief 카메라 차선 추종 폐루프 (YOLO/미션 제외) — 실차 주행 확인용.

@details
스택: camera_node → lane_detect_node → decision_node → controller_node(source=topic).
YOLO 객체인식·미션(mission_node)은 **제외** — 차선 추종만 검증한다.

@par 전제 (별도 터미널)
- T1: 키트 모터 제어 노드 —
  `ros2 run control control_node --ros-args -p use_joystick_control:=False`
  (이게 켜져 있어야 /control 이 서보/모터로 나감)

@par 실행
```bash
# 조향만(throttle=0) — 배관/조향 먼저 눈으로 확인
ros2 launch racer_bringup lane_follow.launch.py
# 저속 주행까지(거치대→바닥): enable_drive:=True drive_throttle:=0.12
ros2 launch racer_bringup lane_follow.launch.py enable_drive:=True drive_throttle:=0.12
# 카메라 노드 이미 따로 띄웠으면: use_camera:=False
```

@par 안전
- 인지가 lane_path 를 못 주거나(차선 소실) 판단이 STOP/LOST 면 controller watchdog →
  throttle=0(조향 유지). enable_drive 기본 False(스로틀 0).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perc_pkg = get_package_share_directory('d_racer_perception')
    lane_cfg = os.path.join(perc_pkg, 'config', 'lane.yaml')

    args = [
        DeclareLaunchArgument('use_camera', default_value='True',
                              description='키트 camera_node 도 함께 띄울지'),
        DeclareLaunchArgument('lane_config', default_value=lane_cfg,
                              description='lane_detect_node 파라미터 YAML'),
        DeclareLaunchArgument('control_topic', default_value='/control'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('enable_drive', default_value='False',
                              description='True 라야 스로틀 발행(기본 조향만)'),
        DeclareLaunchArgument('drive_throttle', default_value='0.0'),
        DeclareLaunchArgument('throttle_limit', default_value='0.15'),
        DeclareLaunchArgument('lane_timeout', default_value='0.3',
                              description='lane_path/판단 끊김 판정[s] → 정지'),
    ]

    # 1) 카메라(키트) — 옵션
    camera = Node(
        package='camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # 2) 인지: 차선 검출 → lane_status + lane_path
    lane = Node(
        package='d_racer_perception',
        executable='lane_detect_node',
        name='lane_detect_node',
        output='screen',
        parameters=[LaunchConfiguration('lane_config')],
    )

    # 3) 판단: 아래층 반응형(lane_status만) → drive_command
    decision = Node(
        package='racer_bringup',
        executable='decision_node',
        name='decision_node',
        output='screen',
    )

    # 4) 제어: topic 모드(lane_path + drive_command) → /control
    controller = Node(
        package='racer_bringup',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{
            'source': 'topic',
            'control_topic': LaunchConfiguration('control_topic'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'enable_drive': LaunchConfiguration('enable_drive'),
            'drive_throttle': LaunchConfiguration('drive_throttle'),
            'throttle_limit': LaunchConfiguration('throttle_limit'),
            'lane_timeout': LaunchConfiguration('lane_timeout'),
        }],
    )

    return LaunchDescription(args + [camera, lane, decision, controller])
