"""@file lane_follow.launch.py
@brief 카메라 차선 추종 폐루프 (YOLO/미션 제외) — 실차 주행 확인용.

@details
스택: camera_node → lane_detect_node → controller_node(source=topic) → control_node(키트 서보/모터).
이제 **키트 액추에이터 노드까지 이 런치가 포함**한다(T1 따로 띄울 필요 없음).
텔레메트리로 battery_node(전압 감시)·monitor_node(웹 UI)도 기본 포함(각각 use_battery/use_monitor 로 끔).
YOLO 객체인식·미션(mission_node)은 **제외** — 차선 추종만 검증한다.
판단(decision_node)은 라인트래킹엔 불필요하므로 기본 off(use_decision:=True 로 켬).

@par 실행
```bash
# 조향만(throttle=0) — 배관/조향 먼저 눈으로 확인 (camera+lane+controller+키트control 한번에)
ros2 launch racer_bringup lane_follow.launch.py
# 저속 주행까지(거치대→바닥): enable_drive:=True drive_throttle:=0.12
ros2 launch racer_bringup lane_follow.launch.py enable_drive:=True drive_throttle:=0.12
# 카메라 이미 따로 띄웠으면: use_camera:=False
# 키트 control_node 이미 따로 띄웠으면: use_control:=False
# 판단 게이트(정지선/색선택)까지 붙이려면: use_decision:=True
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
        DeclareLaunchArgument('use_control', default_value='True',
                              description='키트 control_node(서보/모터 액추에이터) 도 함께 띄울지'),
        DeclareLaunchArgument('use_decision', default_value='False',
                              description='판단 게이트(정지선/색선택). 라인트래킹엔 불필요, 기본 off'),
        DeclareLaunchArgument('use_battery', default_value='True',
                              description='battery_node(전압 감시). 저전압 컷오프 대비 기본 on'),
        DeclareLaunchArgument('use_monitor', default_value='True',
                              description='monitor_node(웹 UI: 카메라/디버그/조향 시각화)'),
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

    # 3) 판단(옵션): 아래층 반응형(lane_status만) → drive_command. 라인트래킹엔 불필요.
    decision = Node(
        package='racer_bringup',
        executable='decision_node',
        name='decision_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_decision')),
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

    # 5) 키트 액추에이터(옵션): /control → 서보/모터. use_joystick_control=False 라야 /control 을 씀.
    control = Node(
        package='control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{'use_joystick_control': False}],
        condition=IfCondition(LaunchConfiguration('use_control')),
    )

    # 6) 배터리 감시(옵션): 전압 발행. 저전압 컷오프 조기 감지용.
    battery = Node(
        package='battery',
        executable='battery_node',
        name='battery_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_battery')),
    )

    # 7) 모니터(옵션): 웹 UI — 카메라/차선디버그/조향값 시각화(주행엔 무관, 관측용).
    monitor = Node(
        package='monitor',
        executable='monitor_node',
        name='monitor_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitor')),
    )

    return LaunchDescription(
        args + [camera, lane, decision, controller, control, battery, monitor])
