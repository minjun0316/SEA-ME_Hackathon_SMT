"""@file lane_follow_pixel.launch.py
@brief [실험] 원본 카메라 픽셀-PID 라인트래킹 폐루프 (racer-run-ex 전용).

@details
스택: camera_node → **pixel_pid_node** → control_node(키트 서보/모터).
정식 `lane_follow.launch.py`(인지 lane_path + Pure Pursuit)와 달리, 여기서는
`pixel_pid_node`가 카메라 압축영상을 직접 받아 BEV+슬라이딩윈도우로 차선중심
픽셀오차를 뽑고 **원본 C++ 픽셀 게인 PID**로 조향한다. lane_detect/decision 불필요.

@par 실행
```bash
# 조향만(throttle=0) — 방향(부호) 먼저 눈으로 확인
ros2 launch racer_bringup lane_follow_pixel.launch.py
# 저속 주행: enable_drive:=True drive_throttle:=0.16 throttle_limit:=0.20
# 게인/부호 즉석 조정: kp:=0.7 steering_sign:=-1.0 steering_offset:=0.2238
```

@par 안전
- 기본 enable_drive=False(throttle=0). pixel_pid_node가 throttle_limit로 하드 클램프.
- 조향 부호(steering_sign)가 틀리면 차선과 반대로 꺾이니 거치대 저속에서 방향부터 확인.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_camera', default_value='True',
                              description='키트 camera_node 도 함께 띄울지'),
        DeclareLaunchArgument('use_control', default_value='True',
                              description='키트 control_node(서보/모터) 도 함께 띄울지'),
        DeclareLaunchArgument('use_battery', default_value='True',
                              description='battery_node(전압 감시)'),
        DeclareLaunchArgument('use_monitor', default_value='True',
                              description='monitor_node(웹 UI)'),
        DeclareLaunchArgument('control_topic', default_value='/control'),
        DeclareLaunchArgument('image_topic', default_value='camera/image/compressed'),
        # --- 안전 게이트 ---
        DeclareLaunchArgument('enable_drive', default_value='False',
                              description='True 라야 스로틀 발행(기본 조향만)'),
        DeclareLaunchArgument('drive_throttle', default_value='0.0'),
        DeclareLaunchArgument('throttle_limit', default_value='0.15'),
        # --- PID 게인(원본 픽셀) / 차량 적응 ---
        DeclareLaunchArgument('kp', default_value='0.65'),
        DeclareLaunchArgument('ki', default_value='0.008'),
        DeclareLaunchArgument('kd', default_value='0.0008'),
        DeclareLaunchArgument('steering_sign', default_value='1.0',
                              description='D-Racer=+1(+좌회전). 원본 그대로=-1.0'),
        DeclareLaunchArgument('steering_offset', default_value='0.2238',
                              description='직진 트림. 원본 그대로=-0.05'),
    ]

    camera = Node(
        package='camera', executable='camera_node', name='camera_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    pixel_pid = Node(
        package='racer_bringup', executable='pixel_pid_node', name='pixel_pid_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'control_topic': LaunchConfiguration('control_topic'),
            'enable_drive': LaunchConfiguration('enable_drive'),
            'drive_throttle': LaunchConfiguration('drive_throttle'),
            'throttle_limit': LaunchConfiguration('throttle_limit'),
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'steering_sign': LaunchConfiguration('steering_sign'),
            'steering_offset': LaunchConfiguration('steering_offset'),
        }],
    )

    control = Node(
        package='control', executable='control_node', name='control_node',
        output='screen',
        parameters=[{'use_joystick_control': False}],
        condition=IfCondition(LaunchConfiguration('use_control')),
    )

    battery = Node(
        package='battery', executable='battery_node', name='battery_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_battery')),
    )

    monitor = Node(
        package='monitor', executable='monitor_node', name='monitor_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_monitor')),
    )

    return LaunchDescription(args + [camera, pixel_pid, control, battery, monitor])
