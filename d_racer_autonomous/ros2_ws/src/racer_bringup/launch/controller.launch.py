"""@file controller.launch.py
@brief Stage 6-b: controller_node(core Pure Pursuit + Speed) 실행 런치.

@details
정적 경로에 대한 조향/속도 명령을 계산해 /control 로 발행한다. 키트
control_node 가 `use_joystick_control:=False` 로 떠 있어야 명령이 반영된다.
아직 perception(pose)이 없어 고정 pose(원점) 기준 조향 검증용이다.

@par 사용 예
@code{.sh}
# 조향만 검증(throttle=0, 안전 기본값). 거치대 위에서 앞바퀴 방향 확인.
ros2 launch racer_bringup controller.launch.py path:=straight   # 앞바퀴 직진(중립)
ros2 launch racer_bringup controller.launch.py path:=circle     # 한쪽으로 꺾임
ros2 launch racer_bringup controller.launch.py path:=sharp_s    # 큰 조향

# 저속 구동까지(거치대에서 시작, throttle 하드클램프 0.15):
ros2 launch racer_bringup controller.launch.py path:=circle enable_drive:=True drive_throttle:=0.12
@endcode
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('path', default_value='straight',
                              description='straight|circle|s_curve|sharp_s|figure_eight|rotary'),
        DeclareLaunchArgument('control_topic', default_value='/control'),
        DeclareLaunchArgument('rate_hz', default_value='30.0'),  # 07-10 10→30: 조향루프 지연 감소(smoothing τ=dt/β를 1/3로, 노이즈필터 유지). 중앙추종 안정창 확대. 인지 카메라~26fps라 30Hz 수용.
        DeclareLaunchArgument('enable_drive', default_value='False',
                              description='True여야 throttle 발행. 거치대에서 시작.'),
        DeclareLaunchArgument('drive_throttle', default_value='0.0'),
        DeclareLaunchArgument('throttle_limit', default_value='0.15'),
        DeclareLaunchArgument('pose_x', default_value='0.0'),
        DeclareLaunchArgument('pose_y', default_value='0.0'),
        DeclareLaunchArgument('pose_yaw', default_value='0.0'),
    ]

    node = Node(
        package='racer_bringup',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{
            'path': LaunchConfiguration('path'),
            'control_topic': LaunchConfiguration('control_topic'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'enable_drive': LaunchConfiguration('enable_drive'),
            'drive_throttle': LaunchConfiguration('drive_throttle'),
            'throttle_limit': LaunchConfiguration('throttle_limit'),
            'pose_x': LaunchConfiguration('pose_x'),
            'pose_y': LaunchConfiguration('pose_y'),
            'pose_yaw': LaunchConfiguration('pose_yaw'),
        }],
    )

    return LaunchDescription(args + [node])
