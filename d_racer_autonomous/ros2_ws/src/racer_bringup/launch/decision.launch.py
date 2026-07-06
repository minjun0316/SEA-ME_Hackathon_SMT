"""@file decision.launch.py
@brief 판단 노드(decision_node) 실행 런치.

@details
`/perception/lane_status`를 구독해 `/decision/drive_command`를 발행한다.
아직 인지 노드가 없으면 lane_status가 안 와서 watchdog로 LOST(정지) 명령만
나가는 게 정상이다(fail-safe 확인용).

@par 사용 예
@code{.sh}
# 판단 노드만 실행(발행 확인).
ros2 launch racer_bringup decision.launch.py

# 다른 토픽명/주기로:
ros2 launch racer_bringup decision.launch.py rate_hz:=20.0 lane_timeout:=0.5

# 발행 확인:
ros2 topic echo /decision/drive_command
@endcode
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('lane_status_topic',
                              default_value='/perception/lane_status'),
        DeclareLaunchArgument('drive_command_topic',
                              default_value='/decision/drive_command'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('lane_timeout', default_value='0.3',
                              description='이 시간 넘게 lane_status 없으면 LOST(정지).'),
    ]

    node = Node(
        package='racer_bringup',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'lane_status_topic': LaunchConfiguration('lane_status_topic'),
            'drive_command_topic': LaunchConfiguration('drive_command_topic'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'lane_timeout': LaunchConfiguration('lane_timeout'),
        }],
    )

    return LaunchDescription(args + [node])
