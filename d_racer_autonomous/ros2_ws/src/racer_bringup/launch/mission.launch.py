"""@file mission.launch.py
@brief 미션 노드(mission_node, 12-state) 실행 런치.

@details
`/perception/lane_status` + `/perception/mission_cues`를 구독해
`/decision/drive_command` + `/decision/lane_mode`를 발행한다. 인지 노드가 없으면
lane_status가 안 와서 WAIT_START_SIGNAL/STOP 또는 watchdog LOST만 나가는 게 정상.

@par 사용 예
@code{.sh}
ros2 launch racer_bringup mission.launch.py
ros2 topic echo /decision/lane_mode
ros2 topic echo /decision/drive_command
@endcode
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('lane_status_topic', default_value='/perception/lane_status'),
        DeclareLaunchArgument('mission_cues_topic', default_value='/perception/mission_cues'),
        DeclareLaunchArgument('drive_command_topic', default_value='/decision/drive_command'),
        DeclareLaunchArgument('lane_mode_topic', default_value='/decision/lane_mode'),
        DeclareLaunchArgument('rate_hz', default_value='30.0'),  # 07-10 10→30: 조향루프 지연 감소(smoothing τ=dt/β를 1/3로, 노이즈필터 유지). 중앙추종 안정창 확대. 인지 카메라~26fps라 30Hz 수용.
        DeclareLaunchArgument('lane_timeout', default_value='0.3'),
    ]
    node = Node(
        package='racer_bringup',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'lane_status_topic': LaunchConfiguration('lane_status_topic'),
            'mission_cues_topic': LaunchConfiguration('mission_cues_topic'),
            'drive_command_topic': LaunchConfiguration('drive_command_topic'),
            'lane_mode_topic': LaunchConfiguration('lane_mode_topic'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'lane_timeout': LaunchConfiguration('lane_timeout'),
        }],
    )
    return LaunchDescription(args + [node])
