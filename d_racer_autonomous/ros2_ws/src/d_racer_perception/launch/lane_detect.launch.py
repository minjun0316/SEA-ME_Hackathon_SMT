"""@file lane_detect.launch.py
@brief 차선 인지 노드 단독 실행(config/lane.yaml 로드). 인지 출력 확인용.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('d_racer_perception')
    default_cfg = os.path.join(pkg, 'config', 'lane.yaml')

    cfg_arg = DeclareLaunchArgument('config', default_value=default_cfg,
                                    description='lane_detect_node 파라미터 YAML')

    return LaunchDescription([
        cfg_arg,
        Node(
            package='d_racer_perception',
            executable='lane_detect_node',
            name='lane_detect_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
