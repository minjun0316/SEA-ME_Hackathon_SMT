from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_default_model_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'models' / 'yolo_seg_test.pt'
        if candidate.exists():
            return str(candidate)
    return 'yolo_seg_test.pt'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='camera/image/compressed'),
        DeclareLaunchArgument('model_path', default_value=get_default_model_path()),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('publish_debug', default_value='True'),
        Node(
            package='d_racer_perception',
            executable='yolo_seg_test_node',
            name='yolo_seg_test_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'conf': LaunchConfiguration('conf'),
                'imgsz': LaunchConfiguration('imgsz'),
                'publish_debug': LaunchConfiguration('publish_debug'),
            }],
        ),
    ])
