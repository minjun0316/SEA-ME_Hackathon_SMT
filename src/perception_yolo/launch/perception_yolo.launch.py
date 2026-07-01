from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_yolo',
            executable='yolo_seg_node',
            name='yolo_seg_node',
            output='screen',
            parameters=[{
                'image_topic': 'camera/image/compressed',
                # ★ TODO: best.pt 실제 경로로 수정 (D3G 기준 절대경로 권장)
                #    예) /home/topst/SEA-ME_Hackathon_SMT/YOLOv26n_seg/models/best.pt
                'model_path': '/home/topst/SEA-ME_Hackathon_SMT/YOLOv26n_seg/models/best.pt',
                'conf': 0.25,      # 신뢰도 임계값 (대회 중 튜닝)
                'imgsz': 640,
                'publish_debug': True,
            }],
        ),
    ])
