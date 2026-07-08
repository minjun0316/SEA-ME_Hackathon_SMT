"""@file lane_follow.launch.py
@brief 카메라 차선 추종 폐루프 (YOLO/미션 제외) — 실차 주행 확인용.

@details
스택: camera_node → lane_detect_node → controller_node(source=topic) → control_node(키트 서보/모터).
이제 **키트 액추에이터 노드까지 이 런치가 포함**한다(T1 따로 띄울 필요 없음).
텔레메트리로 battery_node(전압 감시)·monitor_node(웹 UI)도 기본 포함(각각 use_battery/use_monitor 로 끔).
YOLO 객체인식은 use_yolo(기본 True)로 함께 띄운다(관측·디버그용, 주행엔 무관).
아루코 인지(mission_cues_node)는 use_mission_cues(기본 True)로 띄운다(발행만).
미션 시퀀서(mission_node)는 **제외** — 차선 추종 + 아루코 정지만 검증한다.
판단(decision_node)은 기본 off. **아루코 보이면 정지시키려면 use_decision:=True** 로 켠다
(decision이 mission_cues.aruco_present→stop_request→STOP 게이트를 실행).

@par 실행
```bash
# 조향만(throttle=0) — 배관/조향 먼저 눈으로 확인 (camera+lane+controller+키트control 한번에)
ros2 launch racer_bringup lane_follow.launch.py
# 저속 주행까지(거치대→바닥): enable_drive:=True drive_throttle:=0.16 throttle_limit:=0.20
# 차선추종 + 아루코(ID3) 보이면 정지: use_decision:=True 추가
ros2 launch racer_bringup lane_follow.launch.py enable_drive:=True \
    drive_throttle:=0.16 throttle_limit:=0.20 use_decision:=True
# 카메라 이미 따로 띄웠으면: use_camera:=False / 키트 control 따로면: use_control:=False
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
    yolo_cfg = os.path.join(perc_pkg, 'config', 'yolo_detect_test.yaml')
    cues_cfg = os.path.join(perc_pkg, 'config', 'mission_cues.yaml')

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
        DeclareLaunchArgument('use_yolo', default_value='True',
                              description='yolo_detect_test_node(객체인식 디버그). 웹 YOLO 패널용, 끄려면 False'),
        DeclareLaunchArgument('use_mission_cues', default_value='True',
                              description='mission_cues_node(아루코 ID3 검출→정지신호). 정지엔 use_decision:=True 필요'),
        DeclareLaunchArgument('lane_config', default_value=lane_cfg,
                              description='lane_detect_node 파라미터 YAML'),
        DeclareLaunchArgument('mission_cues_config', default_value=cues_cfg,
                              description='mission_cues_node 파라미터 YAML(ArUco 사전/ID)'),
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

    # 2b) 미션신호 인지(옵션): 아루코 ID3 검출 → /perception/mission_cues.
    mission_cues = Node(
        package='d_racer_perception',
        executable='mission_cues_node',
        name='mission_cues_node',
        output='screen',
        parameters=[LaunchConfiguration('mission_cues_config')],
        condition=IfCondition(LaunchConfiguration('use_mission_cues')),
    )

    # 3) 판단(옵션): 반응형 SM + 아루코 정지 게이트(mission_cues.aruco_present→STOP).
    #    마커 보이면 정지시키려면 use_decision:=True 로 이 노드를 켜야 한다.
    decision = Node(
        package='racer_bringup',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'mission_cues_topic': '/perception/mission_cues',
            'aruco_stop_enable': True,
        }],
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

    # 8) YOLO 객체인식(옵션): camera → best 모델 추론 → 디버그 이미지 발행(웹 YOLO 패널).
    #    config/yolo_detect_test.yaml 로 모델(best_ncnn_model)/imgsz(320) 등을 물린다.
    #    노드명은 yaml 최상위 키(yolo_detect_test_node)와 일치해야 파라미터가 적용된다.
    yolo = Node(
        package='d_racer_perception',
        executable='yolo_detect_test_node',
        name='yolo_detect_test_node',
        output='screen',
        parameters=[yolo_cfg],
        condition=IfCondition(LaunchConfiguration('use_yolo')),
    )

    return LaunchDescription(
        args + [camera, lane, mission_cues, decision, controller,
                control, battery, monitor, yolo])
