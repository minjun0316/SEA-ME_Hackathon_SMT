"""@file race.launch.py
@brief 전체 미션 주행 원샷 런치 (인지 + 판단 + 제어 한번에).

@details
`lane_follow.launch.py`를 그대로 include하면서 **use_mission:=True를 기본으로 켠 것**뿐이다.
노드 정의는 lane_follow 하나에만 두고 여기선 재사용하므로(중복 0), lane_follow를 고치면
race도 자동 반영된다. 즉 이 파일 = "racer-run + 미션 시퀀서(mission_node)".

스택: camera → lane_detect + mission_cues → **mission_node**(출발신호등~도착정지 미션 SM +
YOLO 페이즈 게이트) → controller → control_node. 판단(mission_node)이 /decision/drive_command
+ /decision/lane_mode(YOLO on/off)를 발행한다.

@par 안전
미션 시작 페이즈가 WAIT_START_SIGNAL(go=False)이라 **초록불 확정 전까지 throttle=0**.
enable_drive를 켜도 런치 즉시 튀어나가지 않고 초록불이 출발 스위치가 된다.
그래도 실차 첫 시동은 **거치대→바닥** 순서 권장.

@par 실행
@code{.sh}
# 조향만(거치대에서 로직 확인) — enable_drive 기본 False라 스로틀 0
ros2 launch racer_bringup race.launch.py
# 실차 주행 — 스로틀은 기본값(0.04/0.20 = 07-15 실차 확정)이라 enable_drive만 켜면 된다
ros2 launch racer_bringup race.launch.py enable_drive:=True
@endcode

@note lane_follow의 나머지 인자(pd_* 튜닝 노브, publish_debug, use_* 등)가 필요하면
      lane_follow.launch.py를 직접 쓰고 use_mission:=True를 붙이면 된다(그게 원형).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lane_follow = os.path.join(
        get_package_share_directory('racer_bringup'), 'launch', 'lane_follow.launch.py')

    # 레이스에서 평소 커맨드로 치는 주행 인자만 노출(스로틀 3종 + 자주 쓰는 몇 개).
    # 기본값은 lane_follow와 동일(안전측). 스로틀 값은 사용자가 커맨드로 지정.
    args = [
        DeclareLaunchArgument('enable_drive', default_value='False',
                              description='True 라야 스로틀 발행(기본 조향만). 초록불 게이트가 실제 출발 제어.'),
        # [07-15] 0.0 → 0.04 확정(lane_follow와 동일). FWD_START_US=1575 기준 ≈1592µs
        # = 이 차의 최저 주행속. 근거·실측표: docs/calibration.md 3-b절.
        DeclareLaunchArgument('drive_throttle', default_value='0.04',
                              description='주행 스로틀(정규화). 07-15 실차 확정 0.04(≈1592µs).'),
        DeclareLaunchArgument('throttle_limit', default_value='0.20',
                              description='스로틀 하드 클램프 상한.'),
        DeclareLaunchArgument('stopline_maneuver', default_value='False',
                              description='정지선 카운트→개루프 고정스티어 기동(로터리 진입/탈출). '
                                          '[2026-07-14] 흰선 폐루프 코스=로터리 없음 → 기본 OFF. '
                                          '로터리 코스 복귀 시 stopline_maneuver:=True.'),
        DeclareLaunchArgument('lateral_controller', default_value='lateral_pd',
                              description='횡제어 법칙: lateral_pd(기본) | pure_pursuit.'),
    ]

    full_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lane_follow),
        launch_arguments={
            'use_mission': 'True',                                    # ← 이 런치의 핵심: 판단 노드 항상 포함.
            'enable_drive': LaunchConfiguration('enable_drive'),
            'drive_throttle': LaunchConfiguration('drive_throttle'),
            'throttle_limit': LaunchConfiguration('throttle_limit'),
            'stopline_maneuver': LaunchConfiguration('stopline_maneuver'),
            'lateral_controller': LaunchConfiguration('lateral_controller'),
        }.items())

    return LaunchDescription(args + [full_mission])
