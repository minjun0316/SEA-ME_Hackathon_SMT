"""@file bench_calibration.launch.py
@brief 거치대 캘리브레이션 통합 런치 (보드에서 실행).

@details
액추에이터 캘리브레이션에 필요한 노드를 한 번에 띄운다.
- control_node (use_joystick_control=False) : /control 을 PCA9685로 전달.
- joystick_node                              : E-STOP 안전장치(필수).
- battery_node                               : 전압 모니터(선택).
- calibration_node                           : 우리의 명령 발행 노드.

카메라/인지는 포함하지 않는다(캘리브레이션엔 불필요).

@note 이 런치는 D-Racer 보드에서 control/joystick/battery 패키지가 빌드된
      상태로 실행해야 한다(개발 PC에는 해당 패키지가 없을 수 있음).

@par 사용 예 (보드에서)
@code{.sh}
ros2 launch racer_bringup bench_calibration.launch.py mode:=steer_sweep
@endcode
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    throttle_limit = LaunchConfiguration('throttle_limit')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='hold',
                              description='hold | steer_sweep | throttle_pulse'),
        DeclareLaunchArgument('throttle_limit', default_value='0.15'),
        DeclareLaunchArgument('pulse_throttle', default_value='0.10'),
        DeclareLaunchArgument('sweep_amplitude', default_value='1.0'),

        # D-Racer 모터 제어 노드: 자동 모드(/control 추종).
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{'use_joystick_control': False}],
        ),
        # E-STOP 안전장치 (control_node가 e_stop_en을 최우선 처리).
        Node(
            package='joystick',
            executable='joystick_node',
            name='gamepad_publisher',
            output='screen',
            parameters=[{'calibration_mode': False}],
        ),
        # 배터리 모니터(선택).
        Node(
            package='battery',
            executable='battery_node',
            name='battery_node',
            output='screen',
        ),
        # 우리의 캘리브레이션 명령 발행 노드.
        Node(
            package='racer_bringup',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            parameters=[{
                'mode': mode,
                'throttle_limit': throttle_limit,
                'pulse_throttle': LaunchConfiguration('pulse_throttle'),
                'sweep_amplitude': LaunchConfiguration('sweep_amplitude'),
            }],
        ),
    ])
