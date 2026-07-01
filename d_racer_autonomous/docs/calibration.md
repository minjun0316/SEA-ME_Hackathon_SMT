# Stage 6-a: 실차 액추에이터 캘리브레이션 가이드

> 목적: Pure Pursuit/경로추종을 올리기 **전에**, ROS2 명령 배관을 검증하고
> 이후 모든 제어가 의존하는 실제 액추에이터 값을 측정한다.

## ⚠ 안전 수칙 (먼저 읽기)
1. **차를 거치대에 올려 바퀴를 완전히 띄운 상태**에서만 시작한다.
2. **joystick E-STOP을 항상 손에** 둔다. (control_node가 `e_stop_en`을 최우선 처리)
3. throttle은 노드에서 `throttle_limit`(기본 0.15)로 하드 클램프된다. 처음엔 절대 올리지 않는다.
4. 기본 모드 `hold`는 steering=0/throttle=0이라 실행만으로는 움직이지 않는다.

---

## 0. 배포 (개발 PC → 보드)
보드에는 이미 `control_msgs`가 있으므로 우리 패키지만 복사한다.
```bash
# (개발 PC에서) racer_bringup 패키지를 보드로 복사
scp -r ros2_ws/src/racer_bringup  topst@<보드IP>:~/D-Racer/src/

# (보드에서) 빌드
cd ~/D-Racer && colcon build --packages-select racer_bringup
source install/setup.bash
```

> 또는 같은 네트워크면 DDS로 개발 PC에서 노드를 띄워도 `/control`이 보드의
> control_node로 전달된다(초기 테스트용). 대회 본선은 보드에서 직접 실행 권장.

---

## 1. 배관 검증 (throttle 0, 가장 안전)
명령이 실제로 control_node까지 도달하는지부터 확인한다.
```bash
# 터미널 A: 거치대 캘리브레이션 노드 일괄 실행 (hold)
ros2 launch racer_bringup bench_calibration.launch.py

# 터미널 B: 명령이 흐르는지 확인
ros2 topic echo /control
```
`steering: 0.0, throttle: 0.0`이 20Hz로 발행되면 배관 OK.

---

## 2. 조향 중심(STEER_TRIM) + 최대각 측정
```bash
# hold 모드로 steering만 수동 지정하며 바퀴 방향 관찰
ros2 launch racer_bringup calibration.launch.py mode:=hold steering:=0.0
# 바퀴가 한쪽으로 치우치면 steering 값을 조금씩 바꿔 정확히 직진하는 값을 찾는다
ros2 param set /calibration_node steering 0.05   # 예: 0.05에서 직진이면
```
- **STEER_TRIM** = 직진이 되는 steering 명령값 → `config/vehicle.yaml`의 `steer_trim`
  및 D-Racer `vehicle_config.yaml`의 `STEER_TRIM`에 반영.
- **최대 조향각**: `steer_sweep`로 ±1.0까지 흔들고 각도기로 실제 바퀴 각도를 측정.
  ```bash
  ros2 launch racer_bringup calibration.launch.py mode:=steer_sweep sweep_amplitude:=1.0 sweep_period:=6.0
  ```
  steering=±1.0일 때의 실제 각도 → `config/vehicle.yaml`의 `max_steer_deg`.

---

## 3. throttle 중립/데드존 측정
바퀴가 막 돌기 시작하는 최소 throttle을 찾는다(거치대, 바퀴 띄움).
```bash
ros2 launch racer_bringup calibration.launch.py mode:=throttle_pulse pulse_throttle:=0.08
# 안 돌면 0.02씩 올리며 반복 (throttle_limit 안에서)
ros2 param set /calibration_node pulse_throttle 0.10
```
- 바퀴가 돌기 시작하는 값 = **throttle 데드존**. 이후 SpeedController의 최소
  throttle/매핑에 반영(예: 정규화 속도→throttle 변환 시 데드존 보상).

---

## 4. 측정값 반영
| 측정 항목 | 반영 위치 |
|-----------|-----------|
| 직진 steering 값 | `config/vehicle.yaml: steer_trim`, D-Racer `STEER_TRIM` |
| steering=±1.0 실제 각도 | `config/vehicle.yaml: max_steer_deg` |
| 휠베이스(자로 측정) | `config/vehicle.yaml: wheelbase` |
| throttle 데드존 | (Speed 매핑/SpeedController, Stage 6-b) |

→ 이 값으로 `core.feasibility.check_path`를 다시 돌리면, 실제 차량 기준으로
경로가 추종 가능한지 정확히 판정된다(시뮬과 실차의 파라미터 일원화).

---

## 다음 단계 (Stage 6-b)
캘리브레이션이 끝나면, `core`를 감싼 `controller_node`(Pure Pursuit + Speed)를
만들어 **정적 경로**를 추종시키되, 카메라 pose가 없으므로 우선
"조향 명령이 경로 곡률에 맞게 나오는지"를 거치대/저속에서 확인한다.
실제 폐루프 주행은 Perception(카메라→로컬 차선중심선)이 pose를 제공한 뒤 가능하다.
```
