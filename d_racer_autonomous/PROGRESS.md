# D-Racer 자율주행 — 진행 상황 / 다음 할 일

> 마지막 작업일: 2026-07-03. 이 문서는 매 세션 끝에 갱신한다.
>
> **작업 환경 변경(07-03)**: 이제 보드에서 직접 편집·빌드·커밋한다. 워크스페이스=팀레포 `~/SEA-ME_Hackathon_SMT`(= colcon ws, `build/ install/ src/` 포함). 옛 `~/D-Racer-Kit`는 통합되어 없어짐. scp 왕복 불필요.

## 한 줄 요약
ROS-free 코어로 Stage 1~3(시뮬) 완성. 실차(보드) 연결 시작 — 캘리브레이션 진행 중.

---

## 환경 / 접속 정보
| 항목 | 값 |
|------|-----|
| 개발 PC | `sdw@sdw-desktop`, 프로젝트: `/home/sdw/d_racer_autonomous/` |
| 차(보드) | `ssh topst@192.168.0.123`, 워크스페이스: `~/D-Racer-Kit`, ROS2 Humble |
| 배포 방식 | 데스크탑에서 수정 → `scp`로 보드에 복사 → 보드에서 `colcon build` |
| (편의) | 나중에 VS Code Remote-SSH 세팅하면 scp 없이 차 파일 직접 편집 가능 |

### 자주 쓰는 명령
```bash
# 데스크탑 → 보드 파일 복사 (예: 캘리브 노드)
cd /home/sdw/d_racer_autonomous/ros2_ws/src
scp racer_bringup/racer_bringup/calibration_node.py topst@192.168.0.123:~/D-Racer-Kit/src/racer_bringup/racer_bringup/

# 보드에서 빌드
cd ~/D-Racer-Kit && source /opt/ros/humble/setup.bash && colcon build --packages-select racer_bringup && source install/setup.bash

# 보드: 모터 제어 노드 (터미널 1, 켜둬야 바퀴 움직임)
ros2 run control control_node --ros-args -p use_joystick_control:=False

# 보드: 캘리브레이션 노드 (터미널 2, 백그라운드)
ros2 run racer_bringup calibration_node &
ros2 param set /calibration_node steering 0.10   # 실시간 조정
ros2 param set /calibration_node throttle 0.0    # 0.15로 하드 클램프
pkill -f calibration_node                         # 정지
```

---

## ✅ 완료한 것

### Stage 1~3 (시뮬레이션, 개발 PC) — 완료
- ROS-free 코어 `core/`: geometry, path, path_factory(정적경로 6종), vehicle_model(자전거모델), feasibility, config_schema, controllers(PurePursuit/Speed/Battery)
- 시뮬 `sim/`: simulator, logger(CSV 고정스키마), run_sim, tune(그리드 튜너)
- 분석 `analysis/`: plot_logs.m (MATLAB) + .py
- 설정 `config/`: vehicle/controller/sim YAML
- 테스트 `tests/`: **21개 전부 통과** (`python3 -m pytest -q`)
- 주요 성과: 전방 윈도우 최근접탐색으로 figure-eight 완주, 실현가능성 검사로 불가능 경로 판별, 그리드 튜닝으로 sharp_s CTE 0.75cm

### Stage 6-a (실차 브링업) — 진행 중
- `ros2_ws/src/racer_bringup/` 패키지 작성 → 보드 배포 → 빌드 성공
- ✅ control_node 하드웨어 연결 확인 (i2c bus=3, PCA9685 0x40)
- ✅ **조향 배관 검증**: 우리 노드 → /control → 서보 → 앞바퀴 좌우 움직임 확인 (steer_sweep)
- ✅ calibration_node 실시간 파라미터 조정 동작 (hold 모드, 값 바뀔 때만 로그)

---

## 🔖 2026-07-02/03 세션 메모

### 트림 모델 확정 (가법 가정 폐기) ⭐
- control_node.py 코드 확인: `/control` 명령이 오면 `self.steering = msg.steering` (control_node.py:121) — **트림을 안 더하고 그대로** 서보로. `STEER_TRIM`은 시작 idle값(75)·종료 중립값(151)으로만 쓰임.
- 결론: **직진 서보명령 = 0.2238**. 키트 `vehicle_config.yaml: STEER_TRIM=0.2238`(commit 93c31a0 push 완료), 우리 `vehicle.yaml: steer_trim=0.2238`. **우리 controller가 출력에 0.2238을 직접 더한다.**
- 팀 참고: 팀원이 올렸던 0.3238은 가법가정 실수 → 0.2238로 정정·push됨.

### 데이터 행방 (⚠️ 확인 필요)
- 보드 `YOLOv26n_seg/datasets/` **비어 있음** (collect1 없음, capture_frames.py 저장경로 `datasets/SECOND/`도 없음). 보드 디스크에 07-02 카메라 프레임 없음.
- 07-02 rosbag `bagfile/bag_20260702_091344`는 `/joystick`만 214개(카메라 없음) — 학습데이터 아님.
- → Roboflow 업로드분/데스크탑에 사진이 있는지 확인할 것. 없으면 트랙에서 재수집.

## 🔖 2026-07-01 세션 메모 (배터리 방전으로 중단)

### 배관 재검증 완료 ✅ (오늘 실차에서 확인)
- 캘리 파이프라인 정상 동작 확인: `control_node(use_joystick_control:=False)` + `calibration.launch.py mode:=hold` + 터미널3 `ros2 param set /calibration_node steering <값>`.
- **함정**: `steering` 값을 `0.03~0.05`처럼 작게 주면 바퀴 변화가 눈에 안 보인다. `0.6/-0.6`처럼 크게 주면 확실히 좌우로 꺾인다. → 배관 확인은 큰 값으로 할 것.
- 터미널 구성: T1=control_node(False, 켜둠), T2=calibration.launch(hold, 켜둠), T3=param set 입력.

### ⚠️ 발견: 보드 STEER_TRIM이 0.30으로 커져 있음
- 보드 `~/D-Racer-Kit/src/config/vehicle_config.yaml`의 `STEER_TRIM`이 **0.30** (어제는 0.10). control_node 기동 로그에서 `steer_trim=0.30` 확인. 값이 과해 `steering=0.0`에서 앞바퀴가 오른쪽으로 휨.
- control_node 최종 서보각 = (calibration steering 명령) + STEER_TRIM(0.30), 클램프.
- **직진 되는 calibration steering 값 = X** 를 찾는 중이었음(정면 맞췄다고 함, but 값 기록 전 배터리 방전).
  → 최종 트림 = `0.30 + X`. 이 최종값을 `config/vehicle.yaml: steer_trim`과 보드 `vehicle_config.yaml: STEER_TRIM`에 기록하고, calibration steering은 0으로 되돌릴 것.

### 2차 세션(07-01 오후) 실주행 결과
- **직진값 후보: calibration steering ≈ 0.22** (보드 STEER_TRIM=0.30 상태에서 실제 바닥 주행이 제일 곧게 감). 거치대 눈대중(0.35)보다 낮음 — 역시 굴려봐야 정확. 배터리 충전 후 재확인 필요.
  → ~~확정 시 최종 STEER_TRIM ≈ 0.30+0.22 = 0.52 (additive 가정)~~ **[07-02 폐기]** 가법 가정은 틀림. control_node.py:121에서 `/control` 명령을 트림 없이 그대로 서보에 전달함(steer_trim은 idle/종료 중립값으로만 사용). 실측 직진 서보명령 = **0.2238**. → 키트 vehicle_config.yaml `STEER_TRIM=0.2238`, 우리 controller가 출력에 0.2238을 직접 더한다.
- **모터 데드존(바닥)**: throttle 0.15로는 안 돌고 삐 소리만. throttle_limit을 0.22~0.25로 풀어야 함(`ros2 run racer_bringup calibration_node --ros-args -p mode:=hold -p throttle_limit:=0.25`). launch파일 쓰면 0.15로 리셋되니 주의.
- **⚠️ 배터리 이슈 재발**: 주행 중 방전되어 보드 네트워크 끊김(No route to host) → 재부팅. 이후 모터가 throttle 0.18에도 삐 소리만 나고 안 돎 = **저전압 컷오프 의심**(조향 서보는 됨). 충전/완충 배터리 교체로 해결 예상. 대회 전 여분 배터리·완충 습관 필요.
- 주행 테스트 시 **차가 와이파이 범위 벗어나지 않게**, 멀어지기 전에 throttle 0.

### 조이스틱 조종 이슈 (캘리와 별개, 나중에)
- `manual_driving.launch`로 joystick_node는 뜨지만 스틱/버튼 값이 전부 0.00으로 읽힘.
- 단, `jstest /dev/input/js0`에서는 좌우 움직임에 값이 바뀜 → **하드웨어/리눅스는 정상, ROS joystick_node가 장치를 못 읽는 것**. 노드가 읽는 장치경로/축매핑 확인 필요(`manual_driving.launch.py` + joystick 패키지 소스). 캘리엔 지장 없음.

### 충전 후 재개 순서
1. T1/T2/T3 다시 띄우기(위 구성). control_node는 반드시 `use_joystick_control:=False`.
2. `ros2 param get /calibration_node steering`로 값 확인하며 **정면 X** 다시 찾기.
3. 최종 트림 `0.30+X` 기록 → 손으로 밀어 직진 검증.
4. 이어서 최대 조향각 / throttle 데드존 / 휠베이스 측정.

---

## ⏳ 다음 할 일

### 1. 캘리브레이션 마무리 (거치대, 바퀴 띄운 상태)
- [~] **최대 조향각**: 손측정 ≈ **10도** → `config/vehicle.yaml: max_steer_deg=10.0` (잠정, 실주행 원그리기 δ=atan(L/R)로 재확인)
- [ ] **throttle 데드존 측정**: `throttle 0.08`부터 0.02씩 올려 뒷바퀴 막 도는 값 찾기 (⚠️ 바퀴 띄운 상태 필수)
- [x] **직진 트림 (07-02 확정)**: 실주행 직진 서보명령 = **0.2238**. 가법 가정 폐기(control_node는 /control 명령을 트림 없이 그대로 서보 전달, control_node.py:121). → 키트 `src/config/vehicle_config.yaml: STEER_TRIM=0.2238` (commit 93c31a0, push 완료), 우리 `config/vehicle.yaml: steer_trim=0.2238` 기록됨. controller가 출력에 0.2238을 직접 더한다.
- [x] **휠베이스** 자로 측정 → `config/vehicle.yaml: wheelbase` = **0.175** (2026-07-01, 좌우 1mm차 무시)
- [ ] 측정값으로 `core.feasibility.check_path` 다시 돌려 실제 차량 기준 경로 가능여부 확인

### 2. Stage 6-b: controller_node (core 감싸기)
- [ ] `core`를 import하는 ROS2 `controller_node` 작성 (PurePursuit+Speed → /control)
- [ ] 문제: 카메라 인지 없어 차량 pose 없음 → 폐루프 주행은 아직 불가. 우선 정적경로로 "곡률에 맞는 조향 명령이 나오는지" 거치대/저속 확인
- [ ] core를 보드에서 import 가능하게 패키징 (ament_python 패키지로 만들거나 PYTHONPATH)

### 3. Stage 4: 실차 주행 + 로깅
- [ ] logger_node로 주행 로그(CSV/rosbag) 저장 → MATLAB 분석 → 재튜닝

### 4. Perception (가장 어려움, 카메라 단독)
- [ ] 카메라 이미지 → 차선 중심선(로컬 프레임) 출력. 키트 `bagfile/`, `data_acquisition.sh`로 녹화 이미지 오프라인 개발 가능
- [ ] 이게 pose(로컬 경로)를 제공해야 비로소 폐루프 자율주행 완성

---

## 핵심 설계 원칙 (잊지 말 것)
- 인지/판단/제어 **절대 분리**. Controller는 Path만 입력받음.
- 모든 인지는 **카메라 단독** → Pure Pursuit는 **로컬 프레임**(차량=원점).
- ROS-free 코어 + 얇은 ROS2 노드 래퍼 (시뮬↔실차 동일 코드).
- 하드코딩 금지, 모든 파라미터 YAML.
- 개발 순서 준수: Static Path → Bicycle Model → 튜닝 → 실차 → 분석 → Perception.
- 자세한 설계는 `docs/architecture.md`, 캘리브레이션 절차는 `docs/calibration.md`.
