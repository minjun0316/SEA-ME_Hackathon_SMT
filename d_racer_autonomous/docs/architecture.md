# D-Racer 자율주행 아키텍처

> 목표: 대회에서 **안정적으로** 동작하는 자율주행 시스템.
> 원칙: 자동차 SW 회사식 개발 프로세스 — **인지(Perception) · 판단(Planning) · 제어(Control)를 절대 섞지 않는다.**

---

## 1. 계층 구조와 책임 경계

```
  Camera (유일한 센서 — 모든 인지는 카메라로만)
        │
        ▼
  ┌───────────────┐   출력: 차선 중심선, 객체 위치, 신호등 상태, 차량 위치(로컬)
  │  Perception   │   ※ 절대 조향을 계산하지 않는다
  └───────────────┘
        │  (Path 데이터 계약)
        ▼
  ┌───────────────┐   State Machine으로 현재 미션 판단 → Target Path 생성
  │   Planning    │   상태별로 lookahead / speed / 조향제한만 바꾼다
  └───────────────┘
        │  (core.path.Path)
        ▼
  ┌───────────────┐   Pure Pursuit(조향) + Speed Controller(속도) + Battery 보정
  │  Controller   │   Planning이 만든 Path만 입력받는다 (Perception 직접 호출 금지)
  └───────────────┘
        │  (control_msgs/Control: steering, throttle ∈ [-1, 1])
        ▼
  Motor Driver (control_node → PCA9685 → ESC/서보)  →  Vehicle
```

**불변 규칙**
1. Perception은 "무엇이 어디에 있는가"만 출력한다. 조향/속도 계산 금지.
2. Planning은 "지금 무슨 미션인가 + 따라갈 Path"만 만든다.
3. Controller는 `Path` 하나만 입력으로 받는다. Perception/Planning 내부를 모른다.
4. 세 계층 사이의 유일한 데이터 계약은 **`core.path.Path`** 와 **`control_msgs/Control`** 이다.

---

## 2. 좌표계 결정 (중요)

모든 인지는 카메라 단독 → **로컬 프레임(local frame)** 채택.

- **실차**: Perception이 매 프레임 차선 중심선을 *차량 로컬 좌표*로 출력한다. 차량은 항상 원점 `(0,0,0)`. 글로벌 측위(SLAM/오도메트리) 불필요, 누적 드리프트 없음.
- **시뮬레이션(Stage 1~3)**: Kinematic Bicycle Model이 글로벌 pose를 ground-truth로 제공하고, 정적 경로는 글로벌 프레임에 정의된다.

두 경우를 **같은 컨트롤러 코드**로 처리하는 비결:
`PurePursuitController.compute(pose, path, speed)` 가 프레임을 가리지 않는다.
실차에서는 `pose = Pose2D(0,0,0)` 이라 `to_local_frame` 변환이 항등이 되어, 로컬 경로를 그대로 추종한다. (`core/geometry.py` 참고)

---

## 3. 코드 구조 (ROS-free 코어 + ROS2 래퍼)

```
d_racer_autonomous/
├── core/                       # ROS/하드웨어 의존성 0 — 시뮬·테스트·실차 공용
│   ├── geometry.py             #   2D 좌표 변환 / 각도 정규화
│   ├── path.py                 #   Path: Planning→Control 데이터 계약
│   ├── path_factory.py         #   Stage 1: 정적 경로 생성기
│   ├── vehicle_model.py        #   Stage 2: Kinematic Bicycle Model
│   ├── feasibility.py          #   Stage 3: 경로 실현가능성(최소회전반경) 검사
│   ├── config_schema.py        #   파라미터 스키마(dataclass) + YAML 로더
│   └── controllers/
│       ├── pure_pursuit.py     #   PurePursuitController
│       ├── speed_controller.py #   SpeedController
│       └── battery_compensator.py  # BatteryCompensator
├── sim/                        # Stage 2~5: 시뮬레이션/로깅 (ROS-free)
│   ├── simulator.py            #   폐루프 엔진
│   ├── logger.py               #   고정 스키마 CSV 로거 (실차 Logger와 동일)
│   ├── tune.py                 #   Stage 3: 파라미터 그리드 스윕 튜너
│   └── run_sim.py              #   CLI 엔트리포인트
├── analysis/                   # Stage 5: 로그 분석
│   ├── plot_logs.m             #   MATLAB 분석 스크립트
│   └── plot_logs.py            #   동일 그래프 Python 버전(헤드리스/CI)
├── config/                     # 모든 튜닝 파라미터(하드코딩 금지)
│   ├── vehicle.yaml            #   휠베이스, 조향한계, 트림
│   ├── controller.yaml         #   Pure Pursuit / Speed / Battery
│   └── sim.yaml                #   dt, 경로, 시뮬 설정
├── tests/                      # 단위 + 통합 테스트(pytest)
└── docs/                       # 설계 문서

# (Stage 6) 향후 추가될 ROS2 패키지 — 위 core를 import만 한다:
#   src/perception/   (카메라→차선중심선/객체)
#   src/planning/     (State Machine→Target Path)
#   src/controller/   (core.controllers를 감싼 rclpy 노드)
#   src/mission_manager/, src/logger/  (분리된 노드)
```

**왜 코어를 ROS에서 분리했나**
- PC에서 차 없이 단위 테스트/시뮬레이션 가능 → 빠르고 안전한 반복.
- *동일한* `PurePursuitController` 객체가 시뮬과 실차에서 100% 같은 코드로 동작 → "시뮬에서 튜닝한 값이 실차에서 다르게 동작" 리스크 제거.
- ROS2 노드는 I/O(토픽 구독·발행, 파라미터 로드)만 담당하는 얇은 껍데기가 된다.

---

## 4. State Machine (Planning)

```
READY → START_WAIT → LANE_FOLLOW → S_CURVE → FORK
      → DYNAMIC_OBSTACLE → ROTARY → FINAL_LANE → FINISH
```

각 State는 **컨트롤러를 바꾸지 않는다.** 오직 파라미터만 바꾼다:

| State            | lookahead | speed | 조향 제한 |
|------------------|-----------|-------|-----------|
| LANE_FOLLOW      | 0.6       | 0.8   | 1.0       |
| S_CURVE          | 0.35      | 0.4   | 1.0       |
| ROTARY           | 0.3       | 0.3   | 1.0       |

→ 구현상 각 State는 `controller.yaml`의 한 프로파일(섹션)에 대응한다.
State 전환 시 해당 프로파일을 컨트롤러에 주입한다. (Stage 6에서 구현)

---

## 5. 제어기 설계 요약

### Pure Pursuit (조향) — `core/controllers/pure_pursuit.py`
1. 현재 위치 → 최근접 경로점
2. Lookahead Point 계산 (Adaptive: `Ld = Ld_min + Kv·v − Kc·κ`, 초기엔 고정값 옵션)
3. `alpha = atan2(y_local, x_local)`
4. `delta = atan2(2·L·sin(alpha), Ld)`
5. 후처리: **Smoothing** `out=(1−β)·old+β·new` → **Saturation** `[−max,+max]` → 정규화 `[−1,1]`

### Speed Controller — `core/controllers/speed_controller.py`
- 조향과 **완전 분리**. 곡률 기반: `v = clip(v_max − K·|κ|, v_min, v_max)`.
- 선택적 Speed PID(실차 throttle↔속도 비선형 보정용).

### Battery Compensation — `core/controllers/battery_compensator.py`
- `throttle' = throttle × (V_nominal / V_battery)` 또는 룩업 테이블.
- 과보상 방지 상한(`max_gain`). Motor Driver 직전 마지막 단에 위치.

---

## 6. 개발 순서 (반드시 준수)

| Stage | 내용 | 상태 |
|-------|------|------|
| 1 | 정적 경로 생성 (직선/원/S/급S/8자/로터리) | ✅ `core/path_factory.py` |
| 2 | Kinematic Bicycle Model 시뮬레이션 | ✅ `core/vehicle_model.py`, `sim/` |
| 3 | Pure Pursuit 튜닝 + 강건성 + 실현가능성 검사 | ✅ `sim/tune.py`, `core/feasibility.py` |
| 4 | 실제 차량 테스트 + 로그 저장(CSV/rosbag) | ⬜ Stage 6 ROS2 노드 필요 |
| 5 | MATLAB 분석 그래프 | ✅ `analysis/plot_logs.m` (+ .py) |
| 6 | Perception 연동 (이때 비로소 카메라/YOLO) | ⬜ |

**Stage 3 결과(기본 차량: 휠베이스 0.26m / 조향 24° → 최소회전반경 58cm)**

| 경로 | 실현가능성(margin) | 튜닝 후 RMS CTE |
|------|-------------------|-----------------|
| circle | 3.38× | 0.13 cm |
| s_curve | 2.72× | 1.23 cm |
| sharp_s | 1.16× (한계 근처) | 0.75 cm (그리드 튜닝) |
| figure_eight | 1.25× | 0.83 cm (양쪽 lobe 완주) |
| rotary | 2.54× | 0.68 cm |

**절대 규칙: Stage 1→5로 제어기를 안정화하기 전에 Perception(YOLO 등)을 연결하지 않는다.**

---

## 7. Stage 3에서 해결한 것 / 남은 한계

**해결**
- **전방 윈도우 최근접 탐색 + 진행 인덱스 단조 증가**(`pure_pursuit.search_window`): 자기교차/되돌아오는 경로에서 엉뚱한 분기로 점프하거나 시작점 부근에서 조기 종료하던 문제 해결 → figure-eight가 양쪽 lobe를 완주.
- **경로 실현가능성 검사**(`core/feasibility.py`): 컨트롤러를 탓하기 전에 경로 곡률이 차량 최소 회전반경을 넘는지 먼저 확인. 이전 sharp_s(반경 10cm 요구)는 물리적으로 불가능했음을 발견 → 차량 사양에 맞는 경로로 재정의.
- **체계적 그리드 튜닝**(`sim/tune.py`): 감이 아니라 격자 탐색으로 재현 가능하게 최적 파라미터 도출.

**남은 한계 / 향후**
- Speed PID / Battery 보정은 실차 데이터가 있어야 의미가 크므로 Stage 4 이후 본격 튜닝.
- 곡률은 이산 차분 기반이라 경로 양 끝 점에서 다소 불안정 → 검사 시 edge_trim으로 제외.
- 그리드 튜너는 전수 탐색(조합 수 = 곱)이라 파라미터가 많아지면 느려짐 → 필요 시 베이지안 최적화로 확장 가능.
