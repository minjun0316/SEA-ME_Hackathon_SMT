# `core/` — ROS-free 자율주행 코어

> 📍 위치: [저장소 메인(포트폴리오)](../../README.md) → [d_racer_autonomous](../README.md) → **core/**

**ROS·하드웨어 의존성이 전혀 없는 순수 파이썬 코어.** 시뮬레이션(`sim/`), 테스트(`tests/`), 실차(ROS2 노드 `ros2_ws/`)가 전부 이 코드를 **그대로 import**해서 씁니다 — 시뮬에서 검증한 코드가 실차에서 다르게 도는 일이 없습니다.

모든 파라미터는 하드코딩 금지 원칙에 따라 [`../config/*.yaml`](../config/)에서 오고, [`config_schema.py`](config_schema.py)가 타입 안전하게 로드합니다.

---

## 🗺️ 모듈 지도

```
core/
├── geometry.py            # 최하층 수학 도구 (좌표 변환·각도 정규화)
├── path.py                # 계층 간 유일한 경로 계약 + 경로 기하 질의
├── path_factory.py        # 정적 테스트 경로 생성 (시뮬 Stage 1)
├── vehicle_model.py       # Kinematic Bicycle Model (시뮬 Stage 2)
├── feasibility.py         # 경로 실현가능성 검사 (최소 회전반경 대비)
├── config_schema.py       # 전 파라미터 스키마 + YAML 로더
│
├── control/               # ⚙️ 제어 — Path/오차 → 조향·스로틀
│   ├── lateral_pd.py      #   [실차 채택] 근거리 PD + 곡률 피드포워드 조향
│   ├── pure_pursuit.py    #   [시뮬·대조군] Pure Pursuit + Adaptive Lookahead
│   ├── speed_controller.py#   곡률 기반 목표 속도 (+선택적 Speed PID)
│   └── battery_compensator.py # 전압 기반 throttle 정규화 (액추에이터 직전 단)
│
├── planning/              # 🧠 판단 — 인지 신호 → 미션 상태·주행 지시
│   ├── mission.py         #   상위 6-state 미션 FSM (MissionSequencer)
│   ├── decision.py        #   하위 반응형 SM (DRIVE/SLOW/STOP/LOST)
│   └── stopline_maneuver.py #  개루프 고정조향 기동 (로터리용, 본 코스 비활성)
│
└── perception/            # 👁 인지 — 이미지 → 차선·객체·이벤트
    ├── lane_detect.py     #   BEV(IPM) + HLS 슬라이딩 윈도우 차선 검출
    ├── sign_detect.py     #   방향 팻말 (색 트리거 + YOLO 좌/우)
    ├── traffic_light_detect.py # 신호등 (YOLO green/red)
    └── aruco_detect.py    #   동적 장애물 ArUco 마커
```

ROS2 노드(`ros2_ws/src/`)는 이 코어를 감싸는 **얇은 래퍼**일 뿐입니다: 토픽 구독 → 코어 함수 호출 → 토픽 발행.

---

## 📖 읽는 순서 (처음 오신 분)

1. [`geometry.py`](geometry.py) → [`path.py`](path.py) — 좌표계와 데이터 계약부터. `Path`는 "Planning이 만들어 Controller에 넘기는 유일한 계약"입니다.
2. [`vehicle_model.py`](vehicle_model.py) — 시뮬이 어떻게 실차를 흉내 내는지.
3. [`control/lateral_pd.py`](control/lateral_pd.py) — 실차 주행을 담당한 조향 컨트롤러. 파일 상단 docstring에 제어 법칙 전체 유도가 있습니다.
4. [`planning/mission.py`](planning/mission.py) — 미션 FSM ([설계 문서](../docs/mission_fsm.md)와 함께).
5. [`perception/lane_detect.py`](perception/lane_detect.py) — 모든 것의 입력인 차선 검출.

각 파일은 Doxygen 스타일 docstring으로 **왜 이렇게 설계했는지**까지 적어두었습니다. 코드보다 [`../docs/`](../docs/)의 설계 문서가 진실의 원천입니다.

---

## ⚙️ control/ — 핵심 수식 요약

| 모듈 | 제어 법칙 | 비고 |
|------|-----------|------|
| `lateral_pd.py` | δ = k_ff·κ_eff + k_h·ψ + k_c(κ)·e + k_d·ė | κ_eff = EMA+데드밴드 필터, k_c(κ) = 곡률 스케줄, 출력 = 포화+트림+저역통과 |
| `pure_pursuit.py` | δ = atan(2L·sinα / L_d), α = atan2(y_l, x_l) | Adaptive lookahead: L_d = L_min + K_v·v − K_c·κ |
| `speed_controller.py` | v = clip(v_max − K_κ·\|κ\|, v_min, v_max) | 조향과 완전 독립, 상태별로 파라미터만 교체 |
| `battery_compensator.py` | throttle′ = throttle × min(V_nom/V_bat, g_max) | 제어가 아닌 "출력 정규화" → 모터 드라이버 직전 배치 |

---

## 🔒 불변 규칙 (요약 — 전체는 [`../docs/architecture.md`](../docs/architecture.md))

1. **perception/** 은 "무엇이 어디에 있는가"만 출력 — 조향·속도 계산 금지.
2. **planning/** 은 미션 상태와 지시만 생성.
3. **control/** 은 경로·오차만 입력받고 인지·판단 내부를 모른다.
4. 좌표계: 로컬 프레임 `base_link` (원점=뒷차축 중심, +x 전방, +y 좌측, m/rad).
5. `core/`에 ROS·하드웨어 의존성을 넣지 않는다.

## 🧪 바로 돌려보기

```bash
cd d_racer_autonomous
python3 -m pytest -q                       # 단위·통합 테스트
python3 -m sim.run_sim --path s_curve      # 폐루프 시뮬레이션
python3 -m sim.tune --path sharp_s \
    --param pure_pursuit.ld_min=0.3,0.4 --top 8   # 그리드 스윕 튜닝
```
