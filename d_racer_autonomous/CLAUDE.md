# CLAUDE.md — D-Racer Autonomous

D-Racer-Kit(ROS2 스케일카) 대회용 자율주행 시스템. **인지 · 판단 · 제어 분리** 구조.
Claude는 코드를 만지기 전에 아래 문서들을 **진실의 원천(source of truth)** 으로 먼저 참조한다.

## 📚 먼저 읽을 문서 (docs/ — 코드보다 문서가 진실)
| 문서 | 언제 참조 |
|------|-----------|
| [`docs/interfaces.md`](docs/interfaces.md) | **노드 간 토픽/메시지 계약.** 토픽·QoS·메시지 필드·좌표계를 건드리면 반드시 먼저 확인·수정. |
| [`docs/architecture.md`](docs/architecture.md) | 계층 구조와 책임 경계, 좌표계 결정, ROS-free core 설계. |
| [`docs/mission_fsm.md`](docs/mission_fsm.md) | 미션 상태기계(12-state 로터리 지름길). `core/planning/mission.py` 구현 기준. |
| [`docs/perception_agreement.md`](docs/perception_agreement.md) | 판단↔인지 왕복 계약(follow_color/roi_mode/turn_bias) 상세 합의. |
| [`docs/calibration.md`](docs/calibration.md) | 조향 트림·액추에이터 매핑 등 캘리브레이션 값. |
| [`PROGRESS.md`](PROGRESS.md) | 현재까지 진행 상황·결정 로그. 작업 전 최신 상태 확인. |
| [`../TEAM_GUIDE.md`](../TEAM_GUIDE.md) | 팀 git 협업 규칙(대용량 커밋 금지, force push 금지 등). |

> 계약(토픽/메시지)을 바꾸려면 **먼저 `interfaces.md`를 고치고** 커밋 메시지에 `[iface]` 태그 + 팀 공지.

## 🚗 담당 범위 (이 개발자 = D-Racer)
- **소유: 제어(controller) + 판단 로직(decision/mission).** 인지(perception)는 **팀원 담당** — 인지 코드는 계약을 통해서만 다루고, 임의로 바꾸지 않는다.
- 인지와 맞닿는 변경은 계약(`interfaces.md`/`perception_agreement.md`) 기준으로만.

## ⚙️ 작업 방식
- **설계 먼저 합의.** 미션/요구사항을 확인하고 설계에 합의한 뒤에 코딩한다. 큰 변경은 먼저 계획을 제시.
- 하드코딩 금지 — 모든 파라미터는 `config/*.yaml`(vehicle/controller/sim/decision/mission).
- 제어 코어는 **ROS-free 순수 파이썬**(`core/`) — 시뮬·테스트·실차에서 동일 코드 재사용. `core/`에 ROS/하드웨어 의존성 넣지 않기.
- 개발 순서: **Static Path → Bicycle Model → 튜닝 → 실차 → MATLAB 분석 → Perception 연동.**

## 🔒 불변 규칙 (architecture.md)
1. Perception은 "무엇이 어디에" 만 출력 — 조향/속도 계산 금지.
2. Planning은 "지금 무슨 미션 + 따라갈 Path" 만 생성 — 상태별로 lookahead/speed/조향제한 배율만 바꾼다.
3. Controller는 `core.path.Path` 하나만 입력 — Perception/Planning 내부를 모른다.
4. 계층 간 유일한 데이터 계약: **`core.path.Path`** 와 **`control_msgs/Control`**.
5. 좌표계: 로컬 프레임 `base_link`, 원점=뒷차축 중심, +x 전방/+y 좌측, 길이 m·각도 rad. 실차 pose는 항상 원점(항등변환).
6. steer_trim(0.2238)은 **제어단(Pure Pursuit)에서 이미 포함**되어 발행 — 재가산 금지. 인지/판단은 트림을 모른다.
7. 안전 우선: 입력 끊김/저신뢰 → 정지(fail-safe)가 기본(watchdog `lane_timeout` 기본 0.3s).

## 🗂️ 구조
- `core/` — ROS-free 코어: `geometry.py` `path.py` `path_factory.py` `feasibility.py` `vehicle_model.py`, `control/`(pure_pursuit·speed_controller·battery_compensator), `planning/`(decision·mission), `perception/`(lane_detect).
- `sim/` — 시뮬레이터·로거·엔트리포인트(`run_sim.py`, `tune.py`).
- `config/` — 튜닝 YAML. `tests/` — pytest. `analysis/` — 로그 분석(MATLAB `.m` + Python).
- `ros2_ws/src/` — 얇은 ROS2 래퍼: `racer_msgs`(LaneStatus/DriveCommand/MissionCues/LaneMode), `d_racer_perception`, `racer_bringup`.

## 🧪 자주 쓰는 명령
```bash
# ROS 워크스페이스 준비 (보드 터미널 접속하면 항상)
cd ~/SEA-ME_Hackathon_SMT && source /opt/ros/humble/setup.bash && source install/setup.bash

cd d_racer_autonomous
python3 -m pytest -q                                  # 단위·통합 테스트
python3 -m sim.run_sim --path s_curve                 # 시뮬 실행(경로: straight/circle/s_curve/sharp_s/figure_eight/rotary)
python3 -m sim.run_sim --path circle --set pure_pursuit.use_adaptive_lookahead=false   # 파라미터 즉석 override
python3 -m sim.tune --path sharp_s --param pure_pursuit.ld_min=0.3,0.4 --top 8         # 그리드 튜닝
```

## 🌿 협업
- 브랜치 `dev`에서 작업. push 전 `git pull`. **대용량 파일(이미지/영상/`.bag`/빌드 산출물) 커밋 금지**, `git push --force` 금지.
- **작업 환경(PROGRESS 07-03)**: 워크스페이스=팀레포 `~/SEA-ME_Hackathon_SMT`(colcon ws, `build/ install/ src/` 포함). 이제 **보드에서 직접 편집·빌드·커밋**한다(scp 왕복 없음). ⚠️ 커밋 메시지에 파트 태그(`[iface]`/`[ros]`/`[planning]`/`[perception]`/`[control]` 등).
