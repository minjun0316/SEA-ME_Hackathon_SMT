# D-Racer Autonomous

D-Racer-Kit(ROS2 스케일카) 기반 대회용 자율주행 시스템.
**인지 · 판단 · 제어를 분리**한 자동차 SW 회사식 구조로 개발한다.

> 전체 설계는 [`docs/architecture.md`](docs/architecture.md) 참고.

## 핵심 원칙
- 모든 인지는 **카메라 단독** → Pure Pursuit는 **로컬 프레임**에서 동작.
- 제어 코어는 **ROS-free 순수 파이썬** → 시뮬·테스트·실차에서 동일 코드 재사용.
- 하드코딩 금지 → 모든 파라미터는 `config/*.yaml`.
- 개발 순서 준수: **Static Path → Bicycle Model → 튜닝 → 실차 → MATLAB 분석 → Perception 연동.**

## 현재 구현 범위 (Stage 1 + 2 + 3, 일부 5)
- Stage 1: 정적 경로 생성 (`straight/circle/s_curve/sharp_s/figure_eight/rotary`)
- Stage 2: Kinematic Bicycle Model 폐루프 시뮬레이션
- Stage 3: 그리드 스윕 튜닝(`sim/tune.py`) + 경로 실현가능성 검사(`core/feasibility.py`) + 전방 윈도우 최근접 탐색
- 제어 코어: Pure Pursuit(Adaptive/Fixed Lookahead) · Speed Controller · Battery Compensator
- Stage 5: 로그 분석 그래프 (MATLAB + Python)

## 빠른 시작
```bash
cd d_racer_autonomous
python3 -m pytest -q                       # 단위/통합 테스트 (18개)

# 시뮬레이션 실행 (CSV 로그 + 콘솔 요약)
python3 -m sim.run_sim --path s_curve

# 그래프 저장(헤드리스) / 표시
python3 -m sim.run_sim --path figure_eight --save-plot out.png
python3 -m sim.run_sim --path rotary --plot

# 초기 테스트용 고정 lookahead, 파라미터 즉석 override
python3 -m sim.run_sim --path circle --set pure_pursuit.use_adaptive_lookahead=false
python3 -m sim.run_sim --path sharp_s --set pure_pursuit.ld_min=0.25 --set speed.v_max=0.5

# Stage 3: 파라미터 그리드 튜닝 (감이 아니라 정량 탐색)
python3 -m sim.tune --path sharp_s \
    --param pure_pursuit.ld_min=0.3,0.4 --param pure_pursuit.kc=0.0,0.1,0.2 \
    --param speed.v_max=0.4,0.6,0.8 --top 8 --save-best config/tuned_sharp_s.yaml

# 튜닝 결과 적용해서 재실행
python3 -m sim.run_sim --path sharp_s \
    --config config/vehicle.yaml config/controller.yaml config/sim.yaml config/tuned_sharp_s.yaml

# MATLAB 분석
#   >> plot_logs('sim_log.csv')
```

## 디렉토리
| 경로 | 내용 |
|------|------|
| `core/` | ROS-free 제어 코어 (geometry, path, vehicle_model, controllers) |
| `sim/` | 시뮬레이터 · 로거 · 실행 엔트리포인트 |
| `analysis/` | 로그 분석 (MATLAB `.m` + Python) |
| `config/` | 튜닝 파라미터 YAML (vehicle / controller / sim) |
| `tests/` | pytest 단위·통합 테스트 |
| `docs/` | 아키텍처 문서 |

## 다음 단계
- **Stage 3**: `config/controller.yaml` 로 sharp_s 등 튜닝, 전방 윈도우 최근접 탐색 도입.
- **Stage 6**: `core`를 import하는 얇은 ROS2 노드(perception/planning/controller/mission_manager/logger)로 D-Racer-Kit에 통합.
