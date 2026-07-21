# 제어 (Control) 설계 문서

> [README](../README.md)의 제어 파트 설계 근거·구현·검증을 정리한다.
> 담당 — 신동원 · 코드: [`d_racer_autonomous/core/`](../d_racer_autonomous/core/README.md), 노드: [`ros2_ws/src/racer_bringup/`](../d_racer_autonomous/ros2_ws/src/racer_bringup/)

**목차**

1. [ROS-free 제어 코어와 개발 흐름](#1-ros-free-제어-코어와-개발-흐름)
2. [차량 모델](#2-차량-모델)
3. [곡률 계산](#3-곡률-계산)
4. [횡방향 — Lateral PD (실차 채택)](#4-횡방향--lateral-pd-실차-채택)
5. [횡방향 — Pure Pursuit (대조군)](#5-횡방향--pure-pursuit-대조군)
6. [종방향 속도](#6-종방향-속도)
7. [시뮬레이션 검증과 그리드 튜닝](#7-시뮬레이션-검증과-그리드-튜닝)
8. [실차 문제와 해결](#8-실차-문제와-해결)

> 값 표기 — 게인은 코드 스키마 기본값(`config_schema.py`)과 실차 `controller.yaml`이 다르다. 실차 채택값은 `controller.yaml` 기준으로 적고, 필요 시 스키마 기본을 병기한다.

---

## 1. ROS-free 제어 코어와 개발 흐름

제어 코어는 ROS에 의존하지 않는 순수 파이썬이다([`core/`](../d_racer_autonomous/core/README.md)). 시뮬레이션과 실차가 **같은 코어 코드**로 돈다.

- **개발 순서** — 정적 경로 생성 → Kinematic Bicycle Model 폐루프 시뮬 → 그리드 스윕 자동 튜닝(`sim/tune.py`) → 실차 이식 → 로그 분석.
- 조향·스로틀은 `[-1, 1]` 정규화 계약으로 모터 드라이버(PCA9685)에 전달한다.
- 조향과 속도는 완전 분리 원칙을 지킨다(§6).

> **주의(설계 이력)** — 시뮬레이터는 Pure Pursuit만 구동한다. 실차 채택 조향인 Lateral PD(§4)는 시뮬레이터에 통합돼 있지 않아 그리드 튜닝 대상이 아니며, 게인은 실차 A/B 시험(07-10~07-16)으로 결정했다. 즉 **정적 경로·시뮬·튜닝은 Pure Pursuit 계열의 타당성 검증에 쓰였고, 최종 실차 조향은 그 위에서 A/B로 갈아탄 Lateral PD**다.

---

## 2. 차량 모델

[`core/vehicle_model.py`](../d_racer_autonomous/core/vehicle_model.py) — 후륜축 기준 Kinematic Bicycle Model, 오일러 전진 적분.

$$\dot x = v\cos\psi, \qquad \dot y = v\sin\psi, \qquad \dot\psi = \frac{v}{L}\tan\delta$$

- 조향 포화 $\delta = \mathrm{clip}(\delta_{cmd}, \pm\delta_{max})$, 속도는 1차 지연 $\alpha = dt/(\tau+dt)$ (`speed_tau`, 시뮬 전용).
- 실차 파라미터([`config/vehicle.yaml`](../d_racer_autonomous/config/vehicle.yaml)): 휠베이스 $L=0.175$ m, `max_steer_deg=15.5`, `steer_trim=0.2238`.
- **최대 조향각 15.5°** 는 풀락 원주행 실측을 $\delta=\arctan(L/R)$로 역산해 확정했다(좌 14.85°/우 16.26° 평균, $R_{min}\approx0.63$ m). 손측정 10°(2배 과조향)와 시뮬 잠정 20°(0.78배 언더스티어)를 모두 폐기한 값이다.
- **트림 가법 처리** — 베이스 키트의 `control_node`가 `/control` 명령을 트림 없이 서보에 그대로 전달하므로, 우리 컨트롤러가 출력에 `steer_trim(0.2238)`을 직접 더한다(직진 = 0.2238).

> 코드 스키마 기본값(휠베이스 0.26, `max_steer` 24°, trim 0)은 초기 가정이며 단위 테스트에서만 쓴다. 실차 사양은 위 `vehicle.yaml` 값이다.

---

## 3. 곡률 계산

[`core/path.py`](../d_racer_autonomous/core/path.py) — 경로 생성 시 리샘플(간격 0.05 m) 후 부호 곡률을 사전계산한다.

$$\kappa = \frac{x'y'' - y'x''}{(x'^2 + y'^2)^{3/2}}$$

`np.gradient`(중심차분)로 도함수를 근사한다. 좌회전(반시계) +, 우회전 −. 컨트롤러별로 **다른 곡률 지표**를 쓴다.

- **Lateral PD** → `mean_signed_curvature_ahead`: 최근접점~전방 preview 구간의 **부호 곡률 평균**(스파이크 완화).
- **Pure Pursuit** → `max_abs_curvature_ahead`: 구간 **max |κ|**(곡선을 미리 보고 lookahead·gain 조정).

양 끝점은 수치적으로 불안정해 호출부에서 인덱스를 클립하고, feasibility 검사는 끝 3점(`edge_trim=3`)을 제외한다.

---

## 4. 횡방향 — Lateral PD (실차 채택)

인지가 계약으로 주는 근거리 신호 두 개 — 횡오차 $e$[m], heading 오차 $\psi$[rad] — 에 경로 곡률 $\kappa$를 더해 조향을 만든다. [`core/control/lateral_pd.py`](../d_racer_autonomous/core/control/lateral_pd.py)

$$\delta = k_{ff}\,\kappa_{eff} + k_h\,\psi + k_c(\kappa)\,e + k_d\,\dot e_{EMA}$$

(순서대로 곡률 피드포워드 · heading 정렬 · crosstrack P · crosstrack D)

각 항의 전처리가 노이즈 방어의 핵심이다.

```python
# crosstrack D: EMA 저역통과
deriv_ema = (1-a)*deriv_ema + a*raw_deriv                 # a = deriv_smoothing

# 곡률 피드포워드: EMA 후 데드밴드로 직선 노이즈 격리
curv_ema  = (1-ac)*curv_ema + ac*curvature               # ac = curvature_smoothing
kappa_eff = sign(curv_ema) * max(0, |curv_ema| - db)     # db = curvature_deadband

# k_cross 곡률 스케줄: 직선 낮은 게인(꿀렁임 억제), 커브만 부스트
k_cross_eff = min(k_cross + k_cross_kappa*|kappa_eff|, k_cross_max)

# 합성 → 포화 → 직진 트림 → 출력 저역통과
raw   = steering_sign*(k_ff*kappa_eff + k_cross_eff*e + k_heading*psi + k_deriv*deriv_ema)
raw   = clip(raw, -1, 1)
steer = clip(raw + steer_trim, -1, 1)
steer = (1-beta)*prev_steer + beta*steer                 # beta = steering_smoothing
```

**실차 채택 게인**(`controller.yaml`, 07-14 확정)과 구간 프로파일:

| 파라미터 | baseline | POST_SIGN | 스키마 기본 |
|---|---|---|---|
| `k_ff` (곡률 FF) | 0.10 | (상속) | 0.0 |
| `curvature_smoothing` | 0.5 | (상속) | 0.3 |
| `curvature_deadband` | 0.4 | (상속) | 0.0 |
| `k_cross` (crosstrack P) | 0.47 | 0.47 (의도적 상속) | 1.2 |
| `k_cross_kappa` | 0.85 | (상속) | 0.0 |
| `k_cross_max` | 1.2 | (상속) | 1.5 |
| `k_heading` | 0.30 | **0.231** | 0.8 |
| `k_deriv` (crosstrack D) | 0.05 | **0.10** | 0.0 |
| `steering_smoothing` (β) | 0.45 | **0.55** | 0.3 |

- **곡률 피드포워드** — 순수 PD는 커브를 오차로만 버텨 정상상태 횡오차가 남는다. 곡률은 측정 가능한 외란이므로 필요한 조향을 미리 얹는다. 적분항과 달리 지연·와인드업이 없다. 커브에서 `k_cross_eff`는 `0.47 + 0.85·|κ_eff|`로 최대 1.2까지 부스트된다(κ_eff≈0.7~1.5 → 약 1.06~1.2).
- **heading 항** — Stanley의 heading 항과 같은 역할의 자연 감쇠. 각도 기반이라 BEV 스케일 캘리브 오차에 강건하며, 커브 추종의 주 레버임을 실차에서 확인했다.
- **heading의 정의가 프로파일 분기의 근본 이유** — `ψ`는 접선이 아니라 **경로 최근접점 → 맨 끝점(차 앞 ~1m)의 현(chord) 각도**다(인지 §3). S자에는 선반영 이득이 있지만, 직선 뒤 90°(ㄱ자) 코너에서는 조기 조향으로 라인을 벗어난다. 그래서 판단이 팻말 통과 후 `gain_profile=POST_SIGN`을 지시하면 `k_heading`을 0.30→0.231로 낮춘다(절대값으로 관리).
- **프로파일 전환 시 내부 상태 유지** — `set_config()`는 게인만 교체하고 EMA·직전 조향·직전 오차 상태는 유지한다. 전환 순간이 코너 진입이면 필터 리셋이 조향 튐으로 직결되기 때문이다. POST_SIGN 프로파일은 baseline을 상속(`{**base, **override}`)하고 지정 키만 덮어쓴다.

---

## 5. 횡방향 — Pure Pursuit (대조군)

목표점을 차량 로컬 프레임 $(x_l, y_l)$로 변환한 뒤 기하학적으로 조향각을 구한다. [`core/control/pure_pursuit.py`](../d_racer_autonomous/core/control/pure_pursuit.py)

$$\alpha = \mathrm{atan2}(y_l, x_l), \qquad \delta = \arctan\frac{2L\sin\alpha}{\ell_{eff}}, \qquad L_d = \mathrm{clip}(\ell_{min} + K_v v - K_c \kappa,\ \ell_{min},\ \ell_{max})$$

$L_d$는 adaptive lookahead(빠르면 멀리 보고 안정, 급커브에선 가까이 봐 민첩). 실차 `ld_min=0.45`, `ld_max=1.2`, `K_v=0.2`, `K_c=0.3`, `steering_gain=0.75`. **조향식 분모는 설계식 $L_d$가 아니라 실제 목표점 거리 $\ell_{eff}$** 로, 경로 끝 목표점이 $L_d$보다 가까울 때의 안정성을 위해서다.

- 곡률 데드밴드(`curvature_deadband=0.3`)로 직선 노이즈를 격리한다. adaptive lookahead와 곡률 gain은 곡률 추정에 양의 피드백이 걸리면 발산할 수 있어(07-09 관측), 데드밴드로 끊고 곡률 gain(`use_curvature_gain`)은 실차에서 껐다.
- 시뮬레이션 검증과 실차 A/B의 대조군으로 유지했고, 실차에서는 BEV 원거리 노이즈 증폭 문제로 PD로 전환했다(§8.1).

---

## 6. 종방향 속도

[`core/control/speed_controller.py`](../d_racer_autonomous/core/control/speed_controller.py) — 목표속도는 곡률로 결정한다(조향과 완전 분리).

$$v_{target} = \mathrm{clip}(v_{max} - K_\kappa |\kappa|,\ v_{min},\ v_{max})$$

실차 `v_max=0.8`, `v_min=0.3`, `K_κ=0.5`. 속도 PID는 기본 off(`use_speed_pid=false`)라 목표속도를 그대로 명령으로 낸다.

- **상태별 속도 프로파일은 판단이 `speed_scale`로 처리**한다 — 팻말 분기 감속(`sign_branch_speed_scale=0.3`), 종료 램프(`sign_exit_ramp_sec=1.5`), 장애물 구간 감속 등. 컨트롤러 코드는 모든 상태에서 동일하고 파라미터만 바뀐다.
- **실차 Lateral PD 종방향은 개루프 고정 throttle** — `drive_throttle × speed_scale`을 `throttle_limit`로 클램프해 발행한다(`drive_throttle=0.04`≈1592µs, 물리 최저 주행펄스 1575µs 바로 위). `speed_controller`의 곡률→목표속도 계산은 Pure Pursuit 모드에서만 호출된다.
- **배터리 보정기**([`battery_compensator.py`](../d_racer_autonomous/core/control/battery_compensator.py)) — $throttle' = throttle \times \mathrm{clip}(V_{nom}/V_{bat},\ 0,\ g_{max})$, `V_nom=7.4`, `g_max=1.5`. "제어"가 아니라 "액추에이터 출력 정규화"로 보고 모터 드라이버 직전에 분리 배치했다(켜고 끄기 쉬움). 실차에서는 데드밴드 이동을 못 잡아 **비활성**(`enabled=false`)으로 두고 운용 규칙으로 해결했다(§8.2).

---

## 7. 시뮬레이션 검증과 그리드 튜닝

[`sim/tune.py`](../d_racer_autonomous/sim/tune.py) — 임의의 config 키를 격자로 지정해(`--param section.key=v1,v2,...`) 전 조합을 `itertools.product`로 시뮬 평가한다. "감"이 아니라 정량 탐색으로 파라미터를 골랐다.

$$J = \mathrm{rms\_cte} + 0.3\cdot\mathrm{max\_cte} + 100\cdot[\text{미완주}]$$

완주 조합이 항상 우선하도록 미완주에 큰 페널티(100)를 준다. 비용 오름차순 정렬 후 상위 N을 출력하고 `--save-best`로 YAML에 저장한다.

**시뮬 결과**(`architecture.md`, 정적 경로 · Pure Pursuit · 기본 차량 0.26m/24° 기준):

| 경로 | 안전 마진 | RMS CTE |
|---|---|---|
| circle | 3.38× | 0.13 cm |
| s_curve | 2.72× | 1.23 cm |
| sharp_s | 1.16× | 0.75 cm (그리드 튜닝) |
| figure_eight | 1.25× | 0.83 cm |
| rotary | 2.54× | 0.68 cm |

폐루프 s_curve 단위 테스트는 `rms_cte < 0.15`(15cm)를 통과 기준으로 둔다. 실차 사양(15.5°/0.175m)으로 재검증했을 때도 circle CTE 0.13cm 완주, sharp_s(δ_req 14.4°) 완주를 확인했다.

> 위 CTE 표는 옛 기본 차량(0.26m/24°) 기준이므로 실차 사양과 절대 수치가 다르다. Lateral PD는 시뮬레이터에 통합돼 있지 않아 정량 CTE 로그가 없고, 실차 A/B에서 "직선 사행이 Pure Pursuit보다 확실히 적다"는 비교로 채택했다.

---

## 8. 실차 문제와 해결

### 8.1 Pure Pursuit → 근거리 PD 전환

- **문제** BEV 원거리 영역은 픽셀당 실거리가 커서 노이즈가 증폭되는데, Pure Pursuit의 먼 룩어헤드가 정확히 그 구간을 참조 → 직선에서도 미세 사행.
- **해결** 컨트롤러에 횡방향 제어기 스위치를 두어 실차 A/B 비교 → 근거리 오프셋+헤딩 기반 **Lateral PD** 채택. `k_heading`이 커브 주 레버임을 확인하고, 90° 코너 대응으로 코너 전용 게인 프로파일(POST_SIGN)을 도입했다(§4).

### 8.2 배터리 전압에 따른 스로틀 드리프트

- **문제** 배터리가 닳으면 같은 스로틀에도 차속이 달라진다. 게인 보정 모델로는 ESC **데드밴드 이동**을 잡을 수 없음을 실험으로 확인.
- **해결** 게인 보정(battery_compensator)을 끄고 **운용 규칙으로 고정** — 스로틀 캘리브레이션 기준 배터리(약 61%)를 명문화하고 항상 그 상태에서 주행. 단순하지만 대회에서 가장 확실한 답이었다.

### 8.3 fail-safe

차선 경로/상태가 `lane_timeout(0.3s)`을 넘게 낡으면 곡률을 0으로 폴백하거나 정지시킨다. 정지선 개루프 기동([판단 §6](planning.md#6-정지선-개루프-기동)) 종료 edge에서는 `lateral_pd.reset()`을 호출해 폐루프 복귀 시 조향 튐을 막는다.
