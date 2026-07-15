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

## 3. throttle 중립/데드존 측정 (거치대)
바퀴가 막 돌기 시작하는 최소 throttle을 찾는다(거치대, 바퀴 띄움).
```bash
ros2 launch racer_bringup calibration.launch.py mode:=throttle_pulse pulse_throttle:=0.08
# 안 돌면 0.02씩 올리며 반복 (throttle_limit 안에서)
ros2 param set /calibration_node pulse_throttle 0.10
```
- 바퀴가 돌기 시작하는 값 = **throttle 데드존**(무부하). 배관·방향 확인용.
- ⚠ 이 값으로 `THROTTLE_FWD_START_US`를 정하지 말 것 — 바퀴가 뜬 상태는 부하가
  없어 실제보다 낮게 나온다. 그건 아래 3-b(바닥)에서 잰다.

---

## 3-b. 최저 주행속도 측정 (바닥) — `THROTTLE_FWD_START_US` 확정 [07-15]

> **왜 필요한가**: "throttle을 0.05로 줘도 너무 빠르다"의 원인은 `drive_throttle`이
> 아니라 `src/config/vehicle_config.yaml`의 `THROTTLE_FWD_START_US`다.
> `d3racer.py`의 매핑이 `pulse = FWD_START_US + p × (2000 − FWD_START_US)` 라서,
> 현재값 1650이면 **p가 0을 조금만 넘는 순간 펄스가 중립 1500 → 1650으로 150µs
> 점프**한다. 그 위로는 p=0.05든 0.20이든 겨우 53µs 차이다. 즉 **최저 속도는
> FWD_START_US가 정하고 `drive_throttle`은 거의 무의미**하다.
>
> 배터리 보정으로는 이 문제를 못 고친다: 보정은 `p`만 스케일하는데 저속에서
> 펄스를 지배하는 1650 오프셋은 스케일되지 않는다(gain 0.88 적용 시 1667.5µs →
> 1665.4µs, **차이 2µs**). 속도를 먼저 잡고 나서 배터리 보정을 논해야 한다.

### 원리 — p를 펄스로 직접 읽기
측정 중에만 `THROTTLE_FWD_START_US: 1500`(= 보상 off, 중립부터 선형)으로 두면
매핑이 `pulse = 1500 + p × 500`이 되어 **p가 곧 펄스**가 된다.

| p | pulse |
|---|---|
| 0.10 | 1550µs |
| 0.20 | 1600µs |
| 0.30 | 1650µs |
| 0.35 | 1675µs |
| 0.40 | 1700µs |

`FWD_START_US = 1500 + p × 500` 로 환산한다.

### ⚠️ 측정 전에 — 배터리 충전 상태를 반드시 기록한다 [07-15]
**이 측정값은 잰 그때의 충전 상태에서만 유효하다.** 실측 사례(07-15): 배터리를 많이
충전했더니 같은 `FWD_START_US`인데 눈에 띄게 빨라졌다. 같은 펄스라도 전압이 높으면
ESC 출력이 커지기 때문이다(= 전압 효과는 실재한다. 다만 현재의 `BatteryCompensator`는
`p`를 스케일해서 이걸 못 잡는다 — 위 "왜 필요한가" 참조).

```bash
# 측정 직전·직후 둘 다 기록 (bench_calibration.launch.py가 battery_node를 함께 띄운다)
ros2 topic echo /battery_status --once
```
- `battery_status`는 **볼트가 아니라 %**다(0~100). 환산: `V ≈ 6.4 + pct/100 × 2.0`.
- 측정 결과를 아래 4절 표에 적을 때 **%를 같이 남긴다.** 안 남기면 재현 불가.

> **어느 충전 상태에서 재야 하나** — 정답이 없다. 만충에서 맞추면 배터리가 닳을수록
> 느려지다 데드밴드에 걸려 안 나가고, 저충전에서 맞추면 만충일 때 너무 빠르다.
> 보정이 없는 한 이 드리프트는 못 없앤다. **실제로 주행할 충전 상태에서 잰다.**

### ⛳ 운용 규칙 — 주행 배터리는 60% 언저리 [07-15 결정]
**현재 값(`FWD_START_US=1575`, `drive_throttle=0.04`)은 배터리 61.4%에서 잰 것이다.
주행 전에 배터리를 60% 언저리(±5%)로 맞춘다.** 배터리 보정을 만드는 대신 배터리
상태를 고정하는 쪽을 택했다(07-15).

```bash
ros2 topic echo /battery_status --once   # 주행 전 확인. 60±5% 밖이면 아래 참조.
```

**왜 보정 대신 이 방식인가**
- 엔코더가 없어 보정이 듣는지 **검증할 수단이 없다**(스톱워치 눈대중이 전부).
- `battery_node`가 부하 전압을 내보내서, 보정을 걸면 `스로틀↑ → sag → gain↑ → 스로틀↑`
  **양의 피드백** 위험이 있다.
- 결정적으로 **단순 gain 보정(`V_ref/V`)으로는 이 문제가 안 풀린다.** 아래 참조.

**⚠ 60%를 크게 벗어나면 재측정이 필요하다 — 단순 환산으로 때우면 안 된다**
전압만 보면 61%→80%는 7.63V→8.00V = **5%** 빨라지는 것으로 보인다. 하지만 현재
`drive_throttle=0.04`(1592µs)는 떼는 값(1575µs)보다 **겨우 17µs 위**라, 속도가 펄스에
대해 매우 가파른 구간에 앉아 있다. 게다가 전압이 오르면 **떼는 값 자체가 내려간다**
(힘이 세져 더 낮은 펄스로도 움직임) → 문턱 대비 여유가 17µs에서 22µs로 벌어지면
체감은 5%가 아니라 **30%**가 된다. 실제로 "충전했더니 확 빨라졌다"가 이 현상이다.

즉 지배적 효과는 **선형 스케일이 아니라 데드밴드 문턱의 이동**이고, `V_ref/V` gain
모델에는 그 항이 없다. 제대로 보정하려면 `FWD_START_US`를 전압의 함수로 만들어야 하고,
그러려면 **최소 2~3개 충전 상태에서 `p_break`를 재서 표**를 만들어야 한다(미실시).

### 준비
```bash
# 1) 측정용으로 보상 임시 off
#    src/config/vehicle_config.yaml → THROTTLE_FWD_START_US: 1500  (끝나면 원복/갱신)

# 2) 노드 기동 (control_node가 yaml을 init에서 읽으므로 반드시 수정 후에 띄운다)
ros2 launch racer_bringup bench_calibration.launch.py mode:=hold throttle_limit:=0.5

# 3) 직진시키기 — calibration_node는 raw /control을 쏘므로 steer_trim이 안 실린다
ros2 param set /calibration_node steering 0.2238   # docs 기준 트림값
```

> ⚠️ **`throttle_limit:=0.5` 필수.** 기본값 0.15는 보상 off 상태에서 최대 1575µs라
> 차가 **아예 안 움직인다** — 이걸 모르면 "더 못 늦춘다"고 잘못 결론 내리기 쉽다.
> 그래도 안전을 위해 필요 이상 올리지 말 것(0.5 = 1750µs).

### 재는 것 — 두 값
`hold` 모드는 `throttle` 파라미터를 매 틱 실시간으로 읽으므로 노드 재시작 없이 올린다.

```bash
ros2 param set /calibration_node throttle 0.20   # 0.02씩 올리며 반복
```

1. **떼는 p (`p_break`)** — 정지 상태에서 차가 **처음 움직이기 시작하는** p.
   0.20부터 0.02씩 올린다. (과거 실측 주석 기준 0.35 근처 예상 = 1675µs)
2. **유지되는 최저 p (`p_sustain`)** — 움직이기 시작한 **뒤에** p를 도로 내리면서
   차가 굴러가기를 멈추는 직전 값. 정지마찰 < 운동마찰이라 `p_break`보다 낮을 수 있다.

### 판정 — 여기서 갈린다
- **`p_sustain` < `p_break` 로 뚜렷하게 낮으면** → 저속 여지가 있다.
  `FWD_START_US = 1500 + p_sustain × 500` 근처로 내린다(출발이 굼뜨면 조금 ↑).
  다만 이러면 **출발 시 정지마찰을 못 이길 수 있으니** 반드시 정지→출발을 반복 확인.
- **둘이 비슷하면** → 개루프로는 여기가 이 차의 물리적 최저 속도다. FWD_START_US를
  더 내리면 그냥 안 나간다. 그땐 스로틀 on/off 듀티(`throttle_pulse` 모드가 하는 일)나
  기어비 쪽을 봐야 하고, **이건 별도 설계 합의가 필요하다.**

### 안전
- 바닥에서 하는 측정이다 — **joystick E-STOP을 손에 쥐고**, 벽/사람 없는 직선에서.
- 거치대에서 재면 부하가 없어 실제보다 낮게 나온다. **반드시 바닥에서.**
- 끝나면 `THROTTLE_FWD_START_US`를 측정값으로 갱신하거나 원래값(1650)으로 되돌린다.
  1500인 채로 주행하면 저속이 통째로 데드밴드에 먹혀 차가 안 나간다.

---

## 4. 측정값 반영
| 측정 항목 | 반영 위치 |
|-----------|-----------|
| 직진 steering 값 | `config/vehicle.yaml: steer_trim`, D-Racer `STEER_TRIM` |
| steering=±1.0 실제 각도 | `config/vehicle.yaml: max_steer_deg` |
| 휠베이스(자로 측정) | `config/vehicle.yaml: wheelbase` |
| throttle 데드존(거치대, 3절) | 배관 확인용 — 아래 값으로 대체됨 |
| **최저 주행 p_sustain (바닥, 3-b절)** | `src/config/vehicle_config.yaml: THROTTLE_FWD_START_US` = 1500 + p×500 |

### 실측 기록
| 날짜 | 배터리 | `p_break` | `p_sustain` | 확정 `FWD_START_US` |
|---|---|---|---|---|
| 07-15 | 61.4% | 0.15 (1575µs) | 0.14 (1570µs) | **1575** (이전 1650 = 75µs 과다) |

- `p_break ≈ p_sustain`(5µs 차) → 정지마찰 여유 없음 = **1575µs가 물리적 최저 주행 펄스**.
  개루프로 이보다 느리게는 못 간다. 더 느려야 하면 듀티/기어비 쪽 별도 설계 필요.
- 옛 주석의 "0.35까지 올려야 겨우 감"(1675µs)은 실측과 100µs 어긋나 **폐기**.

→ 이 값으로 `core.feasibility.check_path`를 다시 돌리면, 실제 차량 기준으로
경로가 추종 가능한지 정확히 판정된다(시뮬과 실차의 파라미터 일원화).

---

## 다음 단계 (Stage 6-b)
캘리브레이션이 끝나면, `core`를 감싼 `controller_node`(Pure Pursuit + Speed)를
만들어 **정적 경로**를 추종시키되, 카메라 pose가 없으므로 우선
"조향 명령이 경로 곡률에 맞게 나오는지"를 거치대/저속에서 확인한다.
실제 폐루프 주행은 Perception(카메라→로컬 차선중심선)이 pose를 제공한 뒤 가능하다.
```
