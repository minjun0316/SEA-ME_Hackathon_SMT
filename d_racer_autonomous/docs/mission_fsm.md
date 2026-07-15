# D-Racer 미션 상태기계 (6-state)

> 작성 2026-07-05, 개정 2026-07-14(**흰선 폐루프 재설계**: 지름길 노랑추종·로터리·체커보드
> 도착 폐기 → 방향 팻말 분기 + 빨간불 종료 추가). 이전 개정 07-13(로터리 ROI→고정조향).
> 대회 미션을 **상위 미션 SM(MissionSequencer)** 로 정리한다. 구현: `core/planning/mission.py`.
> 관련: 계약 `interfaces.md`, 합의 `perception_agreement.md`, 진행 `../PROGRESS.md`.

---

## 1. 미션 개요 (트랙)

지름길(중앙 로터리)은 쓰지 않는다. **흰 차선만 따라 끝까지 폐루프**를 돌고, S자 코스 뒤
방향 팻말로 좌/우 갈림을 타며, 마지막에 빨간불로 종료한다.

```
출발(초록불) → S자 흰선 주행 → 방향 팻말 분기(좌/우) → 흰선 주행
  → 동적 장애물(아루코: 정지→재출발) → 빨간불 정지(종료)
```

- **S자 코스 = 흰 차선 양쪽** 슬라이딩윈도우 추종(인지). 노랑 차선추종은 폐기(07-14).
- **방향 팻말**: OpenCV로 팻말 고유색이 잡히면 구간 진입(트리거), 그때 **팻말 YOLO(별도
  모델)** 를 켜 좌/우를 판정한다. 흰선 추종은 유지한 채 조향에 bias만 살짝 얹고, 고정시간
  경과하면 복귀(1회성 래치라 재진입 없음).
- **종료 = 빨간불**: 아루코가 사라져 재출발한 뒤 신호등 YOLO로 빨간불을 보면 정지=끝.
  (예전 체커보드 도착 판정은 폐기 — 필드만 계약에 유지.)
- **분담**: 신호등·팻말방향=YOLO(모델 2개), 팻말색·차선=OpenCV, 아루코=cv2.aruco.

---

## 2. 로터리 / 정지선 기동 (이 코스에선 비활성)

로터리가 없으므로 제어단 개루프 고정조향 기동(`StoplineManeuver`)은 **끈다**
(`race.launch` 기본 `stopline_maneuver:=False`, 07-14). 코드·config는 재사용 위해 남겨둔다
(로터리 코스 복귀 시 `stopline_maneuver:=True`). 정지선 검출/반응(아래층 STOP)도 이 코스엔
정지선이 없어 트리거되지 않는다(코드는 유지).

---

## 3. 2계층 구조

- **상위 = 미션 SM(6-state)** — "지금 미션 어디쯤인지" + **인지 지시**(roi_mode/yolo_enable/
  sign_enable) + **제어 지시**(steer_bias/speed_scale).
- **하위 = 반응형 SM** — 각 주행 상태 안에서 DRIVE/SLOW/STOP/LOST (`core/planning/decision.py`).
- **역할 경계**: ROI 자르기·mask·target·아루코/팻말/신호등 검출은 **인지 몫**. 상위는 지시만.

---

## 4. 6-state 상태 다이어그램

```
 WAIT_START_SIGNAL  (정지, 신호등YOLO ON)   ── 초록불 ──▶ LANE_FOLLOW
 LANE_FOLLOW        (흰,LOWER)              ── 팻말색(sign_detected) ──▶ SIGN_BRANCH
      │                                     ── 아루코 ──▶ OBSTACLE_ZONE
 SIGN_BRANCH        (흰+bias,감속,팻말YOLO ON)── 고정시간 경과 ──▶ LANE_FOLLOW
      │  (팻말 YOLO 좌/우 래치 → steer_bias 얹음. 1회성 래치 _sign_done로 재진입 방지.
      │   분기 중 아루코 보이면 OBSTACLE_ZONE 우선.)
 OBSTACLE_ZONE      (흰,LOWER_ARUCO,감속)    아루코 보임 → STOP, 안 보임 → 재출발
      │                                     ── 아루코 사라짐(체류≥min_dwell) ──▶ FINISH_WATCH
      │                                     ── 아루코 사라짐(체류<min_dwell) ──▶ LANE_FOLLOW
      │  (후자 = 오검출 판정. FINISH_WATCH는 편도라 오검출 1프레임이 팻말 분기를 스킵하고
      │   첫 빨간불에 코스를 끝내던 것을 막는다. _yolo_relatch도 함께 되돌린다.)
 FINISH_WATCH       (흰,LOWER,신호등YOLO ON) ── 빨간불 ──▶ FINISH_STOP
 FINISH_STOP        (정지, 종료)
```

- 값(IntEnum): WAIT_START_SIGNAL=0, LANE_FOLLOW=1, SIGN_BRANCH=2,
  OBSTACLE_ZONE=3, FINISH_WATCH=4, FINISH_STOP=5.
- **팻말 분기**: `sign_detected`(OpenCV 팻말색)로 SIGN_BRANCH 진입 → `sign_enable`로 팻말
  YOLO를 켜 `sign_direction`(좌/우)을 받는다. 미션은 첫 non-NONE 방향을 래치하고 흰선 추종
  위에 `steer_bias`(트림 전 raw, +=좌/-=우)를 얹는다. `sign_branch_duration`(config) 경과 시
  `_sign_done`을 세우고 복귀 — 이후 팻말색을 또 봐도 재진입하지 않는다(1회성).
- **YOLO 게이트(모델 2개 독립)**: 신호등 YOLO = WAIT_START_SIGNAL + (아루코 최초검출
  래치로) FINISH_WATCH까지 ON, 주행중 OFF. 팻말 YOLO = SIGN_BRANCH에서만 ON.
- **아루코 오검출 방어(07-15)**: 인지 `aruco_hold_sec`(1.0)와 판단 `obstacle_min_dwell_sec`
  (2.0)는 **짝으로 튜닝한다**. hold가 present를 늘려주므로 OBSTACLE_ZONE 체류시간은
  (실제 검출시간 + hold)에 해당하고, 실효 요구치 = `min_dwell - hold` = "심판이 마커를
  실제로 들고 있어야 하는 시간"(현재 1.0s). 한쪽만 바꾸면 방어가 깨진다.

---

## 5. 인지·제어 지시 (상태 → LaneMode / DriveCommand) 요약

| 상태 | roi_mode | yolo_enable | sign_enable | speed | steer_bias |
|------|:---:|:---:|:---:|:---:|:---:|
| WAIT_START_SIGNAL | LOWER | ON | off | 정지 | 0 |
| LANE_FOLLOW | LOWER | off | off | 1.0 | 0 |
| SIGN_BRANCH | LOWER | off | **ON** | 감속 | ±값 |
| OBSTACLE_ZONE | LOWER_ARUCO | ON* | off | 정지/감속 | 0 |
| FINISH_WATCH | LOWER | ON | off | 1.0 | 0 |
| FINISH_STOP | LOWER | ON | off | 정지 | 0 |

(*OBSTACLE_ZONE의 신호등 YOLO는 아루코 최초검출 시 래치되어 ON. follow_color는 항상 WHITE.
인지는 LaneMode의 roi_mode/follow_color/turn_bias를 아직 소비하지 않고, `yolo_enable`/
`sign_enable`만 게이트에 쓴다.)

---

## 6. 하위 반응형 오버레이 (주행 상태 공통)

각 주행 상태 안에서 아래층이 동작: 차선 신뢰도↓→SLOW, 소실>grace→LOST(정지·조향유지),
복귀>recover_grace→주행. 상위는 SIGN_BRANCH/OBSTACLE_ZONE에 감속 배율(`sign_branch_speed_scale`
/`slow_speed_scale`)만 얹고, SIGN_BRANCH엔 steer_bias를 더한다.

---

## 7. 인터페이스 (계약, interfaces.md)

- 인지→판단: `lane_path`(Path) + `lane_status` + `mission_cues`(신호등/아루코/팻말색·방향).
- 판단→인지: `lane_mode`(roi_mode/yolo_enable/**sign_enable**) — 역방향 지시.
- 판단→제어: `drive_command`(state/go/speed_scale/steer_limit/**steer_bias**).
- 메시지는 `racer_msgs`에 정의·빌드됨(추가 필드 [iface 2026-07-14]). 상세: `perception_agreement.md`.
- ⚠️ 폐기됐지만 계약에 남은 필드: `lane_status.on_yellow`(항상 False), `mission_cues.
  checkerboard_detected`/`red_zone_detected`, `lane_mode.follow_color`/`roi_mode`/`turn_bias`.

## 8. 미확정 / 트랙 실측 (🏁)
- **팻말 좌/우 YOLO 모델**(학습중): 완성 후 `mission_cues.yaml` `sign_model_path`+class id 지정.
- **팻말색 HSV**(`sign_color_lo/hi`): 대회 팻말 실측색으로 교체(현재 파란팻말 잠정).
- `steer_bias_value`·`sign_branch_duration` 실차 튜닝(갈림길을 타되 흰선 이탈 없게).
- 초록 출발 신호 검출 신뢰도(모델), 아루코 검출 거리.
