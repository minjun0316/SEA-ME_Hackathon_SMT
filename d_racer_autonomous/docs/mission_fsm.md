# D-Racer 미션 상태기계 (5-state)

> 작성 2026-07-05, 개정 2026-07-13(로터리 ROI 방식 폐기 → 제어단 고정조향 기동으로 전환).
> 대회 미션을 **상위 미션 SM(MissionSequencer)** 로 정리한다. 구현: `core/planning/mission.py`.
> 관련: 계약 `interfaces.md`, 합의 `perception_agreement.md`, 진행 `../PROGRESS.md`.

---

## 1. 미션 개요 (트랙)

전체 외곽을 도는 게 아니라, 하단 직선에서 **중앙 원형 로터리로 올라가는 지름길**을 쓴다.

```
출발(체커보드) → 차선 주행 → 로터리(정지선 카운트로 회전/탈출) → 외곽 주행
  → 빨강 장애물 구간(아루코) → 도착(체커보드) 정지
```

- **로터리 회전은 미션 FSM이 아니라 제어단이 처리**(§2). 미션 FSM은 로터리를 별도 페이즈로
  다루지 않고 일반 차선 주행(LANE_FOLLOW)으로 통과한다.
- **분담**: 신호등·체커보드=YOLO, 차선/정지선/빨강=OpenCV, 아루코=cv2.aruco.

---

## 2. 로터리 회전 = 제어단 고정조향 기동 (ROI 방식 폐기)

로터리 진입/탈출 회전은 **개루프 고정조향 기동**으로 처리한다:
`core/planning/stopline_maneuver.py`(`StoplineManeuver`) → `controller_node`에서 실행.

- 정지선을 rising-edge + debounce로 센다.
- 1번째 정지선 → `first_dir` 방향으로 `duration_sec` 동안 고정 조향(트림 전 raw).
- 2번째 정지선 → `second_dir` 방향으로 `duration_sec` 동안 고정 조향(탈출).
- 차선을 잠깐 놓쳐도 정해진 시간만큼 그대로 돌아 나간다(개루프).
- 튜닝값: `config/controller.yaml` 의 `stopline_maneuver`. 런치 인자 `stopline_maneuver:=True`로 켠다.

> 예전엔 미션 SM이 정지선을 세서 `roi_mode`를 좌/우로 전환하고 차선추종으로 도는 **ROI 방식**을
> 썼으나(07-13 폐기), 원거리 BEV 노이즈·색 구분 문제로 고정조향 기동으로 대체했다.

---

## 3. 2계층 구조

- **상위 = 미션 SM(5-state)** — "지금 미션 어디쯤인지" + **인지 지시**(follow_color/roi_mode).
- **하위 = 반응형 SM** — 각 주행 상태 안에서 DRIVE/SLOW/STOP/LOST (`core/planning/decision.py`).
- **역할 경계**: ROI 자르기·mask·target·아루코 검출은 **인지 몫**. 상위는 지시만, 하위는 눈앞 반응만.

---

## 4. 5-state 상태 다이어그램

```
 WAIT_START_SIGNAL     (정지)             ── 초록불(YOLO) ──▶ LANE_FOLLOW
 LANE_FOLLOW           (흰,LOWER)         ── 빨강 구역 ──▶ DYNAMIC_OBSTACLE_ZONE
      │  (로터리 구간 포함 — 회전은 controller StoplineManeuver 몫, FSM엔 투명)
 DYNAMIC_OBSTACLE_ZONE (흰,LOWER_ARUCO,감속)
      │  아루코 보임 → STOP,  안 보임 → 주행
      └─ 빨강 벗어남 ──▶ FINISH_APPROACH
 FINISH_APPROACH       (흰,LOWER)         ── 체커보드(YOLO) ──▶ FINISH_STOP
 FINISH_STOP           (정지, 종료)
```

- 값(IntEnum): WAIT_START_SIGNAL=0, LANE_FOLLOW=1, DYNAMIC_OBSTACLE_ZONE=2,
  FINISH_APPROACH=3, FINISH_STOP=4.
- 정지선은 미션 FSM 전이에 쓰지 않는다(제어단 StoplineManeuver 전용).

---

## 5. 인지 지시 (상태 → LaneMode) 요약

| 상태 | follow_color | roi_mode | turn_bias |
|------|:---:|:---:|:---:|
| LANE_FOLLOW | WHITE | LOWER | NONE |
| DYNAMIC_OBSTACLE_ZONE | WHITE | LOWER_ARUCO | NONE |
| FINISH_APPROACH | WHITE | LOWER | NONE |

(WAIT_START_SIGNAL·FINISH_STOP = 정지. follow_color는 항상 WHITE, turn_bias는 항상 NONE
— 로터리 노랑 추종·좌/우 bias를 쓰던 ROI 방식이 폐기됐기 때문.)

---

## 6. 하위 반응형 오버레이 (주행 상태 공통)

각 주행 상태 안에서 아래층이 동작: 차선 신뢰도↓→SLOW, 소실>grace→LOST(정지·조향유지),
복귀>recover_grace→주행. 상위는 장애물 구역에 감속(`slow_speed_scale`)만 얹는다.

---

## 7. 인터페이스 (계약, interfaces.md)

- 인지→판단: `lane_path`(Path) + `lane_status` + `mission_cues`(신호등/체커보드/빨강/아루코).
- 판단→인지: `lane_mode`(follow_color/roi_mode/turn_bias) — 역방향 지시.
- 메시지는 `racer_msgs`에 정의·빌드됨. 상세: `perception_agreement.md`.
- ⚠️ `lane_mode`의 roi_mode RIGHT/LEFT·turn_bias LEFT/RIGHT 값은 msg 계약엔 남아 있으나
  현재 미션 FSM은 발행하지 않는다(항상 FULL/LOWER/LOWER_ARUCO, NONE).

## 8. 미확정 / 트랙 실측 (🏁)
- 로터리 고정조향 방향/크기/시간(`stopline_maneuver`: first_dir/second_dir/steer/duration_sec).
- 정지선 debounce, 장애물 "일정 프레임" 기준, 체커보드 검출 거리.
