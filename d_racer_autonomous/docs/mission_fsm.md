# D-Racer 미션 상태기계 (로터리 지름길, 12-state)

> 작성 2026-07-05, 개정 2026-07-06(트랙 상세 반영: 12-state, 첫 정지선=오른쪽/둘째=왼쪽).
> 대회 미션을 **상위 미션 SM(MissionSequencer)** 로 정리한다. 구현: `core/planning/mission.py`.
> 관련: 계약 `interfaces.md`, 합의 `perception_agreement.md`, 진행 `../PROGRESS.md`.

---

## 1. 미션 개요 (트랙)

전체 외곽을 도는 게 아니라, 하단 직선에서 **중앙 원형 로터리로 올라가는 노란 지름길**을 쓴다.

```
출발(체커보드) → 하단 직선(흰) → 노란 지름길 → 로터리(노랑)
  → [정지선 1회: 오른쪽 원형으로 계속] → 한 바퀴 → [정지선 2회: 왼쪽 출구로 탈출]
  → 노란 점선/연결도로 → 흰 외곽도로 → 오른쪽 빨강 장애물 구간(아루코)
  → 하단 복귀 → 도착(체커보드) 정지
```

- **색**: 지름길·로터리·탈출 연결도로 = **노랑**, 외곽 = **흰**, 장애물 바닥 구역 = **빨강**.
- **한 바퀴 판정**: yaw 누적각 X, **로터리 내부 정지선 검출 횟수만** 사용.
- **분담**: 신호등·체커보드=YOLO, 차선/색/정지선/빨강=OpenCV, 아루코=cv2.aruco.

---

## 2. 2계층 구조

- **상위 = 미션 SM(12-state)** — "지금 미션 어디쯤인지" + **인지 지시**(follow_color/roi_mode/turn_bias).
- **하위 = 반응형 SM** — 각 주행 상태 안에서 DRIVE/SLOW/STOP/LOST (`core/planning/decision.py`).
- **역할 경계**: ROI 자르기·mask·target·bias·아루코 검출은 **인지 몫**. 상위는 지시만, 하위는 눈앞 반응만.

---

## 3. 12-state 상태 다이어그램

```
 WAIT_START_SIGNAL   (정지)      ── 초록불(YOLO) ──▶ START_STRAIGHT
 START_STRAIGHT      (흰,LOWER)  ── 노랑 검출 ──▶ SHORTCUT_APPROACH
 SHORTCUT_APPROACH   (노랑,LOWER,감속) ── 타이머 ──▶ ROUNDABOUT_ENTRY
 ROUNDABOUT_ENTRY    (노랑,FULL,정지선무시) ── 타이머(ignore) ──▶ ROUNDABOUT_FOLLOW
 ROUNDABOUT_FOLLOW   (노랑,FULL,정지선 카운트)
        │  1번째 정지선(count=1) ──▶ ROUNDABOUT_CONTINUE_RIGHT
        │  2번째 정지선(count=2) ──▶ ROUNDABOUT_EXIT_LEFT
 ROUNDABOUT_CONTINUE_RIGHT (노랑,RIGHT ROI,turn=RIGHT) ── 타이머 ──▶ ROUNDABOUT_FOLLOW
 ROUNDABOUT_EXIT_LEFT      (노랑,LEFT ROI,turn=LEFT)   ── 타이머 ──▶ EXIT_CONNECTOR
 EXIT_CONNECTOR      (노랑,LEFT,점선 크립) ── 흰색 안정 검출 ──▶ OUTER_LANE_FOLLOW
 OUTER_LANE_FOLLOW   (흰,FULL)   ── 빨강 구역 ──▶ DYNAMIC_OBSTACLE_ZONE
 DYNAMIC_OBSTACLE_ZONE (흰,LOWER_ARUCO,감속)
        │  아루코 보임 → STOP,  안 보임 → 주행
        └─ 빨강 벗어남 ──▶ FINISH_APPROACH
 FINISH_APPROACH     (흰,LOWER)  ── 체커보드(YOLO) ──▶ FINISH_STOP
 FINISH_STOP         (정지, 종료)
```

### 정지선 처리 (핵심)
- `roundabout_stopline_count` 초기 0. **ROUNDABOUT_FOLLOW에서만** 카운트.
- 1번째(0→1): **왼쪽 출구 무시, 오른쪽 원형 차선으로 계속** → CONTINUE_RIGHT.
- 2번째(→2): **오른쪽 버리고 왼쪽 출구로 탈출** → EXIT_LEFT.
- **중복 방지**: rising-edge(False→True) + `stopline_debounce_sec`(1.5~2.0s).
- **오검출 방지**: 진입 직후 `stopline_ignore_after_entry_sec` 동안 무시(ROUNDABOUT_ENTRY 지속).
- CONTINUE_RIGHT/EXIT_LEFT 중에는 카운트하지 않음.

### 탈출 후 (커넥터)
- 탈출하면 바로 흰색이 아니라 **노란 점선 → 노란 연결도로 → 흰색** 순.
- EXIT_CONNECTOR에서 노랑 계속 추종, **흰색이 `white_stable_sec` 이상 안정 검출**되면 OUTER.
- 점선 구간은 검출 끊겨도 **정지 금지**(왼쪽 저속 크립, target 유지는 인지 몫).

---

## 4. 인지 지시 (상태 → LaneMode) 요약

| 상태 | follow_color | roi_mode | turn_bias |
|------|:---:|:---:|:---:|
| START_STRAIGHT | WHITE | LOWER | - |
| SHORTCUT_APPROACH | YELLOW | LOWER | - |
| ROUNDABOUT_ENTRY/FOLLOW | YELLOW | FULL | - |
| ROUNDABOUT_CONTINUE_RIGHT | YELLOW | RIGHT | RIGHT |
| ROUNDABOUT_EXIT_LEFT | YELLOW | LEFT | LEFT |
| EXIT_CONNECTOR | YELLOW | LEFT | LEFT |
| OUTER_LANE_FOLLOW | WHITE | FULL | - |
| DYNAMIC_OBSTACLE_ZONE | WHITE | LOWER_ARUCO | - |

(WAIT_START_SIGNAL·FINISH_STOP = 정지, FINISH_APPROACH = WHITE/LOWER.)

---

## 5. 하위 반응형 오버레이 (주행 상태 공통)

각 주행 상태 안에서 아래층이 동작: 차선 신뢰도↓→SLOW, 소실>grace→LOST(정지·조향유지),
복귀>recover_grace→주행. 상위는 여기에 감속(slow_speed_scale)·짧은 lookahead
(roundabout_lookahead_scale)만 얹는다. 로터리 정지선은 카운트 전용이라 아래층엔 마스킹.

---

## 6. 인터페이스 (계약 반영됨, interfaces.md)

- 인지→판단: `lane_path`(Path) + `lane_status`(+노랑/흰 검출) + `mission_cues`(신호등/체커보드/빨강/아루코).
- 판단→인지: `lane_mode`(follow_color/roi_mode/turn_bias) — 역방향 지시.
- 메시지는 `racer_msgs`에 정의·빌드됨. 상세: `perception_agreement.md`.

## 7. 미확정 / 트랙 실측 (🏁)
- 갈림길 실제 좌/우(`roundabout_continue_side`/`exit_side`), 각 타이머값.
- 진입 직선이 출구 정지선 바로 직전인지(정지선 2회=한 바퀴).
- 장애물 "일정 프레임" 기준, 체커보드 검출 거리.
