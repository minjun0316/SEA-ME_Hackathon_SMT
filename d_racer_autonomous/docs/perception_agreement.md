# 인지팀 합의 체크리스트 (인지 ↔ 판단/제어)

> 작성 2026-07-06. **판단/제어(D-Racer)** 가 인지팀과 확정해야 할 항목 모음.
> 분담: **객체(신호등·체커보드) = YOLO**, **차선·색·정지선·빨강구역 = OpenCV**, **아루코 = cv2.aruco**.
> 관련: 계약 `interfaces.md`, 미션 `mission_fsm.md`, 진행 `../PROGRESS.md`.
> 표기: ✅합의 · ⚠️확정필요 · 🏁트랙실측.

핵심 원칙: **ROI 자르기·색 mask·target point·bias·아루코 검출·디버그 시각화는 인지(팀원) 몫.**
판단은 상태에 맞는 **지시(follow_color/roi_mode/turn_bias)** 만 주고, 인지가 그대로 적용해
`lane_path`로 되돌린다(왕복 계약).

---

## A. 인지 → 판단 : 매 프레임 주행 기하 (반드시)

| 토픽/필드 | 타입 | 설명 |
|---|---|---|
| `/perception/lane_path` | nav_msgs/Path | 지시받은 ROI/색/bias 반영해 뽑은 **따라갈 중심선**. 컨트롤러(Pure Pursuit) 입력. base_link(뒷차축 원점), 가까운→먼 순, 점 간격≈0.05m. |
| `/perception/lane_status`.lane_detected/confidence/num_points/lateral_offset/heading_error | racer_msgs/LaneStatus | 반응형 감속/소실 판정 입력. |

> `target_x/target_y`, `last_valid_target`(끊길 때 유지)는 **인지 내부 처리** — 판단에 따로 안 줌.

## B. 인지 → 판단 : 미션 전환 신호

| 신호 | 어디(메시지) | 누가 | 어느 전환/동작 |
|---|---|---|---|
| `yellow_detected` + `yellow_confidence` | LaneStatus | OpenCV | START→SHORTCUT(노랑 등장) |
| `white_detected` + `white_confidence` | LaneStatus | OpenCV | EXIT_CONNECTOR→OUTER(흰색 안정) |
| `stop_line` | LaneStatus | OpenCV | 로터리 정지선 카운트(몇 번째인지는 판단이 셈) |
| `traffic_light`(NONE/RED/GREEN) | MissionCues | YOLO | WAIT→START(초록), FINISH(빨강) |
| `checkerboard_detected` | MissionCues | YOLO | FINISH_APPROACH→FINISH_STOP |
| `red_zone_detected` | MissionCues | OpenCV | OUTER→OBSTACLE(진입), OBSTACLE→FINISH(탈출) |
| `aruco_present` | MissionCues | cv2.aruco(하단 ROI) | OBSTACLE 정지/재출발 |

## C. 판단 → 인지 : 역방향 지시 (★ 새 경로, `/decision/lane_mode`)

| 필드 | 값 | 인지가 할 것 |
|---|---|---|
| `follow_color` | WHITE / YELLOW | 그 색 차선을 우선 추종 |
| `roi_mode` | FULL / LOWER / RIGHT / LEFT / LOWER_ARUCO | 그 ROI로 잘라서 mask·target 추출 |
| `turn_bias` | NONE / LEFT / RIGHT | target_x에 방향 bias(ROI만으로 부족할 때) |

판단이 각 상태에서 내보내는 지시(요약):

| 상태 | follow | roi_mode | turn_bias |
|---|---|---|---|
| START_STRAIGHT | WHITE | LOWER | - |
| SHORTCUT_APPROACH | YELLOW | LOWER | - |
| ROUNDABOUT_ENTRY/FOLLOW | YELLOW | FULL | - |
| ROUNDABOUT_CONTINUE_RIGHT | YELLOW | RIGHT | **RIGHT** |
| ROUNDABOUT_EXIT_LEFT | YELLOW | LEFT | **LEFT** |
| EXIT_CONNECTOR | YELLOW | LEFT | LEFT |
| OUTER_LANE_FOLLOW | WHITE | FULL | - |
| DYNAMIC_OBSTACLE_ZONE | WHITE | LOWER_ARUCO | - |

## D. 검출 신뢰도/정의 — 인지팀 확인 (⚠️/🏁)

| # | 항목 | 확인할 것 |
|---|------|-----------|
| D1 | 노랑↔흰 검출 | 두 색을 **동시에 항상** confidence로 보고 가능한가(조명·그림자 견고성). |
| D2 | 정지선 | "하단 중앙에 넓게 잡히는 수평선"으로 일반 차선과 구분. 폭/신뢰도도 주면 좋음. |
| D3 | 갈림길 추종(B5) | 판단이 준 roi_mode(RIGHT/LEFT)+turn_bias로 그쪽 갈래를 잡을 수 있는가. |
| D4 | 빨강 구역 | 진입/탈출 순간 안정적인가. 검출 시작 거리. |
| D5 | 아루코 | 바닥 가까이 → **하단 ROI 필수**. 마커 ID·크기, 안정 검출 거리. "일정 프레임" 기준. |
| D6 | 체커보드 | YOLO 클래스. 출발/도착 같은 무늬지만 판단은 FINISH_APPROACH에서만 확인(혼동 없음). |
| D7 | 좌표 | base_link=뒷차축 원점, +y=좌측. 카메라 오프셋은 인지가 보정. |

## E. 트랙 실측(🏁)

| # | 항목 |
|---|------|
| E1 | 첫 정지선=오른쪽 계속, 둘째=왼쪽 탈출 — 실제 갈림길 방향 확정(`roundabout_continue_side`/`exit_side`). |
| E2 | 타이머값(shortcut_approach_sec, ignore, continue_right, exit_left, white_stable) 실측 튜닝. |
| E3 | 진입 직선이 출구 정지선 바로 직전인지(정지선 2회=한 바퀴 성립). |

---

## 정리 — 최소 vs 전체
- **폐루프 최소(지금 통합 가능)**: A(lane_path + lane_status 기하). decision_node가 이미 소비.
- **미션 전체**: B(LaneStatus 색 + MissionCues) + C(LaneMode 역방향). 메시지는 `racer_msgs`에 정의됨
  (LaneStatus 확장, MissionCues/LaneMode 신규, 빌드 완료). 인지팀이 발행/구독만 붙이면 됨.
