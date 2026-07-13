# 인지팀 합의 체크리스트 (인지 ↔ 판단/제어)

> 작성 2026-07-06. **판단/제어(D-Racer)** 가 인지팀과 확정해야 할 항목 모음.
> 분담: **객체(신호등·체커보드) = YOLO**, **차선·색·정지선·빨강구역 = OpenCV**, **아루코 = cv2.aruco**.
> 관련: 계약 `interfaces.md`, 미션 `mission_fsm.md`, 진행 `../PROGRESS.md`.
> 표기: ✅합의 · ⚠️확정필요 · 🏁트랙실측.

핵심 원칙: **ROI 자르기·색 mask·target point·bias·아루코 검출·디버그 시각화는 인지(팀원) 몫.**
판단은 상태에 맞는 **지시(follow_color/roi_mode/turn_bias)** 만 주고, 인지가 그대로 적용해
`lane_path`로 되돌린다(왕복 계약).

> **07-13 변경**: 로터리 회전을 **제어단 고정조향 기동**(StoplineManeuver, `controller_node`)으로
> 전환했다. 그 결과 판단은 더 이상 로터리용 `follow_color=YELLOW`·`roi_mode=RIGHT/LEFT`·`turn_bias`를
> 내지 않는다(항상 WHITE·FULL/LOWER류·NONE). 아래 필드/값은 msg 계약엔 남아 있으나 현재 미사용이다.
> 인지팀 영향: 갈림길 좌/우 ROI·bias 추종(D3) 구현은 **불필요**해졌다.

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
| `stop_line` | LaneStatus | OpenCV | **로터리 정지선 카운트 — 제어단 StoplineManeuver가 셈**(고정조향 기동 트리거). |
| `traffic_light`(NONE/RED/GREEN) | MissionCues | YOLO | WAIT_START_SIGNAL→LANE_FOLLOW(초록) |
| `checkerboard_detected` | MissionCues | YOLO | FINISH_APPROACH→FINISH_STOP |
| `red_zone_detected` | MissionCues | OpenCV | LANE_FOLLOW→OBSTACLE(진입), OBSTACLE→FINISH(탈출) |
| `aruco_present` | MissionCues | cv2.aruco(하단 ROI) | OBSTACLE 정지/재출발 |
| ~~`yellow_detected`/`white_detected`~~ | ~~LaneStatus~~ | — | **삭제(07-13)**: 로터리 ROI 방식 폐기로 소비처 없음 → LaneStatus msg에서 제거. 노랑/흰 마스크는 인지 내부 검출에만 사용. |

## C. 판단 → 인지 : 역방향 지시 (★ 새 경로, `/decision/lane_mode`)

| 필드 | 값(계약) | 인지가 할 것 | 현재 판단이 내는 값 |
|---|---|---|---|
| `follow_color` | WHITE / YELLOW | 그 색 차선을 우선 추종 | **항상 WHITE** |
| `roi_mode` | FULL / LOWER / RIGHT / LEFT / LOWER_ARUCO | 그 ROI로 잘라서 mask·target 추출 | FULL / LOWER / LOWER_ARUCO만 |
| `turn_bias` | NONE / LEFT / RIGHT | target_x에 방향 bias | **항상 NONE** |

> RIGHT/LEFT roi_mode·YELLOW·turn_bias는 로터리 ROI 방식 전용이었고 07-13 폐기됨. 값은 msg에
> 남아 있으나 미션 FSM이 발행하지 않는다(인지는 FULL/LOWER/LOWER_ARUCO·WHITE·NONE만 받게 됨).

판단이 각 상태에서 내보내는 지시(요약):

| 상태 | follow | roi_mode | turn_bias |
|---|---|---|---|
| LANE_FOLLOW | WHITE | LOWER | - |
| DYNAMIC_OBSTACLE_ZONE | WHITE | LOWER_ARUCO | - |
| FINISH_APPROACH | WHITE | LOWER | - |

(WAIT_START_SIGNAL·FINISH_STOP = 정지.)

## D. 검출 신뢰도/정의 — 인지팀 확인 (⚠️/🏁)

| # | 항목 | 확인할 것 |
|---|------|-----------|
| D1 | 노랑↔흰 검출 | 두 색을 **동시에 항상** confidence로 보고 가능한가(조명·그림자 견고성). |
| D2 | 정지선 | "하단 중앙에 넓게 잡히는 수평선"으로 일반 차선과 구분. 폭/신뢰도도 주면 좋음. |
| D3 | ~~갈림길 추종~~ | **폐기(07-13)**: 로터리 회전은 제어단 고정조향 기동이 처리 → roi RIGHT/LEFT·bias 추종 불필요. |
| D4 | 빨강 구역 | 진입/탈출 순간 안정적인가. 검출 시작 거리. |
| D5 | 아루코 | 바닥 가까이 → **하단 ROI 필수**. 마커 ID·크기, 안정 검출 거리. "일정 프레임" 기준. |
| D6 | 체커보드 | YOLO 클래스. 출발/도착 같은 무늬지만 판단은 FINISH_APPROACH에서만 확인(혼동 없음). |
| D7 | 좌표 | base_link=뒷차축 원점, +y=좌측. 카메라 오프셋은 인지가 보정. |

## E. 트랙 실측(🏁)

| # | 항목 |
|---|------|
| E1 | 로터리 고정조향 방향/크기/시간 실측(`stopline_maneuver`: first_dir/second_dir/steer/duration_sec). |
| E2 | 정지선 debounce·max_count, 장애물 "일정 프레임" 기준, 체커보드 검출 거리. |
| E3 | 진입 직선이 출구 정지선 바로 직전인지(정지선 2회=한 바퀴 성립). |

---

## 정리 — 최소 vs 전체
- **폐루프 최소(지금 통합 가능)**: A(lane_path + lane_status 기하). decision_node가 이미 소비.
- **미션 전체**: B(LaneStatus 색 + MissionCues) + C(LaneMode 역방향). 메시지는 `racer_msgs`에 정의됨
  (LaneStatus 확장, MissionCues/LaneMode 신규, 빌드 완료). 인지팀이 발행/구독만 붙이면 됨.
