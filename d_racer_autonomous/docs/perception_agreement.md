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
| `on_yellow` | LaneStatus | OpenCV(인지 래치) | **LANE_FOLLOW→SHORTCUT**(지름길 노랑 진입). 인지가 노랑 안정검출로 노랑모드 래치→발행. |
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
| `turn_bias` | NONE / LEFT / RIGHT | **팻말의 어느 쪽 통로로 지날지**(중심선에 ±offset) | SIGN_BRANCH에서 **LEFT/RIGHT**, 그 외 NONE |

> RIGHT/LEFT roi_mode·YELLOW는 로터리 ROI 방식 전용이었고 07-13 폐기됨. 값은 msg에
> 남아 있으나 미션 FSM이 발행하지 않는다(인지는 FULL/LOWER/LOWER_ARUCO·WHITE만 받게 됨).
>
> **[07-15 밤] `turn_bias`는 되살아났다** — 팻말 분기가 고정조향(오픈루프)에서 **앵커 차선
> 지시**(폐루프)로 바뀌면서 이 필드가 그 유일한 전달 경로가 됐다. 의미도 로터리 시절의
> "target_x에 bias"가 아니라 **"팻말의 어느 쪽 통로로 지나라"** 다.
>
> **[07-16] 기준선 선택과 팻말 방향을 분리했다**(`sign_apply: anchor → offset`).
> anchor(= 팻말이 가리키는 쪽 선을 기준선으로)는 실차 2회에서 **한 프레임도 제대로 발동하지
> 못했다**. 튜닝이 아니라 설계 문제다: **S자에선 그 선이 바로 그때 없다** — 커브에선 '안쪽'
> 선이 BEV에서 먼저 빠지므로(`adaptive_anchor` 기하) 우커브=오른선 소실, 좌커브=왼선 소실.
> 기준선을 왼선으로 고정해도 좌커브에서 `dropped:left:왼선없음`이 떠 **거울상으로 실패**했다.
> → **어느 한쪽에 고정 앵커하는 설계는 S자에서 원리적으로 성립하지 않는다.**
> 이제 기준선은 adaptive가 정하고(보이는 바깥선), `turn_bias`는 그 중심선에 **±offset**만
> 얹는다 → 커브/직선·어느 선이 보이든 **항상 발동**(커브 게이트도 선 소실 폴백도 없음).
>
> 인지 동작(`lane_detect.py`, `lane.yaml sign_apply=offset`):
> 1. `turn_bias`=LEFT/RIGHT → `force_side`='left'/'right'
> 2. 중심선(adaptive/dual이 만든 것) 전체를 `∓sign_lane_offset_m`(0.09m=W/4) 평행이동
>    = **팻말 옆 통로 중앙**. 부호: LEFT=`−`(좌), RIGHT=`+`(우). 팻말은 갈림길이 아니라
>    **도로 한가운데 선 장애물**이라 offset 0(=도로 정중앙)은 정면충돌.
> 3. 발동 실패는 '차선 자체 미검출'뿐 → `dropped:<side>:중심선없음`(팻말 이전에 주행이
>    이미 실패한 상태).
> 4. 아래 `sign_force_*` 커브 게이트와 `sign_curve_median_frames`는 **offset 모드에선
>    미사용**(anchor로 되돌릴 때만 의미).
> 5. 이 발동/포기는 07-16까지 **무로그였다** — 지시가 조용히 사라지니 좌/우 동일 증상을
>    모델 탓으로 오진했다. 이제 `LaneResult.sign_force_status`(`applied:`/`gated:`/`dropped:`)로
>    나오고 노드가 상태 변화 시 찍는다.

판단이 각 상태에서 내보내는 지시(요약):

| 상태 | follow | roi_mode | turn_bias |
|---|---|---|---|
| LANE_FOLLOW | WHITE | LOWER | NONE |
| SIGN_BRANCH | WHITE | LOWER | **LEFT / RIGHT**(팻말 래치) |
| DYNAMIC_OBSTACLE_ZONE | WHITE | LOWER_ARUCO | NONE |
| FINISH_APPROACH | WHITE | LOWER | NONE |

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
