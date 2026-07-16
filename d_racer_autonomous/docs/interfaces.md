# D-Racer 노드 인터페이스 계약 (인지 · 판단 · 제어)

> 작성 2026-07-03. 이 문서는 **노드 간 토픽/메시지 계약**을 확정한다.
> 각 파트(인지/판단/제어)는 이 계약만 지키면 서로 독립적으로 개발·교체·병렬작업 가능하다.
> 계약을 바꾸려면 이 문서를 먼저 고치고 팀에 공지한다(코드보다 문서가 진실).

관련 문서: 설계 전반 `architecture.md`, 캘리브레이션 `calibration.md`, 진행상황 `../PROGRESS.md`.

---

## 0. 설계 원칙 (계약의 근거)
- **인지 → 판단 → 제어 단방향 분리.** 제어는 "경로(Path)"만 입력받고 카메라를 모른다.
- **모든 인지는 카메라 단독 → 로컬 프레임.** 차량이 항상 원점이므로 pose 추정이 필요 없다.
- **ROS-free core + 얇은 ROS2 래퍼.** 메시지는 core 자료구조(Path 등)로 쉽게 변환되는 형태로 정한다.
- **안전 우선.** 입력이 끊기거나 신뢰도가 낮으면 정지(fail-safe)가 기본.

---

## 1. 전체 토픽 그래프

```
 [camera_node]
    │  image_raw (sensor_msgs/Image)
    ▼
 [perception_node] ── perception/debug/compressed (sensor_msgs/CompressedImage)  # 디버그
    │  /perception/lane_path   (nav_msgs/Path)          ← ★인지의 핵심 산출물(로컬 경로)
    │  /perception/lane_status (racer_msgs/LaneStatus)  ← 신뢰도·정지선·노랑/흰 검출
    │  /perception/mission_cues(racer_msgs/MissionCues) ← 신호등·체커보드·빨강·아루코
    ▼                        ▲
 [decision_node]             │ /decision/lane_mode (racer_msgs/LaneMode)
    │                        │   ← ★판단→인지 역방향 지시(follow_color/roi_mode/turn_bias)
    │  /decision/drive_command (racer_msgs/DriveCommand) ← 상태·속도배율·정지여부
    ▼
 [controller_node]  (구독: lane_path + drive_command)
    │  /control (control_msgs/Control = {steering, throttle})
    ▼
 [control_node] → PCA9685 → 서보/ESC
```

- `controller_node`는 `/perception/lane_path`(경로)와 `/decision/drive_command`(모드)를 함께 구독한다.
- `lane_status`·`mission_cues`는 판단이 소비한다(제어는 안 봐도 됨).
- **★역방향**: 미션 SM은 상태에 따라 `/decision/lane_mode`로 인지에 "어느 색/ROI/방향으로 볼지"를
  지시한다. 인지는 그 지시대로 ROI/mask/target을 뽑아 lane_path로 되돌린다(왕복 계약).
  자세한 합의 항목은 `perception_agreement.md`.

---

## 2. 좌표계 · 단위 (모든 노드 공통, 반드시 준수)

**로컬 프레임 `base_link`** — 차량 기준, REP-103 준수:
- 원점 = **뒷차축 중심**(Pure Pursuit 기준점). 카메라 장착 오프셋은 인지가 이 원점으로 보정해 출력.
- **+x = 전방**, **+y = 좌측**, **+z = 위**. yaw는 +x 기준 반시계(CCW)가 양수.
- perception이 내보내는 모든 좌표는 이 프레임. → 제어의 pose는 항상 원점(항등변환).

**단위**: 길이 m, 각도 rad, 시간 s.
**액추에이터 정규화**: steering ∈ [-1, +1] (양수=좌회전), throttle ∈ [-1, +1] (양수=전진).
- steering에는 직진 트림(steer_trim=0.2238)이 **제어단(Pure Pursuit)에서 이미 포함**되어 발행된다. 인지/판단은 트림을 모른다.

---

## 3. 토픽 계약 표

| 토픽 | 타입 | 발행자 | 구독자 | 주기 | QoS |
|---|---|---|---|---|---|
| `image_raw` | sensor_msgs/Image | camera | perception | 카메라 fps | best-effort, depth 1 |
| `/perception/lane_path` | nav_msgs/Path | perception | decision, controller | ~10–30 Hz | best-effort, depth 1 |
| `/perception/lane_status` | racer_msgs/LaneStatus | perception | decision | lane_path와 동일 | reliable, depth 1 |
| `/perception/mission_cues` | racer_msgs/MissionCues | perception | decision | ~10 Hz | reliable, depth 1 |
| `/decision/drive_command` | racer_msgs/DriveCommand | decision | controller | ≥10 Hz(변화 시 포함) | reliable, depth 1 |
| `/decision/lane_mode` | racer_msgs/LaneMode | decision | perception | ≥10 Hz(변화 시 포함) | reliable, depth 1 |
| `/control` | control_msgs/Control | controller | control_node | rate_hz(기본 10) | best-effort, depth 10 |
| `perception/debug/compressed` | sensor_msgs/CompressedImage | perception | (뷰어) | 선택 | best-effort |

> 원칙: **센서·명령 스트림(최신값만 중요)은 best-effort/depth 1**, **상태·모드(놓치면 안 됨)는 reliable.**

---

## 4. 메시지 정의 (계약 본체)

### 4.1 이미 존재 — `control_msgs/Control` (변경 없음)
```
std_msgs/Header header
float32 steering   # [-1,+1], +좌회전. steer_trim 포함된 최종 명령.
float32 throttle   # [-1,+1], +전진.
```

### 4.2 `/perception/lane_path` → **`nav_msgs/Path`** (표준, 새 메시지 불필요)
- `header.frame_id = "base_link"`, `header.stamp` = 해당 프레임 촬영시각.
- `poses[]` = 차선 **중심선** 점들, **가까운→먼 순서**로 정렬.
  - `poses[i].pose.position.x/y` = 로컬 좌표 [m] (z=0).
  - `poses[i].pose.orientation` = 그 점에서의 경로 접선 방향(yaw만; 없으면 단위 쿼터니언 허용 — 곡률/heading은 core.Path가 점에서 재계산).
- 점 간격 권장 ≈ 0.05 m, 최소 2점. 전방 유효 거리 권장 ≥ lookahead 최대(1.2 m) 이상.
- **제어측 변환**: `poses`의 (x,y)를 `(N,2)` 배열로 → `core.Path(points)` 생성 → 그대로 Pure Pursuit 입력.

> 왜 nav_msgs/Path? 표준이라 rviz로 바로 시각화되고, core.Path가 점 배열만 있으면 곡률·heading을 스스로 계산하므로 커스텀 메시지가 불필요하다.

### 4.3 `/perception/lane_status` → **`racer_msgs/LaneStatus`** (신규)
```
std_msgs/Header header
bool    lane_detected     # 유효 차선 검출 여부(false면 제어는 watchdog로 정지)
float32 confidence        # 0.0~1.0 검출 신뢰도
int32   num_points        # lane_path 점 개수(0이면 미검출)
float32 lateral_offset    # 차량중심 대비 차선중심 횡오차 [m], +좌측
float32 heading_error     # 차량 전방 대비 차선 접선 오차 [rad]
bool    stop_line         # 정지선 검출 여부(로터리 카운트용 — 제어단 StoplineManeuver가 셈)
float32 stop_line_dist    # 정지선까지 거리 [m] (미검출 시 -1.0)
# [iface 2026-07-13] 색 검출 필드(yellow/white_detected+confidence) 제거 — 로터리 ROI 방식
#   폐기로 소비처 없음. 노랑/흰 마스크는 인지 내부 차선검출에 계속 쓰이나 상태로 보고하지 않음.
# [iface 2026-07-13] 지름길 노랑 래치 신호. 인지가 노랑 차선을 안정 검출해 노랑모드로
#   전환하면 true(점선 끊김엔 hysteresis 유지). 미션이 SHORTCUT 전환에 소비.
bool    on_yellow         # 노랑모드 래치(지름길 노랑 차선 추종 중)
```

### 4.4 `/decision/drive_command` → **`racer_msgs/DriveCommand`** (신규)
```
std_msgs/Header header

uint8   state             # 아래 상수 중 하나(디버그/로깅용)
bool    go                # false면 제어가 throttle=0(정지). 조향은 유지.
float32 speed_scale       # v_max에 곱하는 배율 [0.0~1.0]. 코너/불확실 시 감속.
float32 lookahead_scale   # lookahead 배율 [기본 1.0]. (선택; 미사용 시 1.0)
float32 steer_limit       # 정규화 조향 상한 [0.0~1.0]. (선택; 미사용 시 1.0)
float32 steer_bias        # 팻말 구간 조향 offset(기본 0.0). 제어가 정규화 조향에 가산. [iface 2026-07-14]
uint8   gain_profile      # 제어 게인 프로파일: PROFILE_DEFAULT=0 / PROFILE_POST_SIGN=1. (선택; 미사용 시 DEFAULT) [iface 2026-07-16]

# state 상수
uint8 STATE_INIT=0        # 초기화/대기
uint8 STATE_DRIVE=1       # 정상 주행
uint8 STATE_SLOW=2        # 감속 주행(코너/저신뢰)
uint8 STATE_STOP=3        # 정지(정지선/장애물/명령)
uint8 STATE_LOST=4        # 차선 소실 → 정지 또는 마지막 조향 유지
```

> 판단은 파라미터를 **직접 세팅하지 않고** 이 명령으로 "배율/게이트"만 준다. 제어기의 기본 튜닝값(controller.yaml)은 그대로 두고 상황별로 스케일한다 → 튜닝 일원화.

> **[iface 2026-07-16] `gain_profile` 추가 — 구간별 제어 게인 프로파일.**
> 왜: `heading_error`는 접선이 아니라 **경로 최근접점→맨 끝점의 현 각도**다(lane_detect).
> lane_path가 차 앞 5cm~97cm를 덮으므로 ψ는 **1m 앞을 본다**. S자에선 이 선반영이 이득이지만,
> 직진 후 90도 코너에선 차가 아직 직선인데 경로 끝이 이미 코너를 돌아 ψ가 커져서
> `k_heading·ψ`가 코너 1m 전부터 조향을 걸어 라인을 벗어난다(07-16 실차).
> 팻말 뒤는 **직선-ㄱ자-직선-ㄱ자**라 곡률이 계단처럼 뛴다 = S자와 성격이 정반대라 한 세트로
> 둘 다 만족시킬 수 없다(controller.yaml k_heading 0.5→0.33이 그 싸움의 흔적 = 전역 튜닝의 한계).
> 게인은 제어가 갖고 "팻말을 지났다"는 코스 위치는 판단만 알아서, 둘을 잇는 지시가 필요했다.
> **배율이 아니라 프로파일 id인 이유**: 배율은 튜닝 수치를 판단 쪽 yaml로 새게 하고(§4.4 위반),
> 값을 하나 더 구간별로 나눌 때마다 msg 변경+rebuild가 필요하다. 프로파일이면 수치는
> controller.yaml에 남고, 그 구간 게인을 몇 개를 바꾸든 계약은 그대로다.
> 근본 해법은 ψ의 현 길이를 자르는 것(perception, iface 불필요)이지만 S자의 선반영도 같이
> 사라져 대회 전엔 위험 → 구간별 프로파일로 좁게 간다(대회 후 과제).
> ⚠ k_heading은 위빙 억제 **주감쇠** 항이다. 낮추면 그 구간 직선 꿀렁임↑·런와이드 위험↑.

### 4.5 `/perception/mission_cues` → **`racer_msgs/MissionCues`** (신규, [iface 2026-07-06])
```
std_msgs/Header header
uint8   traffic_light         # TL_NONE=0/TL_RED=1/TL_GREEN=2 (YOLO)
bool    checkerboard_detected # 체커보드 (YOLO). [2026-07-14] 도착 판정 폐기(빨간불 종료로 교체), 필드만 유지
bool    red_zone_detected     # 빨강 바닥 구역 (OpenCV). 폐기 stub(미소비), 필드만 유지
bool    aruco_present         # 아루코 마커, 하단 ROI (cv2.aruco)
bool    sign_detected         # 방향 팻말색 검출 (OpenCV 트리거). 팻말 구간 진입.  [iface 2026-07-14]
uint8   sign_direction        # SIGN_NONE=0/SIGN_LEFT=1/SIGN_RIGHT=2 (팻말 YOLO, 별도 모델)  [iface 2026-07-14]
```
> 미션 페이즈 전환용 객체/구역 신호 묶음. 차선 기하(lane_path/lane_status)와 분리.
> **[2026-07-14]** 종료=빨간불(체커보드 폐기). `sign_detected`(OpenCV색=팻말 구간 트리거)+
> `sign_direction`(팻말 YOLO=좌/우) 추가 → 판단이 SIGN_BRANCH 진입·조향 bias 방향에 소비.

### 4.6 `/decision/lane_mode` → **`racer_msgs/LaneMode`** (신규, [iface 2026-07-06]) ★역방향
```
std_msgs/Header header
uint8 follow_color  # COLOR_WHITE=0/COLOR_YELLOW=1
uint8 roi_mode      # ROI_FULL=0/LOWER=1/RIGHT=2/LEFT=3/LOWER_ARUCO=4
uint8 turn_bias     # BIAS_NONE=0/LEFT=1/RIGHT=2
bool  yolo_enable   # 신호등 YOLO 추론 게이트: true=ON, false=skip  [iface 2026-07-13]
bool  sign_enable   # 팻말 YOLO(별도 모델) 게이트: true=ON, false=skip. yolo_enable과 독립  [iface 2026-07-14]
```
> **판단→인지** 지시. 미션 SM(5-state)이 상태에 따라 "어느 색/ROI로 볼지"를 준다.
> 인지는 이 지시대로 ROI 자르기·mask·target을 적용해 lane_path를 만든다.
> 상수값은 `core.planning`의 LaneColor/RoiMode/TurnHint와 일치. 상세: `perception_agreement.md`.
> **[2026-07-13]** 로터리 ROI 방식 폐기 → 판단은 RIGHT/LEFT·YELLOW·turn_bias를 발행하지 않는다
> (항상 follow_color=WHITE, roi_mode∈{FULL,LOWER,LOWER_ARUCO}, turn_bias=NONE). 값은 계약에 유지.
> **[iface 2026-07-13] `yolo_enable`** — 신호등 YOLO 추론의 페이즈별 게이트.
> **[iface 2026-07-14] `sign_enable` 추가 + 새 6-state FSM 반영** — 팻말 YOLO(별도 모델)를 독립 게이트.
> 미션 SM이 산출(모델 2개 독립 on/off):
> - `yolo_enable`(신호등): `WAIT_START_SIGNAL`=ON(출발 초록불) → 주행중 OFF → aruco 최초 검출 시 ON 래치
>   유지(`FINISH_WATCH`에서 **빨간불** 종료 감시까지). 지름길 SHORTCUT 폐기.
> - `sign_enable`(팻말): `SIGN_BRANCH`에서만 ON(팻말 좌/우 판단). 그 외 OFF.
> `mission_cues_node`가 둘 다 구독해 각 모델 추론 호출을 게이트한다(aruco·팻말색 OpenCV 등은 계속).
> 발행자 없으면 신호등=기본 ON(하위호환), 팻말=기본 OFF. 전체 off는 `mission.yaml`의 `yolo_gate_enable: false`.

---

## 5. 제어측 동작 계약 (controller_node)

현재 `controller_node`(Stage 6-b)는 고정 pose + 정적경로다. 폐루프 전환 시 다음을 따른다:

1. **경로 입력원 파라미터** `source: static | topic` (기본 topic; 거치대 검증은 static).
   - `topic`이면 `/perception/lane_path`를 구독해 매 수신마다 `core.Path`로 갱신.
2. **pose는 항상 원점**(로컬 프레임). Pure Pursuit 코드 변경 없음.
3. **drive_command 반영**:
   - `go=false` 또는 `state∈{STOP,LOST}` → throttle=0(조향 유지).
   - 유효 throttle = `speed_scale`로 스케일한 목표속도를 액추에이터로 매핑(매핑 캘리브는 Stage 4).
   - `steer_limit`/`lookahead_scale`는 있으면 적용.
4. **Watchdog(안전)**: `/perception/lane_path`가 `lane_timeout`(기본 0.3 s) 넘게 안 오거나 `lane_detected=false`면 → **정지(throttle=0)**, 조향은 마지막값 유지. 로그 경고.
5. **트림**: Pure Pursuit 출력에 steer_trim 이미 포함 → 그대로 발행(재가산 금지).

---

## 6. 시간 · 동기화
- 모든 메시지에 `header.stamp` 채운다(지연·유효성 판단용).
- 제어는 최신 lane_path 하나만 쓰고 과거는 버린다(depth 1). 프레임 지연이 크면 watchdog가 정지.
- 좌표는 **동일 프레임(base_link)** 이므로 TF 변환은 불필요(카메라 오프셋만 인지가 흡수).

---

## 7. 버전 · 변경 관리
- 이 계약 변경 = 이 문서 수정 + 커밋 메시지에 `[iface]` 태그 + 팀 공지.
- 필드 추가는 하위호환(구독자는 모르는 필드 무시). 필드 의미 변경/삭제는 반드시 공지.

---

## 8. 현재 상태 대비 갭 (구현 시 처리)
- `d_racer_perception` ROS2 패키지 신규 작성 필요. 테스트/실차 인지 노드는 **`/perception/lane_path`(nav_msgs/Path) + `/perception/lane_status`**를 발행해야 함. (lateral_offset은 LaneStatus로 흡수)
- `racer_msgs` 패키지 **신규 생성** 필요(LaneStatus, DriveCommand). control_msgs와 분리(앱 레벨 메시지).
- controller_node에 lane_path/drive_command 구독 + watchdog + `source` 파라미터 추가.
- decision_node **신규 작성**(State Machine).
- YOLO/OpenCV 테스트 모델과 데이터는 git에 직접 올리지 않고, 다운로드 스크립트와 config만 관리.

---

## 9. 결정 대기 (팀 확인 후 확정)
아래는 잠정 결정. 이견 없으면 그대로 간다.
1. **lane_path 타입**: `nav_msgs/Path` 채택(표준·rviz). 대안 = 커스텀 `LanePath`(곡률 동봉). → **Path 권장**(곡률은 core가 계산).
2. **커스텀 메시지 패키지명**: `racer_msgs`. (기존 관례가 `*_msgs`라 일치)
3. **base_link 원점**: 뒷차축 중심. 카메라 오프셋은 인지가 보정. (대안: 카메라 광학중심 원점 — 그러면 제어 pose에 오프셋 필요, 비권장)
4. **정지선/장애물**: 우선 정지선만(LaneStatus.stop_line). 장애물은 후속 확장.
```
