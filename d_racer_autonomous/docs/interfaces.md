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
    │  /perception/lane_path   (nav_msgs/Path)         ← ★인지의 핵심 산출물(로컬 경로)
    │  /perception/lane_status (racer_msgs/LaneStatus) ← 신뢰도·정지선 등 메타
    ▼
 [decision_node]
    │  /decision/drive_command (racer_msgs/DriveCommand) ← 상태·속도배율·정지여부
    ▼
 [controller_node]  (구독: lane_path + drive_command)
    │  /control (control_msgs/Control = {steering, throttle})
    ▼
 [control_node] → PCA9685 → 서보/ESC
```

- `controller_node`는 `/perception/lane_path`(경로)와 `/decision/drive_command`(모드)를 함께 구독한다.
- `lane_status`는 주로 판단이 소비한다(제어는 안 봐도 됨).

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
| `/decision/drive_command` | racer_msgs/DriveCommand | decision | controller | ≥10 Hz(변화 시 포함) | reliable, depth 1 |
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
bool    stop_line         # 정지선 검출 여부
float32 stop_line_dist    # 정지선까지 거리 [m] (미검출 시 -1.0)
```

### 4.4 `/decision/drive_command` → **`racer_msgs/DriveCommand`** (신규)
```
std_msgs/Header header

uint8   state             # 아래 상수 중 하나(디버그/로깅용)
bool    go                # false면 제어가 throttle=0(정지). 조향은 유지.
float32 speed_scale       # v_max에 곱하는 배율 [0.0~1.0]. 코너/불확실 시 감속.
float32 lookahead_scale   # lookahead 배율 [기본 1.0]. (선택; 미사용 시 1.0)
float32 steer_limit       # 정규화 조향 상한 [0.0~1.0]. (선택; 미사용 시 1.0)

# state 상수
uint8 STATE_INIT=0        # 초기화/대기
uint8 STATE_DRIVE=1       # 정상 주행
uint8 STATE_SLOW=2        # 감속 주행(코너/저신뢰)
uint8 STATE_STOP=3        # 정지(정지선/장애물/명령)
uint8 STATE_LOST=4        # 차선 소실 → 정지 또는 마지막 조향 유지
```

> 판단은 파라미터를 **직접 세팅하지 않고** 이 명령으로 "배율/게이트"만 준다. 제어기의 기본 튜닝값(controller.yaml)은 그대로 두고 상황별로 스케일한다 → 튜닝 일원화.

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
- perception_yolo는 지금 `perception/lane_offset`(Float32)만 발행 → **`/perception/lane_path`(nav_msgs/Path) + `/perception/lane_status`로 확장**해야 함. (lateral_offset은 LaneStatus로 흡수)
- `racer_msgs` 패키지 **신규 생성** 필요(LaneStatus, DriveCommand). control_msgs와 분리(앱 레벨 메시지).
- controller_node에 lane_path/drive_command 구독 + watchdog + `source` 파라미터 추가.
- decision_node **신규 작성**(State Machine).
- (별건) perception_yolo launch의 `model_path` 하드코딩(`/home/topst/...best.pt`) → 파라미터/상대경로화 권장.

---

## 9. 결정 대기 (팀 확인 후 확정)
아래는 잠정 결정. 이견 없으면 그대로 간다.
1. **lane_path 타입**: `nav_msgs/Path` 채택(표준·rviz). 대안 = 커스텀 `LanePath`(곡률 동봉). → **Path 권장**(곡률은 core가 계산).
2. **커스텀 메시지 패키지명**: `racer_msgs`. (기존 관례가 `*_msgs`라 일치)
3. **base_link 원점**: 뒷차축 중심. 카메라 오프셋은 인지가 보정. (대안: 카메라 광학중심 원점 — 그러면 제어 pose에 오프셋 필요, 비권장)
4. **정지선/장애물**: 우선 정지선만(LaneStatus.stop_line). 장애물은 후속 확장.
```
