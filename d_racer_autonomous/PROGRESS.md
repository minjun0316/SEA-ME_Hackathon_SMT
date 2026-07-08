# D-Racer 자율주행 — 진행 상황 / 다음 할 일

> 마지막 작업일: 2026-07-08. 이 문서는 매 세션 끝에 갱신한다.
>
> **작업 환경 변경(07-03)**: 이제 보드에서 직접 편집·빌드·커밋한다. 워크스페이스=팀레포 `~/SEA-ME_Hackathon_SMT`(= colcon ws, `build/ install/ src/` 포함). 옛 `~/D-Racer-Kit`는 통합되어 없어짐. scp 왕복 불필요.

## 한 줄 요약
ROS-free 코어로 Stage 1~3(시뮬) 완성. 실차 브링업·캘리브 완료, Stage 6-b controller_node 작성·검증 완료. 판단 2계층(아래층 반응형 decision + 위층 미션 MissionSequencer) 순수 로직 완성. `racer_msgs` 로터리 12-state로 확장(MissionCues/LaneMode). ROS 래퍼 3종(decision/controller-topic/mission) + **차선 인지 노드(`lane_detect_node`, cpp 파이썬 포팅) 작성, colcon 빌드 OK·테스트 52개 통과.** **차선추종 폐루프 launch(`lane_follow.launch.py`, YOLO 제외) 완성.** 다음은 **실차 카메라 차선추종 테스트 + 픽셀→미터 캘리브**.

---

## ▶ 다음 세션 여기서 시작 (2026-07-08 마감 기준)

### ⚠️ 07-08: 저속 주행 안 됨 원인 규명(throttle_limit 함정 + 배터리) — 코드수정 없음
- **증상**: `racer-run enable_drive:=True drive_throttle:=0.28` → 뒷바퀴 삐 소리만, 안 돎. "예전엔 0.18로 됐었는데 이상하다".
- **원인①(throttle_limit 함정)**: `lane_follow.launch.py` `throttle_limit` **기본값 0.15**. decision이 기본 off라 `speed_scale=1.0`이고, 모터에 가는 값은 `min(drive_throttle, throttle_limit)`. **throttle_limit을 안 주면 drive_throttle을 아무리 올려도 0.15로 클램프됨.** → 0.28 준 게 무의미. (`controller_node.py:284` speed_scale 기본 1.0, `:317-319` clamp)
- **원인②(배터리)**: 예전엔 그 clamp된 0.15가 모터 breakaway 위였음("0.18로 됐었어"). 배터리 방전으로 breakaway가 0.15 위로 올라가 → 같은 0.15인데 삐 소리. **→ 충전이 정답.** 충전 중.
- **핵심 인지사항(다음에 또 헷갈림)**: `racer-run`은 **control_node(모터)까지 기본 포함**(따로 안 띄워도 됨). **decision_node는 기본 off**(`use_decision:=True`로 켬) → 지금은 ×0.9(SLOW) 안 걸리고 **drive_throttle이 곧 실제 throttle**.
- **★ 충전 완료 후 바로 할 것**: `racer-run enable_drive:=True drive_throttle:=0.16 throttle_limit:=0.20`. 안 돌면 0.17→0.18로 조금씩 ↑(막 도는 값=최저속도, 안정주행은 +0.02). 완충이면 저속 가능할 것.
- alias(참고): `racer-run`(실행) / `racer-clean`(노드 kill) / `stop`(비상정지). `.bashrc`가 ROS+workspace 3개 자동 source.

### ✅ 완료(07-08): ArUco 미션신호 인지 노드(학습 불필요, 고전 CV) — 실차 미검증
- **`core/perception/aruco_detect.py`(신규, ROS-free)**: `cv2.aruco`(OpenCV 5.0.0 신 API) 검출기. `ArucoConfig`(dictionary/roi_bottom_frac/min_perimeter_px/target_ids) + `ArucoResult`. 하단 ROI·크기·ID 필터. 합성 마커로 검출 로직 검증.
- **`d_racer_perception/mission_cues_node.py`(신규, 얇은 노드)**: camera/image/compressed 구독 → `/perception/mission_cues`(racer_msgs/MissionCues, reliable) 발행 + 디버그영상(`perception/mission_cues/aruco/debug/compressed`). aruco만 실제값, traffic_light/checkerboard/red_zone은 **stub**(후속). present 홀드(hold_sec=0.4) 디바운스.
- **대회 마커 확정(07-08)**: **DICT_6X6_50, ID 3**. `config/mission_cues.yaml`에 반영. 검증: 6X6_50 ID3 검출 OK, 잘못된 사전(4x4)이면 미검출(사전 일치가 결정적).
- **시나리오**: 심판이 막대에 든 마커를 멀리서 보여주면 정지, 사라지면 재출발. → roi=전체화면, ID 3만 인정. setup.py entry_point 등록, 중첩 ws(`d_racer_autonomous/ros2_ws`) colcon 빌드 OK, 노드 기동 OK.
- **⚠️ 미완**: (1) 실차 카메라로 실제 마커 검출 눈확인, (2) `aruco_present`→**정지 연결**(현재 발행만; mission.py는 M4에서만 반응. "언제든 정지" 게이트 여부 미정), (3) mission_cues_node를 launch에 편입, (4) 미커밋.

### ✅ 완료(07-07 낮): 실차 첫 주행(거치대) + control_node watchdog(안전버그 수정)
- **거치대에서 폐루프 첫 주행 성공**: `lane_follow`(enable_drive:=True drive_throttle:=0.25 throttle_limit:=0.30)로 뒷바퀴 실제 구동 + 차선 따라 조향 확인.
- **문제① 계속 SLOW에 갇힘** → throttle 부족으로 안 돌던 원인. decision이 `lateral_offset≈-0.4m > offset_slow(0.12)`라 항상 SLOW(×0.9). **40cm 실제로 벗어난 게 아니라 `m_per_px_lateral=0.005`(캘리브 안 된 잠정값) 때문에 뻥튀기된 값.** 게다가 DRIVE(0.18)여도 breakaway(0.16) 겨우 위. → 임시로 drive_throttle 0.25(SLOW×0.9=0.225)로 넘김. **진짜 해결=m/px lateral 캘리브**(아래 다음할일).
- **문제② control_node에 watchdog 없음(안전버그)** → 상위 스택 죽거나 Ctrl+C해도 `control_node`가 **마지막 throttle을 물고 계속 주행**(Ctrl+C로 안 멈춤). `src/control/control/control_node.py`에 **명령 watchdog 추가**: 파라미터 `cmd_timeout`(기본 0.5s), 명령 0.5s 끊기면 throttle=0 강제 + 로그. `colcon build --packages-select control` OK. **⚠️ 미커밋 + 실차에서 "Ctrl+C→0.5s 내 자동정지" 검증 남음.**
- **문제③ camera_node 죽으면 전체 정지**: `use_camera:=False`로 lane_follow 띄웠는데 기존 camera_node가 죽어 있어서 lane_path 안 나옴→계속 STOP. camera_node 단독은 30Hz 정상 확인됨. → **lane_follow는 `use_camera:=False` 빼고 띄워 launch가 카메라까지 관리**하는 게 안전.
- **비상정지 도구 추가**: `~/stop.sh` + alias `stop`(~/.bashrc). = 주행 스택 kill + `/control` throttle 0. 폭주 시 `stop` 한 단어.
- **미커밋 변경 4개**(내일 커밋 여부 결정): `control_node.py`(watchdog), `camera_node.py`(C920 16:9 캡처→화각 확보, V4L2 폴백 경로), `vehicle_config.yaml`(CAPTURE_*, flip none, WEB_HOST 0.0.0.0), `decision.yaml`(slow_speed_scale 0.5→0.9).
- 세션 끝 정리 완료: 모든 노드 종료, 모터 throttle 0, ros2 daemon stop.

### ★ 내일 바로 할 일 (우선순위)
1. **주행 재개 + watchdog 검증**: T1 `control_node`(로그에 `cmd_timeout=0.5s` 확인) + T2 `ros2 launch racer_bringup lane_follow.launch.py enable_drive:=True drive_throttle:=0.25 throttle_limit:=0.30` (**use_camera 빼기**). 거치대→바닥. **Ctrl+C 눌러 0.5s 내 자동정지 되는지 꼭 확인.**
2. **m/px lateral 캘리브**(SLOW 오작동 + 조향 정확도 동시 해결): 바닥 거리표식으로 BEV 픽셀↔미터 실측 → `config/lane.yaml` `m_per_px_lateral`. 그러면 lateral_offset이 실제 미터가 돼 SLOW 오판 사라짐.
3. 미커밋 4개 커밋(특히 control_node watchdog).

### ✅ 완료(07-07): ROS 래퍼 3종 + 인지 계약 형식 + 로터리 12-state
- **`decision_node`(얇은 ROS 래퍼)** — `/perception/lane_status` 구독 → 아래층 `DecisionMaker` → `/decision/drive_command` 발행 + watchdog. (커밋 `ef7e5b9 [ros]`)
- **로터리 미션 12-state로 확장** — `core/planning/mission.py` 재작성, `racer_msgs`에 `MissionCues`(신호등/체커보드/빨강/아루코)·`LaneMode`(follow_color/roi_mode/turn_bias) 추가, `LaneStatus` 확장. `docs/interfaces.md`·`mission_fsm.md`·`perception_agreement.md` 갱신. 테스트 갱신. (커밋 `a63ae5b [planning]`)
- **인지 `lane_only_detect.cpp` 계약 형식 재작성** — `PerceptionResult`를 `LaneStatus` 미러 + `lane_path` 중심선 점열(near→far)로. confidence/lateral_offset/heading_error/stop_line_dist 산출. **값 단위는 아직 픽셀**(미터/base_link 변환 후속). 문법검사 통과. (커밋 `ed001ee [perception]`)
- **`controller_node` topic 모드 + `mission_node` 신설** — controller_node에 `source=static|topic` 파라미터(topic 모드: lane_path+drive_command 구독 + lane_timeout watchdog). `mission_node`(위층 MissionSequencer 얇은 래퍼) + `mission.launch.py` 신설. setup.py/package.xml(nav_msgs) 갱신. 문법검사만. (커밋 `a0af826 [ros]`)
- **joystick 후진 허용** — throttle 하한 클램프 제거. (커밋 `9f189a4 [joystick]`)

### ✅ 완료(07-07 오후): 차선 인지 ROS 노드 + 차선추종 폐루프(YOLO 제외)
- **`core/perception/lane_detect.py`(신규, ROS-free)**: cpp 알고리즘 파이썬 포팅. BEV→HLS→슬라이딩윈도우 중심선 → **픽셀→미터(base_link) 변환**. `LaneDetector`/`LaneCalib`/`LaneResult`. 합성 프레임 런타임 검증(findNonZero/HoughLinesP 버전차 reshape).
- **`d_racer_perception/lane_detect_node.py`(신규, 얇은 노드)**: `camera/image/compressed` 구독 → `/perception/lane_status`(reliable) + `/perception/lane_path`(nav_msgs/Path, base_link) + 디버그영상. `config/lane.yaml`.
- **`racer_bringup/lane_follow.launch.py`(신규)**: camera→lane_detect→**decision_node(아래층 반응형, lane_status만)**→controller(source=topic) 원샷. YOLO·mission_node 제외. enable_drive 기본 False.
- setup.py data_files 디렉토리 복사 버그 수정(files_only). colcon build 3패키지 OK, 노드 기동·토픽 발행 확인, 테스트 52개 통과. (커밋 `ead467a [perception]`)

### ✅ 완료(07-07 밤): lane_follow 폐루프 배관 실동작 확인(거치대/데스크)
- **버그 2개 잡음**: (1) `controller_node`가 lane_path를 reliable로 구독 → 인지의 best-effort 발행과 QoS 불일치로 **메시지 0개 수신**(항상 STOP:no_lane_path). best-effort로 수정(`9409b07`). (2) 보드 OpenCV 5.0.0이 **GStreamer:NO** 빌드라 kit `camera_node`가 카메라를 못 엶 → V4L2 직접 캡처 폴백 추가(`3415ea4`).
- 스모크: `lane_follow.launch.py` 띄우니 camera(V4L2 폴백)→lane_detect(`lane=True conf~0.7`)→decision→**controller `[drive] steer=... κ=...`**(STOP 아님) 흐름 확인. throttle=0(enable_drive=False, 안전).

### ★ 다음 세션 바로 할 일: **실제 트랙 차선 + m/px 캘리브**
- **트랙에서 주행 테스트**: T1 `control_node(use_joystick_control:=False)` + T2 `ros2 launch racer_bringup lane_follow.launch.py`. rqt_image_view로 `/perception/lane/debug/compressed` 보며 실제 차선/윈도우가 맞게 잡히는지 확인. 조향만(enable_drive=False) → 저속(`enable_drive:=True drive_throttle:=0.12`).
- **픽셀→미터 캘리브(핵심)**: `config/lane.yaml`의 `m_per_px_forward/lateral`·`x_near_m`은 **잠정값**. 바닥에 알려진 거리 표식 두고 BEV에서 픽셀↔미터 실측해 채워야 Pure Pursuit 조향이 맞음. (지금은 대충이라 조향 게인이 안 맞을 수 있음.)
- **BEV src 튜닝**: `bev_top_y/x`가 카메라 장착각/높이에 안 맞으면 차선이 휘어 보임 → 디버그영상 보며 조정.
- YOLO 학습 끝나면: mission_cues 인지 추가 → mission_node로 미션(신호등/아루코/로터리) 얹기.

### ✅ 완료(07-06): `MissionSequencer` 뼈대 (위층 미션 페이즈 SM)
- `core/planning/mission.py` 신설 — `MissionSequencer`(위층). 아래층 `DecisionMaker`를 **소유·호출**해 합성.
  - `MissionPhase(M0~M6)`, `stopline_count`, `TrafficLight`, `MissionObservation`(차선관측+미션신호).
  - 페이즈: M0신호대기→GREEN출발→M1흰→노랑전환M2→정지선 rising-edge count(1=loop,2=exit `turn_hint`)→흰전환M3→**빨강구역M4(아루코 보이면 STOP, 아니면 주행)**→구역벗어남M5→정지구역접근M6종료.
  - 정지 페이즈(M0·M6·M4아루코)는 STOP 강제. M2 갈림길 정지선은 아래층에 **마스킹**해 통과(정지 방지).
  - `DriveCommand`에 `follow_color`(WHITE/YELLOW)+`turn_hint`(NONE/LEFT/RIGHT) 추가.
  - `MissionConfig`+`config/mission.yaml`(circle_loop_side/exit_side, stop_line_debounce, stop_zone_stop_dist). 좌/우·거리는 **대회장 확정** 전제 기본값.
- 테스트 `tests/test_mission.py` 17개(+기존 33) = **전체 50개 통과**.
- **문서 갱신**: `docs/mission_fsm.md` M4를 빨강 바닥 구역(진입/탈출) + 아루코(정지/재출발)로 반영.

### ✅ 완료(07-06): `racer_msgs` 패키지 (메시지 계약)
- `ros2_ws/src/racer_msgs/` 신설(ament_cmake, `control_msgs` 규약 미러). `msg/LaneStatus.msg`(인지→판단) + `msg/DriveCommand.msg`(판단→제어, STATE_* 상수 포함). 정의는 `docs/interfaces.md` §4.3/4.4 **그대로**(합의된 베이스라인만).
- `colcon build --packages-select racer_msgs` ✅, `ros2 interface show racer_msgs/msg/DriveCommand` 등록 확인. 커밋 `[iface]` push 완료.
- **일부러 뺀 것**: 미션 확장 필드(`follow_color`/`turn_hint`, traffic_light/red_zone/aruco/stop_zone). `follow_color`·`turn_hint`는 **인지(OpenCV)가 받아야** 하는데 현재 토픽 그래프가 decision→controller뿐이라 경로 없음 → 아래 [확장 설계] 선행 필요.

### ~~다음 세션 바로 할 일: `decision_node`~~ ✅ 완료(07-07, 위 07-07 섹션 참조)
- ~~`decision_node`: lane_status 구독 → 아래층 DecisionMaker → drive_command 발행.~~ → `ef7e5b9`.
- ~~controller_node에 drive_command 구독+watchdog.~~ → `a0af826`(source=topic 모드).

### [확장 설계] 미션 신호 경로 (인지팀 합의 필요, decision_node와 병행 가능)
- decision→perception 신호 경로 신설(예: `/decision/lane_mode`{follow_color, turn_hint} 토픽). `interfaces.md`를 `[iface]`로 먼저 갱신 → 팀 공지 → `racer_msgs` 확장.
- 신규 인지 신호(traffic_light/red_zone/aruco/stop_zone)도 `mission_fsm.md §6`대로 합의 후 `LaneStatus` 확장 or 신규 메시지.
- **미확정(대회장서 확정)**: `circle_loop_side/exit_side` 좌/우, 각종 거리 임계값. (A)진입직선=출구정지선 직전인지 (B)M6 4바퀴 정밀정지 데드레커닝.

### 미션 요약 (docs/mission_fsm.md 참조)
M0 신호대기(초록불,YOLO) → M1 흰직진 → M2 지름길(노랑: 직선→갈림길 정지선 1회=count1 loop쪽 → 원 한바퀴 → 정지선 2회=count2 exit쪽 → 직선→커브→흰전환) → M3 흰직진 → M4 아루코 마커(보이는동안 정지) → M5 흰직진 → M6 정지구역(YOLO) 위 4바퀴 정지.
- **갈림길 분기**: 양쪽 노랑·곡률동일 → 색/곡률로 구분 불가. 판단은 count로 `turn_hint=L/R`만, OpenCV가 ROI 반자르기(또는 히스토그램 봉우리 택1)로 그쪽 갈래 추종. 좌/우=YAML(대회장).
- **미확정**: (A) 진입직선이 정지선 바로 직전인지=한바퀴 성립조건(트랙확인) (B) 정지구역 4바퀴 정밀정지=카메라 사각→데드레커닝 별도설계 (C) 인터페이스 신규신호 인지팀 합의.

**환경 준비(터미널 접속하면 항상)**
```bash
cd ~/SEA-ME_Hackathon_SMT && source /opt/ros/humble/setup.bash && source install/setup.bash
```

**git**: `dev` 브랜치, origin/dev push 완료(HEAD=9f189a4). 07-07 커밋: `ef7e5b9 [ros]` decision_node → `9999129 [perception]` lane_only 초안 → `a63ae5b [planning]` 로터리 12-state + racer_msgs 확장 → `ed001ee [perception]` lane_only 계약 형식 → `a0af826 [ros]` controller topic 모드 + mission_node → `9f189a4 [joystick]` 후진 허용. `.vscode/`는 gitignore 처리(미커밋). **주의: ROS 래퍼 3종·확장 메시지는 colcon 빌드/실차 미검증(py·cpp 문법검사만).**

**바로 할 수 있는 다음 후보(택1)**
1. **실차 조향 눈확인**(배터리 완충 필요, 거치대): T1 `ros2 run control control_node --ros-args -p use_joystick_control:=False` + T2 `ros2 launch racer_bringup controller.launch.py path:=circle` → 앞바퀴가 곡률 방향으로 꺾이는지. 이후 저속 `enable_drive:=True drive_throttle:=0.12`.
2. **`racer_msgs` 패키지 신설** (인지/판단 전제): `LaneStatus.msg`, `DriveCommand.msg` (정의는 `docs/interfaces.md` §4.3/4.4 그대로). ament_cmake 메시지 패키지, `src/`에 생성 후 빌드.
3. **`d_racer_perception` 신설**: `ros2_ws/src/d_racer_perception`에 테스트/실차 인지 ROS2 노드 추가. `/perception/lane_path`(nav_msgs/Path) + `/perception/lane_status` 발행. 순수 기하는 `core/perception/`에 구현, YOLO/OpenCV 실행은 노드에 유지.
4. **decision_node(판단) 착수**: `core/planning/`에 State Machine(순수 로직) → 얇은 ROS 노드로 래핑, `/decision/drive_command` 발행.

**착수 전 확인**: `docs/interfaces.md §9 결정 대기 4건`(lane_path 타입=nav_msgs/Path, 패키지명 racer_msgs, base_link 원점=뒷차축, 정지선 우선) 팀 합의.

**미해결/주의**
- ⚠️ **throttle_limit 함정(07-08)**: `lane_follow.launch.py` `throttle_limit` 기본 **0.15**. decision off면 모터 throttle=`min(drive_throttle, throttle_limit)`이라, **throttle_limit을 안 주면 drive_throttle이 전부 0.15로 잘림**. 저속 주행 시 `drive_throttle`과 `throttle_limit`을 **항상 같이** 줄 것(예: `drive_throttle:=0.16 throttle_limit:=0.20`). 필요하면 launch 기본값을 0.15→0.25로 올리는 것도 고려(안전캡이라 보류 중).
- ⚠️ **모터 breakaway는 배터리 전압에 민감**. 완충이면 0.15~0.18로도 돌지만 방전되면 breakaway가 올라가 삐 소리만. 저속 주행엔 완충 배터리 필수.
- ⚠️ **보드 OpenCV 5.0.0 = GStreamer:NO**. `cv2.VideoCapture(..., CAP_GSTREAMER)`는 무조건 실패한다(카메라는 V4L2/FFMPEG로만 열림, `/dev/video1` 정상). camera_node에 V4L2 폴백 넣어 해결했지만, OpenCV 재설치/다른 GStreamer 의존 코드 쓸 때 재발 주의. 확인: `python3 -c "import cv2;print(cv2.getBuildInformation())" | grep GStreamer`.
- collect1/SECOND 학습사진 보드 디스크에 없음 → Roboflow/데스크탑 확인 or 재수집.
- 배터리 방전 잦음 → 완충 여분 필수. i2c 먹통 시 점퍼선 재체결.
- 기존 임시 YOLO 폴더들은 baseline과 분리하기 위해 삭제. 새 인지 코드는 `d_racer_autonomous/ros2_ws/src/d_racer_perception`에서 개발.

---

## 환경 / 접속 정보
| 항목 | 값 |
|------|-----|
| 개발 PC | `sdw@sdw-desktop`, 프로젝트: `/home/sdw/d_racer_autonomous/` |
| 차(보드) | `ssh topst@192.168.0.123`, 워크스페이스: `~/D-Racer-Kit`, ROS2 Humble |
| 배포 방식 | 데스크탑에서 수정 → `scp`로 보드에 복사 → 보드에서 `colcon build` |
| (편의) | 나중에 VS Code Remote-SSH 세팅하면 scp 없이 차 파일 직접 편집 가능 |

### 자주 쓰는 명령
```bash
# 데스크탑 → 보드 파일 복사 (예: 캘리브 노드)
cd /home/sdw/d_racer_autonomous/ros2_ws/src
scp racer_bringup/racer_bringup/calibration_node.py topst@192.168.0.123:~/D-Racer-Kit/src/racer_bringup/racer_bringup/

# 보드에서 빌드
cd ~/D-Racer-Kit && source /opt/ros/humble/setup.bash && colcon build --packages-select racer_bringup && source install/setup.bash

# 보드: 모터 제어 노드 (터미널 1, 켜둬야 바퀴 움직임)
ros2 run control control_node --ros-args -p use_joystick_control:=False

# 보드: 캘리브레이션 노드 (터미널 2, 백그라운드)
ros2 run racer_bringup calibration_node &
ros2 param set /calibration_node steering 0.10   # 실시간 조정
ros2 param set /calibration_node throttle 0.0    # 0.15로 하드 클램프
pkill -f calibration_node                         # 정지
```

---

## ✅ 완료한 것

### Stage 1~3 (시뮬레이션, 개발 PC) — 완료
- ROS-free 코어 `core/`: geometry, path, path_factory(정적경로 6종), vehicle_model(자전거모델), feasibility, config_schema, controllers(PurePursuit/Speed/Battery)
- 시뮬 `sim/`: simulator, logger(CSV 고정스키마), run_sim, tune(그리드 튜너)
- 분석 `analysis/`: plot_logs.m (MATLAB) + .py
- 설정 `config/`: vehicle/controller/sim YAML
- 테스트 `tests/`: **21개 전부 통과** (`python3 -m pytest -q`)
- 주요 성과: 전방 윈도우 최근접탐색으로 figure-eight 완주, 실현가능성 검사로 불가능 경로 판별, 그리드 튜닝으로 sharp_s CTE 0.75cm

### Stage 6-a (실차 브링업) — 진행 중
- `ros2_ws/src/racer_bringup/` 패키지 작성 → 보드 배포 → 빌드 성공
- ✅ control_node 하드웨어 연결 확인 (i2c bus=3, PCA9685 0x40)
- ✅ **조향 배관 검증**: 우리 노드 → /control → 서보 → 앞바퀴 좌우 움직임 확인 (steer_sweep)
- ✅ calibration_node 실시간 파라미터 조정 동작 (hold 모드, 값 바뀔 때만 로그)

---

## 🔖 2026-07-04/05 세션 메모 (판단 로직 + YOLO 벤치마킹)

### 판단(Decision) 상태기계 — core 순수 로직 작성·검증 완료 ⭐ (07-05)
- `core/planning/decision.py` 신설. ROS-free, (LaneObservation, dt) → DriveCommand.
  - `DriveState(IntEnum)`: INIT=0/DRIVE=1/SLOW=2/STOP=3/LOST=4 (**계약 §4.4 값과 일치**).
  - `LaneObservation`: LaneStatus 미러(+외부 `stop_request`). `DriveCommand`: state/go/speed_scale/lookahead_scale/steer_limit.
  - `DecisionMaker.update()`: 우선순위 **정지요청 > 소실(LOST) > 정지선 > 신뢰도/자세(DRIVE/SLOW)**.
    - 소실: 유효검출 끊김이 `lost_grace`(0.3s) 이상 지속돼야 LOST(짧은 끊김은 직전상태 유지 → 떨림 방지).
    - 복귀: INIT/LOST는 유효검출 `recover_grace`(0.1s) 이어져야 주행 진입.
    - SLOW 유발: confidence<conf_drive(0.6) 또는 |heading|≥0.35rad 또는 |offset|≥0.12m 또는 정지선 접근(≤0.6m).
    - STOP: 정지선 ≤0.25m 또는 stop_request. 정지선 STOP은 `stop_dwell`(2s) 후 재출발 래치 → 데드락 방지.
- 임계값 스키마: `core/config_schema.py`에 `DecisionConfig` 추가, `AppConfig.decision`로 편입. 튜닝값 `config/decision.yaml`.
- 판단은 제어기 튜닝(controller.yaml) 안 건드림, "배율/게이트"만 냄(계약 원칙 준수).
- 테스트 `tests/test_decision.py` **12개 전부 통과**(전체 스위트 33개 통과). config 3-파일 병합 로드 확인.
- **다음(ROS화)**: `racer_msgs`(LaneStatus/DriveCommand) 생성 후 얇은 `decision_node`가 이 로직을 감싸 `/perception/lane_status` 구독 → `/decision/drive_command` 발행. controller_node에 drive_command 구독+watchdog 반영(계약 §5).

### YOLO 위치 재정리 (commit 13fb28a, 07-04)

### YOLO 위치 재정리 (commit 13fb28a, 07-04)
- 추론 노드(YOLO 실행): `d_racer_autonomous/ros2_ws/src/d_racer_perception`
- 후처리 로직(순수 기하): `d_racer_autonomous/core/perception/`
- `d_racer_perception`은 현재 **제거 가능한 테스트 노드**만 포함(`yolo_detect_test_node`, `yolo_seg_test_node`) — 모델 런타임·카메라 배선·디버그 출력 확인용. 최종 주행명령 발행 안 함.

### NCNN 변환 + 레이턴시 실측 ⭐ (07-05, D3-G 보드 CPU 4코어)
- 테스트 모델 다운로드(`scripts/download_test_models.sh`) → `models/yolo_{detect,seg}_test.pt`.
- Ultralytics YOLO26n 계열을 **NCNN으로 export** (imgsz **320 고정**): `models/*_ncnn_model/`.
- 벤치: `scripts/bench_seg.py`(torch/ncnn 공통, `YOLO(dir)`로 ncnn 로드). imgsz 320, 20 runs, warmup 3.

| 모델 | 포맷 | p50 latency | 평균 FPS |
|------|------|-------------|----------|
| seg    | torch | 212 ms | 4.6 |
| seg    | **ncnn** | 115 ms | **8.6** |
| seg    | ncnn(OMP4) | 105 ms | 9.0 |
| detect | torch | 157 ms | 6.1 |
| detect | **ncnn** | 65 ms | **14.3** |

- **결론**: 보드 인지는 **무조건 NCNN**(torch 대비 ~2배). OMP 스레드 증량은 NCNN 내부 멀티스레딩과 겹쳐 이득 미미. detect(~14FPS) ≫ seg(~9FPS). 차선 segmentation 채택 시 실질 상한 ≈ **9 FPS @320** — 폐루프 제어엔 빠듯하나 사용 가능.
- **주의**: NCNN export가 imgsz 320에 고정되어 있음. 다른 해상도 쓰려면 재export 필요.

---

## 🔖 2026-07-02/03 세션 메모

### 트림 모델 확정 (가법 가정 폐기) ⭐
- control_node.py 코드 확인: `/control` 명령이 오면 `self.steering = msg.steering` (control_node.py:121) — **트림을 안 더하고 그대로** 서보로. `STEER_TRIM`은 시작 idle값(75)·종료 중립값(151)으로만 쓰임.
- 결론: **직진 서보명령 = 0.2238**. 키트 `vehicle_config.yaml: STEER_TRIM=0.2238`(commit 93c31a0 push 완료), 우리 `vehicle.yaml: steer_trim=0.2238`. **우리 controller가 출력에 0.2238을 직접 더한다.**
- 팀 참고: 팀원이 올렸던 0.3238은 가법가정 실수 → 0.2238로 정정·push됨.

### 데이터 행방 (⚠️ 확인 필요)
- 기존 임시 YOLO 데이터 폴더에는 collect1/SECOND 데이터가 없었음. 보드 디스크에 07-02 카메라 프레임 없음.
- 07-02 rosbag `bagfile/bag_20260702_091344`는 `/joystick`만 214개(카메라 없음) — 학습데이터 아님.
- → Roboflow 업로드분/데스크탑에 사진이 있는지 확인할 것. 없으면 트랙에서 재수집.

## 🔖 2026-07-01 세션 메모 (배터리 방전으로 중단)

### 배관 재검증 완료 ✅ (오늘 실차에서 확인)
- 캘리 파이프라인 정상 동작 확인: `control_node(use_joystick_control:=False)` + `calibration.launch.py mode:=hold` + 터미널3 `ros2 param set /calibration_node steering <값>`.
- **함정**: `steering` 값을 `0.03~0.05`처럼 작게 주면 바퀴 변화가 눈에 안 보인다. `0.6/-0.6`처럼 크게 주면 확실히 좌우로 꺾인다. → 배관 확인은 큰 값으로 할 것.
- 터미널 구성: T1=control_node(False, 켜둠), T2=calibration.launch(hold, 켜둠), T3=param set 입력.

### ⚠️ 발견: 보드 STEER_TRIM이 0.30으로 커져 있음
- 보드 `~/D-Racer-Kit/src/config/vehicle_config.yaml`의 `STEER_TRIM`이 **0.30** (어제는 0.10). control_node 기동 로그에서 `steer_trim=0.30` 확인. 값이 과해 `steering=0.0`에서 앞바퀴가 오른쪽으로 휨.
- control_node 최종 서보각 = (calibration steering 명령) + STEER_TRIM(0.30), 클램프.
- **직진 되는 calibration steering 값 = X** 를 찾는 중이었음(정면 맞췄다고 함, but 값 기록 전 배터리 방전).
  → 최종 트림 = `0.30 + X`. 이 최종값을 `config/vehicle.yaml: steer_trim`과 보드 `vehicle_config.yaml: STEER_TRIM`에 기록하고, calibration steering은 0으로 되돌릴 것.

### 2차 세션(07-01 오후) 실주행 결과
- **직진값 후보: calibration steering ≈ 0.22** (보드 STEER_TRIM=0.30 상태에서 실제 바닥 주행이 제일 곧게 감). 거치대 눈대중(0.35)보다 낮음 — 역시 굴려봐야 정확. 배터리 충전 후 재확인 필요.
  → ~~확정 시 최종 STEER_TRIM ≈ 0.30+0.22 = 0.52 (additive 가정)~~ **[07-02 폐기]** 가법 가정은 틀림. control_node.py:121에서 `/control` 명령을 트림 없이 그대로 서보에 전달함(steer_trim은 idle/종료 중립값으로만 사용). 실측 직진 서보명령 = **0.2238**. → 키트 vehicle_config.yaml `STEER_TRIM=0.2238`, 우리 controller가 출력에 0.2238을 직접 더한다.
- **모터 데드존(바닥)**: throttle 0.15로는 안 돌고 삐 소리만. throttle_limit을 0.22~0.25로 풀어야 함(`ros2 run racer_bringup calibration_node --ros-args -p mode:=hold -p throttle_limit:=0.25`). launch파일 쓰면 0.15로 리셋되니 주의.
- **⚠️ 배터리 이슈 재발**: 주행 중 방전되어 보드 네트워크 끊김(No route to host) → 재부팅. 이후 모터가 throttle 0.18에도 삐 소리만 나고 안 돎 = **저전압 컷오프 의심**(조향 서보는 됨). 충전/완충 배터리 교체로 해결 예상. 대회 전 여분 배터리·완충 습관 필요.
- 주행 테스트 시 **차가 와이파이 범위 벗어나지 않게**, 멀어지기 전에 throttle 0.

### 조이스틱 조종 이슈 (캘리와 별개, 나중에)
- `manual_driving.launch`로 joystick_node는 뜨지만 스틱/버튼 값이 전부 0.00으로 읽힘.
- 단, `jstest /dev/input/js0`에서는 좌우 움직임에 값이 바뀜 → **하드웨어/리눅스는 정상, ROS joystick_node가 장치를 못 읽는 것**. 노드가 읽는 장치경로/축매핑 확인 필요(`manual_driving.launch.py` + joystick 패키지 소스). 캘리엔 지장 없음.

### 충전 후 재개 순서
1. T1/T2/T3 다시 띄우기(위 구성). control_node는 반드시 `use_joystick_control:=False`.
2. `ros2 param get /calibration_node steering`로 값 확인하며 **정면 X** 다시 찾기.
3. 최종 트림 `0.30+X` 기록 → 손으로 밀어 직진 검증.
4. 이어서 최대 조향각 / throttle 데드존 / 휠베이스 측정.

---

## ⏳ 다음 할 일

### 1. 캘리브레이션 마무리 (거치대, 바퀴 띄운 상태)
- [~] **최대 조향각**: 손측정 ≈ **10도** → `config/vehicle.yaml: max_steer_deg=10.0` (잠정, 실주행 원그리기 δ=atan(L/R)로 재확인)
- [ ] **throttle 데드존 측정**: `throttle 0.08`부터 0.02씩 올려 뒷바퀴 막 도는 값 찾기 (⚠️ 바퀴 띄운 상태 필수)
- [x] **직진 트림 (07-02 확정)**: 실주행 직진 서보명령 = **0.2238**. 가법 가정 폐기(control_node는 /control 명령을 트림 없이 그대로 서보 전달, control_node.py:121). → 키트 `src/config/vehicle_config.yaml: STEER_TRIM=0.2238` (commit 93c31a0, push 완료), 우리 `config/vehicle.yaml: steer_trim=0.2238` 기록됨. controller가 출력에 0.2238을 직접 더한다.
- [x] **휠베이스** 자로 측정 → `config/vehicle.yaml: wheelbase` = **0.175** (2026-07-01, 좌우 1mm차 무시)
- [ ] 측정값으로 `core.feasibility.check_path` 다시 돌려 실제 차량 기준 경로 가능여부 확인

### 2. Stage 6-b: controller_node (core 감싸기) — ✅ 노드 작성·검증 완료(07-03), 실차 확인 남음
- [x] `core`를 import하는 ROS2 `controller_node` 작성: `racer_bringup/controller_node.py` (PurePursuit+Speed → /control). launch `controller.launch.py`, entry point 등록.
- [x] core 패키징: 하드코딩·복사 없이 `_find_core_root()`가 상위 디렉토리에서 `d_racer_autonomous/`(core/+config/)를 찾아 sys.path 삽입. **소스·colcon install 위치 둘 다** 해석 확인(install에서 core_root=…/d_racer_autonomous 정상). `D_RACER_ROOT` 환경변수로 override 가능.
- [x] 트림: PP `compute()`가 `steer_trim`(0.2238)을 이미 출력에 더하므로 노드는 `steering_norm` 그대로 발행(재가산 금지). 스모크: straight→0.2238(δ=0°), circle→+5.00°, sharp_s→포화(+10°).
- [x] 안전: `enable_drive`(기본 False)→throttle=0, True라도 `throttle_limit`(0.15) 하드클램프. Speed는 계산·로깅만(속도→throttle 매핑은 Stage 4).
- [x] 빌드/실행 확인: `colcon build --packages-select racer_bringup` OK, `ros2 run racer_bringup controller_node`로 /control 발행·로깅 확인(하드웨어 미접촉 토픽).
- [ ] **실차 확인 남음**: 거치대에서 `control_node(use_joystick_control:=False)` + `controller.launch.py path:=circle`로 앞바퀴가 곡률 방향으로 꺾이는지 눈으로 검증. 이후 저속 `enable_drive:=True drive_throttle:=0.12`.
- [ ] (알아둘 점) max_steer_deg=10 + trim 0.2238이라 정규화 조향 실효범위 비대칭([-0.776, +1.0], 우측 조기 포화). 물리적 사실 — 필요시 트림/최대각 재측정.

  사용법:
  ```bash
  cd ~/SEA-ME_Hackathon_SMT && source /opt/ros/humble/setup.bash && source install/setup.bash
  # T1: 키트 모터 제어(켜둠)
  ros2 run control control_node --ros-args -p use_joystick_control:=False
  # T2: 우리 컨트롤러 (조향만, throttle=0)
  ros2 launch racer_bringup controller.launch.py path:=circle
  # 저속 구동까지(거치대에서): enable_drive:=True drive_throttle:=0.12
  ```

### 3. Stage 4: 실차 주행 + 로깅
- [ ] logger_node로 주행 로그(CSV/rosbag) 저장 → MATLAB 분석 → 재튜닝

### 4. Perception (가장 어려움, 카메라 단독)
- [ ] 카메라 이미지 → 차선 중심선(로컬 프레임) 출력. 키트 `bagfile/`, `data_acquisition.sh`로 녹화 이미지 오프라인 개발 가능
- [ ] 이게 pose(로컬 경로)를 제공해야 비로소 폐루프 자율주행 완성

---

## 핵심 설계 원칙 (잊지 말 것)
- 인지/판단/제어 **절대 분리**. Controller는 Path만 입력받음.
- 모든 인지는 **카메라 단독** → Pure Pursuit는 **로컬 프레임**(차량=원점).
- ROS-free 코어 + 얇은 ROS2 노드 래퍼 (시뮬↔실차 동일 코드).
- 하드코딩 금지, 모든 파라미터 YAML.
- 개발 순서 준수: Static Path → Bicycle Model → 튜닝 → 실차 → 분석 → Perception.
- 자세한 설계는 `docs/architecture.md`, 캘리브레이션 절차는 `docs/calibration.md`.
- **노드 인터페이스 계약(인지/판단/제어 토픽·메시지)은 `docs/interfaces.md`** — 인지·판단 구현 전 반드시 참조/합의.
