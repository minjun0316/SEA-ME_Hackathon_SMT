# 인지 (Perception) 설계 문서

> [README](../README.md)의 인지 파트 설계 근거·구현·검증을 정리한다.
> 담당 — 강민준 · 최정윤 · 코드: [`d_racer_autonomous/core/perception/`](../d_racer_autonomous/core/perception/), 노드: [`ros2_ws/src/d_racer_perception/`](../d_racer_autonomous/ros2_ws/src/d_racer_perception/)

**목차**

1. [역할과 출력 계약](#1-역할과-출력-계약)
2. [차선 검출 파이프라인](#2-차선-검출-파이프라인)
3. [출력 신호 산출](#3-출력-신호-산출)
4. [팻말 분기 offset](#4-팻말-분기-offset)
5. [신호등·팻말 YOLO](#5-신호등팻말-yolo)
6. [팻말 색 트리거 · ArUco](#6-팻말-색-트리거--aruco)
7. [온보드 실시간화](#7-온보드-실시간화)
8. [설계 판단과 문제 해결](#8-설계-판단과-문제-해결)

> 값 표기 규약 — 인지 파라미터는 코드 기본값(`LaneCalib` 등 dataclass)과 실차 운용값(`config/*.yaml`)이 다른 경우가 많다. 이 문서는 **실차 운용값 = YAML**을 기준으로 적고, 코드 기본값은 필요할 때 병기한다.

---

## 1. 역할과 출력 계약

인지는 카메라 프레임에서 **"무엇이 어디에 있는가"만** 출력한다. 조향·속도는 계산하지 않는다(불변 규칙 ①). 센서는 전방 카메라 하나(20Hz)뿐이며, 차선은 OpenCV, 신호등·팻말 방향은 자체 학습 YOLO, 동적 장애물은 ArUco 마커로 분담한다.

출력은 두 개의 ROS2 커스텀 메시지로 고정한다.

| 메시지 | 필드(발췌) | 소비자 |
|---|---|---|
| `LaneStatus` | `lane_detected` · `confidence` · `num_points` · `lateral_offset`[m] · `heading_error`[rad] · `lane_path`(차선 경로점) · `stop_line`·`stop_line_dist` | 판단 → 제어 |
| `MissionCues` | `traffic_light`(GREEN/RED) · `sign_direction`(LEFT/RIGHT/NONE) · `aruco_present` | 판단 |

> 곡률은 인지가 산출하지 않는다. `LaneStatus.lane_path`(중심선 경로점)를 제어단이 받아 부호 곡률로 유도한다([제어 문서 §3](control.md#3-곡률-계산)). 인지의 계약은 "경로점 + 근거리 오차 두 개"까지다.

---

## 2. 차선 검출 파이프라인

`BEV 변환 → HLS 흰색 마스크 → Canny 에지 → 슬라이딩 윈도우 → 중심선 → 미터 변환`
구현: [`core/perception/lane_detect.py`](../d_racer_autonomous/core/perception/lane_detect.py) (`LaneDetector.detect`)

주행 차선은 **흰선 전용 폐루프**다. 노랑 마스크는 차선 추종에서 폐기하고 정지선 검출에만 쓴다(07-14 전환).

### 2.1 BEV(조감도) 변환

`straight` 캡처에서 두 흰 차선을 선형 피팅해 얻은 사다리꼴을 직사각형으로 매핑하는 호모그래피 $H$로 원근을 제거한다. 실차는 실측 행렬을 YAML에 직접 박아 쓰고(`bev_matrix`, 9원소 행우선 3×3), 노드는 4점 자동생성 폴백을 비활성화한다.

$$\begin{bmatrix} u' \\ v' \\ w' \end{bmatrix} = H \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}, \qquad H = \begin{bmatrix} 23.55 & 33.76 & -3795.97 \\ 0 & 54.10 & -2975.77 \\ 0 & 0.214 & 1 \end{bmatrix}$$

(320×160 기준, 두 차선을 x=90·x=230으로, 간격 140px=35cm에 매핑.) `bev_matrix`가 없으면 `bev_top_x=0.2`·`bev_top_y=0.4`로 4점(`getPerspectiveTransform`)을 생성하는 폴백 경로가 있다. 캘리브레이션 절차는 [`docs/calibration.md`](../d_racer_autonomous/docs/calibration.md) 참고.

### 2.2 흰색 마스크 · 에지

- **HLS 흰색 마스크** — `cv2.inRange(hls, white_lo, white_hi)`, 실차값 `white_lo=(0,213,0)`, `white_hi=(172,255,255)` (OpenCV 규약 H 0–179, L·S 0–255). 흰 픽셀 수가 `white_pixel_threshold=200` 이상이면 `white_detected`.
- **에지** — 마스크에 `MORPH_CLOSE`(k=3) → `dilate`(1회) → `Canny(canny_lo, canny_hi)`. 실차 `canny_lo=30`, `canny_hi=90`(흰선 민감도를 위해 코드 기본 50/150에서 낮춤).

### 2.3 슬라이딩 윈도우

창 `nwindows=9`, 마진 `margin=30`, 창별 최소 픽셀 `minpix=5`. 창별로 잡힌 점을 2차 다항식으로 피팅(`np.polyfit`, 점이 3개 미만이면 1차)하고, 잡힌 창이 2개 미만이면 무효 처리한다. `confidence`는 **픽셀이 실제로 잡힌 창 수 / 9**로, 피팅으로 연장된 창은 제외한다.

시드(창 시작 x)는 신뢰도 기반 lock/lost 상태를 가진다.

- **lock** (직전 `confidence ≥ seed_lock_conf=0.40`) — 직전 near-x를 중심으로 search-around-poly. 밴드 `±seed_hist_band_px`(실차 90px) 안에서만 히스토그램 피크와 블렌딩한다.
- **lost 재획득** — 하단 40% 영역 히스토그램의 argmax를 그대로 믿지 않고, **최소 에지량 게이트**를 넘을 때만 직전 시드와 블렌딩한다. argmax는 에지가 없어도 인덱스를 반환하므로, 미달이면 화면 25%/75% 기본 위치로 고정해 노이즈 점프를 차단한다.

$$\text{min\_mass} = 255 \cdot n_{rows} \cdot \text{seed\_min\_fill}, \qquad \text{seed\_min\_fill}=0.05$$

```python
# _reacquire_base: 게이트 통과 시에만 블렌딩, 미달이면 "안 보임" 인정
if peak_mass >= min_mass:
    base = prev * (1 - b) + argmax * b     # b = seed_reacquire_blend
else:
    base = w * 0.25                        # 노이즈 점프 원천 차단
```

차선폭은 양쪽이 모두 잡히면 EMA로 학습하고(`lane_width_learn`), 두 피팅선 간격이 학습폭의 `lane_collapse_frac=0.5` 미만이면 단일선으로 강등해 붕괴를 막는다. 실차는 폭을 `lane_width_px=146` 고정으로 운용한다(학습 off).

### 2.4 픽셀 → 미터

중심선 픽셀을 차량 좌표(m)로 환산한다.

$$x = x_{near} + (h - row)\cdot m_{px}^{fwd}, \qquad y = (mid - col)\cdot m_{px}^{lat}$$

계수는 IPM 이상값이 아니라 **실측 회귀**로 뽑았다(실차 YAML): $m_{px}^{fwd}=0.005721$ (전방 잔차 3.9cm RMS), $m_{px}^{lat}=0.002506$ (횡 잔차 3.7cm RMS, 차선폭 35cm≈140px), $x_{near}=0.0539$. 변환 후 경로를 다시 2차로 스무딩한다.

---

## 3. 출력 신호 산출

스무딩된 미터 경로 `path_m`에서 제어가 받을 근거리 신호 두 개를 뽑는다.

```python
xn, yn = path_m[0]     # 최근접점
xf, yf = path_m[-1]    # 맨 끝점
lateral_offset = yn                              # 횡오차 [m, +좌]
heading_error  = atan2(yf - yn, xf - xn)         # 차선 방향 vs 차량 전방 [rad]
```

- `lateral_offset` = 최근접점의 y (횡오차).
- `heading_error` = 최근접점→맨 끝점 현(chord) 각도. 이 "맨 끝점"이 차 앞 약 1m라는 점이 제어 §4의 프로파일 분기 근거가 된다.
- `lane_detected = (중심선 점 수 ≥ 2) and (confidence > 0)`.

정지선은 별도로 검출한다(coverage / Hough 두 방식, 실차 Hough). 노랑 마스크(`yellow_lo=(0,146,129)`, `yellow_hi=(153,225,255)`)로 가로 띠를 찾고, 연속 색 길이(`_longest_run`)로 벽·얼룩을 배제한 뒤 `stop_line_dist`를 미터로 환산한다.

---

## 4. 팻말 분기 offset

팻말은 도로 정중앙의 장애물이다. 추종 로직은 그대로 두고 **완성된 중심선만 지시된 통로 쪽으로 평행이동**한다(`sign_apply="offset"`). 차선이 잡히면 무조건 발동하는, 게이트 없는 단순 구조다.

```python
# lane_detect.py: force_side = 'left' | 'right'
dx = sign_offset_px * (+1 if force_side == 'right' else -1)   # RIGHT=+, LEFT=−
centerline_px = [(cx + dx, cy) for (cx, cy) in centerline_px]
```

offset의 미터→픽셀 환산은 노드에서 한다: `sign_offset_px = |sign_lane_offset_m| / m_per_px_lateral`. 실차 `sign_lane_offset_m=0.09` → 약 **35.9px**. 이 0.09m는 차선폭 35cm의 1/4(통로 중앙 = W/4 ≈ 8.75cm)에서 나온 값으로, 차폭(~14cm)을 감안하면 좌우 여유가 ±1.5cm뿐이라 정밀 offset이 중요하다.

관측성을 위해 `sign_force_status`에 `applied:right(offset +36px)` / `dropped:...:중심선없음` 형태로 매 프레임 상태를 남긴다. (앵커 방식은 폐기했다 — §8.1.)

---

## 5. 신호등·팻말 YOLO

신호등과 방향 팻말은 **단일 통합 YOLO 모델 1회 추론**으로 검출하고, 노드가 클래스로 라우팅한다.

- 클래스 계약: **0=green, 1=left, 2=red, 3=right**. `green`/`red`는 신호등, `left`/`right`는 팻말 방향으로 라우팅. 클래스 id 순서가 계약이므로 재학습 시 조용히 오작동할 수 있어 교체 스크립트가 자동 대조한다.
- 모델: YOLO26n, `smsmt_smt_coco_combined` 데이터셋 직접 수집·라벨링, 학습 imgsz=640 → NCNN export imgsz=320.
- 추론: `conf=0.35`, `imgsz=320`, 추론 상한 `yolo_max_infer_hz=6.0`.
- 색 판정: 둘 다 보이면 `prefer_red`/신뢰도로 RED 우선. 방향: `right_conf ≥ left_conf`면 RIGHT.
- **오검출 컷 게이트**(실차): 팻말은 거리(`sign_min_box_h_frac=0.10`)·신뢰도(`sign_min_conf=0.8`), 빨강은 `red_min_conf=0.7`. 빨강 확정은 시간이 아니라 **추론 프레임 수**(`tl_red_confirm_frames`)로 카운트한다(프레임 재발행 함정 회피).

---

## 6. 팻말 색 트리거 · ArUco

**팻말 색 트리거** ([`sign_detect.py`](../d_racer_autonomous/core/perception/sign_detect.py)) — HSV 마스크(`cv2.inRange`)로 팻말 구간 진입을 감지한다. 실차 `sign_color_lo=(100,120,60)`, `sign_color_hi=(130,255,255)`, `sign_min_px=400`. 방향 자체는 통합 YOLO가 담당하므로 별도 팻말 모델은 미사용(`sign_model_path=''`).

**ArUco** ([`aruco_detect.py`](../d_racer_autonomous/core/perception/aruco_detect.py)) — 사전 `DICT_6X6_50`, 타깃 `ID 3`. 사전 일치가 검출의 결정 요소다(4×4 사전이면 미검출 확인). OpenCV 4.7+/5.x의 `ArucoDetector`를 우선 쓰고 구버전 함수형 API로 폴백한다. **거리 지표는 solvePnP가 아니라 마커 둘레(perimeter)** 로, 가장 큰 마커를 가장 가까운 것으로 본다. 검출 파라미터는 OpenCV 스톡값을 유지했다(A/B에서 튜닝 이득이 없었다 — §8.3). 유무는 노드가 `aruco_hold_sec=1.0`으로 홀드해 디바운스한다.

---

## 7. 온보드 실시간화

4-core ARM(D3-G, TCC8050 Cortex-A72 ×4) 위에서 20Hz 카메라 파이프라인을 유지하기 위한 부하 분산.

**NCNN 변환** — PyTorch → NCNN, `imgsz=320`. `scripts/bench_yolo.py` 실측:

| 백엔드 | imgsz | 지연 | FPS |
|---|---|---|---|
| torch CPU | 640 | 419 ms | 2.4 |
| torch CPU | 320 | 151 ms | 6.6 |
| **NCNN** | **320** | **64 ms** | **15.5** |
| NCNN | 416 | 99 ms | 10.0 |

**CPU 코어 핀닝** — core0: 카메라·차선 / core1: 판단·제어 / core2·3: YOLO.
**YOLO 페이즈 게이트** — 판단이 미션 상태 기반으로 필요한 모델만 켠다(§8, [판단 문서 §5](planning.md#5-오검출-방어)). 결과적으로 카메라 20Hz 파이프라인을 안정 유지한다.

---

## 8. 설계 판단과 문제 해결

### 8.1 방향 팻말 분기 — 앵커 차선 폐기, offset 재설계

좌/우 팻말 **양쪽에서 분기가 대칭으로 어긋나는** 동일 증상이 나왔다. 좌우가 거울상으로 똑같이 틀어진다는 사실이 원인을 YOLO 모델이 아니라 **기하 구조**로 좁혔다.

- **원인 ⑴** 우커브에서 오른쪽 차선이 화면 밖으로 소실 → "오른쪽 차선 앵커"가 물리적으로 도달 불가.
- **원인 ⑵** 곡률 추정치(`curve_dx`) 노이즈가 커브 게이트를 무력화.
- **해결** 앵커 방식(항상 특정 차선에 붙이는 방식) 폐기 → **항상 보이는 왼쪽 차선 기준 ±offset**(§4). 옛 앵커 경로에 있던 `curve_dx` 중앙값 필터(tap 7)도 검증했다: 창 5는 22프레임 중 4프레임만 발동(불충분), **창 7은 12/22 연속 0.6s 발동**하면서 진짜 커브(dx 30–52px)는 0/22로 정확히 차단. offset 모드로 넘어오며 이 게이트 자체가 불필요해졌다.

### 8.2 흰선 전용 폐루프 전환 (07-14)

노랑 차선 추종을 완전히 폐기하고 흰선 단일 마스크로 통일했다. 노랑은 정지선 검출에만 남긴다. 색 하나로 좁혀 마스크·시드 로직의 변수를 줄였다.

### 8.3 실시간 파이프라인 확보

NCNN 변환(§7) + CPU 코어 핀닝 + 미션 상태 기반 YOLO 페이즈 게이트로 20Hz를 확보했다. ArUco는 스톡 검출 파라미터가 최적이었다 — A/B에서 커스텀 프리셋의 검출 이득은 0이었고(20/36 → 20/36), 창 스텝을 10→4로 줄이면 CPU만 3.9→6.4ms/frame(+64%)로 늘어 스톡값을 유지했다.

> **검증 범위 주의** — 오프라인(캡처 프레임)에서 커브 차선 검출은 확인했으나, 실차 커브 완주 정량 검증은 대회 운영 로그 기준으로 남아 있다. 위 수치는 벤치·회귀·A/B 로그 근거이며, 추정치가 아니다.
