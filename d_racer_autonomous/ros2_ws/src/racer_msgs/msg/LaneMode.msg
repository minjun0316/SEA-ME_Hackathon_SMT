# /decision/lane_mode — 판단이 발행, 인지가 소비 (역방향 지시).
# 판단이 미션 상태에 따라 "어느 색/ROI/방향으로 볼지"를 인지에 지시한다.
# 실제 ROI 자르기·mask·target·bias 적용은 인지(OpenCV) 몫. 계약: docs/interfaces.md.
std_msgs/Header header

uint8 follow_color   # 추종할 차선 색(아래 COLOR_* 상수)
uint8 roi_mode       # 볼 ROI(아래 ROI_* 상수)
uint8 turn_bias      # target 좌/우 bias(아래 BIAS_* 상수)

# YOLO 추론 게이트: 미션 SM이 페이즈에 따라 인지의 무거운 YOLO 추론을 켜고 끈다.
# true=추론 수행(신호등/체커보드 검출), false=추론 skip(CPU/FPS 확보, aruco·차선은 계속).
# 게이트 구간: 출발 신호등 대기=ON → 주행중=OFF → 장애물구역서 아루코 보이면 ON(도착까지 유지).
# 발행자(mission_node) 없으면 인지는 기본 ON 유지(하위호환: 기존처럼 YOLO 항상 켜짐).
bool yolo_enable     # true=YOLO 추론 ON, false=OFF(skip)

# [2026-07-14] 방향 팻말 YOLO(별도 모델) 게이트. 팻말 구간(SIGN_BRANCH)에서만 true.
# yolo_enable(신호등 모델)과 독립 — 두 모델을 페이즈별로 따로 켜서 CPU/FPS를 아낀다.
# 발행자 없으면 인지는 기본 OFF(팻말 모델은 필요 구간에만 켜는 게 안전).
bool sign_enable     # true=팻말 YOLO 추론 ON, false=OFF(skip)

# follow_color 상수 (core.planning.LaneColor 값과 일치)
uint8 COLOR_WHITE=0
uint8 COLOR_YELLOW=1

# roi_mode 상수 (core.planning.RoiMode 값과 일치)
uint8 ROI_FULL=0         # 전체 ROI(로터리 진입/추종, 외곽)
uint8 ROI_LOWER=1        # 하단 ROI(직선/접근)
uint8 ROI_RIGHT=2        # 오른쪽 50~60% ROI(원형 계속)
uint8 ROI_LEFT=3         # 왼쪽 50~60% ROI(출구/커넥터)
uint8 ROI_LOWER_ARUCO=4  # 하단 ROI + 아루코 검출 우선(장애물)

# turn_bias 상수 (core.planning.TurnHint 값과 일치)
uint8 BIAS_NONE=0
uint8 BIAS_LEFT=1
uint8 BIAS_RIGHT=2
