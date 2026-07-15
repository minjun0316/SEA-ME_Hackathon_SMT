# /perception/mission_cues — 인지가 발행, 판단이 소비.
# 미션 페이즈 전환용 객체/구역 신호 묶음(차선 기하 외). 계약: docs/interfaces.md.
# 출처: traffic_light·checkerboard = YOLO, red_zone = OpenCV 색, aruco = cv2.aruco.
std_msgs/Header header

uint8   traffic_light        # 신호등 상태(아래 TL_* 상수). 출발/정지 트리거.
bool    checkerboard_detected # 체커보드 검출(YOLO). [2026-07-14] 도착 판정에서 폐기(빨간불 종료로 교체). 필드는 계약 호환 유지.
bool    red_zone_detected     # 빨강 바닥 구역(OpenCV). 폐기 stub(미소비). 필드는 계약 호환 유지.
bool    aruco_present         # 아루코 마커 검출(하단 ROI). 장애물 정지/재출발.
# [2026-07-14] 방향 팻말 분기(SIGN_BRANCH) 신호. OpenCV 팻말색=구간 진입 트리거,
# 팻말 YOLO(별도 모델)=좌/우 방향. 판단이 팻말 구간 진입 + 조향 bias 방향에 소비한다.
bool    sign_detected         # 방향 팻말색 검출(OpenCV 트리거). true면 팻말 구간 진입 → 팻말 YOLO on.
uint8   sign_direction        # 팻말 지시 방향(아래 SIGN_* 상수, 팻말 YOLO). 조향 bias 방향.

# traffic_light 상수 (core.planning.TrafficLight 값과 일치)
uint8 TL_NONE=0   # 미검출
uint8 TL_RED=1    # 빨간불(대기)
uint8 TL_GREEN=2  # 초록불(출발)

# sign_direction 상수 (core.planning.SignDirection 값과 일치)
uint8 SIGN_NONE=0   # 미검출/미정(→ bias 0, 직진)
uint8 SIGN_LEFT=1   # 좌측 지시
uint8 SIGN_RIGHT=2  # 우측 지시
