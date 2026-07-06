# /perception/mission_cues — 인지가 발행, 판단이 소비.
# 미션 페이즈 전환용 객체/구역 신호 묶음(차선 기하 외). 계약: docs/interfaces.md.
# 출처: traffic_light·checkerboard = YOLO, red_zone = OpenCV 색, aruco = cv2.aruco.
std_msgs/Header header

uint8   traffic_light        # 신호등 상태(아래 TL_* 상수). 출발/정지 트리거.
bool    checkerboard_detected # 체커보드 출발/도착선 검출(YOLO). 도착 판정.
bool    red_zone_detected     # 빨강 바닥 장애물 구역 검출(OpenCV). 구간 진입/탈출.
bool    aruco_present         # 아루코 마커 검출(하단 ROI). 장애물 정지/재출발.

# traffic_light 상수 (core.planning.TrafficLight 값과 일치)
uint8 TL_NONE=0   # 미검출
uint8 TL_RED=1    # 빨간불(대기)
uint8 TL_GREEN=2  # 초록불(출발)
