# /perception/lane_status — 인지가 발행, 판단이 소비.
# 차선 검출 메타(신뢰도·자세·정지선). 계약: docs/interfaces.md §4.3.
# 좌표/단위: base_link(뒷차축 원점), 길이 m, 각도 rad. +y=좌측.
std_msgs/Header header

bool    lane_detected     # 유효 차선 검출 여부(false면 제어는 watchdog로 정지)
float32 confidence        # 0.0~1.0 검출 신뢰도
int32   num_points        # lane_path 점 개수(0이면 미검출)
float32 lateral_offset    # 차량중심 대비 차선중심 횡오차 [m], +좌측
float32 heading_error     # 차량 전방 대비 차선 접선 오차 [rad]
bool    stop_line         # 정지선 검출 여부(제어단 StoplineManeuver가 카운트)
float32 stop_line_dist    # 정지선까지 거리 [m] (미검출 시 -1.0)
# [2026-07-13] 색 검출 필드(yellow/white_detected+confidence) 제거 — 로터리 ROI 방식
# 폐기로 소비처 없음. 노랑/흰 마스크 자체는 인지 내부 차선검출에 계속 사용.
