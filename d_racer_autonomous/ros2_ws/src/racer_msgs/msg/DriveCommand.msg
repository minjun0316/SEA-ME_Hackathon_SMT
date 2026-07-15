# /decision/drive_command — 판단이 발행, 제어가 소비.
# 상태 + "배율/게이트"만 준다(제어기 튜닝값은 안 건드림). 계약: docs/interfaces.md §4.4.
std_msgs/Header header

uint8   state             # 아래 STATE_* 상수 중 하나(디버그/로깅용)
bool    go                # false면 제어가 throttle=0(정지). 조향은 유지.
float32 speed_scale       # v_max에 곱하는 배율 [0.0~1.0]. 코너/불확실 시 감속.
float32 lookahead_scale   # lookahead 배율 [기본 1.0]. (선택; 미사용 시 1.0)
float32 steer_limit       # 정규화 조향 상한 [0.0~1.0]. (선택; 미사용 시 1.0)
# [2026-07-14] 방향 팻말 구간(SIGN_BRANCH) 조향 bias. 제어가 정규화 조향에 가산한다.
# 흰선 추종은 그대로 두고 좌/우로 살짝 치우치게만 한다. 그 외 구간=0.0.
float32 steer_bias        # 정규화 조향 offset(기본 0.0). 부호=제어 조향 규약(좌/우). (선택; 미사용 시 0.0)

# state 상수 (core.planning.DriveState 값과 일치)
uint8 STATE_INIT=0        # 초기화/대기
uint8 STATE_DRIVE=1       # 정상 주행
uint8 STATE_SLOW=2        # 감속 주행(코너/저신뢰)
uint8 STATE_STOP=3        # 정지(정지선/장애물/명령)
uint8 STATE_LOST=4        # 차선 소실 → 정지 또는 마지막 조향 유지
