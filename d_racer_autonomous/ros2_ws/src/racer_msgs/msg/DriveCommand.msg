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
# [2026-07-16] 구간별 제어 게인 프로파일. 아래 PROFILE_* 중 하나.
# 판단은 "지금 어느 성격의 구간인지"만 말하고 게인 **수치는 모른다** — 실제 값은
# controller.yaml(lateral_pd / lateral_pd_post_sign)에 있다(§4.4 튜닝 일원화).
# 왜: S자는 곡률이 연속이라 heading의 선반영(ψ=경로 끝점까지의 현 각도라 1m 앞을 봄)이
# 이득인데, 팻말 뒤는 직선-ㄱ자-직선-ㄱ자로 곡률이 계단처럼 뛰어 같은 선반영이 조기조향
# (라인 이탈)이 된다. 한 세트로 둘 다 만족 못 함 → 구간별 세트로 가른다.
uint8   gain_profile      # PROFILE_* 중 하나(기본 0=DEFAULT). (선택; 미사용 시 DEFAULT)

# gain_profile 상수 (core.planning.GainProfile 값과 일치)
uint8 PROFILE_DEFAULT=0   # 기본(S자 흰선 코스 등) → controller.yaml lateral_pd
uint8 PROFILE_POST_SIGN=1 # 팻말 분기 후(직선-ㄱ자-직선-ㄱ자) → lateral_pd_post_sign

# state 상수 (core.planning.DriveState 값과 일치)
uint8 STATE_INIT=0        # 초기화/대기
uint8 STATE_DRIVE=1       # 정상 주행
uint8 STATE_SLOW=2        # 감속 주행(코너/저신뢰)
uint8 STATE_STOP=3        # 정지(정지선/장애물/명령)
uint8 STATE_LOST=4        # 차선 소실 → 정지 또는 마지막 조향 유지
