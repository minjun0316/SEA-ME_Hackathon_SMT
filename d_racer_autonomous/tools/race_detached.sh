#!/usr/bin/env bash
# @file race_detached.sh
# @brief 대회용: SSH가 끊겨도(= WiFi 동글을 뽑아도) 주행 스택이 계속 돌게 분리 실행.
#
# 왜 이 스크립트인가 (07-16b):
# - 그냥 `ros2 launch ...`는 SSH 세션의 자식이라 동글을 뽑으면 SIGHUP으로 같이 죽는다.
# - `nohup ... &`만으로는 부족하다 — 세션은 그대로라 sshd가 세션을 정리하면 죽을 수
#   있다. `setsid`로 **세션 자체를 분리**해야 확실하다(PPID=1, TTY 없음 실측 확인).
# - DDS(FastDDS)는 같은 보드 노드끼리 기본으로 SHM(공유메모리) 전송을 쓰므로 주행 중
#   wlan0이 사라져도 데이터는 계속 흐른다. 단 이건 이론이므로 아래 "벤치 검증"을
#   대회 전에 반드시 한 번 통과시킬 것.
#   ⚠ ROS_LOCALHOST_ONLY=1 은 이 보드에서 **동작하지 않는다**(07-16b 실측: lo에
#     MULTICAST 플래그가 없어 FastDDS 디스커버리 실패). 쓰지 말 것.
#
# 사용법:
#   bash race_detached.sh                    # 대회 기본 인자로 실행
#   bash race_detached.sh enable_drive:=False  # 인자 추가/오버라이드(뒤가 이김)
#
# 벤치 검증 절차(대회 전 1회, 거치대에서):
#   1) bash race_detached.sh enable_drive:=False   # 바퀴 안 돌게
#   2) WiFi 동글을 뽑는다 → SSH가 끊긴다 → 60초 기다린다
#   3) 동글을 다시 꽂는다 → WiFi 자동 재접속(NetworkManager) → SSH 재접속
#   4) racer-hz 로 토픽이 계속 흐르는지 + 아래 로그에 에러 없는지 확인
#   5) stop 으로 정리
#
# 정지: 동글 다시 꽂고 SSH 접속 → `stop`(비상정지) 또는 `racer-clean`(전체 정리).
# ⚠ 동글이 빠진 동안엔 원격 정지 수단이 없다 — 물리적으로 차를 들거나 전원을 끄는 것뿐.

# ⚠ set -u 금지: ROS setup.bash가 미정의 변수(AMENT_TRACE_SETUP_FILES)를 참조해 죽는다.

# 이 스크립트 위치 기준으로 워크스페이스를 찾는다(어디서 실행해도 동작).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

LOG_DIR="$HOME/race_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/race_$(date +%m%d_%H%M%S).log"

# 대회 기본 인자(07-16b 실차 확정 세팅). "$@"가 뒤에 붙으므로 같은 인자를 다시 주면
# 뒤가 이긴다(ros2 launch는 중복 인자에서 마지막 값을 쓴다) = 오버라이드 가능.
ARGS=(
  enable_drive:=True
  drive_throttle:=0.08
  throttle_limit:=0.10
  use_decision:=True
  use_mission:=True
  publish_debug:=True
)

setsid nohup ros2 launch racer_bringup lane_follow.launch.py \
  "${ARGS[@]}" "$@" > "$LOG" 2>&1 < /dev/null &
PID=$!

sleep 4  # 노드들이 뜰 시간.

echo "── race_detached ──────────────────────────────────────"
echo "  launch PID : $PID (setsid 분리 — SSH 끊겨도 유지)"
echo "  인자       : ${ARGS[*]} $*"
echo "  로그       : $LOG"
echo "  분리 확인  : $(ps -o ppid=,sid=,tty= -p "$PID" 2>/dev/null | awk '{print "PPID="$1" SID="$2" TTY="$3}')"
echo "── 노드 상태 ──────────────────────────────────────────"
for n in camera_node lane_detect_node mission_node controller_node control_node; do
    if pgrep -f "$n" > /dev/null; then echo "  [OK] $n"; else echo "  [--] $n (로그 확인!)"; fi
done
echo "───────────────────────────────────────────────────────"
echo "  이제 동글을 뽑아도 된다. 정지: 동글 재삽입 → SSH → stop"
tail -5 "$LOG" | sed 's/^/  log> /'
