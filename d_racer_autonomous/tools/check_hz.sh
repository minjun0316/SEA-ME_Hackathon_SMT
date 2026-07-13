#!/usr/bin/env bash
# check_hz.sh — 노드 실행 중 주요 토픽 발행 주파수(Hz) 점검용.
#   · /perception/lane_path 와 /control 을 라벨 붙여 계속 로그로 표시.
#   · YOLO 추론 Hz 는 무거우니(이미지 토픽) 3초만 측정하고 종료.
# 사용법:  bash tools/check_hz.sh
# 종료:    Ctrl-C  (백그라운드 hz 프로세스까지 정리됨)

set -u

# --- ROS 환경 준비 (이미 source 돼 있으면 중복 source 무해) ---
WS="${D_RACER_WS:-$HOME/SEA-ME_Hackathon_SMT}"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS/install/setup.bash" 2>/dev/null || true

# --- 토픽 이름 ---
LANE_PATH_TOPIC="/perception/lane_path"
CONTROL_TOPIC="/control"
MISSION_CUES_TOPIC="/perception/mission_cues"   # 판단이 소비하는 미션신호(신호등·체커보드·빨강·아루코). 매 프레임 발행 → 카메라 fps 추종.
YOLO_TOPIC="/perception/mission_cues/yolo/debug/compressed"   # 추론된 프레임에만 발행 → 추론 Hz
YOLO_MEASURE_SEC=3

# 라벨 붙여 hz 출력을 흘려보내는 헬퍼.
run_hz() {
    local label="$1" topic="$2"
    stdbuf -oL ros2 topic hz "$topic" 2>&1 \
        | stdbuf -oL sed "s/^/[$label] /"
}

# 백그라운드 프로세스 정리용 트랩.
PIDS=()
cleanup() {
    echo
    echo ">>> 측정 종료, 백그라운드 프로세스 정리 중..."
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# ============================================================
# 1) YOLO 추론 Hz — 3초만 측정 (이미지 토픽이라 무거움)
# ============================================================
echo "============================================================"
echo " YOLO 추론 Hz 측정 (${YOLO_MEASURE_SEC}s)  topic=$YOLO_TOPIC"
echo "  (publish_debug=True 여야 이 토픽이 나옵니다. 안 나오면 아래에서 대기만 함)"
echo "============================================================"
YOLO_OUT="$(timeout "$YOLO_MEASURE_SEC" ros2 topic hz "$YOLO_TOPIC" 2>&1)"
if echo "$YOLO_OUT" | grep -q "average rate"; then
    echo "$YOLO_OUT" | grep "average rate" | tail -1 | sed 's/^/  YOLO 추론 → /'
else
    echo "  [!] ${YOLO_MEASURE_SEC}s 동안 메시지 없음 — YOLO OFF 이거나 publish_debug=False 이거나"
    echo "      토픽명이 다릅니다. 확인: ros2 topic list | grep yolo"
fi
echo

# ============================================================
# 2) lane_path + control — 라벨 붙여 계속 표시 (Ctrl-C 로 종료)
# ============================================================
echo "============================================================"
echo " 계속 측정 (Ctrl-C 로 종료):"
echo "   [PATH] $LANE_PATH_TOPIC"
echo "   [CUES] $MISSION_CUES_TOPIC"
echo "   [CTRL] $CONTROL_TOPIC"
echo "============================================================"

run_hz "PATH" "$LANE_PATH_TOPIC" &
PIDS+=("$!")
run_hz "CUES" "$MISSION_CUES_TOPIC" &
PIDS+=("$!")
run_hz "CTRL" "$CONTROL_TOPIC" &
PIDS+=("$!")

# 백그라운드 hz 프로세스들이 살아있는 동안 대기.
wait
