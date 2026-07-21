#!/usr/bin/env bash
# 팻말 신호 체인만 한 화면에 모아 보는 진단 도구.
#   YOLO(dir) → [거리게이트] → mission_cues.sign_direction → decision.turn_bias
#              → lane_detect.force_side
# 주행 런치는 그대로 둔 채, 새 터미널에서 ./sign_watch.sh 실행.
# 주의: set -u 금지 — ROS setup.bash가 미정의 변수를 참조해 즉사한다(AMENT_TRACE_SETUP_FILES).
set -o pipefail

source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"

NAMES=(SIGN_H CUES BIAS FORCE)

# 각 단계의 최신값(끊긴 지점 판정용)
declare -A V=( [SIGN_H]="-" [CUES]="-" [BIAS]="-" [FORCE]="-" )

cleanup() { kill 0 2>/dev/null; printf '\n'; exit 0; }
trap cleanup INT TERM

TMP=$(mktemp -d)
trap 'cleanup; rm -rf "$TMP"' INT TERM

# 1) YOLO 원시 검출 + 거리게이트 결과 (mission_cues 로그)
ros2 topic echo /rosout --field msg 2>/dev/null \
  | stdbuf -oL grep -o 'dir=[0-9].*sign_h=[0-9.]* asp=[0-9.]*\( \[GATED[^]]*\]\)\?' \
  | stdbuf -oL sed 's/^/SIGN_H|/' > "$TMP/a" &

# 2) 인지가 발행한 팻말 방향 (0=없음 1=LEFT 2=RIGHT)
ros2 topic echo /perception/mission_cues --field sign_direction 2>/dev/null \
  | stdbuf -oL grep -E '^[0-9]+$' | stdbuf -oL sed 's/^/CUES|/' > "$TMP/b" &

# 3) 판단이 발행한 차선 지시 (0=NONE 1=LEFT 2=RIGHT)
ros2 topic echo /decision/lane_mode --field turn_bias 2>/dev/null \
  | stdbuf -oL grep -E '^[0-9]+$' | stdbuf -oL sed 's/^/BIAS|/' > "$TMP/c" &

# 4) 인지가 실제로 적용한 강제 방향 (lane_detect 로그)
ros2 topic echo /rosout --field msg 2>/dev/null \
  | stdbuf -oL grep -o 'force_side=[A-Za-z(]*' \
  | stdbuf -oL sed 's/^/FORCE|/' > "$TMP/d" &

dirname_of() { case "$1" in 0) echo "NONE";; 1) echo "LEFT";; 2) echo "RIGHT";; *) echo "$1";; esac; }

printf '팻말 신호 체인 감시 — 팻말을 차 앞에 보여주세요. Ctrl+C 종료\n\n'

tail -qF "$TMP"/a "$TMP"/b "$TMP"/c "$TMP"/d 2>/dev/null | while IFS='|' read -r key val; do
  case "$key" in
    SIGN_H) V[SIGN_H]="$val" ;;
    CUES)   V[CUES]="$(dirname_of "$val")" ;;
    BIAS)   V[BIAS]="$(dirname_of "$val")" ;;
    FORCE)  V[FORCE]="${val#force_side=}" ;;
  esac

  # 끊긴 지점 판정
  verdict=""
  case "${V[CUES]}" in
    NONE|-) verdict="① 인지에서 끊김 → YOLO 미검출 or 거리게이트(sign_min_box_h_frac)가 막음" ;;
    *)
      case "${V[BIAS]}" in
        NONE|-) verdict="② 판단에서 끊김 → use_decision / sign_lane_enable 확인" ;;
        *)
          case "${V[FORCE]}" in
            ""|-|NONE*) verdict="③ 인지가 지시를 못 받음 → rebuild 했는지 / lane_mode 구독 확인" ;;
            *)          verdict="✅ 체인 정상 — 신호는 다 감. 남은 건 offset 크기/방향" ;;
          esac ;;
      esac ;;
  esac

  printf '\r\033[K YOLO[%s]  cues=%-5s  bias=%-5s  force=%-6s | %s' \
    "${V[SIGN_H]}" "${V[CUES]}" "${V[BIAS]}" "${V[FORCE]}" "$verdict"
done
