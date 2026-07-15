#!/usr/bin/env bash
# 팻말 좌/우 모델 정확도 측정.
#   알고 있는 방향의 팻말 하나를 카메라 앞에 고정하고 이 스크립트를 돌린 뒤,
#   집계된 LEFT/RIGHT 비율을 본다. 투표(voting)로 건질 수 있는지 판정하는 근거.
#
# 사용:  ./sign_tally.sh LEFT     (왼쪽 팻말을 들고 있을 때)
#        ./sign_tally.sh RIGHT
# 주행 런치가 떠 있어야 한다(mission_cues_node 로그를 읽음). Ctrl+C로 종료·집계.
# 주의: set -u 금지 — ROS setup.bash가 미정의 변수를 참조해 죽는다.
set -o pipefail

TRUTH="${1:-}"
if [[ "$TRUTH" != "LEFT" && "$TRUTH" != "RIGHT" ]]; then
  echo "사용법: $0 LEFT|RIGHT   (지금 들고 있는 팻말의 '진짜' 방향)"; exit 1
fi

source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"

echo "정답=$TRUTH 인 팻말을 카메라 앞에 두고 여러 거리에서 천천히 움직이세요."
echo "충분히(20~30초) 모은 뒤 Ctrl+C 로 집계합니다."
echo

TMP=$(mktemp)
L=0; R=0; BOTH=0; N=0; LSUM=0; RSUM=0

summarize() {
  echo; echo "════════ 집계 (정답: $TRUTH) ════════"
  if (( N == 0 )); then echo "  샘플 없음 — 팻말이 검출되지 않았습니다."; rm -f "$TMP"; exit 0; fi
  local correct wrong
  if [[ "$TRUTH" == "LEFT" ]]; then correct=$L; wrong=$R; else correct=$R; wrong=$L; fi
  local acc=$(( correct * 100 / N ))
  echo "  총 검출 프레임 : $N"
  echo "  LEFT 판정      : $L"
  echo "  RIGHT 판정     : $R"
  echo "  좌·우 동시검출 : $BOTH  (한 팻말에 둘 다 = 모델 혼동의 직접 증거)"
  echo "  ─────────────────────────────"
  echo "  정확도         : ${acc}%  (맞음 $correct / 틀림 $wrong)"
  echo
  if   (( acc >= 75 )); then
    echo "  ✅ 투표로 충분히 건질 수 있음 → 신뢰도 가중 다수결 적용하면 거의 확실해짐"
  elif (( acc >= 60 )); then
    echo "  ⚠️  약하지만 신호는 있음 → 긴 창의 신뢰도 가중 다수결이면 대체로 맞음"
  elif (( acc >= 40 )); then
    echo "  ❌ 사실상 동전던지기 → 어떤 필터로도 못 살림. 재학습 or OpenCV 방향판정 필요"
  else
    echo "  🔄 정답과 반대로 치우침! → 클래스 id가 뒤바뀐 것일 수 있음"
    echo "     mission_cues.yaml 의 tl_class_left/tl_class_right 를 서로 바꿔보세요"
    echo "     (현재: left=1, right=3)"
  fi
  rm -f "$TMP"; exit 0
}
trap summarize INT TERM

ros2 topic echo /rosout --field msg 2>/dev/null \
  | stdbuf -oL grep -o 'dir=[0-9] l=[0-9.]* r=[0-9.]* boxes=[0-9]*' > "$TMP" &
ECHO_PID=$!

tail -qF "$TMP" 2>/dev/null | while read -r line; do
  d=$(sed -n 's/.*dir=\([0-9]\).*/\1/p' <<<"$line")
  lc=$(sed -n 's/.*l=\([0-9.]*\).*/\1/p' <<<"$line")
  rc=$(sed -n 's/.* r=\([0-9.]*\).*/\1/p' <<<"$line")
  [[ "$d" == "0" ]] && continue                       # 미검출 프레임 제외
  N=$((N+1))
  [[ "$d" == "1" ]] && L=$((L+1))
  [[ "$d" == "2" ]] && R=$((R+1))
  # 좌·우 conf가 둘 다 0보다 크면 한 팻말에 양쪽 박스 = 모델 혼동
  if [[ "$lc" != "0.00" && "$rc" != "0.00" ]]; then BOTH=$((BOTH+1)); fi
  printf '\r\033[K  수집중: N=%d  LEFT=%d  RIGHT=%d  동시검출=%d' "$N" "$L" "$R" "$BOTH"
done
