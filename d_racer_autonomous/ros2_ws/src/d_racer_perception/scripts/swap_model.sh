#!/usr/bin/env bash
# YOLO 모델 교체. 새로 학습한 .pt 하나만 주면 아카이브→복사→NCNN export→검증까지 한다.
#
#   ./scripts/swap_model.sh ~/yolo26n_new.pt
#   ./scripts/swap_model.sh ~/yolo26n_new.pt --dry-run    # 검사만, 안 바꿈
#
# 왜 스크립트인가 — 손으로 하면 매번 틀리는 지점이 셋 있다:
#   1) .pt만 복사하면 반영 안 된다. 런타임은 NCNN(best_ncnn_model/)을 읽는다.
#   2) export imgsz=320 필수. 기본 640으로 뽑으면 9.6fps → 1.35fps로 주저앉는다.
#   3) 클래스 id 순서가 바뀌면 에러 없이 조용히 오작동한다(빨간불을 팻말로 읽는 식).
#      mission_cues.yaml의 tl_class_* 와 대조하는 게 이 스크립트의 핵심이다.
#
# 파일명(best.pt / best_ncnn_model)은 고정이다. install→build→src 심링크가 파일명 단위로
# 걸려 있어서, 이름을 바꾸면 심링크가 끊기고 colcon build가 강제된다.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODELS="$PKG_DIR/models"
YAML="$PKG_DIR/config/mission_cues.yaml"
ARCHIVE="$HOME/model_archive"
IMGSZ=320

C_R=$'\e[31m'; C_G=$'\e[32m'; C_Y=$'\e[33m'; C_B=$'\e[1m'; C_0=$'\e[0m'
die() { echo "${C_R}[중단]${C_0} $*" >&2; exit 1; }
ok()  { echo "${C_G}  ok${C_0} $*"; }

NEW_PT=""; DRY=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) NEW_PT="$a" ;;
  esac
done
[ -n "$NEW_PT" ] || die "새 .pt 경로를 주세요.  사용법: $0 <new.pt> [--dry-run]"
[ -f "$NEW_PT" ] || die "파일 없음: $NEW_PT"
[ -f "$YAML" ]   || die "설정 없음: $YAML"

# ── 1. 새 모델 정체 파악 ──────────────────────────────────────────────
echo "${C_B}[1/5] 새 모델 검사${C_0}  $NEW_PT"
INFO=$(python3 - "$NEW_PT" <<'PY'
import sys, torch, os
ck = torch.load(sys.argv[1], map_location="cpu", weights_only=False)
m  = ck.get("model", None)
names = getattr(m, "yaml", {}).get("names", None) or getattr(m, "names", {}) or {}
ta = ck.get("train_args", {}) or {}
data = str(ta.get("data", "unknown"))
label = os.path.basename(os.path.dirname(data)) or "unknown"
print("NAMES=" + ",".join(f"{i}:{n}" for i, n in sorted(names.items())))
print("LABEL=" + label)
print("EPOCHS=" + str(ta.get("epochs", "?")))
print("DATE=" + str(ck.get("date", "?"))[:16])
PY
) || die "체크포인트를 읽을 수 없습니다(손상됐거나 .pt가 아님)."
eval "$(echo "$INFO" | sed 's/^\([A-Z]*\)=\(.*\)$/\1="\2"/')"
echo "     클래스 : $NAMES"
echo "     데이터셋: $LABEL   epochs=$EPOCHS   학습=$DATE"

# ── 2. 클래스 id ↔ mission_cues.yaml 대조 (가장 중요) ─────────────────
echo "${C_B}[2/5] 클래스 id 대조${C_0}  ($(basename "$YAML"))"
MISMATCH=0
for pair in green:tl_class_green red:tl_class_red left:tl_class_left right:tl_class_right; do
  cname="${pair%%:*}"; key="${pair##*:}"
  want=$(echo "$NAMES" | tr ',' '\n' | awk -F: -v n="$cname" '$2==n{print $1}')
  have=$(awk -v k="$key" '$1==k":"{print $2}' "$YAML" | head -1)
  if [ -z "$want" ]; then
    echo "  ${C_R}없음${C_0} 새 모델에 '$cname' 클래스가 없습니다 (yaml $key=$have)"; MISMATCH=1
  elif [ "$want" != "$have" ]; then
    echo "  ${C_R}불일치${C_0} $cname: 모델=$want  yaml $key=$have  ${C_Y}← 고쳐야 함${C_0}"; MISMATCH=1
  else
    ok "$cname = $want"
  fi
done
if [ "$MISMATCH" = 1 ]; then
  echo
  echo "${C_R}${C_B}클래스 id가 안 맞습니다.${C_0} 이대로 두면 에러 없이 오작동합니다."
  echo "$YAML 의 tl_class_* 를 위 '모델=' 값으로 고친 뒤, ${C_B}colcon build 까지${C_0} 하세요"
  echo "(yaml은 install이 복사본이라 리빌드해야 반영됩니다 — 모델 파일과 다릅니다)."
  [ "$DRY" = 1 ] || die "교체를 멈춥니다. yaml의 tl_class_* 를 고쳐서 맞춘 뒤 다시 실행하세요."
fi

if [ "$DRY" = 1 ]; then echo; echo "${C_Y}--dry-run: 아무것도 바꾸지 않았습니다.${C_0}"; exit 0; fi

# ── 3. 현재 라이브 모델 아카이브 (중복이면 생략) ──────────────────────
echo "${C_B}[3/5] 현재 모델 아카이브${C_0}"
if [ -f "$MODELS/best.pt" ]; then
  CUR_MD5=$(md5sum "$MODELS/best.pt" | cut -d' ' -f1)
  DUP=$(find "$ARCHIVE" -name best.pt -exec md5sum {} + 2>/dev/null | grep -c "^$CUR_MD5 " || true)
  if [ "${DUP:-0}" -gt 0 ]; then
    ok "이미 아카이브에 있음 — 생략"
  else
    CUR_LABEL=$(python3 - "$MODELS/best.pt" <<'PY'
import sys, torch, os
ck = torch.load(sys.argv[1], map_location="cpu", weights_only=False)
ta = ck.get("train_args", {}) or {}
d = os.path.basename(os.path.dirname(str(ta.get("data", "")))) or "unknown"
print(f"{str(ck.get('date','x'))[:10]}_{d}")
PY
)
    DST="$ARCHIVE/$CUR_LABEL"; mkdir -p "$DST"
    cp -a "$MODELS/best.pt" "$DST/best.pt"
    [ -d "$MODELS/best_ncnn_model" ] && cp -a "$MODELS/best_ncnn_model" "$DST/best_ncnn_model"
    grep -m1 "trained on" "$DST/best_ncnn_model/metadata.yaml" 2>/dev/null > "$DST/WHAT.txt" || true
    ok "보존: $DST"
  fi
else
  ok "기존 모델 없음 — 생략"
fi

# ── 4. 교체 + NCNN export ─────────────────────────────────────────────
echo "${C_B}[4/5] 교체 + NCNN export (imgsz=$IMGSZ)${C_0}"
cp -f "$NEW_PT" "$MODELS/best.pt"        # 덮어쓰기: 경로 유지 → 심링크 안 깨짐
rm -rf "$MODELS/best_ncnn_model" "$MODELS/best.onnx"
( cd "$MODELS" && python3 -c "
from ultralytics import YOLO
YOLO('best.pt').export(format='ncnn', imgsz=$IMGSZ)
" ) || die "NCNN export 실패."
[ -f "$MODELS/best_ncnn_model/model.ncnn.bin" ] || die "export는 됐는데 model.ncnn.bin이 없습니다."
ok "best_ncnn_model/ 생성"

# ── 5. 검증: export 결과 + 라이브 심링크가 실제로 새 모델을 가리키는가 ──
echo "${C_B}[5/5] 검증${C_0}"
MD="$MODELS/best_ncnn_model/metadata.yaml"
grep -q "^- $IMGSZ$" "$MD" && ok "imgsz=$IMGSZ" || echo "  ${C_R}경고${C_0} metadata의 imgsz가 $IMGSZ가 아닙니다 — fps가 급락합니다"
echo "     export된 클래스: $(grep -A5 '^names:' "$MD" | grep -E '^\s+[0-9]+:' | tr -d ' \n')"

SHARE=$(ros2 pkg prefix d_racer_perception 2>/dev/null || true)
if [ -n "$SHARE" ]; then
  LIVE="$SHARE/share/d_racer_perception/models"
  NEW_MD5=$(md5sum "$MODELS/best_ncnn_model/model.ncnn.bin" | cut -d' ' -f1)
  if [ -e "$LIVE/best_ncnn_model/model.ncnn.bin" ]; then
    LIVE_MD5=$(md5sum "$LIVE/best_ncnn_model/model.ncnn.bin" 2>/dev/null | cut -d' ' -f1 || echo dangling)
    if [ "$NEW_MD5" = "$LIVE_MD5" ]; then
      ok "라이브($SHARE)가 새 모델을 가리킴 — 리빌드 불필요"
    else
      echo "  ${C_R}경고${C_0} 라이브 install이 새 모델과 다릅니다 → ${C_B}colcon build 필요${C_0}"
    fi
  else
    echo "  ${C_R}경고${C_0} 라이브 심링크가 끊어졌습니다 → ${C_B}colcon build 필요${C_0}"
  fi
else
  echo "  ${C_Y}건너뜀${C_0} 워크스페이스 미소싱 셸 — 새 터미널에서 확인하세요"
fi

echo
echo "${C_G}${C_B}교체 완료.${C_0} $LABEL (epochs=$EPOCHS)"
echo "다음: 노드 재시작만 하면 됩니다 (racer-clean 후 racer-run). yaml을 건드렸다면 colcon build 먼저."
echo "되돌리려면: $ARCHIVE 에서 원하는 폴더의 best.pt 로 이 스크립트를 다시 실행."
