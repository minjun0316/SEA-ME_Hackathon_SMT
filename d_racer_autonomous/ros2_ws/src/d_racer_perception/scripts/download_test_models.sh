#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODELS_DIR="${PKG_DIR}/models"
TEST_DATA_DIR="${PKG_DIR}/test_data"

DETECT_MODEL_NAME="${DETECT_MODEL_NAME:-yolo11n.pt}"
SEG_MODEL_NAME="${SEG_MODEL_NAME:-yolo11n-seg.pt}"
DETECT_MODEL_URL="${DETECT_MODEL_URL:-}"
SEG_MODEL_URL="${SEG_MODEL_URL:-}"
TEST_DATA_URL="${TEST_DATA_URL:-}"

mkdir -p "${MODELS_DIR}" "${TEST_DATA_DIR}"

download_url() {
  local url="$1"
  local dest="$2"
  if [[ -f "${dest}" ]]; then
    echo "Already exists: ${dest}"
    return
  fi
  echo "Downloading ${url}"
  if command -v curl >/dev/null 2>&1; then
    curl -L --fail -o "${dest}" "${url}"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "${dest}" "${url}"
  else
    echo "curl or wget is required for URL downloads" >&2
    exit 1
  fi
}

download_ultralytics_model() {
  local model_name="$1"
  local dest="$2"
  if [[ -f "${dest}" ]]; then
    echo "Already exists: ${dest}"
    return
  fi

  MODEL_NAME="${model_name}" DEST="${dest}" python3 - <<'PY'
import os
import shutil
from pathlib import Path

from ultralytics import YOLO

model_name = os.environ["MODEL_NAME"]
dest = Path(os.environ["DEST"]).expanduser().resolve()
model = YOLO(model_name)
source = Path(getattr(model, "ckpt_path", "") or model_name).expanduser()
if not source.exists():
    source = Path(model_name).expanduser()
if not source.exists():
    raise FileNotFoundError(f"Downloaded model not found: {model_name}")
dest.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(source, dest)
print(f"Saved {dest}")
PY
}

if [[ -n "${DETECT_MODEL_URL}" ]]; then
  download_url "${DETECT_MODEL_URL}" "${MODELS_DIR}/yolo_detect_test.pt"
else
  download_ultralytics_model "${DETECT_MODEL_NAME}" "${MODELS_DIR}/yolo_detect_test.pt"
fi

if [[ -n "${SEG_MODEL_URL}" ]]; then
  download_url "${SEG_MODEL_URL}" "${MODELS_DIR}/yolo_seg_test.pt"
else
  download_ultralytics_model "${SEG_MODEL_NAME}" "${MODELS_DIR}/yolo_seg_test.pt"
fi

if [[ -n "${TEST_DATA_URL}" ]]; then
  download_url "${TEST_DATA_URL}" "${TEST_DATA_DIR}/test_data_download"
fi

INSTALL_SHARE="$(cd "${PKG_DIR}/../../.." && pwd)/install/d_racer_perception/share/d_racer_perception"
if [[ -d "${INSTALL_SHARE}" ]]; then
  mkdir -p "${INSTALL_SHARE}/models" "${INSTALL_SHARE}/test_data"
  cp -f "${MODELS_DIR}/yolo_detect_test.pt" "${INSTALL_SHARE}/models/"
  cp -f "${MODELS_DIR}/yolo_seg_test.pt" "${INSTALL_SHARE}/models/"
  echo "Mirrored models to ${INSTALL_SHARE}/models"
fi

echo "Done."
echo "Detect model: ${MODELS_DIR}/yolo_detect_test.pt"
echo "Seg model:    ${MODELS_DIR}/yolo_seg_test.pt"
