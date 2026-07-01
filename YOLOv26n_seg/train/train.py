"""
YOLO26n-seg 학습(파인튜닝) 스크립트  ── 노트북 전용

사전학습된 yolo26n-seg 모델을 우리 트랙 데이터셋으로 파인튜닝합니다.
학습이 끝나면 best.pt 를 ../models/best.pt 로 복사하고,
git add/commit/push 하면 D3G 가 git pull 로 받아서 추론에 씁니다.

사용법:
    pip install ultralytics
    python train.py
"""
from pathlib import Path
import shutil

from ultralytics import YOLO

# ─── 설정 (필요에 맞게 수정) ────────────────────────────────
BASE_MODEL = "yolo26n-seg.pt"          # 사전학습 모델 (실행 시 자동 다운로드)
HERE       = Path(__file__).resolve().parent          # .../YOLOv26n_seg/train
DATA_YAML  = HERE / "dataset.yaml"                     # 데이터셋 설정
RUNS_DIR   = HERE.parent / "runs"                      # 학습 결과 (gitignore됨)
MODELS_DIR = HERE.parent / "models"                    # 배포용 가중치 위치

EPOCHS   = 100
IMG_SIZE = 640
BATCH    = 16          # GPU 메모리에 맞게 조정 (부족하면 8, 4로 줄이기)
# ───────────────────────────────────────────────────────────


def main():
    model = YOLO(BASE_MODEL)           # 사전학습 가중치 로드 (COCO 학습됨)

    results = model.train(
        data=str(DATA_YAML),
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        batch=BATCH,
        project=str(RUNS_DIR),
        name="yolo26n_seg",
    )

    # 학습 결과 중 best.pt 를 배포 위치(../models/best.pt)로 복사
    best = Path(results.save_dir) / "weights" / "best.pt"
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    dest = MODELS_DIR / "best.pt"
    shutil.copy(best, dest)

    print(f"\n[완료] 배포용 가중치 저장 → {dest}")
    print("이제 다음을 실행하면 D3G 가 받을 수 있습니다:")
    print("    git add . && git commit -m 'YOLO 재학습 best.pt 갱신' && git push")


if __name__ == "__main__":
    main()
