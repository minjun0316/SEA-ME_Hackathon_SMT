"""
YOLO26n-seg 추론 스크립트  ── D3G 실행용

학습해서 커밋한 ../models/best.pt 로 이미지를 세그멘테이션합니다.
실제 주행에서는 카메라 프레임을 넣고, 결과 마스크/박스를 제어 로직에 연결하세요.

사용법:
    pip install ultralytics
    python predict.py --source 0          # 0 = 웹캠
    python predict.py --source test.mp4   # 영상 파일
"""
import argparse
from pathlib import Path

from ultralytics import YOLO

MODEL_PATH = Path(__file__).resolve().parent.parent / "models" / "best.pt"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0", help="0=웹캠, 또는 이미지/영상 경로")
    parser.add_argument("--conf", type=float, default=0.25, help="신뢰도 임계값 (튜닝 대상)")
    parser.add_argument("--imgsz", type=int, default=640, help="입력 해상도")
    args = parser.parse_args()

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"가중치가 없습니다: {MODEL_PATH}\n"
            "노트북에서 train.py 로 학습 후 models/best.pt 를 커밋·push 하고,\n"
            "D3G 에서 git pull 로 받으세요."
        )

    model = YOLO(str(MODEL_PATH))      # 우리 학습 가중치 로드

    # stream=True: 프레임 단위로 실시간 처리 (주행에 적합, 메모리 효율적)
    for result in model(args.source, conf=args.conf, imgsz=args.imgsz, stream=True):
        masks = result.masks           # 세그 마스크 (감지 없으면 None)
        boxes = result.boxes           # 바운딩 박스 + 클래스 + 신뢰도

        # TODO: masks/boxes 로 차선·장애물 판단 → 조향/속도 제어에 연결
        #   예) 차선 마스크의 중심선을 계산해 목표 조향각 산출
        #   예) 장애물 박스가 특정 영역에 들어오면 감속/정지
        _ = (masks, boxes)             # (제어 로직 연결 전까지 자리표시)


if __name__ == "__main__":
    main()
