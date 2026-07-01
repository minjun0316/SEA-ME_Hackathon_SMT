"""
영상 → 이미지 프레임 추출 (노트북, 라벨링 데이터 준비용)

녹화 영상에서 일정 간격으로 프레임을 뽑아 이미지로 저장합니다.
뽑은 이미지를 라벨링 툴(예: Roboflow, CVAT, labelme)에 올려 세그 라벨을 만드세요.

사용법:
    pip install opencv-python
    python extract_frames.py --video track.mp4 --out ../datasets/images/raw --every 15
      --every 15 : 15프레임마다 1장 (영상 30fps면 0.5초에 1장)
"""
import argparse
from pathlib import Path

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True, help='입력 영상 경로')
    ap.add_argument('--out', required=True, help='이미지 저장 폴더')
    ap.add_argument('--every', type=int, default=15, help='N프레임마다 1장 저장')
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f'영상을 열 수 없습니다: {args.video}')

    idx = saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            cv2.imwrite(str(out / f'{Path(args.video).stem}_{idx:06d}.jpg'), frame)
            saved += 1
        idx += 1

    cap.release()
    print(f'[완료] {saved}장 저장 → {out}')


if __name__ == '__main__':
    main()
