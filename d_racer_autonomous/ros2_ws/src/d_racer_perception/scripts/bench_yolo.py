#!/usr/bin/env python3
"""D3-G 보드에서 YOLO detection 모델 latency 실측.

사용법:
  python3 scripts/bench_yolo.py                                         # NCNN, 320
  python3 scripts/bench_yolo.py --imgsz 416                             # 해상도 변경
  python3 scripts/bench_yolo.py --model models/best.pt                  # torch 원본과 비교
  python3 scripts/bench_yolo.py --threads 2                             # torch 코어 제한
"""
import argparse
import statistics
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/best_ncnn_model")
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--threads", type=int, default=0, help="0=torch 기본(전체 코어)")
    args = ap.parse_args()

    import torch
    if args.threads > 0:
        torch.set_num_threads(args.threads)

    from ultralytics import YOLO

    model = YOLO(args.model)
    # 640x640 랜덤 프레임 (실제 카메라 프레임 크기와 무관하게 imgsz로 리사이즈됨)
    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    print(f"model={args.model} imgsz={args.imgsz} torch_threads={torch.get_num_threads()}")

    for _ in range(args.warmup):
        model(frame, imgsz=args.imgsz, verbose=False)

    times = []
    for _ in range(args.runs):
        t0 = time.perf_counter()
        model(frame, imgsz=args.imgsz, verbose=False)
        times.append((time.perf_counter() - t0) * 1000.0)

    times.sort()
    mean = statistics.mean(times)
    p50 = statistics.median(times)
    p90 = times[int(len(times) * 0.9)]
    print(f"latency_ms  mean={mean:.1f}  p50={p50:.1f}  p90={p90:.1f}  "
          f"min={times[0]:.1f}  max={times[-1]:.1f}")
    print(f"FPS         mean={1000.0/mean:.2f}  p50={1000.0/p50:.2f}")


if __name__ == "__main__":
    main()
