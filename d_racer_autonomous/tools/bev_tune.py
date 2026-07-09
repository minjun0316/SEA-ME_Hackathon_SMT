#!/usr/bin/env python3
"""BEV 4점(bev_top_x/y) 시각 튜너 — headless 보드용.

lane_detect._to_bev()와 '동일한' src/dst 공식으로, 여러 후보값의 BEV 결과를
한 장의 PNG로 타일링해 저장한다. VS Code에서 열어 차선이 가장 평행/수직으로
펴진 칸의 (x,y)를 골라 config/lane.yaml의 bev_top_x/bev_top_y에 넣으면 된다.

프레임 얻기:
  - 저장된 이미지:   python3 -m tools.bev_tune --image frame.png
  - ROS 토픽에서 1장: python3 -m tools.bev_tune --from-topic          # lane.yaml image_topic
                     python3 -m tools.bev_tune --from-topic /camera/image/compressed

출력: --out (기본 tools/bev_tune_out.png). --fine x y 로 특정값 주변 미세탐색.
"""
import argparse
import numpy as np
import cv2


def bev_matrix(w, h, top_x, top_y):
    """lane_detect._to_bev()와 동일한 src/dst."""
    src = np.float32([
        [w * top_x, h * top_y],
        [w * (1.0 - top_x), h * top_y],
        [w, h],
        [0, h],
    ])
    dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    return cv2.getPerspectiveTransform(src, dst)


def draw_src(img, top_x, top_y):
    """원본에 src 사다리꼴을 겹쳐 그린다(참고용)."""
    h, w = img.shape[:2]
    pts = np.int32([
        [w * top_x, h * top_y],
        [w * (1.0 - top_x), h * top_y],
        [w, h],
        [0, h],
    ])
    out = img.copy()
    cv2.polylines(out, [pts], True, (0, 255, 0), 2)
    return out


def bev_cell(frame, top_x, top_y, cell_w=320):
    """한 후보값의 BEV 결과 + 수직 격자선 + 라벨."""
    h, w = frame.shape[:2]
    M = bev_matrix(w, h, top_x, top_y)
    bev = cv2.warpPerspective(frame, M, (w, h))
    # 평행/수직 판단용 세로 격자선(중앙 노랑, 나머지 회색)
    for frac in (0.25, 0.5, 0.75):
        x = int(w * frac)
        color = (0, 255, 255) if abs(frac - 0.5) < 1e-6 else (120, 120, 120)
        cv2.line(bev, (x, 0), (x, h), color, 1)
    scale = cell_w / w
    bev = cv2.resize(bev, (cell_w, int(h * scale)))
    label = f"x={top_x:.2f} y={top_y:.2f}"
    cv2.rectangle(bev, (0, 0), (len(label) * 11 + 8, 22), (0, 0, 0), -1)
    cv2.putText(bev, label, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return bev


def montage(frame, xs, ys, out_path, cell_w=320):
    rows = []
    for ty in ys:
        row = [bev_cell(frame, tx, ty, cell_w) for tx in xs]
        rows.append(np.hstack(row))
    grid = np.vstack(rows)
    # 좌상단에 원본+대표 사다리꼴(중앙값) 참고 썸네일
    ref = draw_src(frame, xs[len(xs) // 2], ys[len(ys) // 2])
    ref = cv2.resize(ref, (cell_w, int(frame.shape[0] * cell_w / frame.shape[1])))
    cv2.imwrite(out_path, grid)
    cv2.imwrite(out_path.replace('.png', '_ref.png'), ref)
    print(f"[bev_tune] saved: {out_path}  (grid {len(ys)}x{len(xs)})")
    print(f"[bev_tune] ref (원본+사다리꼴): {out_path.replace('.png', '_ref.png')}")


def grab_from_topic(topic):
    """ROS2 compressed image 토픽에서 프레임 1장."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    holder = {}

    class Grab(Node):
        def __init__(self):
            super().__init__('bev_tune_grab')
            self.create_subscription(CompressedImage, topic, self._cb, 1)

        def _cb(self, msg):
            arr = np.frombuffer(msg.data, np.uint8)
            holder['img'] = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    rclpy.init()
    node = Grab()
    print(f"[bev_tune] {topic} 구독, 1프레임 대기...")
    while 'img' not in holder:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    return holder['img']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image', help='저장된 프레임 경로')
    ap.add_argument('--from-topic', nargs='?', const='camera/image/compressed',
                    help='ROS compressed 토픽에서 1장 (기본 lane.yaml image_topic)')
    ap.add_argument('--out', default='tools/bev_tune_out.png')
    ap.add_argument('--save-frame', help='잡은 프레임을 이 경로에 저장')
    ap.add_argument('--fine', nargs=2, type=float, metavar=('X', 'Y'),
                    help='이 값 주변 미세탐색(±0.05, ±0.05)')
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"이미지 못 읽음: {args.image}")
    elif args.from_topic is not None:
        frame = grab_from_topic(args.from_topic)
    else:
        raise SystemExit("--image 또는 --from-topic 필요")

    if args.save_frame:
        cv2.imwrite(args.save_frame, frame)
        print(f"[bev_tune] frame saved: {args.save_frame}  size={frame.shape[1]}x{frame.shape[0]}")

    if args.fine:
        cx, cy = args.fine
        xs = [round(cx + d, 3) for d in (-0.05, -0.025, 0.0, 0.025, 0.05)]
        ys = [round(cy + d, 3) for d in (-0.05, -0.025, 0.0, 0.025, 0.05)]
        xs = [x for x in xs if 0.02 < x < 0.48]
        ys = [y for y in ys if 0.15 < y < 0.85]
    else:  # coarse
        xs = [0.10, 0.20, 0.30, 0.40]
        ys = [0.30, 0.40, 0.50, 0.60, 0.70]

    montage(frame, xs, ys, args.out)


if __name__ == '__main__':
    main()
