"""
YOLO26n-seg 세그멘테이션 추론 노드 (D3G 실행용)

파이프라인:
    camera_node ──(camera/image/compressed)──▶ [이 노드] ──▶ 판단(planning) 노드
                                                  │
                                                  └─(디버그 영상)─▶ rqt 등으로 확인

- camera 패키지가 발행하는 jpeg CompressedImage 를 구독해서 디코딩 후 추론합니다.
- 가중치는 YOLOv26n_seg/models/best.pt (model_path 파라미터로 지정).
- ※ 판단쪽에 넘길 "출력 형태"는 아직 미정이라, 지금은 placeholder(Float32=0.0)만 발행합니다.
  실제 형태(차선 오프셋 / 세그 마스크 / 커스텀 msg)가 정해지면 아래 TODO 부분만 교체하면 됩니다.

실행:
    pip install ultralytics
    ros2 run perception_yolo yolo_seg_node --ros-args -p model_path:=/절대경로/best.pt
"""
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

from ultralytics import YOLO


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')

        # ── 파라미터 (대회 중 튜닝/환경별 조정) ──
        self.declare_parameter('image_topic', 'camera/image/compressed')
        self.declare_parameter('model_path', 'best.pt')   # ★ launch 나 -p 로 실제 경로 지정
        self.declare_parameter('conf', 0.25)              # 신뢰도 임계값 (튜닝 대상)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('publish_debug', True)     # 디버그 영상 발행 여부
        # ── 데이터 수집용 (대회장에서 차로 직접 프레임 뽑을 때) ──
        self.declare_parameter('save_dir', '')            # 비우면 저장 안 함. 경로 주면 프레임 저장
        self.declare_parameter('save_every_n', 10)        # N프레임마다 1장 저장 (전부 저장하면 너무 많음)

        image_topic = str(self.get_parameter('image_topic').value)
        model_path = str(self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.save_every_n = max(1, int(self.get_parameter('save_every_n').value))

        # 저장 폴더 준비 (raw=라벨링용 원본, annotated=추론결과 확인용)
        save_dir = str(self.get_parameter('save_dir').value)
        self.raw_dir = self.ann_dir = None
        if save_dir:
            self.raw_dir = Path(save_dir) / 'raw'
            self.ann_dir = Path(save_dir) / 'annotated'
            self.raw_dir.mkdir(parents=True, exist_ok=True)
            self.ann_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f'[YOLO Seg] 프레임 저장 켜짐 → {save_dir} (매 {self.save_every_n}장)')
        self.frame_idx = 0

        # ── 모델 로드 ──
        if not Path(model_path).exists():
            self.get_logger().warn(
                f'가중치를 찾을 수 없습니다: {model_path}\n'
                '노트북에서 학습한 best.pt 를 커밋/pull 하고, model_path 파라미터를 확인하세요.')
        self.model = YOLO(model_path)

        # ── 구독: 카메라 이미지 ──
        self.sub = self.create_subscription(
            CompressedImage, image_topic, self.on_image, 10)

        # ── 발행 ──
        # (1) 판단쪽에 넘길 결과 — 지금은 placeholder. TODO 에서 실제 타입으로 교체
        self.result_pub = self.create_publisher(Float32, 'perception/lane_offset', 10)
        # (2) 디버그 영상 (세그 오버레이) — rqt_image_view 등으로 눈으로 확인
        self.debug_pub = self.create_publisher(CompressedImage, 'perception/debug/compressed', 10)

        self.get_logger().info(
            f'[YOLO Seg] 구독={image_topic}  모델={model_path}  conf={self.conf}')

    def on_image(self, msg: CompressedImage):
        # jpeg → BGR 이미지
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('이미지 디코딩 실패')
            return

        # YOLO 추론
        result = self.model(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        masks = result.masks   # 세그 마스크 (감지 없으면 None)
        boxes = result.boxes   # 박스 + 클래스 + 신뢰도

        # ──────────────────────────────────────────────────────────────
        # TODO: result → 판단쪽에 넘길 값 계산 (아래 중 택1, 아직 미정)
        #
        #   (A) 차선 중심 오프셋(Float32): 차선 마스크의 중심선 x - 화면중심 x
        #   (B) 세그 마스크 이미지(sensor_msgs/Image or CompressedImage)
        #   (C) 커스텀 메시지: 클래스별 검출 목록 등 (별도 *_msgs 패키지 필요)
        #
        # YOLO class 정의(dataset.yaml 의 names)와 판단쪽 인터페이스가 확정되면 여기 채우기.
        # 지금은 자리표시로 0.0 발행.
        _ = (masks, boxes)
        out = Float32()
        out.data = 0.0
        self.result_pub.publish(out)
        # ──────────────────────────────────────────────────────────────

        # 추론결과 오버레이 이미지 (디버그 발행 + 저장에 재사용)
        annotated = None
        if self.publish_debug or self.raw_dir is not None:
            annotated = result.plot()               # 마스크/박스가 그려진 BGR 이미지

        # 디버그 영상: jpeg 로 발행 (rqt_image_view 등으로 실시간 확인)
        if self.publish_debug and annotated is not None:
            ok, enc = cv2.imencode('.jpg', annotated)
            if ok:
                dbg = CompressedImage()
                dbg.header = msg.header
                dbg.format = 'jpeg'
                dbg.data = enc.tobytes()
                self.debug_pub.publish(dbg)

        # 데이터 수집: N프레임마다 raw(원본)+annotated(추론결과)를 같은 번호로 저장
        # → annotated 보고 인식 안 되는 프레임 찾기 → 같은 번호 raw 를 라벨링에 사용
        self.frame_idx += 1
        if self.raw_dir is not None and self.frame_idx % self.save_every_n == 0:
            name = f'frame_{self.frame_idx:06d}.jpg'
            cv2.imwrite(str(self.raw_dir / name), frame)
            if annotated is not None:
                cv2.imwrite(str(self.ann_dir / name), annotated)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
