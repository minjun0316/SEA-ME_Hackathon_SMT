"""@package core.perception
@brief 인지(Perception)의 **ROS-free 순수 기하** 부분.

@details
"카메라 단독 인지" 결과를 제어가 쓸 수 있는 **로컬 프레임 경로(core.path.Path)**
로 바꾸는, ROS/무거운 ML 의존성이 없는 계산만 담는다.

@par 여기에 들어갈 것 (예정)
- 세그멘테이션 마스크/검출점 → 차선 중심선 포인트 추출.
- 이미지 평면 → 지면(bird's-eye) 좌표 변환(호모그래피).
- 중심선 곡선 피팅/리샘플 → core.path.Path 생성.

@par 여기에 들어가면 안 되는 것 (중요)
- **YOLO/torch/cv2 등 무거운 모델 추론은 넣지 않는다.** core는 numpy 수준의
  가볍고 테스트 가능한 순수 라이브러리를 유지한다. 실제 추론/이미지 구독은
  `ros2_ws/src/d_racer_perception`의 ROS 노드에 두고, 그 결과(마스크/점)만
  이 모듈로 넘겨 기하 변환을 수행한다.

@par 출력 계약
`docs/interfaces.md` 의 `/perception/lane_path`(base_link 로컬 프레임)를 만족하는
core.path.Path 를 산출한다.
"""
