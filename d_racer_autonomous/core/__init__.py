"""@package core
@brief ROS-free 자율주행 코어 라이브러리.

@details
이 패키지는 ROS2 / 하드웨어 의존성이 전혀 없는 순수 파이썬 코드만
포함한다. 시뮬레이션(Stage 1~3), 단위 테스트, 그리고 실차 ROS2 노드
래퍼에서 동일하게 재사용된다.

@par 구성 (인지/판단/제어 세 기둥 + 공유 기반 + 시뮬 도구)
공유 기반
- geometry      : 2D 좌표 변환 / 각도 유틸 (모든 파트가 의존).
- path          : 경로 표현과 기하 질의 (인지 → 제어의 데이터 계약).
- config_schema : 파라미터 스키마 + YAML 로더.
세 기둥
- perception/   : 인지의 ROS-free 순수 기하 (마스크/점 → 로컬 경로). YOLO 추론 제외.
- planning/     : 판단 순수 로직 (State Machine → DriveCommand 값).
- control/      : Pure Pursuit / Speed / Battery 컨트롤러.
시뮬/검증 도구
- path_factory  : Stage 1 정적 경로 생성기.
- vehicle_model : Stage 2 운동학 자전거 모델(시뮬 전용).
- feasibility   : 경로 실현가능성 검사(최소 회전반경 대비).

@note 노드 간 토픽/메시지 계약은 `docs/interfaces.md` 참조.
"""
