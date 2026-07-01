"""@package core
@brief ROS-free 자율주행 코어 라이브러리.

@details
이 패키지는 ROS2 / 하드웨어 의존성이 전혀 없는 순수 파이썬 코드만
포함한다. 시뮬레이션(Stage 1~3), 단위 테스트, 그리고 실차 ROS2 노드
래퍼에서 동일하게 재사용된다.

@par 모듈 구성
- geometry      : 2D 좌표 변환 / 각도 유틸.
- path          : 경로 표현과 기하 질의(Planning → Control의 데이터 계약).
- path_factory  : Stage 1 정적 경로 생성기.
- vehicle_model : Stage 2 운동학 자전거 모델.
- config_schema : 파라미터 스키마 + YAML 로더.
- controllers/  : Pure Pursuit / Speed / Battery 컨트롤러.
"""
