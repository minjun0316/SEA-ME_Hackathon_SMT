"""@package core.planning
@brief 판단(Decision/Planning)의 **ROS-free 순수 로직**.

@details
인지가 준 경로/상태를 보고 "지금 어떻게 달릴지"를 정하는 상태 기계와
결정 로직을 담는다. ROS 의존성 없이 (입력 → DriveCommand 값) 함수/클래스로
구현해 단위 테스트가 가능하게 한다.

@par 여기에 들어갈 것 (예정)
- 주행 상태 기계(State Machine): DRIVE / SLOW / STOP / LOST 등 전이.
- 상황 인식 → 제어 배율 결정(speed_scale, lookahead_scale, steer_limit, go).
- 정지선/장애물 등 이벤트 처리 규칙.

@par 설계 원칙
- 판단은 제어기 파라미터를 **직접 세팅하지 않고** "배율/게이트"만 정한다
  (튜닝은 controller.yaml로 일원화). 자세한 계약은 `docs/interfaces.md` 의
  `/decision/drive_command`(racer_msgs/DriveCommand) 참조.
- 이 순수 로직을 얇은 ROS2 `decision_node` 가 감싸서 토픽으로 노출한다.
"""
from .decision import (
    DecisionMaker,
    DriveCommand,
    DriveState,
    LaneColor,
    LaneObservation,
    TurnHint,
)
from .mission import (
    MissionObservation,
    MissionPhase,
    MissionSequencer,
    TrafficLight,
)

__all__ = [
    # 아래층 반응형 판단
    "DecisionMaker",
    "DriveCommand",
    "DriveState",
    "LaneColor",
    "LaneObservation",
    "TurnHint",
    # 위층 미션 시퀀스
    "MissionObservation",
    "MissionPhase",
    "MissionSequencer",
    "TrafficLight",
]
