"""@package core.control
@brief 제어기 모음 (조향/속도/배터리 보정).

@details 모든 제어기는 상태가 명시적이고(reset 제공), ROS 의존성이 없으며,
타입 있는 Config 객체로 파라미터를 받는다.
"""
from .battery_compensator import BatteryCompensator
from .pure_pursuit import PurePursuitController, PurePursuitResult
from .speed_controller import SpeedController, SpeedResult

__all__ = [
    "PurePursuitController",
    "PurePursuitResult",
    "SpeedController",
    "SpeedResult",
    "BatteryCompensator",
]
