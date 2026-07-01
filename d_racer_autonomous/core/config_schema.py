"""@file config_schema.py
@brief 모든 튜닝 파라미터의 타입 안전한 스키마 + YAML 로더.

@details
"하드코딩 최소화 / 모든 파라미터는 YAML에서 수정 가능" 원칙을 강제하는
단일 지점이다. YAML은 평문이라 오타·타입 오류를 잡지 못하므로,
dataclass로 스키마를 정의하고 from_dict()에서 검증/기본값 처리를 한다.

@par 설계 이유
- 컨트롤러 클래스는 dict가 아니라 타입이 있는 Config 객체를 받는다 →
  IDE 자동완성, 오타 즉시 발견, 단위 테스트에서 객체 직접 생성 용이.
- ROS2 노드에서도 동일 dataclass를 ROS 파라미터로부터 채워 재사용한다
  (시뮬↔실차 코드 일원화).
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Dict

import yaml


def _filter_known(cls, data: Dict[str, Any]) -> Dict[str, Any]:
    """@brief dataclass가 정의한 필드만 추려 알 수 없는 키를 무시한다."""
    known = {f.name for f in fields(cls)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"{cls.__name__}: unknown config keys {sorted(unknown)}")
    return {k: v for k, v in data.items() if k in known}


@dataclass
class VehicleConfig:
    """@brief 차량 물리/액추에이터 파라미터."""

    wheelbase: float = 0.26          ##< 휠베이스 L [m] (스케일카 기준 예시).
    max_steer_deg: float = 24.0      ##< 물리 조향각 한계 [deg].
    steer_trim: float = 0.0          ##< 직진 보정(중립 오프셋), 정규화 단위.
    speed_tau: float = 0.15          ##< 속도 1차 지연 시상수 [s] (시뮬용).

    @property
    def max_steer_rad(self) -> float:
        """@brief 조향각 한계를 라디안으로 반환."""
        import math
        return math.radians(self.max_steer_deg)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "VehicleConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class PurePursuitConfig:
    """@brief Pure Pursuit 조향 파라미터.

    @details Adaptive Lookahead: @f$ L_d = L_{min} + K_v v - K_c \\kappa @f$
    """

    use_adaptive_lookahead: bool = True  ##< False면 fixed_lookahead 사용(초기 테스트).
    fixed_lookahead: float = 0.6         ##< 고정 lookahead [m].
    ld_min: float = 0.3                  ##< Adaptive 최소 lookahead [m].
    ld_max: float = 1.2                  ##< Adaptive 최대 lookahead [m].
    kv: float = 0.4                      ##< 속도 이득 K_v [s].
    kc: float = 0.1                      ##< 곡률 이득 K_c [m^2].
    steering_gain: float = 1.0           ##< Pure Pursuit 출력 스케일 보정.
    steering_smoothing: float = 0.3      ##< 신규 명령 가중치 β: out = (1-β)·old + β·new.
    search_window: int = 60              ##< 전방 최근접 탐색 윈도우(점 개수). 0이면 전역 탐색.

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PurePursuitConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class SpeedConfig:
    """@brief 속도 결정(곡률 기반) + Speed PID 파라미터."""

    v_max: float = 0.8               ##< 직선 최대 속도 [정규화 또는 m/s].
    v_min: float = 0.3               ##< 급커브 최소 속도.
    curvature_gain: float = 0.5      ##< 곡률→감속 이득 (|κ|에 비례 감속).
    use_speed_pid: bool = False      ##< True면 목표속도→throttle을 PID로 산출.
    kp: float = 0.5                  ##< Speed PID 비례 이득.
    ki: float = 0.0                  ##< Speed PID 적분 이득.
    kd: float = 0.0                  ##< Speed PID 미분 이득.

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SpeedConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class BatteryConfig:
    """@brief 배터리 전압 보정 파라미터."""

    enabled: bool = False            ##< 배터리 토픽이 있을 때만 True.
    v_nominal: float = 7.4           ##< 기준(공칭) 전압 [V] (2S LiPo 예시).
    v_min: float = 6.4               ##< 보정 하한 전압(이하 클램프) [V].
    v_max: float = 8.4               ##< 보정 상한 전압(이상 클램프) [V].
    max_gain: float = 1.5            ##< 보정 배수 상한 (과보상 방지).
    # 선택적 룩업 테이블: [[voltage, gain], ...]. 비어 있으면 수식 보정 사용.
    lookup_table: list = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BatteryConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class SimConfig:
    """@brief 시뮬레이션 실행 파라미터."""

    dt: float = 0.02                 ##< 적분 시간 간격 [s] (50 Hz).
    max_time: float = 60.0           ##< 최대 시뮬 시간 [s].
    goal_tolerance: float = 0.15     ##< 경로 끝 도달 판정 거리 [m].
    path_name: str = "s_curve"       ##< 사용할 정적 경로 이름.
    battery_voltage: float = 7.4     ##< 시뮬 중 가정 배터리 전압 [V].

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SimConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class AppConfig:
    """@brief 전체 설정 묶음 (여러 YAML을 합쳐 구성)."""

    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    pure_pursuit: PurePursuitConfig = field(default_factory=PurePursuitConfig)
    speed: SpeedConfig = field(default_factory=SpeedConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    sim: SimConfig = field(default_factory=SimConfig)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AppConfig":
        data = data or {}
        return cls(
            vehicle=VehicleConfig.from_dict(data.get("vehicle", {})),
            pure_pursuit=PurePursuitConfig.from_dict(data.get("pure_pursuit", {})),
            speed=SpeedConfig.from_dict(data.get("speed", {})),
            battery=BatteryConfig.from_dict(data.get("battery", {})),
            sim=SimConfig.from_dict(data.get("sim", {})),
        )


def load_config(*yaml_paths: str) -> AppConfig:
    """@brief 하나 이상의 YAML 파일을 순서대로 병합해 AppConfig를 만든다.

    @param yaml_paths 읽을 YAML 경로들. 뒤쪽 파일이 앞쪽을 덮어쓴다(merge).
    @return 검증된 AppConfig.

    @note 최상위 키(vehicle/pure_pursuit/speed/battery/sim) 단위로 얕게
          병합한다. 같은 섹션의 일부 키만 override 하려면 해당 섹션 전체를
          한 파일에 두는 것을 권장한다.
    """
    merged: Dict[str, Any] = {}
    for path in yaml_paths:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        for section, values in data.items():
            merged.setdefault(section, {})
            if isinstance(values, dict):
                merged[section].update(values)
            else:
                merged[section] = values
    return AppConfig.from_dict(merged)
