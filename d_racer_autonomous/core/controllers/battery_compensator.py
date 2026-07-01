"""@file battery_compensator.py
@brief 배터리 전압 기반 throttle 보정.

@details
같은 throttle 명령이라도 배터리 전압이 낮아지면 모터 출력(실제 속도)이
줄어든다. 주행 중 전압 강하로 속도 프로파일이 흔들리는 것을 막기 위해,
공칭 전압 대비 현재 전압 비율로 throttle을 보정한다.

@par 보정 방식 (둘 중 택1, 설정으로 전환)
1. 수식 보정:
   @f[ throttle' = throttle \\times \\frac{V_{nominal}}{V_{battery}} @f]
2. 룩업 테이블: [전압, 보정배수] 표를 선형 보간.

과보상으로 인한 폭주를 막기 위해 보정 배수에 상한(max_gain)을 둔다.

@par 설계 이유
배터리 보정은 "제어"가 아니라 "액추에이터 출력 정규화"에 해당하므로,
Pure Pursuit / Speed Controller와 분리된 마지막 단(Motor Driver 직전)에
둔다 (책임 분리, 끄고 켜기 쉬움).
"""
from __future__ import annotations

import numpy as np

from ..config_schema import BatteryConfig


class BatteryCompensator:
    """@brief 전압 보정기 (수식 또는 룩업 테이블)."""

    def __init__(self, config: BatteryConfig):
        """@param config 배터리 보정 파라미터."""
        self.cfg = config
        self._lut_v = None
        self._lut_g = None
        if config.lookup_table:
            table = np.asarray(config.lookup_table, dtype=float)
            order = np.argsort(table[:, 0])
            self._lut_v = table[order, 0]
            self._lut_g = table[order, 1]

    def gain(self, voltage: float) -> float:
        """@brief 현재 전압에 대한 보정 배수를 반환한다.

        @param voltage 측정 배터리 전압 [V].
        @return 보정 배수(>0). enabled=False면 항상 1.0.
        """
        if not self.cfg.enabled:
            return 1.0

        v = float(np.clip(voltage, self.cfg.v_min, self.cfg.v_max))
        if self._lut_v is not None:
            g = float(np.interp(v, self._lut_v, self._lut_g))
        else:
            g = self.cfg.v_nominal / max(v, 1e-3)
        return float(np.clip(g, 0.0, self.cfg.max_gain))

    def compensate(self, throttle: float, voltage: float) -> float:
        """@brief throttle에 전압 보정을 적용한다.

        @param throttle 원래 throttle 명령.
        @param voltage  측정 배터리 전압 [V].
        @return 보정된 throttle (부호 보존, 크기만 보정·클램프).
        """
        compensated = throttle * self.gain(voltage)
        return float(np.clip(compensated, -1.0, 1.0))
