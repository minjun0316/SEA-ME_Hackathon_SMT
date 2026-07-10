"""@file lateral_pd.py
@brief 근거리 차선 오차 기반 횡방향 PD(+heading) 컨트롤러.

@details
Pure Pursuit가 **먼 룩어헤드 점**(BEV 원거리=픽셀 적고 왜곡·캘리브 오차 큼)을
향해 조향해 노이즈를 증폭하는 반면, 이 컨트롤러는 인지가 계약으로 주는
**근거리 신호** 두 개만 쓴다:
- `lateral_offset` [m, +좌]: 차량중심 대비 차선중심 횡오차(화면 하단 ROI = 신뢰도↑).
- `heading_error` [rad]: 차선 접선과 차량 전방의 각도차.

@par 제어 법칙
@f[ \\delta = k_h\\,\\psi + k_c\\,e + k_d\\,\\dot e @f]
- @f$k_h\\psi@f$ (heading): 차선 방향에 정렬 → 위빙 억제. 각도라 BEV 스케일 캘리브
  오차에 둔감(강건). Stanley의 heading 항과 동일 역할 = 자연 감쇠.
- @f$k_c e@f$ (crosstrack P): 차선중심으로 복귀.
- @f$k_d\\dot e@f$ (crosstrack D, 선택): 근거리 신호라 노이즈 큼 → EMA로 필터한
  미분만 소량. 기본 0(heading 항이 주 감쇠).

출력은 Pure Pursuit와 동일한 후처리: 정규화 조향 [-1,1] + 직진 트림(vehicle.steer_trim,
규칙 #6) + 저역통과(smoothing) + 포화. 부호 규약: +오차/+heading → +조향(좌회전).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config_schema import LateralPDConfig, VehicleConfig


@dataclass
class LateralPDResult:
    """@brief 한 스텝 조향 결과 + 디버그.

    @var steering_norm  정규화 조향 명령 [-1,1] (control_msgs/Control용, 트림 포함).
    @var p_cross        crosstrack P 기여분(정규화).
    @var p_heading      heading 기여분(정규화).
    @var d_cross        crosstrack D 기여분(정규화).
    @var lateral_offset 이번 스텝 입력 횡오차 [m](클램프 후).
    @var heading_error  이번 스텝 입력 heading 오차 [rad].
    """

    steering_norm: float
    p_cross: float
    p_heading: float
    d_cross: float
    lateral_offset: float
    heading_error: float


class LateralPDController:
    """@brief 근거리 차선 오차 PD(+heading) 조향 컨트롤러(ROS-free).

    @details 상태(직전 조향·직전 오차·미분 EMA)를 내부에 보관한다. 매 스텝
    (lateral_offset, heading_error, dt)만 받고 경로/pose를 모른다.
    """

    def __init__(self, vehicle: VehicleConfig, config: LateralPDConfig):
        self.vehicle = vehicle
        self.cfg = config
        self._prev_steer_norm = 0.0  ##< 출력 smoothing 이력.
        self._prev_offset = 0.0      ##< 미분용 직전 lateral_offset.
        self._deriv_ema = 0.0        ##< 미분 EMA 상태(노이즈 저역통과).
        self._have_prev = False      ##< 첫 스텝 미분 방지.

    def reset(self) -> None:
        """@brief 내부 상태 초기화."""
        self._prev_steer_norm = 0.0
        self._prev_offset = 0.0
        self._deriv_ema = 0.0
        self._have_prev = False

    def compute(self, lateral_offset: float, heading_error: float,
                dt: float) -> LateralPDResult:
        """@brief 한 스텝 조향 명령 계산.

        @param lateral_offset 횡오차 [m, +좌].
        @param heading_error  heading 오차 [rad].
        @param dt             스텝 시간 [s](미분용).
        @return LateralPDResult.
        """
        # 이상치(오검출 스파이크) 클램프 — 근거리라도 한 프레임 튀면 과조향.
        e = float(np.clip(lateral_offset, -self.cfg.max_offset, self.cfg.max_offset))
        psi = float(heading_error)

        # crosstrack 미분(EMA 저역통과). 첫 스텝/비정상 dt는 0.
        if self._have_prev and dt > 1e-6:
            raw_deriv = (e - self._prev_offset) / dt
        else:
            raw_deriv = 0.0
        a = self.cfg.deriv_smoothing
        self._deriv_ema = (1.0 - a) * self._deriv_ema + a * raw_deriv
        self._prev_offset = e
        self._have_prev = True

        # PD(+heading) — 정규화 조향 기여분.
        p_cross = self.cfg.k_cross * e
        p_heading = self.cfg.k_heading * psi
        d_cross = self.cfg.k_deriv * self._deriv_ema
        raw = self.cfg.steering_sign * (p_cross + p_heading + d_cross)
        raw = float(np.clip(raw, -1.0, 1.0))  # 제어노력 포화(트림 전).

        # 직진 트림 가산(규칙 #6: 제어단에서 실어 발행) + 최종 포화.
        steer = float(np.clip(raw + self.vehicle.steer_trim, -1.0, 1.0))

        # 출력 저역통과(smoothing): out = (1-β)·old + β·new.
        beta = self.cfg.steering_smoothing
        steer = (1.0 - beta) * self._prev_steer_norm + beta * steer
        self._prev_steer_norm = steer

        return LateralPDResult(
            steering_norm=steer,
            p_cross=p_cross,
            p_heading=p_heading,
            d_cross=d_cross,
            lateral_offset=e,
            heading_error=psi,
        )
