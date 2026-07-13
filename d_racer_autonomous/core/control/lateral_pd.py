"""@file lateral_pd.py
@brief 근거리 차선 오차 기반 횡방향 PD(+heading) 컨트롤러.

@details
Pure Pursuit가 **먼 룩어헤드 점**(BEV 원거리=픽셀 적고 왜곡·캘리브 오차 큼)을
향해 조향해 노이즈를 증폭하는 반면, 이 컨트롤러는 인지가 계약으로 주는
**근거리 신호** 두 개만 쓴다:
- `lateral_offset` [m, +좌]: 차량중심 대비 차선중심 횡오차(화면 하단 ROI = 신뢰도↑).
- `heading_error` [rad]: 차선 접선과 차량 전방의 각도차.

@par 제어 법칙
@f[ \\delta = k_{ff}\\,\\kappa + k_h\\,\\psi + k_c\\,e + k_d\\,\\dot e @f]
- @f$k_{ff}\\kappa@f$ (곡률 피드포워드, 선택): 커브(곡률 κ)를 유지하는 데 필요한
  조향을 오차 없이 미리 얹는다. 순수 PD는 커브를 오차로만 버텨 정상상태 횡오차
  (커브서 중심 못잡음)가 남는데, 곡률은 측정 가능한 외란이라 피드포워드가 정석
  (적분과 달리 지연·와인드업 없음). κ는 lane_path 근거리에서 유도한 **부호 있는**
  값(좌커브 +, 우커브 −). lane_path 없으면 0으로 폴백(순수 PD로 안전 degrade).
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
    @var p_ff           곡률 피드포워드 기여분(정규화).
    @var k_cross_eff    이번 스텝 실제 적용된 k_cross(곡률 스케줄 후).
    @var p_cross        crosstrack P 기여분(정규화).
    @var p_heading      heading 기여분(정규화).
    @var d_cross        crosstrack D 기여분(정규화).
    @var lateral_offset 이번 스텝 입력 횡오차 [m](클램프 후).
    @var heading_error  이번 스텝 입력 heading 오차 [rad].
    @var curvature      이번 스텝 피드포워드에 쓴 부호곡률 κ_eff [1/m](EMA·데드밴드 후).
    """

    steering_norm: float
    p_ff: float
    k_cross_eff: float
    p_cross: float
    p_heading: float
    d_cross: float
    lateral_offset: float
    heading_error: float
    curvature: float


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
        self._curv_ema = 0.0         ##< 부호곡률 EMA 상태(피드포워드 저역통과).
        self._have_prev = False      ##< 첫 스텝 미분 방지.

    def reset(self) -> None:
        """@brief 내부 상태 초기화."""
        self._prev_steer_norm = 0.0
        self._prev_offset = 0.0
        self._deriv_ema = 0.0
        self._curv_ema = 0.0
        self._have_prev = False

    def compute(self, lateral_offset: float, heading_error: float,
                dt: float, curvature: float = 0.0) -> LateralPDResult:
        """@brief 한 스텝 조향 명령 계산.

        @param lateral_offset 횡오차 [m, +좌].
        @param heading_error  heading 오차 [rad].
        @param dt             스텝 시간 [s](미분용).
        @param curvature      lane_path 근거리 부호곡률 κ [1/m](좌커브 +, 우커브 −).
                              0=피드포워드 없음(lane_path 없거나 k_ff=0). 내부에서
                              EMA·데드밴드로 필터한다.
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

        # 곡률 피드포워드: EMA 저역통과(lane_path 노이즈) 후 데드밴드로 직선 격리.
        ac = self.cfg.curvature_smoothing
        self._curv_ema = (1.0 - ac) * self._curv_ema + ac * float(curvature)
        db = self.cfg.curvature_deadband
        # κ_eff = sign(κ_s)·max(0, |κ_s|−deadband): 직선 곡률노이즈는 0, 커브만 통과.
        kappa_eff = float(np.sign(self._curv_ema)
                          * max(0.0, abs(self._curv_ema) - db))

        # k_cross 곡률 스케줄: 직선(κ_eff=0)은 baseline k_cross(낮게→직선 offset
        # 노이즈 억제=꿀렁↓), 커브(κ_eff↑)만 가산해 중심복귀력 부스트. κ가 조향이
        # 아니라 lane_path 기하에서 오고 데드밴드로 직선을 격리해 PP식 양의 피드백
        # 발산이 없다. k_cross_kappa=0이면 스케줄 off(=상수 k_cross, 기존 동작).
        k_cross_eff = min(self.cfg.k_cross
                          + self.cfg.k_cross_kappa * abs(kappa_eff),
                          self.cfg.k_cross_max)

        # 피드포워드 + PD(+heading) — 정규화 조향 기여분.
        p_ff = self.cfg.k_ff * kappa_eff
        p_cross = k_cross_eff * e
        p_heading = self.cfg.k_heading * psi
        d_cross = self.cfg.k_deriv * self._deriv_ema
        raw = self.cfg.steering_sign * (p_ff + p_cross + p_heading + d_cross)
        raw = float(np.clip(raw, -1.0, 1.0))  # 제어노력 포화(트림 전).

        # 직진 트림 가산(규칙 #6: 제어단에서 실어 발행) + 최종 포화.
        steer = float(np.clip(raw + self.vehicle.steer_trim, -1.0, 1.0))

        # 출력 저역통과(smoothing): out = (1-β)·old + β·new.
        beta = self.cfg.steering_smoothing
        steer = (1.0 - beta) * self._prev_steer_norm + beta * steer
        self._prev_steer_norm = steer

        return LateralPDResult(
            steering_norm=steer,
            p_ff=p_ff,
            k_cross_eff=k_cross_eff,
            p_cross=p_cross,
            p_heading=p_heading,
            d_cross=d_cross,
            lateral_offset=e,
            heading_error=psi,
            curvature=kappa_eff,
        )
