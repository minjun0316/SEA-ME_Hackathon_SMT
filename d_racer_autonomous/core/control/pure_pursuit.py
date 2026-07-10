"""@file pure_pursuit.py
@brief Pure Pursuit 횡방향(조향) 컨트롤러.

@details
Pure Pursuit는 경로 위에서 lookahead 거리만큼 앞의 점(목표점)을 정해,
차량을 그 점으로 향하게 하는 원호를 따라가도록 조향각을 정하는 기하
기반 추종 알고리즘이다.

@par 알고리즘 단계 (사용자 정의 순서 그대로)
1. 현재 차량 위치 → 경로상 최근접 점
2. Lookahead Point 계산
3. alpha(목표점까지의 상대 방위각) 계산
4. Steering 계산

@par 핵심 수식
목표점을 차량 로컬 프레임으로 옮긴 좌표 @f$(x_l, y_l)@f$ 에 대해
@f[
    \\alpha = \\operatorname{atan2}(y_l, x_l), \\qquad
    \\delta = \\operatorname{atan2}\\!\\left(\\frac{2 L \\sin\\alpha}{L_d}\\right)
@f]
여기서 L은 휠베이스, @f$L_d@f$는 lookahead 거리.

@par Adaptive Lookahead
@f[ L_d = L_{min} + K_v\\,v - K_c\\,\\kappa @f]
속도가 빠르면 멀리(안정), 곡률이 크면 가까이(민첩) 본다.

@par 출력 후처리
- Steering Smoothing(저역통과): out = (1-β)·old + β·new
- Steering Saturation: [-max, +max]
조향 명령은 [-1, 1] 정규화 값으로 출력한다(δ/δ_max). 물리각/디버그
값도 함께 반환하여 로깅·분석에 사용한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config_schema import PurePursuitConfig, VehicleConfig
from ..geometry import Pose2D, to_local_frame
from ..path import Path


@dataclass
class PurePursuitResult:
    """@brief 한 스텝의 조향 산출 결과 + 디버그/로깅용 부가 정보.

    @var steering_norm  정규화 조향 명령 [-1, 1] (control_msgs/Control용).
    @var steering_rad   포화 적용 후 물리 조향각 [rad].
    @var lookahead      이번 스텝에 사용한 lookahead 거리 [m].
    @var target_point   추종한 목표점 [x, y] (입력 경로와 동일 프레임).
    @var alpha          목표점까지의 상대 방위각 [rad].
    @var curvature      최근접 점에서의 경로 곡률 [1/m].
    @var nearest_index  최근접 경로 점 인덱스.
    """

    steering_norm: float
    steering_rad: float
    lookahead: float
    target_point: np.ndarray
    alpha: float
    curvature: float
    nearest_index: int


class PurePursuitController:
    """@brief Pure Pursuit 조향 컨트롤러.

    @details 상태(이전 조향값)를 내부에 보관하여 smoothing을 적용한다.
    경로/판단 로직은 전혀 알지 못하고, 매 스텝 (pose, path, speed)만 받는다.
    """

    def __init__(self, vehicle: VehicleConfig, config: PurePursuitConfig):
        """@brief 컨트롤러 초기화.

        @param vehicle 차량 물리 파라미터(휠베이스, 조향 한계 등).
        @param config  Pure Pursuit 튜닝 파라미터.
        """
        self.vehicle = vehicle
        self.cfg = config
        self._prev_steer_norm = 0.0  ##< smoothing을 위한 직전 정규화 조향.
        self._progress_index = 0     ##< 단조 전진 진행 인덱스(윈도우 탐색 기준).
        self._kappa_ema = 0.0        ##< 곡률 스케줄 gain용 |κ| 저역통과 상태.

    def reset(self) -> None:
        """@brief 내부 상태(smoothing 이력, 진행 인덱스)를 초기화한다."""
        self._prev_steer_norm = 0.0
        self._progress_index = 0
        self._kappa_ema = 0.0

    # ------------------------------------------------------------------ #
    def _smooth_kappa(self, curvature: float) -> float:
        """@brief |κ|를 EMA 저역통과 후 데드밴드 → 유효 곡률 κ_eff(≥0) 반환.

        @param curvature 최근접 점 생(raw) 곡률 κ.
        @return κ_eff = max(0, EMA(|κ|) − deadband).

        @details lookahead·gain **공용** 유효 곡률. 스텝당 1회만 EMA를 갱신한다.
        데드밴드가 직선 지터(작은 |κ| 노이즈)를 0으로 눌러, 곡률을 소비하는
        어떤 스케줄도 '직선을 가짜 곡률로 읽어 발산'(07-09 adaptive 실패)하지
        않게 한다. 이 격리가 adaptive lookahead 재활성의 핵심.
        """
        alpha = self.cfg.curvature_smoothing
        self._kappa_ema = (1.0 - alpha) * self._kappa_ema + alpha * abs(curvature)
        return max(0.0, self._kappa_ema - self.cfg.curvature_deadband)

    def compute_gain(self, kappa_eff: float) -> float:
        """@brief 이번 스텝의 조향 gain을 계산한다(곡률 스케줄).

        @param kappa_eff `_smooth_kappa`가 낸 유효 곡률(≥0, 저역통과·데드밴드됨).
        @return 적용할 steering gain.

        @details use_curvature_gain=False면 상수 steering_gain을 반환한다.
        True면 gain = clip(steering_gain + kappa·κ_eff, steering_gain, max).
        직선(κ_eff≈0)은 baseline, 커브(κ_eff↑)는 gain을 가산해 조향력을 키운다.
        """
        if not self.cfg.use_curvature_gain:
            return self.cfg.steering_gain
        gain = self.cfg.steering_gain + self.cfg.steering_gain_kappa * kappa_eff
        return float(np.clip(gain, self.cfg.steering_gain, self.cfg.steering_gain_max))

    # ------------------------------------------------------------------ #
    def compute_lookahead(self, speed: float, kappa_eff: float) -> float:
        """@brief 이번 스텝의 lookahead 거리를 계산한다.

        @param speed     현재(또는 목표) 속도.
        @param kappa_eff `_smooth_kappa`가 낸 유효 곡률(≥0, 저역통과·데드밴드됨).
        @return lookahead 거리 [m].

        @details use_adaptive_lookahead=False면 고정값을 반환한다.
        Adaptive: Ld = ld_min + kv·speed − kc·κ_eff, [ld_min, ld_max]로 clip.
        속도항이 주 레버 — 직선(빠름)은 길게(안정), 커브(속도컨트롤러가 감속)는
        짧게(민첩). κ_eff는 데드밴드된 값이라 직선 노이즈로 Ld가 쪼그라들지 않는다.
        """
        if not self.cfg.use_adaptive_lookahead:
            return self.cfg.fixed_lookahead
        ld = self.cfg.ld_min + self.cfg.kv * speed - self.cfg.kc * kappa_eff
        return float(np.clip(ld, self.cfg.ld_min, self.cfg.ld_max))

    def compute(self, pose: Pose2D, path: Path, speed: float) -> PurePursuitResult:
        """@brief 한 스텝의 조향 명령을 계산한다.

        @param pose  현재 차량 자세(글로벌=시뮬, 또는 로컬 프레임=실차 원점).
        @param path  추종할 경로(Planning이 생성).
        @param speed 현재(또는 목표) 속도. Adaptive Lookahead에 사용.
        @return PurePursuitResult.
        """
        # (1) 최근접 점 + 곡률.
        #     진행 인덱스에서 전방 윈도우로만 탐색해 자기교차/되돌아오는
        #     경로에서 엉뚱한 분기로 점프하는 것을 막는다(단조 전진).
        window = self.cfg.search_window if self.cfg.search_window > 0 else None
        nearest = path.nearest_index(
            pose.x, pose.y, start=self._progress_index, window=window)
        self._progress_index = max(self._progress_index, nearest)
        curvature = path.curvature_at(nearest)  # nearest 물리 곡률(로깅/속도용).
        # adaptive/gain용 곡률 소스: preview>0이면 전방 구간 max|κ|(곡선을 미리 봄
        # → 진입 전 Ld 수축 → 턴인 지연). 0이면 nearest. 그 뒤 저역통과+데드밴드.
        kappa_src = (path.max_abs_curvature_ahead(nearest, self.cfg.curvature_preview)
                     if self.cfg.curvature_preview > 0.0 else curvature)
        kappa_eff = self._smooth_kappa(kappa_src)

        # (2) lookahead point
        lookahead = self.compute_lookahead(speed, kappa_eff)
        target = path.lookahead_point(nearest, lookahead)

        # (3) alpha: 목표점을 차량 로컬 프레임으로 옮긴 뒤 방위각
        local = to_local_frame(pose, target)
        alpha = math.atan2(local[1], local[0])

        # (4) steering: Pure Pursuit 기하식. 실제 목표점 거리를 분모로 사용해
        #     경로 끝(목표점이 lookahead보다 가까운 경우)에서도 안정적.
        ld_eff = max(1e-3, math.hypot(local[0], local[1]))
        delta = math.atan2(2.0 * self.vehicle.wheelbase * math.sin(alpha), ld_eff)
        delta *= self.compute_gain(kappa_eff)

        # 포화 (물리 조향 한계)
        max_rad = self.vehicle.max_steer_rad
        delta = float(np.clip(delta, -max_rad, max_rad))

        # 정규화 [-1, 1] + 직진 트림
        steer_norm = delta / max_rad + self.vehicle.steer_trim
        steer_norm = float(np.clip(steer_norm, -1.0, 1.0))

        # Smoothing (저역통과): out = (1-β)·old + β·new
        beta = self.cfg.steering_smoothing
        steer_norm = (1.0 - beta) * self._prev_steer_norm + beta * steer_norm
        self._prev_steer_norm = steer_norm

        return PurePursuitResult(
            steering_norm=steer_norm,
            steering_rad=delta,
            lookahead=lookahead,
            target_point=target,
            alpha=alpha,
            curvature=curvature,
            nearest_index=nearest,
        )
