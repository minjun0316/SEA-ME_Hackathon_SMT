"""@file feasibility.py
@brief 경로의 기구학적 실현가능성 검사 (차량 최소 회전반경 대비).

@details
Pure Pursuit가 아무리 잘 튜닝돼도, 경로가 차량의 물리적 조향 한계를
넘는 곡률을 요구하면 추종은 불가능하다. 컨트롤러를 탓하기 전에 "이
경로가 애초에 가능한가"를 먼저 확인하는 것이 자동차 SW 개발의 기본
점검 절차다.

@par 핵심 관계
자전거 모델의 최소 회전반경:
@f[ R_{min} = \\frac{L}{\\tan(\\delta_{max})}, \\qquad
    \\kappa_{max} = \\frac{1}{R_{min}} = \\frac{\\tan(\\delta_{max})}{L} @f]
경로의 어느 지점이라도 @f$ |\\kappa| > \\kappa_{max} @f$ 이면 그 구간은
물리적으로 추종 불가능하다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config_schema import VehicleConfig
from .path import Path


@dataclass
class FeasibilityReport:
    """@brief 실현가능성 검사 결과.

    @var feasible           모든 구간이 추종 가능한가.
    @var min_turn_radius    차량 최소 회전반경 [m].
    @var max_feasible_kappa 차량 최대 가능 곡률 [1/m].
    @var path_max_kappa     경로의 최대 곡률 [1/m].
    @var path_min_radius    경로가 요구하는 최소 반경 [m].
    @var margin             여유율 = max_feasible_kappa / path_max_kappa
                            (1 이상이면 가능, 클수록 여유).
    """

    feasible: bool
    min_turn_radius: float
    max_feasible_kappa: float
    path_max_kappa: float
    path_min_radius: float
    margin: float

    def summary(self) -> str:
        """@brief 사람이 읽을 한 줄 요약."""
        status = "FEASIBLE" if self.feasible else "INFEASIBLE"
        return (f"[{status}] vehicle R_min={self.min_turn_radius*100:.1f}cm "
                f"(κ_max={self.max_feasible_kappa:.2f}) | "
                f"path R_min={self.path_min_radius*100:.1f}cm "
                f"(κ_max={self.path_max_kappa:.2f}) | margin={self.margin:.2f}x")


def min_turn_radius(vehicle: VehicleConfig) -> float:
    """@brief 차량 최소 회전반경 [m] = L / tan(δ_max)."""
    return vehicle.wheelbase / math.tan(vehicle.max_steer_rad)


def check_path(path: Path, vehicle: VehicleConfig,
               edge_trim: int = 3, safety_margin: float = 1.0) -> FeasibilityReport:
    """@brief 경로가 차량으로 추종 가능한지 검사한다.

    @param path          검사할 경로.
    @param vehicle       차량 파라미터.
    @param edge_trim     곡률 계산이 불안정한 양 끝 점 개수(검사에서 제외).
    @param safety_margin 안전계수(>1이면 더 보수적). 요구 곡률에 곱해 비교.
    @return FeasibilityReport.
    """
    kappa_max_feasible = 1.0 / min_turn_radius(vehicle)
    k = np.abs(path.curvatures)
    if len(k) > 2 * edge_trim:
        k = k[edge_trim:-edge_trim]
    path_kappa = float(np.max(k)) if len(k) else 0.0
    path_radius = (1.0 / path_kappa) if path_kappa > 1e-9 else float("inf")
    margin = (kappa_max_feasible / path_kappa) if path_kappa > 1e-9 else float("inf")
    feasible = (path_kappa * safety_margin) <= kappa_max_feasible

    return FeasibilityReport(
        feasible=feasible,
        min_turn_radius=min_turn_radius(vehicle),
        max_feasible_kappa=kappa_max_feasible,
        path_max_kappa=path_kappa,
        path_min_radius=path_radius,
        margin=margin,
    )
