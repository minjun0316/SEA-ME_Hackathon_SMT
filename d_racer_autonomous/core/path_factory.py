"""@file path_factory.py
@brief Stage 1: 정적(Static) 테스트 경로 생성기.

@details
제어기를 Perception 없이 검증하기 위한 기준 경로들을 생성한다.
모든 함수는 (N, 2) 점 배열을 만든 뒤 Path 객체로 감싸 반환한다.

@par 설계 이유
대회 미션(직선, S자, 급 S자, 로터리 등)을 수학적으로 정의된
재현 가능한 경로로 먼저 만들어 두면, 컨트롤러의 정량 평가
(cross-track error 등)가 가능하고 튜닝 결과를 비교할 수 있다.

@par 확장 가능성
실제 트랙 측정값(웨이포인트 CSV)을 from_waypoints()로 불러오면
동일한 Path 인터페이스로 그대로 사용할 수 있다.
"""
from __future__ import annotations

import numpy as np

from .path import Path


def straight(length: float = 5.0, n: int = 200, spacing: float = 0.05) -> Path:
    """@brief 직선 경로 (+X 방향)."""
    x = np.linspace(0.0, length, n)
    y = np.zeros_like(x)
    return Path(np.column_stack([x, y]), resample_spacing=spacing)


def circle(radius: float = 2.0, revolutions: float = 1.0,
           n: int = 400, spacing: float = 0.05) -> Path:
    """@brief 원형 경로.

    @param radius     반지름 [m]. 곡률 κ = 1/radius 로 일정하다.
    @param revolutions 회전 수.
    """
    theta = np.linspace(0.0, 2.0 * np.pi * revolutions, n)
    x = radius * np.sin(theta)
    y = radius * (1.0 - np.cos(theta))  # 원점에서 +X 접선으로 출발
    return Path(np.column_stack([x, y]), resample_spacing=spacing)


def s_curve(length: float = 10.0, amplitude: float = 0.4,
            wavelength: float = 5.0, n: int = 400, spacing: float = 0.05) -> Path:
    """@brief 완만한 S자(사인파) 경로.

    @param amplitude  횡방향 진폭 [m].
    @param wavelength  한 주기 길이 [m]. 짧을수록 곡률이 커진다.

    @note 사인파 최대 곡률 ≈ amplitude·(2π/wavelength)².
          기본값은 max|κ|≈0.63 1/m (반경 ~1.6m)로 일반 스케일카에 여유 있게
          추종 가능하다.
    """
    x = np.linspace(0.0, length, n)
    y = amplitude * np.sin(2.0 * np.pi * x / wavelength)
    return Path(np.column_stack([x, y]), resample_spacing=spacing)


def sharp_s_curve(length: float = 9.0, amplitude: float = 0.6,
                  wavelength: float = 4.0, n: int = 400,
                  spacing: float = 0.05) -> Path:
    """@brief 급 S자 경로 (높은 곡률, 조향 한계 근처 시험용).

    @note 기본값 max|κ|≈1.48 1/m (반경 ~0.68m). 휠베이스 0.26m / 조향 24°
          차량의 최소 회전반경(~0.58m) 바로 안쪽으로, 튜닝 난이도가 높은
          '실현가능한 스트레스 케이스'다. 차량 사양을 바꾸면
          core.feasibility.check_path 로 가능 여부를 먼저 확인할 것.
    """
    return s_curve(length=length, amplitude=amplitude,
                   wavelength=wavelength, n=n, spacing=spacing)


def figure_eight(radius: float = 3.5, n: int = 800, spacing: float = 0.05) -> Path:
    """@brief 8자(렘니스케이트) 경로. 좌/우 곡률이 번갈아 나타난다.

    @details 베르누이 렘니스케이트를 변형해 부드러운 8자를 만든다.
    @note 기본 radius=3.5는 max|κ|≈1.35 1/m로 기본 차량이 추종 가능하다.
          radius를 줄이면 곡률이 커져 실현 불가능해질 수 있다.
    """
    t = np.linspace(0.0, 2.0 * np.pi, n)
    scale = radius
    x = scale * np.sin(t)
    y = scale * np.sin(t) * np.cos(t)
    return Path(np.column_stack([x, y]), resample_spacing=spacing)


def rotary(approach: float = 2.0, radius: float = 1.5, exit_len: float = 2.0,
           arc_fraction: float = 0.75, n: int = 600, spacing: float = 0.05) -> Path:
    """@brief 회전교차로(로터리) 모사 경로: 직선 진입 → 원호 → 직선 진출.

    @param approach     진입 직선 길이 [m].
    @param radius       로터리 반지름 [m].
    @param exit_len     진출 직선 길이 [m].
    @param arc_fraction 원호가 차지하는 비율(1.0 = 한 바퀴).
    """
    # 1) 진입 직선 (+X)
    x1 = np.linspace(0.0, approach, n // 4)
    y1 = np.zeros_like(x1)

    # 2) 원호: 진입 끝점에서 접선이 +X가 되도록 원의 좌측 하단에서 시작
    cx, cy = approach, radius
    theta = np.linspace(-np.pi / 2.0,
                        -np.pi / 2.0 + 2.0 * np.pi * arc_fraction,
                        n // 2)
    x2 = cx + radius * np.cos(theta)
    y2 = cy + radius * np.sin(theta)

    # 3) 진출 직선: 원호 끝점의 접선 방향으로 이어 붙인다
    end_theta = theta[-1]
    tx = -np.sin(end_theta)
    ty = np.cos(end_theta)
    s = np.linspace(0.0, exit_len, n // 4)
    x3 = x2[-1] + tx * s
    y3 = y2[-1] + ty * s

    pts = np.column_stack([np.concatenate([x1, x2, x3]),
                           np.concatenate([y1, y2, y3])])
    return Path(pts, resample_spacing=spacing)


def from_waypoints(waypoints: np.ndarray, spacing: float = 0.05) -> Path:
    """@brief 외부 웨이포인트(예: 실측 트랙 CSV)로부터 경로 생성.

    @param waypoints (N, 2) [x, y] 배열.
    """
    return Path(np.asarray(waypoints, dtype=float), resample_spacing=spacing)


## @brief 이름으로 경로를 생성하기 위한 레지스트리 (run_sim 등에서 사용).
PATH_LIBRARY = {
    "straight": straight,
    "circle": circle,
    "s_curve": s_curve,
    "sharp_s": sharp_s_curve,
    "figure_eight": figure_eight,
    "rotary": rotary,
}


def make_path(name: str, **kwargs) -> Path:
    """@brief 이름으로 정적 경로를 생성한다.

    @param name PATH_LIBRARY 의 키 중 하나.
    @param kwargs 해당 생성 함수에 전달할 파라미터.
    @throws KeyError 알 수 없는 경로 이름.
    """
    if name not in PATH_LIBRARY:
        raise KeyError(
            f"unknown path '{name}'. available: {sorted(PATH_LIBRARY)}")
    return PATH_LIBRARY[name](**kwargs)
