"""@file path.py
@brief 경로(Path) 표현과 경로 위 기하 질의(nearest / lookahead / curvature).

@details
Path는 "Planning이 만들어 Controller에게 넘기는 유일한 데이터 계약"이다.
Controller는 Perception이나 Planning의 내부 동작을 절대 알지 못하고,
오직 이 Path 객체만 입력으로 받는다 (인지/판단/제어 분리 원칙).

@par 설계 이유
- 균일 간격 리샘플링: lookahead 거리 누적 검색과 곡률 계산을 안정화한다.
- 곡률 사전 계산: SpeedController(곡률 기반 속도)와 Adaptive Lookahead가
  매 스텝 동일 값을 재사용하도록 한다.
"""
from __future__ import annotations

import numpy as np

from .geometry import normalize_angle


class Path:
    """@brief 2D 경로. 점 배열과 그 위의 기하 질의를 제공한다.

    @details 내부적으로 점을 균일 간격으로 리샘플링하여 보관한다.
    """

    def __init__(self, points: np.ndarray, resample_spacing: float = 0.05):
        """@brief 점 배열로부터 경로를 생성한다.

        @param points           (N, 2) 형태의 [x, y] 점 배열.
        @param resample_spacing 리샘플 간격 [m]. <=0 이면 리샘플하지 않는다.
        @throws ValueError 점이 2개 미만이면 경로를 만들 수 없다.
        """
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2:
            raise ValueError("points must be an (N, 2) array")
        if pts.shape[0] < 2:
            raise ValueError("a path needs at least 2 points")

        if resample_spacing and resample_spacing > 0.0:
            pts = self._resample(pts, resample_spacing)

        self._points = pts
        self._cum_dist = self._cumulative_distance(pts)
        self._curvature = self._compute_curvature(pts)

    # ------------------------------------------------------------------ #
    # Properties
    # ------------------------------------------------------------------ #
    @property
    def points(self) -> np.ndarray:
        """@brief (N, 2) 경로 점 배열."""
        return self._points

    @property
    def length(self) -> float:
        """@brief 경로 전체 길이 [m]."""
        return float(self._cum_dist[-1])

    @property
    def curvatures(self) -> np.ndarray:
        """@brief 각 점에서의 곡률 κ [1/m] 배열 (N,)."""
        return self._curvature

    def __len__(self) -> int:
        return self._points.shape[0]

    # ------------------------------------------------------------------ #
    # Queries
    # ------------------------------------------------------------------ #
    def nearest_index(self, x: float, y: float,
                      start: int = 0, window: int | None = None) -> int:
        """@brief 주어진 점에 가장 가까운 경로 점의 인덱스.

        @param x, y   질의 점 (보통 차량의 후륜축 중심).
        @param start  탐색 시작 인덱스. 진행 인덱스를 넣으면 단조 전진 탐색.
        @param window 탐색 윈도우 크기(점 개수). None이면 전역 탐색.
        @return 가장 가까운 점의 인덱스.

        @details
        window를 주면 [start, start+window] 구간에서만 최근접을 찾는다.
        이는 두 가지 문제를 동시에 막는다.
        1. 자기교차 경로(8자 등)에서 멀리 떨어진 분기로 점프.
        2. 닫힌/되돌아오는 경로에서 시작·끝점이 같아 조기 종료.
        전역 탐색(window=None)은 초기 정렬이나 큰 위치 점프 복구에 쓴다.
        """
        if window is None:
            d2 = (self._points[:, 0] - x) ** 2 + (self._points[:, 1] - y) ** 2
            return int(np.argmin(d2))

        lo = int(np.clip(start, 0, len(self) - 1))
        hi = int(np.clip(start + window, lo + 1, len(self)))
        seg = self._points[lo:hi]
        d2 = (seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2
        return lo + int(np.argmin(d2))

    def curvature_at(self, index: int) -> float:
        """@brief 인덱스 위치의 곡률 κ [1/m]."""
        index = int(np.clip(index, 0, len(self) - 1))
        return float(self._curvature[index])

    def heading_at(self, index: int) -> float:
        """@brief 인덱스 위치에서의 경로 접선 방향(heading) [rad]."""
        i = int(np.clip(index, 0, len(self) - 2))
        dx = self._points[i + 1, 0] - self._points[i, 0]
        dy = self._points[i + 1, 1] - self._points[i, 1]
        return float(np.arctan2(dy, dx))

    def lookahead_point(self, from_index: int, lookahead: float) -> np.ndarray:
        """@brief from_index에서 호 길이로 lookahead 만큼 떨어진 경로 점.

        @details 누적 거리 테이블을 이용해 from_index 이후로
        @f$ s \\ge s_{from} + L_d @f$ 를 만족하는 첫 점을 찾는다.
        경로 끝을 넘어가면 마지막 점을 반환한다(종료 처리는 호출자 책임).

        @param from_index 검색 시작 인덱스(보통 nearest_index 결과).
        @param lookahead  lookahead 거리 @f$ L_d @f$ [m].
        @return lookahead point [x, y].
        """
        target_s = self._cum_dist[from_index] + max(0.0, lookahead)
        idx = np.searchsorted(self._cum_dist, target_s)
        if idx >= len(self):
            return self._points[-1].copy()
        return self._points[idx].copy()

    def cross_track_error(self, x: float, y: float, yaw: float) -> float:
        """@brief 부호 있는 횡방향 오차(cross-track error) [m].

        @details 가장 가까운 점 기준, 경로 접선의 좌측(+) / 우측(-) 부호를
        부여한다. 부호는 차량 진행 방향이 아니라 경로 접선을 기준으로 한다.

        @param x, y 차량 위치.
        @param yaw  차량 방향(현재 미사용, 시그니처 호환용).
        @return 부호 있는 CTE [m].
        """
        return self.cross_track_error_at(x, y, self.nearest_index(x, y))

    def cross_track_error_at(self, x: float, y: float, index: int) -> float:
        """@brief 이미 알고 있는 최근접 인덱스로 부호 있는 CTE 계산.

        @param x, y  차량 위치.
        @param index 최근접 경로 점 인덱스(컨트롤러가 찾은 값 재사용).
        @return 부호 있는 CTE [m] (경로 접선 좌측이 양수).
        """
        i = int(np.clip(index, 0, len(self) - 1))
        path_yaw = self.heading_at(i)
        dx = x - self._points[i, 0]
        dy = y - self._points[i, 1]
        # 경로 접선에 수직인 방향으로의 투영 (좌측이 양수).
        return float(-np.sin(path_yaw) * dx + np.cos(path_yaw) * dy)

    def heading_error(self, yaw: float, index: int) -> float:
        """@brief 경로 접선 대비 차량 방향 오차 [rad], (-pi, pi]."""
        return normalize_angle(yaw - self.heading_at(index))

    # ------------------------------------------------------------------ #
    # Internal helpers (단일 책임 함수)
    # ------------------------------------------------------------------ #
    @staticmethod
    def _cumulative_distance(pts: np.ndarray) -> np.ndarray:
        """@brief 각 점까지의 누적 호 길이 배열 (N,)을 계산."""
        seg = np.sqrt(np.sum(np.diff(pts, axis=0) ** 2, axis=1))
        return np.concatenate([[0.0], np.cumsum(seg)])

    @classmethod
    def _resample(cls, pts: np.ndarray, spacing: float) -> np.ndarray:
        """@brief 경로를 호 길이 기준 균일 간격으로 선형 보간 리샘플."""
        cum = cls._cumulative_distance(pts)
        total = cum[-1]
        if total <= 0.0:
            return pts
        n = max(2, int(np.floor(total / spacing)) + 1)
        s_new = np.linspace(0.0, total, n)
        x_new = np.interp(s_new, cum, pts[:, 0])
        y_new = np.interp(s_new, cum, pts[:, 1])
        return np.column_stack([x_new, y_new])

    @staticmethod
    def _compute_curvature(pts: np.ndarray) -> np.ndarray:
        """@brief 1차/2차 차분 기반 이산 곡률 κ [1/m] 계산.

        @details 매개변수 곡선의 곡률 공식
        @f[
            \\kappa = \\frac{|x' y'' - y' x''|}{(x'^2 + y'^2)^{3/2}}
        @f]
        을 numpy.gradient(중심 차분)로 근사한다. 양 끝점은 한쪽 차분이라
        값이 다소 불안정할 수 있어 호출부에서 인덱스를 클립해 사용한다.
        """
        x = pts[:, 0]
        y = pts[:, 1]
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        denom = (dx * dx + dy * dy) ** 1.5
        denom = np.where(denom < 1e-9, 1e-9, denom)
        return np.abs(dx * ddy - dy * ddx) / denom
