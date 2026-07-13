"""@file stopline_maneuver.py
@brief 정지선 카운트 기반 개루프 고정스티어 기동(로터리 진입/탈출) — ROS-free.

@details
로터리(원) 안에서 정지선을 셀 때마다 정해진 방향으로 **일정 시간 고정 조향**을
내보내는 최소 상태기계. 폐루프 차선추종(RoiMode bias)이 아니라 열린 루프라
차선을 잠깐 놓쳐도 정해진 시간만큼 그대로 돌아 나간다.

@par 동작 (스펙: 07-12 실차 결정)
- 1번째 정지선 → `first_dir` 방향(기본 +1=좌회전) 고정스티어 `duration_sec` 동안.
- 2번째 정지선 → `second_dir` 방향(기본 -1=우회전) 고정스티어 `duration_sec` 동안(탈출).
- `max_count` 초과 정지선은 무시(기동 종료 상태 유지).

@par 부호/단위
반환하는 조향은 **트림 전 raw** [-1,1]: +면 좌, -면 우(lateral_pd와 동일 규약).
호출부(controller)가 `steer = clip(trim + raw, -1, 1)`로 트림을 실어 발행한다.
`steer` 크기는 "조금"에 해당하는 튜닝값(config).

@par 중복 카운트 방지
정지선은 접근~통과 동안 여러 프레임 True다. rising-edge(False→True) + 마지막
카운트 이후 `debounce_sec` 경과 조건으로 한 정지선을 한 번만 센다. 기동 중에는
새 카운트를 받지 않고(차량이 이미 정지선을 지나 회전 중), 기동 종료 시 debounce를
재무장해 방금 지난 정지선을 재카운트하지 않는다.
"""
from __future__ import annotations

from typing import Optional

from ..config_schema import StoplineManeuverConfig


class StoplineManeuver:
    """@brief 정지선 카운트 → 개루프 고정스티어 상태기계(ROS-free).

    @details 상태(카운트/디바운스/활성 기동 잔여시간)를 내부에 보관한다. 매 스텝
    (stop_line, dt)만 받고 경로/pose/조향루프를 모른다. 활성 기동 중이면 고정
    조향(raw)을, 아니면 None(=평소 제어 유지)을 반환한다.
    """

    def __init__(self, cfg: StoplineManeuverConfig):
        self.cfg = cfg
        self._count = 0            ##< 누적 정지선 카운트(0→1→2).
        self._prev_stop = False    ##< 직전 스텝 stop_line(rising-edge용).
        self._since_count = float("inf")  ##< 마지막 카운트 이후 경과[s](디바운스).
        self._active = False       ##< 기동 활성 여부.
        self._remaining = 0.0      ##< 활성 기동 잔여 시간[s].
        self._steer = 0.0          ##< 현재 기동 고정 조향(raw).

    def reset(self) -> None:
        """@brief 상태 초기화(로터리 재시도/노드 재시작)."""
        self._count = 0
        self._prev_stop = False
        self._since_count = float("inf")
        self._active = False
        self._remaining = 0.0
        self._steer = 0.0

    @property
    def count(self) -> int:
        """@brief 지금까지 센 정지선 수."""
        return self._count

    @property
    def active(self) -> bool:
        """@brief 기동(고정스티어) 진행 중 여부."""
        return self._active

    def update(self, stop_line: bool, dt: float) -> Optional[float]:
        """@brief 한 스텝 진행. @return 활성 기동 중이면 고정 조향(raw), 아니면 None.

        @param stop_line 이번 스텝 정지선 검출 여부(인지). 차선 소실 등으로 알 수
                         없으면 False로 넘긴다(그동안 새 카운트만 막힐 뿐, 진행 중인
                         기동은 stop_line과 무관하게 잔여시간으로 계속된다).
        @param dt        스텝 시간[s].
        """
        stop = bool(stop_line)

        # 1) 활성 기동: stop_line과 무관하게 잔여시간만 소모(개루프).
        if self._active:
            self._remaining -= dt
            self._prev_stop = stop
            if self._remaining <= 0.0:
                self._active = False
                self._steer = 0.0
                self._since_count = 0.0  # 방금 지난 정지선 재카운트 방지(debounce 재무장).
                return None
            return self._steer

        # 2) 비활성: 카운트 판정(rising-edge + debounce + max_count).
        self._since_count += dt
        rising = stop and not self._prev_stop
        self._prev_stop = stop
        if not (rising and self._since_count >= self.cfg.debounce_sec
                and self._count < self.cfg.max_count):
            return None

        # 새 정지선 카운트 → 방향 결정 후 기동 개시.
        self._count += 1
        self._since_count = 0.0
        direction = self.cfg.first_dir if self._count == 1 else self.cfg.second_dir
        self._steer = float(direction) * float(self.cfg.steer)
        self._remaining = float(self.cfg.duration_sec)
        self._active = True
        return self._steer
