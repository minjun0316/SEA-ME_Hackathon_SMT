"""@file logger.py
@brief 주행/시뮬 로그를 CSV로 저장 (Stage 4/5 분석용 고정 스키마).

@details
컬럼 순서·이름은 실차 ROS2 Logger 노드와 동일하게 유지한다. 그래야
Stage 5 MATLAB 분석 스크립트가 시뮬 로그와 실차 로그를 구분 없이
읽을 수 있다 (시뮬↔실차 분석 일원화).

@par 고정 스키마
time, x, y, yaw, target_x, target_y, lookahead, curvature,
cross_track_error, heading_error, steering_cmd, speed_cmd,
battery_voltage, mission_state
"""
from __future__ import annotations

import csv
from typing import List

## @brief CSV 컬럼 정의(순서 고정). 실차 Logger 노드와 반드시 동일하게 유지.
LOG_COLUMNS: List[str] = [
    "time",
    "x",
    "y",
    "yaw",
    "target_x",
    "target_y",
    "lookahead",
    "curvature",
    "cross_track_error",
    "heading_error",
    "steering_cmd",
    "speed_cmd",
    "battery_voltage",
    "mission_state",
]


class DriveLogger:
    """@brief 메모리에 행을 모았다가 CSV로 한 번에 저장하는 로거."""

    def __init__(self):
        self._rows: List[dict] = []

    def log(self, **kwargs) -> None:
        """@brief 한 스텝의 로그 행을 추가한다.

        @param kwargs LOG_COLUMNS의 키들. 누락된 키는 빈 값으로 채운다.
        @throws KeyError 스키마에 없는 키가 들어오면(오타 방지) 예외.
        """
        unknown = set(kwargs) - set(LOG_COLUMNS)
        if unknown:
            raise KeyError(f"unknown log fields: {sorted(unknown)}")
        self._rows.append({col: kwargs.get(col, "") for col in LOG_COLUMNS})

    def __len__(self) -> int:
        return len(self._rows)

    @property
    def rows(self) -> List[dict]:
        """@brief 누적된 로그 행 목록."""
        return self._rows

    def save_csv(self, path: str) -> None:
        """@brief 누적 로그를 CSV 파일로 저장한다.

        @param path 출력 CSV 경로.
        """
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=LOG_COLUMNS)
            writer.writeheader()
            writer.writerows(self._rows)
