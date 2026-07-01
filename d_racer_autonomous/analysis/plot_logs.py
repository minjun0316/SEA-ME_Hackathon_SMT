"""@file plot_logs.py
@brief Stage 5(보조): 주행 로그 시각화 (Python/matplotlib).

@details
MATLAB 스크립트(plot_logs.m)와 동일한 그래프를 파이썬으로도 그릴 수 있게
한다(MATLAB 미설치 환경/CI 대비). CSV 또는 DriveLogger 객체를 입력받는다.

그래프 구성(사용자 명세):
  1) Reference Path vs Vehicle Path
  2) Cross Track Error   3) Heading Error
  4) Steering            5) Speed
  6) Lookahead           7) Curvature
  8) Battery Voltage
"""
from __future__ import annotations

import csv
from typing import List, Optional

import numpy as np


def _read_csv(path: str) -> dict:
    """@brief CSV 로그를 컬럼별 numpy 배열 dict로 읽는다."""
    with open(path, "r", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    cols = {}
    for key in reader.fieldnames or []:
        try:
            cols[key] = np.array([float(r[key]) for r in rows])
        except ValueError:
            cols[key] = np.array([r[key] for r in rows])  # mission_state 등 문자열
    return cols


def _rows_to_cols(rows: List[dict]) -> dict:
    """@brief DriveLogger.rows(list[dict])를 컬럼 배열 dict로 변환."""
    cols = {}
    if not rows:
        return cols
    for key in rows[0].keys():
        try:
            cols[key] = np.array([float(r[key]) for r in rows])
        except (ValueError, TypeError):
            cols[key] = np.array([r[key] for r in rows])
    return cols


def _plot(cols: dict, ref_xy: Optional[np.ndarray],
          show: bool, save_path: Optional[str]) -> None:
    """@brief 컬럼 dict로부터 8개 분석 그래프를 그린다."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")  # 디스플레이 없는 환경(헤드리스/CI) 대비.
    import matplotlib.pyplot as plt

    t = cols["time"]
    fig, axes = plt.subplots(4, 2, figsize=(13, 14))
    fig.suptitle("D-Racer Controller Analysis (Stage 5)", fontsize=14)

    # 1) 경로 비교
    ax = axes[0, 0]
    if ref_xy is not None:
        ax.plot(ref_xy[:, 0], ref_xy[:, 1], "k--", lw=1.2, label="Reference")
    ax.plot(cols["x"], cols["y"], "b-", lw=1.5, label="Vehicle")
    ax.scatter(cols["target_x"][::15], cols["target_y"][::15],
               c="r", s=8, alpha=0.4, label="Lookahead target")
    ax.set_title("Reference vs Vehicle Path")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.axis("equal"); ax.legend(); ax.grid(True)

    # 2~8) 시계열
    series = [
        ("cross_track_error", "Cross Track Error [m]", (0, 1)),
        ("heading_error", "Heading Error [rad]", (1, 0)),
        ("steering_cmd", "Steering Cmd [-1,1]", (1, 1)),
        ("speed_cmd", "Speed Cmd", (2, 0)),
        ("lookahead", "Lookahead [m]", (2, 1)),
        ("curvature", "Curvature [1/m]", (3, 0)),
        ("battery_voltage", "Battery Voltage [V]", (3, 1)),
    ]
    for key, title, (r, c) in series:
        ax = axes[r, c]
        ax.plot(t, cols[key], lw=1.2)
        ax.set_title(title); ax.set_xlabel("time [s]"); ax.grid(True)
        if key == "cross_track_error":
            ax.axhline(0.0, color="k", lw=0.8, alpha=0.5)

    fig.tight_layout(rect=[0, 0, 1, 0.98])
    if save_path:
        fig.savefig(save_path, dpi=120)
        print(f"[plot] saved figure to {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def plot_from_csv(csv_path: str, show: bool = True,
                  save_path: Optional[str] = None) -> None:
    """@brief CSV 파일로부터 그래프를 그린다.

    @param csv_path  로그 CSV 경로.
    @param show      화면 표시 여부.
    @param save_path PNG 저장 경로(선택).
    """
    cols = _read_csv(csv_path)
    _plot(cols, ref_xy=None, show=show, save_path=save_path)


def plot_from_logger(logger, path=None, show: bool = True,
                     save_path: Optional[str] = None) -> None:
    """@brief DriveLogger 객체로부터 그래프를 그린다(참조 경로 포함).

    @param logger    DriveLogger 인스턴스.
    @param path      core.path.Path (참조 경로 오버레이용, 선택).
    @param show      화면 표시 여부.
    @param save_path PNG 저장 경로(선택).
    """
    cols = _rows_to_cols(logger.rows)
    ref_xy = path.points if path is not None else None
    _plot(cols, ref_xy=ref_xy, show=show, save_path=save_path)


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="주행 로그 CSV 시각화")
    p.add_argument("csv", help="로그 CSV 경로")
    p.add_argument("--save", default=None, help="PNG 저장 경로")
    p.add_argument("--no-show", action="store_true", help="화면 표시 안 함")
    a = p.parse_args()
    plot_from_csv(a.csv, show=not a.no_show, save_path=a.save)
