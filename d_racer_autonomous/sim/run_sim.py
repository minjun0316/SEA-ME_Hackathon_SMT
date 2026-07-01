"""@file run_sim.py
@brief 시뮬레이션 실행 엔트리포인트 (Stage 2/3).

@details
설정 YAML과 경로 이름을 받아 시뮬레이션을 돌리고, CSV 로그와
요약/플롯을 생성한다. 하드코딩 없이 모든 파라미터는 YAML/CLI로 제어한다.

@par 사용 예
@code{.sh}
# 기본(config의 path_name 사용)
python -m sim.run_sim

# 경로/설정 지정 + 플롯 표시
python -m sim.run_sim --path sharp_s --plot

# 고정 lookahead로 초기 테스트
python -m sim.run_sim --path circle --set pure_pursuit.use_adaptive_lookahead=false
@endcode
"""
from __future__ import annotations

import argparse
import os
from typing import List

from core import path_factory
from core.config_schema import AppConfig, load_config

from .simulator import Simulator

## @brief 기본 설정 디렉토리(프로젝트 루트의 config/).
_DEFAULT_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")


def _apply_overrides(config_dict: dict, overrides: List[str]) -> dict:
    """@brief CLI --set a.b=c 형태의 override를 설정 dict에 반영.

    @param config_dict 병합 전 설정 dict.
    @param overrides   "section.key=value" 문자열 목록.
    @return override가 반영된 dict.
    """
    import yaml
    for item in overrides:
        if "=" not in item or "." not in item.split("=", 1)[0]:
            raise ValueError(f"--set expects section.key=value, got '{item}'")
        keypath, raw = item.split("=", 1)
        section, key = keypath.split(".", 1)
        value = yaml.safe_load(raw)  # 타입 자동 추론(bool/int/float/str)
        config_dict.setdefault(section, {})[key] = value
    return config_dict


def build_config(args: argparse.Namespace) -> AppConfig:
    """@brief CLI 인자로부터 최종 AppConfig를 구성한다."""
    import yaml
    merged: dict = {}
    for path in args.config:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        for section, values in data.items():
            if isinstance(values, dict):
                merged.setdefault(section, {}).update(values)
            else:
                merged[section] = values

    if args.set:
        merged = _apply_overrides(merged, args.set)
    if args.path:
        merged.setdefault("sim", {})["path_name"] = args.path

    return AppConfig.from_dict(merged)


def main(argv: List[str] | None = None) -> int:
    """@brief 엔트리포인트.

    @param argv 테스트용 인자 리스트(기본은 sys.argv).
    @return 종료 코드(0=성공).
    """
    parser = argparse.ArgumentParser(description="D-Racer 자율주행 시뮬레이터 (Stage 2/3)")
    parser.add_argument(
        "--config", nargs="+",
        default=[os.path.join(_DEFAULT_CONFIG_DIR, "vehicle.yaml"),
                 os.path.join(_DEFAULT_CONFIG_DIR, "controller.yaml"),
                 os.path.join(_DEFAULT_CONFIG_DIR, "sim.yaml")],
        help="병합할 설정 YAML 경로들(뒤가 우선).")
    parser.add_argument("--path", default=None,
                        help=f"경로 이름 {sorted(path_factory.PATH_LIBRARY)}")
    parser.add_argument("--set", action="append", default=[],
                        help="설정 override: section.key=value (반복 가능)")
    parser.add_argument("--out", default="sim_log.csv", help="CSV 로그 출력 경로")
    parser.add_argument("--plot", action="store_true", help="결과 그래프 표시")
    parser.add_argument("--save-plot", default=None, help="그래프 PNG 저장 경로")
    args = parser.parse_args(argv)

    config = build_config(args)
    path = path_factory.make_path(config.sim.path_name)

    # Stage 3 사전 점검: 경로가 차량으로 추종 가능한 곡률인지 먼저 확인.
    from core.feasibility import check_path
    report = check_path(path, config.vehicle)

    sim = Simulator(config, path)
    summary = sim.run()
    sim.logger.save_csv(args.out)

    print("=" * 56)
    print(f"  feasibility    : {report.summary()}")
    if not report.feasible:
        print("  ⚠ 경로가 차량 최소 회전반경을 초과합니다 → 추종 불가, "
              "튜닝 전 경로/차량 사양 재검토 필요")
    print(f"  path           : {config.sim.path_name}")
    print(f"  lookahead mode : "
          f"{'adaptive' if config.pure_pursuit.use_adaptive_lookahead else 'fixed'}")
    print(f"  reached_goal   : {summary.reached_goal}")
    print(f"  steps / time   : {summary.steps} / {summary.sim_time:.2f}s")
    print(f"  RMS  CTE       : {summary.rms_cte*100:.2f} cm")
    print(f"  MAX  CTE       : {summary.max_cte*100:.2f} cm")
    print(f"  log saved      : {args.out}  ({len(sim.logger)} rows)")
    print("=" * 56)

    if args.plot or args.save_plot:
        from analysis.plot_logs import plot_from_logger
        plot_from_logger(sim.logger, path, show=args.plot, save_path=args.save_plot)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
