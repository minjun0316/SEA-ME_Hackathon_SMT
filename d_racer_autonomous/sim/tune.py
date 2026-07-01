"""@file tune.py
@brief Stage 3: 파라미터 그리드 스윕 기반 체계적 제어기 튜닝.

@details
"감으로" 파라미터를 바꾸는 대신, 정의한 파라미터 격자 전체를 시뮬레이션
하여 비용(주행 실패 + cross-track error)이 가장 낮은 조합을 찾는다.
이는 자동차 SW 개발의 "재현 가능한 튜닝" 원칙을 따른다.

@par 비용 함수
@f[ J = w_{fail}\\,[\\text{미완주}] + RMS_{CTE} + 0.3\\,MAX_{CTE} @f]
미완주(경로 이탈/시간 초과)에는 큰 페널티를 주어 항상 완주하는 조합을
우선한다.

@par 사용 예
@code{.sh}
# sharp_s에서 lookahead/속도/스무딩을 격자 탐색하고 상위 10개 출력
python -m sim.tune --path sharp_s \\
    --param pure_pursuit.ld_min=0.2,0.3,0.4 \\
    --param pure_pursuit.kv=0.2,0.4 \\
    --param speed.v_max=0.4,0.6,0.8 \\
    --top 10

# 최적 조합을 YAML로 저장 (config에 반영)
python -m sim.tune --path sharp_s --param speed.v_max=0.4,0.5,0.6 \\
    --save-best config/tuned_sharp_s.yaml
@endcode
"""
from __future__ import annotations

import argparse
import itertools
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

import yaml

from core import path_factory
from core.config_schema import AppConfig

from .simulator import Simulator

_DEFAULT_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")

## @brief 미완주에 부여하는 비용 페널티(완주 조합을 항상 우선).
FAIL_PENALTY = 100.0


@dataclass
class TuneResult:
    """@brief 한 파라미터 조합의 평가 결과."""

    params: Dict[str, object]   ##< "section.key" -> value.
    cost: float                 ##< 비용(작을수록 좋음).
    reached: bool
    rms_cte: float
    max_cte: float


def _load_base(config_paths: List[str]) -> dict:
    """@brief 기본 설정 YAML들을 병합해 dict로 반환."""
    merged: dict = {}
    for path in config_paths:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        for section, values in data.items():
            if isinstance(values, dict):
                merged.setdefault(section, {}).update(values)
            else:
                merged[section] = values
    return merged


def _parse_grid(param_specs: List[str]) -> Dict[str, List]:
    """@brief "section.key=v1,v2,.." 목록을 {keypath: [values]} 격자로 변환."""
    grid: Dict[str, List] = {}
    for spec in param_specs:
        if "=" not in spec or "." not in spec.split("=", 1)[0]:
            raise ValueError(f"--param expects section.key=v1,v2,..  got '{spec}'")
        keypath, raw = spec.split("=", 1)
        values = [yaml.safe_load(v) for v in raw.split(",")]
        grid[keypath] = values
    return grid


def _apply(base: dict, combo: Dict[str, object]) -> AppConfig:
    """@brief 기본 설정에 한 조합을 덮어쓴 AppConfig 생성."""
    merged = {k: dict(v) if isinstance(v, dict) else v for k, v in base.items()}
    for keypath, value in combo.items():
        section, key = keypath.split(".", 1)
        merged.setdefault(section, {})[key] = value
    return AppConfig.from_dict(merged)


def evaluate(base: dict, combo: Dict[str, object], path_name: str) -> TuneResult:
    """@brief 한 파라미터 조합을 시뮬레이션하여 비용을 계산한다.

    @param base      기본 설정 dict.
    @param combo     덮어쓸 파라미터 조합.
    @param path_name 평가에 사용할 경로 이름.
    @return TuneResult.
    """
    config = _apply(base, combo)
    config.sim.path_name = path_name
    path = path_factory.make_path(path_name)
    summary = Simulator(config, path).run()

    cost = summary.rms_cte + 0.3 * summary.max_cte
    if not summary.reached_goal:
        cost += FAIL_PENALTY
    return TuneResult(
        params=combo, cost=cost, reached=summary.reached_goal,
        rms_cte=summary.rms_cte, max_cte=summary.max_cte)


def run_sweep(base: dict, grid: Dict[str, List],
              path_name: str) -> List[TuneResult]:
    """@brief 격자의 모든 조합을 평가하고 비용 오름차순으로 정렬해 반환."""
    keys = list(grid.keys())
    results: List[TuneResult] = []
    for combo_values in itertools.product(*(grid[k] for k in keys)):
        combo = dict(zip(keys, combo_values))
        results.append(evaluate(base, combo, path_name))
    results.sort(key=lambda r: r.cost)
    return results


def _grid_to_nested(combo: Dict[str, object]) -> dict:
    """@brief {"a.b": v} 를 {"a": {"b": v}} 중첩 dict로(YAML 저장용)."""
    out: dict = {}
    for keypath, value in combo.items():
        section, key = keypath.split(".", 1)
        out.setdefault(section, {})[key] = value
    return out


def main(argv: List[str] | None = None) -> int:
    """@brief 튜닝 엔트리포인트."""
    parser = argparse.ArgumentParser(description="Stage 3 파라미터 그리드 튜너")
    parser.add_argument("--config", nargs="+",
                        default=[os.path.join(_DEFAULT_CONFIG_DIR, "vehicle.yaml"),
                                 os.path.join(_DEFAULT_CONFIG_DIR, "controller.yaml"),
                                 os.path.join(_DEFAULT_CONFIG_DIR, "sim.yaml")])
    parser.add_argument("--path", required=True, help="튜닝 대상 경로 이름")
    parser.add_argument("--param", action="append", default=[], required=True,
                        help="section.key=v1,v2,.. (반복 가능)")
    parser.add_argument("--top", type=int, default=10, help="상위 N개 출력")
    parser.add_argument("--save-best", default=None, help="최적 조합 YAML 저장 경로")
    args = parser.parse_args(argv)

    base = _load_base(args.config)
    grid = _parse_grid(args.param)
    total = 1
    for v in grid.values():
        total *= len(v)
    print(f"[tune] path={args.path}  combinations={total}")

    results = run_sweep(base, grid, args.path)

    print("=" * 72)
    print(f"{'rank':>4} {'cost':>8} {'reach':>6} {'rmsCTE':>8} {'maxCTE':>8}  params")
    print("-" * 72)
    for i, r in enumerate(results[:args.top]):
        params = ", ".join(f"{k}={v}" for k, v in r.params.items())
        print(f"{i+1:>4} {r.cost:>8.3f} {str(r.reached):>6} "
              f"{r.rms_cte*100:>7.2f}c {r.max_cte*100:>7.2f}c  {params}")
    print("=" * 72)

    if args.save_best and results:
        best = _grid_to_nested(results[0].params)
        with open(args.save_best, "w", encoding="utf-8") as stream:
            yaml.safe_dump(best, stream, sort_keys=False, allow_unicode=True)
        print(f"[tune] best params saved to {args.save_best}")
        print(f"       -> use: python -m sim.run_sim --path {args.path} "
              f"--config {' '.join(args.config)} {args.save_best}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
