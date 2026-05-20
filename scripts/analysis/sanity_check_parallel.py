"""Sanity check: aggregate metrics for parallel-bench experiments and
compare against the iter05 progression baseline.

Run after one of the _bench_w* iter-tags has been generated. The point is
to confirm parallel execution does not introduce result drift beyond noise.
"""

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.analysis.analyze_baseline import parse_csv, compute_metrics, aggregate_seeds  # noqa: E402

EXP_ROOT = PROJECT_ROOT / "data" / "experiments"
TERRAINS = ["flat", "slope", "rough", "transition"]


def collect(iter_tag: str):
    folder = EXP_ROOT / iter_tag
    rows = []
    for terrain in TERRAINS:
        per_terrain = []
        for csv in folder.glob(f"{terrain}_adaptive_navigator_seed*.csv"):
            data = parse_csv(csv)
            if not data:
                continue
            m = compute_metrics(data, terrain)
            if m is not None:
                per_terrain.append(m)
        if per_terrain:
            agg = aggregate_seeds(per_terrain)
            rows.append((terrain, agg, len(per_terrain)))
    return rows


def fmt(x):
    return f"{x:.4f}" if isinstance(x, float) else str(x)


def main():
    if len(sys.argv) < 2:
        print("usage: sanity_check_parallel.py <iter_tag> [<iter_tag2> ...]")
        sys.exit(2)
    tags = sys.argv[1:]

    print(f"{'tag':<14} {'terrain':<10} {'success':<8} {'path_eff':<9} "
          f"{'time':<7} {'energy':<10} {'cls_acc':<8} {'n':<3}")
    print("-" * 80)
    for tag in tags:
        results = collect(tag)
        if not results:
            print(f"{tag:<14}  (no data found in {EXP_ROOT / tag})")
            continue
        for terrain, agg, n in results:
            print(f"{tag:<14} {terrain:<10} "
                  f"{fmt(agg['success_rate']['mean']):<8} "
                  f"{fmt(agg['path_efficiency']['mean']):<9} "
                  f"{fmt(agg['time_to_goal']['mean']):<7} "
                  f"{fmt(agg['energy_proxy']['mean']):<10} "
                  f"{fmt(agg['classification_accuracy']['mean']):<8} "
                  f"{n}")

    # Also load iter05 metrics for reference
    iter05_path = PROJECT_ROOT / "results" / "reports" / "metrics" / "iter_05.json"
    if iter05_path.exists():
        print("-" * 80)
        print(f"reference iter05 (main, serial):")
        data = json.loads(iter05_path.read_text(encoding="utf-8"))
        results = data.get("results", {})
        for terrain in TERRAINS:
            t = results.get(terrain, {}).get("adaptive_navigator")
            if not t:
                continue
            print(f"iter05_serial  {terrain:<10} "
                  f"{fmt(t.get('success_rate', {}).get('mean', 0)):<8} "
                  f"{fmt(t.get('path_efficiency', {}).get('mean', 0)):<9} "
                  f"{fmt(t.get('time_to_goal', {}).get('mean', 0)):<7} "
                  f"{fmt(t.get('energy_proxy', {}).get('mean', 0)):<10} "
                  f"{fmt(t.get('classification_accuracy', {}).get('mean', 0)):<8} "
                  f"{t.get('success_rate', {}).get('n', 0)}")


if __name__ == "__main__":
    main()
