"""Rebuild progression.csv from metrics_v2 JSONs.

This script reads every results/reports/metrics_v2/iter_NN.json file and writes
a unified progression.csv that mirrors the metric-v2 contract (TSP-corrected
path_efficiency, strict success_rate, etc.).

The earlier hand-curated progression.csv mixed v1 and v2 metric definitions,
which made cross-iteration comparison unreliable. This rebuild is idempotent:
re-run after each merge to refresh.

Topic + accept metadata is parsed out of results/reports/INDEX.md.
"""

import csv
import json
import os
import re
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
METRICS_DIR = PROJECT_ROOT / "results" / "reports" / "metrics_v2"
INDEX_PATH = PROJECT_ROOT / "results" / "reports" / "INDEX.md"
OUT_PATH = PROJECT_ROOT / "results" / "thesis_archive" / "progression.csv"

TERRAIN_KEYS = ["flat", "slope", "rough", "transition"]
PRIMARY_CTRL = "adaptive_navigator"


def parse_index() -> dict:
    """Return {iter_num: {'topic': str, 'accept': bool|None}}."""
    rows = {}
    if not INDEX_PATH.exists():
        return rows
    line_re = re.compile(r"^\|\s*(\d{1,2})\s*\|\s*([^|]*)\|\s*([^|]*)\|\s*([^|]*)\|")
    with open(INDEX_PATH, "r", encoding="utf-8") as f:
        for line in f:
            m = line_re.match(line)
            if not m:
                continue
            iter_num = int(m.group(1))
            topic = m.group(3).strip()
            accept_cell = m.group(4).strip()
            if accept_cell == "✓":
                accept = True
            elif accept_cell == "✗":
                accept = False
            else:
                accept = None
            rows[iter_num] = {"topic": topic, "accept": accept}
    return rows


def aggregate_overall(metrics_path: Path, controller: str = PRIMARY_CTRL) -> dict:
    """Average a metric across the 4 terrains for one controller, ignoring None."""
    with open(metrics_path, "r", encoding="utf-8") as f:
        d = json.load(f)
    res = d["results"]
    out = {}
    keys = ["success_rate", "path_efficiency", "completion_fraction",
            "time_to_goal", "actual_path_length", "classification_accuracy",
            "energy_proxy", "cross_track_mean", "replan_count"]
    for k in keys:
        vals = []
        for t in TERRAIN_KEYS:
            cell = res.get(t, {}).get(controller, {}).get(k)
            if cell is None:
                continue
            v = cell.get("mean")
            if v is None:
                continue
            vals.append(v)
        out[k] = sum(vals) / len(vals) if vals else None

    # Per-terrain rough-completion is the headline iter21-30 metric, surface it
    rough_cell = res.get("rough", {}).get(controller, {}).get("completion_fraction")
    out["rough_completion"] = rough_cell.get("mean") if rough_cell else None
    rough_succ = res.get("rough", {}).get(controller, {}).get("success_rate")
    out["rough_success"] = rough_succ.get("mean") if rough_succ else None
    return out


def main():
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    index = parse_index()

    metric_files = sorted(METRICS_DIR.glob("iter_*.json"),
                          key=lambda p: int(p.stem.split("_")[1]))

    fieldnames = [
        "iter", "topic", "accept",
        "success_overall", "completion_overall", "path_eff_overall",
        "rough_completion", "rough_success",
        "classification_acc_overall", "time_to_goal_overall",
        "cross_track_mean_overall", "energy_overall", "replan_count_overall",
    ]

    with open(OUT_PATH, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for mf in metric_files:
            iter_num = int(mf.stem.split("_")[1])
            agg = aggregate_overall(mf)
            meta = index.get(iter_num, {"topic": "", "accept": None})
            row = {
                "iter": iter_num,
                "topic": meta["topic"],
                "accept": ("Y" if meta["accept"] is True
                           else "N" if meta["accept"] is False else ""),
                "success_overall": _r(agg["success_rate"], 4),
                "completion_overall": _r(agg["completion_fraction"], 4),
                "path_eff_overall": _r(agg["path_efficiency"], 4),
                "rough_completion": _r(agg["rough_completion"], 4),
                "rough_success": _r(agg["rough_success"], 4),
                "classification_acc_overall": _r(agg["classification_accuracy"], 4),
                "time_to_goal_overall": _r(agg["time_to_goal"], 2),
                "cross_track_mean_overall": _r(agg["cross_track_mean"], 4),
                "energy_overall": _r(agg["energy_proxy"], 1),
                "replan_count_overall": _r(agg["replan_count"], 2),
            }
            writer.writerow(row)
            print(f"  iter{iter_num:02d}  succ={row['success_overall']}  "
                  f"completion={row['completion_overall']}  "
                  f"path_eff={row['path_eff_overall']}  "
                  f"rough_comp={row['rough_completion']}")
    print(f"\nWrote {OUT_PATH} ({len(metric_files)} rows, metric_version=v2)")


def _r(v, n):
    return "" if v is None else round(v, n)


if __name__ == "__main__":
    main()
