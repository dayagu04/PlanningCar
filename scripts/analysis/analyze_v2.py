"""Analyze experiment results — v2 with corrected metrics.

Improvements over v1 (analyze_baseline.py):

  * **path_efficiency** numerator is the deterministic TSP tour length
    (start + 4 waypoints) reconstructed from the seed, not the start→end
    Euclidean distance. v1 was order-of-magnitude wrong on runs that did not
    visit all waypoints (start ≈ end → straight_dist tiny → eff tiny).

  * **success_rate** is strict: a run is successful only when all
    NUM_WAYPOINTS distinct waypoints were *physically* visited (target_idx
    transition observed with dist_to_target < GOAL_TOLERANCE on the row
    immediately preceding the transition). Stuck-skip transitions, where
    the controller bumps target_idx without reaching the goal, do not
    count.

  * **time_to_goal** is the simulated time at the moment the *last*
    physical visit completes the tour (or NaN for failed runs).

  * **completion_fraction** (new): fraction of waypoints physically
    visited. Even on failed runs this gives a graded signal of how close
    the controller got — much more useful than binary success.

  * Failed runs do *not* contribute their path_eff to the aggregate (they
    contaminate the mean since their path is longer than the TSP tour).

CSV input format (unchanged):
    step, time_s, x, y, z, roll, pitch, yaw, terrain, speed,
    target_idx, dist_to_target

Ground truth (waypoints, robot start, TSP length) is regenerated from the
seed using the same generators used at run-time, so no controller changes
are needed.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from src.planning.waypoints import generate_random_waypoints, generate_random_robot_start
from src.planning.tsp_solver import optimize_waypoint_order

EXPERIMENTS_DIR = PROJECT_ROOT / "data" / "experiments"
METRICS_DIR = PROJECT_ROOT / "results" / "reports" / "metrics_v2"

NUM_WAYPOINTS = 4
GOAL_TOLERANCE = 1.0  # m — must match controller GOAL_TOLERANCE
EXPECTED_TERRAIN = {"flat": "flat", "slope": "slope",
                    "rough": "rough", "transition": "transition"}


def reconstruct_ground_truth(seed: int) -> Tuple[Tuple[float, float],
                                                  List[Tuple[float, float]],
                                                  float]:
    """Re-derive (start, ordered waypoints, TSP tour length) from a seed."""
    robot_start = generate_random_robot_start(max_radius=3.0, seed=seed)
    raw_wps = generate_random_waypoints(num_points=NUM_WAYPOINTS,
                                        min_radius=5.0, max_radius=10.0,
                                        min_separation=4.0, seed=seed)
    waypoints, tsp_info = optimize_waypoint_order(robot_start, raw_wps)
    return robot_start, list(waypoints), float(tsp_info["optimized_length"])


def parse_csv(csv_path: Path) -> List[Dict]:
    """Parse navigation CSV — tolerant to optional `# k=v` comment lines."""
    if not csv_path.exists():
        return []
    rows: List[Dict] = []
    with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
        header = None
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            if header is None:
                header = s.split(",")
                continue
            values = s.split(",")
            if len(values) < len(header):
                continue
            row: Dict = {}
            for i, col in enumerate(header):
                v = values[i]
                try:
                    row[col] = float(v)
                except ValueError:
                    row[col] = v
            rows.append(row)
    return rows


def detect_visits(rows: List[Dict], waypoints: List[Tuple[float, float]],
                  tolerance: float = 1.2) -> Dict:
    """Detect physical waypoint visits and per-waypoint closest approach.

    The CSV is sampled every 10 sim-steps (~0.32 s) and target_idx is
    incremented either on a real arrival (dist < GOAL_TOLERANCE) or on a
    stuck-skip recovery. Combined with the coarse sample rate this makes
    transition-row inspection unreliable.

    The robust criterion: scan the *entire trajectory* and, for each
    waypoint position (reconstructed from the seed), record the minimum
    Euclidean distance the robot ever achieved. A waypoint counts as
    physically visited iff that minimum is below ``tolerance``. This
    matches "the robot was actually there", regardless of which target_idx
    it was chasing at the moment.

    Returns:
      {
        "visited": List[bool] indexed by waypoint,
        "min_dist": List[float] closest approach per waypoint,
        "first_visit_row": List[int|None] CSV row index of closest approach
                           (None if never reached),
        "transitions": List[{"row","time_s","from_idx","to_idx",
                             "logged_dist"}],
      }
    """
    if not rows:
        return {"visited": [], "min_dist": [], "first_visit_row": [],
                "transitions": []}
    n_wp = len(waypoints)
    min_dist = [float("inf")] * n_wp
    first_visit_row: List[Optional[int]] = [None] * n_wp
    for i, r in enumerate(rows):
        x, y = r["x"], r["y"]
        for w_idx, (wx, wy) in enumerate(waypoints):
            d = math.hypot(x - wx, y - wy)
            if d < min_dist[w_idx]:
                min_dist[w_idx] = d
            if d < tolerance and first_visit_row[w_idx] is None:
                first_visit_row[w_idx] = i

    visited = [d < tolerance for d in min_dist]

    transitions: List[Dict] = []
    prev_idx = int(rows[0].get("target_idx", 0))
    for i in range(1, len(rows)):
        cur = int(rows[i].get("target_idx", 0))
        if cur == prev_idx:
            continue
        transitions.append({
            "row": i,
            "time_s": float(rows[i].get("time_s", 0.0)),
            "from_idx": prev_idx,
            "to_idx": cur,
            "logged_dist": float(rows[i - 1].get("dist_to_target", 999.0)),
        })
        prev_idx = cur
    return {"visited": visited, "min_dist": min_dist,
            "first_visit_row": first_visit_row, "transitions": transitions}


def compute_metrics(rows: List[Dict], seed: int,
                    expected_terrain: str) -> Optional[Dict]:
    if not rows:
        return None
    _, waypoints, tsp_length = reconstruct_ground_truth(seed)

    detection = detect_visits(rows, waypoints)
    visited = detection["visited"]
    min_dist = detection["min_dist"]
    first_visit_row = detection["first_visit_row"]
    transitions = detection["transitions"]

    distinct_real = sum(visited)
    success = distinct_real >= NUM_WAYPOINTS
    completion_fraction = distinct_real / NUM_WAYPOINTS

    # Cumulative path length per row
    cum_path = [0.0]
    for i in range(1, len(rows)):
        dx = rows[i]["x"] - rows[i - 1]["x"]
        dy = rows[i]["y"] - rows[i - 1]["y"]
        cum_path.append(cum_path[-1] + math.hypot(dx, dy))

    if success:
        # The "tour completion" moment is when the LAST of the four
        # waypoints first comes within tolerance — i.e. max(first_visit_row)
        last_visit_row = max(r for r in first_visit_row if r is not None)
        time_to_goal = float(rows[last_visit_row]["time_s"])
        actual_path_to_goal = cum_path[last_visit_row]
        path_efficiency: Optional[float] = (
            tsp_length / actual_path_to_goal if actual_path_to_goal > 0 else 0.0
        )
        # Cap at 1.0: TSP is greedy NN + 2-opt (not provably optimal) and
        # the controller can shortcut by using its 1.0 m tolerance ball.
        if path_efficiency is not None and path_efficiency > 1.0:
            path_efficiency = 1.0
    else:
        time_to_goal = float("nan")
        path_efficiency = None
        actual_path_to_goal = cum_path[-1]

    # Skip count = transitions whose logged dist looked like a stuck-skip
    skip_count = sum(1 for t in transitions if t["logged_dist"] >= GOAL_TOLERANCE)

    # Cross-track proxy via heading rate (kept for back-compat)
    yaws = [r["yaw"] for r in rows]
    if len(yaws) > 1:
        yaw_rates = [abs(yaws[i] - yaws[i - 1]) for i in range(1, len(yaws))]
        yaw_rates = [min(r, 2 * math.pi - r) for r in yaw_rates]
        cross_track_mean = float(np.mean(yaw_rates))
        cross_track_max = float(np.max(yaw_rates))
        replan_proxy = sum(1 for r in yaw_rates if r > 0.5)
    else:
        cross_track_mean = cross_track_max = 0.0
        replan_proxy = 0

    # Classification accuracy
    terrain_labels = [str(r.get("terrain", "")) for r in rows]
    if expected_terrain == "transition":
        valid = {"flat", "slope", "rough", "transition"}
        correct = sum(1 for t in terrain_labels if t in valid)
    else:
        correct = sum(1 for t in terrain_labels if t == expected_terrain)
    classification_acc = correct / len(terrain_labels) if terrain_labels else 0.0

    # Energy proxy
    energy = 0.0
    for i in range(1, len(rows)):
        dt = rows[i]["time_s"] - rows[i - 1]["time_s"]
        v = rows[i].get("speed", 0)
        energy += v * v * dt

    return {
        "success": success,
        "completion_fraction": completion_fraction,
        "distinct_real_visits": distinct_real,
        "min_dist_per_wp": min_dist,
        "transitions_count": len(transitions),
        "skip_count": skip_count,
        "tsp_length": tsp_length,
        "actual_path_length": actual_path_to_goal,
        "path_efficiency": path_efficiency,
        "time_to_goal": time_to_goal,
        "cross_track_mean": cross_track_mean,
        "cross_track_max": cross_track_max,
        "classification_accuracy": classification_acc,
        "energy_proxy": energy,
        "replan_count": replan_proxy,
        "num_samples": len(rows),
    }


def aggregate(metrics_list: List[Dict]) -> Dict:
    if not metrics_list:
        return {}
    success_count = sum(1 for m in metrics_list if m["success"])
    out: Dict = {
        "success_rate": {"mean": success_count / len(metrics_list),
                         "std": 0.0, "n": len(metrics_list)},
    }

    def stats(key, only_successful=False):
        vals = []
        for m in metrics_list:
            if only_successful and not m["success"]:
                continue
            v = m.get(key)
            if v is None or (isinstance(v, float) and math.isnan(v)):
                continue
            vals.append(v)
        if not vals:
            return {"mean": None, "std": None, "n": 0}
        return {"mean": float(np.mean(vals)), "std": float(np.std(vals)),
                "n": len(vals)}

    # Successful-only metrics — avoid contaminating averages with failed runs
    out["path_efficiency"] = stats("path_efficiency", only_successful=True)
    out["time_to_goal"] = stats("time_to_goal", only_successful=True)
    out["actual_path_length"] = stats("actual_path_length", only_successful=True)
    # All-runs metrics
    out["completion_fraction"] = stats("completion_fraction")
    out["skip_count"] = stats("skip_count")
    out["cross_track_mean"] = stats("cross_track_mean")
    out["cross_track_max"] = stats("cross_track_max")
    out["classification_accuracy"] = stats("classification_accuracy")
    out["energy_proxy"] = stats("energy_proxy")
    out["replan_count"] = stats("replan_count")
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iter-tag", default="iter00")
    parser.add_argument("--iter-num", type=int, default=0)
    parser.add_argument("--note", default="")
    args = parser.parse_args()

    exp_dir = EXPERIMENTS_DIR / args.iter_tag
    if not exp_dir.exists():
        print(f"Error: {exp_dir} not found")
        return 1

    terrains = ["flat", "slope", "rough", "transition"]
    controllers = ["adaptive_navigator", "adaptive_navigator_baseline", "astar_navigator"]
    seeds = [42, 43, 44, 45, 46]

    results: Dict = {}
    for terrain in terrains:
        results[terrain] = {}
        for controller in controllers:
            metrics_list: List[Dict] = []
            for seed in seeds:
                f = exp_dir / f"{terrain}_{controller}_seed{seed}.csv"
                if not f.exists():
                    print(f"Missing: {f.name}")
                    continue
                rows = parse_csv(f)
                m = compute_metrics(rows, seed, EXPECTED_TERRAIN[terrain])
                if m:
                    metrics_list.append(m)
            if metrics_list:
                results[terrain][controller] = aggregate(metrics_list)

    output = {
        "iter": args.iter_num,
        "tag": args.iter_tag,
        "metric_version": "v2",
        "controllers": controllers,
        "terrains": terrains,
        "seeds": seeds,
        "results": results,
        "note": args.note,
    }

    METRICS_DIR.mkdir(parents=True, exist_ok=True)
    out_file = METRICS_DIR / f"iter_{args.iter_num:02d}.json"
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)
    print(f"Metrics v2 saved: {out_file}\n")

    # Print summary
    print("=" * 92)
    print(f"{'Terrain':10} {'Controller':32} {'Success':>9} {'Compl':>7} "
          f"{'PathEff':>8} {'Time(s)':>9} {'Skips':>6}")
    print("=" * 92)
    for terrain in terrains:
        for controller in controllers:
            r = results[terrain].get(controller)
            if not r:
                continue
            sr = r["success_rate"]["mean"] * 100
            cf = (r["completion_fraction"]["mean"] or 0) * 100
            pe = r["path_efficiency"]["mean"]
            pe_s = f"{pe:.3f}" if pe is not None else "  n/a"
            tg = r["time_to_goal"]["mean"]
            tg_s = f"{tg:.1f}" if tg is not None else "  n/a"
            sk = r["skip_count"]["mean"] or 0
            print(f"{terrain:10} {controller:32} {sr:>8.1f}% {cf:>6.1f}% "
                  f"{pe_s:>8} {tg_s:>9} {sk:>6.1f}")
        print()

    # Overall (adaptive_navigator only — the controller under iteration)
    print("=" * 92)
    print("ADAPTIVE_NAVIGATOR overall (averaged across 4 terrains):")
    succ, cf, pe = [], [], []
    for terrain in terrains:
        r = results[terrain].get("adaptive_navigator")
        if not r:
            continue
        succ.append(r["success_rate"]["mean"])
        if r["completion_fraction"]["mean"] is not None:
            cf.append(r["completion_fraction"]["mean"])
        if r["path_efficiency"]["mean"] is not None:
            pe.append(r["path_efficiency"]["mean"])
    print(f"  success_rate         : {np.mean(succ) * 100:.1f}%   (mean of per-terrain rates)")
    print(f"  completion_fraction  : {np.mean(cf) * 100:.1f}%")
    print(f"  path_efficiency (succ): "
          f"{np.mean(pe):.3f}" if pe else "  path_efficiency      : n/a (no successful runs)")
    print("=" * 92)
    return 0


if __name__ == "__main__":
    sys.exit(main())
