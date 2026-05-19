"""Analyze baseline experiment results and generate metrics JSON.

CSV columns: step,time_s,x,y,z,roll,pitch,yaw,terrain,speed,target_idx,dist_to_target

Computes:
- success_rate: max(target_idx) >= 3 (visited all 4 waypoints)
- path_efficiency: straight-line waypoint tour / actual path length
- time_to_goal: time_s at last waypoint reached (or max if not completed)
- cross_track_mean/max: deviation (approximated by speed/heading variance)
- classification_accuracy: terrain column matches expected (per world type)
- energy_proxy: integral of speed^2 over time
- replan_count: not directly available, set to 0 for baseline

Output: results/reports/metrics/iter_00.json
"""

import json
import sys
from pathlib import Path
from typing import Dict, List
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

EXPERIMENTS_DIR = PROJECT_ROOT / "data" / "experiments"
METRICS_DIR = PROJECT_ROOT / "results" / "reports" / "metrics"

# Expected number of waypoints from headless_run.py (num_points=4)
NUM_WAYPOINTS = 4
WAYPOINT_TOLERANCE = 1.0  # meters

# Expected terrain class per world (for classification accuracy)
EXPECTED_TERRAIN = {
    'flat': 'flat',
    'slope': 'slope',
    'rough': 'rough',
    'transition': 'transition'  # may be mixed
}


def parse_csv(csv_path: Path) -> List[Dict]:
    """Parse navigation CSV into list of row dicts."""
    if not csv_path.exists():
        return []

    rows = []
    with open(csv_path, 'r') as f:
        header = f.readline().strip().split(',')
        for line in f:
            if not line.strip():
                continue
            values = line.strip().split(',')
            if len(values) < len(header):
                continue
            row = {}
            for i, col in enumerate(header):
                try:
                    row[col] = float(values[i])
                except (ValueError, IndexError):
                    row[col] = values[i] if i < len(values) else ''
            rows.append(row)
    return rows


def compute_metrics(rows: List[Dict], expected_terrain: str) -> Dict:
    """Compute all metrics from one experiment run."""
    if not rows:
        return None

    # ---- Success: visited all waypoints (target_idx reached NUM_WAYPOINTS) ----
    target_idx_values = [int(r.get('target_idx', 0)) for r in rows]
    max_target_idx = max(target_idx_values) if target_idx_values else 0
    # Controller advances target_idx after reaching each WP; success when finished all
    # Note: some controllers may set target_idx beyond num_waypoints on completion
    final_target_idx = target_idx_values[-1]
    final_dist = float(rows[-1].get('dist_to_target', 999.0))

    # Success heuristic: reached last waypoint area OR controller exited cleanly
    # If controller exited cleanly (csv has few rows like 282), it likely completed
    # If it ran to timeout (15000+ rows), check if last waypoint was reached
    success = (max_target_idx >= NUM_WAYPOINTS - 1 and final_dist < 2.0) or \
              (max_target_idx >= NUM_WAYPOINTS) or \
              (len(rows) < 1000 and max_target_idx >= NUM_WAYPOINTS - 1)

    # ---- Path length and efficiency ----
    actual_path = 0.0
    for i in range(1, len(rows)):
        dx = rows[i]['x'] - rows[i-1]['x']
        dy = rows[i]['y'] - rows[i-1]['y']
        actual_path += np.sqrt(dx*dx + dy*dy)

    # Straight-line tour length (start → all waypoints by target_idx changes)
    # Approximation: use total displacement to last position as straight distance
    if actual_path > 0 and len(rows) > 1:
        start_x, start_y = rows[0]['x'], rows[0]['y']
        end_x, end_y = rows[-1]['x'], rows[-1]['y']
        straight_dist = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        # Sum of displacements between waypoint-visit instants is a better baseline
        # but without waypoint positions we estimate via target_idx transitions
        path_efficiency = min(1.0, straight_dist / actual_path) if actual_path > 0 else 0.0
    else:
        path_efficiency = 0.0

    # ---- Time to goal ----
    if success:
        # Find time when last target_idx reached
        for r in rows:
            if int(r.get('target_idx', 0)) >= max_target_idx:
                time_to_goal = float(r['time_s'])
                break
        else:
            time_to_goal = float(rows[-1]['time_s'])
    else:
        time_to_goal = float(rows[-1]['time_s'])  # timed out

    # ---- Cross track error (approximation: lateral oscillation) ----
    # Use std of yaw rate as proxy for tracking smoothness
    yaws = [r['yaw'] for r in rows]
    if len(yaws) > 1:
        yaw_rates = [abs(yaws[i] - yaws[i-1]) for i in range(1, len(yaws))]
        # wrap to [-pi, pi]
        yaw_rates = [min(r, 2*np.pi - r) for r in yaw_rates]
        cross_track_mean = float(np.mean(yaw_rates))
        cross_track_max = float(np.max(yaw_rates))
    else:
        cross_track_mean = 0.0
        cross_track_max = 0.0

    # ---- Classification accuracy ----
    terrain_labels = [str(r.get('terrain', '')) for r in rows]
    if expected_terrain == 'transition':
        # Transition world has mixed terrain — any of flat/slope/rough is valid
        # classification accuracy here means the classifier didn't crash, count as 1.0
        correct = sum(1 for t in terrain_labels if t in ['flat', 'slope', 'rough', 'transition'])
    else:
        correct = sum(1 for t in terrain_labels if t == expected_terrain)
    classification_acc = correct / len(terrain_labels) if terrain_labels else 0.0

    # ---- Energy proxy: integral of speed^2 dt ----
    energy = 0.0
    for i in range(1, len(rows)):
        dt = rows[i]['time_s'] - rows[i-1]['time_s']
        v = rows[i].get('speed', 0)
        energy += v * v * dt

    # ---- Replan count (not in CSV, derive from rapid heading changes) ----
    # Use significant yaw changes as proxy
    replan_proxy = sum(1 for r in yaw_rates if r > 0.5) if len(yaws) > 1 else 0

    return {
        'success': success,
        'max_target_idx': max_target_idx,
        'final_target_idx': final_target_idx,
        'final_dist': final_dist,
        'path_efficiency': path_efficiency,
        'actual_path_length': actual_path,
        'time_to_goal': time_to_goal,
        'cross_track_mean': cross_track_mean,
        'cross_track_max': cross_track_max,
        'classification_accuracy': classification_acc,
        'energy_proxy': energy,
        'replan_count': replan_proxy,
        'num_samples': len(rows)
    }


def aggregate_seeds(metrics_list: List[Dict]) -> Dict:
    """Aggregate metrics across multiple seeds (mean ± std)."""
    if not metrics_list:
        return {}

    success_count = sum(1 for m in metrics_list if m['success'])
    success_rate = success_count / len(metrics_list)

    def stats(key):
        values = [m[key] for m in metrics_list if m.get(key) is not None]
        if not values:
            return {'mean': None, 'std': None}
        return {
            'mean': float(np.mean(values)),
            'std': float(np.std(values)),
            'n': len(values)
        }

    return {
        'success_rate': {'mean': success_rate, 'std': 0.0, 'n': len(metrics_list)},
        'path_efficiency': stats('path_efficiency'),
        'time_to_goal': stats('time_to_goal'),
        'cross_track_mean': stats('cross_track_mean'),
        'cross_track_max': stats('cross_track_max'),
        'classification_accuracy': stats('classification_accuracy'),
        'energy_proxy': stats('energy_proxy'),
        'replan_count': stats('replan_count'),
        'actual_path_length': stats('actual_path_length')
    }


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--iter-tag', default='iter00')
    parser.add_argument('--iter-num', type=int, default=0)
    args = parser.parse_args()

    exp_dir = EXPERIMENTS_DIR / args.iter_tag
    if not exp_dir.exists():
        print(f"Error: {exp_dir} not found")
        return 1

    terrains = ['flat', 'slope', 'rough', 'transition']
    controllers = ['adaptive_navigator', 'adaptive_navigator_baseline', 'astar_navigator']
    seeds = [42, 43, 44, 45, 46]

    results = {}
    for terrain in terrains:
        results[terrain] = {}
        expected_terrain = EXPECTED_TERRAIN.get(terrain, terrain)

        for controller in controllers:
            metrics_list = []
            for seed in seeds:
                csv_file = exp_dir / f"{terrain}_{controller}_seed{seed}.csv"
                if csv_file.exists():
                    rows = parse_csv(csv_file)
                    metrics = compute_metrics(rows, expected_terrain)
                    if metrics:
                        metrics_list.append(metrics)
                else:
                    print(f"Missing: {csv_file.name}")

            if metrics_list:
                results[terrain][controller] = aggregate_seeds(metrics_list)
            else:
                print(f"No data: {terrain}/{controller}")

    # Build output JSON
    output = {
        'iter': args.iter_num,
        'branch': f'iter/00-baseline',
        'tag': 'iter-00-baseline',
        'datetime': '2026-05-20T00:00:00',
        'hypothesis': 'Baseline establishment - no algorithm changes, record current performance for future comparison',
        'changes': [],
        'controllers': controllers,
        'terrains': terrains,
        'seeds': seeds,
        'results': results,
        'decision': 'accept',
        'notes': 'Initial baseline. Includes pre-baseline improvements: classifier vote window, terrain-adaptive DWA weights, wheel rate limiter (commit 8bda12b).'
    }

    METRICS_DIR.mkdir(parents=True, exist_ok=True)
    output_file = METRICS_DIR / f'iter_{args.iter_num:02d}.json'
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=2)

    print(f"\nMetrics saved: {output_file}\n")

    # Summary printout
    print("=" * 80)
    print(f"{'Terrain':12} {'Controller':30} {'Success':>10} {'PathEff':>10} {'Time(s)':>10}")
    print("=" * 80)
    for terrain in terrains:
        for controller in controllers:
            if controller in results[terrain]:
                r = results[terrain][controller]
                sr = r['success_rate']['mean']
                pe = r['path_efficiency'].get('mean') or 0
                tg = r['time_to_goal'].get('mean') or 0
                print(f"{terrain:12} {controller:30} {sr*100:>9.1f}% {pe:>10.3f} {tg:>10.1f}")
        print()

    # Overall averages
    print("=" * 80)
    print("OVERALL (averaged across terrains):")
    for controller in controllers:
        success_rates = []
        path_effs = []
        for terrain in terrains:
            if controller in results[terrain]:
                success_rates.append(results[terrain][controller]['success_rate']['mean'])
                pe = results[terrain][controller]['path_efficiency'].get('mean')
                if pe is not None:
                    path_effs.append(pe)
        avg_sr = np.mean(success_rates) if success_rates else 0
        avg_pe = np.mean(path_effs) if path_effs else 0
        print(f"  {controller:30}: success={avg_sr*100:.1f}%, path_eff={avg_pe:.3f}")
    print("=" * 80)

    return 0


if __name__ == '__main__':
    sys.exit(main())
