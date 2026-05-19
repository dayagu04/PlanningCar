"""Python vs C++ benchmark for the core navigation algorithms.

Runs each algorithm in two configurations:
  - **Python**: pure-Python reference implementations in `*_py.py`
  - **C++**:    pybind11-wrapped versions exposed by `src.nav_core_cpp`

Reports wall-clock time, throughput, and speedup; writes a CSV to
`results/cpp_vs_python_benchmark.csv` and prints a Markdown table that
can be pasted into the thesis.

Run from the project root:

    python scripts/experiments/benchmark_cpp_vs_python.py
"""

import csv
import os
import random
import sys
import time
from statistics import mean

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# Current (C++ backed) modules
from src.perception import terrain_features as tf_cpp
from src.classification.rule_classifier import TerrainClassifier as ClassifierCpp
from src.planning.astar import AStarPlanner as PlannerCpp
from src.planning.tsp_solver import optimize_waypoint_order as tsp_cpp

# Reference (pure-Python) modules
from src.perception import terrain_features_py as tf_py
from src.classification.rule_classifier_py import TerrainClassifier as ClassifierPy
from src.planning.astar_py import AStarPlanner as PlannerPy
from src.planning.tsp_solver_py import optimize_waypoint_order as tsp_py


# ---------------------------------------------------------------------------
# Bench helpers
# ---------------------------------------------------------------------------

def time_it(fn, repeats: int) -> float:
    """Median wall-clock seconds across `repeats` runs (skips the first as warm-up)."""
    samples = []
    for _ in range(repeats + 1):
        t0 = time.perf_counter()
        fn()
        samples.append(time.perf_counter() - t0)
    samples.sort()
    # drop fastest+slowest as outliers when there are enough samples
    if len(samples) >= 5:
        samples = samples[1:-1]
    return mean(samples)


def fmt_time(seconds: float) -> str:
    if seconds < 1e-3:
        return f"{seconds * 1e6:.1f} μs"
    if seconds < 1:
        return f"{seconds * 1e3:.2f} ms"
    return f"{seconds:.3f} s"


# ---------------------------------------------------------------------------
# Benchmark cases
# ---------------------------------------------------------------------------

def bench_extract_features(repeats=20):
    rng = np.random.RandomState(42)
    grids = [rng.rand(64, 64) * 0.2 for _ in range(50)]

    def run_py():
        for g in grids:
            tf_py.extract_features(g, 0.1)

    def run_cpp():
        for g in grids:
            tf_cpp.extract_features(g, 0.1)

    return ("extract_features (50× 64×64 grids)",
            time_it(run_py, repeats),
            time_it(run_cpp, repeats))


def bench_classifier(repeats=20):
    rng = np.random.RandomState(0)
    samples = [(rng.uniform(0, 20),    # slope_deg
                rng.uniform(0, 0.15),  # roughness
                rng.uniform(0, 10),    # imu_pitch
                rng.uniform(0, 5))     # imu_roll
               for _ in range(5000)]

    def run_py():
        clf = ClassifierPy()
        for s, r, p, ro in samples:
            clf.classify({"slope_deg": s, "roughness": r,
                          "imu_pitch_deg": p, "imu_roll_deg": ro})

    def run_cpp():
        clf = ClassifierCpp()
        for s, r, p, ro in samples:
            clf.classify({"slope_deg": s, "roughness": r,
                          "imu_pitch_deg": p, "imu_roll_deg": ro})

    return ("TerrainClassifier (5000 samples)",
            time_it(run_py, repeats),
            time_it(run_cpp, repeats))


def bench_astar(repeats=5):
    """A* on a 40×40 grid with 5 random obstacles."""
    rng = random.Random(7)
    starts = [(rng.uniform(-9, -7), rng.uniform(-9, -7)) for _ in range(20)]
    goals  = [(rng.uniform(7, 9),   rng.uniform(7, 9))   for _ in range(20)]
    obstacles = [(rng.uniform(-3, 3), rng.uniform(-3, 3), 0.8) for _ in range(5)]

    def run_py():
        p = PlannerPy(grid_size=0.5, world_size=20.0)
        for ox, oy, r in obstacles:
            p.set_obstacle(ox, oy, r)
        for s, g in zip(starts, goals):
            p.plan(s, g)

    def run_cpp():
        p = PlannerCpp(grid_size=0.5, world_size=20.0)
        for ox, oy, r in obstacles:
            p.set_obstacle(ox, oy, r)
        for s, g in zip(starts, goals):
            p.plan(s, g)

    return ("A* (20 plans on 40×40 grid, 5 obstacles)",
            time_it(run_py, repeats),
            time_it(run_cpp, repeats))


def bench_astar_large(repeats=3):
    """A* on a 100×100 grid — closer to thesis-scale workloads."""
    rng = random.Random(11)
    starts = [(rng.uniform(-23, -20), rng.uniform(-23, -20)) for _ in range(10)]
    goals  = [(rng.uniform(20, 23),   rng.uniform(20, 23))   for _ in range(10)]
    obstacles = [(rng.uniform(-10, 10), rng.uniform(-10, 10), 1.2) for _ in range(10)]

    def run_py():
        p = PlannerPy(grid_size=0.5, world_size=50.0)
        for ox, oy, r in obstacles:
            p.set_obstacle(ox, oy, r)
        for s, g in zip(starts, goals):
            p.plan(s, g)

    def run_cpp():
        p = PlannerCpp(grid_size=0.5, world_size=50.0)
        for ox, oy, r in obstacles:
            p.set_obstacle(ox, oy, r)
        for s, g in zip(starts, goals):
            p.plan(s, g)

    return ("A* (10 plans on 100×100 grid, 10 obstacles)",
            time_it(run_py, repeats),
            time_it(run_cpp, repeats))


def bench_tsp(repeats=5):
    rng = random.Random(3)
    waypoints_set = [
        [(rng.uniform(-10, 10), rng.uniform(-10, 10)) for _ in range(n)]
        for n in (8, 12, 16)
    ]

    def run_py():
        for wp in waypoints_set:
            tsp_py((0.0, 0.0), wp)

    def run_cpp():
        for wp in waypoints_set:
            tsp_cpp((0.0, 0.0), wp)

    return ("TSP (greedy NN + 2-opt, 8/12/16 waypoints)",
            time_it(run_py, repeats),
            time_it(run_cpp, repeats))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Python vs C++ benchmark — navigation core")
    print("=" * 70)

    benches = [
        bench_extract_features,
        bench_classifier,
        bench_astar,
        bench_astar_large,
        bench_tsp,
    ]

    rows = []
    for b in benches:
        name, t_py, t_cpp = b()
        speedup = t_py / t_cpp if t_cpp > 0 else float("inf")
        rows.append({
            "case": name,
            "python_seconds": t_py,
            "cpp_seconds": t_cpp,
            "speedup": speedup,
        })
        print(f"\n{name}")
        print(f"  Python : {fmt_time(t_py)}")
        print(f"  C++    : {fmt_time(t_cpp)}")
        print(f"  Speedup: {speedup:6.2f}×")

    # CSV
    out_dir = os.path.join(PROJECT_ROOT, "results")
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "cpp_vs_python_benchmark.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["case", "python_seconds", "cpp_seconds", "speedup"])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"\nCSV saved to: {csv_path}")

    # Markdown table for the thesis
    print("\n--- Markdown ---")
    print("| 算法 | Python (s) | C++ (s) | 加速比 |")
    print("|------|-----------:|--------:|------:|")
    for r in rows:
        print(f"| {r['case']} | {r['python_seconds']:.4f} | {r['cpp_seconds']:.4f} | {r['speedup']:.2f}× |")


if __name__ == "__main__":
    main()
