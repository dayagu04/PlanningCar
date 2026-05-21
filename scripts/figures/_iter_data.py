"""Shared loader for iter30 (latest accepted) experiment data.

Centralises the choice of "current" iteration so figure scripts don't drift.
Bump CURRENT_ITER when a new merge moves the state forward.
"""

import json
import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

CURRENT_ITER = 30
ITER_DIR = os.path.join(PROJECT_ROOT, "data", "experiments", f"iter{CURRENT_ITER}")
METRICS_PATH = os.path.join(PROJECT_ROOT, "results", "reports", "metrics_v2",
                            f"iter_{CURRENT_ITER:02d}.json")
DEFAULT_SEED = 42

TERRAIN_KEYS = ["flat", "slope", "rough", "transition"]
TERRAIN_CN = {
    "flat": "平坦 (Flat)",
    "slope": "斜坡 (Slope)",
    "rough": "凹凸 (Rough)",
    "transition": "过渡区 (Transition)",
}
TERRAIN_COLORS = {
    "flat": "#4CAF50",
    "slope": "#FF9800",
    "rough": "#F44336",
    "transition": "#9C27B0",
}
CONTROLLERS = ["adaptive_navigator", "adaptive_navigator_baseline", "astar_navigator"]
CONTROLLER_LABEL = {
    "adaptive_navigator": "Adaptive (Ours)",
    "adaptive_navigator_baseline": "Baseline (PD)",
    "astar_navigator": "A* Planning",
}
CONTROLLER_COLOR = {
    "adaptive_navigator": "#42A5F5",
    "adaptive_navigator_baseline": "#9E9E9E",
    "astar_navigator": "#FF7043",
}


def load_metrics(iter_num=CURRENT_ITER):
    path = os.path.join(PROJECT_ROOT, "results", "reports", "metrics_v2",
                        f"iter_{iter_num:02d}.json")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def trace_path(terrain, controller, seed=DEFAULT_SEED, iter_num=CURRENT_ITER):
    """Return absolute path to a single seed CSV from a given iter."""
    return os.path.join(PROJECT_ROOT, "data", "experiments", f"iter{iter_num}",
                        f"{terrain}_{controller}_seed{seed}.csv")


def metric_value(metrics, terrain, controller, key):
    """Return mean value, or None if n=0 (failed run)."""
    cell = metrics["results"][terrain][controller][key]
    return cell["mean"]
