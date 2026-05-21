"""Generate Figure 6-1: Slope terrain trajectory comparison (Adaptive vs A*).

Refreshed for iter30: adaptive_navigator now achieves 100% slope success after
iter22 (max-tilt slope rule) + iter25 (DWA horizon). Both controllers complete
the TSP tour; this figure highlights the path-efficiency / smoothness contrast.
"""

import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
from _iter_data import (CURRENT_ITER, DEFAULT_SEED, load_metrics, trace_path,
                        metric_value)

FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def _load_trace(terrain, controller, seed):
    return pd.read_csv(trace_path(terrain, controller, seed))


def main():
    metrics = load_metrics()
    terrain = "slope"

    df_adaptive = _load_trace(terrain, "adaptive_navigator", DEFAULT_SEED)
    df_astar = _load_trace(terrain, "astar_navigator", DEFAULT_SEED)

    eff_adapt = metric_value(metrics, terrain, "adaptive_navigator", "path_efficiency")
    eff_astar = metric_value(metrics, terrain, "astar_navigator", "path_efficiency")
    succ_adapt = metric_value(metrics, terrain, "adaptive_navigator", "success_rate")
    succ_astar = metric_value(metrics, terrain, "astar_navigator", "success_rate")
    len_adapt = metric_value(metrics, terrain, "adaptive_navigator", "actual_path_length")
    len_astar = metric_value(metrics, terrain, "astar_navigator", "actual_path_length")
    comp_adapt = metric_value(metrics, terrain, "adaptive_navigator",
                              "completion_fraction")
    comp_astar = metric_value(metrics, terrain, "astar_navigator",
                              "completion_fraction")

    fig, ax = plt.subplots(figsize=(11, 8))

    label_adapt = (f"Adaptive (Ours)  成功率 {succ_adapt*100:.0f}%, "
                   f"path_eff {eff_adapt:.2f}, 路径 {len_adapt:.1f}m")
    label_astar = (f"A* Planning      成功率 {succ_astar*100:.0f}%, "
                   f"完成度 {comp_astar*100:.0f}%, "
                   f"path_eff {eff_astar:.2f}")

    ax.plot(df_adaptive["x"], df_adaptive["y"],
            color="#1976D2", linewidth=2.2, alpha=0.9, label=label_adapt)
    ax.scatter(df_adaptive["x"].iloc[::20], df_adaptive["y"].iloc[::20],
               c="#1976D2", s=12, alpha=0.5, zorder=4)

    ax.plot(df_astar["x"], df_astar["y"],
            color="#E64A19", linewidth=1.8, alpha=0.85, label=label_astar,
            linestyle="--")
    ax.scatter(df_astar["x"].iloc[::20], df_astar["y"].iloc[::20],
               c="#E64A19", s=12, alpha=0.5, zorder=4)

    # Start
    ax.plot(0, 0, "k*", markersize=22, markeredgecolor="white",
            markeredgewidth=1.0, zorder=10, label="起点 Start")

    # End markers
    ax.plot(df_adaptive["x"].iloc[-1], df_adaptive["y"].iloc[-1],
            "^", color="#1976D2", markersize=14, markeredgecolor="black",
            markeredgewidth=1.0, zorder=10)
    ax.plot(df_astar["x"].iloc[-1], df_astar["y"].iloc[-1],
            "v", color="#E64A19", markersize=14, markeredgecolor="black",
            markeredgewidth=1.0, zorder=10)

    # Annotation block: key takeaway
    note = (f"iter{CURRENT_ITER} 状态:\n"
            f"- Adaptive 完成 6 路点 TSP 巡游\n"
            f"- 完成度 {comp_adapt*100:.0f}%, "
            f"成功率 {succ_adapt*100:.0f}%")
    ax.text(0.02, 0.98, note, transform=ax.transAxes,
            fontsize=9.5, color="#0D47A1", fontweight="bold",
            ha="left", va="top",
            bbox=dict(boxstyle="round,pad=0.4",
                      facecolor="#E3F2FD", edgecolor="#0D47A1"))

    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title(f"斜坡地形轨迹对比 (iter{CURRENT_ITER}, seed={DEFAULT_SEED})",
                 fontsize=12, fontweight="bold")
    ax.legend(fontsize=9, loc="lower right", facecolor="white",
              edgecolor="black")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图6-1_斜坡地形运动轨迹对比图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
