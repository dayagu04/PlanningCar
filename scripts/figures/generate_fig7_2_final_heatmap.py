"""Generate Figure 7-2: Final-state heatmap (iter30, 3 controllers x 4 terrains).

Six small panels (one per metric) — each is a 3 x 4 cell heatmap.
Shows the final adoption-ready snapshot of the controller stack vs the two
baselines. Companion figure to 6-4 (which shows it as bars).
"""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
from _iter_data import (CURRENT_ITER, TERRAIN_KEYS, CONTROLLERS,
                        CONTROLLER_LABEL, load_metrics, metric_value)

FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def heatmap_panel(ax, matrix, title, fmt="{:.2f}", cmap="RdYlGn", vmin=0, vmax=1):
    im = ax.imshow(matrix, cmap=cmap, vmin=vmin, vmax=vmax, aspect="auto")
    ax.set_xticks(range(len(TERRAIN_KEYS)))
    ax.set_xticklabels([t.title() for t in TERRAIN_KEYS], fontsize=9)
    ax.set_yticks(range(len(CONTROLLERS)))
    ax.set_yticklabels([CONTROLLER_LABEL[c] for c in CONTROLLERS], fontsize=9)
    ax.set_title(title, fontsize=10, fontweight="bold")
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            v = matrix[i, j]
            if np.isnan(v):
                txt = "—"
            else:
                txt = fmt.format(v)
            color = "black" if (vmin + vmax) * 0.4 < v < vmax * 0.85 else "white"
            ax.text(j, i, txt, ha="center", va="center", fontsize=9,
                    fontweight="bold", color=color)
    return im


def collect(metrics, key):
    rows = []
    for ctrl in CONTROLLERS:
        row = []
        for terrain in TERRAIN_KEYS:
            v = metric_value(metrics, terrain, ctrl, key)
            row.append(np.nan if v is None else v)
        rows.append(row)
    return np.array(rows, dtype=float)


def main():
    metrics = load_metrics()

    panels = [
        ("success_rate", "成功率 Success Rate", "{:.2f}", "RdYlGn", 0, 1),
        ("completion_fraction", "完成度 Completion", "{:.2f}", "RdYlGn", 0, 1),
        ("path_efficiency", "路径效率 Path Eff.", "{:.2f}", "RdYlGn", 0, 1),
        ("classification_accuracy", "分类准确率 Class. Acc.", "{:.2f}",
         "RdYlGn", 0, 1),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axes = axes.flatten()

    for idx, (key, title, fmt, cmap, vmin, vmax) in enumerate(panels):
        ax = axes[idx]
        m = collect(metrics, key)
        im = heatmap_panel(ax, m, title, fmt=fmt, cmap=cmap, vmin=vmin, vmax=vmax)
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.suptitle(f"Figure 7-2  iter{CURRENT_ITER} 终态: 3 控制器 × 4 地形指标矩阵 "
                 f"(5 seeds 均值)", fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    save_path = os.path.join(FIGURES_DIR, "图7-2_终态指标热力图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
