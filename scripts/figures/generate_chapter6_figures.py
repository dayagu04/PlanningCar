"""Generate Chapter 6 figures: 6-2 trajectory, 6-3 speed curves, 6-4 metrics.

Refreshed for iter30 — uses metrics_v2 aggregated across 5 seeds, with one
representative trace per terrain (seed=42) for trajectory/speed plots.
"""

import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
from _iter_data import (CURRENT_ITER, DEFAULT_SEED, TERRAIN_KEYS, TERRAIN_CN,
                        TERRAIN_COLORS, CONTROLLERS, CONTROLLER_LABEL,
                        CONTROLLER_COLOR, load_metrics, trace_path,
                        metric_value)

FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def load_adaptive(terrain):
    """Load adaptive_navigator trace for one terrain (seed=42, iter30)."""
    df = pd.read_csv(trace_path(terrain, "adaptive_navigator", DEFAULT_SEED))
    dt = df["time_s"].diff().fillna(0.032)
    df["speed_actual"] = np.sqrt(df["x"].diff() ** 2 + df["y"].diff() ** 2) / dt
    df["speed_actual"] = df["speed_actual"].clip(0, 15).fillna(0)
    return df


# ============================================================
# Figure 6-2: Four-terrain trajectory comparison
# ============================================================
def generate_fig6_2():
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    axes = axes.flatten()

    terrain_class_colors = {
        "flat": "#4CAF50", "slope": "#FF9800",
        "rough": "#F44336", "transition": "#9C27B0",
    }

    for idx, terrain in enumerate(TERRAIN_KEYS):
        ax = axes[idx]
        df = load_adaptive(terrain)

        for t_type, color in terrain_class_colors.items():
            mask = df["terrain"] == t_type
            if mask.any():
                ax.scatter(df.loc[mask, "x"], df.loc[mask, "y"],
                           c=color, s=8, alpha=0.7, label=t_type)

        ax.plot(df["x"].iloc[0], df["y"].iloc[0], "g*", markersize=14,
                markeredgecolor="black", markeredgewidth=0.8, label="起点", zorder=10)
        ax.plot(df["x"].iloc[-1], df["y"].iloc[-1], "r^", markersize=10,
                markeredgecolor="black", markeredgewidth=0.8, label="终点", zorder=10)

        ax.set_title(TERRAIN_CN[terrain], fontsize=11, fontweight="bold")
        ax.set_xlabel("X (m)", fontsize=9)
        ax.set_ylabel("Y (m)", fontsize=9)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc="upper right", ncol=2)

    fig.suptitle(f"四种地形 Adaptive Navigator 轨迹 (iter{CURRENT_ITER}, "
                 f"seed={DEFAULT_SEED})", fontsize=12, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    save_path = os.path.join(FIGURES_DIR, "图6-2_四种地形轨迹对比.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


# ============================================================
# Figure 6-3: Four-terrain speed curves
# ============================================================
def generate_fig6_3():
    fig, ax = plt.subplots(figsize=(12, 5.5))

    for terrain in TERRAIN_KEYS:
        df = load_adaptive(terrain)
        speed_smooth = df["speed_actual"].rolling(window=5, min_periods=1).mean()
        ax.plot(df["time_s"], speed_smooth,
                color=TERRAIN_COLORS[terrain], linewidth=1.8, alpha=0.85,
                label=TERRAIN_CN[terrain])

    ax.set_xlabel("时间 (s) / Time (s)", fontsize=11)
    ax.set_ylabel("实际速度 (m/s) / Actual Speed (m/s)", fontsize=11)
    ax.legend(fontsize=10, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    ax.set_title(f"四种地形速度响应曲线 (iter{CURRENT_ITER}, seed={DEFAULT_SEED})",
                 fontsize=12, fontweight="bold")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图6-3_四种地形速度曲线对比.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


# ============================================================
# Figure 6-4: Aggregated metrics bar chart (3 controllers x 4 terrains)
# ============================================================
def generate_fig6_4():
    metrics = load_metrics()

    panels = [
        ("success_rate",        "成功率\nSuccess Rate",    False, 1.0),
        ("completion_fraction", "完成度\nCompletion Frac.", False, 1.0),
        ("path_efficiency",     "路径效率\nPath Efficiency", False, 1.0),
        ("classification_accuracy", "分类准确率\nClass. Acc.", False, 1.0),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9.5))
    axes = axes.flatten()

    x = np.arange(len(TERRAIN_KEYS))
    width = 0.26

    for idx, (key, ylabel, use_log, ymax) in enumerate(panels):
        ax = axes[idx]

        for i, ctrl in enumerate(CONTROLLERS):
            values = []
            for terrain in TERRAIN_KEYS:
                v = metric_value(metrics, terrain, ctrl, key)
                values.append(v if v is not None else 0.0)

            offset = (i - 1) * width
            bars = ax.bar(x + offset, values, width,
                          label=CONTROLLER_LABEL[ctrl],
                          color=CONTROLLER_COLOR[ctrl], alpha=0.88,
                          edgecolor="black", linewidth=0.5)
            for j, v in enumerate(values):
                if v > 0.01:
                    ax.text(x[j] + offset, v + 0.02, f"{v:.2f}",
                            ha="center", va="bottom", fontsize=7)

        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_xticks(x)
        ax.set_xticklabels([t.title() for t in TERRAIN_KEYS], fontsize=10)
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, alpha=0.3, axis="y")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.set_ylim(0, ymax * 1.15)

    fig.suptitle(f"三控制器 × 四地形 综合指标 (iter{CURRENT_ITER}, 5 seeds 均值)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    save_path = os.path.join(FIGURES_DIR, "图6-4_算法对比指标柱状图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")

    # Refresh companion CSV in same directory
    rows = [["Terrain", "Controller", "success_rate", "completion_fraction",
             "path_efficiency", "time_to_goal", "classification_accuracy",
             "actual_path_length", "replan_count", "n_seeds"]]
    for terrain in TERRAIN_KEYS:
        for ctrl in CONTROLLERS:
            cell = metrics["results"][terrain][ctrl]
            rows.append([
                terrain.title(), CONTROLLER_LABEL[ctrl],
                cell["success_rate"]["mean"],
                cell["completion_fraction"]["mean"],
                cell["path_efficiency"]["mean"],
                cell["time_to_goal"]["mean"],
                cell["classification_accuracy"]["mean"],
                cell["actual_path_length"]["mean"],
                cell["replan_count"]["mean"],
                cell["success_rate"]["n"],
            ])
    csv_path = os.path.join(FIGURES_DIR, "表6-1_算法对比指标.csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(",".join("" if v is None else f"{v}" for v in r) + "\n")
    print(f"[OK] {csv_path}")


def main():
    print(f"生成第6章图表 (iter{CURRENT_ITER})...\n")
    generate_fig6_2()
    generate_fig6_3()
    generate_fig6_4()
    print(f"\n所有图表已保存到: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
