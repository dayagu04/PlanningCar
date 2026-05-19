"""Generate Chapter 6 figures: 6-2 trajectory, 6-3 speed curves, 6-4 metrics bar chart."""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import LogLocator, ScalarFormatter

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)

DATA_DIR = os.path.join(PROJECT_ROOT, "data", "experiments")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300

TERRAINS = ["flat_terrain", "slope_terrain", "rough_terrain", "transition_terrain"]
TERRAIN_CN = {
    "flat_terrain": "平坦 (Flat)",
    "slope_terrain": "斜坡 (Slope)",
    "rough_terrain": "凹凸 (Rough)",
    "transition_terrain": "过渡区 (Transition)",
}
TERRAIN_COLORS = {
    "flat_terrain": "#4CAF50",
    "slope_terrain": "#FF9800",
    "rough_terrain": "#F44336",
    "transition_terrain": "#9C27B0",
}


def load_adaptive(terrain):
    path = os.path.join(DATA_DIR, f"{terrain}_adaptive_navigator.csv")
    df = pd.read_csv(path)
    dt = df["time_s"].diff().fillna(0.032)
    df["speed_actual"] = np.sqrt(df["x"].diff()**2 + df["y"].diff()**2) / dt
    df["speed_actual"] = df["speed_actual"].clip(0, 15).fillna(0)
    return df


# ============================================================
# Figure 6-2: Four terrain trajectory comparison
# ============================================================
def generate_fig6_2():
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    axes = axes.flatten()

    for idx, terrain in enumerate(TERRAINS):
        ax = axes[idx]
        df = load_adaptive(terrain)

        # Color by terrain classification
        terrain_type_colors = {
            "flat": "#4CAF50", "slope": "#FF9800",
            "rough": "#F44336", "transition": "#9C27B0"
        }
        for t_type, color in terrain_type_colors.items():
            mask = df["terrain"] == t_type
            if mask.any():
                ax.scatter(df.loc[mask, "x"], df.loc[mask, "y"],
                           c=color, s=8, alpha=0.7, label=t_type)

        # Start and end markers
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

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    save_path = os.path.join(FIGURES_DIR, "图6-2_四种地形轨迹对比.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


# ============================================================
# Figure 6-3: Four terrain speed curves comparison
# ============================================================
def generate_fig6_3():
    fig, ax = plt.subplots(figsize=(12, 5.5))

    for terrain in TERRAINS:
        df = load_adaptive(terrain)
        # Smooth with rolling window
        speed_smooth = df["speed_actual"].rolling(window=5, min_periods=1).mean()
        ax.plot(df["time_s"], speed_smooth,
                color=TERRAIN_COLORS[terrain], linewidth=1.8, alpha=0.85,
                label=TERRAIN_CN[terrain])

    ax.set_xlabel("时间 (s) / Time (s)", fontsize=11)
    ax.set_ylabel("实际速度 (m/s) / Actual Speed (m/s)", fontsize=11)
    ax.legend(fontsize=10, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # Annotate speed adaptation
    ax.axhline(y=0.4, color="#4CAF50", linestyle="--", alpha=0.4, linewidth=1)
    ax.text(1, 0.45, "Flat ~0.4 m/s", fontsize=8, color="#4CAF50")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图6-3_四种地形速度曲线对比.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


# ============================================================
# Figure 6-4: Algorithm comparison bar chart (2x2 subplots)
# ============================================================
def generate_fig6_4():
    csv_path = os.path.join(FIGURES_DIR, "表6-1_算法对比指标.csv")
    df = pd.read_csv(csv_path)

    metrics = [
        ("avg_speed", "平均速度 (m/s)\nAvg Speed", False),
        ("attitude_stability", "姿态稳定性 (deg)\nAttitude Stability", True),
        ("path_tracking_error", "跟踪误差 (m)\nTracking Error", True),
        ("total_distance", "总路径 (m)\nTotal Distance", True),
    ]

    terrains_order = ["Flat", "Slope", "Rough", "Transition"]
    algorithms = ["Adaptive (Ours)", "A* Planning"]
    algo_colors = {"Adaptive (Ours)": "#42A5F5", "A* Planning": "#FF7043"}

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    axes = axes.flatten()

    x = np.arange(len(terrains_order))
    width = 0.35

    for idx, (metric_col, ylabel, use_log) in enumerate(metrics):
        ax = axes[idx]

        for i, algo in enumerate(algorithms):
            values = []
            for terrain in terrains_order:
                row = df[(df["Terrain"] == terrain) & (df["Algorithm"] == algo)]
                if not row.empty:
                    values.append(row[metric_col].values[0])
                else:
                    values.append(0)

            offset = -width/2 + i * width
            bars = ax.bar(x + offset, values, width, label=algo,
                          color=algo_colors[algo], alpha=0.85,
                          edgecolor="black", linewidth=0.6)

            # Value labels
            for j, v in enumerate(values):
                if v > 0:
                    if use_log and v > 100:
                        # Skip labels for extreme outliers on log scale
                        pass
                    elif use_log:
                        ax.text(x[j] + offset, v * 1.15, f"{v:.1f}",
                                ha="center", va="bottom", fontsize=6.5)
                    else:
                        ax.text(x[j] + offset, v + max(values)*0.03, f"{v:.2f}",
                                ha="center", va="bottom", fontsize=7)

        if use_log:
            ax.set_yscale("log")
            ax.yaxis.set_major_formatter(ScalarFormatter())
            ax.yaxis.get_major_formatter().set_scientific(False)

        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels(terrains_order, fontsize=9)
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, alpha=0.3, axis="y")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

        # Annotate Slope anomaly below the legend
        if metric_col in ("attitude_stability", "total_distance"):
            ax.text(0.98, 0.85, "Slope/Adaptive\n异常(失稳滑移)",
                    transform=ax.transAxes, fontsize=7, color="#C62828",
                    ha="right", va="top", fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="#FFCDD2",
                              edgecolor="#C62828", alpha=0.8))

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    save_path = os.path.join(FIGURES_DIR, "图6-4_算法对比指标柱状图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


def main():
    print("生成第6章图表...\n")
    generate_fig6_2()
    generate_fig6_3()
    generate_fig6_4()
    print(f"\n所有图表已保存到: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
