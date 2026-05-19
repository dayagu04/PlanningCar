"""Generate Figure 6-1: Slope terrain trajectory comparison (Adaptive vs A*)."""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "data", "experiments")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def main():
    adaptive_path = os.path.join(DATA_DIR, "slope_terrain_adaptive_navigator.csv")
    astar_path = os.path.join(DATA_DIR, "slope_terrain_astar_navigator.csv")

    df_adaptive = pd.read_csv(adaptive_path)
    df_astar = pd.read_csv(astar_path)

    fig, ax = plt.subplots(figsize=(11, 8))

    # A* trajectory (smooth)
    ax.plot(df_astar["x"], df_astar["y"],
            color="#2E7D32", linewidth=2.5, alpha=0.9,
            label=f"A* 算法 (总路径 {12.97:.2f} m)")
    ax.scatter(df_astar["x"].iloc[::5], df_astar["y"].iloc[::5],
               c="#2E7D32", s=15, alpha=0.5, zorder=4)

    # Adaptive trajectory (chaotic - real 390m drift)
    ax.plot(df_adaptive["x"], df_adaptive["y"],
            color="#C62828", linewidth=1.5, alpha=0.75,
            label=f"Adaptive 算法 (失稳, 总路径 {390.62:.2f} m)")

    # Start point (origin)
    ax.plot(0, 0, "b*", markersize=22, markeredgecolor="black",
            markeredgewidth=1.2, zorder=10, label="起点 Start")

    # End points
    ax.plot(df_astar["x"].iloc[-1], df_astar["y"].iloc[-1],
            "g^", markersize=14, markeredgecolor="black",
            markeredgewidth=1.0, zorder=10)
    ax.text(df_astar["x"].iloc[-1] + 0.3, df_astar["y"].iloc[-1],
            "A* 终点", fontsize=9, color="#2E7D32", fontweight="bold")

    ax.plot(df_adaptive["x"].iloc[-1], df_adaptive["y"].iloc[-1],
            "rv", markersize=14, markeredgecolor="black",
            markeredgewidth=1.0, zorder=10)
    ax.text(df_adaptive["x"].iloc[-1] + 0.3, df_adaptive["y"].iloc[-1],
            "Adaptive 终点", fontsize=9, color="#C62828", fontweight="bold")

    # Annotation for key finding
    # Find rough zone of adaptive trajectory
    adaptive_x_range = df_adaptive["x"].max() - df_adaptive["x"].min()
    adaptive_y_range = df_adaptive["y"].max() - df_adaptive["y"].min()
    ax.annotate(f"剧烈滑移与振荡\n姿态偏差 112.04°\n路径长度 ×30",
                xy=(df_adaptive["x"].mean(), df_adaptive["y"].min()),
                xytext=(df_adaptive["x"].mean() + 2,
                        df_adaptive["y"].min() - 2),
                fontsize=10, color="#C62828", fontweight="bold",
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#C62828"),
                bbox=dict(boxstyle="round,pad=0.4",
                          facecolor="#FFEBEE", edgecolor="#C62828"))

    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.legend(fontsize=10, loc="upper left", facecolor="white",
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
