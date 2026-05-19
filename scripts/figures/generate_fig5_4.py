"""Generate Figure 5-4: TSP optimization before/after comparison."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300

from src.planning.tsp_solver import optimize_waypoint_order, tour_length


def plot_tour(ax, start, tour, title, color, total_len):
    """Plot a tour with arrows between waypoints."""
    all_points = [start] + list(tour)

    # Draw path with arrows
    for i in range(len(all_points) - 1):
        p1 = all_points[i]
        p2 = all_points[i + 1]
        ax.annotate("", xy=p2, xytext=p1,
                    arrowprops=dict(arrowstyle="-|>", lw=1.5, color=color, alpha=0.7))

    # Draw waypoints
    wx = [p[0] for p in tour]
    wy = [p[1] for p in tour]
    ax.scatter(wx, wy, c="royalblue", s=120, zorder=5, edgecolors="black", linewidths=1.2)
    for i, (x, y) in enumerate(tour):
        ax.text(x + 0.3, y + 0.3, f"W{i+1}", fontsize=9, fontweight="bold", color="navy")

    # Draw start
    ax.scatter([start[0]], [start[1]], c="red", s=180, zorder=6,
              marker="*", edgecolors="darkred", linewidths=1)
    ax.text(start[0] + 0.3, start[1] - 0.5, "起点\nStart", fontsize=8,
            color="red", fontweight="bold")

    # Order labels on path
    for i in range(len(all_points) - 1):
        mx = (all_points[i][0] + all_points[i+1][0]) / 2
        my = (all_points[i][1] + all_points[i+1][1]) / 2
        ax.text(mx, my, str(i+1), fontsize=7, ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white",
                          edgecolor=color, alpha=0.8))

    ax.set_title(f"{title}\n总路径: {total_len:.2f} m", fontsize=11, fontweight="bold")
    ax.set_xlabel("X (m)", fontsize=10)
    ax.set_ylabel("Y (m)", fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")
    ax.set_xlim(-14, 14)
    ax.set_ylim(-14, 14)


def main():
    # Use a fixed seed for reproducibility
    np.random.seed(42)

    start = (0.0, 0.0)
    # Generate 6 waypoints that will show clear crossing in random order
    waypoints = [
        (8.0, 6.0),
        (-7.0, 9.0),
        (10.0, -5.0),
        (-9.0, -7.0),
        (3.0, -10.0),
        (-5.0, 3.0),
    ]

    # Random (bad) order — deliberately create crossings
    bad_order = [waypoints[0], waypoints[3], waypoints[1], waypoints[4], waypoints[2], waypoints[5]]
    bad_length = tour_length(start, bad_order)

    # Optimized order
    optimized, info = optimize_waypoint_order(start, waypoints)
    opt_length = info["optimized_length"]

    improvement = 100 * (bad_length - opt_length) / bad_length

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))

    plot_tour(ax1, start, bad_order, "(a) 优化前 (随机顺序)", "#E53935", bad_length)
    plot_tour(ax2, start, optimized, "(b) 2-opt 优化后", "#43A047", opt_length)

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图5-4_TSP优化前后对比图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")
    print(f"  随机顺序路径: {bad_length:.2f} m")
    print(f"  2-opt优化后:  {opt_length:.2f} m")
    print(f"  路径缩短:     {improvement:.1f}%")


if __name__ == "__main__":
    main()
