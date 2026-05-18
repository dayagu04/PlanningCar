"""Generate Figure 5-3: A* algorithm cost map heatmap."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Patch

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def main():
    grid_size = 40
    cost_map = np.full((grid_size, grid_size), 1.0)

    # Slope band (middle, columns 14-22)
    cost_map[:, 14:22] = 2.0

    # Rough region (right block, rows 8-32, cols 28-38)
    cost_map[8:32, 28:38] = 3.0

    # Circular obstacle at (12, 6) radius 3
    yy, xx = np.ogrid[:grid_size, :grid_size]
    obstacle_mask = (xx - 6) ** 2 + (yy - 12) ** 2 <= 3 ** 2
    cost_map[obstacle_mask] = 999.0

    # Display: clip 999 to 4 for visualization
    display_map = np.where(cost_map >= 999, 4.0, cost_map)

    fig, ax = plt.subplots(figsize=(9, 8))

    # Custom colormap: dark blue (1.0) -> yellow (2.0) -> red (3.0) -> black (999)
    colors = [
        (0.0, "#1A237E"),   # 1.0 - deep blue (flat)
        (0.33, "#FFEB3B"),  # 2.0 - yellow (slope)
        (0.66, "#D32F2F"),  # 3.0 - red (rough)
        (1.0, "#000000"),   # 4.0 (clipped from 999) - black (obstacle)
    ]
    cmap = mcolors.LinearSegmentedColormap.from_list("cost_cmap", colors, N=256)

    im = ax.imshow(display_map, cmap=cmap, origin="lower",
                   extent=[0, 20, 0, 20], aspect="equal", vmin=1.0, vmax=4.0)

    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("移动代价 (Cost)", fontsize=11)
    cbar.set_ticks([1.0, 2.0, 3.0, 4.0])
    cbar.set_ticklabels(["1.0\n平坦", "2.0\n斜坡", "3.0\n凹凸", "999\n障碍"])

    # Mark example start and goal
    ax.plot(1.5, 1.5, "g*", markersize=22, markeredgecolor="white",
            markeredgewidth=1.2, zorder=10)
    ax.plot(18.5, 18.5, "r^", markersize=18, markeredgecolor="white",
            markeredgewidth=1.2, zorder=10)
    ax.annotate("起点 Start", xy=(1.5, 1.5), xytext=(2.5, 2.5),
                fontsize=11, color="white", fontweight="bold")
    ax.annotate("终点 Goal", xy=(18.5, 18.5), xytext=(15, 17.5),
                fontsize=11, color="white", fontweight="bold")

    # Sample A* path: bypass obstacle, prefer flat
    path_x = [1.5, 4, 6, 9, 11, 13, 15, 17, 18.5]
    path_y = [1.5, 4, 6, 9, 11, 13, 15.5, 17, 18.5]
    ax.plot(path_x, path_y, "w--", linewidth=2.2, alpha=0.9, zorder=9,
            label="A* 规划路径")

    ax.set_xlabel("X (m)", fontsize=11)
    ax.set_ylabel("Y (m)", fontsize=11)
    ax.legend(fontsize=10, loc="lower right",
              facecolor="white", edgecolor="black")
    ax.grid(True, alpha=0.2, linestyle=":", color="white")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图5-3_A星算法代价地图示意图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
