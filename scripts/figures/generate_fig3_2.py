"""Generate Figure 3-2: Four terrain simulation scenes (programmatic visualization).

Uses the ACTUAL terrain generation functions from generate_terrain.py so the
figure matches the real .wbt world heightmaps (40m x 40m, 80x80 grid).
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "scripts", "figures"))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300

from generate_terrain import generate_slope, generate_rough, generate_transition

TERRAIN_SIZE = 40
RESOLUTION = 80


def _make_grid(heights):
    """Convert a (RESOLUTION x RESOLUTION) height array to X, Y, Z in world coords."""
    x = np.linspace(-TERRAIN_SIZE / 2, TERRAIN_SIZE / 2, RESOLUTION)
    y = np.linspace(-TERRAIN_SIZE / 2, TERRAIN_SIZE / 2, RESOLUTION)
    X, Y = np.meshgrid(x, y)
    return X, Y, heights


def generate_flat():
    """Flat terrain: 100x100m box at z=0 (matches flat_terrain.wbt)."""
    x = np.linspace(-TERRAIN_SIZE / 2, TERRAIN_SIZE / 2, RESOLUTION)
    y = np.linspace(-TERRAIN_SIZE / 2, TERRAIN_SIZE / 2, RESOLUTION)
    X, Y = np.meshgrid(x, y)
    return X, Y, np.zeros_like(X)


def main():
    fig = plt.figure(figsize=(14, 13))

    terrains = [
        ("(a) 平坦地形 (Flat Terrain)", generate_flat, "Greens"),
        ("(b) 斜坡地形 (Slope Terrain, 15deg)", generate_slope, "YlOrBr"),
        ("(c) 凹凸地形 (Rough Terrain)", generate_rough, "RdYlGn_r"),
        ("(d) 过渡区地形 (Transition Terrain, 20deg ramp)", generate_transition, "coolwarm"),
    ]

    for idx, (title, gen_func, cmap_name) in enumerate(terrains):
        ax = fig.add_subplot(2, 2, idx + 1, projection="3d")

        if gen_func == generate_flat:
            X, Y, Z = generate_flat()
        else:
            Z = gen_func()
            X, Y, Z = _make_grid(Z)

        surf = ax.plot_surface(X, Y, Z, cmap=cmap_name, alpha=0.85,
                               linewidth=0, antialiased=True,
                               rstride=2, cstride=2)

        ax.set_xlabel("X (m)", fontsize=8, labelpad=1)
        ax.set_ylabel("Y (m)", fontsize=8, labelpad=1)
        ax.set_zlabel("Z (m)", fontsize=8, labelpad=1)
        ax.set_title(title, fontsize=10, fontweight="bold", pad=12)

        ax.view_init(elev=30, azim=-60)
        ax.tick_params(labelsize=6, pad=1)

        z_at_origin = Z[RESOLUTION // 2, RESOLUTION // 2]
        ax.scatter([0], [0], [z_at_origin + 0.3], c="blue", s=50,
                   marker="^", zorder=10, label="Robot")

    plt.subplots_adjust(left=0.02, right=0.98, top=0.95, bottom=0.02,
                        hspace=0.25, wspace=0.1)
    save_path = os.path.join(FIGURES_DIR, "图3-2a_四种地形仿真场景_3D示意图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
