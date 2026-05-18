"""Generate Figure 3-2: Four terrain simulation scenes (programmatic visualization)."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def generate_flat_terrain(n=80):
    x = np.linspace(-20, 20, n)
    y = np.linspace(-20, 20, n)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)
    return X, Y, Z


def generate_slope_terrain(n=80):
    x = np.linspace(-20, 20, n)
    y = np.linspace(-20, 20, n)
    X, Y = np.meshgrid(x, y)
    Z = X * np.tan(np.radians(5))
    return X, Y, Z


def generate_rough_terrain(n=80):
    x = np.linspace(-20, 20, n)
    y = np.linspace(-20, 20, n)
    X, Y = np.meshgrid(x, y)
    Z = (0.2 * np.sin(0.8 * X) * np.cos(0.6 * Y) +
         0.15 * np.sin(1.5 * X + 0.5) * np.cos(1.2 * Y + 0.3) +
         0.1 * np.sin(2.5 * X) * np.sin(2.0 * Y))
    return X, Y, Z


def generate_transition_terrain(n=80):
    x = np.linspace(-20, 20, n)
    y = np.linspace(-20, 20, n)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)

    # Region 1: flat (x < -10)
    # Region 2: slope (-10 <= x < 0)
    mask_slope = (X >= -10) & (X < 0)
    Z[mask_slope] = (X[mask_slope] + 10) * np.tan(np.radians(5))

    # Region 3: rough (0 <= x < 10)
    mask_rough = (X >= 0) & (X < 10)
    Z[mask_rough] = (10 * np.tan(np.radians(5)) +
                     0.2 * np.sin(1.0 * X[mask_rough]) * np.cos(0.8 * Y[mask_rough]) +
                     0.1 * np.sin(2.0 * X[mask_rough]))

    # Region 4: flat again (x >= 10)
    mask_flat2 = X >= 10
    Z[mask_flat2] = 10 * np.tan(np.radians(5))

    return X, Y, Z


def main():
    fig = plt.figure(figsize=(14, 13))

    terrains = [
        ("(a) 平坦地形 (Flat Terrain)", generate_flat_terrain, "Greens"),
        ("(b) 斜坡地形 (Slope Terrain, 5°)", generate_slope_terrain, "YlOrBr"),
        ("(c) 凹凸地形 (Rough Terrain)", generate_rough_terrain, "RdYlGn_r"),
        ("(d) 过渡区地形 (Transition Terrain)", generate_transition_terrain, "coolwarm"),
    ]

    for idx, (title, gen_func, cmap_name) in enumerate(terrains):
        ax = fig.add_subplot(2, 2, idx + 1, projection="3d")
        X, Y, Z = gen_func()

        surf = ax.plot_surface(X, Y, Z, cmap=cmap_name, alpha=0.85,
                               linewidth=0, antialiased=True,
                               rstride=2, cstride=2)

        ax.set_xlabel("X (m)", fontsize=8, labelpad=1)
        ax.set_ylabel("Y (m)", fontsize=8, labelpad=1)
        ax.set_zlabel("Z (m)", fontsize=8, labelpad=1)
        ax.set_title(title, fontsize=10, fontweight="bold", pad=12)

        ax.view_init(elev=30, azim=-60)
        ax.tick_params(labelsize=6, pad=1)

        # Robot marker at origin
        z_at_origin = Z[len(Z)//2, len(Z[0])//2]
        ax.scatter([0], [0], [z_at_origin + 0.3], c="blue", s=50,
                   marker="^", zorder=10, label="Robot")

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    save_path = os.path.join(FIGURES_DIR, "图3-2_四种地形仿真场景.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
