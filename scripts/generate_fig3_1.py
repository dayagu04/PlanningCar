"""Generate Figure 3-1: Robot model schematic diagram."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def draw_robot_top_view(ax):
    """Top-down view of the robot showing chassis, wheels, and sensors."""
    ax.set_xlim(-0.35, 0.35)
    ax.set_ylim(-0.30, 0.30)
    ax.set_aspect("equal")
    ax.set_title("(a) 俯视图 (Top View)", fontsize=11, fontweight="bold")

    # Chassis body: 0.4 x 0.3 x 0.1
    chassis = FancyBboxPatch((-0.20, -0.15), 0.40, 0.30,
                             boxstyle="round,pad=0.01",
                             edgecolor="black", facecolor="#4A90D9",
                             linewidth=2, alpha=0.7)
    ax.add_patch(chassis)

    # Wheels: radius=0.06, height=0.04
    wheel_positions = [
        (0.15, 0.18, "FL"), (0.15, -0.18, "FR"),
        (-0.15, 0.18, "RL"), (-0.15, -0.18, "RR"),
    ]
    for wx, wy, name in wheel_positions:
        wheel = Rectangle((wx - 0.03, wy - 0.06), 0.06, 0.12,
                          edgecolor="black", facecolor="#333333", linewidth=1.5)
        ax.add_patch(wheel)
        ax.text(wx, wy, name, ha="center", va="center", fontsize=7, color="white")

    # Lidar on top center
    lidar_circle = Circle((0, 0), 0.04, edgecolor="red", facecolor="#FF6B6B",
                          linewidth=2, alpha=0.8)
    ax.add_patch(lidar_circle)
    ax.text(0, 0, "L", ha="center", va="center", fontsize=8, color="white", fontweight="bold")

    # Lidar scan lines (360 degree)
    for angle in np.linspace(0, 2 * np.pi, 12, endpoint=False):
        dx = 0.28 * np.cos(angle)
        dy = 0.22 * np.sin(angle)
        ax.plot([0, dx], [0, dy], "r-", alpha=0.2, linewidth=0.5)

    # GPS/IMU/Compass cluster
    ax.plot(0.08, 0.05, "g^", markersize=8)
    ax.text(0.08, 0.08, "GPS", fontsize=6, ha="center", color="green")
    ax.plot(-0.08, 0.05, "bs", markersize=6)
    ax.text(-0.08, 0.08, "IMU", fontsize=6, ha="center", color="blue")
    ax.plot(0, -0.08, "mD", markersize=6)
    ax.text(0, -0.11, "Compass", fontsize=6, ha="center", color="purple")

    # Direction arrow
    ax.annotate("", xy=(0.30, 0), xytext=(0.20, 0),
                arrowprops=dict(arrowstyle="-|>", lw=2, color="darkgreen"))
    ax.text(0.30, 0.03, "前进方向", fontsize=7, color="darkgreen")

    ax.grid(True, alpha=0.2)
    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Y (m)", fontsize=9)


def draw_robot_side_view(ax):
    """Side view showing height profile."""
    ax.set_xlim(-0.35, 0.35)
    ax.set_ylim(-0.12, 0.25)
    ax.set_aspect("equal")
    ax.set_title("(b) 侧视图 (Side View)", fontsize=11, fontweight="bold")

    # Ground line
    ax.axhline(y=-0.09, color="brown", linewidth=2, linestyle="-")
    ax.fill_between([-0.35, 0.35], -0.12, -0.09, color="#8B4513", alpha=0.2)

    # Wheels (side view - circles)
    for wx in [0.15, -0.15]:
        wheel = Circle((wx, -0.03), 0.06, edgecolor="black",
                       facecolor="#333333", linewidth=1.5)
        ax.add_patch(wheel)
        # Axle
        ax.plot([wx, wx], [-0.03, 0.05], "k-", linewidth=1)

    # Chassis
    chassis = FancyBboxPatch((-0.20, 0.0), 0.40, 0.10,
                             boxstyle="round,pad=0.005",
                             edgecolor="black", facecolor="#4A90D9",
                             linewidth=2, alpha=0.7)
    ax.add_patch(chassis)
    ax.text(0, 0.05, "底盘 0.4×0.3×0.1m\n质量 1.2kg", ha="center",
            va="center", fontsize=8, color="white")

    # Lidar on top
    lidar = Circle((0, 0.13), 0.03, edgecolor="red", facecolor="#FF6B6B",
                   linewidth=2, alpha=0.8)
    ax.add_patch(lidar)
    ax.text(0, 0.18, "激光雷达\n360°, 15m", ha="center", fontsize=7, color="red")

    # Dimension annotations
    ax.annotate("", xy=(-0.20, -0.11), xytext=(0.20, -0.11),
                arrowprops=dict(arrowstyle="<->", lw=1, color="gray"))
    ax.text(0, -0.115, "0.4m", ha="center", va="top", fontsize=7, color="gray")

    # Height annotation
    ax.annotate("", xy=(0.25, -0.09), xytext=(0.25, 0.10),
                arrowprops=dict(arrowstyle="<->", lw=1, color="gray"))
    ax.text(0.28, 0.0, "0.19m", ha="left", va="center", fontsize=7, color="gray")

    ax.grid(True, alpha=0.2)
    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Z (m)", fontsize=9)


def draw_sensor_table(ax):
    """Table showing sensor specifications."""
    ax.axis("off")
    ax.set_title("(c) 传感器配置参数", fontsize=11, fontweight="bold")

    table_data = [
        ["传感器", "型号/参数", "用途"],
        ["2D激光雷达", "360点, FOV=360°, 15m", "障碍物检测/地形特征"],
        ["GPS", "位置精度 ±0.01m", "全局定位"],
        ["IMU", "Roll/Pitch/Yaw", "姿态测量(地形分类)"],
        ["罗盘", "方位角精度 ±0.1°", "航向参考"],
    ]

    table = ax.table(cellText=table_data[1:], colLabels=table_data[0],
                     loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.8)

    # Style header
    for j in range(3):
        table[0, j].set_facecolor("#4A90D9")
        table[0, j].set_text_props(color="white", fontweight="bold")
    for i in range(1, 5):
        for j in range(3):
            table[i, j].set_facecolor("#F0F8FF" if i % 2 == 0 else "white")


def main():
    fig = plt.figure(figsize=(14, 11))

    ax1 = fig.add_subplot(2, 2, 1)
    draw_robot_top_view(ax1)

    ax2 = fig.add_subplot(2, 2, 2)
    draw_robot_side_view(ax2)

    ax3 = fig.add_subplot(2, 1, 2)
    draw_sensor_table(ax3)

    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05, hspace=0.3, wspace=0.3)
    save_path = os.path.join(FIGURES_DIR, "图3-1_机器人模型示意图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
