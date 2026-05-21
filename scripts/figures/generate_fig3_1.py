"""Generate Figure 3-1: Robot model schematic diagram."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def draw_robot_top_view(ax):
    """Top-down view of the robot showing chassis, wheels, and sensors."""
    ax.set_xlim(-0.40, 0.40)
    ax.set_ylim(-0.32, 0.32)
    ax.set_aspect("equal")
    ax.set_title("(a) 俯视图 (Top View)", fontsize=11, fontweight="bold")

    # Chassis body: 0.4 x 0.3 x 0.1 (wbt: Box{size 0.4 0.3 0.1})
    chassis = FancyBboxPatch((-0.20, -0.15), 0.40, 0.30,
                             boxstyle="round,pad=0.01",
                             edgecolor="black", facecolor="#4A90D9",
                             linewidth=2, alpha=0.7)
    ax.add_patch(chassis)

    # Wheels: radius=0.10 m, thickness=0.05 m, anchors at (±0.15, ±0.18)
    wheel_positions = [
        (0.15, 0.18, "FL"), (0.15, -0.18, "FR"),
        (-0.15, 0.18, "RL"), (-0.15, -0.18, "RR"),
    ]
    for wx, wy, name in wheel_positions:
        # Top-view: wheel projects as 0.05 (thickness, along Y) × 0.20 (diameter, along X)
        wheel = Rectangle((wx - 0.10, wy - 0.025), 0.20, 0.05,
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
    ax.set_xlim(-0.40, 0.40)
    ax.set_ylim(-0.05, 0.32)
    ax.set_aspect("equal")
    ax.set_title("(b) 侧视图 (Side View)", fontsize=11, fontweight="bold")

    # Ground line (z=0)
    ax.axhline(y=0.0, color="brown", linewidth=2, linestyle="-")
    ax.fill_between([-0.40, 0.40], -0.05, 0.0, color="#8B4513", alpha=0.2)

    # Wheels (side view): radius=0.10, axle at z=0.12 (robot z=0.15, anchor z=-0.03)
    for wx in [0.15, -0.15]:
        wheel = Circle((wx, 0.10), 0.10, edgecolor="black",
                       facecolor="#333333", linewidth=1.5)
        ax.add_patch(wheel)
        # Axle marker
        ax.plot(wx, 0.10, "ko", markersize=3)

    # Chassis: bottom at z=0.10, top at z=0.20 (robot z=0.15, box ±0.05)
    chassis = FancyBboxPatch((-0.20, 0.10), 0.40, 0.10,
                             boxstyle="round,pad=0.005",
                             edgecolor="black", facecolor="#4A90D9",
                             linewidth=2, alpha=0.7)
    ax.add_patch(chassis)
    ax.text(0, 0.15, "底盘 0.4×0.3×0.1m\n8.4 kg (整车 11.6 kg)", ha="center",
            va="center", fontsize=8, color="white")

    # Lidar on top: relative z=0.08 → absolute z=0.23
    lidar = Circle((0, 0.23), 0.025, edgecolor="red", facecolor="#FF6B6B",
                   linewidth=2, alpha=0.8)
    ax.add_patch(lidar)
    ax.text(0, 0.28, "激光雷达\n360°, 15m", ha="center", fontsize=7, color="red")

    # Length annotation
    ax.annotate("", xy=(-0.20, -0.03), xytext=(0.20, -0.03),
                arrowprops=dict(arrowstyle="<->", lw=1, color="gray"))
    ax.text(0, -0.04, "0.4m (车长)", ha="center", va="top", fontsize=7, color="gray")

    # Wheel diameter annotation
    ax.annotate("", xy=(0.30, 0.0), xytext=(0.30, 0.20),
                arrowprops=dict(arrowstyle="<->", lw=1, color="gray"))
    ax.text(0.32, 0.10, "直径\n0.20m", ha="left", va="center", fontsize=7, color="gray")

    # Total height annotation
    ax.annotate("", xy=(-0.30, 0.0), xytext=(-0.30, 0.245),
                arrowprops=dict(arrowstyle="<->", lw=1, color="gray"))
    ax.text(-0.32, 0.12, "0.245m\n(到雷达)", ha="right", va="center", fontsize=7, color="gray")

    ax.grid(True, alpha=0.2)
    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Z (m)", fontsize=9)


def draw_sensor_table(ax):
    """Table showing robot mechanical and sensor specifications."""
    ax.axis("off")
    ax.set_title("(c) 机械与传感器配置参数", fontsize=11, fontweight="bold")

    table_data = [
        ["类别", "项目", "参数 / 数值", "用途 / 备注"],
        ["机械", "底盘尺寸", "0.4 × 0.3 × 0.1 m", "Box 几何"],
        ["机械", "底盘质量", "8.4 kg (密度 700 kg/m^3)", "0.012 m^3 × 700"],
        ["机械", "整车质量", "≈ 11.6 kg", "底盘 + 4 轮 (各 0.79 kg)"],
        ["机械", "车轮", "直径 0.20 m × 厚 0.05 m", "圆柱, 4 轮独立驱动"],
        ["机械", "轮距 / 轴距", "0.36 m / 0.30 m", "差速驱动"],
        ["驱动", "最大轮速", "20 rad/s (≈2.0 m/s)", "RotationalMotor.maxVelocity"],
        ["驱动", "最大力矩", "8.0 N·m (每轮)", "RotationalMotor.maxTorque"],
        ["传感", "2D 激光雷达", "360 点, FOV=360°, 15 m", "障碍/地形特征"],
        ["传感", "GPS", "位置精度 ±0.01 m", "全局定位"],
        ["传感", "IMU", "Roll / Pitch / Yaw", "姿态测量(地形分类)"],
        ["传感", "罗盘", "方位角精度 ±0.1°", "航向参考"],
    ]

    table = ax.table(cellText=table_data[1:], colLabels=table_data[0],
                     loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(8.5)
    table.scale(1.0, 1.45)

    # Style header
    for j in range(4):
        table[0, j].set_facecolor("#4A90D9")
        table[0, j].set_text_props(color="white", fontweight="bold")
    for i in range(1, len(table_data)):
        for j in range(4):
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
