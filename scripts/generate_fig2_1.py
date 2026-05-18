"""Generate Figure 2-1: System architecture diagram (three-layer data flow)."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def draw_layer_bg(ax, y_bottom, y_top, color, label):
    """Draw a translucent background band for a layer."""
    rect = mpatches.FancyBboxPatch(
        (0.03, y_bottom), 0.94, y_top - y_bottom,
        boxstyle="round,pad=0.01",
        edgecolor=color, facecolor=color, alpha=0.08, linewidth=2.5
    )
    ax.add_patch(rect)
    ax.text(0.97, (y_bottom + y_top) / 2, label,
            ha="right", va="center", fontsize=10, fontweight="bold",
            color=color, rotation=-90)


def draw_box(ax, x, y, w, h, text, facecolor, edgecolor, fontsize=9):
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                         boxstyle="round,pad=0.008",
                         edgecolor=edgecolor, facecolor=facecolor, linewidth=1.8)
    ax.add_patch(box)
    ax.text(x, y, text, ha="center", va="center", fontsize=fontsize,
            fontweight="bold", color="#222", linespacing=1.3)


def arrow_down(ax, x1, y1, x2, y2, color="black", style="-|>"):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, lw=1.8, color=color))


def arrow_right(ax, x1, y1, x2, y2, color="black"):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", lw=1.8, color=color))


def main():
    fig, ax = plt.subplots(figsize=(13, 9))
    ax.axis("off")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    # ============ Layer backgrounds ============
    draw_layer_bg(ax, 0.72, 0.96, "#1565C0", "感知层\nPerception")
    draw_layer_bg(ax, 0.32, 0.70, "#E65100", "决策层\nDecision")
    draw_layer_bg(ax, 0.02, 0.30, "#2E7D32", "执行层\nExecution")

    # ============ Perception Layer (top) ============
    sensors = [
        (0.20, 0.86, "2D激光雷达\n(360pts, 15m)"),
        (0.50, 0.86, "IMU\n(Roll/Pitch/Yaw)"),
        (0.80, 0.86, "GPS + 罗盘\n(位置/航向)"),
    ]
    for sx, sy, txt in sensors:
        draw_box(ax, sx, sy, 0.18, 0.12, txt, "#E3F2FD", "#1565C0", 9)

    # Data output from perception
    draw_box(ax, 0.50, 0.74, 0.30, 0.06, "点云数据 + 姿态角 + 位置/航向",
             "#BBDEFB", "#1565C0", 8)

    # Arrows: sensors -> data output
    for sx, _, _ in sensors:
        arrow_down(ax, sx, 0.80, 0.50 + (sx - 0.50) * 0.3, 0.77, "#1565C0")

    # ============ Decision Layer (middle) ============
    # Feature extraction
    draw_box(ax, 0.20, 0.60, 0.20, 0.10,
             "特征提取\n坡度 / 粗糙度 / 高度差", "#FFF3E0", "#E65100", 8)

    # Terrain classifier
    draw_box(ax, 0.50, 0.60, 0.22, 0.10,
             "地形分类器\n平坦/斜坡/凹凸/过渡", "#FFF3E0", "#E65100", 8)

    # Two outputs from classifier
    draw_box(ax, 0.50, 0.44, 0.20, 0.08,
             "A* 路径规划\n(全局航点序列)", "#FFF3E0", "#E65100", 8)

    draw_box(ax, 0.80, 0.52, 0.18, 0.10,
             "自适应参数表\nVmax / Kp / Accel", "#FFF3E0", "#E65100", 8)

    # TSP module
    draw_box(ax, 0.20, 0.44, 0.18, 0.08,
             "TSP航点排序\n(2-opt优化)", "#FFF3E0", "#E65100", 8)

    # Arrows within decision layer
    # perception output -> feature extraction
    arrow_down(ax, 0.35, 0.71, 0.20, 0.65, "#E65100")
    # perception output -> classifier (IMU data)
    arrow_down(ax, 0.50, 0.71, 0.50, 0.65, "#E65100")
    # feature extraction -> classifier
    arrow_right(ax, 0.30, 0.60, 0.39, 0.60, "#E65100")
    # classifier -> A* planner
    arrow_down(ax, 0.50, 0.55, 0.50, 0.48, "#E65100")
    # classifier -> adaptive params
    arrow_right(ax, 0.61, 0.60, 0.71, 0.55, "#E65100")
    # TSP -> A* (waypoint order)
    arrow_right(ax, 0.29, 0.44, 0.40, 0.44, "#E65100")

    # Data flow label
    ax.text(0.35, 0.68, "雷达点云+IMU", fontsize=7, color="#555", ha="center")
    ax.text(0.65, 0.57, "地形类型", fontsize=7, color="#555", ha="center", rotation=-30)

    # ============ Execution Layer (bottom) ============
    draw_box(ax, 0.35, 0.22, 0.22, 0.10,
             "差速转向控制器\n(比例+原地旋转)", "#E8F5E9", "#2E7D32", 9)

    draw_box(ax, 0.65, 0.22, 0.18, 0.10,
             "左/右轮电机\n(VL, VR)", "#E8F5E9", "#2E7D32", 9)

    draw_box(ax, 0.85, 0.10, 0.18, 0.08,
             "Webots 物理引擎\n(ODE)", "#C8E6C9", "#2E7D32", 8)

    # Arrows: decision -> execution
    # A* path -> controller
    arrow_down(ax, 0.50, 0.40, 0.40, 0.27, "#2E7D32")
    # Adaptive params -> controller
    arrow_down(ax, 0.80, 0.47, 0.45, 0.27, "#2E7D32")

    # Controller -> motors
    arrow_right(ax, 0.46, 0.22, 0.56, 0.22, "#2E7D32")

    # Motors -> Webots
    arrow_right(ax, 0.74, 0.20, 0.78, 0.12, "#2E7D32")

    # Feedback loop (Webots -> sensors)
    ax.annotate("", xy=(0.08, 0.86), xytext=(0.08, 0.10),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="gray",
                                linestyle="dashed",
                                connectionstyle="arc3,rad=0.0"))
    ax.text(0.05, 0.50, "传感器\n反馈", fontsize=7, color="gray",
            ha="center", va="center", rotation=90)

    # Data flow labels on execution arrows
    ax.text(0.48, 0.34, "目标航点\n+ 参数", fontsize=7, color="#555", ha="center")
    ax.text(0.51, 0.19, "VL, VR", fontsize=7, color="#2E7D32", ha="center")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图2-1_系统总体架构图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
