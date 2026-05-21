"""Generate Figure 5-2: Navigation control flowchart (rendered as PNG)."""

import os
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def draw_box(ax, x, y, w, h, text, color="#E3F2FD", edge="#1565C0", fontsize=9):
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                         boxstyle="round,pad=0.01",
                         edgecolor=edge, facecolor=color, linewidth=1.8)
    ax.add_patch(box)
    ax.text(x, y, text, ha="center", va="center", fontsize=fontsize,
            fontweight="bold", color="#222")


def draw_diamond(ax, x, y, w, h, text, color="#FFF9C4", edge="#F57F17", fontsize=8):
    points = np.array([[x, y + h/2], [x + w/2, y], [x, y - h/2], [x - w/2, y]])
    poly = mpatches.Polygon(points, closed=True, edgecolor=edge,
                            facecolor=color, linewidth=2)
    ax.add_patch(poly)
    ax.text(x, y, text, ha="center", va="center", fontsize=fontsize, color="#333")


def arrow(ax, x1, y1, x2, y2, label="", label_offset=(0, 0), color="black"):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", lw=1.8, color=color))
    if label:
        mx = (x1 + x2) / 2 + label_offset[0]
        my = (y1 + y2) / 2 + label_offset[1]
        ax.text(mx, my, label, fontsize=8, ha="center", va="center", color=color)


def main():
    fig, ax = plt.subplots(figsize=(10, 12))
    ax.axis("off")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 14)

    # Nodes (x, y)
    # Row 1: Start
    draw_box(ax, 5, 13, 2.5, 0.7, "开始: 读取传感器", "#C8E6C9", "#2E7D32", 10)

    # Row 2: Compute beta
    draw_box(ax, 5, 11.5, 3.2, 0.8,
             "计算方位偏差角\nbeta = target_angle - bearing", "#E3F2FD", "#1565C0", 9)

    # Row 3: Decision
    draw_diamond(ax, 5, 9.8, 3.5, 1.4, "|beta| > 5pi/6 ?", "#FFF9C4", "#F57F17", 10)

    # Left branch: spin
    draw_box(ax, 2, 7.8, 2.8, 0.8,
             "原地旋转模态\n(Spin-in-place)", "#FFCDD2", "#C62828", 9)
    draw_box(ax, 2, 6.2, 3.0, 0.9,
             "左轮 = -0.7*Vmax\n右轮 = +0.7*Vmax\n(或反向)", "#FFCDD2", "#C62828", 8)

    # Right branch: PD steering
    draw_box(ax, 8, 7.8, 2.8, 0.8,
             "PD转向模态\n(Proportional-Derivative)", "#E8F5E9", "#2E7D32", 9)
    draw_box(ax, 8, 6.2, 3.2, 0.9,
             "turn = Kp*beta + Kd*d_beta\n(饱和: +/-0.8*Vmax)", "#E8F5E9", "#2E7D32", 8)
    draw_box(ax, 8, 4.6, 3.2, 0.9,
             "forward = Vmax*(floor +\n  (1-floor)*cos(beta))", "#E8F5E9", "#2E7D32", 8)

    # Merge
    draw_box(ax, 5, 3.0, 3.0, 0.8,
             "VL = forward - turn\nVR = forward + turn", "#E3F2FD", "#1565C0", 9)

    # End
    draw_box(ax, 5, 1.5, 2.5, 0.7, "等待下一时间步 (32ms)", "#C8E6C9", "#2E7D32", 9)

    # Arrows
    arrow(ax, 5, 12.65, 5, 11.9)
    arrow(ax, 5, 11.1, 5, 10.5)

    # Decision -> left (Yes)
    ax.annotate("", xy=(3.3, 9.8), xytext=(3.75, 9.8),
                arrowprops=dict(arrowstyle="-|>", lw=1.8, color="#C62828"))
    ax.plot([3.3, 2], [9.8, 9.8], color="#C62828", lw=1.8)
    ax.annotate("", xy=(2, 8.2), xytext=(2, 9.8),
                arrowprops=dict(arrowstyle="-|>", lw=1.8, color="#C62828"))
    ax.text(2.8, 10.1, "是 (Yes)", fontsize=9, color="#C62828", fontweight="bold")

    # Decision -> right (No)
    ax.annotate("", xy=(6.7, 9.8), xytext=(6.25, 9.8),
                arrowprops=dict(arrowstyle="-|>", lw=1.8, color="#2E7D32"))
    ax.plot([6.7, 8], [9.8, 9.8], color="#2E7D32", lw=1.8)
    ax.annotate("", xy=(8, 8.2), xytext=(8, 9.8),
                arrowprops=dict(arrowstyle="-|>", lw=1.8, color="#2E7D32"))
    ax.text(7.0, 10.1, "否 (No)", fontsize=9, color="#2E7D32", fontweight="bold")

    # Left branch down
    arrow(ax, 2, 7.4, 2, 6.65, color="#C62828")

    # Right branch down
    arrow(ax, 8, 7.4, 8, 6.65, color="#2E7D32")
    arrow(ax, 8, 5.75, 8, 5.05, color="#2E7D32")

    # Merge arrows
    ax.plot([2, 2, 5], [5.75, 3.0, 3.0], color="#C62828", lw=1.5, linestyle="--")
    ax.annotate("", xy=(5, 3.0), xytext=(4.5, 3.0),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#C62828"))

    ax.plot([8, 8, 5.5], [4.15, 3.0, 3.0], color="#2E7D32", lw=1.5, linestyle="--")
    ax.annotate("", xy=(5.5, 3.0), xytext=(6.0, 3.0),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#2E7D32"))

    # Output -> loop back
    arrow(ax, 5, 2.6, 5, 1.9)

    # Loop arrow back to top
    ax.annotate("", xy=(9.5, 13), xytext=(9.5, 1.5),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="gray",
                                connectionstyle="arc3,rad=0.15"))
    ax.text(9.7, 7.5, "循环", fontsize=8, color="gray", rotation=90, ha="center")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图5-2_导航控制流程图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
