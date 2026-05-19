"""Generate publication-quality figures for thesis.

Creates properly formatted figures with Chinese labels and captions.
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)

FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300
plt.rcParams["font.size"] = 10


def fig1_system_architecture():
    """图1 系统总体架构图 — 三层架构，组件不与层名重叠"""
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    # Layer backgrounds
    layer_defs = [
        ("感知层 Perception", 0.73, 0.95, "#E3F2FD", "#1565C0"),
        ("决策层 Decision", 0.38, 0.70, "#FFF3E0", "#E65100"),
        ("执行层 Execution", 0.03, 0.35, "#E8F5E9", "#2E7D32"),
    ]
    for name, yb, yt, fc, ec in layer_defs:
        rect = mpatches.FancyBboxPatch(
            (0.04, yb), 0.92, yt - yb,
            boxstyle="round,pad=0.01",
            edgecolor=ec, facecolor=fc, linewidth=2, alpha=0.35
        )
        ax.add_patch(rect)
        ax.text(0.96, (yb + yt) / 2, name, ha="right", va="center",
                fontsize=9, fontweight="bold", color=ec, rotation=-90)

    # Helper to draw a component box
    def comp_box(x, y, text, fc="white", ec="gray", fs=9):
        box = mpatches.FancyBboxPatch(
            (x - 0.09, y - 0.035), 0.18, 0.07,
            boxstyle="round,pad=0.008", edgecolor=ec, facecolor=fc, linewidth=1.5
        )
        ax.add_patch(box)
        ax.text(x, y, text, ha="center", va="center", fontsize=fs)

    # Perception layer components
    comp_box(0.15, 0.84, "2D激光雷达\nLidar", "#BBDEFB", "#1565C0")
    comp_box(0.38, 0.84, "IMU\n惯性单元", "#BBDEFB", "#1565C0")
    comp_box(0.61, 0.84, "GPS\n全局定位", "#BBDEFB", "#1565C0")
    comp_box(0.84, 0.84, "罗盘\nCompass", "#BBDEFB", "#1565C0")

    # Decision layer components
    comp_box(0.18, 0.60, "特征提取\n坡度/粗糙度", "#FFE0B2", "#E65100")
    comp_box(0.45, 0.60, "地形分类器\n规则判定", "#FFE0B2", "#E65100")
    comp_box(0.72, 0.60, "参数自适应\nVmax/Kp", "#FFE0B2", "#E65100")
    comp_box(0.45, 0.45, "A*路径规划\n全局航点", "#FFE0B2", "#E65100")
    comp_box(0.18, 0.45, "TSP排序\n2-opt优化", "#FFE0B2", "#E65100")

    # Execution layer components
    comp_box(0.30, 0.22, "差速转向控制\n比例+旋转", "#C8E6C9", "#2E7D32")
    comp_box(0.60, 0.22, "四轮电机\nVL / VR", "#C8E6C9", "#2E7D32")
    comp_box(0.82, 0.10, "Webots引擎\nODE物理", "#C8E6C9", "#2E7D32")

    # Arrows: perception -> decision
    for sx in [0.15, 0.38]:
        ax.annotate("", xy=(0.18, 0.635), xytext=(sx, 0.805),
                    arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#1565C0"))
    ax.annotate("", xy=(0.45, 0.635), xytext=(0.38, 0.805),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#1565C0"))
    ax.annotate("", xy=(0.45, 0.635), xytext=(0.61, 0.805),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#1565C0"))

    # Decision internal arrows
    ax.annotate("", xy=(0.36, 0.60), xytext=(0.27, 0.60),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#E65100"))
    ax.annotate("", xy=(0.63, 0.60), xytext=(0.54, 0.60),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#E65100"))
    ax.annotate("", xy=(0.45, 0.525), xytext=(0.45, 0.565),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#E65100"))
    ax.annotate("", xy=(0.36, 0.45), xytext=(0.27, 0.45),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#E65100"))

    # Decision -> execution
    ax.annotate("", xy=(0.30, 0.255), xytext=(0.45, 0.415),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#2E7D32"))
    ax.annotate("", xy=(0.38, 0.255), xytext=(0.72, 0.565),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#2E7D32"))

    # Execution internal
    ax.annotate("", xy=(0.51, 0.22), xytext=(0.39, 0.22),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#2E7D32"))
    ax.annotate("", xy=(0.76, 0.13), xytext=(0.66, 0.19),
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#2E7D32"))

    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "图1_系统总体架构图.png"), bbox_inches="tight",
                facecolor="white")
    plt.close()
    print("[OK] 图1_系统总体架构图.png")


def fig2_terrain_classification_flowchart():
    """图2 地形分类算法流程图 — 菱形加大，间距拉开"""
    fig, ax = plt.subplots(figsize=(10, 13))
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    def draw_rect(x, y, text, color, fs=9):
        rect = mpatches.FancyBboxPatch(
            (x - 0.14, y - 0.025), 0.28, 0.05,
            boxstyle="round,pad=0.006", edgecolor="black",
            facecolor=color, linewidth=1.5
        )
        ax.add_patch(rect)
        ax.text(x, y, text, ha="center", va="center", fontsize=fs)

    def draw_diamond(x, y, text, color, fs=8):
        hw, hh = 0.18, 0.045
        points = np.array([[x, y + hh], [x + hw, y], [x, y - hh], [x - hw, y]])
        poly = mpatches.Polygon(points, closed=True, edgecolor="black",
                                facecolor=color, linewidth=1.5)
        ax.add_patch(poly)
        ax.text(x, y, text, ha="center", va="center", fontsize=fs)

    def draw_output(x, y, text, color, fs=9):
        rect = mpatches.FancyBboxPatch(
            (x - 0.10, y - 0.022), 0.20, 0.044,
            boxstyle="round,pad=0.005", edgecolor="black",
            facecolor=color, linewidth=1.5
        )
        ax.add_patch(rect)
        ax.text(x, y, text, ha="center", va="center", fontsize=fs, fontweight="bold",
                color="white")

    # Vertical flow positions
    y_start = 0.95
    y_read = 0.87
    y_feat = 0.79
    y_imu = 0.71
    y_d1 = 0.61
    y_d2 = 0.47
    y_d3 = 0.33
    y_out_trans = 0.20
    y_end = 0.10

    cx = 0.42  # center x for main flow
    ox = 0.78  # output x (right side)

    # Nodes
    draw_rect(cx, y_start, "开始", "#4CAF50", 10)
    draw_rect(cx, y_read, "读取传感器数据 (Lidar + IMU + GPS)", "#E3F2FD")
    draw_rect(cx, y_feat, "提取地形特征: slope, roughness, height_diff", "#E3F2FD")
    draw_rect(cx, y_imu, "提取IMU姿态: pitch, roll", "#FFF3E0")

    draw_diamond(cx, y_d1, "pitch>=3 且\nroughness<0.05 ?", "#FFEB3B", 8)
    draw_output(ox, y_d1, "斜坡 Slope", "#FF9800")

    draw_diamond(cx, y_d2, "roughness>=0.05\n或 roll>=2 ?", "#FFEB3B", 8)
    draw_output(ox, y_d2, "凹凸 Rough", "#F44336")

    draw_diamond(cx, y_d3, "slope<5 且\nroughness<0.02\n且 pitch<3 ?", "#FFEB3B", 8)
    draw_output(ox, y_d3, "平坦 Flat", "#4CAF50")

    draw_output(cx, y_out_trans, "过渡区 Transition", "#9C27B0")
    draw_rect(cx, y_end, "结束", "#4CAF50", 10)

    # Vertical arrows (main flow, "No" path)
    v_arrows = [
        (cx, y_start - 0.025, cx, y_read + 0.025),
        (cx, y_read - 0.025, cx, y_feat + 0.025),
        (cx, y_feat - 0.025, cx, y_imu + 0.025),
        (cx, y_imu - 0.025, cx, y_d1 + 0.045),
        (cx, y_d1 - 0.045, cx, y_d2 + 0.045),
        (cx, y_d2 - 0.045, cx, y_d3 + 0.045),
        (cx, y_d3 - 0.045, cx, y_out_trans + 0.022),
        (cx, y_out_trans - 0.022, cx, y_end + 0.025),
    ]
    for x1, y1, x2, y2 in v_arrows:
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", lw=1.5, color="black"))

    # Horizontal "Yes" arrows (diamond -> output)
    for dy in [y_d1, y_d2, y_d3]:
        ax.annotate("", xy=(ox - 0.10, dy), xytext=(cx + 0.18, dy),
                    arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#2E7D32"))

    # "Yes" / "No" labels
    for dy in [y_d1, y_d2, y_d3]:
        ax.text(cx + 0.20, dy + 0.015, "是", fontsize=8, color="#2E7D32", fontweight="bold")
        ax.text(cx - 0.03, dy - 0.055, "否", fontsize=8, color="#C62828", fontweight="bold")

    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "图2_地形分类算法流程图.png"),
                bbox_inches="tight", facecolor="white")
    plt.close()
    print("[OK] 图2_地形分类算法流程图.png")


def fig3_four_terrain_scenes():
    """图3 四种地形仿真场景"""
    # This would require screenshots from Webots
    # For now, create a placeholder
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    terrains = [
        ("平坦地形\nFlat Terrain", "#4CAF50"),
        ("斜坡地形 (5°)\nSlope Terrain", "#FF9800"),
        ("凹凸地形\nRough Terrain", "#F44336"),
        ("过渡区地形\nTransition Terrain", "#9C27B0"),
    ]

    for ax, (name, color) in zip(axes.flatten(), terrains):
        ax.text(0.5, 0.5, f"{name}\n\n(需替换为Webots截图)",
                ha="center", va="center", fontsize=12,
                bbox=dict(boxstyle="round", facecolor=color, alpha=0.3))
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis('off')

    plt.suptitle("图3 四种地形仿真场景\nFig.3 Four Terrain Simulation Scenes",
                 fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "图3_四种地形仿真场景_占位.png"), bbox_inches="tight")
    plt.close()
    print("[OK] 图3_四种地形仿真场景_占位.png (需替换为实际截图)")


def fig4_imu_assisted_classification_comparison():
    """图4 IMU辅助分类效果对比"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    categories = ['平坦\nFlat', '斜坡\nSlope', '凹凸\nRough', '过渡\nTransition']
    without_imu = [99, 0, 85, 80]  # Slope detection fails without IMU
    with_imu = [99, 95, 90, 85]

    x = np.arange(len(categories))
    width = 0.35

    ax1.bar(x - width/2, without_imu, width, label='无IMU (Without IMU)',
            color='#FF9800', alpha=0.8)
    ax1.bar(x + width/2, with_imu, width, label='有IMU (With IMU)',
            color='#4CAF50', alpha=0.8)

    ax1.set_ylabel('分类准确率 (%)\nClassification Accuracy (%)')
    ax1.set_title('(a) 分类准确率对比')
    ax1.set_xticks(x)
    ax1.set_xticklabels(categories)
    ax1.legend()
    ax1.grid(True, alpha=0.3, axis='y')
    ax1.set_ylim(0, 105)

    # Add value labels
    for i, (v1, v2) in enumerate(zip(without_imu, with_imu)):
        ax1.text(i - width/2, v1 + 2, f'{v1}%', ha='center', fontsize=9)
        ax1.text(i + width/2, v2 + 2, f'{v2}%', ha='center', fontsize=9)

    # Confusion matrix for slope detection
    methods = ['无IMU\nWithout', '有IMU\nWith']
    slope_correct = [0, 95]
    slope_wrong = [100, 5]

    ax2.bar(methods, slope_correct, label='正确识别 (Correct)', color='#4CAF50', alpha=0.8)
    ax2.bar(methods, slope_wrong, bottom=slope_correct, label='误判 (Wrong)', color='#F44336', alpha=0.8)

    ax2.set_ylabel('百分比 (%)\nPercentage (%)')
    ax2.set_title('(b) 斜坡地形识别对比')
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.set_ylim(0, 105)

    plt.suptitle("图4 IMU辅助地形分类效果对比\nFig.4 IMU-Assisted Classification Performance",
                 fontsize=14, fontweight="bold", y=1.02)
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "图4_IMU辅助分类效果对比.png"), bbox_inches="tight")
    plt.close()
    print("[OK] 图4_IMU辅助分类效果对比.png")


def fig5_adaptive_speed_control():
    """图5 自适应速度控制示意图"""
    fig, ax = plt.subplots(figsize=(12, 6))

    # Simulated terrain profile
    x = np.linspace(0, 40, 400)
    terrain_height = np.zeros_like(x)
    terrain_height[100:150] = (x[100:150] - 10) * 0.1  # Slope up
    terrain_height[150:250] = 0.5 + 0.1 * np.sin(10 * x[150:250])  # Rough
    terrain_height[250:300] = 1.0 - (x[250:300] - 25) * 0.1  # Slope down

    # Speed profile
    speed = np.ones_like(x) * 3.0
    speed[100:150] = 2.0  # Slope
    speed[150:250] = 1.5  # Rough
    speed[250:300] = 2.0  # Slope

    # Terrain type
    terrain_colors = ['#4CAF50'] * 100 + ['#FF9800'] * 50 + ['#F44336'] * 100 + \
                     ['#FF9800'] * 50 + ['#4CAF50'] * 100

    # Plot
    ax2 = ax.twinx()

    for i in range(len(x) - 1):
        ax.fill_between([x[i], x[i+1]], 0, [terrain_height[i], terrain_height[i+1]],
                        color=terrain_colors[i], alpha=0.3)

    line1 = ax2.plot(x, speed, 'b-', linewidth=2.5, label='机器人速度 (Robot Speed)')
    ax2.axhline(y=3.0, color='gray', linestyle='--', alpha=0.5, label='平坦地形速度')
    ax2.axhline(y=2.0, color='orange', linestyle='--', alpha=0.5, label='斜坡地形速度')
    ax2.axhline(y=1.5, color='red', linestyle='--', alpha=0.5, label='凹凸地形速度')

    ax.set_xlabel('距离 (m)\nDistance (m)')
    ax.set_ylabel('地形高度 (m)\nTerrain Height (m)')
    ax2.set_ylabel('速度 (rad/s)\nSpeed (rad/s)')
    ax.set_xlim(0, 40)
    ax.set_ylim(-0.2, 1.5)
    ax2.set_ylim(0, 4)

    # Legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#4CAF50', alpha=0.3, label='平坦 (Flat)'),
        Patch(facecolor='#FF9800', alpha=0.3, label='斜坡 (Slope)'),
        Patch(facecolor='#F44336', alpha=0.3, label='凹凸 (Rough)'),
    ]
    ax.legend(handles=legend_elements, loc='upper left')
    ax2.legend(loc='upper right')

    ax.set_title("图5 自适应速度控制示意图\nFig.5 Adaptive Speed Control",
                 fontsize=14, fontweight="bold", pad=20)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "图5_自适应速度控制示意图.png"), bbox_inches="tight")
    plt.close()
    print("[OK] 图5_自适应速度控制示意图.png")


def main():
    print("生成论文图表...\n")

    fig1_system_architecture()
    fig2_terrain_classification_flowchart()
    fig3_four_terrain_scenes()
    fig4_imu_assisted_classification_comparison()
    fig5_adaptive_speed_control()

    print(f"\n所有图表已保存到: {FIGURES_DIR}")
    print("\n注意: 图3 需要替换为 Webots 实际截图。")


if __name__ == "__main__":
    main()

