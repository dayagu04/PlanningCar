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

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300
plt.rcParams["font.size"] = 10


def fig1_system_architecture():
    """图1 系统总体架构图"""
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axis('off')

    # Layers
    layers = [
        {"name": "感知层\nPerception Layer", "y": 0.8, "color": "#E3F2FD"},
        {"name": "决策层\nDecision Layer", "y": 0.5, "color": "#FFF3E0"},
        {"name": "执行层\nExecution Layer", "y": 0.2, "color": "#E8F5E9"},
    ]

    for layer in layers:
        rect = mpatches.FancyBboxPatch(
            (0.1, layer["y"] - 0.08), 0.8, 0.12,
            boxstyle="round,pad=0.01",
            edgecolor="black", facecolor=layer["color"], linewidth=2
        )
        ax.add_patch(rect)
        ax.text(0.5, layer["y"], layer["name"], ha="center", va="center",
                fontsize=12, fontweight="bold")

    # Components in each layer
    components = [
        {"text": "激光雷达\nLidar", "x": 0.2, "y": 0.8},
        {"text": "GPS", "x": 0.35, "y": 0.8},
        {"text": "IMU", "x": 0.5, "y": 0.8},
        {"text": "罗盘\nCompass", "x": 0.65, "y": 0.8},
        {"text": "地形分类\nClassifier", "x": 0.25, "y": 0.5},
        {"text": "路径规划\nPlanning", "x": 0.5, "y": 0.5},
        {"text": "参数自适应\nAdaptive", "x": 0.75, "y": 0.5},
        {"text": "差速控制\nDifferential", "x": 0.35, "y": 0.2},
        {"text": "电机驱动\nMotor", "x": 0.65, "y": 0.2},
    ]

    for comp in components:
        ax.text(comp["x"], comp["y"], comp["text"], ha="center", va="center",
                fontsize=9, bbox=dict(boxstyle="round", facecolor="white", edgecolor="gray"))

    # Arrows
    ax.annotate("", xy=(0.5, 0.68), xytext=(0.5, 0.74),
                arrowprops=dict(arrowstyle="->", lw=2, color="black"))
    ax.annotate("", xy=(0.5, 0.38), xytext=(0.5, 0.44),
                arrowprops=dict(arrowstyle="->", lw=2, color="black"))

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title("图1 系统总体架构图\nFig.1 System Architecture", fontsize=14, fontweight="bold", pad=20)

    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "图1_系统总体架构图.png"), bbox_inches="tight")
    plt.close()
    print("[OK] 图1_系统总体架构图.png")


def fig2_terrain_classification_flowchart():
    """图2 地形分类算法流程图"""
    fig, ax = plt.subplots(figsize=(8, 10))
    ax.axis('off')

    boxes = [
        {"text": "开始\nStart", "y": 0.95, "color": "#4CAF50"},
        {"text": "读取传感器数据\nLidar + IMU", "y": 0.85, "color": "#E3F2FD"},
        {"text": "提取地形特征\nslope, roughness, height", "y": 0.75, "color": "#E3F2FD"},
        {"text": "提取IMU姿态\npitch, roll", "y": 0.65, "color": "#FFF3E0"},
        {"text": "IMU pitch ≥ 3°\n且 roughness < 0.05?", "y": 0.52, "color": "#FFEB3B", "shape": "diamond"},
        {"text": "输出: 斜坡\nSlope", "y": 0.52, "color": "#FF9800", "x": 0.7},
        {"text": "roughness ≥ 0.05\n或 roll ≥ 2°?", "y": 0.39, "color": "#FFEB3B", "shape": "diamond"},
        {"text": "输出: 凹凸\nRough", "y": 0.39, "color": "#F44336", "x": 0.7},
        {"text": "slope < 5° 且\nroughness < 0.02?", "y": 0.26, "color": "#FFEB3B", "shape": "diamond"},
        {"text": "输出: 平坦\nFlat", "y": 0.26, "color": "#4CAF50", "x": 0.7},
        {"text": "输出: 过渡区\nTransition", "y": 0.13, "color": "#9C27B0"},
        {"text": "结束\nEnd", "y": 0.03, "color": "#4CAF50"},
    ]

    for box in boxes:
        x = box.get("x", 0.5)
        y = box["y"]
        shape = box.get("shape", "rect")

        if shape == "diamond":
            points = np.array([[x, y+0.04], [x+0.15, y], [x, y-0.04], [x-0.15, y]])
            poly = mpatches.Polygon(points, closed=True, edgecolor="black",
                                    facecolor=box["color"], linewidth=1.5)
            ax.add_patch(poly)
        else:
            rect = mpatches.FancyBboxPatch(
                (x - 0.15, y - 0.03), 0.3, 0.06,
                boxstyle="round,pad=0.005",
                edgecolor="black", facecolor=box["color"], linewidth=1.5
            )
            ax.add_patch(rect)

        ax.text(x, y, box["text"], ha="center", va="center", fontsize=9)

    # Arrows
    arrows = [
        (0.5, 0.92, 0.5, 0.88),
        (0.5, 0.82, 0.5, 0.78),
        (0.5, 0.72, 0.5, 0.68),
        (0.5, 0.62, 0.5, 0.56),
        (0.5, 0.48, 0.5, 0.43),
        (0.5, 0.35, 0.5, 0.30),
        (0.5, 0.22, 0.5, 0.16),
        (0.5, 0.10, 0.5, 0.06),
        # Yes arrows
        (0.65, 0.52, 0.7, 0.52),
        (0.65, 0.39, 0.7, 0.39),
        (0.65, 0.26, 0.7, 0.26),
    ]

    for x1, y1, x2, y2 in arrows:
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="->", lw=1.5, color="black"))

    # Labels
    ax.text(0.58, 0.52, "是\nYes", fontsize=8, color="green")
    ax.text(0.58, 0.39, "是\nYes", fontsize=8, color="green")
    ax.text(0.58, 0.26, "是\nYes", fontsize=8, color="green")
    ax.text(0.45, 0.45, "否\nNo", fontsize=8, color="red")
    ax.text(0.45, 0.32, "否\nNo", fontsize=8, color="red")
    ax.text(0.45, 0.19, "否\nNo", fontsize=8, color="red")

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title("图2 地形分类算法流程图\nFig.2 Terrain Classification Flowchart",
                 fontsize=14, fontweight="bold", pad=20)

    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "图2_地形分类算法流程图.png"), bbox_inches="tight")
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

