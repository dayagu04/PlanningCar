"""Generate Figure 4-1: IMU and Lidar feature complementarity architecture diagram."""

import os
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def main():
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.axis("off")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    # --- Left column: Sensors ---
    # Lidar box
    lidar_box = FancyBboxPatch((0.02, 0.60), 0.16, 0.25,
                               boxstyle="round,pad=0.015",
                               edgecolor="#1565C0", facecolor="#E3F2FD", linewidth=2)
    ax.add_patch(lidar_box)
    ax.text(0.10, 0.725, "2D激光雷达\n(Lidar)", ha="center", va="center",
            fontsize=11, fontweight="bold", color="#1565C0")

    # IMU box
    imu_box = FancyBboxPatch((0.02, 0.15), 0.16, 0.25,
                             boxstyle="round,pad=0.015",
                             edgecolor="#E65100", facecolor="#FFF3E0", linewidth=2)
    ax.add_patch(imu_box)
    ax.text(0.10, 0.275, "IMU\n(惯性测量单元)", ha="center", va="center",
            fontsize=11, fontweight="bold", color="#E65100")

    # --- Middle column: Feature extraction ---
    # Lidar features
    lidar_feat_box = FancyBboxPatch((0.28, 0.58), 0.22, 0.29,
                                    boxstyle="round,pad=0.015",
                                    edgecolor="#2E7D32", facecolor="#E8F5E9", linewidth=2)
    ax.add_patch(lidar_feat_box)
    ax.text(0.39, 0.725, "高度网格 (4×4)\n↓\n粗糙度 (roughness)\n坡度 (slope_deg)\n高度差 (height_diff)",
            ha="center", va="center", fontsize=9, color="#2E7D32")

    # IMU features
    imu_feat_box = FancyBboxPatch((0.28, 0.13), 0.22, 0.29,
                                  boxstyle="round,pad=0.015",
                                  edgecolor="#BF360C", facecolor="#FBE9E7", linewidth=2)
    ax.add_patch(imu_feat_box)
    ax.text(0.39, 0.275, "Pitch角 (俯仰)\n→ 检测斜坡\n\nRoll角 (侧倾)\n→ 检测凹凸",
            ha="center", va="center", fontsize=9, color="#BF360C")

    # --- Right column: Classifier ---
    classifier_box = FancyBboxPatch((0.60, 0.25), 0.22, 0.50,
                                    boxstyle="round,pad=0.02",
                                    edgecolor="#4A148C", facecolor="#F3E5F5", linewidth=2.5)
    ax.add_patch(classifier_box)
    ax.text(0.71, 0.50, "规则分类器\n(TerrainClassifier)\n\n─────────\n平坦 (Flat)\n斜坡 (Slope)\n凹凸 (Rough)\n过渡 (Transition)",
            ha="center", va="center", fontsize=9, fontweight="bold", color="#4A148C")

    # --- Far right: Output ---
    output_box = FancyBboxPatch((0.87, 0.35), 0.11, 0.30,
                                boxstyle="round,pad=0.015",
                                edgecolor="#1B5E20", facecolor="#C8E6C9", linewidth=2)
    ax.add_patch(output_box)
    ax.text(0.925, 0.50, "自适应\n参数\n(MotionParams)",
            ha="center", va="center", fontsize=9, fontweight="bold", color="#1B5E20")

    # --- Arrows ---
    arrow_style = "Simple,tail_width=3,head_width=10,head_length=8"
    # Lidar -> Lidar features
    ax.annotate("", xy=(0.28, 0.725), xytext=(0.18, 0.725),
                arrowprops=dict(arrowstyle="-|>", lw=2, color="#1565C0"))
    # IMU -> IMU features
    ax.annotate("", xy=(0.28, 0.275), xytext=(0.18, 0.275),
                arrowprops=dict(arrowstyle="-|>", lw=2, color="#E65100"))
    # Lidar features -> Classifier
    ax.annotate("", xy=(0.60, 0.60), xytext=(0.50, 0.70),
                arrowprops=dict(arrowstyle="-|>", lw=2, color="#2E7D32",
                                connectionstyle="arc3,rad=-0.1"))
    # IMU features -> Classifier
    ax.annotate("", xy=(0.60, 0.40), xytext=(0.50, 0.30),
                arrowprops=dict(arrowstyle="-|>", lw=2, color="#BF360C",
                                connectionstyle="arc3,rad=0.1"))
    # Classifier -> Output
    ax.annotate("", xy=(0.87, 0.50), xytext=(0.82, 0.50),
                arrowprops=dict(arrowstyle="-|>", lw=2, color="#4A148C"))

    # --- Labels on arrows ---
    ax.text(0.23, 0.76, "扇区化\n最小值", fontsize=7, ha="center", color="#555")
    ax.text(0.23, 0.31, "读取\n姿态角", fontsize=7, ha="center", color="#555")
    ax.text(0.55, 0.68, "特征\n融合", fontsize=7, ha="center", color="#555", rotation=-15)
    ax.text(0.55, 0.33, "特征\n融合", fontsize=7, ha="center", color="#555", rotation=15)

    # Title
    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图4-1_IMU与激光雷达特征互补示意图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
