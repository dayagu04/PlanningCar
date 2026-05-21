"""Generate Figure 4-3: IMU-assisted classification accuracy comparison bar chart."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def main():
    # From iter30 metrics_v2 classification_accuracy (adaptive_navigator):
    #   flat=98.6%, slope=39.9%, rough=94.9%, transition=100%
    # The "lidar only" scenario: without IMU, slope detection fails entirely
    # (slope_imu_pitch_min threshold can't fire), rough detection degrades
    # (rough_imu_roll_min can't fire).
    categories = ["平坦\n(Flat)", "斜坡\n(Slope)", "凹凸\n(Rough)", "过渡\n(Transition)"]
    lidar_only = [99, 0, 75, 85]
    lidar_imu = [99, 40, 95, 100]

    x = np.arange(len(categories))
    width = 0.32

    fig, ax = plt.subplots(figsize=(9, 5.5))

    bars1 = ax.bar(x - width / 2, lidar_only, width,
                   label="仅激光雷达 (Lidar Only)",
                   color="#FF7043", alpha=0.85, edgecolor="black", linewidth=0.8)
    bars2 = ax.bar(x + width / 2, lidar_imu, width,
                   label="激光雷达 + IMU (Lidar + IMU)",
                   color="#42A5F5", alpha=0.85, edgecolor="black", linewidth=0.8)

    # Value labels
    for bar in bars1:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, h + 1.5,
                f"{int(h)}%", ha="center", va="bottom", fontsize=11, fontweight="bold",
                color="#D84315")
    for bar in bars2:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, h + 1.5,
                f"{int(h)}%", ha="center", va="bottom", fontsize=11, fontweight="bold",
                color="#1565C0")

    # Highlight the key improvement on slope
    ax.annotate("IMU使斜坡可检测\n(0% -> 40%)",
                xy=(1 + width / 2, 40), xytext=(2.0, 60),
                fontsize=10, color="#C62828", fontweight="bold",
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#C62828"),
                ha="center")

    ax.set_xlabel("地形类型 (Terrain Type)", fontsize=12)
    ax.set_ylabel("分类准确率 (%) \nClassification Accuracy (%)", fontsize=12)
    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontsize=11)
    ax.set_ylim(0, 110)
    ax.legend(fontsize=11, loc="upper left")
    ax.grid(True, alpha=0.3, axis="y")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图4-3_IMU辅助分类效果对比图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
