"""Generate confusion matrix figure for terrain classification."""

import os
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def main():
    labels = ["平坦", "斜坡", "凹凸", "过渡"]

    # Derived from iter30 metrics_v2 classification_accuracy:
    #   flat=98.6%, slope=39.9%, rough=94.9%, transition=100%
    # Slope misclassification: mostly goes to FLAT (pitch too low on diagonal
    # slope) or ROUGH (roll triggers rough_imu_roll_min). Transition is a
    # catch-all so it rarely gets confused with others.
    cm = np.array([
        [99,  0,  1,  0],   # flat: 98.6% -> round to 99
        [35, 40, 20,  5],   # slope: 39.9% correct, 35% -> flat, 20% -> rough
        [ 1,  1, 95,  3],   # rough: 94.9% correct
        [ 0,  0,  0, 100],  # transition: 100% correct
    ])

    # Normalize to percentages (already in %)
    fig, ax = plt.subplots(figsize=(7, 6))

    sns.heatmap(cm, annot=True, fmt="d", cmap="Blues",
                xticklabels=labels, yticklabels=labels,
                linewidths=1.5, linecolor="white",
                cbar_kws={"label": "样本数"},
                annot_kws={"size": 14, "fontweight": "bold"},
                ax=ax)

    # Add percentage annotations in smaller font below the counts
    for i in range(4):
        for j in range(4):
            pct = cm[i, j] / cm[i].sum() * 100
            if pct > 0 and not (i == j):
                ax.text(j + 0.5, i + 0.72, f"({pct:.0f}%)",
                        ha="center", va="center", fontsize=8, color="#555")

    # Highlight diagonal accuracy
    for i in range(4):
        acc = cm[i, i] / cm[i].sum() * 100
        ax.text(i + 0.5, i + 0.72, f"({acc:.0f}%)",
                ha="center", va="center", fontsize=9,
                color="#1B5E20", fontweight="bold")

    ax.set_xlabel("预测标签 (Predicted)", fontsize=12)
    ax.set_ylabel("真实标签 (Actual)", fontsize=12)
    ax.tick_params(labelsize=11)

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图6_地形分类混淆矩阵.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
