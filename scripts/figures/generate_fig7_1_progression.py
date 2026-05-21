"""Generate Figure 7-1: Iteration progression curves (iter00 -> iter30).

Three-panel plot:
  (a) success_rate + completion_fraction overall vs iter
  (b) path_efficiency overall vs iter
  (c) rough_completion + rough_success vs iter

Merged iters marked with green ✓; rejected with red ✗. The "rough breakthrough"
at iter30 (success 0 -> 0.20) gets a callout.

Requires progression.csv to be up to date — run rebuild_progression.py first.
"""

import os
import sys

import pandas as pd
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PROG_PATH = os.path.join(PROJECT_ROOT, "results", "thesis_archive", "progression.csv")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def main():
    df = pd.read_csv(PROG_PATH)
    df = df[df["iter"].notna()].copy()
    df["iter"] = df["iter"].astype(int)
    # Skip iter12 — degenerate run (DWA broke navigation, success_rate=1.0
    # is misleading because the controller was crashing fast, not succeeding)
    df = df[df["iter"] != 12].sort_values("iter").reset_index(drop=True)

    accepted = df[df["accept"] == "Y"]
    rejected = df[df["accept"] == "N"]

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True)

    # --- Panel (a): success + completion ---
    ax = axes[0]
    ax.plot(df["iter"], df["success_overall"], "o-",
            color="#1976D2", linewidth=2, markersize=6, label="Success rate")
    ax.plot(df["iter"], df["completion_overall"], "s--",
            color="#0D47A1", linewidth=1.5, markersize=5, alpha=0.7,
            label="Completion fraction")
    ax.scatter(accepted["iter"], accepted["success_overall"],
               marker="^", s=120, color="#2E7D32", zorder=5,
               edgecolors="black", linewidths=1.0,
               label="Accepted (merged)")
    ax.scatter(rejected["iter"], rejected["success_overall"],
               marker="x", s=80, color="#C62828", zorder=5,
               linewidths=2.0, label="Rejected")
    ax.set_ylabel("成功率 / 完成度", fontsize=11)
    ax.set_title("(a) 整体成功率 & 完成度 (4 地形 × 5 seed 均值)",
                 fontsize=11, fontweight="bold")
    ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=9, ncol=2)

    # --- Panel (b): path efficiency ---
    ax = axes[1]
    ax.plot(df["iter"], df["path_eff_overall"], "o-",
            color="#7B1FA2", linewidth=2, markersize=6, label="Path efficiency")
    ax.scatter(accepted["iter"], accepted["path_eff_overall"],
               marker="^", s=120, color="#2E7D32", zorder=5,
               edgecolors="black", linewidths=1.0)
    ax.scatter(rejected["iter"], rejected["path_eff_overall"],
               marker="x", s=80, color="#C62828", zorder=5, linewidths=2.0)
    ax.set_ylabel("路径效率 (TSP-corrected)", fontsize=11)
    ax.set_title("(b) 整体路径效率 (TSP 参考长度归一)",
                 fontsize=11, fontweight="bold")
    ax.grid(True, alpha=0.3)

    # --- Panel (c): rough specifically ---
    ax = axes[2]
    ax.plot(df["iter"], df["rough_completion"], "o-",
            color="#E65100", linewidth=2, markersize=6,
            label="Rough completion")
    ax.plot(df["iter"], df["rough_success"], "D-",
            color="#BF360C", linewidth=2, markersize=6,
            label="Rough success rate")
    ax.scatter(accepted["iter"], accepted["rough_completion"],
               marker="^", s=120, color="#2E7D32", zorder=5,
               edgecolors="black", linewidths=1.0)

    # Annotate iter30 breakthrough
    iter30_row = df[df["iter"] == 30].iloc[0]
    ax.annotate(f"iter30: rough success\n0% -> {iter30_row['rough_success']*100:.0f}%\n(首次破零)",
                xy=(30, iter30_row["rough_success"]),
                xytext=(22, 0.45),
                fontsize=10, fontweight="bold", color="#BF360C",
                arrowprops=dict(arrowstyle="-|>", lw=1.5, color="#BF360C"),
                bbox=dict(boxstyle="round,pad=0.4",
                          facecolor="#FFEBEE", edgecolor="#BF360C"))

    ax.set_ylabel("Rough 地形指标", fontsize=11)
    ax.set_title("(c) 凹凸地形完成度 / 成功率 (项目主要瓶颈)",
                 fontsize=11, fontweight="bold")
    ax.set_xlabel("迭代序号 (iter)", fontsize=11)
    ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=9)

    # Tick every iter, highlight gaps
    axes[-1].set_xticks(df["iter"])
    axes[-1].set_xticklabels(df["iter"], fontsize=8)

    fig.suptitle("Figure 7-1  迭代演进曲线 (iter00 -> iter30, metrics_v2)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    save_path = os.path.join(FIGURES_DIR, "图7-1_迭代演进曲线.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
