"""Generate Figure 7-3: Rough-terrain breakthrough timeline.

Highlights the iter21 -> iter30 cycle, isolating rough-completion and
rough-success trajectories with each merged iter's contribution annotated.
This is the headline figure for Chapter 7's "iter21-30 cycle" section.
"""

import os
import sys
import json

import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
METRICS_DIR = os.path.join(PROJECT_ROOT, "results", "reports", "metrics_v2")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300

ITERS = list(range(20, 31))
ANNOTATIONS = {
    22: ("merged: max(|pitch|,|roll|)\n分类规则", "#2E7D32"),
    24: ("merged: rough\nalign_floor 0.65→0.85", "#2E7D32"),
    25: ("merged: DWA rough\npredict_time 1.2→2.0s", "#2E7D32"),
    30: ("merged: rough replan\n400→150 步", "#2E7D32"),
}
REJECTED = {21, 23, 26, 27, 28, 29}


def load_rough(it):
    p = os.path.join(METRICS_DIR, f"iter_{it:02d}.json")
    if not os.path.exists(p):
        return None, None, None
    with open(p, "r", encoding="utf-8") as f:
        d = json.load(f)
    rough = d["results"]["rough"]["adaptive_navigator"]
    return (rough["completion_fraction"]["mean"],
            rough["success_rate"]["mean"],
            rough["completion_fraction"]["std"])


def main():
    completion = []
    success = []
    std = []
    for it in ITERS:
        c, s, sd = load_rough(it)
        completion.append(c)
        success.append(s)
        std.append(sd)

    fig, ax = plt.subplots(figsize=(13, 7))

    # Completion ± std band
    upper = [c + s if c is not None and s is not None else None
             for c, s in zip(completion, std)]
    lower = [max(0, c - s) if c is not None and s is not None else None
             for c, s in zip(completion, std)]
    ax.fill_between(ITERS, lower, upper, color="#FFAB40", alpha=0.25,
                    label="Completion ± std")
    ax.plot(ITERS, completion, "o-", color="#E65100", linewidth=2.2,
            markersize=8, label="Rough completion (5-seed mean)")
    ax.plot(ITERS, success, "D-", color="#BF360C", linewidth=2.2,
            markersize=7, label="Rough success rate")

    # Mark accept/reject
    for it, c, s in zip(ITERS, completion, success):
        if it in REJECTED:
            ax.scatter([it], [c], marker="x", s=120, color="#C62828",
                       linewidths=2.5, zorder=5)
        elif it in ANNOTATIONS:
            ax.scatter([it], [c], marker="^", s=160, color="#2E7D32",
                       edgecolors="black", linewidths=1.0, zorder=5)

    # Annotate merges
    y_offsets = {22: 0.20, 24: 0.30, 25: 0.20, 30: 0.30}
    x_offsets = {22: -1.0, 24: -1.2, 25: 0.5, 30: -1.2}
    for it, (label, color) in ANNOTATIONS.items():
        idx = ITERS.index(it)
        c = completion[idx]
        if c is None:
            continue
        ax.annotate(label, xy=(it, c),
                    xytext=(it + x_offsets[it], c + y_offsets[it]),
                    fontsize=9, color=color, fontweight="bold",
                    ha="center",
                    arrowprops=dict(arrowstyle="-|>", lw=1.2, color=color),
                    bbox=dict(boxstyle="round,pad=0.3",
                              facecolor="#E8F5E9", edgecolor=color))

    ax.set_xlabel("迭代序号 (iter)", fontsize=12)
    ax.set_ylabel("Rough 地形指标", fontsize=12)
    ax.set_title("Figure 7-3  Rough 地形迭代突破时间轴 (iter20 → iter30)",
                 fontsize=13, fontweight="bold")
    ax.set_xticks(ITERS)
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=10)

    # Add a "before" reference line
    ax.axhline(y=0.30, color="#999", linestyle=":", alpha=0.7)
    ax.text(20.2, 0.31, "iter20 baseline (0.30)", fontsize=8, color="#666")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图7-3_Rough地形突破时间轴.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
