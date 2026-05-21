"""Generate Figure 7-4: Per-terrain path_efficiency progression (small-multiples).

Four panels (one per terrain), showing how path_efficiency evolved across
all iterations for adaptive_navigator. Demonstrates the
"hard-constraint not regressed" guarantee per terrain.
"""

import os
import sys
import json

import pandas as pd
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
from _iter_data import TERRAIN_KEYS, TERRAIN_CN, TERRAIN_COLORS

METRICS_DIR = os.path.join(PROJECT_ROOT, "results", "reports", "metrics_v2")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300


def gather():
    """Collect per-terrain path_eff per iter for adaptive_navigator."""
    rows = []
    for path in sorted(os.listdir(METRICS_DIR)):
        if not path.startswith("iter_") or not path.endswith(".json"):
            continue
        it = int(path.split("_")[1].split(".")[0])
        with open(os.path.join(METRICS_DIR, path), "r", encoding="utf-8") as f:
            d = json.load(f)
        for terrain in TERRAIN_KEYS:
            cell = d["results"].get(terrain, {}).get("adaptive_navigator")
            if not cell:
                continue
            rows.append({
                "iter": it,
                "terrain": terrain,
                "path_eff": cell["path_efficiency"]["mean"],
                "path_eff_std": cell["path_efficiency"]["std"],
                "n": cell["path_efficiency"]["n"],
                "success": cell["success_rate"]["mean"],
            })
    return pd.DataFrame(rows)


def main():
    df = gather()
    df = df[df["iter"] != 12]  # drop degenerate iter

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    axes = axes.flatten()

    for idx, terrain in enumerate(TERRAIN_KEYS):
        ax = axes[idx]
        sub = df[df["terrain"] == terrain].sort_values("iter")

        valid = sub[sub["path_eff"].notna()]
        ax.plot(valid["iter"], valid["path_eff"], "o-",
                color=TERRAIN_COLORS[terrain], linewidth=2, markersize=6,
                label=f"path_eff (n>=1)")

        # Highlight low-n points (only 1 successful seed) with empty marker
        low_n = valid[valid["n"] <= 1]
        if not low_n.empty:
            ax.scatter(low_n["iter"], low_n["path_eff"],
                       marker="o", s=120, facecolors="none",
                       edgecolors=TERRAIN_COLORS[terrain], linewidths=2,
                       label="n<=1 (single-seed estimate)")

        # ± std error bars (only where std defined and n>1)
        with_std = valid[(valid["path_eff_std"] > 0) & (valid["n"] > 1)]
        if not with_std.empty:
            ax.errorbar(with_std["iter"], with_std["path_eff"],
                        yerr=with_std["path_eff_std"],
                        fmt="none", ecolor=TERRAIN_COLORS[terrain], alpha=0.3,
                        capsize=3)

        # Iter00 reference dashed
        iter00 = sub[sub["iter"] == 0]
        if not iter00.empty and not pd.isna(iter00["path_eff"].iloc[0]):
            ax.axhline(y=iter00["path_eff"].iloc[0], color="#888",
                       linestyle="--", alpha=0.6,
                       label=f"iter00 baseline ({iter00['path_eff'].iloc[0]:.2f})")

        ax.set_title(TERRAIN_CN[terrain], fontsize=12, fontweight="bold")
        ax.set_ylabel("Path efficiency", fontsize=10)
        ax.set_ylim(0, 1.05)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")
        if idx >= 2:
            ax.set_xlabel("迭代序号 (iter)", fontsize=10)

    fig.suptitle("Figure 7-4  Adaptive Navigator 各地形路径效率演进 "
                 "(iter00 → iter30, metrics_v2)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    save_path = os.path.join(FIGURES_DIR, "图7-4_各地形路径效率演进.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
