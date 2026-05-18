"""Compare navigation algorithms across terrains — generates thesis-ready figures."""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

DATA_DIR = os.path.join(PROJECT_ROOT, "data", "experiments")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures")
THESIS_DIR = os.path.join(FIGURES_DIR, "thesis")
os.makedirs(FIGURES_DIR, exist_ok=True)
os.makedirs(THESIS_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

CONTROLLERS = ["adaptive_navigator", "astar_navigator"]
CTRL_LABELS = {"adaptive_navigator": "Adaptive (Ours)", "astar_navigator": "A* Planning"}
TERRAINS = ["flat_terrain", "slope_terrain", "rough_terrain", "transition_terrain"]
TERRAIN_LABELS = {
    "flat_terrain": "Flat",
    "slope_terrain": "Slope",
    "rough_terrain": "Rough",
    "transition_terrain": "Transition",
}


def load_experiment(terrain: str, controller: str) -> pd.DataFrame:
    path = os.path.join(DATA_DIR, f"{terrain}_{controller}.csv")
    if not os.path.exists(path):
        return None
    df = pd.read_csv(path)
    dt = df["time_s"].diff().fillna(0.032)
    df["speed_actual"] = np.sqrt(df["x"].diff() ** 2 + df["y"].diff() ** 2) / dt
    df["speed_actual"] = df["speed_actual"].clip(0, 15)
    df["roll_deg"] = np.degrees(df["roll"])
    df["pitch_deg"] = np.degrees(df["pitch"])
    return df


def compute_metrics(df: pd.DataFrame) -> dict:
    if df is None or len(df) < 5:
        return None
    return {
        "avg_speed": df["speed_actual"].iloc[3:].mean(),
        "attitude_stability": np.sqrt(df["roll_deg"].std() ** 2 + df["pitch_deg"].std() ** 2),
        "path_tracking_error": df["dist_to_target"].mean(),
        "waypoints_reached": int(df["target_idx"].max()),
        "total_distance": np.sqrt(df["x"].diff() ** 2 + df["y"].diff() ** 2).iloc[1:].sum(),
    }


def plot_metrics_comparison(all_metrics: dict, save_path: str):
    """Bar chart comparing 4 metrics across terrains for both algorithms."""
    metric_names = ["avg_speed", "attitude_stability", "path_tracking_error", "waypoints_reached"]
    metric_labels = [
        "Avg Speed (m/s)",
        "Attitude Instability (deg)",
        "Avg Tracking Error (m)",
        "Waypoints Reached",
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    axes = axes.flatten()

    x = np.arange(len(TERRAINS))
    width = 0.35

    for idx, (metric, label) in enumerate(zip(metric_names, metric_labels)):
        ax = axes[idx]
        for i, ctrl in enumerate(CONTROLLERS):
            values = []
            for terrain in TERRAINS:
                key = f"{terrain}/{ctrl}"
                if key in all_metrics and all_metrics[key] is not None:
                    values.append(all_metrics[key].get(metric, 0))
                else:
                    values.append(0)
            offset = -width / 2 + i * width
            bars = ax.bar(x + offset, values, width, label=CTRL_LABELS[ctrl], alpha=0.8)

        ax.set_xlabel("Terrain Type")
        ax.set_ylabel(label)
        ax.set_title(label)
        ax.set_xticks(x)
        ax.set_xticklabels([TERRAIN_LABELS[t] for t in TERRAINS])
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")

    plt.suptitle("Algorithm Comparison Across Terrain Types", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {save_path}")


def plot_trajectory_comparison(save_path: str):
    """Side-by-side trajectory plots for each terrain."""
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))

    for col, terrain in enumerate(TERRAINS):
        for row, ctrl in enumerate(CONTROLLERS):
            ax = axes[row, col]
            df = load_experiment(terrain, ctrl)
            if df is not None:
                terrain_colors = {"flat": "#4CAF50", "slope": "#FF9800",
                                  "rough": "#F44336", "transition": "#9C27B0"}
                for t_type, color in terrain_colors.items():
                    mask = df["terrain"] == t_type
                    if mask.any():
                        ax.scatter(df.loc[mask, "x"], df.loc[mask, "y"],
                                   c=color, s=3, alpha=0.6)
                ax.plot(df["x"].iloc[0], df["y"].iloc[0], "go", markersize=8)
                ax.plot(df["x"].iloc[-1], df["y"].iloc[-1], "r^", markersize=8)
            ax.set_xlim(-6, 6)
            ax.set_ylim(-6, 6)
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title(TERRAIN_LABELS[terrain])
            if col == 0:
                ax.set_ylabel(CTRL_LABELS[ctrl])

    plt.suptitle("Trajectory Comparison", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {save_path}")


def plot_speed_comparison(save_path: str):
    """Speed profiles overlaid for both algorithms on each terrain."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()

    for idx, terrain in enumerate(TERRAINS):
        ax = axes[idx]
        for ctrl in CONTROLLERS:
            df = load_experiment(terrain, ctrl)
            if df is not None:
                ax.plot(df["time_s"], df["speed_actual"].rolling(5).mean(),
                        linewidth=1.0, label=CTRL_LABELS[ctrl], alpha=0.8)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.set_title(TERRAIN_LABELS[terrain])
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.suptitle("Speed Profile Comparison", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {save_path}")


def generate_metrics_table(all_metrics: dict) -> pd.DataFrame:
    """Generate a summary table for the thesis."""
    rows = []
    for terrain in TERRAINS:
        for ctrl in CONTROLLERS:
            key = f"{terrain}/{ctrl}"
            m = all_metrics.get(key)
            if m:
                rows.append({
                    "Terrain": TERRAIN_LABELS[terrain],
                    "Algorithm": CTRL_LABELS[ctrl],
                    **m,
                })
    return pd.DataFrame(rows)


def main():
    print("Loading experiment data...")
    all_metrics = {}
    for terrain in TERRAINS:
        for ctrl in CONTROLLERS:
            df = load_experiment(terrain, ctrl)
            key = f"{terrain}/{ctrl}"
            all_metrics[key] = compute_metrics(df)
            if df is not None:
                print(f"  {key}: {len(df)} rows")
            else:
                print(f"  {key}: NO DATA")

    has_data = any(v is not None for v in all_metrics.values())
    if not has_data:
        print("\nNo experiment data found. Run 'python scripts/run_comparison.py' first.")
        sys.exit(1)

    print("\nGenerating figures...")
    plot_metrics_comparison(all_metrics, os.path.join(FIGURES_DIR, "metrics_comparison.png"))
    plot_trajectory_comparison(os.path.join(FIGURES_DIR, "trajectory_comparison.png"))
    plot_speed_comparison(os.path.join(FIGURES_DIR, "speed_comparison.png"))

    # Thesis-named copies under results/figures/thesis/
    plot_trajectory_comparison(os.path.join(THESIS_DIR, "图6-2_四种地形轨迹对比.png"))
    plot_speed_comparison(os.path.join(THESIS_DIR, "图6-3_四种地形速度曲线对比.png"))
    plot_metrics_comparison(all_metrics, os.path.join(THESIS_DIR, "图6-4_算法对比指标柱状图.png"))

    table = generate_metrics_table(all_metrics)
    table_path = os.path.join(FIGURES_DIR, "metrics_table.csv")
    table.to_csv(table_path, index=False, encoding="utf-8-sig")
    table.to_csv(os.path.join(THESIS_DIR, "表6-1_算法对比指标.csv"),
                 index=False, encoding="utf-8-sig")
    print(f"\nMetrics table saved: {table_path}")
    print("\n" + table.to_string(index=False))


if __name__ == "__main__":
    main()
