"""Visualize navigation experiment results from CSV logs."""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)

RESULTS_DIR = os.path.join(PROJECT_ROOT, "results", "figures")
os.makedirs(RESULTS_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def load_log(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df["speed_actual"] = np.sqrt(
        df["x"].diff() ** 2 + df["y"].diff() ** 2
    ) / df["time_s"].diff()
    df["roll_deg"] = np.degrees(df["roll"])
    df["pitch_deg"] = np.degrees(df["pitch"])
    df["yaw_deg"] = np.degrees(df["yaw"])
    return df


def plot_trajectory(df: pd.DataFrame, title: str, save_path: str):
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    terrain_colors = {"flat": "#4CAF50", "slope": "#FF9800", "rough": "#F44336", "transition": "#9C27B0"}

    for terrain_type, color in terrain_colors.items():
        mask = df["terrain"] == terrain_type
        if mask.any():
            ax.scatter(df.loc[mask, "x"], df.loc[mask, "y"],
                       c=color, s=10, label=terrain_type, alpha=0.7)

    ax.plot(df["x"].iloc[0], df["y"].iloc[0], "go", markersize=12, label="Start")
    ax.plot(df["x"].iloc[-1], df["y"].iloc[-1], "r^", markersize=12, label="End")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Saved: {save_path}")


def plot_speed_profile(df: pd.DataFrame, title: str, save_path: str):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax1.plot(df["time_s"], df["speed_actual"], "b-", linewidth=0.8, label="Actual speed")
    ax1.plot(df["time_s"], df["speed"], "r--", linewidth=1.0, label="Max allowed")
    ax1.set_ylabel("Speed (m/s)")
    ax1.set_title(title)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    terrain_colors = {"flat": "#4CAF50", "slope": "#FF9800", "rough": "#F44336", "transition": "#9C27B0"}
    for i, row in df.iterrows():
        color = terrain_colors.get(row["terrain"], "gray")
        ax2.axvspan(row["time_s"] - 0.16, row["time_s"] + 0.16,
                    color=color, alpha=0.4)

    for terrain_type, color in terrain_colors.items():
        ax2.axvspan(0, 0, color=color, alpha=0.4, label=terrain_type)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Terrain")
    ax2.set_yticks([])
    ax2.legend(loc="upper right", ncol=4)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Saved: {save_path}")


def plot_attitude(df: pd.DataFrame, title: str, save_path: str):
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(df["time_s"], df["roll_deg"], "r-", linewidth=0.8)
    axes[0].set_ylabel("Roll (deg)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(df["time_s"], df["pitch_deg"], "g-", linewidth=0.8)
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(df["time_s"], df["yaw_deg"], "b-", linewidth=0.8)
    axes[2].set_ylabel("Yaw (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    axes[0].set_title(title)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Saved: {save_path}")


def plot_tracking_error(df: pd.DataFrame, targets: list, title: str, save_path: str):
    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    ax.plot(df["time_s"], df["dist_to_target"], "b-", linewidth=0.8)
    ax.axhline(y=0.3, color="r", linestyle="--", label="Tolerance (0.3m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance to target (m)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Saved: {save_path}")


def compute_metrics(df: pd.DataFrame) -> dict:
    metrics = {
        "avg_speed": df["speed_actual"].mean(),
        "max_speed": df["speed_actual"].max(),
        "roll_std": df["roll_deg"].std(),
        "pitch_std": df["pitch_deg"].std(),
        "attitude_stability": np.sqrt(df["roll_deg"].std() ** 2 + df["pitch_deg"].std() ** 2),
        "avg_tracking_error": df["dist_to_target"].mean(),
        "max_tracking_error": df["dist_to_target"].max(),
        "total_time": df["time_s"].iloc[-1],
        "total_distance": np.sqrt(df["x"].diff() ** 2 + df["y"].diff() ** 2).sum(),
        "waypoints_reached": df["target_idx"].max(),
    }
    return metrics


def analyze_log(csv_path: str, terrain_name: str):
    print(f"\nAnalyzing: {terrain_name} ({csv_path})")
    df = load_log(csv_path)

    prefix = terrain_name.lower().replace(" ", "_")
    plot_trajectory(df, f"Trajectory - {terrain_name}",
                    os.path.join(RESULTS_DIR, f"{prefix}_trajectory.png"))
    plot_speed_profile(df, f"Speed Profile - {terrain_name}",
                       os.path.join(RESULTS_DIR, f"{prefix}_speed.png"))
    plot_attitude(df, f"Attitude (RPY) - {terrain_name}",
                  os.path.join(RESULTS_DIR, f"{prefix}_attitude.png"))
    plot_tracking_error(df, [], f"Tracking Error - {terrain_name}",
                        os.path.join(RESULTS_DIR, f"{prefix}_tracking.png"))

    metrics = compute_metrics(df)
    print(f"  Metrics:")
    for k, v in metrics.items():
        print(f"    {k}: {v:.4f}")

    return metrics


if __name__ == "__main__":
    log_dir = os.path.join(PROJECT_ROOT, "data", "logs")
    logs = [f for f in os.listdir(log_dir) if f.endswith(".csv")]

    if not logs:
        print("No CSV logs found. Run experiments first.")
        sys.exit(1)

    all_metrics = {}
    for log_file in sorted(logs):
        name = log_file.replace(".csv", "").replace("_", " ").title()
        m = analyze_log(os.path.join(log_dir, log_file), name)
        all_metrics[name] = m

    if len(all_metrics) > 1:
        metrics_df = pd.DataFrame(all_metrics).T
        metrics_df.to_csv(os.path.join(RESULTS_DIR, "metrics_summary.csv"))
        print(f"\nMetrics summary saved to: {os.path.join(RESULTS_DIR, 'metrics_summary.csv')}")
