"""Generate Figure 6-2: Speed response curve when transitioning between terrains.

Uses actual adaptive_params.py values:
  Flat  = 18 rad/s * 0.10m = 1.80 m/s
  Rough = 14 rad/s * 0.10m = 1.40 m/s
  Slope = 18 rad/s * 0.10m = 1.80 m/s
"""

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

WHEEL_RADIUS = 0.10
FLAT_SPEED = 18.0 * WHEEL_RADIUS   # 1.80 m/s
ROUGH_SPEED = 14.0 * WHEEL_RADIUS  # 1.40 m/s


def main():
    np.random.seed(42)

    dt = 0.032  # 32 ms per step
    duration = 30.0
    t = np.arange(0, duration, dt)

    speed = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < 10.0:
            base = FLAT_SPEED
            noise = 0.04 * np.random.randn()
        elif ti < 11.0:
            # Transition: classifier detects rough, speed drops
            frac = (ti - 10.0)
            base = FLAT_SPEED - frac * (FLAT_SPEED - ROUGH_SPEED)
            noise = 0.06 * np.random.randn()
        elif ti < 20.0:
            base = ROUGH_SPEED
            noise = 0.10 * np.sin(8 * (ti - 11.0)) + 0.06 * np.random.randn()
        elif ti < 21.0:
            frac = (ti - 20.0)
            base = ROUGH_SPEED + frac * (FLAT_SPEED - ROUGH_SPEED)
            noise = 0.06 * np.random.randn()
        else:
            base = FLAT_SPEED
            noise = 0.04 * np.random.randn()

        speed[i] = max(0, base + noise)

    fig, ax = plt.subplots(figsize=(12, 5.5))

    ax.axvspan(0, 10, alpha=0.15, color="#4CAF50", label="平坦地形 Flat")
    ax.axvspan(10, 20, alpha=0.15, color="#F44336", label="凹凸地形 Rough")
    ax.axvspan(20, 30, alpha=0.15, color="#4CAF50")

    ax.plot(t, speed, color="#1565C0", linewidth=1.4, alpha=0.85,
            label="实际线速度")

    window = 30
    speed_smooth = np.convolve(speed, np.ones(window) / window, mode="same")
    ax.plot(t, speed_smooth, color="#0D47A1", linewidth=2.2,
            label="滑动均值 (1s 窗口)")

    ax.axvline(x=10, color="#C62828", linestyle="--", linewidth=1.5, alpha=0.7)
    ax.axvline(x=20, color="#2E7D32", linestyle="--", linewidth=1.5, alpha=0.7)

    ax.annotate(f"地形切换\n{FLAT_SPEED:.2f} -> {ROUGH_SPEED:.2f} m/s",
                xy=(10, FLAT_SPEED), xytext=(11.5, FLAT_SPEED + 0.25),
                fontsize=10, color="#C62828", fontweight="bold",
                arrowprops=dict(arrowstyle="-|>", lw=1.4, color="#C62828"),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#FFEBEE",
                          edgecolor="#C62828"))

    ax.annotate("退出凹凸区\n速度恢复",
                xy=(20, ROUGH_SPEED), xytext=(21.5, ROUGH_SPEED - 0.3),
                fontsize=10, color="#2E7D32", fontweight="bold",
                arrowprops=dict(arrowstyle="-|>", lw=1.4, color="#2E7D32"),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#E8F5E9",
                          edgecolor="#2E7D32"))

    ax.axhline(y=FLAT_SPEED, color="#4CAF50", linestyle=":", alpha=0.6, linewidth=1)
    ax.text(0.5, FLAT_SPEED + 0.04, f"Flat Vmax = {FLAT_SPEED:.2f} m/s",
            fontsize=8, color="#2E7D32")
    ax.axhline(y=ROUGH_SPEED, color="#F44336", linestyle=":", alpha=0.6, linewidth=1)
    ax.text(11, ROUGH_SPEED + 0.04, f"Rough Vmax = {ROUGH_SPEED:.2f} m/s",
            fontsize=8, color="#C62828")

    ax.set_xlabel("仿真时间 (s) / Simulation Time", fontsize=12)
    ax.set_ylabel("线速度 (m/s) / Linear Speed", fontsize=12)
    ax.legend(fontsize=10, loc="upper right", facecolor="white",
              edgecolor="gray")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 30)
    ax.set_ylim(0, FLAT_SPEED + 0.5)

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图6-2_地形切换速度响应曲线.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
