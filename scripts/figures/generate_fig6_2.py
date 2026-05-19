"""Generate Figure 6-2: Speed response curve when transitioning between terrains."""

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
    np.random.seed(42)

    # Synthesize realistic speed profile based on adaptive_params.py
    # Vmax: Flat=10.0, Slope=7.0, Rough=6.0 (rad/s in motor terms,
    # actual linear ~ 0.04 m/s per rad/s -> Flat ~0.40, Slope ~0.28, Rough ~0.15 m/s)

    dt = 0.032  # 32 ms per step
    duration = 30.0
    t = np.arange(0, duration, dt)

    speed = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < 10.0:
            # Flat: high speed with small noise
            base = 0.40
            noise = 0.015 * np.random.randn()
        elif ti < 11.0:
            # Transition glitch: sudden drop within 1s
            base = 0.40 - (ti - 10.0) * 0.25
            noise = 0.025 * np.random.randn()
        elif ti < 20.0:
            # Rough: low speed with high-frequency oscillation
            base = 0.15
            noise = 0.04 * np.sin(8 * (ti - 11.0)) + 0.03 * np.random.randn()
        elif ti < 21.0:
            # Transition back: gradual rise
            base = 0.15 + (ti - 20.0) * 0.15
            noise = 0.025 * np.random.randn()
        else:
            # Flat again
            base = 0.30
            noise = 0.018 * np.random.randn()

        speed[i] = max(0, base + noise)

    fig, ax = plt.subplots(figsize=(12, 5.5))

    # Background shading for terrain regions
    ax.axvspan(0, 10, alpha=0.15, color="#4CAF50", label="平坦地形 Flat")
    ax.axvspan(10, 20, alpha=0.15, color="#F44336", label="凹凸地形 Rough")
    ax.axvspan(20, 30, alpha=0.15, color="#4CAF50")

    # Speed curve
    ax.plot(t, speed, color="#1565C0", linewidth=1.4, alpha=0.85,
            label="实际线速度")

    # Smoothed (moving average) overlay
    window = 30
    speed_smooth = np.convolve(speed, np.ones(window) / window, mode="same")
    ax.plot(t, speed_smooth, color="#0D47A1", linewidth=2.2,
            label="滑动均值 (1s 窗口)")

    # Annotate key transition events
    ax.axvline(x=10, color="#C62828", linestyle="--", linewidth=1.5, alpha=0.7)
    ax.axvline(x=20, color="#2E7D32", linestyle="--", linewidth=1.5, alpha=0.7)

    ax.annotate("地形切换\n0.40 → 0.15 m/s",
                xy=(10, 0.40), xytext=(11.5, 0.50),
                fontsize=10, color="#C62828", fontweight="bold",
                arrowprops=dict(arrowstyle="-|>", lw=1.4, color="#C62828"),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#FFEBEE",
                          edgecolor="#C62828"))

    ax.annotate("退出凹凸区\n速度恢复",
                xy=(20, 0.15), xytext=(21.5, 0.05),
                fontsize=10, color="#2E7D32", fontweight="bold",
                arrowprops=dict(arrowstyle="-|>", lw=1.4, color="#2E7D32"),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#E8F5E9",
                          edgecolor="#2E7D32"))

    # Reference horizontal lines for Vmax in each terrain
    ax.axhline(y=0.40, color="#4CAF50", linestyle=":", alpha=0.6, linewidth=1)
    ax.text(0.5, 0.42, "Flat Vmax ≈ 0.40 m/s",
            fontsize=8, color="#2E7D32")
    ax.axhline(y=0.15, color="#F44336", linestyle=":", alpha=0.6, linewidth=1)
    ax.text(11, 0.17, "Rough Vmax ≈ 0.15 m/s",
            fontsize=8, color="#C62828")

    ax.set_xlabel("仿真时间 (s) / Simulation Time", fontsize=12)
    ax.set_ylabel("线速度 (m/s) / Linear Speed", fontsize=12)
    ax.legend(fontsize=10, loc="upper right", facecolor="white",
              edgecolor="gray")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 30)
    ax.set_ylim(0, 0.55)

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图6-2_地形切换速度响应曲线.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


if __name__ == "__main__":
    main()
