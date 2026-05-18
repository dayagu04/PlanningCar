"""Generate terrain height data for Webots ElevationGrid nodes."""

import numpy as np
import os

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "worlds")


def generate_slope(size=20, resolution=40, max_angle_deg=5):
    """Generate a slope terrain (tilted plane)."""
    x = np.linspace(0, size, resolution)
    y = np.linspace(0, size, resolution)
    xx, _ = np.meshgrid(x, y)
    slope_rad = np.radians(max_angle_deg)
    heights = xx * np.tan(slope_rad)
    return heights


def generate_rough(size=20, resolution=40, amplitude=0.15, frequency=3):
    """Generate rough/bumpy terrain using superimposed sine waves."""
    x = np.linspace(0, size, resolution)
    y = np.linspace(0, size, resolution)
    xx, yy = np.meshgrid(x, y)
    heights = (amplitude * np.sin(frequency * xx) * np.cos(frequency * yy)
               + amplitude * 0.5 * np.sin(frequency * 2.3 * xx + 1.0)
               + amplitude * 0.3 * np.cos(frequency * 1.7 * yy + 0.5))
    return heights


def generate_transition(size=20, resolution=40):
    """Generate transition terrain: flat -> slope -> rough -> flat."""
    x = np.linspace(0, size, resolution)
    y = np.linspace(0, size, resolution)
    xx, yy = np.meshgrid(x, y)
    heights = np.zeros_like(xx)

    quarter = size / 4.0
    flat1 = xx < quarter
    slope_mask = (xx >= quarter) & (xx < 2 * quarter)
    rough_mask = (xx >= 2 * quarter) & (xx < 3 * quarter)

    slope_angle = np.radians(15)
    heights[slope_mask] = (xx[slope_mask] - quarter) * np.tan(slope_angle)
    max_slope_h = quarter * np.tan(slope_angle)

    heights[rough_mask] = (max_slope_h
                           + 0.1 * np.sin(5 * xx[rough_mask]) * np.cos(5 * yy[rough_mask]))

    flat2 = xx >= 3 * quarter
    heights[flat2] = max_slope_h

    return heights


def heights_to_string(heights):
    """Convert height array to space-separated string for Webots."""
    flat = heights.flatten()
    return " ".join(f"{h:.4f}" for h in flat)


if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for name, gen_func in [("slope", generate_slope),
                           ("rough", generate_rough),
                           ("transition", generate_transition)]:
        h = gen_func()
        out_path = os.path.join(OUTPUT_DIR, f"{name}_heights.txt")
        with open(out_path, "w") as f:
            f.write(heights_to_string(h))
        print(f"Generated {name}: shape={h.shape}, range=[{h.min():.3f}, {h.max():.3f}]")
        print(f"  Saved to: {out_path}")
