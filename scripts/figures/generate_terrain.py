"""Generate terrain height data — dramatic, large-scale features for clear visual impact."""

import numpy as np
import os

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "worlds")

TERRAIN_SIZE = 40
RESOLUTION = 80


def _gaussian_bump(xx, yy, cx, cy, amp, sigma):
    return amp * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * sigma ** 2))


def _smooth_step(t):
    """C1-smooth step from 0 to 1 over t in [0,1]."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def generate_slope(size=TERRAIN_SIZE, resolution=RESOLUTION):
    """Steep slope (~15°) climbing across the field with embedded hill-tops and a small valley."""
    x = np.linspace(0, size, resolution)
    y = np.linspace(0, size, resolution)
    xx, yy = np.meshgrid(x, y)

    # Spawn area is at world (0,0) → local (size/2, size/2). Keep it flat so the
    # robot has a stable launch pad, then ramp up steeply away from it.
    base_angle = np.radians(15.0)
    flat_cx, flat_cy = size * 0.5, size * 0.5
    flat_radius = 4.5

    # Slope direction: diagonal so x and y both contribute → looks like a tilted hillside
    direction = np.array([0.7, 0.7])  # unit-ish vector
    slope_field = ((xx - flat_cx) * direction[0] + (yy - flat_cy) * direction[1]) * np.tan(base_angle)
    slope_field = np.maximum(slope_field, 0.0)

    dist = np.sqrt((xx - flat_cx) ** 2 + (yy - flat_cy) ** 2)
    blend = _smooth_step((dist - flat_radius) / 4.0)
    heights = blend * slope_field

    # Two prominent hilltops far from spawn for clear 3D character
    heights += _gaussian_bump(xx, yy, size * 0.85, size * 0.25, amp=2.8, sigma=3.5)
    heights += _gaussian_bump(xx, yy, size * 0.8, size * 0.85, amp=2.3, sigma=4.0)

    # A saddle / valley between the hills
    heights -= _gaussian_bump(xx, yy, size * 0.7, size * 0.55, amp=1.4, sigma=3.2)

    # Tiny micro-roughness so it's not glassy
    heights += 0.06 * np.sin(1.7 * xx) * np.cos(1.9 * yy)

    heights = np.maximum(heights, 0.0)
    return heights


def generate_rough(size=TERRAIN_SIZE, resolution=RESOLUTION):
    """Bold rolling hills — multi-octave noise with big amplitude (~3 m peaks)."""
    x = np.linspace(0, size, resolution)
    y = np.linspace(0, size, resolution)
    xx, yy = np.meshgrid(x, y)

    # Macro hills (low freq, big amp)
    heights = (
        1.5 * np.sin(0.18 * xx) * np.cos(0.22 * yy + 0.5)
        + 1.2 * np.sin(0.25 * xx + 1.3) * np.cos(0.20 * yy)
        + 0.8 * np.sin(0.35 * xx + 0.6 * yy)
    )

    # Mid-scale undulation
    heights += (
        0.5 * np.sin(0.6 * xx + 0.3) * np.cos(0.55 * yy)
        + 0.4 * np.sin(0.85 * xx + 0.7 * yy + 1.0)
    )

    # Surface texture
    heights += 0.15 * np.sin(2.4 * xx) * np.cos(2.1 * yy + 0.4)
    heights += 0.08 * np.sin(3.7 * xx + 1.5 * yy)

    # A couple of standout rocky mounds (away from spawn at center)
    heights += _gaussian_bump(xx, yy, size * 0.25, size * 0.3, amp=2.2, sigma=2.5)
    heights += _gaussian_bump(xx, yy, size * 0.78, size * 0.65, amp=2.5, sigma=2.8)
    heights -= _gaussian_bump(xx, yy, size * 0.7, size * 0.25, amp=1.5, sigma=3.0)

    # Flat-ish landing pad at spawn (world origin → local center)
    spawn_cx, spawn_cy = size * 0.5, size * 0.5
    spawn_dist = np.sqrt((xx - spawn_cx) ** 2 + (yy - spawn_cy) ** 2)
    flatten = np.exp(-(spawn_dist ** 2) / (2.0 * 2.5 ** 2))
    heights = heights * (1.0 - 0.9 * flatten)

    return heights


def generate_transition(size=TERRAIN_SIZE, resolution=RESOLUTION):
    """Flat → steep climb (≈20°) → rocky plateau → gentle descent.

    Spawn at world origin (local center) sits at the foot of the climb.
    """
    x = np.linspace(0, size, resolution)
    y = np.linspace(0, size, resolution)
    xx, yy = np.meshgrid(x, y)

    heights = np.zeros_like(xx)
    half = size / 2.0  # local center / spawn
    ramp_len = 8.0
    plateau_len = 10.0
    descent_len = 8.0

    ramp_start = half  # spawn is at the foot
    ramp_end = ramp_start + ramp_len
    plateau_end = ramp_end + plateau_len
    descent_end = plateau_end + descent_len

    steep = np.radians(20.0)
    ramp_max = ramp_len * np.tan(steep)

    # Zone: ramp
    ramp_mask = (xx >= ramp_start) & (xx < ramp_end)
    heights = np.where(ramp_mask, (xx - ramp_start) * np.tan(steep), heights)

    # Zone: plateau (rocky)
    plateau_mask = (xx >= ramp_end) & (xx < plateau_end)
    plateau_h = (
        ramp_max
        + 0.7 * np.sin(0.9 * xx + 0.3) * np.cos(0.85 * yy)
        + 0.5 * np.sin(1.4 * xx + 0.8 * yy)
        + 0.4 * np.cos(0.7 * yy + 0.5)
    )
    heights = np.where(plateau_mask, plateau_h, heights)

    # Zone: descent
    descent_mask = (xx >= plateau_end) & (xx < descent_end)
    t = (xx - plateau_end) / descent_len
    descent_h = ramp_max * (1.0 - _smooth_step(t)) + 0.4 * _smooth_step(t)
    heights = np.where(descent_mask, descent_h, heights)

    # Hold at low after descent
    heights = np.where(xx >= descent_end, 0.4, heights)

    # Prominent boulders on the plateau
    heights += _gaussian_bump(xx, yy, ramp_end + plateau_len * 0.3, size * 0.3, amp=1.8, sigma=1.8)
    heights += _gaussian_bump(xx, yy, ramp_end + plateau_len * 0.6, size * 0.7, amp=2.0, sigma=2.0)
    heights += _gaussian_bump(xx, yy, plateau_end - 1.0, size * 0.5, amp=1.2, sigma=1.5)

    # Cross-axis undulation to break monotony
    heights += 0.15 * np.sin(0.4 * yy) * np.cos(0.3 * xx)

    # Light flatten right at spawn so robot doesn't slide
    spawn_dist = np.sqrt((xx - half) ** 2 + (yy - half) ** 2)
    flatten = np.exp(-(spawn_dist ** 2) / (2.0 * 1.8 ** 2))
    heights = heights * (1.0 - 0.85 * flatten)

    return heights


def heights_to_string(heights):
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
