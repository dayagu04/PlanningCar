"""Terrain height sampling — read elevation grid file and bilinearly interpolate."""

import os
from functools import lru_cache


# Elevation grid layout (matches scripts/generate_worlds.py)
TERRAIN_SIZE = 40.0          # Total span in meters per axis
RESOLUTION = 80              # 80 x 80 vertices
SPACING = TERRAIN_SIZE / (RESOLUTION - 1)
ORIGIN_OFFSET = -TERRAIN_SIZE / 2.0  # Terrain Solid translated to (-20, -20, 0)


@lru_cache(maxsize=8)
def _load_heights(heights_path: str):
    with open(heights_path, "r") as f:
        tokens = f.read().split()
    heights = [float(t) for t in tokens]
    if len(heights) != RESOLUTION * RESOLUTION:
        raise ValueError(
            f"Heights file {heights_path} has {len(heights)} values, "
            f"expected {RESOLUTION * RESOLUTION}"
        )
    return tuple(heights)


def _heights_path_for(world_file: str, worlds_dir: str):
    """Map world file → corresponding heights file (or None for flat)."""
    base = os.path.basename(world_file).replace(".wbt", "").replace("_temp", "")
    mapping = {
        "slope_terrain": "slope_heights.txt",
        "rough_terrain": "rough_heights.txt",
        "transition_terrain": "transition_heights.txt",
    }
    fname = mapping.get(base)
    if fname is None:
        return None
    return os.path.join(worlds_dir, fname)


def sample_terrain_height(world_file: str, worlds_dir: str, x: float, y: float) -> float:
    """Bilinearly interpolate the terrain height at world (x, y).

    Returns 0.0 for flat terrain or when the point is outside the grid.
    """
    heights_path = _heights_path_for(world_file, worlds_dir)
    if heights_path is None or not os.path.exists(heights_path):
        return 0.0

    heights = _load_heights(heights_path)

    # World → local (terrain Solid is translated by (-20, -20, 0))
    lx = x - ORIGIN_OFFSET
    ly = y - ORIGIN_OFFSET

    # Convert to grid coordinates
    gx = lx / SPACING
    gy = ly / SPACING

    if gx < 0 or gy < 0 or gx > (RESOLUTION - 1) or gy > (RESOLUTION - 1):
        return 0.0

    i0 = int(gx)
    j0 = int(gy)
    i1 = min(i0 + 1, RESOLUTION - 1)
    j1 = min(j0 + 1, RESOLUTION - 1)
    fx = gx - i0
    fy = gy - j0

    def h(i, j):
        return heights[j * RESOLUTION + i]

    h00 = h(i0, j0)
    h10 = h(i1, j0)
    h01 = h(i0, j1)
    h11 = h(i1, j1)

    h0 = h00 * (1 - fx) + h10 * fx
    h1 = h01 * (1 - fx) + h11 * fx
    return h0 * (1 - fy) + h1 * fy
