"""Extract terrain features from RangeFinder depth image."""

import numpy as np


def rangefinder_to_height_grid(depth_image: np.ndarray, max_range: float = 5.0) -> np.ndarray:
    """Convert RangeFinder depth image to height grid.

    Args:
        depth_image: 2D array from RangeFinder.getRangeImage(), shape (height, width)
        max_range: Maximum valid range in meters

    Returns:
        Height grid (downsampled to reasonable size for feature extraction)
    """
    if depth_image is None or depth_image.size == 0:
        return np.zeros((8, 8))

    # Filter invalid readings
    valid_mask = (depth_image > 0.1) & (depth_image < max_range)
    depth_filtered = np.where(valid_mask, depth_image, np.nan)

    # Downsample to 8x8 grid for efficiency
    h, w = depth_filtered.shape
    grid_h, grid_w = 8, 8
    cell_h = h // grid_h
    cell_w = w // grid_w

    height_grid = np.zeros((grid_h, grid_w))
    for i in range(grid_h):
        for j in range(grid_w):
            cell = depth_filtered[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]
            height_grid[i, j] = np.nanmean(cell) if not np.all(np.isnan(cell)) else 0.0

    # Convert depth to relative height (closer = higher obstacle)
    max_depth = np.nanmax(height_grid)
    if max_depth > 0:
        height_grid = max_depth - height_grid

    return height_grid


def compute_depth_variance(depth_image: np.ndarray) -> float:
    """Compute depth variance as a roughness indicator."""
    if depth_image is None or depth_image.size == 0:
        return 0.0
    valid = depth_image[(depth_image > 0.1) & (depth_image < 5.0)]
    return float(np.std(valid)) if len(valid) > 10 else 0.0


def compute_forward_slope(depth_image: np.ndarray, fov_vertical: float = 0.7854) -> float:
    """Estimate forward slope from depth image vertical gradient.

    Args:
        depth_image: 2D depth array
        fov_vertical: Vertical field of view in radians (default 45°)

    Returns:
        Estimated slope in degrees
    """
    if depth_image is None or depth_image.size == 0:
        return 0.0

    h, w = depth_image.shape
    # Compare top half vs bottom half
    top_half = depth_image[:h//2, :]
    bottom_half = depth_image[h//2:, :]

    top_mean = np.nanmean(top_half[top_half > 0.1])
    bottom_mean = np.nanmean(bottom_half[bottom_half > 0.1])

    if np.isnan(top_mean) or np.isnan(bottom_mean):
        return 0.0

    # Depth difference indicates slope
    depth_diff = bottom_mean - top_mean
    angle_per_pixel = fov_vertical / h
    slope_rad = np.arctan(depth_diff / (h/2 * angle_per_pixel))
    return float(np.degrees(slope_rad))
