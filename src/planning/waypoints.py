"""Random waypoint generation for navigation experiments."""

import random
import math


def generate_random_waypoints(num_points: int = 6,
                              min_radius: float = 5.0,
                              max_radius: float = 15.0,
                              min_separation: float = 4.0,
                              seed: int = None) -> list:
    """Generate random waypoints within a circular area.

    Args:
        num_points: Number of waypoints to generate
        min_radius: Minimum distance from origin
        max_radius: Maximum distance from origin
        min_separation: Minimum distance between consecutive waypoints
        seed: Random seed (None = use time-based seed)

    Returns:
        List of (x, y) tuples
    """
    rng = random.Random(seed)
    waypoints = []
    attempts = 0
    max_attempts = num_points * 100

    while len(waypoints) < num_points and attempts < max_attempts:
        attempts += 1
        # Generate random point in polar coordinates
        angle = rng.uniform(0, 2 * math.pi)
        radius = rng.uniform(min_radius, max_radius)
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        # Check separation from previous waypoint
        if waypoints:
            last = waypoints[-1]
            dist = math.sqrt((x - last[0])**2 + (y - last[1])**2)
            if dist < min_separation:
                continue

        waypoints.append((x, y))

    return waypoints


def generate_random_robot_start(max_radius: float = 3.0, seed: int = None) -> tuple:
    """Generate a random starting position for the robot near origin."""
    rng = random.Random(seed)
    angle = rng.uniform(0, 2 * math.pi)
    radius = rng.uniform(0, max_radius)
    return (radius * math.cos(angle), radius * math.sin(angle))
