"""Waypoint ordering optimization (TSP solver).

Uses greedy nearest-neighbor + 2-opt local search to find shortest tour.
"""

import math
from typing import List, Tuple


def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def tour_length(start: Tuple[float, float], tour: List[Tuple[float, float]]) -> float:
    """Compute total length of tour starting from `start`."""
    if not tour:
        return 0.0
    total = euclidean_distance(start, tour[0])
    for i in range(len(tour) - 1):
        total += euclidean_distance(tour[i], tour[i + 1])
    return total


def nearest_neighbor(start: Tuple[float, float],
                     waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Greedy nearest-neighbor tour."""
    remaining = list(waypoints)
    tour = []
    current = start

    while remaining:
        nearest_idx = 0
        nearest_dist = euclidean_distance(current, remaining[0])
        for i in range(1, len(remaining)):
            d = euclidean_distance(current, remaining[i])
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i
        next_point = remaining.pop(nearest_idx)
        tour.append(next_point)
        current = next_point

    return tour


def two_opt(start: Tuple[float, float],
            tour: List[Tuple[float, float]],
            max_iter: int = 100) -> List[Tuple[float, float]]:
    """2-opt local search: reverse segments to reduce tour length."""
    if len(tour) < 4:
        return tour

    best = list(tour)
    best_length = tour_length(start, best)
    improved = True
    iteration = 0

    while improved and iteration < max_iter:
        improved = False
        iteration += 1

        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                new_tour = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                new_length = tour_length(start, new_tour)
                if new_length < best_length - 1e-6:
                    best = new_tour
                    best_length = new_length
                    improved = True

    return best


def optimize_waypoint_order(start: Tuple[float, float],
                            waypoints: List[Tuple[float, float]]) -> Tuple[List[Tuple[float, float]], dict]:
    """Optimize waypoint visit order to minimize total distance.

    Args:
        start: Robot starting position
        waypoints: List of waypoints to visit

    Returns:
        (optimized_tour, info_dict) where info_dict contains:
            - original_length: tour length with input order
            - greedy_length: after nearest-neighbor
            - optimized_length: after 2-opt
            - improvement_pct: percentage improvement
    """
    if not waypoints:
        return [], {"original_length": 0, "greedy_length": 0, "optimized_length": 0, "improvement_pct": 0}

    original_length = tour_length(start, waypoints)
    greedy_tour = nearest_neighbor(start, waypoints)
    greedy_length = tour_length(start, greedy_tour)
    optimized_tour = two_opt(start, greedy_tour)
    optimized_length = tour_length(start, optimized_tour)

    improvement = 100 * (original_length - optimized_length) / original_length if original_length > 0 else 0

    info = {
        "original_length": original_length,
        "greedy_length": greedy_length,
        "optimized_length": optimized_length,
        "improvement_pct": improvement,
    }

    return optimized_tour, info
