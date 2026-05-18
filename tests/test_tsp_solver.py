"""Tests for TSP solver."""

import math
from src.planning.tsp_solver import (
    nearest_neighbor,
    two_opt,
    optimize_waypoint_order,
    tour_length,
)


def test_nearest_neighbor_picks_closest():
    start = (0.0, 0.0)
    waypoints = [(10.0, 0.0), (1.0, 0.0), (5.0, 0.0)]
    tour = nearest_neighbor(start, waypoints)
    assert tour[0] == (1.0, 0.0)
    assert tour[1] == (5.0, 0.0)
    assert tour[2] == (10.0, 0.0)


def test_nearest_neighbor_empty():
    tour = nearest_neighbor((0.0, 0.0), [])
    assert tour == []


def test_two_opt_improves_or_keeps():
    start = (0.0, 0.0)
    # Tour with obvious crossing
    tour = [(5.0, 0.0), (0.0, 5.0), (5.0, 5.0), (10.0, 0.0)]
    original_len = tour_length(start, tour)
    optimized = two_opt(start, tour)
    optimized_len = tour_length(start, optimized)
    assert optimized_len <= original_len + 1e-6


def test_optimize_returns_all_waypoints():
    start = (0.0, 0.0)
    waypoints = [(3.0, 4.0), (-2.0, 1.0), (5.0, -3.0), (1.0, 7.0)]
    optimized, info = optimize_waypoint_order(start, waypoints)
    assert len(optimized) == len(waypoints)
    assert set(optimized) == set(waypoints)


def test_optimize_improves_bad_order():
    """Worst case: zigzag should be improved."""
    start = (0.0, 0.0)
    # Deliberately bad order: alternating far and near
    waypoints = [(10.0, 0.0), (1.0, 0.0), (10.0, 1.0), (1.0, 1.0)]
    optimized, info = optimize_waypoint_order(start, waypoints)
    assert info["optimized_length"] <= info["original_length"]


def test_optimize_returns_info():
    start = (0.0, 0.0)
    waypoints = [(3.0, 4.0), (-2.0, 1.0), (5.0, -3.0)]
    _, info = optimize_waypoint_order(start, waypoints)
    assert "original_length" in info
    assert "greedy_length" in info
    assert "optimized_length" in info
    assert "improvement_pct" in info
    assert info["optimized_length"] > 0
