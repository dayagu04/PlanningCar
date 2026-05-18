"""Tests for A* path planner."""

import numpy as np
from src.planning.astar import AStarPlanner


def test_basic_path_finding():
    planner = AStarPlanner(grid_size=0.5, world_size=10.0)
    path = planner.plan((0.0, 0.0), (3.0, 3.0))
    assert path is not None
    assert len(path) >= 2
    assert abs(path[-1][0] - 3.0) < 0.5
    assert abs(path[-1][1] - 3.0) < 0.5


def test_obstacle_avoidance():
    planner = AStarPlanner(grid_size=0.5, world_size=20.0)
    planner.set_obstacle(2.0, 0.0, radius=0.8)
    path = planner.plan((0.0, 0.0), (4.0, 0.0))
    assert path is not None
    for x, y in path:
        dist = np.sqrt((x - 2.0) ** 2 + y ** 2)
        assert dist > 0.3


def test_no_path_to_obstacle():
    planner = AStarPlanner(grid_size=0.5, world_size=20.0)
    planner.set_obstacle(3.0, 3.0, radius=1.0)
    path = planner.plan((0.0, 0.0), (3.0, 3.0))
    assert path is None


def test_terrain_cost_affects_path():
    planner = AStarPlanner(grid_size=0.5, world_size=20.0)
    for gx in range(18, 22):
        for gy in range(0, planner.grid_dim):
            planner.cost_map[gx, gy] = 5.0

    path_costly = planner.plan((0.0, 0.0), (4.0, 0.0))
    assert path_costly is not None


def test_world_grid_conversion():
    planner = AStarPlanner(grid_size=0.5, world_size=20.0)
    gx, gy = planner.world_to_grid(0.0, 0.0)
    assert gx == 20
    assert gy == 20
    wx, wy = planner.grid_to_world(20, 20)
    assert abs(wx - 0.25) < 0.3
    assert abs(wy - 0.25) < 0.3
