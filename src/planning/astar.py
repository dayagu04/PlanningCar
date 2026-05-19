"""A* / Theta* path planning on a 2D grid — C++ backed.

Wraps `nav_core_cpp.GridPlanner`. Keeps the same public API as the
pure-Python reference in `astar_py.py`:

  - constructor signature: `AStarPlanner(grid_size, world_size, heuristic_weight=1.3)`
  - `.plan(start, goal)` returns a list of (x, y) tuples or `None`
  - `.set_obstacle(x, y, radius=0.5)`, `.add_obstacles(list)`, `.clear_obstacles()`
  - `.update_cost_map(terrain_grid, terrain_type, elevation_sampler=None)`
  - `.world_to_grid(x, y)` / `.grid_to_world(gx, gy)`
  - `.cost_map` numpy view (read/write — kept for compatibility with tests
    that touch the cost map directly; note that the underlying C++ planner
    rebuilds its own cost map from `update_cost_map`/`add_obstacles`,
    so direct in-place writes here are visible to Python callers but do
    NOT alter the C++ search.)
"""

import math
import numpy as np
from typing import List, Tuple, Optional

from src import nav_core_cpp as _cpp


_TERRAIN_NAME_TO_CPP = {
    "flat": _cpp.TerrainType.FLAT,
    "slope": _cpp.TerrainType.SLOPE,
    "rough": _cpp.TerrainType.ROUGH,
    "transition": _cpp.TerrainType.TRANSITION,
}


class AStarPlanner:
    def __init__(self, grid_size: float = 0.5, world_size: float = 20.0,
                 heuristic_weight: float = 1.3):
        self.grid_size = grid_size
        self.world_size = world_size
        self.grid_dim = int(world_size / grid_size)
        self.heuristic_weight = heuristic_weight

        cfg = _cpp.PlannerConfig()
        cfg.grid_size = grid_size
        cfg.world_size = world_size
        cfg.heuristic_weight = heuristic_weight
        cfg.use_theta_star = False  # match Python reference: 8-connected A*
        self._planner = _cpp.GridPlanner(cfg)

        # Mirror cost map (numpy) — kept for API compatibility; updated by
        # update_cost_map() and by _paint_obstacle(). The C++ planner has
        # its own internal cost map, kept in sync via add_obstacles() and
        # update_cost_map() calls.
        self.cost_map = np.ones((self.grid_dim, self.grid_dim), dtype=np.float64)
        self._obstacles: list = []

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x + self.world_size / 2) / self.grid_size)
        gy = int((y + self.world_size / 2) / self.grid_size)
        gx = max(0, min(self.grid_dim - 1, gx))
        gy = max(0, min(self.grid_dim - 1, gy))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = gx * self.grid_size - self.world_size / 2 + self.grid_size / 2
        y = gy * self.grid_size - self.world_size / 2 + self.grid_size / 2
        return x, y

    def update_cost_map(self, terrain_grid: np.ndarray, terrain_type: str,
                        elevation_sampler=None):
        # 1. Update C++ side — this is what plan() actually consults
        cpp_terrain = _TERRAIN_NAME_TO_CPP.get(terrain_type, _cpp.TerrainType.FLAT)
        self._planner.update_cost_map(cpp_terrain, elevation_sampler)
        # Persistent obstacles must be re-registered on the C++ side because
        # update_cost_map() rebuilds the cost grid from scratch.
        if self._obstacles:
            self._planner.add_obstacles(self._obstacles)

        # 2. Update Python-side mirror so callers that inspect .cost_map see
        # a representative grid (terrain base cost + elevation gradient).
        cost_multipliers = {"flat": 1.0, "slope": 2.0, "rough": 3.0, "transition": 2.5}
        base = cost_multipliers.get(terrain_type, 1.5)
        self.cost_map[:] = base

        if elevation_sampler is not None:
            gs = self.grid_size
            ws = self.world_size
            coords = np.arange(self.grid_dim) * gs - ws / 2 + gs / 2
            elev = np.empty((self.grid_dim, self.grid_dim), dtype=np.float64)
            for i, x in enumerate(coords):
                for j, y in enumerate(coords):
                    elev[i, j] = float(elevation_sampler(float(x), float(y)))
            gx = np.zeros_like(elev)
            gy = np.zeros_like(elev)
            gx[1:-1, :] = (elev[2:, :] - elev[:-2, :]) / (2 * gs)
            gy[:, 1:-1] = (elev[:, 2:] - elev[:, :-2]) / (2 * gs)
            slope = np.sqrt(gx * gx + gy * gy)
            self.cost_map = base + 10.0 * slope

        for ox, oy, oradius in self._obstacles:
            self._paint_obstacle_mirror(ox, oy, oradius)

    def add_obstacles(self, obstacles):
        for ox, oy, oradius in obstacles:
            self._obstacles.append((ox, oy, oradius))
            self._paint_obstacle_mirror(ox, oy, oradius)
        self._planner.add_obstacles(list(obstacles))

    def clear_obstacles(self):
        self._obstacles = []
        self._planner.clear_obstacles()

    def set_obstacle(self, x: float, y: float, radius: float = 0.5):
        """Legacy one-shot obstacle marker — registers on both sides."""
        self._obstacles.append((x, y, radius))
        self._paint_obstacle_mirror(x, y, radius)
        self._planner.add_obstacles([(x, y, radius)])

    def _paint_obstacle_mirror(self, x: float, y: float, radius: float):
        gx, gy = self.world_to_grid(x, y)
        r_cells = int(math.ceil(radius / self.grid_size)) + 1
        rr = (radius / self.grid_size) ** 2
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy <= rr:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim:
                        self.cost_map[nx, ny] = 999.0

    def plan(self, start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        path = self._planner.plan(tuple(start_world), tuple(goal_world))
        if not path:
            return None
        return [tuple(p) for p in path]
