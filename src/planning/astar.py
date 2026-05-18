"""A* path planning on a 2D grid with terrain cost awareness."""

import heapq
import numpy as np
from typing import List, Tuple, Optional


class AStarPlanner:
    def __init__(self, grid_size: float = 0.5, world_size: float = 20.0):
        self.grid_size = grid_size
        self.world_size = world_size
        self.grid_dim = int(world_size / grid_size)
        self.cost_map = np.ones((self.grid_dim, self.grid_dim), dtype=np.float64)

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

    def update_cost_map(self, terrain_grid: np.ndarray, terrain_type: str):
        """Update cost map based on terrain classification."""
        cost_multipliers = {
            "flat": 1.0,
            "slope": 2.0,
            "rough": 3.0,
            "transition": 2.5,
        }
        multiplier = cost_multipliers.get(terrain_type, 1.5)
        self.cost_map[:] = multiplier

    def set_obstacle(self, x: float, y: float, radius: float = 0.5):
        gx, gy = self.world_to_grid(x, y)
        r_cells = int(radius / self.grid_size) + 1
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim:
                    if dx * dx + dy * dy <= r_cells * r_cells:
                        self.cost_map[nx, ny] = 999.0

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim:
                neighbors.append((nx, ny))
        return neighbors

    def plan(self, start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*goal_world)

        if self.cost_map[goal[0], goal[1]] >= 999.0:
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(*current))
                    current = came_from[current]
                path.append(self.grid_to_world(*start))
                path.reverse()
                return self._simplify_path(path)

            for neighbor in self.get_neighbors(current):
                dx = abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1])
                move_cost = 1.0 if dx == 1 else 1.414
                tentative_g = g_score[current] + move_cost * self.cost_map[neighbor[0], neighbor[1]]

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def _simplify_path(self, path: List[Tuple[float, float]],
                       tolerance: float = 0.3) -> List[Tuple[float, float]]:
        """Douglas-Peucker simplification to reduce waypoint count."""
        if len(path) <= 2:
            return path

        max_dist = 0.0
        max_idx = 0
        start = np.array(path[0])
        end = np.array(path[-1])
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)

        if line_len < 1e-6:
            return [path[0], path[-1]]

        for i in range(1, len(path) - 1):
            point = np.array(path[i])
            dist = abs(np.cross(line_vec, start - point)) / line_len
            if dist > max_dist:
                max_dist = dist
                max_idx = i

        if max_dist > tolerance:
            left = self._simplify_path(path[:max_idx + 1], tolerance)
            right = self._simplify_path(path[max_idx:], tolerance)
            return left[:-1] + right
        else:
            return [path[0], path[-1]]
