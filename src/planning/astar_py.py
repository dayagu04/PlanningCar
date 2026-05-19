"""A* path planning on a 2D grid with terrain elevation cost awareness."""

import heapq
import math
import numpy as np
from typing import List, Tuple, Optional


class AStarPlanner:
    def __init__(self, grid_size: float = 0.5, world_size: float = 20.0,
                 heuristic_weight: float = 1.3):
        self.grid_size = grid_size
        self.world_size = world_size
        self.grid_dim = int(world_size / grid_size)
        self.cost_map = np.ones((self.grid_dim, self.grid_dim), dtype=np.float64)
        # Persistent obstacle layer — reapplied after each update_cost_map() so
        # that terrain-aware cost rebuilds don't wipe out registered obstacles.
        self._obstacles: list = []  # [(x, y, radius), ...]
        # weighted A*: heuristic_weight > 1 trades a bit of optimality for much faster search
        self.heuristic_weight = heuristic_weight
        self._neighbor_offsets = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.41421356), (-1, 1, 1.41421356),
            (1, -1, 1.41421356), (1, 1, 1.41421356),
        )

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
        """Build per-cell cost using terrain category + (optional) local elevation gradient.

        elevation_sampler: callable (x, y) -> float, in world coordinates. When
        provided, cells lying on steep elevation gradients receive a higher cost,
        so A* steers around hills instead of climbing over them.
        """
        cost_multipliers = {"flat": 1.0, "slope": 2.0, "rough": 3.0, "transition": 2.5}
        base = cost_multipliers.get(terrain_type, 1.5)
        self.cost_map[:] = base

        if elevation_sampler is not None:
            # Sample elevation at every cell center
            gs = self.grid_size
            ws = self.world_size
            coords = np.arange(self.grid_dim) * gs - ws / 2 + gs / 2
            elev = np.empty((self.grid_dim, self.grid_dim), dtype=np.float64)
            for i, x in enumerate(coords):
                for j, y in enumerate(coords):
                    elev[i, j] = elevation_sampler(float(x), float(y))

            # Local slope = max neighbor height difference / cell size
            gx = np.zeros_like(elev)
            gy = np.zeros_like(elev)
            gx[1:-1, :] = (elev[2:, :] - elev[:-2, :]) / (2 * gs)
            gy[:, 1:-1] = (elev[:, 2:] - elev[:, :-2]) / (2 * gs)
            slope = np.sqrt(gx * gx + gy * gy)

            self.cost_map = base + 10.0 * slope

        # Reapply persistent obstacles so they survive cost-map rebuilds
        for ox, oy, oradius in self._obstacles:
            self._paint_obstacle(ox, oy, oradius)

    def add_obstacles(self, obstacles):
        """Register persistent obstacles (list of (x, y, radius) tuples).

        Calls beyond initial registration are additive. Cells inside the radius
        are marked unwalkable on the current cost map and remembered for future
        update_cost_map() calls.
        """
        for ox, oy, oradius in obstacles:
            self._obstacles.append((ox, oy, oradius))
            self._paint_obstacle(ox, oy, oradius)

    def clear_obstacles(self):
        self._obstacles = []

    def _paint_obstacle(self, x: float, y: float, radius: float):
        """Mark a circular region as untraversable (high cost) on cost_map."""
        gx, gy = self.world_to_grid(x, y)
        r_cells = int(math.ceil(radius / self.grid_size)) + 1
        rr = (radius / self.grid_size) ** 2
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy <= rr:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim:
                        self.cost_map[nx, ny] = 999.0

    def set_obstacle(self, x: float, y: float, radius: float = 0.5):
        """Legacy one-shot obstacle marker (does not persist across rebuilds)."""
        self._paint_obstacle(x, y, radius)

    def heuristic(self, ax: int, ay: int, bx: int, by: int) -> float:
        # Octile distance — exact for 8-connected grid with unit/√2 step costs
        dx = abs(ax - bx)
        dy = abs(ay - by)
        return (dx + dy) + (1.41421356 - 2.0) * min(dx, dy)

    def _nearest_free_cell(self, cell: Tuple[int, int], max_cells: int = 6) -> Optional[Tuple[int, int]]:
        """BFS outward from `cell` to find the nearest cell with cost_map < 999."""
        cx, cy = cell
        if self.cost_map[cx, cy] < 999.0:
            return (cx, cy)
        for ring in range(1, max_cells + 1):
            # Walk the square ring at distance `ring`
            for dx in range(-ring, ring + 1):
                for dy in (-ring, ring):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim:
                        if self.cost_map[nx, ny] < 999.0:
                            return (nx, ny)
            for dy in range(-ring + 1, ring):
                for dx in (-ring, ring):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim:
                        if self.cost_map[nx, ny] < 999.0:
                            return (nx, ny)
        return None

    def plan(self, start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*goal_world)
        gdim = self.grid_dim

        # If the goal cell itself is blocked (waypoint sits inside an obstacle's
        # safety radius), find the nearest free cell within 3 m and use it.
        if self.cost_map[goal[0], goal[1]] >= 999.0:
            replacement = self._nearest_free_cell(goal, max_cells=int(3.0 / self.grid_size))
            if replacement is None:
                return None
            goal = replacement

        if start == goal:
            return [start_world, goal_world]

        # Use flat arrays + integer node ids for speed (avoid dict hashing of tuples)
        INF = float("inf")
        size = gdim * gdim
        g_score = np.full(size, INF, dtype=np.float64)
        came_from = np.full(size, -1, dtype=np.int32)
        closed = np.zeros(size, dtype=np.bool_)

        start_id = start[0] * gdim + start[1]
        goal_id = goal[0] * gdim + goal[1]
        g_score[start_id] = 0.0

        open_set = [(self.heuristic(*start, *goal), 0, start_id)]
        counter = 1
        w = self.heuristic_weight

        while open_set:
            _, _, current = heapq.heappop(open_set)
            if closed[current]:
                continue
            if current == goal_id:
                # reconstruct
                path_ids = []
                node = current
                while node != -1:
                    path_ids.append(node)
                    node = int(came_from[node])
                path_ids.reverse()
                world_path = [self.grid_to_world(n // gdim, n % gdim) for n in path_ids]
                return self._simplify_path(world_path)

            closed[current] = True
            cx = current // gdim
            cy = current % gdim
            cur_g = g_score[current]
            for dx, dy, step in self._neighbor_offsets:
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or nx >= gdim or ny < 0 or ny >= gdim:
                    continue
                nid = nx * gdim + ny
                if closed[nid]:
                    continue
                tentative = cur_g + step * self.cost_map[nx, ny]
                if tentative < g_score[nid]:
                    g_score[nid] = tentative
                    came_from[nid] = current
                    f = tentative + w * self.heuristic(nx, ny, goal[0], goal[1])
                    heapq.heappush(open_set, (f, counter, nid))
                    counter += 1

        return None

    def _simplify_path(self, path: List[Tuple[float, float]],
                       tolerance: float = 0.3) -> List[Tuple[float, float]]:
        """Iterative Douglas-Peucker simplification (no recursion stack)."""
        n = len(path)
        if n <= 2:
            return path
        keep = [False] * n
        keep[0] = True
        keep[-1] = True
        stack = [(0, n - 1)]
        pts = np.asarray(path)
        while stack:
            i0, i1 = stack.pop()
            if i1 <= i0 + 1:
                continue
            seg_start = pts[i0]
            seg_end = pts[i1]
            seg = seg_end - seg_start
            seg_len = float(np.hypot(seg[0], seg[1]))
            if seg_len < 1e-9:
                continue
            # Perpendicular distance from each interior point to the segment
            rel = pts[i0 + 1:i1] - seg_start
            cross = rel[:, 0] * seg[1] - rel[:, 1] * seg[0]
            dist = np.abs(cross) / seg_len
            j = int(np.argmax(dist))
            if dist[j] > tolerance:
                idx = i0 + 1 + j
                keep[idx] = True
                stack.append((i0, idx))
                stack.append((idx, i1))
        return [path[i] for i, k in enumerate(keep) if k]
