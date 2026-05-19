"""Pure Pursuit path tracker with terrain-adaptive lookahead.

Pure Pursuit treats the path as a sequence of waypoints and, at each control
step, picks the point on the path that is approximately ``lookahead`` metres
in front of the robot. It then steers as if driving an arc tangent to the
robot's heading that passes through that point.

Why this beats the original "point-to-point with PD heading":
  - lookahead acts as an inherent low-pass filter on the path, smoothing
    polyline corners without needing an explicit spline pass;
  - cruising speed is decoupled from heading error, so the robot keeps
    moving through turns instead of stopping to re-aim;
  - lookahead scales with measured speed and terrain class — short on
    rough/curvy terrain, long on flat/straight stretches — preventing both
    cut-corner instability and lazy oversteer.

Algorithm references:
  Coulter R. C., "Implementation of the Pure Pursuit Path Tracking
  Algorithm," Carnegie Mellon Univ. Robotics Inst., 1992.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


Point = Tuple[float, float]


@dataclass
class LookaheadResult:
    """One pure-pursuit query."""
    target: Point          # the lookahead point in world coords
    curvature: float       # 1 / radius of the arc tangent at the robot
    cross_track: float     # signed lateral offset from path (m, +left)
    segment_index: int     # which path segment the lookahead landed on
    path_progress: float   # arclength consumed from start, in metres


class PurePursuit:
    """State-bearing tracker over a fixed polyline path.

    ``lookahead_base`` is the lookahead at zero speed; ``lookahead_gain``
    multiplies into the current robot speed (m/s) to add a velocity term:
    L = lookahead_base + lookahead_gain * speed, clipped to
    [lookahead_min, lookahead_max]. Defaults are tuned for the 0.4×0.3 m
    chassis at ≤2 m/s.
    """

    def __init__(self,
                 lookahead_base: float = 0.6,
                 lookahead_gain: float = 0.35,
                 lookahead_min: float = 0.4,
                 lookahead_max: float = 2.5,
                 goal_tolerance: float = 0.5):
        self.lookahead_base = lookahead_base
        self.lookahead_gain = lookahead_gain
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.goal_tolerance = goal_tolerance
        self._path: List[Point] = []
        self._last_segment: int = 0  # progress monotonicity
        self.last_cross_track: float = 0.0  # exposed for controller replan logic

    # ------------------------------------------------------------------
    # Path management
    # ------------------------------------------------------------------
    def set_path(self, path: List[Point]):
        self._path = list(path) if path else []
        self._last_segment = 0

    @property
    def path(self) -> List[Point]:
        return self._path

    def has_path(self) -> bool:
        return len(self._path) >= 2

    def reached_goal(self, pos: Point) -> bool:
        if not self._path:
            return True
        gx, gy = self._path[-1]
        return math.hypot(pos[0] - gx, pos[1] - gy) <= self.goal_tolerance

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _project_to_segment(p: Point, a: Point, b: Point) -> Tuple[Point, float, float]:
        """Return (closest_point, t∈[0,1], signed lateral offset)."""
        ax, ay = a
        bx, by = b
        px, py = p
        dx, dy = bx - ax, by - ay
        seg_sq = dx * dx + dy * dy
        if seg_sq < 1e-12:
            return a, 0.0, math.hypot(px - ax, py - ay)
        t = ((px - ax) * dx + (py - ay) * dy) / seg_sq
        t_clamped = max(0.0, min(1.0, t))
        cx, cy = ax + t_clamped * dx, ay + t_clamped * dy
        # signed cross-track: + if robot is left of segment (CCW)
        cross = (dx * (py - ay) - dy * (px - ax)) / math.sqrt(seg_sq)
        return (cx, cy), t_clamped, cross

    def _closest_segment(self, pos: Point) -> Tuple[int, float, float]:
        """Find the path segment closest to ``pos`` (searching forward only).

        Returns (segment_index, t_on_segment, cross_track).
        Searching forward from ``self._last_segment`` enforces monotonic
        progress and prevents the controller from snapping back to earlier
        path sections when the path loops or self-crosses.
        """
        best_idx = self._last_segment
        best_d = float("inf")
        best_t = 0.0
        best_cross = 0.0
        for i in range(self._last_segment, len(self._path) - 1):
            (_cx, _cy), t, cross = self._project_to_segment(
                pos, self._path[i], self._path[i + 1])
            d = abs(cross) if 0.0 <= t <= 1.0 else (
                math.hypot(pos[0] - self._path[i][0], pos[1] - self._path[i][1])
                if t < 0 else
                math.hypot(pos[0] - self._path[i + 1][0], pos[1] - self._path[i + 1][1])
            )
            if d < best_d:
                best_d = d
                best_idx = i
                best_t = t
                best_cross = cross
        # Allow the closest-point search a small backward window so brief
        # excursions (e.g. obstacle deflection) don't permanently strand
        # the tracker behind its own progress.
        for i in range(max(0, self._last_segment - 1), self._last_segment):
            (_cx, _cy), t, cross = self._project_to_segment(
                pos, self._path[i], self._path[i + 1])
            d = abs(cross) if 0.0 <= t <= 1.0 else float("inf")
            if d < best_d:
                best_d = d
                best_idx = i
                best_t = t
                best_cross = cross
        return best_idx, best_t, best_cross

    def _walk_lookahead(self, start_idx: int, start_t: float, L: float) -> Tuple[Point, int]:
        """Walk arclength L along the path from (start_idx, start_t)."""
        remaining = L
        # current point on segment
        ax, ay = self._path[start_idx]
        bx, by = self._path[start_idx + 1]
        cur = (ax + start_t * (bx - ax), ay + start_t * (by - ay))
        idx = start_idx
        while remaining > 0 and idx < len(self._path) - 1:
            sx, sy = cur
            ex, ey = self._path[idx + 1]
            seg_len = math.hypot(ex - sx, ey - sy)
            if seg_len <= remaining:
                cur = (ex, ey)
                remaining -= seg_len
                idx += 1
                if idx >= len(self._path) - 1:
                    break
            else:
                t = remaining / seg_len if seg_len > 1e-9 else 0.0
                cur = (sx + t * (ex - sx), sy + t * (ey - sy))
                remaining = 0
        return cur, idx

    # ------------------------------------------------------------------
    # Main query
    # ------------------------------------------------------------------
    def _path_curvature_ahead(self, idx: int, t: float, samples: int = 6) -> float:
        """Worst-case angular change among the next `samples` path segments.

        Used by ``lookahead_distance`` to shrink the lookahead on twisty
        sections so the tracker doesn't cut corners.
        """
        if len(self._path) < 3:
            return 0.0
        worst = 0.0
        end = min(idx + samples, len(self._path) - 2)
        for i in range(idx, end):
            a = self._path[i]
            b = self._path[i + 1]
            c = self._path[i + 2]
            v1 = (b[0] - a[0], b[1] - a[1])
            v2 = (c[0] - b[0], c[1] - b[1])
            n1 = math.hypot(*v1) + 1e-9
            n2 = math.hypot(*v2) + 1e-9
            dot = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
            angle = math.acos(dot)
            worst = max(worst, angle)
        return worst

    def lookahead_distance(self, speed_mps: float,
                           override_max: Optional[float] = None,
                           dist_to_goal: Optional[float] = None,
                           path_curvature: Optional[float] = None) -> float:
        """L(v) = base + gain·v, clipped, with goal taper and curvature taper."""
        L = self.lookahead_base + self.lookahead_gain * max(0.0, speed_mps)
        upper = override_max if override_max is not None else self.lookahead_max
        L = max(self.lookahead_min, min(upper, L))
        if dist_to_goal is not None:
            L = max(self.lookahead_min, min(L, dist_to_goal))
        if path_curvature is not None and path_curvature > 0.5:
            # >29°: only shrink lookahead on truly sharp bends; keep ≥0.65×
            # so lookahead can still preview the curve.
            curve_factor = max(0.65, 1.0 - 0.25 * (path_curvature - 0.5))
            L = max(self.lookahead_min, L * curve_factor)
        return L

    def query(self, pos: Point, heading: float,
              speed_mps: float = 0.0,
              max_lookahead: Optional[float] = None) -> Optional[LookaheadResult]:
        """Compute a steering command for the current robot pose.

        Returns ``None`` once the goal is within ``goal_tolerance``.
        """
        if not self.has_path() or self.reached_goal(pos):
            return None

        idx, t, cross = self._closest_segment(pos)
        self._last_segment = idx

        goal = self._path[-1]
        d_goal = math.hypot(pos[0] - goal[0], pos[1] - goal[1])
        path_curve = self._path_curvature_ahead(idx, t)
        L = self.lookahead_distance(speed_mps,
                                    override_max=max_lookahead,
                                    dist_to_goal=d_goal,
                                    path_curvature=path_curve)
        target, end_idx = self._walk_lookahead(idx, t, L)

        # Endgame guard: when the robot is on the last segment AND already
        # within ~lookahead+0.5 m of the goal, aim straight at the goal.
        # This prevents the classic Pure-Pursuit circle-back without
        # being so aggressive that we ignore the path on long final segments.
        last_seg = len(self._path) - 2
        if idx == last_seg and d_goal < L + 0.5:
            target = goal
            end_idx = last_seg

        # Curvature κ = 2·y_local / L²  in the robot frame
        # (y_local positive ⇒ target is to the left of the heading)
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        # rotate world delta into robot frame: x forward, y left
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        y_local = -sin_h * dx + cos_h * dy
        L_eff_sq = dx * dx + dy * dy
        curvature = 2.0 * y_local / L_eff_sq if L_eff_sq > 1e-9 else 0.0

        # Path progress in metres up to ``idx``
        progress = 0.0
        for i in range(idx):
            ax, ay = self._path[i]
            bx, by = self._path[i + 1]
            progress += math.hypot(bx - ax, by - ay)
        ax, ay = self._path[idx]
        bx, by = self._path[idx + 1]
        progress += t * math.hypot(bx - ax, by - ay)

        self.last_cross_track = abs(cross)
        return LookaheadResult(
            target=target,
            curvature=curvature,
            cross_track=cross,
            segment_index=end_idx,
            path_progress=progress,
        )

    # ------------------------------------------------------------------
    # Differential-drive command (convenience)
    # ------------------------------------------------------------------
    def steering_to_wheels(self, res: LookaheadResult, params,
                           wheelbase: float = 0.36,
                           dist_to_goal: Optional[float] = None,
                           heading_to_target: Optional[float] = None) -> Tuple[float, float]:
        """Convert curvature + cruising speed to (left, right) wheel rad/s.

        The robot is modelled as a tank-style differential drive (no separate
        kinematic chain), so we treat ``wheelbase`` as the effective track
        width between the wheels and apply v_l = v − κ·v·b/2, v_r = v + κ·v·b/2.

        Two speed-shaping terms are applied on top of ``params.max_speed``:

        * **Curvature taper** — soft brake on tight curvature; bottoms at 65%.
          Kicks in at |κ| > ~0.5 (radius < 2 m).
        * **End-of-path braking** — when ``dist_to_goal`` is supplied and
          smaller than ~2 m, scale velocity linearly down to 50% at the goal
          itself. Stops the robot from sailing past the final waypoint at
          peak speed and overshooting.
        """
        v = params.max_speed
        kappa = res.curvature

        # Spin-in-place when the lookahead is well behind the robot.
        # Pure Pursuit's curvature command is only valid when the target lies
        # roughly forward; for anything past ±120°, prefer a hard turn (e.g.
        # right after switching to a waypoint that lies in the opposite
        # direction). Without this, PP's curvature command tries to drive
        # forward in a big arc and the robot escapes the map.
        if heading_to_target is not None and abs(heading_to_target) > 2.0944:  # 120°
            spin = v * 0.7
            return (-spin, spin) if heading_to_target > 0 else (spin, -spin)

        # Soft brake on tight curvature (gentle on near-straight, biting on hairpins).
        brake = 1.0 / (1.0 + 0.25 * abs(kappa))
        v *= max(0.65, brake)
        # End-of-path linear taper: 0.5·v at d=0, 1.0·v at d≥2 m.
        # Combined with a hard brake when the controller declares the goal
        # reached, this is enough to stop overshooting without giving up
        # cruising speed on long open segments.
        if dist_to_goal is not None and dist_to_goal < 2.0:
            v *= 0.5 + 0.25 * dist_to_goal
        v_diff = kappa * v * wheelbase * 0.5
        left = v - v_diff
        right = v + v_diff
        m = params.max_speed
        left = max(-m, min(m, left))
        right = max(-m, min(m, right))
        return left, right
