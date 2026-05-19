"""Catmull-Rom path smoothing.

Given a sparse waypoint polyline (typically the output of A*/Theta* after
LOS simplification), produces a dense, C¹-continuous curve that Pure Pursuit
can track without the cogging artifacts induced by sharp polyline corners.

Catmull-Rom (centripetal parameterisation, α=0.5) is chosen over cubic
B-splines because it passes *through* the control points — important when
the input is a planner output that already routes around obstacles. A
B-spline would round off the obstacle-skirting corners and pull the path
back into walls.

References:
  Catmull E., Rom R., "A class of local interpolating splines," 1974.
  Yuksel C. et al., "On the parameterisation of Catmull-Rom curves," 2011.
"""

import math
from typing import List, Tuple

Point = Tuple[float, float]


def _segment_t(p0: Point, p1: Point, alpha: float) -> float:
    """Centripetal parameter increment."""
    return math.pow(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) + 1e-9, alpha)


def _catmull_rom_segment(p0: Point, p1: Point, p2: Point, p3: Point,
                         samples: int, alpha: float) -> List[Point]:
    t0 = 0.0
    t1 = t0 + _segment_t(p0, p1, alpha)
    t2 = t1 + _segment_t(p1, p2, alpha)
    t3 = t2 + _segment_t(p2, p3, alpha)
    if t2 - t1 < 1e-9:
        return [p1]
    out: List[Point] = []
    for i in range(samples):
        t = t1 + (t2 - t1) * (i / samples)
        # De Boor recurrence
        a1 = (
            (t1 - t) / (t1 - t0 + 1e-12) * p0[0] + (t - t0) / (t1 - t0 + 1e-12) * p1[0],
            (t1 - t) / (t1 - t0 + 1e-12) * p0[1] + (t - t0) / (t1 - t0 + 1e-12) * p1[1],
        )
        a2 = (
            (t2 - t) / (t2 - t1 + 1e-12) * p1[0] + (t - t1) / (t2 - t1 + 1e-12) * p2[0],
            (t2 - t) / (t2 - t1 + 1e-12) * p1[1] + (t - t1) / (t2 - t1 + 1e-12) * p2[1],
        )
        a3 = (
            (t3 - t) / (t3 - t2 + 1e-12) * p2[0] + (t - t2) / (t3 - t2 + 1e-12) * p3[0],
            (t3 - t) / (t3 - t2 + 1e-12) * p2[1] + (t - t2) / (t3 - t2 + 1e-12) * p3[1],
        )
        b1 = (
            (t2 - t) / (t2 - t0 + 1e-12) * a1[0] + (t - t0) / (t2 - t0 + 1e-12) * a2[0],
            (t2 - t) / (t2 - t0 + 1e-12) * a1[1] + (t - t0) / (t2 - t0 + 1e-12) * a2[1],
        )
        b2 = (
            (t3 - t) / (t3 - t1 + 1e-12) * a2[0] + (t - t1) / (t3 - t1 + 1e-12) * a3[0],
            (t3 - t) / (t3 - t1 + 1e-12) * a2[1] + (t - t1) / (t3 - t1 + 1e-12) * a3[1],
        )
        c = (
            (t2 - t) / (t2 - t1 + 1e-12) * b1[0] + (t - t1) / (t2 - t1 + 1e-12) * b2[0],
            (t2 - t) / (t2 - t1 + 1e-12) * b1[1] + (t - t1) / (t2 - t1 + 1e-12) * b2[1],
        )
        out.append(c)
    return out


def smooth_path(path: List[Point],
                samples_per_segment: int = 6,
                alpha: float = 0.5) -> List[Point]:
    """Smooth a polyline into a dense Catmull-Rom curve.

    Returns at most ``samples_per_segment × (len(path)-1) + 1`` points.
    Input with fewer than 3 points is returned unchanged.
    """
    n = len(path)
    if n < 3:
        return list(path)
    # Pad endpoints by mirroring so endpoints stay anchored
    p0_pad = (2 * path[0][0] - path[1][0], 2 * path[0][1] - path[1][1])
    pn_pad = (2 * path[-1][0] - path[-2][0], 2 * path[-1][1] - path[-2][1])
    extended = [p0_pad] + list(path) + [pn_pad]
    out: List[Point] = []
    for i in range(len(extended) - 3):
        seg = _catmull_rom_segment(
            extended[i], extended[i + 1], extended[i + 2], extended[i + 3],
            samples_per_segment, alpha,
        )
        out.extend(seg)
    out.append(path[-1])
    return out


def path_arclength(path: List[Point]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        total += math.hypot(path[i + 1][0] - path[i][0],
                            path[i + 1][1] - path[i][1])
    return total
