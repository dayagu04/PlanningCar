"""Unit tests for Catmull-Rom path smoothing."""

import math
import pytest

from src.planning.path_smoother import smooth_path, path_arclength


def test_short_path_returned_unchanged():
    assert smooth_path([(0.0, 0.0)]) == [(0.0, 0.0)]
    assert smooth_path([(0.0, 0.0), (1.0, 1.0)]) == [(0.0, 0.0), (1.0, 1.0)]


def test_smoothed_curve_passes_through_waypoints():
    raw = [(0.0, 0.0), (2.0, 1.0), (4.0, 0.0), (6.0, 2.0)]
    smooth = smooth_path(raw, samples_per_segment=10)
    # endpoints must match exactly
    assert smooth[0] == pytest.approx(raw[0])
    assert smooth[-1] == pytest.approx(raw[-1])
    # every original interior waypoint must lie within ε of *some* sample
    for wp in raw[1:-1]:
        d = min(math.hypot(p[0] - wp[0], p[1] - wp[1]) for p in smooth)
        assert d < 0.05, f"waypoint {wp} drifted by {d:.3f} m from smoothed curve"


def test_smoothing_increases_density():
    raw = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)]
    smooth = smooth_path(raw, samples_per_segment=8)
    assert len(smooth) > len(raw)


def test_arclength_is_not_shorter_than_chord():
    raw = [(0.0, 0.0), (3.0, 4.0)]
    chord_len = 5.0
    arc = path_arclength(raw)
    assert arc == pytest.approx(chord_len)
    smooth = smooth_path([(0.0, 0.0), (3.0, 4.0), (6.0, 0.0)], samples_per_segment=10)
    # Smoothed curve through a corner is at least as long as the polyline chord
    assert path_arclength(smooth) >= chord_len - 1e-6


def test_smoothing_reduces_angular_corners():
    """A polyline with a sharp 90° corner has one large angular jump.

    After smoothing, the maximum interior-angle change between successive
    segments should drop substantially.
    """
    raw = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]

    def max_corner_turn(path):
        worst = 0.0
        for i in range(1, len(path) - 1):
            a, b, c = path[i - 1], path[i], path[i + 1]
            v1 = (b[0] - a[0], b[1] - a[1])
            v2 = (c[0] - b[0], c[1] - b[1])
            n1 = math.hypot(*v1) + 1e-12
            n2 = math.hypot(*v2) + 1e-12
            dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
            dot = max(-1.0, min(1.0, dot))
            worst = max(worst, math.acos(dot))
        return worst

    raw_worst = max_corner_turn(raw)
    smooth_worst = max_corner_turn(smooth_path(raw, samples_per_segment=8))
    assert smooth_worst < raw_worst * 0.5
