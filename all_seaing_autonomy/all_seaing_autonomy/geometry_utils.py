"""Shared geometry utilities using NumPy.

Replaces duplicated tuple-based math helpers across task files.
Most trivial operations (norm, dot, midpoint, etc.) are now native NumPy
and don't need wrappers — use them directly:

    np.linalg.norm(a - b)   instead of  self.norm(a, b)
    a @ b                   instead of  self.dot(a, b)
    (a + b) / 2             instead of  self.midpoint(a, b)
    b - a                   instead of  self.difference(a, b)
    np.cross(a, b)          instead of  self.cross(a, b)
    np.array([-v[1], v[0]]) instead of  self.perp_vec(v)
"""

import math

import numpy as np


def ccw(a, b, c):
    """Return True if the points a, b, c are in counterclockwise order."""
    return np.cross(b - a, c - a) > 0


def angle_between(v1, v2):
    """Signed angle from v1 to v2 via atan2(cross, dot)."""
    return math.atan2(np.cross(v1, v2), v1 @ v2)


def angle_segments(seg1, seg2):
    """Angle between two line segments.

    Each segment is a pair of points (p_start, p_end).
    Returns the signed angle between the direction vectors.
    """
    d1 = seg1[1] - seg1[0]
    d2 = seg2[1] - seg2[0]
    return angle_between(d1, d2)


def fit_line(p1, p2):
    """Fit a line ax + by + c = 0 through two points.

    Returns:
        (coeffs, dist): where coeffs = np.array([a, b, c]) and
        dist = distance from origin to the line.
    """
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1
    dist = abs(c) / math.sqrt(a**2 + b**2)
    return np.array([a, b, c]), dist
