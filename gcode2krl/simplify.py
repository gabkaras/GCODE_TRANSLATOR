"""Polyline simplification (Douglas-Peucker) used to thin SPL point runs."""
from __future__ import annotations

import numpy as np

from .geometry import point_segment_distances


def douglas_peucker_indices(points: np.ndarray, tol: float) -> list[int]:
    """Return indices of the vertices kept by Douglas-Peucker within ``tol``."""
    n = len(points)
    if n <= 2:
        return list(range(n))
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        d = point_segment_distances(points[i + 1 : j], points[i], points[j])
        k = int(np.argmax(d))
        if d[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return [int(i) for i in np.flatnonzero(keep)]


def douglas_peucker(points: np.ndarray, tol: float) -> np.ndarray:
    return points[douglas_peucker_indices(points, tol)]
