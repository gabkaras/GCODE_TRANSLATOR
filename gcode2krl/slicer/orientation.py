"""Tool-axis orientation along the path: lead / side tilt relative to the travel direction.

The nozzle axis (pointing away from the part) is tilted by ``lead`` degrees towards
the direction of travel (negative = trailing) and by ``side`` degrees to the left of
it. The rotation about the axis (spin) is left to the translator's solver.
"""
from __future__ import annotations

import math

import numpy as np


def tangents(points: np.ndarray, closed: bool) -> np.ndarray:
    P = np.asarray(points, float)
    n = len(P)
    t = np.zeros((n, 3))
    if n < 2:
        t[:] = [1.0, 0, 0]
        return t
    if closed:
        t = np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0)
    else:
        t[1:-1] = P[2:] - P[:-2]
        t[0] = P[1] - P[0]
        t[-1] = P[-1] - P[-2]
    ln = np.linalg.norm(t, axis=1)
    bad = ln < 1e-9
    t[bad] = [1.0, 0, 0]
    ln[bad] = 1.0
    return t / ln[:, None]


def tilt_axes(points: np.ndarray, axes: np.ndarray | None, closed: bool, lead_deg: float, side_deg: float) -> np.ndarray:
    """Per-point tool axes tilted relative to the direction of travel."""
    P = np.asarray(points, float)
    n = len(P)
    base = np.tile([0.0, 0.0, 1.0], (n, 1)) if axes is None else np.asarray(axes, float).copy()
    if abs(lead_deg) < 1e-9 and abs(side_deg) < 1e-9:
        return base
    t = tangents(P, closed)
    # travel direction in the layer plane (perpendicular to the axis)
    t = t - np.sum(t * base, axis=1)[:, None] * base
    ln = np.linalg.norm(t, axis=1)
    ok = ln > 1e-9
    t[ok] /= ln[ok][:, None]
    t[~ok] = 0.0
    left = np.cross(base, t)
    out = base + math.tan(math.radians(lead_deg)) * t + math.tan(math.radians(side_deg)) * left
    return out / np.linalg.norm(out, axis=1)[:, None]
