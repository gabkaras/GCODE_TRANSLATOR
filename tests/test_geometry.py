import math

import numpy as np
import pytest

from gcode2krl.geometry import (arc_from_center, fit_arc_3d, fit_arc_fixed_ends, fit_circle_taubin,
                                fit_plane, sample_arc)


def circle_pts(cx, cy, r, a0, a1, n, z=0.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    a = np.linspace(a0, a1, n)
    pts = np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a), np.full(n, z)])
    if noise:
        pts += rng.uniform(-noise, noise, pts.shape)
    return pts


def test_taubin_exact_circle():
    pts = circle_pts(10, -5, 20, 0, 1.2, 15)
    cx, cy, r = fit_circle_taubin(pts[:, 0], pts[:, 1])
    assert abs(cx - 10) < 1e-6 and abs(cy + 5) < 1e-6 and abs(r - 20) < 1e-6


def test_taubin_large_coordinates_short_arc():
    pts = circle_pts(1234.5, -987.6, 50, 0.3, 0.35, 8)
    cx, cy, r = fit_circle_taubin(pts[:, 0], pts[:, 1])
    assert abs(r - 50) < 1e-3


def test_taubin_collinear_returns_none():
    x = np.linspace(0, 10, 6)
    y = 2 * x + 1
    assert fit_circle_taubin(x, y) is None


def test_fixed_ends_fit_passes_through_endpoints():
    pts = circle_pts(0, 0, 30, 0.2, 1.5, 20, noise=0.01)
    cx, cy, r = fit_arc_fixed_ends(pts[:, 0], pts[:, 1])
    d0 = math.hypot(pts[0, 0] - cx, pts[0, 1] - cy)
    dn = math.hypot(pts[-1, 0] - cx, pts[-1, 1] - cy)
    assert abs(d0 - r) < 1e-9 and abs(dn - r) < 1e-9
    assert abs(r - 30) < 0.05


def test_fit_plane_tilted():
    rng = np.random.default_rng(1)
    base = rng.uniform(-10, 10, (30, 2))
    pts = np.column_stack([base, 0.5 * base[:, 0] - 0.25 * base[:, 1] + 3])
    pl = fit_plane(pts)
    _, _, w = pl.project(pts)
    assert np.abs(w).max() < 1e-9


def test_fit_arc_3d_tilted_plane():
    arc = arc_from_center(np.array([10.0, 0.0, 0.0]), np.array([0.0, 10.0, 0.0]), np.zeros(3), np.array([0.0, 0.0, 1.0]), True)
    pts = sample_arc(arc, 0.01)
    # rotate about X by 35 degrees
    a = math.radians(35)
    rot = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    pts = pts @ rot.T + np.array([5.0, 6.0, 7.0])
    fit = fit_arc_3d(pts, tol=0.02)
    assert fit is not None
    assert abs(fit.radius - 10) < 1e-6
    assert abs(math.degrees(fit.sweep) - 90) < 1e-6
    assert np.allclose(fit.center, np.array([5.0, 6.0, 7.0]), atol=1e-6)
    assert np.allclose(fit.mid, pts[len(pts) // 2], atol=1e-6) or abs(np.linalg.norm(fit.mid - fit.center) - 10) < 1e-6


def test_fit_arc_3d_rejects_square_and_line():
    square = np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0], [1, 0, 0]], dtype=float)
    assert fit_arc_3d(square, tol=0.05) is None
    line = np.column_stack([np.linspace(0, 10, 8), np.zeros(8), np.zeros(8)])
    assert fit_arc_3d(line, tol=0.05) is None


def test_fit_arc_3d_rejects_reversal():
    pts = circle_pts(0, 0, 10, 0, 1.0, 10)
    pts = np.vstack([pts, pts[-2::-1]])  # go back along the same arc
    assert fit_arc_3d(pts, tol=0.05) is None


def test_fit_arc_3d_full_circle_closed():
    pts = circle_pts(3, 4, 12, 0, 2 * math.pi, 73)
    fit = fit_arc_3d(pts, tol=0.1)
    assert fit is not None and fit.closed
    assert abs(fit.sweep - 2 * math.pi) < 1e-9
    assert abs(fit.radius - 12) < 1e-6


def test_fit_arc_3d_respects_sagitta_tolerance():
    # 4 points on R=100 spaced 30 degrees: sagitta = 100*(1-cos(15deg)) = 3.4 mm
    pts = circle_pts(0, 0, 100, 0, math.pi / 2, 4)
    assert fit_arc_3d(pts, tol=1.0) is None
    assert fit_arc_3d(pts, tol=4.0) is not None


def test_arc_from_center_cw_and_ccw():
    s = np.array([10.0, 0.0, 0.0])
    e = np.array([0.0, 10.0, 0.0])
    ccw = arc_from_center(s, e, np.zeros(3), np.array([0, 0, 1.0]), True)
    cw = arc_from_center(s, e, np.zeros(3), np.array([0, 0, 1.0]), False)
    assert abs(math.degrees(ccw.sweep) - 90) < 1e-9
    assert abs(math.degrees(cw.sweep) - 270) < 1e-9
    assert np.allclose(ccw.mid, [10 / math.sqrt(2), 10 / math.sqrt(2), 0])
    assert np.allclose(cw.mid, [-10 / math.sqrt(2), -10 / math.sqrt(2), 0])
    assert np.allclose(ccw.t_in, [0, 1, 0]) and np.allclose(cw.t_in, [0, -1, 0])


def test_sample_arc_chord_error():
    arc = arc_from_center(np.array([20.0, 0, 0]), np.array([-20.0, 0, 0]), np.zeros(3), np.array([0, 0, 1.0]), True)
    pts = sample_arc(arc, 0.05)
    mids = (pts[1:] + pts[:-1]) / 2
    sag = 20 - np.linalg.norm(mids, axis=1)
    assert sag.max() <= 0.05 + 1e-9
    assert np.allclose(pts[0], [20, 0, 0]) and np.allclose(pts[-1], [-20, 0, 0])
