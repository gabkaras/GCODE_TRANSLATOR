"""Geometric primitives: planes, circle fitting and arc validation.

Circle fitting follows Chernov's formulation of the Taubin algebraic fit
(Al-Sharadqah & Chernov, "Error analysis for circle fitting algorithms",
EJS 2009) for the initial estimate, followed by a geometric least-squares
fit with the two end points of the chain fixed on the circle (the idea of
Gribov, "Approximate fitting of a circular arc when two points are known").
Fixing the end points guarantees that the emitted arc starts and ends
exactly on original vertices, which is what CIRC/SCIRC need.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

EPS = 1e-12
TWO_PI = 2.0 * math.pi


def unit(v: np.ndarray) -> np.ndarray:
    n = math.sqrt(float(v @ v))
    return v / n if n > EPS else np.zeros_like(v)


def cross3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Cross product of two 3-vectors without numpy's generic overhead."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def point_segment_distances(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Euclidean distances from each row of ``points`` to segment a-b."""
    ab = b - a
    l2 = float(ab @ ab)
    if l2 < EPS:
        return np.linalg.norm(points - a, axis=1)
    t = np.clip(((points - a) @ ab) / l2, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(points - proj, axis=1)


def turning_angle(t_in: np.ndarray, t_out: np.ndarray) -> float:
    """Angle in radians between two direction vectors (0 = straight on)."""
    a = unit(t_in)
    b = unit(t_out)
    if not a.any() or not b.any():
        return 0.0
    return math.acos(max(-1.0, min(1.0, float(a @ b))))


@dataclass
class Plane:
    origin: np.ndarray
    u: np.ndarray
    v: np.ndarray
    normal: np.ndarray

    def project(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        q = points - self.origin
        return q @ self.u, q @ self.v, q @ self.normal

    def lift(self, x: float, y: float) -> np.ndarray:
        return self.origin + x * self.u + y * self.v


def fit_plane(points: np.ndarray) -> Plane:
    """Least-squares plane through ``points`` (n >= 3).

    Fast path for planar chains aligned with a coordinate plane (the usual
    2.5-D milling case); otherwise the eigenvectors of the 3x3 scatter matrix.
    """
    c = points.mean(axis=0)
    q = points - c
    ext = np.abs(q).max(axis=0)
    flat = np.flatnonzero(ext < 1e-9)
    if flat.size == 1:
        k = int(flat[0])
        normal = np.zeros(3)
        normal[k] = 1.0
        u = np.zeros(3)
        u[(k + 1) % 3] = 1.0
        v = cross3(normal, u)
        return Plane(c, u, v, normal)
    cov = q.T @ q
    w, vecs = np.linalg.eigh(cov)
    normal = unit(vecs[:, 0])
    u = unit(vecs[:, 2])
    v = cross3(normal, u)
    return Plane(c, u, v, normal)


def fit_circle_taubin(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float] | None:
    """Taubin algebraic circle fit. Returns (cx, cy, r) or None if degenerate."""
    n = len(x)
    if n < 3:
        return None
    mx, my = float(x.mean()), float(y.mean())
    u = x - mx
    v = y - my
    scale = float(max(np.abs(u).max(), np.abs(v).max(), EPS))
    u = u / scale
    v = v / scale
    z = u * u + v * v
    inv = 1.0 / n
    mxx = float(u @ u) * inv
    myy = float(v @ v) * inv
    mxy = float(u @ v) * inv
    mxz = float(u @ z) * inv
    myz = float(v @ z) * inv
    mzz = float(z @ z) * inv
    mz = mxx + myy
    cov_xy = mxx * myy - mxy * mxy
    var_z = mzz - mz * mz
    a3 = 4.0 * mz
    a2 = -3.0 * mz * mz - mzz
    a1 = var_z * mz + 4.0 * cov_xy * mz - mxz * mxz - myz * myz
    a0 = mxz * (mxz * myy - myz * mxy) + myz * (myz * mxx - mxz * mxy) - var_z * cov_xy
    a22 = 2.0 * a2
    a33 = 3.0 * a3
    xnew = 0.0
    ynew = 1e20
    for _ in range(25):
        yold = ynew
        ynew = a0 + xnew * (a1 + xnew * (a2 + xnew * a3))
        if abs(ynew) > abs(yold):
            xnew = 0.0
            break
        dy = a1 + xnew * (a22 + xnew * a33)
        if abs(dy) < EPS:
            break
        xold = xnew
        xnew = xold - ynew / dy
        if abs(xnew - xold) < 1e-12 * max(1.0, abs(xnew)):
            break
        if xnew < 0.0:
            xnew = 0.0
            break
    det = xnew * xnew - xnew * mz + cov_xy
    if abs(det) < 1e-13:
        return None
    cx = (mxz * (myy - xnew) - myz * mxy) / det / 2.0
    cy = (myz * (mxx - xnew) - mxz * mxy) / det / 2.0
    r = math.sqrt(cx * cx + cy * cy + mz)
    return cx * scale + mx, cy * scale + my, r * scale


def fit_arc_fixed_ends(
    x: np.ndarray, y: np.ndarray, init_center: tuple[float, float] | None = None
) -> tuple[float, float, float] | None:
    """Geometric least-squares circle through the first and last point.

    The centre is constrained to the perpendicular bisector of the chord
    P0-Pn, leaving one degree of freedom (t) that is solved by Gauss-Newton.
    Returns (cx, cy, r) or None (degenerate chord).
    """
    p0x, p0y = float(x[0]), float(y[0])
    pnx, pny = float(x[-1]), float(y[-1])
    chx, chy = pnx - p0x, pny - p0y
    length = math.hypot(chx, chy)
    if length < 1e-9:
        return None
    mx, my = (p0x + pnx) / 2.0, (p0y + pny) / 2.0
    nx, ny = -chy / length, chx / length
    half = length / 2.0
    t = 0.0
    if init_center is not None:
        t = (init_center[0] - mx) * nx + (init_center[1] - my) * ny
    xm = x - mx
    ym = y - my
    for _ in range(40):
        dx = xm - t * nx
        dy = ym - t * ny
        d = np.hypot(dx, dy)
        np.maximum(d, EPS, out=d)
        radius = math.sqrt(t * t + half * half)
        res = d - radius
        jac = -(dx * nx + dy * ny) / d - t / radius
        jj = float(jac @ jac)
        if jj < EPS:
            break
        step = -float(jac @ res) / jj
        t += step
        if abs(step) < 1e-10 * (1.0 + abs(t)):
            break
    cx, cy = mx + t * nx, my + t * ny
    return cx, cy, math.sqrt(t * t + half * half)


@dataclass
class ArcFit:
    """A circular arc fitted to an ordered chain of 3-D points."""

    center: np.ndarray
    radius: float
    normal: np.ndarray  # oriented so that the motion is counter-clockwise about it
    u: np.ndarray
    v: np.ndarray
    theta0: float
    sweep: float  # > 0, radians, 2*pi for a closed chain
    max_dev: float
    closed: bool

    def point_at(self, theta: float) -> np.ndarray:
        return self.center + self.radius * (math.cos(theta) * self.u + math.sin(theta) * self.v)

    def tangent_at(self, theta: float) -> np.ndarray:
        return unit(-math.sin(theta) * self.u + math.cos(theta) * self.v)

    @property
    def mid(self) -> np.ndarray:
        return self.point_at(self.theta0 + self.sweep / 2.0)

    @property
    def t_in(self) -> np.ndarray:
        return self.tangent_at(self.theta0)

    @property
    def t_out(self) -> np.ndarray:
        return self.tangent_at(self.theta0 + self.sweep)


def _point_segments_distance_2d(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance from one 2-D point to each segment a[k]-b[k]."""
    ab = b - a
    l2 = np.einsum("ij,ij->i", ab, ab)
    l2 = np.where(l2 < EPS, EPS, l2)
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / l2, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(p - proj, axis=1)


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + math.pi) % TWO_PI - math.pi


def fit_arc_3d(
    points: np.ndarray,
    tol: float,
    max_radius: float = 5000.0,
    min_radius: float = 0.1,
    close_eps: float = 1e-6,
) -> ArcFit | None:
    """Fit one circular arc to an ordered chain of points.

    Returns None unless all of the following hold:
      * every point lies within ``tol`` of the fitted plane and circle,
      * the chain progresses monotonically along the circle (no reversals),
      * the sagitta of every chord is within ``tol`` (the arc replaces the
        polyline, so the polyline must stay inside the tolerance band),
      * min_radius <= radius <= max_radius,
      * total sweep <= 360 degrees.
    """
    n = len(points)
    if n < 3:
        return None
    plane = fit_plane(points)
    x, y, w = plane.project(points)
    if float(np.abs(w).max()) > tol:
        return None
    dse = points[-1] - points[0]
    closed = math.sqrt(float(dse @ dse)) < close_eps
    init = fit_circle_taubin(x, y)
    if closed:
        fit = init
    else:
        fit = fit_arc_fixed_ends(x, y, (init[0], init[1]) if init else None)
    if fit is None:
        return None
    cx, cy, r = fit
    if not (min_radius <= r <= max_radius) or not math.isfinite(r):
        return None
    dx, dy = x - cx, y - cy
    d = np.sqrt(dx * dx + dy * dy)
    radial = float(np.abs(d - r).max())
    if radial > tol:
        return None
    theta = np.arctan2(dy, dx)
    dth = _wrap(np.diff(theta))
    if n == 2 or dth.size == 0:
        return None
    dth_inner = dth
    sign = 1.0 if float(dth_inner.sum()) >= 0 else -1.0
    if np.any(sign * dth_inner <= 1e-12):
        return None  # reversal or duplicate angle
    sweep = float(abs(dth_inner.sum()))
    if closed:
        if abs(sweep - TWO_PI) > 1e-6:
            return None
        sweep = TWO_PI
    elif sweep >= TWO_PI - 1e-9:
        return None
    # Exact in-plane deviation of every chord from the circle: along a chord the
    # radial distance is extremal either at the vertices (already checked) or at
    # the foot of the perpendicular from the centre, i.e. R - dist(centre, chord).
    chords = np.column_stack([x, y])
    dmin = _point_segments_distance_2d(np.array([cx, cy]), chords[:-1], chords[1:])
    chord_dev = float(np.clip(r - dmin, 0.0, None).max())
    inplane = max(radial, chord_dev)
    max_dev = math.hypot(inplane, float(np.abs(w).max()))
    if max_dev > tol:
        return None
    u, v, normal = plane.u, plane.v, plane.normal
    if sign < 0:
        v = -v
        normal = -normal
        theta = -theta
    center = plane.lift(cx, cy)
    return ArcFit(
        center=center,
        radius=r,
        normal=unit(normal),
        u=u,
        v=v,
        theta0=float(theta[0]),
        sweep=sweep,
        max_dev=max_dev,
        closed=closed,
    )


def arc_from_center(
    start: np.ndarray, end: np.ndarray, center: np.ndarray, normal: np.ndarray, ccw: bool
) -> ArcFit:
    """Build an ArcFit from explicit G2/G3 data (planar arc, start==end means full circle)."""
    normal = unit(normal)
    rs = start - center
    rs -= (rs @ normal) * normal
    r = float(np.linalg.norm(rs))
    u = unit(rs)
    v = cross3(normal, u)
    re = end - center
    re -= (re @ normal) * normal
    ang = math.atan2(float(re @ v), float(re @ u))
    if ccw:
        sweep = ang % TWO_PI
    else:
        sweep = (-ang) % TWO_PI
        v = -v
        normal = -normal
    dse = end - start
    closed = math.sqrt(float(dse @ dse)) < 1e-9
    if closed or sweep < 1e-12:
        sweep = TWO_PI
        closed = True
    return ArcFit(center=center, radius=r, normal=normal, u=u, v=v, theta0=0.0,
                  sweep=sweep, max_dev=0.0, closed=closed)


def sample_arc(arc: ArcFit, max_chord_err: float, min_points: int = 2) -> np.ndarray:
    """Sample points along the arc so that chord deviation <= max_chord_err."""
    if arc.radius <= max_chord_err:
        step = math.pi / 2
    else:
        step = 2.0 * math.acos(1.0 - max_chord_err / arc.radius)
    k = max(min_points - 1, int(math.ceil(arc.sweep / max(step, 1e-6))))
    thetas = arc.theta0 + arc.sweep * np.linspace(0.0, 1.0, k + 1)
    return np.array([arc.point_at(float(t)) for t in thetas])
