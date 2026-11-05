"""Layer modes by deformation: slice a deformed model with planar layers, then map the
paths back (deform -> slice -> undeform), the approach of conical slicing (RotBot,
Wüthrich & Gubser 2021) and CurviSlicer (Etienne et al. 2019).

A deformation D maps model space (x, y, z) to slicing space (x, y, z') with
z' = z + g(x, y) or z' = s(x, y) * z (XY unchanged). Planar layers z' = const in
slicing space become the surfaces z = z' - g(x, y) in model space; the local
layer normal is the tool axis (pointing away from the part), and the layer
thickness along that normal shrinks by cos(tilt).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


class Deformation:
    name = "planar"

    def forward(self, p: np.ndarray) -> np.ndarray:
        return p

    def inverse(self, p: np.ndarray) -> np.ndarray:
        return p

    def normal(self, p: np.ndarray) -> np.ndarray:
        """Unit layer normals (tool axes) at model-space points (N,3)."""
        n = np.zeros_like(p)
        n[:, 2] = 1.0
        return n

    def thickness_factor(self, p: np.ndarray) -> np.ndarray:
        """Real layer thickness / slicing-space layer height at model-space points."""
        return self.normal(p)[:, 2]

    @property
    def is_planar(self) -> bool:
        return True


@dataclass
class Conical(Deformation):
    """Conical layers: z' = z + k * r (outward) or z' = z - k * r (inward), k = tan(angle),
    r = distance from the vertical axis through (cx, cy). The nozzle tilts by ``angle``."""

    angle_deg: float = 30.0
    cx: float = 0.0
    cy: float = 0.0
    outward: bool = True
    name = "conical"

    def _k(self) -> float:
        return math.tan(math.radians(self.angle_deg)) * (1.0 if self.outward else -1.0)

    def forward(self, p):
        q = np.array(p, dtype=float)
        r = np.hypot(q[:, 0] - self.cx, q[:, 1] - self.cy)
        q[:, 2] = q[:, 2] + self._k() * r
        return q

    def inverse(self, p):
        q = np.array(p, dtype=float)
        r = np.hypot(q[:, 0] - self.cx, q[:, 1] - self.cy)
        q[:, 2] = q[:, 2] - self._k() * r
        return q

    r_blend: float = 6.0  # within this radius of the axis the tilt fades to vertical (apex is singular)

    def normal(self, p):
        q = np.asarray(p, dtype=float)
        dx, dy = q[:, 0] - self.cx, q[:, 1] - self.cy
        r = np.hypot(dx, dy)
        safe = np.where(r < 1e-9, 1e-9, r)
        fade = np.clip(r / max(self.r_blend, 1e-6), 0.0, 1.0)
        k = self._k()
        n = np.column_stack([k * fade * dx / safe, k * fade * dy / safe, np.ones(len(q))])
        return n / np.linalg.norm(n, axis=1)[:, None]

    @property
    def is_planar(self) -> bool:
        return False


@dataclass
class HeightField(Deformation):
    """Layers following a height field h(x, y): z' = z - h(x, y) (bottom-following,
    layers parallel to the surface below) or z' = z * H / h(x, y) (top-following,
    the top surface becomes the last flat layer, CurviSlicer-like). ``grid`` is
    a regular sample of h over [x0, x1] x [y0, y1]; slopes are clamped to
    ``max_slope_deg`` so layers never tilt more than the robot/nozzle allows."""

    grid: np.ndarray = None  # (ny, nx) heights
    x0: float = 0.0
    y0: float = 0.0
    dx: float = 1.0
    dy: float = 1.0
    mode: str = "bottom"  # bottom | top
    h_ref: float = 1.0  # top mode: reference height (model max z)
    name = "heightfield"

    def h(self, x, y) -> np.ndarray:
        g = self.grid
        ny, nx = g.shape
        fx = np.clip((np.asarray(x) - self.x0) / self.dx, 0, nx - 1 - 1e-9)
        fy = np.clip((np.asarray(y) - self.y0) / self.dy, 0, ny - 1 - 1e-9)
        ix, iy = fx.astype(int), fy.astype(int)
        tx, ty = fx - ix, fy - iy
        ix1, iy1 = np.minimum(ix + 1, nx - 1), np.minimum(iy + 1, ny - 1)
        return (g[iy, ix] * (1 - tx) * (1 - ty) + g[iy, ix1] * tx * (1 - ty) + g[iy1, ix] * (1 - tx) * ty + g[iy1, ix1] * tx * ty)

    def grad(self, x, y) -> tuple[np.ndarray, np.ndarray]:
        e = 0.5 * min(self.dx, self.dy)
        return (self.h(x + e, y) - self.h(x - e, y)) / (2 * e), (self.h(x, y + e) - self.h(x, y - e)) / (2 * e)

    def forward(self, p):
        q = np.array(p, dtype=float)
        hv = self.h(q[:, 0], q[:, 1])
        if self.mode == "top":
            q[:, 2] = q[:, 2] * self.h_ref / np.where(hv < 1e-6, 1e-6, hv)
        else:
            q[:, 2] = q[:, 2] - hv
        return q

    def inverse(self, p):
        q = np.array(p, dtype=float)
        hv = self.h(q[:, 0], q[:, 1])
        if self.mode == "top":
            q[:, 2] = q[:, 2] * hv / self.h_ref
        else:
            q[:, 2] = q[:, 2] + hv
        return q

    def normal(self, p):
        q = np.asarray(p, dtype=float)
        gx, gy = self.grad(q[:, 0], q[:, 1])
        if self.mode == "top":
            # layer z = z' * h / H  ->  z - c*h(x,y) = 0 with c = z'/H = z/h
            hv = np.where(self.h(q[:, 0], q[:, 1]) < 1e-6, 1e-6, self.h(q[:, 0], q[:, 1]))
            c = q[:, 2] / hv
            gx, gy = c * gx, c * gy
        n = np.column_stack([-gx, -gy, np.ones(len(q))])
        return n / np.linalg.norm(n, axis=1)[:, None]

    @property
    def is_planar(self) -> bool:
        return False


def height_field_from_mesh(tris: np.ndarray, which: str, cell: float, max_slope_deg: float) -> HeightField:
    """Sample the lowest ("bottom") or highest ("top") surface of a mesh on a grid by
    vertical ray casting; slopes above ``max_slope_deg`` are smoothed/clamped."""
    v = tris.reshape(-1, 3)
    x0, y0 = float(v[:, 0].min()) - cell, float(v[:, 1].min()) - cell
    x1, y1 = float(v[:, 0].max()) + cell, float(v[:, 1].max()) + cell
    nx, ny = int(math.ceil((x1 - x0) / cell)) + 1, int(math.ceil((y1 - y0) / cell)) + 1
    xs = x0 + np.arange(nx) * cell
    ys = y0 + np.arange(ny) * cell
    gx, gy = np.meshgrid(xs, ys)
    px, py = gx.ravel(), gy.ravel()
    best = np.full(len(px), np.inf if which == "bottom" else -np.inf)
    # vectorised ray/triangle intersection (vertical rays): barycentric test in XY
    for t in tris:
        a, b, c = t
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-12:
            continue
        bx = px.min() - 1e-9 <= max(a[0], b[0], c[0]) and px.max() + 1e-9 >= min(a[0], b[0], c[0])
        if not bx:
            continue
        l1 = ((b[1] - c[1]) * (px - c[0]) + (c[0] - b[0]) * (py - c[1])) / d
        l2 = ((c[1] - a[1]) * (px - c[0]) + (a[0] - c[0]) * (py - c[1])) / d
        l3 = 1.0 - l1 - l2
        inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
        if not inside.any():
            continue
        z = l1 * a[2] + l2 * b[2] + l3 * c[2]
        if which == "bottom":
            best = np.where(inside, np.minimum(best, z), best)
        else:
            best = np.where(inside, np.maximum(best, z), best)
    grid = best.reshape(ny, nx)
    # fill cells outside the footprint with the nearest valid value (simple diffusion)
    valid = np.isfinite(grid)
    if not valid.any():
        raise ValueError("height field: the mesh has no vertical extent")
    fill = grid.copy()
    fill[~valid] = np.nan
    # extrapolate outside the footprint along rows and columns (keeps the slope at the edges)
    for _ in range(max(nx, ny)):
        if not np.isnan(fill).any():
            break
        new = fill.copy()
        for axis in (0, 1):
            for direction in (1, -1):
                a = np.roll(fill, direction, axis=axis)      # neighbour one step back
                b = np.roll(fill, 2 * direction, axis=axis)  # two steps back
                if axis == 0:
                    edge = slice(0, 2) if direction == 1 else slice(-2, None)
                    a[edge, :] = np.nan
                    b[edge, :] = np.nan
                else:
                    edge = slice(0, 2) if direction == 1 else slice(-2, None)
                    a[:, edge] = np.nan
                    b[:, edge] = np.nan
                cand = 2 * a - b
                cand = np.where(np.isnan(cand), a, cand)
                new = np.where(np.isnan(new) & ~np.isnan(cand), cand, new)
        if np.array_equal(np.isnan(new), np.isnan(fill)):
            # isolated gaps: fall back to the mean of neighbours
            pad = np.pad(fill, 1, mode="edge")
            neigh = np.stack([pad[:-2, 1:-1], pad[2:, 1:-1], pad[1:-1, :-2], pad[1:-1, 2:]])
            with np.errstate(all="ignore"):
                mean = np.nanmean(neigh, axis=0)
            new = np.where(np.isnan(fill), mean, fill)
            if np.array_equal(np.isnan(new), np.isnan(fill)):
                break
        fill = new
    grid = np.nan_to_num(fill, nan=float(np.nanmean(fill)))
    # clamp slopes: iterative limiting of neighbour differences
    max_d = math.tan(math.radians(max_slope_deg)) * cell
    for _ in range(50):
        changed = False
        for axis in (0, 1):
            diff = np.diff(grid, axis=axis)
            over = np.abs(diff) > max_d
            if over.any():
                changed = True
                corr = np.sign(diff) * (np.abs(diff) - max_d) * 0.5
                corr[~over] = 0
                if axis == 0:
                    grid[:-1, :] += corr
                    grid[1:, :] -= corr
                else:
                    grid[:, :-1] += corr
                    grid[:, 1:] -= corr
        if not changed:
            break
    return HeightField(grid=grid, x0=x0, y0=y0, dx=cell, dy=cell)


def make_deformation(mode: str, tris: np.ndarray, **kw) -> Deformation:
    mode = (mode or "planar").lower()
    if mode == "planar":
        return Deformation()
    if mode == "conical":
        v = tris.reshape(-1, 3)
        cx = kw.get("cx")
        cy = kw.get("cy")
        if cx is None:
            cx = float((v[:, 0].min() + v[:, 0].max()) / 2)
        if cy is None:
            cy = float((v[:, 1].min() + v[:, 1].max()) / 2)
        return Conical(float(kw.get("angle", 30.0)), cx, cy, bool(kw.get("outward", True)), r_blend=float(kw.get("r_blend", 6.0)))
    if mode in ("bottom", "top"):
        cell = float(kw.get("cell", 5.0))
        hf = height_field_from_mesh(tris, mode, cell, float(kw.get("max_slope", 45.0)))
        hf.mode = mode
        v = tris.reshape(-1, 3)
        hf.h_ref = float(v[:, 2].max() - v[:, 2].min())
        if mode == "top":
            hf.grid = hf.grid - float(v[:, 2].min())
            hf.grid = np.maximum(hf.grid, 1e-3)
        return hf
    raise ValueError(f"unknown layer mode {mode!r} (planar, conical, bottom, top)")
