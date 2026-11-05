"""Planar slicing of a triangle soup into closed contours per layer.

For every layer height z the triangles crossing the plane are intersected,
producing segments; segments are linked end-to-end into closed loops using a
spatial hash with a tolerance, so meshes with small gaps still close. Open
chains that cannot be closed are dropped (reported in stats).
"""
from __future__ import annotations

import math
from collections import defaultdict

import numpy as np


def plane_segments(tris: np.ndarray, z: float, eps: float = 1e-9) -> np.ndarray:
    """Intersection segments (M,2,2) of triangles (N,3,3) with the plane Z=z."""
    zc = tris[:, :, 2]
    zmin = zc.min(axis=1)
    zmax = zc.max(axis=1)
    cand = tris[(zmin <= z) & (zmax >= z) & (zmax - zmin > eps)]
    if len(cand) == 0:
        return np.zeros((0, 2, 2))
    segs = []
    for t in cand:
        pts = []
        for i in range(3):
            a, b = t[i], t[(i + 1) % 3]
            za, zb = a[2] - z, b[2] - z
            if za == 0 and zb == 0:
                continue  # edge in plane: handled by the neighbouring triangles' other edges
            if za == 0:
                pts.append(a[:2])
            elif zb == 0:
                pts.append(b[:2])
            elif (za < 0) != (zb < 0):
                s = za / (za - zb)
                pts.append(a[:2] + s * (b[:2] - a[:2]))
        # dedupe (vertex exactly on plane appears twice)
        uniq = []
        for p in pts:
            if not any(np.hypot(*(p - q)) < 1e-9 for q in uniq):
                uniq.append(p)
        if len(uniq) == 2:
            segs.append(uniq)
    return np.array(segs) if segs else np.zeros((0, 2, 2))


def link_segments(segs: np.ndarray, tol: float = 1e-3) -> tuple[list[np.ndarray], int]:
    """Join segments into closed loops. Returns (loops as (k,2) arrays, number of dropped open chains)."""
    if len(segs) == 0:
        return [], 0
    n = len(segs)
    cell = max(tol, 1e-6)

    def key(p):
        return (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)))

    grid: dict[tuple, list[tuple[int, int]]] = defaultdict(list)
    for i in range(n):
        for e in (0, 1):
            grid[key(segs[i, e])].append((i, e))

    def near(p):
        kx, ky = key(p)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for item in grid.get((kx + dx, ky + dy), ()):
                    yield item

    used = np.zeros(n, dtype=bool)
    loops = []
    dropped = 0
    for start in range(n):
        if used[start]:
            continue
        used[start] = True
        chain = [segs[start, 0].copy(), segs[start, 1].copy()]
        closed = False
        # extend forward from the tail
        for _ in range(n):
            tail = chain[-1]
            if len(chain) > 2 and np.hypot(*(tail - chain[0])) <= tol:
                closed = True
                break
            best = None
            for i, e in near(tail):
                if used[i]:
                    continue
                d = np.hypot(*(segs[i, e] - tail))
                if d <= tol and (best is None or d < best[0]):
                    best = (d, i, e)
            if best is None:
                break
            _, i, e = best
            used[i] = True
            chain.append(segs[i, 1 - e].copy())
        if not closed and len(chain) > 2 and np.hypot(*(chain[-1] - chain[0])) <= 10 * tol:
            closed = True
        if closed:
            loop = np.array(chain[:-1]) if np.hypot(*(chain[-1] - chain[0])) <= 10 * tol else np.array(chain)
            if len(loop) >= 3:
                loops.append(loop)
        else:
            dropped += 1
    return loops, dropped


def layer_heights(z_min: float, z_max: float, layer_height: float, first_layer: float) -> list[float]:
    """Slice planes: first layer at z_min + first_layer, then every layer_height, up to z_max."""
    out = []
    z = z_min + first_layer
    while z <= z_max + 1e-6:
        out.append(round(z, 6))
        z += layer_height
    return out


def slice_layers(tris: np.ndarray, zs: list[float], tol: float = 1e-3) -> tuple[list[list[np.ndarray]], int]:
    """Closed loops for each plane (list per layer) and the number of dropped open chains."""
    layers = []
    dropped = 0
    for z in zs:
        # slice slightly below the plane to avoid hitting flat faces exactly
        segs = plane_segments(tris, z - 1e-6)
        loops, d = link_segments(segs, tol)
        dropped += d
        layers.append(loops)
    return layers, dropped
