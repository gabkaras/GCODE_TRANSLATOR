"""Adaptive (variable) layer heights from the surface slope.

For a layer of height h the stair-step cusp on a facet with unit normal n is
h * |n_z|; the layer height is chosen so the cusp stays below ``quality`` on
every facet the layer crosses, clamped to [h_min, h_max]. Horizontal facets
(flat tops/bottoms) are ignored like in PrusaSlicer, otherwise every flat
surface would force the minimum height.
"""
from __future__ import annotations

import numpy as np


def facet_limits(tris: np.ndarray, quality: float, h_min: float, h_max: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per facet: z range and the largest layer height allowed by the cusp rule."""
    t = np.asarray(tris, float)
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-12
    nz = np.zeros(len(t))
    nz[ok] = np.abs(n[ok, 2] / ln[ok])
    z0 = t[:, :, 2].min(axis=1)
    z1 = t[:, :, 2].max(axis=1)
    allowed = np.full(len(t), h_max)
    sloped = ok & (nz > 1e-3) & (nz < 0.999)
    allowed[sloped] = np.clip(quality / nz[sloped], h_min, h_max)
    return z0, z1, allowed


def adaptive_heights(tris: np.ndarray, first_layer: float, h_min: float, h_max: float, quality: float) -> tuple[list[float], list[float]]:
    """Slice planes and layer heights from the bottom (z = 0) to the top of the mesh."""
    z0, z1, allowed = facet_limits(tris, quality, h_min, h_max)
    z_top = float(np.asarray(tris)[:, :, 2].max())
    zs, hs = [], []
    z = 0.0
    h = min(first_layer, z_top) if z_top > 0 else first_layer
    zs.append(round(z + h, 6))
    hs.append(h)
    z += h
    while z + h_min <= z_top + 1e-6:
        # facets crossed by a layer [z, z + h_max]
        m = (z1 > z + 1e-9) & (z0 < z + h_max - 1e-9)
        h = float(allowed[m].min()) if m.any() else h_max
        h = min(max(h, h_min), h_max)
        if z + h > z_top + 1e-6:
            h = max(h_min, z_top - z)
            if h < h_min * 0.5:
                break
        z += h
        zs.append(round(z, 6))
        hs.append(h)
    return zs, hs
