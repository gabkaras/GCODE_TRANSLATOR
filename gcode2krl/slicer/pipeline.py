"""Slice a mesh file or triangle array into G-code."""
from __future__ import annotations

import numpy as np

from ..meshio import load_mesh
from .gcode_writer import SliceStats, write_gcode
from .mesh_slice import layer_heights, slice_layers
from .params import SliceParams
from .toolpath import build_layers


def slice_triangles(tris: np.ndarray, params: SliceParams | None = None, source: str = "") -> tuple[str, SliceStats, list]:
    params = params or SliceParams()
    tris = np.asarray(tris, dtype=float)
    if len(tris) == 0:
        raise ValueError("empty mesh")
    v = tris.reshape(-1, 3)
    shift = np.zeros(3)
    if params.origin_center:
        shift[0] = -(v[:, 0].min() + v[:, 0].max()) / 2.0
        shift[1] = -(v[:, 1].min() + v[:, 1].max()) / 2.0
    shift[2] = -v[:, 2].min()  # model sits on Z = 0
    tris = tris + shift
    z_max = float(tris[:, :, 2].max())
    zs = layer_heights(0.0, z_max, params.layer_height, params.first_layer_height)
    if not zs:
        raise ValueError("model is thinner than the first layer")
    loops, dropped = slice_layers(tris, zs, tol=max(1e-3, params.bead_width * 0.02))
    layers = build_layers(loops, zs, params)
    gcode, stats = write_gcode(layers, params, source)
    stats.dropped_chains = dropped
    return gcode, stats, layers


def slice_mesh(path: str, params: SliceParams | None = None, units: str = "mm", deflection: float = 0.1) -> tuple[str, SliceStats, list]:
    import os
    tris = load_mesh(path, units, deflection=deflection)
    return slice_triangles(tris, params, source=os.path.basename(path))
