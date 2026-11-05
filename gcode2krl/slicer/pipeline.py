"""Slice a mesh file or triangle array into G-code."""
from __future__ import annotations

import numpy as np

from ..meshio import load_mesh
from .deform import make_deformation
from .gcode_writer import SliceStats, write_gcode
from .mesh_slice import layer_heights, slice_layers
from .params import SliceParams
from .toolpath import build_layers, support_report


def slice_triangles(tris: np.ndarray, params: SliceParams | None = None, source: str = "",
                    segments: str | list | None = None) -> tuple[str, SliceStats, list]:
    """Slice ``tris`` (mm). ``segments`` (text or list of Segment) switches to
    multi-directional printing (layer_mode "multi")."""
    params = params or SliceParams()
    tris = np.asarray(tris, dtype=float)
    if len(tris) == 0:
        raise ValueError("empty mesh")
    if segments is None and params.layer_mode == "multi":
        raise ValueError("layer_mode 'multi' needs segment definitions")
    if segments is not None and (not isinstance(segments, str) or segments.strip()):
        from .decompose import parse_segments, slice_segments
        segs = parse_segments(segments) if isinstance(segments, str) else list(segments)
        if segs:
            v = tris.reshape(-1, 3)
            shift = np.zeros(3)
            if params.origin_center:
                shift[0] = -(v[:, 0].min() + v[:, 0].max()) / 2.0
                shift[1] = -(v[:, 1].min() + v[:, 1].max()) / 2.0
            shift[2] = -v[:, 2].min()
            gcode, st, per_seg = slice_segments(tris + shift, segs, params, source)
            return gcode, st, [L for layers in per_seg for L in layers]
    v = tris.reshape(-1, 3)
    shift = np.zeros(3)
    if params.origin_center:
        shift[0] = -(v[:, 0].min() + v[:, 0].max()) / 2.0
        shift[1] = -(v[:, 1].min() + v[:, 1].max()) / 2.0
    shift[2] = -v[:, 2].min()  # model sits on Z = 0
    tris = tris + shift
    deform = make_deformation(params.layer_mode, tris, angle=params.cone_angle, outward=params.cone_outward,
                              cx=params.cone_cx, cy=params.cone_cy, cell=params.hf_cell, max_slope=params.max_slope)
    if not deform.is_planar:
        from ..robot_import import subdivide
        tris = subdivide(tris, max(2.0 * params.bead_width, 4.0))
        v = tris.reshape(-1, 3)
        tris_s = deform.forward(v).reshape(-1, 3, 3)
        tris_s = tris_s - np.array([0.0, 0.0, float(tris_s[:, :, 2].min())])
        z_shift = float(deform.forward(v)[:, 2].min())
    else:
        tris_s = tris
        z_shift = 0.0
    z_max = float(tris_s[:, :, 2].max())
    zs = layer_heights(0.0, z_max, params.layer_height, params.first_layer_height)
    if not zs:
        raise ValueError("model is thinner than the first layer")
    loops, dropped = slice_layers(tris_s, zs, tol=max(1e-3, params.bead_width * 0.02))
    layers = build_layers(loops, zs, params)
    if not deform.is_planar:
        for L in layers:
            for path in L.paths:
                p3 = np.column_stack([path.points, np.full(len(path.points), L.z + z_shift)])
                xyz = deform.inverse(p3)
                path.xyz = xyz
                path.axes = deform.normal(xyz)
                path.tf = deform.thickness_factor(xyz)
    gcode, stats = write_gcode(layers, params, source)
    stats.dropped_chains = dropped
    sup = support_report(layers, params.bead_width, params.overhang_max)
    stats.support_area = sup["support_area"]
    stats.support_layers = sup["support_layers"]
    return gcode, stats, layers


def slice_mesh(path: str, params: SliceParams | None = None, units: str = "mm", deflection: float = 0.1,
               segments: str | None = None) -> tuple[str, SliceStats, list]:
    import os
    tris = load_mesh(path, units, deflection=deflection)
    return slice_triangles(tris, params, source=os.path.basename(path), segments=segments)
