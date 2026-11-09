"""Slice a mesh file or triangle array into G-code."""
from __future__ import annotations

import numpy as np

from ..meshio import load_mesh
from .deform import make_deformation
from .gcode_writer import SliceStats, write_gcode
from .adaptive import adaptive_heights
from .mesh_slice import layer_heights, slice_layers
from .params import SliceParams
from .toolpath import build_layers, support_report


def placed(tris: np.ndarray, params: SliceParams) -> np.ndarray:
    """Apply the model placement of the parameters (rotation / scale / mirror or the
    automatic orientation)."""
    from .plater import Placement, auto_orient, transform_triangles

    rot = (params.rot_x, params.rot_y, params.rot_z)
    if params.auto_orient in ("lay_flat", "min_overhang"):
        rot = tuple(auto_orient(tris, params.auto_orient, params.overhang_max)["rot"])
    pl = Placement(rot=rot, scale=(params.scale,) * 3, mirror=(params.mirror_x, params.mirror_y, params.mirror_z))
    if rot == (0.0, 0.0, 0.0) and params.scale == 1.0 and not any(pl.mirror):
        return tris
    return transform_triangles(tris, pl)


def slice_triangles(tris: np.ndarray, params: SliceParams | None = None, source: str = "",
                    segments: str | list | None = None, pre_placed: bool = False) -> tuple[str, SliceStats, list]:
    """Slice ``tris`` (mm). ``segments`` (text or list of Segment) switches to
    multi-directional printing (layer_mode "multi"). ``pre_placed`` skips the
    placement from the parameters (used by the plater)."""
    params = params or SliceParams()
    tris = np.asarray(tris, dtype=float)
    if len(tris) == 0:
        raise ValueError("empty mesh")
    if not pre_placed:
        tris = placed(tris, params)
    from .ranges import layer_heights_with_ranges, parse_ranges
    ranges = parse_ranges(params.ranges)
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
        # features that only make sense with flat layers
        params = SliceParams.from_dict({**params.to_dict(), "spiral_vase": False, "raft_layers": 0, "support": "none", "adaptive_layers": False})
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
    heights = None
    if ranges and any("layer_height" in r["set"] for r in ranges) and deform.is_planar:
        zs, heights = layer_heights_with_ranges(z_max, params, ranges)
    elif params.adaptive_layers and deform.is_planar:
        h_min = max(0.05, min(params.layer_height_min, params.layer_height_max))
        h_max = max(h_min, params.layer_height_max)
        zs, heights = adaptive_heights(tris_s, min(params.first_layer_height, h_max), h_min, h_max, max(0.01, params.adaptive_quality))
    else:
        zs = layer_heights(0.0, z_max, params.layer_height, params.first_layer_height)
    if not zs:
        raise ValueError("model is thinner than the first layer")
    loops, dropped = slice_layers(tris_s, zs, tol=max(1e-3, params.bead_width * 0.02))
    layers = build_layers(loops, zs, params, heights, ranges=ranges)
    if not deform.is_planar:
        for L in layers:
            for path in L.paths:
                p3 = np.column_stack([path.points, np.full(len(path.points), L.z + z_shift)]) if path.xyz is None else path.xyz + np.array([0.0, 0.0, z_shift])
                xyz = deform.inverse(p3)
                path.xyz = xyz
                path.axes = deform.normal(xyz)
                path.tf = deform.thickness_factor(xyz)
    from .ranges import params_for_z
    for L in layers:
        lp = params_for_z(params, ranges, L.z) if ranges else params
        if abs(lp.tilt_lead) > 1e-9 or abs(lp.tilt_side) > 1e-9:
            from .orientation import tilt_axes
            for path in L.paths:
                p3 = path.xyz if path.xyz is not None else np.column_stack([path.points, np.full(len(path.points), L.z)])
                path.axes = tilt_axes(p3, path.axes, path.closed and path.xyz is None, lp.tilt_lead, lp.tilt_side)
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


def slice_models(models: list, params: SliceParams | None = None, source: str = "",
                 segments: str | list | None = None) -> tuple[str, SliceStats, list]:
    """Slice several placed models: ``models`` = [(triangles, Placement), ...].

    ``params.print_sequence`` "all" prints them layer by layer as one object,
    "sequential" finishes each object before the next one (the G-code of the
    objects is concatenated; mind the clearance of the extruder next to finished
    objects)."""
    from .plater import combine, transform_triangles

    params = params or SliceParams()
    if not models:
        raise ValueError("no models")
    if len(models) == 1 or params.print_sequence != "sequential":
        tris = combine(models)
        return slice_triangles(tris, params, source=source, segments=segments, pre_placed=True)
    # sequential: one common XY shift (so coordinates agree with the "all" mode)
    allt = combine(models)
    v = allt.reshape(-1, 3)
    shift = np.zeros(3)
    if params.origin_center:
        shift[0] = -(v[:, 0].min() + v[:, 0].max()) / 2.0
        shift[1] = -(v[:, 1].min() + v[:, 1].max()) / 2.0
    p2 = SliceParams.from_dict({**params.to_dict(), "origin_center": False})
    texts, layers_all = [], []
    total = None
    for k, (t, pl) in enumerate(models):
        tt = transform_triangles(t, pl) + shift
        g, st, layers = slice_triangles(tt, p2, source=f"{source} [{pl.name}]", segments=segments, pre_placed=True)
        lines = g.rstrip().splitlines()
        if lines and lines[-1] == "M30":
            lines = lines[:-1]
        texts.append("\n".join(lines) + "\n")
        for L in layers:
            L.index += 100000 * k  # keep layer indices unique across objects
        layers_all.extend(layers)
        if total is None:
            total = st
        else:
            for f in ("layers", "paths", "extrusion_length", "travel_length", "e_total", "volume_mm3", "mass_g", "time_s", "dropped_chains",
                      "support_area", "support_layers", "support_volume", "bridges", "overhang_length", "slowed_layers"):
                setattr(total, f, getattr(total, f) + getattr(st, f))
            for kk, vv in st.by_type.items():
                total.by_type[kk] = total.by_type.get(kk, 0.0) + vv
            for kk, vv in st.by_type_time.items():
                total.by_type_time[kk] = total.by_type_time.get(kk, 0.0) + vv
            total.max_tilt = max(total.max_tilt, st.max_tilt)
            total.min_layer_height = min(total.min_layer_height, st.min_layer_height)
            total.max_layer_height = max(total.max_layer_height, st.max_layer_height)
            if st.bounds:
                b = total.bounds
                total.bounds = {"min": [min(a, c) for a, c in zip(b["min"], st.bounds["min"])], "max": [max(a, c) for a, c in zip(b["max"], st.bounds["max"])]}
    return "".join(texts) + "M30\n", total, layers_all


def preview_segments(layers: list) -> dict:
    """Line segments per feature kind for the viewer: {kind: float32 (N*6,) array}."""
    out: dict = {}
    for L in layers:
        for path in L.paths:
            P = path.xyz if path.xyz is not None else np.column_stack([path.points, np.full(len(path.points), L.z)])
            if len(P) < 2:
                continue
            seq = np.vstack([P, P[:1]]) if path.closed else P
            seg = np.hstack([seq[:-1], seq[1:]]).astype(np.float32).ravel()
            out.setdefault(path.kind, []).append(seg)
    return {k: np.concatenate(v) for k, v in out.items()}
