"""Multi-directional printing by manual decomposition.

The model is split into ordered segments; each segment is the part of the
model inside a set of half-spaces (cut planes) and is printed with planar
layers along its own direction (RoboFDM, Wu et al. ICRA 2017; General
Support-Effective Decomposition, Wu et al. 2019). Here the decomposition
is given by the user (planes and directions), the slicer does the rest:
the model is rotated so the segment direction is +Z, planar slices are
clipped by the segment's half-planes layer by layer (no mesh booleans
needed), tool paths are generated and rotated back; the tool axis of the
segment is its direction. Segments are printed in list order; the caller
can add the already printed segments as obstacles for collision checks.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import Polygon, box as sbox

from .params import SliceParams


@dataclass
class HalfSpace:
    point: np.ndarray  # a point on the plane (model coordinates)
    normal: np.ndarray  # unit normal; the kept side is where (p - point) . normal >= 0


@dataclass
class Segment:
    name: str
    direction: np.ndarray  # printing direction (layer normal) in model coordinates, unit
    planes: list[HalfSpace] = field(default_factory=list)
    seed_z: float | None = None  # optional: start layers at this height along the direction


def rotation_to_z(d: np.ndarray) -> np.ndarray:
    """Rotation matrix R with R @ d = +Z (minimal rotation)."""
    d = np.asarray(d, dtype=float)
    d = d / np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.clip(d @ z, -1.0, 1.0))
    if c > 1 - 1e-12:
        return np.eye(3)
    if c < -1 + 1e-12:
        return np.diag([1.0, -1.0, -1.0])
    k = np.cross(d, z)
    s = float(np.linalg.norm(k))
    k = k / s
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * kx + (1 - c) * kx @ kx


def parse_segments(text: str) -> list[Segment]:
    """One segment per block of lines:
        segment NAME dx dy dz
        plane px py pz nx ny nz      (keep the side the normal points to)
    """
    segs: list[Segment] = []
    for ln, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.replace(",", " ").split()
        try:
            if parts[0].lower() == "segment":
                d = np.array([float(v) for v in parts[2:5]])
                if np.linalg.norm(d) < 1e-9:
                    raise ValueError("zero direction")
                segs.append(Segment(parts[1], d / np.linalg.norm(d)))
            elif parts[0].lower() == "plane":
                if not segs:
                    raise ValueError("plane before any segment")
                p = np.array([float(v) for v in parts[1:4]])
                n = np.array([float(v) for v in parts[4:7]])
                if np.linalg.norm(n) < 1e-9:
                    raise ValueError("zero plane normal")
                segs[-1].planes.append(HalfSpace(p, n / np.linalg.norm(n)))
            else:
                raise ValueError(f"unknown keyword {parts[0]!r}")
        except (IndexError, ValueError) as exc:
            raise ValueError(f"segments line {ln}: {exc}") from exc
    return segs


def clip_region_by_halfspaces(region, z: float, planes_rot: list[HalfSpace], extent: float):
    """Clip a layer region (slicing space, plane Z = z) by rotated half-spaces."""
    out = region
    for hs in planes_rot:
        n, p = hs.normal, hs.point
        nxy = np.hypot(n[0], n[1])
        c = float(n @ p) - n[2] * z  # n_x x + n_y y >= c
        if nxy < 1e-9:
            # plane parallel to the layer: whole layer kept or dropped
            if n[2] * (z - p[2]) < 0:
                return region.__class__()  # empty of the same type
            continue
        # half-plane as a huge polygon
        ux, uy = n[0] / nxy, n[1] / nxy
        cx, cy = ux * c / nxy, uy * c / nxy
        tx, ty = -uy, ux
        big = extent * 4
        poly = Polygon([(cx + tx * big, cy + ty * big), (cx - tx * big, cy - ty * big),
                        (cx - tx * big + ux * big, cy - ty * big + uy * big), (cx + tx * big + ux * big, cy + ty * big + uy * big)])
        out = out.intersection(poly)
        if out.is_empty:
            break
    return out


def slice_segments(tris: np.ndarray, segments: list[Segment], params: SliceParams, source: str = ""):
    """Slice every segment in its own direction; returns (gcode, stats, layers_per_segment)."""
    from .gcode_writer import write_gcode
    from .mesh_slice import layer_heights, slice_layers
    from .toolpath import as_multi, build_layers

    all_layers = []
    gcode_parts = []
    stats_total = None
    v_all = tris.reshape(-1, 3)
    extent = float(np.linalg.norm(v_all.max(axis=0) - v_all.min(axis=0))) + 1.0
    layer_index = 0
    for si, seg in enumerate(segments):
        R = rotation_to_z(seg.direction)
        tris_r = (tris.reshape(-1, 3) @ R.T).reshape(-1, 3, 3)
        planes_r = [HalfSpace(R @ hs.point, R @ hs.normal) for hs in seg.planes]
        z_lo = float(tris_r[:, :, 2].min())
        z_hi = float(tris_r[:, :, 2].max())
        # the segment's own Z range from its planes (planes parallel to the layers bound it)
        for hs in planes_r:
            if np.hypot(hs.normal[0], hs.normal[1]) < 1e-9:
                zp = float(hs.point[2])
                if hs.normal[2] > 0:
                    z_lo = max(z_lo, zp)
                else:
                    z_hi = min(z_hi, zp)
        if seg.seed_z is not None:
            z_lo = max(z_lo, seg.seed_z)
        zs = layer_heights(z_lo, z_hi, params.layer_height, params.first_layer_height)
        if not zs:
            continue
        loops, dropped = slice_layers(tris_r, zs, tol=max(1e-3, params.bead_width * 0.02))
        layers = build_layers(loops, zs, SliceParams.from_dict({**params.to_dict(), "layer_mode": "planar"}))
        # clip after region construction: rebuild paths on clipped regions
        for L in layers:
            L.region = as_multi(clip_region_by_halfspaces(L.region, L.z, planes_r, extent))
        # regions changed -> regenerate tool paths
        layers = build_layers([[np.array(r.exterior.coords)[:-1] for r in L.region.geoms] +
                               [np.array(i.coords)[:-1] for poly in L.region.geoms for i in poly.interiors] for L in layers],
                              zs, SliceParams.from_dict({**params.to_dict(), "layer_mode": "planar"}))
        axis = seg.direction / np.linalg.norm(seg.direction)
        for L in layers:
            L.index = layer_index
            layer_index += 1
            for path in L.paths:
                p3 = np.column_stack([path.points, np.full(len(path.points), L.z)])
                path.xyz = p3 @ R  # rotate back (R^T applied to rows)
                path.axes = np.tile(axis, (len(p3), 1))
                path.tf = np.ones(len(p3))
        gcode, st = write_gcode(layers, SliceParams.from_dict({**params.to_dict(), "tool_axis": True, "layer_mode": "multi"}), f"{source} [{seg.name}]")
        gcode_parts.append(f";SEGMENT:{seg.name} direction {axis[0]:.4f} {axis[1]:.4f} {axis[2]:.4f}\n" + gcode.replace("M30\n", ""))
        all_layers.append(layers)
        if stats_total is None:
            stats_total = st
        else:
            stats_total.layers += st.layers
            stats_total.paths += st.paths
            stats_total.extrusion_length += st.extrusion_length
            stats_total.travel_length += st.travel_length
            stats_total.time_s += st.time_s
            stats_total.max_tilt = max(stats_total.max_tilt, st.max_tilt)
            for k, v in st.by_type.items():
                stats_total.by_type[k] = stats_total.by_type.get(k, 0.0) + v
        stats_total.dropped_chains += dropped
    if stats_total is None:
        raise ValueError("no segment produced any layer")
    stats_total.layer_mode = "multi"
    stats_total.max_tilt = max(float(np.degrees(np.arccos(np.clip(s.direction[2], -1, 1)))) for s in segments)
    return "\n".join(gcode_parts) + "M30\n", stats_total, all_layers
