"""Infill patterns: parallel lines (zig-zag or monotonic), connected zigzag, grid,
triangles, cubic, gyroid (wave approximation), honeycomb and concentric.

Every function returns a list of (k, 2) point arrays (open polylines unless noted).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import LineString, MultiLineString
from shapely.ops import linemerge, unary_union

SPARSE_PATTERNS = ("lines", "zigzag", "grid", "triangles", "cubic", "gyroid", "honeycomb", "concentric", "none")
SOLID_PATTERNS = ("lines", "zigzag", "monotonic", "concentric")


def _segments(clipped) -> list[np.ndarray]:
    segs: list[np.ndarray] = []
    if clipped is None or clipped.is_empty:
        return segs
    stack = [clipped]
    while stack:
        g = stack.pop()
        if g.geom_type == "LineString":
            if g.length > 1e-6:
                segs.append(np.array(g.coords))
        elif hasattr(g, "geoms"):
            stack.extend(g.geoms)
    return segs


def _order_family(segs: list[np.ndarray], d: np.ndarray, nrm: np.ndarray, spacing: float, monotonic: bool) -> list[np.ndarray]:
    """Group pieces by line, sort lines by offset; alternate the direction of every
    other line (zig-zag) unless ``monotonic``."""
    if not segs:
        return []
    groups: dict[int, list[np.ndarray]] = {}
    for s in segs:
        k = round(float(np.mean(s @ nrm)) / spacing)
        groups.setdefault(k, []).append(s)
    out = []
    flip = False
    for k in sorted(groups):
        pieces = sorted(groups[k], key=lambda s: float(min(s[0] @ d, s[-1] @ d)))
        pieces = [s if float((s[-1] - s[0]) @ d) >= 0 else s[::-1] for s in pieces]
        if flip:
            pieces = [s[::-1] for s in pieces[::-1]]
        out.extend(pieces)
        if not monotonic:
            flip = not flip
    return out


def line_fill(area, spacing: float, angle_deg: float, shift: float = 0.0, monotonic: bool = False) -> list[np.ndarray]:
    """Parallel lines at ``angle_deg`` with ``spacing`` clipped to ``area``.

    The family is anchored in world space (not to the bounding box) so lines of
    successive layers stack. ``shift`` moves the family along its normal (the
    cubic pattern uses it to build a 3-D structure)."""
    if area is None or area.is_empty or spacing <= 0:
        return []
    minx, miny, maxx, maxy = area.bounds
    a = math.radians(angle_deg)
    d = np.array([math.cos(a), math.sin(a)])
    nrm = np.array([-d[1], d[0]])
    corners = np.array([[minx, miny], [maxx, miny], [minx, maxy], [maxx, maxy]])
    offs = corners @ nrm
    along = corners @ d
    k0 = int(math.floor((offs.min() - shift) / spacing)) - 1
    k1 = int(math.ceil((offs.max() - shift) / spacing)) + 1
    t0, t1 = float(along.min()) - spacing, float(along.max()) + spacing
    lines = [LineString([nrm * (k * spacing + shift) + d * t0, nrm * (k * spacing + shift) + d * t1]) for k in range(k0, k1 + 1)]
    segs = _segments(area.intersection(MultiLineString(lines)))
    return _order_family(segs, d, nrm, spacing, monotonic)


def zigzag_fill(area, spacing: float, angle_deg: float, shift: float = 0.0) -> list[np.ndarray]:
    """Lines connected end-to-end along the boundary where the connector stays inside the area."""
    segs = line_fill(area, spacing, angle_deg, shift)
    if not segs:
        return []
    probe = area.buffer(0.05 * spacing)
    out: list[np.ndarray] = []
    cur = segs[0]
    for s in segs[1:]:
        gap = float(np.linalg.norm(s[0] - cur[-1]))
        if gap <= 2.5 * spacing and probe.covers(LineString([cur[-1], s[0]])):
            cur = np.vstack([cur, s])
        else:
            out.append(cur)
            cur = s
    out.append(cur)
    return out


def grid_fill(area, spacing: float, angle_deg: float) -> list[np.ndarray]:
    return line_fill(area, spacing * 2.0, angle_deg) + line_fill(area, spacing * 2.0, angle_deg + 90.0)


def triangle_fill(area, spacing: float, angle_deg: float) -> list[np.ndarray]:
    out = []
    for k in range(3):
        out += line_fill(area, spacing * 3.0, angle_deg + 60.0 * k)
    return out


def cubic_fill(area, spacing: float, angle_deg: float, z: float) -> list[np.ndarray]:
    """Three line families (60 deg apart) whose offsets move with Z, forming stacked
    cubes standing on a corner (what desktop slicers call "cubic")."""
    out = []
    period = spacing * 3.0
    for k in range(3):
        sign = 1.0 if k % 2 == 0 else -1.0
        out += line_fill(area, period, angle_deg + 60.0 * k, shift=sign * z)
    return out


def gyroid_fill(area, spacing: float, angle_deg: float, z: float) -> list[np.ndarray]:
    """Wave approximation of the gyroid: sine-shaped lines whose phase advances with
    Z and whose orientation turns by 90 deg every ``spacing`` of height, so
    successive layers interlock into a 3-D lattice."""
    if area is None or area.is_empty or spacing <= 0:
        return []
    period = 2.0 * spacing
    phase = 2.0 * math.pi * (z / period)
    ang = angle_deg + (90.0 if int(math.floor(z / spacing)) % 2 else 0.0)
    a = math.radians(ang)
    d = np.array([math.cos(a), math.sin(a)])
    nrm = np.array([-d[1], d[0]])
    minx, miny, maxx, maxy = area.bounds
    corners = np.array([[minx, miny], [maxx, miny], [minx, maxy], [maxx, maxy]])
    offs, along = corners @ nrm, corners @ d
    k0, k1 = int(math.floor(offs.min() / spacing)) - 1, int(math.ceil(offs.max() / spacing)) + 1
    step = max(spacing / 6.0, 0.3)
    t = np.arange(float(along.min()) - spacing, float(along.max()) + spacing + step, step)
    amp = 0.5 * spacing
    lines = []
    for k in range(k0, k1 + 1):
        off = k * spacing + amp * np.sin(2.0 * math.pi * t / period + phase + (math.pi if k % 2 else 0.0))
        lines.append(LineString(np.outer(t, d) + np.outer(off, nrm)))
    segs = _segments(area.intersection(MultiLineString(lines)))
    return _order_family(segs, d, nrm, spacing, False)


def honeycomb_fill(area, spacing: float, angle_deg: float) -> list[np.ndarray]:
    """Hexagonal cells; the side length is chosen so the line density equals
    parallel lines at ``spacing``. Shared cell edges are printed once."""
    if area is None or area.is_empty or spacing <= 0:
        return []
    s = 1.1547 * spacing  # 2/sqrt(3): same material per area as parallel lines
    a = math.radians(angle_deg)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    minx, miny, maxx, maxy = area.bounds
    r = math.hypot(maxx - minx, maxy - miny) / 2.0 + 2 * s
    cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
    w, h = 1.5 * s, math.sqrt(3) * s  # flat-top hexagons: column pitch, row pitch
    nx, ny = int(r / w) + 2, int(r / h) + 2
    base = np.array([[s * math.cos(math.radians(60 * i)), s * math.sin(math.radians(60 * i))] for i in range(6)])
    centre = np.array([cx, cy])
    hexes = []
    for i in range(-nx, nx + 1):
        for j in range(-ny, ny + 1):
            c = np.array([i * w, j * h + (h / 2.0 if i % 2 else 0.0)])
            pts = (base + c) @ rot.T + centre
            hexes.append(LineString(np.vstack([pts, pts[:1]])))
    lines = unary_union(MultiLineString(hexes))
    clipped = area.intersection(lines)
    if clipped.is_empty:
        return []
    merged = linemerge(clipped) if clipped.geom_type != "LineString" else clipped
    return _segments(merged)


def concentric_fill(area, spacing: float, rings_fn) -> list[np.ndarray]:
    out = []
    g = area
    while g is not None and not g.is_empty:
        out.extend(rings_fn(g))
        g = g.buffer(-spacing, join_style="mitre", mitre_limit=2.0)
    return out


def sparse_fill(area, pattern: str, spacing: float, angle_deg: float, layer_index: int, z: float, rings_fn) -> tuple[list[np.ndarray], bool]:
    """Dispatch on the pattern. Returns (polylines, closed)."""
    if pattern == "none" or area is None or area.is_empty:
        return [], False
    alt = angle_deg + (90.0 if layer_index % 2 else 0.0)
    if pattern == "concentric":
        return concentric_fill(area, spacing, rings_fn), True
    if pattern == "grid":
        return grid_fill(area, spacing, angle_deg), False
    if pattern == "triangles":
        return triangle_fill(area, spacing, angle_deg), False
    if pattern == "cubic":
        return cubic_fill(area, spacing, angle_deg, z), False
    if pattern == "gyroid":
        return gyroid_fill(area, spacing, angle_deg, z), False
    if pattern == "honeycomb":
        return honeycomb_fill(area, spacing, angle_deg), False
    if pattern == "zigzag":
        return zigzag_fill(area, spacing, alt), False
    return line_fill(area, spacing, alt), False


def solid_fill(area, pattern: str, width: float, angle_deg: float, rings_fn) -> tuple[list[np.ndarray], bool]:
    if area is None or area.is_empty:
        return [], False
    if pattern == "concentric":
        return concentric_fill(area, width, rings_fn), True
    if pattern == "zigzag":
        return zigzag_fill(area, width, angle_deg), False
    return line_fill(area, width, angle_deg, monotonic=(pattern == "monotonic")), False
