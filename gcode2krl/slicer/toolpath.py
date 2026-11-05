"""Layer tool paths: perimeters, solid and sparse infill, skirt/brim, ordering.

Polygon work uses Shapely. All coordinates in mm.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

from .params import SliceParams

EMPTY = MultiPolygon()


@dataclass
class Path:
    kind: str  # WALL-OUTER | WALL-INNER | SKIN | FILL | SKIRT | BRIM
    points: np.ndarray  # (k, 2)
    closed: bool
    width: float
    speed: float

    @property
    def length(self) -> float:
        p = self.points
        if len(p) < 2:
            return 0.0
        d = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        if self.closed:
            d += float(np.linalg.norm(p[0] - p[-1]))
        return d


@dataclass
class Layer:
    index: int
    z: float
    height: float
    region: MultiPolygon
    paths: list[Path] = field(default_factory=list)


# ------------------------------------------------------------------ polygons
def loops_to_region(loops: list[np.ndarray], min_feature: float) -> MultiPolygon:
    """Build a valid (multi)polygon from closed loops using even-odd nesting."""
    polys = []
    for lp in loops:
        if len(lp) < 3:
            continue
        p = Polygon(lp)
        if not p.is_valid:
            p = make_valid(p)
        if p.is_empty or p.area < 1e-9:
            continue
        polys.append(p)
    if not polys:
        return EMPTY
    # even-odd fill: XOR of all loop polygons = inside an odd number of loops
    result = polys[0]
    for q in polys[1:]:
        result = result.symmetric_difference(q)
    if result is None or result.is_empty:
        return EMPTY
    if min_feature > 0:
        result = result.buffer(-min_feature / 2.0).buffer(min_feature / 2.0)
    return as_multi(result)


def as_multi(g) -> MultiPolygon:
    if g is None or g.is_empty:
        return EMPTY
    if g.geom_type == "Polygon":
        return MultiPolygon([g])
    if g.geom_type == "MultiPolygon":
        return g
    polys = [x for x in getattr(g, "geoms", []) if x.geom_type == "Polygon" and x.area > 1e-9]
    return MultiPolygon(polys) if polys else EMPTY


def rings(g) -> list[np.ndarray]:
    """Exterior and interior rings of a (multi)polygon as point arrays (without the repeated last point)."""
    out = []
    for poly in as_multi(g).geoms:
        for ring in [poly.exterior, *poly.interiors]:
            pts = np.array(ring.coords)[:-1]
            if len(pts) >= 3:
                out.append(pts)
    return out


def offset(g, d: float) -> MultiPolygon:
    if g is None or g.is_empty:
        return EMPTY
    return as_multi(g.buffer(d, join_style="mitre", mitre_limit=2.0))


# ------------------------------------------------------------------- infill
def line_fill(area, spacing: float, angle_deg: float) -> list[np.ndarray]:
    """Parallel lines at ``angle_deg`` with ``spacing`` clipped to ``area``, zig-zag ordered."""
    if area is None or area.is_empty or spacing <= 0:
        return []
    minx, miny, maxx, maxy = area.bounds
    cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
    diag = math.hypot(maxx - minx, maxy - miny) + spacing
    a = math.radians(angle_deg)
    d = np.array([math.cos(a), math.sin(a)])
    nrm = np.array([-d[1], d[0]])
    n_lines = int(diag / spacing) + 1
    lines = []
    for k in range(-n_lines // 2 - 1, n_lines // 2 + 2):
        o = np.array([cx, cy]) + nrm * (k * spacing)
        lines.append(LineString([o - d * diag, o + d * diag]))
    clipped = area.intersection(MultiLineString(lines))
    segs = []
    geoms = list(getattr(clipped, "geoms", [clipped])) if not clipped.is_empty else []
    for g in geoms:
        if g.geom_type == "LineString" and g.length > 1e-6:
            segs.append(np.array(g.coords))
        elif g.geom_type == "MultiLineString":
            segs.extend(np.array(x.coords) for x in g.geoms if x.length > 1e-6)
    if not segs:
        return []
    # order: by line index (projection on the normal), alternate direction along d
    keyed = sorted(segs, key=lambda s: (round(float((s[0] - [cx, cy]) @ nrm) / spacing), float(s[0] @ d)))
    out = []
    flip = False
    last_k = None
    for s in keyed:
        k = round(float((s[0] - [cx, cy]) @ nrm) / spacing)
        if k != last_k:
            flip = not flip
            last_k = k
        out.append(s[::-1] if flip else s)
    return out


def concentric_fill(area, spacing: float) -> list[np.ndarray]:
    out = []
    g = area
    while g is not None and not g.is_empty:
        out.extend(rings(g))
        g = offset(g, -spacing)
    return out


# ----------------------------------------------------------------- ordering
def nearest_start(points: np.ndarray, closed: bool, from_pt: np.ndarray, seam: str, rng: random.Random) -> np.ndarray:
    if not closed or len(points) < 3:
        return points
    if seam == "random":
        k = rng.randrange(len(points))
    elif seam == "aligned":
        k = int(np.argmax(points[:, 0] + points[:, 1]))
    else:
        k = int(np.argmin(np.linalg.norm(points - from_pt, axis=1)))
    return np.roll(points, -k, axis=0)


def order_paths(paths: list[Path], start: np.ndarray, seam: str, rng: random.Random) -> list[Path]:
    """Greedy nearest-neighbour ordering, respecting the kind priority already in the list order groups."""
    out = []
    cur = start.copy()
    remaining = list(paths)
    while remaining:
        best, bi, bflip = None, -1, False
        for i, p in enumerate(remaining):
            if p.closed:
                d = float(np.min(np.linalg.norm(p.points - cur, axis=1)))
                flip = False
            else:
                d0 = float(np.linalg.norm(p.points[0] - cur))
                d1 = float(np.linalg.norm(p.points[-1] - cur))
                d, flip = (d0, False) if d0 <= d1 else (d1, True)
            if best is None or d < best:
                best, bi, bflip = d, i, flip
        p = remaining.pop(bi)
        pts = p.points[::-1] if bflip else p.points
        pts = nearest_start(pts, p.closed, cur, seam, rng)
        out.append(Path(p.kind, pts, p.closed, p.width, p.speed))
        cur = pts[0] if p.closed else pts[-1]
    return out


# ------------------------------------------------------------------- layers
def build_layers(layer_loops: list[list[np.ndarray]], zs: list[float], params: SliceParams) -> list[Layer]:
    regions = [loops_to_region(loops, params.min_feature) for loops in layer_loops]
    layers = []
    for i, (z, reg) in enumerate(zip(zs, regions)):
        h = params.first_layer_height if i == 0 else params.layer_height
        layers.append(Layer(i, z, h, reg))
    w = params.bead_width
    n = len(layers)
    rng = random.Random(1)
    cur = np.array([0.0, 0.0])
    for i, L in enumerate(layers):
        if L.region.is_empty:
            continue
        paths: list[Path] = []
        first = i == 0
        # ---- skirt / brim on the first layer
        if first and params.brim_width > 0:
            k = 0
            while (k + 0.5) * w <= params.brim_width:
                for r in rings(offset(L.region, (k + 0.5) * w)):
                    paths.append(Path("BRIM", r, True, w, params.speed_first_layer))
                k += 1
        if first and params.skirt_loops > 0:
            for k in range(params.skirt_loops):
                for r in rings(offset(L.region, params.skirt_distance + (k + 0.5) * w)):
                    paths.append(Path("SKIRT", r, True, w, params.speed_first_layer))
        # ---- perimeters, innermost first
        walls = []
        inner_region = L.region
        for k in range(params.perimeters):
            ring_region = offset(L.region, -(k + 0.5) * w)
            if ring_region.is_empty:
                break
            kind = "WALL-OUTER" if k == 0 else "WALL-INNER"
            speed = params.speed_first_layer if first else (params.speed_outer if k == 0 else params.speed_perimeter)
            for r in rings(ring_region):
                walls.append(Path(kind, r, True, w, speed))
            inner_region = offset(L.region, -(k + 1) * w)
        walls.reverse()  # inner walls first, outer wall last
        paths.extend(walls)
        # ---- infill area (overlapping the innermost wall a little)
        fill_area = offset(L.region, -(params.perimeters * w) + params.infill_overlap * w) if params.perimeters > 0 else L.region
        if fill_area.is_empty:
            L.paths = order_paths(paths, cur, params.seam, rng)
            if L.paths:
                cur = L.paths[-1].points[0] if L.paths[-1].closed else L.paths[-1].points[-1]
            continue
        # ---- solid (top/bottom) detection
        solid = None
        if i < params.bottom_layers or i >= n - params.top_layers:
            solid = fill_area
        else:
            above = [layers[j].region for j in range(i + 1, min(n, i + 1 + params.top_layers))]
            below = [layers[j].region for j in range(max(0, i - params.bottom_layers), i)]
            cover_above = above[0]
            for g in above[1:]:
                cover_above = cover_above.intersection(g)
            cover_below = below[0]
            for g in below[1:]:
                cover_below = cover_below.intersection(g)
            uncovered = fill_area.difference(cover_above.buffer(-0.01)).union(fill_area.difference(cover_below.buffer(-0.01)))
            uncovered = as_multi(uncovered.buffer(w).buffer(-w).intersection(fill_area))
            solid = uncovered if not uncovered.is_empty else None
        sparse_area = fill_area if solid is None else as_multi(fill_area.difference(solid))
        speed_fill = params.speed_first_layer if first else params.speed_infill
        if solid is not None and not solid.is_empty:
            ang = params.infill_angle + (90.0 if i % 2 else 0.0)
            for seg in line_fill(solid, w, ang):
                paths.append(Path("SKIN", seg, False, w, speed_fill))
        if params.infill_density > 0 and params.infill_pattern != "none" and not sparse_area.is_empty:
            spacing = w / max(params.infill_density, 1e-3)
            if params.infill_pattern == "concentric":
                for r in concentric_fill(sparse_area, spacing):
                    paths.append(Path("FILL", r, True, w, speed_fill))
            elif params.infill_pattern == "grid":
                for seg in line_fill(sparse_area, spacing * 2.0, params.infill_angle) + line_fill(sparse_area, spacing * 2.0, params.infill_angle + 90.0):
                    paths.append(Path("FILL", seg, False, w, speed_fill))
            else:
                ang = params.infill_angle + (90.0 if i % 2 else 0.0)
                for seg in line_fill(sparse_area, spacing, ang):
                    paths.append(Path("FILL", seg, False, w, speed_fill))
        # ---- ordering: walls block, then skin, then fill (nearest neighbour inside each block)
        ordered: list[Path] = []
        for kinds in (("BRIM", "SKIRT"), ("WALL-INNER", "WALL-OUTER"), ("SKIN",), ("FILL",)):
            block = [p for p in paths if p.kind in kinds]
            if block:
                ordered.extend(order_paths(block, cur, params.seam, rng))
                last = ordered[-1]
                cur = last.points[0] if last.closed else last.points[-1]
        L.paths = ordered
    return layers
