"""Overhang classification of wall segments and bridge detection.

Overhangs: every wall ring is densified and each vertex gets an "over-air"
degree 0..1 from its distance to the layer below. Speeds are interpolated
towards the overhang speed (like the overhang speed tiers of OrcaSlicer).

Bridges: an unsupported solid area whose straight fill lines land on supported
material at both ends is a bridge; it is printed with straight lines in the
anchoring direction at bridge speed / flow instead of as an overhang.
"""
from __future__ import annotations

import math

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPolygon

from .infill import line_fill

EMPTY = MultiPolygon()


def densify(points: np.ndarray, closed: bool, step: float) -> np.ndarray:
    """Insert vertices so that no segment is longer than ``step``."""
    pts = np.asarray(points, float)
    if len(pts) < 2 or step <= 0:
        return pts
    seq = np.vstack([pts, pts[:1]]) if closed else pts
    out = [seq[0]]
    for a, b in zip(seq[:-1], seq[1:]):
        d = float(np.linalg.norm(b - a))
        n = max(1, int(math.ceil(d / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    out = np.array(out)
    return out[:-1] if closed else out


def overhang_degree(points: np.ndarray, below, width: float) -> np.ndarray:
    """0 = fully on the layer below, 1 = the whole bead hangs in the air."""
    if below is None or below.is_empty:
        return np.ones(len(points))
    dist = shapely.distance(shapely.points(np.asarray(points, float)), below)
    return np.clip(dist / max(width, 1e-6), 0.0, 1.0)


def overhang_speeds(points: np.ndarray, below, width: float, speed: float, speed_overhang: float,
                    start: float = 0.25, full: float = 0.75) -> tuple[np.ndarray, float]:
    """Per-vertex speed (for the segment ending at the vertex) and the length over air."""
    deg = overhang_degree(points, below, width)
    t = np.clip((deg - start) / max(full - start, 1e-6), 0.0, 1.0)
    sp = speed + (speed_overhang - speed) * t
    # a segment is as slow as its slower end
    seg = np.minimum(sp, np.roll(sp, 1))
    seg[0] = sp[0]
    seglen = np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)
    over = float(seglen[np.maximum(deg, np.roll(deg, 1)) > start].sum())
    return seg, over


def bridge_direction(unsupported, below, width: float, step: float = 15.0) -> tuple[float, float]:
    """Best bridging angle and the fraction of fill lines anchored on both ends."""
    if unsupported is None or unsupported.is_empty or below is None or below.is_empty:
        return 0.0, 0.0
    anchor = below.buffer(0.6 * width)
    probe = unsupported.buffer(width)  # lines are clipped to the area grown by one bead so they reach the anchors
    best, best_frac = 0.0, 0.0
    for ang in np.arange(0.0, 180.0, step):
        segs = line_fill(probe, max(2.0 * width, 1.0), float(ang))
        if not segs:
            continue
        ok = sum(1 for s in segs if anchor.covers(shapely.points(s[0])) and anchor.covers(shapely.points(s[-1])))
        frac = ok / len(segs)
        if frac > best_frac + 1e-9:
            best, best_frac = float(ang), frac
    return best, best_frac


def split_bridges(solid_below, below, width: float, min_area: float = 4.0, min_anchor: float = 0.5) -> tuple[list[tuple], MultiPolygon]:
    """Split the bottom-facing solid area of a layer into bridges and the rest.

    Returns ([(polygon, angle), ...], remaining_area). ``solid_below`` is the solid
    area not covered by the layer below; ``below`` is the layer below."""
    if solid_below is None or solid_below.is_empty:
        return [], EMPTY
    if below is None or below.is_empty:
        return [], solid_below
    unsupported = solid_below.difference(below)
    bridges = []
    rest = solid_below
    polys = [unsupported] if unsupported.geom_type == "Polygon" else list(getattr(unsupported, "geoms", []))
    for poly in polys:
        if poly.geom_type != "Polygon" or poly.area < min_area:
            continue
        ang, frac = bridge_direction(poly, below, width)
        if frac >= min_anchor:
            # anchor the bridge lines one bead into the supported material on both sides
            region = poly.buffer(width).intersection(solid_below)
            bridges.append((region, ang))
            rest = rest.difference(region)
    rest = rest if rest.is_empty else rest.buffer(-0.1 * width).buffer(0.1 * width)
    return bridges, (EMPTY if rest.is_empty else rest)


def fuzzy_ring(points: np.ndarray, thickness: float, point_distance: float, rng) -> np.ndarray:
    """Resample a closed ring every ``point_distance`` and jitter each point along the
    local normal by up to +-thickness/2 (fuzzy skin)."""
    pts = densify(np.asarray(points, float), True, max(point_distance, 0.1))
    n = len(pts)
    if n < 3 or thickness <= 0:
        return pts
    t = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    ln = np.linalg.norm(t, axis=1)
    ln[ln < 1e-9] = 1.0
    t /= ln[:, None]
    nrm = np.column_stack([t[:, 1], -t[:, 0]])
    amp = np.array([rng.uniform(-0.5, 0.5) for _ in range(n)]) * thickness
    return pts + nrm * amp[:, None]
