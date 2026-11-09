"""Tree support: branches grow down from the overhang islands, merge into trunks,
keep an XY distance from the part and may rest on it.

Layer-based (like the tree supports of Cura / the organic supports of
PrusaSlicer, simplified):

1. tips: points on a hexagonal grid inside every overhang island (automatic
   detection: `overhang_areas`), starting ``support_z_gap`` layers below the
   overhang;
2. descent: layer by layer each node moves down; its radius grows with the
   height already descended (up to the trunk diameter), it may drift
   sideways by at most ``h * tan(branch_angle)`` per layer - towards the
   nearest neighbour (to merge) and away from the part (XY gap);
3. nodes whose discs overlap merge (area preserving); a node that cannot escape
   the part rests on it (``support_on_model``) or is clipped by the part;
4. the union of the discs per layer is the support area; the first layers
   under each tip form the interface.
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import MultiPolygon, Point
from shapely.ops import nearest_points, unary_union

from .params import SliceParams
from .support import EMPTY, _multi, overhang_areas


class Node:
    __slots__ = ("x", "y", "r", "age", "descended", "tip_r")

    def __init__(self, x: float, y: float, r: float):
        self.x, self.y, self.r = float(x), float(y), float(r)
        self.age = 0  # layers below the tip
        self.descended = 0.0  # mm of height below the tip
        self.tip_r = float(r)


def hex_grid_points(poly, spacing: float) -> list[tuple[float, float]]:
    """Points of a hexagonal grid inside ``poly`` (at least one point per island)."""
    pts: list[tuple[float, float]] = []
    polys = [poly] if poly.geom_type == "Polygon" else list(getattr(poly, "geoms", []))
    for g in polys:
        if g.is_empty:
            continue
        minx, miny, maxx, maxy = g.bounds
        dy = spacing * math.sqrt(3) / 2.0
        row = 0
        y = miny + spacing / 2.0
        found = False
        while y <= maxy:
            x0 = minx + (spacing / 2.0 if row % 2 else spacing)
            x = x0
            while x <= maxx:
                if g.contains(Point(x, y)):
                    pts.append((x, y))
                    found = True
                x += spacing
            y += dy
            row += 1
        if not found:
            c = g.representative_point()
            pts.append((float(c.x), float(c.y)))
    return pts


def tree_support_areas(regions: list, heights: list[float], params: SliceParams) -> tuple[list, list]:
    """Support and interface areas per layer (same contract as support_areas)."""
    n = len(regions)
    w = params.bead_width
    gap = params.support_xy_gap
    zg = max(0, params.support_z_gap)
    ni = max(0, params.support_interface_layers)
    tip_r = (params.tree_tip_diameter if params.tree_tip_diameter > 0 else 2.0 * w) / 2.0
    trunk_r = max(tip_r, params.tree_trunk_diameter / 2.0)
    growth = math.tan(math.radians(max(0.0, params.tree_diameter_growth)))  # radius per mm of height
    tan_branch = math.tan(math.radians(max(0.0, min(80.0, params.tree_branch_angle))))
    spacing = params.tree_tip_spacing if params.tree_tip_spacing > 0 else 2.0 * tip_r * 1.8
    oh = overhang_areas(regions, heights, params.overhang_max, w, params.support_min_area)
    # tips: nodes that start at layer (overhang layer - zg - 1)
    starts: dict[int, list[Node]] = {}
    for i in range(1, n):
        if oh[i].is_empty:
            continue
        j = i - 1 - zg
        if j < 0:
            continue
        for x, y in hex_grid_points(oh[i].buffer(-0.25 * tip_r), spacing):
            starts.setdefault(j, []).append(Node(x, y, tip_r))
    printed: list = [EMPTY] * n
    interface: list = [EMPTY] * n
    active: list[Node] = []
    for i in range(n - 1, -1, -1):
        active.extend(starts.get(i, []))
        if not active:
            continue
        avoid = regions[i].buffer(gap) if not regions[i].is_empty else None
        step = heights[i] * tan_branch
        # --- lateral motion: towards the nearest neighbour, away from the part
        survivors: list[Node] = []
        for nd in active:
            # merge target
            best, bd = None, None
            for other in active:
                if other is nd:
                    continue
                d = math.hypot(other.x - nd.x, other.y - nd.y)
                if d < 6.0 * trunk_r and (bd is None or d < bd):
                    best, bd = other, d
            if best is not None and bd > 1e-9:
                mv = min(step, bd / 2.0)
                nd.x += (best.x - nd.x) / bd * mv
                nd.y += (best.y - nd.y) / bd * mv
            if avoid is not None and not avoid.is_empty:
                disc = Point(nd.x, nd.y)
                depth = avoid.distance(disc)  # 0 when inside
                inside = avoid.contains(disc)
                if inside or depth < nd.r:
                    p_av, p_nd = nearest_points(avoid.exterior if avoid.geom_type == "Polygon" else avoid.boundary, disc)
                    dx, dy = nd.x - p_av.x, nd.y - p_av.y
                    dist = math.hypot(dx, dy)
                    if inside:
                        need = dist + nd.r
                        if dist > 1e-9:
                            dx, dy = -dx / dist, -dy / dist  # point is inside: move towards the boundary and beyond
                        else:
                            dx, dy = 1.0, 0.0
                    else:
                        need = nd.r - dist
                        dx, dy = dx / dist, dy / dist
                    mv = min(step, need)
                    nd.x += dx * mv
                    nd.y += dy * mv
                    if mv < need - 1e-9:
                        # cannot get clear of the part on this layer: rest on it or be clipped
                        if params.support_on_model and avoid.contains(Point(nd.x, nd.y)):
                            continue  # branch ends here (it stands on the part)
            survivors.append(nd)
        active = survivors
        # --- merging (area preserving)
        merged: list[Node] = []
        for nd in sorted(active, key=lambda q: -q.r):
            hit = None
            for m in merged:
                if math.hypot(m.x - nd.x, m.y - nd.y) < m.r + nd.r:
                    hit = m
                    break
            if hit is None:
                merged.append(nd)
            else:
                a1, a2 = hit.r ** 2, nd.r ** 2
                hit.x = (hit.x * a1 + nd.x * a2) / (a1 + a2)
                hit.y = (hit.y * a1 + nd.y * a2) / (a1 + a2)
                hit.r = min(trunk_r, math.sqrt(a1 + a2))
                hit.age = max(hit.age, nd.age)
                hit.descended = max(hit.descended, nd.descended)
        active = merged
        # --- areas of this layer
        discs = [Point(nd.x, nd.y).buffer(nd.r, quad_segs=12) for nd in active]
        area = unary_union(discs)
        if avoid is not None and not avoid.is_empty:
            area = area.difference(avoid)
        printed[i] = _multi(area)
        tips = [Point(nd.x, nd.y).buffer(nd.r, quad_segs=12) for nd in active if nd.age < ni]
        if tips:
            iface = unary_union(tips)
            if avoid is not None and not avoid.is_empty:
                iface = iface.difference(avoid)
            interface[i] = _multi(iface)
        # --- grow for the next layer down
        for nd in active:
            nd.age += 1
            nd.descended += heights[i]
            nd.r = min(trunk_r, nd.tip_r + growth * nd.descended)
    return printed, interface
