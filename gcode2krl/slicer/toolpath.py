"""Layer tool paths: walls, gap fill, top/bottom skins, bridges, sparse infill,
support, raft, skirt/brim, overhang slow-down, spiral vase, ordering.

Polygon work uses Shapely. All coordinates in mm (slicing space).
"""
from __future__ import annotations

import dataclasses
import math
import random
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import MultiPolygon, Polygon
from shapely.validation import make_valid

from . import infill as _infill
from .features import densify, fuzzy_ring, overhang_speeds, split_bridges
from .params import SliceParams
from .support import overhang_areas, support_areas

EMPTY = MultiPolygon()

WALL_KINDS = ("WALL-OUTER", "WALL-INNER")
MODEL_ORDER = (("WALL-INNER", "WALL-OUTER"), ("GAP",), ("BRIDGE",), ("SKIN",), ("IRONING",), ("FILL",))
ISLAND_KINDS = ("WALL-OUTER", "WALL-INNER", "GAP", "BRIDGE", "SKIN", "IRONING", "FILL")


@dataclass
class Path:
    kind: str  # WALL-OUTER | WALL-INNER | GAP | SKIN | BRIDGE | FILL | SUPPORT | SUPPORT-INTERFACE | SKIRT | BRIM | RAFT
    points: np.ndarray  # (k, 2) in slicing space
    closed: bool
    width: float
    speed: float
    xyz: np.ndarray | None = None  # (k, 3) model-space points (non-planar modes / spiral vase)
    axes: np.ndarray | None = None  # (k, 3) tool axes (away from the part)
    tf: np.ndarray | None = None  # (k,) real thickness / slicing layer height
    speeds: np.ndarray | None = None  # (k,) feed of the segment ending at vertex i (overhang slow-down)
    flow: float = 1.0  # extrusion factor (bridges, combined infill)
    island: int = 0
    spiral: bool = False  # spiral-vase ring: Z ramps along the path

    @property
    def length(self) -> float:
        p = self.xyz if self.xyz is not None else self.points
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
    kind: str = "model"  # model | raft
    support: MultiPolygon = EMPTY
    sparse: MultiPolygon = EMPTY  # sparse-infill area (for infill combining)
    overhang_area: float = 0.0


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


def compensate(region: MultiPolygon, xy: float, hole: float) -> MultiPolygon:
    """XY size compensation (grow contours by ``xy``) and hole compensation (grow holes by ``hole``)."""
    if abs(xy) < 1e-9 and abs(hole) < 1e-9:
        return region
    out = []
    for poly in as_multi(region).geoms:
        ext = Polygon(poly.exterior).buffer(xy, join_style="mitre", mitre_limit=2.0)
        for ring in poly.interiors:
            ext = ext.difference(Polygon(ring).buffer(hole, join_style="mitre", mitre_limit=2.0))
        out.append(ext)
    g = out[0]
    for q in out[1:]:
        g = g.union(q)
    return as_multi(g)


def intersect_all(regs: list) -> MultiPolygon:
    if not regs:
        return EMPTY
    g = regs[0]
    for r in regs[1:]:
        g = g.intersection(r)
        if g.is_empty:
            return EMPTY
    return as_multi(g)


# ------------------------------------------------------------------- infill
line_fill = _infill.line_fill  # re-exported for tests / callers


def concentric_fill(area, spacing: float) -> list[np.ndarray]:
    return _infill.concentric_fill(area, spacing, rings)


# ----------------------------------------------------------------- ordering
def start_index(points: np.ndarray, from_pt: np.ndarray, seam: str, rng: random.Random) -> int:
    if seam == "random":
        return rng.randrange(len(points))
    if seam == "aligned":
        return int(np.argmax(points[:, 0] + points[:, 1]))
    if seam == "rear":
        return int(np.argmax(points[:, 1] * 1000.0 + points[:, 0]))
    if seam == "corner":
        a = np.roll(points, 1, axis=0) - points
        b = np.roll(points, -1, axis=0) - points
        la, lb = np.linalg.norm(a, axis=1), np.linalg.norm(b, axis=1)
        ok = (la > 1e-9) & (lb > 1e-9)
        cosang = np.ones(len(points))
        cosang[ok] = np.einsum("ij,ij->i", a[ok], b[ok]) / (la[ok] * lb[ok])
        return int(np.argmax(cosang))  # smallest interior angle = sharpest corner
    return int(np.argmin(np.linalg.norm(points - from_pt, axis=1)))


def nearest_start(points: np.ndarray, closed: bool, from_pt: np.ndarray, seam: str, rng: random.Random) -> np.ndarray:
    if not closed or len(points) < 3:
        return points
    return np.roll(points, -start_index(points, from_pt, seam, rng), axis=0)


def _roll_path(p: Path, k: int) -> Path:
    """Rotate a closed path so that it starts at vertex k (speeds follow)."""
    if k == 0:
        return p
    sp = p.speeds
    if sp is not None:
        # speeds[i] belongs to the segment ending at vertex i; after rolling, the segment
        # ending at the new vertex 0 is the old segment ending at old vertex k
        sp = np.roll(sp, -k)
    return dataclasses.replace(p, points=np.roll(p.points, -k, axis=0), speeds=sp)


def _reverse_path(p: Path) -> Path:
    sp = p.speeds
    if sp is not None:
        # segment i (ending at vertex i) becomes the segment ending at vertex n-i for the reversed order
        sp = np.roll(sp[::-1], 1) if p.closed else np.roll(sp[::-1], -1)
    return dataclasses.replace(p, points=p.points[::-1].copy(), speeds=sp)


def order_paths(paths: list[Path], start: np.ndarray, seam: str, rng: random.Random) -> list[Path]:
    """Greedy nearest-neighbour ordering; closed loops are rotated to the seam, open
    lines are reversed when their far end is closer."""
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
        if bflip:
            p = _reverse_path(p)
        if p.closed and len(p.points) >= 3:
            p = _roll_path(p, start_index(p.points, cur, seam, rng))
        out.append(p)
        cur = p.points[0] if p.closed else p.points[-1]
    return out


def order_layer(paths: list[Path], cur: np.ndarray, params: SliceParams, rng: random.Random) -> tuple[list[Path], np.ndarray]:
    """Adhesion first, then island by island (walls, gap, bridges, skin, fill), then support."""
    ordered: list[Path] = []

    def block(ps):
        nonlocal cur
        if ps:
            ordered.extend(order_paths(ps, cur, params.seam, rng))
            last = ordered[-1]
            cur = last.points[0] if last.closed else last.points[-1]

    block([p for p in paths if p.kind == "PRIME"])
    block([p for p in paths if p.kind in ("BRIM", "SKIRT")])
    block([p for p in paths if p.kind == "RAFT"])
    islands = sorted({p.island for p in paths if p.kind in ISLAND_KINDS})
    # visit islands nearest-first
    rep = {}
    for isl in islands:
        pts = np.concatenate([p.points for p in paths if p.island == isl and p.kind in ISLAND_KINDS])
        rep[isl] = pts
    remaining = list(islands)
    while remaining:
        isl = min(remaining, key=lambda k: float(np.min(np.linalg.norm(rep[k] - cur, axis=1))))
        remaining.remove(isl)
        if params.wall_order == "outer_inner":
            groups = (("WALL-OUTER",), ("WALL-INNER",)) + MODEL_ORDER[1:]
        elif params.wall_order == "infill_first":
            groups = (("FILL",), ("SKIN",), ("BRIDGE",), ("GAP",), ("WALL-INNER", "WALL-OUTER"), ("IRONING",))
        else:
            groups = MODEL_ORDER
        for kinds in groups:
            block([p for p in paths if p.island == isl and p.kind in kinds])
    block([p for p in paths if p.kind == "SUPPORT-INTERFACE"])
    block([p for p in paths if p.kind == "SUPPORT"])
    return ordered, cur


# ------------------------------------------------------------------- layers
def _wall_paths(island, i_model: int, L: Layer, prev_region, params: SliceParams, first_on_plate: bool, isl: int, rng=None) -> tuple[list[Path], list]:
    """Perimeter rings of one island. Returns (paths, coverage polygons of the beads)."""
    w_o, w_i = params.width("WALL-OUTER"), params.width("WALL-INNER")
    n_walls = 1 if params.spiral_vase and i_model >= params.bottom_layers else params.perimeters
    paths, cover = [], []
    for k in range(n_walls):
        w = w_o if k == 0 else w_i
        centre = w_o / 2.0 if k == 0 else w_o + (k - 0.5) * w_i
        ring_region = offset(island, -centre)
        if ring_region.is_empty:
            break
        cover.append(ring_region.buffer(w / 2.0 + 0.01))
        kind = "WALL-OUTER" if k == 0 else "WALL-INNER"
        base_speed = params.speed_first_layer if first_on_plate else (params.speed_outer if k == 0 else params.speed_perimeter)
        for r in rings(ring_region):
            length = float(np.linalg.norm(np.diff(np.vstack([r, r[:1]]), axis=0), axis=1).sum())
            speed = base_speed
            if params.small_perimeter_length > 0 and length < params.small_perimeter_length and not first_on_plate:
                speed = min(speed, params.small_perimeter_speed)
            pts = r
            speeds = None
            if params.fuzzy_skin != "none" and not first_on_plate and (k == 0 or params.fuzzy_skin == "all") and rng is not None:
                pts = fuzzy_ring(r, params.fuzzy_thickness, params.fuzzy_point_distance, rng)
            if params.overhang_detect and not first_on_plate:
                pts = densify(pts, True, w)
                speeds, _ = overhang_speeds(pts, prev_region, w, speed, params.speed_overhang)
            paths.append(Path(kind, pts, True, w, speed, speeds=speeds, island=isl,
                              spiral=params.spiral_vase and i_model >= params.bottom_layers))
    return paths, cover


def _gap_paths(island, cover: list, fill_area, params: SliceParams, speed: float, isl: int) -> list[Path]:
    """Single beads in the space the walls and the infill do not cover (gaps between
    walls, thin walls, tips)."""
    w = params.bead_width
    covered = fill_area.buffer(params.width("FILL") / 2.0) if not fill_area.is_empty else EMPTY
    for c in cover:
        covered = covered.union(c)
    gaps = island.difference(covered) if not covered.is_empty else island
    min_w = max(params.gap_fill_min_width * w, 0.05)
    gaps = as_multi(gaps.buffer(-min_w / 2.0).buffer(min_w / 2.0))
    out = []
    for g in gaps.geoms:
        if g.area < 0.5 * w * w:
            continue
        est_w = min(w, max(min_w, 2.0 * g.area / max(g.length, 1e-6)))
        rect = g.minimum_rotated_rectangle
        rc = np.array(rect.exterior.coords)[:4]
        e0, e1 = rc[1] - rc[0], rc[2] - rc[1]
        axis = e0 if np.linalg.norm(e0) >= np.linalg.norm(e1) else e1
        ang = math.degrees(math.atan2(axis[1], axis[0]))
        a = math.radians(ang)
        nrm = np.array([-math.sin(a), math.cos(a)])
        c = np.array(g.centroid.coords[0])
        for seg in _infill.line_fill(g, max(est_w, 0.5 * w), ang, shift=float(c @ nrm)):
            seg = _shorten(seg, est_w / 2.0)
            if seg is not None:
                out.append(Path("GAP", seg, False, est_w, speed, island=isl))
    return out


def _shorten(seg: np.ndarray, d: float) -> np.ndarray | None:
    """Trim ``d`` from both ends of a polyline (so the bead ends inside the area)."""
    if len(seg) < 2:
        return None
    lens = np.linalg.norm(np.diff(seg, axis=0), axis=1)
    total = float(lens.sum())
    if total <= 2.0 * d + 1e-6:
        return None
    pts = [seg[0]]
    cum = np.concatenate([[0.0], np.cumsum(lens)])
    out = []
    for a, b in ((d, total - d),):
        ia = int(np.searchsorted(cum, a, side="right") - 1)
        ib = int(np.searchsorted(cum, b, side="right") - 1)
        pa = seg[ia] + (seg[ia + 1] - seg[ia]) * ((a - cum[ia]) / max(lens[ia], 1e-9))
        pb = seg[ib] + (seg[min(ib + 1, len(seg) - 1)] - seg[ib]) * ((b - cum[ib]) / max(lens[min(ib, len(lens) - 1)], 1e-9)) if ib < len(seg) - 1 else seg[-1]
        out = [pa] + [seg[k] for k in range(ia + 1, ib + 1)] + [pb]
    return np.array(out)


def build_layers(layer_loops: list[list[np.ndarray]], zs: list[float], params: SliceParams,
                 heights: list[float] | None = None, ranges: list | None = None) -> list[Layer]:
    from .ranges import params_for_z
    from .support import apply_support_modifiers, parse_support_modifiers

    p = params
    n = len(zs)
    ranges = ranges or []
    cache: dict = {}

    def layer_params(z: float) -> SliceParams:
        if not ranges:
            return p
        key = round(z, 6)
        if key not in cache:
            cache[key] = params_for_z(p, ranges, z)
        return cache[key]
    if heights is None:
        heights = [p.first_layer_height if i == 0 else p.layer_height for i in range(n)]
    regions = [compensate(loops_to_region(loops, p.min_feature), p.xy_compensation, p.hole_compensation) for loops in layer_loops]
    if p.elephant_foot > 0 and regions and not regions[0].is_empty:
        regions[0] = offset(regions[0], -p.elephant_foot)
    # ---- support (planar only)
    sup_print, sup_iface = ([EMPTY] * n, [EMPTY] * n)
    if p.support in ("normal", "tree") and p.planar and not p.spiral_vase:
        if p.support == "tree":
            from .tree_support import tree_support_areas
            sup_print, sup_iface = tree_support_areas(regions, heights, p)
        else:
            sup_print, sup_iface = support_areas(regions, heights, p)
        mods = parse_support_modifiers(p.support_modifiers)
        if mods:
            sup_print, sup_iface = list(sup_print), list(sup_iface)
            apply_support_modifiers(sup_print, sup_iface, regions, list(zs), mods, p.support_xy_gap)
    oh = overhang_areas(regions, heights, p.overhang_max, p.bead_width, 0.0)
    # ---- raft
    n_raft = max(0, p.raft_layers) if p.planar and not p.spiral_vase else 0
    raft_heights = [p.first_layer_height if k == 0 else p.layer_height for k in range(n_raft)]
    raft_h = sum(raft_heights)
    layers: list[Layer] = []
    if n_raft:
        base = regions[0] if not regions[0].is_empty else EMPTY
        for s in sup_print[:1]:
            base = as_multi(base.union(s))
        raft_region = offset(base, p.raft_margin)
        zr = 0.0
        for k in range(n_raft):
            zr += raft_heights[k]
            layers.append(Layer(k, round(zr, 6), raft_heights[k], raft_region, kind="raft"))
    for i in range(n):
        L = Layer(n_raft + i, round(zs[i] + raft_h, 6), heights[i], regions[i], kind="model", support=sup_print[i])
        L.overhang_area = float(oh[i].area) if not oh[i].is_empty else 0.0
        layers.append(L)
    model = layers[n_raft:]
    rng = random.Random(1)
    cur = np.array([0.0, 0.0])
    w = p.bead_width
    w_f, w_s = p.width("FILL"), p.width("SKIN")
    # ---- raft paths
    for k in range(n_raft):
        L = layers[k]
        paths = []
        if k == 0:
            paths += _adhesion(L.region, p, w)
        if k == 0:
            segs, spacing = _infill.line_fill(L.region, 2.5 * w, 0.0), 2.5 * w
        elif k == n_raft - 1:
            segs = _infill.line_fill(L.region, w, 90.0 if k % 2 else 0.0)
        else:
            segs = _infill.line_fill(L.region, 1.5 * w, 90.0 if k % 2 else 0.0)
        for s in segs:
            paths.append(Path("RAFT", s, False, w, p.speed_first_layer if k == 0 else p.speed_infill, flow=1.0))
        L.paths, cur = order_layer(paths, cur, p, rng)
    # ---- model layers
    for i, L in enumerate(model):
        lp = layer_params(L.z)
        w = lp.bead_width
        w_f, w_s = lp.width("FILL"), lp.width("SKIN")
        rng_layer = random.Random(1000 + i)
        paths: list[Path] = []
        first = i == 0
        first_on_plate = first and n_raft == 0
        prev_region = model[i - 1].region if i > 0 else (layers[n_raft - 1].region if n_raft else None)
        # what carries this layer for the overhang slow-down: the part below plus support (under the Z gap)
        carrier = prev_region
        if i > 0:
            for j in (i - 1, i - 1 - lp.support_z_gap):
                if 0 <= j < n and not sup_print[j].is_empty:
                    carrier = as_multi((carrier if carrier is not None else EMPTY).union(sup_print[j]))
        if n_raft == 0 and i < max(1, lp.skirt_layers):
            base = L.region
            if not L.support.is_empty:
                base = as_multi(base.union(L.support))
            paths += _adhesion(base, lp, w, i)
        if L.region.is_empty and L.support.is_empty:
            L.paths, cur = order_layer(paths, cur, lp, rng)
            continue
        speed_fill = lp.speed_first_layer if first_on_plate else lp.speed_infill
        speed_top = lp.speed_first_layer if first_on_plate else lp.speed_top
        skin_angle = lp.infill_angle + (90.0 if i % 2 else 0.0)
        spiral_layer = lp.spiral_vase and i >= lp.bottom_layers
        sparse_all = EMPTY
        for isl, island in enumerate(L.region.geoms):
            island = as_multi(island)
            walls, cover = _wall_paths(island, i, L, carrier, lp, first_on_plate, isl, rng_layer)
            paths += walls
            if spiral_layer:
                continue
            n_walls = lp.perimeters
            inset = (lp.width("WALL-OUTER") + max(0, n_walls - 1) * lp.width("WALL-INNER")) if n_walls > 0 else 0.0
            fill_area = offset(island, -inset + lp.infill_overlap * w_f) if n_walls > 0 else island
            fill_area = as_multi(fill_area.intersection(island)) if not fill_area.is_empty else EMPTY
            if lp.gap_fill and n_walls > 0:
                paths += _gap_paths(island, cover, fill_area, lp, lp.speed_first_layer if first_on_plate else lp.speed_gap, isl)
            if fill_area.is_empty:
                continue
            # ---- top / bottom solid detection
            top_solid, bottom_solid = EMPTY, EMPTY
            if lp.bottom_layers and i < lp.bottom_layers:
                bottom_solid = fill_area
            if lp.top_layers and i >= n - lp.top_layers:
                top_solid = fill_area
            if lp.top_layers and top_solid.is_empty:
                above = intersect_all([model[j].region for j in range(i + 1, min(n, i + 1 + lp.top_layers))])
                unc = fill_area.difference(above.buffer(-0.01)) if not above.is_empty else fill_area
                top_solid = as_multi(unc.buffer(w).buffer(-w).intersection(fill_area)) if not unc.is_empty else EMPTY
            if lp.bottom_layers and bottom_solid.is_empty and i > 0:
                below = intersect_all([model[j].region for j in range(max(0, i - lp.bottom_layers), i)])
                unc = fill_area.difference(below.buffer(-0.01)) if not below.is_empty else fill_area
                bottom_solid = as_multi(unc.buffer(w).buffer(-w).intersection(fill_area)) if not unc.is_empty else EMPTY
            solid_total = as_multi(top_solid.union(bottom_solid)) if not (top_solid.is_empty and bottom_solid.is_empty) else EMPTY
            sparse_area = as_multi(fill_area.difference(solid_total)) if not solid_total.is_empty else fill_area
            # small sparse islands become solid
            if lp.solid_threshold_area > 0 and not sparse_area.is_empty:
                small = [g for g in sparse_area.geoms if g.area < lp.solid_threshold_area]
                if small:
                    extra = MultiPolygon(small)
                    top_solid = as_multi(top_solid.union(extra))
                    sparse_area = as_multi(sparse_area.difference(extra))
            # ---- bridges and bottom skins
            if not bottom_solid.is_empty:
                bridges, rest = ([], bottom_solid)
                if lp.bridges and i > 0 and prev_region is not None and not prev_region.is_empty:
                    bridges, rest = split_bridges(bottom_solid, prev_region, w_s)
                for region, ang in bridges:
                    for seg in _infill.line_fill(region, w_s, ang):
                        paths.append(Path("BRIDGE", seg, False, w_s, lp.speed_bridge, flow=lp.bridge_flow, island=isl))
                segs, closed = _infill.solid_fill(rest, lp.bottom_pattern, w_s, skin_angle, rings)
                for seg in segs:
                    speeds = None
                    pts = seg
                    if lp.overhang_detect and not first_on_plate and carrier is not None:
                        pts = densify(seg, closed, w_s)
                        speeds, _ = overhang_speeds(pts, carrier, w_s, speed_top, lp.speed_overhang)
                    paths.append(Path("SKIN", pts, closed, w_s, speed_top, speeds=speeds, island=isl))
                top_solid = as_multi(top_solid.difference(bottom_solid)) if not top_solid.is_empty else EMPTY
            if not top_solid.is_empty:
                segs, closed = _infill.solid_fill(top_solid, lp.top_pattern, w_s, skin_angle, rings)
                for seg in segs:
                    paths.append(Path("SKIN", seg, closed, w_s, speed_top, island=isl))
                if lp.ironing and (i >= n - lp.top_layers or lp.top_layers == 0):
                    iron_area = top_solid.buffer(-w_s / 2.0)
                    for seg in _infill.line_fill(iron_area, max(lp.ironing_spacing, 0.2), skin_angle + 90.0, monotonic=True):
                        paths.append(Path("IRONING", seg, False, w_s, lp.ironing_speed, flow=lp.ironing_flow, island=isl))
            # ---- sparse infill
            if lp.infill_density > 0 and lp.infill_pattern != "none" and not sparse_area.is_empty:
                sparse_all = as_multi(sparse_all.union(sparse_area))
                nc = max(1, lp.infill_combine)
                area = sparse_area
                flow = 1.0
                if nc > 1:
                    if (i + 1) % nc != 0 and i != n - 1:
                        area = EMPTY
                    else:
                        back = [model[j].sparse for j in range(max(0, i - nc + 1), i)]
                        area = intersect_all([sparse_area] + back) if back else sparse_area
                        flow = float(len(back) + 1)
                if not area.is_empty:
                    spacing = w_f / max(lp.infill_density, 1e-3)
                    segs, closed = _infill.sparse_fill(area, lp.infill_pattern, spacing, lp.infill_angle, i, L.z, rings)
                    for seg in segs:
                        paths.append(Path("FILL", seg, closed, w_f, speed_fill, flow=flow, island=isl))
        L.sparse = sparse_all
        # ---- support
        if not L.support.is_empty:
            iface = sup_iface[i]
            body = as_multi(L.support.difference(iface)) if not iface.is_empty else L.support
            sp = lp.speed_first_layer if first_on_plate else lp.speed_support
            if not iface.is_empty:
                for seg in _infill.line_fill(iface, w / max(lp.support_interface_density, 0.05), lp.support_angle + 90.0):
                    paths.append(Path("SUPPORT-INTERFACE", seg, False, w, sp))
            if not body.is_empty:
                spacing = w / max(lp.support_density, 0.02)
                if lp.support_pattern == "walls":
                    for r in rings(offset(body, -w / 2.0)):
                        paths.append(Path("SUPPORT", r, True, w, sp))
                    body_in = offset(body, -w)
                    segs = _infill.line_fill(body_in, spacing, lp.support_angle) if not body_in.is_empty else []
                else:
                    segs = _infill.line_fill(body, spacing, lp.support_angle)
                    if lp.support_pattern == "grid":
                        segs += _infill.line_fill(body, spacing, lp.support_angle + 90.0)
                for seg in segs:
                    paths.append(Path("SUPPORT", seg, False, w, sp))
        L.paths, cur = order_layer(paths, cur, lp, rng)
        if spiral_layer:
            _spiralize(L, cur)
    return layers


def _adhesion(base, p: SliceParams, w: float, layer_index: int = 0) -> list[Path]:
    paths = []
    if base is None or base.is_empty:
        return paths
    if p.brim_width > 0 and layer_index == 0:
        k = 0
        while (k + 0.5) * w <= p.brim_width:
            for r in rings(offset(base, (k + 0.5) * w)):
                paths.append(Path("BRIM", r, True, w, p.speed_first_layer))
            k += 1
    if p.skirt_loops > 0 and layer_index < max(1, p.skirt_layers):
        for k in range(p.skirt_loops):
            for r in rings(offset(base, p.brim_width + p.skirt_distance + (k + 0.5) * w)):
                paths.append(Path("SKIRT", r, True, w, p.speed_first_layer))
    if p.prime_line_length > 0 and layer_index == 0:
        minx, miny, maxx, maxy = base.bounds
        extra = p.brim_width + (p.skirt_distance + p.skirt_loops * w if p.skirt_loops > 0 else 0.0)
        y = miny - extra - p.prime_line_offset
        cx = (minx + maxx) / 2.0
        pts = np.array([[cx - p.prime_line_length / 2.0, y], [cx + p.prime_line_length / 2.0, y]])
        paths.append(Path("PRIME", pts, False, w, p.speed_first_layer))
    return paths


def _spiralize(L: Layer, cur: np.ndarray) -> None:
    """Turn the outer wall of a spiral-vase layer into an open path whose Z ramps from
    the previous layer to this one (continuous helix, no layer change seam)."""
    out = []
    for path in L.paths:
        if path.kind != "WALL-OUTER" or not path.closed:
            out.append(path)
            continue
        pts = np.vstack([path.points, path.points[:1]])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = s[-1] if s[-1] > 0 else 1.0
        z = (L.z - L.height) + L.height * (s / total)
        sp = None
        if path.speeds is not None:
            sp = np.concatenate([path.speeds, path.speeds[:1]])
        out.append(dataclasses.replace(path, points=pts, closed=False, xyz=np.column_stack([pts, z]), speeds=sp, spiral=True))
    L.paths = out


def support_report(layers: list[Layer], bead_width: float, overhang_max_deg: float) -> dict:
    """Area (mm^2) of each model layer not supported by the layer below within the
    printable overhang angle, and the number of such layers (support indicator)."""
    model = [L for L in layers if L.kind == "model"]
    total = sum(L.overhang_area for L in model)
    n_layers = sum(1 for L in model if L.overhang_area > 0.25 * bead_width * bead_width)
    worst = max([L.overhang_area for L in model], default=0.0)
    return {"support_area": total, "support_layers": n_layers, "worst_layer_area": worst}
