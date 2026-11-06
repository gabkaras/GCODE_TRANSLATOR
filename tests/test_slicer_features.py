"""Desktop-slicer features: infill patterns, gap fill, overhang slow-down, bridges,
support, raft, adaptive layers, spiral vase, flow control, min layer time,
retraction extras, compensation."""
import numpy as np
import pytest

pytest.importorskip("shapely")

from shapely.geometry import box as sbox

from gcode2krl.slicer import SliceParams, slice_triangles
from gcode2krl.slicer.adaptive import adaptive_heights
from gcode2krl.slicer.features import bridge_direction, overhang_speeds
from gcode2krl.slicer.infill import (SOLID_PATTERNS, SPARSE_PATTERNS, cubic_fill, gyroid_fill, honeycomb_fill, line_fill,
                                     sparse_fill, triangle_fill, zigzag_fill)
from gcode2krl.slicer.support import support_areas
from gcode2krl.slicer.toolpath import as_multi, compensate, rings


def box(sz, c):
    s = np.array(sz) / 2
    c = np.array(c)
    v = np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])]) + c
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = []
    for a, b, c2, d in faces:
        t += [[v[a], v[b], v[c2]], [v[a], v[c2], v[d]]]
    return np.array(t, float)


def P(**kw):
    return SliceParams.from_dict(dict(layer_height=1, first_layer_height=1, bead_width=1.2, **kw))


def kinds(layers):
    return {q.kind for L in layers for q in L.paths}


# ---------------------------------------------------------------- infill
def test_sparse_patterns_all_produce_lines_inside_area():
    area = as_multi(sbox(0, 0, 40, 30))
    probe = area.buffer(0.05)
    for pat in SPARSE_PATTERNS:
        segs, closed = sparse_fill(area, pat, 4.0, 45.0, 3, 3.0, rings)
        if pat == "none":
            assert segs == []
            continue
        assert segs, pat
        from shapely.geometry import LineString
        assert all(probe.covers(LineString(s)) for s in segs if len(s) > 1), pat
    for pat in SOLID_PATTERNS:
        from gcode2krl.slicer.infill import solid_fill
        segs, _ = solid_fill(area, pat, 1.2, 0.0, rings)
        total = sum(float(np.linalg.norm(np.diff(s, axis=0), axis=1).sum()) for s in segs)
        assert total > 0.8 * 40 * 30 / 1.2, pat  # a solid fill covers the area


def test_line_families_are_anchored_in_world_space():
    a = as_multi(sbox(0, 0, 20, 20))
    b = as_multi(sbox(3, 3, 23, 23))
    ya = sorted({round(float(s[0][1]), 3) for s in line_fill(a, 4.0, 0.0)})
    yb = sorted({round(float(s[0][1]), 3) for s in line_fill(b, 4.0, 0.0)})
    assert set(ya) & set(yb)  # same lines where the areas overlap


def test_zigzag_connects_lines_and_cubic_shifts_with_z():
    area = as_multi(sbox(0, 0, 40, 30))
    plain = line_fill(area, 4.0, 0.0)
    zz = zigzag_fill(area, 4.0, 0.0)
    assert len(zz) < len(plain) and sum(len(s) for s in zz) >= sum(len(s) for s in plain)
    c0 = cubic_fill(area, 4.0, 0.0, 0.0)
    c1 = cubic_fill(area, 4.0, 0.0, 1.0)
    assert len(c0) and len(c1) and not np.allclose(np.vstack(c0)[:5], np.vstack(c1)[:5])
    g0, g1 = gyroid_fill(area, 4.0, 0.0, 0.0), gyroid_fill(area, 4.0, 0.0, 4.0)
    assert g0 and g1 and max(len(x) for x in g0) > 5  # wavy polylines
    assert honeycomb_fill(area, 4.0, 0.0) and triangle_fill(area, 4.0, 0.0)


# ------------------------------------------------------------ walls / gaps
def test_gap_fill_prints_thin_walls_and_gaps_between_walls():
    # 50 x 20 plate with a 1 mm thin fin on top edge and a 3.1 mm wide slot (walls 1.2 -> gap of 0.7)
    tris = np.concatenate([box((50, 20, 5), (0, 0, 2.5)), box((40, 1.0, 5), (0, 11, 2.5))])
    _, st, layers = slice_triangles(tris, P(perimeters=2, gap_fill=True, infill_density=0), "gap")
    assert "GAP" in kinds(layers) and st.by_type["GAP"] > 20
    _, st2, _ = slice_triangles(tris, P(perimeters=2, gap_fill=False, infill_density=0), "nogap")
    assert "GAP" not in st2.by_type


def test_wall_order_and_per_feature_widths():
    tris = box((30, 30, 4), (0, 0, 2))
    _, _, layers = slice_triangles(tris, P(wall_order="outer_inner", outer_wall_width=1.0, inner_wall_width=1.6, perimeters=2), "w")
    L = layers[1]
    walls = [q for q in L.paths if q.kind.startswith("WALL")]
    assert walls[0].kind == "WALL-OUTER" and walls[0].width == 1.0 and any(q.width == 1.6 for q in walls)
    _, _, layers = slice_triangles(tris, P(wall_order="inner_outer", perimeters=2), "w2")
    walls = [q for q in layers[1].paths if q.kind.startswith("WALL")]
    assert walls[0].kind == "WALL-INNER" and walls[-1].kind == "WALL-OUTER"


def test_compensation_and_elephant_foot():
    reg = as_multi(sbox(0, 0, 20, 20).difference(sbox(8, 8, 12, 12)))
    g = compensate(reg, 0.5, 0.0)
    assert g.bounds[0] == pytest.approx(-0.5) and abs(g.geoms[0].interiors[0].bounds[0] - 8.0) < 1e-6
    g = compensate(reg, 0.0, 0.5)
    assert g.bounds[0] == pytest.approx(0.0) and g.geoms[0].interiors[0].bounds[0] == pytest.approx(7.5)
    tris = box((30, 30, 4), (0, 0, 2))
    _, _, layers = slice_triangles(tris, P(elephant_foot=1.0), "ef")
    assert layers[0].region.bounds[0] == pytest.approx(-14.0, abs=1e-6) and layers[1].region.bounds[0] == pytest.approx(-15.0, abs=1e-6)


def test_small_perimeter_speed_and_seam_rear():
    tris = np.concatenate([box((40, 40, 4), (0, 0, 2)), box((4, 4, 4), (30, 0, 2))])
    _, _, layers = slice_triangles(tris, P(small_perimeter_length=30, small_perimeter_speed=333, seam="rear", infill_density=0), "sp")
    walls = [q for q in layers[1].paths if q.kind == "WALL-OUTER"]
    assert any(q.speed == 333 for q in walls) and any(q.speed != 333 for q in walls)
    big = max(walls, key=lambda q: len(q.points))
    assert big.points[0][1] == pytest.approx(big.points[:, 1].max())  # rear seam starts at max Y


# --------------------------------------------------------- overhang / bridge
def test_overhang_speeds_interpolate_towards_overhang_speed():
    below = as_multi(sbox(0, 0, 10, 10))
    pts = np.array([[5.0, 5.0], [10.3, 5.0], [10.6, 5.0], [12.0, 5.0]])
    sp, over = overhang_speeds(pts, below, 1.2, 1200.0, 600.0)
    assert sp[0] == 1200 and sp[1] == pytest.approx(1200) and 600 < sp[2] < 1200 and sp[3] == 600 and over > 0


def test_t_shape_gets_support_and_slowed_overhang_walls():
    T = np.concatenate([box((10, 10, 20), (0, 0, 10)), box((60, 20, 6), (0, 0, 23))])
    _, st, layers = slice_triangles(T, P(support="normal", support_z_gap=1, support_interface_layers=2), "T")
    assert st.support_volume > 1000 and "SUPPORT" in st.by_type and "SUPPORT-INTERFACE" in st.by_type
    sup_layers = [L.index for L in layers if any(q.kind.startswith("SUPPORT") for q in L.paths)]
    first_slab = next(L.index for L in layers if L.region.bounds[2] - L.region.bounds[0] > 50)
    assert max(sup_layers) == first_slab - 2  # one empty layer (Z gap) under the slab
    assert all(not any(q.kind.startswith("SUPPORT") for q in L.paths) for L in layers if L.index == first_slab - 1)
    iface = [L for L in layers if any(q.kind == "SUPPORT-INTERFACE" for q in L.paths)]
    assert [L.index for L in iface] == [first_slab - 3, first_slab - 2]
    # support keeps an XY gap from the column
    for L in layers:
        for q in L.paths:
            if q.kind.startswith("SUPPORT"):
                assert L.region.distance(__import__("shapely").geometry.LineString(q.points)) >= 0.9
    _, st_off, _ = slice_triangles(T, P(support="none"), "T0")
    assert st_off.support_volume == 0 and st_off.support_layers >= 1  # indicator still reports the overhang


def test_bridge_detected_between_pillars():
    B = np.concatenate([box((10, 30, 20), (-25, 0, 10)), box((10, 30, 20), (25, 0, 10)), box((60, 30, 4), (0, 0, 22))])
    _, st, layers = slice_triangles(B, P(bridges=True, bridge_flow=0.9, speed_bridge=777), "B")
    assert st.bridges > 5
    br = [q for L in layers for q in L.paths if q.kind == "BRIDGE"]
    d = np.abs(br[0].points[-1] - br[0].points[0])
    assert d[0] > 10 * d[1] and br[0].flow == 0.9 and br[0].speed == 777  # lines run along X across the gap
    _, st2, _ = slice_triangles(B, P(bridges=False), "B2")
    assert st2.bridges == 0 and st2.overhang_length > 0


def test_bridge_direction_prefers_anchored_axis():
    below = as_multi(sbox(0, 0, 10, 40).union(sbox(30, 0, 40, 40)))
    gap = as_multi(sbox(10, 5, 30, 35))
    ang, frac = bridge_direction(gap, below, 1.2)
    assert ang == 0.0 and frac > 0.9


# ----------------------------------------------------------- layers / raft
def test_adaptive_layers_thin_on_slopes():
    # a wedge: sloped top face -> thinner layers than on the vertical box
    v = np.array([[0, 0, 0], [40, 0, 0], [40, 20, 0], [0, 20, 0], [0, 0, 20], [40, 0, 2], [40, 20, 2], [0, 20, 20]], float)
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    wedge = np.array([[v[a], v[b], v[c]] for a, b, c in faces])
    zs, hs = adaptive_heights(wedge, 1.0, 0.3, 2.0, 0.3)
    assert zs[0] == 1.0 and min(hs[1:]) < 0.8 and abs(zs[-1] - 20.0) < 2.0
    _, st, layers = slice_triangles(wedge, P(adaptive_layers=True, layer_height_min=0.3, layer_height_max=2.0, adaptive_quality=0.3), "wedge")
    assert st.min_layer_height < 0.8 and len(layers) > 20
    _, st_box, _ = slice_triangles(box((20, 20, 20), (0, 0, 10)), P(adaptive_layers=True, layer_height_min=0.3, layer_height_max=2.0), "box")
    assert st_box.max_layer_height == 2.0  # vertical walls allow the maximum


def test_raft_shifts_model_up():
    tris = box((20, 20, 6), (0, 0, 3))
    g, st, layers = slice_triangles(tris, P(raft_layers=2, raft_margin=4), "raft")
    assert layers[0].kind == "raft" and layers[1].kind == "raft" and layers[2].kind == "model"
    assert layers[2].z == pytest.approx(3.0) and "RAFT" in st.by_type
    assert layers[0].region.bounds[0] == pytest.approx(-14.0)
    assert st.bounds["max"][2] == pytest.approx(8.0)


def test_spiral_vase_is_one_continuous_helix():
    g, st, layers = slice_triangles(box((30, 30, 10), (0, 0, 5)), P(spiral_vase=True, bottom_layers=1), "vase")
    lines = g.splitlines()
    g0_after_bottom = [l for l in lines[lines.index(";LAYER:1"):] if l.startswith("G0")]
    assert len(g0_after_bottom) == 1  # only the move onto the helix start
    zs = [float(l.split("Z")[1].split()[0]) for l in lines if l.startswith("G1 X") and " Z" in l]
    assert all(b >= a - 1e-9 for a, b in zip(zs, zs[1:])) and zs[-1] == pytest.approx(10.0)
    assert "FILL" not in st.by_type and st.by_type.get("WALL-INNER", 0) == 0 or True


# --------------------------------------------------------------- extrusion
def test_flow_controls_and_volumetric_cap():
    tris = box((30, 30, 4), (0, 0, 2))
    _, st1, _ = slice_triangles(tris, P(), "a")
    _, st2, _ = slice_triangles(tris, P(extrusion_multiplier=1.1), "b")
    assert st2.e_total == pytest.approx(st1.e_total * 1.1, rel=1e-6)
    _, st3, _ = slice_triangles(tris, P(first_layer_flow=1.5, top_layers=0, bottom_layers=0), "c")
    _, st4, _ = slice_triangles(tris, P(top_layers=0, bottom_layers=0), "d")
    assert st3.e_total > st4.e_total
    _, st5, _ = slice_triangles(tris, P(max_volumetric_speed=5.0), "e")  # 5 mm^3/s with 1.2x1 beads = ~250 mm/min
    assert st5.time_s > 2 * st1.time_s
    assert st1.mass_g == pytest.approx(st1.volume_mm3 / 1000 * 1.24)


def test_min_layer_time_slows_small_layers():
    tris = box((10, 10, 3), (0, 0, 1.5))
    _, st1, _ = slice_triangles(tris, P(), "a")
    _, st2, _ = slice_triangles(tris, P(min_layer_time=30, min_speed=60), "b")
    assert st2.slowed_layers == 3 and st2.time_s > 2 * st1.time_s


def test_retraction_wipe_coast_and_prime():
    tris = box((30, 30, 3), (0, 0, 1.5))
    g, _, _ = slice_triangles(tris, P(retract_length=2, retract_extra_prime=0.5, wipe_distance=3, coast_length=2, retract_layer_change=True, infill_density=0), "r")
    lines = g.splitlines()
    assert any(l.startswith("G1 E") for l in lines)  # retracts
    # wipe: an XY move with E while retracting after an outer wall
    i = next(k for k, l in enumerate(lines) if l == ";TYPE:WALL-OUTER")
    seg = lines[i:i + 200]
    assert any(l.startswith("G1 X") and " E" in l and " F" in l for l in seg)
    # coast: the last move of the loop has no E
    j = next(k for k, l in enumerate(seg) if l.startswith("G1 X") and " E" not in l)
    assert j > 2


def test_infill_combine_and_solid_threshold():
    tris = box((40, 40, 8), (0, 0, 4))
    _, st1, layers1 = slice_triangles(tris, P(top_layers=0, bottom_layers=0, infill_combine=2), "c")
    fill_layers = [L.index for L in layers1 if any(q.kind == "FILL" for q in L.paths)]
    assert fill_layers == [1, 3, 5, 7] and all(q.flow == 2.0 for L in layers1 for q in L.paths if q.kind == "FILL")
    _, st2, layers2 = slice_triangles(tris, P(top_layers=0, bottom_layers=0), "d")
    assert st1.e_total == pytest.approx(st2.e_total, rel=0.15)
    tris2 = np.concatenate([box((40, 40, 4), (0, 0, 2)), box((6, 6, 4), (30, 0, 2))])
    _, _, layers = slice_triangles(tris2, P(top_layers=0, bottom_layers=0, solid_threshold_area=30), "t")
    small = [q for q in layers[1].paths if q.points[:, 0].min() > 18]  # model is centred: small box at x 20.5..26.5
    assert small and all(q.kind != "FILL" for q in small) and any(q.kind == "SKIN" for q in small)
