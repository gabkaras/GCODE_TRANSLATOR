"""Plater (placement, auto-orientation, several models), height ranges, robot program
hooks, fuzzy skin, ironing, prime line, skirt layers, seam at corners, support
modifiers, feature preview."""
import numpy as np
import pytest

pytest.importorskip("shapely")

from gcode2krl.gcode_parser import KrlRaw, parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.slicer import SliceParams, slice_triangles
from gcode2krl.slicer.pipeline import preview_segments, slice_models
from gcode2krl.slicer.plater import Placement, auto_orient, overhang_area, rotation_to_xyz, rotation_xyz, transform_triangles
from gcode2krl.slicer.ranges import hooks_for_layer, layer_heights_with_ranges, params_for_z, parse_hooks, parse_ranges
from gcode2krl.slicer.support import parse_support_modifiers
from gcode2krl.translator import EventItem, TranslateOptions, translate


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
    return SliceParams.from_dict(dict(layer_height=2, first_layer_height=2, bead_width=4, **kw))


def kinds(layers):
    return {q.kind for L in layers for q in L.paths}


# ---------------------------------------------------------------- plater
def test_placement_transform_and_rotation_roundtrip():
    t = box((40, 20, 10), (100, 50, 30))
    out = transform_triangles(t, Placement(rot=(0, 0, 90), scale=(2, 2, 2), offset=(10, -5, 0)))
    v = out.reshape(-1, 3)
    assert v[:, 2].min() == pytest.approx(0.0) and v[:, 2].max() == pytest.approx(20.0)
    assert (v[:, 0].min() + v[:, 0].max()) / 2 == pytest.approx(10.0) and (v[:, 1].min() + v[:, 1].max()) / 2 == pytest.approx(-5.0)
    assert v[:, 0].max() - v[:, 0].min() == pytest.approx(40.0) and v[:, 1].max() - v[:, 1].min() == pytest.approx(80.0)  # 20x40 scaled 2, turned 90
    for r in ((10, 20, 30), (0, 90, 0), (-45, 0, 120)):
        assert np.allclose(rotation_xyz(*rotation_to_xyz(rotation_xyz(*r))), rotation_xyz(*r), atol=1e-9)
    mir = transform_triangles(t, Placement(mirror=(True, False, False)))
    n = np.cross(mir[:, 1] - mir[:, 0], mir[:, 2] - mir[:, 0])
    assert (n[:, 2] > 0).sum() == 2 and (n[:, 2] < 0).sum() == 2  # winding restored: two up, two down facets


def test_auto_orient_reduces_overhang():
    T = np.concatenate([box((10, 10, 20), (0, 0, 10)), box((60, 20, 6), (0, 0, 23))])
    tilted = transform_triangles(T, Placement(rot=(90, 0, 0)))
    res = auto_orient(tilted, "min_overhang", 45)
    assert res["overhang_after"] < res["overhang_before"]
    fixed = transform_triangles(tilted, Placement(rot=tuple(res["rot"])))
    assert overhang_area(fixed, 45) == pytest.approx(res["overhang_after"])
    flat = auto_orient(tilted, "lay_flat", 45)
    assert flat["contact_area"] >= 1200 - 1e-6
    _, st, _ = slice_triangles(tilted, P(auto_orient="min_overhang", support="normal"), "ao")
    assert st.support_volume < 1000  # upside-down T: no support column needed


def test_slice_models_all_and_sequential():
    A, B = box((30, 30, 10), (0, 0, 5)), box((20, 20, 16), (0, 0, 8))
    models = [(A, Placement(name="a", offset=(-30, 0, 0))), (B, Placement(name="b", offset=(30, 0, 0), rot=(0, 0, 45)))]
    g1, s1, l1 = slice_models(models, P(print_sequence="all"), "m")
    g2, s2, l2 = slice_models(models, P(print_sequence="sequential"), "m")
    assert s1.layers == 8 and s2.layers == 13 and g2.count("M30") == 1
    assert np.allclose(s1.bounds["min"], s2.bounds["min"], atol=0.5) and np.allclose(s1.bounds["max"], s2.bounds["max"], atol=0.5)
    assert s2.e_total == pytest.approx(s1.e_total, rel=0.05)
    res = parse_gcode(g2)
    assert res.counts["G1"] > 100  # concatenated program parses


# --------------------------------------------------------- ranges / hooks
def test_ranges_override_per_height():
    rs = parse_ranges("0 8 layer_height=1 speed_outer=5 infill_density=0.5\n8 20 infill_pattern=gyroid")
    p = P()
    assert params_for_z(p, rs, 4).layer_height == 1 and params_for_z(p, rs, 4).speed_outer == 5
    assert params_for_z(p, rs, 12).infill_pattern == "gyroid" and params_for_z(p, rs, 12).layer_height == 2
    zs, hs = layer_heights_with_ranges(20, p, rs)
    assert hs[0] == 2 and hs[1] == 1 and 2 in hs[-3:] and abs(zs[-1] - 20) < 2
    with pytest.raises(ValueError):
        parse_ranges("0 10 nonsense=1")
    g, st, layers = slice_triangles(box((40, 40, 20), (0, 0, 10)), P(ranges="0 8 layer_height=1 speed_outer=5"), "r")
    low = [L for L in layers if L.z <= 8]
    assert all(L.height == 1 for L in low[1:]) and all(q.speed == 5 for L in low[1:] for q in L.paths if q.kind == "WALL-OUTER")
    assert all(L.height == 2 for L in layers if L.z > 9)


def test_layer_hooks_reach_the_krl_program():
    hooks = parse_hooks("layer 2: WAIT SEC 3\nz 15: HALT\nevery 4: $OUT[12] = TRUE")
    assert hooks_for_layer(hooks, 2, 6.0, 4.0) == ["WAIT SEC 3"] and hooks_for_layer(hooks, 8, 16.0, 14.0) == ["HALT", "$OUT[12] = TRUE"]
    assert hooks_for_layer(hooks, 9, 18.0, 16.0) == []
    with pytest.raises(ValueError):
        parse_hooks("sometimes: HALT")
    g, st, layers = slice_triangles(box((40, 40, 20), (0, 0, 10)), P(layer_krl="layer 2: WAIT SEC 3\nz 15: HALT", layer_wait=2), "h")
    krl = [l for l in g.splitlines() if l.startswith(";KRL:")]
    assert krl.count(";KRL:WAIT SEC 3") == 1 and krl.count(";KRL:HALT") == 1 and krl.count(";KRL:WAIT SEC 2") == len(layers) - 1
    res = parse_gcode(g)
    assert sum(isinstance(c, KrlRaw) for c in res.commands) == len(krl)
    items, stt = translate(res, TranslateOptions(extruder=True))
    assert sum(1 for i in items if isinstance(i, EventItem) and i.kind == "krl") == len(krl)
    src = write_krl(items, KrlOptions(name="H", extruder_out=7), stt).files["H.src"]
    assert src.count("WAIT SEC 3") == 1 and "HALT" in src and src.count("WAIT SEC 2") == len(layers) - 1
    # the hook sits between the extruder-off of the previous layer and the next spline block
    i = src.index("WAIT SEC 3")
    assert src.rfind("ENDSPLINE", 0, i) > src.rfind("SPLINE WITH", 0, i)


# ---------------------------------------------------------------- walls
def test_fuzzy_skin_ironing_prime_skirt_layers_and_corner_seam():
    g, st, layers = slice_triangles(box((40, 40, 12), (0, 0, 6)), P(fuzzy_skin="outer", fuzzy_thickness=1.0, fuzzy_point_distance=2.0, ironing=True,
                                                                     prime_line_length=50, skirt_loops=1, skirt_layers=3, seam="corner", infill_density=0.1), "w")
    k = kinds(layers)
    assert {"PRIME", "SKIRT", "IRONING", "WALL-OUTER"} <= k
    assert sum(1 for L in layers if any(q.kind == "SKIRT" for q in L.paths)) == 3
    assert sum(1 for L in layers if any(q.kind == "PRIME" for q in L.paths)) == 1
    outer = [q for q in layers[2].paths if q.kind == "WALL-OUTER"][0]
    d = np.abs(np.abs(outer.points).max(axis=1) - 18.0)  # the nominal outer wall is at |x| or |y| = 18
    assert d.max() > 0.3 and d.max() <= 0.5 + 1e-6  # jitter up to half the thickness
    assert outer.points[0] in outer.points and layers[2].paths[0].kind != "PRIME"
    plain = [q for q in slice_triangles(box((40, 40, 12), (0, 0, 6)), P(seam="corner", infill_density=0), "c")[2][1].paths if q.kind == "WALL-OUTER"][0]
    assert abs(abs(plain.points[0][0]) - 18.0) < 1e-3 and abs(abs(plain.points[0][1]) - 18.0) < 1e-3  # starts at a corner
    iron = [q for L in layers for q in L.paths if q.kind == "IRONING"]
    assert iron and all(q.flow == 0.15 and q.speed == 15 for q in iron)
    g2, st2, l2 = slice_triangles(box((40, 40, 12), (0, 0, 6)), P(wall_order="infill_first", top_layers=0, bottom_layers=0), "o")
    kinds_seq = [q.kind for q in l2[1].paths]
    assert kinds_seq.index("FILL") < kinds_seq.index("WALL-OUTER")


def test_support_modifiers():
    mods = parse_support_modifiers("block 0 0 10 100 100 20\nforce 40 0 5 10 10 10")
    assert mods[0]["kind"] == "block" and mods[1]["center"] == [40, 0, 5]
    T = np.concatenate([box((10, 10, 20), (0, 0, 10)), box((60, 20, 6), (0, 0, 23))])
    _, st_a, _ = slice_triangles(T, P(support="normal"), "a")
    _, st_b, _ = slice_triangles(T, P(support="normal", support_modifiers="block 20 0 10 30 40 24"), "b")
    _, st_c, _ = slice_triangles(T, P(support="normal", support_modifiers="force 0 20 5 10 10 10"), "c")
    assert st_b.support_volume < st_a.support_volume * 0.7 and st_c.support_volume > st_a.support_volume


def test_preview_segments():
    g, st, layers = slice_triangles(box((20, 20, 4), (0, 0, 2)), P(infill_density=0.2), "p")
    pv = preview_segments(layers)
    assert set(pv) >= {"WALL-OUTER", "SKIN"} and all(v.dtype == np.float32 and len(v) % 6 == 0 for v in pv.values())
