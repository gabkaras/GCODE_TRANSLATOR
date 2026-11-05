import numpy as np
import pytest

pytest.importorskip("shapely")

from gcode2krl.gcode_parser import LinearMove, parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.slicer import SliceParams, slice_triangles
from gcode2krl.slicer.mesh_slice import layer_heights, link_segments, plane_segments, slice_layers
from gcode2krl.slicer.toolpath import line_fill, loops_to_region
from gcode2krl.translator import TranslateOptions, translate


def box(sz, c):
    s = np.array(sz) / 2
    c = np.array(c)
    v = np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])]) + c
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = []
    for a, b, c2, d in faces:
        t += [[v[a], v[b], v[c2]], [v[a], v[c2], v[d]]]
    return np.array(t, float)


def test_plane_segments_and_linking():
    tris = box((40, 30, 20), (0, 0, 10))
    segs = plane_segments(tris, 5.0)
    loops, dropped = link_segments(segs, 1e-3)
    assert dropped == 0 and len(loops) == 1
    from shapely.geometry import Polygon
    assert Polygon(loops[0]).area == pytest.approx(1200, abs=1e-6)
    assert plane_segments(tris, 25.0).shape[0] == 0


def test_hollow_part_has_hole():
    outer = box((60, 60, 20), (0, 0, 10))
    inner = box((20, 20, 20), (0, 0, 10))[:, ::-1]  # inverted normals = cavity
    tris = np.concatenate([outer, inner])
    layers, dropped = slice_layers(tris, [5.0])
    assert dropped == 0 and len(layers[0]) == 2
    region = loops_to_region(layers[0], 0.5)
    assert region.area == pytest.approx(3600 - 400, abs=1.0)
    assert len(region.geoms[0].interiors) == 1


def test_layer_heights():
    zs = layer_heights(0, 10, 2.0, 0.5)
    assert zs[0] == 0.5 and zs[-1] <= 10.0 + 1e-9 and len(zs) == 5


def test_line_fill_covers_area():
    from shapely.geometry import box as sbox
    area = sbox(0, 0, 50, 20)
    segs = line_fill(area, 5.0, 0.0)
    assert len(segs) >= 4
    assert sum(np.linalg.norm(s[-1] - s[0]) for s in segs) == pytest.approx(50 * len(segs), rel=0.01)


def test_slice_box_end_to_end():
    tris = box((40, 30, 20), (0, 0, 10))
    p = SliceParams(layer_height=1.0, first_layer_height=1.0, bead_width=2.0, perimeters=2, infill_density=0.3, top_layers=2, bottom_layers=2)
    gcode, st, layers = slice_triangles(tris, p)
    assert st.layers == 20 and st.dropped_chains == 0
    kinds = {k for L in layers for k in [q.kind for q in L.paths]}
    assert {"WALL-OUTER", "WALL-INNER", "SKIN", "FILL"} <= kinds
    assert all(any(q.kind == "SKIN" for q in L.paths) for L in layers[:2] + layers[-2:])
    assert any(any(q.kind == "FILL" for q in L.paths) for L in layers[5:15])
    # outer wall of a 40x30 box offset by half a bead: (38 + 28) * 2 per layer
    outer = sum(q.length for L in layers for q in L.paths if q.kind == "WALL-OUTER")
    assert outer == pytest.approx(132 * 20, rel=0.01)
    # G-code: absolute E strictly increasing on extrusion moves, parses, translates, extruder events
    res = parse_gcode(gcode)
    moves = [c for c in res.commands if isinstance(c, LinearMove)]
    assert any(m.extrude > 0 for m in moves) and all(m.extrude == 0 for m in moves if m.rapid)
    assert st.e_total > 0
    items, stats = translate(res, TranslateOptions(extruder=True, tol=0.05))
    from gcode2krl.translator import EventItem
    ev = [i.kind for i in items if isinstance(i, EventItem)]
    assert "extruder_on" in ev and "extruder_off" in ev
    src = write_krl(items, KrlOptions(name="BOX", extruder_out=7), stats).files["BOX.src"]
    assert "$OUT[7] = TRUE" in src and "$OUT[7] = FALSE" in src
    assert st.bounds["max"][2] == pytest.approx(20.0)


def test_relative_e_and_g92():
    res = parse_gcode("M83\nG1 X10 E1 F600\nG1 X20 E0\nM82\nG92 E0\nG1 X30 E2\nG1 X40 E1\n")
    m = [c for c in res.commands if isinstance(c, LinearMove)]
    assert [round(c.extrude, 6) for c in m] == [1.0, 0.0, 2.0, -1.0]


def test_concentric_and_grid_patterns():
    tris = box((30, 30, 6), (0, 0, 3))
    for pat in ("concentric", "grid", "none"):
        gcode, st, layers = slice_triangles(tris, SliceParams(layer_height=1, bead_width=2, perimeters=1, infill_density=0.2, infill_pattern=pat, top_layers=1, bottom_layers=1))
        assert st.layers == 6
        has_fill = any(q.kind == "FILL" for L in layers for q in L.paths)
        assert has_fill == (pat != "none")
