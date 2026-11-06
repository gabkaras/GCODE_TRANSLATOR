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
    # robot-native switching: triggers at the start / end of each extrusion, no advance-run stop
    assert "TRIGGER WHEN DISTANCE=0 DELAY=0 DO $OUT[7]=TRUE PRIO=-1" in src and "TRIGGER WHEN PATH=0 DELAY=0 DO $OUT[7]=FALSE PRIO=-1" in src
    assert "$OUT[7] = FALSE" not in src  # every switch-off is a trigger at a block end
    assert st.bounds["max"][2] == pytest.approx(20.0)


def test_relative_e_and_g92():
    res = parse_gcode("M83\nG1 X10 E1 F600\nG1 X20 E0\nM82\nG92 E0\nG1 X30 E2\nG1 X40 E1\n")
    m = [c for c in res.commands if isinstance(c, LinearMove)]
    assert [round(c.extrude, 6) for c in m] == [1.0, 0.0, 2.0, -1.0]


def test_concentric_and_grid_patterns():
    tris = box((30, 30, 6), (0, 0, 3))
    for pat in ("concentric", "grid", "none"):
        gcode, st, layers = slice_triangles(tris, SliceParams(layer_height=1, first_layer_height=1, bead_width=2, perimeters=1, infill_density=0.2, infill_pattern=pat, top_layers=1, bottom_layers=1))
        assert st.layers == 6
        has_fill = any(q.kind == "FILL" for L in layers for q in L.paths)
        assert has_fill == (pat != "none")


def test_conical_slicing_tilts_tool_and_keeps_geometry():
    tris = box((40, 40, 20), (0, 0, 10))
    p = SliceParams(layer_height=1.0, bead_width=2.0, perimeters=1, infill_density=0.0, top_layers=0, bottom_layers=0,
                    layer_mode="conical", cone_angle=30.0)
    gcode, st, layers = slice_triangles(tris, p)
    assert st.layer_mode == "conical" and st.max_tilt == pytest.approx(30.0, abs=0.5)
    pts = np.concatenate([q.xyz for L in layers for q in L.paths])
    assert pts[:, 2].min() >= -1e-6 and pts[:, 2].max() <= 20.0 + 0.05  # densified wall points: chord error of the cone map
    # outer walls lie inside the box footprint (offset half a bead); gap-fill beads stay within the part
    walls = np.concatenate([q.xyz for L in layers for q in L.paths if q.kind.startswith("WALL")])
    assert np.abs(walls[:, :2]).max() <= 19.0 + 1e-6
    assert all(np.abs(q.xyz[:, :2]).max() + q.width / 2.0 <= 20.0 + 0.5 for L in layers for q in L.paths)
    # the G-code carries the tool axis and real Z
    lines = [l for l in gcode.splitlines() if l.startswith("G1 X")]
    assert all(" I" in l and " K" in l and " Z" in l for l in lines)
    # axis points outward and up for an outward cone
    axes = np.concatenate([q.axes for L in layers for q in L.paths])
    assert axes[:, 2].min() > 0.8
    # translates into KRL with varying orientation
    from gcode2krl.gcode_parser import parse_gcode
    from gcode2krl.translator import SplItem, TranslateOptions, has_orientation, translate
    items, _ = translate(parse_gcode(gcode), TranslateOptions())
    assert has_orientation(items) and any(isinstance(i, SplItem) and i.axes is not None for i in items)


def test_bottom_following_layers_follow_sloped_base():
    # wedge-shaped slab: bottom surface rises 10 mm over 100 mm in X, 20 mm thick
    base_z = lambda x: 0.1 * (x + 50)  # noqa: E731
    v = np.array([[x, y, base_z(x) + dz] for x in (-50, 50) for y in (-20, 20) for dz in (0, 20)], float)
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = np.array([[v[a], v[b], v[c]] for a, b, c, d in faces] + [[v[a], v[c], v[d]] for a, b, c, d in faces], float)
    p = SliceParams(layer_height=2.0, bead_width=2.0, perimeters=1, infill_density=0.0, top_layers=0, bottom_layers=0,
                    layer_mode="bottom", hf_cell=5.0, max_slope=45.0)
    gcode, st, layers = slice_triangles(tris, p)
    assert st.layers == pytest.approx(10, abs=1)
    first = np.concatenate([q.xyz for q in layers[0].paths])
    # the first layer follows the sloped bottom: z grows with x at the base slope
    slope = np.polyfit(first[:, 0], first[:, 2], 1)[0]
    assert slope == pytest.approx(0.1, abs=0.02)
    axes = np.concatenate([q.axes for q in layers[0].paths])
    assert np.degrees(np.arccos(axes[:, 2].mean())) == pytest.approx(np.degrees(np.arctan(0.1)), abs=1.0)
    assert st.support_layers == 0


def test_support_indicator_flags_overhang():
    # T shape: narrow stem then a wide top -> the top layers are unsupported
    stem = box((20, 20, 20), (0, 0, 10))
    cap = box((80, 80, 10), (0, 0, 25))
    gcode, st, layers = slice_triangles(np.concatenate([stem, cap]), SliceParams(layer_height=2, bead_width=2, perimeters=1, infill_density=0.1, top_layers=1, bottom_layers=1))
    assert st.support_layers >= 1 and st.support_area > 1000
    gcode2, st2, _ = slice_triangles(stem, SliceParams(layer_height=2, bead_width=2, perimeters=1, infill_density=0.1, top_layers=1, bottom_layers=1))
    assert st2.support_layers == 0


def test_multi_direction_segments():
    from gcode2krl.slicer.decompose import parse_segments, rotation_to_z, slice_segments
    # an L shape: vertical stem and a horizontal arm sticking out in +X; the arm is printed sideways
    stem = box((20, 20, 60), (0, 0, 30))
    arm = box((40, 20, 20), (30, 0, 50))
    tris = np.concatenate([stem, arm])
    text = """
segment stem 0 0 1
plane 10 0 0  -1 0 0      # keep x <= 10
segment arm 1 0 0
plane 10 0 0   1 0 0      # keep x >= 10
"""
    segs = parse_segments(text)
    assert len(segs) == 2 and np.allclose(rotation_to_z(segs[1].direction) @ segs[1].direction, [0, 0, 1])
    p = SliceParams(layer_height=2.0, bead_width=2.0, perimeters=1, infill_density=0.1, top_layers=1, bottom_layers=1)
    gcode, st, per_seg = slice_segments(tris, segs, p)
    assert len(per_seg) == 2 and st.layer_mode == "multi"
    stem_pts = np.concatenate([q.xyz for L in per_seg[0] for q in L.paths])
    arm_pts = np.concatenate([q.xyz for L in per_seg[1] for q in L.paths])
    assert stem_pts[:, 0].max() <= 10.0 + 1e-6 and arm_pts[:, 0].min() >= 10.0 - 1e-6
    # arm layers are planes of constant x
    first_arm = np.concatenate([q.xyz for q in per_seg[1][0].paths])
    assert np.ptp(first_arm[:, 0]) < 1e-6
    axes = np.concatenate([q.axes for L in per_seg[1] for q in L.paths])
    assert np.allclose(axes, [1, 0, 0])
    assert ";SEGMENT:arm" in gcode and gcode.count("M30") == 1
    from gcode2krl.gcode_parser import parse_gcode
    from gcode2krl.translator import TranslateOptions, has_orientation, translate
    items, _ = translate(parse_gcode(gcode), TranslateOptions())
    assert has_orientation(items)
