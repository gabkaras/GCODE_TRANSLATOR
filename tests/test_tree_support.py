"""Tree support: automatic tips on overhang islands, merging branches, XY gap,
resting on the model, walls pattern."""
import numpy as np
import pytest

pytest.importorskip("shapely")

from shapely.geometry import box as sbox

from gcode2krl.slicer import SliceParams, slice_triangles
from gcode2krl.slicer.toolpath import as_multi
from gcode2krl.slicer.tree_support import hex_grid_points, tree_support_areas


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


T = np.concatenate([box((10, 10, 40), (0, 0, 20)), box((80, 30, 6), (0, 0, 43))])


def test_hex_grid_covers_islands():
    pts = hex_grid_points(sbox(0, 0, 40, 20), 6.0)
    assert len(pts) > 10 and all(0 <= x <= 40 and 0 <= y <= 20 for x, y in pts)
    assert len(hex_grid_points(sbox(0, 0, 2, 2), 10.0)) == 1  # a tiny island still gets one tip


def test_tree_branches_merge_and_save_material():
    _, st_n, l_n = slice_triangles(T, P(support="normal"), "n")
    _, st_t, l_t = slice_triangles(T, P(support="tree", support_pattern="walls"), "t")
    assert 0 < st_t.support_volume < 0.7 * st_n.support_volume
    sup = [L for L in l_t if not L.support.is_empty]
    tips = len(sup[-1].support.geoms)
    trunks = len(sup[0].support.geoms)
    assert tips >= 4 and trunks < tips  # many tips under the slab merge into fewer trunks
    # the branches grow on the way down
    assert sup[0].support.area / trunks > sup[-1].support.area / tips
    # XY gap to the column is kept on every layer
    assert min(L.region.distance(L.support) for L in sup) >= P().support_xy_gap - 0.02  # disc polygons approximate the circle
    # z gap: the layer right under the slab carries no support
    first_slab = next(L for L in l_t if L.region.bounds[2] - L.region.bounds[0] > 50)
    assert all(L.support.is_empty for L in l_t if L.index == first_slab.index - 1)
    assert "SUPPORT" in st_t.by_type and "SUPPORT-INTERFACE" in st_t.by_type
    assert any(q.kind == "SUPPORT" and q.closed for L in l_t for q in L.paths)  # walls pattern: rings


def test_tree_rests_on_the_model_or_is_clipped():
    # a wide base with a thin wall and an overhanging slab above the base: branches land on the base
    part = np.concatenate([box((80, 80, 10), (0, 0, 5)), box((6, 6, 30), (0, 0, 25)), box((40, 40, 4), (0, 0, 42))])
    _, st, layers = slice_triangles(part, P(support="tree"), "r")
    sup = [L for L in layers if not L.support.is_empty]
    assert sup and sup[0].z > 10.0  # the lowest support sits on the base, not on the bed
    assert all(L.region.distance(L.support) >= 1.0 - 0.02 for L in sup)
    _, st2, layers2 = slice_triangles(part, P(support="tree", support_on_model=False), "r2")
    assert st2.support_volume > 0


def test_tree_areas_direct_api():
    regions = [as_multi(sbox(-5, -5, 5, 5))] * 10 + [as_multi(sbox(-40, -15, 40, 15))] * 3
    heights = [2.0] * 13
    printed, iface = tree_support_areas(regions, heights, P(support="tree"))
    assert printed[9].is_empty and not printed[8].is_empty  # z gap of one layer
    assert all(not printed[i].is_empty for i in range(0, 9))
    assert not iface[8].is_empty and iface[2].is_empty
