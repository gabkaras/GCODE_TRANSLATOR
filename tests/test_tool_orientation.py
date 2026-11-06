"""Tool orientation: angled nozzle mounts ($TOOL with A B C), spin strategies about
the nozzle axis (free / fixed / tangent), lead / side tilt from the slicer and the
collision capsules of an angled tool."""
import math

import numpy as np
import pytest

from gcode2krl.collision import robot_capsules
from gcode2krl.robot import ROBOTS, abc_from_axis, abc_to_rot, check_path, frame, inverse, path_tangents, solve_spins, spin_for_tangent

R = ROBOTS["KR90R2700PRO"]
BASE = frame((1400.0, 0.0, 500.0, 0.0, 0.0, 0.0))


def rot_x(t):
    return abc_to_rot(*t)


def test_spin_for_tangent_points_tool_x_along_path():
    axis = (0.0, 0.0, 1.0)
    for tangent in ((1.0, 0, 0), (0, 1.0, 0), (-1.0, 1.0, 0), (0.3, -0.9, 0.2)):
        sp = spin_for_tangent(axis, tangent, offset_deg=0.0)
        r = abc_to_rot(*abc_from_axis(axis, sp))
        t = np.asarray(tangent, float)
        t = t - (t @ np.array(axis)) * np.array(axis)
        t /= np.linalg.norm(t)
        assert np.allclose(r[:, 0], t, atol=1e-6)
        assert np.allclose(r[:, 2], [0, 0, -1], atol=1e-9)  # tool Z still into the part
    sp180 = spin_for_tangent(axis, (1.0, 0, 0), offset_deg=180.0)
    r = abc_to_rot(*abc_from_axis(axis, sp180))
    assert np.allclose(r[:, 0], [-1, 0, 0], atol=1e-6)


def test_path_tangents_central_differences():
    t = path_tangents(np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [1, 2, 0.0]]))
    assert np.allclose(t[0], [1, 0, 0]) and np.allclose(t[-1], [0, 1, 0]) and np.allclose(t[1], [1, 1, 0] / np.sqrt(2))


def square_path(n_side=10, z=20.0):
    pts = []
    for (x0, y0), (x1, y1) in [((-100, -100), (100, -100)), ((100, -100), (100, 100)), ((100, 100), (-100, 100)), ((-100, 100), (-100, -100))]:
        for k in range(n_side):
            s = k / n_side
            pts.append([x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, z])
    return np.array(pts)


def test_spin_modes_fixed_and_tangent_with_straight_tool():
    pts = square_path()
    axes = np.tile([0.0, 0.0, 1.0], (len(pts), 1))
    tool = frame((0.0, 0.0, 150.0, 0.0, 0.0, 0.0))
    abc_fixed, inf = solve_spins(R, pts, axes, BASE, tool, spin_mode="fixed", spin_fixed=90.0)
    assert inf["unreachable"] == 0
    want = abc_from_axis((0, 0, 1), 90.0)
    assert np.allclose(abc_fixed, np.tile(want, (len(pts), 1)), atol=1e-6)
    abc_t, inf = solve_spins(R, pts, axes, BASE, tool, spin_mode="tangent", spin_offset=0.0)
    assert inf["unreachable"] == 0
    tg = path_tangents(pts)
    for a, t in zip(abc_t[2:8], tg[2:8]):
        assert np.allclose(abc_to_rot(*a)[:, 0], t, atol=1e-6)
    rep = check_path(R, pts, abc_t, BASE, tool)
    assert rep.n_unreachable == 0 and rep.n_limit == 0


def test_angled_nozzle_mount_is_reachable_and_keeps_tcp_orientation():
    pts = square_path(z=10.0)
    axes = np.tile([0.0, 0.0, 1.0], (len(pts), 1))
    for mount in ((0.0, 45.0, 0.0), (0.0, 90.0, 0.0), (0.0, 0.0, 90.0)):
        tool = frame((0.0, 0.0, 150.0, *mount))
        abc, inf = solve_spins(R, pts, axes, BASE, tool)
        assert inf["unreachable"] == 0, mount
        rep = check_path(R, pts, abc, BASE, tool)
        assert rep.n_unreachable == 0 and rep.n_limit == 0, mount
        # the TCP frame (nozzle) points into the part, while the flange is rotated by the mount
        q = rep.joints[5]
        from gcode2krl.robot import forward
        fl = forward(R, q)[-1]
        tcp = BASE @ frame((pts[5][0], pts[5][1], pts[5][2], *abc[5]))
        flange_from_tcp = tcp @ np.linalg.inv(tool)
        assert np.allclose(fl[:3, :3], flange_from_tcp[:3, :3], atol=1e-6)
        assert np.allclose(tcp[:3, 2], [0, 0, -1], atol=1e-9)
        flange_z = fl[:3, 2]
        ang = math.degrees(math.acos(abs(float(flange_z @ tcp[:3, 2]))))
        assert ang == pytest.approx(max(abs(mount[1]), abs(mount[2])), abs=1e-6)


def test_capsules_follow_the_nozzle_axis_of_an_angled_tool():
    tool = frame((0.0, 0.0, 150.0, 0.0, 90.0, 0.0))  # nozzle axis along flange -X... (B = 90 rotates Z onto X)
    caps = robot_capsules(R, tool, tool_radius=25.0, cutter_len=40.0, cutter_radius=6.0)
    cut = next(c for c in caps if c.kind == "cutter")
    zt = tool[:3, 2]
    assert np.allclose(cut.b, tool[:3, 3]) and np.allclose(cut.a, tool[:3, 3] - zt * 40.0)
    hold = next(c for c in caps if c.kind == "tool")
    assert np.allclose(hold.a, 0) and np.linalg.norm(hold.b - cut.a) < 25.0 + 1e-6


def test_slicer_tilt_writes_axes_in_planar_mode():
    pytest.importorskip("shapely")
    from gcode2krl.gcode_parser import parse_gcode
    from gcode2krl.slicer import SliceParams, slice_triangles
    from gcode2krl.slicer.orientation import tilt_axes
    from gcode2krl.translator import SplItem, TranslateOptions, translate

    pts = np.array([[0, 0, 0], [10, 0, 0], [20, 0, 0.0]])
    ax = tilt_axes(pts, None, False, 30.0, 0.0)
    assert np.allclose(ax[1], [math.sin(math.radians(30)), 0, math.cos(math.radians(30))], atol=1e-9)
    ax = tilt_axes(pts, None, False, 0.0, 30.0)
    assert np.allclose(ax[1], [0, math.sin(math.radians(30)), math.cos(math.radians(30))], atol=1e-9)  # left of +X is +Y

    def box(sz, c):
        s = np.array(sz) / 2
        c = np.array(c)
        v = np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])]) + c
        faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        t = []
        for a, b, c2, d in faces:
            t += [[v[a], v[b], v[c2]], [v[a], v[c2], v[d]]]
        return np.array(t, float)

    g, st, layers = slice_triangles(box((40, 40, 6), (0, 0, 3)), SliceParams.from_dict(dict(layer_height=2, first_layer_height=2, bead_width=4, perimeters=1, infill_density=0, top_layers=0, bottom_layers=0, tilt_lead=-20)), "t")
    lines = [l for l in g.splitlines() if l.startswith("G1 X")]
    assert all(" I" in l and " K" in l for l in lines)
    assert st.max_tilt == pytest.approx(20.0, abs=0.5)
    items, _ = translate(parse_gcode(g), TranslateOptions())
    assert any(isinstance(i, SplItem) and i.axes is not None for i in items)
    g0, st0, _ = slice_triangles(box((40, 40, 6), (0, 0, 3)), SliceParams.from_dict(dict(layer_height=2, first_layer_height=2, bead_width=4, perimeters=1, infill_density=0, top_layers=0, bottom_layers=0)), "t0")
    assert not any(" I" in l for l in g0.splitlines() if l.startswith("G1 X"))  # planar without tilt: constant orientation
