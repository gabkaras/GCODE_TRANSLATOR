"""Tool STEP/STL for planar printing: per-component capsules clamped to the nozzle
axis, tool mesh frame, collisions with the already printed part, automatic spin of
a constant orientation."""
import numpy as np
import pytest

pytest.importorskip("shapely")

from shapely.geometry import box as sbox

from gcode2krl.collision import mesh_components, robot_capsules
from gcode2krl.robot import ROBOTS, check_path, check_printed, frame
from gcode2krl.server import run_translation
from gcode2krl.slicer import SliceParams, slice_triangles

R = ROBOTS["KR90R2700PRO"]
BASE = frame((1400.0, 0.0, 500.0, 0.0, 0.0, 0.0))


def box(sz, c):
    s = np.array(sz) / 2
    c = np.array(c)
    v = np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])]) + c
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = []
    for a, b, c2, d in faces:
        t += [[v[a], v[b], v[c2]], [v[a], v[c2], v[d]]]
    return np.array(t, float)


# an extruder: a body beside the holder (+X), the holder on the axis, nozzle tip at z = 150 (flange frame, Z out)
TOOL_MESH = np.concatenate([box((40, 40, 130), (60, 0, 65)), box((20, 20, 140), (0, 0, 70))])
TOOL = frame((0.0, 0.0, 150.0, 0.0, 0.0, 0.0))


def test_components_and_clamped_tool_capsules():
    comps = mesh_components(TOOL_MESH)
    assert len(comps) == 2 and sorted(len(c) for c in comps) == [12, 12]
    caps = [c for c in robot_capsules(R, TOOL, tool_mesh=TOOL_MESH, tool_mesh_key="t1") if c.kind == "tool"]
    assert len(caps) == 2
    for c in caps:
        lo = min(c.a[2], c.b[2]) - c.radius
        hi = max(c.a[2], c.b[2]) + c.radius
        assert lo >= -1e-6 and hi <= 140.0 + 1e-6  # never past the flange plane nor past the body (nozzle part is the cutter)
    body = max(caps, key=lambda c: c.a[0])
    assert abs(body.a[0] - 60.0) < 1.0 and max(body.a[2], body.b[2]) + body.radius <= 130.0 + 1e-6
    cutter = next(c for c in robot_capsules(R, TOOL, tool_mesh=TOOL_MESH, tool_mesh_key="t1") if c.kind == "cutter")
    assert np.allclose(cutter.b, [0, 0, 150]) and np.allclose(cutter.a, [0, 0, 130])


def test_check_printed_detects_a_dipping_holder():
    pts = np.array([[x, 0.0, 20.0] for x in np.linspace(-20, 20, 9)])
    fp = sbox(-30, -30, 30, 30)
    caps = robot_capsules(R, TOOL, tool_mesh=TOOL_MESH, tool_mesh_key="t1")
    rep = check_path(R, pts, (0.0, 180.0, 0.0), BASE, TOOL)
    res = check_printed(R, rep.joints, rep.ok_mask, caps, pts, BASE, fp, margin=2.0)
    assert res["n_collision"] == 0
    tilted = frame((0.0, 0.0, 150.0, 0.0, 60.0, 0.0))  # nozzle 60 deg to the flange: the holder dips under the layer
    caps_t = robot_capsules(R, tilted, tool_mesh=TOOL_MESH, tool_mesh_key="t1")
    rep_t = check_path(R, pts, (0.0, 180.0, 0.0), BASE, tilted)
    res_t = check_printed(R, rep_t.joints, rep_t.ok_mask, caps_t, pts, BASE, fp, margin=2.0)
    assert res_t["n_collision"] == len(pts) and res_t["max_depth"] > 10 and "narzedzie" in res_t["by_capsule"]
    # outside the footprint nothing counts
    res_o = check_printed(R, rep_t.joints, rep_t.ok_mask, caps_t, pts, BASE, sbox(200, 200, 210, 210), margin=2.0)
    assert res_o["n_collision"] == 0


def test_server_auto_spin_and_tool_mesh_frame(tmp_path, monkeypatch):
    from gcode2krl import meshstore
    from gcode2krl.meshio import save_stl
    monkeypatch.setenv("GCODE2KRL_MESH_DIR", str(tmp_path))
    p = tmp_path / "tool.stl"
    save_stl(str(p), TOOL_MESH)
    meta = meshstore.save_mesh("tool.stl", p.read_bytes())
    g, st, _ = slice_triangles(box((60, 60, 40), (0, 0, 20)), SliceParams.from_dict(dict(layer_height=4, first_layer_height=4, bead_width=6, infill_density=0.1)), "cube")
    base = {"robot": "KR90R2700PRO", "base_frame": "1400 0 500 0 0 0", "tool_frame": "0 0 150 0 0 0", "extruder": True, "tool_mesh": meta["id"],
            "obstacles": "floor posadzka robot 0\nbox stol robot 1300 0 240 900 900 480\nbox sciana base 120 0 75 20 300 150"}
    # A = 180 puts the body on +X next to the wall -> collisions; auto spin finds a clean A
    r_bad = run_translation(g, "cube.gcode", {**base, "abc": "180 180 0"})
    assert r_bad["robot"]["collision"]["n_collision"] > 0 and r_bad["robot"]["printed"]["n_collision"] == 0
    r = run_translation(g, "cube.gcode", {**base, "abc": "180 180 0", "auto_spin": 90})
    a = r["robot"]["auto_spin"]
    assert a and r["robot"]["collision"]["n_collision"] == 0 and len(a["candidates"]) == 4
    assert any(c["collisions"] > 0 for c in a["candidates"]) and abs(a["a"] - 180.0) > 1
    src = r["files"]["CUBE.src"]
    assert f"A {a['a']:.3f}, B 180.000" in src and any("auto spin" in w for w in r["warnings"])
    # the tool mesh frame turns the body to the other side: same effect as the spin
    r2 = run_translation(g, "cube.gcode", {**base, "abc": "180 180 0", "tool_mesh_frame": "0 0 0 180 0 0"})
    assert r2["robot"]["collision"]["n_collision"] == 0 and r2["robot"]["collision"]["tool_mesh_frame"] == [0, 0, 0, 180, 0, 0]
    # a tilted mount makes the holder dip into the print: reported and flagged
    r3 = run_translation(g, "cube.gcode", {**base, "tool_frame": "0 0 150 0 60 0", "abc": "0 180 0", "obstacles": "floor posadzka robot 0"})
    assert r3["robot"]["printed"]["n_collision"] > 100 and r3["robot"]["printed_points"] and any("dips into the printed part" in w for w in r3["warnings"])
    r4 = run_translation(g, "cube.gcode", {**base, "tool_frame": "0 0 150 0 60 0", "abc": "0 180 0", "printed_check": False})
    assert r4["robot"]["printed"] is None
