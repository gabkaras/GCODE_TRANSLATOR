import base64
import json
import os
import threading
import urllib.request

import numpy as np
import pytest

from gcode2krl import meshstore
from gcode2krl.collision import check_collisions, mesh_obstacles, parse_obstacles, robot_capsules, tool_mesh_capsules
from gcode2krl.meshio import load_mesh, save_stl
from gcode2krl.robot import ROBOTS, frame


def cube(size, center):
    s = size / 2.0
    c = np.array(center)
    v = np.array([[x, y, z] for x in (-s, s) for y in (-s, s) for z in (-s, s)]) + c
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c2, d in faces:
        tris.append([v[a], v[b], v[c2]])
        tris.append([v[a], v[c2], v[d]])
    return np.array(tris, dtype=float)


@pytest.fixture
def store(tmp_path, monkeypatch):
    monkeypatch.setenv("GCODE2KRL_MESH_DIR", str(tmp_path / "meshes"))
    return tmp_path


def test_obj_loader_and_units(tmp_path):
    p = tmp_path / "t.obj"
    p.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n")
    tris = load_mesh(str(p), "m")
    assert tris.shape == (2, 3, 3) and tris.max() == 1000.0
    with pytest.raises(ValueError):
        load_mesh(str(p), "furlongs")


def test_meshstore_save_list_resolve(store):
    tris = cube(100, (0, 0, 50))
    tmp = store / "box.stl"
    save_stl(str(tmp), tris)
    meta = meshstore.save_mesh("box.stl", tmp.read_bytes())
    assert meta["tris"] == 12 and meta["max"][2] == pytest.approx(100)
    assert any(m["id"] == meta["id"] for m in meshstore.list_meshes())
    assert meshstore.resolve(meta["id"]).endswith(".stl")
    t2, key = meshstore.load(meta["id"])
    assert len(t2) == 12
    assert meshstore.delete_mesh(meta["id"]) and not meshstore.list_meshes()
    with pytest.raises(ValueError):
        meshstore.resolve("nope")


def test_mesh_obstacle_box_is_tight_and_placed():
    base = frame((1000, 0, 500, 0, 0, 0))
    tris = cube(200, (0, 0, 100))  # 200 mm cube standing on the base plane
    obs = mesh_obstacles({"kind": "mesh", "name": "uchwyt", "frame": "base", "center": [300, 0, 0], "abc": [0, 0, 0]}, base, tris, mesh_id="c")
    assert len(obs) == 1 and obs[0].kind == "box" and obs[0].group == "uchwyt"
    assert np.allclose(obs[0].center, [1300, 0, 600])
    assert np.allclose(obs[0].half, [101, 101, 101])


def test_mesh_obstacle_line_and_collision(store):
    r = ROBOTS["KR90R2700PRO"]
    base = frame((1400, 0, 500, 0, 0, 0))
    tris = cube(100, (0, 0, 50))
    path = store / "post.stl"
    save_stl(str(path), tris)
    d = parse_obstacles(f"mesh slupek base 50 30 0 0 0 0 {path}")[0]
    assert d["kind"] == "mesh" and d["file"] == str(path)
    tri2, key = meshstore.load(d["file"], d["units"])
    obs = mesh_obstacles(d, base, tri2, mesh_id=d["file"])
    tool = frame((0, 0, 150, 0, 0, 0))
    caps = robot_capsules(r, tool)
    # TCP right above the post centre, 20 mm above its top: holder capsule overlaps the post
    from gcode2krl.robot import check_path
    pts = np.array([[50.0, 30.0, 105.0]])  # cutter tip 1 mm inside the post (not a workpiece)
    reach = check_path(r, pts, (0, 180, 0), base, tool)
    rep = check_collisions(r, reach.joints, reach.ok_mask, caps, obs, margin=5)
    assert rep.n_collision == 1 and rep.pairs[0].b == "slupek"
    far = check_path(r, np.array([[50.0, 30.0, 400.0]]), (0, 180, 0), base, tool)
    rep2 = check_collisions(r, far.joints, far.ok_mask, caps, obs, margin=5)
    assert rep2.n_collision == 0


def test_tool_mesh_capsules_skip_cutter():
    # tool mesh: holder cylinder-ish box 60 mm wide from z=0..130 and a thin cutter box z=130..150
    tris = np.concatenate([cube(60, (0, 0, 65)), np.array(cube(8, (0, 0, 140)))])
    tool = frame((0, 0, 150, 0, 0, 0))
    caps = tool_mesh_capsules(tris, tool, cutter_len=20)
    assert caps and all(c.kind == "tool" for c in caps)
    # no holder capsule reaches the cutter zone (z > 130 + pad)
    for c in caps:
        assert max(c.a[2], c.b[2]) + c.radius < 150
    r = ROBOTS["KR90R2700PRO"]
    allc = robot_capsules(r, tool, tool_mesh=tris)
    assert any(c.kind == "cutter" for c in allc) and any(c.kind == "tool" for c in allc)


def test_server_mesh_upload_and_translate(store):
    from gcode2krl.server import make_server
    srv = make_server("127.0.0.1", 0)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    url = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        tris = cube(100, (0, 0, 50))
        tmp = store / "u.stl"
        save_stl(str(tmp), tris)
        body = json.dumps({"name": "u.stl", "data": base64.b64encode(tmp.read_bytes()).decode()}).encode()
        req = urllib.request.Request(url + "/api/mesh", data=body, headers={"Content-Type": "application/json"}, method="POST")
        meta = json.loads(urllib.request.urlopen(req, timeout=30).read())
        assert meta["tris"] == 12
        lst = json.loads(urllib.request.urlopen(url + "/api/meshes", timeout=10).read())
        assert any(m["id"] == meta["id"] for m in lst["meshes"])
        pay = json.loads(urllib.request.urlopen(url + "/api/mesh/" + meta["id"], timeout=10).read())
        assert pay["tris"] == 12
        gcode = "G1 X10 F500\nG1 X20\nM30\n"
        opts = {"robot": "KR90R2700PRO", "base_frame": "1400 0 500 0 0 0", "tool_frame": "0 0 150 0 0 0",
                "obstacles": f"floor f robot 0\nmesh u base 15 0 0 0 0 0 {meta['id']}", "tool_mesh": meta["id"]}
        req = urllib.request.Request(url + "/api/translate", data=json.dumps({"gcode": gcode, "filename": "t.nc", "options": opts}).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        res = json.loads(urllib.request.urlopen(req, timeout=60).read())
        col = res["robot"]["collision"]
        assert any(o["group"] == "u" and o["mesh_id"] == meta["id"] for o in col["obstacles"])
        assert col["tool_mesh"] == meta["id"]
        assert col["n_collision"] >= 1  # tool box overlaps the post under the path
    finally:
        srv.shutdown()
        srv.server_close()


@pytest.mark.skipif(not __import__("gcode2krl.stepio", fromlist=["available_backends"]).available_backends(), reason="no CAD kernel")
def test_step_import_and_tool(store):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer

    from gcode2krl.meshio import load_mesh
    from gcode2krl.stepio import step_to_triangles

    shape = BRepPrimAPI_MakeBox(100, 80, 40).Shape()
    for unit, fname in (("MM", "box_mm.step"), ("INCH", "box_inch.step")):
        Interface_Static.SetCVal_s("write.step.unit", unit)
        w = STEPControl_Writer()
        w.Transfer(shape, STEPControl_AsIs)
        w.Write(str(store / fname))
    for fname in ("box_mm.step", "box_inch.step"):
        tris = step_to_triangles(str(store / fname), 0.2)
        v = tris.reshape(-1, 3)
        assert np.allclose(v.min(axis=0), [0, 0, 0], atol=1e-3) and np.allclose(v.max(axis=0), [100, 80, 40], atol=1e-3), fname
        assert len(tris) >= 12
    # through the store (tessellated STL cached) and as a mesh obstacle line
    meta = meshstore.save_mesh("box_mm.step", (store / "box_mm.step").read_bytes(), deflection=0.5)
    assert meta["format"] == "step" and meta["max"][0] == pytest.approx(100, abs=1e-3)
    assert meshstore.resolve(meta["id"]).endswith(".stl")
    tris2, key = meshstore.load(meta["id"])
    assert tris2.reshape(-1, 3).max(axis=0)[2] == pytest.approx(40, abs=1e-3)
    # load_mesh ignores STL/OBJ units for STEP (file units rule)
    assert load_mesh(str(store / "box_inch.step"), "m").reshape(-1, 3).max(axis=0)[0] == pytest.approx(100, abs=1e-3)
