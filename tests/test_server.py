import io
import json
import os
import threading
import urllib.request
import zipfile

import pytest

from gcode2krl.server import make_server, run_translation

HERE = os.path.dirname(__file__)
EXAMPLE = os.path.join(HERE, "..", "examples", "fillet_rect.nc")


@pytest.fixture(scope="module")
def server():
    srv = make_server("127.0.0.1", 0)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


def get(url):
    with urllib.request.urlopen(url, timeout=10) as r:
        return r.status, r.headers, r.read()


def post_json(url, payload):
    req = urllib.request.Request(url, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_static_and_examples(server):
    status, headers, body = get(server + "/")
    assert status == 200 and b"gcode2krl" in body and b"viewer.js" in body
    for f in ("/viewer.js", "/app.js", "/style.css"):
        assert get(server + f)[0] == 200
    status, _, body = get(server + "/api/examples")
    names = json.loads(body)["examples"]
    assert "fillet_rect.nc" in names
    status, _, body = get(server + "/api/example/fillet_rect.nc")
    assert "G1" in json.loads(body)["gcode"]


def test_translate_and_zip(server):
    gcode = open(EXAMPLE, encoding="utf-8").read()
    status, data = post_json(server + "/api/translate", {"gcode": gcode, "filename": "fillet_rect.nc",
                                                         "options": {"tol": "0.05", "spindle_out": "3", "abc": "0 180 0"}})
    assert status == 200, data
    assert data["stats"]["arcs_out"] == 4 and data["stats"]["lines_out"] == 5
    assert data["main_file"] == "FILLET_RECT.src" and "FILLET_RECT.dat" in data["files"]
    assert "$OUT[3] = TRUE" in data["files"]["FILLET_RECT.src"]
    pv = data["preview"]
    assert all(k in pv for k in ("input", "rapid", "slin", "scirc", "spl", "arc_points", "bounds"))
    import base64
    import numpy as np
    arcs = np.frombuffer(base64.b64decode(pv["scirc"]), dtype=np.float32)
    assert arcs.size % 6 == 0 and arcs.size > 0
    mids = np.frombuffer(base64.b64decode(pv["arc_points"]), dtype=np.float32)
    assert mids.size == 4 * 3
    assert pv["bounds"]["min"][0] == 0.0 and pv["bounds"]["max"][0] == 100.0
    status, headers, body = get(server + "/api/zip/" + data["token"])
    assert status == 200 and headers["Content-Type"] == "application/zip"
    with zipfile.ZipFile(io.BytesIO(body)) as zf:
        assert sorted(zf.namelist()) == ["FILLET_RECT.dat", "FILLET_RECT.src"]
        assert b"\r\n  SCIRC" in zf.read("FILLET_RECT.src")


def test_translate_errors(server):
    status, data = post_json(server + "/api/translate", {"gcode": "", "filename": "x.nc", "options": {}})
    assert status == 400 and "error" in data
    status, data = post_json(server + "/api/translate", {"gcode": "G1 X1 F100", "filename": "x.nc", "options": {"tol": "abc"}})
    assert status == 400 and "tol" in data["error"]
    status, data = post_json(server + "/api/translate", {"gcode": "G1 X1 F100", "filename": "x.nc", "options": {"mode": "weird"}})
    assert status == 400
    try:
        get(server + "/api/zip/doesnotexist")
    except urllib.error.HTTPError as e:
        assert e.code == 404


def test_run_translation_options_roundtrip():
    gcode = "G21 G90\nG1 X10 Y0 F600\nG1 X10 Y10\nM30\n"
    res = run_translation(gcode, "my file.nc", {"mode": "lin", "offset": [1, 2, 3], "name": "", "round_corners": True})
    assert res["name"] == "MY_FILE"
    src = res["files"]["MY_FILE.src"]
    assert "LIN {X 11.000, Y 2.000, Z 3.000" in src and "C_DIS" in src
    assert res["stats"]["time_total"] >= 0


def test_robot_report_in_response(server):
    gcode = open(EXAMPLE, encoding="utf-8").read()
    opts = {"robot": "KR90R2700PRO", "base_frame": "1200 0 500 0 0 0", "tool_frame": "0 0 150 0 0 0", "abc": "0 180 0",
            "base_literal": True}
    status, data = post_json(server + "/api/translate", {"gcode": gcode, "filename": "f.nc", "options": opts})
    assert status == 200, data
    rep = data["robot"]
    assert rep["robot"] == "KR90R2700PRO" and rep["n_unreachable"] == 0 and rep["n_limit"] == 0
    assert rep["n_samples"] > 10 and len(rep["joint_min"]) == 6
    import base64
    import numpy as np
    joints = np.frombuffer(base64.b64decode(rep["joints"]), dtype=np.float32).reshape(-1, 6)
    assert joints.shape[0] == rep["n_samples"]
    assert "$BASE = {X 1200.000, Y 0.000, Z 500.000" in data["files"]["F.src"]
    assert "$TOOL = TOOL_DATA[1]" in data["files"]["F.src"]
    # far away base -> unreachable, warning, bad points
    opts["base_frame"] = "4000 0 500 0 0 0"
    status, data = post_json(server + "/api/translate", {"gcode": gcode, "filename": "f.nc", "options": opts})
    assert data["robot"]["n_unreachable"] == data["robot"]["n_samples"]
    assert any("out of reach" in w for w in data["warnings"])
    assert len(base64.b64decode(data["robot"]["bad_points"])) == data["robot"]["n_samples"] * 12
    # robot none -> no report
    status, data = post_json(server + "/api/translate", {"gcode": gcode, "filename": "f.nc", "options": {"robot": "none"}})
    assert data["robot"] is None
    status, _, body = get(server + "/api/robots")
    assert any(r["key"] == "KR90R2700PRO" for r in json.loads(body)["robots"])
