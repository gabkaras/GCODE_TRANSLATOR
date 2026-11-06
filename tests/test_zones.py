"""Safety zones: parsing, TCP / whole-robot checks, KRL $WORKSPACE output, server report."""
import numpy as np
import pytest

from gcode2krl.collision import ZONE_HELP, check_zones, parse_zones, robot_capsules, zone_from_dict
from gcode2krl.gcode_parser import parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.robot import ROBOTS, frame
from gcode2krl.translator import TranslateOptions, translate

R = ROBOTS["KR90R2700PRO"]
BASE = frame((1400.0, 0.0, 500.0, 0.0, 0.0, 0.0))
TOOL = frame((0.0, 0.0, 150.0, 0.0, 0.0, 0.0))
G = "G21\nG90\nG1 F1200\nG1 X0 Y0 Z2\nG1 X100 Y0\nG1 X100 Y100\nG1 X0 Y100\nM30\n"


def test_parse_and_frames():
    zs = parse_zones(ZONE_HELP)
    assert [z["name"] for z in zs] == ["druk", "operator"] and zs[0]["mode"] == "allowed" and zs[1]["scope"] == "robot"
    z = zone_from_dict(zs[0], BASE)
    assert np.allclose(z.center, [1400.0, 0.0, 1100.0]) and np.allclose(z.half, [700, 700, 700])  # base frame -> robot frame
    assert z.krl_mode() == "#OUTSIDE_STOP" and zone_from_dict(zs[1], BASE).krl_mode() == "#INSIDE_STOP"
    with pytest.raises(ValueError):
        parse_zones("zone x allowed tcp robot 0 0 0 10 10")
    with pytest.raises(ValueError):
        parse_zones("zone x maybe tcp robot 0 0 0 10 10 10")
    rot = zone_from_dict({"name": "r", "mode": "allowed", "scope": "tcp", "frame": "robot", "center": [0, 0, 0], "size": [200, 100, 100], "abc": [90, 0, 0]}, np.eye(4))
    assert rot.inside(np.array([[0.0, 90.0, 0.0]]))[0] and not rot.inside(np.array([[90.0, 0.0, 0.0]]))[0]  # box turned by 90 deg about Z


def test_tcp_zone_checks():
    allowed = zone_from_dict({"name": "a", "mode": "allowed", "scope": "tcp", "frame": "robot", "center": [1400, 0, 600], "size": [400, 400, 400]}, np.eye(4))
    forbidden = zone_from_dict({"name": "f", "mode": "forbidden", "scope": "tcp", "frame": "robot", "center": [1400, 300, 600], "size": [100, 100, 100]}, np.eye(4))
    tcp = np.array([[1400, 0, 600], [1400, 190, 600], [1400, 210, 600], [1400, 300, 600.0]])
    res = check_zones([allowed, forbidden], tcp)
    assert res[0]["indices"] == [2, 3] and res[1]["indices"] == [3]


def test_robot_scope_zone_sees_the_arm_not_only_the_tcp():
    caps = robot_capsules(R, TOOL)
    # the TCP sits in a small allowed box: fine for scope tcp, violated for scope robot (the arm is outside)
    q = np.array([[0.0, -70.0, 100.0, 0.0, 60.0, 0.0]])
    from gcode2krl.robot import forward
    tcp = (forward(R, q[0])[-1] @ TOOL)[:3, 3][None, :]
    small = {"name": "s", "mode": "allowed", "frame": "robot", "center": tcp[0].tolist(), "size": [300, 300, 300]}
    z_tcp = zone_from_dict({**small, "scope": "tcp"}, np.eye(4))
    z_rob = zone_from_dict({**small, "scope": "robot"}, np.eye(4))
    res = check_zones([z_tcp, z_rob], tcp, R, q, np.ones(1, dtype=bool), caps)
    assert res[0]["n_violations"] == 0 and res[1]["n_violations"] == 1
    # a forbidden box around the robot base is entered by the links, never by the TCP
    fb = zone_from_dict({"name": "b", "mode": "forbidden", "frame": "robot", "center": [0, 0, 400], "size": [600, 600, 800], "scope": "robot"}, np.eye(4))
    fb_tcp = zone_from_dict({"name": "b2", "mode": "forbidden", "frame": "robot", "center": [0, 0, 400], "size": [600, 600, 800], "scope": "tcp"}, np.eye(4))
    res = check_zones([fb, fb_tcp], tcp, R, q, np.ones(1, dtype=bool), caps)
    assert res[0]["n_violations"] == 1 and res[1]["n_violations"] == 0


def test_krl_workspaces():
    items, st = translate(parse_gcode(G), TranslateOptions())
    ws = [{"name": "druk", "mode": "allowed", "scope": "tcp", "center": [1400.0, 0.0, 1100.0], "half": [700.0, 700.0, 700.0], "abc": [0.0, 0.0, 0.0]},
          {"name": "operator", "mode": "forbidden", "scope": "robot", "center": [0.0, 1600.0, 900.0], "half": [600.0, 400.0, 900.0], "abc": [30.0, 0.0, 0.0]}]
    src = write_krl(items, KrlOptions(name="Z", workspaces=ws), st).files["Z.src"]
    assert ("$WORKSPACE[1] = {BASE {X 1400.000, Y 0.000, Z 1100.000, A 0.000, B 0.000, C 0.000}, X1 -700.000, Y1 -700.000, Z1 -700.000, "
            "X2 700.000, Y2 700.000, Z2 700.000, MODE #OUTSIDE_STOP, STATE FALSE}") in src
    assert "$WORKSPACE[2] = {BASE {X 0.000, Y 1600.000, Z 900.000, A 30.000," in src and "MODE #INSIDE_STOP" in src
    assert src.index("$WORKSPACE[1]") < src.index("PTP XHOME")
    many = ws * 5
    src2 = write_krl(items, KrlOptions(name="M", workspaces=many), st).files["M.src"]
    assert "$WORKSPACE[8]" in src2 and "$WORKSPACE[9]" not in src2 and "pominieto 2" in src2


def test_server_zone_report_and_points():
    from gcode2krl.server import run_translation
    opts = {"robot": "KR90R2700PRO", "base_frame": "1400 0 500 0 0 0", "tool_frame": "0 0 150 0 0 0",
            "zones": "zone druk allowed tcp base 50 50 50 300 300 300\nzone mala allowed tcp base 0 0 0 20 20 20\nzone zakaz forbidden tcp base 100 100 0 40 40 40"}
    res = run_translation(G, "z.nc", opts)
    zr = {r["name"]: r for r in res["robot"]["zones"]["results"]}
    assert zr["druk"]["n_path"] == 0 and zr["mala"]["n_path"] > 0 and zr["zakaz"]["n_path"] > 0
    assert zr["druk"]["n_home"] > 0  # the HOME moves leave the small box over the bed (allowed zone, tcp)
    assert res["robot"]["zone_points"] and any("safety zone mala" in w for w in res["warnings"])
    src = res["files"]["Z.src"]
    assert src.count("$WORKSPACE[") == 3 and "MODE #INSIDE_STOP" in src
    res2 = run_translation(G, "z.nc", {**opts, "zones_krl": False})
    assert "$WORKSPACE[" not in res2["files"]["Z.src"]
