"""PTP HOME at the start and end of the program and the collision check of those
joint-space moves."""
import json

import numpy as np
import pytest

from gcode2krl.collision import obstacle_from_dict, parse_obstacles, robot_capsules
from gcode2krl.gcode_parser import parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.robot import ROBOTS, check_joint_move, frame
from gcode2krl.translator import TranslateOptions, translate

G = "G21\nG90\nG1 F1200\nG1 X0 Y0 Z2\nG1 X50 Y0 E100\nG1 X50 Y50 E200\nM30\n"


def test_home_moves_bracket_the_program():
    items, st = translate(parse_gcode(G), TranslateOptions(extruder=True))
    src = write_krl(items, KrlOptions(name="H", extruder_out=7, krl_end="$OUT[9] = FALSE"), st).files["H.src"]
    lines = src.splitlines()
    home = [i for i, l in enumerate(lines) if l == "PTP XHOME"]
    assert len(home) == 2
    assert lines[home[0] - 1] == "BAS (#VEL_PTP, 100)" and lines[home[0] - 2] == "; HOME (start)"
    assert "PTP $POS_ACT" not in src  # the HOME move is the BCO run
    first_motion = next(i for i, l in enumerate(lines) if l.startswith(("LIN ", "SPLINE", "PTP")))
    assert first_motion == home[0]
    last_motion = max(i for i, l in enumerate(lines) if l.startswith(("LIN ", "ENDSPLINE", "PTP")))
    assert last_motion == home[1]
    # end: extruder off (trigger at the block end) -> HOME -> user lines -> END
    assert home[1] > max(i for i, l in enumerate(lines) if l.startswith("ENDSPLINE"))
    assert lines.index("$OUT[9] = FALSE") > home[1]


def test_home_axes_literal_and_switches():
    items, st = translate(parse_gcode(G), TranslateOptions())
    src = write_krl(items, KrlOptions(name="A", home_mode="axes", home_axes=(0, -90, 90, 0, 30, 0), home_vel=50), st).files["A.src"]
    assert src.count("PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 30.000, A6 0.000}") == 2
    assert "BAS (#VEL_PTP, 50)" in src
    src2 = write_krl(items, KrlOptions(name="B", home_start=False, home_end=False), st).files["B.src"]
    assert "XHOME" not in src2 and "PTP $POS_ACT" in src2
    src3 = write_krl(items, KrlOptions(name="C", home_start=False, home_end=True, bco_ptp=False), st).files["C.src"]
    assert src3.count("PTP XHOME") == 1 and src3.index("PTP XHOME") > src3.rindex("ENDSPLINE")


def test_joint_move_check_detects_an_obstacle_on_the_way():
    r = ROBOTS["KR90R2700PRO"]
    tool = frame((0, 0, 150, 0, 0, 0))
    caps = robot_capsules(r, tool)
    home = np.array(r.home, float)
    target = np.array([0.0, -60.0, 100.0, 0.0, 50.0, 0.0])
    free = check_joint_move(r, home, target, caps, [], margin=10.0)
    assert free.n_collision == 0 and free.n_samples >= 16
    # a wall right in front of the robot that the arm sweeps through
    wall = obstacle_from_dict(parse_obstacles("box sciana robot 1200 0 1200 100 2000 2400")[0], np.eye(4))
    hit = check_joint_move(r, home, target, caps, [wall], margin=10.0)
    assert hit.n_collision > 0 and hit.min_clearance < 0


def test_server_reports_home_moves():
    from gcode2krl.server import run_translation
    res = run_translation(G, "h.nc", {"robot": "KR90R2700PRO", "base_frame": "1400 0 500 0 0 0", "tool_frame": "0 0 150 0 0 0",
                                       "extruder": True, "extruder_out": 7})
    hm = res["robot"]["home_moves"]
    assert hm["home"] == list(ROBOTS["KR90R2700PRO"].home)
    assert hm["start"]["n_collision"] == 0 and hm["end"]["n_collision"] == 0
    res2 = run_translation(G, "h.nc", {"robot": "KR90R2700PRO", "base_frame": "1400 0 500 0 0 0", "home_end": False, "home_axes": "0 -90 90 0 10 0"})
    assert res2["robot"]["home_moves"]["end"] is None and res2["robot"]["home_moves"]["home"][4] == 10.0
    assert "PTP XHOME" in res2["files"]["H.src"]
