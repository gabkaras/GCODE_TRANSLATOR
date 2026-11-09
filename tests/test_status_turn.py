"""KUKA Status / Turn: bit definitions, PTP approach with S/T, configuration changes."""
import numpy as np

from gcode2krl.gcode_parser import parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.robot import ROBOTS, assign_configurations, critical_a3, frame, status_turn, status_turn_batch
from gcode2krl.translator import RapidItem, TranslateOptions, translate

R = ROBOTS["KR90R2700PRO"]
BASE = frame((1400.0, 0.0, 500.0, 0.0, 0.0, 0.0))
TOOL = frame((0.0, 0.0, 150.0, 0.0, 0.0, 0.0))


def test_status_bits_follow_the_kuka_definition():
    assert abs(critical_a3(R) - (-1.957)) < 0.01  # atan(-41 / 1200): stretched arm
    assert status_turn(R, R.home) == (2, 2)  # S 2 (A3 >= angle), T 2 (A2 < 0)
    s, t = status_turn(R, (10.0, -80.0, 100.0, 20.0, -30.0, 5.0))
    assert s == 2 | 4 and t == (1 << 1) | (1 << 4)  # A5 < 0 -> bit 2; negative axes A2, A5
    s, t = status_turn(R, (0.0, -60.0, -30.0, 0.0, 40.0, 0.0))
    assert s & 2 == 0  # A3 below the stretched-arm angle
    # overhead: the wrist root behind the A1 axis (found by scanning A2 with a folded arm)
    from gcode2krl.robot import forward
    for a2 in range(-180, 0, 10):
        q = (0.0, float(a2), 120.0, 0.0, 0.0, 0.0)
        fr = forward(R, q)
        local_x = (fr[1][:3, :3].T @ (fr[4][:3, 3] - fr[1][:3, 3]))[0]
        assert (status_turn(R, q)[0] & 1 == 1) == (local_x < 0)
    assert any(status_turn(R, (0.0, float(a2), 120.0, 0.0, 0.0, 0.0))[0] & 1 for a2 in range(-180, 0, 10))
    st, tn = status_turn_batch(R, np.array([R.home, [10.0, -80.0, 100.0, 20.0, -30.0, 5.0], [np.nan] * 6]))
    assert list(st) == [2, 6, -1] and list(tn) == [2, 18, -1]


def test_ptp_approach_carries_status_and_turn():
    g = "G21\nG90\nG1 F1200\nG1 X0 Y0 Z2\nG1 X50 Y0\nG0 X0 Y50\nG1 X50 Y50\nM30\n"
    items, st = translate(parse_gcode(g), TranslateOptions())
    info = assign_configurations(items, R, BASE, TOOL, ref=(0.0, 180.0, 0.0))
    rapids = [i for i in items if isinstance(i, RapidItem)]
    assert isinstance(items[0], RapidItem) and rapids[0].ptp and rapids[0].st == (2, 2)  # inserted approach
    assert info["ptp_travels"] == 1 and info["status_changes_in_path"] == 0 and info["start"] == (2, 2)
    assert not rapids[1].ptp and rapids[1].st is not None  # same status: the travel stays LIN
    src = write_krl(items, KrlOptions(name="S"), st).files["S.src"]
    lines = src.splitlines()
    i = lines.index("PTP {X 0.000, Y 0.000, Z 0.000, A 0.000, B 180.000, C 0.000, S 2, T 2}")
    assert lines[i - 1].startswith("; konfiguracja") and lines[i - 2] == "PTP XHOME"
    assert "LIN {X 0.000, Y 50.000, Z 2.000, A 0.000, B 180.000, C 0.000}" in lines
    assert src.count(", S ") == 1


def test_status_change_forces_a_ptp_travel():
    # two far apart targets reached with different elbow configurations are joined by a PTP
    items = [RapidItem(np.array([0.0, 0.0, 10.0]), np.array([0.0, 0.0, 10.0])),
             RapidItem(np.array([0.0, 0.0, 10.0]), np.array([0.0, 0.0, 10.0]))]
    info = assign_configurations(items, R, BASE, TOOL)
    assert items[0].ptp and not items[1].ptp and info["ptp_travels"] == 1
    # force a different status on the second travel by faking the previous one
    items[0].st = (6, 2)
    items2 = [RapidItem(np.array([0.0, 0.0, 10.0]), np.array([0.0, 0.0, 10.0]), st=(6, 2), ptp=True),
              RapidItem(np.array([0.0, 0.0, 10.0]), np.array([0.0, 0.0, 10.0]), st=(2, 2), ptp=True)]
    src = write_krl(items2, KrlOptions(name="P", home_start=False, bco_ptp=False)).files["P.src"]
    assert src.count("PTP {") == 2 and "S 6, T 2}" in src and "S 2, T 2}" in src and src.index("BAS (#VEL_PTP") < src.index("PTP {")


def test_server_reports_configuration():
    from gcode2krl.server import run_translation
    g = "G21\nG90\nG1 F1200\nG1 X0 Y0 Z2\nG1 X50 Y0\nM30\n"
    res = run_translation(g, "c.nc", {"robot": "KR90R2700PRO", "base_frame": "1400 0 500 0 0 0", "tool_frame": "0 0 150 0 0 0"})
    cfg = res["robot"]["configuration"]
    assert cfg["ptp_travels"] == 1 and cfg["status_changes_sampled"] == 0 and cfg["status_values"] == [2]
    assert ", S 2, T 2}" in res["files"]["C.src"]
    res2 = run_translation(g, "c.nc", {"robot": "KR90R2700PRO", "base_frame": "1400 0 500 0 0 0", "status_turn": False})
    assert res2["robot"]["configuration"] is None and ", S " not in res2["files"]["C.src"]
