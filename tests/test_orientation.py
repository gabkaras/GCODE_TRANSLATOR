import math

import numpy as np
import pytest

from gcode2krl.gcode_parser import LinearMove, parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.robot import ROBOTS, abc_from_axis, abc_to_rot, check_path, frame, solve_spins
from gcode2krl.translator import (ArcItem, EventItem, LineItem, RapidItem, SplItem, TranslateOptions, has_orientation,
                                  resolve_orientations, sample_items, translate)


def test_abc_from_axis_conventions():
    assert abc_from_axis((0, 0, 1)) == (0.0, 180.0, 0.0)
    r = abc_to_rot(*abc_from_axis((1, 0, 1)))
    assert np.allclose(r[:, 2], [-1, 0, -1] / np.sqrt(2))  # tool Z points into the part (opposite of the axis)
    r2 = abc_to_rot(*abc_from_axis((0, 0, 1), 90))
    assert np.allclose(r2[:, 2], [0, 0, -1])  # spin keeps the axis
    assert not np.allclose(r2[:, 0], abc_to_rot(0, 180, 0)[:, 0])  # but rotates X about it


def test_parser_orientation_words():
    res = parse_gcode("G1 X10 Y0 A0 B170 C0 F600\nG1 X20\nG1 X30 I0 J0.1 K1\nG0 X40\nG2 X50 Y10 I0 J10 B160\n")
    mv = [c for c in res.commands if isinstance(c, LinearMove)]
    assert mv[0].abc == (0, 170, 0) and mv[1].abc == (0, 170, 0) and mv[1].axis is None
    assert mv[2].abc is None and np.allclose(mv[2].axis, [0, 0.1 / np.hypot(0.1, 1), 1 / np.hypot(0.1, 1)])
    assert mv[3].axis == mv[2].axis  # modal through G0
    arc = res.commands[-1]
    assert arc.abc == (0, 160, 0) and arc.axis is None  # G2 I J are the arc centre, A B C the orientation


def test_translate_axis_chain_and_mixed_chain():
    g = ("G1 X0 Y0 Z0 I0 J0 K1 F600\nG1 X10 I0.1 J0 K1\nG1 X20 I0.2 J0 K1\nG0 X20 Y10\n"
         "G1 X30 Y10 A0 B170 C0\nG1 X40 Y10 A0 B160 C0\nG1 X50 Y10\nM30\n")
    items, st = translate(parse_gcode(g), TranslateOptions())
    spl = [i for i in items if isinstance(i, SplItem)]
    assert len(spl) == 2 and has_orientation(items)
    assert spl[0].axes is not None and spl[0].abcs.shape == (3, 3)
    assert spl[1].axes is None and np.allclose(spl[1].abcs[1:], [[0, 170, 0], [0, 160, 0], [0, 160, 0]])
    rapid = [i for i in items if isinstance(i, RapidItem)][0]
    assert rapid.abc is not None  # modal axis carried through the G0
    assert st.oriented_vertices == 7


def test_constant_orientation_chain_still_fits_arcs():
    a = np.linspace(0, math.pi, 91)
    g = "G1 X100 Y0 A0 B170 C0 F600\n" + "\n".join(f"G1 X{100 * math.cos(t):.3f} Y{100 * math.sin(t):.3f}" for t in a[1:]) + "\nM30\n"
    items, st = translate(parse_gcode(g), TranslateOptions(tol=0.05))
    arcs = [i for i in items if isinstance(i, ArcItem)]
    assert arcs and all(np.allclose(i.abc, (0, 170, 0)) for i in arcs)


def test_sample_items_interpolates_orientation():
    items, _ = translate(parse_gcode("G1 X0 Y0 Z0 A0 B180 C0 F600\nG1 X100 A0 B160 C0\nM30\n"), TranslateOptions())
    pts, abcs = sample_items(items, step=25, with_abc=True)
    assert len(pts) == 4
    # compare rotations (Euler triples are not unique around B = 180 deg)
    for abc, b in zip(abcs, [175, 170, 165, 160]):
        assert np.allclose(abc_to_rot(*abc), abc_to_rot(0, b, 0), atol=1e-9)


def test_solve_spins_feasible_and_smooth():
    r = ROBOTS["KR90R2700PRO"]
    base = frame((1400, 0, 500, 0, 0, 0))
    tool = frame((0, 0, 150, 0, 0, 0))
    pts = np.array([[x, 0.0, 50.0] for x in np.linspace(-200, 200, 41)])
    tilt = np.radians(30 * np.sin(pts[:, 0] / 100.0))
    axes = np.column_stack([np.sin(tilt), np.zeros(41), np.cos(tilt)])
    abc, info = solve_spins(r, pts, axes, base, tool, singular_deg=8.0)
    rep = check_path(r, pts, abc, base, tool)
    assert rep.n_unreachable == 0 and rep.n_limit == 0 and info["unreachable"] == 0
    assert rep.joint_min[4] > 8.0 or rep.joint_max[4] < -8.0  # away from the wrist singularity
    # every chosen orientation still points the tool along the requested axis
    for a, ax in zip(abc, axes):
        assert np.allclose(abc_to_rot(*a)[:, 2], -ax, atol=1e-6)
    # a sideways tool axis forces spins to be re-sampled but stays reachable
    axes2 = np.tile([1.0, 0.0, 0.0], (len(pts), 1))
    abc2, info2 = solve_spins(r, pts, axes2, base, tool)
    assert check_path(r, pts, abc2, base, tool).n_unreachable == 0


def test_krl_with_orientation_and_analog_extruder():
    g = "G1 X0 Y0 Z0 I0 J0 K1 F600 E0\nG1 X10 I0.1 J0 K1 E1\nG1 X20 I0.2 J0 K1 E2\nG0 X20 Y10\nG1 X30 Y10 E3\nM30\n"
    items, st = translate(parse_gcode(g), TranslateOptions(extruder=True, extruder_analog=True))
    r = ROBOTS["KR90R2700PRO"]
    info = resolve_orientations(items, r, frame((1400, 0, 500, 0, 0, 0)), frame((0, 0, 150, 0, 0, 0)))
    assert info["points"] == 3 and info["unreachable"] == 0
    src = write_krl(items, KrlOptions(name="O", ori_var=has_orientation(items), extruder_anout=2, extruder_flow_full=50.0), st).files["O.src"]
    assert "$ORI_TYPE = #VAR" in src and "$VEL_FLT_OFF = TRUE" in src
    assert "ANOUT ON $ANOUT[2] = 2.00000 * $VEL_ACT" in src and "ANOUT OFF $ANOUT[2]" in src  # 0.1 E/mm * 1000 / 50
    spl_lines = [l for l in src.splitlines() if l.strip().startswith("SPL ")]
    assert len(spl_lines) == 2 and "B -5.711" in spl_lines[0]
    ev = [i.kind for i in items if isinstance(i, EventItem)]
    assert ev.count("extruder_rate") == 1
