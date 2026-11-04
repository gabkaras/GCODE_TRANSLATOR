import math

import numpy as np

from gcode2krl.gcode_parser import (ArcMove, Comment, Coolant, Dwell, LinearMove, Pause, ProgramEnd, Spindle,
                                    ToolChange, parse_gcode)


def moves(res):
    return [c for c in res.commands if isinstance(c, (LinearMove, ArcMove))]


def test_modal_motion_and_feed():
    res = parse_gcode("G21 G90\nG1 X10 F500\nY20\nX0 Y0 F800\n")
    m = moves(res)
    assert len(m) == 3
    assert all(isinstance(x, LinearMove) and not x.rapid for x in m)
    assert np.allclose(m[1].end, [10, 20, 0])
    assert m[0].feed == 500 and m[2].feed == 800
    assert np.allclose(m[2].start, m[1].end)


def test_rapid_incremental_inches_and_comments():
    res = parse_gcode("G20 G91 (inch incremental)\nG0 X1 ; half\nG0 X1\n")
    m = moves(res)
    assert len(m) == 2 and all(x.rapid for x in m)
    assert np.allclose(m[1].end, [50.8, 0, 0])
    assert any(isinstance(c, Comment) and c.text == "inch incremental" for c in res.commands)
    assert any(isinstance(c, Comment) and c.text == "half" for c in res.commands)


def test_compact_words_without_spaces():
    res = parse_gcode("G1X12.5Y-3F100\n")
    m = moves(res)
    assert np.allclose(m[0].end, [12.5, -3, 0])


def test_arc_ij_center_and_direction():
    res = parse_gcode("G90 G17\nG0 X10 Y0\nG3 X0 Y10 I-10 J0 F300\n")
    arc = [c for c in res.commands if isinstance(c, ArcMove)][0]
    assert arc.ccw
    assert np.allclose(arc.center, [0, 0, 0])
    assert np.allclose(arc.normal, [0, 0, 1])
    assert not arc.helical


def test_arc_absolute_center_mode():
    res = parse_gcode("G90.1\nG0 X10 Y0\nG2 X0 Y10 I0 J0\n")
    arc = [c for c in res.commands if isinstance(c, ArcMove)][0]
    assert np.allclose(arc.center, [0, 0, 0]) and not arc.ccw


def test_arc_radius_format_minor_and_major():
    res = parse_gcode("G0 X10 Y0\nG3 X0 Y10 R10\nG0 X10 Y0\nG3 X0 Y10 R-10\n")
    arcs = [c for c in res.commands if isinstance(c, ArcMove)]
    assert np.allclose(arcs[0].center, [0, 0, 0], atol=1e-9)
    assert np.allclose(arcs[1].center, [10, 10, 0], atol=1e-9)


def test_full_circle_and_helix():
    res = parse_gcode("G0 X10 Y0\nG2 I-10 J0\nG2 X10 Y0 Z-5 I-10 J0\n")
    arcs = [c for c in res.commands if isinstance(c, ArcMove)]
    assert len(arcs) == 2
    assert np.allclose(arcs[0].start, arcs[0].end)
    assert arcs[1].helical


def test_arc_in_g18_plane():
    res = parse_gcode("G18\nG0 X10 Z0\nG2 X0 Z10 I-10 K0\n")
    arc = [c for c in res.commands if isinstance(c, ArcMove)][0]
    assert np.allclose(arc.normal, [0, 1, 0])
    assert np.allclose(arc.center, [0, 0, 0])


def test_radius_mismatch_is_reconciled_with_warning():
    res = parse_gcode("G0 X10 Y0\nG3 X0 Y10.2 I-10 J0\n")
    arc = [c for c in res.commands if isinstance(c, ArcMove)][0]
    r1 = np.linalg.norm(arc.start[:2] - arc.center[:2])
    r2 = np.linalg.norm(arc.end[:2] - arc.center[:2])
    assert abs(r1 - r2) < 1e-9
    assert any("mismatch" in w for w in res.warnings)


def test_events():
    res = parse_gcode("M3 S8000\nG4 P1.5\nM8\nT2 M6\nM0\nM5 M9\nM30\n")
    kinds = [type(c).__name__ for c in res.commands]
    assert kinds == ["Spindle", "Dwell", "Coolant", "ToolChange", "Pause", "Spindle", "Coolant", "ProgramEnd"]
    sp = res.commands[0]
    assert isinstance(sp, Spindle) and sp.on and sp.rpm == 8000 and not sp.ccw
    assert isinstance(res.commands[1], Dwell) and res.commands[1].seconds == 1.5
    assert isinstance(res.commands[3], ToolChange) and res.commands[3].tool == 2
    assert isinstance(res.commands[4], Pause) and not res.commands[4].optional
    assert isinstance(res.commands[5], Spindle) and not res.commands[5].on


def test_dwell_ms_option():
    res = parse_gcode("G4 P500\n", dwell_in_ms=True)
    assert res.commands[0].seconds == 0.5


def test_unsupported_codes_warn_and_do_not_move():
    res = parse_gcode("G28 X0 Y0\nG92 X0\nG81 X1 Y1 Z-1 R2\nG1 X5 F100\n")
    assert len(moves(res)) == 1
    assert len(res.warnings) == 3


def test_counts():
    res = parse_gcode("G0 X1\nG1 X2\nG1 X3\nG2 X4 Y1 R1\nG3 X5 Y0 R1\n")
    assert res.counts == {"G0": 1, "G1": 2, "G2": 1, "G3": 1}
