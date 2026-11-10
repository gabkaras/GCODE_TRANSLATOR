import math
import os
import re

import numpy as np

from gcode2krl.cli import run
from gcode2krl.gcode_parser import parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.translator import ArcItem, EventItem, LineItem, RapidItem, SplItem, TranslateOptions, translate

HERE = os.path.dirname(__file__)
EXAMPLES = os.path.join(HERE, "..", "examples")


def circle_gcode(n=72, r=25.0, cx=50.0, cy=50.0):
    lines = ["G21 G90", "G0 X%.3f Y%.3f" % (cx + r, cy), "G1 Z-1 F500"]
    for k in range(1, n + 1):
        a = 2 * math.pi * k / n
        lines.append("G1 X%.3f Y%.3f" % (cx + r * math.cos(a), cy + r * math.sin(a)))
    lines.append("M30")
    return "\n".join(lines) + "\n"


def test_translate_circle_to_two_scirc():
    items, stats = translate(parse_gcode(circle_gcode()), TranslateOptions(tol=0.05))
    arcs = [i for i in items if isinstance(i, ArcItem)]
    assert len(arcs) == 2 and stats.arcs_out == 2
    assert np.allclose(arcs[0].start, [75, 50, -1]) and np.allclose(arcs[1].end, [75, 50, -1])
    assert np.allclose(arcs[0].end, [25, 50, -1], atol=1e-3)
    assert isinstance(items[-1], EventItem) and items[-1].kind == "end"


def test_spline_block_syntax():
    items, stats = translate(parse_gcode(circle_gcode()), TranslateOptions(tol=0.05))
    prog = write_krl(items, KrlOptions(name="CIRC_T"), stats)
    src = prog.files["CIRC_T.src"]
    assert src.startswith("&ACCESS RVP\n&REL 1\n")
    assert "DEF CIRC_T( )" in src and src.rstrip().endswith("END")
    assert "BAS (#INITMOV,0 )" in src
    assert "$TOOL = TOOL_DATA[1]" in src and "$BASE = BASE_DATA[1]" in src
    assert "PTP XHOME" in src  # HOME move is the BCO run by default
    # plunge (own block, corner) + the two half circles in one block; the last block ends
    # without C_SPL because PTP HOME / END follow (KSS: approximation inadmissible)
    assert src.count("SPLINE WITH $VEL.CP = 0.0083, $ACC.CP = 2.00") == 2
    assert src.count("ENDSPLINE C_SPL") == 1 and src.count("ENDSPLINE") == 2
    assert len(re.findall(r"^  SCIRC \{X [-\d.]+, Y [-\d.]+, Z [-\d.]+, A [-\d.]+, B [-\d.]+, C [-\d.]+\}, \{X ", src, re.M)) == 2
    assert "LIN {X 75.000, Y 50.000, Z 0.000, A 0.000, B 180.000, C 0.000}" in src  # rapid
    assert prog.files["CIRC_T.dat"].startswith("&ACCESS RVP\n&REL 1\nDEFDAT CIRC_T\n")


def test_spline_tangency_bridges_and_no_approx_before_statements():
    # a long line into an arc whose start tangent is 3 deg off the line: KSS would stop at the
    # SLIN-SCIRC junction ("velocity 0 in point"), so the writer ends the line with an SPL bridge
    pts = [(0.0, 0.0)]
    pts += [(x, 0.0) for x in (20.0, 40.0, 60.0)]
    r, k = 30.0, math.radians(3.0)
    cx, cy = 60.0 + r * math.sin(k), r * math.cos(k)  # circle tangent at the junction tilted by 3 deg
    for i in range(1, 25):
        a = -math.pi / 2 - k + i * 0.05
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    g = "G1 Z0 F600\n" + "\n".join(f"G1 X{x:.4f} Y{y:.4f}" for x, y in pts) + "\nM30\n"
    items, stats = translate(parse_gcode(g), TranslateOptions(tol=0.05))
    kinds = [type(i).__name__ for i in items if type(i).__name__ in ("LineItem", "ArcItem", "SplItem")]
    assert kinds[:2] == ["LineItem", "ArcItem"], kinds
    src = write_krl(items, KrlOptions(name="T", home_start=False, home_end=False), stats).files["T.src"]
    seq = [w for w in (l.strip().split()[0] for l in src.splitlines() if l.strip()) if w in ("SLIN", "SCIRC", "SPL")]
    assert seq[:3] == ["SLIN", "SPL", "SCIRC"], seq
    # the SPL bridge ends exactly at the junction (60, 0); the SLIN stops shortly before it
    lines = [l.strip() for l in src.splitlines() if l.strip().split()[0:1] and l.strip().split()[0] in ("SLIN", "SPL")]
    assert "SPL {X 60.000, Y 0.000" in lines[1] and "SLIN {X 5" in lines[0]
    # a statement after the block (extruder output off) forbids the C_SPL approximation
    g2 = "G1 Z0 F600\nG1 X50 E5\nG1 X100 E10\nG1 X100 Y50 E15\nG4 P500\nG1 X0 E20\nM30\n"
    items, stats = translate(parse_gcode(g2), TranslateOptions(tol=0.05, extruder=True))
    src = write_krl(items, KrlOptions(name="T", extruder_out=5, extruder_trigger=False, home_end=False), stats).files["T.src"]
    pos = src.index("WAIT SEC")
    before = [l for l in src[:pos].splitlines() if l.startswith("ENDSPLINE")]
    assert before and before[-1] == "ENDSPLINE", before


def test_lint_catches_kss_spline_problems():
    from gcode2krl.krl_lint import lint_krl
    bad = "\n".join([
        "LIN {X 0.000, Y 0.000, Z 0.000, A 0.000, B 180.000, C 0.000}",
        "SPLINE WITH $VEL.CP = 0.01, $ACC.CP = 2.00",
        "  SLIN {X 100.000, Y 0.000, Z 0.000, A 0.000, B 180.000, C 0.000}",
        "  SLIN {X 200.000, Y 5.000, Z 0.000, A 0.000, B 180.000, C 0.000}",   # 2.9 deg kink
        "  SLIN {X 200.000, Y 5.000, Z 0.000, A 0.000, B 180.000, C 0.000}",   # repeated point
        "ENDSPLINE C_SPL",
        "WAIT SEC 1.0",                                                        # approximation into a WAIT
        "END"])
    msgs = [m for _, m in lint_krl(bad)]
    assert any("SLIN-SLIN" in m for m in msgs) and any("powtorzony" in m for m in msgs) and any("C_SPL" in m for m in msgs)
    good = bad.replace("ENDSPLINE C_SPL", "ENDSPLINE").replace("  SLIN {X 200.000, Y 5.000, Z 0.000, A 0.000, B 180.000, C 0.000}\n  SLIN", "  SPL {X 200.000, Y 5.000, Z 0.000, A 0.000, B 180.000, C 0.000}\n  SLIN {X 250.000, Y 5.000, Z 0.000, A 0.000, B 180.000, C 0.000}\n  SLIN")
    assert [m for _, m in lint_krl(good) if "C_SPL" in m] == []


def test_leading_z_rapid_is_folded_into_the_first_xy_rapid():
    # "G0 Z2" from the assumed X0 Y0, then "G0 X.. Y..": the robot comes from HOME and must go
    # straight to the first point, not to the base origin (a table corner) first
    g = "G0 Z2 F9000\nG0 X350 Y360\nG1 X400 F900\nG1 Y400\nM30\n"
    items, stats = translate(parse_gcode(g), TranslateOptions())
    rapids = [i for i in items if isinstance(i, RapidItem)]
    assert len(rapids) == 1 and np.allclose(rapids[0].target, [350, 360, 2]) and np.allclose(rapids[0].start, rapids[0].target)
    src = write_krl(items, KrlOptions(name="T"), stats).files["T.src"]
    assert "X 0.000, Y 0.000" not in src
    # a program that really starts cutting from X0 Y0 keeps its Z move
    items, _ = translate(parse_gcode("G0 Z5\nG1 X10 Y10 F600\nM30\n"), TranslateOptions())
    assert [type(i).__name__ for i in items][:2] == ["RapidItem", "LineItem"]


def test_frame_modes_declare_data_literal():
    items, stats = translate(parse_gcode("G1 X0 Y0 Z0 F600\nG1 X10 Y0\nG1 X10 Y10\n"), TranslateOptions())
    frames = dict(tool_frame=(0, 0, 150, 0, 45, 0), base_frame=(1400, 0, 500, 0, 0, 0))
    src = write_krl(items, KrlOptions(name="T", **frames), stats).files["T.src"]  # default: declare
    assert "TOOL_DATA[1] = {X 0.000, Y 0.000, Z 150.000, A 0.000, B 45.000, C 0.000}" in src
    assert "BASE_DATA[1] = {X 1400.000, Y 0.000, Z 500.000, A 0.000, B 0.000, C 0.000}" in src
    assert src.index("TOOL_DATA[1] = {") < src.index("$TOOL = TOOL_DATA[1]") < src.index("$BASE = BASE_DATA[1]")
    src = write_krl(items, KrlOptions(name="T", frame_mode="data", **frames), stats).files["T.src"]
    assert "TOOL_DATA[1] = {" not in src and "$TOOL = TOOL_DATA[1]" in src and "$BASE = BASE_DATA[1]" in src
    src = write_krl(items, KrlOptions(name="T", frame_mode="literal", **frames), stats).files["T.src"]
    assert "$TOOL = {X 0.000, Y 0.000, Z 150.000, A 0.000, B 45.000, C 0.000}" in src and "TOOL_DATA[1]" not in src
    # no values known: only the selection of the configured data sets
    src = write_krl(items, KrlOptions(name="T"), stats).files["T.src"]
    assert "TOOL_DATA[1] = {" not in src and "$TOOL = TOOL_DATA[1]" in src


def test_lin_mode_syntax_and_exact_corners():
    text = "G1 X10 F600\nG1 Y10\nG1 X20 Y20\nM30\n"
    items, stats = translate(parse_gcode(text), TranslateOptions(use_spl=False, fit_arcs=False))
    src = write_krl(items, KrlOptions(name="L", mode="lin", cdis=0.3), stats).files["L.src"]
    body = [l for l in src.splitlines() if l.startswith("LIN")]
    assert body[0].endswith("}")  # 90 degree corner -> exact
    assert body[1].endswith("}")  # 45 degree corner -> exact
    assert body[2].endswith("}")  # last motion
    assert "$APO.CDIS = 0.30" in src and "$VEL.CP = 0.0100" in src
    assert "SPLINE" not in src


def test_lin_mode_rounds_collinear():
    text = "G1 X10 F600\nG1 X20\nG1 X30 Y0.001\nM30\n"
    items, stats = translate(parse_gcode(text), TranslateOptions(use_spl=False, fit_arcs=False))
    src = write_krl(items, KrlOptions(name="L", mode="lin"), stats).files["L.src"]
    body = [l for l in src.splitlines() if l.startswith("LIN")]
    assert body[0].endswith("C_DIS") and body[1].endswith("C_DIS")


def test_events_and_outputs():
    text = "M3 S12000\nM8\nG1 X5 F100\nG4 P2\nT3 M6\nM0\nM5\nM9\nM30\n"
    items, stats = translate(parse_gcode(text), TranslateOptions())
    src = write_krl(items, KrlOptions(name="EV", spindle_out=5, coolant_out=6, spindle_anout=1, spindle_max_rpm=24000), stats).files["EV.src"]
    assert "$OUT[5] = TRUE" in src and "$OUT[5] = FALSE" in src
    assert "$ANOUT[1] = 0.5000" in src
    assert "$OUT[6] = TRUE" in src and "$OUT[6] = FALSE" in src
    assert "WAIT SEC 2.000" in src
    assert "; tool change T3" in src and src.count("HALT") == 2


def test_file_splitting_and_subprogram_calls():
    lines = ["G1 F600"] + [f"G1 X{k} Y{(k % 2) * 5}" for k in range(1, 301)] + ["M30"]
    items, stats = translate(parse_gcode("\n".join(lines)), TranslateOptions(use_spl=False, fit_arcs=False))
    prog = write_krl(items, KrlOptions(name="BIG", max_motions_per_file=100, max_block_segments=40), stats)
    names = sorted(prog.files)
    assert "BIG.src" in names and "BIG_001.src" in names and "BIG_001.dat" in names
    main = prog.files["BIG.src"]
    subs = [n for n in names if re.match(r"BIG_\d{3}\.src", n)]
    assert len(subs) >= 3
    for s in subs:
        assert f"{s[:-4]}( )" in main
        body = prog.files[s]
        assert body.count("SPLINE WITH") == body.count("ENDSPLINE")
        assert body.rstrip().endswith("END")


def test_spl_grouping_for_wiggly_micro_segments():
    rng = np.random.default_rng(5)
    x = np.linspace(0, 20, 81)
    y = 2 * np.sin(x / 3.0) + rng.uniform(-0.03, 0.03, 81)  # variable curvature + jitter above tolerance
    text = "G1 F500\n" + "\n".join(f"G1 X{a:.3f} Y{b:.3f}" for a, b in zip(x, y)) + "\nM30\n"
    items, stats = translate(parse_gcode(text), TranslateOptions(tol=0.02, fit_arcs=False, corner_angle_deg=45))
    spl = [i for i in items if isinstance(i, SplItem)]
    assert len(spl) == 1 and stats.spl_points == len(spl[0].points) - 1
    assert 3 <= len(spl[0].points) < 81  # Douglas-Peucker thinned the run
    assert np.allclose(spl[0].points[0], [0, y[0], 0], atol=1e-3) and np.allclose(spl[0].points[-1], [20, y[-1], 0], atol=1e-3)
    src = write_krl(items, KrlOptions(name="W", corner_angle_deg=45), stats).files["W.src"]
    assert src.count("  SPL {X") == stats.spl_points
    # isolated two-vertex lines between arcs stay SLIN
    items2, _ = translate(parse_gcode(text), TranslateOptions(tol=0.02, max_radius=50))
    assert any(isinstance(i, ArcItem) for i in items2)


def test_cli_end_to_end(tmp_path):
    out = tmp_path / "krl"
    rc = run([os.path.join(EXAMPLES, "fillet_rect.nc"), "-o", str(out), "-q", "--spindle-out", "3"])
    assert rc == 0
    src = (out / "FILLET_RECT.src").read_text()
    assert src.count("SCIRC") == 4 and src.count("SLIN") == 5
    assert b"\r\n" in (out / "FILLET_RECT.src").read_bytes()
    rc = run([os.path.join(EXAMPLES, "mixed.nc"), "-o", str(out), "-q", "--mode", "lin", "-n", "MIXED_L"])
    assert rc == 0 and (out / "MIXED_L.src").exists()


def test_cli_name_validation(tmp_path):
    import pytest
    with pytest.raises(ValueError):
        run([os.path.join(EXAMPLES, "circle_g1.nc"), "-o", str(tmp_path), "-q", "-n", "1bad-name"])
