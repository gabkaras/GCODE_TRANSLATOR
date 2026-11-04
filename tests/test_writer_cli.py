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
    assert "PTP $POS_ACT" in src
    # plunge (own block, corner) + the two half circles in one block
    assert src.count("SPLINE WITH $VEL.CP = 0.0083, $ACC.CP = 2.00") == 2
    assert src.count("ENDSPLINE C_SPL") == 2
    assert len(re.findall(r"^  SCIRC \{X [-\d.]+, Y [-\d.]+, Z [-\d.]+, A [-\d.]+, B [-\d.]+, C [-\d.]+\}, \{X ", src, re.M)) == 2
    assert "LIN {X 75.000, Y 50.000, Z 0.000, A 0.000, B 180.000, C 0.000}" in src  # rapid
    assert prog.files["CIRC_T.dat"].startswith("&ACCESS RVP\n&REL 1\nDEFDAT CIRC_T\n")


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
