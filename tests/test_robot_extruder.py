"""Robot-native extruder control in KRL: TRIGGER switching, reverse pulse,
prime wait, analog rate kept across travels, user KRL lines."""
import numpy as np
import pytest

from gcode2krl.gcode_parser import ExtrudeOnly, parse_gcode
from gcode2krl.krl_writer import KrlOptions, write_krl
from gcode2krl.translator import EventItem, TranslateOptions, translate

G = """G21
G90
M82
G92 E0
G1 F1200
G1 X0 Y0 Z2
G1 X50 Y0 E100
G1 X50 Y50 E200
G1 E195
G0 X0 Y50
G1 E202
G1 X0 Y0 E302
M30
"""


def test_parser_emits_extrude_only_moves():
    res = parse_gcode(G)
    eo = [c for c in res.commands if isinstance(c, ExtrudeOnly)]
    assert [round(c.delta, 6) for c in eo] == [-5.0, 7.0]


def test_translator_events_and_trigger_krl():
    items, st = translate(parse_gcode(G), TranslateOptions(extruder=True, tol=0.05))
    ev = [i.kind for i in items if isinstance(i, EventItem)]
    assert ev.index("extruder_on") < ev.index("extruder_retract") < ev.index("extruder_prime")
    assert ev.count("extruder_on") == 2 and ev.count("extruder_off") == 2  # off at the travel, on again after the prime
    assert ev.index("extruder_off") < ev.index("end")
    src = write_krl(items, KrlOptions(name="T", extruder_out=7, extruder_reverse_out=8, extruder_flow_full=50.0,
                                      extruder_on_delay=-0.12, extruder_off_delay=0.03,
                                      krl_start="$OUT[9] = TRUE", krl_end="$OUT[9] = FALSE"), st).files["T.src"]
    lines = src.splitlines()
    on = [i for i, l in enumerate(lines) if l == "TRIGGER WHEN DISTANCE=0 DELAY=-120 DO $OUT[7]=TRUE PRIO=-1"]
    off = [i for i, l in enumerate(lines) if l == "TRIGGER WHEN PATH=0 DELAY=30 DO $OUT[7]=FALSE PRIO=-1"]
    assert len(on) == 1 and len(off) >= 1
    # the ON trigger directly precedes a SPLINE block, the OFF trigger too (it refers to the block end)
    assert lines[on[0] + 1].startswith("SPLINE") and all(lines[i + 1].startswith("SPLINE") for i in off)
    # reverse pulse: 5 mm^3 at 50 mm^3/s = 0.1 s
    k = lines.index("$OUT[8] = TRUE")
    assert lines[k + 1] == "WAIT SEC 0.10" and lines[k + 2] == "$OUT[8] = FALSE"
    # prime after the travel: output on and wait 7/50 = 0.14 s before the next spline
    kp = [i for i, l in enumerate(lines) if l == "WAIT SEC 0.14"]
    assert kp and lines[kp[0] - 1] == "$OUT[7] = TRUE"
    assert "$OUT[9] = TRUE" in lines and lines.index("$OUT[9] = FALSE") > max(i for i, l in enumerate(lines) if l.startswith("ENDSPLINE"))
    assert src.index("; user start") < src.index("PTP $POS_ACT")
    # no plain switching: only the prime turns the output on directly, every off is a trigger
    assert src.count("$OUT[7] = TRUE") == 1 and src.count("$OUT[7] = FALSE") == 0


def test_plain_out_when_trigger_disabled():
    items, st = translate(parse_gcode(G), TranslateOptions(extruder=True))
    src = write_krl(items, KrlOptions(name="P", extruder_out=7, extruder_trigger=False), st).files["P.src"]
    assert "TRIGGER" not in src and src.count("$OUT[7] = TRUE") >= 1 and src.count("$OUT[7] = FALSE") >= 1


def test_analog_rate_stays_on_across_travel_when_gated():
    items, st = translate(parse_gcode(G), TranslateOptions(extruder=True, extruder_analog=True))
    src = write_krl(items, KrlOptions(name="A", extruder_out=7, extruder_anout=2, extruder_flow_full=50.0), st).files["A.src"]
    assert src.count("ANOUT ON $ANOUT[2]") == 1 and src.count("ANOUT OFF $ANOUT[2]") == 1  # once on, off at the end
    assert src.index("ANOUT OFF") > src.rindex("ENDSPLINE")
    # analog only (no digital gate): the signal is switched off for the travel
    src2 = write_krl(items, KrlOptions(name="B", extruder_anout=2, extruder_flow_full=50.0), st).files["B.src"]
    assert src2.count("ANOUT ON $ANOUT[2]") == 2 and src2.count("ANOUT OFF $ANOUT[2]") == 2


def test_lin_mode_triggers_before_first_motion():
    items, st = translate(parse_gcode(G), TranslateOptions(extruder=True, use_spl=False, fit_arcs=False))
    src = write_krl(items, KrlOptions(name="L", mode="lin", extruder_out=7), st).files["L.src"]
    lines = src.splitlines()
    i = next(k for k, l in enumerate(lines) if l.startswith("TRIGGER WHEN DISTANCE=0"))
    assert lines[i + 1].startswith("LIN ")
    assert "$OUT[7] = FALSE" in src  # LIN mode falls back to a plain switch-off
