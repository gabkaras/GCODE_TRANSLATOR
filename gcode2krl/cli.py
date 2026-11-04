"""Command line interface: gcode2krl input.nc [options]"""
from __future__ import annotations

import argparse
import os
import sys

from . import __version__
from .gcode_parser import parse_gcode
from .krl_writer import KrlOptions, write_krl
from .translator import TranslateOptions, translate


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="gcode2krl",
                                description="Translate G-code (G0/G1/G2/G3) into KUKA KRL, reconstructing arcs from G1 polylines.")
    p.add_argument("input", help="G-code file (.nc, .gcode, .tap, ...)")
    p.add_argument("-o", "--out-dir", default=".", help="output directory for .src/.dat files (default: .)")
    p.add_argument("-n", "--name", default=None, help="KRL program name (default: derived from input file name)")
    p.add_argument("--version", action="version", version=f"gcode2krl {__version__}")

    g = p.add_argument_group("arc reconstruction")
    g.add_argument("--tol", type=float, default=0.05, help="max deviation between original polyline and emitted path, mm (default 0.05)")
    g.add_argument("--max-radius", type=float, default=5000.0, help="largest arc radius accepted, mm (default 5000)")
    g.add_argument("--min-radius", type=float, default=0.1, help="smallest arc radius accepted, mm (default 0.1)")
    g.add_argument("--min-arc-points", type=int, default=4, help="minimum polyline vertices per arc (default 4)")
    g.add_argument("--refine-shift", type=int, default=3, help="break-point refinement range in vertices, 0 disables (default 3)")
    g.add_argument("--no-arcs", action="store_true", help="do not reconstruct arcs (lines/SPL only)")
    g.add_argument("--no-spl", action="store_true", help="never emit SPL points, keep residual micro-segments as SLIN/LIN")
    g.add_argument("--spl-max-segment", type=float, default=3.0, help="two-vertex lines up to this length become SPL points, mm (default 3)")
    g.add_argument("--corner-angle", type=float, default=10.0, help="turning angle treated as a corner, deg (default 10)")

    k = p.add_argument_group("KRL output")
    k.add_argument("--mode", choices=["spline", "lin"], default="spline", help="SPLINE blocks (SLIN/SCIRC/SPL) or LIN/CIRC (default spline)")
    k.add_argument("--tool", type=int, default=1, help="TOOL_DATA index (default 1)")
    k.add_argument("--base", type=int, default=1, help="BASE_DATA index (default 1)")
    k.add_argument("--abc", type=float, nargs=3, default=(0.0, 180.0, 0.0), metavar=("A", "B", "C"),
                   help="constant tool orientation in degrees (default 0 180 0: tool Z pointing down in BASE)")
    k.add_argument("--offset", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("DX", "DY", "DZ"),
                   help="translation added to all G-code coordinates, mm")
    k.add_argument("--default-feed", type=float, default=1000.0, help="feed used when G-code sets no F, mm/min (default 1000)")
    k.add_argument("--rapid-vel", type=float, default=0.5, help="velocity for G0 moves, m/s (default 0.5)")
    k.add_argument("--max-vel", type=float, default=2.0, help="cap for $VEL.CP, m/s (default 2.0)")
    k.add_argument("--acc", type=float, default=2.0, help="$ACC.CP, m/s^2 (default 2.0)")
    k.add_argument("--cdis", type=float, default=0.2, help="$APO.CDIS approximation distance for LIN/CIRC, mm (default 0.2)")
    k.add_argument("--round-corners", action="store_true", help="lin mode: approximate (C_DIS) even at corners")
    k.add_argument("--max-block-segments", type=int, default=500, help="max segments per SPLINE block (default 500)")
    k.add_argument("--max-motions-per-file", type=int, default=4000, help="split into sub-programs above this many motions (default 4000)")
    k.add_argument("--no-block-approx", action="store_true", help="omit C_SPL after ENDSPLINE (older KSS versions)")
    k.add_argument("--no-bco", action="store_true", help="do not emit 'PTP $POS_ACT' before the first motion")
    k.add_argument("--advance", type=int, default=3, help="$ADVANCE (default 3)")
    k.add_argument("--spindle-out", type=int, default=0, help="digital output for spindle on/off, 0 = comment only")
    k.add_argument("--spindle-anout", type=int, default=0, help="analog output for spindle speed, 0 = none")
    k.add_argument("--spindle-max-rpm", type=float, default=24000.0, help="rpm that maps to analog output 1.0")
    k.add_argument("--coolant-out", type=int, default=0, help="digital output for coolant, 0 = comment only")
    k.add_argument("--no-halt-tool-change", action="store_true", help="do not HALT at M6")
    k.add_argument("--dwell-ms", action="store_true", help="interpret G4 P as milliseconds instead of seconds")
    k.add_argument("--start", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"),
                   help="assumed tool position before the first G-code move (default 0 0 0)")
    r = p.add_argument_group("robot check")
    r.add_argument("--robot", default="none", help="robot model for the reach/axis-limit check, e.g. KR90R2700PRO (default none; 'list' prints models)")
    r.add_argument("--base-frame", type=float, nargs=6, default=None, metavar=("X", "Y", "Z", "A", "B", "C"),
                   help="$BASE in robot coordinates (also written literally into the program)")
    r.add_argument("--tool-frame", type=float, nargs=6, default=None, metavar=("X", "Y", "Z", "A", "B", "C"),
                   help="$TOOL in flange coordinates (also written literally into the program)")
    r.add_argument("--robot-samples", type=int, default=3000, help="max path samples for the IK check (default 3000)")
    p.add_argument("-q", "--quiet", action="store_true", help="no report on stdout")
    return p


def derive_name(path: str) -> str:
    base = os.path.splitext(os.path.basename(path))[0]
    name = "".join(c if c.isalnum() else "_" for c in base).upper()
    if not name or not name[0].isalpha():
        name = "P_" + name
    return name[:20]


def run(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.robot.lower() == "list":
        from .robot import ROBOTS
        for r in ROBOTS.values():
            print(f"{r.key:18s} {r.name}  reach {r.reach:g} mm, payload {r.payload:g} kg")
        return 0
    with open(args.input, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    parsed = parse_gcode(text, start=tuple(args.start), dwell_in_ms=args.dwell_ms)
    topts = TranslateOptions(tol=args.tol, max_radius=args.max_radius, min_radius=args.min_radius,
                             min_arc_points=args.min_arc_points, refine_shift=args.refine_shift,
                             spl_max_segment=args.spl_max_segment, corner_angle_deg=args.corner_angle,
                             default_feed=args.default_feed, offset=tuple(args.offset),
                             fit_arcs=not args.no_arcs, use_spl=not args.no_spl)
    items, stats = translate(parsed, topts)
    kopts = KrlOptions(name=args.name or derive_name(args.input), mode=args.mode, tool=args.tool, base=args.base,
                       abc=tuple(args.abc), rapid_vel=args.rapid_vel, max_vel=args.max_vel, acc=args.acc,
                       cdis=args.cdis, corner_angle_deg=args.corner_angle, exact_corners=not args.round_corners,
                       max_block_segments=args.max_block_segments, max_motions_per_file=args.max_motions_per_file,
                       block_approx=not args.no_block_approx, spindle_out=args.spindle_out,
                       coolant_out=args.coolant_out, spindle_anout=args.spindle_anout,
                       spindle_max_rpm=args.spindle_max_rpm, bco_ptp=not args.no_bco, advance=args.advance,
                       source_name=os.path.basename(args.input), halt_on_tool_change=not args.no_halt_tool_change,
                       base_frame=tuple(args.base_frame) if args.base_frame else None,
                       tool_frame=tuple(args.tool_frame) if args.tool_frame else None)
    prog = write_krl(items, kopts, stats)
    robot_rep = None
    if args.robot.lower() != "none":
        from .robot import ROBOTS, check_path, frame
        from .translator import sample_items
        robot = ROBOTS.get(args.robot.upper())
        if robot is None:
            print(f"unknown robot {args.robot}; use --robot list", file=sys.stderr)
            return 2
        pts = sample_items(items, max_samples=args.robot_samples)
        robot_rep = check_path(robot, pts, kopts.abc, frame(args.base_frame or (0,) * 6), frame(args.tool_frame or (0,) * 6))
    os.makedirs(args.out_dir, exist_ok=True)
    for fname, content in prog.files.items():
        with open(os.path.join(args.out_dir, fname), "w", encoding="utf-8", newline="\r\n") as fh:
            fh.write(content)
    if not args.quiet:
        ic = stats.input_counts
        n_in = sum(ic.values())
        print(f"input : {n_in} moves (G0={ic.get('G0',0)} G1={ic.get('G1',0)} G2={ic.get('G2',0)} G3={ic.get('G3',0)}), "
              f"{parsed.n_lines} lines")
        print(f"output: {stats.motions_out} motions (lines={stats.lines_out} arcs={stats.arcs_out} "
              f"spl points={stats.spl_points} in {stats.spl_runs} runs, rapids={stats.rapids})")
        ratio = (n_in / stats.motions_out) if stats.motions_out else 0.0
        print(f"compression {ratio:.1f}x, max deviation {stats.max_dev:.4f} mm (tolerance {args.tol:g})")
        print("files : " + ", ".join(os.path.join(args.out_dir, f) for f in prog.files))
        if robot_rep is not None:
            print(f"robot : {args.robot.upper()} - {robot_rep.n_samples} samples, {robot_rep.n_unreachable} out of reach, "
                  f"{robot_rep.n_limit} beyond axis limits")
            for k in range(6):
                lo, hi = robot_rep.joint_min[k], robot_rep.joint_max[k]
                if lo != float("inf"):
                    print(f"        A{k + 1}: {lo:8.2f} .. {hi:8.2f} deg" + (f"  ({robot_rep.axis_violations[k]} violations)" if robot_rep.axis_violations[k] else ""))
        for w in stats.warnings:
            print("warning: " + w, file=sys.stderr)
    return 0 if (robot_rep is None or (robot_rep.n_unreachable == 0 and robot_rep.n_limit == 0)) else 3


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
