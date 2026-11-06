"""Command line interface: gcode2krl input.nc [options]"""
from __future__ import annotations

import argparse

import numpy as np
import os
import sys

from . import __version__
from .gcode_parser import parse_gcode
from .krl_writer import KrlOptions, write_krl
from .robot import ROBOTS
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
    k.add_argument("--extruder-out", type=int, default=-1, help="3D printing: digital output switched on for extruding moves (0 = comments only, -1 = ignore E)")
    k.add_argument("--extruder-anout", type=int, default=0, help="analog output driven proportionally to path velocity while extruding (ANOUT ON ... $VEL_ACT)")
    k.add_argument("--extruder-flow-full", type=float, default=100.0, help="E units per second at analog signal 1.0 (default 100)")
    k.add_argument("--extruder-delay", type=float, default=0.0, help="ANOUT DELAY in seconds (default 0)")
    k.add_argument("--extruder-on-delay", type=float, default=0.0, help="delay of the extruder ON trigger in s (negative = earlier)")
    k.add_argument("--extruder-off-delay", type=float, default=0.0, help="delay of the extruder OFF trigger in s")
    k.add_argument("--extruder-reverse-out", type=int, default=0, help="digital output running the extruder backwards for retraction (0 = none)")
    k.add_argument("--no-extruder-trigger", action="store_true", help="switch the extruder output with plain $OUT instead of TRIGGER (advance-run stop)")
    k.add_argument("--zones", default=None, help="file with safety zones (zone NAME allowed|forbidden tcp|robot robot|base cx cy cz sx sy sz [a b c])")
    k.add_argument("--no-zones-krl", action="store_true", help="do not write the zones as $WORKSPACE[n] into the program")
    k.add_argument("--no-home-start", action="store_true", help="do not start the program with PTP HOME")
    k.add_argument("--no-home-end", action="store_true", help="do not end the program with PTP HOME")
    k.add_argument("--home-mode", choices=("xhome", "axes"), default="xhome", help="HOME as PTP XHOME ($CONFIG.DAT) or a literal axis position (--home-axes)")
    k.add_argument("--home-axes", default="", help="A1..A6 in deg for --home-mode axes (default: the robot model's home)")
    k.add_argument("--home-vel", type=float, default=100.0, help="PTP velocity for the HOME moves in %% (default 100)")
    k.add_argument("--krl-start", default="", help="file with user KRL lines inserted after the settings")
    k.add_argument("--krl-end", default="", help="file with user KRL lines inserted before END")
    k.add_argument("--no-orient-solve", action="store_true", help="keep zero spin for I J K tool axes instead of the singularity-aware solver (needs --robot)")
    k.add_argument("--spin-step", type=float, default=15.0, help="orientation solver: spin sampling step, deg (default 15)")
    k.add_argument("--singular-deg", type=float, default=8.0, help="orientation solver: keep |A5| above this, deg (default 8)")
    k.add_argument("--max-tilt", type=float, default=0.0, help="orientation solver: allowed extra tilt of the tool axis, deg (default 0)")
    k.add_argument("--spin-mode", choices=("free", "fixed", "tangent"), default="free",
                   help="rotation about the nozzle axis: free (solver), fixed (--spin-fixed), tangent (tool X along the path + --spin-offset)")
    k.add_argument("--spin-fixed", type=float, default=0.0, help="spin in deg for --spin-mode fixed")
    k.add_argument("--spin-offset", type=float, default=0.0, help="spin offset in deg for --spin-mode tangent")
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
    r.add_argument("--base-points", type=float, nargs=9, default=None,
                   metavar=("OX", "OY", "OZ", "XX", "XY", "XZ", "PX", "PY", "PZ"),
                   help="$BASE from the KUKA 3-point method: origin, point on +X, point in the XY plane (robot coordinates)")
    r.add_argument("--robot-samples", type=int, default=20000, help="max path samples for the IK check (default 20000)")
    r.add_argument("--path-step", type=float, default=5.0, help="max distance between path samples, mm (default 5)")
    r.add_argument("--obstacles", default=None, help="obstacle file (one per line: box/cylinder/sphere/floor ...; 'help' prints the format)")
    r.add_argument("--margin", type=float, default=10.0, help="required clearance, mm (default 10)")
    r.add_argument("--tool-radius", type=float, default=25.0, help="tool holder radius for collisions, mm (default 25)")
    r.add_argument("--cutter-len", type=float, default=20.0, help="cutting length at the tool tip, mm (default 20)")
    r.add_argument("--cutter-radius", type=float, default=6.0, help="cutter radius, mm (default 6)")
    r.add_argument("--no-collisions", action="store_true", help="only reach/axis limits, no collision check")
    r.add_argument("--tool-mesh", default=None, help="STL/OBJ of the tool in flange coordinates (holder capsules fitted from it)")
    r.add_argument("--tool-mesh-units", default="mm", help="units of --tool-mesh: mm, cm, m, in (default mm)")
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
        for r in ROBOTS.values():
            print(f"{r.key:18s} {r.name}  reach {r.reach:g} mm, payload {r.payload:g} kg")
        return 0
    if args.obstacles == "help":
        from .collision import OBSTACLE_HELP
        print(OBSTACLE_HELP)
        return 0
    with open(args.input, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    if args.base_points:
        from .robot import frame_from_3_points
        bp = args.base_points
        args.base_frame = list(frame_from_3_points(bp[0:3], bp[3:6], bp[6:9]))
    parsed = parse_gcode(text, start=tuple(args.start), dwell_in_ms=args.dwell_ms)
    topts = TranslateOptions(tol=args.tol, max_radius=args.max_radius, min_radius=args.min_radius,
                             min_arc_points=args.min_arc_points, refine_shift=args.refine_shift,
                             spl_max_segment=args.spl_max_segment, corner_angle_deg=args.corner_angle,
                             default_feed=args.default_feed, offset=tuple(args.offset),
                             fit_arcs=not args.no_arcs, use_spl=not args.no_spl, extruder=args.extruder_out >= 0 or args.extruder_anout > 0,
                             extruder_analog=args.extruder_anout > 0)
    items, stats = translate(parsed, topts)
    kopts = KrlOptions(name=args.name or derive_name(args.input), mode=args.mode, tool=args.tool, base=args.base,
                       abc=tuple(args.abc), rapid_vel=args.rapid_vel, max_vel=args.max_vel, acc=args.acc,
                       cdis=args.cdis, corner_angle_deg=args.corner_angle, exact_corners=not args.round_corners,
                       max_block_segments=args.max_block_segments, max_motions_per_file=args.max_motions_per_file,
                       block_approx=not args.no_block_approx, spindle_out=args.spindle_out,
                       coolant_out=args.coolant_out, spindle_anout=args.spindle_anout, extruder_out=max(0, args.extruder_out),
                       extruder_anout=args.extruder_anout, extruder_flow_full=args.extruder_flow_full, extruder_delay=args.extruder_delay,
                       extruder_trigger=not args.no_extruder_trigger, extruder_on_delay=args.extruder_on_delay, extruder_off_delay=args.extruder_off_delay,
                       extruder_reverse_out=args.extruder_reverse_out,
                       home_start=not args.no_home_start, home_end=not args.no_home_end, home_mode=args.home_mode,
                       home_axes=tuple(float(v) for v in args.home_axes.replace(",", " ").split()) if args.home_axes.strip() else
                       (tuple(ROBOTS[args.robot.upper()].home) if args.robot and args.robot.upper() in ROBOTS else (0.0, -90.0, 90.0, 0.0, 0.0, 0.0)),
                       home_vel=args.home_vel,
                       krl_start=open(args.krl_start, encoding="utf-8").read() if args.krl_start else "",
                       krl_end=open(args.krl_end, encoding="utf-8").read() if args.krl_end else "",
                       spindle_max_rpm=args.spindle_max_rpm, bco_ptp=not args.no_bco, advance=args.advance,
                       source_name=os.path.basename(args.input), halt_on_tool_change=not args.no_halt_tool_change,
                       base_frame=tuple(args.base_frame) if args.base_frame else None,
                       tool_frame=tuple(args.tool_frame) if args.tool_frame else None)
    from .translator import has_orientation, resolve_orientations
    robot_rep = None
    orient_info = None
    if args.robot.lower() != "none":
        from .robot import check_path, frame
        from .translator import sample_items
        robot = ROBOTS.get(args.robot.upper())
        if robot is None:
            print(f"unknown robot {args.robot}; use --robot list", file=sys.stderr)
            return 2
        base_t = frame(args.base_frame or (0,) * 6)
        tool_t = frame(args.tool_frame or (0,) * 6)
        if has_orientation(items) and not args.no_orient_solve:
            orient_info = resolve_orientations(items, robot, base_t, tool_t, ref=kopts.abc, spin_step=args.spin_step,
                                               spin_mode=args.spin_mode, spin_fixed=args.spin_fixed, spin_offset=args.spin_offset,
                                               singular_deg=args.singular_deg, max_tilt_deg=args.max_tilt)
        if has_orientation(items):
            pts, abc_used = sample_items(items, max_samples=args.robot_samples, step=args.path_step if args.path_step > 0 else None,
                                         default_abc=kopts.abc, with_abc=True)
        else:
            pts = sample_items(items, max_samples=args.robot_samples, step=args.path_step if args.path_step > 0 else None)
            abc_used = kopts.abc
        robot_rep = check_path(robot, pts, abc_used, base_t, tool_t)
    kopts.ori_var = has_orientation(items)
    zone_dicts = []
    if args.zones:
        from .collision import parse_zones
        zone_dicts = parse_zones(open(args.zones, encoding="utf-8").read())
        if not args.no_zones_krl:
            from .collision import zone_from_dict
            from .robot import rot_to_abc
            for zd in zone_dicts:
                z = zone_from_dict(zd, base_t)
                kopts.workspaces.append({"name": z.name, "mode": z.mode, "scope": z.scope, "center": z.center.tolist(),
                                         "half": z.half.tolist(), "abc": [float(v) for v in rot_to_abc(z.rot)]})
    prog = write_krl(items, kopts, stats)
    zone_res = []
    if zone_dicts and args.robot.lower() != "none" and robot_rep is not None:
        from .collision import check_zones, robot_capsules, zone_from_dict
        zones = [zone_from_dict(zd, base_t) for zd in zone_dicts]
        caps = robot_capsules(robot, tool_t, args.tool_radius, args.cutter_len, args.cutter_radius)
        tcp_robot = (np.c_[pts, np.ones(len(pts))] @ base_t.T)[:, :3]
        zone_res = check_zones(zones, tcp_robot, robot, robot_rep.joints, robot_rep.ok_mask, caps)
        for zr in zone_res:
            if zr["n_violations"]:
                stats.warnings.append(f"safety zone {zr['name']} ({zr['scope']}): {'outside' if zr['mode'] == 'allowed' else 'inside'} at {zr['n_violations']} samples")
    if args.robot.lower() != "none":
        col_rep = None
        if not args.no_collisions:
            from .collision import OBSTACLE_HELP, obstacle_from_dict, parse_obstacles
            from .robot import check_path_collisions
            from .server import build_obstacles
            text = open(args.obstacles, encoding="utf-8").read() if args.obstacles else OBSTACLE_HELP
            obstacles = build_obstacles(parse_obstacles(text), base_t)
            tool_mesh = tool_mesh_key = None
            if args.tool_mesh:
                from . import meshstore
                tool_mesh, tool_mesh_key = meshstore.load(args.tool_mesh, args.tool_mesh_units)
            col_rep, _ = check_path_collisions(robot, pts, abc_used, base_t, tool_t, robot_rep, obstacles, margin=args.margin,
                                               tool_radius=args.tool_radius, cutter_len=args.cutter_len, cutter_radius=args.cutter_radius,
                                               tool_mesh=tool_mesh, tool_mesh_key=tool_mesh_key)
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
        for zr in zone_res:
            print(f"zone  : {zr['name']} ({zr['mode']}, {zr['scope']}) - {zr['n_violations']} violating samples")
        if robot_rep is not None:
            print(f"robot : {args.robot.upper()} - {robot_rep.n_samples} samples, {robot_rep.n_unreachable} out of reach, "
                  f"{robot_rep.n_limit} beyond axis limits")
            if orient_info:
                print(f"orient: {orient_info['points']} oriented points, {orient_info['resampled']} spins re-sampled, "
                      f"{orient_info['singular']} near singularity, {orient_info['unreachable']} unreachable at every spin")
            for k in range(6):
                lo, hi = robot_rep.joint_min[k], robot_rep.joint_max[k]
                if lo != float("inf"):
                    print(f"        A{k + 1}: {lo:8.2f} .. {hi:8.2f} deg" + (f"  ({robot_rep.axis_violations[k]} violations)" if robot_rep.axis_violations[k] else ""))
            if col_rep is not None:
                print(f"collis: margin {col_rep.margin:g} mm, {col_rep.n_collision} colliding samples, {col_rep.n_margin} below margin, "
                      f"min clearance {col_rep.min_clearance:.1f} mm, {col_rep.refined_samples} refinement IK calls, {col_rep.unresolved} unresolved")
                for pr in col_rep.pairs[:8]:
                    flag = "COLLISION" if pr.min_clearance <= 0 else ("margin" if pr.min_clearance < col_rep.margin else "ok")
                    print(f"        {pr.a:12s} vs {pr.b:14s} min {pr.min_clearance:8.1f} mm  @sample {pr.argmin:6d}  {flag}")
                for note in col_rep.notes:
                    print("        note: " + note)
        for w in stats.warnings:
            print("warning: " + w, file=sys.stderr)
    bad_robot = robot_rep is not None and (robot_rep.n_unreachable or robot_rep.n_limit)
    bad_col = args.robot.lower() != "none" and not args.no_collisions and col_rep is not None and (col_rep.n_collision or col_rep.unresolved)
    bad_zone = any(zr["n_violations"] for zr in zone_res)
    return 3 if (bad_robot or bad_col or bad_zone) else 0


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
