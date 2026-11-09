"""Local web GUI server for gcode2krl (standard library only).

    gcode2krl-gui                 # starts http://127.0.0.1:8765 and opens the browser
    gcode2krl-gui --port 9000 --no-browser

API (JSON):
    GET  /api/examples               list of bundled example files
    GET  /api/example/<name>         example G-code text
    POST /api/translate              {gcode, filename, options} -> result
    GET  /api/zip/<token>            zip with the generated .src/.dat files
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import socket
import sys
import threading
import time
import uuid
import webbrowser
import zipfile
from collections import OrderedDict
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import numpy as np

from . import __version__
from .gcode_parser import parse_gcode
from .krl_writer import KrlOptions, write_krl
from . import meshstore
from .collision import OBSTACLE_HELP, mesh_obstacles, obstacle_from_dict, parse_obstacles, parse_zones, zone_from_dict, check_zones, ZONE_HELP
from .robot import DEFAULT_ROBOT, ROBOTS, check_path, check_path_collisions, forward, frame
from .translator import ArcItem, EventItem, LineItem, RapidItem, SplItem, TranslateOptions, sample_items, translate

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
EXAMPLES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "examples")
MAX_BODY = 200 * 1024 * 1024

_results: "OrderedDict[str, dict]" = OrderedDict()
_results_lock = threading.Lock()


def _remember(files: dict[str, str]) -> str:
    token = uuid.uuid4().hex
    with _results_lock:
        _results[token] = files
        while len(_results) > 16:
            _results.popitem(last=False)
    return token


def _f32(arr: list[float] | np.ndarray) -> str:
    return base64.b64encode(np.asarray(arr, dtype=np.float32).tobytes()).decode("ascii")


def _opt(options: dict, key: str, default, cast):
    val = options.get(key, default)
    if val is None or val == "":
        return default
    try:
        return cast(val)
    except (TypeError, ValueError):
        raise ValueError(f"invalid value for {key}: {val!r}")


def _ntuple(options: dict, key: str, default, n: int):
    val = options.get(key, default)
    if val is None or val == "":
        val = default
    if isinstance(val, str):
        val = [float(x) for x in val.replace(",", " ").split()]
    if val is None:
        return None
    if len(val) != n:
        raise ValueError(f"{key} needs {n} numbers")
    return tuple(float(x) for x in val)


def _triple(options: dict, key: str, default):
    return _ntuple(options, key, default, 3)


def build_options(options: dict, filename: str) -> tuple[TranslateOptions, KrlOptions, dict]:
    topts = TranslateOptions(
        tol=_opt(options, "tol", 0.05, float),
        max_radius=_opt(options, "max_radius", 5000.0, float),
        min_radius=_opt(options, "min_radius", 0.1, float),
        min_arc_points=_opt(options, "min_arc_points", 4, int),
        refine_shift=_opt(options, "refine_shift", 3, int),
        spl_max_segment=_opt(options, "spl_max_segment", 3.0, float),
        corner_angle_deg=_opt(options, "corner_angle", 10.0, float),
        default_feed=_opt(options, "default_feed", 1000.0, float),
        offset=_triple(options, "offset", (0.0, 0.0, 0.0)),
        fit_arcs=_opt(options, "fit_arcs", True, bool),
        use_spl=_opt(options, "use_spl", True, bool),
        extruder=_opt(options, "extruder", False, bool),
        extruder_analog=_opt(options, "extruder_analog", False, bool),
    )
    base = os.path.splitext(os.path.basename(filename or "PROG"))[0]
    default_name = "".join(c if c.isalnum() else "_" for c in base).upper()[:20] or "PROG"
    if not default_name[0].isalpha():
        default_name = "P_" + default_name[:18]
    kopts = KrlOptions(
        name=_opt(options, "name", default_name, str) or default_name,
        mode=_opt(options, "mode", "spline", str),
        tool=_opt(options, "tool", 1, int),
        base=_opt(options, "base", 1, int),
        abc=_triple(options, "abc", (0.0, 180.0, 0.0)),
        rapid_vel=_opt(options, "rapid_vel", 0.5, float),
        max_vel=_opt(options, "max_vel", 2.0, float),
        acc=_opt(options, "acc", 2.0, float),
        cdis=_opt(options, "cdis", 0.2, float),
        corner_angle_deg=_opt(options, "corner_angle", 10.0, float),
        tangent_deg=_opt(options, "tangent_deg", 0.1, float),
        bridge_mm=_opt(options, "bridge_mm", 3.0, float),
        exact_corners=not _opt(options, "round_corners", False, bool),
        max_block_segments=_opt(options, "max_block_segments", 500, int),
        max_motions_per_file=_opt(options, "max_motions_per_file", 4000, int),
        block_approx=_opt(options, "block_approx", True, bool),
        spindle_out=_opt(options, "spindle_out", 0, int),
        coolant_out=_opt(options, "coolant_out", 0, int),
        spindle_anout=_opt(options, "spindle_anout", 0, int),
        spindle_max_rpm=_opt(options, "spindle_max_rpm", 24000.0, float),
        extruder_out=_opt(options, "extruder_out", 0, int),
        extruder_anout=_opt(options, "extruder_anout", 0, int),
        extruder_flow_full=_opt(options, "extruder_flow_full", 100.0, float),
        extruder_delay=_opt(options, "extruder_delay", 0.0, float),
        extruder_trigger=_opt(options, "extruder_trigger", True, bool),
        extruder_on_delay=_opt(options, "extruder_on_delay", 0.0, float),
        extruder_off_delay=_opt(options, "extruder_off_delay", 0.0, float),
        extruder_reverse_out=_opt(options, "extruder_reverse_out", 0, int),
        home_start=_opt(options, "home_start", True, bool),
        home_end=_opt(options, "home_end", True, bool),
        home_mode=str(options.get("home_mode") or "xhome"),
        home_axes=_home_axes(options),
        home_vel=_opt(options, "home_vel", 100.0, float),
        workspaces=_workspaces(options),
        krl_start=str(options.get("krl_start") or ""),
        krl_end=str(options.get("krl_end") or ""),
        bco_ptp=_opt(options, "bco", True, bool),
        advance=_opt(options, "advance", 3, int),
        source_name=os.path.basename(filename or ""),
        halt_on_tool_change=_opt(options, "halt_tool_change", True, bool),
        base_frame=_ntuple(options, "base_frame", None, 6) if _frame_known(options, "base") else None,
        tool_frame=_ntuple(options, "tool_frame", None, 6) if _frame_known(options, "tool") else None,
        frame_mode=_frame_mode(options),
    )
    if kopts.mode not in ("spline", "lin"):
        raise ValueError("mode must be 'spline' or 'lin'")
    parser_kw = {
        "start": _triple(options, "start", (0.0, 0.0, 0.0)),
        "dwell_in_ms": _opt(options, "dwell_ms", False, bool),
    }
    return topts, kopts, parser_kw


def preview_geometry(parsed, items, offset) -> dict:
    """Flat float32 segment lists (x0,y0,z0,x1,y1,z1,...) per layer for WebGL."""
    from .gcode_parser import ArcMove, LinearMove
    from .geometry import sample_arc

    off = np.array(offset, dtype=float)
    inp: list[np.ndarray] = []
    inp_rapid: list[np.ndarray] = []
    for c in parsed.commands:
        if isinstance(c, LinearMove):
            (inp_rapid if c.rapid else inp).append(np.concatenate([c.start + off, c.end + off]))
        elif isinstance(c, ArcMove):
            from .geometry import arc_from_center
            fit = arc_from_center(c.start + off, c.end + off, c.center + off, c.normal, c.ccw)
            pts = sample_arc(fit, 0.02, min_points=8)
            if c.helical:
                rise = float((c.end - c.start) @ c.normal)
                pts = pts + np.outer(np.linspace(0, 1, len(pts)) * rise, c.normal)
            inp.append(np.column_stack([pts[:-1], pts[1:]]).ravel())
    layers: dict[str, list[np.ndarray]] = {"rapid": [], "slin": [], "scirc": [], "spl": []}
    arc_mids: list[np.ndarray] = []
    for it in items:
        if isinstance(it, RapidItem):
            layers["rapid"].append(np.concatenate([it.start, it.target]))
        elif isinstance(it, LineItem):
            layers["slin"].append(np.concatenate([it.start, it.end]))
        elif isinstance(it, ArcItem):
            pts = sample_arc(it.fit, 0.01, min_points=12)
            pts[0] = it.start
            pts[-1] = it.end
            layers["scirc"].append(np.column_stack([pts[:-1], pts[1:]]).ravel())
            arc_mids.append(it.mid)
        elif isinstance(it, SplItem):
            p = it.points
            layers["spl"].append(np.column_stack([p[:-1], p[1:]]).ravel())

    def pack(chunks):
        return _f32(np.concatenate(chunks) if chunks else np.zeros(0))

    allpts = [np.asarray(x).reshape(-1, 3) for x in inp + inp_rapid]
    if allpts:
        a = np.concatenate(allpts)
        bounds = {"min": a.min(axis=0).tolist(), "max": a.max(axis=0).tolist()}
    else:
        bounds = {"min": [0, 0, 0], "max": [1, 1, 1]}
    return {
        "input": pack(inp),
        "input_rapid": pack(inp_rapid),
        "rapid": pack(layers["rapid"]),
        "slin": pack(layers["slin"]),
        "scirc": pack(layers["scirc"]),
        "spl": pack(layers["spl"]),
        "arc_points": pack(arc_mids),
        "bounds": bounds,
    }


_mesh_cache: dict[str, dict] = {}


def robot_mesh_payload(robot) -> dict:
    """Per-link triangle soup (float32 base64, mm, link frames) with flat normals."""
    if robot.key in _mesh_cache:
        return _mesh_cache[robot.key]
    from .meshio import flat_normals, load_stl
    links = {}
    for idx, rel in robot.meshes.items():
        path = os.path.join(robot.base_dir, rel)
        if not os.path.isfile(path):
            continue
        tris = load_stl(path)
        links[idx] = {"positions": _f32(tris.ravel()), "normals": _f32(flat_normals(tris).ravel()), "tris": int(len(tris))}
    payload = {"key": robot.key, "links": links}
    _mesh_cache[robot.key] = payload
    return payload


def _zone_dicts(options: dict) -> list[dict]:
    src = options.get("zones", None)
    if src is None or src == "":
        return []
    if isinstance(src, str):
        return parse_zones(src)
    return list(src)


def _workspaces(options: dict) -> list[dict]:
    """$WORKSPACE entries (robot frame) for the KRL program."""
    if not _opt(options, "zones_krl", True, bool):
        return []
    from .robot import rot_to_abc
    base = frame(_ntuple(options, "base_frame", (0, 0, 0, 0, 0, 0), 6))
    out = []
    for d in _zone_dicts(options):
        z = zone_from_dict(d, base)
        out.append({"name": z.name, "mode": z.mode, "scope": z.scope, "center": z.center.tolist(), "half": z.half.tolist(),
                    "abc": [float(v) for v in rot_to_abc(z.rot)]})
    return out


def _home_axes(options: dict) -> tuple[float, ...]:
    """HOME joint angles: the option (6 numbers) or the robot model's home."""
    txt = str(options.get("home_axes") or "").strip()
    if txt:
        vals = [float(v) for v in txt.replace(",", " ").split()]
        if len(vals) == 6:
            return tuple(vals)
    key = str(options.get("robot", DEFAULT_ROBOT) or "none").upper()
    r = ROBOTS.get(key)
    return tuple(r.home) if r is not None else (0.0, -90.0, 90.0, 0.0, 0.0, 0.0)


def robot_report(items, options: dict, abc) -> dict | None:
    """IK along the output path: reach / axis-limit report, collisions and joint trace for animation."""
    key = str(options.get("robot", DEFAULT_ROBOT) or "none").upper()
    if key == "NONE":
        return None
    robot = ROBOTS.get(key)
    if robot is None:
        raise ValueError(f"unknown robot {key}; known: {', '.join(ROBOTS)}")
    base = frame(_ntuple(options, "base_frame", (0, 0, 0, 0, 0, 0), 6))
    tool = frame(_ntuple(options, "tool_frame", (0, 0, 0, 0, 0, 0), 6))
    step = _opt(options, "path_step", 5.0, float)
    max_samples = _opt(options, "robot_samples", 20000, int)
    from .translator import has_orientation, resolve_orientations
    orient_info = None
    if has_orientation(items) and _opt(options, "orient_solve", True, bool):
        orient_info = resolve_orientations(items, robot, base, tool, ref=tuple(abc),
                                           spin_step=_opt(options, "spin_step", 15.0, float),
                                           singular_deg=_opt(options, "singular_deg", 8.0, float),
                                           max_tilt_deg=_opt(options, "max_tilt", 0.0, float),
                                           spin_mode=str(options.get("spin_mode") or "free"),
                                           spin_fixed=_opt(options, "spin_fixed", 0.0, float),
                                           spin_offset=_opt(options, "spin_offset", 0.0, float))
    from .robot import assign_configurations, status_turn_batch
    cfg = assign_configurations(items, robot, base, tool, ref=tuple(abc)) if _opt(options, "status_turn", True, bool) else None
    if has_orientation(items):
        pts, abcs = sample_items(items, max_samples=max_samples, step=step if step > 0 else None, default_abc=abc, with_abc=True)
        abc_used = abcs
    else:
        pts = sample_items(items, max_samples=max_samples, step=step if step > 0 else None)
        abc_used = abc
    rep = check_path(robot, pts, abc_used, base, tool)
    d = rep.to_dict()
    d["robot"] = robot.key
    d["orientation"] = orient_info
    if cfg is not None and rep.joints is not None:
        st, tn = status_turn_batch(robot, rep.joints)
        okm = st >= 0
        changes = int(np.sum((st[1:] != st[:-1]) & okm[1:] & okm[:-1]))
        cfg["status_changes_sampled"] = changes
        cfg["status_values"] = sorted({int(v) for v in st[okm]})
        d["status"] = _f32(st.astype(np.float32))
    d["configuration"] = cfg
    d["oriented"] = bool(has_orientation(items))
    d["step"] = step
    d["points"] = _f32(pts.ravel())
    d["joints"] = _f32(np.nan_to_num(rep.joints, nan=0.0).ravel()) if rep.joints is not None else ""
    d["ok"] = base64.b64encode(np.asarray(rep.ok_mask, dtype=np.uint8).tobytes()).decode("ascii") if rep.ok_mask is not None else ""
    bad = pts[~rep.ok_mask] if rep.ok_mask is not None else np.zeros((0, 3))
    d["bad_points"] = _f32(bad.ravel())
    from .robot import inv_frame
    d["robot_in_base"] = inv_frame(base).ravel().tolist()
    d["tool"] = tool.ravel().tolist()
    # ---- collisions
    obs_src = options.get("obstacles", None)
    if obs_src is None:
        obs_dicts = parse_obstacles(OBSTACLE_HELP)
    elif isinstance(obs_src, str):
        obs_dicts = parse_obstacles(obs_src)
    else:
        obs_dicts = list(obs_src)
    obstacles = build_obstacles(obs_dicts, base)
    margin = _opt(options, "margin", 10.0, float)
    tool_mesh = None
    tool_mesh_key = None
    tool_mesh_ref = str(options.get("tool_mesh") or "").strip()
    if tool_mesh_ref:
        tool_mesh, tool_mesh_key = meshstore.load(tool_mesh_ref, _opt(options, "tool_mesh_units", "mm", str))
        mf = _ntuple(options, "tool_mesh_frame", (0, 0, 0, 0, 0, 0), 6)
        if any(abs(v) > 1e-9 for v in mf):
            # the STEP / STL of the tool is placed in the flange frame by this frame
            fm = frame(mf)
            tool_mesh = tool_mesh @ fm[:3, :3].T + fm[:3, 3]
            tool_mesh_key = (tool_mesh_key, tuple(round(float(v), 3) for v in mf))
    cap_kw = dict(tool_radius=_opt(options, "tool_radius", 25.0, float), cutter_len=_opt(options, "cutter_len", 20.0, float),
                  cutter_radius=_opt(options, "cutter_radius", 6.0, float), tool_mesh=tool_mesh, tool_mesh_key=tool_mesh_key)
    # ---- the printed part (planar printing): footprint of the path, top = current layer
    from shapely.geometry import MultiPoint
    from .robot import check_printed
    printed_on = _opt(options, "printed_check", True, bool)
    printed_margin = _opt(options, "printed_margin", 2.0, float)
    footprint = MultiPoint(pts[:, :2]).convex_hull if printed_on and len(pts) >= 3 else None
    # ---- automatic spin of a constant orientation about the nozzle axis to avoid collisions
    auto_spin = _opt(options, "auto_spin", 0.0, float)
    spin_info = None
    if auto_spin > 0 and not has_orientation(items) and len(pts) > 1:
        sub = pts[:: max(1, len(pts) // 600)]
        a0 = float(abc[0])
        cands = []
        for k in range(int(round(360.0 / auto_spin))):
            cand = (((a0 + k * auto_spin) + 180.0) % 360.0 - 180.0, float(abc[1]), float(abc[2]))
            rc = check_path(robot, sub, cand, base, tool)
            cc, caps_c = check_path_collisions(robot, sub, cand, base, tool, rc, obstacles, margin=margin, max_refine=0, **cap_kw)
            pr = check_printed(robot, rc.joints, rc.ok_mask, caps_c, sub, base, footprint, printed_margin) if footprint is not None else {"n_collision": 0}
            score = (rc.n_unreachable + rc.n_limit, cc.n_collision + pr["n_collision"], cc.n_margin, -(cc.min_clearance if np.isfinite(cc.min_clearance) else 0.0))
            cands.append({"a": round(cand[0], 1), "unreachable": rc.n_unreachable + rc.n_limit, "collisions": cc.n_collision + pr["n_collision"],
                          "below_margin": cc.n_margin, "min_clearance": None if not np.isfinite(cc.min_clearance) else round(float(cc.min_clearance), 1), "_score": score})
        best = min(cands, key=lambda c: c["_score"])
        for c in cands:
            c.pop("_score")
        abc = (best["a"], float(abc[1]), float(abc[2]))
        abc_used = abc
        rep = check_path(robot, pts, abc_used, base, tool)
        d.update(rep.to_dict())
        d["joints"] = _f32(np.nan_to_num(rep.joints, nan=0.0).ravel()) if rep.joints is not None else ""
        d["ok"] = base64.b64encode(np.asarray(rep.ok_mask, dtype=np.uint8).tobytes()).decode("ascii") if rep.ok_mask is not None else ""
        bad = pts[~rep.ok_mask] if rep.ok_mask is not None else np.zeros((0, 3))
        d["bad_points"] = _f32(bad.ravel())
        spin_info = {"abc": list(abc), "a": best["a"], "step": auto_spin, "candidates": cands}
    d["auto_spin"] = spin_info
    col, caps = check_path_collisions(robot, pts, abc_used, base, tool, rep, obstacles, margin=margin,
                                      max_refine=_opt(options, "max_refine", 6, int), **cap_kw)
    if footprint is not None:
        pr = check_printed(robot, rep.joints, rep.ok_mask, caps, pts, base, footprint, printed_margin)
        d["printed"] = {"n_collision": pr["n_collision"], "max_depth": round(pr["max_depth"], 2), "by_capsule": pr["by_capsule"], "margin": printed_margin}
        d["printed_points"] = _f32(pts[pr["mask"]].ravel())
    else:
        d["printed"] = None
        d["printed_points"] = ""
    # ---- PTP moves HOME -> first point and last point -> HOME (joint interpolation)
    from .robot import check_joint_move
    home_q = np.asarray(_home_axes(options), dtype=float)
    hm = {"home": home_q.tolist(), "start": None, "end": None}
    if rep.joints is not None and rep.ok_mask is not None and rep.ok_mask.any():
        idx = np.flatnonzero(rep.ok_mask)
        for label, on, qi in (("start", _opt(options, "home_start", True, bool), rep.joints[idx[0]]),
                              ("end", _opt(options, "home_end", True, bool), rep.joints[idx[-1]])):
            if not on:
                continue
            r2 = check_joint_move(robot, home_q, qi, caps, obstacles, margin=margin, max_refine=_opt(options, "max_refine", 6, int)) \
                if label == "start" else check_joint_move(robot, qi, home_q, caps, obstacles, margin=margin, max_refine=_opt(options, "max_refine", 6, int))
            worst = None
            if r2.pairs:
                wp = min(r2.pairs, key=lambda pr: pr.min_clearance)
                worst = {"a": wp.a, "b": wp.b, "min_clearance": wp.min_clearance}
            hm[label] = {"n_collision": r2.n_collision, "n_margin": r2.n_margin, "min_clearance": r2.min_clearance, "worst": worst}
    d["home_moves"] = hm
    # ---- safety zones: TCP (or the whole robot) on the path and on the HOME moves
    zones = [zone_from_dict(zd, base) for zd in _zone_dicts(options)]
    zd_out = {"zones": [z.to_dict() for z in zones], "results": [], "n_violations": 0}
    zone_pts = []
    if zones:
        tcp_robot = (np.c_[pts, np.ones(len(pts))] @ base.T)[:, :3]
        res = check_zones(zones, tcp_robot, robot, rep.joints, rep.ok_mask, caps)
        for r0 in res:
            r0["n_path"] = r0.pop("n_violations")
            idx = r0.pop("indices")
            r0["n_home"] = 0
            if idx:
                zone_pts.append(pts[np.array(idx, dtype=int)])
        # HOME moves (joint interpolation): TCP from the forward kinematics
        from .collision import batch_fk
        if rep.joints is not None and rep.ok_mask is not None and rep.ok_mask.any():
            idx_ok = np.flatnonzero(rep.ok_mask)
            for label, on, qi in (("start", _opt(options, "home_start", True, bool), rep.joints[idx_ok[0]]),
                                  ("end", _opt(options, "home_end", True, bool), rep.joints[idx_ok[-1]])):
                if not on:
                    continue
                t = np.linspace(0.0, 1.0, 16)[:, None]
                qq = home_q[None, :] * (1 - t) + np.asarray(qi)[None, :] * t if label == "start" else np.asarray(qi)[None, :] * (1 - t) + home_q[None, :] * t
                fr = batch_fk(robot, qq)
                r_fl, p_fl = fr[-1]
                tcp_h = p_fl + np.einsum("nij,j->ni", r_fl, tool[:3, 3])
                res_h = check_zones(zones, tcp_h, robot, qq, np.ones(len(qq), dtype=bool), caps)
                for r0, rh in zip(res, res_h):
                    r0["n_home"] += rh["n_violations"]
        zd_out["results"] = res
        zd_out["n_violations"] = int(sum(r0["n_path"] + r0["n_home"] for r0 in res))
    d["zones"] = zd_out
    d["zone_points"] = _f32(np.concatenate(zone_pts).ravel()) if zone_pts else ""
    cd = col.to_dict()
    cd["clearance"] = _f32(np.nan_to_num(col.clearance, nan=1e9).ravel())
    cd["worst_pair"] = col.worst_pair
    cd["collision_points"] = _f32(pts[col.collision_mask].ravel())
    cd["margin_points"] = _f32(pts[col.margin_mask].ravel())
    cd["obstacles"] = [o.to_dict() for o in obstacles]
    cd["tool_mesh"] = tool_mesh_ref or None
    cd["tool_mesh_frame"] = list(_ntuple(options, "tool_mesh_frame", (0, 0, 0, 0, 0, 0), 6))
    cd["tool_mesh_units"] = _opt(options, "tool_mesh_units", "mm", str)
    cd["capsules"] = [{"name": c.name, "link": c.link, "a": c.a.tolist(), "b": c.b.tolist(), "radius": c.radius, "kind": c.kind} for c in caps]
    cd["pair_names"] = [[p.a, p.b, p.kind] for p in col.pairs]
    d["collision"] = cd
    return d


def build_obstacles(obs_dicts: list, base: np.ndarray) -> list:
    """Obstacle objects from parsed dicts; mesh entries expand to their convex primitives."""
    out = []
    for o in obs_dicts:
        if str(o.get("kind", "")).lower() == "mesh":
            tris, key = meshstore.load(o["file"], o.get("units", "mm"))
            out.extend(mesh_obstacles(o, base, tris, mesh_id=o["file"]))
        else:
            out.append(obstacle_from_dict(o, base))
    return out


def mesh_render_payload(ref: str, units: str = "mm") -> dict:
    from .meshio import flat_normals
    tris, key = meshstore.load(ref, units)
    if len(tris) > 400000:
        tris = tris[:: int(np.ceil(len(tris) / 400000))]
    return {"id": ref, "positions": _f32(tris.ravel()), "normals": _f32(flat_normals(tris).ravel()), "tris": int(len(tris))}


def run_translation(gcode: str, filename: str, options: dict) -> dict:
    t0 = time.perf_counter()
    topts, kopts, parser_kw = build_options(options or {}, filename)
    parsed = parse_gcode(gcode, **parser_kw)
    t1 = time.perf_counter()
    items, stats = translate(parsed, topts)
    t2 = time.perf_counter()
    robot = robot_report(items, options or {}, kopts.abc)  # may refine orientations (spin solver) before writing
    if robot and robot.get("auto_spin"):
        kopts.abc = tuple(robot["auto_spin"]["abc"])  # the collision-free spin chosen for the constant orientation
    t3 = time.perf_counter()
    from .translator import has_orientation
    kopts.ori_var = has_orientation(items)
    prog = write_krl(items, kopts, stats)
    from .krl_lint import lint_files
    stats.warnings.extend(lint_files(prog.files, kopts.tangent_deg))
    t4 = time.perf_counter()
    token = _remember(prog.files)
    if robot:
        if robot["n_unreachable"]:
            stats.warnings.append(f"robot {robot['robot']}: {robot['n_unreachable']} of {robot['n_samples']} path samples are out of reach")
        if robot["n_limit"]:
            axes = ", ".join(f"A{k + 1}" for k, v in enumerate(robot["axis_violations"]) if v)
            stats.warnings.append(f"robot {robot['robot']}: {robot['n_limit']} samples exceed axis limits ({axes})")
        c = robot["collision"]
        if c["n_collision"]:
            worst = c["pairs"][0]
            stats.warnings.append(f"COLLISION: {c['n_collision']} samples collide (worst {worst['a']} vs {worst['b']}, "
                                  f"{worst['min_clearance']:.1f} mm)")
        elif c["n_margin"]:
            stats.warnings.append(f"collision margin {c['margin']:g} mm violated at {c['n_margin']} samples (min clearance {c['min_clearance']:.1f} mm)")
        pr = robot.get("printed")
        if pr and pr["n_collision"]:
            worst = ", ".join(f"{k} ({v})" for k, v in sorted(pr["by_capsule"].items(), key=lambda kv: -kv[1])[:3])
            stats.warnings.append(f"tool / robot dips into the printed part at {pr['n_collision']} samples (max {pr['max_depth']:.1f} mm): {worst}")
        if robot.get("auto_spin"):
            a = robot["auto_spin"]
            stats.warnings.append(f"auto spin: orientation A set to {a['a']:g} deg (best of {len(a['candidates'])} candidates)")
        cfg = robot.get("configuration")
        if cfg:
            if cfg["status_changes_in_path"]:
                stats.warnings.append(f"robot configuration (Status) changes {cfg['status_changes_in_path']}x inside a continuous path: "
                                      f"the controller cannot pass that (singularity) - raise min |A5|, change the spin mode or the tool")
            if cfg["unreachable_travels"]:
                stats.warnings.append(f"{cfg['unreachable_travels']} travel targets out of reach (no Status/Turn assigned)")
        for zr in (robot.get("zones") or {}).get("results", []):
            if zr["n_path"] or zr["n_home"]:
                what = "outside the allowed zone" if zr["mode"] == "allowed" else "inside the forbidden zone"
                stats.warnings.append(f"safety zone {zr['name']} ({zr['scope']}): {what} at {zr['n_path']} path samples"
                                      + (f" and {zr['n_home']} HOME-move samples" if zr["n_home"] else ""))
        for label, name in (("start", "PTP HOME -> start"), ("end", "PTP end -> HOME")):
            h = (robot.get("home_moves") or {}).get(label)
            if h and h["n_collision"]:
                stats.warnings.append(f"{name}: collision ({h['worst']['a']} / {h['worst']['b']}, {h['min_clearance']:.1f} mm)")
            elif h and h["n_margin"]:
                stats.warnings.append(f"{name}: below the collision margin (min clearance {h['min_clearance']:.1f} mm)")
        for note in c["notes"]:
            stats.warnings.append(f"robot: {note}")
    ic = stats.input_counts
    n_in = sum(ic.values())
    result = {
        "token": token,
        "name": kopts.name,
        "files": prog.files,
        "main_file": f"{kopts.name}.src",
        "stats": {
            "input_moves": n_in,
            "input_counts": ic,
            "input_lines": parsed.n_lines,
            "chains": stats.chains,
            "chain_vertices": stats.chain_vertices,
            "lines_out": stats.lines_out,
            "arcs_out": stats.arcs_out,
            "spl_points": stats.spl_points,
            "spl_runs": stats.spl_runs,
            "rapids": stats.rapids,
            "motions_out": stats.motions_out,
            "compression": (n_in / stats.motions_out) if stats.motions_out else 0.0,
            "max_dev": stats.max_dev,
            "tol": topts.tol,
            "time_parse": t1 - t0,
            "time_fit": t2 - t1,
            "time_write": t4 - t3,
            "time_robot": t3 - t2,
            "time_total": t4 - t0,
            "oriented_vertices": stats.oriented_vertices,
        },
        "warnings": stats.warnings,
        "preview": preview_geometry(parsed, items, topts.offset),
        "robot": robot,
    }
    return result


class Handler(SimpleHTTPRequestHandler):
    server_version = f"gcode2krl/{__version__}"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, fmt, *args):  # quieter
        if getattr(self.server, "verbose", False):
            super().log_message(fmt, *args)

    # ------------------------------------------------------------ helpers
    def send_json(self, obj, status: int = 200) -> None:
        data = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_bytes(self, data: bytes, ctype: str, filename: str | None = None) -> None:
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(data)

    def read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > MAX_BODY:
            raise ValueError("request body missing or too large")
        return json.loads(self.rfile.read(length).decode("utf-8"))

    # --------------------------------------------------------------- GET
    def do_GET(self) -> None:
        path = unquote(self.path.split("?", 1)[0])
        if path == "/api/examples":
            names = sorted(f for f in os.listdir(EXAMPLES_DIR) if f.lower().endswith((".nc", ".gcode", ".tap", ".ngc")))\
                if os.path.isdir(EXAMPLES_DIR) else []
            self.send_json({"examples": names})
            return
        if path.startswith("/api/example/"):
            name = os.path.basename(path[len("/api/example/"):])
            full = os.path.join(EXAMPLES_DIR, name)
            if not os.path.isfile(full):
                self.send_json({"error": "not found"}, 404)
                return
            with open(full, "r", encoding="utf-8", errors="replace") as fh:
                self.send_json({"name": name, "gcode": fh.read()})
            return
        if path.startswith("/api/zip/"):
            token = path[len("/api/zip/"):].strip("/")
            with _results_lock:
                files = _results.get(token)
            if files is None:
                self.send_json({"error": "unknown or expired token"}, 404)
                return
            buf = io.BytesIO()
            with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
                for fname, content in files.items():
                    zf.writestr(fname, content.replace("\n", "\r\n"))
            main = next((f[:-4] for f in files if f.endswith(".src") and "_" not in f[-8:-4]), "krl")
            self.send_bytes(buf.getvalue(), "application/zip", f"{main}.zip")
            return
        if path == "/api/version":
            self.send_json({"version": __version__})
            return
        if path == "/api/meshes":
            from .stepio import available_backends
            self.send_json({"meshes": meshstore.list_meshes(), "dir": meshstore.mesh_dir(), "step_backends": available_backends()})
            return
        if path.startswith("/api/mesh/"):
            ref = path[len("/api/mesh/"):]
            units = "mm"
            if "?" in self.path:
                from urllib.parse import parse_qs
                units = parse_qs(self.path.split("?", 1)[1]).get("units", ["mm"])[0]
            try:
                self.send_json(mesh_render_payload(ref, units))
            except ValueError as exc:
                self.send_json({"error": str(exc)}, 404)
            return
        if path == "/api/robots":
            self.send_json({"default": DEFAULT_ROBOT, "robots": [r.to_dict() for r in ROBOTS.values()], "zone_help": ZONE_HELP,
                            "obstacle_help": OBSTACLE_HELP})
            return
        if path.startswith("/api/robot_mesh/"):
            key = os.path.basename(path).upper()
            robot = ROBOTS.get(key)
            if robot is None:
                self.send_json({"error": "unknown robot"}, 404)
                return
            self.send_json(robot_mesh_payload(robot))
            return
        if path == "/":
            self.path = "/index.html"
        super().do_GET()

    # -------------------------------------------------------------- POST
    def do_POST(self) -> None:
        path = unquote(self.path.split("?", 1)[0])
        if path == "/api/mesh":
            try:
                body = self.read_json()
                data = base64.b64decode(body.get("data", ""))
                if not data:
                    raise ValueError("empty file")
                meta = meshstore.save_mesh(str(body.get("name", "mesh.stl")), data, deflection=float(body.get("deflection", 0.2) or 0.2))
            except ValueError as exc:
                self.send_json({"error": str(exc)}, 400)
                return
            self.send_json(meta)
            return
        if path == "/api/mesh/delete":
            try:
                body = self.read_json()
                ok = meshstore.delete_mesh(str(body.get("id", "")))
            except ValueError as exc:
                self.send_json({"error": str(exc)}, 400)
                return
            self.send_json({"deleted": ok})
            return
        if path == "/api/orient":
            try:
                body = self.read_json()
                from .slicer.plater import auto_orient
                ref = str(body.get("mesh") or "")
                if not ref:
                    raise ValueError("no mesh given")
                tris, _ = meshstore.load(ref, str(body.get("units") or "mm"), float(body.get("deflection") or 0.2))
                res = auto_orient(tris, str(body.get("mode") or "min_overhang"), float(body.get("overhang") or 45.0))
                self.send_json({k: (list(v) if isinstance(v, tuple) else v) for k, v in res.items()})
            except ValueError as exc:
                self.send_json({"error": str(exc)}, 400)
            return
        if path == "/api/slice":
            try:
                body = self.read_json()
                from .slicer import SliceParams, slice_triangles
                from .slicer.pipeline import preview_segments, slice_models
                from .slicer.plater import Placement
                params = SliceParams.from_dict(body.get("params") or {})
                t0 = time.perf_counter()
                segments = body.get("segments") or None
                names = {m["id"]: m["name"] for m in meshstore.list_meshes()}
                models = body.get("models") or []
                deflection = float(body.get("deflection") or 0.1)
                if models:
                    # the plater positions are authoritative: no re-centring of the combined set
                    params = SliceParams.from_dict({**params.to_dict(), "origin_center": False})
                    placed = []
                    for md in models:
                        ref = str(md.get("mesh") or "")
                        if not ref:
                            continue
                        tris, _ = meshstore.load(ref, str(md.get("units") or body.get("units") or "mm"), deflection)
                        pl = Placement.from_dict({**md, "name": md.get("name") or names.get(ref, ref)})
                        # the global placement of the parameters applies on top of each row
                        from .slicer.pipeline import placed as _placed
                        placed.append((_placed(tris, params), pl))
                    if not placed:
                        raise ValueError("no models given")
                    source = ", ".join(pl.name for _, pl in placed)
                    gcode, st, layers = slice_models(placed, params, source=source, segments=segments)
                    name = placed[0][1].name if len(placed) == 1 else "stol"
                else:
                    ref = str(body.get("mesh") or "")
                    if not ref:
                        raise ValueError("no mesh given")
                    tris, key = meshstore.load(ref, str(body.get("units") or "mm"), deflection)
                    name = names.get(ref, ref)
                    off = body.get("offset")
                    if params.origin_center and off and any(abs(float(v)) > 1e-9 for v in off[:2]):
                        # place the single model on the bed (e.g. the table centre) instead of the base origin
                        from .slicer.pipeline import placed as _placed
                        pl = Placement(name=name, offset=(float(off[0]), float(off[1]), 0.0))
                        p2 = SliceParams.from_dict({**params.to_dict(), "origin_center": False})
                        gcode, st, layers = slice_models([(_placed(tris, params), pl)], p2, source=ref, segments=segments)
                    else:
                        gcode, st, layers = slice_triangles(tris, params, source=ref, segments=segments)
                preview = {k: _f32(v) for k, v in preview_segments(layers).items()}
                res = {"gcode": gcode, "stats": {**{k: v for k, v in vars(st).items()}, "time": time.perf_counter() - t0},
                       "name": name, "preview": preview}
            except ImportError as exc:
                self.send_json({"error": f"slicer needs shapely: pip install shapely ({exc})"}, 500)
                return
            except ValueError as exc:
                self.send_json({"error": str(exc)}, 400)
                return
            except Exception as exc:  # noqa: BLE001
                self.send_json({"error": f"{type(exc).__name__}: {exc}"}, 500)
                return
            self.send_json(res)
            return
        if path != "/api/translate":
            self.send_json({"error": "not found"}, 404)
            return
        try:
            body = self.read_json()
            gcode = body.get("gcode", "")
            if not isinstance(gcode, str) or not gcode.strip():
                raise ValueError("no G-code supplied")
            result = run_translation(gcode, body.get("filename", "PROG"), body.get("options") or {})
        except ValueError as exc:
            self.send_json({"error": str(exc)}, 400)
            return
        except Exception as exc:  # noqa: BLE001 - report to the UI
            self.send_json({"error": f"{type(exc).__name__}: {exc}"}, 500)
            return
        self.send_json(result)


def _frame_mode(options: dict) -> str:
    """declare (default) | data | literal; the old base_literal / tool_literal switches mean literal."""
    mode = str(options.get("frame_mode") or "").lower()
    if mode in ("declare", "data", "literal"):
        return mode
    if _opt(options, "base_literal", False, bool) or _opt(options, "tool_literal", False, bool):
        return "literal"
    return "declare"


def _frame_known(options: dict, which: str) -> bool:
    """With the old per-frame switches only the switched-on frame is written literally;
    the other one stays a plain TOOL_DATA[n] / BASE_DATA[n] selection."""
    if str(options.get("frame_mode") or "").lower() in ("declare", "data", "literal"):
        return True
    old = _opt(options, "base_literal", False, bool) or _opt(options, "tool_literal", False, bool)
    return _opt(options, f"{which}_literal", False, bool) if old else True


def free_port(preferred: int) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # the HTTP server itself sets allow_reuse_address, so lingering
        # TIME_WAIT sockets from a previous run must not push us off the port
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", preferred))
            return preferred
        except OSError:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]


def make_server(host: str = "127.0.0.1", port: int = 0, verbose: bool = False) -> ThreadingHTTPServer:
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    srv.verbose = verbose  # type: ignore[attr-defined]
    return srv


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="gcode2krl-gui", description="Local web GUI for gcode2krl")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    port = free_port(args.port) if args.host == "127.0.0.1" else args.port
    srv = make_server(args.host, port, args.verbose)
    url = f"http://{args.host}:{srv.server_address[1]}/"
    print(f"gcode2krl GUI: {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
