"""Import KUKA robot definitions from ROS-Industrial URDF/xacro files and STL meshes.

The ROS-Industrial ``kuka_experimental`` packages (Apache-2.0) carry joint
origins and collision/visual meshes converted from KUKA CAD data. This module
turns one of those macros into a gcode2krl robot definition:

  * joint origins -> link parameters (d1, a1, a2, a3, d4, d6) and per-link frames,
  * limits / velocities -> axis data (flagged as "urdf" source, to be checked
    against the KUKA data sheet),
  * collision STL per link -> conservative capsule set (all mesh vertices are
    enclosed; capsules overlap along the link axis).

    gcode2krl-import --xacro kr120r2500pro_macro.xacro --out gcode2krl/robots --key KR120R2500PRO ...
    gcode2krl-import --kuka-experimental /path/to/clone --out gcode2krl/robots   # everything compatible
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

from .meshio import load_stl, save_stl

JOINTS = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
EXPECTED_AXES = [(0, 0, -1), (0, 1, 0), (0, 1, 0), (-1, 0, 0), (0, 1, 0), (-1, 0, 0)]
LINK_NAMES_PL = ["podstawa", "kolumna", "ramie", "przedramie", "nadgarstek", "A5", "kolnierz"]


def _eval_expr(text: str) -> float:
    """Evaluate xacro expressions like ${radians(-185)} or ${-radians(5)}."""
    text = text.strip()
    m = re.fullmatch(r"\$\{(.*)\}", text)
    if m:
        expr = m.group(1)
        expr = expr.replace("radians", "math.radians").replace("pi", "math.pi")
        if not re.fullmatch(r"[\d\s.+\-*/()a-z_.]+", expr):
            raise ValueError(f"unsupported xacro expression {text!r}")
        return float(eval(expr, {"__builtins__": {}}, {"math": math}))  # noqa: S307 - whitelisted characters
    return float(text)


def _xyz(s: str) -> list[float]:
    return [_eval_expr(v) for v in s.split()]


def parse_xacro(path: str) -> dict:
    """Return joints (origin m, axis, limits deg, velocity deg/s), links (mesh paths), flange offset."""
    text = open(path, encoding="utf-8").read()
    text = text.replace("${prefix}", "")
    text = re.sub(r"<xacro:macro[^>]*>", "", text).replace("</xacro:macro>", "")
    text = re.sub(r"<xacro:[^>]*/>", "", text)
    text = re.sub(r"<(/?)xacro:[a-zA-Z_]+", r"<\1xacro_", text)
    root = ET.fromstring(text)
    joints: dict[str, dict] = {}
    for j in root.iter("joint"):
        name = j.get("name")
        origin = j.find("origin")
        xyz = _xyz(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0]
        rpy = _xyz(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]
        axis = j.find("axis")
        lim = j.find("limit")
        joints[name] = {
            "type": j.get("type"), "xyz": xyz, "rpy": rpy,
            "axis": _xyz(axis.get("xyz")) if axis is not None else None,
            "parent": j.find("parent").get("link") if j.find("parent") is not None else None,
            "child": j.find("child").get("link") if j.find("child") is not None else None,
            "lower": math.degrees(_eval_expr(lim.get("lower"))) if lim is not None and lim.get("lower") else None,
            "upper": math.degrees(_eval_expr(lim.get("upper"))) if lim is not None and lim.get("upper") else None,
            "velocity": math.degrees(_eval_expr(lim.get("velocity"))) if lim is not None and lim.get("velocity") else None,
        }
    links: dict[str, dict] = {}
    for l in root.iter("link"):
        d = {}
        for tag in ("visual", "collision"):
            el = l.find(tag)
            if el is not None:
                mesh = el.find("geometry/mesh")
                origin = el.find("origin")
                if mesh is not None:
                    d[tag] = {"file": mesh.get("filename"), "scale": mesh.get("scale"),
                              "xyz": _xyz(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0],
                              "rpy": _xyz(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]}
        links[l.get("name")] = d
    return {"joints": joints, "links": links}


def chain_from_xacro(parsed: dict) -> dict:
    """Validate the standard KUKA 6R structure and extract origins (mm)."""
    j = parsed["joints"]
    for name, axis in zip(JOINTS, EXPECTED_AXES):
        if name not in j:
            raise ValueError(f"missing {name}")
        if j[name]["rpy"] != [0, 0, 0] and any(abs(v) > 1e-9 for v in j[name]["rpy"]):
            raise ValueError(f"{name}: rotated joint origins (rpy) are not supported")
        ax = tuple(round(v) for v in j[name]["axis"])
        if ax != axis:
            raise ValueError(f"{name}: axis {ax} differs from the KUKA convention {axis}")
    flange = None
    for name, d in j.items():
        if d["type"] == "fixed" and d["parent"] == "link_6" and d["child"] == "flange":
            flange = d
    if flange is None:
        raise ValueError("no fixed joint link_6 -> flange")
    if any(abs(v) > 1e-9 for v in flange["rpy"]):
        raise ValueError("flange joint with rotation is not supported")
    origins = [[v * 1000.0 for v in j[n]["xyz"]] for n in JOINTS] + [[v * 1000.0 for v in flange["xyz"]]]
    for k, o in enumerate(origins):
        if abs(o[1]) > 1e-6:
            raise ValueError(f"joint origin {k} has a Y offset; not a planar KUKA chain")
    if abs(origins[0][0]) > 1e-6 or abs(origins[1][2]) > 1e-6 or abs(origins[2][2]) > 1e-6:
        raise ValueError("joint origins do not follow the KUKA pattern (a1: z only, a2/a3: x only)")
    if abs(origins[4][2]) > 1e-6 or abs(origins[5][2]) > 1e-6 or abs(origins[6][2]) > 1e-6:
        raise ValueError("wrist joint origins must lie on the forearm axis")
    limits = [[j[n]["lower"], j[n]["upper"]] for n in JOINTS]
    speeds = [j[n]["velocity"] for n in JOINTS]
    return {"origins": origins, "limits": limits, "speeds": speeds}


# ------------------------------------------------------------ capsule fit
def subdivide(tris: np.ndarray, max_edge: float) -> np.ndarray:
    """Split triangles (vectorised) until every edge is <= max_edge."""
    t = np.asarray(tris, dtype=float)
    for _ in range(12):
        e = np.stack([np.linalg.norm(t[:, 1] - t[:, 0], axis=1), np.linalg.norm(t[:, 2] - t[:, 1], axis=1),
                      np.linalg.norm(t[:, 0] - t[:, 2], axis=1)], axis=1)
        k = e.argmax(axis=1)
        big = e[np.arange(len(t)), k] > max_edge
        if not big.any():
            break
        keep = t[~big]
        tb = t[big]
        kb = k[big]
        i = kb
        j = (kb + 1) % 3
        m = (kb + 2) % 3
        rows = np.arange(len(tb))
        pi, pj, pm = tb[rows, i], tb[rows, j], tb[rows, m]
        mid = (pi + pj) / 2.0
        t1 = np.stack([pi, mid, pm], axis=1)
        t2 = np.stack([mid, pj, pm], axis=1)
        t = np.concatenate([keep, t1, t2])
    return t


def kmeans(points: np.ndarray, k: int, iters: int = 25, seed: int = 0) -> np.ndarray:
    """Plain k-means with farthest-point seeding; returns labels."""
    rng = np.random.default_rng(seed)
    centers = [points[rng.integers(len(points))]]
    d = np.linalg.norm(points - centers[0], axis=1)
    for _ in range(1, k):
        centers.append(points[int(np.argmax(d))])
        d = np.minimum(d, np.linalg.norm(points - centers[-1], axis=1))
    c = np.array(centers)
    labels = np.zeros(len(points), dtype=int)
    for _ in range(iters):
        dist = np.linalg.norm(points[:, None, :] - c[None, :, :], axis=2)
        new = dist.argmin(axis=1)
        if np.array_equal(new, labels) and _ > 0:
            break
        labels = new
        for j in range(k):
            m = labels == j
            if m.any():
                c[j] = points[m].mean(axis=0)
    return labels


def _capsule_along(v: np.ndarray, axis: np.ndarray, pad: float) -> dict:
    """Capsule along ``axis`` enclosing ``v``; the axis shortening at the ends is
    chosen (from a few candidates) to minimise the capsule volume, so flat-ended
    housings keep a long thin axis instead of a short fat one."""
    c = v.mean(axis=0)
    q = v - c
    t = q @ axis
    radial = np.linalg.norm(q - np.outer(t, axis), axis=1)
    r0 = float(radial.max()) + pad
    t0, t1 = float(t.min()), float(t.max())
    half = (t1 - t0) / 2.0
    best = None
    for s_frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        sh = min(r0 * s_frac, half)
        a = c + (t0 + sh) * axis
        b = c + (t1 - sh) * axis
        ab = b - a
        l2 = float(ab @ ab)
        tt = np.clip(((v - a) @ ab) / l2, 0, 1) if l2 > 1e-12 else np.zeros(len(v))
        d = np.linalg.norm(v - (a + tt[:, None] * ab), axis=1)
        r = float(d.max()) + pad
        cap = {"a": a.tolist(), "b": b.tolist(), "radius": r}
        vol = capsule_volume(cap)
        if best is None or vol < best[0]:
            best = (vol, cap)
    return best[1]


def _capsule_for(v: np.ndarray, pad: float) -> dict:
    """Capsule enclosing the points with the smallest radius among candidate axes
    (PCA axis, the three link-frame axes and the two in-plane diagonals)."""
    c = v.mean(axis=0)
    q = v - c
    cands = [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])]
    if len(v) >= 3:
        _, _, vt = np.linalg.svd(q, full_matrices=False)
        cands.append(vt[0])
    best = None
    for ax in cands:
        cap = _capsule_along(v, ax / np.linalg.norm(ax), pad)
        if best is None or cap["radius"] < best["radius"]:
            best = cap
    return best


def capsule_volume(c: dict) -> float:
    L = float(np.linalg.norm(np.array(c["b"]) - np.array(c["a"])))
    r = c["radius"]
    return math.pi * r * r * L + 4.0 / 3.0 * math.pi * r ** 3


def _bins_along(v: np.ndarray, axis: np.ndarray, pad: float, bin_len: float, merge_tol: float) -> list[dict]:
    """Capsules from axial bins, merged while merging does not fatten the result."""
    c = v.mean(axis=0)
    t = (v - c) @ axis
    t0, t1 = float(t.min()), float(t.max())
    n_bins = max(1, int(math.ceil((t1 - t0) / bin_len)))
    edges = np.linspace(t0, t1, n_bins + 1)
    groups = []
    for k in range(n_bins):
        m = (t >= edges[k] - 1e-9) & (t <= edges[k + 1] + 1e-9)
        if m.any():
            groups.append(np.flatnonzero(m))
    caps = [_capsule_along(v[g], axis, pad) for g in groups]
    changed = True
    while changed and len(groups) > 1:
        changed = False
        k = 0
        while k < len(groups) - 1:
            union = np.concatenate([groups[k], groups[k + 1]])
            cap = _capsule_along(v[union], axis, pad)
            if cap["radius"] <= merge_tol * min(caps[k]["radius"], caps[k + 1]["radius"]):
                groups[k : k + 2] = [union]
                caps[k : k + 2] = [cap]
                changed = True
            else:
                k += 1
    return caps


def fit_capsules(tris: np.ndarray, max_edge: float = 16.0, pad: float = 1.0, bin_len: float = 80.0,
                 merge_tol: float = 1.15) -> list[dict]:
    """Enclose the whole mesh surface with capsules.

    Triangles are subdivided to ``max_edge`` so every surface point lies within
    max_edge/2 of a vertex, and the capsules are padded by max_edge/2 + pad.
    Along a candidate axis (link-frame axes and the PCA axis) the vertices are
    cut into ``bin_len`` slices, each enclosed by a capsule; neighbouring slices
    are merged while the merged capsule is not more than ``merge_tol`` times
    fatter than the thinner of the two (a thin tube never inherits the radius
    of the motor housing next to it). The axis with the smallest total
    cylinder volume wins, so a thin tube next to a big motor housing keeps a
    thin capsule instead of inheriting the housing radius.
    """
    dense = subdivide(np.asarray(tris, dtype=float), max_edge)
    v = np.unique(np.round(dense.reshape(-1, 3), 3), axis=0)
    full_pad = max_edge / 2.0 + pad
    cands = [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])]
    if len(v) >= 3:
        _, _, vt = np.linalg.svd(v - v.mean(axis=0), full_matrices=False)
        cands.append(vt[0])
    best = None
    for ax in cands:
        caps = _bins_along(v, ax / np.linalg.norm(ax), full_pad, bin_len, merge_tol)
        score = sum(c["radius"] ** 2 * (np.linalg.norm(np.array(c["b"]) - np.array(c["a"])) + c["radius"]) for c in caps)
        if best is None or score < best[0]:
            best = (score, caps)
    return best[1]


def fit_obb(tris: np.ndarray, pad: float = 1.0) -> dict:
    """Oriented bounding box of a mesh: the smaller of the axis-aligned box and the
    PCA-aligned box. Returns {center, rot (row-major 3x3, columns = axes), half}."""
    v = np.unique(tris.reshape(-1, 3), axis=0)
    c = v.mean(axis=0)
    _, _, vt = np.linalg.svd(v - c, full_matrices=False)
    pca = vt.T
    if np.linalg.det(pca) < 0:
        pca[:, 2] *= -1
    best = None
    for rot in (np.eye(3), pca):
        q = (v - c) @ rot
        lo, hi = q.min(axis=0), q.max(axis=0)
        half = (hi - lo) / 2.0 + pad
        vol = float(np.prod(half))
        if best is None or vol < best[0]:
            center = c + rot @ ((lo + hi) / 2.0)
            best = (vol, {"type": "box", "center": center.tolist(), "rot": rot.ravel().tolist(), "half": half.tolist()})
    return best[1]


def surface_outside(tris: np.ndarray, caps: list[dict], max_edge: float = 8.0) -> float:
    """Largest distance of any (densely sampled) surface point outside the capsule union."""
    dense = subdivide(np.asarray(tris, dtype=float), max_edge)
    v = dense.reshape(-1, 3)
    best = np.full(len(v), np.inf)
    for cdef in caps:
        a, b, r = np.array(cdef["a"]), np.array(cdef["b"]), cdef["radius"]
        ab = b - a
        l2 = float(ab @ ab)
        tt = np.clip(((v - a) @ ab) / l2, 0, 1) if l2 > 1e-12 else np.zeros(len(v))
        d = np.linalg.norm(v - (a + tt[:, None] * ab), axis=1) - r
        best = np.minimum(best, d)
    return float(max(0.0, best.max()))


vertices_inside = surface_outside


# --------------------------------------------------------------- builder
def resolve_mesh(filename: str, xacro_path: str, repo_dir: str | None) -> str | None:
    m = re.match(r"package://([^/]+)/(.*)", filename or "")
    if not m:
        p = os.path.join(os.path.dirname(xacro_path), filename or "")
        return p if os.path.isfile(p) else None
    pkg, rel = m.group(1), m.group(2)
    candidates = []
    if repo_dir:
        candidates.append(os.path.join(repo_dir, pkg, rel))
    candidates.append(os.path.join(os.path.dirname(os.path.dirname(xacro_path)), rel))  # <pkg>/urdf/.. -> <pkg>/
    for cnd in candidates:
        if os.path.isfile(cnd):
            return cnd
    return None


def build_definition(xacro_path: str, key: str, name: str, repo_dir: str | None = None, out_dir: str | None = None,
                     sources: dict | None = None, overrides: dict | None = None, forearm_extend: float = 0.0) -> dict:
    parsed = parse_xacro(xacro_path)
    chain = chain_from_xacro(parsed)
    overrides = overrides or {}
    if forearm_extend:
        chain["origins"][3][0] += forearm_extend  # joint_a4 origin moves along the forearm
    d = {
        "key": key, "name": name, "origins": chain["origins"],
        "limits": overrides.get("limits", chain["limits"]), "speeds": overrides.get("speeds", chain["speeds"]),
        "payload": overrides.get("payload", 0.0), "reach": overrides.get("reach", 0.0), "mass": overrides.get("mass", 0.0),
        "home": overrides.get("home", [0, -90, 90, 0, 0, 0]),
        "sources": sources or {"geometry": f"ROS-Industrial kuka_experimental {os.path.basename(xacro_path)} (converted from KUKA CAD)",
                               "axes": f"ROS-Industrial kuka_experimental {os.path.basename(xacro_path)} (check against the KUKA data sheet)"},
        "meshes": {}, "capsules": [], "notes": overrides.get("notes", []),
    }
    link_names = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
    mesh_dir = os.path.join(out_dir, "meshes", key.lower()) if out_dir else None
    if mesh_dir:
        os.makedirs(mesh_dir, exist_ok=True)
    for idx, ln in enumerate(link_names):
        info = parsed["links"].get(ln, {})
        src = info.get("collision") or info.get("visual")
        if not src:
            continue
        path = resolve_mesh(src["file"], xacro_path, repo_dir)
        if path is None or not path.lower().endswith(".stl"):
            vis = info.get("visual")
            path = resolve_mesh(vis["file"], xacro_path, repo_dir) if vis else None
            if path is None or not path.lower().endswith(".stl"):
                d["notes"].append(f"{ln}: no STL mesh found, procedural capsule used")
                continue
        tris = load_stl(path) * 1000.0  # m -> mm
        if any(abs(v) > 1e-9 for v in src.get("xyz", [0, 0, 0]) + src.get("rpy", [0, 0, 0])):
            d["notes"].append(f"{ln}: mesh origin offset ignored")
        if forearm_extend and ln == "link_3":
            x = tris[:, :, 0]
            cut = float(np.percentile(x, 60))
            tris[:, :, 0] = np.where(x > cut, x + forearm_extend, x)
            d["notes"].append(f"link_3 mesh stretched by {forearm_extend:g} mm beyond x={cut:.0f} mm (longer forearm variant)")
        if idx == 0:
            box = fit_obb(tris)
            box["link"] = 0
            box["name"] = LINK_NAMES_PL[0]
            d["capsules"].append(box)
        else:
            caps = fit_capsules(tris)
            for c in caps:
                c["link"] = idx
                c["name"] = f"{LINK_NAMES_PL[idx]}"
            d["capsules"].extend(caps)
        if mesh_dir:
            fname = f"{ln}.stl"
            save_stl(os.path.join(mesh_dir, fname), tris, ln)
            d["meshes"][str(idx)] = f"meshes/{key.lower()}/{fname}"
    return d


def import_kuka_experimental(repo_dir: str, out_dir: str, only: list[str] | None = None) -> list[str]:
    """Import every macro with the standard KUKA chain; returns written keys."""
    written = []
    for pkg in sorted(os.listdir(repo_dir)):
        urdf = os.path.join(repo_dir, pkg, "urdf")
        if not os.path.isdir(urdf):
            continue
        for f in sorted(os.listdir(urdf)):
            if not f.endswith("_macro.xacro"):
                continue
            key = f[: -len("_macro.xacro")].upper().replace("-", "_")
            if only and key not in only:
                continue
            try:
                d = build_definition(os.path.join(urdf, f), key, f"KUKA {f[:-len('_macro.xacro')]} (ROS-Industrial URDF)",
                                     repo_dir=repo_dir, out_dir=out_dir)
            except ValueError as exc:
                print(f"skip {f}: {exc}", file=sys.stderr)
                continue
            with open(os.path.join(out_dir, f"{key}.json"), "w", encoding="utf-8") as fh:
                json.dump(d, fh, indent=1)
            written.append(key)
    return written


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="gcode2krl-import", description=__doc__.split("\n\n")[0])
    ap.add_argument("--xacro", help="one *_macro.xacro file to import")
    ap.add_argument("--key", help="robot key for --xacro (default from file name)")
    ap.add_argument("--name", help="display name for --xacro")
    ap.add_argument("--kuka-experimental", help="path to a clone of ros-industrial/kuka_experimental: import all compatible robots")
    ap.add_argument("--out", required=True, help="output directory for <KEY>.json and meshes/")
    ap.add_argument("--only", nargs="*", help="keys to import from the repository")
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    if args.xacro:
        key = args.key or os.path.basename(args.xacro).replace("_macro.xacro", "").upper()
        d = build_definition(args.xacro, key, args.name or key, repo_dir=args.kuka_experimental, out_dir=args.out)
        with open(os.path.join(args.out, f"{key}.json"), "w", encoding="utf-8") as fh:
            json.dump(d, fh, indent=1)
        print(f"written {key}: {len(d['capsules'])} capsules, meshes for links {sorted(d['meshes'])}")
        return 0
    if args.kuka_experimental:
        keys = import_kuka_experimental(args.kuka_experimental, args.out, args.only)
        print("written: " + ", ".join(keys))
        return 0
    ap.error("give --xacro or --kuka-experimental")
    return 2


if __name__ == "__main__":
    sys.exit(main())
