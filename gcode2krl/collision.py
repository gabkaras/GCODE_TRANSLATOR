"""Collision checking for the simplified robot model.

Geometry:
  * robot links are capsules (segment + radius) attached to link frames,
  * the tool is two capsules: the holder/shank (collides with everything) and
    the cutter (collides with everything except obstacles flagged ``workpiece``),
  * obstacles are convex solids given in the robot or BASE frame: oriented
    boxes, vertical cylinders, spheres and the floor (half-space).

Distances are exact: segment-segment (self collision) and segment-convex solid
(golden-section search on the convex clearance function along the segment,
converged to 1e-6 mm). Everything is vectorised over all path samples.

Continuity: between two consecutive samples every link point moves at most
as far as the capsule end points, so an interval is provably clear when
max(c_i, c_i+1) - delta_i >= margin. Intervals that cannot be proven clear
are bisected (IK at the midpoint) up to ``max_refine`` rounds; what is still
unresolved at sub-millimetre motion is reported as "unresolved", never
silently accepted.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

from .robot import DEG, RobotModel, abc_to_rot, frame, rot_x, rot_y, rot_z, FLANGE0

GOLDEN = (math.sqrt(5.0) - 1.0) / 2.0


# ------------------------------------------------------------------ obstacles
@dataclass
class Obstacle:
    name: str
    kind: str  # box | cylinder | sphere | floor
    center: np.ndarray  # robot frame
    rot: np.ndarray  # 3x3, robot frame (boxes)
    half: np.ndarray  # box half sizes
    radius: float = 0.0
    height: float = 0.0  # cylinder: full height along local Z (center = mid)
    workpiece: bool = False
    frame: str = "robot"
    raw: str = ""

    def distance(self, p: np.ndarray) -> np.ndarray:
        """Signed-ish distance from points (N,3) to the solid (0 inside)."""
        q = (p - self.center) @ self.rot  # into local frame
        if self.kind == "box":
            d = np.abs(q) - self.half
            outside = np.linalg.norm(np.maximum(d, 0.0), axis=1)
            inside = np.minimum(np.max(d, axis=1), 0.0)
            return outside + inside
        if self.kind == "cylinder":
            r = np.hypot(q[:, 0], q[:, 1]) - self.radius
            z = np.abs(q[:, 2]) - self.height / 2.0
            outside = np.hypot(np.maximum(r, 0.0), np.maximum(z, 0.0))
            inside = np.minimum(np.maximum(r, z), 0.0)
            return outside + inside
        if self.kind == "sphere":
            return np.linalg.norm(q, axis=1) - self.radius
        if self.kind == "floor":
            return q[:, 2]
        raise ValueError(self.kind)

    def to_dict(self) -> dict:
        return {"name": self.name, "kind": self.kind, "center": self.center.tolist(), "rot": self.rot.ravel().tolist(),
                "half": self.half.tolist(), "radius": self.radius, "height": self.height, "workpiece": self.workpiece,
                "frame": self.frame}


def _to_robot(frame_name: str, base: np.ndarray, p: np.ndarray, r: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if frame_name == "robot":
        return p, r
    t = base @ np.append(p, 1.0)
    return t[:3], base[:3, :3] @ r


def obstacle_from_dict(d: dict, base: np.ndarray) -> Obstacle:
    kind = str(d.get("kind", "box")).lower()
    fr = str(d.get("frame", "robot")).lower()
    if fr not in ("robot", "base"):
        raise ValueError(f"obstacle {d.get('name')}: frame must be robot or base")
    name = str(d.get("name") or kind)
    abc = d.get("abc", (0, 0, 0))
    rot = abc_to_rot(*[float(v) for v in abc])
    center = np.array(d.get("center", (0, 0, 0)), dtype=float)
    if kind == "floor":
        z = float(d.get("z", center[2] if len(center) else 0.0))
        center = np.array([0.0, 0.0, z])
        c, r = _to_robot(fr, base, center, np.eye(3))
        # floor normal is local +Z: keep orientation from the frame
        return Obstacle(name, "floor", c, r, np.zeros(3), frame=fr, raw=d.get("raw", ""))
    c, r = _to_robot(fr, base, center, rot)
    if kind == "box":
        size = np.array(d.get("size", (100, 100, 100)), dtype=float)
        if size.shape != (3,) or np.any(size <= 0):
            raise ValueError(f"obstacle {name}: box size must be three positive numbers")
        return Obstacle(name, "box", c, r, size / 2.0, workpiece=bool(d.get("workpiece", False)), frame=fr, raw=d.get("raw", ""))
    if kind == "cylinder":
        rad = float(d.get("radius", 100))
        h = float(d.get("height", 100))
        if rad <= 0 or h <= 0:
            raise ValueError(f"obstacle {name}: cylinder needs positive radius and height")
        return Obstacle(name, "cylinder", c, r, np.zeros(3), rad, h, bool(d.get("workpiece", False)), fr, d.get("raw", ""))
    if kind == "sphere":
        rad = float(d.get("radius", 100))
        if rad <= 0:
            raise ValueError(f"obstacle {name}: sphere needs a positive radius")
        return Obstacle(name, "sphere", c, r, np.zeros(3), rad, 0.0, bool(d.get("workpiece", False)), fr, d.get("raw", ""))
    raise ValueError(f"obstacle {name}: unknown kind {kind}")


OBSTACLE_HELP = """# one obstacle per line:  kind name frame params... [workpiece]
# box NAME robot|base  cx cy cz  sx sy sz  [a b c]      (centre, full size, rotation)
# cylinder NAME robot|base cx cy cz radius height       (vertical, centre = middle)
# sphere NAME robot|base cx cy cz radius
# floor NAME robot|base z
floor posadzka robot 0
box stol robot 1300 0 240 900 900 480
box material base 50 30 -10 140 100 20 workpiece"""


def parse_obstacles(text: str) -> list[dict]:
    """Parse the line format (see OBSTACLE_HELP) into obstacle dicts."""
    out: list[dict] = []
    for ln, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = re.split(r"[\s,]+", line)
        kind = parts[0].lower()
        workpiece = False
        if parts and parts[-1].lower() in ("workpiece", "material", "stock"):
            workpiece = True
            parts = parts[:-1]
        try:
            if kind == "floor":
                name, fr, z = parts[1], parts[2].lower(), float(parts[3])
                out.append({"kind": "floor", "name": name, "frame": fr, "z": z, "raw": raw.strip()})
            elif kind == "box":
                name, fr = parts[1], parts[2].lower()
                nums = [float(v) for v in parts[3:]]
                if len(nums) not in (6, 9):
                    raise ValueError("box needs cx cy cz sx sy sz [a b c]")
                out.append({"kind": "box", "name": name, "frame": fr, "center": nums[0:3], "size": nums[3:6],
                            "abc": nums[6:9] if len(nums) == 9 else (0, 0, 0), "workpiece": workpiece, "raw": raw.strip()})
            elif kind in ("cylinder", "cyl"):
                name, fr = parts[1], parts[2].lower()
                nums = [float(v) for v in parts[3:]]
                if len(nums) != 5:
                    raise ValueError("cylinder needs cx cy cz radius height")
                out.append({"kind": "cylinder", "name": name, "frame": fr, "center": nums[0:3], "radius": nums[3],
                            "height": nums[4], "workpiece": workpiece, "raw": raw.strip()})
            elif kind == "sphere":
                name, fr = parts[1], parts[2].lower()
                nums = [float(v) for v in parts[3:]]
                if len(nums) != 4:
                    raise ValueError("sphere needs cx cy cz radius")
                out.append({"kind": "sphere", "name": name, "frame": fr, "center": nums[0:3], "radius": nums[3],
                            "workpiece": workpiece, "raw": raw.strip()})
            else:
                raise ValueError(f"unknown obstacle kind '{kind}'")
        except (IndexError, ValueError) as exc:
            raise ValueError(f"obstacles line {ln}: {exc}") from exc
    return out


# ------------------------------------------------------------------- capsules
@dataclass
class Capsule:
    name: str
    link: int  # index into batch_fk frames
    a: np.ndarray  # local end points
    b: np.ndarray
    radius: float
    kind: str = "body"  # body | tool | cutter


LINK_PL = ["podstawa", "kolumna", "ramie", "przedramie", "nadgarstek", "A5", "kolnierz", "flansza"]
FLANGE_LINK = 7


def procedural_capsules(robot: RobotModel) -> list[Capsule]:
    """Fallback capsule model when no mesh-derived capsules exist (sizes from ``visual``)."""
    V = robot.visual or {}
    br, bh = V.get("base_r", 300), V.get("base_h", 250)
    col, arm, fa, wr = V.get("column", 400), V.get("arm", 240), V.get("forearm", 190), V.get("wrist", 150)
    o = robot.origins
    return [
        Capsule("podstawa", 0, np.array([0, 0, 0.0]), np.array([0, 0, bh * 0.6]), br * 0.95),
        Capsule("kolumna", 1, np.array([o[1][0] * 0.3, 0, bh - o[0][2]]), np.array([o[1][0], 0, 0.0]), col * 0.6),
        Capsule("ramie", 2, np.array([0.0, 0, 0]), np.array([o[2][0], 0, 0]), arm * 0.6),
        Capsule("przedramie", 3, np.array([fa * 0.2, 0, o[3][2]]), np.array([o[3][0] - wr * 0.8, 0, o[3][2]]), fa * 0.6),
        Capsule("nadgarstek", 4, np.array([0.0, -wr * 0.5, 0]), np.array([o[4][0], wr * 0.5, 0]), wr * 0.6),
        Capsule("A5", 5, np.array([0.0, 0, 0]), np.array([max(o[5][0] + o[6][0] - 10.0, 1.0), 0, 0]), wr * 0.42),
    ]


def robot_capsules(robot: RobotModel, tool: np.ndarray, tool_radius: float = 25.0, cutter_len: float = 20.0,
                   cutter_radius: float = 6.0) -> list[Capsule]:
    """Body capsules (mesh-derived when the definition has them) + tool capsules on the flange."""
    caps: list[Capsule] = []
    if robot.capsules:
        for c in robot.capsules:
            if c.get("type") == "box":
                continue  # static base box, handled by base_boxes()
            caps.append(Capsule(str(c.get("name", LINK_PL[int(c["link"])])), int(c["link"]), np.array(c["a"], float),
                                np.array(c["b"], float), float(c["radius"])))
    else:
        caps = procedural_capsules(robot)
    tcp = tool[:3, 3]
    length = float(np.linalg.norm(tcp))
    if length > 1e-6:
        d = tcp / length
        cut = min(cutter_len, length)
        cutter_start = d * (length - cut)
        shank_end = d * max(0.0, length - cut - tool_radius)
        caps.append(Capsule("narzedzie", FLANGE_LINK, np.zeros(3), shank_end, tool_radius, "tool"))
        caps.append(Capsule("ostrze", FLANGE_LINK, cutter_start, tcp, cutter_radius, "cutter"))
    else:
        caps.append(Capsule("narzedzie", FLANGE_LINK, np.zeros(3), np.array([0, 0, 1e-3]), tool_radius, "tool"))
    return caps


def base_boxes(robot: RobotModel) -> list[Obstacle]:
    """The robot base (link 0, static) as oriented boxes in the robot frame."""
    out = []
    for c in robot.capsules:
        if c.get("type") == "box" and int(c.get("link", 0)) == 0:
            rot = np.array(c["rot"], float).reshape(3, 3)
            out.append(Obstacle(str(c.get("name", "podstawa")), "box", np.array(c["center"], float), rot,
                                np.array(c["half"], float), frame="robot"))
    return out


def self_pairs(caps: list[Capsule], disabled: set[tuple[int, int]] | None = None) -> list[tuple[int, int]]:
    """Capsule index pairs for self collision: links at least two joints apart, tool vs links 0-3."""
    disabled = disabled or set()
    out = []
    for i, a in enumerate(caps):
        for j, b in enumerate(caps):
            if j <= i:
                continue
            la, lb = a.link, b.link
            if la == lb:
                continue
            hi, lo = max(la, lb), min(la, lb)
            if hi == FLANGE_LINK:
                if lo > 3:
                    continue
            elif hi - lo < 2:
                continue
            if (lo, hi) in disabled:
                continue
            out.append((i, j))
    return out


# --------------------------------------------------------------- batch FK
def _rx(a):
    c, s = np.cos(a), np.sin(a)
    m = np.zeros((len(a), 3, 3))
    m[:, 0, 0] = 1
    m[:, 1, 1] = c
    m[:, 1, 2] = -s
    m[:, 2, 1] = s
    m[:, 2, 2] = c
    return m


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    m = np.zeros((len(a), 3, 3))
    m[:, 1, 1] = 1
    m[:, 0, 0] = c
    m[:, 0, 2] = s
    m[:, 2, 0] = -s
    m[:, 2, 2] = c
    return m


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    m = np.zeros((len(a), 3, 3))
    m[:, 2, 2] = 1
    m[:, 0, 0] = c
    m[:, 0, 1] = -s
    m[:, 1, 0] = s
    m[:, 1, 1] = c
    return m


def batch_fk(robot: RobotModel, q_deg: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """Frames (R (N,3,3), p (N,3)) for [base, link_1..link_6, flange] - same as robot.forward."""
    q = np.asarray(q_deg, dtype=float) * DEG
    n = len(q)
    o = [np.array(v, dtype=float) for v in robot.origins]
    r = np.broadcast_to(np.eye(3), (n, 3, 3)).copy()
    p = np.zeros((n, 3))
    out = [(r, p)]
    rots = [_rz(-q[:, 0]), _ry(q[:, 1]), _ry(q[:, 2]), _rx(-q[:, 3]), _ry(q[:, 4]), _rx(-q[:, 5])]
    for k in range(6):
        p = p + r @ o[k]
        r = r @ rots[k]
        out.append((r, p))
    p = p + r @ o[6]
    r = r @ FLANGE0
    out.append((r, p))
    return out


def capsule_endpoints(frames, caps: list[Capsule]) -> np.ndarray:
    """(N, K, 2, 3) world end points of all capsules."""
    n = frames[0][1].shape[0]
    pts = np.zeros((n, len(caps), 2, 3))
    for k, c in enumerate(caps):
        r, p = frames[c.link]
        pts[:, k, 0] = p + r @ c.a
        pts[:, k, 1] = p + r @ c.b
    return pts


# ------------------------------------------------------------- distances
def seg_seg_distance(p0, p1, q0, q1) -> np.ndarray:
    """Vectorised closest distance between segments p0-p1 and q0-q1, shapes (N,3)."""
    d1 = p1 - p0
    d2 = q1 - q0
    r = p0 - q0
    a = np.einsum("ij,ij->i", d1, d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    eps = 1e-12
    s = np.zeros_like(a)
    t = np.zeros_like(a)
    both_pts = (a <= eps) & (e <= eps)
    only_p = (a <= eps) & ~both_pts
    only_q = (e <= eps) & ~both_pts
    gen = ~(both_pts | only_p | only_q)
    # p degenerate
    t[only_p] = np.clip(f[only_p] / e[only_p], 0, 1)
    c = np.einsum("ij,ij->i", d1, r)
    # q degenerate
    s[only_q] = np.clip(-c[only_q] / a[only_q], 0, 1)
    # general
    b = np.einsum("ij,ij->i", d1, d2)
    denom = a * e - b * b
    sg = np.where(denom > eps, np.clip((b * f - c * e) / np.where(denom > eps, denom, 1), 0, 1), 0.0)
    tg = (b * sg + f) / np.where(e > eps, e, 1)
    # clamp t and recompute s
    tneg = tg < 0
    tpos = tg > 1
    tg = np.clip(tg, 0, 1)
    sg = np.where(tneg, np.clip(-c / np.where(a > eps, a, 1), 0, 1), sg)
    sg = np.where(tpos, np.clip((b - c) / np.where(a > eps, a, 1), 0, 1), sg)
    s[gen] = sg[gen]
    t[gen] = tg[gen]
    cp = p0 + s[:, None] * d1
    cq = q0 + t[:, None] * d2
    return np.linalg.norm(cp - cq, axis=1)


def seg_convex_distance(p0: np.ndarray, p1: np.ndarray, dist_fn, iters: int = 48) -> np.ndarray:
    """Minimum of the convex clearance function along the segments (golden-section, vectorised)."""
    lo = np.zeros(len(p0))
    hi = np.ones(len(p0))
    d = p1 - p0
    x1 = hi - GOLDEN * (hi - lo)
    x2 = lo + GOLDEN * (hi - lo)
    f1 = dist_fn(p0 + x1[:, None] * d)
    f2 = dist_fn(p0 + x2[:, None] * d)
    for _ in range(iters):
        move_hi = f1 < f2  # minimum is in [lo, x2]
        hi = np.where(move_hi, x2, hi)
        lo = np.where(move_hi, lo, x1)
        x2n = np.where(move_hi, x1, lo + GOLDEN * (hi - lo))
        x1n = np.where(move_hi, hi - GOLDEN * (hi - lo), x2)
        f2n = np.where(move_hi, f1, 0.0)
        f1n = np.where(move_hi, 0.0, f2)
        need1 = move_hi
        need2 = ~move_hi
        if need1.any():
            f1n[need1] = dist_fn(p0[need1] + x1n[need1][:, None] * d[need1])
        if need2.any():
            f2n[need2] = dist_fn(p0[need2] + x2n[need2][:, None] * d[need2])
        x1, x2, f1, f2 = x1n, x2n, f1n, f2n
    return np.minimum(np.minimum(f1, f2), np.minimum(dist_fn(p0), dist_fn(p1)))


# ---------------------------------------------------------------- report
@dataclass
class PairResult:
    a: str
    b: str
    kind: str  # self | obstacle
    min_clearance: float
    argmin: int
    n_collision: int
    n_margin: int


@dataclass
class CollisionReport:
    n_samples: int = 0
    margin: float = 0.0
    clearance: np.ndarray | None = None  # (N,) min over pairs, NaN when IK failed
    worst_pair: list[int] | None = None  # (N,) pair index of the minimum
    collision_mask: np.ndarray | None = None
    margin_mask: np.ndarray | None = None
    pairs: list[PairResult] = field(default_factory=list)
    n_collision: int = 0
    n_margin: int = 0
    min_clearance: float = float("inf")
    unresolved: int = 0
    refine_rounds: int = 0
    refined_samples: int = 0
    config_jumps: int = 0
    singular: int = 0
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "n_samples": self.n_samples, "margin": self.margin, "n_collision": self.n_collision,
            "n_margin": self.n_margin, "min_clearance": None if not math.isfinite(self.min_clearance) else self.min_clearance,
            "unresolved": self.unresolved, "refine_rounds": self.refine_rounds, "refined_samples": self.refined_samples,
            "config_jumps": self.config_jumps, "singular": self.singular, "notes": self.notes,
            "pairs": [vars(p) for p in self.pairs],
        }


def pair_clearances(robot: RobotModel, q: np.ndarray, caps: list[Capsule], obstacles: list[Obstacle]) -> tuple[np.ndarray, list[tuple[str, str, str]]]:
    """Clearance (N, P) for every checked pair (self pairs + capsule x obstacle)."""
    frames = batch_fk(robot, q)
    ends = capsule_endpoints(frames, caps)
    cols: list[np.ndarray] = []
    names: list[tuple[str, str, str]] = []
    disabled = {tuple(p) for p in getattr(robot, "disabled_pairs", [])}
    for ka, kb in self_pairs(caps, disabled):
        d = seg_seg_distance(ends[:, ka, 0], ends[:, ka, 1], ends[:, kb, 0], ends[:, kb, 1]) - caps[ka].radius - caps[kb].radius
        cols.append(d)
        names.append((caps[ka].name, caps[kb].name, "self"))
    for box in base_boxes(robot):
        for k, c in enumerate(caps):
            if c.link < 2 or (0, c.link) in disabled:
                continue
            d = seg_convex_distance(ends[:, k, 0], ends[:, k, 1], box.distance) - c.radius
            cols.append(d)
            names.append((c.name, box.name, "self"))
    for ob in obstacles:
        for k, c in enumerate(caps):
            if ob.workpiece and c.kind == "cutter":
                continue
            if c.link == 0 or (c.link == 1 and ob.kind == "floor"):
                continue  # the base is static (set-up error, not a path collision)
            d = seg_convex_distance(ends[:, k, 0], ends[:, k, 1], ob.distance) - c.radius
            cols.append(d)
            names.append((c.name, ob.name, "obstacle"))
    if not cols:
        return np.full((len(q), 0), np.inf), names
    return np.column_stack(cols), names


def motion_bound(robot: RobotModel, q0: np.ndarray, q1: np.ndarray, caps: list[Capsule]) -> np.ndarray:
    """Upper bound (N,) of how far any point of any capsule moves between poses q0[i] and q1[i]."""
    e0 = capsule_endpoints(batch_fk(robot, q0), caps)
    e1 = capsule_endpoints(batch_fk(robot, q1), caps)
    disp = np.linalg.norm(e1 - e0, axis=3).max(axis=(1, 2))
    # joint-space swing between the two samples is not a straight line: add a
    # conservative allowance proportional to the largest joint change
    dq = np.abs(((q1 - q0) + 180) % 360 - 180).max(axis=1) * DEG
    reach = robot.a1 + robot.wrist_reach + robot.d6 + 500.0
    return 2.0 * (disp + 0.5 * dq * dq * reach)


def check_collisions(robot: RobotModel, q: np.ndarray, ok: np.ndarray, caps: list[Capsule], obstacles: list[Obstacle],
                     margin: float, refine_fn=None, max_refine: int = 6, singular_deg: float = 5.0,
                     jump_deg: float = 30.0) -> CollisionReport:
    """Check all samples (q: (N,6), ok: valid IK mask). ``refine_fn(i)`` returns the
    joint vector for the Cartesian midpoint of interval i (or None) and enables
    the continuous check."""
    rep = CollisionReport(n_samples=len(q), margin=margin)
    n = len(q)
    clear = np.full(n, np.nan)
    worst = np.full(n, -1, dtype=int)
    qq = np.nan_to_num(q, nan=0.0)
    valid = ok.copy()
    names: list[tuple[str, str, str]] = []
    per_pair_min = None
    per_pair_arg = None
    per_pair_coll = None
    per_pair_marg = None
    if valid.any():
        d, names = pair_clearances(robot, qq[valid], caps, obstacles)
        if d.shape[1]:
            clear[valid] = d.min(axis=1)
            worst[valid] = d.argmin(axis=1)
            per_pair_min = d.min(axis=0)
            vidx = np.flatnonzero(valid)
            per_pair_arg = vidx[d.argmin(axis=0)]
            per_pair_coll = (d <= 0).sum(axis=0)
            per_pair_marg = ((d > 0) & (d < margin)).sum(axis=0)
    # --- continuous check between consecutive valid samples ---
    unresolved = 0
    refined = 0
    rounds = 0
    if refine_fn is not None and n > 1 and len(names):
        intervals = [(i, i + 1, qq[i], qq[i + 1], clear[i], clear[i + 1]) for i in range(n - 1) if valid[i] and valid[i + 1]]
        for rnd in range(max_refine):
            if not intervals:
                break
            q0 = np.array([it[2] for it in intervals])
            q1 = np.array([it[3] for it in intervals])
            c0 = np.array([it[4] for it in intervals])
            c1 = np.array([it[5] for it in intervals])
            delta = motion_bound(robot, q0, q1, caps)
            safe = (np.maximum(c0, c1) - delta >= margin) | (np.minimum(c0, c1) <= 0)
            todo = [it for it, s in zip(intervals, safe) if not s]
            if not todo:
                intervals = []
                break
            rounds = rnd + 1
            # IK for all midpoints of this round, then ONE vectorised clearance pass
            mids = []
            kept = []
            for it in todo:
                qm = refine_fn(it[0], it[1], it[2], it[3])
                if qm is not None:
                    mids.append(qm)
                    kept.append(it)
            new_intervals = []
            if mids:
                refined += len(mids)
                qm_arr = np.array(mids)
                dm, _ = pair_clearances(robot, qm_arr, caps, obstacles)
                cm_all = dm.min(axis=1)
                km_all = dm.argmin(axis=1)
                for (i0, i1, qa, qb, ca, cb), qm, cm, km, row in zip(kept, mids, cm_all, km_all, dm):
                    cm = float(cm)
                    # bookkeeping on the left sample: keep the worst value seen inside the interval
                    if np.isnan(clear[i0]) or cm < clear[i0]:
                        clear[i0] = cm
                        worst[i0] = int(km)
                    new_intervals.append((i0, i1, qa, qm, ca, cm))
                    new_intervals.append((i0, i1, qm, qb, cm, cb))
                if per_pair_min is not None:
                    pm = dm.min(axis=0)
                    better = pm < per_pair_min
                    arg_rows = dm.argmin(axis=0)
                    left_idx = np.array([it[0] for it in kept])
                    per_pair_min = np.where(better, pm, per_pair_min)
                    per_pair_arg = np.where(better, left_idx[arg_rows], per_pair_arg)
                    per_pair_coll = per_pair_coll + (dm <= 0).sum(axis=0)
                    per_pair_marg = per_pair_marg + ((dm > 0) & (dm < margin)).sum(axis=0)
            intervals = new_intervals
        if intervals:
            q0 = np.array([it[2] for it in intervals])
            q1 = np.array([it[3] for it in intervals])
            c0 = np.array([it[4] for it in intervals])
            c1 = np.array([it[5] for it in intervals])
            delta = motion_bound(robot, q0, q1, caps)
            safe = (np.maximum(c0, c1) - delta >= margin) | (np.minimum(c0, c1) <= 0)
            unresolved = int((~safe).sum())
    rep.unresolved = unresolved
    rep.refine_rounds = rounds
    rep.refined_samples = refined
    rep.clearance = clear
    rep.worst_pair = worst.tolist()
    rep.collision_mask = np.nan_to_num(clear, nan=np.inf) <= 0
    rep.margin_mask = (np.nan_to_num(clear, nan=np.inf) > 0) & (np.nan_to_num(clear, nan=np.inf) < margin)
    rep.n_collision = int(rep.collision_mask.sum())
    rep.n_margin = int(rep.margin_mask.sum())
    finite = clear[np.isfinite(clear)]
    rep.min_clearance = float(finite.min()) if finite.size else float("inf")
    if per_pair_min is not None:
        for k, (a, b, kind) in enumerate(names):
            rep.pairs.append(PairResult(a, b, kind, float(per_pair_min[k]), int(per_pair_arg[k]), int(per_pair_coll[k]), int(per_pair_marg[k])))
        rep.pairs.sort(key=lambda p: p.min_clearance)
    # --- configuration jumps and wrist singularity ---
    if n > 1:
        vq = np.flatnonzero(valid)
        if len(vq) > 1:
            dq = np.abs(((qq[vq[1:]] - qq[vq[:-1]]) + 180) % 360 - 180)
            rep.config_jumps = int((dq[:, :3].max(axis=1) > jump_deg).sum())
        rep.singular = int((np.abs(qq[valid][:, 4]) < singular_deg).sum()) if valid.any() else 0
    if rep.config_jumps:
        rep.notes.append(f"{rep.config_jumps} configuration jumps (>{jump_deg:g} deg on A1-A3 between neighbouring samples)")
    if rep.singular:
        rep.notes.append(f"{rep.singular} samples near the wrist singularity (|A5| < {singular_deg:g} deg)")
    if unresolved:
        rep.notes.append(f"{unresolved} intervals could not be proven clear after {max_refine} refinement rounds")
    return rep
