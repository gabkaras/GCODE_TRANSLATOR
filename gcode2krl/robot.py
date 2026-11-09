"""Simplified kinematic models of KUKA 6-axis robots (forward + inverse kinematics).

Conventions (robot base frame: origin on the base plate, X forward, Z up):
  A1  rotates about -Z            (KUKA: positive A1 turns clockwise seen from above)
  A2  rotates about +Y            (A2 = 0: upper arm horizontal forward, A2 = -90: vertical)
  A3  rotates about +Y            (A3 = 0: forearm continues the upper arm, A3 = +90 at HOME)
  A4  rotates about the forearm axis (-X local)
  A5  rotates about +Y local
  A6  rotates about the flange axis (-X local)
  At A4 = A5 = A6 = 0 the flange Z axis points along the forearm and the
  flange X axis points "down", so HOME (0,-90,90,0,0,0) reads B = 90.
KUKA orientation A, B, C is ZYX Euler: R = Rz(A) * Ry(B) * Rx(C).

Link parameters: d1 base height to A2, a1 A1-A2 horizontal offset, a2 upper
arm, d4 forearm (A3 to A5), a3 wrist offset (A4 axis below A3 axis), d6 A5 to
flange. Values are taken from KUKA data sheets and are *approximate* - good
enough for reach/limit checks and visualisation, not for calibration.
"""
from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

DEG = math.pi / 180.0


@dataclass
class RobotModel:
    """KUKA 6R robot. ``origins`` are the joint origins (mm) of the standard chain
    exactly as in the ROS-Industrial URDFs: a1 (0,0,d1), a2 (a1,0,0), a3 (a2,0,0),
    a4 (d4a,0,a3), a5 (d4b,0,0), a6 (d6a,0,0), flange (d6b,0,0)."""

    key: str
    name: str
    origins: list[list[float]]
    limits: list[tuple[float, float]]  # deg, per axis
    speeds: list[float]  # deg/s, per axis (nominal)
    payload: float = 0.0
    reach: float = 0.0
    mass: float = 0.0
    home: tuple[float, ...] = (0.0, -90.0, 90.0, 0.0, 0.0, 0.0)
    sources: dict = field(default_factory=dict)
    meshes: dict = field(default_factory=dict)  # link index (str) -> relative STL path
    capsules: list = field(default_factory=list)  # mesh-derived capsules {link, a, b, radius, name}
    notes: list = field(default_factory=list)
    visual: dict = field(default_factory=dict)  # procedural fallback sizes
    base_dir: str = ""
    disabled_pairs: list = field(default_factory=list)  # link index pairs always touching (skipped in self collision)

    # ---- derived link parameters used by the closed-form IK
    @property
    def d1(self) -> float:
        return self.origins[0][2]

    @property
    def a1(self) -> float:
        return self.origins[1][0]

    @property
    def a2(self) -> float:
        return self.origins[2][0]

    @property
    def a3(self) -> float:
        """Signed offset of the forearm axis from the A3 axis (local Z; negative = below)."""
        return self.origins[3][2]

    @property
    def d4(self) -> float:
        return self.origins[3][0] + self.origins[4][0]

    @property
    def d6(self) -> float:
        return self.origins[5][0] + self.origins[6][0]

    @property
    def wrist_reach(self) -> float:
        return self.a2 + math.hypot(self.d4, self.a3)

    def to_dict(self) -> dict:
        return {
            "key": self.key, "name": self.name, "origins": self.origins, "d1": self.d1, "a1": self.a1, "a2": self.a2,
            "a3": self.a3, "d4": self.d4, "d6": self.d6, "limits": self.limits, "speeds": self.speeds,
            "payload": self.payload, "reach": self.reach, "mass": self.mass, "home": list(self.home),
            "visual": self.visual, "wrist_reach": self.wrist_reach, "sources": self.sources, "notes": self.notes,
            "has_mesh": bool(self.meshes), "capsules": self.capsules, "disabled_pairs": self.disabled_pairs,
        }


def _visual_from_origins(o) -> dict:
    scale = max(0.2, (o[2][0] + o[3][0] + o[4][0]) / 2350.0)
    return {"base_r": 330 * scale, "base_h": 280 * scale, "column": 420 * scale, "arm": 260 * scale,
            "forearm": 200 * scale, "wrist": 160 * scale}


def model_from_dict(d: dict, base_dir: str = "") -> RobotModel:
    m = RobotModel(
        key=d["key"], name=d.get("name", d["key"]), origins=[list(map(float, o)) for o in d["origins"]],
        limits=[tuple(map(float, l)) for l in d["limits"]], speeds=[float(v) for v in d["speeds"]],
        payload=float(d.get("payload", 0) or 0), reach=float(d.get("reach", 0) or 0), mass=float(d.get("mass", 0) or 0),
        home=tuple(float(v) for v in d.get("home", (0, -90, 90, 0, 0, 0))), sources=d.get("sources", {}),
        meshes=d.get("meshes", {}), capsules=d.get("capsules", []), notes=d.get("notes", []),
        visual=d.get("visual") or _visual_from_origins(d["origins"]), base_dir=base_dir,
        disabled_pairs=[tuple(p) for p in d.get("disabled_pairs", [])],
    )
    return m


ROBOTS: dict[str, RobotModel] = {}
ROBOT_DIRS = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "robots"),
              os.path.join(os.path.expanduser("~"), ".gcode2krl", "robots")]


def load_robots(dirs: list[str] | None = None) -> dict[str, RobotModel]:
    """(Re)load robot definitions from the package data dir and ~/.gcode2krl/robots."""
    ROBOTS.clear()
    for d in dirs or ROBOT_DIRS:
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.lower().endswith(".json"):
                try:
                    with open(os.path.join(d, f), encoding="utf-8") as fh:
                        data = json.load(fh)
                    m = model_from_dict(data, d)
                    ROBOTS[m.key] = m
                except (ValueError, KeyError, OSError) as exc:
                    print(f"robot definition {f} skipped: {exc}", file=sys.stderr)
    return ROBOTS


DEFAULT_ROBOT = "KR90R2700PRO"
load_robots()
if DEFAULT_ROBOT not in ROBOTS and ROBOTS:
    DEFAULT_ROBOT = next(iter(ROBOTS))


# --------------------------------------------------------------- transforms
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def abc_to_rot(a_deg: float, b_deg: float, c_deg: float) -> np.ndarray:
    """KUKA Euler ZYX: R = Rz(A) Ry(B) Rx(C)."""
    return rot_z(a_deg * DEG) @ rot_y(b_deg * DEG) @ rot_x(c_deg * DEG)


def rot_to_abc(r: np.ndarray) -> tuple[float, float, float]:
    b = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(b)) < 1e-9:
        a = 0.0
        c = math.atan2(r[0, 1], r[1, 1]) if b > 0 else -math.atan2(r[0, 1], r[1, 1])
    else:
        a = math.atan2(r[1, 0], r[0, 0])
        c = math.atan2(r[2, 1], r[2, 2])
    return a / DEG, b / DEG, c / DEG


def abc_from_axis(axis, spin_deg: float = 0.0, ref: tuple[float, float, float] = (0.0, 180.0, 0.0)) -> tuple[float, float, float]:
    """Tool orientation (A B C) whose tool Z axis points *into* the part along -axis,
    i.e. ``axis`` is the direction away from the part (printing: +Z of the layer normal).
    ``spin_deg`` rotates about the tool axis (the free DOF of a 5-axis task); ``ref``
    is the orientation used when axis == (0, 0, 1) and spin == 0."""
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return ref
    a = a / n
    r_ref = abc_to_rot(*ref)
    z_ref = r_ref[:, 2]  # tool Z of the reference orientation (points down for 0 180 0)
    target_z = -a
    # minimal rotation taking z_ref to target_z
    c = float(np.clip(z_ref @ target_z, -1.0, 1.0))
    if c > 1 - 1e-12:
        r_align = np.eye(3)
    elif c < -1 + 1e-12:
        # 180 deg about any axis perpendicular to z_ref
        perp = np.cross(z_ref, [1.0, 0, 0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(z_ref, [0, 1.0, 0])
        k = perp / np.linalg.norm(perp)
        r_align = 2 * np.outer(k, k) - np.eye(3)
    else:
        k = np.cross(z_ref, target_z)
        sn = float(np.linalg.norm(k))
        k = k / sn
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        r_align = np.eye(3) + sn * kx + (1 - c) * kx @ kx
    if c > 1 - 1e-12 and not spin_deg:
        return tuple(float(v) for v in ref)
    r = r_align @ r_ref
    if spin_deg:
        sp = spin_deg * DEG
        cs, ss = math.cos(sp), math.sin(sp)
        kz = target_z
        kx = np.array([[0, -kz[2], kz[1]], [kz[2], 0, -kz[0]], [-kz[1], kz[0], 0]])
        r_spin = np.eye(3) + ss * kx + (1 - cs) * kx @ kx
        r = r_spin @ r
    return rot_to_abc(r)


def frame(xyzabc) -> np.ndarray:
    """4x4 homogeneous transform from KUKA FRAME {X Y Z A B C}."""
    x, y, z, a, b, c = [float(v) for v in xyzabc]
    t = np.eye(4)
    t[:3, :3] = abc_to_rot(a, b, c)
    t[:3, 3] = (x, y, z)
    return t


def inv_frame(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    r = t[:3, :3].T
    out[:3, :3] = r
    out[:3, 3] = -r @ t[:3, 3]
    return out


FLANGE0 = rot_y(math.pi / 2)  # flange orientation in the forearm frame at A4=A5=A6=0


def frame_from_3_points(origin, x_point, xy_point) -> tuple[float, float, float, float, float, float]:
    """KUKA 3-point base measurement: origin, a point on the +X axis and a point in
    the XY plane with positive Y. Returns X Y Z A B C of the base in the frame the
    points were given in (usually the robot $ROBROOT frame)."""
    o = np.asarray(origin, dtype=float)
    px = np.asarray(x_point, dtype=float)
    pxy = np.asarray(xy_point, dtype=float)
    x = px - o
    if np.linalg.norm(x) < 1e-9:
        raise ValueError("X point coincides with the origin")
    x = x / np.linalg.norm(x)
    z = np.cross(x, pxy - o)
    if np.linalg.norm(z) < 1e-9:
        raise ValueError("XY point lies on the X axis")
    z = z / np.linalg.norm(z)
    y = np.cross(z, x)
    r = np.column_stack([x, y, z])
    a, b, c = rot_to_abc(r)
    return float(o[0]), float(o[1]), float(o[2]), a, b, c


# ------------------------------------------------------------------ forward
def forward(robot: RobotModel, joints_deg) -> list[np.ndarray]:
    """Return 4x4 frames [base, link_1 .. link_6, flange] (URDF link frames; flange Z out)."""
    q = [float(v) * DEG for v in joints_deg]
    o = robot.origins
    t = np.eye(4)
    frames = [t.copy()]
    t = t @ _h(rot_z(-q[0]), o[0])
    frames.append(t.copy())  # link_1
    t = t @ _h(rot_y(q[1]), o[1])
    frames.append(t.copy())  # link_2
    t = t @ _h(rot_y(q[2]), o[2])
    frames.append(t.copy())  # link_3
    t = t @ _h(rot_x(-q[3]), o[3])
    frames.append(t.copy())  # link_4
    t = t @ _h(rot_y(q[4]), o[4])
    frames.append(t.copy())  # link_5
    t = t @ _h(rot_x(-q[5]), o[5])
    frames.append(t.copy())  # link_6
    t = t @ _h(FLANGE0, o[6])
    frames.append(t.copy())  # flange
    return frames


def _h(r: np.ndarray, p) -> np.ndarray:
    """Translate by p (parent frame) then rotate by r: the URDF joint convention."""
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = p
    return t


# ------------------------------------------------------------------ inverse
def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _within(v: float, lim: tuple[float, float]) -> float | None:
    """Return an equivalent angle (v + k*360) inside the limits, or None."""
    for k in (0, 1, -1, 2, -2):
        w = v + 360.0 * k
        if lim[0] - 1e-9 <= w <= lim[1] + 1e-9:
            return w
    return None


def inverse_all(robot: RobotModel, flange: np.ndarray) -> list[list[float]]:
    """All closed-form IK solutions (deg, not limit-checked, wrapped to +-180)."""
    r = flange[:3, :3]
    p = flange[:3, 3]
    wc = p - robot.d6 * r[:, 2]  # wrist centre (A5)
    sols: list[list[float]] = []
    base_angle = math.atan2(wc[1], wc[0])
    l2 = math.hypot(robot.d4, robot.a3)
    phi = math.atan2(-robot.a3, robot.d4)  # angle from the forearm axis to the A3->A5 line (a3 < 0: below)
    for shoulder in (0, 1):
        th1 = -(base_angle + shoulder * math.pi)  # A1 about -Z
        # position of the wrist centre in the arm plane (after undoing A1)
        rad = math.hypot(wc[0], wc[1]) * (1 if shoulder == 0 else -1) - robot.a1
        z = wc[2] - robot.d1
        dist2 = rad * rad + z * z
        dist = math.sqrt(dist2)
        if dist > robot.a2 + l2 + 1e-9 or dist < abs(robot.a2 - l2) - 1e-9:
            continue
        cos_g = (robot.a2 * robot.a2 + l2 * l2 - dist2) / (2 * robot.a2 * l2)
        cos_g = max(-1.0, min(1.0, cos_g))
        gamma = math.acos(cos_g)  # elbow interior angle
        cos_k = (robot.a2 * robot.a2 + dist2 - l2 * l2) / (2 * robot.a2 * dist)
        cos_k = max(-1.0, min(1.0, cos_k))
        kappa = math.acos(cos_k)  # angle between upper arm and A2->W
        base_dir = math.atan2(z, rad)  # angle of A2->W, ccw from +x towards +z
        for elbow in (1, -1):
            alpha = base_dir + elbow * kappa  # upper arm angle
            psi_virtual = alpha - elbow * (math.pi - gamma)  # virtual forearm angle
            psi_axis = psi_virtual + phi
            th2 = -alpha
            th3 = -psi_axis - th2
            r03 = rot_z(-th1) @ rot_y(th2 + th3)
            m = r03.T @ r @ FLANGE0.T  # = Rx(-th4) Ry(th5) Rx(-th6)
            sb = math.hypot(m[0, 1], m[0, 2])
            for flip in (0, 1):
                if sb < 1e-9:
                    a = 0.0
                    b = 0.0 if m[0, 0] > 0 else math.pi
                    c = math.atan2(m[2, 1], m[1, 1])
                    if flip:
                        continue
                else:
                    b = math.atan2(sb, m[0, 0])
                    a = math.atan2(m[1, 0], -m[2, 0])
                    c = math.atan2(m[0, 1], m[0, 2])
                    if flip:
                        b, a, c = -b, a + math.pi, c + math.pi
                sols.append([_wrap(v / DEG) for v in (th1, th2, th3, -a, b, -c)])
    return sols


def inverse(robot: RobotModel, flange: np.ndarray, prev: list[float] | None = None) -> tuple[list[float] | None, list[int]]:
    """Best IK solution within axis limits, closest to ``prev`` (deg).

    Returns (joints or None if unreachable, list of axes that are out of limits
    for the *closest* solution when none is fully inside the limits).
    """
    prev = list(prev) if prev is not None else list(robot.home)
    sols = inverse_all(robot, flange)
    if not sols:
        return None, []
    best = None
    best_cost = float("inf")
    best_viol: list[int] = []
    best_any = None
    best_any_cost = float("inf")
    best_any_viol: list[int] = []
    for s in sols:
        adj = []
        viol = []
        for k, v in enumerate(s):
            lim = robot.limits[k]
            w = _within(v, lim)
            if w is None:
                viol.append(k)
                adj.append(v)
            else:
                # choose the equivalent angle nearest to prev
                cands = [v + 360.0 * j for j in (-2, -1, 0, 1, 2) if lim[0] - 1e-9 <= v + 360.0 * j <= lim[1] + 1e-9]
                adj.append(min(cands, key=lambda c: abs(c - prev[k])))
        cost = sum(abs(adj[k] - prev[k]) * (1.0 if k < 3 else 0.5) for k in range(6))
        if not viol and cost < best_cost:
            best, best_cost, best_viol = adj, cost, []
        if cost < best_any_cost:
            best_any, best_any_cost, best_any_viol = adj, cost, viol
    if best is not None:
        return best, []
    return best_any, best_any_viol


# --------------------------------------------------------------- path check
@dataclass
class ReachReport:
    n_samples: int = 0
    n_unreachable: int = 0
    n_limit: int = 0
    axis_violations: list[int] = field(default_factory=lambda: [0] * 6)
    joint_min: list[float] = field(default_factory=lambda: [float("inf")] * 6)
    joint_max: list[float] = field(default_factory=lambda: [-float("inf")] * 6)
    joints: np.ndarray | None = None  # (n, 6) deg, NaN where unreachable
    ok_mask: np.ndarray | None = None

    def to_dict(self) -> dict:
        return {
            "n_samples": self.n_samples, "n_unreachable": self.n_unreachable, "n_limit": self.n_limit,
            "axis_violations": self.axis_violations,
            "joint_min": [None if v == float("inf") else v for v in self.joint_min],
            "joint_max": [None if v == -float("inf") else v for v in self.joint_max],
        }


def check_path(robot: RobotModel, points: np.ndarray, abc, base: np.ndarray,
               tool: np.ndarray) -> ReachReport:
    """IK along ``points`` (in BASE coordinates). ``abc`` is one orientation
    (A, B, C) for the whole path or an (N, 3) array with one per point.

    base: 4x4 transform of $BASE in the robot base frame; tool: 4x4 $TOOL in
    the flange frame.
    """
    rep = ReachReport(n_samples=len(points))
    abc_arr = np.asarray(abc, dtype=float)
    per_point = abc_arr.ndim == 2
    tool_inv = inv_frame(tool)
    joints = np.full((len(points), 6), np.nan)
    ok = np.zeros(len(points), dtype=bool)
    prev = list(robot.home)
    t_path = np.eye(4)
    if not per_point:
        t_path[:3, :3] = abc_to_rot(*abc_arr)
    last_abc = None
    for i, p in enumerate(points):
        if per_point:
            a = tuple(abc_arr[i])
            if a != last_abc:
                t_path[:3, :3] = abc_to_rot(*a)
                last_abc = a
        t_path[:3, 3] = p
        flange = base @ t_path @ tool_inv
        q, viol = inverse(robot, flange, prev)
        if q is None:
            rep.n_unreachable += 1
            continue
        joints[i] = q
        if viol:
            rep.n_limit += 1
            for k in viol:
                rep.axis_violations[k] += 1
        else:
            ok[i] = True
            prev = q
        for k in range(6):
            rep.joint_min[k] = min(rep.joint_min[k], q[k])
            rep.joint_max[k] = max(rep.joint_max[k], q[k])
    rep.joints = joints
    rep.ok_mask = ok
    return rep


def _slerp_rot(ra: np.ndarray, rb: np.ndarray, t: float) -> np.ndarray:
    """Interpolate rotation matrices (axis-angle of the relative rotation)."""
    rel = ra.T @ rb
    ang = math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1.0) / 2.0)))
    if ang < 1e-9:
        return ra
    k = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]) / (2.0 * math.sin(ang))
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    th = ang * t
    r_t = np.eye(3) + math.sin(th) * kx + (1 - math.cos(th)) * kx @ kx
    return ra @ r_t


def spin_for_tangent(axis, tangent, ref=(0.0, 180.0, 0.0), offset_deg: float = 0.0) -> float:
    """Spin (deg) about the tool axis that points the tool X axis along the path
    tangent (projected onto the plane perpendicular to the axis), plus ``offset_deg``.
    Used for asymmetric nozzles, wire feeders, cable-fed or bent extruders that must
    trail / lead the motion."""
    a = np.asarray(axis, float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return offset_deg
    a = a / n
    t = np.asarray(tangent, float)
    t = t - (t @ a) * a
    if np.linalg.norm(t) < 1e-9:
        return offset_deg
    t /= np.linalg.norm(t)
    x0 = abc_to_rot(*abc_from_axis(a, 0.0, ref))[:, 0]
    x0 = x0 - (x0 @ a) * a
    if np.linalg.norm(x0) < 1e-9:
        return offset_deg
    x0 /= np.linalg.norm(x0)
    kz = -a  # the spin rotates about the tool Z axis (into the part)
    ang = math.degrees(math.atan2(float(np.cross(x0, t) @ kz), float(x0 @ t)))
    return (ang + offset_deg) % 360.0


def path_tangents(points: np.ndarray) -> np.ndarray:
    """Unit tangents along a polyline (central differences, one-sided at the ends)."""
    P = np.asarray(points, float)
    n = len(P)
    t = np.zeros((n, 3))
    if n == 1:
        t[0] = [1.0, 0, 0]
        return t
    t[1:-1] = P[2:] - P[:-2]
    t[0] = P[1] - P[0]
    t[-1] = P[-1] - P[-2]
    ln = np.linalg.norm(t, axis=1)
    bad = ln < 1e-9
    t[bad] = [1.0, 0, 0]
    ln[bad] = 1.0
    return t / ln[:, None]


def solve_spins(robot: RobotModel, points: np.ndarray, axes: np.ndarray, base: np.ndarray, tool: np.ndarray,
                ref=(0.0, 180.0, 0.0), spin_step: float = 15.0, singular_deg: float = 8.0,
                max_tilt_deg: float = 0.0, start=None, spin_mode: str = "free", spin_fixed: float = 0.0,
                spin_offset: float = 0.0) -> tuple[np.ndarray, dict]:
    """Choose the free rotation about the tool axis (and optionally a small tilt) per
    point so that the robot stays reachable, far from the wrist singularity (|A5| >
    ``singular_deg``) and moves its joints as little as possible between points.

    A lightweight version of singularity-aware orientation planning (Zhang et al.,
    IEEE RA-L 2021): per point the previous spin is kept while it is feasible and
    non-singular; otherwise spins (and tilts within ``max_tilt_deg``) are sampled
    and the cheapest feasible one is taken. Returns (abc (N,3), info).

    ``spin_mode``: "free" (solver picks, favouring small changes), "fixed" (keep
    ``spin_fixed`` degrees, the solver only deviates when the robot cannot reach or
    is singular) or "tangent" (tool X along the path direction plus ``spin_offset``,
    for asymmetric / bent / wire-fed tools; same fallback).
    """
    from itertools import product

    tangents = path_tangents(points) if spin_mode == "tangent" else None

    def desired(i: int) -> float | None:
        if spin_mode == "fixed":
            return float(spin_fixed) % 360.0
        if spin_mode == "tangent":
            return spin_for_tangent(axes[i], tangents[i], ref, spin_offset)
        return None

    n = len(points)
    abc_out = np.zeros((n, 3))
    tool_inv = inv_frame(tool)
    prev_q = list(start) if start is not None else list(robot.home)
    spin = 0.0
    tilt_a, tilt_b = 0.0, 0.0
    info = {"resampled": 0, "unreachable": 0, "singular": 0}
    spins = np.arange(0.0, 360.0, spin_step)
    tilts = [(0.0, 0.0)]
    if max_tilt_deg > 0:
        for t in (max_tilt_deg / 2.0, max_tilt_deg):
            for ang in range(0, 360, 60):
                tilts.append((t * math.cos(math.radians(ang)), t * math.sin(math.radians(ang))))

    def tilted_axis(ax, ta, tb):
        if ta == 0 and tb == 0:
            return ax
        ax = np.asarray(ax, float)
        u = np.cross(ax, [0, 0, 1.0])
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(ax, [1.0, 0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(ax, u)
        out = ax + math.tan(ta * DEG) * u + math.tan(tb * DEG) * v
        return out / np.linalg.norm(out)

    def evaluate(i, sp, ta, tb):
        a = abc_from_axis(tilted_axis(axes[i], ta, tb), sp, ref)
        t = frame((points[i][0], points[i][1], points[i][2], *a))
        q, viol = inverse(robot, base @ t @ tool_inv, prev_q)
        if q is None or viol:
            return None, a
        cost = sum(abs(q[k] - prev_q[k]) * (1.0 if k < 3 else 0.5) for k in range(6))
        want = desired(i)
        dsp = abs((sp - (spin if want is None else want) + 180.0) % 360.0 - 180.0)
        cost += (0.3 if want is None else 3.0) * dsp + 2.0 * (abs(ta) + abs(tb))  # prefer the wanted spin and no extra tilt
        if abs(q[4]) < singular_deg:
            cost += 1000.0 + (singular_deg - abs(q[4])) * 50.0
        return (cost, q), a

    for i in range(n):
        want = desired(i)
        if want is not None:
            spin = want
        res, a = evaluate(i, spin, tilt_a, tilt_b)
        if res is not None and res[0] < 1000.0:
            prev_q = res[1]
            abc_out[i] = a
            continue
        best = None
        info["resampled"] += 1
        for (ta, tb), sp in product(tilts, spins):
            r2, a2 = evaluate(i, sp, ta, tb)
            if r2 is None:
                continue
            if best is None or r2[0] < best[0][0]:
                best = (r2, a2, sp, ta, tb)
        if best is None:
            # unreachable at every sampled orientation: keep the nominal one and mark it
            abc_out[i] = a
            info["unreachable"] += 1
            continue
        (cost, q), a2, spin, tilt_a, tilt_b = best
        if cost >= 1000.0:
            info["singular"] += 1
        prev_q = q
        abc_out[i] = a2
    return abc_out, info


# ------------------------------------------------------------ Status / Turn
def critical_a3(robot: RobotModel) -> float:
    """A3 angle (deg) at which the wrist root lies on the line through A2 and A3
    (stretched arm): the boundary of status bit 1 (KUKA: bit 1 = 1 for A3 >= angle)."""
    o = np.asarray(robot.origins[3], dtype=float)  # A3 -> wrist root in the link_3 frame
    return math.degrees(math.atan2(o[2], o[0]))


def status_turn(robot: RobotModel, q) -> tuple[int, int]:
    """KUKA Status and Turn of a joint position (deg).

    Status: bit 0 = wrist root behind the A1 axis (overhead area), bit 1 = A3 >= the
    stretched-arm angle (basic arm configuration), bit 2 = A5 < 0.
    Turn: bit i = 1 when axis A(i+1) is negative."""
    qf = [float(v) for v in q]
    fr = forward(robot, qf)
    wc = fr[4][:3, 3]  # wrist root (A4/A5/A6 intersection)
    r1 = fr[1]
    local = r1[:3, :3].T @ (wc - r1[:3, 3])
    s = 0
    if local[0] < 0:
        s |= 1
    if qf[2] >= critical_a3(robot):
        s |= 2
    if qf[4] < 0:
        s |= 4
    t = 0
    for i in range(6):
        if qf[i] < 0:
            t |= 1 << i
    return s, t


def status_turn_batch(robot: RobotModel, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Vectorised status_turn for (N,6) joint arrays (NaN rows -> -1)."""
    from .collision import batch_fk

    qq = np.asarray(q, dtype=float)
    ok = ~np.isnan(qq).any(axis=1)
    frames = batch_fk(robot, np.nan_to_num(qq, nan=0.0))
    r1, p1 = frames[1]
    wc = frames[4][1]
    local_x = np.einsum("ni,ni->n", r1[:, :, 0], wc - p1)
    st = np.where(local_x < 0, 1, 0) | np.where(qq[:, 2] >= critical_a3(robot), 2, 0) | np.where(qq[:, 4] < 0, 4, 0)
    turn = np.zeros(len(qq), dtype=int)
    for i in range(6):
        turn |= np.where(qq[:, i] < 0, 1 << i, 0)
    st = np.where(ok, st, -1)
    turn = np.where(ok, turn, -1)
    return st, turn


def assign_configurations(items: list, robot: RobotModel, base: np.ndarray, tool: np.ndarray, ref=(0.0, 180.0, 0.0),
                          start=None) -> dict:
    """Give every travel a Status / Turn from the inverse kinematics and mark the travels
    that have to be PTP (first approach, or a configuration change): CP motions keep the
    configuration they start with, so a status change is only possible on a PTP."""
    from .translator import ArcItem, LineItem, RapidItem, SplItem

    tool_inv = inv_frame(tool)
    prev_q = list(start) if start is not None else list(robot.home)
    last_st = None
    info = {"ptp_travels": 0, "status_changes_in_path": 0, "unreachable_travels": 0, "start": None}

    def ik(p, abc):
        nonlocal prev_q
        t = frame((float(p[0]), float(p[1]), float(p[2]), *(abc if abc is not None else ref)))
        q, viol = inverse(robot, base @ t @ tool_inv, prev_q)
        if q is None:
            return None
        prev_q = list(q)
        return q

    # a path that starts without a travel still needs a PTP approach
    first_motion = next((i for i, it in enumerate(items) if isinstance(it, (RapidItem, LineItem, ArcItem, SplItem))), None)
    if first_motion is not None and not isinstance(items[first_motion], RapidItem):
        it = items[first_motion]
        p0 = it.start if not isinstance(it, SplItem) else it.points[0]
        abc0 = (it.abcs[0] if isinstance(it, SplItem) and it.abcs is not None else getattr(it, "abc", None))
        p0 = np.asarray(p0, dtype=float)
        items.insert(first_motion, RapidItem(p0, p0.copy(), abc=None if abc0 is None else tuple(abc0)))
    for it in items:
        if isinstance(it, RapidItem):
            q = ik(it.target, it.abc)
            if q is None:
                info["unreachable_travels"] += 1
                it.ptp = last_st is None
                continue
            st = status_turn(robot, q)
            it.st = st
            # a Turn change is fine inside CP motions; only a Status change needs a PTP
            it.ptp = last_st is None or st[0] != last_st[0]
            if it.ptp:
                info["ptp_travels"] += 1
            if info["start"] is None:
                info["start"] = st
            last_st = st
        elif isinstance(it, (LineItem, ArcItem)):
            q = ik(it.end, it.abc)
            if q is not None and last_st is not None:
                st = status_turn(robot, q)
                if st[0] != last_st[0]:
                    info["status_changes_in_path"] += 1
                last_st = st
        elif isinstance(it, SplItem):
            n = len(it.points)
            step = max(1, n // 20)
            for k in list(range(step, n, step)) + [n - 1]:
                abc = None if it.abcs is None else tuple(it.abcs[k])
                q = ik(it.points[k], abc)
                if q is not None and last_st is not None:
                    st = status_turn(robot, q)
                    if st[0] != last_st[0]:
                        info["status_changes_in_path"] += 1
                    last_st = st
    return info


def check_joint_move(robot: RobotModel, q_from, q_to, caps: list, obstacles: list, margin: float = 10.0,
                     n: int = 16, max_refine: int = 6):
    """Collision report of a PTP (joint-interpolated) move, e.g. HOME -> first path
    point. The continuous check is exact here: the midpoint of a joint interval is
    the mean of its ends."""
    from .collision import check_collisions

    qa = np.asarray(q_from, dtype=float)
    qb = np.asarray(q_to, dtype=float)
    t = np.linspace(0.0, 1.0, max(2, n))[:, None]
    q = qa[None, :] * (1.0 - t) + qb[None, :] * t
    ok = np.ones(len(q), dtype=bool)
    return check_collisions(robot, q, ok, caps, obstacles, margin,
                            refine_fn=lambda i0, i1, a, b: (np.asarray(a) + np.asarray(b)) / 2.0, max_refine=max_refine)


def check_path_collisions(robot: RobotModel, points: np.ndarray, abc, base: np.ndarray, tool: np.ndarray,
                          reach: "ReachReport", obstacles: list, margin: float = 10.0, tool_radius: float = 25.0,
                          cutter_len: float = 20.0, cutter_radius: float = 6.0, max_refine: int = 6,
                          tool_mesh: np.ndarray | None = None, tool_mesh_key=None):
    """Collision report along ``points`` using the IK results of ``reach`` (check_path)."""
    from .collision import check_collisions, robot_capsules

    caps = robot_capsules(robot, tool, tool_radius, cutter_len, cutter_radius, tool_mesh=tool_mesh, tool_mesh_key=tool_mesh_key)
    abc_arr = np.asarray(abc, dtype=float)
    per_point = abc_arr.ndim == 2
    tool_inv = inv_frame(tool)
    t_path = np.eye(4)
    t_path[:3, :3] = abc_to_rot(*(abc_arr[0] if per_point else abc_arr))

    def refine(i0: int, i1: int, qa, qb):
        mid = (points[i0] + points[i1]) / 2.0
        t_path[:3, 3] = mid
        q, viol = inverse(robot, base @ t_path @ tool_inv, list(qa))
        if q is None:
            return None
        return np.array(q, dtype=float)

    # refinement positions are halved each round: keep a per-interval midpoint
    # by closing over the interval end points
    def refine_fn(i0, i1, qa, qb):
        # the Cartesian midpoint between the *poses* qa/qb: both lie on the
        # straight line between points[i0] and points[i1]; recover their
        # parameter from the TCP positions
        pa = forward(robot, qa)[-1] @ tool
        pb = forward(robot, qb)[-1] @ tool
        mid_robot = (pa[:3, 3] + pb[:3, 3]) / 2.0
        flange = np.eye(4)
        if per_point:
            # orientation at the midpoint: half-way rotation between the two end orientations
            ra, rb = pa[:3, :3], pb[:3, :3]
            flange[:3, :3] = _slerp_rot(ra, rb, 0.5)
        else:
            flange[:3, :3] = (base @ t_path)[:3, :3]
        flange[:3, 3] = mid_robot
        q, viol = inverse(robot, flange @ tool_inv, list(qa))
        return None if q is None else np.array(q, dtype=float)

    q = reach.joints if reach.joints is not None else np.full((len(points), 6), np.nan)
    ok = reach.ok_mask if reach.ok_mask is not None else np.zeros(len(points), dtype=bool)
    return check_collisions(robot, q, ok, caps, obstacles, margin, refine_fn=refine_fn, max_refine=max_refine), caps
