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


def check_path(robot: RobotModel, points: np.ndarray, abc: tuple[float, float, float], base: np.ndarray,
               tool: np.ndarray) -> ReachReport:
    """IK along ``points`` (in BASE coordinates) with a constant tool orientation.

    base: 4x4 transform of $BASE in the robot base frame; tool: 4x4 $TOOL in
    the flange frame.
    """
    rep = ReachReport(n_samples=len(points))
    r_path = abc_to_rot(*abc)
    tool_inv = inv_frame(tool)
    joints = np.full((len(points), 6), np.nan)
    ok = np.zeros(len(points), dtype=bool)
    prev = list(robot.home)
    # tool orientation in robot frame is constant: R_flange = R_base * R_abc * R_tool^-1
    t_path = np.eye(4)
    t_path[:3, :3] = r_path
    for i, p in enumerate(points):
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


def check_path_collisions(robot: RobotModel, points: np.ndarray, abc, base: np.ndarray, tool: np.ndarray,
                          reach: "ReachReport", obstacles: list, margin: float = 10.0, tool_radius: float = 25.0,
                          cutter_len: float = 20.0, cutter_radius: float = 6.0, max_refine: int = 6):
    """Collision report along ``points`` using the IK results of ``reach`` (check_path)."""
    from .collision import check_collisions, robot_capsules

    caps = robot_capsules(robot, tool, tool_radius, cutter_len, cutter_radius)
    r_path = abc_to_rot(*abc)
    tool_inv = inv_frame(tool)
    t_path = np.eye(4)
    t_path[:3, :3] = r_path

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
        flange[:3, :3] = (base @ t_path)[:3, :3]
        flange[:3, 3] = mid_robot
        q, viol = inverse(robot, flange @ tool_inv, list(qa))
        return None if q is None else np.array(q, dtype=float)

    q = reach.joints if reach.joints is not None else np.full((len(points), 6), np.nan)
    ok = reach.ok_mask if reach.ok_mask is not None else np.zeros(len(points), dtype=bool)
    return check_collisions(robot, q, ok, caps, obstacles, margin, refine_fn=refine_fn, max_refine=max_refine), caps
