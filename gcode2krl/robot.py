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

import math
from dataclasses import dataclass, field

import numpy as np

DEG = math.pi / 180.0


@dataclass
class RobotModel:
    key: str
    name: str
    d1: float
    a1: float
    a2: float
    a3: float
    d4: float
    d6: float
    limits: list[tuple[float, float]]  # deg, per axis
    speeds: list[float]  # deg/s, per axis (nominal)
    payload: float = 0.0
    reach: float = 0.0
    home: tuple[float, ...] = (0.0, -90.0, 90.0, 0.0, 0.0, 0.0)
    # visual hints (mm): base radius, column width, arm section sizes
    visual: dict = field(default_factory=dict)

    @property
    def wrist_reach(self) -> float:
        return self.a2 + math.hypot(self.d4, self.a3)

    def to_dict(self) -> dict:
        return {
            "key": self.key, "name": self.name, "d1": self.d1, "a1": self.a1, "a2": self.a2, "a3": self.a3,
            "d4": self.d4, "d6": self.d6, "limits": self.limits, "speeds": self.speeds, "payload": self.payload,
            "reach": self.reach, "home": list(self.home), "visual": self.visual, "wrist_reach": self.wrist_reach,
        }


ROBOTS: dict[str, RobotModel] = {}


def _add(m: RobotModel) -> None:
    ROBOTS[m.key] = m


_add(RobotModel("KR90R2700PRO", "KUKA KR 90 R2700 pro (QUANTEC)", d1=675, a1=350, a2=1150, a3=41, d4=1200, d6=215,
                limits=[(-185, 185), (-140, -5), (-120, 155), (-350, 350), (-125, 125), (-350, 350)],
                speeds=[136, 120, 120, 190, 190, 260], payload=90, reach=2696,
                visual={"base_r": 330, "base_h": 280, "column": 420, "arm": 260, "forearm": 200, "wrist": 160}))
_add(RobotModel("KR120R2700EXTRA", "KUKA KR 120 R2700 extra (QUANTEC)", d1=675, a1=350, a2=1150, a3=41, d4=1200, d6=215,
                limits=[(-185, 185), (-140, -5), (-120, 155), (-350, 350), (-125, 125), (-350, 350)],
                speeds=[120, 115, 120, 190, 180, 260], payload=120, reach=2701,
                visual={"base_r": 330, "base_h": 280, "column": 420, "arm": 260, "forearm": 200, "wrist": 160}))
_add(RobotModel("KR210R2700EXTRA", "KUKA KR 210 R2700 extra (QUANTEC)", d1=675, a1=350, a2=1150, a3=41, d4=1200, d6=240,
                limits=[(-185, 185), (-140, -5), (-120, 155), (-350, 350), (-125, 125), (-350, 350)],
                speeds=[123, 115, 112, 179, 172, 219], payload=210, reach=2701,
                visual={"base_r": 340, "base_h": 280, "column": 440, "arm": 280, "forearm": 220, "wrist": 180}))
_add(RobotModel("KR60R2033", "KUKA KR 60-3 (approx.)", d1=815, a1=350, a2=850, a3=0, d4=820, d6=170,
                limits=[(-185, 185), (-135, 35), (-120, 158), (-350, 350), (-119, 119), (-350, 350)],
                speeds=[128, 102, 128, 260, 245, 322], payload=60, reach=2033,
                visual={"base_r": 300, "base_h": 260, "column": 380, "arm": 220, "forearm": 170, "wrist": 140}))
_add(RobotModel("KR16R1610", "KUKA KR 16-2 (approx.)", d1=675, a1=260, a2=680, a3=35, d4=670, d6=158,
                limits=[(-185, 185), (-155, 35), (-130, 154), (-350, 350), (-130, 130), (-350, 350)],
                speeds=[156, 156, 156, 330, 330, 615], payload=16, reach=1611,
                visual={"base_r": 220, "base_h": 200, "column": 280, "arm": 170, "forearm": 130, "wrist": 110}))
_add(RobotModel("KR6R900", "KUKA KR 6 R900 sixx (AGILUS)", d1=400, a1=25, a2=455, a3=35, d4=420, d6=80,
                limits=[(-170, 170), (-190, 45), (-120, 156), (-185, 185), (-120, 120), (-350, 350)],
                speeds=[360, 300, 360, 381, 388, 615], payload=6, reach=901,
                visual={"base_r": 110, "base_h": 120, "column": 150, "arm": 100, "forearm": 80, "wrist": 70}))

DEFAULT_ROBOT = "KR90R2700PRO"


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


# ------------------------------------------------------------------ forward
def forward(robot: RobotModel, joints_deg) -> list[np.ndarray]:
    """Return 4x4 frames: [base, A2 axis, A3 axis, A5 centre (wrist), flange]."""
    q = [float(v) * DEG for v in joints_deg]
    t = np.eye(4)
    frames = [t.copy()]
    r1 = rot_z(-q[0])
    t = t @ _h(r1, (0, 0, 0))
    t = t @ _h(np.eye(3), (robot.a1, 0, robot.d1))
    frames.append(t.copy())  # A2 axis
    t = t @ _h(rot_y(q[1]), (0, 0, 0))
    t = t @ _h(np.eye(3), (robot.a2, 0, 0))
    frames.append(t.copy())  # A3 axis
    t = t @ _h(rot_y(q[2]), (0, 0, 0))
    t = t @ _h(np.eye(3), (robot.d4, 0, -robot.a3))
    frames.append(t.copy())  # A5 centre
    t = t @ _h(rot_x(-q[3]), (0, 0, 0))
    t = t @ _h(rot_y(q[4]), (0, 0, 0))
    t = t @ _h(rot_x(-q[5]), (0, 0, 0))
    t = t @ _h(FLANGE0, (robot.d6, 0, 0))
    frames.append(t.copy())  # flange (Z out of the flange)
    return frames


def _h(r: np.ndarray, p) -> np.ndarray:
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
    phi = math.atan2(robot.a3, robot.d4)
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
