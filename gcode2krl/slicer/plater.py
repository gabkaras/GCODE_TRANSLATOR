"""Plater: model placement (rotate / scale / mirror / offset), automatic orientation
(lay flat, minimal overhang) and combining several models for one print.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Placement:
    """Transform of one model on the print bed (applied in this order: mirror,
    scale, rotation X -> Y -> Z, offset)."""
    name: str = "model"
    mesh: str = ""  # mesh store reference (server) or file path (CLI)
    units: str = "mm"
    rot: tuple[float, float, float] = (0.0, 0.0, 0.0)  # deg about X, Y, Z
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    mirror: tuple[bool, bool, bool] = (False, False, False)
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)  # mm, after centring of the combined model
    extra: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, d: dict) -> "Placement":
        def tup(key, n, default, cast=float):
            v = d.get(key, default)
            if isinstance(v, (int, float, bool)):
                v = [v] * n
            v = list(v) + list(default)[len(v):]
            return tuple(cast(x) for x in v[:n])
        sc = d.get("scale", 1.0)
        if isinstance(sc, (int, float, str)):
            sc = [float(sc)] * 3
        return cls(name=str(d.get("name") or d.get("mesh") or "model"), mesh=str(d.get("mesh") or ""), units=str(d.get("units") or "mm"),
                   rot=tup("rot", 3, (0.0, 0.0, 0.0)), scale=tuple(float(x) for x in list(sc)[:3] + [1.0] * (3 - len(list(sc)[:3]))),
                   mirror=tup("mirror", 3, (False, False, False), lambda x: x in (True, "true", "1", 1, "on")),
                   offset=tup("offset", 3, (0.0, 0.0, 0.0)))

    def matrix(self) -> np.ndarray:
        r = rotation_xyz(*self.rot)
        s = np.diag([(-1.0 if m else 1.0) * sc for m, sc in zip(self.mirror, self.scale)])
        return r @ s


def rotation_xyz(rx: float, ry: float, rz: float) -> np.ndarray:
    """Rotation about the fixed X, then Y, then Z axis (deg)."""
    a, b, c = math.radians(rx), math.radians(ry), math.radians(rz)
    rxm = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    rym = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    rzm = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    return rzm @ rym @ rxm


def transform_triangles(tris: np.ndarray, placement: Placement) -> np.ndarray:
    """Apply the placement; the model is set on Z = 0 and its XY bounding-box centre
    is moved to the offset."""
    t = np.asarray(tris, dtype=float)
    if len(t) == 0:
        return t
    m = placement.matrix()
    v = t.reshape(-1, 3) @ m.T
    if np.linalg.det(m) < 0:
        # a mirror flips the winding: restore outward normals
        v = v.reshape(-1, 3, 3)[:, ::-1, :].reshape(-1, 3)
    v = v.reshape(-1, 3, 3)
    flat = v.reshape(-1, 3)
    cx = (flat[:, 0].min() + flat[:, 0].max()) / 2.0
    cy = (flat[:, 1].min() + flat[:, 1].max()) / 2.0
    shift = np.array([placement.offset[0] - cx, placement.offset[1] - cy, placement.offset[2] - flat[:, 2].min()])
    return v + shift


def facet_normals_areas(tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    t = np.asarray(tris, dtype=float)
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    area = 0.5 * np.linalg.norm(n, axis=1)
    ok = area > 1e-12
    unit = np.zeros_like(n)
    unit[ok] = n[ok] / (2.0 * area[ok])[:, None]
    return unit, area


def overhang_area(tris: np.ndarray, overhang_deg: float = 45.0) -> float:
    """Area of downward facing facets steeper than the printable overhang (the
    facets that would need support), excluding those lying on the bed."""
    n, area = facet_normals_areas(tris)
    zmin = float(np.asarray(tris)[:, :, 2].min())
    on_bed = np.all(np.abs(np.asarray(tris)[:, :, 2] - zmin) < 1e-6, axis=1)
    down = n[:, 2] < -math.sin(math.radians(overhang_deg)) - 1e-9
    return float(area[down & ~on_bed].sum())


def candidate_rotations(tris: np.ndarray, n_faces: int = 24) -> list[np.ndarray]:
    """Rotations that put a (large) facet direction of the convex hull onto the
    bed, plus the six axis-aligned ones."""
    n, area = facet_normals_areas(tris)
    # cluster normals: keep the biggest distinct directions
    order = np.argsort(-area)
    dirs: list[np.ndarray] = []
    for i in order:
        d = n[i]
        if area[i] <= 0 or any(float(d @ e) > 0.999 for e in dirs):
            continue
        dirs.append(d)
        if len(dirs) >= n_faces:
            break
    for e in ([1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]):
        dirs.append(np.array(e, dtype=float))
    rots = []
    for d in dirs:
        # rotation taking d to -Z (so the facet faces the bed)
        target = np.array([0.0, 0.0, -1.0])
        c = float(np.clip(d @ target, -1.0, 1.0))
        if c > 1 - 1e-9:
            rots.append(np.eye(3))
            continue
        if c < -1 + 1e-9:
            rots.append(np.diag([1.0, -1.0, -1.0]))
            continue
        k = np.cross(d, target)
        s = float(np.linalg.norm(k))
        k /= s
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        rots.append(np.eye(3) + s * kx + (1 - c) * kx @ kx)
    return rots


def rotation_to_xyz(r: np.ndarray) -> tuple[float, float, float]:
    """Decompose into the fixed-axis X, Y, Z angles used by Placement."""
    sy = -r[2, 0]
    cy = math.hypot(r[0, 0], r[1, 0])
    b = math.atan2(sy, cy)
    if cy < 1e-9:
        a = math.atan2(-r[1, 2], r[1, 1])
        c = 0.0
    else:
        a = math.atan2(r[2, 1], r[2, 2])
        c = math.atan2(r[1, 0], r[0, 0])
    return math.degrees(a), math.degrees(b), math.degrees(c)


def auto_orient(tris: np.ndarray, mode: str = "min_overhang", overhang_deg: float = 45.0) -> dict:
    """Best orientation: ``lay_flat`` = largest flat facet of the hull on the bed,
    ``min_overhang`` = least facet area needing support (ties: lower height, bigger
    bed contact)."""
    t = np.asarray(tris, dtype=float)
    base_score = overhang_area(t, overhang_deg)
    best = None
    for r in candidate_rotations(t):
        v = t.reshape(-1, 3) @ r.T
        tt = v.reshape(-1, 3, 3)
        zmin = tt[:, :, 2].min()
        height = float(tt[:, :, 2].max() - zmin)
        n, area = facet_normals_areas(tt)
        contact = float(area[(n[:, 2] < -0.999) & (np.abs(tt[:, :, 2] - zmin).max(axis=1) < 1e-6)].sum())
        oh = overhang_area(tt, overhang_deg)
        if mode == "lay_flat":
            score = (-contact, oh, height)
        else:
            score = (oh, -contact, height)
        if best is None or score < best[0]:
            best = (score, r, oh, contact, height)
    _, r, oh, contact, height = best
    rx, ry, rz = rotation_to_xyz(r)
    return {"rot": (rx, ry, rz), "overhang_before": base_score, "overhang_after": oh, "contact_area": contact, "height": height}


def combine(models: list[tuple[np.ndarray, Placement]]) -> np.ndarray:
    """Transformed triangles of all models in one soup."""
    parts = [transform_triangles(t, pl) for t, pl in models if len(t)]
    return np.concatenate(parts) if parts else np.zeros((0, 3, 3))
