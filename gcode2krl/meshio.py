"""Minimal STL reader/writer (binary and ASCII) returning (N,3,3) triangle arrays."""
from __future__ import annotations

import re
import struct

import numpy as np

_ASCII_VERTEX = re.compile(rb"vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)")


def load_stl(path: str) -> np.ndarray:
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:5] == b"solid" and b"facet" in data[:2000]:
        v = _ASCII_VERTEX.findall(data)
        if v:
            return np.array(v, dtype=float).reshape(-1, 3, 3)
    if len(data) < 84:
        raise ValueError(f"{path}: not an STL file")
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    if len(data) < 84 + n * 50:
        raise ValueError(f"{path}: truncated binary STL")
    arr = np.frombuffer(data[84 : 84 + n * 50], dtype=rec)
    return arr["v"].reshape(-1, 3, 3).astype(float)


def save_stl(path: str, tris: np.ndarray, name: str = "mesh") -> None:
    tris = np.asarray(tris, dtype=np.float32).reshape(-1, 3, 3)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(normals, axis=1)
    normals = normals / np.where(ln < 1e-12, 1.0, ln)[:, None]
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"] = normals
    rec["v"] = tris.reshape(-1, 9)
    header = (name.encode("ascii", "replace")[:80]).ljust(80, b"\0")
    with open(path, "wb") as fh:
        fh.write(header)
        fh.write(struct.pack("<I", len(tris)))
        fh.write(rec.tobytes())


def flat_normals(tris: np.ndarray) -> np.ndarray:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1)
    n = n / np.where(ln < 1e-12, 1.0, ln)[:, None]
    return np.repeat(n[:, None, :], 3, axis=1)


def load_obj(path: str) -> np.ndarray:
    """Minimal Wavefront OBJ reader (v / f lines, polygons fan-triangulated)."""
    verts: list[list[float]] = []
    tris: list[list[list[float]]] = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith("v "):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                idx = []
                for tok in line.split()[1:]:
                    i = int(tok.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                for k in range(1, len(idx) - 1):
                    tris.append([verts[idx[0]], verts[idx[k]], verts[idx[k + 1]]])
    if not tris:
        raise ValueError(f"{path}: no faces found")
    return np.array(tris, dtype=float)


UNIT_SCALE = {"mm": 1.0, "cm": 10.0, "m": 1000.0, "in": 25.4, "inch": 25.4}


def load_mesh(path: str, units: str = "mm", deflection: float = 0.2) -> np.ndarray:
    """Load STL, OBJ, STEP or IGES and return triangles in millimetres.
    STEP/IGES carry their own units and are tessellated with ``deflection`` (mm);
    ``units`` applies to STL/OBJ only."""
    ext = path.lower().rsplit(".", 1)[-1]
    if ext == "obj":
        tris = load_obj(path)
    elif ext == "stl":
        tris = load_stl(path)
    elif ext in ("step", "stp", "iges", "igs"):
        from .stepio import step_to_triangles
        return step_to_triangles(path, deflection)
    else:
        raise ValueError(f"unsupported mesh format .{ext} (use STL, OBJ, STEP or IGES)")
    scale = UNIT_SCALE.get(str(units).lower())
    if scale is None:
        raise ValueError(f"unknown units {units!r}")
    return tris * scale


def mesh_stats(tris: np.ndarray) -> dict:
    v = tris.reshape(-1, 3)
    return {"tris": int(len(tris)), "min": v.min(axis=0).tolist(), "max": v.max(axis=0).tolist()}
