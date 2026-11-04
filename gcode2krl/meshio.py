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
