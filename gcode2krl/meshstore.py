"""Local store for imported meshes (tables, fixtures, tools).

Files live in ``~/.gcode2krl/meshes`` (override with GCODE2KRL_MESH_DIR). A mesh is
referenced by its id (sha1 of the content, first 16 hex digits) or by a file path.
"""
from __future__ import annotations

import hashlib
import json
import os
import time

import numpy as np

from .meshio import load_mesh, mesh_stats

_cache: dict[tuple, np.ndarray] = {}


def mesh_dir() -> str:
    d = os.environ.get("GCODE2KRL_MESH_DIR") or os.path.join(os.path.expanduser("~"), ".gcode2krl", "meshes")
    os.makedirs(d, exist_ok=True)
    return d


def save_mesh(name: str, data: bytes) -> dict:
    """Store uploaded mesh bytes; returns {id, name, path, tris, min, max}."""
    ext = name.lower().rsplit(".", 1)[-1] if "." in name else ""
    if ext not in ("stl", "obj"):
        raise ValueError("only STL and OBJ files are supported (export STEP/IGES from CAD as STL)")
    mid = hashlib.sha1(data).hexdigest()[:16]
    path = os.path.join(mesh_dir(), f"{mid}.{ext}")
    if not os.path.exists(path):
        with open(path, "wb") as fh:
            fh.write(data)
    tris = load_mesh(path, "mm")
    meta = {"id": mid, "name": os.path.basename(name), "path": path, "added": time.time(), **mesh_stats(tris)}
    with open(os.path.join(mesh_dir(), f"{mid}.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh)
    return meta


def list_meshes() -> list[dict]:
    out = []
    d = mesh_dir()
    for f in sorted(os.listdir(d)):
        if f.endswith(".json"):
            try:
                with open(os.path.join(d, f), encoding="utf-8") as fh:
                    out.append(json.load(fh))
            except (OSError, ValueError):
                continue
    return out


def delete_mesh(mid: str) -> bool:
    d = mesh_dir()
    found = False
    for f in os.listdir(d):
        if f.startswith(mid + "."):
            os.remove(os.path.join(d, f))
            found = True
    return found


def resolve(ref: str) -> str:
    """A path (absolute or relative to the CWD) or a mesh id -> file path."""
    if os.path.isfile(ref):
        return os.path.abspath(ref)
    d = mesh_dir()
    for ext in ("stl", "obj"):
        p = os.path.join(d, f"{ref}.{ext}")
        if os.path.isfile(p):
            return p
    raise ValueError(f"mesh {ref!r} not found (give a file path or an imported mesh id)")


def load(ref: str, units: str = "mm") -> tuple[np.ndarray, str]:
    """Triangles (mm) and a cache key for an imported mesh or a file path."""
    path = resolve(ref)
    key = (path, os.path.getmtime(path), str(units).lower())
    if key not in _cache:
        _cache[key] = load_mesh(path, units)
    return _cache[key], f"{path}:{units}"
