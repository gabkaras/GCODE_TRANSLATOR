"""STEP (and IGES) import: tessellate CAD geometry into triangles (mm).

Backends, tried in this order:
  * OCP (``pip install cadquery-ocp``): OpenCASCADE bindings, BRepMesh with a
    linear deflection tolerance; STEP units are converted to millimetres.
  * gmsh (``pip install gmsh``; needs libGLU on Linux): OpenCASCADE kernel
    inside gmsh, surface mesh with curvature-based sizing.
"""
from __future__ import annotations

import os

import numpy as np

STEP_EXTS = ("step", "stp", "iges", "igs")


def available_backends() -> list[str]:
    out = []
    try:
        import OCP  # noqa: F401
        out.append("ocp")
    except Exception:  # noqa: BLE001
        pass
    try:
        import gmsh  # noqa: F401
        out.append("gmsh")
    except Exception:  # noqa: BLE001
        pass
    return out


def _ocp_tessellate(path: str, deflection: float, angle: float) -> np.ndarray:
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    ext = path.lower().rsplit(".", 1)[-1]
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    if ext in ("iges", "igs"):
        from OCP.IGESControl import IGESControl_Reader
        reader = IGESControl_Reader()
    else:
        from OCP.STEPControl import STEPControl_Reader
        reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise ValueError(f"{os.path.basename(path)}: cannot be read as {ext.upper()}")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise ValueError(f"{os.path.basename(path)}: no geometry found")
    BRepMesh_IncrementalMesh(shape, float(deflection), False, float(angle), True)
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    tris: list[np.ndarray] = []
    while ex.More():
        face = TopoDS.Face_s(ex.Current()) if hasattr(TopoDS, "Face_s") else TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None and tri.NbTriangles() > 0:
            trsf = loc.Transformation()
            n = tri.NbNodes()
            pts = np.empty((n, 3))
            for i in range(1, n + 1):
                p = tri.Node(i).Transformed(trsf)
                pts[i - 1] = (p.X(), p.Y(), p.Z())
            idx = np.empty((tri.NbTriangles(), 3), dtype=int)
            for k in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(k).Get()
                idx[k - 1] = (a - 1, b - 1, c - 1)
            if face.Orientation() == TopAbs_REVERSED:
                idx = idx[:, [0, 2, 1]]
            tris.append(pts[idx])
        ex.Next()
    if not tris:
        raise ValueError(f"{os.path.basename(path)}: tessellation produced no triangles")
    return np.concatenate(tris)


def _gmsh_tessellate(path: str, deflection: float, angle: float) -> np.ndarray:
    import gmsh

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setString("Geometry.OCCTargetUnit", "MM")
        gmsh.model.add("import")
        gmsh.model.occ.importShapes(path)
        gmsh.model.occ.synchronize()
        bb = gmsh.model.getBoundingBox(-1, -1)
        diag = float(np.linalg.norm(np.array(bb[3:]) - np.array(bb[:3]))) or 100.0
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", max(8, int(2 * 3.14159 / max(angle, 0.05))))
        gmsh.option.setNumber("Mesh.MeshSizeMax", max(deflection * 20.0, diag / 20.0))
        gmsh.option.setNumber("Mesh.MeshSizeMin", deflection)
        gmsh.model.mesh.generate(2)
        tags, coords, _ = gmsh.model.mesh.getNodes()
        pts = np.array(coords).reshape(-1, 3)
        idmap = {int(t): i for i, t in enumerate(tags)}
        etypes, _, enodes = gmsh.model.mesh.getElements(2)
        tris = []
        for et, en in zip(etypes, enodes):
            if et == 2:
                tris.append(pts[np.array([idmap[int(n)] for n in en]).reshape(-1, 3)])
        if not tris:
            raise ValueError(f"{os.path.basename(path)}: tessellation produced no triangles")
        return np.concatenate(tris)
    finally:
        gmsh.finalize()


def step_to_triangles(path: str, deflection: float = 0.2, angle: float = 0.3) -> np.ndarray:
    """Tessellate a STEP/IGES file into (N,3,3) triangles in millimetres.

    ``deflection``: max chord distance between mesh and surface (mm);
    ``angle``: angular deflection (rad). Smaller = finer and bigger.
    """
    backends = available_backends()
    if not backends:
        raise ValueError("STEP import needs a CAD kernel: pip install cadquery-ocp  (or: pip install gmsh)")
    errors = []
    for b in backends:
        try:
            return _ocp_tessellate(path, deflection, angle) if b == "ocp" else _gmsh_tessellate(path, deflection, angle)
        except Exception as exc:  # noqa: BLE001 - try the next backend
            errors.append(f"{b}: {exc}")
    raise ValueError("STEP import failed: " + "; ".join(errors))
