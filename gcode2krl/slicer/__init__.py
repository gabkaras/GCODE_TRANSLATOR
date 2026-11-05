"""gcode2krl slicer: mesh -> layers -> perimeters, infill, travels -> G-code.

Original code (MIT). Polygon offsetting and boolean operations use Shapely
(BSD-3-Clause, GEOS under LGPL via dynamic linking).
"""
from .params import SliceParams
from .pipeline import slice_mesh, slice_triangles

__all__ = ["SliceParams", "slice_mesh", "slice_triangles"]
