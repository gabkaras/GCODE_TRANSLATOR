from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class SliceParams:
    """Slicing parameters (mm, mm/min). Defaults suit a robot pellet/filament extruder."""

    layer_height: float = 1.0
    first_layer_height: float = 1.0
    bead_width: float = 2.0  # extrusion width
    perimeters: int = 2
    top_layers: int = 3
    bottom_layers: int = 3
    infill_density: float = 0.2  # 0..1
    infill_pattern: str = "lines"  # lines | grid | concentric | none
    infill_angle: float = 45.0  # deg, alternates +/- 90 every layer for "lines"
    infill_overlap: float = 0.15  # fraction of bead width the infill overlaps the inner perimeter
    seam: str = "nearest"  # nearest | aligned | random
    speed_perimeter: float = 1800.0
    speed_outer: float = 1200.0
    speed_infill: float = 2400.0
    speed_first_layer: float = 900.0
    speed_travel: float = 6000.0
    retract_length: float = 0.0  # mm of E to retract on travel (0 = none)
    retract_speed: float = 1800.0
    z_hop: float = 0.0
    min_travel_for_retract: float = 5.0
    extrusion_mode: str = "volumetric"  # volumetric | filament | none
    filament_diameter: float = 1.75  # used when extrusion_mode == "filament"
    extrusion_multiplier: float = 1.0
    skirt_loops: int = 0
    skirt_distance: float = 5.0
    brim_width: float = 0.0
    min_feature: float = 0.5  # polygons thinner than this are dropped
    origin_center: bool = True  # move the model so its XY bounding box centre is at (0, 0)
    z_offset: float = 0.0
    start_gcode: str = ""
    end_gcode: str = ""
    comments: bool = True
    # ---- multi-axis layer modes (deform -> slice -> undeform)
    layer_mode: str = "planar"  # planar | conical | bottom | top
    cone_angle: float = 30.0  # conical: nozzle tilt (deg)
    cone_outward: bool = True  # conical: layers slope down away from the axis (print outwards)
    cone_cx: float | None = None  # conical axis (model XY after centring); None = model centre
    cone_cy: float | None = None
    hf_cell: float = 5.0  # bottom/top: height-field cell (mm)
    max_slope: float = 45.0  # bottom/top: max layer tilt (deg)
    tool_axis: bool = True  # non-planar modes: write I J K (tool axis) with every point
    overhang_max: float = 45.0  # support indicator: printable overhang angle (deg)

    def extrusion_per_mm(self, width: float, height: float) -> float:
        """E units per mm of path for a bead of width x height (rounded-rectangle cross section)."""
        area = height * (width - height) + 3.14159265 * (height / 2.0) ** 2 if width > height else 3.14159265 * (width / 2.0) ** 2
        area *= self.extrusion_multiplier
        if self.extrusion_mode == "volumetric":
            return area  # mm^3 per mm
        if self.extrusion_mode == "filament":
            fil = 3.14159265 * (self.filament_diameter / 2.0) ** 2
            return area / fil
        return 0.0

    def to_dict(self) -> dict:
        return dict(self.__dict__)

    @classmethod
    def from_dict(cls, d: dict) -> "SliceParams":
        p = cls()
        for k, v in (d or {}).items():
            if k in ("cone_cx", "cone_cy"):
                setattr(p, k, None if v in (None, "", "auto") else float(v))
                continue
            if hasattr(p, k) and v is not None and v != "":
                cur = getattr(p, k)
                if isinstance(cur, bool):
                    setattr(p, k, v in (True, "true", "1", 1, "on"))
                elif isinstance(cur, int):
                    setattr(p, k, int(float(v)))
                elif isinstance(cur, float):
                    setattr(p, k, float(v))
                else:
                    setattr(p, k, str(v))
        return p
