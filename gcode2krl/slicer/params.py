from __future__ import annotations

from dataclasses import dataclass

PI = 3.14159265358979


def bead_area(width: float, height: float) -> float:
    """Cross-section (mm^2) of a bead ``width`` x ``height`` with rounded sides."""
    if width > height:
        return height * (width - height) + PI * (height / 2.0) ** 2
    return PI * (width / 2.0) ** 2


@dataclass
class SliceParams:
    """Slicing parameters for robot extrusion (mm, mm/s, mm^3). Defaults suit a 6-axis
    robot with a pellet / paste extruder (4 mm bead, 2 mm layers).

    Feature set follows the big desktop slicers (PrusaSlicer / OrcaSlicer / Cura):
    per-feature widths and speeds, wall order, gap fill, thin walls, XY and
    elephant-foot compensation, spiral vase, several sparse-infill patterns,
    infill combining, top/bottom patterns, bridge detection, overhang slow-down,
    support generation with Z/XY gaps and interface layers, raft, adaptive
    layer heights, flow control (multiplier, first-layer flow, max volumetric
    speed), minimum layer time, retraction with wipe / coast / extra prime.
    """

    # ---- model placement -----------------------------------------------
    rot_x: float = 0.0  # deg about X (fixed axes, X then Y then Z)
    rot_y: float = 0.0
    rot_z: float = 0.0
    scale: float = 1.0
    mirror_x: bool = False
    mirror_y: bool = False
    mirror_z: bool = False
    auto_orient: str = "none"  # none | lay_flat | min_overhang (replaces rot_x/y/z)
    print_sequence: str = "all"  # several models: all (layer by layer) | sequential (object by object)
    # ---- height ranges / hooks ------------------------------------------
    ranges: str = ""  # per-Z overrides: "z_from z_to key=value ..." per line
    layer_krl: str = ""  # robot program hooks: "layer N: KRL", "z Z: KRL", "every [N]: KRL"
    layer_wait: float = 0.0  # s, WAIT SEC before every layer except the first (curing / cooling)
    # ---- layers ---------------------------------------------------------
    layer_height: float = 2.0
    first_layer_height: float = 2.0
    adaptive_layers: bool = False  # variable layer height from the surface slope (planar only)
    layer_height_min: float = 1.0
    layer_height_max: float = 3.0
    adaptive_quality: float = 0.5  # max stair-step cusp height (mm)
    # ---- walls ----------------------------------------------------------
    bead_width: float = 4.0  # default extrusion width (nozzle) (used where a per-feature width is 0)
    outer_wall_width: float = 0.0
    inner_wall_width: float = 0.0
    infill_width: float = 0.0
    top_bottom_width: float = 0.0
    perimeters: int = 2
    wall_order: str = "inner_outer"  # inner_outer | outer_inner | infill_first
    gap_fill: bool = True  # fill gaps between walls / thin walls with single beads
    gap_fill_min_width: float = 0.25  # as a fraction of the bead width
    xy_compensation: float = 0.0  # grow (+) / shrink (-) every contour (mm)
    hole_compensation: float = 0.0  # grow (+) holes (mm)
    elephant_foot: float = 0.0  # shrink the first layer contour (mm)
    seam: str = "nearest"  # nearest | aligned | rear | corner | random
    fuzzy_skin: str = "none"  # none | outer | all: jitter the wall points for a rough surface
    fuzzy_thickness: float = 0.6  # mm, peak-to-peak jitter
    fuzzy_point_distance: float = 3.0  # mm between jittered points
    spiral_vase: bool = False  # one continuous outer wall with a Z ramp, no infill above the bottom layers
    small_perimeter_length: float = 0.0  # loops shorter than this (mm) use small_perimeter_speed (0 = off)
    # ---- sparse infill --------------------------------------------------
    infill_density: float = 0.2  # 0..1
    infill_pattern: str = "lines"  # lines | zigzag | grid | triangles | cubic | gyroid | honeycomb | concentric | none
    infill_angle: float = 45.0  # deg, alternates +/- 90 every layer for "lines"
    infill_overlap: float = 0.15  # fraction of bead width the infill overlaps the inner perimeter
    infill_combine: int = 1  # print sparse infill every N layers with N x flow
    solid_threshold_area: float = 0.0  # sparse areas smaller than this (mm^2) become solid
    # ---- top / bottom ---------------------------------------------------
    top_layers: int = 2
    bottom_layers: int = 2
    top_pattern: str = "zigzag"  # lines | zigzag | monotonic | concentric
    bottom_pattern: str = "zigzag"
    bridges: bool = True  # detect bridges: straight lines across, anchored on both ends
    bridge_flow: float = 1.0
    ironing: bool = False  # extra low-flow pass over top surfaces
    ironing_flow: float = 0.15
    ironing_spacing: float = 1.0  # mm between ironing lines
    ironing_speed: float = 15.0  # mm/s
    # ---- overhangs / support -------------------------------------------
    overhang_detect: bool = True  # slow down wall segments printed over air
    overhang_max: float = 45.0  # printable overhang angle (deg): support indicator / generation threshold
    support: str = "none"  # none | normal (columns) | tree (branches that merge into trunks)
    support_density: float = 0.2
    support_pattern: str = "lines"  # lines | grid | walls (one wall ring + sparse lines, good for trees)
    support_z_gap: int = 1  # layers left empty between support top and the part
    support_xy_gap: float = 1.0  # mm between support and the part's walls
    support_interface_layers: int = 2
    support_interface_density: float = 0.6
    support_min_area: float = 4.0  # ignore overhang islands smaller than this (mm^2)
    support_angle: float = 0.0  # support line direction (deg)
    tree_branch_angle: float = 40.0  # deg from vertical a branch may lean per layer
    tree_tip_diameter: float = 0.0  # mm, 0 = 2 x bead width
    tree_trunk_diameter: float = 40.0  # mm, largest branch / trunk diameter
    tree_diameter_growth: float = 5.0  # deg: radius grows by tan(angle) per mm descended
    tree_tip_spacing: float = 0.0  # mm between tips on an overhang (0 = auto)
    support_on_model: bool = True  # tree branches may rest on the part
    support_modifiers: str = ""  # "block cx cy cz sx sy sz" / "force cx cy cz sx sy sz" per line (model coordinates)
    # ---- extrusion ------------------------------------------------------
    extrusion_mode: str = "volumetric"  # volumetric (E in mm^3, drives the robot extruder outputs) | none
    extrusion_multiplier: float = 1.0
    first_layer_flow: float = 1.0
    max_volumetric_speed: float = 0.0  # extruder capacity in mm^3/s, 0 = unlimited (caps the feed of every path)
    material_density: float = 1.24  # g/cm^3, for the mass estimate
    retract_length: float = 0.0  # mm^3 run backwards before a travel (KRL: reverse output pulse), 0 = none
    retract_extra_prime: float = 0.0  # extra mm^3 primed after the travel (KRL: WAIT with the extruder on)
    retract_layer_change: bool = False
    min_travel_for_retract: float = 5.0
    z_hop: float = 0.0
    wipe_distance: float = 0.0  # mm moved back along a closed loop while retracting
    coast_length: float = 0.0  # mm at the end of a closed loop printed without extrusion
    # ---- speeds (mm/s, TCP speed of the robot) --------------------------
    speed_perimeter: float = 30.0
    speed_outer: float = 20.0
    speed_infill: float = 40.0
    speed_top: float = 20.0
    speed_bridge: float = 15.0
    speed_overhang: float = 10.0
    speed_gap: float = 20.0
    speed_support: float = 40.0
    speed_first_layer: float = 15.0
    speed_travel: float = 150.0
    small_perimeter_speed: float = 10.0
    min_layer_time: float = 0.0  # s; slow the whole layer down so it takes at least this long
    min_speed: float = 5.0  # mm/s floor for the layer-time slow-down
    # ---- adhesion -------------------------------------------------------
    skirt_loops: int = 0
    skirt_distance: float = 5.0
    skirt_layers: int = 1  # print the skirt on the first N layers (a draft shield when high)
    prime_line_length: float = 0.0  # mm, straight purge line in front of the part before printing (0 = none)
    prime_line_offset: float = 20.0  # mm from the part / brim
    brim_width: float = 0.0
    raft_layers: int = 0
    raft_margin: float = 5.0
    # ---- misc -----------------------------------------------------------
    min_feature: float = 0.3  # polygons thinner than this are dropped
    origin_center: bool = True  # move the model so its XY bounding box centre is at (0, 0)
    z_offset: float = 0.0
    comments: bool = True
    # ---- multi-axis layer modes (deform -> slice -> undeform) -----------
    layer_mode: str = "planar"  # planar | conical | bottom | top
    cone_angle: float = 30.0  # conical: nozzle tilt (deg)
    cone_outward: bool = True  # conical: layers slope down away from the axis (print outwards)
    cone_cx: float | None = None  # conical axis (model XY after centring); None = model centre
    cone_cy: float | None = None
    hf_cell: float = 5.0  # bottom/top: height-field cell (mm)
    max_slope: float = 45.0  # bottom/top: max layer tilt (deg)
    tool_axis: bool = True  # write I J K (tool axis) with every point when the orientation varies / is tilted
    tilt_lead: float = 0.0  # deg, nozzle axis tilted towards (+) / against (-) the direction of travel
    tilt_side: float = 0.0  # deg, nozzle axis tilted to the left (+) / right (-) of the direction of travel

    # ---- derived --------------------------------------------------------
    def width(self, feature: str) -> float:
        """Extrusion width for a feature kind (falls back to ``bead_width``)."""
        w = {"WALL-OUTER": self.outer_wall_width, "WALL-INNER": self.inner_wall_width, "FILL": self.infill_width,
             "SKIN": self.top_bottom_width, "BRIDGE": self.top_bottom_width}.get(feature, 0.0)
        return w if w and w > 0 else self.bead_width

    def extrusion_per_mm(self, width: float, height: float) -> float:
        """E units per mm of path for a bead of width x height (rounded-rectangle cross section)."""
        area = bead_area(width, height) * self.extrusion_multiplier
        if self.extrusion_mode == "none":
            return 0.0
        return area  # mm^3 per mm

    @property
    def planar(self) -> bool:
        return self.layer_mode in ("planar", "multi")

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
