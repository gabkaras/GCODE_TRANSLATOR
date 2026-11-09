"""Height ranges: per-Z overrides of slicing parameters (like the height-range
modifiers of desktop slicers) and layer-hook lines for the robot program.

Range syntax, one per line::

    z_from z_to key=value [key=value ...]
    0 20 layer_height=1 speed_outer=10 infill_density=0.5

Hook syntax (``layer_krl``), one per line::

    layer 10: WAIT SEC 5          # before layer index 10 (0-based)
    z 50: HALT                    # before the first layer at or above Z = 50 mm
    every: $OUT[12] = TRUE        # before every layer
    every 5: WAIT FOR $IN[3]      # before every 5th layer
"""
from __future__ import annotations

import dataclasses
import re

from .params import SliceParams

# fields that may differ from layer to layer
RANGE_FIELDS = {
    "layer_height", "bead_width", "outer_wall_width", "inner_wall_width", "infill_width", "top_bottom_width", "perimeters", "wall_order",
    "gap_fill", "xy_compensation", "hole_compensation", "seam", "small_perimeter_length", "infill_density", "infill_pattern", "infill_angle",
    "infill_overlap", "infill_combine", "solid_threshold_area", "top_layers", "bottom_layers", "top_pattern", "bottom_pattern", "bridges",
    "bridge_flow", "overhang_detect", "speed_perimeter", "speed_outer", "speed_infill", "speed_top", "speed_bridge", "speed_overhang",
    "speed_gap", "speed_support", "small_perimeter_speed", "min_layer_time", "min_speed", "extrusion_multiplier", "max_volumetric_speed",
    "tilt_lead", "tilt_side", "fuzzy_skin", "fuzzy_thickness", "fuzzy_point_distance", "ironing", "ironing_flow", "ironing_spacing",
    "ironing_speed", "support_density", "support_pattern", "support_interface_density",
}


def parse_ranges(text: str) -> list[dict]:
    out: list[dict] = []
    for ln, raw in enumerate((text or "").splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = re.split(r"\s+", line)
        try:
            z0, z1 = float(parts[0]), float(parts[1])
            if z1 <= z0:
                raise ValueError("z_to must be above z_from")
            over = {}
            for kv in parts[2:]:
                k, _, v = kv.partition("=")
                if k not in RANGE_FIELDS:
                    raise ValueError(f"unknown or non-overridable field {k!r}")
                over[k] = v
            if not over:
                raise ValueError("no key=value overrides")
            out.append({"z0": z0, "z1": z1, "set": over, "raw": raw.strip()})
        except (IndexError, ValueError) as exc:
            raise ValueError(f"ranges line {ln}: {raw.strip()!r}: {exc}") from None
    return out


def params_for_z(params: SliceParams, ranges: list[dict], z: float) -> SliceParams:
    """Parameters valid at height ``z`` (top of the layer); later lines win."""
    over: dict = {}
    for r in ranges:
        if r["z0"] - 1e-9 <= z <= r["z1"] + 1e-9:
            over.update(r["set"])
    if not over:
        return params
    return SliceParams.from_dict({**params.to_dict(), **over})


def layer_heights_with_ranges(z_max: float, params: SliceParams, ranges: list[dict]) -> tuple[list[float], list[float]]:
    """Layer tops and heights when ranges override ``layer_height``."""
    zs, hs = [], []
    z = 0.0
    h = params.first_layer_height
    first = True
    while z + 1e-6 < z_max:
        if not first:
            h = params_for_z(params, ranges, z + 1e-6).layer_height
        h = max(0.05, float(h))
        z = round(z + h, 6)
        zs.append(z)
        hs.append(h)
        first = False
        if len(zs) > 100000:
            break
    return zs, hs


def parse_hooks(text: str) -> list[dict]:
    out: list[dict] = []
    for ln, raw in enumerate((text or "").splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        m = re.match(r"^(layer|z|every)\s*([\d.]*)\s*:\s*(.+)$", line, re.I)
        if not m:
            raise ValueError(f"hooks line {ln}: {raw.strip()!r}: expected 'layer N: ...', 'z Z: ...' or 'every [N]: ...'")
        kind, num, krl = m.group(1).lower(), m.group(2), m.group(3).strip()
        if kind in ("layer", "z") and not num:
            raise ValueError(f"hooks line {ln}: {kind} needs a number")
        out.append({"kind": kind, "value": float(num) if num else 1.0, "krl": krl})
    return out


def hooks_for_layer(hooks: list[dict], index: int, z: float, prev_z: float | None) -> list[str]:
    lines = []
    for h in hooks:
        if h["kind"] == "layer" and int(h["value"]) == index:
            lines.append(h["krl"])
        elif h["kind"] == "z" and (prev_z is None or prev_z < h["value"] - 1e-9) and z >= h["value"] - 1e-9:
            lines.append(h["krl"])
        elif h["kind"] == "every" and index % max(1, int(h["value"])) == 0:
            lines.append(h["krl"])
    return lines
