"""Support generation (planar layers): overhang areas propagated down to the build
plate, with Z gap, XY gap, interface layers and a lines / grid pattern.

Returns the support area to print on every layer and the interface part of it.
"""
from __future__ import annotations

import math

from shapely.geometry import MultiPolygon

from .params import SliceParams

EMPTY = MultiPolygon()


def _multi(g) -> MultiPolygon:
    if g is None or g.is_empty:
        return EMPTY
    if g.geom_type == "Polygon":
        return MultiPolygon([g])
    if g.geom_type == "MultiPolygon":
        return g
    polys = [x for x in getattr(g, "geoms", []) if x.geom_type == "Polygon" and x.area > 1e-9]
    return MultiPolygon(polys) if polys else EMPTY


def overhang_areas(regions: list, heights: list[float], angle_deg: float, width: float, min_area: float) -> list:
    """Per layer: the part of the layer that the layer below does not carry within the
    printable overhang angle (the layer below is grown by ``tan(angle) * h`` plus
    half a bead). Layer 0 sits on the plate and never needs support."""
    reach = math.tan(math.radians(max(0.0, min(89.0, angle_deg))))
    out = [EMPTY]
    for i in range(1, len(regions)):
        cur, prev = regions[i], regions[i - 1]
        if cur.is_empty:
            out.append(EMPTY)
            continue
        allowed = prev.buffer(reach * heights[i] + 0.5 * width) if not prev.is_empty else None
        unsupported = cur if allowed is None else cur.difference(allowed)
        unsupported = _multi(unsupported.buffer(-0.25 * width).buffer(0.25 * width))  # drop slivers
        if unsupported.is_empty:
            out.append(EMPTY)
            continue
        keep = [p for p in unsupported.geoms if p.area >= min_area]
        out.append(MultiPolygon(keep) if keep else EMPTY)
    return out


def parse_support_modifiers(text: str) -> list[dict]:
    """Lines ``block|force cx cy cz sx sy sz`` (model coordinates, Z from the bed)."""
    out = []
    for ln, raw in enumerate((text or "").splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.replace(",", " ").split()
        try:
            kind = parts[0].lower()
            if kind not in ("block", "force"):
                raise ValueError("kind must be block or force")
            nums = [float(v) for v in parts[1:7]]
            if len(nums) != 6 or any(v <= 0 for v in nums[3:]):
                raise ValueError("needs cx cy cz sx sy sz with positive sizes")
            out.append({"kind": kind, "center": nums[:3], "size": nums[3:]})
        except (IndexError, ValueError) as exc:
            raise ValueError(f"support modifiers line {ln}: {raw.strip()!r}: {exc}") from None
    return out


def apply_support_modifiers(printed: list, interface: list, regions: list, zs: list[float], modifiers: list[dict], gap: float) -> None:
    """Blockers remove support inside their box, enforcers add support (minus the part) inside theirs."""
    from shapely.geometry import box as sbox

    for i, z in enumerate(zs):
        for m in modifiers:
            cx, cy, cz = m["center"]
            sx, sy, sz = m["size"]
            if not (cz - sz / 2.0 - 1e-9 <= z <= cz + sz / 2.0 + 1e-9):
                continue
            b = sbox(cx - sx / 2.0, cy - sy / 2.0, cx + sx / 2.0, cy + sy / 2.0)
            if m["kind"] == "block":
                printed[i] = _multi(printed[i].difference(b)) if not printed[i].is_empty else EMPTY
                interface[i] = _multi(interface[i].difference(b)) if not interface[i].is_empty else EMPTY
            else:
                add = b.difference(regions[i].buffer(gap)) if not regions[i].is_empty else b
                if i == len(zs) - 1 or regions[i + 1].is_empty:
                    pass
                printed[i] = _multi(printed[i].union(add))


def support_areas(regions: list, heights: list[float], params: SliceParams) -> tuple[list, list]:
    """Support to print on each layer and its interface part (dense, right under the part).

    Overhang islands of layer i must be carried by support on layers below; the
    support column is propagated down (minus the part grown by the XY gap) until it
    reaches the plate or the part. The top ``support_z_gap`` layers under an
    overhang are left empty so the support detaches."""
    n = len(regions)
    w = params.bead_width
    oh = overhang_areas(regions, heights, params.overhang_max, w, params.support_min_area)
    gap = params.support_xy_gap
    need = [EMPTY] * n  # support column (before the Z gap is applied)
    carry = EMPTY
    for i in range(n - 1, -1, -1):
        # support needed on layer i: overhangs of the layers above that have not been reached yet
        if i + 1 < n:
            carry = _multi(carry.union(oh[i + 1].buffer(0.5 * w)))
        if carry.is_empty:
            continue
        carry = _multi(carry.difference(regions[i].buffer(gap))) if not regions[i].is_empty else carry
        need[i] = carry
    zg = max(0, params.support_z_gap)
    ni = max(0, params.support_interface_layers)
    printed, interface = [], []
    for i in range(n):
        s = need[i]
        if s.is_empty:
            printed.append(EMPTY)
            interface.append(EMPTY)
            continue
        # Z gap: nothing where the part lies within the next zg layers above
        for k in range(1, zg + 1):
            if i + k < n and not regions[i + k].is_empty:
                s = _multi(s.difference(regions[i + k].buffer(gap)))
        if s.is_empty:
            printed.append(EMPTY)
            interface.append(EMPTY)
            continue
        # interface: the part of the column within ni layers below a part surface (after the gap)
        iface = EMPTY
        for k in range(zg + 1, zg + ni + 1):
            j = i + k
            if j < n and not regions[j].is_empty:
                # downward-facing surface at layer j: what appears there and was not in the layer below
                face = regions[j].difference(regions[j - 1]) if j > 0 and not regions[j - 1].is_empty else regions[j]
                if not face.is_empty:
                    iface = _multi(iface.union(s.intersection(face.buffer(gap + w))))
        printed.append(s)
        interface.append(iface)
    return printed, interface


def support_summary(printed: list, interface: list, heights: list[float]) -> dict:
    vol = sum(float(p.area) * h for p, h in zip(printed, heights))
    layers = sum(1 for p in printed if not p.is_empty)
    return {"support_volume": vol, "support_layers_generated": layers}
