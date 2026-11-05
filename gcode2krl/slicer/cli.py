"""gcode2krl-slice: slice STL/OBJ/STEP into G-code (then feed it to gcode2krl)."""
from __future__ import annotations

import argparse
import sys

from .params import SliceParams
from .pipeline import slice_mesh


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="gcode2krl-slice", description="Slice a mesh into G-code for robot extrusion.")
    p.add_argument("input", help="STL, OBJ, STEP or IGES file")
    p.add_argument("-o", "--output", default=None, help="output .gcode (default: input name with .gcode)")
    p.add_argument("--units", default="mm", help="STL/OBJ units (mm, cm, m, in); STEP uses its own")
    p.add_argument("--deflection", type=float, default=0.1, help="STEP tessellation tolerance, mm")
    d = SliceParams()
    for name, val in d.to_dict().items():
        if name in ("start_gcode", "end_gcode"):
            p.add_argument("--" + name.replace("_", "-"), default=val, help=f"{name} text")
        elif isinstance(val, bool):
            p.add_argument("--" + name.replace("_", "-"), default=val, type=lambda v: v.lower() in ("1", "true", "yes", "on"), help=f"{name} (default {val})")
        else:
            p.add_argument("--" + name.replace("_", "-"), default=val, type=type(val), help=f"{name} (default {val})")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    params = SliceParams.from_dict({k: getattr(args, k) for k in SliceParams().to_dict()})
    gcode, st, _ = slice_mesh(args.input, params, args.units, args.deflection)
    out = args.output or (args.input.rsplit(".", 1)[0] + ".gcode")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(gcode)
    print(f"{out}: {st.layers} layers, {st.paths} paths, extrusion {st.extrusion_length:.0f} mm, travel {st.travel_length:.0f} mm, "
          f"E {st.e_total:.1f}, est. {st.time_s / 60:.1f} min" + (f", {st.dropped_chains} open chains dropped" if st.dropped_chains else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
