"""Emit KUKA KRL (.src/.dat) from motion items.

Two output modes:
  spline : SPLINE ... ENDSPLINE blocks with SLIN / SCIRC / SPL segments
           (planned by the KRC as one motion; recommended on KRC4).
  lin    : classic LIN / CIRC with C_DIS approximation.

Velocities are m/s ($VEL.CP), accelerations m/s^2 ($ACC.CP), distances mm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .geometry import turning_angle
from .translator import ArcItem, EventItem, LineItem, RapidItem, SplItem, Stats


@dataclass
class KrlOptions:
    name: str = "GCODE_PROG"
    mode: str = "spline"  # spline | lin
    tool: int = 1
    base: int = 1
    abc: tuple[float, float, float] = (0.0, 180.0, 0.0)
    rapid_vel: float = 0.5  # m/s
    max_vel: float = 2.0  # m/s
    acc: float = 2.0  # m/s^2
    cdis: float = 0.2  # mm, approximation distance for LIN/CIRC and rapids
    corner_angle_deg: float = 10.0
    exact_corners: bool = True
    max_block_segments: int = 500
    max_motions_per_file: int = 4000
    block_approx: bool = True  # ENDSPLINE C_SPL
    spindle_out: int = 0  # digital output number, 0 = comment only
    coolant_out: int = 0
    spindle_anout: int = 0  # analog output for spindle speed, 0 = none
    spindle_max_rpm: float = 24000.0
    bco_ptp: bool = True  # emit "PTP $POS_ACT" before the first motion
    advance: int = 3
    source_name: str = ""
    halt_on_tool_change: bool = True
    extruder_out: int = 0  # digital output switched on while material is deposited (0 = comment only)
    base_frame: tuple[float, ...] | None = None  # literal $BASE {X Y Z A B C} instead of BASE_DATA[n]
    tool_frame: tuple[float, ...] | None = None  # literal $TOOL {X Y Z A B C} instead of TOOL_DATA[n]


@dataclass
class KrlProgram:
    files: dict[str, str] = field(default_factory=dict)  # file name -> content
    main: str = ""


def fnum(x: float, nd: int = 3) -> str:
    s = f"{x:.{nd}f}"
    if s.startswith("-") and float(s) == 0.0:
        s = s[1:]
    return s


def fvel(v: float) -> str:
    return fnum(v, 4)


def check_name(name: str) -> str:
    name = name.strip()
    if not name or len(name) > 20 or not name[0].isalpha() or not all(c.isalnum() or c == "_" for c in name):
        raise ValueError("program name must be 1-20 chars (letters, digits, underscore) and start with a letter; "
                         "sub-program suffixes need 4 more characters")
    return name.upper()


class KrlWriter:
    def __init__(self, opts: KrlOptions):
        self.o = opts
        self.o.name = check_name(opts.name)
        self.abc = opts.abc
        # chunks: list of (lines, n_motions); blocks are atomic
        self.chunks: list[tuple[list[str], int]] = []
        self.block: list[str] = []
        self.block_feed: float | None = None
        self.block_segments = 0
        self.block_motions = 0
        self.last_t_out: np.ndarray | None = None
        self.global_vel: float | None = opts.rapid_vel  # set in the settings header
        self.first_motion_done = False

    # ------------------------------------------------------------ format
    def frame(self, p: np.ndarray) -> str:
        a, b, c = self.abc
        return (f"{{X {fnum(p[0])}, Y {fnum(p[1])}, Z {fnum(p[2])}, "
                f"A {fnum(a)}, B {fnum(b)}, C {fnum(c)}}}")

    def vel_of(self, feed_mm_min: float) -> float:
        v = feed_mm_min / 60000.0
        return max(0.0001, min(self.o.max_vel, v))

    # ------------------------------------------------------------ chunks
    def emit(self, lines: list[str], n_motions: int = 0) -> None:
        self.chunks.append((lines, n_motions))

    def set_global_vel(self, v: float) -> None:
        if self.global_vel is None or abs(self.global_vel - v) > 1e-9:
            self.emit([f"$VEL.CP = {fvel(v)}"])
            self.global_vel = v

    def ensure_bco(self) -> None:
        if not self.first_motion_done:
            self.first_motion_done = True
            if self.o.bco_ptp:
                self.emit(["PTP $POS_ACT  ; BCO run"])

    # ------------------------------------------------------------ spline
    def open_block(self, feed: float) -> None:
        self.block = [f"SPLINE WITH $VEL.CP = {fvel(self.vel_of(feed))}, $ACC.CP = {fnum(self.o.acc, 2)}"]
        self.block_feed = feed
        self.block_segments = 0
        self.block_motions = 0

    def close_block(self) -> None:
        if not self.block:
            return
        self.block.append("ENDSPLINE C_SPL" if self.o.block_approx else "ENDSPLINE")
        self.emit(self.block, self.block_motions)
        self.block = []
        self.block_feed = None

    def is_corner(self, item) -> bool:
        if self.last_t_out is None:
            return False
        return math.degrees(turning_angle(self.last_t_out, item.t_in)) > self.o.corner_angle_deg

    def spline_motion(self, item) -> None:
        self.ensure_bco()
        corner = self.is_corner(item)
        if self.block and (abs(self.block_feed - item.feed) > 1e-9 or corner
                           or self.block_segments >= self.o.max_block_segments):
            self.close_block()
        if not self.block:
            self.open_block(item.feed)
        if isinstance(item, LineItem):
            self.block.append(f"  SLIN {self.frame(item.end)}")
            self.block_segments += 1
            self.block_motions += 1
        elif isinstance(item, ArcItem):
            self.block.append(f"  SCIRC {self.frame(item.mid)}, {self.frame(item.end)}")
            self.block_segments += 1
            self.block_motions += 1
        else:
            for p in item.points[1:]:
                self.block.append(f"  SPL {self.frame(p)}")
            self.block_segments += len(item.points) - 1
            self.block_motions += len(item.points) - 1
        self.last_t_out = item.t_out

    # --------------------------------------------------------------- lin
    def lin_motion(self, item, next_item) -> None:
        self.ensure_bco()
        self.set_global_vel(self.vel_of(item.feed))
        approx = " C_DIS"
        if self.o.exact_corners and next_item is not None and hasattr(next_item, "t_in"):
            if math.degrees(turning_angle(item.t_out, next_item.t_in)) > self.o.corner_angle_deg:
                approx = ""
        elif next_item is None or not hasattr(next_item, "t_in"):
            approx = ""
        if isinstance(item, LineItem):
            self.emit([f"LIN {self.frame(item.end)}{approx}"], 1)
        elif isinstance(item, ArcItem):
            self.emit([f"CIRC {self.frame(item.mid)}, {self.frame(item.end)}{approx}"], 1)
        else:
            pts = item.points[1:]
            lines = [f"LIN {self.frame(p)} C_DIS" for p in pts[:-1]]
            lines.append(f"LIN {self.frame(pts[-1])}{approx}")
            self.emit(lines, len(pts))
        self.last_t_out = item.t_out

    # ------------------------------------------------------------ events
    def rapid(self, item: RapidItem, next_item) -> None:
        self.close_block()
        self.ensure_bco()
        self.set_global_vel(self.o.rapid_vel)
        approx = " C_DIS" if isinstance(next_item, RapidItem) else ""
        self.emit([f"LIN {self.frame(item.target)}{approx}"], 1)
        self.last_t_out = None

    def event(self, ev: EventItem) -> None:
        k = ev.kind
        if k == "comment":
            text = str(ev.value).replace("\n", " ")
            if self.block:
                self.block.append(f"  ; {text}")
            else:
                self.emit([f"; {text}"])
            return
        self.close_block()
        self.last_t_out = None
        if k == "dwell":
            self.emit([f"WAIT SEC {fnum(float(ev.value), 3)}"])
        elif k == "spindle_on":
            rpm, ccw = ev.value
            lines = [f"; spindle on {'CCW' if ccw else 'CW'} {rpm:g} rpm"]
            if self.o.spindle_anout > 0 and self.o.spindle_max_rpm > 0:
                lines.append(f"$ANOUT[{self.o.spindle_anout}] = {fnum(min(1.0, rpm / self.o.spindle_max_rpm), 4)}")
            if self.o.spindle_out > 0:
                lines.append(f"$OUT[{self.o.spindle_out}] = TRUE")
            self.emit(lines)
        elif k == "spindle_off":
            lines = ["; spindle off"]
            if self.o.spindle_out > 0:
                lines.append(f"$OUT[{self.o.spindle_out}] = FALSE")
            if self.o.spindle_anout > 0:
                lines.append(f"$ANOUT[{self.o.spindle_anout}] = 0.0")
            self.emit(lines)
        elif k == "spindle_speed":
            lines = [f"; spindle speed {float(ev.value):g} rpm"]
            if self.o.spindle_anout > 0 and self.o.spindle_max_rpm > 0:
                lines.append(f"$ANOUT[{self.o.spindle_anout}] = {fnum(min(1.0, float(ev.value) / self.o.spindle_max_rpm), 4)}")
            self.emit(lines)
        elif k in ("coolant_on", "coolant_off"):
            on = k == "coolant_on"
            lines = [f"; coolant {'on' if on else 'off'}"]
            if self.o.coolant_out > 0:
                lines.append(f"$OUT[{self.o.coolant_out}] = {'TRUE' if on else 'FALSE'}")
            self.emit(lines)
        elif k in ("extruder_on", "extruder_off"):
            on = k == "extruder_on"
            lines = [f"; extruder {'on' if on else 'off'}"]
            if self.o.extruder_out > 0:
                lines.append(f"$OUT[{self.o.extruder_out}] = {'TRUE' if on else 'FALSE'}")
            self.emit(lines)
        elif k == "tool_change":
            lines = [f"; tool change T{ev.value}"]
            if self.o.halt_on_tool_change:
                lines.append("HALT")
            self.emit(lines)
        elif k == "pause":
            self.emit(["; optional stop (M1)" if ev.value else "; program stop (M0)", "HALT"])
        elif k == "end":
            self.emit(["; program end"])
        else:
            self.emit([f"; unknown event {k}"])

    # --------------------------------------------------------------- run
    def write(self, items: list, stats: Stats | None = None) -> KrlProgram:
        for idx, item in enumerate(items):
            nxt = items[idx + 1] if idx + 1 < len(items) else None
            if isinstance(item, RapidItem):
                self.rapid(item, nxt)
            elif isinstance(item, (LineItem, ArcItem, SplItem)):
                if self.o.mode == "spline":
                    self.spline_motion(item)
                else:
                    self.lin_motion(item, nxt)
            elif isinstance(item, EventItem):
                self.event(item)
            else:
                raise TypeError(f"unknown item {item!r}")
        self.close_block()
        return self.assemble(stats)

    # ---------------------------------------------------------- assembly
    def header_lines(self, stats: Stats | None) -> list[str]:
        a, b, c = self.abc
        lines = [
            f"; Generated by gcode2krl" + (f" from {self.o.source_name}" if self.o.source_name else ""),
            f"; mode={self.o.mode} tool={self.o.tool} base={self.o.base} orientation A {fnum(a)} B {fnum(b)} C {fnum(c)}",
        ]
        if stats is not None:
            ic = stats.input_counts
            lines.append(f"; input: G0={ic.get('G0', 0)} G1={ic.get('G1', 0)} G2={ic.get('G2', 0)} G3={ic.get('G3', 0)}"
                         f"  output: lines={stats.lines_out} arcs={stats.arcs_out} spl={stats.spl_points}"
                         f" rapids={stats.rapids}  max_dev={stats.max_dev:.4f} mm")
        return lines

    @staticmethod
    def frame_literal(f) -> str:
        x, y, z, a, b, c = f
        return f"{{X {fnum(x)}, Y {fnum(y)}, Z {fnum(z)}, A {fnum(a)}, B {fnum(b)}, C {fnum(c)}}}"

    def settings_lines(self) -> list[str]:
        tool = self.frame_literal(self.o.tool_frame) if self.o.tool_frame else f"TOOL_DATA[{self.o.tool}]"
        base = self.frame_literal(self.o.base_frame) if self.o.base_frame else f"BASE_DATA[{self.o.base}]"
        return [
            f"$TOOL = {tool}",
            f"$BASE = {base}",
            f"$VEL.CP = {fvel(self.o.rapid_vel)}",
            f"$ACC.CP = {fnum(self.o.acc, 2)}",
            f"$APO.CDIS = {fnum(self.o.cdis, 2)}",
            f"$ADVANCE = {self.o.advance}",
        ]

    def assemble(self, stats: Stats | None) -> KrlProgram:
        prog = KrlProgram(main=self.o.name)
        # split chunks into files
        parts: list[list[str]] = []
        cur: list[str] = []
        cur_motions = 0
        for lines, n in self.chunks:
            if cur_motions > 0 and cur_motions + n > self.o.max_motions_per_file and n > 0:
                parts.append(cur)
                cur = []
                cur_motions = 0
            cur.extend(lines)
            cur_motions += n
        parts.append(cur)
        single = len(parts) == 1
        main_body: list[str] = []
        if single:
            main_body = parts[0]
        else:
            for k, body in enumerate(parts, start=1):
                sub = f"{self.o.name}_{k:03d}"
                prog.files[f"{sub}.src"] = self.sub_src(sub, body)
                prog.files[f"{sub}.dat"] = self.dat(sub)
                main_body.append(f"{sub}( )")
        prog.files[f"{self.o.name}.src"] = self.main_src(main_body, stats)
        prog.files[f"{self.o.name}.dat"] = self.dat(self.o.name)
        return prog

    def main_src(self, body: list[str], stats: Stats | None) -> str:
        lines = [
            "&ACCESS RVP",
            "&REL 1",
            "&PARAM EDITMASK = *",
            "&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe",
            f"DEF {self.o.name}( )",
            ";FOLD INI",
            "  ;FOLD BASISTECH INI",
            "    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )",
            "    INTERRUPT ON 3 ",
            "    BAS (#INITMOV,0 )",
            "  ;ENDFOLD (BASISTECH INI)",
            "  ;FOLD USER INI",
            "  ;ENDFOLD (USER INI)",
            ";ENDFOLD (INI)",
            "",
        ]
        lines += self.header_lines(stats)
        lines.append("")
        lines += self.settings_lines()
        lines.append("")
        lines += body
        lines.append("")
        lines.append("END")
        return "\n".join(lines) + "\n"

    def sub_src(self, name: str, body: list[str]) -> str:
        lines = ["&ACCESS RVP", "&REL 1", f"DEF {name}( )", f"; part of {self.o.name}", ""]
        lines += body
        lines += ["", "END"]
        return "\n".join(lines) + "\n"

    @staticmethod
    def dat(name: str) -> str:
        return "\n".join(["&ACCESS RVP", "&REL 1", f"DEFDAT {name}", "EXTERNAL DECLARATIONS", "ENDDAT"]) + "\n"


def write_krl(items: list, opts: KrlOptions, stats: Stats | None = None) -> KrlProgram:
    return KrlWriter(opts).write(items, stats)
