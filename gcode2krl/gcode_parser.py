"""Modal G-code parser producing a flat list of moves and machine events.

Supported:
  G0/G1 linear, G2/G3 arcs (I/J/K centre offsets or R radius, full circles,
  helical arcs), G4 dwell, G17/G18/G19 planes, G20/G21 units, G90/G91,
  G90.1/G91.1 arc-centre mode, G93/G94 feed modes (G93 warns), G54-G59
  (ignored, warned), G40-G43/G49 (ignored), G28/G30/G92/G53 (warned, skipped),
  M0/M1 pause, M2/M30 end, M3/M4/M5 spindle, M6 tool change, M7/M8/M9 coolant,
  F/S/T words, comments in parentheses and after ';'.
  Tool orientation: A B C words (KUKA ZYX Euler, modal) or I J K on G0/G1
  lines as the tool axis vector pointing away from the part (5-axis style,
  modal). Without them the orientation is constant (the translator default).

All output coordinates are in millimetres, absolute.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

WORD_RE = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
PAREN_COMMENT_RE = re.compile(r"\(([^)]*)\)")


@dataclass
class LinearMove:
    start: np.ndarray
    end: np.ndarray
    feed: float
    rapid: bool
    line_no: int
    extrude: float = 0.0  # E delta of this move (> 0 = material deposited)
    abc: tuple | None = None  # tool orientation A B C (KUKA ZYX, deg) at the end point, if given
    axis: tuple | None = None  # tool axis unit vector (pointing away from the part) at the end point, if given
    abc0: tuple | None = None  # orientation at the start point (modal state before this move)
    axis0: tuple | None = None


@dataclass
class ArcMove:
    start: np.ndarray
    end: np.ndarray
    center: np.ndarray
    normal: np.ndarray  # plane normal (+Z for G17, +Y for G18, +X for G19)
    ccw: bool
    feed: float
    line_no: int
    extrude: float = 0.0
    abc: tuple | None = None
    axis: tuple | None = None
    abc0: tuple | None = None
    axis0: tuple | None = None

    @property
    def helical(self) -> bool:
        return abs(float((self.end - self.start) @ self.normal)) > 1e-9


@dataclass
class Dwell:
    seconds: float
    line_no: int


@dataclass
class Spindle:
    on: bool
    ccw: bool
    rpm: float
    line_no: int


@dataclass
class SpindleSpeed:
    rpm: float
    line_no: int


@dataclass
class Coolant:
    on: bool
    line_no: int


@dataclass
class ToolChange:
    tool: int
    line_no: int


@dataclass
class Pause:
    optional: bool
    line_no: int


@dataclass
class ProgramEnd:
    line_no: int


@dataclass
class Comment:
    text: str
    line_no: int


Command = LinearMove | ArcMove | Dwell | Spindle | SpindleSpeed | Coolant | ToolChange | Pause | ProgramEnd | Comment

PLANES = {
    17: (0, 1, 2),  # XY, normal Z
    18: (2, 0, 1),  # ZX, normal Y
    19: (1, 2, 0),  # YZ, normal X
}


@dataclass
class ParseResult:
    commands: list = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    n_lines: int = 0
    counts: dict = field(default_factory=lambda: {"G0": 0, "G1": 0, "G2": 0, "G3": 0})


class GCodeParser:
    def __init__(self, start: tuple[float, float, float] = (0.0, 0.0, 0.0), default_feed: float = 0.0,
                 dwell_in_ms: bool = False, keep_comments: bool = True):
        self.pos = np.array(start, dtype=float)
        self.scale = 1.0  # 25.4 for G20
        self.absolute = True
        self.arc_absolute = False
        self.plane = 17
        self.motion: int | None = None
        self.feed = default_feed
        self.rpm = 0.0
        self.inverse_time = False
        self.e = 0.0
        self.e_absolute = True
        self.abc: tuple | None = None  # modal tool orientation from A B C words
        self.axis: tuple | None = None  # modal tool axis from I J K on G1 (5-axis style)
        self.dwell_in_ms = dwell_in_ms
        self.keep_comments = keep_comments
        self.result = ParseResult()

    # ----------------------------------------------------------------- utils
    def warn(self, line_no: int, msg: str) -> None:
        self.result.warnings.append(f"line {line_no}: {msg}")

    def emit(self, cmd) -> None:
        self.result.commands.append(cmd)

    # ------------------------------------------------------------------ main
    def parse(self, text: str) -> ParseResult:
        for idx, raw in enumerate(text.splitlines(), start=1):
            self.result.n_lines = idx
            self.parse_line(raw, idx)
        return self.result

    def parse_line(self, raw: str, line_no: int) -> None:
        line = raw.strip()
        if not line or line.startswith("%") or line.startswith("/"):
            return
        comments = []
        for m in PAREN_COMMENT_RE.finditer(line):
            comments.append(m.group(1).strip())
        line = PAREN_COMMENT_RE.sub(" ", line)
        if ";" in line:
            line, _, c = line.partition(";")
            comments.append(c.strip())
        if self.keep_comments:
            for c in comments:
                if c:
                    self.emit(Comment(c, line_no))
        line = line.strip()
        if not line:
            return
        if re.fullmatch(r"[Oo]\d+.*", line):
            return  # program number
        gcodes: list[float] = []
        mcodes: list[int] = []
        words: dict[str, float] = {}
        for letter, value in WORD_RE.findall(line):
            letter = letter.upper()
            val = float(value)
            if letter == "G":
                gcodes.append(round(val, 1))
            elif letter == "M":
                mcodes.append(int(val))
            elif letter == "N":
                continue
            else:
                words[letter] = val

        if "F" in words:
            self.feed = words["F"] * self.scale
        if "S" in words:
            self.rpm = words["S"]
            if not any(m in (3, 4) for m in mcodes):
                self.emit(SpindleSpeed(self.rpm, line_no))

        dwell = False
        for g in gcodes:
            gi = int(g)
            if g in (0.0, 1.0, 2.0, 3.0):
                self.motion = gi
            elif g == 4.0:
                dwell = True
            elif gi in PLANES and g == float(gi):
                self.plane = gi
            elif g == 20.0:
                self.scale = 25.4
            elif g == 21.0:
                self.scale = 1.0
            elif g == 90.0:
                self.absolute = True
            elif g == 91.0:
                self.absolute = False
            elif g == 90.1:
                self.arc_absolute = True
            elif g == 91.1:
                self.arc_absolute = False
            elif g == 93.0:
                self.inverse_time = True
                self.warn(line_no, "G93 inverse-time feed is not supported; feed words will be treated as mm/min")
            elif g == 94.0:
                self.inverse_time = False
            elif 54.0 <= g <= 59.3:
                self.warn(line_no, f"G{g:g} work offset ignored (coordinates passed through unchanged)")
            elif g in (40.0, 41.0, 42.0, 41.1, 42.1):
                if g != 40.0:
                    self.warn(line_no, f"G{g:g} cutter compensation is not applied by the translator")
            elif g in (43.0, 43.1, 49.0, 64.0, 61.0, 61.1, 80.0, 98.0, 99.0, 50.0, 69.0):
                pass
            elif g in (28.0, 30.0, 28.1, 30.1):
                self.warn(line_no, f"G{g:g} homing skipped; position is assumed unchanged")
                return
            elif g == 92.0 and "E" in words and not any(a in words for a in "XYZ"):
                self.e = words["E"]
                return
            elif g in (92.0, 92.1, 92.2, 92.3, 53.0, 10.0, 52.0, 68.0):
                self.warn(line_no, f"G{g:g} is not supported and was skipped")
                return
            elif g == 5.0 or g == 5.1 or g == 5.2 or g == 5.3:
                self.warn(line_no, f"G{g:g} spline input is not supported and was skipped")
                return
            elif 81.0 <= g <= 89.0:
                self.warn(line_no, f"G{g:g} canned cycle is not supported and was skipped")
                return
            else:
                self.warn(line_no, f"unknown G{g:g} ignored")

        if dwell:
            p = words.get("P", words.get("X", words.get("U", 0.0)))
            seconds = p / 1000.0 if self.dwell_in_ms else p
            self.emit(Dwell(seconds, line_no))
            return

        for m in mcodes:
            if m == 82:
                self.e_absolute = True
                continue
            if m == 83:
                self.e_absolute = False
                continue
            if m == 0:
                self.emit(Pause(False, line_no))
            elif m == 1:
                self.emit(Pause(True, line_no))
            elif m in (2, 30):
                self.emit(ProgramEnd(line_no))
            elif m in (3, 4):
                self.emit(Spindle(True, m == 4, self.rpm, line_no))
            elif m == 5:
                self.emit(Spindle(False, False, self.rpm, line_no))
            elif m == 6:
                self.emit(ToolChange(int(words.get("T", 0)), line_no))
            elif m in (7, 8):
                self.emit(Coolant(True, line_no))
            elif m == 9:
                self.emit(Coolant(False, line_no))
            elif m in (48, 49, 10, 11, 60, 104, 105, 106, 107, 109, 140, 190, 117, 73, 220, 221, 400):
                pass
            else:
                self.warn(line_no, f"M{m} ignored")

        has_axis = any(a in words for a in "XYZ")
        has_arc_words = any(a in words for a in "IJKR")
        abc0, axis0 = self.abc, self.axis
        if any(a in words for a in "ABC"):
            cur = list(self.abc) if self.abc else [0.0, 0.0, 0.0]
            for i, a in enumerate("ABC"):
                if a in words:
                    cur[i] = words[a]
            self.abc = tuple(cur)
            self.axis = None
        if self.motion in (0, 1) and any(a in words for a in "IJK") and "R" not in words:
            v = np.array([words.get("I", 0.0), words.get("J", 0.0), words.get("K", 0.0)], dtype=float)
            n = float(np.linalg.norm(v))
            if n > 1e-9:
                self.axis = tuple((v / n).tolist())
                self.abc = None
        de = 0.0
        if "E" in words:
            if self.e_absolute:
                de = words["E"] - self.e
                self.e = words["E"]
            else:
                de = words["E"]
                self.e += de
        if self.motion is None:
            if has_axis:
                self.warn(line_no, "axis words without a motion mode (G0/G1/G2/G3) ignored")
            return
        if not has_axis and not (self.motion in (2, 3) and has_arc_words):
            return

        target = self.pos.copy()
        for i, a in enumerate("XYZ"):
            if a in words:
                val = words[a] * self.scale
                target[i] = val if self.absolute else self.pos[i] + val

        if self.motion in (0, 1):
            self.result.counts[f"G{self.motion}"] += 1
            if float(np.linalg.norm(target - self.pos)) > 1e-9:
                self.emit(LinearMove(self.pos.copy(), target, self.feed, self.motion == 0, line_no, de, self.abc, self.axis, abc0, axis0))
            self.pos = target
            return

        # --- arcs ---------------------------------------------------------
        self.result.counts[f"G{self.motion}"] += 1
        ccw = self.motion == 3
        ia, ib, inorm = PLANES[self.plane]
        normal = np.zeros(3)
        normal[inorm] = 1.0
        start = self.pos.copy()
        if "R" in words:
            center = self._center_from_radius(start, target, words["R"] * self.scale, ia, ib, ccw, line_no)
            if center is None:
                self.pos = target
                return
        else:
            offs = {"I": 0, "J": 1, "K": 2}
            center = start.copy()
            given = False
            for letter, axis in offs.items():
                if letter in words:
                    given = True
                    val = words[letter] * self.scale
                    center[axis] = val if self.arc_absolute else start[axis] + val
            if not given:
                self.warn(line_no, "arc without I/J/K or R skipped (treated as linear move)")
                self.emit(LinearMove(start, target, self.feed, False, line_no))
                self.pos = target
                return
            center[inorm] = start[inorm]
            center = self._reconcile_center(start, target, center, ia, ib, line_no)
        self.emit(ArcMove(start, target, center, normal, ccw, self.feed, line_no, de, self.abc, self.axis, abc0, axis0))
        self.pos = target

    # ------------------------------------------------------------------ arcs
    def _center_from_radius(self, start, end, r, ia, ib, ccw, line_no):
        dx = end[ia] - start[ia]
        dy = end[ib] - start[ib]
        d = math.hypot(dx, dy)
        if d < 1e-9:
            self.warn(line_no, "R-format arc with coincident start and end skipped (use I/J for full circles)")
            return None
        h2 = r * r - (d / 2.0) ** 2
        if h2 < 0:
            if h2 > -1e-6 * max(1.0, r * r):
                h2 = 0.0
            else:
                self.warn(line_no, f"R={r:g} too small for chord {d:.4f}; arc skipped")
                return None
        h = math.sqrt(h2)
        mx, my = start[ia] + dx / 2.0, start[ib] + dy / 2.0
        nx, ny = -dy / d, dx / d
        # Positive R -> minor arc; negative R -> major arc.
        sgn = 1.0 if (ccw == (r > 0)) else -1.0
        center = start.copy()
        center[ia] = mx + sgn * h * nx
        center[ib] = my + sgn * h * ny
        return center

    def _reconcile_center(self, start, end, center, ia, ib, line_no):
        r1 = math.hypot(start[ia] - center[ia], start[ib] - center[ib])
        r2 = math.hypot(end[ia] - center[ia], end[ib] - center[ib])
        if abs(r1 - r2) <= 1e-6:
            return center
        tol = 0.002 + 0.001 * max(r1, r2)
        if abs(r1 - r2) > tol:
            self.warn(line_no, f"arc radius mismatch start {r1:.4f} vs end {r2:.4f}; centre moved to the bisector")
        dx = end[ia] - start[ia]
        dy = end[ib] - start[ib]
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return center
        mx, my = start[ia] + dx / 2.0, start[ib] + dy / 2.0
        nx, ny = -dy / d, dx / d
        t = (center[ia] - mx) * nx + (center[ib] - my) * ny
        fixed = center.copy()
        fixed[ia] = mx + t * nx
        fixed[ib] = my + t * ny
        return fixed


def parse_gcode(text: str, **kwargs) -> ParseResult:
    return GCodeParser(**kwargs).parse(text)
