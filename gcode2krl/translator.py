"""Pipeline: parsed G-code commands -> KRL motion items."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .arc_fitter import ArcElement, FitParams, LineElement, dedupe_points, segment_polyline
from .gcode_parser import (ArcMove, Comment, Coolant, Dwell, ExtrudeOnly, LinearMove, ParseResult, Pause,
                           ProgramEnd, Spindle, SpindleSpeed, ToolChange)
from .geometry import ArcFit, arc_from_center, sample_arc, unit
from .simplify import douglas_peucker


@dataclass
class TranslateOptions:
    tol: float = 0.05
    max_radius: float = 5000.0
    min_radius: float = 0.1
    min_arc_points: int = 4
    refine_shift: int = 3
    spl_max_segment: float = 3.0  # lines shorter than this (and 2 vertices only) become SPL points
    corner_angle_deg: float = 10.0
    default_feed: float = 1000.0  # mm/min when the G-code never sets F
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    fit_arcs: bool = True
    use_spl: bool = True
    extruder: bool = False  # emit extruder on/off events around extruding moves (3D printing / WAAM)
    extruder_analog: bool = False  # also emit extrusion-rate events (E per mm) for a velocity-proportional analog output


# ---------------------------------------------------------------- items
@dataclass
class RapidItem:
    target: np.ndarray
    start: np.ndarray
    abc: tuple | None = None
    axis: tuple | None = None


@dataclass
class LineItem:
    start: np.ndarray
    end: np.ndarray
    feed: float
    n_points: int = 2
    abc: tuple | None = None  # orientation at the end point (None = program default)

    @property
    def t_in(self):
        return unit(self.end - self.start)

    t_out = t_in


@dataclass
class ArcItem:
    start: np.ndarray
    mid: np.ndarray
    end: np.ndarray
    feed: float
    fit: ArcFit
    abc: tuple | None = None

    @property
    def t_in(self):
        return self.fit.t_in

    @property
    def t_out(self):
        return self.fit.t_out


@dataclass
class SplItem:
    points: np.ndarray  # includes the start point as row 0
    feed: float
    abcs: np.ndarray | None = None  # (n, 3) orientation per point (row 0 = start), None = default
    axes: np.ndarray | None = None  # (n, 3) tool axes when the orientation is still to be resolved

    @property
    def start(self):
        return self.points[0]

    @property
    def end(self):
        return self.points[-1]

    @property
    def t_in(self):
        return unit(self.points[1] - self.points[0])

    @property
    def t_out(self):
        return unit(self.points[-1] - self.points[-2])


@dataclass
class EventItem:
    kind: str  # dwell, spindle_on, spindle_off, spindle_speed, coolant_on, coolant_off, tool_change, pause, end, comment
    value: object = None


MotionItem = LineItem | ArcItem | SplItem


@dataclass
class Stats:
    input_counts: dict = field(default_factory=dict)
    chains: int = 0
    chain_vertices: int = 0
    oriented_vertices: int = 0
    lines_out: int = 0
    arcs_out: int = 0
    spl_runs: int = 0
    spl_points: int = 0
    rapids: int = 0
    max_dev: float = 0.0
    warnings: list[str] = field(default_factory=list)

    @property
    def motions_out(self) -> int:
        return self.lines_out + self.arcs_out + self.spl_points + self.rapids


class Translator:
    def __init__(self, opts: TranslateOptions):
        self.o = opts
        self.items: list = []
        self.stats = Stats()
        self.chain: list[np.ndarray] = []
        self.chain_feed = 0.0
        self.chain_ori: list = []  # per chain point: ("abc", (a,b,c)) | ("axis", (i,j,k)) | None
        self.pending_comments: list[EventItem] = []
        self.extruding = False
        self.rate = None
        self.last_ori = None
        self.offset = np.array(opts.offset, dtype=float)
        self.fit_params = FitParams(tol=opts.tol, max_radius=opts.max_radius, min_radius=opts.min_radius,
                                    min_arc_points=opts.min_arc_points, refine_shift=opts.refine_shift)

    # ------------------------------------------------------------ chains
    def feed_of(self, f: float) -> float:
        if f <= 0:
            if "no-feed" not in self.stats.warnings:
                self.stats.warnings.append("no-feed")
            return self.o.default_feed
        return f

    def push_linear(self, start: np.ndarray, end: np.ndarray, feed: float, ori=None, ori0=None) -> None:
        feed = self.feed_of(feed)
        if self.chain and (feed != self.chain_feed or float(np.linalg.norm(self.chain[-1] - start)) > 1e-9):
            self.flush()
        if not self.chain:
            self.chain = [start]
            self.chain_feed = feed
            self.chain_ori = [ori0 if ori0 is not None else self.last_ori]
        self.chain.append(end)
        self.chain_ori.append(ori)
        self.last_ori = ori

    def flush(self) -> None:
        if len(self.chain) < 2:
            self.chain = []
            self.chain_ori = []
            self.items.extend(self.pending_comments)
            self.pending_comments = []
            return
        raw = np.array(self.chain)
        oris = list(self.chain_ori)
        feed = self.chain_feed
        self.chain = []
        self.chain_ori = []
        # orientation varies along the chain -> keep every vertex with its orientation (SPL interpolates it)
        kinds = {o[0] for o in oris if o is not None}
        values = {tuple(np.round(o[1], 6)) for o in oris if o is not None}
        if len(values) > 1:
            keep = [0] + [k for k in range(1, len(raw)) if float(np.linalg.norm(raw[k] - raw[k - 1])) > 1e-9]
            pts_v = raw[keep]
            o_v = [oris[k] for k in keep]
            self.stats.chains += 1
            self.stats.chain_vertices += len(pts_v)
            self.stats.oriented_vertices += len(pts_v)
            # fill leading None with the first defined orientation
            first = next((o for o in o_v if o is not None), None)
            o_v = [o if o is not None else first for o in o_v]
            from .robot import abc_from_axis
            if "abc" in kinds:
                # explicit A B C (possibly mixed with I J K at transitions): fixed orientations, no spin solving
                abcs = np.array([(o[1] if o[0] == "abc" else abc_from_axis(o[1])) for o in o_v], dtype=float)
                self.add_motion(SplItem(pts_v, feed, abcs=abcs))
            else:
                axes = np.array([o[1] for o in o_v], dtype=float)
                abcs = np.array([abc_from_axis(ax) for ax in axes], dtype=float)
                self.add_motion(SplItem(pts_v, feed, abcs=abcs, axes=axes))
            self.items.extend(self.pending_comments)
            self.pending_comments = []
            return
        chain_abc = None
        if values:
            o0 = next(o for o in oris if o is not None)
            if o0[0] == "abc":
                chain_abc = tuple(float(v) for v in o0[1])
            else:
                from .robot import abc_from_axis
                chain_abc = abc_from_axis(o0[1])
        pts = dedupe_points(raw)
        if len(pts) < 2:
            return
        self.stats.chains += 1
        self.stats.chain_vertices += len(pts)
        if len(pts) == 2 or not self.o.fit_arcs:
            elems = [LineElement(k, k + 1, pts[k], pts[k + 1], 0.0) for k in range(len(pts) - 1)]
        else:
            elems = segment_polyline(pts, self.fit_params)
        motion: list[MotionItem] = []
        for e in elems:
            self.stats.max_dev = max(self.stats.max_dev, e.max_dev)
            if isinstance(e, ArcElement):
                motion.extend(self.arc_items(e.fit, e.start, e.end, feed))
            else:
                motion.append(LineItem(e.start, e.end, feed, e.n_points))
        if self.o.use_spl:
            motion = self.group_spl(motion, pts)
        for m in motion:
            if chain_abc is not None:
                if isinstance(m, SplItem):
                    m.abcs = np.tile(np.array(chain_abc, dtype=float), (len(m.points), 1))
                else:
                    m.abc = chain_abc
            self.add_motion(m)
        self.items.extend(self.pending_comments)
        self.pending_comments = []

    def add_motion(self, m: MotionItem) -> None:
        if isinstance(m, LineItem):
            self.stats.lines_out += 1
        elif isinstance(m, ArcItem):
            self.stats.arcs_out += 1
        else:
            self.stats.spl_runs += 1
            self.stats.spl_points += len(m.points) - 1
        self.items.append(m)

    def arc_items(self, fit: ArcFit, start: np.ndarray, end: np.ndarray, feed: float) -> list[ArcItem]:
        """One ArcItem for open arcs, two half arcs for a full circle."""
        if not fit.closed:
            return [ArcItem(start, fit.mid, end, feed, fit)]
        half = ArcFit(fit.center, fit.radius, fit.normal, fit.u, fit.v, fit.theta0, math.pi, fit.max_dev, False)
        p_half = fit.point_at(fit.theta0 + math.pi)
        second = ArcFit(fit.center, fit.radius, fit.normal, fit.u, fit.v, fit.theta0 + math.pi, math.pi,
                        fit.max_dev, False)
        return [ArcItem(start, half.mid, p_half, feed, half), ArcItem(p_half, second.mid, end, feed, second)]

    def group_spl(self, motion: list[MotionItem], pts: np.ndarray) -> list[MotionItem]:
        """Collapse runs of short two-vertex lines into SPL point runs."""
        corner = math.radians(self.o.corner_angle_deg)
        out: list[MotionItem] = []
        run: list[LineItem] = []

        def is_spl_candidate(m) -> bool:
            return isinstance(m, LineItem) and m.n_points == 2 and float(np.linalg.norm(m.end - m.start)) <= self.o.spl_max_segment

        def flush_run():
            nonlocal run
            if len(run) >= 2:
                points = np.array([run[0].start] + [m.end for m in run])
                thin = douglas_peucker(points, self.o.tol)
                if len(thin) >= 3:
                    out.append(SplItem(thin, run[0].feed))
                else:
                    out.append(LineItem(thin[0], thin[-1], run[0].feed, len(points)))
            else:
                out.extend(run)
            run = []

        for m in motion:
            if is_spl_candidate(m):
                if run and self._angle(run[-1].t_out, m.t_in) > corner:
                    flush_run()
                run.append(m)
            else:
                flush_run()
                out.append(m)
        flush_run()
        return out

    @staticmethod
    def _angle(a, b) -> float:
        d = float(np.clip(unit(a) @ unit(b), -1.0, 1.0))
        return math.acos(d)

    # ------------------------------------------------------------- arcs
    def push_arc(self, mv: ArcMove) -> None:
        feed = self.feed_of(mv.feed)
        start = mv.start + self.offset
        end = mv.end + self.offset
        center = mv.center + self.offset
        fit = arc_from_center(start, end, center, mv.normal, mv.ccw)
        if fit.radius < self.o.min_radius:
            self.push_linear(start, end, feed)
            return
        if mv.helical:
            rise = end - start
            rise_n = float(rise @ fit.normal)
            planar_end = end - rise_n * fit.normal
            fit = arc_from_center(start, planar_end, center, mv.normal, mv.ccw)
            samples = sample_arc(fit, self.o.tol / 2.0, min_points=3)
            k = len(samples) - 1
            for idx in range(1, len(samples)):
                p = samples[idx] + fit.normal * (rise_n * idx / k)
                self.push_linear(samples[idx - 1] + fit.normal * (rise_n * (idx - 1) / k), p, feed)
            return
        self.flush()
        abc = None
        if mv.abc is not None:
            abc = tuple(float(v) for v in mv.abc)
        elif mv.axis is not None:
            from .robot import abc_from_axis
            abc = abc_from_axis(mv.axis)
        for item in self.arc_items(fit, start, end, feed):
            item.abc = abc
            self.add_motion(item)
        self.last_ori = ("abc", mv.abc) if mv.abc is not None else (("axis", mv.axis) if mv.axis is not None else None)

    # -------------------------------------------------------------- run
    def translate(self, parsed: ParseResult) -> tuple[list, Stats]:
        self.stats.input_counts = dict(parsed.counts)
        self.stats.warnings.extend(parsed.warnings)
        for cmd in parsed.commands:
            if isinstance(cmd, LinearMove):
                ori = ("abc", cmd.abc) if cmd.abc is not None else (("axis", cmd.axis) if cmd.axis is not None else None)
                ori0 = ("abc", cmd.abc0) if cmd.abc0 is not None else (("axis", cmd.axis0) if cmd.axis0 is not None else None)
                if self.o.extruder:
                    self.set_extruding(cmd.extrude > 1e-9 and not cmd.rapid, cmd.extrude, cmd.start, cmd.end)
                if cmd.rapid:
                    self.flush()
                    rabc = cmd.abc
                    if rabc is None and cmd.axis is not None:
                        from .robot import abc_from_axis
                        rabc = abc_from_axis(cmd.axis)
                    self.items.append(RapidItem(cmd.end + self.offset, cmd.start + self.offset,
                                                abc=rabc, axis=cmd.axis))
                    self.last_ori = ori
                    self.stats.rapids += 1
                else:
                    self.push_linear(cmd.start + self.offset, cmd.end + self.offset, cmd.feed, ori, ori0)
            elif isinstance(cmd, ArcMove):
                if self.o.extruder:
                    self.set_extruding(cmd.extrude > 1e-9, cmd.extrude, cmd.start, cmd.end)
                self.push_arc(cmd)
            elif isinstance(cmd, ExtrudeOnly):
                if self.o.extruder:
                    self.flush()
                    self.items.append(EventItem("extruder_retract" if cmd.delta < 0 else "extruder_prime", abs(cmd.delta)))
            elif isinstance(cmd, Comment):
                # comments never break a chain (CAM output is full of them);
                # they are emitted right after the chain they interrupt
                if self.chain:
                    self.pending_comments.append(self.event(cmd))
                else:
                    self.items.append(self.event(cmd))
            else:
                self.flush()
                self.items.append(self.event(cmd))
        self.flush()
        if self.o.extruder and self.extruding:
            off = EventItem("extruder_off")
            k = next((i for i, it in enumerate(self.items) if isinstance(it, EventItem) and it.kind == "end"), None)
            if k is None:
                self.items.append(off)
            else:
                self.items.insert(k, off)
            self.extruding = False
        if not any(isinstance(i, EventItem) and i.kind == "end" for i in self.items):
            self.items.append(EventItem("end"))
        if "no-feed" in self.stats.warnings:
            self.stats.warnings.remove("no-feed")
            self.stats.warnings.append(f"feed never set in G-code; default {self.o.default_feed:g} mm/min used")
        return self.items, self.stats

    def set_extruding(self, on: bool, de: float = 0.0, start=None, end=None) -> None:
        if on and self.o.extruder_analog and start is not None:
            length = float(np.linalg.norm(np.asarray(end) - np.asarray(start)))
            if length > 1e-9:
                rate = round(de / length, 5)  # E per mm of path
                if self.rate is None or abs(rate - self.rate) > max(1e-6, 0.02 * abs(self.rate)):
                    self.flush()
                    self.items.append(EventItem("extruder_rate", rate))
                    self.rate = rate
        if on == self.extruding:
            return
        self.flush()
        self.items.append(EventItem("extruder_on" if on else "extruder_off"))
        self.extruding = on

    @staticmethod
    def event(cmd) -> EventItem:
        if isinstance(cmd, Dwell):
            return EventItem("dwell", cmd.seconds)
        if isinstance(cmd, Spindle):
            return EventItem("spindle_on" if cmd.on else "spindle_off", (cmd.rpm, cmd.ccw))
        if isinstance(cmd, SpindleSpeed):
            return EventItem("spindle_speed", cmd.rpm)
        if isinstance(cmd, Coolant):
            return EventItem("coolant_on" if cmd.on else "coolant_off")
        if isinstance(cmd, ToolChange):
            return EventItem("tool_change", cmd.tool)
        if isinstance(cmd, Pause):
            return EventItem("pause", cmd.optional)
        if isinstance(cmd, ProgramEnd):
            return EventItem("end")
        if isinstance(cmd, Comment):
            return EventItem("comment", cmd.text)
        raise TypeError(f"unknown command {cmd!r}")


def translate(parsed: ParseResult, opts: TranslateOptions | None = None) -> tuple[list, Stats]:
    return Translator(opts or TranslateOptions()).translate(parsed)


def sample_items(items: list, arc_points: int = 8, max_samples: int = 3000, step: float | None = None,
                 default_abc=(0.0, 180.0, 0.0), with_abc: bool = False):
    """Representative TCP positions along the output path (for reach / collision checks).

    With ``step`` every line, arc and SPL segment is subdivided so that
    consecutive samples are at most ``step`` mm apart (rapids included).
    With ``with_abc`` also returns an (N, 3) array of orientations (linear
    interpolation of A B C along SPL runs with varying orientation).
    """
    from .geometry import sample_arc

    pts: list[np.ndarray] = []
    abcs: list[np.ndarray] = []
    d_abc = np.array(default_abc, dtype=float)

    from .robot import _slerp_rot, abc_to_rot, rot_to_abc

    def seg(a: np.ndarray, b: np.ndarray, oa=None, ob=None) -> None:
        oa = d_abc if oa is None else np.asarray(oa, float)
        ob = d_abc if ob is None else np.asarray(ob, float)
        if step is None or step <= 0:
            pts.append(b)
            abcs.append(ob)
            return
        n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step)))
        same = np.allclose(oa, ob)
        if not same:
            ra, rb = abc_to_rot(*oa), abc_to_rot(*ob)
        for k in range(1, n + 1):
            t = k / n
            pts.append(a + (b - a) * t)
            if same:
                abcs.append(ob)
            else:
                # orientation interpolated as a rotation (like the controller), not as Euler angles
                abcs.append(np.array(rot_to_abc(_slerp_rot(ra, rb, t))) if k < n else ob)

    last_abc = None
    for it in items:
        if isinstance(it, RapidItem):
            o = it.abc if it.abc is not None else last_abc
            seg(it.start, it.target, last_abc, o)
            last_abc = o
        elif isinstance(it, LineItem):
            o = it.abc if it.abc is not None else last_abc
            seg(it.start, it.end, last_abc, o)
            last_abc = o
        elif isinstance(it, ArcItem):
            o = it.abc if it.abc is not None else last_abc
            n = arc_points if (step is None or step <= 0) else max(arc_points, int(math.ceil(it.fit.radius * it.fit.sweep / step)) + 1)
            arc_pts = sample_arc(it.fit, 1e9, min_points=n)[1:]
            pts.extend(arc_pts)
            abcs.extend([d_abc if o is None else np.asarray(o, float)] * len(arc_pts))
            last_abc = o
        elif isinstance(it, SplItem):
            for k in range(len(it.points) - 1):
                oa = it.abcs[k] if it.abcs is not None else last_abc
                ob = it.abcs[k + 1] if it.abcs is not None else last_abc
                seg(it.points[k], it.points[k + 1], oa, ob)
            last_abc = it.abcs[-1] if it.abcs is not None else last_abc
    if not pts:
        return (np.zeros((0, 3)), np.zeros((0, 3))) if with_abc else np.zeros((0, 3))
    arr = np.array(pts)
    arr_abc = np.array(abcs)
    if len(arr) > max_samples:
        idx = np.unique(np.linspace(0, len(arr) - 1, max_samples).astype(int))
        arr = arr[idx]
        arr_abc = arr_abc[idx]
    return (arr, arr_abc) if with_abc else arr


def resolve_orientations(items: list, robot, base: np.ndarray, tool: np.ndarray, ref=(0.0, 180.0, 0.0),
                         spin_step: float = 15.0, singular_deg: float = 8.0, max_tilt_deg: float = 0.0) -> dict:
    """Replace zero-spin orientations of axis-defined SPL runs by singularity-aware ones."""
    from .robot import solve_spins

    info = {"resampled": 0, "unreachable": 0, "singular": 0, "points": 0}
    start = None
    for it in items:
        if isinstance(it, SplItem) and it.axes is not None:
            abcs, inf = solve_spins(robot, it.points, it.axes, base, tool, ref=ref, spin_step=spin_step,
                                    singular_deg=singular_deg, max_tilt_deg=max_tilt_deg, start=start)
            it.abcs = abcs
            for k in ("resampled", "unreachable", "singular"):
                info[k] += inf[k]
            info["points"] += len(it.points)
    return info


def has_orientation(items: list) -> bool:
    return any((isinstance(i, (LineItem, ArcItem, RapidItem)) and i.abc is not None) or (isinstance(i, SplItem) and i.abcs is not None)
               for i in items)
