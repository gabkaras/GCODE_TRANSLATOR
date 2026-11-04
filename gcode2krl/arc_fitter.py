"""Segmentation of a polyline into straight segments and circular arcs.

Strategy (see README for the rationale and literature):
  1. Greedy left-to-right extension: from the current vertex extend a line
     as far as the tolerance allows and, independently, an arc (refitting
     the circle after every added vertex, end points fixed). Take the arc
     only if it explains more vertices than the line and has at least
     ``min_arc_points`` vertices; otherwise take the line.
  2. Break-point refinement: for every pair of neighbouring elements the
     common break point is shifted by up to ``refine_shift`` vertices in
     both directions and the combination with the lowest deviation is kept.
     This repairs the "stolen points" at tangent line/arc and arc/arc joints.
  3. Merge pass: neighbouring arcs that can be explained by a single circle
     within tolerance are merged, so one big arc is never reported as a
     chain of small ones.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np

from .geometry import ArcFit, fit_arc_3d, point_segment_distances, unit


PARALLEL_MIN_POINTS = 4000


@dataclass
class FitParams:
    tol: float = 0.05
    max_radius: float = 5000.0
    min_radius: float = 0.1
    min_arc_points: int = 4
    refine_shift: int = 3
    merge_arcs: bool = True


@dataclass
class LineElement:
    i: int
    j: int
    start: np.ndarray
    end: np.ndarray
    max_dev: float

    @property
    def n_points(self) -> int:
        return self.j - self.i + 1

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end - self.start))

    @property
    def t_in(self) -> np.ndarray:
        return unit(self.end - self.start)

    t_out = t_in


@dataclass
class ArcElement:
    i: int
    j: int
    start: np.ndarray
    end: np.ndarray
    fit: ArcFit

    @property
    def n_points(self) -> int:
        return self.j - self.i + 1

    @property
    def max_dev(self) -> float:
        return self.fit.max_dev

    @property
    def mid(self) -> np.ndarray:
        return self.fit.mid

    @property
    def radius(self) -> float:
        return self.fit.radius

    @property
    def sweep(self) -> float:
        return self.fit.sweep

    @property
    def t_in(self) -> np.ndarray:
        return self.fit.t_in

    @property
    def t_out(self) -> np.ndarray:
        return self.fit.t_out


Element = LineElement | ArcElement


def dedupe_points(points: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    if len(points) < 2:
        return points
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.all(d > eps):
        return points
    keep = [0]
    for k in range(1, len(points)):
        if float(np.linalg.norm(points[k] - points[keep[-1]])) > eps:
            keep.append(k)
    return points[keep]


class PolylineSegmenter:
    def __init__(self, points: np.ndarray, params: FitParams):
        self.p = points
        self.n = len(points)
        self.prm = params

    # ---------------------------------------------------------- fitting
    def line_dev(self, i: int, j: int) -> float:
        if j - i < 2:
            return 0.0
        return float(point_segment_distances(self.p[i + 1 : j], self.p[i], self.p[j]).max())

    def fit_line(self, i: int, j: int) -> LineElement | None:
        dev = self.line_dev(i, j)
        if dev > self.prm.tol:
            return None
        return LineElement(i, j, self.p[i], self.p[j], dev)

    def fit_arc(self, i: int, j: int) -> ArcElement | None:
        if j - i + 1 < self.prm.min_arc_points:
            return None
        fit = fit_arc_3d(self.p[i : j + 1], self.prm.tol, self.prm.max_radius, self.prm.min_radius)
        if fit is None:
            return None
        return ArcElement(i, j, self.p[i], self.p[j], fit)

    def fit_same_kind(self, kind: type, i: int, j: int) -> Element | None:
        return self.fit_line(i, j) if kind is LineElement else self.fit_arc(i, j)

    # -------------------------------------------------------- extension
    def extend_line(self, i: int) -> int:
        j = i + 1
        while j + 1 < self.n and self.line_dev(i, j + 1) <= self.prm.tol:
            j += 1
        return j

    def extend_arc(self, i: int) -> ArcElement | None:
        """Longest arc starting at vertex i (galloping search + binary search)."""
        first = i + max(2, self.prm.min_arc_points - 1)
        if first >= self.n:
            return None
        best: ArcElement | None = None
        j = first
        tries = 0
        while j < self.n:  # find the first valid arc
            arc = self.fit_arc(i, j)
            if arc is not None:
                best = arc
                break
            tries += 1
            if tries > 2:
                return None
            j += 1
        if best is None:
            return None
        # gallop forward
        step = 1
        lo = best.j
        hi: int | None = None
        while True:
            j = lo + step
            if j >= self.n:
                j = self.n - 1
                if j == lo:
                    break
            arc = self.fit_arc(i, j)
            if arc is not None:
                best = arc
                lo = j
                if j == self.n - 1:
                    break
                step *= 2
            else:
                hi = j
                break
        # binary search between lo (valid) and hi (invalid)
        while hi is not None and hi - lo > 1:
            mid = (lo + hi) // 2
            arc = self.fit_arc(i, mid)
            if arc is not None:
                best = arc
                lo = mid
            else:
                hi = mid
        # the predicate is only nearly monotone: creep forward a little more
        j = lo + 1
        while j < self.n and j <= lo + 2:
            arc = self.fit_arc(i, j)
            if arc is None:
                break
            best = arc
            lo = j
            j += 1
        return best

    # ------------------------------------------------------------- main
    def segment(self) -> list[Element]:
        if self.n < 2:
            return []
        elems: list[Element] = []
        i = 0
        while i < self.n - 1:
            jl = self.extend_line(i)
            arc = self.extend_arc(i)
            if arc is not None and arc.j > jl:
                elems.append(arc)
                i = arc.j
            else:
                elems.append(LineElement(i, jl, self.p[i], self.p[jl], self.line_dev(i, jl)))
                i = jl
        if self.prm.refine_shift > 0:
            elems = self.refine_breakpoints(elems)
        if self.prm.merge_arcs:
            elems = self.merge_arcs(elems)
        return elems

    # ---------------------------------------------------------- passes
    def refine_breakpoints(self, elems: list[Element]) -> list[Element]:
        changed = True
        rounds = 0
        while changed and rounds < 3:
            changed = False
            rounds += 1
            for k in range(len(elems) - 1):
                a, b = elems[k], elems[k + 1]
                best_pair = (a, b)
                best_score = max(a.max_dev, b.max_dev)
                if best_score < 0.2 * self.prm.tol:
                    continue  # nothing worth repairing
                for s in range(-self.prm.refine_shift, self.prm.refine_shift + 1):
                    if s == 0:
                        continue
                    m = a.j + s
                    if m <= a.i or m >= b.j:
                        continue
                    na = self.fit_same_kind(type(a), a.i, m)
                    nb = self.fit_same_kind(type(b), m, b.j)
                    if na is None or nb is None:
                        continue
                    score = max(na.max_dev, nb.max_dev)
                    if score < best_score - 1e-12:
                        best_score = score
                        best_pair = (na, nb)
                if best_pair[0] is not a:
                    elems[k], elems[k + 1] = best_pair
                    changed = True
        return elems

    def merge_arcs(self, elems: list[Element]) -> list[Element]:
        out: list[Element] = []
        for e in elems:
            if out and isinstance(e, ArcElement) and isinstance(out[-1], ArcElement):
                merged = self.fit_arc(out[-1].i, e.j)
                if merged is not None:
                    out[-1] = merged
                    continue
            out.append(e)
        return out


def split_at_hard_corners(points: np.ndarray, angle_deg: float = 60.0) -> list[int]:
    """Indices where the polyline turns by more than ``angle_deg`` (chunk borders)."""
    if len(points) < 3:
        return []
    d = np.diff(points, axis=0)
    n = np.linalg.norm(d, axis=1)
    n = np.where(n < 1e-12, 1e-12, n)
    d = d / n[:, None]
    cos = np.einsum("ij,ij->i", d[:-1], d[1:])
    return [int(k) + 1 for k in np.flatnonzero(cos < math.cos(math.radians(angle_deg)))]


def _segment_chunk(args):
    pts, params, offset = args
    elems = PolylineSegmenter(pts, params).segment()
    for e in elems:
        e.i += offset
        e.j += offset
    return elems


def segment_polyline(points: np.ndarray, params: FitParams | None = None, workers: int | None = None) -> list[Element]:
    """Split an ordered chain of 3-D points into LineElement / ArcElement.

    Long chains are cut at hard corners and the pieces are fitted in a
    process pool (``workers`` processes, default: CPU count) when the chain
    has more than ``PARALLEL_MIN_POINTS`` vertices.
    """
    params = params or FitParams()
    pts = dedupe_points(np.asarray(points, dtype=float))
    n = len(pts)
    if n < 2:
        return []
    cuts = split_at_hard_corners(pts) if n > PARALLEL_MIN_POINTS else []
    if workers is None:
        workers = os.cpu_count() or 1
    if not cuts or workers <= 1 or n <= PARALLEL_MIN_POINTS:
        return PolylineSegmenter(pts, params).segment()
    bounds = [0] + cuts + [n - 1]
    jobs = []
    for a, b in zip(bounds, bounds[1:]):
        if b > a:
            jobs.append((pts[a : b + 1], params, a))
    if len(jobs) < 2:
        return PolylineSegmenter(pts, params).segment()
    from concurrent.futures import ProcessPoolExecutor

    out: list[Element] = []
    try:
        with ProcessPoolExecutor(max_workers=min(workers, len(jobs))) as ex:
            for elems in ex.map(_segment_chunk, jobs, chunksize=max(1, len(jobs) // (4 * workers))):
                out.extend(elems)
    except (OSError, RuntimeError, PermissionError):  # no fork/spawn available
        for job in jobs:
            out.extend(_segment_chunk(job))
    return out


def max_deviation(points: np.ndarray, elems: list[Element], samples_per_arc: int = 200) -> float:
    """Hausdorff-style check of a segmentation against the original polyline."""
    pts = dedupe_points(np.asarray(points, dtype=float))
    worst = 0.0
    for e in elems:
        chain = pts[e.i : e.j + 1]
        if isinstance(e, LineElement):
            worst = max(worst, float(point_segment_distances(chain, e.start, e.end).max()))
            continue
        thetas = e.fit.theta0 + e.fit.sweep * np.linspace(0.0, 1.0, samples_per_arc)
        arc_pts = np.array([e.fit.point_at(float(t)) for t in thetas])
        # distance arc -> polyline
        dmin = np.full(len(arc_pts), np.inf)
        for k in range(len(chain) - 1):
            dmin = np.minimum(dmin, point_segment_distances(arc_pts, chain[k], chain[k + 1]))
        worst = max(worst, float(dmin.max()))
        # distance polyline vertices -> circle (radial + planar)
        q = chain - e.fit.center
        w = q @ e.fit.normal
        radial = np.abs(np.linalg.norm(q - np.outer(w, e.fit.normal), axis=1) - e.fit.radius)
        worst = max(worst, float(np.sqrt(radial * radial + w * w).max()))
    return worst
