import math

import numpy as np

from gcode2krl.arc_fitter import ArcElement, FitParams, LineElement, max_deviation, segment_polyline


def rnd(pts, nd=3):
    return np.round(pts, nd)


def fillet_rect():
    pts = [(10, 0)]
    def arc(cx, cy, a0, a1, r=10, step=2):
        n = int(round((a1 - a0) / step))
        for k in range(1, n + 1):
            a = math.radians(a0 + (a1 - a0) * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    pts.append((90, 0)); arc(90, 10, -90, 0)
    pts.append((100, 50)); arc(90, 50, 0, 90)
    pts.append((10, 60)); arc(10, 50, 90, 180)
    pts.append((0, 10)); arc(10, 10, 180, 270)
    return rnd(np.array([(x, y, -1.0) for x, y in pts]))


def test_fillet_rectangle_gives_four_lines_four_arcs():
    pts = fillet_rect()
    elems = segment_polyline(pts, FitParams(tol=0.05))
    kinds = ["A" if isinstance(e, ArcElement) else "L" for e in elems]
    assert kinds == ["L", "A", "L", "A", "L", "A", "L", "A"]
    for e in elems:
        if isinstance(e, ArcElement):
            assert abs(e.radius - 10) < 0.01
            assert abs(math.degrees(e.sweep) - 90) < 0.1
    # elements chain exactly through original vertices
    for a, b in zip(elems, elems[1:]):
        assert np.allclose(a.end, b.start)
    assert np.allclose(elems[0].start, pts[0]) and np.allclose(elems[-1].end, pts[-1])
    assert max_deviation(pts, elems) <= 0.05


def test_full_circle_single_closed_arc():
    a = np.linspace(0, 2 * math.pi, 73)
    pts = rnd(np.column_stack([50 + 25 * np.cos(a), 50 + 25 * np.sin(a), np.zeros_like(a)]))
    elems = segment_polyline(pts, FitParams(tol=0.05))
    assert len(elems) == 1 and isinstance(elems[0], ArcElement)
    assert elems[0].fit.closed and abs(elems[0].radius - 25) < 0.01


def test_one_big_arc_not_many_small_ones():
    # 180 degrees, R=200, 1 degree steps, coordinates rounded to 1 um
    a = np.linspace(0, math.pi, 181)
    pts = rnd(np.column_stack([200 * np.cos(a), 200 * np.sin(a), np.full_like(a, 3.0)]))
    elems = segment_polyline(pts, FitParams(tol=0.02))
    assert len(elems) == 1
    assert isinstance(elems[0], ArcElement) and abs(elems[0].radius - 200) < 0.05


def test_tangent_arcs_of_different_radius_are_separated():
    # R=10 quarter followed tangentially by R=30 quarter
    a1 = np.linspace(-math.pi / 2, 0, 46)
    p1 = np.column_stack([10 * np.cos(a1), 10 + 10 * np.sin(a1)])
    a2 = np.linspace(0, math.pi / 2, 91)
    p2 = np.column_stack([-20 + 30 * np.cos(a2), 10 + 30 * np.sin(a2)])
    pts = rnd(np.column_stack([np.vstack([p1, p2[1:]]), np.zeros(len(p1) + len(p2) - 1)]))
    elems = segment_polyline(pts, FitParams(tol=0.01))
    arcs = [e for e in elems if isinstance(e, ArcElement)]
    assert len(elems) == 2 and len(arcs) == 2
    assert abs(arcs[0].radius - 10) < 0.05 and abs(arcs[1].radius - 30) < 0.05
    # break point exactly at the tangency vertex (index 45)
    assert arcs[0].j == 45


def test_straight_polyline_is_one_line():
    pts = np.column_stack([np.linspace(0, 100, 51), np.zeros(51), np.zeros(51)])
    elems = segment_polyline(pts, FitParams(tol=0.01))
    assert len(elems) == 1 and isinstance(elems[0], LineElement)


def test_noisy_line_with_wobble_is_not_an_arc():
    rng = np.random.default_rng(3)
    x = np.linspace(0, 50, 26)
    pts = np.column_stack([x, rng.uniform(-0.004, 0.004, 26), np.zeros(26)])
    elems = segment_polyline(pts, FitParams(tol=0.01, max_radius=5000))
    assert all(isinstance(e, LineElement) for e in elems)


def test_spiral_gives_valid_segments_within_tolerance():
    th = np.radians(np.arange(0, 720, 3))
    r = 5 + 0.3 * th
    pts = rnd(np.column_stack([r * np.cos(th), r * np.sin(th), np.zeros_like(th)]))
    tol = 0.05
    elems = segment_polyline(pts, FitParams(tol=tol))
    assert len(elems) < len(pts) / 4
    assert max_deviation(pts, elems) <= tol + 1e-9
    assert any(isinstance(e, ArcElement) for e in elems)


def test_sharp_corner_between_arc_and_line():
    a = np.linspace(0, math.pi / 2, 30)
    arc = np.column_stack([10 * np.cos(a), 10 * np.sin(a), np.zeros(30)])
    line = np.array([[0, 10, 0], [0, 30, 0], [20, 30, 0]], dtype=float)
    pts = np.vstack([arc, line[1:]])
    elems = segment_polyline(pts, FitParams(tol=0.02))
    assert [type(e).__name__ for e in elems] == ["ArcElement", "LineElement", "LineElement"]


def test_helix_is_not_a_planar_arc():
    a = np.linspace(0, math.pi, 40)
    pts = np.column_stack([20 * np.cos(a), 20 * np.sin(a), np.linspace(0, 10, 40)])
    elems = segment_polyline(pts, FitParams(tol=0.02))
    assert not any(isinstance(e, ArcElement) and e.n_points == 40 for e in elems)
    assert max_deviation(pts, elems) <= 0.02 + 1e-9
