import math
import random

import numpy as np
import pytest

from gcode2krl.robot import (DEFAULT_ROBOT, ROBOTS, abc_to_rot, check_path, forward, frame, inv_frame, inverse,
                             inverse_all, rot_to_abc)


def test_home_pose_matches_kuka_quantec():
    r = ROBOTS["KR90R2700PRO"]
    fl = forward(r, r.home)[-1]
    assert np.allclose(fl[:3, 3], [1765, 0, 1784])
    a, b, c = rot_to_abc(fl[:3, :3])
    assert abs(a) < 1e-9 and abs(b - 90) < 1e-9 and abs(c) < 1e-9


def test_a2_a3_conventions():
    r = ROBOTS["KR90R2700PRO"]
    # A2 = 0: upper arm horizontal forward -> A3 axis at (350+1150, 0, 675)
    assert np.allclose(forward(r, (0, 0, 0, 0, 0, 0))[2][:3, 3], [1500, 0, 675])
    # A2 = -90: vertical
    assert np.allclose(forward(r, (0, -90, 0, 0, 0, 0))[2][:3, 3], [350, 0, 1825])
    # A1 positive turns clockwise seen from above (towards -Y)
    assert forward(r, (30, -90, 90, 0, 0, 0))[-1][1, 3] < 0


@pytest.mark.parametrize("key", list(ROBOTS))
def test_ik_round_trip(key):
    r = ROBOTS[key]
    rng = random.Random(7)
    for _ in range(150):
        q = [rng.uniform(*lim) for lim in r.limits]
        q[3] = rng.uniform(-170, 170)
        q[5] = rng.uniform(-170, 170)
        fl = forward(r, q)[-1]
        sols = inverse_all(r, fl)
        assert any(np.allclose(forward(r, s)[-1], fl, atol=1e-6) for s in sols)
        best, viol = inverse(r, fl, q)
        assert best is not None and not viol
        assert np.allclose(forward(r, best)[-1], fl, atol=1e-6)
        assert max(abs(b - a) for a, b in zip(q, best)) < 1e-6  # closest solution is the original


def test_ik_unreachable():
    r = ROBOTS[DEFAULT_ROBOT]
    far = frame((5000, 0, 1000, 0, 90, 0))
    assert inverse_all(r, far) == []
    assert inverse(r, far)[0] is None


def test_abc_round_trip():
    for abc in [(10, 20, 30), (-170, 85, 5), (0, 180, 0), (45, -90, 0)]:
        r = abc_to_rot(*abc)
        a, b, c = rot_to_abc(r)
        assert np.allclose(abc_to_rot(a, b, c), r, atol=1e-9)


def test_frame_inverse():
    t = frame((100, -50, 20, 30, -15, 60))
    assert np.allclose(t @ inv_frame(t), np.eye(4), atol=1e-12)


def test_check_path_reach_and_limits():
    r = ROBOTS[DEFAULT_ROBOT]
    # workpiece base 1200 mm in front of the robot, on a 500 mm table; tool pointing down (B 180 -> tool Z = -Z)
    base = frame((1200, 0, 500, 0, 0, 0))
    tool = frame((0, 0, 150, 0, 0, 0))
    pts = np.array([[x, y, 0.0] for x in range(0, 401, 100) for y in range(-200, 201, 100)])
    rep = check_path(r, pts, (0, 180, 0), base, tool)
    assert rep.n_samples == len(pts) and rep.n_unreachable == 0 and rep.n_limit == 0
    assert rep.joints is not None and np.isfinite(rep.joints).all()
    # the flange really is 150 mm above each point with Z pointing down
    for p, q in zip(pts, rep.joints):
        fl = forward(r, q)[-1]
        tcp = fl @ tool
        assert np.allclose(tcp[:3, 3], (base @ np.append(p, 1))[:3], atol=1e-6)
        assert np.allclose(tcp[:3, 2], [0, 0, -1], atol=1e-9)
    # far away: unreachable
    rep2 = check_path(r, np.array([[3000.0, 0, 0]]), (0, 180, 0), base, tool)
    assert rep2.n_unreachable == 1
