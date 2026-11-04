import math

import numpy as np
import pytest

from gcode2krl.collision import (OBSTACLE_HELP, check_collisions, obstacle_from_dict, parse_obstacles, robot_capsules,
                                 seg_convex_distance, seg_seg_distance)
from gcode2krl.robot import ROBOTS, check_path, check_path_collisions, frame
from gcode2krl.translator import sample_items


def A(*v):
    return np.array([v], dtype=float)


def test_seg_seg_distance_cases():
    assert seg_seg_distance(A(0, 0, 0), A(10, 0, 0), A(5, 3, 0), A(5, 3, 10))[0] == pytest.approx(3)
    assert seg_seg_distance(A(0, 0, 0), A(10, 0, 0), A(20, 0, 0), A(30, 0, 0))[0] == pytest.approx(10)
    assert seg_seg_distance(A(0, 0, 0), A(10, 0, 0), A(5, -1, 1), A(5, 1, 1))[0] == pytest.approx(1)
    assert seg_seg_distance(A(0, 0, 0), A(0, 0, 0), A(3, 4, 0), A(3, 4, 0))[0] == pytest.approx(5)  # two points
    assert seg_seg_distance(A(0, 0, 0), A(1, 1, 0), A(1, 0, 0), A(0, 1, 0))[0] == pytest.approx(0)  # crossing


def test_seg_box_distance_exact():
    box = obstacle_from_dict({"kind": "box", "name": "b", "center": [0, 0, 0], "size": [10, 10, 10]}, np.eye(4))
    assert seg_convex_distance(A(20, -50, 0), A(20, 50, 0), box.distance)[0] == pytest.approx(15, abs=1e-6)
    assert seg_convex_distance(A(-50, 0, 0), A(50, 0, 0), box.distance)[0] == pytest.approx(-5, abs=1e-6)
    # corner approach
    assert seg_convex_distance(A(10, 10, 10), A(30, 30, 30), box.distance)[0] == pytest.approx(5 * math.sqrt(3), abs=1e-6)
    rot = obstacle_from_dict({"kind": "box", "name": "b", "center": [0, 0, 0], "size": [10, 10, 10], "abc": [45, 0, 0]}, np.eye(4))
    assert seg_convex_distance(A(20, 0, -5), A(20, 0, 5), rot.distance)[0] == pytest.approx(20 - 5 * math.sqrt(2), abs=1e-6)


def test_cylinder_sphere_floor():
    cyl = obstacle_from_dict({"kind": "cylinder", "name": "c", "center": [0, 0, 50], "radius": 10, "height": 100}, np.eye(4))
    assert cyl.distance(A(20, 0, 50))[0] == pytest.approx(10)
    assert cyl.distance(A(0, 0, 120))[0] == pytest.approx(20)
    assert cyl.distance(A(0, 0, 50))[0] < 0
    sph = obstacle_from_dict({"kind": "sphere", "name": "s", "center": [1, 2, 3], "radius": 5}, np.eye(4))
    assert sph.distance(A(1, 2, 13))[0] == pytest.approx(5)
    fl = obstacle_from_dict({"kind": "floor", "name": "f", "z": 100}, np.eye(4))
    assert fl.distance(A(0, 0, 130))[0] == pytest.approx(30)
    # obstacle given in BASE coordinates is moved into the robot frame
    base = frame((1000, 0, 500, 90, 0, 0))
    box = obstacle_from_dict({"kind": "box", "name": "b", "frame": "base", "center": [100, 0, 0], "size": [10, 10, 10]}, base)
    assert np.allclose(box.center, [1000, 100, 500])


def test_parse_obstacles_format_and_errors():
    obs = parse_obstacles(OBSTACLE_HELP)
    assert [o["kind"] for o in obs] == ["floor", "box", "box"]
    assert obs[2]["workpiece"] and obs[2]["frame"] == "base"
    with pytest.raises(ValueError, match="line 1"):
        parse_obstacles("box x robot 1 2 3")
    with pytest.raises(ValueError, match="unknown"):
        parse_obstacles("pyramid x robot 1 2 3 4")


def robot_setup():
    r = ROBOTS["KR90R2700PRO"]
    base = frame((1200, 0, 500, 0, 0, 0))
    tool = frame((0, 0, 150, 0, 0, 0))
    return r, base, tool


def test_free_path_has_no_collision_and_workpiece_is_allowed_for_cutter():
    r, base, tool = robot_setup()
    pts = np.array([[x, 0.0, -1.0] for x in range(0, 101, 5)])
    reach = check_path(r, pts, (0, 180, 0), base, tool)
    obstacles = [obstacle_from_dict(o, base) for o in parse_obstacles(OBSTACLE_HELP)]
    rep, caps = check_path_collisions(r, pts, (0, 180, 0), base, tool, reach, obstacles, margin=5.0)
    assert rep.n_collision == 0 and rep.unresolved == 0
    assert rep.min_clearance > 5.0
    names = {(p.a, p.b) for p in rep.pairs}
    assert ("ostrze", "material") not in names and ("narzedzie", "material") in names


def test_tool_holder_hits_workpiece_when_cutting_too_deep():
    r, base, tool = robot_setup()
    pts = np.array([[x, 0.0, -30.0] for x in range(0, 101, 5)])  # 30 mm deep, cutter only 20 mm long
    reach = check_path(r, pts, (0, 180, 0), base, tool)
    obstacles = [obstacle_from_dict(o, base) for o in parse_obstacles(OBSTACLE_HELP)]
    rep, _ = check_path_collisions(r, pts, (0, 180, 0), base, tool, reach, obstacles, margin=5.0)
    assert rep.n_collision == len(pts)
    colliding = {(p.a, p.b) for p in rep.pairs if p.min_clearance <= 0}
    assert ("narzedzie", "material") in colliding
    assert ("ostrze", "stol") in colliding  # 30 mm deep also goes through the table top


def test_obstacle_between_samples_is_found_by_refinement():
    r, base, tool = robot_setup()
    # two samples 200 mm apart, a thin wall in between (in BASE coords, x = 100, 4 mm thick, tall)
    pts = np.array([[0.0, 0, 50], [200.0, 0, 50]])
    reach = check_path(r, pts, (0, 180, 0), base, tool)
    wall = obstacle_from_dict({"kind": "box", "name": "wall", "frame": "base", "center": [100, 0, 100], "size": [4, 400, 200]}, base)
    rep, _ = check_path_collisions(r, pts, (0, 180, 0), base, tool, reach, [wall], margin=5.0)
    assert rep.refined_samples > 0
    assert rep.n_collision >= 1 or rep.unresolved >= 1
    assert rep.pairs[0].b == "wall" and rep.pairs[0].min_clearance <= 0
    # the same wall far away: provably clear without any refinement
    far = obstacle_from_dict({"kind": "box", "name": "wall", "frame": "base", "center": [100, 2000, 100], "size": [4, 400, 200]}, base)
    rep2, _ = check_path_collisions(r, pts, (0, 180, 0), base, tool, reach, [far], margin=5.0)
    assert rep2.n_collision == 0 and rep2.unresolved == 0


def test_self_collision_detected_in_folded_pose():
    r = ROBOTS["KR90R2700PRO"]
    caps = robot_capsules(r, frame((0, 0, 600, 0, 0, 0)), tool_radius=40)
    folded = np.array([[0, -140, 150, 0, 120, 0]], float)  # arm back, forearm folded, long tool bent down
    rep = check_collisions(r, folded, np.ones(1, bool), caps, [], margin=10)
    assert rep.n_collision >= 1 or rep.min_clearance < 10
    home = np.array([r.home], float)
    rep2 = check_collisions(r, home, np.ones(1, bool), caps, [], margin=10)
    assert rep2.n_collision == 0 and rep2.min_clearance > 10


def test_dense_sampling_step():
    from gcode2krl.gcode_parser import parse_gcode
    from gcode2krl.translator import translate
    items, _ = translate(parse_gcode("G1 X100 F500\nG2 X200 Y100 I0 J100\nM30\n"))
    pts = sample_items(items, step=2.0)
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert d.max() <= 2.0 + 1e-9 and len(pts) > 100
    assert np.allclose(pts[-1], [200, 100, 0])
