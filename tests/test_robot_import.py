import json
import math
import os

import numpy as np

from gcode2krl.meshio import load_stl, save_stl
from gcode2krl.robot import model_from_dict, forward
from gcode2krl.robot_import import (build_definition, chain_from_xacro, fit_capsules, fit_obb, kmeans, parse_xacro,
                                    subdivide, surface_outside)

XACRO = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:macro name="t" params="prefix">
    <link name="${prefix}base_link"><collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="base_link.stl"/></geometry></collision></link>
    <link name="${prefix}link_1"><collision><geometry><mesh filename="link_1.stl"/></geometry></collision></link>
    <link name="${prefix}link_2"/><link name="${prefix}link_3"/><link name="${prefix}link_4"/><link name="${prefix}link_5"/><link name="${prefix}link_6"/><link name="${prefix}flange"/>
    <joint name="${prefix}joint_a1" type="revolute"><origin xyz="0 0 0.5" rpy="0 0 0"/><parent link="${prefix}base_link"/><child link="${prefix}link_1"/><axis xyz="0 0 -1"/><limit effort="0" lower="${-radians(185)}" upper="${radians(185)}" velocity="${radians(100)}"/></joint>
    <joint name="${prefix}joint_a2" type="revolute"><origin xyz="0.2 0 0" rpy="0 0 0"/><parent link="${prefix}link_1"/><child link="${prefix}link_2"/><axis xyz="0 1 0"/><limit effort="0" lower="${radians(-140)}" upper="${radians(-5)}" velocity="${radians(90)}"/></joint>
    <joint name="${prefix}joint_a3" type="revolute"><origin xyz="1.0 0 0" rpy="0 0 0"/><parent link="${prefix}link_2"/><child link="${prefix}link_3"/><axis xyz="0 1 0"/><limit effort="0" lower="${radians(-120)}" upper="${radians(155)}" velocity="${radians(90)}"/></joint>
    <joint name="${prefix}joint_a4" type="revolute"><origin xyz="0.9 0 -0.04" rpy="0 0 0"/><parent link="${prefix}link_3"/><child link="${prefix}link_4"/><axis xyz="-1 0 0"/><limit effort="0" lower="${radians(-350)}" upper="${radians(350)}" velocity="${radians(200)}"/></joint>
    <joint name="${prefix}joint_a5" type="revolute"><origin xyz="0 0 0" rpy="0 0 0"/><parent link="${prefix}link_4"/><child link="${prefix}link_5"/><axis xyz="0 1 0"/><limit effort="0" lower="${radians(-125)}" upper="${radians(125)}" velocity="${radians(200)}"/></joint>
    <joint name="${prefix}joint_a6" type="revolute"><origin xyz="0 0 0" rpy="0 0 0"/><parent link="${prefix}link_5"/><child link="${prefix}link_6"/><axis xyz="-1 0 0"/><limit effort="0" lower="${radians(-350)}" upper="${radians(350)}" velocity="${radians(300)}"/></joint>
    <joint name="${prefix}joint_a6-flange" type="fixed"><parent link="${prefix}link_6"/><child link="${prefix}flange"/><origin xyz="0.2 0 0" rpy="0 0 0"/></joint>
  </xacro:macro>
</robot>
"""


def cube(size, center):
    s = size / 2.0
    c = np.array(center)
    v = np.array([[x, y, z] for x in (-s, s) for y in (-s, s) for z in (-s, s)]) + c
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c2, d in faces:
        tris.append([v[a], v[b], v[c2]])
        tris.append([v[a], v[c2], v[d]])
    return np.array(tris)


def test_stl_round_trip(tmp_path):
    tris = cube(0.1, (0, 0, 0.05))
    save_stl(str(tmp_path / "c.stl"), tris)
    back = load_stl(str(tmp_path / "c.stl"))
    assert np.allclose(back, tris, atol=1e-6)


def test_subdivide_and_capsules_enclose_surface():
    tris = cube(100.0, (0, 0, 0))
    dense = subdivide(tris, 16.0)
    e = np.linalg.norm(dense[:, 1] - dense[:, 0], axis=1)
    assert e.max() <= 16.0 + 1e-9 and len(dense) > len(tris)
    caps = fit_capsules(tris)
    assert surface_outside(tris, caps) <= 1e-9
    box = fit_obb(tris)
    assert np.allclose(sorted(box["half"]), [51, 51, 51], atol=1e-6)


def test_kmeans_separates_two_blobs():
    pts = np.vstack([np.random.default_rng(0).normal(0, 1, (50, 3)), np.random.default_rng(1).normal(100, 1, (50, 3))])
    labels = kmeans(pts, 2)
    assert len(set(labels[:50])) == 1 and len(set(labels[50:])) == 1 and labels[0] != labels[-1]


def test_import_xacro_definition(tmp_path):
    xp = tmp_path / "t_macro.xacro"
    xp.write_text(XACRO)
    save_stl(str(tmp_path / "base_link.stl"), cube(0.4, (0, 0, 0.2)))
    save_stl(str(tmp_path / "link_1.stl"), cube(0.3, (0.1, 0, -0.2)))
    chain = chain_from_xacro(parse_xacro(str(xp)))
    assert chain["origins"][0] == [0, 0, 500] and chain["origins"][3] == [900, 0, -40]
    assert chain["limits"][1] == [-140, -5] and chain["speeds"][0] == 100
    d = build_definition(str(xp), "TEST", "test robot", out_dir=str(tmp_path / "out"))
    assert d["capsules"][0]["type"] == "box" and d["capsules"][0]["link"] == 0
    assert any(c["link"] == 1 for c in d["capsules"])
    assert os.path.isfile(os.path.join(tmp_path / "out", d["meshes"]["1"]))
    m = model_from_dict(d, str(tmp_path / "out"))
    assert (m.d1, m.a1, m.a2, m.a3, m.d4, m.d6) == (500, 200, 1000, -40, 900, 200)
    fl = forward(m, m.home)[-1]
    assert np.allclose(fl[:3, 3], [200 + 900 + 200, 0, 500 + 1000 - 40])
