"""The scene file against layout.py, and the layout against the maker's numbers (NumPy and SciPy only)."""

import re
from importlib.resources import files

import numpy as np

from build_assets import usd as regenerate_usd
from rdb_filter_clamp import fixed_scene as F
from rdb_filter_clamp import layout as L

USD = F.USD.read_text(encoding="utf-8")


def prim(name):
    m = re.search(rf'def \w+ "{name}" \((.*?)\n {{4,12}}\}}\n', USD, re.S)
    assert m, name
    return m.group(0)


def test_asset_is_regenerated_from_layout():
    packaged_asset = files("rdb_filter_clamp").joinpath("filter_clamp.usda")
    assert F.USD.read_bytes() == packaged_asset.read_bytes()
    assert regenerate_usd() == packaged_asset.read_text(encoding="utf-8")


def test_bodies_and_masses():
    masses = dict(re.findall(r'def Xform "(\w+)" \(\n\s+prepend apiSchemas = \["PhysicsRigidBodyAPI".*?\n\s+\)\n\s+\{\n\s+float physics:mass = ([\d.e-]+)', USD, re.S))
    assert {k: round(float(v), 4) for k, v in masses.items()} == {
        "holder": round(L.HOLDER_MASS, 4), "funnel": L.FUNNEL_MASS, "knuckle": L.KNUCKLE_MASS, "jawL": L.JAW_MASS, "jawR": L.JAW_MASS}
    assert USD.count("PhysicsArticulationRootAPI") == 1 and "PhysicsFixedJoint" not in USD       # the clamp floats free


def test_clamp_hinges_are_sprung_shut_within_their_stops():
    joints = re.findall(r'PhysicsRevoluteJoint "(hinge[LR])".*?stiffness = ([\d.e-]+).*?targetPosition = ([-\d.e]+).*?'
                        r'lowerLimit = ([-\d.e]+)\n.*?upperLimit = ([-\d.e]+)', USD, re.S)
    got = {n: tuple(float(v) for v in rest) for n, *rest in joints}
    k = L.SPRING * np.pi / 180
    assert np.allclose(got["hingeL"], (k, -40., -2., 27.), atol=1e-6)
    assert np.allclose(got["hingeR"], (k, 40., -27., 2.), atol=1e-6)


def test_every_collider_is_a_small_convex_hull_and_display_parts_do_not_collide():
    meshes = re.findall(r'def Mesh "(\w+)" \(\n\s+prepend apiSchemas = \[(.*?)\]', USD)
    colliders = [n for n, apis in meshes if "PhysicsCollisionAPI" in apis]
    display = {n for n, apis in meshes if "PhysicsCollisionAPI" not in apis}
    assert len(colliders) == len(L.holder_pieces()) + len(L.funnel_pieces()) + 1 + 2 * (2 * L.SEGMENTS + 1)
    assert {"flask_glass", "funnel_glass", "base_glass", "membrane", "frit", "stopper_body", "sidearm", "coil", "wipe"} <= display
    for block in re.findall(r'PhysxConvexHullCollisionAPI"\]\n.*?point3f\[\] points = \[(.*?)\]', USD, re.S):
        assert block.count("(") <= 64
    assert 'uniform token physics:approximation = "convexHull"' in USD and "meshSimplification" not in USD


def test_glass_colliders_are_hidden_under_drawn_glass():
    for name in list(L.funnel_pieces()) + list(L.holder_pieces()):
        assert 'visibility = "invisible"' in prim(name)
    assert 'visibility = "invisible"' not in prim("seg0u")


def test_published_sizes_hold():
    assert abs(2 * L.FLANGE_R - .076) < 1e-9                          # filter holder diameter 7.6 cm
    assert abs((L.JOINT_Z + L.FUNNEL_H) - L.STEM_TIP - .235) < 1e-9   # assembled height 23.5 cm
    assert 2 * L.STEM_R <= .015                                       # stopper's 15 mm hole
    body = 2 * L.BODY_R
    assert .006 <= body <= .076 and L.FUNNEL_H - L.TAPER_TOP > .08     # a grasp in the cookbook's range, room for the pads
    volume = np.pi * (L.BODY_R - L.FUNNEL_WALL) ** 2 * (L.FUNNEL_H - L.TAPER_TOP)
    assert 2.5e-4 < volume < 3.5e-4                                   # about the 300 mL capacity


def test_clamp_geometry():
    # open fully, the stack passes into the mouth without touching; at rest it cannot
    assert L.mouth_clearance(L.THETA_MAX - np.radians(1.)) > 0.
    assert L.mouth_clearance(np.radians(15.)) < 0.
    # the V faces lie on the flanges' bevels when shut: the line z = 41 mm - r through both
    for r, z in ((L.LIP_R, L.FACE_Z_LIP), (L.ROOT_R, L.FACE_Z_ROOT), (L.CHAMFER_R, L.FLANGE_T), (L.FLANGE_R, L.CHAMFER_Z)):
        assert abs(z - (.041 - r)) < 1e-9
    # the lips clear the funnel's neck and the base's body
    assert L.LIP_R > L.NECK_R + .005 and L.LIP_R > L.CHAMFER_R - .002 + .0005
    assert 15. <= L.HANDLE_FORCE_N <= 25. and L.HANDLE_FORCE_N < 30.          # inside the gripper's ~30 N per finger
    handles = L.jaw_pieces(1)["handle"][:, :2]
    # at full opening each handle lies parallel to -x, inner face 0.5 mm off the centre line
    opened = (handles - L.PIVOT) @ L.rot2(L.THETA_MAX).T + L.PIVOT
    assert np.allclose(np.sort(np.unique(np.round(opened[:, 1], 6))), [.0005, .0055])


def test_policy_view_sees_the_parts_from_the_first_frame():
    # everything sits inside the arms' shared workspace in front of the robot, nothing hidden under another part
    for p in (F.HOLDER_START, F.FUNNEL_START, F.CLAMP_START):
        assert .2 < p[0] < .45 and abs(p[1]) < .25
    assert np.hypot(*(F.FUNNEL_START[:2] - F.HOLDER_START[:2])) > .15 and np.hypot(*(F.CLAMP_START[:2] - F.HOLDER_START[:2])) > .15
