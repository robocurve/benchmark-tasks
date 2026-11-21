"""The generated asset, the layout and the fixed scene agree."""

import re
import subprocess
import sys
from pathlib import Path

import numpy as np

from rdb_tablets_cap import fixed_scene as fixture
from rdb_tablets_cap import layout as L

ROOT = Path(__file__).resolve().parents[1]
USD = fixture.USD.read_text()


def test_asset_is_what_the_generator_writes():
    sys.path.insert(0, str(ROOT / "tools"))
    import build_assets

    assert build_assets.usd() == USD


def test_bodies_and_masses():
    names = re.findall(r'def Xform "(\w+)" \(\n            prepend apiSchemas = \["PhysicsRigidBodyAPI"', USD)
    assert tuple(names) == (*fixture.BODIES, "spring_collar")
    masses = [float(v) for v in re.findall(r"float physics:mass = ([\d.e-]+)", USD)]
    assert np.allclose(masses, (*fixture.MASSES, L.COLLAR_MASS))
    assert "kinematicEnabled" not in USD
    assert USD.count("def PhysicsPrismaticJoint") == 1
    assert "rel physics:body1 = </Fixture/Closure/spring_collar>" in USD
    assert "rel physics:body0 = </Fixture/Actors/bottle>" in USD
    assert "rel physics:body1 = </Fixture/Actors/cap>" not in USD


def test_every_collider_is_convex_and_display_meshes_do_not_collide():
    assert USD.count('physics:approximation = "convexHull"') == USD.count('"PhysicsCollisionAPI"')
    assert USD.count('"PhysicsCollisionAPI"') == 5 * L.SIDES + 3 + L.N_TABLETS + 16 + 24 + 2
    for block in re.findall(r'def Mesh "(?:shell|label\w*|ribs|mark_\d)" \((.*?)\)', USD, flags=re.S):
        assert "PhysicsCollisionAPI" not in block


def test_tablets_start_inside_the_bottle_without_touching():
    points = np.array([L.tablet_start(i) for i in range(L.N_TABLETS)])
    assert (np.hypot(points[:, 0], points[:, 1]) + L.TABLET_R < L.BODY_R_IN * np.cos(np.pi / L.SIDES)).all()
    assert points[:, 2].min() - L.TABLET_T / 2 > L.BOTTLE_FLOOR and points[:, 2].max() < L.BODY_H
    for i in range(L.N_TABLETS):
        for j in range(i):
            d = points[i] - points[j]
            assert np.hypot(d[0], d[1]) >= 2 * L.TABLET_R + .001 or abs(d[2]) >= L.TABLET_T + .0005


def test_things_fit_the_gripper_and_each_other():
    assert 2 * L.BOTTLE_R < .076 and 2 * L.CAP_R < .076
    assert L.CAP_H >= .015                                        # enough skirt for the finger pads
    assert 2 * L.NECK_R_IN > 2.5 * 2 * L.TABLET_R                 # tablets cannot bridge the mouth
    assert 2 * L.CAP_R_IN > 3 * 2 * L.TABLET_R and L.CAP_DEPTH > 2 * L.TABLET_T
    assert L.cup_inner_radius(L.CUP_H) > 2 * 2 * L.TABLET_R
    gaps = [np.hypot(*(np.subtract(a, b))) for a, b in ((fixture.BOTTLE_XY, fixture.CAP_XY), (fixture.CAP_XY, fixture.CUP_XY))]
    assert min(gaps) > .085


def test_closure_geometry_has_an_axial_stop_a_detent_and_an_extraction_window():
    import build_assets
    import math

    def interval_at(mesh, degrees):
        # Cast a vertical line through the lug path using the actual vertices
        # of each convex sector. A point in a four-sided footprint is inside if
        # each edge's signed cross product has the same sign.
        hit = []
        ray = .0186 * np.array((math.cos(math.radians(degrees)), math.sin(math.radians(degrees))))
        for block in re.findall(r"point3f\[\] points = \[(.*?)\]", mesh):
            pts = np.array([list(map(float, p.split(","))) for p in re.findall(r"\((.*?)\)", block)])
            floor = pts[:4,:2]
            edges = np.roll(floor, -1, axis=0) - floor
            delta = ray - floor
            cross = edges[:, 0] * delta[:, 1] - edges[:, 1] * delta[:, 0]
            if (cross >= 0).all() or (cross <= 0).all():
                hit.append((pts[:,2].min(), pts[:,2].max()))
        return hit

    lock = build_assets.neck_lock()
    assert min(x[0] for x in interval_at(lock, 0.)) >= L.LUG_TOP
    assert min(x[0] for x in interval_at(lock, 12.)) < L.LUG_TOP
    assert min(x[0] for x in interval_at(lock, 12.)) > L.LUG_TOP - .003
    assert min(x[0] for x in interval_at(lock, -11.)) < L.LUG_TOP - .004
    for yaw in (41., 47., 53., 221., 227., 233.):
        assert not interval_at(lock, yaw)
    assert fixture.START["cap"] == (*fixture.BOTTLE_XY, L.CLOSED_Z)
    assert 'quatf xformOp:orient = (0, 1, 0, 0)' in USD
