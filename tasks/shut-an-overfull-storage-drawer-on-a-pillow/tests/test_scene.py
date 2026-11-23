"""The generated scene file agrees with layout.py, and the layout with the published sizes."""

import importlib.util
import re
from pathlib import Path

import numpy as np

from rdb_overfull_drawer import fixed_scene as fixture
from rdb_overfull_drawer import layout as L

ROOT = Path(__file__).resolve().parents[1]
TEXT = fixture.USD.read_text()


def builder():
    spec = importlib.util.spec_from_file_location("build_scene", ROOT / "tools/build_scene.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def block(kind, name):
    start = TEXT.index(f'def {kind} "{name}"')
    following = re.search(r'\n        def (Xform|PhysicsPrismaticJoint|Material) "', TEXT[start + 10:])
    return TEXT[start:start + 10 + following.start()] if following else TEXT[start:]


def test_scene_file_is_what_the_generator_writes():
    assert builder().usd() == TEXT


def test_published_sizes():
    inch = .0254
    assert np.allclose(L.FRAME_SIZE, (17 * inch, 14.375 * inch, 6.875 * inch), atol=.0006)
    assert np.allclose(L.DRAWER_INNER, (16.375 * inch, 11.875 * inch, 4.875 * inch), atol=.0006)
    assert L.PILLOW_PUBLISHED == (.46, .33, .15) and L.PILLOW_MASS == .325
    assert max(L.REST_TOP) == L.PILLOW_PUBLISHED[2]
    assert abs(sum(L.MASSES[1:]) - L.PILLOW_MASS) < 1e-9


def test_frame_and_drawer_boxes_fit():
    b = builder()
    frame = {name: (np.array(low), np.array(high)) for name, low, high in b.frame_boxes()}
    low = np.min([v[0] for v in frame.values()], axis=0)
    high = np.max([v[1] for v in frame.values()], axis=0)
    assert np.allclose(high - low, L.FRAME_SIZE) and low[0] == L.FRAME_FRONT_X and low[2] == 0.
    assert frame["top"][0][2] == L.RAIL_Z
    drawer = {name: (np.array(low), np.array(high)) for name, low, high in b.drawer_boxes()}
    inside = (drawer["wall_back"][0][0] - drawer["front"][1][0], drawer["wall_left"][0][1] - drawer["wall_right"][1][1],
              drawer["wall_left"][1][2] - drawer["floor"][1][2])
    assert np.allclose(inside, L.DRAWER_INNER)
    # shut, the drawer is inside the frame with at least 2 mm to spare everywhere except its front and pull
    for name, (lo, hi) in drawer.items():
        if name == "pull":
            continue
        lo, hi = lo + (L.FRAME_FRONT_X, 0, 0), hi + (L.FRAME_FRONT_X, 0, 0)
        assert hi[2] <= L.RAIL_Z - .002 and lo[2] >= .010 + .002
        assert abs(lo[1]) <= L.FRAME_SIZE[1] / 2 - L.FRAME_WALL - .002 and hi[1] <= L.FRAME_SIZE[1] / 2 - L.FRAME_WALL - .002
        assert hi[0] <= L.FRAME_FRONT_X + L.FRAME_SIZE[0] - L.FRAME_BACK - .002


def test_bodies_masses_and_start_poses():
    for name, mass in zip(L.ACTORS, L.MASSES, strict=True):
        body = block("Xform", name)
        assert f"float physics:mass = {mass:.6g}" in body
        assert "PhysicsRigidBodyAPI" in body and "PhysxContactReportAPI" in body
        x, y, z = fixture.WORLD_START[name]
        assert f"double3 xformOp:translate = ({x:.6g}, {y:.6g}, {z:.6g})" in body
    assert "PhysicsArticulationRootAPI" not in TEXT


def test_every_drawn_box_collides():
    cubes = re.findall(r'def Cube "[^"]+" \(\n\s+prepend apiSchemas = \[([^\]]+)\]', TEXT)
    assert len(cubes) == 5 + 6 + 1 + len(L.PADS) and all("PhysicsCollisionAPI" in c for c in cubes)
    assert "Mesh" not in TEXT and "Sphere" not in TEXT


def test_drawer_slide_is_a_friction_capped_slide_with_no_spring():
    joint = block("PhysicsPrismaticJoint", "drawer_slide")
    assert "drive:linear:physics:stiffness = 0\n" in joint
    assert f"drive:linear:physics:maxForce = {L.RUNNER_FRICTION_N:.6g}\n" in joint
    assert "physics:lowerLimit = 0\n" in joint and f"physics:upperLimit = {L.OPEN_LIMIT:.6g}\n" in joint
    assert "rel physics:body0" not in joint and "rel physics:body1 = </Fixture/Actors/drawer>" in joint


def test_each_pad_is_on_its_own_spring():
    for row, col in L.PADS:
        joint = block("PhysicsPrismaticJoint", f"spring_{row}_{'lr'[col]}")
        assert f"drive:linear:physics:stiffness = {L.PAD_STIFFNESS:.6g}\n" in joint
        assert f"drive:linear:physics:damping = {L.PAD_DAMPING:.6g}\n" in joint
        assert f"physics:lowerLimit = {-L.PAD_TRAVEL:.6g}\n" in joint and "physics:upperLimit = 0\n" in joint
        assert "rel physics:body0 = </Fixture/Actors/pillow_base>" in joint
        assert f"rel physics:body1 = </Fixture/Actors/{L.pad_name(row, col)}>" in joint
    assert TEXT.count("def PhysicsPrismaticJoint") == 1 + len(L.PADS)
    # no joint ties the pillow to the drawer, the frame or a gripper
    assert not re.search(r"body0 = </Fixture/Actors/drawer>", TEXT)


def test_pillow_stands_above_the_opening_and_starts_caught():
    assert [round(1000 * p) for p in L.ROW_PROUD] == [-12, 10, 20, 20, 20, 20, 10, -12]
    start = L.start_poses()
    for row, col in L.PADS:
        x, _, top = start[L.pad_name(row, col)]
        lead = x + L.PAD_SIZE[0] / 2
        if lead > L.RAIL_X:
            assert top < L.RAIL_Z                       # under the frame: started below its underside
        else:
            assert abs(top - (L.FLOOR_Z + L.REST_TOP[row])) < .001
    assert 0 < L.RAIL_X - (start["pad_3_l"][0] + L.PAD_SIZE[0] / 2) < .008
    # forces stay within what the arms can do
    assert L.press_force(2, .025) < 5.
    under = sum(L.press_force(r, L.ROW_PROUD[r]) for r in L.PROUD_ROWS)
    assert L.RUNNER_FRICTION_N + .3 * under < 10.


def test_pads_do_not_collide_with_each_other_or_their_base():
    base = block("Xform", "pillow_base")
    assert all(f"</Fixture/Actors/{L.pad_name(r, c)}>" in base for r, c in L.PADS)
    assert "PhysicsFilteredPairsAPI" in block("Xform", "pad_0_l")
