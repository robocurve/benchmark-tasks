"""The generated scene file agrees with layout.py and fixed_scene.py."""

import importlib.util
import re
from pathlib import Path

import numpy as np

from rdb_etch_triangle import fixed_scene as fixture
from rdb_etch_triangle import layout

ROOT = Path(__file__).resolve().parents[1]
TEXT = fixture.USD.read_text()


def builder():
    spec = importlib.util.spec_from_file_location("build_assets", ROOT / "tools/build_assets.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def block(kind, name):
    start = TEXT.index(f'def {kind} "{name}"')
    following = re.search(r'\n        def (Xform|PhysicsRevoluteJoint|Material) "', TEXT[start + 10:])
    return TEXT[start:start + 10 + following.start()] if following else TEXT[start:]


def test_scene_file_is_what_the_generator_writes():
    assert builder().usd() == TEXT


def test_bodies_masses_and_one_articulation():
    masses = {"frame": layout.MASS - 2 * layout.KNOB_MASS, "left_knob": layout.KNOB_MASS, "right_knob": layout.KNOB_MASS}
    for name, mass in masses.items():
        body = block("Xform", name)
        assert f"float physics:mass = {mass:.6g}" in body and "PhysicsRigidBodyAPI" in body
    assert abs(sum(masses.values()) - .454) < 1e-9
    assert TEXT.count("PhysicsArticulationRootAPI") == 1 and "PhysicsArticulationRootAPI" in block("Xform", "frame")


def test_knobs_turn_freely_about_the_vertical_with_a_capped_drag():
    for name in fixture.JOINTS:
        joint = block("PhysicsRevoluteJoint", name)
        assert 'uniform token physics:axis = "Z"' in joint
        assert "lowerLimit" not in joint and "upperLimit" not in joint          # no stops: many turns
        assert f"float drive:angular:physics:maxForce = {layout.KNOB_FRICTION:.6g}" in joint
        assert "drive:angular:physics:stiffness = 0" in joint and "PhysxMimicJointAPI" not in joint
    assert TEXT.count("def PhysicsRevoluteJoint") == 2


def test_colliders_are_plain_shapes_hidden_and_display_meshes_do_not_collide():
    for kind, name, apis in re.findall(r'def (Cube|Cylinder|Mesh) "([^"]+)" \(\n\s+prepend apiSchemas = \[([^\]]*)\]', TEXT):
        if kind == "Mesh":
            assert "PhysicsCollisionAPI" not in apis, name
        else:
            assert "PhysicsCollisionAPI" in apis, name
            shape = TEXT[TEXT.index(f'def {kind} "{name}"'):]
            assert 'token visibility = "invisible"' in shape[:shape.index("}")], name
    knob = block("Xform", "left_knob")
    assert f"double radius = {layout.KNOB_R:.6g}" in knob and f"double height = {layout.KNOB_H:.6g}" in knob


def test_toy_and_knobs_are_where_the_setup_says():
    x, y, z = fixture.TOY_START
    assert f"double3 xformOp:translate = ({x:.6g}, {y:.6g}, {z + fixture.TOP:.6g})" in block("Xform", "frame")
    left, right = (fixture.knob_station(n) for n in fixture.JOINTS)
    assert np.allclose(left, (.3051, .0947, .0404), atol=2e-4) and np.allclose(right, (.3051, -.0947, .0404), atol=2e-4)
    assert left[1] > 0 > right[1]                                     # each knob on its own arm's side
    assert np.isclose(np.linalg.norm(left - right), .1893, atol=2e-4)
    # the triangle lies on the screen, inside the stylus travel
    lo = np.asarray(layout.TRAVEL_LL)
    for corner in layout.CORNERS:
        p = layout.screen_to_toy(corner)
        assert (p >= lo).all() and (p <= lo + layout.TRAVEL).all()


def test_screen_and_lip_heights():
    assert layout.SCREEN_Z < layout.BODY_H < layout.RIDGE_H < layout.KNOB_BOTTOM < layout.KNOB_TOP
    assert np.allclose(np.asarray(layout.SCREEN) * 1000, (161.4, 114.6))
