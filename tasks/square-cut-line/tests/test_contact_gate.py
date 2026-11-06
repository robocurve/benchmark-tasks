# SPDX-License-Identifier: MIT
"""Run the exact pure runtime functions without importing a GPU simulator."""
import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from square_cut_line.contact import contact_gate, lift_gate, guide_metrics, rotation_xyzw


def functions():
    source = Path(__file__).parents[1]/"square_cut_line"/"runtime.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef)
             and node.name in {"rotation_xyzw", "contact_gate", "lift_gate", "_json_clean"}]
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    nodes += [node for node in cls.body if isinstance(node, ast.FunctionDef)
              and node.name == "_add_ink"]
    namespace = dict(np=np, math=math, spec=SimpleNamespace(
        TARGET_START=(.23, -.03), TARGET_END=(.35, -.03),
        BOARD_SIZE=(.14, .28, .016), MARKER_NIB_RADIUS=.002,
        REQUIRE_GUIDE_CONTACT=False, INK_WIDTH=.002))
    namespace.update(contact_gate=contact_gate, lift_gate=lift_gate,
                     rotation_xyzw=rotation_xyzw,
                     _guide_metrics=lambda _, *args: guide_metrics(*args))
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), namespace)
    return namespace


@pytest.fixture
def gate():
    return functions()["contact_gate"]


def test_contact_requires_pair_force_and_tip_geometry(gate):
    marker = [.29, -.03, .8155, 0, 0, 0, 1]
    board = [.29, 0, .7555, 0, 0, 0, 1]
    point = [.29, -.03, .7635]
    assert gate([0, 0, .03], point, marker, board)[0]
    assert not gate([0, 0, 0], point, marker, board)[0]
    assert not gate([0, 0, .005], point, marker, board)[0]
    marker[2] += .01
    assert not gate([0, 0, .03], point, marker, board)[0]


def test_barrel_contact_and_upside_down_marker_rejected(gate):
    board = [.29, 0, .7555, 0, 0, 0, 1]
    assert not gate([0, 0, .1], [.29, -.03, .7635],
                    [.29, -.03, .7635, 0, 0, 0, 1], board)[0]
    assert not gate([0, 0, .1], [.29, -.03, .7635],
                    [.29, -.03, .7155, 1, 0, 0, 0], board)[0]


@pytest.mark.parametrize("point", [[float("nan"), 0, 0], [.4, 0, .7635], [.29, 0, .768]])
def test_nonfinite_outside_and_wrong_surface_rejected(gate, point):
    assert not gate([0, 0, .1], point, [.29, 0, .8155, 0, 0, 0, 1],
                    [.29, 0, .7555, 0, 0, 0, 1])[0]


def test_guide_must_be_physically_seated_flat_and_aligned():
    guide = functions()["_guide_metrics"]
    point = np.array([.29, -.03, .7635])
    square = np.array([.208, .09, .7655, 0, 0, 0, 1])
    board = np.array([.29, 0, .7555, 0, 0, 0, 1])
    assert guide(None, point, square, board, np.zeros(3))["guide_used"]
    square[2] += .01
    assert not guide(None, point, square, board, np.zeros(3))["guide_used"]
    square[2] -= .01
    square[0] += .01
    assert not guide(None, point, square, board, np.zeros(3))["guide_used"]


def test_null_json_contacts_preserve_standard_json():
    clean = functions()["_json_clean"]
    assert clean({"pair_point": [float("nan"), float("inf"), .1]}) == {
        "pair_point": [None, None, .1]}


def test_full_result_requires_physical_lift_and_quiet_interval():
    lift = functions()["lift_gate"]
    board = [.29, 0, .7555, 0, 0, 0, 1]
    touching = [.29, -.03, .8155, 0, 0, 0, 1]
    clear = [.29, -.03, .8205, 0, 0, 0, 1]
    assert not lift(touching, board, 1.0, clear_s=1.0)[0]
    assert not lift(clear, board, .499, clear_s=.499)[0]
    assert lift(clear, board, .5, clear_s=.5)[0]
    assert lift(clear, board, .5, clear_s=.5)[1]["nib_clearance"] == pytest.approx(.005)
    assert not lift(clear, board, 1.0, clear_s=1/240)[0]
    assert not lift([float("nan")]*7, board, 1.0)[0]


def test_visible_ink_matches_two_millimetre_capsule_width():
    add = functions()["_add_ink"]
    state = SimpleNamespace(_points=[], _faces=[], _face_counts=[], _ink_dirty=False)
    add(state, [0, 0], None)
    add(state, [.01, 0], np.array([0, 0]))
    points = np.asarray(state._points)
    assert points[:, 0].min() == pytest.approx(-.001)
    assert points[:, 0].max() == pytest.approx(.011)
    assert np.ptp(points[:, 1]) == pytest.approx(.002)
    assert sum(state._face_counts) == len(state._faces)
    assert state._ink_dirty


def test_blade_rolled_about_contact_edge_is_not_a_flat_guide():
    angle = np.radians(10)
    quaternion = [np.sin(angle/2), 0, 0, np.cos(angle/2)]
    rotation = rotation_xyzw(quaternion)
    # Keep both physical blade edge endpoints at their correct board-top
    # positions while rotating the blade plane by ten degrees.
    root = np.array([.208, -.0275, .7635])-rotation @ np.array([0, -.1175, -.002])
    square = np.r_[root, quaternion]
    metrics = guide_metrics(np.array([.29, -.03, .7635]), square,
                            np.array([.29, 0, .7555, 0, 0, 0, 1]), np.zeros(3))
    assert metrics["guide_flat_error"] < 1e-10
    assert metrics["guide_tilt_deg"] == pytest.approx(10)
    assert not metrics["guide_used"]
