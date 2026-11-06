"""Offline checks (no GPU): the task file loads, the reader's slot and stop and the card's cut corner let the card in
only one way round, the scorer covers each outcome, the latch's memory works, and the demo stays within reach with the
arms apart. Run in the yam_arena environment: python -m pytest -q test_sd_card_insert.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "sd_card_insert.py")
T = TASK.module

FLAT = np.array([0.0, 0.0, 0.0, 1.0])


def _quat(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.array([*(axis * math.sin(angle / 2)), math.cos(angle / 2)])


UPSIDE_DOWN = _quat((1, 0, 0), math.pi)  # flipped about the slot's axis: label down, cut corner on the left
REVERSED = _quat((0, 0, 1), math.pi)  # contacts toward the robot
FLIPPED_END = _quat((0, 1, 0), math.pi)  # label down and contacts toward the robot


def _state(pos, quat=FLAT, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.02, high=pos + 0.02)


def _card_at(card_depth, quat=FLAT, dy=0.0, dz=0.0):
    """The card in the slot, centred, its deepest point ``card_depth`` past the mouth."""
    x = T.READER_POS[0] + T.FRONT_X + card_depth - T.CARD_SIZE[0] / 2
    return _state((x, T.SLOT_Y + dy, T.SLOT_Z + dz), quat)


def _scene(card=None, reader=None):
    return {"card": card or _state(T.CARD_START), "reader": reader or _state(T.READER_POS)}


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


# -- the scene -------------------------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"reader", "eject_plate", "card"}


def test_card_and_slot_sizes():
    assert T.CARD_SIZE == pytest.approx((0.032, 0.024, 0.0021))
    assert T.SLOT_W - T.CARD_SIZE[1] == pytest.approx(0.001)
    assert T.SLOT_H - T.CARD_SIZE[2] == pytest.approx(0.0009)
    assert T.SLOT_Z - T.SLOT_H / 2 > T.SHELF_TOP  # the slot's floor is above the shelf


def test_the_card_overhangs_but_does_not_tip():
    back = T.CARD_START[0] - T.CARD_SIZE[0] / 2
    assert T.SHELF_FRONT - back == pytest.approx(T.OVERHANG)
    assert T.CARD_START[0] > T.SHELF_FRONT + 0.002  # its centre 2 mm or more onto the shelf
    assert T.CARD_START[2] - T.CARD_SIZE[2] / 2 == pytest.approx(T.SHELF_TOP)


def test_reader_sits_on_the_shelf_edge():
    assert T.READER_POS[0] - T.READER_SIZE[0] / 2 == pytest.approx(T.SHELF_FRONT)
    assert T.READER_POS[2] - T.READER_SIZE[2] / 2 == pytest.approx(T.SHELF_TOP)
    gap = (T.READER_POS[1] - T.READER_SIZE[1] / 2) - (T.CARD_START[1] + T.CARD_SIZE[1] / 2)
    assert gap > 0.05  # the card starts well clear of the reader


def _in_cavity(p):
    """A point of the reader's frame is inside the slot's open space (not in a collider)."""
    return not any(part.contains(p) for part in T.READER_PARTS)


def test_the_mouth_flares_and_the_slot_is_open():
    c = np.asarray(T.SLOT_C)
    for depth in np.linspace(0.0002, T.SLOT_DEPTH - 0.0002, 30):
        for dy in np.linspace(-T.SLOT_W / 2 + 0.0002, T.SLOT_W / 2 - 0.0002, 9):
            for dz in (-T.SLOT_H / 2 + 0.0001, 0.0, T.SLOT_H / 2 - 0.0001):
                p = c + (depth, dy, dz)
                key = (T.FRONT_X + T.SLOT_DEPTH - p[0]) + (p[1] - (c[1] - T.SLOT_W / 2)) < T.KEY + 0.0002
                assert _in_cavity(p) or key, (depth, dy, dz)
    # at the front face the opening is 2 mm wider each side and 1 mm taller above and below
    for dy, dz in ((-T.SLOT_W / 2 - 0.0018, 0.0), (T.SLOT_W / 2 + 0.0018, 0.0), (0.0, -T.SLOT_H / 2 - 0.0008),
                   (0.0, T.SLOT_H / 2 + 0.0008)):
        assert _in_cavity(c + (0.0001, dy, dz))
        assert not _in_cavity(c + (T.LEAD[0] + 0.0005, dy, dz))
    # and closed beyond the flare
    assert not _in_cavity(c + (0.0001, T.SLOT_W / 2 + 0.003, 0.0))
    assert not _in_cavity(c + (0.0001, 0.0, T.SLOT_H / 2 + 0.0015))


def _card_outline(quat, shift_y=0.0, n=60):
    """Points on the card's outline (its colliders' footprint) at its centre plane, turned by ``quat``, in the reader
    frame with its deepest point at the slot's back wall."""
    pts = []
    cx, cy, c = T.CARD_SIZE[0] / 2, T.CARD_SIZE[1] / 2, T.CHAMFER
    for t in np.linspace(0, 1, n):
        pts += [(-cx + t * 2 * cx, cy), (-cx + t * (2 * cx - c), -cy), (-cx, -cy + t * 2 * cy), (cx, -cy + c + t * (2 * cy - c)),
                (cx - c + t * c, -cy + t * c)]
    rot = T._rot(quat)
    pts = np.array([rot @ np.array([x, y, 0.0]) for x, y in pts])
    pts[:, 0] += T.FRONT_X + T.SLOT_DEPTH - pts[:, 0].max()
    pts[:, 1] += T.SLOT_C[1] + shift_y
    pts[:, 2] += T.SLOT_C[2]
    return pts


@pytest.mark.parametrize("shift", [-0.0005, 0.0, 0.0005])
def test_right_way_reaches_the_back_wall(shift):
    pts = _card_outline(FLAT, shift)
    assert all(_in_cavity(p - np.array([0.0002, 0.0, 0.0])) for p in pts)


@pytest.mark.parametrize("quat", [UPSIDE_DOWN, REVERSED, FLIPPED_END], ids=["upside down", "reversed", "both"])
@pytest.mark.parametrize("shift", [-0.0005, 0.0005])
def test_wrong_way_stops_short_of_the_click(quat, shift):
    pts = _card_outline(quat, shift)
    # Slide it out until no point is inside a collider: that is where it stops.
    for back in np.arange(0.0, 0.01, 0.0001):
        if all(_in_cavity(p - np.array([back, 0.0, 0.0])) for p in pts):
            break
    stop = T.SLOT_DEPTH - back
    assert stop < T.LATCH_AT - 0.0008, stop


def test_the_eject_plate_is_in_the_slot():
    p = T.PLATE
    assert p.to == "reader" and p.axis == "X" and not p.collide
    local = np.asarray(p.pos) - np.asarray(T.READER_POS)
    assert local[0] - p.size[0] / 2 == pytest.approx(T.FRONT_X + T.EJECT_AT)
    assert abs(local[2] - T.SLOT_C[2]) < 1e-9 and p.size[2] < T.SLOT_H
    assert local[1] - p.size[1] / 2 >= T.SLOT_C[1] - T.SLOT_W / 2 + T.CHAMFER - 1e-9  # clear of the stop
    assert T.EJECT_AT < T.LATCH_AT < T.SLOT_DEPTH


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    s = _scene()
    assert T.score(s) == 0.0 and not T.success(s)
    assert not T.tip_in(s)


@pytest.mark.parametrize("d,expected", [(0.002, 0.0), (0.004, 0.25), (0.0149, 0.25), (0.016, 0.5), (0.0235, 0.5)])
def test_ladder_right_way_unlatched(d, expected):
    assert T.score(_scene(card=_card_at(d))) == expected


def test_latched_scores_one():
    s = _scene(card=_card_at(0.0225))
    assert T.ready_to_click(s)
    T.STATE["latched"] = True
    assert T.score(s) == 1.0 and T.success(s)
    assert T.distance(s) == 0.0


def test_latch_needs_the_card_at_the_click():
    T.STATE["latched"] = True
    assert T.score(_scene(card=_card_at(0.020))) == 0.5  # pulled back out past the click: not latched
    assert not T.ready_to_click(_scene(card=_card_at(0.0215)))


def test_memory_resets():
    T.STATE["latched"] = True
    T.reset_memory()
    assert T.score(_scene(card=_card_at(0.0225))) == 0.5


@pytest.mark.parametrize("quat", [UPSIDE_DOWN, REVERSED, FLIPPED_END], ids=["upside down", "reversed", "both"])
def test_wrong_way_in_the_slot_scores_zero(quat):
    for d in (0.004, 0.016, 0.021):
        s = _scene(card=_card_at(d, quat))
        assert T.score(s) == 0.0 and not T.ready_to_click(s)


def test_card_beside_the_slot_scores_zero():
    assert T.score(_scene(card=_card_at(0.016, dy=0.02))) == 0.0  # past the mouth's line but not in the opening
    assert T.score(_scene(card=_card_at(0.016, dz=0.004))) == 0.0


def test_drooping_card_with_its_front_in_counts():
    """Let go 9 mm in, the card's back end droops: its front is still in the slot."""
    tilt = _quat((0, 1, 0), math.radians(8))  # nose up, back end down
    front_mid = np.array([T.READER_POS[0] + T.FRONT_X + 0.009, T.SLOT_Y, T.SLOT_Z])
    centre = front_mid - T._rot(tilt) @ np.array([T.CARD_SIZE[0] / 2, 0.0, 0.0])
    s = _scene(card=_state(centre, tilt))
    assert T.depth(s) == pytest.approx(0.009, abs=0.0005)
    assert T.score(s) == 0.25


def test_the_reader_moving_with_the_card_still_counts():
    shift = np.array([0.01, 0.005, 0.0])
    reader = _state(np.asarray(T.READER_POS) + shift)
    card = _state(_card_at(0.016).pos + shift)
    assert T.score(_scene(card=card, reader=reader)) == 0.5


def test_turned_reader_measured_in_its_frame():
    q = _quat((0, 0, 1), math.radians(10))
    reader = _state(T.READER_POS, q)
    rot = T._rot(q)
    centre = np.asarray(T.READER_POS) + rot @ np.array([T.FRONT_X + 0.016 - T.CARD_SIZE[0] / 2, T.SLOT_C[1], T.SLOT_C[2]])
    assert T.score(_scene(card=_state(centre, q), reader=reader)) == 0.5
    assert T.score(_scene(card=_state(centre, FLAT), reader=reader)) == 0.5  # 10 degrees off is still the right way


def test_violations_score_zero():
    T.STATE["latched"] = True
    fallen = _state(np.asarray(T.READER_POS) - (0.0, 0.0, 0.05))
    assert T.score(_scene(card=_card_at(0.0225), reader=fallen)) == 0.0
    assert T.score(_scene(card=_state((0.3, -0.2, T.TOP - 0.1)))) == 0.0
    assert T.score(_scene(card=_state((np.nan, 0.0, 1.0)))) == 0.0


# -- the demo --------------------------------------------------------------------------------------------------------


def _at(plan, label):
    t = 0.0
    for w in plan:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_card_depths():
    plan = T.demo()["right"]
    _, pick = _at(plan, "pads around its back end")
    tips = pick.pos[0] + T.PAD_TIP_PAST_TCP
    assert tips < T.SHELF_FRONT - 0.004  # the lower fingertip stops short of the shelf's front edge
    back = T.CARD_START[0] - T.CARD_SIZE[0] / 2
    assert tips - back == pytest.approx(T.GRIP_OVERLAP)
    _, slide = _at(plan, "slide it in")
    assert slide.pos[0] + T.PAD_TIP_PAST_TCP < T.SHELF_FRONT - 0.005  # held, the fingertips stay out of the reader
    _, push = _at(plan, "push until it clicks")
    assert push.pos[0] + T.PAD_TIP_PAST_TCP < T.SHELF_FRONT - 0.004
    assert push.pos[0] + T.PAD_TIP_PAST_TCP + T.CARD_SIZE[0] - T.SHELF_FRONT > T.LATCH_AT


def test_demo_keeps_the_arms_apart():
    plan = T.demo()
    assert max(w.pos[1] for w in plan["right"]) < -0.08
    assert min(w.pos[1] for w in plan["left"]) > -0.02
    _, push = _at(plan["right"], "push until it clicks")
    _, press = _at(plan["left"], "press it down")
    assert np.linalg.norm(np.asarray(push.pos) - np.asarray(press.pos)) > 0.08


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- the meshes --------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


@pytest.mark.parametrize("name", ["card", "reader"])
def test_scored_meshes_fit_in_their_boxes(name):
    """The card's and the reader's meshes stay inside their boxes, so their centres are their boxes' centres."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray({"card": T.CARD_SIZE, "reader": T.READER_SIZE}[name]) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)


def test_compound_parts_fit_in_their_boxes():
    for spec in (T.CARD, T.READER):
        spec._check()  # raises if a part sticks out
