"""Checks of stud_layout_jig.py's scene, scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_stud_layout_jig.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("stud_layout_jig.py"))
M = TASK.module


def stud_state(x, y, yaw_deg=0.0, roll_deg=0.0, bottom=None, speed=0.0, quat=None):
    """A stud with its centre at x, y, turned ``yaw_deg`` about the vertical and rolled ``roll_deg`` about its own
    length, its lowest point at ``bottom``. The world-aligned box comes from the turned half-extents, as the
    simulator reports it."""
    if quat is None:
        yaw, roll = math.radians(yaw_deg), math.radians(roll_deg)
        cr, sr, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(yaw / 2), math.sin(yaw / 2)
        quat = np.array([sr * cy, -sr * sy, cr * sy, cr * cy])  # Rz(yaw) * Rx(roll), xyzw
    quat = np.asarray(quat, dtype=float)
    probe = ObjectState(np.zeros(3), quat, np.zeros(3), np.zeros(3), np.zeros(3), np.ones(3))
    half = np.abs(M._rotation(probe)) @ M.STUD_HALF
    centre = np.array([x, y, (M.BASE_TOP if bottom is None else bottom) + half[2]])
    return ObjectState(centre, quat, np.array([speed, 0.0, 0.0]), np.zeros(3), centre - half, centre + half)


def seated(slot, short=0.0, **kw):
    return stud_state(M.SEATED_X - short, M.SLOT_Y[slot], **kw)


def all_seated(**override):
    objects = {s.name: seated(i) for i, s in enumerate(M.STUDS)}
    objects.update(override)
    return objects


def at_start():
    return {s.name: stud_state(s.pos[0], s.pos[1], math.degrees(s.yaw), bottom=s.pos[2] - M.STUD_DEPTH / 2)
            for s in M.STUDS}


# -- scene -------------------------------------------------------------------------------------


def test_real_lumber_sizes_and_mass():
    assert M.STUD_THICK == pytest.approx(0.0381)
    assert M.STUD_DEPTH == pytest.approx(0.0889)
    assert M.STUD_LENGTH == pytest.approx(0.3683)
    assert 0.5 < M.STUD_MASS < 0.6
    assert M.SLOT_W == pytest.approx(0.0441)
    assert M.SLOT_Y[1] - M.SLOT_Y[0] == pytest.approx(12 * 0.0254)


def test_scene_names_are_unique_and_studs_are_the_only_free_objects():
    assert len(set(TASK.names)) == len(TASK.names)
    assert set(TASK.free_objects) == {s.name for s in M.STUDS}


@pytest.mark.parametrize("slot", range(4))
def test_rounded_caps_keep_the_slot_width(slot):
    """Each cap sphere meets its cleat's slot-side face at the top of the square part, never inside the slot."""
    y = M.SLOT_Y[slot]
    for obj in M.OBJECTS:
        if obj.name.startswith(f"cleat_{slot + 1}") and "_round_" in obj.name:
            inner = abs(obj.pos[1] - y) - obj.radius
            assert inner == pytest.approx(M.SLOT_W / 2, abs=1e-9)
            assert obj.pos[2] == pytest.approx(M.BASE_TOP + M.CLEAT_BOX_H)


def test_cleats_are_their_real_height():
    for obj in M.OBJECTS:
        if "_round_" in obj.name:
            assert obj.pos[2] + obj.radius == pytest.approx(M.BASE_TOP + M.CLEAT_H)


# -- scoring -----------------------------------------------------------------------------------


def test_full_score_when_all_seated_and_flush():
    assert M.score(all_seated()) == pytest.approx(1.0)
    assert M.success(all_seated())


def test_nothing_done_scores_zero():
    assert M.score(at_start()) == 0.0
    assert not M.success(at_start())


def test_seated_short_of_the_fence_earns_seat_credit_only():
    objects = {s.name: seated(i, short=0.010) for i, s in enumerate(M.STUDS)}
    assert M.score(objects) == pytest.approx(0.6)


@pytest.mark.parametrize("short,expected", [(0.0015, 1.0), (0.0025, 0.9), (0.003, 0.9)])
def test_flush_tolerance_is_two_millimetres(short, expected):
    assert M.score(all_seated(stud_1=seated(0, short=short))) == pytest.approx(expected)


@pytest.mark.parametrize(
    "label,stud",
    [
        ("laid flat on the cleats", lambda: stud_state(M.SEATED_X, M.SLOT_Y[0], roll_deg=90,
                                                       bottom=M.BASE_TOP + M.CLEAT_H)),
        ("resting on the cleat tops", lambda: seated(0, bottom=M.BASE_TOP + M.CLEAT_H)),
        ("beside its slot on the plywood", lambda: stud_state(M.SEATED_X, M.SLOT_Y[0] + 0.03)),
        ("behind the cleats", lambda: seated(0, short=M.CLEAT_L)),
        ("tilted 5 degrees", lambda: seated(0, roll_deg=5)),
        ("turned 5 degrees", lambda: seated(0, yaw_deg=5)),
        ("still moving", lambda: seated(0, speed=0.05)),
        ("on the table, not the jig", lambda: seated(0, bottom=M.TOP)),
    ],
)
def test_a_stud_not_seated_earns_nothing(label, stud):
    assert M.score(all_seated(stud_1=stud())) == pytest.approx(0.75), label


def test_seated_with_a_small_turn_inside_the_slot_counts():
    assert M.score(all_seated(stud_1=seated(0, yaw_deg=1.5))) == pytest.approx(1.0)


def test_one_slot_credits_one_stud():
    assert M.score(all_seated(stud_2=seated(0))) == pytest.approx(0.75)


@pytest.mark.parametrize(
    "label,objects",
    [
        ("NaN pose", lambda: all_seated(stud_1=ObjectState(np.full(3, np.nan), np.array([0, 0, 0, 1.0]),
                                                           np.zeros(3), np.zeros(3), np.zeros(3), np.ones(3)))),
        ("non-unit quaternion", lambda: all_seated(stud_1=seated(0, quat=[0, 0, 0, 1.2]))),
        ("missing stud", lambda: {k: v for k, v in all_seated().items() if k != "stud_3"}),
        ("outline not a 2x4", lambda: all_seated(stud_1=ObjectState(seated(0).pos, seated(0).quat_xyzw,
                                                                    np.zeros(3), np.zeros(3), seated(0).low,
                                                                    seated(0).high + 0.004))),
    ],
)
def test_fail_safes_void_the_trial(label, objects):
    assert M.score(objects()) == 0.0, label
    assert not M.success(objects()), label


def test_distance_is_zero_when_seated():
    assert M.distance(all_seated()) == pytest.approx(0.0, abs=1e-9)


# -- demo --------------------------------------------------------------------------------------


def test_demo_uses_both_arms_and_fits_the_run_limit():
    plans = M.demo()
    assert set(plans) == {"left", "right"}
    longest = max(sum(w.duration for w in plans[arm]) for arm in plans)
    assert longest * 20 < 2400


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable_with_the_official_ik(arm):
    plan = ArmPlan(list(M.demo()[arm]), station.BASES[arm])
    lo, hi = np.asarray(ik.LOWER), np.asarray(ik.UPPER)
    t = 0.0
    while t <= plan.duration:
        q = plan.action7(t)[0][:6]
        assert np.all(q >= lo - 1e-6) and np.all(q <= hi + 1e-6)
        t += 0.05
    assert plan.worst_ik_mm < 1.0
    assert plan.worst_ik_deg < 0.5
