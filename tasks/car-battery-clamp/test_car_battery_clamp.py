"""Checks of car_battery_clamp.py's scoring and demo reach, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_car_battery_clamp.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.task_file import load

TASK = load(Path(__file__).with_name("car_battery_clamp.py"))
M = TASK.module


def _rot(axis, deg):
    return M._rot(axis, math.radians(deg))


def _battery():
    from bodykit import state_from_pose

    return state_from_pose(M.BATTERY, (*M.BATT_XY, M.BATT_CZ), (0.0, 0.0, 0.0, 1.0))


def body(spec, pos, rot=np.eye(3), vel=(0.0, 0.0, 0.0)):
    from bodykit import state_from_pose

    return state_from_pose(spec, pos, matrix_to_quat_xyzw(rot), vel, (0.0, 0.0, 0.0))


def turn_for(torque):
    """The nut's turn from its finger-tight start that gives ``torque`` N m with the collar seated."""
    return M.SNUG_SEATED + (torque - M.T_RUN) / M.K_TORQUE


def scene(z=None, tilt=0.0, lug=0.0, torque=None, wrench_on_nut=False, x_off=0.0, moving=False, pads=None):
    """The clamp on the positive post (``z`` its origin height), the nut tightened to ``torque`` N m (None: run down by
    hand only, the bore not yet closed on the post), the wrench back in its rack (or on the nut), the grippers far
    away unless ``pads`` says otherwise."""
    r = _rot("z", lug) @ _rot("y", tilt)
    c = np.array([M.POS_XY[0] + x_off, M.POS_XY[1], M.SEATED_Z if z is None else z])
    nut_pos = c + r @ np.array([M.BOLT_X, M.NUT_Y, 0.0])
    if wrench_on_nut:
        w = body(M.WRENCH, nut_pos, r)
    else:
        w = body(M.WRENCH, M.WRENCH_START, M.bodykit.rotation(M.WRENCH_Q0))
    M._reset()
    if torque is not None:
        M._EP["bite_turn"] = M.SNUG_SEATED
        M._EP["turn"] = turn_for(torque)
    M._EP["caught"] = wrench_on_nut
    turn = M._EP["turn"]
    far = np.array([0.0, 0.0, 1.5])
    M._EP["pads"] = pads or {"left": [far, far], "right": [far, far]}
    v = (0.05, 0.0, 0.0) if moving else (0.0, 0.0, 0.0)
    BATTERY = _battery()
    return {"clamp": body(M.CLAMP, c, r, v), "nut": body(M.NUT, nut_pos, r @ _rot("y", -turn)), "wrench": w,
            "battery": BATTERY}


@pytest.mark.parametrize("torque", [5.0, 6.0, 7.0])
def test_seated_in_spec_square_and_let_go_is_full_score(torque):
    o = scene(torque=torque)
    assert (M.score(o), M.success(o)) == (1.0, True)


@pytest.mark.parametrize(("kwargs", "expected"), [
    (dict(torque=4.9), 0.25),  # seated, under-tightened
    (dict(torque=2.0), 0.25),
    (dict(torque=7.1), 0.25),  # over-tightened
    (dict(torque=12.0), 0.25),
    (dict(torque=6.0, lug=20.0), 0.5),  # lug dragged round
    (dict(torque=6.0, lug=45.0), 0.25),
    (dict(torque=6.0, wrench_on_nut=True), 0.75),  # wrench left on the nut
    (dict(torque=6.0, moving=True), 0.75),
    (dict(torque=6.0, z=M.SEATED_Z + 0.003), 0.0),  # tightened 3 mm high
    (dict(torque=6.0, tilt=5.0, z=M.SEATED_Z + 0.002), 0.0),  # crooked
    (dict(torque=6.0, x_off=0.005), 0.0),  # off the post's axis
    (dict(torque=None), 0.25),  # pushed down, nut only run down by hand
])
def test_partial_and_zero_credit(kwargs, expected):
    assert M.score(scene(**kwargs)) == expected


def test_the_nut_torque_rises_from_where_the_bore_closes():
    assert M.snug_turn(M.S_SEATED) == pytest.approx(30.0)
    assert M.snug_turn(M.S_SEATED + 0.006) == pytest.approx(630.0)  # 6 mm up the taper: 600 degrees more
    assert M.nut_torque(turn=100.0, bite_turn=None) == M.T_RUN
    assert M.nut_torque(turn=30.0 + 120.0, bite_turn=30.0) == pytest.approx(5.0)
    assert M.nut_torque(turn=30.0 + 180.0, bite_turn=30.0) == pytest.approx(7.0)


def test_the_taper_numbers():
    """100 degrees per mm: the post narrows 2 mm across over its 18 mm, so 1 mm up the bore is 0.111 mm looser across,
    0.349 mm round, which the ears close at 1.25 mm per turn (M8 coarse)."""
    per_mm = math.pi * (M.POS_D[0] - M.POS_D[1]) / M.POST_H / 1.25 * 360  # degrees per mm of height
    assert M.SNUG_PER_MM == pytest.approx(per_mm, abs=1.0)


def test_a_hand_near_the_cable_is_not_full_score():
    o = scene(torque=6.0)
    cable = np.array([M.POS_XY[0] + 0.10, M.POS_XY[1], M.SEATED_Z + M.LUG_Z])  # 10 cm behind the post, on the cable
    near = cable + [0.0, 0.0, 0.03]
    o = scene(torque=6.0, pads={"left": [near, near], "right": [cable + [0, 0, 1], cable + [0, 0, 1]]})
    assert M.score(o) == 0.75
    assert M.pad_clearance(o) == pytest.approx(0.03 - M.CABLE_R, abs=0.002)


def test_the_clip_stands_behind_the_battery_and_the_cable_reaches_it():
    lo, hi = M.CABLE_CLIP.local_bounds()
    back_face = M.BATT_XY[0] + M.BATT[0] / 2
    assert M.CLIP_XY[0] + lo[0] > back_face  # nothing of the clip stand under or against the battery
    c_lo, c_hi = M.CLAMP.local_bounds()
    assert M.CLAMP_START[0] + c_hi[0] > M.CLIP_XY[0] - 0.01  # the drawn cable runs into the clip
    assert M.CLIP_Z - M.TOP == pytest.approx(M.BATT[2] + 0.022, abs=0.006)  # about 2 cm above the lid


def test_the_start_scores_zero():
    o = {"clamp": body(M.CLAMP, M.CLAMP_START, M.bodykit.rotation(M.CLAMP_Q0)),
         "nut": body(M.NUT, M.NUT_START, M.bodykit.rotation(M.CLAMP_Q0)),
         "wrench": body(M.WRENCH, M.WRENCH_START, M.bodykit.rotation(M.WRENCH_Q0)), "battery": _battery()}
    M._reset()
    assert M.score(o) == 0.0


def test_a_hand_still_on_the_clamp_is_not_full_score():
    c = np.array([M.POS_XY[0], M.POS_XY[1], M.SEATED_Z])
    o = scene(torque=6.0, pads={"left": [c + [0, 0.02, 0], c - [0, 0.02, 0]], "right": [c + [0, 0, 1], c + [0, 0, 1]]})
    assert M.score(o) == 0.75


def test_the_nut_angle_reads_tightening_as_positive():
    o = scene(torque=2.0)
    assert M.nut_rel_deg(o["clamp"], o["nut"]) == pytest.approx(turn_for(2.0), abs=1e-6)


def test_a_pushed_battery_scores_zero():
    o = scene(torque=6.0)
    M._EP["batt0"] = np.array([M.BATT_XY[0] - 0.01, M.BATT_XY[1], M.BATT_CZ])
    assert M.score(o) == 0.0


def test_every_seeded_start_is_off_its_seat_and_scores_zero():
    for seed in range(20):
        lay = M.layout(seed)
        o = {n: body(getattr(M, n.upper()), lay[n][0], M.bodykit.rotation(lay[n][1])) for n in ("clamp", "nut", "wrench")}
        from bodykit import state_from_pose
        o["battery"] = state_from_pose(M.BATTERY, lay["battery"][0], lay["battery"][1])
        M._reset()
        assert M.score(o) == 0.0
        assert M.gaps(o["clamp"], o["battery"]).max() > 0.003


def test_real_sizes():
    assert M.BATT == (0.173, 0.260, 0.225)
    assert M.POS_D == (0.0195, 0.0175) and M.NEG_D[0] < M.POS_D[0]
    assert 2 * M.BORE_R - M.POS_D[0] == pytest.approx(0.0006)
    assert M.SEATED_Z + M.H / 2 < M.BOSS_TOP + M.POST_H  # the post stands proud of the seated collar


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plans = M.demo()
    plan = ArmPlan(plans[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
