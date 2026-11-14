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


def scene(z=None, tilt=0.0, lug=0.0, turn=0.0, wrench_on_nut=False, x_off=0.0, moving=False, pads=None):
    """The clamp on the positive post (``z`` its origin height), the nut turned ``turn`` degrees past its mark, the
    wrench back in its rack (or on the nut), the grippers far away unless ``pads`` says otherwise."""
    r = _rot("z", lug) @ _rot("y", tilt)
    c = np.array([M.POS_XY[0] + x_off, M.POS_XY[1], M.SEATED_Z if z is None else z])
    nut_pos = c + r @ np.array([M.BOLT_X, M.NUT_Y, 0.0])
    if wrench_on_nut:
        w = body(M.WRENCH, nut_pos, r)
    else:
        w = body(M.WRENCH, M.WRENCH_START, M.bodykit.rotation(M.WRENCH_Q0))
    M._reset()
    M._EP["turn"] = turn
    M._EP["caught"] = wrench_on_nut
    far = np.array([0.0, 0.0, 1.5])
    M._EP["pads"] = pads or {"left": [far, far], "right": [far, far]}
    v = (0.05, 0.0, 0.0) if moving else (0.0, 0.0, 0.0)
    BATTERY = _battery()
    return {"clamp": body(M.CLAMP, c, r, v), "nut": body(M.NUT, nut_pos, r @ _rot("y", -turn)), "wrench": w,
            "battery": BATTERY}


def test_seated_tight_square_and_let_go_is_full_score():
    o = scene(turn=100.0)
    assert (M.score(o), M.success(o)) == (1.0, True)


@pytest.mark.parametrize(("kwargs", "expected"), [
    (dict(turn=40.0), 0.25),  # seated, not tight
    (dict(turn=380.0), 0.25),  # crushed
    (dict(turn=100.0, lug=20.0), 0.5),  # lug dragged round
    (dict(turn=100.0, lug=45.0), 0.25),
    (dict(turn=100.0, wrench_on_nut=True), 0.75),  # wrench left on the nut
    (dict(turn=100.0, moving=True), 0.75),
    (dict(turn=100.0, z=M.SEATED_Z + 0.003), 0.0),  # tightened 3 mm high
    (dict(turn=100.0, tilt=5.0, z=M.SEATED_Z + 0.002), 0.0),  # crooked
    (dict(turn=100.0, x_off=0.005), 0.0),  # off the post's axis
    (dict(turn=0.0), 0.25),  # pushed down, never tightened
])
def test_partial_and_zero_credit(kwargs, expected):
    assert M.score(scene(**kwargs)) == expected


def test_the_start_scores_zero():
    o = {"clamp": body(M.CLAMP, M.CLAMP_START, M.bodykit.rotation(M.CLAMP_Q0)),
         "nut": body(M.NUT, M.NUT_START, M.bodykit.rotation(M.CLAMP_Q0)),
         "wrench": body(M.WRENCH, M.WRENCH_START, M.bodykit.rotation(M.WRENCH_Q0)), "battery": _battery()}
    M._reset()
    assert M.score(o) == 0.0


def test_a_hand_still_on_the_clamp_is_not_full_score():
    c = np.array([M.POS_XY[0], M.POS_XY[1], M.SEATED_Z])
    o = scene(turn=100.0, pads={"left": [c + [0, 0.02, 0], c - [0, 0.02, 0]], "right": [c + [0, 0, 1], c + [0, 0, 1]]})
    assert M.score(o) == 0.75


def test_the_nut_angle_reads_tightening_as_positive():
    o = scene(turn=30.0)
    assert M.nut_rel_deg(o["clamp"], o["nut"]) == pytest.approx(30.0, abs=1e-6)


def test_a_pushed_battery_scores_zero():
    o = scene(turn=100.0)
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
