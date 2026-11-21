"""Offline checks for folding_step_stool.py (no simulator): the legs' angle, the spring locks, the check, the scoring,
the sizes, and the scripted demo against the YAM's reach."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import folding_step_stool as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

HERE = Path(__file__).resolve().parent


def frame(pos, rot=np.eye(3), moving=False) -> Frame:
    p = np.asarray(pos, float)
    v = np.array([0.1, 0.0, 0.0]) if moving else np.zeros(3)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=v, spin=np.zeros(3), com=p)


def leg_rot(leg, open_deg, top_rot=M.UPSIDE_DOWN):
    turn = M.rot_x(-math.radians(open_deg)) if leg == "leg_a" else M.rot_x(math.radians(open_deg))
    return top_rot @ turn


def torque_on(acc, name):
    return acc[name][1]


class Rig:
    def __init__(self):
        self.m = M.Locks()
        self.m.log = self.m.trace = None

    def step(self, open_a, open_b, top_z=M.TOP_Z, top_rot=M.UPSIDE_DOWN, n=24, held=None, moving=False):
        acc = {}
        if held is not None:
            self.m.hands_off = not held
            if held:
                self.m.ever_held = True
        for _ in range(n):
            acc = {}
            top = M.TOP_AT.copy()
            top[2] = top_z
            self.m.update({"stool_top": frame(top, top_rot, moving), "leg_a": frame(top, leg_rot("leg_a", open_a, top_rot)),
                           "leg_b": frame(top, leg_rot("leg_b", open_b, top_rot))}, acc)
        return acc


def test_the_task_file_loads():
    task = load(HERE / "folding_step_stool.py")
    assert [o.name for o in task.objects] == ["stool_top", "leg_a", "leg_b"]


def test_the_legs_angle_reads_folded_square_and_open_whichever_way_up():
    for top_rot in (M.UPSIDE_DOWN, np.eye(3)):
        for leg in ("leg_a", "leg_b"):
            for deg in (0.0, 45.0, 90.0, 100.0):
                assert M.leg_angle_deg(top_rot, leg_rot(leg, deg, top_rot), leg) == pytest.approx(deg, abs=1e-6)


def test_short_of_the_lock_the_spring_pushes_back_and_past_98_it_locks():
    assert M.push_back_nm(79.0) == 0.0 and M.push_back_nm(98.0) == 0.0
    assert M.push_back_nm(80.0) == pytest.approx(0.10) and M.push_back_nm(97.9) == pytest.approx(0.165, abs=0.002)
    r = Rig()
    acc = r.step(90.0, 0.0)
    assert not r.m.locked["leg_a"] and np.linalg.norm(torque_on(acc, "leg_a")) > 0.0
    r.step(98.5, 0.0)
    assert r.m.locked["leg_a"] and not r.m.locked["leg_b"]
    r.step(95.0, 0.0)  # once locked, it stays locked
    assert r.m.locked["leg_a"]


def test_the_check_comes_on_only_after_a_hand_has_held_it_and_is_off_with_the_stool_still_on_the_table():
    r = Rig()
    r.step(100.0, 100.0)
    assert not r.m.checked  # nobody has touched it yet
    r.step(100.0, 100.0, held=True)
    assert not r.m.checked  # a hand on it
    r.step(100.0, 100.0, held=False, moving=True)
    assert not r.m.checked  # still moving
    r.step(100.0, 100.0, held=False, top_z=M.TOP_Z + 0.03)
    assert not r.m.checked  # lifted
    r.step(100.0, 100.0, held=False, top_rot=np.eye(3))
    assert not r.m.checked  # right way up (it should lie upside down)
    r.step(100.0, 100.0, held=False)
    assert r.m.checked


def test_under_the_check_an_unlocked_leg_folds_and_a_locked_one_holds():
    r = Rig()
    r.step(100.0, 94.0, held=True)
    acc = r.step(100.0, 94.0, held=False)
    assert r.m.checked and r.m.locked["leg_a"] and not r.m.locked["leg_b"] and r.m.gave_way["leg_b"]
    a_axis, b_axis = M.opening_axis(M.UPSIDE_DOWN, "leg_a"), M.opening_axis(M.UPSIDE_DOWN, "leg_b")
    assert float(torque_on(acc, "leg_a") @ a_axis) == pytest.approx(M.HOLD_TORQUE)
    assert float(torque_on(acc, "leg_b") @ b_axis) == pytest.approx(-M.CHECK_TORQUE)
    # a hand back on it: the check is off, and the folded leg can be opened again without the check's push
    acc = r.step(100.0, 30.0, held=True)
    assert not r.m.checked and "leg_b" not in acc


def test_the_tiers():
    M.MODEL.clear()
    assert M.score({}) == 0.0
    M.MODEL.locked["leg_a"] = True
    assert M.score({}) == 0.25
    M.MODEL.locked["leg_b"] = True
    assert M.score({}) == 0.5 and not M.success({})
    M.MODEL.checked = True
    assert M.score({}) == M.COMPLETE_SCORE and M.success({})
    M.MODEL.clear()


def test_the_stool_is_the_vive_lva2068blk():
    assert M.TOP_L == pytest.approx(0.381) and M.TOP_D == pytest.approx(0.2667)
    assert M.STAND_TOP == pytest.approx(0.1651)
    assert M.TOP_MASS + 2 * M.LEG_MASS == pytest.approx(1.8)
    # standing open, the platform's top is at the Vive's 6.5 in
    h = M.KNUCKLE_R + M.BAR_R + M.LEG_L * math.cos(math.radians(M.OPEN_DEG - 90.0)) + M.TOP_T
    assert h == pytest.approx(M.STAND_TOP, abs=1e-6)


def test_folded_the_legs_lie_inside_the_platform_and_do_not_overlap():
    for leg in ("leg_a", "leg_b"):
        assert abs(M.bar_centre(leg, 0.0)[1]) < M.TOP_L / 2
    assert abs(M.bar_centre("leg_a", 0.0)[1]) > M.BAR_R  # the two crossbars clear of each other at the middle


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    p = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, p.duration, 0.05):
        p.action7(t)
    assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, (p.worst_ik_mm, p.worst_ik_deg)


def test_the_demo_fits_the_run_limit():
    plans = M.demo()
    assert max(sum(w.duration for w in plans[a]) for a in ("left", "right")) * 20 < 1200
