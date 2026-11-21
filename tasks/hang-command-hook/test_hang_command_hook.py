"""Scoring, geometry and demo checks for hang-command-hook that run without the simulator:
python -m pytest -q test_hang_command_hook.py"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import hang_command_hook as M

HERE = Path(__file__).resolve().parent
Z3 = np.zeros(3)
MM = 0.001


def pads(x, y, tips_z) -> M.Pads:
    return M.Pads(back=np.array([x, y, tips_z + M.PAD_TIP_PAST_TCP + M.PAD_BACK]), tips=np.array([x, y, tips_z]))


AWAY = {"left_gripper": pads(0.20, 0.30, M.TOP + 0.15), "right_gripper": pads(0.20, -0.30, M.TOP + 0.15)}


def quat_about_x(deg: float) -> np.ndarray:
    h = math.radians(deg) / 2
    return np.array([math.sin(h), 0.0, 0.0, math.cos(h)])


def wall(push_n=0.0) -> M.MechanismState:
    p = np.array([M.WALL_X, M.WALL_Y, M.TOP])
    return M.MechanismState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3, ang_vel=Z3, low=p, high=p,
                            joints={"panel_give": push_n / M.WALL_K})


def hook_at(dy_mm=0.0, dz_mm=0.0, tilt_deg=0.0, gap_mm=0.0, push_n=0.0) -> ObjectState:
    """The hook with its sticky face's middle ``gap_mm`` in front of the (pushed-in) wall, off the cross by dy, dz."""
    p = M.MARK + np.array([push_n / M.WALL_K - gap_mm * MM, dy_mm * MM, dz_mm * MM])
    return ObjectState(pos=p, quat_xyzw=quat_about_x(tilt_deg), lin_vel=Z3, ang_vel=Z3, low=p, high=p)


def scene(bond=M.BondState(), push_n=0.0, other_n=0.0, **hook) -> dict:
    return {"wall": wall(push_n), "hook": hook_at(push_n=push_n, **hook), "bond": bond, "panel_other_n": other_n,
            **AWAY}


STUCK = M.BondState(bonded=True, firm_s=M.PRESS_S, force_n=0.0)


# -- the task file and the parts ---------------------------------------------------------------------------------------

def test_the_task_file_loads():
    task = load(HERE / "hang_command_hook.py")
    assert task.names == ("wall", "holder_front", "holder_back", "hook")
    assert task.free_objects == ("hook",)
    assert [link.name for link in M.WALL.links] == ["panel"] and M.WALL.joints[0].name == "panel_give"


def test_the_hook_is_the_bought_hook():
    # 3M Command Medium Utility Hook 17001: 76 x 23 x 25 mm
    assert M.PLATE_H == pytest.approx(0.076) and M.PLATE_W == pytest.approx(0.023)
    assert M.STRIP_T + M.PLATE_T + M.ARM_OUT == pytest.approx(0.025, abs=0.0005)
    assert M.STRIP_H < M.PLATE_H and M.STRIP_W < M.PLATE_W  # the strip lies within the plate


def test_the_hook_stands_in_its_slot_without_its_strip_touching_the_holder():
    margin = (M.PLATE_H - M.STRIP_H) / 2
    assert M.SLOT_DEPTH < margin  # the strip starts above the slot's blocks
    assert M.SLOT_GAP > M.PLATE_T  # the plate fits the slot
    assert -M.PLATE_H / 2 + M.ARM_Z0 > -M.PLATE_H / 2 + M.SLOT_DEPTH  # the arm clears the front block


def test_the_wall_measures_the_press():
    joint = M.WALL.joints[0]
    assert joint.stiffness == M.WALL_K and (joint.lower, joint.upper) == (0.0, M.WALL_TRAVEL)
    assert M.wall_force(scene(push_n=12.0)) == pytest.approx(12.0)
    assert M.FIRM_N / M.WALL_K < 0.0005  # the wall gives under half a millimetre at a firm press


def test_the_pencil_cross_is_on_the_wall_where_the_docstring_says():
    assert M.MARK[0] == M.WALL_X and M.MARK[1] == pytest.approx(0.12) and M.MARK[2] - M.TOP == pytest.approx(0.13)
    assert 0.0 < M.MARK[1] < M.WALL_W and 0.02 < M.MARK[2] - M.TOP < 0.02 + M.WALL_H


# -- the bond ---------------------------------------------------------------------------------------------------------

class FakeEnv:
    class unwrapped:  # noqa: N801
        step_dt = 0.05


def run_bond(monkeypatch, sequence):
    calls = []
    monkeypatch.setattr(M, "_set_bond", lambda rel: calls.append(rel))
    bond = M._Bond()
    return [bond.update(FakeEnv, s) for s in sequence], calls


def test_the_strip_bonds_when_it_lies_flat_on_the_wall_under_pressure(monkeypatch):
    seq = [scene(gap_mm=3.0), scene(gap_mm=0.2), scene(gap_mm=0.2, push_n=2.0), scene(gap_mm=0.0, push_n=12.0)]
    out, calls = run_bond(monkeypatch, seq)
    assert [b.bonded for b in out] == [False, False, True, True]
    assert len(calls) == 1 and calls[0] is not None  # switched on once, at the pose of that moment
    assert out[-1].firm_s == pytest.approx(0.05)


def test_a_strip_touching_at_one_edge_does_not_bond(monkeypatch):
    s = scene(gap_mm=0.0, push_n=5.0)
    s["hook"] = ObjectState(pos=s["hook"].pos - np.array([0.003, 0, 0]),
                            quat_xyzw=np.array([0.0, math.sin(math.radians(4) / 2), 0.0, math.cos(math.radians(4) / 2)]),
                            lin_vel=Z3, ang_vel=Z3, low=Z3, high=Z3)  # tipped 4 degrees: only its top edge touches
    gaps = M.corner_gaps_mm(s)
    assert int(np.sum(gaps <= M.TOUCH_MM)) < 3
    out, _ = run_bond(monkeypatch, [s])
    assert not out[0].bonded


def test_the_press_counts_only_firm_seconds(monkeypatch):
    seq = [scene(gap_mm=0.0, push_n=12.0)] + [scene(gap_mm=0.0, push_n=12.0)] * 4 + [scene(gap_mm=0.0, push_n=5.0)] * 3
    out, _ = run_bond(monkeypatch, seq)
    assert out[-1].firm_s == pytest.approx(4 * 0.05)


def test_a_press_beside_the_hook_does_not_count(monkeypatch):
    # the gauge feels 15 N, but 14 N of it comes from a finger on the bare wall
    seq = [scene(gap_mm=0.0, push_n=2.0)] + [scene(gap_mm=0.0, push_n=15.0, other_n=14.0)] * 5
    out, _ = run_bond(monkeypatch, seq)
    assert out[0].bonded and out[-1].firm_s == 0.0 and out[-1].other_n == pytest.approx(14.0)


def test_a_finger_resting_on_the_wall_while_pressing_the_hook_does_not_count(monkeypatch):
    seq = [scene(gap_mm=0.0, push_n=2.0)] + [scene(gap_mm=0.0, push_n=15.0, other_n=M.OTHER_MAX_N)] * 3
    out, _ = run_bond(monkeypatch, seq)
    assert out[-1].firm_s == 0.0


def test_unreadable_contacts_never_count(monkeypatch):
    seq = [scene(gap_mm=0.0, push_n=2.0)] + [scene(gap_mm=0.0, push_n=15.0, other_n=math.inf)] * 3
    out, _ = run_bond(monkeypatch, seq)
    assert out[-1].firm_s == 0.0


def test_the_contact_reader_without_a_simulator_reports_inf():
    assert M._PanelContacts().other_n(FakeEnv) == math.inf


def test_arrays_without_detach_are_read():
    # The simulator's contact views return warp arrays, which have .numpy() but no .detach() (first official run,
    # 2026-11-20: every press was dropped as unreadable until this was handled).
    class WarpLike:
        def numpy(self):
            return np.array([[1.0, 2.0, 3.0]], dtype=np.float32)

    assert M._numpy(WarpLike()).tolist() == [[1.0, 2.0, 3.0]]


# -- scoring -----------------------------------------------------------------------------------------------------------

def test_not_stuck_scores_0():
    assert M.grade(scene()) == 0.0 and M.grade(scene(gap_mm=0.0, push_n=12.0)) == 0.0


def test_stuck_on_the_mark_level_pressed_30_s_and_let_go_succeeds():
    s = scene(bond=STUCK, dy_mm=2.0, dz_mm=-2.0, tilt_deg=1.0)
    assert M.grade(s) == 1.0 and M.success(s) and M.score(s) == M.COMPLETE_SCORE


def test_pressed_too_briefly_scores_0_75():
    s = scene(bond=M.BondState(bonded=True, firm_s=5.0))
    assert M.grade(s) == 0.75 and not M.success(s)


def test_a_hand_still_on_it_scores_0_75():
    h = M.MARK + np.array([-0.012, 0.0, 0.0])
    s = scene(bond=STUCK)
    s["left_gripper"] = pads(h[0], h[1], h[2] - 0.01)
    assert not M.hands_off(s) and M.grade(s) == 0.75


@pytest.mark.parametrize("kw", [dict(dy_mm=8.0), dict(dz_mm=-12.0), dict(tilt_deg=5.0)])
def test_near_the_mark_scores_0_5(kw):
    assert M.grade(scene(bond=STUCK, **kw)) == 0.5


@pytest.mark.parametrize("kw", [dict(dy_mm=30.0), dict(tilt_deg=12.0)])
def test_stuck_far_off_scores_0_25(kw):
    assert M.grade(scene(bond=STUCK, **kw)) == 0.25


def test_the_tilt_is_measured_in_the_walls_plane():
    assert abs(M.placement(scene(tilt_deg=4.0)).tilt_deg) == pytest.approx(4.0)  # its sign: about the wall normal


# -- the demo ----------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("mode", ["demo", "brief", "off", "beside"])
def test_the_demo_is_within_reach(mode, monkeypatch):
    monkeypatch.setenv("HOOK_DEMO", mode)
    d = M.demo()
    for arm in ("left", "right"):
        plan = ArmPlan(d[arm], station.BASES[arm])
        t = 0.0
        while t < plan.duration:
            plan.action7(t)
            t += 0.05
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 1.0, (mode, arm)


def test_the_fingertips_stay_behind_the_strip():
    tcp = M.tcp_for(M.MARK)
    a = M.GRIP[:, 2]
    b = np.cross(a, [0.0, 1.0, 0.0])
    b /= np.linalg.norm(b)
    corners = [tcp + M.PAD_TIP_PAST_TCP * a + s * 0.013 * b for s in (-1, 1)]
    assert max(c[0] for c in corners) == pytest.approx(M.MARK[0] - M.TIP_CLEAR)
    mid = tcp + M.PAD_TIP_PAST_TCP * a
    assert mid[2] == pytest.approx(M.MARK[2])  # the fingertips level with the plate's middle


def test_the_press_lasts_long_enough_and_the_brief_one_does_not(monkeypatch):
    d = M.demo()
    labels = {w.label: w.duration for w in d["left"]}
    assert labels["keep pressing"] >= M.PRESS_S
    monkeypatch.setenv("HOOK_DEMO", "brief")
    assert {w.label: w.duration for w in M.demo()["left"]}["keep pressing"] < M.PRESS_S / 2


def test_the_demo_finishes_within_the_run_limit():
    assert sum(w.duration for w in M.demo()["left"]) * 20 < 1800


def test_the_beside_probe_presses_the_bare_wall_long_enough(monkeypatch):
    monkeypatch.setenv("HOOK_DEMO", "beside")
    steps = M.demo()["left"]
    labels = [w.label for w in steps]
    assert "straight in until the strip touches" in labels and "press firmly" not in labels  # never pressed through it
    assert {w.label: w.duration for w in steps}["keep pressing the wall"] >= M.PRESS_S
    assert abs(M.BESIDE[1] - M.MARK[1]) - M.PLATE_W / 2 > 0.02  # the press is well clear of the hook
    assert M.TACK_N < M.LIGHT_N < M.FIRM_N  # the hook is tacked on hard enough to bond, never firmly
    assert sum(w.duration for w in steps) * 20 < 1800
