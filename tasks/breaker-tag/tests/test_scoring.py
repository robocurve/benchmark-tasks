"""Scoring checks without the simulator: task.py's predicates on hand-built states, and the grippers' collision hulls
placed by yam_arena's IK and forward kinematics (NumPy only).

    python tests/test_scoring.py          # or: pytest tests/
"""
from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geometry as g  # noqa: E402
import task  # noqa: E402
from yam_arena import analysis, ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402

T = g.TOP
START = dict(g.START_ON)
HOLE = np.array([0.3186, 0.0423, T + 0.2097])           # the hanging tag's hole, as the simulation settles it


def ends(**changes) -> dict:
    on = dict(START)
    on.update({int(k[1:]): v for k, v in changes.items()})
    return on


def rad(q_on: dict) -> dict:
    return {n: (g.HANDLE_STOP if v else -g.HANDLE_STOP) for n, v in q_on.items()}


def hulls(side, tcp=None, rot=None, gap=0.0):
    """The gripper hulls of one arm: at READY, or with its tool point at ``tcp``, fingers ``gap`` apart."""
    if tcp is None:
        q = np.array(READY, float)
    else:
        q, err, rerr = ik.solve(np.asarray(tcp, float), ik.matrix_to_quat_xyzw(rot), q_init=np.array(READY, float),
                                base_pos=station.BASES[side], iters=300)
        assert err < 0.002, err
    joints = {f"joint{i + 1}": float(v) for i, v in enumerate(q)}
    f = -gap / 2
    joints.update(joint7=f, joint8=f)
    poses = analysis.fk_link_poses_world(joints, station.BASES[side])
    return [(n, h) for n, h in analysis.world_hulls(poses) if n in task.HAND_LINKS]


def flat():
    a, c = np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
    return np.column_stack([c, np.cross(a, c), a])


def boxes(on=None):
    return task._boxes(rad(on or START), (HOLE, np.eye(3)))


# -- the tiers ----------------------------------------------------------------------------------------------------------
def test_start_scores_0_flag_1():
    assert task.tier(START, False, False, True) == 0.0
    assert task.flag(False, False) == 1


def test_both_on_released_scores_1():
    assert task.tier(ends(s2=True, s3=True), False, False, True) == 1.0


def test_both_on_not_released_scores_075():
    assert task.tier(ends(s2=True, s3=True), False, False, False) == 0.75


def test_space4_on_then_off_scores_0_sticky():
    assert task.tier(ends(s2=True, s3=True), True, False, True) == 0.0
    assert task.flag(True, False) == 0


def test_space4_on_at_end_scores_0():
    assert task.tier(ends(s2=True, s3=True, s4=True), False, False, True) == 0.0


def test_tag_off_scores_0():
    assert task.tier(ends(s2=True, s3=True), False, True, True) == 0.0
    assert task.flag(False, True) == 0


def test_one_target_scores_half():
    assert task.tier(ends(s3=True), False, False, True) == 0.5
    assert task.tier(ends(s2=True), False, False, False) == 0.5


def test_other_breaker_off_scores_quarter():
    assert task.tier(ends(s2=True, s3=True, s6=False, s7=False), False, False, True) == 0.25
    assert task.tier(ends(s3=True, s1=False), False, False, True) == 0.25


def test_every_end_state_has_one_tier():
    """3 states of the targets (neither, one, both) x 2^5 of the other five, released or not."""
    seen = {}
    for t2, t3 in ((False, False), (True, False), (True, True)):
        for others in itertools.product((False, True), repeat=5):
            for rel in (False, True):
                on = dict(START)
                on[2], on[3] = t2, t3
                on.update(dict(zip(g.OTHERS, others)))
                s = task.tier(on, False, False, rel)
                assert s in (0.0, 0.25, 0.5, 0.75, 1.0)
                if not t2 and not t3:
                    assert s == 0.0
                elif not all(others):
                    assert s == 0.25
                elif t2 and t3:
                    assert s == (1.0 if rel else 0.75)
                else:
                    assert s == 0.5
                seen[s] = seen.get(s, 0) + 1
    assert set(seen) == {0.0, 0.25, 0.5, 0.75, 1.0}


def test_midway_is_the_line():
    assert task.handle_on(math.radians(0.5))
    assert not task.handle_on(math.radians(-0.5))


# -- the spring --------------------------------------------------------------------------------------------------------
def test_spring_forces_at_the_tip():
    """10 N at the tip to bring a handle to midway from OFF, 6 N from ON; it rests on its stop with a preload."""
    on_t, off_t = task.T_ON, task.T_OFF
    f_on = g.K_HANDLE * (0.0 - off_t) / g.HANDLE_R
    f_off = g.K_HANDLE * (on_t - 0.0) / g.HANDLE_R
    assert abs(f_on - 10.0) < 0.01 and abs(f_off - 6.0) < 0.01
    assert g.K_HANDLE * (-g.HANDLE_STOP - off_t) > 0 and g.K_HANDLE * (on_t - g.HANDLE_STOP) > 0


# -- released: the grippers' hulls against the panel, the handles and the tag -------------------------------------------
def test_ready_pose_is_released():
    d, _ = task.gripper_gap(hulls("left") + hulls("right"), boxes())
    assert d > 2 * g.RELEASE_GAP, d


def test_finger_on_a_handle_is_not_released():
    tip = g.handle_tip(2, g.HANDLE_STOP) + [0, 0, T]
    tcp = np.array([g.FRONT_X - 0.0055 - 0.013, tip[1] - 0.011, tip[2] - 0.003])
    d, name = task.gripper_gap(hulls("right", tcp, flat()), boxes(ends(s2=True)))
    assert d <= g.RELEASE_GAP, (d, name)


def test_finger_15mm_under_the_tag_is_not_released():
    bottom = HOLE + np.array([0.0, 0.0, g.TAG_HOLE - g.TAG_L])
    tcp = bottom + np.array([0.004 - 0.013, 0.0, -0.015 - 0.0075])
    d, name = task.gripper_gap(hulls("right", tcp, flat()), boxes())
    assert d <= g.RELEASE_GAP, (d, name)


def test_hand_60mm_in_front_is_released():
    tcp = np.array([g.FRONT_X - 0.060 - 0.013 - 0.030, 0.20, T + 0.30])
    d, name = task.gripper_gap(hulls("left", tcp, flat()), boxes())
    assert d > g.RELEASE_GAP, (d, name)


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted((k, v) for k, v in globals().items() if k.startswith("test_") and callable(v)):
        try:
            fn()
            print("PASS", name)
        except Exception as e:                           # noqa: BLE001
            failed += 1
            print("FAIL", name, repr(e))
    print(f"{failed} failed")
    sys.exit(1 if failed else 0)
