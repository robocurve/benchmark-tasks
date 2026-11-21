"""Offline checks of the scorer (no simulator): python -m pytest -q test_task.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import task as t  # noqa: E402
from hoistkit import HoistState  # noqa: E402

DT = 0.05
FAR = np.array([0.15, 0.30, 1.0])  # a gripper well away from everything


def state(lift: float, rope=None) -> HoistState:
    """The kettlebell hanging plumb under the block with its base ``lift`` m above the mat."""
    pos = np.array([t.AXLE_X + t.R_PITCH, 0.0, t.MAT_TOP + lift + 0.076])
    if rope is None:
        zs = np.linspace(pos[2] + 0.14, t.AXLE_Z - 0.01, 8)
        rope = np.column_stack([np.full(8, pos[0]), np.zeros(8), zs])
        arc = [[t.AXLE_X + t.R_PITCH * np.cos(a), 0.0, t.AXLE_Z + t.R_PITCH * np.sin(a)] for a in np.linspace(0, 2.6, 6)]
        rope = np.vstack([rope, arc, [t.haul_point(d) for d in (0.2, 0.4, 0.6, 0.8)]])
    return HoistState(
        pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos,
        load_pos=pos, load_rot=np.eye(3), load_bottom_z=t.MAT_TOP + lift, load_lin_vel=np.zeros(3),
        rope_points=rope, rope_rots=None, rope_lengths=None,
    )


def run(lifts, tcps=(FAR, FAR)):
    ep = t.Episode()
    for h in lifts:
        t.advance(ep, DT, state(h), tcps)
    return ep


def test_tiers_follow_the_highest_line_reached():
    assert t.score_of(run([0.0] * 10)) == 0.0
    assert t.score_of(run(np.linspace(0, 0.099, 10))) == 0.0
    assert t.score_of(run(np.linspace(0, 0.10, 10))) == 0.25
    assert t.score_of(run(np.linspace(0, 0.20, 10))) == 0.5
    assert t.score_of(run(np.linspace(0, 0.30, 10))) == 0.75
    assert t.score_of(run(np.linspace(0, 0.35, 10))) == 0.75


def test_full_score_needs_two_seconds_above_the_red_line():
    up = list(np.linspace(0, 0.41, 20))
    assert t.score_of(run(up + [0.41] * 30)) == 0.75  # 1.5 s
    assert t.score_of(run(up + [0.41] * 40)) == 1.0  # 2.0 s
    # a dip below the line restarts the count
    assert t.score_of(run(up + [0.41] * 30 + [0.39] + [0.41] * 30)) == 0.75


def test_letting_the_load_run_back_scores_zero():
    up = list(np.linspace(0, 0.35, 20))
    assert t.score_of(run(up + [0.26])) == 0.75  # 9 cm back: still fine
    assert t.score_of(run(up + [0.24])) == 0.0  # 11 cm back
    # once it has run back, holding it high afterwards does not help
    assert t.score_of(run(up + [0.24] + [0.45] * 60)) == 0.0


def test_touching_the_kettlebell_or_its_rope_scores_zero():
    s = state(0.0)
    on_bell = s.load_pos + np.array([t.BELL_R + 0.01, 0.0, 0.0])
    assert t.score_of(run(np.linspace(0, 0.45, 60), (on_bell, FAR))) == 0.0
    on_handle = s.load_pos + np.array([0.0, 0.0, t.HANDLE_Z + 0.02])
    assert t.score_of(run([0.0, 0.0], (FAR, on_handle))) == 0.0
    on_load_rope = s.load_pos + np.array([0.0, 0.0, 0.30])
    assert t.score_of(run([0.0, 0.0], (on_load_rope, FAR))) == 0.0


def test_gripping_the_hauling_rope_is_allowed():
    tcp = np.array(t.haul_point(0.6))
    assert t.score_of(run(np.linspace(0, 0.20, 10), (tcp, FAR))) == 0.5


def test_hauling_line_starts_on_the_sheave():
    p0 = t.haul_point(0.0)
    d = np.hypot(p0[0] - t.AXLE_X, p0[2] - t.AXLE_Z)
    assert abs(d - t.R_PITCH) < 1e-9
