"""Offline checks for drain.py (no Isaac Sim): every scoring outcome, the demo's reach, and the demo replayed through
the scorer (the part riding in the gripper between grasp and release).

    python -m pytest test_drain.py -q
"""

from pathlib import Path

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("drain.py"))
M = TASK.module
DT = 1.0 / M.CONTROL_HZ
E = M.PART_EDGE

START = np.array((*M.BATH_XY, M.FLOOR_TOP + E / 2))
ABOVE_BATH = START + (0.0, 0.0, 0.10)  # bottom well above the rim
ON_PAD = np.array((M.PAD.pos[0], M.PAD.pos[1], M.MAT_TOP + E / 2))
ON_RACK = np.array((M.RACK.pos[0], M.RACK.pos[1], M.MAT_TOP + E / 2))


def state(pos, vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, dtype=float), np.full(3, E / 2)
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


class Episode:
    """Plays the part's motion through score() one control step at a time, as the embodiment does."""

    def __init__(self, fresh=True):
        if fresh:
            M._state.clear()
            M._state.update(M._fresh_state())
        self.pos = START.copy()
        self.last = self.step()

    def step(self, vel=(0.0, 0.0, 0.0)):
        self.last = TASK.score({"metal_part": state(self.pos, vel)})
        return self.last

    def go(self, goal, speed=0.2, settle=True):
        """Straight line to goal at speed."""
        start, goal = self.pos.copy(), np.asarray(goal, float)
        steps = max(1, int(np.ceil(np.linalg.norm(goal - start) / (speed * DT))))
        vel = (goal - start) / (steps * DT)
        for k in range(1, steps + 1):
            self.pos = start + (goal - start) * k / steps
            self.step(vel)
        if settle:
            for _ in range(3):
                self.step()
        return self.last

    def hold(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.step()
        return self.last

    def to(self, goal):
        """Carry at the height above the bath, then lower onto goal."""
        self.go(np.array((goal[0], goal[1], ABOVE_BATH[2])), settle=False)
        return self.go(goal)


def test_doing_nothing_scores_zero():
    assert Episode().hold(10) == 0.0


def test_drain_then_pad_then_rack_scores_one():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    e.hold(3.2)
    e.to(ON_PAD)
    assert e.to(ON_RACK) == 1.0 and not M.violated()


def test_carrying_it_away_without_draining_scores_zero():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    e.to(ON_PAD)
    assert e.to(ON_RACK) == 0.0 and M.violated()


def test_two_seconds_is_not_enough():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    e.hold(2.0)
    e.to(ON_PAD)
    assert e.last == 0.0 and M.violated()


def test_moving_above_the_bath_does_not_count_as_draining():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    for _ in range(4):  # 4 s of slow circling above the bath, never still
        e.go(ABOVE_BATH + (0.03, 0.0, 0.0), speed=0.06, settle=False)
        e.go(ABOVE_BATH, speed=0.06, settle=False)
    e.to(ON_PAD)
    assert e.last == 0.0 and M.violated()


def test_dipping_it_again_needs_a_new_drain():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    e.hold(3.2)
    e.go(START, settle=False)  # back into the bath
    e.go(ABOVE_BATH, settle=False)
    e.to(ON_PAD)
    assert e.last == 0.0 and M.violated()


def test_drained_and_on_the_pad_scores_half():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    e.hold(3.2)
    assert e.to(ON_PAD) == 0.5


def test_drained_but_still_held_above_the_bath_scores_a_quarter():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    assert e.hold(4.0) == 0.25


def test_a_part_off_the_table_scores_zero():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    e.hold(3.2)
    assert e.go(np.array((0.25, 0.10, M.TOP - 0.20))) == 0.0


def test_a_reset_starts_a_new_episode():
    e = Episode()
    e.go(ABOVE_BATH, settle=False)
    e.to(ON_PAD)
    assert M.violated()
    Episode(fresh=False)  # the embodiment resets: the part jumps back into the bath, at rest
    assert not M.violated()


# -- the scripted demo ----------------------------------------------------------------------------------------------


def grasp_intervals(waypoints):
    """(time the fingers finish closing, time they start opening) for each grasp in an arm's plan."""
    out, t, closed_at = [], 0.0, None
    for w in waypoints:
        if w.label.startswith("close"):
            closed_at = t + w.duration
        elif w.label.startswith("release"):
            out.append((closed_at, t))
        t += w.duration
    return out


def test_the_demo_replayed_through_the_scorer_scores_one():
    M._state.clear()
    M._state.update(M._fresh_state())
    plan = TASK.demo()
    arms = {a: ArmPlan(plan[a], station.BASES[a]) for a in plan}
    holds = [(a, t0, t1) for a in ("left", "right") for t0, t1 in grasp_intervals(plan[a])]
    assert len(holds) == 2
    pos, score = START.copy(), 0.0
    for k in range(int(round((max(a.duration for a in arms.values()) + 2.0) / DT))):
        t = (k + 1) * DT
        new = pos
        for arm, t0, t1 in holds:
            if t0 <= t <= t1:
                new = np.asarray(arms[arm].target(t)[0], float)  # the grip is at the part's centre
        score = TASK.score({"metal_part": state(new, (new - pos) / DT)})
        pos = new
    assert score == 1.0 and not M.violated()


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(arm):
    a = ArmPlan(TASK.demo()[arm], station.BASES[arm])
    actions = np.array([a.action7((k + 1) / CONTROL_HZ)[0] for k in range(round(a.duration * CONTROL_HZ) + 5)])
    assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
