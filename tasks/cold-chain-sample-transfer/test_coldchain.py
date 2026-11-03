"""Offline checks for coldchain.py (no Isaac Sim): every scoring outcome, the demo's reach, and the demo replayed
through the scorer (the vial riding in the gripper between grasp and release).

    python -m pytest test_coldchain.py -q
"""

from pathlib import Path

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("coldchain.py"))
M = TASK.module
DT = 1.0 / M.CONTROL_HZ
SIZE = (2 * M.VIAL_R, 2 * M.VIAL_R, M.VIAL_H)

START = np.array((*M.LEFT_BOX_XY, M.FLOOR_TOP + M.VIAL_H / 2))
ON_PAD = np.array((M.PAD.pos[0], M.PAD.pos[1], M.PAD_TOP + M.VIAL_H / 2))
IN_RIGHT = np.array((*M.RIGHT_BOX_XY, M.FLOOR_TOP + M.VIAL_H / 2))


def state(pos, vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, dtype=float), np.asarray(SIZE, dtype=float) / 2
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


class Episode:
    """Plays the vial's motion through score() one control step at a time, as the embodiment does."""

    def __init__(self, fresh=True):
        if fresh:
            M._state.clear()
            M._state.update(M._fresh_state())
        self.pos = START.copy()
        self.last = self.step()

    def step(self, vel=(0.0, 0.0, 0.0)):
        self.last = TASK.score({"sample_vial": state(self.pos, vel)})
        return self.last

    def move(self, goal, lift=0.10, speed=0.2):
        up = np.array((0.0, 0.0, lift))
        goal = np.asarray(goal, float)
        for end in ([self.pos + up, goal + up, goal] if lift else [goal]):
            start = self.pos.copy()
            steps = max(1, int(np.ceil(np.linalg.norm(end - start) / (speed * DT))))
            vel = (end - start) / (steps * DT)
            for k in range(1, steps + 1):
                self.pos = start + (end - start) * k / steps
                self.step(vel)
        for _ in range(3):
            self.step()
        return self.last

    def wait(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.step()
        return self.last


def transfer(pause_on_pad):
    e = Episode()
    e.move(ON_PAD)
    e.wait(pause_on_pad)
    return e.move(IN_RIGHT)


def test_doing_nothing_scores_zero_and_keeps_the_vial_cold():
    e = Episode()
    assert e.wait(10) == 0.0 and M.seconds_outside() == 0.0


def test_a_quick_transfer_scores_one():
    assert transfer(0.0) == 1.0 and M.seconds_outside() < 8


@pytest.mark.parametrize("pause, expected", [(20.0, 0.5), (32.0, 0.25), (45.0, 0.0)])
def test_slower_transfers_score_less(pause, expected):
    assert transfer(pause) == expected


def test_lifted_above_its_box_counts_as_outside():
    e = Episode()
    e.move(START + (0.0, 0.0, 0.08), lift=0.0)
    e.wait(2.0)
    assert M.seconds_outside() > 2.0


def test_a_vial_left_on_the_pad_scores_zero():
    e = Episode()
    assert e.move(ON_PAD) == 0.0


def test_a_vial_off_the_table_scores_zero():
    e = Episode()
    assert e.move(np.array((0.25, 0.0, M.TOP - 0.20)), lift=0.0) == 0.0


def test_a_reset_starts_a_new_episode():
    e = Episode()
    e.move(ON_PAD)
    e.wait(30)
    Episode(fresh=False)  # the embodiment resets: the vial jumps back into the left box, at rest
    assert M.seconds_outside() == 0.0


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


def test_the_demo_replayed_through_the_scorer_scores_one_with_time_to_spare():
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
                new = np.asarray(arms[arm].target(t)[0], float)  # the grip is at the vial's centre
        score = TASK.score({"sample_vial": state(new, (new - pos) / DT)})
        pos = new
    assert score == 1.0
    assert 5.0 < M.seconds_outside() < 16.0  # at least 4 s inside the 20 s limit


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(arm):
    a = ArmPlan(TASK.demo()[arm], station.BASES[arm])
    actions = np.array([a.action7((k + 1) / CONTROL_HZ)[0] for k in range(round(a.duration * CONTROL_HZ) + 5)])
    assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
