"""Offline checks for apart.py (no Isaac Sim): every scoring outcome, the demo's reach, and the demo replayed through
the scorer (each bottle riding in the gripper between grasp and release).

    python -m pytest test_apart.py -q
"""

from pathlib import Path

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("apart.py"))
M = TASK.module
DT = 1.0 / 20  # the runs' control rate
H = M.BOTTLE_H
SIZE = (2 * M.BOTTLE_R, 2 * M.BOTTLE_R, H)
SIZES = {"bleach": SIZE, "ammonia": SIZE}


def at(xy, surface):
    return np.array((xy[0], xy[1], surface + H / 2))


START = {"bleach": at(M.BLEACH_XY, M.TOP), "ammonia": at(M.AMMONIA_XY, M.TOP)}
MID_B, MID_A = at(M.MID_FOR_BLEACH, M.MAT_TOP), at(M.MID_FOR_AMMONIA, M.MAT_TOP)
ON_RIGHT, ON_LEFT = at(M.RIGHT_MAT.pos[:2], M.MAT_TOP), at(M.LEFT_MAT.pos[:2], M.MAT_TOP)
TIPPED = (np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4))  # lying on its side


def state(pos, size, vel=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0)):
    pos, half = np.asarray(pos, dtype=float), np.asarray(size, dtype=float) / 2
    return ObjectState(pos, np.asarray(quat, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


class Episode:
    """Plays object motion through score() one 20 Hz control step at a time, as the embodiment does."""

    def __init__(self, fresh=True):
        if fresh:
            M._state.clear()
            M._state.update(M._fresh_state())
        self.pos = {n: p.copy() for n, p in START.items()}
        self.quat = {n: (0.0, 0.0, 0.0, 1.0) for n in START}
        self.last = self.step()

    def step(self, vel=None):
        vel = vel or {}
        f = {n: state(self.pos[n], SIZES[n], vel.get(n, (0.0, 0.0, 0.0)), self.quat[n]) for n in self.pos}
        self.last = TASK.score(f)
        return self.last

    def move(self, goals, lift=0.10, speed=0.2):
        """Carry the named objects together to their goals, lifted on the way, then let them settle."""
        names = list(goals)
        up = np.array((0.0, 0.0, lift))
        paths = {n: [self.pos[n] + up, np.asarray(goals[n], float) + up, np.asarray(goals[n], float)] if lift
                 else [np.asarray(goals[n], float)] for n in names}
        for leg in range(len(paths[names[0]])):
            starts = {n: self.pos[n].copy() for n in names}
            ends = {n: paths[n][leg] for n in names}
            steps = max(1, int(np.ceil(max(np.linalg.norm(ends[n] - starts[n]) for n in names) / (speed * DT))))
            vel = {n: (ends[n] - starts[n]) / (steps * DT) for n in names}
            for k in range(1, steps + 1):
                for n in names:
                    self.pos[n] = starts[n] + (ends[n] - starts[n]) * k / steps
                self.step(vel)
        for _ in range(3):
            self.step()
        return self.last


def test_doing_nothing_scores_zero():
    e = Episode()
    for _ in range(40):
        e.step()
    assert e.last == 0.0


def test_one_at_a_time_scores_one():
    e = Episode()
    e.move({"bleach": MID_B})
    e.move({"bleach": ON_RIGHT})
    e.move({"ammonia": MID_A})
    assert e.move({"ammonia": ON_LEFT}) == 1.0 and not M.violated()


def test_both_through_the_middle_at_once_scores_zero_even_when_finished():
    e = Episode()
    e.move({"bleach": MID_B, "ammonia": MID_A})
    assert M.violated()
    e.move({"bleach": ON_RIGHT, "ammonia": ON_LEFT})
    assert e.last == 0.0


def test_carrying_one_over_the_middle_while_the_other_stands_there_is_a_violation():
    e = Episode()
    e.move({"bleach": MID_B})
    e.move({"ammonia": MID_A})  # set down beside it
    assert e.last == 0.0 and M.violated()


def test_one_bottle_done_scores_half():
    e = Episode()
    e.move({"bleach": MID_B})
    assert e.move({"bleach": ON_RIGHT}) == 0.5


def test_a_bottle_staged_in_the_middle_scores_a_quarter():
    assert Episode().move({"bleach": MID_B}) == 0.25


def test_a_tipped_bottle_does_not_count_as_done():
    e = Episode()
    e.move({"bleach": MID_B})
    e.move({"bleach": ON_RIGHT})
    e.quat["bleach"] = TIPPED
    assert e.step() == 0.0


def test_a_bottle_off_the_table_scores_zero():
    e = Episode()
    e.move({"bleach": MID_B})
    e.move({"bleach": ON_RIGHT})
    assert e.move({"ammonia": np.array((0.20, -0.50, M.TOP - 0.20))}, lift=0.0) == 0.0


def test_the_separation_margin_is_tracked():
    e = Episode()
    e.move({"bleach": MID_B})
    assert M.min_separation() == pytest.approx(np.linalg.norm(MID_B[:2] - START["ammonia"][:2]), abs=1e-6)


def test_a_reset_starts_a_new_episode():
    e = Episode()
    e.move({"bleach": MID_B, "ammonia": MID_A})
    assert M.violated()
    Episode(fresh=False)  # the embodiment resets: the bottles jump back to their starts, at rest
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


def demo_rollout(carries, grip_offset):
    """Replay the demo's TCP paths with each object riding in the gripper between grasp and release; score each step."""
    M._state.clear()
    M._state.update(M._fresh_state())
    plan = TASK.demo()
    arms = {a: ArmPlan(plan[a], station.BASES[a]) for a in plan}
    holds = []
    for arm, names in carries.items():
        ivs = grasp_intervals(plan[arm])
        assert len(ivs) == len(names)
        holds += [(name, arm, t0, t1) for name, (t0, t1) in zip(names, ivs)]
    pos = {n: p.copy() for n, p in START.items()}
    steps = int(round((max(a.duration for a in arms.values()) + 2.0) / DT))
    scores = []
    for k in range(steps):
        t = (k + 1) * DT
        new = dict(pos)
        for name, arm, t0, t1 in holds:
            if t0 <= t <= t1:
                new[name] = np.asarray(arms[arm].target(t)[0], float) + grip_offset
        scores.append(TASK.score({n: state(new[n], SIZES[n], (new[n] - pos[n]) / DT) for n in new}))
        pos = new
    return scores, pos


def test_the_demo_replayed_through_the_scorer_reaches_one_without_a_violation():
    grip_offset = np.array((0.0, 0.0, -(H / 2 - M.GRIP_BELOW_TOP)))  # the grip is just below the top
    scores, final = demo_rollout({"left": ["bleach", "ammonia"], "right": ["bleach", "ammonia"]}, grip_offset)
    assert scores[-1] == 1.0 and not M.violated()
    assert M.min_separation() > 0.18  # the layout keeps them about 20 cm apart even outside the middle area
    assert len(scores) < 2400  # well inside the run limit


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(arm):
    a = ArmPlan(TASK.demo()[arm], station.BASES[arm])
    actions = np.array([a.action7((k + 1) / CONTROL_HZ)[0] for k in range(round(a.duration * CONTROL_HZ) + 5)])
    assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
