"""Scoring counterexamples, run offline: every tier, both sides of each limit, and the shortcuts."""
import copy
import math

import numpy as np

import parts
import task

KEYS = ("_known", "_invalid", "_touched", "_left_mat", "_held", "_stamped")


def states(rel=(0.0, 0.0, task.SEAT_Z), cup_z=None, cup_xy=(0.30, 0.05), lips=(0.02, 0.02), v=0.0, w=0.0,
           launcher_yaw=task.LAUNCHER_YAW, ball_pos=None):
    """The launcher (cup centre at cup_xy, cup_z) and the ball at ``rel`` in the launcher frame."""
    cup_z = task.BALL_Z - task.SEAT_Z + 0.075 if cup_z is None else cup_z
    rot = parts._rot((0, 0, 1), launcher_yaw)
    lpos = np.array([cup_xy[0], cup_xy[1], cup_z])
    launcher = task.LAUNCHER.state(joints={"lip_tip": lips[0], "lip_shaft": lips[1]}, pos=lpos,
                                   quat_xyzw=parts._matrix_to_quat(rot))
    bpos = lpos + rot @ np.asarray(rel) if ball_pos is None else np.asarray(ball_pos, float)
    ball = task.BALL.state(pos=bpos, lin_vel=(v, 0.0, 0.0))
    ball = parts.PartState(**{**ball.__dict__, "ang_vel": np.array([0.0, 0.0, w])})
    return {"ball": ball, "ball_launcher": launcher, "lawn_mat": task.MAT.state(), "edging_board": task.BOARD.state()}


def grade(objs, held=2.0, known=True, invalid=False, touched=False, left_mat=False, stale=False):
    task._known, task._invalid, task._touched, task._left_mat, task._held = known, invalid, touched, left_mat, held
    task._stamped = () if stale else task._stamp(objs["ball"], objs["ball_launcher"])
    return task.score(objs)


# check.py runs these against the task's own history, which is unobserved before any simulator step: all score 0.
CASES = [
    ("seated and lifted, but no observed history", {k: v for k, v in states().items() if k in ("ball", "ball_launcher")},
     0.0, False),
    ("ball pushed off the start, no cup", {"ball": task.BALL.state(pos=(0.30, 0.10, task.BALL_Z))}, 0.0, False),
    ("ball thrown off the mat", {"ball": task.BALL.state(pos=(0.50, 0.10, task.TOP + task.R))}, 0.0, False),
]

SEAT_C = task.BALL_Z - task.SEAT_Z  # cup centre with the ball seated on the mat
LIFT_EDGE = task.MAT_TOP + task.LIFT + task.R - task.SEAT_Z  # cup centre putting the ball bottom exactly 60 mm up


class _Sim:
    """Stands in for the simulator handle: a fixed hand far from the ball, and the given states."""

    def __init__(self, objs, t):
        self.objs, self.t = objs, t

    def states(self):
        return self.objs


def drift_cases():
    """The hold timer on a played-back sequence: a held ball that drifts more than 2 mm restarts the hold."""
    out = []
    real = task.observation.hand_state
    task.observation.hand_state = lambda sim: ([np.array([[0.0, 0.5, 1.5]])], 10.0)
    try:
        for label, drift, want in (("held ball drifts 1.9 mm over 2.5 s: 1.0", 0.0019, 1.0),
                                   ("held ball drifts 2.1 mm over 2.5 s: 0.5", 0.0021, 0.5)):
            task._reset(None)
            objs = None
            for k in range(1, 51):
                d = drift * min(1.0, k / 50)
                cup = (0.30 + d, 0.05)
                objs = states(cup_xy=cup)
                task._step(_Sim(objs, 0.05 * k))
            out.append((label, task.score(objs) == want))
    finally:
        task.observation.hand_state = real
        task._reset(None)
    return out


def run():
    old = {k: copy.deepcopy(getattr(task, k)) for k in KEYS}
    r = []
    try:
        r.append(("held up, still 2 s: 1.0", grade(states()) == 1.0))
        r.append(("held up but only 1.9 s: 0.5", grade(states(), held=1.9) == 0.5))
        r.append(("held up exactly 2.0 s: 1.0", grade(states(), held=2.0) == 1.0))
        r.append(("held up, no hold timed yet: 0.5", grade(states(), held=0.0) == 0.5))
        r.append(("ball bottom exactly 60 mm up: 1.0", grade(states(cup_z=LIFT_EDGE)) == 1.0))
        r.append(("ball bottom 59.5 mm up: 0.5", grade(states(cup_z=LIFT_EDGE - 0.0005)) == 0.5))
        r.append(("seated on the mat, cup pressed down: 0.5", grade(states(cup_z=SEAT_C), held=0.0) == 0.5))
        r.append(("seated, launcher turned any yaw: 1.0", grade(states(launcher_yaw=0.7)) == 1.0))
        r.append(("ball at the seat band edge, x 4 mm: 1.0", grade(states(rel=(0.004, 0, task.SEAT_Z))) == 1.0))
        r.append(("ball 4.5 mm off along the shaft: 0", grade(states(rel=(0.0045, 0, task.SEAT_Z))) == 0.0))
        r.append(("ball 6.5 mm off across the cup: 0", grade(states(rel=(0, 0.0065, task.SEAT_Z))) == 0.0))
        r.append(("ball hanging 3.5 mm low on the teeth: 0",
                  grade(states(rel=(0, 0, task.SEAT_Z - 0.0035))) == 0.0))
        r.append(("one lip still sprung open (0.13 rad): 0", grade(states(lips=(0.13, 0.0))) == 0.0))
        r.append(("lips at the shut limit 0.12 rad: 1.0", grade(states(lips=(0.12, 0.12))) == 1.0))
        r.append(("cup lowered over the ball but ball not in it: 0",
                  grade(states(rel=(0, 0, -0.015), cup_z=task.BALL_Z + 0.015)) == 0.0))
        r.append(("ball pushed along the mat, cup beside it: 0",
                  grade(states(rel=(0.06, 0, -0.03), cup_z=task.BALL_Z + 0.03)) == 0.0))
        r.append(("ball touched by a finger at any time: 0", grade(states(), touched=True) == 0.0))
        r.append(("ball left the mat at any time: 0", grade(states(), left_mat=True) == 0.0))
        r.append(("hand state unknown: 0", grade(states(), known=False) == 0.0))
        r.append(("a bad step earlier: 0", grade(states(), invalid=True) == 0.0))
        r.append(("stale state: 0", grade(states(), stale=True) == 0.0))
        r.append(("ball off the mat edge now: 0",
                  grade(states(ball_pos=(task.MAT_X[1] + 0.001, 0.0, task.BALL_Z), rel=None)) == 0.0))
        # the touch rule, on sampled points
        ball = states()["ball"]
        c = np.asarray(ball.pos)
        r.append(("hand point 4.9 mm from the ball counts as a touch",
                  task.hand_gap([np.array([c + (task.R + 0.0049, 0, 0)])], ball) <= task.TOUCH))
        r.append(("hand point 5.1 mm away does not",
                  task.hand_gap([np.array([c + (0, task.R + 0.0051, 0)])], ball) > task.TOUCH))
        r.append(("NaN hand points fail closed", task.hand_gap([np.full((3, 3), np.nan)], ball) == -math.inf))
        r.append(("no hand points fail closed", task.hand_gap([], ball) == -math.inf))
        r.append(("ball on the mat at its start is on the mat", task.on_mat(task.BALL.state())))
        r.extend(drift_cases())
    finally:
        for k, v in old.items():
            setattr(task, k, v)
    for name, ok in r:
        print("ok  " if ok else "FAIL", name)
    return sum(not ok for _, ok in r)


if __name__ == "__main__":
    raise SystemExit(run())
