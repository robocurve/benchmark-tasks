"""Set a closing travel stop and demonstrate its actual travel on a passive parallel-jaw rig."""
from dataclasses import dataclass
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from bench import (
    OPEN,
    TABLE,
    TIP,
    HOVER,
    Track,
    waypoints,
    bodies,
    contacts,
    rig,
    moving,
    register_trial,
    return_spring,
    move_pick,
    move_place,
    proof,
)

from touch import stop_contact

SCENE = "station"
SCORE_OVER = "end"
MAX_STEPS = 850
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Fit the blue closing-stop block. Close the spring-loaded jaw to the stop, then let it reopen."
BASE = (0.294, -0.082, TABLE)
SEAT = (BASE[0] + 0.001, BASE[1] + 0.035, TABLE + 0.015)
BLUE = (0.221, -0.209, TABLE)
GRAY = (0.362, -0.209, TABLE)
PAD_Z = 0.031
KNOB = (BASE[0] - 0.050, BASE[1], TABLE + 0.046)
TRAVEL = 0.029
state = {}


@dataclass
class Trial:
    verified: bool = False
    over: bool = False
    hold: float = 0.0
    score: float = 0.0

    def step(self, seated, q, touch, against_stop, released, dt):
        self.over |= seated and q > TRAVEL + 0.004
        self.verified |= seated and touch and against_stop and TRAVEL - 0.003 <= q <= TRAVEL + 0.003
        ready = seated and self.verified and q < 0.003 and released and not self.over
        self.hold = self.hold + dt if ready else 0.0
        self.score = (
            0.0
            if self.over
            else 1.0
            if self.hold >= 1.0 - 1e-8
            else 0.65
            if seated and self.verified
            else 0.30
            if seated
            else 0.0
        )


_trial = Trial()


def score(objects):
    return _trial.score


def success(objects):
    return _trial.score == 1.0


def telemetry():
    return dict(
        state,
        verified=_trial.verified,
        overtravel=_trial.over,
        hold_s=round(_trial.hold, 2),
    )


def setup():
    items = [
        rig(BASE, (0.147, 0.107, 0.061), {"close": 0.0}, 2.2, 0.0, 6.2),
        moving("blue_stop", BLUE, (0.026, 0.022, 0.041), 0.038),
        moving("gray_stop", GRAY, (0.018, 0.022, 0.041), 0.030),
    ]

    def sample(env):
        return_spring(env, "close", 132.0, 6.2)
        units = bodies(env, ["rig", "blue_stop"])
        blue = units["blue_stop"]
        q = units["rig"].joints["close"]
        d = blue.pos - np.array(SEAT)
        seated = (
            np.linalg.norm(d[:2]) < 0.004
            and abs(d[2]) < 0.004
            and blue.rotation()[2, 2] > 0.988
        )
        hit = contacts(env, ["blue_stop", "rig/moving_jaw"])
        against_stop = seated and 0.025 <= q <= 0.033 and stop_contact(env)
        state.update(
            stop_mm=np.round(1000 * d, 1).tolist(),
            stroke_mm=round(1000 * q, 1),
            seated=bool(seated),
            jaw_touched=bool(hit["rig/moving_jaw"]),
            against_stop=bool(against_stop),
            released=not any(hit.values()),
        )
        return seated, q, hit["rig/moving_jaw"], against_stop, not any(hit.values())

    return register_trial(items, Trial, state, sample, globals())


def tracks():
    l, r = Track(), Track()
    move_pick(r, BLUE[:2], BLUE[2] + PAD_Z, -0.007, "specified blue spacer")
    # The upper grip head sits slightly ahead of the gauge datum during carry.
    drop = (SEAT[0] + 0.0032, SEAT[1] + 0.0028)
    move_place(r, drop, SEAT[2] + PAD_Z, -0.007, "keyed stop pocket")
    approach = (KNOB[0] - 0.028, KNOB[1], KNOB[2] + TIP - 0.001)
    close = (KNOB[0] + TRAVEL - 0.003, approach[1], approach[2])
    r.go((*approach[:2], HOVER), 0.0, "above the approach side of the jaw paddle", 1.4)
    r.go(approach, 0.0, "lower closed fingertips in front of the paddle", 1.2)
    r.go(close, 0.0, "push the jaw against its hard stop", 2.7)
    r.go(close, 0.0, "hold the jaw against the block", 0.4)
    r.go(approach, 0.0, "back away for spring return", 1.45)
    r.go((*approach[:2], HOVER), OPEN, "clear the jaw paddle", 1.35)
    l.go((0.172, 0.194, HOVER), OPEN, "park clear of gripper fixture")
    l.wait_until(r.duration)
    return {"left": l, "right": r}


def demo():
    t = tracks()
    return waypoints(t["left"], t["right"])


if __name__ == "__main__":
    t = Trial()
    t.step(True, 0.029, True, False, False, 0.05)
    assert t.score == 0.30
    t.step(True, 0.029, True, True, False, 0.05)
    t.step(True, 0.0, False, False, True, 1.0)
    assert t.score == 1.0
    t.step(True, 0.039, True, False, False, 0.05)
    assert t.score == 0.0 and t.over
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
