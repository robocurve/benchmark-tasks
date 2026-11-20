"""Fit a linear-axis interrupt flag, then verify the Omron fork's home trigger."""
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
    bodies,
    contacts,
    rig,
    fixed,
    moving,
    register_trial,
    move_pick,
    move_place,
    proof,
)

from plan import steered

SCENE = "station"
SCORE_OVER = "end"
MAX_STEPS = 930
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Fit the homing flag to the carriage. Slide the carriage toward the sensor to verify that the flag triggers home, then let go."
BASE = (0.278, -0.119, TABLE)
FLAG = (0.229, -0.224, TABLE + 0.023)
SEAT = (BASE[0] + 0.024, BASE[1], TABLE + 0.039)
KNOB = (BASE[0] - 0.032, BASE[1], TABLE + 0.052)
STROKE = 0.063
TRIGGER_AT = 0.044
BLUE = (0.11, 0.47, 0.75)
state = {}


@dataclass
class Trial:
    prior: bool = False
    home: bool = False
    invalid: bool = False
    hold: float = 0.0
    value: float = 0.0

    def step(self, seated, beam, travel, released, dt):
        if beam and not self.prior:
            if seated and 0.036 <= travel <= 0.061:
                self.home = True
            else:
                self.invalid = True
        self.prior = beam
        self.hold = (
            self.hold + dt
            if seated and self.home and released and travel > 0.054
            else 0.0
        )
        self.value = (
            0.0
            if self.invalid
            else 1.0
            if self.hold >= 1.0 - 1e-8
            else 0.65
            if seated and self.home
            else 0.30
            if seated
            else 0.0
        )


_trial = Trial()


def score(objects):
    return _trial.value


def success(objects):
    return _trial.value == 1.0


def telemetry():
    return dict(
        state,
        triggered=_trial.home,
        hold_s=round(_trial.hold, 2),
        invalid=_trial.invalid,
    )


def setup():
    parts = [
        rig(BASE, (0.152, 0.176, 0.070), {"axis": 0.0}, 0.35, 0.0, 5.0),
        moving("flag", FLAG, (0.032, 0.027, 0.077), 0.022),
        fixed("flag_rest", (*FLAG[:2], TABLE), (0.041, 0.041, 0.023)),
    ]

    def sample(env):
        units = bodies(env, ["rig", "flag"])
        flag = units["flag"]
        q = units["rig"].joints["axis"]
        tol = flag.pos - (np.array(SEAT) + (0, q, 0))
        seated = bool(
            np.linalg.norm(tol[:2]) < 0.0045
            and abs(tol[2]) < 0.004
            and flag.rotation()[2, 2] > 0.987
        )
        rotation = flag.rotation()
        blade = flag.pos + rotation @ np.array((0.018, 0.0, -0.010))
        beam = (
            abs(blade[0] - (SEAT[0] + 0.018)) < 0.0023
            and abs(blade[2] - (SEAT[2] - 0.010)) < 0.005
            and abs(blade[1] - (SEAT[1] + 0.050)) < 0.007
        )
        touch = contacts(env, ["flag", "rig/carriage"])
        state.update(
            travel_mm=round(1000 * q, 1),
            seat_error_mm=np.round(1000 * tol, 1).tolist(),
            beam=bool(beam),
            flag_up=rotation[:, 2].tolist(),
        )
        return seated, beam, q, not any(touch.values())

    return register_trial(parts, Trial, state, sample, globals())


def tracks():
    l, r = Track(), Track()
    move_pick(r, FLAG[:2], FLAG[2] + 0.045, -0.007, "flag carrier")
    move_place(r, SEAT[:2], SEAT[2] + 0.045, -0.007, "carriage keyway")
    start = (KNOB[0], KNOB[1] - 0.028, KNOB[2] + TIP)
    end = (KNOB[0], KNOB[1] + STROKE - 0.012, start[2])
    r.go((*start[:2], HOVER), 0.0, "above the approach side of carriage handhold", 1.4)
    r.go(start, 0.0, "approach with closed jaws", 1.3)
    r.go(end, 0.0, "push carriage through optical fork", 3.4)
    r.go((end[0], end[1] - 0.014, end[2]), 0.0, "back away from carriage", 0.9)
    r.go((*end[:2], HOVER), OPEN, "clear carriage", 1.3)
    l.go((0.169, 0.209, HOVER), OPEN, "park left away from linear rail")
    l.wait_until(r.duration)
    return {"left": l, "right": r}


def steer_flag(label, pos, rotation, finger):
    if label not in ("seat into carriage keyway", "release at carriage keyway", "clear carriage keyway"):
        return pos, rotation, finger
    trim = np.asarray(state.get("seat_trim_mm", [0.0, 0.0]))
    error = np.asarray(state.get("seat_error_mm", [0.0, 0.0, 0.0]))
    if label == "seat into carriage keyway" and np.linalg.norm(error[:2]) < 16:
        trim = np.clip(trim - np.clip(0.15 * error[:2], -0.65, 0.65), -8, 8)
        state["seat_trim_mm"] = trim.tolist()
    return pos + np.array([*trim, 0.0]) * 0.001, rotation, finger


def demo():
    t = tracks()
    return steered(t["left"], t["right"], steer_flag)


if __name__ == "__main__":
    t = Trial()
    t.step(True, True, 0.042, False, 0.05)
    t.step(True, False, 0.061, True, 1.0)
    assert t.value == 1.0
    t = Trial()
    t.step(True, True, 0.010, True, 1.0)
    assert t.value == 0 and t.invalid
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
