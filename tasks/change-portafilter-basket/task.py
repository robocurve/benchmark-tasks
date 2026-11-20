"""Use a bench ejector fixture to change a Breville-size bottomless portafilter basket."""
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
    fixed,
    moving,
    register_trial,
    move_pick,
    move_place,
    proof,
)
from runtime import asset, install_writer

SCENE = "station"
SCORE_OVER = "end"
MAX_STEPS = 1270
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Remove the one-cup basket using the service lever. Install the two-cup basket evenly in the portafilter and release it."
BASE = (0.283, -0.023, TABLE)
SEAT = (*BASE[:2], TABLE + 0.068)
START = (0.229, -0.225, TABLE + 0.031)
DISCARD = (0.253, 0.193, TABLE)
LEVER = (BASE[0] - 0.113, BASE[1], TABLE + 0.047)
OLD_DEPTH = 0.024
NEW_DEPTH = 0.029
state = {}
_drive = None


@dataclass
class Trial:
    ejected: bool = False
    discarded: bool = False
    hold: float = 0.0
    value: float = 0.0

    def step(self, angle, old_lift, stowed, seated, released, dt):
        self.ejected |= angle < -0.15 and old_lift > 0.0045
        self.discarded = self.ejected and stowed
        self.hold = (
            self.hold + dt
            if self.discarded and seated and released and angle > -0.045
            else 0.0
        )
        self.value = (
            1.0
            if self.hold >= 1.0 - 1e-8
            else 0.65
            if self.discarded and seated
            else 0.35
            if self.discarded
            else 0.15
            if self.ejected
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
        ejected=_trial.ejected,
        discarded=_trial.discarded,
        hold_s=round(_trial.hold, 2),
    )


def setup():
    parts = [
        rig(BASE, (0.330, 0.145, 0.077), {"lever": 0.0}, 0.023, 0.0, 0.125),
        moving("single_basket", SEAT, (0.056, 0.056, OLD_DEPTH), 0.047),
        moving("double_basket", START, (0.056, 0.056, NEW_DEPTH), 0.061),
        fixed("basket_tray", DISCARD, (0.083, 0.081, 0.014)),
    ]

    def sample(env):
        global _drive
        unit = asset(env, "rig")
        if _drive is not unit:
            index = unit.joint_names.index("lever")

            def detent():
                import torch

                q = unit.data.joint_pos
                q = getattr(q, "torch", q)[:, index : index + 1]
                stop = torch.where(q < -0.17, -0.335, 0.0)
                unit.set_joint_effort_target(
                    (0.31 * (stop - q)).clamp(-0.095, 0.095), joint_ids=[index]
                )

            install_writer(unit, detent)
            _drive = unit
        obj = bodies(env, ["rig", "single_basket", "double_basket"])
        old, new = obj["single_basket"], obj["double_basket"]
        q = obj["rig"].joints["lever"]
        d = new.pos - np.array(SEAT)
        hit = contacts(env, ["single_basket", "double_basket", "rig/lever_body"])
        stowed = (
            np.linalg.norm(old.pos[:2] - DISCARD[:2]) < 0.025
            and TABLE + 0.023 < old.pos[2] < TABLE + 0.039
            and not hit["single_basket"]
        )
        seated = (
            np.linalg.norm(d[:2]) < 0.0036
            and abs(d[2]) < 0.0032
            and new.rotation()[2, 2] > 0.99
        )
        state.update(
            lever_deg=round(float(np.degrees(q)), 1),
            seat_error_mm=np.round(1000 * d, 1).tolist(),
        )
        return (
            q,
            old.pos[2] - SEAT[2],
            stowed,
            seated,
            not any(hit.values()) and new.resting(0.021, 0.23),
        )

    return register_trial(parts, Trial, state, sample, globals())


def tracks():
    l, r = Track(), Track()
    press = (LEVER[0] + 0.009, LEVER[1], TABLE + 0.0405)
    r.go((*press[:2], HOVER), 0.0, "above ejector pad", 1.35)
    r.go(press, 0.0, "press ejector past its detent", 2.0)
    r.go((*press[:2], HOVER), 0.0, "clear detented service lever", 1.4)
    r.go((0.181, -0.189, HOVER), OPEN, "park right hand", 1.5)
    l.go((0.180, 0.207, HOVER), OPEN, "wait clear of ejector", 1.4)
    l.wait_until(r.duration + 0.25)
    rim = (*SEAT[:2], SEAT[2] + 0.007 + TIP - 0.003)
    l.go((*rim[:2], HOVER), OPEN, "above raised basket", 1.45)
    l.go(rim, OPEN, "lower around exposed rim", 1.25)
    l.go(rim, -0.020, "hold the raised one-cup rim", 0.9)
    l.go((*rim[:2], HOVER), -0.020, "lift one-cup basket away", 1.35)
    move_place(l, DISCARD[:2], DISCARD[2] + OLD_DEPTH + 0.005, -0.020, "discard tray")
    l.go((0.179, 0.210, HOVER), OPEN, "park left arm clear", 1.45)
    r.wait_until(l.duration)
    grip = (press[0] - 0.004, press[1], TABLE + 0.0355)
    r.go((*grip[:2], HOVER), OPEN, "above low lever grip", 1.3)
    r.go(grip, OPEN, "take the lever return grip", 1.2)
    r.go(grip, -0.007, "grasp the ejector handle", 0.75)
    r.go((LEVER[0], LEVER[1], LEVER[2] + TIP), -0.007, "raise lever out of detent", 1.4)
    r.go((LEVER[0], LEVER[1], LEVER[2] + TIP), OPEN, "release the returned lever", 0.70)
    r.go((LEVER[0], LEVER[1], HOVER), OPEN, "clear lever", 1.15)
    move_pick(r, START[:2], START[2], -0.021, "two-cup basket")
    move_place(r, SEAT[:2], SEAT[2], -0.021, "portafilter retaining rim")
    r.go((0.191, -0.214, HOVER), OPEN, "park clear", 1.35)
    l.wait_until(r.duration)
    return {"left": l, "right": r}


def demo():
    t = tracks()
    return waypoints(t["left"], t["right"])


if __name__ == "__main__":
    t = Trial()
    t.step(0, 0.020, True, True, True, 1.0)
    assert t.value == 0
    t.step(-0.22, 0.011, False, False, False, 0.05)
    assert t.value == 0.15
    t.step(0.0, 0.02, True, True, True, 1.0)
    assert t.value == 1.0
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
