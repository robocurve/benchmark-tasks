"""Fit a replacement fascia and verify the center bumper switch and return."""
from dataclasses import dataclass
from pathlib import Path
import sys
from math import atan2, cos, sin

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
    moving,
    register_trial,
    return_spring,
    move_pick,
    proof,
)

from plan import steered

SCENE = "station"
SCORE_OVER = "end"
MAX_STEPS = 800
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Fit the replacement front bumper to the mobile-base fixture. Push the center of the bumper to verify the switch, then let it return."
BASE = (0.304, -0.070, TABLE)
SEAT = (BASE[0] - 0.042, BASE[1], TABLE + 0.030)
START = (0.210, -0.224, TABLE + 0.0054)
GRIP_Z = 0.034
SWITCH = 0.0048
MAX_STROKE = 0.010
state = {}


@dataclass
class Trial:
    tested: bool = False
    hold: float = 0.0
    value: float = 0.0

    def step(self, seated, travel, touched, released, dt):
        self.tested |= seated and touched and SWITCH <= travel <= MAX_STROKE + 0.001
        ready = seated and self.tested and abs(travel) < 0.0013 and released
        self.hold = self.hold + dt if ready else 0.0
        self.value = (
            1.0
            if self.hold >= 1.0 - 1e-8
            else 0.65
            if seated and self.tested
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
    return dict(state, tested=_trial.tested, hold_s=round(_trial.hold, 2))


def setup():
    parts = [
        rig(BASE, (0.172, 0.172, 0.072), {"stroke": 0.0}, 2.4, 0.0, 5.5),
        moving("bumper", START, (0.062, 0.132, 0.047), 0.061),
    ]

    def sample(env):
        return_spring(env, "stroke", 170.0, 5.5)
        obj = bodies(env, ["rig", "bumper"])
        bumper = obj["bumper"]
        q = obj["rig"].joints["stroke"]
        d = bumper.pos - (np.array(SEAT) + (q, 0, 0))
        rotation = bumper.rotation()
        offsets = (np.array([0.0, -0.042, 0.001]), np.array([0.0, 0.042, 0.001]))
        post_errors = [d + rotation @ offset - offset for offset in offsets]
        seated = all(
            np.linalg.norm(error[:2]) < 0.005 and abs(error[2]) < 0.004
            for error in post_errors
        ) and rotation[2, 2] > 0.986 and rotation[0, 0] > 0.98
        touch = contacts(env, ["bumper", "rig/sled"])
        state.update(
            switch=bool(q >= SWITCH),
            travel_mm=round(1000 * q, 1),
            mount_mm=np.round(1000 * d, 1).tolist(),
            post_error_mm=np.round(1000 * np.array(post_errors), 1).tolist(),
            mount_yaw=atan2(rotation[1, 0], rotation[0, 0]),
            mounted=bool(seated),
        )
        return seated, q, touch["bumper"], not any(touch.values())

    return register_trial(parts, Trial, state, sample, globals())


def tracks():
    l, r = Track(), Track()
    move_pick(r, START[:2], START[2] + GRIP_Z, -0.007, "mobile-base fascia")
    mount = (SEAT[0], SEAT[1], SEAT[2] + GRIP_Z + TIP - 0.002)
    lead = (mount[0], mount[1], mount[2] + 0.026)
    r.go((*SEAT[:2], HOVER), -0.007, "transport to bumper mounts", 1.6)
    r.go(lead, -0.007, "align both bumper posts", 1.55)
    r.go(lead, -0.007, "align both bumper posts", 1.1)
    r.go(mount, -0.007, "lower both bumper posts", 2.1)
    r.go(mount, -0.007, "lower both bumper posts", 0.65)
    r.go(mount, OPEN, "release mounted bumper", 0.8)
    r.go((*SEAT[:2], HOVER), OPEN, "lift clear of mounted bumper", 1.25)
    at = (SEAT[0] - 0.034, SEAT[1], SEAT[2] + 0.020 + TIP)
    r.go((*at[:2], HOVER), 0.0, "above center press pad", 1.4)
    r.go(at, 0.0, "approach the front of the bumper", 1.2)
    press = (at[0] + 0.020, at[1], at[2])
    r.go(press, 0.0, "push toward chassis until microswitch trips", 1.8)
    r.go(at, 0.0, "back away to let the bumper return", 1.25)
    r.go((*at[:2], HOVER), OPEN, "clear front fascia", 1.35)
    l.go((0.176, 0.196, HOVER), OPEN, "park clear of mobile base")
    l.wait_until(r.duration)
    return {"left": l, "right": r}


def steer_bumper(label, pos, rotation, finger):
    aligning = label in ("align both bumper posts", "lower both bumper posts")
    departing = label in ("release mounted bumper", "lift clear of mounted bumper")
    if not (aligning or departing):
        return pos, rotation, finger
    trim = np.array(state.get("mount_trim_mm", [0.0, 0.0]))
    yaw = state.get("yaw_trim", 0.0)
    if aligning and "mount_mm" in state:
        error = np.array(state["mount_mm"][:2])
        if np.linalg.norm(error) < 45:
            trim = np.clip(trim - np.clip(error * 0.13, -0.8, 0.8), -24, 24)
            yaw = float(np.clip(yaw - np.clip(state["mount_yaw"] * 0.09, -0.018, 0.018), -0.5, 0.5))
            state["mount_trim_mm"] = trim.tolist()
            state["yaw_trim"] = yaw
    if aligning and state.get("mounted"):
        state["release_mount"] = True
    if departing and not state.get("release_mount"):
        finger = -0.007
    c, s = cos(yaw), sin(yaw)
    adjust = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))
    return pos + np.array([*trim, 0.0]) * 0.001, adjust @ rotation, finger


def demo():
    t = tracks()
    return steered(t["left"], t["right"], steer_bumper)


if __name__ == "__main__":
    t = Trial()
    t.step(True, 0.007, False, False, 0.05)
    assert t.value == 0.30
    t.step(True, 0.006, True, False, 0.05)
    assert t.value == 0.65
    t.step(True, 0.0003, False, True, 1.0)
    assert t.value == 1.0
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
