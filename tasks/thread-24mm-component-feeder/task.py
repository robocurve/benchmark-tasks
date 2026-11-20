"""Thread a leader, then verify three indexed component pockets on a cold feeder bench."""
from dataclasses import dataclass
import numpy as np
from bench import TABLE, TIP, OPEN, HOVER, Track, rig, moving, bodies, contacts, register_trial, proof, waypoints
from runtime import asset, install_writer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 820
INSTRUCTION = "Guide the cover film into the peel gears. Press and release the blue feed button three times to present three successive occupied pockets."
BASE = (0.294, -0.119, TABLE)
POCKETS = (0.006, -0.010, -0.026)
PITCH = 0.016
STATE = {}


@dataclass
class Trial:
    count: int = 0
    target: int = 0
    prior_pressed: bool = False
    lost: bool = False
    jammed: bool = False
    dwell: float = 0.0
    elapsed: float = 0.0
    hold: float = 0.0
    value: float = 0.0

    def step(self, film, button, travel, errors, retained, free, dt):
        threaded = 0.020 <= film <= 0.025
        self.lost |= not retained
        pressed = button >= 0.006
        if threaded and pressed and not self.prior_pressed and self.target == self.count and self.target < 3:
            self.target += 1
            self.elapsed = 0
        self.prior_pressed = pressed
        if self.target > self.count:
            self.elapsed += dt
            aligned = abs(travel - self.target * PITCH) <= 0.001 and abs(errors[self.count]) <= 0.001
            self.dwell = self.dwell + dt if aligned else 0.0
            if self.dwell >= 0.20 - 1e-8:
                self.count += 1
                self.dwell = 0.0
            elif self.elapsed > 2.0:
                self.jammed = True
        good = threaded and self.count == 3 and free and not pressed and abs(errors[2]) <= 0.001
        self.hold = self.hold + dt if good else 0.0
        self.value = (
            0.0 if self.lost or self.jammed or not threaded
            else 1.0 if self.hold >= 1 - 1e-8 else min(0.75, 0.25 * (self.count + 1))
        )


_trial = Trial()


def score(objects): return _trial.value
def success(objects): return _trial.value == 1.0
def telemetry(): return dict(STATE, count=_trial.count, target=_trial.target, jammed=_trial.jammed, lost=_trial.lost)


def chip_start(i):
    return (BASE[0] + POCKETS[i], BASE[1], TABLE + 0.015)


def setup():
    station = rig(BASE, (0.20, 0.167, 0.048), {"feed": 0.0, "index": 0.0, "button": 0.0},
                  0.0, 0.0, 5.0)
    parts = [station] + [moving(f"chip{i}", chip_start(i), (0.010, 0.010, 0.0006), 0.002) for i in range(3)]

    def sample(env):
        unit = asset(env, "rig")
        if not getattr(unit, "_feed_motor", False):
            index = unit.joint_names.index("index")
            button = unit.joint_names.index("button")

            def advance():
                q = unit.data.joint_pos.torch
                v = unit.data.joint_vel.torch
                effort = q.new_zeros(q.shape)
                effort[:, index] = (110 * (_trial.target * PITCH - q[:, index]) - 5 * v[:, index]).clamp(-1.3, 1.3)
                effort[:, button] = (-200 * q[:, button] - 3 * v[:, button]).clamp(-3, 3)
                unit.set_joint_effort_target(effort)

            install_writer(unit, advance)
            unit._feed_motor = True
        readings = bodies(env, ["rig", "chip0", "chip1", "chip2"])
        q = readings["rig"].joints
        positions = [readings[f"chip{i}"].pos for i in range(3)]
        expected = [np.array(chip_start(i)) + (q["index"], 0, 0) for i in range(3)]
        retained = all(np.linalg.norm((p - e)[:2]) <= 0.004 and abs(p[2] - e[2]) <= 0.004
                       for p, e in zip(positions, expected))
        errors = [float(p[0] - (BASE[0] + 0.022)) for p in positions]
        hands = contacts(env, ["rig/film", "rig/button_body", "chip0", "chip1", "chip2"])
        STATE.update(nip_travel_mm=round(q["feed"] * 1000, 2), tape_travel_mm=round(q["index"] * 1000, 2),
                     button_mm=round(q["button"] * 1000, 2), errors_mm=np.round(np.array(errors) * 1000, 2).tolist(),
                     parts_retained=bool(retained), hands_clear=not any(hands.values()), hold_s=round(_trial.hold, 2))
        return q["feed"], q["button"], q["index"], errors, retained, not any(hands.values())

    return register_trial(parts, Trial, STATE, sample, globals())


def tracks():
    l, r = Track(), Track()
    at = (BASE[0] - 0.089, BASE[1], TABLE + 0.030 + TIP)
    target = (at[0] + 0.048, at[1], at[2])
    r.go((*at[:2], HOVER), OPEN, "above exposed film")
    r.go(at, 0.0, "approach orange tab")
    r.go(target, 0.0, "feed cover into the nip", 2.6)
    r.go(at, 0.0, "release film")
    r.go((0.161, -0.229, HOVER), OPEN, "park away from feeder")
    button = (BASE[0] - 0.064, BASE[1] + 0.142, TABLE + 0.037 + TIP)
    press = (button[0] + 0.036, button[1], button[2])
    l.go((*button[:2], HOVER), OPEN, "wait above feed button")
    l.wait_until(r.duration)
    l.go(button, 0.0, "approach button")
    for index in range(3):
        l.go(press, 0.0, f"press for pocket {index + 1}", 1.15)
        l.go(button, 0.0, "release indexed advance", 1.20)
    l.go((*button[:2], HOVER), OPEN, "clear controls")
    r.wait_until(l.duration)
    return {"left": l, "right": r}


def demo():
    path = tracks()
    return waypoints(path["left"], path["right"])


if __name__ == "__main__":
    import sys
    t = Trial()
    t.step(0, 0.007, 0.0, [-.016, -.032, -.048], True, False, 1)
    assert t.value == 0
    t.step(.024, 0, 0.0, [-.016, -.032, -.048], True, True, 1)
    assert t.value == .25
    t.step(.024, 0, 0.0, [-.016, -.032, -.048], False, True, 1)
    assert t.value == 0
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
