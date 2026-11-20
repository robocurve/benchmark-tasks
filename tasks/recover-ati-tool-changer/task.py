"""Support the tool plate while pressing the manual release of a cold QC-001 fixture."""
from dataclasses import dataclass, replace
from math import pi
import numpy as np
from bench import TABLE, TIP, OPEN, HOVER, Track, bodies, contacts, rig, fixed, moving, register_trial, proof, waypoints
from runtime import asset, install_writer, wrench, to_tensor
from bench import return_spring

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 800
INSTRUCTION = "Support the tool plate. Use the handled ball-end key to press the release piston while supporting the plate. Withdraw the tool and park it in the stand."
BASE = (0.286, 0.0, TABLE)
MOUNT = (0.286, 0.025, TABLE + 0.022)
PARK = (0.286, 0.197, TABLE + 0.006)
UNLOCK = 0.007
STATE = {}


@dataclass
class Trial:
    carry_hold: float = 0.0
    opened: bool = False
    invalid: bool = False
    release_z: float = 0.0
    watch_s: float = 0.0
    max_drop: float = 0.0
    hold: float = 0.0
    value: float = 0.0

    def step(self, stroke, key, supported, xyz, parked, dt):
        self.carry_hold = self.carry_hold + dt if supported else 0.0
        if not self.opened and stroke >= UNLOCK:
            self.opened = True
            self.release_z = xyz[2]
            self.invalid |= not key or self.carry_hold < 0.2
        if self.opened and self.watch_s < 0.30:
            self.watch_s += dt
            self.max_drop = max(self.max_drop, self.release_z - xyz[2], MOUNT[2] - xyz[2])
            self.invalid |= self.max_drop > 0.002
        safe = self.opened and self.watch_s >= 0.30 and not self.invalid
        self.hold = self.hold + dt if safe and parked else 0.0
        self.value = (
            0.0 if self.invalid else 1.0 if self.hold >= 1.0 - 1e-8
            else 0.65 if safe else 0.25 if self.carry_hold >= 0.20 else 0.0
        )


_trial = Trial()


def score(objects):
    return _trial.value


def success(objects):
    return _trial.value == 1.0


def telemetry():
    return dict(STATE, unlocked=_trial.opened, invalid=_trial.invalid, drop_mm=round(_trial.max_drop * 1000, 2))


def setup():
    parts = [
        rig(BASE, (0.12, 0.19, 0.065), {"release": 0.0}, 0.45, 0.0, 6.0),
        fixed("stand", (*PARK[:2], TABLE), (0.065, 0.066, 0.016)),
        moving("plate", MOUNT, (0.036, 0.091, 0.040), 0.07),
    ]

    def sample(environment):
        return_spring(environment, "release", 190.0, 4.0)
        plate = asset(environment, "plate")
        if not getattr(plate, "_ati_locked", False):
            def retain():
                pos = plate.data.root_pos_w.torch
                velocity = plate.data.root_lin_vel_w.torch
                target = to_tensor(MOUNT, pos) + environment.unwrapped.scene.env_origins
                force = ((target - pos) * 1700 - velocity * 10).clamp(-14, 14)
                if _trial.opened:
                    force = force * 0
                wrench(plate, force.unsqueeze(1))

            install_writer(plate, retain)
            plate._ati_locked = True
        readings = bodies(environment, ["rig", "plate"])
        tool = readings["plate"]
        q = readings["rig"].joints["release"]
        hit = contacts(environment, ["plate", "rig/key"])
        err = tool.pos - PARK
        parked = np.linalg.norm(err[:2]) <= 0.009 and abs(err[2]) <= 0.003 and not hit["plate"]
        parked = bool(parked and tool.resting(0.018, 0.18))
        STATE.update(stroke_mm=round(1000 * q, 2), supported=hit["plate"], key_touch=hit["rig/key"],
                     park_error_mm=np.round(err * 1000, 1).tolist(), parked=parked)
        return q, hit["rig/key"], hit["plate"], tuple(tool.pos), parked

    return register_trial(parts, Trial, STATE, sample, globals())


def tracks():
    l, r = Track(), Track()
    at = (MOUNT[0], MOUNT[1] + 0.077, MOUNT[2] + 0.019 + TIP)
    opened = (PARK[0] - 0.006, PARK[1] + 0.077, at[2] + 0.003)
    down = (opened[0], opened[1], opened[2] - 0.019)
    l.go((*at[:2], HOVER), OPEN, "approach tool-side support")
    l.go(at, OPEN, "lower around tool support grip")
    l.go(at, 0.0, "support tool plate", 1.0)
    key = (BASE[0] - 0.052, BASE[1] - 0.101, TABLE + 0.040 + TIP)
    r.go((*key[:2], HOVER), 0.0, "above guided ball-end key")
    r.wait_until(l.duration + 0.3)
    r.go(key, 0.0, "contact key handle", 1.6)
    r.go((key[0] + 0.032, key[1], key[2]), 0.0, "press manual release piston", 1.4)
    l.wait_until(r.duration + 0.55)
    l.go((at[0], at[1], at[2] + 0.003), 0.0, "clear mounting ledge", 0.55)
    l.go(opened, 0.0, "withdraw supported tool horizontally", 2.6)
    l.go(down, 0.0, "lower tool onto stand", 1.3)
    l.go(down, OPEN, "release parked tool", 0.8)
    l.go((*down[:2], HOVER), OPEN, "clear parked tool")
    r.wait_until(l.duration - 1.5)
    r.go((*key[:2], HOVER), OPEN, "release and clear key")
    l.wait_until(r.duration)
    l[:] = [replace(step, yaw=pi / 2) for step in l]
    return {"left": l, "right": r}


def demo():
    t = tracks()
    return waypoints(t["left"], t["right"])


if __name__ == "__main__":
    import sys

    t = Trial()
    t.step(0.008, True, False, MOUNT, False, 0.05)
    assert t.invalid and t.value == 0
    t = Trial()
    t.step(0, False, True, MOUNT, 0, 0.25)
    assert t.value == 0.25
    t.step(0.008, True, True, MOUNT, False, 0.30)
    assert t.value == 0.65
    t.step(0, False, False, PARK, True, 1)
    assert t.value == 1
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
