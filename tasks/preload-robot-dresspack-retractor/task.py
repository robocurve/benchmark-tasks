"""Set two-link preload, close the protector and cycle the retractor."""
from dataclasses import dataclass
from bench import TABLE, TIP, OPEN, HOVER, Track, rig, bodies, contacts, register_trial, proof, waypoints
from runtime import asset, install_writer
from touch import contact_details

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 900
INSTRUCTION = "Advance the spring-loaded carrier by two links. Close the protector while holding that position, then push the wrist mount through its travel and release it."
BASE = (0.29, 0.0, TABLE)
PITCH = 0.013
PRELOAD = 2 * PITCH
STROKE = 0.070
STATE = {}


@dataclass
class Trial:
    preload: float | None = None
    far: bool = False
    invalid: bool = False
    hold: float = 0.0
    value: float = 0.0

    def step(self, q, cover, touched, cover_touch, free, dt):
        aligned = abs(q - PRELOAD) <= 0.0025
        if self.preload is None and cover >= 0.009 and cover_touch and touched and aligned:
            self.preload = q
        if self.preload is not None:
            self.invalid |= q > STROKE - 0.010
            self.far |= q >= self.preload + 0.024 and touched
        ready = self.far and cover >= 0.009 and abs(q - (self.preload or 0.0)) <= 0.005
        self.hold = self.hold + dt if ready and free and not self.invalid else 0.0
        self.value = (0.0 if self.invalid else 1.0 if self.hold >= 1 - 1e-8 else 0.75 if self.far
                      else 0.50 if self.preload is not None and cover >= 0.009 else 0.25 if aligned else 0.0)


_trial = Trial()


def score(objects): return _trial.value
def success(objects): return _trial.value == 1.0
def telemetry(): return dict(STATE, preload_mm=None if _trial.preload is None else round(1000 * _trial.preload, 2),
                             far=_trial.far, invalid=_trial.invalid, release_s=round(_trial.hold, 2))


def setup():
    fixture = rig(BASE, (0.18, 0.15, 0.06), {"carrier_travel": 0.0, "protector_travel": 0.0}, 1.1, 0.0, 12.0)

    def sample(env):
        unit = asset(env, "rig")
        if not getattr(unit, "_retraction", False):
            carrier = [unit.joint_names.index("carrier_travel")]
            protector = [unit.joint_names.index("protector_travel")]

            def drive():
                q = unit.data.joint_pos.torch[:, carrier]
                cover = unit.data.joint_pos.torch[:, protector]
                velocity = unit.data.joint_vel.torch[:, carrier]
                if _trial.preload is None:
                    force = -36 * q - ((q - PRELOAD).clamp(min=0) * 4000)
                    cover_force = -50 * cover
                else:
                    force = -65 * (q - _trial.preload)
                    cover_force = 130 * (0.012 - cover)
                unit.set_joint_effort_target((force - 4.5 * velocity).clamp(-10, 10), joint_ids=carrier)
                unit.set_joint_effort_target(cover_force.clamp(-4, 4), joint_ids=protector)

            install_writer(unit, drive)
            unit._retraction = True
        q = bodies(env, ["rig"])["rig"].joints
        touch = contacts(env, ["rig/carrier", "rig/protector"])
        STATE.update(travel_mm=round(1000 * q["carrier_travel"], 2),
                     protector_mm=round(1000 * q["protector_travel"], 2),
                     reserve_mm=round(1000 * (STROKE - q["carrier_travel"]), 2),
                     carrier_touched=touch["rig/carrier"], protector_touched=touch["rig/protector"], contact=contact_details())
        return q["carrier_travel"], q["protector_travel"], touch["rig/carrier"], touch["rig/protector"], not any(touch.values())

    return register_trial([fixture], Trial, STATE, sample, globals())


def tracks():
    l, r = Track(), Track()
    a = (BASE[0] - 0.082, -0.101, TABLE + 0.037 + TIP)
    b = (a[0] + 0.047, a[1], a[2])
    far = (a[0] + 0.073, a[1], a[2])
    r.go((*a[:2], HOVER), 0.0, "above carriage paddle")
    r.go(a, 0.0, "approach carriage paddle")
    r.go(b, 0.0, "advance spring-loaded carrier by two links", 2.4)
    c = (BASE[0] + 0.002, 0.072, TABLE + 0.04 + TIP)
    d = (c[0] + 0.038, c[1], c[2])
    l.go((*c[:2], HOVER), OPEN, "above protector handle")
    l.wait_until(r.duration + 0.2)
    l.go(c, 0.0, "lower ahead of protector")
    l.go(d, 0.0, "close protector on tracked position", 1.9)
    l.go((c[0], c[1], c[2]), 0.0, "back away from protector")
    l.go((*c[:2], HOVER), OPEN, "clear protector")
    r.wait_until(l.duration)
    r.go(far, 0.0, "exercise wrist mount with stroke in reserve", 2)
    r.go(a, 0.0, "retract and allow spring return", 1.7)
    r.go((*a[:2], HOVER), OPEN, "release wrist mount")
    r.go((0.155, -0.199, HOVER), OPEN, "park beyond the carrier envelope")
    l.go((0.158, 0.204, HOVER), OPEN, "park beyond the protector envelope")
    r.wait_until(l.duration)
    l.wait_until(r.duration)
    return {"left": l, "right": r}


def demo():
    path = tracks()
    return waypoints(path["left"], path["right"])


if __name__ == "__main__":
    import sys
    t = Trial()
    t.step(0, 0.012, True, True, False, 1)
    assert t.value == 0
    t.step(PRELOAD, 0.011, True, True, False, 1)
    assert t.value == 0.50
    t.step(0.054, 0.012, True, False, False, 1)
    assert t.value == 0.75
    t.step(PRELOAD, 0.012, False, False, True, 1)
    assert t.value == 1
    t.step(0.064, 0.012, True, False, False, 1)
    assert t.value == 0
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
