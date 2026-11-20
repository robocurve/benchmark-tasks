"""Seat two floating cold coolant couplers with one guided tray insertion."""
from dataclasses import dataclass
from bench import TABLE, TIP, OPEN, HOVER, Track, rig, bodies, contacts, register_trial, proof, waypoints
from runtime import asset, install_writer
from scenekit import UsdBody

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 750
INSTRUCTION = "Push the unpowered tray straight into the manifold until both coolant ports seat, then release it."
BASE = (0.306, -0.079, TABLE)
STROKE = 0.038
OFFSET = 0.0022
STATE = {}


@dataclass
class Trial:
    hit_limit: bool = False
    hold: float = 0.0
    value: float = 0.0

    def step(self, travel, floats, free, dt):
        self.hit_limit |= travel > 0.010 and any(abs(x) > 0.0039 for x in floats)
        aligned = sum(abs(f - OFFSET) <= 0.0005 for f in floats)
        deep = abs(STROKE - travel) <= 0.0005
        good = deep and aligned == 2 and not self.hit_limit
        self.hold = self.hold + dt if good and free else 0.0
        self.value = (
            0.0 if self.hit_limit else 1.0 if self.hold >= 1 - 1e-8
            else 0.75 if good else 0.25 * aligned if travel >= 0.020 else 0.0
        )


_trial = Trial()


def score(objects): return _trial.value
def success(objects): return _trial.value == 1.0
def telemetry(): return dict(STATE, float_limit=_trial.hit_limit, released_s=round(_trial.hold, 2))


def setup():
    rail = rig(BASE, (0.16, 0.10, 0.06), {"insertion": 0.0}, 0.6, 0.0, 8.0)
    ports = UsdBody("manifold", usd="assets/manifold.usda", pos=BASE, size=(0.13, 0.09, 0.05),
                    articulation=True, joint_pos=(("supply", 0.0), ("return", 0.0)),
                    joint_damping=0.40, joint_stiffness=0.0, joint_effort_limit=3.0)

    def sample(env):
        unit = asset(env, "manifold")
        if not getattr(unit, "_float_springs", False):
            def restore():
                q = unit.data.joint_pos.torch
                unit.set_joint_effort_target((-32 * q).clamp(-1.5, 1.5))
            install_writer(unit, restore)
            unit._float_springs = True
        parts = bodies(env, ["rig", "manifold"])
        x = parts["rig"].joints["insertion"]
        floats = [parts["manifold"].joints[name] for name in ("supply", "return")]
        hit = contacts(env, ["rig/insertion_body", "manifold/supply_body", "manifold/return_body"])
        STATE.update(depth_error_mm=round(1000 * (STROKE - x), 2),
                     lateral_error_mm=[round(1000 * (OFFSET - f), 2) for f in floats],
                     floats_mm=[round(1000 * f, 2) for f in floats], released=not any(hit.values()))
        return x, floats, not any(hit.values())

    return register_trial([rail, ports], Trial, STATE, sample, globals())


def tracks():
    l, r = Track(), Track()
    at = (BASE[0] - 0.104, BASE[1] + OFFSET, TABLE + 0.033 + TIP)
    end = (at[0] + 0.064, at[1], at[2])
    r.go((*at[:2], HOVER), OPEN, "above rear tray pad")
    r.go(at, 0.0, "lower ahead of pad")
    r.go(end, 0.0, "push both couplers toward seat", 3.0)
    r.go(end, 0.0, "hold full seat", 0.6)
    r.go((end[0] - 0.022, end[1], end[2]), 0.0, "back off rear pad")
    r.go((*end[:2], HOVER), OPEN, "clear tray")
    l.go((0.17, 0.20, HOVER), OPEN, "park away from tray")
    l.wait_until(r.duration)
    return {"left": l, "right": r}


def demo():
    path = tracks()
    return waypoints(path["left"], path["right"])


if __name__ == "__main__":
    import sys
    t = Trial()
    t.step(STROKE, (OFFSET, 0), True, 3)
    assert t.value == 0.25
    t.step(STROKE, (OFFSET, OFFSET), True, 1)
    assert t.value == 1.0
    t.step(STROKE, (OFFSET, 0.004), True, 1)
    assert t.value == 0
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
