"""Align the three full-size wafers by exercising a shared edge roller."""
from dataclasses import dataclass
from math import radians, degrees, sin, cos
from bench import TABLE, TIP, OPEN, HOVER, Track, rig, bodies, contacts, register_trial, proof, waypoints
from runtime import asset, install_writer
from scenekit import UsdBody

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 780
INSTRUCTION = "Press the blue crank to turn the shared roller and align the three notches. Keep both hands off the wafers and release the crank."
BASE = (0.306, -0.007, TABLE)
ERRORS = tuple(map(radians, (4, 7, 9)))
STATE = {}


@dataclass
class Trial:
    max_drive: float = 0.0
    contaminated: bool = False
    hold: float = 0.0
    value: float = 0.0

    def step(self, errors, touched_face, released, dt):
        self.contaminated |= touched_face
        count = sum(abs(error) <= radians(2) for error in errors)
        good = count == 3 and not self.contaminated and released
        self.hold = self.hold + dt if good else 0.0
        self.value = 0.25 * count + (0.25 if self.hold >= 1.0 - 1e-8 else 0)


_trial = Trial()


def score(objects): return _trial.value
def success(objects): return _trial.value == 1.0
def telemetry(): return dict(STATE, prohibited_contact=_trial.contaminated, released_s=round(_trial.hold, 2))


def setup():
    fixture = rig(BASE, (0.23, 0.22, 0.25),
                  {"spin0": 0.0, "spin1": 0.0, "spin2": 0.0},
                  0.0, 0.0, 5.0)
    drive = UsdBody("drive_unit",usd="assets/drive_unit.usda",pos=BASE,size=(.10,.23,.065),
                    articulation=True,joint_pos=(("drive",0.),),joint_stiffness=0.0,
                    joint_damping=0.025,joint_effort_limit=5.)

    def sample(env):
        unit = asset(env, "rig")
        if not getattr(unit, "_roller", False):
            motor = asset(env,"drive_unit")
            roller = motor.joint_names.index("drive")
            wheels = [unit.joint_names.index(f"spin{i}") for i in range(3)]

            def rotate():
                import torch

                q = unit.data.joint_pos.torch
                roller_travel = (-motor.data.joint_pos.torch[:, roller:roller + 1]).clamp(min=_trial.max_drive)
                catches = torch.as_tensor([-e for e in ERRORS], device=q.device, dtype=q.dtype)
                target = torch.maximum(-0.24 * roller_travel, catches)
                v = unit.data.joint_vel.torch[:, wheels]
                force = q.new_zeros(q.shape)
                force[:, wheels] = ((target - q[:, wheels]) * 0.38 - v * 0.018).clamp(-0.035, 0.035)
                unit.set_joint_effort_target(force)

            install_writer(unit, rotate)
            unit._roller = True
        parts = bodies(env, ["rig","drive_unit"])
        q = parts["rig"].joints
        angle = parts["drive_unit"].joints["drive"]
        # Wafer notch directions reverse under the authored positive-Y frame.
        _trial.max_drive = max(_trial.max_drive, -angle)
        errors = [angle + q[f"spin{i}"] for i, angle in enumerate(ERRORS)]
        hit = contacts(env, ["drive_unit/roller", "rig/wafer0", "rig/wafer1", "rig/wafer2"])
        STATE.update(roller_deg=round(degrees(angle), 2), notch_deg=[round(degrees(e), 2) for e in errors])
        return errors, any(hit[f"rig/wafer{i}"] for i in range(3)), not any(hit.values())

    return register_trial([fixture,drive], Trial, STATE, sample, globals())


def tracks():
    l, r = Track(), Track()
    at = (BASE[0] - .046, BASE[1] - .176, TABLE + .055 + TIP)
    r.go((*at[:2], HOVER), OPEN, "above the crank")
    r.go((*at[:2], at[2] + .008), 0.0, "approach crank pad")
    for theta in map(radians, (12, 24, 36, 48)):
        end = (BASE[0] - .046*cos(theta), at[1], at[2] - .046*sin(theta) - .005)
        r.go(end, 0.0, "rotate shared roller by downward lever stroke", 1.05)
    r.go(end, 0.0, "allow the catches to settle")
    r.go((*end[:2], HOVER), OPEN, "release and clear")
    l.go((0.17, 0.21, HOVER), OPEN, "park away from cassette")
    l.wait_until(r.duration)
    return {"left": l, "right": r}


def demo():
    path = tracks()
    return waypoints(path["left"], path["right"])


if __name__ == "__main__":
    import sys
    t = Trial()
    t.step(ERRORS, False, True, 1)
    assert t.value == 0
    t.step([0.0] * 3, True, True, 2)
    assert t.value == 0.75
    t = Trial()
    t.step([0.0] * 3, False, True, 1)
    assert t.value == 1.0
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
