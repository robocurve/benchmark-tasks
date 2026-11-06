"""Set the radial and angular adjusters and run a guarded coupled spindle."""

import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parts import Rotor, Slider, _to_numpy, on_reset, on_step, part
from yam_arena.demo import Waypoint


SCENE = "station"
SUCCESS_HOLD_S = 0.3
SCORE_OVER = "end"
INSTRUCTION = (
    "Slide the two blue shaft adjusters toward the spindle, then slide the orange cradle lock. "
    "Press the green TEST cap and let the guarded output spindle turn twice."
)
DARK = (0.105, 0.145, 0.21)
SILVER = (0.65, 0.72, 0.78)
BLUE = (0.12, 0.50, 0.86)
ORANGE = (0.96, 0.48, 0.16)
GREEN = (0.24, 0.80, 0.51)
WHITE = (0.86, 0.89, 0.92)
FRONT, REAR, LOCK = 0.211, 0.291, 0.372
CAP = (0.270, -0.106)


def box(x, y, z, dx, dy, dz, color):
    return ((x, y, z), (dx, dy, dz), color)


def tab(name, x, color):
    return Slider(name, (x, 0.098, 0.798), boxes=(box(x, 0.098, 0.798, 0.048, 0.027, 0.026, color),),
                  travel=0.015, rate=0.001, damping=8.0, mass=0.065, axis="Y")


guard = tuple(box(x, y, 0.800, dx, dy, 0.037, SILVER) for x, y, dx, dy in (
    (0.178, 0.0, 0.010, 0.076), (0.418, 0.0, 0.010, 0.076),
    (0.298, -0.042, 0.248, 0.009), (0.298, 0.042, 0.248, 0.009),
))
cover = tuple(box(x, y, 0.832, 0.090, 0.006, 0.006, DARK)
              for x in (0.236, 0.357) for y in (-0.034, -0.016, 0.0, 0.016, 0.034))

OBJECTS = [part("alignment_fixture", pos=(0.0, 0.0, 0.0), mass=15.0, friction=1.2, boxes=(
    box(0.298, -0.002, 0.7515, 0.356, 0.255, 0.008, DARK),
    box(0.298, 0.0, 0.766, 0.238, 0.075, 0.020, SILVER),
    box(CAP[0], CAP[1], 0.764, 0.048, 0.040, 0.014, SILVER),
    *guard, *cover,
), sliders=(
    tab("front", FRONT, BLUE), tab("rear", REAR, BLUE), tab("lock", LOCK, ORANGE),
    Slider("test", (*CAP, 0.789), boxes=(box(*CAP, 0.789, 0.038, 0.027, 0.016, GREEN),),
           travel=0.007, rate=1600.0, damping=15.0, mass=0.045, axis="Z"),
    Rotor("input", (0.236, 0.0, 0.793), cylinders=(((0.236, 0.0, 0.793), 0.026, 0.014, BLUE, "Z"),),
          boxes=(box(0.252, 0.0, 0.802, 0.014, 0.005, 0.003, WHITE),), mass=0.13, axis="Z", armature=0.045),
    Rotor("spindle", (0.355, 0.0, 0.802), cylinders=(((0.355, 0.0, 0.802), 0.015, 0.081, SILVER, "X"),),
          boxes=(box(0.357, 0.0, 0.818, 0.045, 0.003, 0.004, BLUE),), mass=0.17, axis="X", armature=0.040),
))]


class Trial:
    def reset(self):
        self.settings = self.last = self.start = None
        self.angle = [0.0, 0.0]
        self.demand = self.turns = self.peak = self.result = 0.0
        self.fault = False
        self.steps = 0


trial = Trial()
trial.reset()


@on_reset
def reset(environment):
    trial.reset()


@on_step
def update(environment):
    import torch

    base = environment.unwrapped
    fixture = base.scene["alignment_fixture"]
    ids = [fixture.joint_names.index(name + "_slide") for name in ("front", "rear", "lock", "test", "input", "spindle")]
    q = _to_numpy(fixture.data.joint_pos)[0, ids]
    wi, wo = (float(value) for value in _to_numpy(fixture.data.joint_vel)[0, ids[-2:]])
    root = _to_numpy(fixture.data.root_link_pose_w)[0]
    origin = _to_numpy(base.scene.env_origins)[0]
    trial.steps += 1
    trial.fault |= not all(math.isfinite(v) for v in (*q, wi, wo))
    trial.fault |= math.dist(root[:3], origin) > 0.010 or abs(float(root[6])) < 0.998
    front, rear, lock, cap = (-float(value) for value in q[:4])
    if lock >= 0.012 and trial.settings is None:
        trial.settings = (front, rear)
    if trial.settings is not None and (lock < 0.012 or max(abs(front - trial.settings[0]), abs(rear - trial.settings[1])) > 0.0008):
        trial.fault = True
    radial = abs(1.2 * (1.0 - 0.75 * front / 0.015))
    angular = abs(1.5 * (1.0 - 0.75 * rear / 0.015))
    load = 0.03 + 0.06 * (radial / 0.5) ** 1.5 + 0.08 * (angular / 0.5) ** 1.5
    if trial.last is not None:
        for i, (now, before) in enumerate(zip(q[-2:], trial.last)):
            trial.angle[i] += math.remainder(now - before, 2 * math.pi)
    trial.last = tuple(q[-2:])
    ai, ao = trial.angle
    if cap >= 0.003 and trial.settings is not None and trial.start is None:
        trial.start = ao
    if trial.start is not None:
        trial.turns = max(trial.turns, (ao - trial.start) / (2 * math.pi))
    target = 4.1 if trial.start is not None and trial.turns < 2.1 else 0.0
    step = base.step_dt
    trial.demand += max(-3.0 * step, min(1.6 * step, target - trial.demand))
    coupling = 1.1 * (ai - ao) + 0.023 * (wi - wo)
    motor = max(-0.46, min(0.46, 0.18 * (trial.demand - wi) + 0.012 * trial.demand))
    if trial.start is not None and trial.turns < 2.1:
        trial.peak = max(trial.peak, abs(coupling))
    trial.fault |= trial.peak > 0.50
    forces = (0.0, 0.0) if trial.fault else (motor - coupling, coupling - load * math.tanh(wo / 0.45))
    fixture.actuators.target_command.set_effort_index(
        value=torch.tensor([forces], device=base.device, dtype=torch.float32), joint_ids=ids[-2:])
    trial.result = 0.25 if trial.settings is not None and min(radial, angular) <= 0.5 else 0.0
    if trial.settings is not None and max(radial, angular) <= 0.5:
        trial.result = 0.5
        if trial.peak <= 0.25 and trial.turns >= 1.0:
            trial.result = 1.0 if trial.turns >= 2.0 and cap < 0.003 else 0.75
    if trial.fault:
        trial.result = 0.0
    if os.getenv("TASK_TRACE"):
        row = dict(step=trial.steps, front=float(front), rear=float(rear), lock=float(lock), cap=float(cap),
                   radial_mm=radial, angular_deg=angular, output_turns=trial.turns, peak_nm=trial.peak,
                   fault=trial.fault, score=trial.result)
        with open(os.environ["TASK_TRACE"], "a", encoding="utf-8") as trace:
            trace.write(json.dumps(row) + "\n")


def score(objects):
    return trial.result


def success(objects):
    return trial.result == 1.0


def demo():
    paths = {"left": [], "right": []}
    homes = {"left": (0.208, 0.209, 0.855), "right": (0.208, -0.209, 0.855)}

    def phase(duration, arm=None, pos=None, label="wait", yaw=0.0, finger=-0.015):
        for side, path in paths.items():
            path.append(Waypoint(duration, pos if side == arm else homes[side], finger if side == arm else -0.015,
                                 label, yaw if side == arm else 0.0))

    phase(8.0 if os.getenv("SETTLE_ONLY") else 1.0)
    if os.getenv("SETTLE_ONLY"):
        return paths
    for x, label in ((FRONT, "front"), (REAR, "rear"), (LOCK, "lock")):
        if label == "rear" and os.getenv("SKIP_REAR"):
            continue
        phase(1.0, "left", (x, 0.151, 0.859), label + " approach", math.pi / 2)
        phase(0.8, "left", (x, 0.151, 0.809), label + " lower", math.pi / 2)
        phase(1.8, "left", (x, 0.110, 0.809), label + " push", math.pi / 2)
        phase(0.65, "left", (x, 0.110, 0.859), label + " lift", math.pi / 2)
    phase(0.9)
    phase(1.15, "right", (*CAP, 0.865), "TEST approach", finger=0.0)
    phase(1.15, "right", (*CAP, 0.785), "TEST descend", finger=0.0)
    phase(0.65, "right", (*CAP, 0.785), "TEST hold", finger=0.0)
    phase(0.7, "right", (*CAP, 0.865), "TEST release", finger=0.0)
    phase(7.2)
    return paths
