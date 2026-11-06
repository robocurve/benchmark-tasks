"""Exercise an overload clutch on a guarded dual-YAM proof stand."""

import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parts import Rotor, Slider, _to_numpy, on_reset, on_step, part
from yam_arena.demo import Waypoint


SCENE = "station"
SUCCESS_HOLD_S = 0.25
SCORE_OVER = "end"
INSTRUCTION = (
    "Press blue RUN and let the silver output wheel turn at least once. "
    "Hold the orange JAM cap until the output stops while the input keeps turning. "
    "Release JAM, then press green RESET and let the output turn once again."
)

DARK = (0.105, 0.145, 0.21)
SILVER = (0.61, 0.68, 0.76)
BLUE = (0.13, 0.52, 0.88)
ORANGE = (0.95, 0.47, 0.16)
GREEN = (0.24, 0.80, 0.51)
WHITE = (0.86, 0.89, 0.92)
RUN_XY = (0.234, -0.111)
JAM_XY = (0.297, 0.105)
RESET_XY = (0.347, -0.111)
ROTOR_HEIGHT = 0.792
TRIP_NM = 0.23
LOAD_NM = 0.12


def box(x, y, z, dx, dy, dz, color):
    return ((x, y, z), (dx, dy, dz), color)


def button(name, xy, color):
    x, y = xy
    return Slider(name, (x, y, 0.789), boxes=(box(x, y, 0.789, 0.036, 0.028, 0.016, color),),
                  travel=0.007, rate=1600.0, damping=15.0, mass=0.045, axis="Z")


def rotor(name, x, color, inertia):
    return Rotor(name, (x, 0.0, ROTOR_HEIGHT),
                 cylinders=(((x, 0.0, ROTOR_HEIGHT), 0.026, 0.014, color, "Z"),),
                 boxes=(box(x + 0.016, 0.0, ROTOR_HEIGHT + 0.008, 0.015, 0.004, 0.003, WHITE),),
                 mass=0.13, axis="Z", armature=inertia)


guard = tuple(box(x, y, 0.800, dx, dy, 0.034, SILVER) for x, y, dx, dy in (
    (0.175, 0.0, 0.009, 0.080), (0.414, 0.0, 0.009, 0.080),
    (0.2945, -0.045, 0.248, 0.009), (0.2945, 0.045, 0.248, 0.009),
))
shield = tuple(box(x, y, 0.824, 0.071, 0.006, 0.005, DARK)
               for x in (0.232, 0.351) for y in (-0.037, -0.019, 0.0, 0.019, 0.037))

OBJECTS = [part("clutch_fixture", pos=(0.0, 0.0, 0.0), mass=15.0, friction=1.2, boxes=(
    box(0.296, 0.0, 0.7515, 0.354, 0.255, 0.008, DARK),
    box(0.2945, 0.0, 0.766, 0.238, 0.076, 0.020, DARK),
    *(box(x, y, 0.763, 0.049, 0.043, 0.013, SILVER) for x, y in (RUN_XY, JAM_XY, RESET_XY)),
    *guard, *shield,
), sliders=(button("run", RUN_XY, BLUE), button("jam", JAM_XY, ORANGE),
             button("reset", RESET_XY, GREEN), rotor("input", 0.232, BLUE, 0.045),
             rotor("output", 0.351, SILVER, 0.040)))]


class Clutch:
    def reset(self):
        self.enabled = self.reset_pressed = self.reset_seen = self.fault = False
        self.engaged = True
        self.phase = 0.0
        self.jam_position = None
        self.trip = None
        self.pre = self.post = self.demand = self.score = 0.0
        self.last = None
        self.angle = [0.0, 0.0]
        self.steps = 0

    def advance(self, dt, positions, velocities, run, jam, reset):
        self.steps += 1
        if self.last is not None:
            for axis, (now, previous) in enumerate(zip(positions, self.last)):
                self.angle[axis] += math.remainder(now - previous, 2 * math.pi)
        self.last = positions
        ai, ao = self.angle
        wi, wo = velocities
        relative = ai - ao
        if run >= 0.003:
            self.enabled = True
        jammed = jam >= 0.003
        pressed = reset >= 0.003
        if pressed and not self.reset_pressed and not jammed and not self.engaged:
            self.phase = relative
            self.engaged = self.reset_seen = True
        self.reset_pressed = pressed
        if jammed and self.jam_position is None:
            self.jam_position = ao
        elif not jammed:
            self.jam_position = None
        measured_load = LOAD_NM * math.tanh(wo / 0.16)
        torque = 1.15 * (relative - self.phase) + 0.025 * (wi - wo) if self.engaged else 0.0
        if self.engaged and abs(torque) >= TRIP_NM:
            if self.trip is None and jammed and self.pre >= 2 * math.pi:
                # The calibrated detent opens when its torque crosses the threshold.
                self.trip = TRIP_NM
            else:
                self.fault = True
            self.engaged = False
            torque = 0.0
        if self.enabled:
            target = 3.6 if self.engaged or jammed else 0.0
            self.demand += max(-5.0 * dt, min(1.2 * dt, target - self.demand))
        motor = max(-0.48, min(0.48, 0.17 * (self.demand - wi) + 0.012 * self.demand))
        brake = 0.0 if self.jam_position is None else 3.4 * (ao - self.jam_position) + 0.32 * wo
        if self.enabled and self.engaged and not jammed and measured_load >= 0.118:
            if self.reset_seen:
                self.post += max(0.0, wo) * dt
            elif self.trip is None:
                self.pre += max(0.0, wo) * dt
        self.score = 0.25 if self.pre >= 2 * math.pi else 0.0
        if self.trip is not None:
            self.score = 0.5
        if self.trip is not None and self.reset_seen and self.engaged:
            self.score = 1.0 if self.post >= 2 * math.pi and not pressed else 0.75
        if self.fault:
            self.score = 0.0
        return motor - torque, torque - brake - measured_load, torque


clutch = Clutch()
clutch.reset()


@on_reset
def reset_trial(environment):
    clutch.reset()


@on_step
def update(environment):
    import torch

    base = environment.unwrapped
    fixture = base.scene["clutch_fixture"]
    names = fixture.joint_names
    ids = [names.index(f"{name}_slide") for name in ("input", "output", "run", "jam", "reset")]
    q = _to_numpy(fixture.data.joint_pos)[0, ids]
    v = _to_numpy(fixture.data.joint_vel)[0, ids]
    root = _to_numpy(fixture.data.root_link_pose_w)[0]
    origin = _to_numpy(base.scene.env_origins)[0]
    bad_root = math.dist(root[:3], origin) > 0.010 or abs(root[6]) < 0.998
    if bad_root or not all(math.isfinite(x) for x in (*q, *v)):
        clutch.fault = True
        clutch.score = 0.0
        efforts = (0.0, 0.0, 0.0)
    else:
        efforts = clutch.advance(base.step_dt, tuple(q[:2]), tuple(v[:2]), *(-q[2:]))
    command = fixture.actuators.target_command
    command.set_effort_index(value=torch.tensor([efforts[:2]], device=base.device, dtype=torch.float32),
                             joint_ids=ids[:2])
    if os.getenv("TASK_TRACE"):
        row = dict(step=clutch.steps, pre_turns=clutch.pre / (2 * math.pi), post_turns=clutch.post / (2 * math.pi),
                   run_m=float(-q[2]), jam_m=float(-q[3]), reset_m=float(-q[4]),
                   input_velocity=float(v[0]), output_velocity=float(v[1]), torque_nm=efforts[2],
                   trip_nm=clutch.trip, fault=clutch.fault, score=clutch.score)
        with open(os.environ["TASK_TRACE"], "a", encoding="utf-8") as trace:
            trace.write(json.dumps(row) + "\n")


def score(objects):
    return clutch.score


def success(objects):
    return clutch.score == 1.0


def demo():
    paths = {"left": [], "right": []}
    homes = {"left": (0.20, 0.203, 0.865), "right": (0.20, -0.203, 0.865)}

    def phase(duration, arm=None, pos=None, label="wait"):
        for side, path in paths.items():
            path.append(Waypoint(duration, pos if side == arm else homes[side], 0.0, label))

    def press(arm, xy, name, dwell):
        phase(1.1, arm, (*xy, 0.861), name + " approach")
        phase(1.1, arm, (*xy, 0.785), name + " descend")
        phase(dwell, arm, (*xy, 0.785), name + " hold")
        phase(0.75, arm, (*xy, 0.861), name + " lift")
        phase(0.75)

    phase(8.0 if os.getenv("SETTLE_ONLY") else 1.0)
    if not os.getenv("SETTLE_ONLY"):
        press("right", RUN_XY, "RUN", 0.6)
        phase(5.4)
        press("left", JAM_XY, "JAM", 1.5)
        phase(0.8)
        if not os.getenv("SKIP_RESET"):
            press("right", RESET_XY, "RESET", 0.6)
        phase(5.7)
    return paths
