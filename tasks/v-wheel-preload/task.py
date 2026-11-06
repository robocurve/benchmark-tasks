"""Set a V-wheel carriage preload and verify stiffness and drag through contact."""

import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parts import MountedSlider, Slider, _to_numpy, on_reset, on_step, part
from yam_arena.demo import Waypoint


SCENE = "station"
SUCCESS_HOLD_S = 0.30
SCORE_OVER = "end"
INSTRUCTION = (
    "Slide the blue preload tab and the orange lock tab toward the rail. "
    "Hold the green side-load cap with the carriage untouched, then release it. "
    "Push the carriage along the rail from its green start marker to its amber end marker."
)
DARK = (0.105, 0.145, 0.21)
SILVER = (0.65, 0.72, 0.78)
BLUE = (0.12, 0.50, 0.86)
ORANGE = (0.96, 0.48, 0.16)
GREEN = (0.24, 0.80, 0.51)
Y = -0.051
START = 0.417
END = 0.309
CAP = (0.372, 0.106)
ADJUST = 0.210
LOCK = 0.294


def box(x, y, z, dx, dy, dz, color):
    return ((x, y, z), (dx, dy, dz), color)


def tab(name, x, color):
    return Slider(name, (x, 0.098, 0.798), boxes=(box(x, 0.098, 0.798, 0.047, 0.027, 0.026, color),),
                  travel=0.015, rate=0.001, damping=8.0, mass=0.065, axis="Y")


OBJECTS = [part("preload_fixture", pos=(0.0, 0.0, 0.0), mass=15.0, friction=1.2, boxes=(
    box(0.304, -0.002, 0.7515, 0.357, 0.261, 0.008, DARK),
    box(0.361, Y, 0.767, 0.191, 0.077, 0.020, SILVER),
    box(0.361, Y + 0.041, 0.780, 0.194, 0.008, 0.012, SILVER),
    box(0.361, Y - 0.041, 0.780, 0.194, 0.008, 0.012, SILVER),
    box(START, Y + 0.042, 0.789, 0.007, 0.011, 0.005, GREEN),
    box(END, Y + 0.042, 0.789, 0.007, 0.011, 0.005, ORANGE),
    box(CAP[0], CAP[1], 0.766, 0.050, 0.039, 0.016, SILVER),
), sliders=(
    tab("adjust", ADJUST, BLUE), tab("lock", LOCK, ORANGE),
    Slider("side_cap", (*CAP, 0.790), boxes=(box(*CAP, 0.790, 0.040, 0.028, 0.017, GREEN),),
           travel=0.007, rate=1650.0, damping=15.0, mass=0.045, axis="Z"),
    Slider("lateral", (START, Y, 0.779), boxes=(box(START, Y, 0.779, 0.018, 0.038, 0.008, SILVER),),
           travel=0.003, rate=2000.0, damping=22.0, mass=0.17, axis="Y"),
    MountedSlider("carriage", (START, Y, 0.804), boxes=(
        box(START, Y, 0.797, 0.044, 0.047, 0.012, SILVER),
        box(START, Y, 0.813, 0.044, 0.062, 0.024, DARK),
        box(START, Y, 0.827, 0.032, 0.043, 0.004, BLUE),
    ), travel=0.108, rate=0.001, damping=0.20, mass=0.18, axis="X", parent="lateral"),
))]


class Trial:
    def reset(self):
        self.lock_setting = None
        self.side_time = self.deflection = self.peak_drag = self.travel = 0.0
        self.side_pass = self.fault = False
        self.result = 0.0
        self.steps = 0


trial = Trial()
trial.reset()


@on_reset
def reset(environment):
    trial.reset()


@on_step
def update(environment):
    import torch
    from yam_arena.runtime import pad_tips_world

    base = environment.unwrapped
    fixture = base.scene["preload_fixture"]
    names = fixture.joint_names
    ids = [names.index(name + "_slide") for name in ("adjust", "lock", "side_cap", "lateral", "carriage")]
    adj, locked, cap, lateral, travel = (-float(q) for q in _to_numpy(fixture.data.joint_pos)[0, ids])
    velocity = float(_to_numpy(fixture.data.joint_vel)[0, ids[-1]])
    root = _to_numpy(fixture.data.root_link_pose_w)[0]
    origin = _to_numpy(base.scene.env_origins)[0]
    trial.steps += 1
    trial.fault |= not all(math.isfinite(v) for v in (adj, locked, cap, lateral, travel, velocity))
    trial.fault |= math.dist(root[:3], origin) > 0.010 or abs(root[6]) < 0.998 or not -0.001 <= travel <= 0.109
    preload = (1 - math.cos(min(0.75, max(0.0, 0.75 * adj / 0.015)))) / 2
    drag_capacity = 0.20 + 4.0 * preload
    if locked >= 0.012 and trial.lock_setting is None:
        trial.lock_setting = adj
        stiffness = 2000.0 + 18000.0 * preload
        fixture.write_joint_stiffness_to_sim_index(
            stiffness=torch.tensor([[stiffness]], device=base.device, dtype=torch.float32), joint_ids=[ids[3]])
    if trial.lock_setting is not None and (locked < 0.012 or abs(adj - trial.lock_setting) > 0.0008):
        trial.fault = True
    side_active = cap >= 0.003
    if side_active and trial.lock_setting is not None:
        trial.side_time += base.step_dt
        for arm in ("left", "right"):
            x, y, z = pad_tips_world(base.scene[arm + "_robot"]) - origin
            if abs(x - (START - travel)) <= 0.048 and abs(y - Y) <= 0.051 and z < 0.868:
                trial.fault = True
        if trial.side_time >= 0.35:
            trial.deflection = max(trial.deflection, abs(lateral))
            if 0.00005 <= trial.deflection < 0.0005:
                trial.side_pass = True
    else:
        trial.side_time = 0.0
    drag = drag_capacity * math.tanh(velocity / 0.007)
    if not side_active and trial.lock_setting is not None:
        trial.travel = max(trial.travel, travel)
        trial.peak_drag = max(trial.peak_drag, abs(drag) + 0.20 * abs(velocity))
    command = fixture.actuators.target_command
    command.set_effort_index(
        value=torch.tensor([[-float(side_active), -drag]], device=base.device, dtype=torch.float32), joint_ids=ids[3:])
    slide_pass = trial.travel >= 0.100 and 0.0 < trial.peak_drag < 2.0
    trial.result = 0.25 if trial.lock_setting is not None else 0.0
    if slide_pass or trial.side_pass:
        trial.result = 0.5
    if slide_pass and trial.side_pass and trial.deflection < 0.0005 and not side_active:
        trial.result = 1.0
    if trial.fault:
        trial.result = 0.0
    if os.getenv("TASK_TRACE"):
        with open(os.environ["TASK_TRACE"], "a", encoding="utf-8") as trace:
            trace.write(json.dumps(dict(step=trial.steps, adjust=adj, lock=locked, side=cap, lateral=lateral,
                travel=travel, peak_drag=trial.peak_drag, deflection=trial.deflection, fault=trial.fault, score=trial.result))+"\n")


def score(objects):
    return trial.result


def success(objects):
    return trial.result == 1.0


def demo():
    paths = {"left": [], "right": []}
    homes = {"left": (0.21, 0.209, 0.855), "right": (0.21, -0.209, 0.855)}

    def phase(duration, arm=None, pos=None, label="wait", yaw=0.0, finger=-0.015):
        for side, path in paths.items():
            path.append(Waypoint(duration, pos if side == arm else homes[side], finger if side == arm else -0.015,
                                 label, yaw if side == arm else 0.0))

    phase(8.0 if os.getenv("SETTLE_ONLY") else 1.0)
    if os.getenv("SETTLE_ONLY"):
        return paths
    for x, label in ((ADJUST, "preload"), (LOCK, "lock")):
        phase(1.0, "left", (x, 0.151, 0.859), label + " approach", math.pi / 2)
        phase(0.8, "left", (x, 0.151, 0.809), label + " lower", math.pi / 2)
        phase(1.8, "left", (x, 0.110, 0.809), label + " push", math.pi / 2)
        phase(0.6, "left", (x, 0.110, 0.859), label + " lift", math.pi / 2)
    if not os.getenv("SKIP_SIDE"):
        phase(1.15, "left", (*CAP, 0.865), "side test approach", finger=0.0)
        phase(1.15, "left", (*CAP, 0.787), "side test descend", finger=0.0)
        phase(0.85, "left", (*CAP, 0.787), "side test hold", finger=0.0)
        phase(0.7, "left", (*CAP, 0.865), "side test release", finger=0.0)
    phase(1.0)
    phase(1.25, "right", (0.470, Y, 0.866), "carriage approach")
    phase(0.85, "right", (0.470, Y, 0.820), "carriage lower")
    phase(3.2, "right", (0.333, Y, 0.820), "100 mm carriage sweep")
    phase(0.70, "right", (0.333, Y, 0.866), "carriage lift clear")
    phase(2.0)
    return paths
