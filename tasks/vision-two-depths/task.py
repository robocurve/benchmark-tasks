"""Set one lens configuration and inspect two rendered moving charts."""

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parts import Slider, _to_numpy, on_reset, on_step, part
from yam_arena.demo import Waypoint


SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.25
INSTRUCTION = (
    "Slide the two blue focus and aperture tabs toward the rail, then slide the orange lens lock. "
    "Press the green TEST cap and clear both arms from the chart carriage and camera."
)
DARK = (0.11, 0.14, 0.20)
SILVER = (0.64, 0.71, 0.76)
BLUE = (0.11, 0.51, 0.89)
GREEN = (0.21, 0.81, 0.48)
ORANGE = (0.94, 0.49, 0.16)
FOCUS, APERTURE, LOCK = 0.212, 0.292, 0.371
CAP = (0.346, -0.127)
X0, Y, CAM_X, CAM_Z = 0.249, -0.056, 0.300, 0.9875
CHARTS = {"near": (0.029, 0.18), "far": (-0.029, 0.205)}


def box(x, y, z, dx, dy, dz, color):
    return ((x, y, z), (dx, dy, dz), color)


def tab(name, x, color):
    return Slider(name, (x, 0.098, 0.798), boxes=(box(x, 0.098, 0.798, 0.048, 0.027, 0.026, color),),
                  travel=0.015, rate=0.001, damping=8.0, mass=0.065, axis="Y")


chart_geometry = []
for offset, depth in CHARTS.values():
    top = CAM_Z - depth
    chart_geometry.append(box(X0 + offset, Y, (0.773 + top) / 2 - 0.001, 0.025, 0.021, top - 0.773, DARK))
    for i in range(10):
        color = (0.96, 0.96, 0.96) if i % 2 else (0.025, 0.025, 0.025)
        chart_geometry.append(box(X0 + offset + (i - 4.5) * 0.0023, Y, top, 0.0023, 0.018, 0.0006, color))


OBJECTS = [part("vision_fixture", pos=(0., 0., 0.), mass=15.0, friction=1.2, boxes=(
    box(0.302, -0.004, 0.7515, 0.366, 0.266, 0.008, DARK),
    box(0.303, Y, 0.765, 0.217, 0.042, 0.014, SILVER),
    box(0.470, -0.081, 0.911, 0.018, 0.018, 0.295, DARK),
    box(0.386, -0.068, 1.060, 0.183, 0.021, 0.016, DARK),
    box(CAM_X, Y, 1.039, 0.048, 0.044, 0.035, SILVER),
    box(*CAP, 0.764, 0.058, 0.032, 0.017, SILVER),
), sliders=(
    tab("focus", FOCUS, BLUE), tab("aperture", APERTURE, BLUE), tab("lock", LOCK, ORANGE),
    Slider("test", (*CAP, 0.790), boxes=(box(*CAP, 0.792, 0.039, 0.026, 0.017, GREEN),),
           travel=0.007, rate=2100.0, damping=14.0, mass=0.045, axis="Z"),
    Slider("stage", (X0, Y, 0.774), boxes=(box(X0, Y, 0.774, 0.088, 0.029, 0.010, SILVER), *chart_geometry),
           travel=0.104, rate=0.001, damping=95.0, mass=0.22, axis="+X"),
))]


def render_sensor(rgb, depth, speed, focus, aperture):
    focal_plane = 0.150 + 0.060 * focus
    stop = 1.8 + 6.2 * aperture
    defocus = 0.025**2 / stop * abs(1 / focal_plane - 1 / depth) * 0.06 / 0.000005
    motion = 0.025 / (depth - 0.025) * abs(speed) * 0.00025 / 0.000005
    blur = math.hypot(defocus, motion)
    pixels = np.asarray(rgb, dtype=float)[..., :3].mean(axis=-1)
    if pixels.max() > 1:
        pixels /= 255.0
    if blur > 0.05:
        radius = max(1, min(28, math.ceil(blur * 3)))
        kernel = np.exp(-0.5 * (np.arange(-radius, radius + 1) / blur) ** 2)
        kernel /= kernel.sum()
        pixels = np.apply_along_axis(lambda line: np.convolve(np.pad(line, radius, mode="edge"), kernel, "valid"), 1, pixels)
    exposed = np.clip(pixels * 2.7 * (0.00025 / 0.00035) * (4 / stop) ** 2, 0, 1)
    sample = exposed[54:74, 53:107]
    low, high = np.quantile(sample, (0.10, 0.90))
    contrast = float((high - low) / (high + low)) if high + low > 0 else 0.0
    clipped = float(np.mean(sample >= 0.999))
    return exposed, dict(contrast=contrast, blur_px=blur, clipped_fraction=clipped, speed_m_s=speed,
                         depth_m=depth, passed=contrast >= 0.30 and blur <= 1.0 and clipped <= 0.01 and 0.095 <= speed <= 0.105)


class Trial:
    def reset(self):
        self.lock = None
        self.delay = None
        self.started = self.moving = self.fault = False
        self.captures = {}
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
    fixture = base.scene["vision_fixture"]
    ids = [fixture.joint_names.index(name + "_slide") for name in ("focus", "aperture", "lock", "test", "stage")]
    a, b, lock, cap, stage = (float(v) for v in _to_numpy(fixture.data.joint_pos)[0, ids])
    speed = float(_to_numpy(fixture.data.joint_vel)[0, ids[-1]])
    root = _to_numpy(fixture.data.root_link_pose_w)[0]
    origin = _to_numpy(base.scene.env_origins)[0]
    trial.steps += 1
    trial.fault |= not all(math.isfinite(v) for v in (a, b, lock, cap, stage, speed))
    trial.fault |= math.dist(root[:3], origin) > 0.010 or abs(float(root[6])) < 0.998
    if -lock >= 0.012 and trial.lock is None:
        trial.lock = (a, b)
    if trial.lock is not None and (-lock < 0.012 or max(abs(a - trial.lock[0]), abs(b - trial.lock[1])) > 0.0008):
        trial.fault = True
    if not trial.started and trial.lock is not None and -cap >= 0.003:
        trial.started = True
        trial.delay = 5.0
    if trial.delay is not None:
        trial.delay -= base.step_dt
        if trial.delay <= 0.0:
            trial.moving, trial.delay = True, None
    if trial.moving and stage >= 0.097:
        trial.moving = False
    if trial.moving:
        for arm in ("left", "right"):
            x, y, z = pad_tips_world(base.scene[arm + "_robot"]) - origin
            if abs(x - (X0 + stage)) <= 0.053 and abs(y - Y) <= 0.038 and z <= 0.860:
                trial.fault = True
        for label, (offset, depth) in CHARTS.items():
            if label not in trial.captures and stage >= CAM_X - X0 - offset:
                rgb = _to_numpy(base.scene["inspection_cam"].data.output["rgb"])[0]
                image, trial.captures[label] = render_sensor(rgb, depth, speed, -a * 50, -b * 50)
                if os.getenv("TASK_TRACE"):
                    from PIL import Image
                    Image.fromarray((255 * image).astype(np.uint8)).save(Path(os.environ["TASK_TRACE"]).parent / f"{label}.png")
    fixture.actuators.target_command.set_velocity_index(
        value=torch.tensor([[0.1 if trial.moving and not trial.fault else 0.0]], device=base.device),
        joint_ids=[ids[-1]])
    count = sum(result["passed"] for result in trial.captures.values())
    trial.result = 0.0 if trial.fault else (0.25 if trial.lock is not None else 0.0, 0.5, 1.0)[count]
    if os.getenv("TASK_TRACE"):
        row = dict(step=trial.steps, focus_m=-a, aperture_m=-b, lock_m=-lock, test_m=-cap,
                   stage_m=stage, speed_m_s=speed, captures=trial.captures, fault=trial.fault, score=trial.result)
        with open(os.environ["TASK_TRACE"], "a", encoding="utf-8") as trace:
            trace.write(json.dumps(row) + "\n")


def score(objects):
    return trial.result


def success(objects):
    return trial.result == 1.0


def demo():
    paths = {"left": [], "right": []}
    homes = {"left": (0.192, 0.241, 0.874), "right": (0.192, -0.241, 0.874)}

    def phase(duration, arm=None, pos=None, label="wait", yaw=0.0, finger=-0.015):
        for side, path in paths.items():
            path.append(Waypoint(duration, pos if side == arm else homes[side], finger if side == arm else -0.015,
                                 label, yaw if side == arm else 0.0))

    phase(8.0 if os.getenv("SETTLE_ONLY") else 1.0)
    if os.getenv("SETTLE_ONLY"):
        return paths
    for x, label in ((FOCUS, "focus"), (APERTURE, "aperture"), (LOCK, "lock")):
        if label == "aperture" and os.getenv("SKIP_APERTURE"):
            continue
        phase(1.0, "left", (x, 0.151, 0.859), label + " approach", math.pi / 2)
        phase(0.8, "left", (x, 0.151, 0.809), label + " lower", math.pi / 2)
        phase(1.8, "left", (x, 0.110, 0.809), label + " push", math.pi / 2)
        phase(0.65, "left", (x, 0.110, 0.859), label + " lift", math.pi / 2)
    phase(0.85)
    phase(1.15, "right", (*CAP, 0.865), "TEST approach", finger=0.0)
    phase(1.15, "right", (*CAP, 0.789), "TEST descend", finger=0.0)
    phase(0.75, "right", (*CAP, 0.789), "TEST hold", finger=0.0)
    phase(0.7, "right", (*CAP, 0.865), "TEST release", finger=0.0)
    phase(7.2)
    return paths
