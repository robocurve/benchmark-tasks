"""Hang one connected reinforced edge on a pair of Moen SR2201 hooks."""
from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from plan import OPEN, TABLE, TIP, Track, waypoints, check_tracks
from runtime import bodies, contacts, register, reset_contacts, tick

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 860
INSTRUCTION = "Hang the two loose grommets of the shower liner on the inner hooks. Let go of the liner."
ROD = (0.340, 0.0, TABLE + 0.122)
YS = (-0.126, 0.048)
CENTER_Y = sum(YS) / 2
TIP_X = ROD[0] - 0.037
TIP_Z = TABLE + 0.079
NEST_Z = TABLE + 0.055
START = (0.190, CENTER_Y, TABLE + 0.032)
GRASP_X = -0.023
EYE_ID = 0.0160
LINER_SURFACE = np.array([
    (x + fold, y, z)
    for i, y in enumerate(np.linspace(-0.116, 0.116, 28))
    for x, z, fold in (
        (-0.033, 0.0, 0.0),
        (-0.038, -0.004, 0.0015 * np.cos(4 * np.pi * i / 27)),
        (-0.042, -0.010, 0.0020 * np.cos(4 * np.pi * i / 27)),
        (-0.046, -0.016, 0.0020 * np.cos(4 * np.pi * i / 27)),
    )
])


@dataclass
class Trial:
    above: list = field(default_factory=lambda: [False, False])
    value: float = 0.0
    hold: float = 0.0

    def step(self, centers, released, dt):
        landed = 0
        for i, (x, y, z) in enumerate(centers):
            centered = abs(x - TIP_X) < 0.007 and abs(y - YS[i]) < 0.006
            self.above[i] |= centered and TIP_Z + 0.001 < z < TIP_Z + 0.035
            landed += bool(
                self.above[i] and centered and NEST_Z - 0.006 < z < NEST_Z + 0.010
            )
        self.hold = self.hold + dt if landed == 2 and released else 0.0
        self.value = 1.0 if self.hold >= 1.0 - 1e-8 else 0.35 * landed


_trial = Trial()
_state = {}


def score(objects):
    return _trial.value


def success(objects):
    return score(objects) == 1.0


def telemetry():
    return dict(_state, passed_above=_trial.above, hold_s=round(_trial.hold, 2))


def setup():
    from scenekit import UsdBody

    items = [
        UsdBody(
            "shower_rail",
            usd="assets/shower_rail.usda",
            pos=ROD,
            size=(0.18, 1.20, 0.17),
            mass=None,
        ),
        UsdBody(
            "liner",
            usd="assets/liner.usda",
            pos=START,
            size=(0.060, 0.232, 0.0195),
            mass=0.049,
            friction=(0.78, 0.57),
            torsional_patch=0.008,
            solver_iters=32,
        ),
        UsdBody(
            "liner_rest",
            usd="assets/stage.usda",
            pos=(*START[:2], TABLE),
            size=(0.078, 0.26, 0.032),
            mass=None,
        ),
    ]

    def reset(env):
        global _trial
        _trial = Trial()
        _state.clear()
        reset_contacts()
        if os.environ.get("FAILSAFE_SIDE_CAMERA") == "1":
            import torch

            scene = env.unwrapped.scene
            origin = getattr(scene.env_origins, "torch", scene.env_origins)
            eyes = origin + torch.tensor((0.04, -0.50, 0.98), device=origin.device)
            targets = origin + torch.tensor((0.26, CENTER_Y, 0.805), device=origin.device)
            scene["top_cam"].set_world_poses_from_view(eyes, targets)
        engine = env.unwrapped.sim
        camera = getattr(engine, "set_camera_view", None)
        gui = getattr(engine, "has_gui", False)
        if camera and (gui() if callable(gui) else gui):
            camera((-0.09, -0.58, 1.19), (0.244, CENTER_Y, 0.804))

    def update(env):
        liner = bodies(env, ["liner"])["liner"]
        rot = liner.rotation()
        centers = [liner.pos + rot @ np.array((0.0, y - CENTER_Y, 0.0)) for y in YS]
        free = not contacts(env, ["liner"])["liner"] and liner.resting(0.025, 0.27)
        clearance = float(liner.pos[2] + min(LINER_SURFACE @ rot[2]) - TABLE)
        _trial.step(centers, free and clearance >= -0.001, tick(env))
        _state.update(
            released=free,
            liner_clearance_mm=round(1000 * clearance, 1),
            eye_errors_mm=[
                np.round((p - np.array((TIP_X, y, NEST_Z))) * 1000, 1).tolist()
                for p, y in zip(centers, YS)
            ],
        )

    return register(items, reset, update)


def tracks():
    left, right = Track(), Track()
    clear = TABLE + 0.126
    for track, i, park_y in ((right, 0, -0.224), (left, 1, 0.191)):
        y = YS[i]
        pinch = (START[0] + GRASP_X, y, START[2] + TIP - 0.002)
        above = (TIP_X + GRASP_X, y, TIP_Z + 0.014 + TIP - 0.002)
        low = (above[0], y, NEST_Z + 0.006 + TIP - 0.002)
        track.go((*pinch[:2], clear), OPEN, "hover above reinforced hem", 1.7)
        track.go(pinch, OPEN, "lower around hem with eye unobstructed", 1.55)
        track.go(pinch, -0.008, "pinch the continuous hem", 1.0)
        track.go((*pinch[:2], clear), -0.008, "lift both ends together", 1.6)
        track.go(above, -0.008, "align both eyes over the paired hooks", 2.35)
        track.go(low, -0.008, "thread both ball caps through the eyes", 2.35)
        track.go(low, -0.020, "release the shared hem", 0.85)
        track.go((*low[:2], clear), -0.020, "lift both hands clear", 1.5)
        track.go((0.164, park_y, clear), OPEN, "park", 1.55)
    return {"left": left, "right": right}


def demo():
    ts = tracks()
    return waypoints(ts["left"], ts["right"])


if __name__ == "__main__":
    t = Trial()
    low = [(TIP_X, y, NEST_Z) for y in YS]
    t.step(low, True, 1.0)
    assert t.value == 0
    t.step([(TIP_X, y, TIP_Z + 0.010) for y in YS], False, 0.1)
    t.step(low, False, 1.0)
    assert t.value == 0.70
    t.step(low, True, 1.0)
    assert t.value == 1.0
    check_tracks(tracks(), MAX_STEPS, ik="--ik" in sys.argv)
    print("Continuous two-eye liner tests passed.")
else:
    OBJECTS = setup()
