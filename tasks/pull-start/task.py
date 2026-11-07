"""A small engine's recoil starter on a light plate, loose on the table: a finned dummy block with no fuel and no moving parts inside, a round starter cover, a yellow T-handle on a white cord, and a top pad to press the unit down by. The cord's spring pulls with 15 N at the full 30 cm. In the simulator the handle runs on a straight slide, which stands in for the cord.

Run a model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 300 --instruction "..."

The demo:  python demokit.py task.py OUT --mode full                 the final video
           python check_scorer.py                                    the scoring table, with no simulator
"""

from __future__ import annotations

import dataclasses
import os
import pathlib
import re
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Hold the engine down by its top pad with one arm, and with the other pull the yellow starter handle out fast over its full 30 cm, in under half a second. The engine must not shift more than 10 cm."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP

BASE_XY = (0.35, 0.12)  # the part's origin, under the top pad; the plate's middle is 8 cm further along y
PLATE = (0.20, 0.40, 0.012)
PLATE_MASS = 0.5  # light enough that a pull with nobody holding it cannot reach the full stroke within 10 cm
HANDLE_HOME_Y = -0.1435  # in the part's frame: the crossbar's middle with the handle at home, its socket against the cover
HANDLE = (0.071, 0.019, 0.024)  # the crossbar, across the pull; a real handle's overall size is about 7.1 x 5.3 x 1.9 cm. y is what the fingers close on
GRIP_OFF_X = -0.026  # the fingers close on the crossbar beside its neck
HANDLE_Z = 0.09
STROKE = 0.30
CORD_K = 50.0  # N/m: 15 N at the full stroke
BLOCK_TOP = 0.13
PAD_Y, PAD_Z = 0.0, BLOCK_TOP + 0.01  # the top pad's middle, and its top above the plate's underside

LOOKS = {
    "plate": m.Look(m.rgb(0.45, 0.47, 0.50), metallic=0.6, roughness=0.5),
    "block": m.Look(m.rgb(0.30, 0.31, 0.33), metallic=0.5, roughness=0.6),
    "cover": m.Look(m.rgb(0.75, 0.12, 0.08), roughness=0.5),
    "pad": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.9),
    "handle": m.Look(m.rgb(0.95, 0.75, 0.05), roughness=0.5),
    "cord": m.Look(m.rgb(0.90, 0.90, 0.88), roughness=0.8),
}

# The score.
FULL_STROKE = STROKE - 0.005
STROKE_LIMIT_S = 0.5
STARTED = 0.01
MOVED_LIMIT = 0.10  # the engine from where it started, either way


def _unit() -> m.Part:
    hz = HANDLE_Z
    fins = [m.box((0.14, 0.006, 0.05), (0, y, BLOCK_TOP - 0.03)) for y in np.arange(-0.05, 0.0601, 0.022)]
    frame = [
        ("plate", m.box(PLATE, (0, 0.08, PLATE[2] / 2)), "plate"),
        ("cover", m.cylinder(0.06, 0.03, (0, -0.085, hz), axis="y"), "cover"),
        ("grommet", m.cylinder(0.009, 0.004, (0, -0.1015, hz), axis="y"), "pad"),
        ("block", m.box((0.10, 0.14, BLOCK_TOP - PLATE[2]), (0, 0.0, (BLOCK_TOP + PLATE[2]) / 2)), "block"),
        ("fins", m.join(*fins), "block"),
        ("rear", m.box((0.09, 0.13, 0.09), (0, 0.135, PLATE[2] + 0.045)), "block"),
        ("pad", m.box((0.06, 0.06, 0.01), (0, PAD_Y, PAD_Z - 0.005)), "pad"),
    ]
    cols = [m.col_box("plate_c", PLATE, (0, 0.08, PLATE[2] / 2)), m.col_box("block_c", (0.10, 0.14, BLOCK_TOP - PLATE[2]), (0, 0.0, (BLOCK_TOP + PLATE[2]) / 2)), m.col_box("pad_c", (0.06, 0.06, 0.01), (0, PAD_Y, PAD_Z - 0.005))]
    bar_y = HANDLE_HOME_Y
    neck_y0 = bar_y + HANDLE[1] / 2
    bar = m.box(HANDLE, (0, bar_y, hz))
    bar_ends = m.join(*[m.cylinder(HANDLE[1] / 2, HANDLE[2], (sx * HANDLE[0] / 2, bar_y, hz)) for sx in (-1, 1)])
    neck = m.cylinder(0.006, 0.026, (0, neck_y0 + 0.013, hz), axis="y", segments=14)
    socket = m.cylinder(0.011, 0.012, (0, neck_y0 + 0.026 + 0.006, hz), axis="y", segments=18)
    cord = m.cylinder(0.002, 0.33, (0, neck_y0 + 0.038 + 0.165, hz), axis="y", segments=10)
    handle = m.slider("handle", "Y", -STROKE - 0.02, 0.0, [("bar", bar, "handle"), ("bar_ends", bar_ends, "handle"), ("neck", neck, "handle"), ("socket", socket, "handle"), ("cord", cord, "cord")], [m.col_box("handle_c", HANDLE, (0, bar_y, hz))], mass=0.05, spring=CORD_K, damping=0.2, friction=0.0, armature=1e-4)
    part = m.mechanism("unit", (*BASE_XY, TOP + 0.001), [handle], LOOKS, frame_parts=frame, frame_colliders=cols, friction=(0.8, 0.7))
    plain = part.build

    def build() -> str:
        """Loose on the table: no joint to the world, and the plate's own mass."""
        text = re.sub(r'\s*def PhysicsFixedJoint "world_joint"\s*\{[^}]*\}', "", plain())
        inertia = PLATE_MASS * (PLATE[0] ** 2 + PLATE[1] ** 2) / 12
        com = f"point3f physics:centerOfMass = (0, 0.08, 0.03)\n        float3 physics:diagonalInertia = ({inertia:.4f}, {inertia:.4f}, {inertia:.4f})"
        return text.replace("float physics:mass = 1\n        float3 physics:diagonalInertia = (0.01, 0.01, 0.01)", f"float physics:mass = {PLATE_MASS}\n        {com}", 1)

    return dataclasses.replace(part, build=build)


OBJECTS = [_unit()]


def task_score(moved: bool, stroke_s: float | None) -> tuple[float, bool]:
    """The scoring text's table, line by line. Returns the score, and whether that is the full one."""
    if moved or stroke_s is None or stroke_s > STROKE_LIMIT_S:
        return 0.0, False
    return 1.0, True


class Trial:
    """What the scorer holds across a trial. The starter is going once the handle has come out its full 30 cm within 0.5 s of starting to move. A slow pull can be tried again, from the handle back home, as on a real engine: `stroke_s` keeps the fastest."""

    def reset(self, env=None) -> None:
        self.t = 0.0
        self.began = None
        self.stroke_s = None
        self.counted = False
        self.moved = False
        self.start_xy = None
        self.most_moved = self.top_speed = 0.0
        self.last_travel = 0.0
        self.score, self.full = 0.0, False
        self.rep = {}
        self.n = 0

    def step(self, env) -> None:
        self.t += 1.0 / d.CONTROL_HZ
        self.n += 1
        if os.environ.get("TASK_STEP_FILE"):  # the simulated time, for anything following the run from outside
            with open(os.environ["TASK_STEP_FILE"], "w") as f:
                f.write(f"{self.t:.2f}")
        s = m.state(env, "unit")
        travel = -s.joint("handle")
        self.top_speed = max(self.top_speed, (travel - self.last_travel) * d.CONTROL_HZ)
        self.last_travel = travel
        if self.start_xy is None:
            self.start_xy = s.pos[:2].copy()
        shift = float(np.linalg.norm(s.pos[:2] - self.start_xy))
        self.most_moved = max(self.most_moved, shift)
        self.moved = self.moved or shift > MOVED_LIMIT
        if travel < STARTED:
            self.began, self.counted = None, False
        elif self.began is None:
            self.began = self.t
        if travel >= FULL_STROKE and self.began is not None and not self.counted:
            self.counted = True
            took = self.t - self.began
            self.stroke_s = took if self.stroke_s is None else min(self.stroke_s, took)
        self.score, self.full = task_score(self.moved, self.stroke_s)
        self.rep = {"t": round(self.t, 2), "score": self.score, "stroke_s": None if self.stroke_s is None else round(self.stroke_s, 3), "travel_m": round(travel, 4), "most_moved_m": round(self.most_moved, 4), "top_speed": round(self.top_speed, 3), "moved": self.moved}
        if self.n % 10 == 0 or self.full:
            print(f"[pull] {self.rep}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo --------------------------------------------------------------------------------

STROKE_CMD = 0.36  # the soft arm falls about 6 cm short of its command under the spring
STROKE_T = 0.40
PRESS_BELOW = 0.05  # how far below the pad's top the left arm is commanded: the soft arm presses with about 20 N
PARK = {"left": (0.25, 0.32, TOP + 0.20), "right": (0.25, -0.32, TOP + 0.20)}
TIP = 0.013  # fingertips past the TCP


def demo_script(ctx):
    """Closed loop on the unit's seen position: the grips go where the unit is. No scorer state is read."""
    mode, left, right = ctx.mode, ctx.left, ctx.right
    if mode not in ("full", "one_arm", "slow"):
        raise ValueError(f"unknown mode {mode!r}")
    at = ctx.state("unit").to_world(np.array([[GRIP_OFF_X, HANDLE_HOME_Y, HANDLE_Z], [0.0, PAD_Y, PAD_Z]]))
    handle, pad = at[0], at[1]
    if mode != "one_arm":
        yield from left.move(pad + [0.0, 0.0, TIP + 0.03], grip=d.down(0.0), finger=d.FINGER_CLOSED, t=2.0, label="over the top pad")
        yield from left.move(pad + [0.0, 0.0, TIP + 0.002], t=0.8, label="onto the top pad")
        yield from left.move(pad + [0.0, 0.0, TIP - PRESS_BELOW], t=0.6, label="press the unit down")
    yield from right.move(handle + [0.0, 0.0, 0.04], grip=d.down(0.0), finger=-0.03, t=2.0, label="over the handle")
    yield from right.move(handle, t=0.8, label="down round the crossbar")
    yield from right.close(width=HANDLE[1], force=25.0, t=0.4, label="close on the crossbar")
    yield from right.wait(0.3)
    # `one_arm` pulls as hard as the arm will go.
    far, took = (0.40, 0.30) if mode == "one_arm" else (STROKE_CMD, 1.0 if mode == "slow" else STROKE_T)
    yield from right.move(handle - [0.0, far, 0.0], t=took, label="pull")
    yield from right.wait(2.0)
