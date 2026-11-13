"""Roll a cabinet shelf wider than its face-frame opening through the opening, level it inside within a millimetre and a half of the sides, and lower it onto its four pins.

A 15 in face-frame wall cabinet, 18 in tall, stands on the table with its opening towards the robot: inside 343 mm wide, but the face frame's stiles narrow the opening to
305 mm. The shelf, 340 x 265 x 16 mm melamine particleboard, 0.98 kg, is wider than the opening, so it goes in rolled about 35 degrees, is levelled about its own centre once
inside (1.5 mm spare a side), and comes down onto four 5 mm pins standing 6 mm out of the sides. All rigid, no joints.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --mode short,notback,tilted --hold 1   failure checks
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the shelf in the cabinet on its four pins."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 1 s hold, so score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz

IN = 0.0254
W, D, H = 15 * IN, 12 * IN, 18 * IN  # the case outside: 381 x 305 x 457 mm
T_CASE, T_FRAME, T_BACK = 0.019, 0.019, 0.006
STILE, RAIL = 0.038, 0.038
CX = 0.610  # the case's middle: far enough back that the shelf can lie in front of it, within the arms' reach
X_FRONT = CX - D / 2  # the face frame's front
X_IN0, X_IN1 = X_FRONT + T_FRAME, CX + D / 2 - T_BACK  # inside, front to back
Y_IN = W / 2 - T_CASE  # inside half-width: 171.5 mm
Y_OPEN = W / 2 - STILE  # the opening's half-width: 152.5 mm
Z_FLOOR, Z_ROOF = T_CASE, H - T_CASE
SW, SD, ST = 0.340, 0.265, 0.016  # the shelf: width (y), depth (x), thickness
S_MASS = SW * SD * ST * 680.0
PIN_R, PIN_OUT = 0.0025, 0.006
Z_PIN = float(os.environ.get("SHELF_PIN_Z", "0.200"))  # the pins' axes, above the table
PIN_X = (X_IN0 + 0.037, X_IN1 - 0.037)
PIN_TOP = Z_PIN + PIN_R
LOOKS = {"white": m.Look(color=m.rgb(0.92, 0.91, 0.88), roughness=0.45), "frame": m.Look(color=m.rgb(0.72, 0.55, 0.36), roughness=0.55),
         "back": m.Look(color=m.rgb(0.86, 0.85, 0.82), roughness=0.6), "brass": m.Look(color=m.rgb(0.78, 0.62, 0.30), metallic=0.85, roughness=0.3),
         "shelf": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.4), "foam": m.Look(color=m.rgb(0.25, 0.27, 0.30), roughness=0.9)}


def _box(parts, cols, name, size, centre, look):
    parts.append((name, m.box(size, centre), look))
    cols.append(m.col_box(f"{name}_c", size, centre))


def _case() -> m.Part:
    parts, cols = [], []
    for k, sy in enumerate((-1, 1)):  # sides, full depth behind the frame
        _box(parts, cols, f"side{k}", (D - T_FRAME, T_CASE, H), (X_IN0 + (D - T_FRAME) / 2, sy * (W / 2 - T_CASE / 2), H / 2), "white")
        _box(parts, cols, f"stile{k}", (T_FRAME, STILE, H), (X_FRONT + T_FRAME / 2, sy * (W / 2 - STILE / 2), H / 2), "frame")
    _box(parts, cols, "floor", (D - T_FRAME, 2 * Y_IN, T_CASE), (X_IN0 + (D - T_FRAME) / 2, 0.0, T_CASE / 2), "white")
    _box(parts, cols, "roof", (D - T_FRAME, 2 * Y_IN, T_CASE), (X_IN0 + (D - T_FRAME) / 2, 0.0, H - T_CASE / 2), "white")
    _box(parts, cols, "back", (T_BACK, 2 * Y_IN, H - 2 * T_CASE), (X_IN1 + T_BACK / 2, 0.0, H / 2), "back")
    for k, zc in enumerate((RAIL / 2, H - RAIL / 2)):
        _box(parts, cols, f"rail{k}", (T_FRAME, W - 2 * STILE, RAIL), (X_FRONT + T_FRAME / 2, 0.0, zc), "frame")
    for k, (px, sy) in enumerate([(px, sy) for px in PIN_X for sy in (-1, 1)]):
        pin = m.cylinder(PIN_R, PIN_OUT, (px, sy * (Y_IN - PIN_OUT / 2), Z_PIN), axis="y", segments=12)
        parts.append((f"pin{k}", pin, "brass"))
        cols.append(m.col_hull(f"pin{k}_c", pin))
    return m.fixed("case", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.45))


SHELF0 = np.array([0.185 + SD / 2, 0.0, TOP + 0.090 + ST / 2 + 0.0005])  # lying on two foam blocks 9 cm tall, its front edge free at x 0.185


def _blocks() -> m.Part:
    parts, cols = [], []
    for k, sy in enumerate((-1, 1)):
        _box(parts, cols, f"block{k}", (0.20, 0.06, 0.090), (SHELF0[0] + 0.03, sy * 0.155, 0.045), "foam")
    return m.fixed("blocks", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.8, 0.7))


def _shelf() -> m.Part:
    return m.free("shelf", tuple(SHELF0), [("shelf", m.box((SD, SW, ST)), "shelf")], [m.col_box("c", (SD, SW, ST))], LOOKS, mass=S_MASS, friction=(0.5, 0.45),
                  torsional_patch=None)


OBJECTS = [_case(), _blocks(), _shelf()]


def measure(s) -> dict:
    """From the shelf's pose: each pin's top under the shelf's underside, the shelf's attitude, its back edge's gap to the back panel, and whether it is inside."""
    rot = s.rotation()
    gaps = []
    for px in PIN_X:
        for sy in (-1, 1):
            top = np.array([px, sy * (Y_IN - PIN_OUT / 2), TOP + PIN_TOP])
            p = (top - s.pos) @ rot  # the pin's top in the shelf's frame
            under = abs(p[0]) <= SD / 2 and abs(p[1]) <= SW / 2
            gaps.append(float(-ST / 2 - p[2]) if under else float("nan"))  # shelf underside above the pin top (negative: through it)
    roll = math.degrees(math.atan2(rot[2, 1], rot[2, 2]))
    pitch = math.degrees(math.asin(-float(np.clip(rot[2, 0], -1, 1))))
    yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    back = X_IN1 - (s.pos[0] + abs(rot[0, 0]) * SD / 2 + abs(rot[0, 1]) * SW / 2)
    inside = X_IN0 < s.pos[0] < X_IN1 and abs(s.pos[1]) < Y_IN and Z_FLOOR < s.pos[2] - TOP < Z_ROOF
    return {"gaps_mm": [None if math.isnan(g) else round(g * 1000, 2) for g in gaps], "roll": round(roll, 2), "pitch": round(pitch, 2), "yaw": round(yaw, 2),
            "back_mm": round(back * 1000, 1), "inside": bool(inside), "z_mm": round((s.pos[2] - TOP) * 1000, 1)}


class Tracker:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.last, self.clear, self.hold = 0, {}, False, 0

    def step(self, env):
        self.steps += 1
        s = m.state(env, "shelf")
        r = measure(s)
        tips = m.fingertips(env)
        local = (tips - s.pos) @ s.rotation()
        out = np.maximum(np.abs(local) - np.array([SD / 2, SW / 2, ST / 2]), 0.0)
        self.clear = bool(np.min(np.linalg.norm(out, axis=1)) > 0.03)
        self.last = {**r, "grippers_clear": self.clear}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0  # steps in a row with every 1.0 condition met
        self.last["hold"] = self.hold
        if self.steps % 5 == 0:
            print("[shelf] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "t": round(self.steps * float(env.unwrapped.step_dt), 2), **self.last}), flush=True)


TRACK = Tracker()
m.on_step(TRACK.step)
m.on_reset(TRACK.reset)


def _tier(r) -> float:
    if not r or not r["inside"] or not r["grippers_clear"]:
        return 0.0
    gaps = [g for g in r["gaps_mm"] if g is not None]
    on = [g for g in gaps if -0.5 <= g <= 1.0]
    level = abs(r["roll"]) <= 1.0 and abs(r["pitch"]) <= 1.0
    square = abs(r["yaw"]) <= 1.0
    if len(on) == 4 and level and square and r["back_mm"] <= 10.0:
        return 1.0
    if len(on) == 4:
        return 0.5
    if len(on) == 3 or (r["z_mm"] > Z_PIN * 1000):
        return 0.25
    return 0.0


def score(objects) -> float:
    """1.0 once every 1.0 condition has held HOLD_STEPS steps in a row; until then a shelf meeting them scores 0.5."""
    if TRACK.hold >= HOLD_STEPS:
        return 1.0
    return min(_tier(TRACK.last), 0.5)


def success(objects) -> bool:
    return TRACK.hold >= HOLD_STEPS


# -- the scripted demo --------------------------------------------------------------------------

ROLL = float(os.environ.get("SHELF_ROLL", "-35"))  # this sense keeps both wrists inside their limits


def _pinch(side: str):
    """Fingers level, pointing into the cabinet (+x), closing up and down on the shelf's 16 mm front edge."""
    g = d.tilted(90.0, 0.0)
    return d._as_grip(g.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi / 2))


def demo_script(ctx):
    s = ctx.pos("shelf")  # a scripted reference: reads the shelf's pose, which the cameras see
    depth = 0.030  # how far the fingers reach over the shelf from its front edge
    arms = [("left", 0.09), ("right", -0.09)]
    front = s[0] - SD / 2

    def grip_at(x_front, y, z):
        return np.array([x_front + depth - 0.0133, y, z])

    yield from ctx.together(*[getattr(ctx, a).move(grip_at(front - 0.03, y, s[2] + 0.07), grip=_pinch(a), finger=d.FINGER_OPEN, t=2.5, label="above, in front of the shelf") for a, y in arms])
    yield from ctx.together(*[getattr(ctx, a).move(grip_at(front - 0.03, y, s[2]), t=1.2, label="down in front of the edge") for a, y in arms])
    yield from ctx.together(*[getattr(ctx, a).move(grip_at(front, y, s[2]), t=1.2, label="fingers over and under the front edge") for a, y in arms])
    yield from ctx.together(*[getattr(ctx, a).close(ST, force=45.0, label="pinch") for a, _ in arms])
    zc = Z_PIN + TOP + 0.090  # carried high: the wrists let its back edge droop several centimetres as it goes in, and it must stay above the pins
    yield from ctx.together(*[getattr(ctx, a).move(grip_at(front, y, zc), t=2.0, label="lift") for a, y in arms])
    axis = np.array([1.0, 0.0, 0.0])
    yield from ctx.together(*[getattr(ctx, a).arc((0.0, 0.0, zc), axis, ROLL, t=2.0, label="roll it narrower than the opening") for a, _ in arms])
    # forward through the opening, until the back edge is 6 mm off the back panel
    dx = (X_IN1 - 0.006 - SD) - front - {"short": 0.10, "notback": 0.025}.get(ctx.mode, 0.0)  # failure checks: pushed in 10 cm or 2.5 cm short
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array([dx, 0.0, 0.0]), t=3.0, label="through the opening") for a, _ in arms])
    def attitude():
        st = ctx.state("shelf")
        rot = m.quat_matrix(st.quat_xyzw)
        return np.asarray(st.pos, float), math.degrees(math.atan2(rot[2, 1], rot[2, 2])), math.degrees(math.atan2(-rot[2, 0], rot[0, 0]))

    # level it about its own centre, read from its pose, and take out the pitch the wrists let in under its weight
    for k in range(4):
        c, roll, pitch = attitude()
        if abs(roll) < 0.4 and abs(pitch) < 0.4:
            break
        yield from ctx.together(*[getattr(ctx, a).arc(c, np.array([1.0, 0.0, 0.0]), -roll, t=1.2 if k == 0 else 0.6, label="level it about its centre") for a, _ in arms])
        c, roll, pitch = attitude()
        yield from ctx.together(*[getattr(ctx, a).arc(c, np.array([0.0, 1.0, 0.0]), -pitch, t=0.5, label="take out the pitch") for a, _ in arms])
    if ctx.mode == "tilted":  # a failure check: lowered with its left end 4 degrees high
        c, _, _ = attitude()
        yield from ctx.together(*[getattr(ctx, a).arc(c, np.array([1.0, 0.0, 0.0]), 4.0, t=0.5, label="tilt it") for a, _ in arms])
    # down until its underside is 1 mm over the pin tops, then let the pins take it
    c, _, _ = attitude()
    dz = (TOP + PIN_TOP + ST / 2 + 0.001) - c[2]
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array([0.0, 0.0, dz]), t=2.0, label="down onto the pins") for a, _ in arms])
    yield from ctx.together(*[getattr(ctx, a).open(label="let go") for a, _ in arms])
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array([-0.08, 0.0, 0.0]), t=1.5, label="out of the cabinet") for a, _ in arms])
    yield from ctx.left.wait(1.5)
    print("[shelf] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
