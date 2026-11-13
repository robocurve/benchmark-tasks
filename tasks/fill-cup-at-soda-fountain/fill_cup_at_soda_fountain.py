"""Fast-food counter: fill an empty paper cup at a lever soda valve. The drink runs only while the lever paddle is
pushed in; its spring pushes back, so the cup has to be held pressed against it with its mouth under the nozzle. The
drink level shows as cola layers appearing inside the cup."""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import trialkit as tk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
NOZZLE = (0.43, 0.15)
CUP_START = (0.30, 0.15)
R_LOW, R_TOP, CUP_H = 0.030, 0.045, 0.116
LAYERS, LAYER_H = 11, 0.0102
RATE = 0.016  # m of drink level per s while the lever is in
PRESSED_DEG = 6.0
STIFF = 0.03  # lever spring, N m per degree
COLA, PAPER, RED, STEEL, DARK = (0.22, 0.09, 0.04), (0.95, 0.95, 0.93), (0.80, 0.10, 0.12), (0.75, 0.76, 0.78), \
    (0.15, 0.15, 0.16)
PANEL, AMBER = (0.08, 0.20, 0.42), (0.95, 0.62, 0.10)
LOW_FILL, HIGH_FILL = CUP_H - 0.020, CUP_H - 0.005


def _quat(yaw, pitch):
    cz, sz, cy, sy = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (cz * cy, -sz * sy, cz * sy, sz * cy)


def _radius(z):
    return R_LOW + (R_TOP - R_LOW) * z / CUP_H


TAPER = math.atan((R_TOP - R_LOW) / CUP_H)
walls = [("box", (_radius(CUP_H / 2) * math.cos(t), _radius(CUP_H / 2) * math.sin(t), 0.0),
          (0.003, 0.024, CUP_H + 0.002), RED if k % 6 == 0 else PAPER, False, _quat(t, TAPER))
         for k in range(12) for t in [2 * math.pi * k / 12]]
liquid = [("cyl", (0, 0, 0.004 + LAYER_H * (i + 0.5) - CUP_H / 2), (_radius(0.004 + LAYER_H * (i + 1)) - 0.003, LAYER_H),
           ("hidden", COLA), False) for i in range(LAYERS)]
tk.parts("cup", *walls, *liquid)
tk.hide_base("cup")
FIRST_LAYER = len(walls)

tk.parts("valve_head",
         ("box", (0.0, 0.0, -0.025), (0.05, 0.07, 0.004), DARK, False),
         ("box", (-0.0305, 0.0, 0.010), (0.002, 0.05, 0.022), AMBER, False))
tk.hinge("paddle", "valve_head", (-0.015, 0.0, -0.030), "Y", (-15.0, 2.0), STIFF, 0.001)
tk.parts("fountain",
         ("box", (-0.0505, 0.0, 0.040), (0.002, 0.20, 0.16), PANEL, False),
         ("box", (-0.0505, 0.0, 0.135), (0.002, 0.20, 0.012), STEEL, False),
         ("box", (0.0, 0.0, 0.166), (0.104, 0.244, 0.012), STEEL, False))
tk.parts("drip_tray", *[("box", (0.0, -0.075 + 0.025 * k, 0.0062), (0.12, 0.008, 0.001), DARK, False)
                        for k in range(7)])
tk.parts("cup_sleeve", ("cyl", (0.0, 0.0, 0.07), (0.046, 0.012), RED, False),
         ("cyl", (0.0, 0.0, -0.02), (0.0405, 0.012), RED, False))
tk.parts("straw_holder", *[("cyl", (0.012 * math.cos(k), 0.012 * math.sin(k), 0.085), (0.0035, 0.05),
                            (0.92, 0.92, 0.90), False) for k in range(6)])
tk.parts("napkins", ("box", (0.0, 0.0, 0.046), (0.07, 0.11, 0.004), (0.97, 0.97, 0.96), False))

OBJECTS = [
    Box("counter", pos=(0.34, 0.0, TOP + 0.0015), size=(0.50, 0.80, 0.003), color=(0.30, 0.33, 0.37)),
    Box("fountain", pos=(0.55, 0.15, TOP + 0.16), size=(0.10, 0.24, 0.32), color=STEEL),
    Box("valve_head", pos=(0.47, 0.15, TOP + 0.205), size=(0.06, 0.08, 0.05), color=STEEL),
    Cylinder("nozzle", pos=(*NOZZLE, TOP + 0.170), radius=0.012, height=0.02, color=STEEL),
    Box("drip_tray", pos=(0.45, 0.15, TOP + 0.009), size=(0.14, 0.20, 0.012), color=STEEL),
    Box("paddle", pos=(0.455, 0.15, TOP + 0.135), size=(0.006, 0.05, 0.07), mass=0.03, color=STEEL),
    Cylinder("cup_sleeve", pos=(0.40, 0.33, TOP + 0.003 + 0.10), radius=0.044, height=0.20, color=PAPER),
    Cylinder("lid_stack", pos=(0.40, -0.10, TOP + 0.003 + 0.03), radius=0.047, height=0.06, color=(0.90, 0.90, 0.88)),
    Cylinder("straw_holder", pos=(0.47, -0.22, TOP + 0.003 + 0.06), radius=0.03, height=0.12, color=(0.75, 0.80, 0.82)),
    Box("napkins", pos=(0.50, -0.06, TOP + 0.003 + 0.045), size=(0.08, 0.12, 0.09), color=STEEL),
    Cylinder("cup", pos=(*CUP_START, TOP + 0.003 + CUP_H / 2), radius=0.038, height=CUP_H, mass=0.03, color=PAPER),
]

T = {"level": 0.0, "spilled": 0.0, "n": 0, "shown": 0}


@tk.on_reset
def _new_trial(env, rng):
    T.update(level=0.0, spilled=0.0, n=0, shown=0)
    for i in range(LAYERS):
        tk.show("cup", FIRST_LAYER + i, False)


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _paddle_deg(obj):
    x, y, z, w = obj.quat_xyzw
    return float(np.degrees(2 * math.atan2(y, w)))


def score(objects):
    """Every step: while the lever is pushed in past 6 degrees the drink runs, into the cup if its mouth is within
    2.5 cm of the nozzle and it is upright, otherwise onto the drip tray. 0 if the cup overflows, more than 0.5 cm
    of drink runs outside it, or it tips past 30 degrees. Otherwise 1.0 with the drink 0.5 to 2 cm below the rim
    and the cup set down upright and still; 0.5 with the cup at least half full and set down; else 0."""
    cup, paddle = objects["cup"], objects["paddle"]
    deg = -_paddle_deg(paddle)
    if deg > PRESSED_DEG:
        under = np.linalg.norm(cup.pos[:2] - np.array(NOZZLE)) < 0.025 and _tilt_deg(cup) < 15
        if under:
            T["level"] += RATE / 20
        else:
            T["spilled"] += RATE / 20
    shown = min(LAYERS, int(T["level"] / LAYER_H + 0.5))
    while T["shown"] < shown:
        tk.show("cup", FIRST_LAYER + T["shown"])
        T["shown"] += 1
    T["n"] += 1
    if os.environ.get("SODA_DEBUG") and T["n"] % 10 == 0:
        print(f"SODA t={T['n'] / 20:.1f} paddle={deg:.1f} level={T['level'] * 100:.1f}cm spilled="
              f"{T['spilled'] * 100:.1f}cm cup={np.round(cup.pos - [0, 0, TOP], 3)} tilt={_tilt_deg(cup):.1f}",
              flush=True)
    if T["level"] > CUP_H or T["spilled"] > 0.005 or _tilt_deg(cup) > 30:
        return 0.0
    set_down = cup.low[2] < TOP + 0.02 and _tilt_deg(cup) < 10 and cup.resting()
    if LOW_FILL <= T["level"] <= HIGH_FILL and set_down:
        return 1.0
    return 0.5 if T["level"] >= CUP_H / 2 and set_down else 0.0


def demo():
    """The left arm holds the cup from behind with its fingers round the cup's sides, so the hand stays clear of
    the nozzle, pushes the rim against the lever for 6.3 s, pulls back and sets the cup down. SODA_VARIANT scripts
    the wrong ways checked in the README."""
    variant = os.environ.get("SODA_VARIANT", "fill")
    hold = {"fill": 6.3, "overfill": 8.5, "release": 2.0, "underfill": 4.0, "offcentre": 6.3}[variant]
    dy = 0.035 if variant == "offcentre" else 0.0
    side = tk.Tilt(0.0, 1.3, 0.0)
    x0, y0 = CUP_START
    grip = TOP + 0.003 + CUP_H / 2
    lift = grip + 0.030
    left = [
        Waypoint(2.0, (x0 - 0.10, y0, grip + 0.03), FINGER_OPEN, "behind the cup", side),
        Waypoint(1.0, (x0 - 0.10, y0, grip), FINGER_OPEN, "down beside the counter", side),
        Waypoint(1.5, (x0, y0, grip), FINGER_OPEN, "fingers round the cup", side),
        Waypoint(0.8, (x0, y0, grip), FINGER_CLOSED, "grip", side),
        Waypoint(0.6, (x0, y0, lift), FINGER_CLOSED, "lift", side),
        Waypoint(2.0, (0.40, y0 + dy, lift), FINGER_CLOSED, "under the nozzle", side),
        Waypoint(1.0, (0.428, y0 + dy, lift), FINGER_CLOSED, "press the lever", side),
    ]
    if variant == "release":
        left += [Waypoint(0.6, (0.428, y0, lift), FINGER_OPEN, "let go", side),
                 Waypoint(hold, (0.33, y0, lift), FINGER_OPEN, "away", side)]
    else:
        left += [
            Waypoint(hold, (0.428, y0 + dy, lift), FINGER_CLOSED, "hold", side),
            Waypoint(0.6, (0.39, y0, lift), FINGER_CLOSED, "off the lever", side),
            Waypoint(2.0, (0.31, y0, lift), FINGER_CLOSED, "back", side),
            Waypoint(0.8, (0.31, y0, grip + 0.002), FINGER_CLOSED, "set down", side),
            Waypoint(0.6, (0.31, y0, grip + 0.002), FINGER_OPEN, "let go", side),
            Waypoint(1.0, (0.21, y0, grip + 0.03), FINGER_OPEN, "back out", side),
        ]
    left.append(Waypoint(2.0, (0.08, 0.30, TOP + 0.18), FINGER_OPEN, "out of the camera's view"))
    return {"left": left}
