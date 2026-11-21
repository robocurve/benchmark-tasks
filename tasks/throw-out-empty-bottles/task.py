"""On the station table: four identical opaque white plastic bottles (7 cm across, 15 cm tall, 0.5 l, caps on) stand in a
row 32 cm in front of the arms. Two of them, drawn at random, are full (0.55 kg); the other two are empty (40 g).
They look the same; only lifting them tells. In front of the row, between the arms, stands a green recycling bin
(24 x 16 cm inside, room for all four bottles, 4 cm walls).

The robot must put the empty bottles into the recycling bin and leave the full ones standing on the table.

Simulation: with FULL unset, two bottles are drawn at random afresh for every trial (each reset sets the four
bottles' masses); FULL=0,2 fixes the full ones for every trial (0 = leftmost).
"""

from __future__ import annotations

import math
import os
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the empty bottles into the recycling bin. Leave the full ones on the table."

WHITE, CAP = (0.95, 0.95, 0.93), (0.92, 0.92, 0.90)
GREY, GREEN = (0.45, 0.47, 0.50), (0.15, 0.55, 0.25)
SIDES = 16
R, HB = 0.035, 0.15
BX = 0.32
BY = (0.15, 0.05, -0.05, -0.15)
NAMES = ("bottle_1", "bottle_2", "bottle_3", "bottle_4")
FULL_KG, EMPTY_KG = 0.55, 0.04

_PICK = os.environ.get("FULL")
_RNG = random.SystemRandom()


def draw() -> tuple[int, ...]:
    """Which two bottles are full: the ones FULL names, or two of the four at random."""
    return tuple(int(i) for i in _PICK.split(",")) if _PICK else tuple(sorted(_RNG.sample(range(4), 2)))


# "full": the bottles full in the current trial. "next": the next trial's draw, made one reset ahead because
# inspect-robots resets the policy (where the scripted demo makes its plan) before the embodiment (where the
# reset hook below applies the draw). Every reset makes "next" current and draws a new "next".
_DRAW: dict = {"full": draw()}
_DRAW["next"] = _DRAW["full"]


def ring(a_in, a_out, z0, z1, color, sides=SIDES):
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    return [(((a_in + a_out) / 2 * math.cos(t), (a_in + a_out) / 2 * math.sin(t), (z0 + z1) / 2),
             (a_out - a_in, side, z1 - z0), color, math.degrees(t))
            for t in (2 * math.pi * k / sides for k in range(sides))]


BOTTLE = ring(0, R, 0, HB - 0.03, WHITE) + ring(0, 0.026, HB - 0.03, HB - 0.012, WHITE) + ring(0, 0.018, HB - 0.012, HB, CAP)
BOTTLES = [part(n, pos=(BX, y, TOP), boxes=BOTTLE, mass=FULL_KG if i in _DRAW["full"] else EMPTY_KG, color=WHITE, friction=0.8)
           for i, (n, y) in enumerate(zip(NAMES, BY))]

# -- the recycling bin (fixed, centred between the arms) --------------------------------------------------------------
CX, CYB = 0.18, 0.0
IN_X, IN_Y, WALL, WH, FLOOR = 0.16, 0.24, 0.006, 0.04, 0.004


def container(name, y, color):
    ox, oy = IN_X / 2 + WALL / 2, IN_Y / 2 + WALL / 2
    return [Box(f"{name}_floor", pos=(CX, y, TOP + FLOOR / 2), size=(IN_X + 2 * WALL, IN_Y + 2 * WALL, FLOOR), color=color),
            Box(f"{name}_back", pos=(CX + ox, y, TOP + WH / 2), size=(WALL, IN_Y + 2 * WALL, WH), color=color),
            Box(f"{name}_front", pos=(CX - ox, y, TOP + WH / 2), size=(WALL, IN_Y + 2 * WALL, WH), color=color),
            Box(f"{name}_left", pos=(CX, y + oy, TOP + WH / 2), size=(IN_X, WALL, WH), color=color),
            Box(f"{name}_right", pos=(CX, y - oy, TOP + WH / 2), size=(IN_X, WALL, WH), color=color)]


OBJECTS = BOTTLES + container("bin", CYB, GREEN)


# -- the draw, applied at every reset ------------------------------------------------------------------------------
_STATE: dict = {"tipped": False, "fell": False, "trial": 0, "pending": None}
_HANDS: dict = {}                # each gripper's TCP (world, m), read after every control step
_UNIT: dict = {}                 # each bottle's inertia per kg as spawned (same shape full or empty: rigid prism)


def _set_masses(environment, full) -> None:
    """Give the four bottles this trial's masses (and inertias, scaled with the mass) through the physics view.
    If the physics view is not ready yet (it reads back no mass), nothing is written and the draw is applied after
    the next control step instead; values are written only if they are finite, and the result is read back."""
    import torch

    scene = environment.unwrapped.scene
    now = []
    for n in NAMES:
        m0 = float(scene[n].data.body_mass.torch.reshape(-1)[0])
        now.append(m0)
        if n not in _UNIT and m0 > 0:
            unit = scene[n].data.body_inertia.torch.reshape(-1)[:9].clone() / m0
            if bool(torch.isfinite(unit).all()) and float(unit.abs().sum()) > 0:
                _UNIT[n] = unit
    if not all(m > 0 and math.isfinite(m) for m in now) or len(_UNIT) < len(NAMES):
        _STATE["pending"] = full
        print(f"[bottles] trial {_STATE['trial']}: physics view not ready (masses read {now}); "
              f"the draw is applied after the next step", flush=True)
        return
    got = []
    for i, n in enumerate(NAMES):
        asset = scene[n]
        m = FULL_KG if i in full else EMPTY_KG
        asset.set_masses_index(masses=torch.full((1, 1), m, dtype=torch.float32, device=asset.device))
        asset.set_inertias_index(inertias=(_UNIT[n] * m).reshape(1, 1, 9).to(torch.float32))
        back = asset.root_view.get_masses()
        got.append(round(float(np.asarray(back.numpy() if hasattr(back, "numpy") else back).reshape(-1)[0]), 3))
    ok = all(abs(g - (FULL_KG if i in full else EMPTY_KG)) < 1e-3 for i, g in enumerate(got))
    _STATE["pending"] = None if ok else full
    print(f"[bottles] trial {_STATE['trial']}: full bottles {[i + 1 for i in full]}, masses in the physics view "
          f"(kg) {got}{'' if ok else ' (not applied yet; retried after the next step)'}", flush=True)


def _new_trial(environment=None) -> None:
    """After every reset: this trial's draw takes effect, the next one is drawn, the whole-trial flags clear."""
    _DRAW["full"], _DRAW["next"] = _DRAW["next"], draw()
    _STATE.update(tipped=False, fell=False, trial=_STATE["trial"] + 1)
    _HANDS.clear()
    if environment is not None:
        _set_masses(environment, _DRAW["full"])


def _hands(environment) -> None:
    """After every control step: where the two grippers' TCPs are (to tell whether a bottle was let go)."""
    from jointkit import _np, quat_to_matrix

    if _STATE.get("pending") is not None:
        _set_masses(environment, _STATE["pending"])
    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    for hand in ("left", "right"):
        robot = base.scene[f"{hand}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = _np(robot.data.body_link_pose_w)[0, i]
        _HANDS[hand] = pose[:3] - origin + quat_to_matrix(pose[3:7]) @ np.array([0.0, 0.0, 0.134])


on_reset(_new_trial)
on_step(_hands)


# -- scoring --------------------------------------------------------------------------------------------------------
TIPPED = math.radians(30)        # a bottle tilted more than this at any moment: tipped over
OFF_TABLE = TOP - 0.05           # a bottle's base this far below the table top: fell off
ON_FLOOR = 0.003                 # credited bottle: lowest point of its base within 3 mm of the bin floor's top
STILL = 0.01                     # credited bottle: moving under 1 cm/s
LET_GO = 0.09                    # credited bottle: both TCPs at least 9 cm from its centre line, so it is not
                                 # between the fingers (open pads 47 mm either side of the TCP; wall radius 3.5 cm)


def in_bin(b) -> bool:
    """Anywhere in the bin: centre inside the walls, base below the walls' mid-height (for the full-bottle rule)."""
    x, y, z = b.pos
    return bool(abs(x - CX) < IN_X / 2 and abs(y - CYB) < IN_Y / 2 and z < TOP + FLOOR + 0.02)


def base_low(b) -> float:
    """Height (m) of the lowest point of the bottle's base, a circle of radius R about the body origin."""
    u = b.rotation()[:, 2]
    return float(b.pos[2] - R * math.sqrt(max(0.0, 1.0 - u[2] ** 2)))


def hand_gap(b, tcp) -> float:
    """Distance (m) from a TCP to the bottle's centre line (base to cap)."""
    p = b.to_local(tcp)
    return float(math.hypot(p[0], p[1], p[2] - min(max(p[2], 0.0), HB)))


def resting_in_bin(b, hands=None) -> bool:
    """Standing on the bin floor and let go: centre inside the walls, the base's lowest point within 3 mm of the
    floor, upright (30 degrees or less), still (under 1 cm/s), and both TCPs at least 9 cm from its centre line."""
    hands = _HANDS if hands is None else hands
    x, y, _ = b.pos
    return bool(abs(x - CX) < IN_X / 2 and abs(y - CYB) < IN_Y / 2
                and abs(base_low(b) - (TOP + FLOOR)) <= ON_FLOOR
                and b.tilt() <= TIPPED
                and np.linalg.norm(b.lin_vel) < STILL
                and all(hand_gap(b, t) >= LET_GO for t in hands.values()))


def score(objects) -> float:
    """Checked after every control step. 0 for the whole trial once any bottle has tilted more than 30 degrees or
    fallen off the table (latched until the next reset), and 0 while a full bottle is in the bin. Otherwise 0.5 for
    each empty bottle resting on the bin floor: 1.0 with both. The value at the end of the episode counts."""
    for n in NAMES:
        b = objects[n]
        for flag, now in (("tipped", b.tilt() > TIPPED), ("fell", b.pos[2] < OFF_TABLE)):
            if now and not _STATE[flag]:
                _STATE[flag] = True
                print(f"[bottles] {n} {flag} (tilt {math.degrees(b.tilt()):.0f} deg, base z "
                      f"{(b.pos[2] - TOP) * 1000:.0f} mm): the trial scores 0", flush=True)
    if _STATE["tipped"] or _STATE["fell"]:
        return 0.0
    total = 0.0
    for i, n in enumerate(NAMES):
        b = objects[n]
        if i in _DRAW["full"]:
            if in_bin(b):
                return 0.0
        else:
            total += 0.5 * resting_in_bin(b)
    return total


# No success(): the trial does not end early once both empty bottles are in the bin, so every bottle gets tested;
# the score at the end of the episode counts (SCORE_OVER = "end").


# -- scripted demo: test-lift every bottle; full ones go straight back down, empty ones into the bin -----------------
OPEN, PINCH = -0.04695, -(R - 0.006)
GRASP = TOP + 0.10              # pads on the body 10 cm up, above the centre of mass: it hangs
TEST = TOP + 0.14               # lifted 4 cm to feel the weight
UP = TOP + 0.175                # bottoms clear the 4 cm walls by 1.5 cm
HOP = TOP + 0.175               # open fingertips pass 1 cm over the 15 cm bottle tops
YAW = math.pi / 2               # fingers close along x (neighbours stand along y)
SLOTS = [0.045, -0.045]         # places in the bin, filled in turn


def _test(i, yaw):
    x, y = BX, BY[i]
    return [(1.5, (x, y, HOP), OPEN, "over the bottle", yaw), (1.2, (x, y, GRASP), OPEN, "down", yaw),
            (0.8, (x, y, GRASP), PINCH, "grip", yaw), (1.2, (x, y, TEST), PINCH, "lift it: how heavy?", yaw),
            (0.8, (x, y, TEST), PINCH, "lift it: how heavy?", yaw)]


def _keep(i, yaw):
    x, y = BX, BY[i]
    return [(1.2, (x, y, GRASP + 0.002), PINCH, "full: set it back", yaw), (0.6, (x, y, GRASP + 0.002), OPEN, "let go", yaw),
            (1.0, (x, y, HOP), OPEN, "up", yaw)]


def _throw(i, slot, yaw):
    place = TOP + FLOOR + 0.004 + 0.10
    return [(1.5, (CX, slot, UP), PINCH, "empty: into the bin", yaw), (1.0, (CX, slot, place), PINCH, "lower", yaw),
            (0.6, (CX, slot, place), OPEN, "let go", yaw), (1.0, (CX, slot, HOP), OPEN, "up", yaw)]


def plan(full=None) -> dict[str, list]:
    """The left arm tests its two bottles, then moves away; only then does the right arm start on its two, so the
    hands never share the bin. ``full``: the draw to plan for (default: the coming trial's)."""
    full = _DRAW["next"] if full is None else full
    slots = list(SLOTS)
    park_l, park_r = (0.20, 0.30, UP), (0.20, -0.30, UP)
    left, right = [], []
    for i in (0, 1):
        left += _test(i, -YAW) + (_keep(i, -YAW) if i in full else _throw(i, slots.pop(0), -YAW))
    left += [(1.5, park_l, OPEN, "away", -YAW)]
    t_left = sum(w[0] for w in left)
    right += [(t_left, park_r, OPEN, "wait for the left arm", YAW)]
    for i in (3, 2):
        right += _test(i, YAW) + (_keep(i, YAW) if i in full else _throw(i, slots.pop(0), YAW))
    right += [(1.5, park_r, OPEN, "away", YAW)]
    return {"left": left, "right": right}


def demo():
    """Every bottle is lifted 4 cm to feel its weight: a full one is set straight back down, an empty one goes into
    the bin. The plan follows the coming trial's draw (made before the reset that applies it), as an operator would
    after lifting each bottle."""
    from yam_arena.demo import Waypoint

    return {arm: [Waypoint(d, tuple(p), f, label, yaw) for d, p, f, label, yaw in wps] for arm, wps in plan().items()}
