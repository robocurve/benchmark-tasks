"""Trial scene for check (a): candles standing in cupcakes, held by the law of candles.py, nothing else touching them.

Tilted row (6 cupcakes, x 0.22 and 0.30): each cupcake (0.3 kg here, to be a firm stand) turns on a hinge to the world
about a level axis along its base's far edge (it tips away from the arms), driven by a stiff spring whose rest this file moves: a jolt of 3 degrees at 1 s
(0.1 s out and back: the nudge), then from 3 s on a tilt of 7.5 degrees a second, to 60 (the kitchen test's "tilt
the cupcake until each candle falls over"). A candle stands 8, 12 and 15 mm deep in each three; the near three are
held as the spec's table has it (0.02 N m per radian at 8 mm), the far three with a hundredth of that.

Resistance row (7 cupcakes fixed to the bench, x 0.38 and 0.44): a candle 12 mm deep in each, pushed straight down
with 0.9 of its resistance along its length (1 N) for 3 s, then 1.1 of it until it is 25 mm in; the variants differ
in how the resistance is built (candle mass, the slide's stiffness and damping, its rest following the candle or not).
(Trial w7c-a1 tilted cupcakes moved by write_pose, kinematic: a weld to them did not drag the candles' chains.)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import candles as cd  # noqa: E402
import mechkit as mk  # noqa: E402
from mechkit import body, cylinder, hinge  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = "Check of the candle in the cake: nothing to do."
TOP = TABLE_TOP["station"]

CAKE_H, CAKE_R_TOP = 0.035, 0.030
CAKE_LAYERS = ((0.000, 0.007, 0.026), (0.007, 0.014, 0.027), (0.014, 0.021, 0.028), (0.021, 0.029, 0.029))  # z0, z1, r
CHOC, ICING = (0.24, 0.13, 0.07), (0.16, 0.08, 0.05)
COLORS = ((0.95, 0.35, 0.55), (0.25, 0.55, 0.95), (0.98, 0.85, 0.20), (0.35, 0.80, 0.40))
K_TABLE, K_LOW = 0.02, 0.0002
NUDGE_AT, NUDGE_DEG = 1.0, 3.0
TILT_FROM, TILT_RATE, TILT_MOST = 3.0, math.radians(7.5), math.radians(60.0)
STAND_MASS, STAND_K, STAND_DAMP = 0.3, 20.0, 0.2  # a tilting cupcake: kg, N m/rad, N m s/rad
PUSH_PARTS = (0.9, 1.1)


def cupcake_shapes() -> list:
    shapes = [cylinder(r, z1 - z0, pos=(0.0, 0.0, (z0 + z1) / 2), color=CHOC) for z0, z1, r in CAKE_LAYERS]
    shapes.append(cylinder(CAKE_R_TOP, CAKE_H - 0.029, pos=(0.0, 0.0, (CAKE_H + 0.029) / 2), color=ICING))
    return shapes


def make_host(name: str, k_at: float) -> None:
    cd.host(name, CAKE_H, CAKE_R_TOP - 0.0025, resist_top=3.0, top_depth=0.002, resist_body=1.0, k_at=k_at, tear_deg=25.0)


TILTED: list[str] = []
for _row, (_x, _k) in {"near": (0.22, K_TABLE), "far": (0.30, K_LOW)}.items():
    for _i, (_y, _d) in enumerate(zip((0.10, 0.0, -0.10), (0.008, 0.012, 0.015))):
        _cake = f"cake_{_row}{_i}"
        body(_cake, cupcake_shapes(), pos=(_x, _y, TOP + 0.0005), mass=STAND_MASS, color=CHOC)
        hinge(f"{_cake}_tilt", _cake, None, anchor=(_x + CAKE_LAYERS[0][2], _y, TOP + 0.0005), axis=(0.0, 1.0, 0.0), limits=(-0.2, 1.3),
              spring=STAND_K, damping=STAND_DAMP)
        make_host(_cake, _k)
        cd.declare(f"candle_{_row}{_i}", (_x, _y, TOP + 0.0005 + CAKE_H - _d), (0.0, 0.0, 0.0, 1.0), (_cake,), COLORS[_i],
                   built_in=_cake, built_depth=_d)
        TILTED.append(_cake)

# the resistance variants: (candle mass kg, slide N/m, slide damping s, rest follows)
VARIANTS = {"v0": (0.001, 5000.0, 0.01, True), "v1": (0.001, 50000.0, 0.01, True), "v2": (0.001, 5000.0, 0.001, True),
            "v3": (0.010, 5000.0, 0.01, True), "v4": (0.010, 50000.0, 0.001, True), "v5": (0.001, 5000.0, 0.0, True),
            "v6": (0.001, 5000.0, 0.01, False)}  # fmt: skip
PUSHED: dict[str, str] = {}
for _j, (_v, (_m, _ks, _ds, _f)) in enumerate(VARIANTS.items()):
    _x, _y = (0.38, 0.15 - 0.10 * _j) if _j < 4 else (0.46, 0.15 - 0.10 * (_j - 4))
    _cake = f"cake_{_v}"
    body(_cake, cupcake_shapes(), pos=(_x, _y, TOP), color=CHOC)  # fixed to the bench
    make_host(_cake, K_LOW)
    cd.declare(f"candle_{_v}", (_x, _y, TOP + CAKE_H - 0.012), (0.0, 0.0, 0.0, 1.0), (_cake,), COLORS[_j % 4],
               built_in=_cake, built_depth=0.012, mass=_m, slide_k=_ks, slide_damp_s=_ds, follows=_f)
    PUSHED[f"candle_{_v}"] = _cake

OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def tilt_at(t: float) -> float:
    """rad the tilted cupcakes' springs rest at, at time t."""
    if NUDGE_AT <= t < NUDGE_AT + 0.1:
        return math.radians(NUDGE_DEG)
    return min(max(t - TILT_FROM, 0.0) * TILT_RATE, TILT_MOST)


@mk.on_reset
def start(seed) -> None:
    STATE.update(pushing={name: 0.0 for name in PUSHED})
    if mk.MECH.environment is not None:
        cd.reset()
        for cake in TILTED:
            mk.set_spring(f"{cake}_tilt", rest=0.0)


@mk.on_step
def step() -> None:
    if mk.MECH.environment is None:
        return
    t = mk.now()
    for cake in TILTED:
        mk.set_spring(f"{cake}_tilt", rest=tilt_at(t))
    cd.weightless()
    cd.step()
    for name in PUSHED:
        c = cd.CANDLES[name]
        part = PUSH_PARTS[0] if t < 3.0 else (PUSH_PARTS[1] if t < 5.0 else 0.0)
        if c.host is None or cd.depth_of(c) > 0.025:
            part = 0.0
        force = part * cd.resistance(cd.depth_of(c), cd.HOSTS[c.host]) if c.host else 0.0
        STATE["pushing"][name] = force
        mk.push(name, force=(0.0, 0.0, -force))


def readings() -> dict:
    out = {"tilt_asked_deg": round(math.degrees(tilt_at(mk.now())), 2),
           "tilt_deg": {cake: round(math.degrees(mk.joint(f"{cake}_tilt")), 2) for cake in TILTED},
           "pushing_N": {k: round(v, 3) for k, v in STATE.get("pushing", {}).items()}}
    for name, c in cd.CANDLES.items():
        out[name] = cd.reading(c)
    return out


def report() -> dict:
    return {name: c.events for name, c in cd.CANDLES.items()}


def score(objects) -> float:
    return 0.0


def success(objects) -> bool:
    return False
