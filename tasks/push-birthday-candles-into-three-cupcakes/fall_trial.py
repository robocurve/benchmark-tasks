"""Trial scene: how long a shallow candle takes to fall, the task's 10 g stand-in against a real 1 g candle.

Six cupcakes, each held as in task.py (the cake's hold 0.0002 N m per radian at 8 mm and 0.002 at 15 mm, a power law
through both; tearing out 25 degrees from where the candle rests; 3 N through the icing, 1 N in the cake). In each
stands one candle, upright, 8, 9 or 15 mm deep. Each cupcake (0.3 kg, a firm stand) turns on a hinge to the world
about a level axis along its base's far edge, held by a stiff spring whose rest steps from 0 to 1 degree at 0.5 s:
the cupcake leans 1 degree away from the arms and the candle with it (its hole along it), as a push that left it
1 degree off would. Near row (blue): the task's candle, 10 g, its weight over 1 g carried every control step; far row
(red): a 1 g candle, nothing carried. Nothing else touches them; the arms stay in their ready pose. (Run w7c-s1 built
the candles leaning 1 degree in cupcakes fixed to the bench: five of six were torn out at the first control step.)
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import candles as cd  # noqa: E402
import mechkit as mk  # noqa: E402
from mechkit import body, cylinder, hinge  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = "Check of a shallow candle's fall: nothing to do."
TOP = TABLE_TOP["station"]

CAKE_H, CAKE_R_TOP = 0.035, 0.030
CAKE_LAYERS = ((0.000, 0.007, 0.026), (0.007, 0.014, 0.027), (0.014, 0.021, 0.028), (0.021, 0.029, 0.029))  # z0, z1, r
CHOC, ICING = (0.24, 0.13, 0.07), (0.16, 0.08, 0.05)
BLUE, RED = (0.12, 0.30, 0.85), (0.85, 0.10, 0.12)
LEAN_DEG = 1.0
LEAN = math.radians(LEAN_DEG)
DEPTHS = (0.008, 0.009, 0.015)
ROWS = {"standin": (0.28, cd.STANDIN_MASS, BLUE), "real": (0.43, cd.CANDLE_MASS, RED)}  # x, candle mass kg, colour


def cupcake_shapes() -> list:
    shapes = [cylinder(r, z1 - z0, pos=(0.0, 0.0, (z0 + z1) / 2), color=CHOC) for z0, z1, r in CAKE_LAYERS]
    shapes.append(cylinder(CAKE_R_TOP, CAKE_H - 0.029, pos=(0.0, 0.0, (CAKE_H + 0.029) / 2), color=ICING))
    return shapes


STAND_MASS, STAND_K, STAND_DAMP = 0.3, 20.0, 0.2  # a tilting cupcake: kg, N m/rad, N m s/rad (as in hold_trial.py)
TILT_AT = 0.5  # s
CANDLE_OF: dict[str, tuple[str, float]] = {}  # candle -> (row, depth m)
CAKES: list[str] = []
for _row, (_x, _mass, _color) in ROWS.items():
    for _y, _d in zip((0.12, 0.0, -0.12), DEPTHS):
        _cake, _name = f"cake_{_row}_{round(_d * 1000)}", f"candle_{_row}_{round(_d * 1000)}"
        body(_cake, cupcake_shapes(), pos=(_x, _y, TOP + 0.0005), mass=STAND_MASS, color=CHOC)
        hinge(f"{_cake}_tilt", _cake, None, anchor=(_x + CAKE_LAYERS[0][2], _y, TOP + 0.0005), axis=(0.0, 1.0, 0.0), limits=(-0.2, 1.3),
              spring=STAND_K, damping=STAND_DAMP)
        cd.host(_cake, CAKE_H, CAKE_R_TOP - 0.0025, resist_top=3.0, top_depth=0.002, resist_body=1.0, k_at=0.0002, tear_deg=25.0, k_15=0.002)
        cd.declare(_name, (_x, _y, TOP + 0.0005 + CAKE_H - _d), (0.0, 0.0, 0.0, 1.0), (_cake,), _color, built_in=_cake, built_depth=_d, mass=_mass)
        CANDLE_OF[_name] = (_row, _d)
        CAKES.append(_cake)

OBJECTS = list(mk.MECH.bodies.values())


@mk.on_reset
def start(seed) -> None:
    if mk.MECH.environment is not None:
        cd.reset()
        for cake in CAKES:
            mk.set_spring(f"{cake}_tilt", rest=0.0)


@mk.on_step
def step() -> None:
    if mk.MECH.environment is None:
        return
    for cake in CAKES:
        mk.set_spring(f"{cake}_tilt", rest=LEAN if mk.now() >= TILT_AT else 0.0)
    cd.weightless()
    cd.step()


def readings() -> dict:
    out = {name: cd.reading(c) for name, c in cd.CANDLES.items()}
    out["tilt_deg"] = {cake: round(math.degrees(mk.joint(f"{cake}_tilt")), 3) for cake in CAKES}
    return out


def report() -> dict:
    return {name: c.events for name, c in cd.CANDLES.items()}


def score(objects) -> float:
    return 0.0


def success(objects) -> bool:
    return False
