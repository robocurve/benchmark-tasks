"""Checks of the scoring rule on hand-made states (no simulator needed, only yam_arena importable).

    python test_scoring.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402
from eggkit import BodyState  # noqa: E402

UP = (0.0, 0.0, 0.0, 1.0)
ON_SIDE = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))      # an egg lying with its long axis along x


def state(pos, quat=UP, vel=(0.0, 0.0, 0.0), flaps=None) -> BodyState:
    p = np.array(pos, float)
    return BodyState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                     low=p - 0.03, high=p + 0.03, flaps=flaps or {})


def start() -> dict:
    objects = {"carton": state((*T.CARTON_XY, T.TOP), quat=(0.0, 0.0, math.sin(T.CARTON_YAW / 2),
                                                             math.cos(T.CARTON_YAW / 2)), flaps={"lid": 0.0})}
    for i, name in enumerate(T.EGGS):
        objects[name] = state(T.egg_start(i))
    return objects


def lying_in_basket(dx=0.0, dy=0.0, lift=0.0):
    return state((T.BASKET_XY[0] + dx, T.BASKET_XY[1] + dy, T.TOP + T.BASKET_FLOOR + T.EGG_R + lift), quat=ON_SIDE)


def standing_in_basket(dx=0.0, dy=0.0, lift=0.0):
    return state((T.BASKET_XY[0] + dx, T.BASKET_XY[1] + dy, T.TOP + T.BASKET_FLOOR + T.EGG_LEN / 2 + lift))


def check(label: str, got, want) -> None:
    ok = abs(got - want) < 1e-9 if isinstance(want, float) else got == want
    print(f"{'ok  ' if ok else 'FAIL'} {label}: {got!r}" + ("" if ok else f" (wanted {want!r})"))
    if not ok:
        check.failed += 1


check.failed = 0


def main() -> int:
    T._clear()
    o = start()
    check("start: every egg is in the carton", [T.where(o, n) for n in T.EGGS], ["carton"] * T.N)
    check("start: score", T.score(o), 0.0)

    o["egg_1"] = lying_in_basket(0.04, 0.05)
    o["egg_2"] = standing_in_basket(-0.04, -0.05)
    check("one egg lying and one standing on the cloth", T.score(o), 2 / T.N)

    o["egg_3"] = lying_in_basket(0.0, 0.0, lift=0.03)
    check("an egg lying on top of other eggs, 3 cm above the cloth, does not count", T.score(o), 2 / T.N)

    o["egg_3"] = lying_in_basket(T.BASKET_IN_X / 2 + 0.03, 0.0)
    check("an egg on the table beside the basket is loose", T.where(o, "egg_3"), "loose")

    o["egg_3"] = lying_in_basket(0.0, 0.0)
    o["egg_3"] = state(o["egg_3"].pos, quat=ON_SIDE, vel=(0.2, 0.0, 0.0))
    check("an egg still rolling does not count yet", T.score(o), 2 / T.N)

    T._STATE["held"] = {"egg_1"}
    check("an egg a gripper still holds does not count", T.score(o), 1 / T.N)
    T._STATE["held"] = set()

    T._STATE["broken"] = {"egg_2"}
    check("a broken egg does not count", T.score(o), 1 / T.N)
    T._clear()

    check("a 2 cm fall does not break an egg", T.impact_speed(0.02, 0.0, 0.0) > T.BREAK_SPEED, False)
    check("a 4 cm fall breaks an egg", T.impact_speed(0.04, 0.0, 0.0) > T.BREAK_SPEED, True)
    check("a fall seen part-way (2 cm fallen, 2 cm to go) still breaks it",
          T.impact_speed(0.02, math.sqrt(2 * T.G * 0.02), 0.0) > T.BREAK_SPEED, True)
    check("carrying an egg down at 0.15 m/s does not break it",
          T.impact_speed(0.10, 0.15, 0.10 - 0.15 / 20) > T.BREAK_SPEED, False)

    # The whole carton set down inside the basket: the eggs are over the cloth but still in their cups.
    o = start()
    shift = np.array([T.BASKET_XY[0] - T.CARTON_XY[0], T.BASKET_XY[1] - T.CARTON_XY[1], T.BASKET_FLOOR])
    for name in ("carton", *T.EGGS):
        o[name] = state(o[name].pos + shift, quat=o[name].quat_xyzw, flaps={"lid": 180.0} if name == "carton" else None)
    check("the carton put in the basket with its eggs: score", T.score(o), 0.0)
    check("  and those eggs read as still in the carton", [T.where(o, n) for n in T.EGGS], ["carton"] * T.N)

    o = start()
    for i, name in enumerate(T.EGGS):
        o[name] = lying_in_basket((i % 2 - 0.5) * T.SLOT_DX, (i // 2 - 1) * T.SLOT_DY)
    check("all six lying on the cloth", T.score(o), 1.0)
    check("  success", T.success(o), True)
    o["egg_1"] = standing_in_basket(0.0, T.BASKET_IN_Y / 2 + T.BASKET_WALL / 2, lift=T.BASKET_H - T.BASKET_FLOOR)
    check("an egg balanced on the basket's rim does not count", T.score(o), 5 / T.N)
    print("ALL OK" if not check.failed else f"{check.failed} FAILED")
    return 1 if check.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
