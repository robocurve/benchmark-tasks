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
from strawkit import PartState  # noqa: E402

UP = (0.0, 0.0, 0.0, 1.0)


def state(pos, quat=UP, vel=(0.0, 0.0, 0.0)) -> PartState:
    p = np.array(pos, float)
    return PartState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                     low=p - 0.05, high=p + 0.05)


def start() -> dict:
    objects = {"lifter": state(T.dispenser_point(0.0, 0.0))}
    for i, name in enumerate(T.GLASSES):
        objects[name] = state((*T.GLASS_XY[i], T.TOP))
    for i, name in enumerate(T.STRAWS):
        objects[name] = state(T.straw_start(i))
    return objects


def leaning_in(glass: int, slant_deg: float = 50.0, sink: float = 0.0, vel=(0.0, 0.0, 0.0)) -> PartState:
    """A straw leaning in a glass: lower end at the base's far corner, lifted by ``sink`` metres, slanting up to +y."""
    gx, gy = T.GLASS_XY[glass]
    a = math.radians(slant_deg)
    low = np.array([gx, gy - T.GLASS_R_IN + 0.006, T.TOP + T.GLASS_BASE + 0.004 + sink])
    centre = low + (T.STRAW_LEN / 2) * np.array([0.0, math.cos(a), math.sin(a)])
    return state(centre, quat=(math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)), vel=vel)   # turned about x: axis y tips up


def check(label: str, got, want) -> None:
    ok = abs(got - want) < 1e-9 if isinstance(want, float) else got == want
    print(f"{'ok  ' if ok else 'FAIL'} {label}: {got!r}" + ("" if ok else f" (wanted {want!r})"))
    if not ok:
        check.failed += 1


check.failed = 0


def main() -> int:
    T._clear()
    o = start()
    check("start: no straw is in a glass", [T.glass_of(o, s) for s in T.STRAWS], [None] * T.N_STRAWS)
    check("start: score", T.score(o), 0.0)

    o["straw_1"] = leaning_in(0)
    check("a straw leaning in glass 1 is read as in it", T.glass_of(o, "straw_1"), "glass_1")
    check("one glass served", T.score(o), 1 / T.N_GLASSES)

    o["straw_2"] = leaning_in(0, slant_deg=58.0)
    check("two straws in the same glass: that glass does not count", T.score(o), 0.0)
    o["straw_2"] = leaning_in(1)
    check("a straw each in glasses 1 and 2", T.score(o), 2 / T.N_GLASSES)

    o["straw_3"] = leaning_in(2, sink=T.GLASS_H - T.IN_DEPTH)
    check("a straw whose end is less than 3 cm below the rim is not in the glass", T.glass_of(o, "straw_3"), None)
    gx, gy = T.GLASS_XY[2]
    o["straw_3"] = state((gx, gy, T.TOP + T.GLASS_H + T.STRAW_R))
    check("a straw lying across a glass's rim does not count", T.score(o), 2 / T.N_GLASSES)

    o["straw_3"] = leaning_in(2, vel=(0.0, 0.2, 0.0))
    check("a straw still moving does not count yet", T.score(o), 2 / T.N_GLASSES)
    o["straw_3"] = leaning_in(2)
    T._STATE["held"] = {"straw_3"}
    check("a straw a gripper still holds does not count", T.score(o), 2 / T.N_GLASSES)
    T._STATE["held"] = set()
    check("three glasses served", T.score(o), 3 / T.N_GLASSES)

    tilt = math.radians(20.0)
    upright = o["glass_1"]
    o["glass_1"] = state(upright.pos, quat=(math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)))
    check("a glass leaning 20 degrees does not count", T.served(o, "glass_1"), False)
    o["glass_1"] = upright
    T._STATE["spilled"] = {"glass_1"}
    check("a glass that was tipped over earlier does not count even if stood up again", T.served(o, "glass_1"), False)
    T._clear()

    o["straw_4"], o["straw_5"] = leaning_in(3), leaning_in(4)
    check("all five glasses served", T.score(o), 1.0)
    check("  success", T.success(o), True)
    check("  the spare straw is still in the dispenser", T.glass_of(o, "straw_6"), None)
    print("ALL OK" if not check.failed else f"{check.failed} FAILED")
    return 1 if check.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
