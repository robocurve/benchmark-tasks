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
from bookkit import PartState  # noqa: E402

FLAT = (0.0, 0.0, 0.0, 1.0)                                    # lying face up, spine toward the arms
UPRIGHT = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))   # a quarter turn about x: head up, spine out


def turned(quat, yaw_deg: float) -> tuple:
    """``quat`` followed by a turn of ``yaw_deg`` about the vertical."""
    h = math.radians(yaw_deg) / 2
    x1, y1, z1, w1 = 0.0, 0.0, math.sin(h), math.cos(h)
    x2, y2, z2, w2 = quat
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def state(pos, quat=FLAT, vel=(0.0, 0.0, 0.0)) -> PartState:
    p = np.array(pos, float)
    return PartState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                     low=p - 0.09, high=p + 0.09)


def start() -> dict:
    return {name: state(T.book_start(i)) for i, name in enumerate(T.BOOKS)}


def standing(place: int, quat=UPRIGHT, lift: float = 0.0, vel=(0.0, 0.0, 0.0)) -> PartState:
    """A book standing on the shelf at the ``place``-th position from the left end."""
    return state((T.SHELF_X + T.BOOK_D / 2, T.slot_y(place), T.TOP + T.SHELF_T + T.BOOK_H / 2 + lift), quat, vel)


def check(label: str, got, want) -> None:
    ok = abs(got - want) < 1e-9 if isinstance(want, float) else got == want
    print(f"{'ok  ' if ok else 'FAIL'} {label}: {got!r}" + ("" if ok else f" (wanted {want!r})"))
    if not ok:
        check.failed += 1


check.failed = 0


def main() -> int:
    T._clear()
    by_code = {T.CODE[n]: n for n in T.BOOKS}
    first, second, third, last = (by_code[c] for c in sorted(by_code))

    o = start()
    check("start: nothing is shelved", T.row(o), [])
    check("start: score", T.score(o), 0.0)
    check("a book in the pile lies flat", round(T.lean_deg(o[first])), 90)

    o[last] = standing(3)
    check("one book standing on the shelf is in place", T.score(o), 0.25)
    check("  it stands head up with its spine out", (round(T.lean_deg(o[last])), round(T.facing_deg(o[last]))), (0, 0))

    o[first] = standing(0)
    check("two books in order", T.score(o), 0.5)
    o[first], o[last] = standing(3), standing(0)
    check("the same two the wrong way round: neither is in place", T.score(o), 0.0)

    o = start()
    o[first], o[second], o[third] = standing(0), standing(2), standing(1)
    check("three shelved with the middle two swapped: only the first is in place", T.score(o), 0.25)

    o = start()
    for k, n in enumerate((first, second, third, last)):
        o[n] = standing(k)
    check("all four in alphabetical order from the left", T.score(o), 1.0)
    check("  success", T.success(o), True)

    o[second] = standing(1, quat=turned(UPRIGHT, 180.0))
    check("a book with its spine to the back is not shelved", T.shelved(o, second), False)
    o[second] = standing(1, quat=(-UPRIGHT[0], 0.0, 0.0, UPRIGHT[3]))
    check("a book standing on its head is not shelved", T.shelved(o, second), False)
    o[second] = state((T.SHELF_X + T.BOOK_D / 2, T.slot_y(1), T.TOP + T.SHELF_T + T.BOOK_T / 2))
    check("a book lying flat on the shelf is not shelved", T.shelved(o, second), False)
    o[second] = standing(1, lift=0.02)
    check("a book held 2 cm above the board is not shelved", T.shelved(o, second), False)
    o[second] = standing(1, vel=(0.0, 0.2, 0.0))
    check("a book still moving is not shelved yet", T.shelved(o, second), False)
    o[second] = standing(1)
    T._STATE["held"] = {second}
    check("a book a gripper still holds is not shelved", T.shelved(o, second), False)
    check("  and with it missing the other three still count", T.score(o), 0.75)
    T._clear()

    o[second] = state((T.SHELF_X - 0.10, T.slot_y(1), T.TOP + T.BOOK_H / 2), UPRIGHT)
    check("a book standing on the table in front of the shelf is not shelved", T.shelved(o, second), False)
    print("ALL OK" if not check.failed else f"{check.failed} FAILED")
    return 1 if check.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
