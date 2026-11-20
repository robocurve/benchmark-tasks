"""Scorer tests for carryon.py, run without the simulator:  python test_carryon.py"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import carryon as C  # noqa: E402

AWAY_L, AWAY_R = (0.18, 0.27, C.TOP + 0.20), (0.18, -0.27, C.TOP + 0.20)
HALF = {"power_bank": (0.0775, 0.035, 0.0125), "camera_battery": (0.03, 0.0175, 0.011), "toiletry_case": (0.101, 0.0385, 0.027),
        "hardback_guide": (0.065, 0.0425, 0.012), "slide_a": (0.048, 0.14, 0.022), "slide_b": (0.048, 0.14, 0.022)}


def obj(pos, half):
    pos, half = np.asarray(pos, float), np.asarray(half, float)
    return ObjectState(pos, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), pos - half, pos + half)


# resting places: (x, y) of the item's centre on a case floor; z on the floor
CARRY_SPOT = {"power_bank": (0.40, -0.17), "camera_battery": (0.36, -0.53), "toiletry_case": (0.425, -0.30),
              "hardback_guide": (0.395, -0.43)}
CHECK_SPOT = {"slide_a": (0.40, 0.12), "slide_b": (0.40, 0.40), "power_bank": (0.40, 0.25), "camera_battery": (0.40, 0.55)}
START = {n: v[0] for n, v in C.ITEMS.items()}


def frame(place=None, held=None, tcp=(AWAY_L, AWAY_R)):
    """place: item -> (x, y) on a case floor, or (x, y, z) anywhere; others stay where they started."""
    place = place or {}
    o = {}
    for n, h in HALF.items():
        if n in place:
            p = place[n]
            z = p[2] if len(p) > 2 else C.FLOOR_TOP + h[2]
            o[n] = obj((p[0], p[1], z), h)
        else:
            o[n] = obj((*START[n], C.TOP + h[2]), h)
    o["left_tcp"], o["right_tcp"] = obj(tcp[0], (0, 0, 0)), obj(tcp[1], (0, 0, 0))
    return o


def run(frames_fn, steps):
    C._state.clear()
    C._state.update(C._fresh())
    s = 0.0
    for i in range(steps):
        s = C.score(frames_fn(i))
    return s


def hold(f):
    return lambda i: f


def test_full():
    f = frame({**CARRY_SPOT, "slide_a": CHECK_SPOT["slide_a"], "slide_b": CHECK_SPOT["slide_b"]})
    assert run(hold(f), 60) == 1.0
    assert run(hold(f), 15) == 0.0  # not yet still for 1 s


def test_nothing():
    assert run(hold(frame()), 80) == 0.0


def test_partial_and_cap():
    f = frame({"toiletry_case": CARRY_SPOT["toiletry_case"], "hardback_guide": CARRY_SPOT["hardback_guide"],
               "slide_a": CHECK_SPOT["slide_a"], "slide_b": CHECK_SPOT["slide_b"]})
    assert abs(run(hold(f), 60) - 0.4) < 1e-9  # four packed but the lithium items on the table: capped
    f = frame({"toiletry_case": CARRY_SPOT["toiletry_case"]})
    assert abs(run(hold(f), 60) - 0.2) < 1e-9
    f = frame({"power_bank": CARRY_SPOT["power_bank"], "camera_battery": CARRY_SPOT["camera_battery"],
               "toiletry_case": CARRY_SPOT["toiletry_case"]})
    assert abs(run(hold(f), 60) - 0.6) < 1e-9  # lithium safe: no cap
    f = frame({"slide_a": CHECK_SPOT["slide_a"]})
    assert run(hold(f), 60) == 0.0  # one slide is not a pair: 0 units


def test_hazard_latched():
    ok = frame({**CARRY_SPOT, "slide_a": CHECK_SPOT["slide_a"], "slide_b": CHECK_SPOT["slide_b"]})
    bad = frame({**{k: v for k, v in CARRY_SPOT.items() if k != "power_bank"}, "power_bank": CHECK_SPOT["power_bank"],
                 "slide_a": CHECK_SPOT["slide_a"], "slide_b": CHECK_SPOT["slide_b"]})
    assert run(hold(bad), 80) == 0.0
    assert run(lambda i: bad if i < 15 else ok, 100) == 0.0  # moved to the carry-on later: still latched
    # carried through above the rim: not resting inside, no violation
    over = frame({"power_bank": (0.40, 0.25, C.TOP + 0.20)})
    assert run(hold(over), 40) == 0.0 and not C._state["violated"]
    # a quick pass (under half a second) is not a violation
    assert run(lambda i: bad if i < 5 else ok, 100) == 1.0


def test_held_or_outside():
    f = frame({**CARRY_SPOT, "slide_a": CHECK_SPOT["slide_a"], "slide_b": CHECK_SPOT["slide_b"]},
              tcp=(AWAY_L, (CARRY_SPOT["power_bank"][0], CARRY_SPOT["power_bank"][1], C.TOP + 0.03)))
    assert abs(run(hold(f), 60) - 0.4) < 1e-9  # the held bank isn't counted; lithium not both packed -> capped 0.4
    f2 = frame({**CARRY_SPOT, "slide_a": (0.40, 0.12, C.TOP + 0.20), "slide_b": CHECK_SPOT["slide_b"]})
    assert abs(run(hold(f2), 60) - 0.8) < 1e-9  # slides not a pair in a case: 4 units, lithium safe
    off = frame({**CARRY_SPOT, "toiletry_case": (0.425, -0.30, C.TOP - 0.2)})
    assert abs(run(hold(off), 60) - 0.6) < 1e-9


if __name__ == "__main__":
    n = 0
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            v()
            n += 1
    print(n, "tests passed")
