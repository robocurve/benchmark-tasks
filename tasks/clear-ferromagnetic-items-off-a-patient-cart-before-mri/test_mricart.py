"""Scorer tests for mricart.py, run without the simulator:  python test_mricart.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import mricart as M  # noqa: E402

AWAY = {"left_tcp": (0.18, 0.27, M.TOP + 0.20), "right_tcp": (0.18, -0.27, M.TOP + 0.20)}
TRAY = {"o2_cylinder": (0.27, 0.14, M.TOP + 0.04), "trauma_shears": (0.27, -0.06, M.TOP + 0.01),
        "phone": (0.27, -0.26, M.TOP + 0.01)}


def _obj(pos, q=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), pos - 0.01, pos + 0.01)


def frame(over=None, tcp=None):
    d = {k: _obj(v) for k, v in M.START.items()}
    for k, v in (over or {}).items():
        d[k] = v if isinstance(v, ObjectState) else _obj(v)
    for k, v in (tcp or AWAY).items():
        d[k] = _obj(v)
    return d


def run(over=None, n=60, tcp=None, pre=None):
    M._state.clear()
    M._state.update(M._fresh())
    s = 0.0
    for f in [frame()] + (pre or []):
        s = M.score(f)
    for _ in range(n):
        s = M.score(frame(over, tcp))
    return s


def test():
    assert run() == 0.0                                           # doing nothing
    assert run(TRAY) == 1.0                                       # all three cleared
    assert run({"o2_cylinder": TRAY["o2_cylinder"]}) == 0.25
    assert run({k: TRAY[k] for k in ("o2_cylinder", "phone")}) == 0.5
    assert run({"o2_cylinder": (0.27, 0.14, M.TOP + 0.04)}, n=10) == 0.0        # not still for 1 s yet
    assert run({**TRAY, "water_bottle": (0.30, 0.27, M.TOP + 0.10)}) == 0.5     # safe item taken off the cart
    assert run({**TRAY, "clipboard": (0.68, 0.14, M.SH + 0.006)}) == 0.75       # safe item shoved 10 cm, still on cart
    tilted = _obj(M.START["water_bottle"], (0.0, 0.7071, 0.0, 0.7071))           # bottle knocked over
    assert run({**TRAY, "water_bottle": tilted}) == 0.75
    assert run({**TRAY, "trauma_shears": (0.27, -0.06, M.TOP - 0.2)}) == 0.0    # steel on the floor
    assert run({"o2_cylinder": TRAY["o2_cylinder"], "phone": (0.27, -0.26, M.TOP - 0.3)}) == 0.0
    assert run({**TRAY, "emesis_basin": (0.72, 0.9, M.TOP + 0.01)}) == 0.5      # safe item off to the far side
    held = {"left_tcp": (0.27, 0.14, M.TOP + 0.1), "right_tcp": AWAY["right_tcp"]}
    assert run(TRAY, tcp=held) == 0.5                                           # cylinder still held: 2 cleared
    assert run({**TRAY, "phone": (0.50, -0.30, M.SH + 0.007)}) == 0.5           # phone left on the cart
    assert run({"o2_cylinder": (0.50, 0.5, M.TOP + 0.04)}) == 0.0               # 8 cm past the side: not 10 cm
    assert run({"o2_cylinder": (0.50, 0.55, M.TOP + 0.04)}) == 0.25             # 13 cm past the side
    # falls then recovered: still 0
    drop = [frame({"phone": (0.27, -0.26, M.TOP - 0.3)})]
    assert run(TRAY, pre=drop) == 0.0
    print("all mricart scorer tests passed")


if __name__ == "__main__":
    test()
