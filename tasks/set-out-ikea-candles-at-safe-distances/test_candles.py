"""Scorer tests for candles.py, run without the simulator:  python test_candles.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import candles as c  # noqa: E402

H = {n: h for n, _, _, h, _ in c.CANDLES}
START = {n: xy for n, _, _, _, xy in c.CANDLES}


def _state(name, xy, on_tray=True, tilt=0.0):
    r, h = c.RAD[name], H[name]
    z0 = c.TOP + (c.TRAY_H if on_tray else 0.0)
    pos = np.array([xy[0], xy[1], z0 + h / 2])
    t = math.radians(tilt) / 2
    q = np.array([math.sin(t), 0.0, 0.0, math.cos(t)])
    half = np.array([r, r, h / 2])
    return ObjectState(pos, q, np.zeros(3), np.zeros(3), pos - half, pos + half)


def frame(where=None, tilt=None):
    """where: {candle: xy on the tray}; candles not named stay at their start on the table."""
    where, tilt = where or {}, tilt or {}
    return {n: (_state(n, where[n], True, tilt.get(n, 0.0)) if n in where else _state(n, START[n], False))
            for n in c.KIND}


def run(frames):
    out = None
    for f in frames:
        out = c.score(f)
    return out


T = c.TARGETS


def test_nothing_done_scores_zero():
    assert run([frame()]) == 0.0


def test_the_demo_layout_meets_every_rule_with_room_to_spare():
    f = frame(T)
    assert c.broken(list(T), f) == []
    for n, xy in T.items():
        assert c.flammable_gap(n, f[n]) >= c.FLAMMABLE_M + 0.02
        assert not c.under_shelf(n, f[n])
    for a in T:
        for b in T:
            if a < b:
                assert c.gap(a, f[a], b, f[b]) >= c.needed_gap(a, b) + 0.02


def test_full_layout_scores_one():
    assert run([frame(), frame(T)]) == 1.0


def test_each_safely_placed_candle_scores_a_third():
    assert abs(run([frame({"block": T["block"]})]) - 1 / 3) < 1e-9
    assert abs(run([frame({k: T[k] for k in ("block", "tealight_1")})]) - 2 / 3) < 1e-9


def test_a_tealight_needs_seven_cm_from_the_block_candle():
    w = dict(T, tealight_1=(T["block"][0] + 0.10, T["block"][1]))  # 5.25 cm gap: enough for two tealights, not here
    assert run([frame(w)]) == 0.0


def test_a_mixed_pair_needs_the_larger_distance():
    w = {"block": (0.20, 0.06), "tealight_2": (0.20, -0.015)}  # gap 2.75 cm: < 7 cm
    assert run([frame(w)]) == 0.0


def test_tealights_need_three_cm():
    w = {"tealight_1": (0.22, -0.10), "tealight_2": (0.22, -0.05)}  # gap 0.5 cm
    assert run([frame(w)]) == 0.0
    w = {"tealight_1": (0.22, -0.11), "tealight_2": (0.22, -0.035)}  # gap 3 cm
    assert abs(run([frame(w)]) - 2 / 3) < 1e-9


def test_near_the_dried_flowers_scores_zero():
    w = dict(T, block=(0.33, 0.19))  # on the board's back left, within 30 cm of the flowers
    assert c.flammable_gap("block", frame(w)["block"]) < 0.30
    assert run([frame(w)]) == 0.0


def test_under_the_shelf_scores_zero():
    w = dict(T, tealight_1=(0.33, -0.12))
    assert run([frame(w)]) == 0.0


def test_a_candle_on_its_side_does_not_count():
    assert run([frame({"block": T["block"]}, tilt={"block": 80.0})]) == 0.0


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
