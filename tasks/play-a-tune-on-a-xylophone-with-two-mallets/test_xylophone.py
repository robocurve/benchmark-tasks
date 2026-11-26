"""Scorer tests for xylophone.py, run without the simulator:  python test_xylophone.py"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import xylophone as X  # noqa: E402

AWAY = {"left_tcp": (0.18, 0.27, X.TOP + 0.20), "right_tcp": (0.18, -0.27, X.TOP + 0.20)}


def _o(p, h=0.01):
    p = np.asarray(p, float)
    return ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)


def head_at_bar(j):
    return (X.FRAME_C[0], X.BAR_Y[j], X.BAR_TOP + X.HEAD_R)


def rest(side):
    return (X.HEAD_REST_X, X.MALLETS[side], X.REST_Z)


def frame(heads, tcps=AWAY):
    d = {t: _o(p) for t, p in tcps.items()}
    for s in X.MALLETS:
        h = np.asarray(heads[s], float)
        d[f"head_{s}"] = _o(h)
        d[f"handle_{s}"] = _o(h + np.array([X.HANDLE_DX, 0, 0]))
    return d


def run(events, tail=30, tcps=AWAY):
    """events: list of (hand, bar, fast). Each is an approach from above, a strike and a lift; others lie at rest."""
    X._state.clear()
    X._state.update(X._fresh())
    pos = {s: rest(s) for s in X.MALLETS}
    sc = 0.0
    for s, j, fast in events:
        hb = np.asarray(head_at_bar(j))
        top = hb + np.array([0, 0, 0.06])
        pos[s] = tuple(top)
        sc = X.score(frame(pos, tcps))
        pos[s] = tuple(hb + np.array([0, 0, 0.012 if fast else 0.001]))
        pos[s] = tuple(top - np.array([0, 0, 0.06 if fast else 0.0]) + np.array([0, 0, 0.0 if fast else 0.0]))
        sc = X.score(frame(pos, tcps))
        pos[s] = tuple(top)
        sc = X.score(frame(pos, tcps))
    for s in X.MALLETS:
        pos[s] = rest(s)
    for _ in range(tail):
        sc = X.score(frame(pos, tcps))
    return sc, list(X._state["played"]), X._state["wrong"]


GOOD = [("left", 0, 1), ("left", 0, 1), ("right", 4, 1), ("right", 4, 1), ("right", 5, 1), ("right", 5, 1), ("right", 4, 1)]


def main():
    sc, pl, w = run(GOOD)
    assert sc == 1.0 and pl == X.TUNE and not w, (sc, pl, w)
    sc, *_ = run(GOOD[:4])
    assert abs(sc - 0.6 * 4 / 7) < 1e-9, sc
    sc, *_ = run([])
    assert sc == 0.0
    sc, pl, w = run(GOOD[:3] + [("right", 5, 1)] + GOOD[4:])
    assert w and abs(sc - 0.6 * 3 / 7) < 1e-9, (sc, pl)
    sc, pl, w = run(GOOD + [("right", 4, 1)])  # extra note after the tune
    assert w and abs(sc - 0.6) < 1e-9, sc
    sc, *_ = run([("right", b, 1) for b in X.TUNE])  # one hand only
    assert abs(sc - 0.6) < 1e-9, sc
    sc, *_ = run(GOOD, tcps={**AWAY, "left_tcp": (X.HEAD_REST_X + X.HANDLE_DX, X.MALLETS["left"], X.REST_Z + 0.02)})  # holding
    assert abs(sc - 0.6) < 1e-9, sc
    # head resting slowly on the right bar counts nothing
    X._state.clear(); X._state.update(X._fresh())
    pos = {s: rest(s) for s in X.MALLETS}
    for dz in (0.03, 0.028, 0.026, 0.024, 0.022, 0.02, 0.018, 0.016, 0.014, 0.012, 0.01, 0.008, 0.006, 0.004, 0.002, 0.0, 0.0):
        pos["left"] = tuple(np.asarray(head_at_bar(0)) + np.array([0, 0, dz]))
        sc = X.score(frame(pos))
    assert sc == 0.0 and X._state["played"] == []
    # fast arrival at 5 mm above the bar then lifting counts; arriving and staying down does not
    for gap, lift, want in ((0.005, True, 1), (0.005, False, 0), (0.008, True, 0)):
        X._state.clear(); X._state.update(X._fresh())
        pos = {s: rest(s) for s in X.MALLETS}
        b = np.asarray(head_at_bar(0))
        for dz in [0.06, 0.06, gap] + ([0.06] if lift else [gap, gap]):
            pos["left"] = tuple(b + np.array([0, 0, dz]))
            X.score(frame(pos))
        assert len(X._state["played"]) == want, (gap, lift, X._state["played"])
    # a mallet dropped off the table scores 0
    X._state.clear(); X._state.update(X._fresh())
    sc = 1
    for _ in range(3):
        sc = X.score(frame({"left": (0.3, 0.3, X.TOP - 0.3), "right": rest("right")}))
    assert sc == 0.0
    # mallet left on the frame at the end: not full
    X._state.clear(); X._state.update(X._fresh())
    # mallets left lying on the frame board
    pos = {s: rest(s) for s in X.MALLETS}
    print("all tests passed")


if __name__ == "__main__":
    main()
