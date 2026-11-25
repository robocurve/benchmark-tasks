"""Offline tests of the scorer (no simulator): python3 test_offline.py

Checks the start layouts, the geometry of the buckle, and the scorer's verdict on recorded belt states (poses saved
from simulation runs, in test_fixtures.json): the start, a pulled but unfastened belt, the prong standing through
hole 3, and the belt fastened at hole 3 before, under and after the pull test.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
os.environ.setdefault("YAM_ARENA_OFFLINE", "1")

import beltkit as B  # noqa: E402
import geom as G  # noqa: E402
import measure as M  # noqa: E402
import task as T  # noqa: E402


def state(bodies):
    return B.belt_state_from({k: (np.array(v[1]), np.array(v[0])) for k, v in bodies.items()})


def test_geometry():
    assert G.N_HOLES == 5 and G.HOLE_LINK[3] == G.HOLE_LINK[1] - 2
    assert abs(G.HOLE_S[3] - 0.7375) < 1e-9, G.HOLE_S[3]              # 29 in belt (fold to middle hole)
    assert G.HOLE_S[1] > G.HOLE_S[5]                                    # hole 1 is nearest the tip
    assert G.PRONG_L > G.FRAME_L                                        # the prong reaches the front bar
    slack3 = G.HOLE_S[3] - G.PERIM
    assert 0.055 < slack3 < 0.062, slack3                               # about 58 mm of slack at hole 3
    print(f"geometry ok: hole 3 at {G.HOLE_S[3] * 1000:.1f} mm from the fold, slack {slack3 * 1000:.0f} mm")


def test_start_layouts():
    for k in range(len(G.START_LAYOUTS)):
        poses, _ = B.layout("threaded", k)
        st = B.belt_state_from(poses)
        info = T.evaluate(st)
        assert info["state"] == "none" and info["hole"] == 0 and info["loops"] == 5, (k, info)
        assert M.threaded(st) and info["window"] is not None, (k, info)
        print(f"start layout {k} ok: threaded, no hole, 5 loops")


def run_scorer(states, hands_off=True):
    """Drive the scorer's step logic through a list of (state, steps) with a fake environment."""
    T._pull = lambda env, on: None
    T.hands_off_distance = (lambda env, st: 0.2) if hands_off else (lambda env, st: 0.005)
    T._reset_state()
    for st, n in states:
        T._belt = lambda env, s=st: s
        for _ in range(n):
            T._on_step(None)
    return T._S["score"], T._S["phase"]


def test_fixtures():
    fx = json.loads((HERE / "test_fixtures.json").read_text())
    s = {k: state(v) for k, v in fx.items()}
    e = {k: T.evaluate(v) for k, v in s.items()}
    assert e["start_threaded"]["state"] == "none" and e["start_threaded"]["hole"] == 0
    assert e["pulled_not_engaged"]["state"] == "none" and e["pulled_not_engaged"]["hole"] == 0
    assert e["prong_standing_through_hole3"]["hole"] == 3 and not e["prong_standing_through_hole3"]["on_bar"]
    assert e["prong_standing_through_hole3"]["state"] != "fastened"
    for k in ("fastened_hole3_rest", "fastened_hole3_under_pull", "fastened_hole3_after_pull", "fastened_hole3_layout2"):
        assert e[k]["state"] == "fastened", (k, e[k])
        assert e[k]["tip_bar_mm"] <= T.PRONG_ON_BAR_M * 1000 and e[k]["right_way"], (k, e[k])
    travel = np.linalg.norm(M.hole_in_frame(s["fastened_hole3_after_pull"], 3) - M.hole_in_frame(s["fastened_hole3_rest"], 3))
    assert travel < T.STRAP_MOVE_M, travel
    # the scorer: hands off 1 s, pull 1 s, settle 0.5 s -> 1.0
    sc, ph = run_scorer([(s["fastened_hole3_rest"], 25), (s["fastened_hole3_under_pull"], 20),
                         (s["fastened_hole3_after_pull"], 15)])
    assert (sc, ph) == (1.0, "done"), (sc, ph)
    # the same fastened state with a gripper still on the belt: 0
    sc, _ = run_scorer([(s["fastened_hole3_rest"], 60)], hands_off=False)
    assert sc == 0.0, sc
    # pulled and let go (nothing engaged), the start: 0; the prong standing through hole 3 (not laid down): 0.25
    for k in ("pulled_not_engaged", "start_threaded"):
        sc, _ = run_scorer([(s[k], 60)])
        assert sc == 0.0, (k, sc)
    sc, _ = run_scorer([(s["prong_standing_through_hole3"], 60)])
    assert sc == 0.25, sc
    # the belt out of its loops (the whole belt shifted 4 cm along the form): loop_lost, 0 even when fastened
    st = s["fastened_hole3_rest"]
    st2 = B.belt_state_from({k: (R_, p_ + np.array([0.04, 0.0, 0.0])) for k, (p_, R_) in st.poses.items()})
    assert T.evaluate(st2)["state"] == "loop_lost"
    sc, _ = run_scorer([(st2, 60)])
    assert sc == 0.0, sc
    # a wrong hole earns nothing: the recorded hole-3 fastening scored against a requested hole 1, 2, 4 or 5 gives 0
    hole0 = T.HOLE
    try:
        for req, want in ((2, 0.0), (4, 0.0), (1, 0.0), (5, 0.0)):
            T.HOLE = req
            sc, _ = run_scorer([(s["fastened_hole3_rest"], 25), (s["fastened_hole3_under_pull"], 20),
                                (s["fastened_hole3_after_pull"], 15)])
            assert sc == want, (req, sc)
    finally:
        T.HOLE = hole0
    print("fixtures ok: start 0, pulled-and-let-go 0, standing prong 0.25, hand on belt 0, wrong hole 0, "
          "fastened + pull held 1.0, "
          f"strap travel under the pull {travel * 1000:.1f} mm")


if __name__ == "__main__":
    test_geometry()
    test_start_layouts()
    test_fixtures()
    print("all offline tests passed")
