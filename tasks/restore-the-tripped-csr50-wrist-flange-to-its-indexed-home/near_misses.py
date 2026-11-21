"""Near misses and shortcuts for the CSR50 reset.

check.py scores the static CASES below against a task module with no trial record: every one must score 0, because
score() only credits what the simulator's step hook has recorded (fail closed). Run this file directly
(``python near_misses.py``) to replay each scenario through the same record the simulator keeps: the sensor's
mechanism (lands, lead-in, pressure back, ready) and the scorer's whole-trial and hold rules, at 20 control steps a
second, with stand-in hand points. Each scenario prints its score and the score it must get.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _load():
    spec = importlib.util.spec_from_file_location("csr50_task_copy", HERE / "task.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


_T = _load()
CSR = _T.CSR
D = math.radians

CASES = [
    ("start: tripped, nothing recorded", {}, 0.0, False),
    ("flange at home with no trial record (fail closed)", {"csr50": CSR.state(joints={"axial": 0.0, "turn": 0.0})},
     0.0, False),
    ("flange turned to the mark, no trial record", {"csr50": CSR.state(joints={"axial": 0.0125, "turn": 0.0})},
     0.0, False),
]

FAR = np.array([[0.10, 0.30, 1.05]])  # a hand parked well away from the sensor and the stopped arm


def _near_flange(m, objects, gap):
    """A hand point ``gap`` metres outside the flange's rim, level with its middle."""
    c, a, hl, r = m.csr_cylinders(objects)[1]
    side = np.cross(a, [0.0, 0.0, 1.0])
    side /= np.linalg.norm(side)
    return (c + side * (r + gap))[None, :]


def _objects(m, axial, turn_deg, tilt_x=0.0, tilt_y=0.0):
    return {"ur3e": m.UR3E.state(),
            "csr50": m.CSR.state(joints={"axial": axial, "tilt_x": D(tilt_x), "tilt_y": D(tilt_y),
                                          "turn": D(turn_deg)})}


def replay(m, steps):
    """steps: list of (seconds, axial m, turn deg, tilt_x deg, hand) held for that long; hand is 'far', 'flange'
    (5 mm off the rim, i.e. holding) or None (unknown). Returns the final score."""
    m.REC.clear()
    m.REC.update(m._fresh())
    t, objects = 0.0, None
    for secs, axial, turn, tx, hand in steps:
        for _ in range(max(1, round(secs * 20))):
            t += 0.05
            if m.REC["mode"] == "ready":  # the pressurised drive holds it exactly home
                axial, turn, tx = 0.0, 0.0, 0.0
            objects = _objects(m, axial, turn, tx)
            m.mechanism(objects, 0.05)
            hands = {"far": FAR, "flange": None, None: None}[hand]
            if hand == "flange":
                hands = np.vstack([FAR, _near_flange(m, objects, 0.005)])
            m.observe(objects, hands, t)
    return m.score(objects)


OUT = 0.0125
SCENARIOS = [
    ("untouched tripped flange", [(3.0, OUT, 75.0, 0.0, "far")], 0.0),
    ("naive straight push: stops on the lands 75 degrees off the mark, let go",
     [(2.0, 0.0025, 75.0, 0.0, "flange"), (2.0, 0.0025, 75.0, 0.0, "far")], 0.0),
    ("turned the wrong way to its stop, then pushed onto the lands",
     [(2.0, OUT, 175.0, 0.0, "flange"), (2.0, 0.0025, 175.0, 0.0, "flange"), (2.0, 0.0025, 175.0, 0.0, "far")], 0.0),
    ("turned to within 5.1 degrees only, let go", [(2.0, OUT, 5.1, 0.0, "flange"), (2.0, OUT, 5.1, 0.0, "far")], 0.0),
    ("turned to exactly 5 degrees (limit), let go", [(2.0, OUT, 5.0, 0.0, "flange"), (2.0, OUT, 5.0, 0.0, "far")], 0.25),
    ("turned to the mark but never pushed home", [(2.0, OUT, 1.0, 0.0, "flange"), (2.0, OUT, 1.0, 0.0, "far")], 0.25),
    ("turned 3 degrees off, pushed onto the seat (not captured), let go",
     [(2.0, 0.0008, 3.0, 0.0, "flange"), (2.0, 0.0008, 3.0, 0.0, "far")], 0.25),
    ("indexed and flush for 0.6 s, then pulled back out before the air re-seated it",
     [(0.6, 0.0, 0.5, 0.0, "flange"), (2.0, OUT, 0.5, 0.0, "flange"), (2.0, OUT, 0.5, 0.0, "far")], 0.5),
    ("on the mark, held 0.9 mm out all round (flush, not seated), never ready",
     [(3.0, 0.0009, 0.5, 0.0, "flange"), (2.0, 0.0009, 0.5, 0.0, "far")], 0.5),
    ("on the mark, home but tilted 1.7 degrees (0.93 mm gap at the rim), never ready",
     [(3.0, 0.0, 0.5, 1.7, "flange"), (2.0, 0.0, 0.5, 1.7, "far")], 0.5),
    ("on the mark, held 1.1 mm out (not flush)", [(3.0, 0.0011, 0.5, 0.0, "flange"), (2.0, 0.0011, 0.5, 0.0, "far")],
     0.25),
    ("on the mark, home but tilted 1.2 degrees (0.66 mm at the rim): seated, ready, released",
     [(1.5, 0.0, 0.5, 1.2, "flange"), (1.2, 0.0, 0.0, 0.0, "far")], 1.0),
    ("ready, but a hand stays within 20 mm of the flange to the end",
     [(1.5, 0.0, 0.5, 0.0, "flange"), (3.0, 0.0, 0.0, 0.0, "flange")], 0.75),
    ("ready, hands clear for only 0.9 s at the end",
     [(1.5, 0.0, 0.5, 0.0, "flange"), (0.9, 0.0, 0.0, 0.0, "far")], 0.75),
    ("ready, released, hands clear and still for 1 s", [(1.5, 0.0, 0.5, 0.0, "flange"), (1.2, 0.0, 0.0, 0.0, "far")],
     1.0),
    ("full reset, but the hand state was unknown at one step",
     [(1.5, 0.0, 0.5, 0.0, "flange"), (0.1, 0.0, 0.0, 0.0, None), (1.5, 0.0, 0.0, 0.0, "far")], 0.0),
]


def run():
    m = _load()
    bad = 0
    for label, steps, want in SCENARIOS:
        got = replay(m, steps)
        ok = abs(got - want) < 1e-9
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {got:.2f} (want {want:.2f})  {label}")
    # A stale record: the scorer is handed a state the record was not written from.
    replay(m, [(1.5, 0.0, 0.5, 0.0, "flange"), (1.2, 0.0, 0.0, 0.0, "far")])
    stale = m.score(_objects(m, 0.0004, 0.0))
    print(f"{'ok  ' if stale == 0.0 else 'FAIL'} {stale:.2f} (want 0.00)  record from another state (stale stamp)")
    bad += stale != 0.0
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
