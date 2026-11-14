"""The scorer on hand-built states, with no simulator: the ladder, the full result, the 2 s hold with its pull test, and the latched failures.

    python check_score.py

Exits non-zero if any case gives another value than the scoring text.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import types
from dataclasses import dataclass


sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # the harness is not needed: only its names, so that mechkit's meshes can be built
sys.modules["yam_arena"].__path__ = []
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")


@dataclass(frozen=True)
class Box:  # the harness's shape, its fields only
    name: str
    pos: tuple
    color: tuple = (0.5, 0.5, 0.5)
    mass: float | None = None
    yaw: float = 0.0
    size: tuple = (0.04, 0.04, 0.04)


@dataclass(frozen=True)
class ObjectState:
    pos: object
    quat_xyzw: object
    lin_vel: object
    ang_vel: object
    low: object
    high: object


tf.TABLE_TOP, tf.Box, tf.ObjectState = {"station": 0.0}, Box, ObjectState
sys.path.insert(0, str(pathlib.Path(__file__).parent))
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
t.m.push = lambda *a, **kw: None  # the pull test's force: here the comb's height is given

bad = 0


def check(label, got, want):
    global bad
    ok = got == want if isinstance(want, bool) else abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {label}: {got} (want {want})")


OPEN = t.TURNS_OPEN  # a screw backed out far enough for a pin
SHUT = 0.0


def state(home=True, down=(True, True, True), hands=0.08, driver_free=True, fell=False, out=(SHUT, SHUT, SHUT), comb_z=0.2, gaps=(0.0005,) * 3):
    return {"home": home, "down": list(down), "hands": hands, "driver_free": driver_free, "fell": fell, "out": list(out), "comb_z": comb_z, "gaps": list(gaps)}


# the ladder
for label, c, opened, want in (("nothing done", state(home=False, down=(False,) * 3), 0, 0.0), ("one terminal opened", state(home=False, down=(False,) * 3), 1, 0.03),
                               ("the row opened", state(home=False, down=(False,) * 3), 3, 0.09), ("comb home, no screw down", state(down=(False,) * 3), 3, 0.34),
                               ("one screw down", state(down=(True, False, False)), 3, 0.44), ("two screws down", state(down=(True, True, False)), 3, 0.54),
                               ("three down, a hand still on the comb", state(hands=0.03), 3, 0.64)):
    check(f"ladder, {label}", t._ladder(c, opened), want)

# the full result, before its hold
for label, c, want in (("all done", state(), True), ("a fingertip 4.9 cm from the comb", state(hands=0.049), False),
                       ("the screwdriver held or its tip at a head", state(driver_free=False), False), ("one screw not down", state(down=(True, True, False)), False),
                       ("the comb not home", state(home=False), False)):
    check(f"complete, {label}", t._complete(c), want)


def run(states):
    """Step the trial through hand-built states, the comb's height following them (the pull test reads it)."""
    tr = t.TRIAL
    tr.reset()
    for c in states:
        t.now = lambda env, c=c: c
        tr.step(None)
    return tr


opened_row = (OPEN, OPEN, OPEN)
tr = run([state(down=(False,) * 3, home=False, out=opened_row)] + [state()] * 39)
check("39 steps at the full result: the ladder (0.64), not yet 1.0", tr.score, 0.64)
tr.step(None)
check("40 steps at the full result, the pull test passed: 1.0", tr.score, 1.0)
check("success() then", t.success(None), True)
pulled = [state(out=opened_row)] + [state(comb_z=0.2 + (0.002 if 12 <= k <= 25 else 0.0)) for k in range(60)]
tr = run(pulled)
check("the comb lifts 2 mm in the pull test: stays at the ladder", tr.score, 0.64)
check("success() then", t.success(None), False)
tr = run([state(out=opened_row), state(fell=True)] + [state()] * 50)
check("the comb or screwdriver off the table once: 0 for the rest", tr.score, 0.0)
tr = run([state(down=(True, True, True))] * 5)
check("the comb home, screws read down but no terminal ever opened: the comb only", tr.score, 0.25)
tr = run([state(home=False, down=(False,) * 3, gaps=(0.0036,) * 3)] + [state(home=False, down=(False,) * 3)] * 5)
check("the jaws forced open by the pins, no screw turned: nothing opened", tr.score, 0.0)

check("a jaw pushed open with its screw still at its furthest out: no clamp", t.may_clamp(4.0, 4.0), False)
check("a jaw pushed open with its screw 0.1 turn in: no clamp", t.may_clamp(3.9, 4.0), False)
check("a jaw held open as its screw is driven 1.3 turns back in: it clamps", t.may_clamp(2.7, 4.0), True)
print("all cases as the scoring text" if not bad else f"{bad} cases differ")
sys.exit(1 if bad else 0)
