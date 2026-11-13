"""Near misses and unintended strategies for the peeling task.

Peeling is a history of blade-edge contacts, so most cases are driven through the scorer's own contact recorder
(`task.record_contact`, the function the simulator callback calls every control step with the contact the
simulator reports between the blade-edge collider and the cucumber). Each behaviour case below starts from a
fresh history and checks score() with everything let go and still. CASES (run by factory/check.py) then checks the
end-state conditions on top of a fully peeled history."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import task  # noqa: E402

DT = 0.05
STRIP = 2 * math.pi / task.N_STRIP


def stroke(phi, force=3.0, facing=1.0, across=0.0, s0=-0.0668, s1=0.0733, speed=0.03, t0=0.0, dphi=0.0):
    """Feed one edge-first stroke along the body, as the simulator would report it step by step."""
    n = max(2, int(abs(s1 - s0) / (speed * DT)))
    for i in range(n + 1):
        s = s0 + (s1 - s0) * i / n
        task.record_contact(s, (phi + dphi * i / n) % (2 * math.pi), force, facing, across, t0 + i * DT)
    task.no_contact()


def released():
    """Both hands let go and everything still for the 1 s hold (the counter the simulator callback keeps)."""
    task._S.update(known=True, clear=True, still_steps=task.HOLD_STEPS)


def start_states():
    return {o.name: o.state() for o in task.OBJECTS}


def peel(strips, **kw):
    for k in strips:
        stroke((k + 0.5) * STRIP, **kw)


RESULTS = []


def case(label, setup, want):
    task.reset_state()
    setup()
    if "clear" not in task._S or task._S.get("known") is None:
        released()
    got = task.score(start_states())
    RESULTS.append((label, got, want))
    mark = "ok  " if abs(got - want) < 1e-9 else "FAIL"
    print(f"{mark} {label}: {got} (want {want})")


def _full():
    released()
    peel(range(8))


case("full peel, all 8 strips, let go and still", _full, 1.0)
case("nothing done", released, 0.0)
case("hover: blade over every strip, no contact reported", lambda: (released(), task.no_contact()), 0.0)
case("too light: 0.8 N on every strip", lambda: (released(), peel(range(8), force=0.8)), 0.0)
case("too hard: 10.5 N on every strip (gouged)", lambda: (released(), peel(range(8), force=10.5)), 0.0)
case("crushing: 13 N on every strip (gouged)", lambda: (released(), peel(range(8), force=13.0)), 0.0)
case("exactly 1 N and exactly 10 N count", lambda: (released(), peel(range(4), force=1.0),
                                                   peel(range(4, 8), force=10.0)), 1.0)
case("backwards: drawn blade first", lambda: (released(), peel(range(8), facing=1.0, s0=0.0733, s1=-0.0668)), 0.0)
case("blade along the body (sideways wipe position)", lambda: (released(), peel(range(8), across=0.95)), 0.0)
case("too slow: 0.5 cm/s strokes", lambda: (released(), peel(range(8), speed=0.005)), 0.0)
case("jabs: the edge pressed at 20 points, no travel",
     lambda: (released(), [task.record_contact(s, (k + .5) * STRIP, 3.0, 1.0, 0.0, 0.0) or task.no_contact()
                           for k in range(8) for s in np.linspace(-0.065, 0.065, 20)]), 0.0)
case("one stroke on the top only (6 patches)", lambda: (released(), peel([2])), 0.0)
case("two strips (12 patches)", lambda: (released(), peel([2, 3])), 0.25)
case("five strips (30 patches)", lambda: (released(), peel(range(5))), 0.25)
case("six strips (36 patches)", lambda: (released(), peel(range(6))), 0.5)
case("all strips, two gouged patches", lambda: (released(), peel(range(8)),
                                                task.record_contact(-0.0542, 0.5 * STRIP, 13.0, 1.0, 0.0, 99.0),
                                                task.record_contact(-0.0108, 2.5 * STRIP, 11.0, 1.0, 0.0, 99.1)), 0.5)
case("all strips, one gouged patch", lambda: (released(), peel(range(8)),
                                              task.record_contact(-0.0542, 0.5 * STRIP, 13.0, 1.0, 0.0, 99.0)), 0.5)
case("half-length strokes only (middle 6.5 cm)", lambda: (released(), peel(range(8), s0=-0.03, s1=0.035)), 0.25)
case("strokes run into one end (past 80 mm)", lambda: (released(), peel(range(8), s1=0.085)), 0.0)
case("strokes start inside the other end", lambda: (released(), peel(range(8), s0=-0.085)), 0.0)
case("strokes stop 1 cm short of the scored skin's end (last band half done)",
     lambda: (released(), peel(range(8), s1=0.050)), 0.5)
case("full peel, let go, but still for only 19 steps (under 1 s)",
     lambda: (released(), peel(range(8)), task._S.update(still_steps=task.HOLD_STEPS - 1)), 0.5)
case("full peel, let go, still for exactly 20 steps (1 s)",
     lambda: (released(), peel(range(8)), task._S.update(still_steps=task.HOLD_STEPS)), 1.0)
case("full peel but a hand still on the cucumber or peeler",
     lambda: (task._S.update(known=True, clear=False), peel(range(8))), 0.0)
case("full peel but hand state never read (fail closed)",
     lambda: (task._S.update(known=False, clear=True), peel(range(8))), 0.0)
case("full peel but the blade touched a gripper",
     lambda: (released(), peel(range(8)), task._S.update(blade_hit_hand=True)), 0.0)
case("full peel but the cucumber fell off the board on the way",
     lambda: (released(), peel(range(8)), task._S.update(dropped=True)), 0.0)
case("full peel but a contact read failed", lambda: (released(), peel(range(8)), task._S.update(broken=True)), 0.0)
case("strokes while the body creeps 5 degrees round under the blade",
     lambda: (released(), peel(range(8), dphi=math.radians(5))), 1.0)

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# End-state cases on top of a fully peeled, released history (run in order by factory/check.py).
task.reset_state()
_full()
C = task.CUCUMBER.pos
P = task.PEELER.pos
CASES = [
    ("fully peeled, everything let go and still", {}, 1.0, True),
    ("cucumber still rolling (2 cm/s)", {"cucumber": task.CUCUMBER.state(lin_vel=(0.0, 0.02, 0.0))}, 0.0, False),
    ("peeler still moving (2 cm/s)", {"peeler": task.PEELER.state(lin_vel=(0.02, 0.0, 0.0))}, 0.0, False),
    ("cucumber held 1 cm above the board", {"cucumber": task.CUCUMBER.state(pos=(C[0], C[1], C[2] + 0.01))}, 0.0,
     False),
    ("cucumber resting 2 mm high (within the 3 mm tolerance)", {"cucumber": task.CUCUMBER.state(pos=(C[0], C[1], C[2] + 0.002))},
     1.0, True),
]
