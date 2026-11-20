"""The scorer on hand-built states, with no simulator: each rung, each rule and a case that fails it.

    python -B check_score.py

Every case is a run of per-step Seen states fed to task.py's own Trial. Exits non-zero if any case scores other than the scoring text says.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
dk = sys.modules["demokit"] = types.ModuleType("demokit")  # the simulator side of the kit: task.py needs only its constants here
dk.TABLE_TOP, dk.DT = 0.0, 0.05
for fn in ("level", "tilted", "down", "_as_grip"):
    setattr(dk, fn, lambda *a, **kw: None)
ya = sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # mechkit's geometry and USD text work without the simulator
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")
tf.Box = type("Box", (), {})
tf.ObjectState = type("ObjectState", (), {})
import dataclasses  # noqa: E402

tf.Box = dataclasses.make_dataclass("Box", [("name", str), ("pos", tuple), ("size", tuple), ("mass", float, 0.0)], frozen=True)
spec = importlib.util.spec_from_file_location("mailer_task", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
m = t.m
m.lock = m.unlock = lambda *a, **kw: None
T, H = t.T, t.H
HOLE = t.HOLE_X


class Env:
    def __init__(self, seen):
        self.seen = seen


class Box:
    joints = {f"{n}_joint": 0.0 for n in ("tab_l", "tab_r", "tab2_l", "tab2_r")}
    pos = np.zeros(3)

    def rotation(self):
        return np.eye(3)


m.state = lambda env, name: Box()
t.Trial._spring = lambda self, *a, **kw: None
t.Trial.read = lambda self, env: env.seen

UP = (-0.05, 0.0, 0.30)
EDGE_DOWN = (0.017, 0.0, 0.006)


def end(state="open", outer=None, ears=(0.0, 0.0)):
    """One end: open (unrolled, wall leaning), rolled (over the top, edge inside), down (rolled flat inside, no tab in), one (one tab in),
    locked (both tabs in, wall upright), laid (rolled down, tabs on the base's edge, not in their holes)."""
    roll, edge, o = {"open": (0.0, UP, -10.0), "rolled": (120.0, (0.017, 0.0, 0.10), -3.0)}.get(state, (179.5, EDGE_DOWN, -0.8))
    o = o if outer is None else outer
    zin, zup = 0.0005, 0.30
    if state == "laid":
        zin = zup = 0.004
    z = (zin, zin) if state in ("locked",) else (zin, zup) if state == "one" else (zup, zup) if state != "laid" else (0.004, 0.004)
    tips = (np.array([HOLE, t.TAB_Y, z[0]]), np.array([HOLE, -t.TAB_Y, z[1]]))
    return t.EndSeen(o, roll, edge, tips, ears)


LID_UP, LID_SHUT = 0.0, 89.5
TUCK_IN = (t.LX / 2, -t.WY / 2 + t.T + 0.002, 0.115)  # mid-board, its face on the wall (GPU demo: 1.8 mm)
TUCK_OUT = (t.LX / 2, -t.WY / 2 - 0.038, t.H + 0.002)


def seen(e1="open", e2="open", lid=LID_UP, tuck=0.0, tuck_edge=(t.LX / 2, t.WY / 2 + t.T / 2, 0.51), clear=0.2, off=False, ears1=(0.0, 0.0), outer1=None, outer2=None):
    return t.Seen((end(e1, outer=outer1, ears=ears1), end(e2, outer=outer2)), lid, tuck, tuck_edge, clear, off)


DONE = dict(e1="locked", e2="locked", lid=LID_SHUT, tuck=100.0, tuck_edge=TUCK_IN)


def run(phases):
    tr = t.TRIAL
    tr.reset()
    for n, kw in phases:
        for _ in range(n):
            tr.step(Env(seen(**kw)))
    return tr.score


bad = 0


def check(label, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {label}")


N = 3 * t.HOLD_STEPS
check("doing nothing", run([(N, {})]), 0.0)
check("0.1: end 1 rolled over the top", run([(N, dict(e1="rolled"))]), 0.1)
check("0.2: end 1 down inside, no tab in", run([(N, dict(e1="down"))]), 0.2)
check("0.2: end 1 tabs laid on the base, not in their holes (lazy route)", run([(N, dict(e1="laid"))]), 0.2)
check("0.3: end 1 one tab in", run([(N, dict(e1="one"))]), 0.3)
check("0.3: end 1 both tabs in, its wall leaning 10 degrees", run([(N, dict(e1="locked", outer1=-10.0))]), 0.3)
check("0.4: end 1 locked only (one end, lazy route)", run([(N, dict(e1="locked"))]), 0.4)
check("0.4: end 2 locked only", run([(N, dict(e2="locked"))]), 0.4)
check("0.8: both ends locked, lid standing", run([(N, dict(e1="locked", e2="locked"))]), 0.8)
check("0.7: both ends locked but end 2's wall leaning 10 degrees (end 2 at 0.75)", run([(N, dict(e1="locked", e2="locked", outer2=-10.0))]), 0.7)
check("0.9: both ends locked, lid shut, tuck out", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, tuck_edge=TUCK_OUT))]), 0.9)
check("0.9: both ends locked, lid shut, tuck folded but its edge still above the wall's top", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, tuck=100.0, tuck_edge=(t.LX / 2, -t.WY / 2 + t.T + 0.004, t.H - 0.01)))]), 0.9)
check("0.9: both ends locked, lid shut, tuck folded 96 degrees with its edge 2.6 mm in, a 1 mm gap to the wall (hovering; red team)", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, tuck=96.0, tuck_edge=(t.LX / 2, -t.WY / 2 + t.T + 0.0026, 0.115)))]), 0.9)
check("1.0: the same with the edge 2.0 mm in (face on the wall, as the demo reads 1.8)", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, tuck=96.0, tuck_edge=(t.LX / 2, -t.WY / 2 + t.T + 0.0020, 0.115)))]), 1.0)
check("1.0 (lean): end 1's tabs in with its wall leaning 6 degrees for 2 s, then stood upright: it locks then, the rest done", run([(2 * N, dict(e1="locked", e2="locked", outer1=-6.0, lid=LID_SHUT, tuck=100.0, tuck_edge=TUCK_IN)), (N, DONE)]), 1.0)
check("0.7 (lean): the same, the wall left leaning 6 degrees to the end", run([(3 * N, dict(e1="locked", e2="locked", outer1=-6.0, lid=LID_SHUT, tuck=100.0, tuck_edge=TUCK_IN))]), 0.7)
check("0.9: both ends locked, lid shut, tuck curled under it with its edge 2.6 cm clear of the front wall (not bearing on it)", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, tuck=121.0, tuck_edge=(t.LX / 2, -t.WY / 2 + t.T + 0.026, 0.118)))]), 0.9)
check("0.9: both ends locked, lid shut, tuck folded 96 degrees with its face 6 mm off the front wall (hovering, not bearing)", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, tuck=96.0, tuck_edge=(t.LX / 2, -t.WY / 2 + t.T + 0.0076, 0.115)))]), 0.9)
check("0: the lid shut and tucked with both ends open (lid only, lazy route)", run([(N, dict(lid=LID_SHUT, tuck=100.0, tuck_edge=TUCK_IN))]), 0.0)
check("0.4: end 1 locked, lid shut and tucked over end 2 open", run([(N, dict(e1="locked", lid=LID_SHUT, tuck=100.0, tuck_edge=TUCK_IN))]), 0.4)
check("0.95: everything done, a fingertip 2 cm from the box", run([(N, DONE | {"clear": 0.02})]), 0.95)
check("0: everything done for 19 steps only, after nothing (not held 1 s)", run([(N - 1, {}), (t.HOLD_STEPS - 1, DONE)]), 0.0)
check("1.0: everything done, hands clear, still 1 s", run([(N, DONE)]), 1.0)
check("0.95: everything done, hands clear, the lid still rocking 1.5 degrees", run([(N, DONE)] + [(1, dict(DONE, lid=LID_SHUT - 1.5 * (k % 2))) for k in range(N)]), 0.95)
check("1.0: everything done, the lid rocking until the last 1 s, then still (1 s of stillness is enough)", run([(1, dict(DONE, lid=LID_SHUT - 1.5 * (k % 2))) for k in range(N)] + [(t.HOLD_STEPS, DONE)]), 1.0)
check("0.9: both ends locked, the lid held shut by a hand, tuck unfolded (shut, lazy route)", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, clear=0.0))]), 0.9)
check("0.8: both ends locked, the lid held shut, then let go: it springs to 20 degrees (held then released)", run([(N, dict(e1="locked", e2="locked", lid=LID_SHUT, clear=0.0)), (N, dict(e1="locked", e2="locked", lid=20.0))]), 0.8)
check("0.6: end 1 locked, lid shut and tucked over end 2 rolled down but not locked (lid over an unlocked end)", run([(N, dict(e1="locked", e2="down", lid=LID_SHUT, tuck=100.0, tuck_edge=TUCK_IN))]), 0.6)
check("0: nothing folded, the box pushed and turned (tiers are read in the box's frame; only off the bench fails)", run([(N, dict(clear=0.0))]), 0.0)

# partial credit is the state at the end, held 1 s: a step undone before the end loses its rung
check("0.8 (undo): everything done for 2 s, then the lid springs open with its tuck out", run([(2 * N, DONE), (N, dict(e1="locked", e2="locked", lid=20.0, tuck_edge=TUCK_OUT))]), 0.8)
check("0.8 (undo): lid shut and tucked for 3 s, then opened to 80 degrees for the last 0.5 s", run([(N, DONE), (10, dict(DONE, lid=80.0, tuck_edge=TUCK_OUT))]), 0.8)
check("0.4 (undo): end 2 rolled down, then let spring back open", run([(N, dict(e1="locked", e2="down")), (N, dict(e1="locked"))]), 0.4)

# failures latch: the box leaves the bench or a dust flap is forced out, then the act is done
check("0: the box off the bench while all else is done", run([(N, DONE | {"off": True})]), 0.0)
check("0: a dust flap swung out 40 degrees (damage)", run([(N, DONE | {"ears1": (0.0, 40.0)})]), 0.0)
check("0: box off the bench for 3 steps, then back, then the whole act", run([(3, {"off": True}), (N, DONE)]), 0.0)
check("0: a flap forced out 40 degrees for 3 steps, then back, then the act", run([(3, {"ears1": (40.0, 0.0)}), (N, DONE)]), 0.0)

print("FAILED" if bad else "all cases as the scoring text says")
sys.exit(1 if bad else 0)
