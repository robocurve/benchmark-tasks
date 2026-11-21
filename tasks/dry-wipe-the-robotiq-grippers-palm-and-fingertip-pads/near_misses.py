"""Near misses and unintended strategies for the gripper-pad cleaning task.

Cleaning is a history of cloth contacts, so the behaviour cases drive the scorer's own recorder
(`task.record_wipe`, which the simulator callback calls every control step with the force the simulator reports
between the cloth and each pad, and the cloth's pose). Each behaviour case starts from a fresh history and checks
score() with both hands clear and everything still. CASES (run by factory/check.py) then checks end states on top
of a fully cleaned history."""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import task  # noqa: E402

H = np.asarray(task.CLOTH_SIZE) / 2
ROBOT0 = task.ROBOT.state()
CELLS = task.pad_cells(ROBOT0)
I3 = np.eye(3)
RNG = np.random.default_rng(7)


def start_states():
    return {o.name: o.state() for o in task.OBJECTS}


def released(hold=None):
    """Both hands 30 mm clear and the scene still for the 1 s hold (what the simulator callback keeps)."""
    task._S.update(known=True, clear=True, hold=task.HOLD_STEPS if hold is None else hold)


def path(pad, centres, force=3.0, rot=I3, forces=None):
    """Feed the cloth along ``centres`` (its centre, world), pressed on ``pad`` with ``force`` N, step by step."""
    for c in centres:
        task.record_wipe(forces if forces is not None else {pad: force}, (np.asarray(c, float), rot), CELLS)
    task.no_wipe()


def palm_centres(dz=0.0, sweep=0.008, n=4, step=0.002, gap=0.0):
    """The cloth's end on the palm pad (``gap`` m off its face), swept across it ``n`` times each way."""
    x = task.X_PALM - H[0] - gap
    ys = []
    for _ in range(n):
        ys += list(np.arange(-sweep, sweep + 1e-9, step)) + list(np.arange(sweep, -sweep - 1e-9, -step))
    return [(x, y, task.Z_AX + dz) for y in ys]


def tip_centres(s, sweep=0.005, n=4, step=0.002, deep=0.0225, gap=0.0):
    """The cloth's side on fingertip pad ``s`` (+1 left, -1 right), its end ``deep`` m short of the palm,
    wiped up and down ``n`` times."""
    x = task.X_PALM - deep - H[0]
    y = s * (task.PAD_TIP[0] / 1000 - H[1] - gap)
    zs = []
    for _ in range(n):
        zs += list(np.arange(-sweep, sweep + 1e-9, step)) + list(np.arange(sweep, -sweep - 1e-9, -step))
    return [(x, y, task.Z_AX + z) for z in zs]


def palm(**kw):
    path("palm_pad", palm_centres(**kw))


def tip(s, force=3.0, **kw):
    path("tip_l" if s > 0 else "tip_r", tip_centres(s, **kw), force=force)


def full(force=3.0, step_scale=1.0):
    path("palm_pad", palm_centres(step=0.002 * step_scale, n=max(4, int(4 / step_scale))), force=force)
    tip(1, force=force, step=0.002 * step_scale, n=max(4, int(4 / step_scale)))
    tip(-1, force=force, step=0.002 * step_scale, n=max(4, int(4 / step_scale)))


RESULTS = []


def case(label, setup, want, objects=None):
    task.reset_state()
    released()
    setup()
    st = start_states()
    st.update(objects or {})
    got = task.score(st)
    RESULTS.append((label, got, want))
    mark = "ok  " if abs(got - want) < 1e-9 else "FAIL"
    print(f"{mark} {label}: {got} (want {want}; zones clean {task.n_clean()})")


case("all three pads wiped, cloth back on the mat, hands clear, still", full, 1.0)
case("nothing done", lambda: None, 0.0)
case("cloth moved over every pad, no contact reported (hover)", lambda: (
    path("palm_pad", palm_centres(), force=0.0), path("tip_l", tip_centres(1), force=0.0),
    path("tip_r", tip_centres(-1), force=0.0)), 0.0)
case("cloth 3 mm off every pad face, contact force reported elsewhere on the pad", lambda: (
    path("palm_pad", palm_centres(gap=0.003)), path("tip_l", tip_centres(1, gap=0.003)),
    path("tip_r", tip_centres(-1, gap=0.003))), 0.0)
case("too light: 0.4 N everywhere", lambda: full(force=0.4), 0.0)
case("exactly 0.5 N counts", lambda: full(force=0.5), 1.0)
case("a hard press (40 N) still cleans", lambda: full(force=40.0), 1.0)
case("jabs: cloth pressed on each pad at 20 points without sliding", lambda: [
    (task.record_wipe({p: 3.0}, (np.asarray(c, float), I3), CELLS), task.no_wipe())
    for p, cs in (("palm_pad", palm_centres()), ("tip_l", tip_centres(1)), ("tip_r", tip_centres(-1)))
    for c in cs[::4]], 0.0)
case("force read on the palm while the cloth slides over the fingertips (wrong pad)", lambda: (
    path("tip_l", tip_centres(1), forces={"palm_pad": 3.0}),
    path("tip_r", tip_centres(-1), forces={"palm_pad": 3.0})), 0.0)
case("cloth pressed and held on every pad, jittering 0.1 mm a step for 2,000 steps (not a wipe)", lambda: [
    path(p, [np.asarray(c, float) + RNG.uniform(-0.0001, 0.0001, 3) for _ in range(2000)])
    for p, c in (("palm_pad", palm_centres()[0]), ("tip_l", tip_centres(1)[0]), ("tip_r", tip_centres(-1)[0]))], 0.0)
case("slow wipe: 0.24 mm a step (4.8 mm/s, under 5 mm/s) counts nothing", lambda: full(step_scale=0.12), 0.0)
case("wipe at 0.26 mm a step (5.2 mm/s) counts", lambda: full(step_scale=0.13), 1.0)
case("teleport-sized steps (13 mm a step) are not wiping", lambda: path(
    "palm_pad", [(task.X_PALM - H[0], 0.0065 * (-1) ** k, task.Z_AX) for k in range(40)]), 0.0)
case("one short sweep on the palm only (under 20 mm of travel)", lambda: path(
    "palm_pad", palm_centres(n=1, sweep=0.004)), 0.0)
case("palm pad only: 2 zones", palm, 0.25)
case("palm pad and the left fingertip: 5 zones", lambda: (palm(), tip(1)), 0.5)
case("both fingertips only: 6 zones", lambda: (tip(1), tip(-1)), 0.5)
case("everything but the right fingertip's tip third: 7 zones", lambda: (
    palm(), tip(1), path("tip_r", tip_centres(-1, deep=0.0225 + 0.0127 * 2 / 3))), 0.5)
case("fingertips wiped along one edge only (cloth 17 mm low): their top rows missed", lambda: (
    palm(), path("tip_l", [(x, y, z - 0.017) for x, y, z in tip_centres(1)]),
    path("tip_r", [(x, y, z - 0.017) for x, y, z in tip_centres(-1)])), 0.25)
case("all 8 zones but a hand still within 30 mm (holding the cloth)", lambda: (
    full(), task._S.update(clear=False)), 0.0)
case("all 8 zones, let go, still for only 19 steps (under 1 s)", lambda: (full(), released(task.HOLD_STEPS - 1)),
     0.0)
case("all 8 zones, let go, still for exactly 20 steps (1 s)", lambda: (full(), released(task.HOLD_STEPS)), 1.0)
case("all 8 zones but a gripper finger touched a pad on the way", lambda: (full(), task._S.update(touched=True)), 0.0)
case("all 8 zones but the cloth fell off the table on the way", lambda: (full(), task._S.update(dropped=True)), 0.0)
case("all 8 zones but a contact or hand read failed (fail closed)", lambda: (full(), task._S.update(broken=True)),
     0.0)
case("all 8 zones but the hands were never read (fail closed)", lambda: (full(), task._S.update(known=False)), 0.0)


def _five_of_six():
    """The right fingertip's tip third with 5 of its 6 cells cleared (83%, under the 90% rule)."""
    palm()
    tip(1)
    tip(-1)
    z = np.flatnonzero(task.CELL_ZONE == 7)
    task._S["travel"][z[0]] = 0.0


case("one fingertip zone with 5 of its 6 cells clean is not clean (7 zones)", _five_of_six, 0.5)


def _eleven_of_twelve():
    full()
    z = np.flatnonzero(task.CELL_ZONE == 0)
    task._S["travel"][z[0]] = 0.0


case("a palm zone with 11 of its 12 cells clean (92%) is clean", _eleven_of_twelve, 1.0)

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# End-state cases on top of a fully cleaned, released history (run in order by factory/check.py).
task.reset_state()
released()
full()
C = np.array(task.CLOTH.pos)
CL = task.CLOTH


def _tilted(deg):
    a = np.radians(deg) / 2
    return (float(np.sin(a)), 0.0, 0.0, float(np.cos(a)))


CASES = [
    ("cleaned, cloth back where it started on the mat, all still", {}, 1.0, True),
    ("cloth still sliding (2 cm/s)", {"cloth": CL.state(lin_vel=(0.0, 0.02, 0.0))}, 0.0, False),
    ("cloth resting 2.5 mm high on the mat (within 3 mm)", {"cloth": CL.state(pos=tuple(C + [0, 0, 0.0025]))}, 1.0,
     True),
    ("cloth held 1 cm above the mat, still", {"cloth": CL.state(pos=tuple(C + [0, 0, 0.010]))}, 0.75, False),
    ("cloth on the table beside the mat, not on it", {"cloth": CL.state(pos=(0.0, 0.0, task.TOP + H[2]))}, 0.75,
     False),
    ("cloth half over the mat's edge", {"cloth": CL.state(pos=(task.MAT_CENTER[0], 0.29, task.MAT_TOP + H[2]))}, 0.75,
     False),
    ("cloth on the mat, under the open gripper but not touching it",
     {"cloth": CL.state(pos=(0.30, 0.0, task.MAT_TOP + H[2]))}, 1.0, True),
    ("cloth propped up at 20 degrees on the mat", {"cloth": CL.state(pos=tuple(C + [0, 0, 0.012]),
                                                                    quat_xyzw=_tilted(20))}, 0.75, False),
    ("cloth left lying in the gripper on the palm and links",
     {"cloth": CL.state(pos=(task.X_PALM - H[0], 0.0, task.Z_AX))}, 0.0, False),
    ("cloth left hanging on a fingertip pad",
     {"cloth": CL.state(pos=(task.X_PALM - 0.0225 - H[0], task.PAD_TIP[0] / 1000 - H[1], task.Z_AX))}, 0.0, False),
    ("cloth back where it started, still (after the cases above)", {}, 1.0, True),
]
