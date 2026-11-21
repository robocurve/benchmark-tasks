"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial in which no hand
came within 1 mm of the hot iron or the cornbread, the hands ended 50 mm from the skillet and the holder, and the end
state has held still for 1 s. The cases then vary the end state: how far the holder is on, where the skillet stands.
_run() afterwards drives the step-side function observe() directly, step by step, to check the whole-trial rules (a
hand on the bare iron or the cornbread, the skillet tipped past 30 degrees, set on the bare counter, off the counter),
the 1 s hold, the 30 mm let-go distance and the unknown-arm rule in both lots, and raises if anything scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import HANDLE_TIP, HANDLE_ZC, HOB_TOP, MOUTH_X, CAV, TOP, TRIV_T, TRIVET_AT

lots.use("A")
FLAT = (0.0, 0.0, 0.0, 1.0)
CLEAR = (0.05, 0.05)


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def q_tilt(deg, yaw=0.0):
    """Yaw first, then a lean of ``deg`` about the world x axis."""
    a = math.radians(deg) / 2
    qx = np.array([math.sin(a), 0.0, 0.0, math.cos(a)])
    qy = np.array(qz(yaw))
    x1, y1, z1, w1 = qx
    x2, y2, z2, w2 = qy
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def lot_of(name):
    return lots.all_lots()[lots.index(name)]


def pan_at(pos, yaw, quat=None):
    return task.SKILLET.state(pos=pos, quat_xyzw=quat if quat is not None else qz(yaw))


def holder_on(pan_state, depth, off_yaw=0.0, dy=0.0):
    """The holder on the skillet's handle with ``depth`` m of the handle inside it (its mouth facing the pan),
    turned ``off_yaw`` rad off line, and moved ``dy`` m sideways, all in the skillet's frame."""
    r = task._rot(pan_state.quat_xyzw)
    c_local = np.array([HANDLE_TIP - depth + MOUTH_X, dy, HANDLE_ZC - (CAV["z0"] + CAV["z1"]) / 2])
    pos = np.asarray(pan_state.pos, float) + r @ c_local
    pyaw = math.atan2(r[1, 0], r[0, 0])
    return task.HOLDER.state(pos=pos, quat_xyzw=qz(pyaw + math.pi + off_yaw))


def start(lot="A"):
    lt = lot_of(lot)
    return {
        "skillet": pan_at(lt.pos("skillet"), lt.yaw_of("skillet")),
        "holder": task.HOLDER.state(pos=lt.pos("holder"), quat_xyzw=qz(lt.yaw_of("holder"))),
        "trivet": task.TRIVET.state(pos=(*TRIVET_AT, TOP)),
        "hob": task.HOB.state(),
    }


def served(lot="A", depth=0.050, dx=0.0, dz=0.0, yaw=math.pi, quat=None, off_yaw=0.0):
    """The skillet on the trivet (its middle ``dx`` m off the trivet's middle, ``dz`` m above the cork) with the
    holder ``depth`` m onto the handle."""
    s = start(lot)
    p = pan_at((TRIVET_AT[0] + dx, TRIVET_AT[1], TOP + TRIV_T + dz), yaw, quat)
    s["skillet"] = p
    s["holder"] = holder_on(p, depth, off_yaw)
    return s


def on_hob(lot="A", depth=0.050, dy=0.0):
    s = start(lot)
    s["holder"] = holder_on(s["skillet"], depth, dy=dy)
    return s


def lifted(lot="A", depth=0.050, z=0.06):
    s = start(lot)
    lt = lot_of(lot)
    x, y, _ = lt.pos("skillet")
    p = pan_at((x, y + 0.1, HOB_TOP + z), lt.yaw_of("skillet"))
    s["skillet"] = p
    s["holder"] = holder_on(p, depth)
    return s


def preset():
    task.reset_record()
    task._rec.update(steps=400, gaps=CLEAR, since=18.0, held=True, t=20.0)


preset()
CASES = [
    ("skillet on the trivet, holder 50 mm on the handle, hands clear, still 1 s", served(), 1.0, True),
    ("holder exactly at the white line (40 mm on)", served(depth=0.040), 1.0, True),
    ("holder 1 mm short of the white line (39 mm on)", served(depth=0.039), 0.0, False),
    ("holder only 20 mm onto the handle", served(depth=0.020), 0.0, False),
    ("holder lying on the counter, skillet on the trivet with a bare handle",
     dict(served(), holder=task.HOLDER.state(pos=(0.15, 0.01, TOP), quat_xyzw=qz(-math.pi / 2))), 0.0, False),
    ("holder on but turned 25 degrees off the handle's line (resting on it, not over it)",
     served(off_yaw=math.radians(25.0)), 0.0, False),
    ("skillet's middle 30 mm off the trivet's middle", served(dx=0.030), 1.0, True),
    ("skillet's middle 35 mm off the trivet's middle (base overhangs the cork)", served(dx=0.035), 0.5, False),
    ("skillet held 6 mm above the trivet", served(dz=0.006), 0.5, False),
    ("skillet on the trivet leaning 8 degrees", served(quat=q_tilt(8.0, math.pi)), 1.0, True),
    ("skillet on the trivet leaning 10 degrees (propped on the holder)", served(quat=q_tilt(10.0, math.pi)), 0.5,
     False),
    ("handle pointing away from the robot on the trivet (any direction is fine)", served(yaw=0.0), 1.0, True),
    ("holder on, skillet still on the hob", on_hob(), 0.25, False),
    ("holder pushed fully on, skillet still on the hob", on_hob(depth=0.057), 0.25, False),
    ("holder only 30 mm on, skillet still on the hob", on_hob(depth=0.030), 0.0, False),
    ("holder on, skillet lifted 60 mm off the hob", lifted(), 0.5, False),
    ("trivet tipped up on edge under the skillet",
     dict(served(), trivet=task.TRIVET.state(pos=(*TRIVET_AT, TOP + 0.02), quat_xyzw=q_tilt(20.0))), 0.5, False),
]


def _expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: scored {got}, expected {want}")
    print(f"ok    {label}: {got}")


def _run(lot, steps, label, want):
    """Drive observe() over ``steps``: (objects, gaps) per 0.05 s step, then score the last scene."""
    lots.use(lot)
    task.reset_record()
    t, last = 0.0, None
    for objects, g in steps:
        t += 0.05
        last = objects
        task.observe(objects, g, t)
    _expect(f"lot {lot}: {label}", float(task.score(last)), want)


def seq(*parts):
    out = []
    for n, objects, g in parts:
        out += [(objects, g)] * n
    return out


def _checks():
    for lot in ("A", "B"):
        s0, h, up, sv = start(lot), on_hob(lot), lifted(lot), served(lot)
        _run(lot, seq((40, s0, CLEAR)), "nothing done", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR)), "holder slid on, skillet left on the hob", 0.25)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (20, up, CLEAR)), "then lifted by the holder", 0.5)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (20, up, CLEAR), (19, sv, CLEAR)),
             "then set on the trivet, still 0.95 s", 0.75)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (20, up, CLEAR), (22, sv, CLEAR)),
             "then set on the trivet, still 1.1 s", 1.0)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (20, up, CLEAR), (30, sv, (0.05, 0.025))),
             "set on the trivet, a finger still 25 mm from the holder", 0.75)
        _run(lot, seq((10, s0, (0.0005, 0.0005)), (20, h, CLEAR), (20, up, CLEAR), (30, sv, CLEAR)),
             "a finger on the bare iron for one step before the holder went on", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (1, up, (0.0, 0.0)), (20, up, CLEAR), (30, sv, CLEAR)),
             "a finger on the cornbread or the rim for one step while carrying", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (20, up, CLEAR), (30, sv, (0.0011, 0.05))),
             "the closest hand point 1.1 mm from the iron all through the end", 1.0)
        # the naive errand: the bare handle gripped and the skillet carried to the trivet
        bare = served(lot)
        bare["holder"] = start(lot)["holder"]
        _run(lot, seq((10, s0, CLEAR), (5, s0, (-0.001, -0.001)), (20, lifted(lot), (-0.001, -0.001)),
                      (30, bare, CLEAR)), "naive: bare handle gripped, skillet carried to the trivet", 0.0)
        _run(lot, seq((10, s0, CLEAR), (5, s0, (-0.001, -0.001)), (20, up, (-0.001, -0.001)), (30, sv, CLEAR)),
             "bare handle gripped first, holder put on afterwards, served", 0.0)
        tipped = dict(up)
        tipped["skillet"] = pan_at(up["skillet"].pos, 0.0, q_tilt(35.0, lot_of(lot).yaw_of("skillet")))
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (2, tipped, CLEAR), (20, up, CLEAR), (30, sv, CLEAR)),
             "skillet tipped 35 degrees while carried (cornbread falls), then served", 0.0)
        down = dict(up)
        down["skillet"] = pan_at((0.15, 0.20, TOP), math.pi)
        down["holder"] = holder_on(down["skillet"], 0.050)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (3, down, CLEAR), (20, up, CLEAR), (30, sv, CLEAR)),
             "skillet set down on the bare counter for 0.15 s, then served", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (20, up, CLEAR), (3, sv, None), (30, sv, CLEAR)),
             "arms unreadable for some steps", 0.0)
        _run(lot, seq((10, s0, CLEAR), (20, h, CLEAR), (20, up, CLEAR), (30, sv, CLEAR),
                      (5, dict(sv, holder=start(lot)["holder"]), CLEAR)),
             "served, then the holder taken off again and put back on the counter", 0.0)
    lots.use("A")
    preset()


if __name__ == "__main__":
    _checks()
    st = start("A")
    for label, changes, want, want_success in CASES:
        s = dict(st)
        s.update(changes)
        got = 1.0 if task.success(s) else float(task.score(s))
        _expect(label, got, want)
else:
    _checks()
