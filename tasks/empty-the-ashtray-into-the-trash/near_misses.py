"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial in which water
was put on the glowing butt before anything left the ashtray, the hands ended 50 mm from both glasses and the end
state has held still for 1 s. The cases then vary the end state. _run() afterwards drives the step-side function
observe() directly, step by step, to check the whole-trial rule (the glowing butt tipped into the can, onto the
counter or onto the floor while it still glows; water put on afterwards), how the glow is put out (water deep
enough, too little water, water poured beside the ashtray or into the can, the butt dropped into the tumbler), the
1 s hold, the 5 mm let-go distance, the unknown-arm rule and lot B, and raises if anything scores wrongly. A glass thrown into the can makes every butt stop counting.
"""

import math

import numpy as np

import lots
import task
from task import ASHTRAY, BIN_CX, BIN_CY, BIN_RIM, BUTT_A, BUTT_B, BUTT_LIT, BUTT_R, D, TOP, TUMBLER, V_ASH, V_CUP

lots.use("A")
PARTS = {"ashtray": ASHTRAY, "tumbler": TUMBLER, "butt_lit": BUTT_LIT, "butt_a": BUTT_A, "butt_b": BUTT_B}
FLAT = (0.0, 0.0, 0.0, 1.0)
CLEAR = (0.05, 0.05)


def lean_quat(deg, heading=0.0):
    """A glass leaned ``deg`` degrees, its top toward ``heading`` (rad)."""
    a = math.radians(deg) / 2
    return (-math.sin(heading) * math.sin(a), math.cos(heading) * math.sin(a), 0.0, math.cos(a))


def at(name, pos, quat=FLAT):
    return PARTS[name].state(pos=pos, quat_xyzw=quat)


def start(lot="A"):
    lt = lots.all_lots()[lots.index(lot)]
    out = {}
    for n in PARTS:
        y = lt.yaw_of(n) if n.startswith("butt") else 0.0
        out[n] = at(n, lt.pos(n), (0.0, 0.0, math.sin(y / 2), math.cos(y / 2)))
    out["trash_can"] = task.BIN.state()
    return out


def in_can(k, dz=-0.070):
    """The k-th place on the paper in the can (the butt's middle ``dz`` from the rim; negative is below it)."""
    return (BIN_CX - 0.03 + 0.03 * k, BIN_CY - 0.05, BIN_RIM + dz)


def scene(lot="A", **kw):
    """The lot's start with the named objects moved: name=(pos, quat) or name=pos."""
    s = start(lot)
    for n, v in kw.items():
        pos, quat = (v, FLAT) if len(v) == 3 else v
        s[n] = at(n, pos, quat)
    return s


def binned(lot="A", which=("butt_lit", "butt_a", "butt_b"), **kw):
    return scene(lot, **({n: in_can(k) for k, n in enumerate(which)} | kw))


def pouring(lot="A", lean=55.0, over=(0.0, 0.0)):
    """The tumbler leaned over the ashtray, the lowest point of its rim 60 mm above the counter over the ashtray's
    middle (plus ``over``, m)."""
    lt = lots.all_lots()[lots.index(lot)]
    ax, ay, _ = lt.pos("ashtray")
    q = lean_quat(lean, 0.0)
    probe = at("tumbler", (0.0, 0.0, 0.0), q)
    off = V_CUP.lip(probe)
    return (ax + over[0] - off[0], ay + over[1] - off[1], TOP + 0.060 - off[2]), q


def preset():
    task.reset_record()
    task._rec.update(steps=400, lit=False, out_t=8.0, gaps=CLEAR, since=18.0, held=True, t=20.0)
    task._rec["water"].update(ashtray=0.0, tumbler=100.0)


ASH_A = lots.all_lots()[0].pos("ashtray")
CUP_A = lots.all_lots()[0].pos("tumbler")
preset()
CASES = [
    ("all three butts in the can, both glasses standing, hands clear, still 1 s", binned(), 1.0, True),
    ("one cold butt still in the ashtray", binned(which=("butt_lit", "butt_a")), 0.5, False),
    ("the doused butt still in the ashtray, two in the can", binned(which=("butt_a", "butt_b")), 0.5, False),
    ("glow out, nothing in the can yet", scene(), 0.25, False),
    ("a butt lying on the counter beside the ashtray", binned(butt_b=(0.20, 0.30, TOP + BUTT_R)), 0.5, False),
    ("a butt on the floor in front of the can", binned(butt_b=(0.20, 0.60, -0.008)), 0.5, False),
    ("a butt's middle exactly level with the rim, inside the walls", binned(butt_a=in_can(1, 0.0)), 1.0, True),
    ("a butt 1 mm above the rim (balanced on the paper heap)", binned(butt_a=in_can(1, 0.001)), 0.5, False),
    ("a butt resting on the can's rim wall, outside the inner walls",
     binned(butt_a=(BIN_CX, BIN_CY - 0.1395, BIN_RIM + 0.004)), 0.5, False),
    ("ashtray leaning 15 degrees exactly", binned(ashtray=(ASH_A, lean_quat(15.0))), 1.0, True),
    ("ashtray leaning 16 degrees (propped on something)", binned(ashtray=(ASH_A, lean_quat(16.0))), 0.75, False),
    ("ashtray left upside down on the counter",
     binned(ashtray=((ASH_A[0], ASH_A[1], TOP + 0.036), lean_quat(180.0))), 0.75, False),
    ("ashtray held 3 mm above the counter", binned(ashtray=(ASH_A[0], ASH_A[1], TOP + 0.003)), 0.75, False),
    ("ashtray dropped into the can with the butts: no butt counts", binned(ashtray=(BIN_CX + 0.1, BIN_CY, BIN_RIM - 0.08)),
     0.25, False),
    ("ashtray and tumbler both thrown into the can with the butts",
     binned(ashtray=(BIN_CX + 0.1, BIN_CY, BIN_RIM - 0.08), tumbler=(BIN_CX - 0.15, BIN_CY, BIN_RIM - 0.09)), 0.25, False),
    ("ashtray held over the can, level with its rim plus 20 mm (not in it)",
     binned(ashtray=(BIN_CX + 0.1, BIN_CY, BIN_RIM + 0.02)), 0.75, False),
    ("tumbler knocked over on the counter",
     binned(tumbler=((CUP_A[0], CUP_A[1], TOP + 0.035), lean_quat(90.0))), 0.75, False),
    ("tumbler stood somewhere else on the counter", binned(tumbler=(0.40, 0.00, TOP)), 1.0, True),
]


def _expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: scored {got}, expected {want}")
    print(f"ok    {label}: {got}")


def _run(lot, steps, label, want, check=None):
    """Drive observe() over ``steps``: (objects, gaps) per 0.05 s step, then score the last scene."""
    lots.use(lot)
    task.reset_record()
    t, last = 0.0, None
    for objects, g in steps:
        t += 0.05
        last = objects
        task.observe(objects, g, t)
    _expect(f"lot {lot}: {label}", float(task.score(last)), want)
    if check:
        check()


def seq(*parts):
    out = []
    for n, objects, g in parts:
        out += [(objects, g)] * n
    return out


def _water(lo, hi, name="ashtray"):
    def check():
        w = task._rec["water"][name]
        if not lo <= w <= hi:
            raise AssertionError(f"{name} holds {w:.1f} mL, expected {lo}..{hi}")
        print(f"ok        ({name} holds {w:.1f} mL)")
    return check


def _checks():
    for lot in ("A", "B"):
        pour = scene(lot, tumbler=pouring(lot))
        safe = seq((10, scene(lot), CLEAR), (60, pour, CLEAR), (10, scene(lot), CLEAR))
        _run(lot, seq((40, scene(lot), CLEAR)), "nothing done", 0.0)
        _run(lot, safe, "water poured on the butts, nothing tipped yet", 0.25, _water(55, 90))
        _run(lot, safe + seq((5, binned(lot, which=("butt_lit",)), CLEAR)), "then one butt in the can", 0.5)
        _run(lot, safe + seq((19, binned(lot), CLEAR)), "then all three in the can, 0.95 s still", 0.75)
        _run(lot, safe + seq((22, binned(lot), CLEAR)), "then all three in the can, 1.1 s still", 1.0)
        _run(lot, seq((10, scene(lot), CLEAR), (30, binned(lot), CLEAR)),
             "ashtray tipped straight into the can, glowing butt and all", 0.0)
        _run(lot, seq((10, scene(lot), CLEAR), (2, binned(lot), CLEAR), (60, scene(lot, tumbler=pouring(lot)), CLEAR),
                      (30, binned(lot), CLEAR)), "tipped in glowing, taken back out, doused, tipped in again", 0.0)
        _run(lot, seq((10, scene(lot), CLEAR), (30, binned(lot, which=("butt_a", "butt_b")), CLEAR)),
             "only the two cold butts in the can, the glowing one left in the dry ashtray", 0.0)
    lots.use("A")
    a = scene("A")
    lit0 = lots.all_lots()[0].pos("butt_lit")
    _run("A", seq((5, a, CLEAR), (30, scene("A", butt_lit=(0.20, 0.30, TOP + BUTT_R)), CLEAR)),
         "glowing butt tipped out onto the counter", 0.0)
    _run("A", seq((5, a, CLEAR), (3, scene("A", butt_lit=(0.20, 0.30, TOP + BUTT_R)), CLEAR),
                  (60, scene("A", tumbler=pouring("A")), CLEAR), (30, binned("A"), CLEAR)),
         "glowing butt on the counter for 0.15 s, put back, doused, binned", 0.0)
    _run("A", seq((5, a, CLEAR), (30, scene("A", butt_lit=(0.20, 0.62, -0.008)), CLEAR)),
         "glowing butt dropped on the floor beside the can", 0.0)
    _run("A", seq((5, a, CLEAR), (30, scene("A", butt_lit=(lit0[0], lit0[1], lit0[2] + 0.10)), CLEAR)),
         "ashtray lifted with the glowing butt in it (in the air, nothing landed)", 0.0)
    rim = (BIN_CX, task.BIN_Y0 + 0.0015, BIN_RIM + BUTT_R)
    _run("A", seq((5, a, CLEAR), (2, scene("A", butt_lit=rim), CLEAR), (60, scene("A", tumbler=pouring("A")), CLEAR),
                  (30, binned("A"), CLEAR)), "glowing butt left lying on the can's rim, then put back, doused, binned", 0.0)
    _run("A", seq((5, a, CLEAR), (2, scene("A", butt_lit=(rim[0], rim[1], rim[2] + 0.003)), CLEAR),
                  (60, scene("A", tumbler=pouring("A")), CLEAR), (22, binned("A"), CLEAR)),
         "glowing butt carried 3 mm above the can's rim (in the ashtray, nothing landed), doused, binned", 1.0)
    # how the glow goes out
    _run("A", seq((10, a, CLEAR), (60, scene("A", tumbler=pouring("A", lean=40.0)), CLEAR), (30, binned("A"), CLEAR)),
         "tumbler leaned only 40 degrees: under 8 mm of water in the ashtray, then tipped in", 0.0, _water(1, 37))
    _run("A", seq((10, a, CLEAR), (60, scene("A", tumbler=pouring("A", over=(0.06, 0.0))), CLEAR),
                  (30, binned("A"), CLEAR)), "water poured beside the ashtray (onto the counter), then tipped in", 0.0,
         _water(0, 0))
    can = scene("A", tumbler=((BIN_CX, BIN_CY - 0.02, BIN_RIM + 0.10), lean_quat(100.0, math.pi / 2)))
    _run("A", seq((10, a, CLEAR), (80, can, CLEAR), (30, binned("A"), CLEAR)),
         "water poured into the can over the paper instead, then the glowing butt tipped in", 0.0, _water(0, 0))
    cup = lots.all_lots()[0].pos("tumbler")
    drop = scene("A", butt_lit=((cup[0], cup[1], TOP + task.CUP_FLOOR + BUTT_R), FLAT))
    _run("A", seq((10, a, CLEAR), (10, drop, CLEAR)), "glowing butt dropped into the tumbler of water", 0.25)
    wet = seq((10, a, CLEAR), (60, scene("A", tumbler=pouring("A")), CLEAR))
    held_up = scene("A", butt_lit=(lit0[0], lit0[1], lit0[2] + 0.02))
    _run("A", seq((10, held_up, CLEAR), (60, dict(held_up, tumbler=scene("A", tumbler=pouring("A"))["tumbler"]),
                                         CLEAR)),
         "glowing end propped 20 mm up, above the water", 0.0)
    # the hold and the hands
    _run("A", wet + seq((30, binned("A"), (0.004, 0.05))), "a fingertip 4 mm from the ashtray at the end", 0.75)
    _run("A", wet + seq((30, binned("A"), (0.05, 0.004))), "a fingertip 4 mm from the tumbler at the end", 0.75)
    _run("A", wet + seq((30, binned("A"), (0.0051, 0.0051))), "every hand point 5.1 mm away", 1.0)
    _run("A", wet + seq((3, binned("A"), None), (30, binned("A"), CLEAR)), "arms unreadable for some steps", 0.0)
    moving = [(binned("A", butt_a=in_can(1, -0.070 + 0.002 * (k % 2))), CLEAR) for k in range(40)]
    _run("A", wet + moving, "a butt still rocking 2 mm in the can", 0.75)
    _run("A", wet + seq((30, binned("A"), CLEAR), (1, binned("A", ashtray=(ASH_A[0], ASH_A[1], TOP + 0.05)), CLEAR)),
         "full score reached, then the ashtray picked up again", 0.75)
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
