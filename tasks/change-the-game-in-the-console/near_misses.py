"""Near misses and unintended strategies, each played as a short timeline through the scorer's own step record.

The scorer keeps a per-trial record that only the simulator's step and reset hooks write (the push-push catch, the
whole-trial faults, hands clear, still for how long). Each case below resets that record, plays a timeline of states
at 20 Hz through every loaded copy of the task's step function, and leaves the timeline's last state for check.py to
score. Run alone: python near_misses.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task  # noqa: E402
import lots  # noqa: E402
import parts  # noqa: E402

DT = 0.05
MM = 1e-3
FAR = [np.array([[0.15, 0.30, task.TOP + 0.30]])]  # hands parked well away
UP, DOWN, PRESSED = task.RAISED, 0.0, -0.025
LID_OPEN = task.LID_OPEN


def _quat(rot):
    return tuple(parts._matrix_to_quat(rot))


def console(tray=0.0, lid=0.0):
    return task.CONSOLE.state(joints={"tray": tray, "lid": lid})


def in_tray(spec, tray, d=(0.0, 0.0, 0.0), turn_deg=0.0, axis=(0, 0, 1), vel=None):
    """A pak in the tray at angle `tray`, offset d (tray frame, m) from its loaded home, turned about `axis`."""
    p = task.world(task.tray_point(task.PAK_C0 + np.asarray(d, float), tray))
    r = task._rot_y(tray) @ parts._rot(axis, math.radians(turn_deg))
    return spec.state(pos=tuple(p), quat_xyzw=_quat(r), lin_vel=vel)


def on_stand(spec, side, dz=0.0, dx=0.0, tilt_deg=0.0):
    p = np.asarray(task.parked_pos(side)) + np.array([dx, 0.0, dz])
    return spec.state(pos=tuple(p), quat_xyzw=_quat(task._rot_y(math.radians(tilt_deg))))


def on_bench(spec, y=-0.20):
    return spec.state(pos=(0.25, y, task.TOP + task.PAK[2] / 2))


def scene(tray=0.0, lid=0.0, old="in", spare=None, lot="A"):
    """old/spare: 'in' (loaded in the tray), a stand side (+1/-1), 'bench', or a ready PartState."""
    sp = task.LOTS[lots.index(lot)].spare_side
    spare = sp if spare is None else spare
    out = {task.CONSOLE.name: console(tray, lid)}
    for spec, how in ((task.OLD, old), (task.SPARE, spare)):
        if how == "in":
            out[spec.name] = in_tray(spec, tray)
        elif how == "bench":
            out[spec.name] = on_bench(spec)
        elif isinstance(how, (int, float)):
            out[spec.name] = on_stand(spec, how)
        else:
            out[spec.name] = how
    for n, st in task.STANDS.items():
        out[n] = st.state()
    return out


def near_hand(objects, name, gap=0.003):
    st = objects[name]
    top = np.asarray(st.pos) + np.array([0.0, 0.0, task.PAK[2] / 2 + gap])
    return FAR + [np.array([top])]


def _instances():
    """Every loaded copy of task.py (check.py loads its own), found through the step hooks they registered."""
    seen = {}
    for fn in parts._ON_STEP:
        g = fn.__globals__
        if "observe" in g and "REC" in g:
            seen[id(g["REC"])] = g
    return list(seen.values())


def play(*segments):
    """Reset every copy's trial record, then play (objects, hands, seconds) segments at 20 Hz."""
    t = 0.0
    for g in _instances():
        g["REC"].clear()
        g["REC"].update(g["_fresh"]())
    if not segments:
        return None
    for objects, hands, seconds in segments:
        for _ in range(max(1, round(seconds / DT))):
            t += DT
            for g in _instances():
                g["observe"](objects, hands, t)
    return segments[-1][0]


def swap(lot="A", final=None, spare_in=None, lid=0.0):
    """The intended trial: release, old out onto the empty stand, spare in, caught down, lid shut. spare_in: a
    function of the tray angle giving the loaded spare's state (default: fully home)."""
    sp = task.LOTS[lots.index(lot)].spare_side
    f = spare_in or (lambda q: "in")
    seg = [
        (scene(lid=LID_OPEN, lot=lot), FAR, 0.5),
        (scene(PRESSED, LID_OPEN, lot=lot), FAR, 0.3),  # press: the catch lets go
        (scene(UP, LID_OPEN, lot=lot), FAR, 0.5),  # popped up
        (scene(UP, LID_OPEN, old=-sp, lot=lot), FAR, 0.5),  # old drawn out onto the empty stand
        (scene(UP, LID_OPEN, old=-sp, spare=f(UP), lot=lot), FAR, 0.5),  # spare slid in
        (scene(PRESSED, LID_OPEN, old=-sp, spare=f(PRESSED), lot=lot), FAR, 0.3),  # press: caught
        (scene(DOWN, LID_OPEN, old=-sp, spare=f(DOWN), lot=lot), FAR, 0.5),
    ]
    if final is not None:
        seg.append(final(sp))
    else:
        seg.append((scene(DOWN, lid, old=-sp, spare="in", lot=lot), FAR, 1.5))
    return play(*seg)


class _Timelines:
    """check.py iterates CASES in order; each case's timeline is played just before check.py scores it."""

    def __init__(self, cases):
        self.cases = cases

    def __iter__(self):
        for label, build, want, ok in self.cases:
            lot = "B" if label.startswith("lot B") else "A"
            lots.use(lot)
            yield label, build(), want, ok
        lots.use(None)


S = scene  # short alias for the cases below

SPEC = [
    ("no history: nothing observed yet", lambda: play() or S(), 0.0, False),
    ("do nothing: old game still caught down, lid shut", lambda: play((S(), FAR, 2.0)), 0.0, False),
    ("full: swapped, caught down, lid shut, hands clear, still 1.5 s", lambda: swap(), 1.0, True),
    ("full but still only 0.9 s", lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=-sp, spare="in"), FAR, 0.9)), 0.0, False),
    ("swapped and caught down, lid left open", lambda: swap(lid=LID_OPEN), 0.75, False),
    ("lid resting 2.5 degrees open", lambda: swap(lid=math.radians(2.5)), 0.75, False),
    ("lid exactly 2 degrees open counts as shut", lambda: swap(lid=math.radians(2.0)), 1.0, True),
    ("spare slid in, tray left popped up, lid open",
     lambda: swap(final=lambda sp: (S(UP, LID_OPEN, old=-sp, spare="in"), FAR, 1.5)), 0.5, False),
    ("spare slid in, tray popped up, lid pushed shut on top of it",
     lambda: swap(final=lambda sp: (S(UP, 0.0, old=-sp, spare="in"), FAR, 1.5)), 0.5, False),
    ("spare caught down but 3.5 mm short of the connector",
     lambda: swap(spare_in=lambda q: in_tray(task.SPARE, q, d=(-0.0035, 0, 0)),
                  final=lambda sp: (S(DOWN, 0.0, old=-sp, spare=in_tray(task.SPARE, DOWN, d=(-0.0035, 0, 0))), FAR, 1.5)),
     0.25, False),
    ("spare caught down 3 mm short of the connector still counts as fully in",
     lambda: swap(spare_in=lambda q: in_tray(task.SPARE, q, d=(-0.003, 0, 0)),
                  final=lambda sp: (S(DOWN, 0.0, old=-sp, spare=in_tray(task.SPARE, DOWN, d=(-0.003, 0, 0))), FAR, 1.5)),
     1.0, True),
    ("spare loaded label down (turned over)",
     lambda: swap(spare_in=lambda q: in_tray(task.SPARE, q, turn_deg=180, axis=(1, 0, 0)),
                  final=lambda sp: (S(DOWN, 0.0, old=-sp, spare=in_tray(task.SPARE, DOWN, turn_deg=180, axis=(1, 0, 0))), FAR, 1.5)),
     0.25, False),
    ("spare skewed 6 degrees in the tray, caught down",
     lambda: swap(spare_in=lambda q: in_tray(task.SPARE, q, turn_deg=6),
                  final=lambda sp: (S(DOWN, 0.0, old=-sp, spare=in_tray(task.SPARE, DOWN, turn_deg=6)), FAR, 1.5)),
     0.25, False),
    ("old game put on the stand the spare came from (also empty by then)",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=sp, spare="in"), FAR, 1.5)), 1.0, True),
    ("old game dropped on the bench, spare loaded and caught, lid shut",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old="bench", spare="in"), FAR, 1.5)), 0.0, False),
    ("old game half on its stand (centre 45 mm past the stand's front)",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=on_stand(task.OLD, -sp, dx=-0.0455), spare="in"), FAR, 1.5)), 0.0, False),
    ("old game on its stand, tilted 12 degrees (propped on a finger)",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=on_stand(task.OLD, -sp, tilt_deg=12), spare="in"), FAR, 1.5)),
     0.0, False),
    ("old game on its stand, spare still on its own stand, lid open",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1), FAR, 1.5)), 0.25, False),
    ("pried: old game pulled 8 mm out with the tray still caught down, then everything finished",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5),
                  (S(DOWN, LID_OPEN, old=in_tray(task.OLD, DOWN, d=(-0.008, 0, 0))), FAR, 0.3),
                  (S(DOWN, LID_OPEN), FAR, 0.3), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                  (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5)), 0.0, False),
    ("old game nudged 4 mm before the release (below the 5 mm pry limit), then finished",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5),
                  (S(DOWN, LID_OPEN, old=in_tray(task.OLD, DOWN, d=(-0.004, 0, 0))), FAR, 0.3),
                  (S(DOWN, LID_OPEN), FAR, 0.3), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                  (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5)), 1.0, True),
    ("prised up: popped-up old game levered 20 degrees against the tray inside the chamber, then finished",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=-sp, spare="in"), FAR, 1.5))
     if False else play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                        (S(UP, LID_OPEN, old=in_tray(task.OLD, UP, d=(-0.03, 0, 0.02), turn_deg=-20, axis=(0, 1, 0))), FAR, 0.3),
                        (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                        (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5)), 0.0, False),
    ("bypass: empty tray pressed down and caught, spare then pushed in flat and lid shut",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1), FAR, 0.3),
                  (S(DOWN, LID_OPEN, old=-1), FAR, 0.5), (S(DOWN, LID_OPEN, old=-1, spare="in"), FAR, 0.5),
                  (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5)), 0.0, False),
    ("bypass: carrier freed, held down by hand (not caught), spare slid in flat, then pressed to catch, lid shut",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1), FAR, 0.5), (S(DOWN, LID_OPEN, old=-1), FAR, 0.5),
                  (S(DOWN, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                  (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5)), 0.0, False),
    ("spare lifts 2.5 mm off its rails for a moment during the catching press, then settles: still full",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                  (S(DOWN, LID_OPEN, old=-1, spare=in_tray(task.SPARE, DOWN, d=(0, 0, 0.0025))), FAR, 0.05),
                  (S(DOWN, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5)),
     1.0, True),
    ("caught old game's label end prised up 9 mm (3.9 degrees) and let back down, then everything finished",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5),
                  (S(DOWN, LID_OPEN, old=in_tray(task.OLD, DOWN, d=(0, 0, 0.0045), turn_deg=-3.9, axis=(0, 1, 0))), FAR, 0.3),
                  (S(DOWN, LID_OPEN), FAR, 0.3), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                  (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5)), 0.0, False),
    ("tray never caught: spare in, tray pressed only to 1.5 mm below down (not past the catch)",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(-0.010, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                  (S(UP, 0.0, old=-1, spare="in"), FAR, 1.5)), 0.5, False),
    ("caught, then pressed again: the catch lets go and the tray pops up, lid shut",
     lambda: swap(final=lambda sp: (S(UP, 0.0, old=-sp, spare="in"), FAR, 1.5))
     if False else play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                        (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                        (S(DOWN, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                        (S(UP, 0.0, old=-1, spare="in"), FAR, 1.5)), 0.5, False),
    ("held: all in place but a fingertip 3 mm above the spare's top",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=-sp, spare="in"), near_hand(S(DOWN, 0.0, old=-sp, spare="in"), task.SPARE.name), 2.0)),
     0.0, False),
    ("hand state unknown at the end",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=-sp, spare="in"), None, 0.1)), 0.0, False),
    ("full, then the old game slides 0.6 mm 0.6 s before the end: the hold restarts",
     lambda: play((S(lid=LID_OPEN), FAR, 0.5), (S(PRESSED, LID_OPEN), FAR, 0.3), (S(UP, LID_OPEN), FAR, 0.5),
                  (S(UP, LID_OPEN, old=-1, spare="in"), FAR, 0.5), (S(PRESSED, LID_OPEN, old=-1, spare="in"), FAR, 0.3),
                  (S(DOWN, 0.0, old=-1, spare="in"), FAR, 1.5),
                  (S(DOWN, 0.0, old=on_stand(task.OLD, -1, dx=-0.0006), spare="in"), FAR, 0.6)), 0.0, False),
    ("a pak still moving at 1 cm/s",
     lambda: swap(final=lambda sp: (S(DOWN, 0.0, old=-sp, spare=in_tray(task.SPARE, DOWN, vel=(0.0, 0.0, 0.01))), FAR, 1.5)),
     0.0, False),
    ("both games left in place, spare pushed on top of the console: nothing swapped",
     lambda: play((S(), FAR, 0.5), (S(old="in", spare=task.SPARE.state(pos=(task.X0 + 0.12, 0.0, task.TOP + 0.1))), FAR, 1.5)),
     0.0, False),
    ("lot B: full in lot B's layout", lambda: swap("B"), 1.0, True),
    ("lot B: a replay of lot A: old game laid on the right stand on top of the spare, left stand empty, lid shut",
     lambda: play((S(lid=LID_OPEN, lot="B"), FAR, 0.5), (S(PRESSED, LID_OPEN, lot="B"), FAR, 0.3),
                  (S(UP, LID_OPEN, lot="B"), FAR, 0.5), (S(UP, LID_OPEN, old=on_stand(task.OLD, -1, dz=0.020), lot="B"), FAR, 0.5),
                  (S(UP, 0.0, old=on_stand(task.OLD, -1, dz=0.020), lot="B"), FAR, 1.5)), 0.0, False),
]

CASES = _Timelines(SPEC)


def run():
    bad = 0
    for label, objects, want, ok in CASES:
        got = task.score(objects)
        good = abs(got - want) < 1e-9 and task.success(objects) == ok
        bad += not good
        print("ok  " if good else "FAIL", f"{got:.2f} (want {want:.2f})", label)
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
