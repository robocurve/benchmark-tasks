"""Offline tests of score(): hand-built end states, no simulator or GPU.

Run from this folder with Isaac Lab-Arena's environment sourced (task.py imports yam_arena):
    python test_scoring.py
Each line prints the state, the score and the score wanted; the last line is ALL OK, and the exit code is 0, only if
every case matches. The scripted runs in the README check the same rules in the simulator.
"""
import copy
import importlib.util
import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np

_spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).resolve().parent / "task.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)


# -- building states -------------------------------------------------------------------------------------------------
def straight(cab, start_plug_len=None, end_plug_len=None):
    """A cable as set up: its segment centres, and its end bodies' centres (None where it has none)."""
    pts = cab.start_points()
    d = cab.direction
    sp = None if start_plug_len is None else cab.point_at(0) - d * start_plug_len / 2
    ep = None if end_plug_len is None else cab.point_at(cab.length) + d * end_plug_len / 2
    return SimpleNamespace(points=pts, start_plug=sp, end_plug=ep)


def start_world():
    return {T.INTACT.name: straight(T.INTACT, None, T.CON[0]),
            T.DAMAGED.name: straight(T.DAMAGED, T.OPLG[0], T.CON[0]),
            T.SAWCORD.name: straight(T.SAWCORD, None, T.PLG[0])}


def put_end(st, centre, axis, length):
    """Move a cable's end body so its centre is ``centre`` and its axis out of the face ``axis``."""
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    st.end_plug = np.asarray(centre, float)
    st.points = st.points.copy()
    st.points[-1] = st.end_plug - axis * (length / 2 + 0.012)


def put_start(st, centre, axis, length):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    st.start_plug = np.asarray(centre, float)
    st.points = st.points.copy()
    st.points[0] = st.start_plug - axis * (length / 2 + 0.012)


def saw_into(w, cord, gap):
    """The saw's plug face to face with ``cord``'s connector, lined up, at face gap ``gap`` (negative: blades in)."""
    c, a = T.end_pose(w[cord])
    face = c + a * T.CON[0] / 2
    put_end(w[T.SAWCORD.name], face + a * (gap + T.PLG[0] / 2), -a, T.PLG[0])


def dplug_out(w, dist):
    """The damaged cord's plug pulled straight out of its outlet: its centre ``dist`` from the outlet face."""
    f = T.outlet2_face(T.DAM_OUTLET)
    put_start(w[T.DAMAGED.name], f + T.DAM_DIR * dist, -T.DAM_DIR, T.OPLG[0])


def lay(w, x0, y0, deg):
    """The whole damaged cord laid straight from its plug at (x0, y0) towards ``deg``."""
    st = w[T.DAMAGED.name]
    d = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])
    plug_c = np.array([x0, y0, T.TOP + T.EXT_R])
    seg = T.DAMAGED.seg_len
    first = plug_c + d * (T.OPLG[0] / 2 + seg / 2)
    st.points = np.array([first + d * seg * k for k in range(len(st.points))])
    st.start_plug = plug_c
    st.end_plug = st.points[-1] + d * (seg / 2 + T.CON[0] / 2)


def aside(w):
    """The damaged cord set aside where the demo leaves it: along the bench's front right edge."""
    lay(w, 0.10, -0.44, 0.0)


def seat_at(w, centre, deg):
    """The intact connector moved to ``centre`` facing ``deg``, and the saw's plug seated in it."""
    a = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])
    put_end(w[T.INTACT.name], np.asarray(centre, float), a, T.CON[0])
    saw_into(w, T.INTACT.name, 0.001)


MATING_SPOT = (0.33, 0.06)  # (x, y) where the demo makes the connection


def mated_at_spot(w, dz=0.07):
    seat_at(w, (MATING_SPOT[0], MATING_SPOT[1], T.TOP + dz), 240.0)


# -- one case --------------------------------------------------------------------------------------------------------
def case(name, w, want, latch=None, monitor_err="", contact_err=False):
    """Score ``w`` with the given latches and the contact model's and touch monitor's state."""
    T.LATCH.update(damaged=False, split=False, cord=False, what="")
    if latch:
        T.LATCH.update(latch)
    T.TOUCH["err"] = monitor_err
    T.LAST_FORCE.pop("warned", None)
    if contact_err:
        T.LAST_FORCE["warned"] = True
    got = T.score(w)
    st = T.status(w)
    ok = abs(got - want) < 1e-9
    print(f"{'ok ' if ok else 'BAD'} {name:70s} score {got:.2f} (want {want:.2f}) | damaged in={st['damaged_in']} "
          f"away {st['away'] * 100:4.1f} cm clear {st['clear'] * 100:5.1f} cm")
    T.TOUCH["err"] = ""
    T.LAST_FORCE.pop("warned", None)
    return ok


def main() -> int:
    w0 = start_world()
    new = lambda: copy.deepcopy(w0)  # noqa: E731
    results = []

    w = new()
    results.append(case("start: damaged cord plugged in, saw free", w, 0.0))
    w = new(); saw_into(w, T.INTACT.name, 0.001)
    results.append(case("saw seated in the intact cord, damaged cord still plugged in", w, 0.0))
    w = new(); aside(w); saw_into(w, T.INTACT.name, 0.001)
    results.append(case("FULL: damaged cord set aside, saw seated", w, 1.0))
    w = new(); aside(w); mated_at_spot(w)
    results.append(case("FULL at the demo's mating spot", w, 1.0))
    results.append(case("  same, split touched while live earlier (latched)", w, 0.0, latch={"split": True}))
    results.append(case("  same, plug pulled out by its cord earlier (latched)", w, 0.0, latch={"cord": True}))
    results.append(case("  same, saw plug went into the damaged connector earlier (latched)", w, 0.0,
                        latch={"damaged": True}))
    results.append(case("  same, touch monitor failed to start", w, 0.0, monitor_err="RuntimeError: test"))
    results.append(case("  same, contact model failed", w, 0.0, contact_err=True))
    w = new(); aside(w); mated_at_spot(w, dz=0.30)
    results.append(case("FULL, pair lifted 30 cm (heights don't matter)", w, 1.0))
    w = new(); aside(w); saw_into(w, T.INTACT.name, 0.006)
    results.append(case("set aside, saw plug part-way in (6 mm gap, blades live)", w, 0.0))
    w = new(); aside(w)
    results.append(case("set aside, saw not connected", w, 0.5))
    w = new(); aside(w); saw_into(w, T.DAMAGED.name, 0.001)
    results.append(case("set aside, saw seated in the damaged cord's connector", w, 0.0))
    w = new(); aside(w)
    f = T.outlet2_face(1 - T.DAM_OUTLET)
    put_end(w[T.SAWCORD.name], f + T.DAM_DIR * (0.0005 + T.PLG[0] / 2), -T.DAM_DIR, T.PLG[0])
    results.append(case("set aside, saw plugged straight into the bench outlet's free outlet", w, 0.5))
    w = new(); dplug_out(w, 0.20); mated_at_spot(w)
    results.append(case("connected, damaged cord unplugged 20 cm but left across the bench", w, 0.5))
    w = new(); dplug_out(w, 0.20); mated_at_spot(w, dz=0.30)
    results.append(case("  same, pair lifted 30 cm (no way round the set-aside rule)", w, 0.5))
    w = new(); dplug_out(w, 0.20); saw_into(w, T.INTACT.name, 0.001)
    results.append(case("connected where the intact connector starts, damaged cord across", w, 0.5))
    w = new(); dplug_out(w, 0.20)
    results.append(case("damaged cord unplugged 20 cm, across the bench, saw free", w, 0.0))
    w = new(); dplug_out(w, 0.035 + 0.008); saw_into(w, T.INTACT.name, 0.001)
    results.append(case("damaged cord's plug half out (8 mm, blades live), saw seated", w, 0.0))
    w = new(); lay(w, 0.10, -0.44, 75.0); mated_at_spot(w)
    results.append(case("damaged cord swung round to point its connector at the work", w, 0.5))
    w = new(); lay(w, 0.30, -0.60, 90.0); saw_into(w, T.INTACT.name, 0.001)
    results.append(case("damaged plug 2 cm from its outlet, cord across the bench", w, 0.5))
    w = new(); lay(w, 0.12, 0.30, 180.0); mated_at_spot(w)
    results.append(case("damaged cord moved to the front left instead (any direction)", w, 1.0))

    good = all(results)
    print(f"{sum(results)}/{len(results)} cases;", "ALL OK" if good else "SOME BAD")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
