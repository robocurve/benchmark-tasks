"""Check that fabrication/params.scad matches task.py, and that the fabrication choices in it are consistent.

    python fabrication/check_params.py      (with yam_arena importable, as for test_scoring.py)

Every assignment in params.scad carries a comment: ``// task.py: <expr>`` (``<expr>`` is evaluated against the
imported task module and must equal the value) or ``// fab: <reason>`` (a fabrication choice, checked below for
consistency with the task's numbers). Exits non-zero on the first failure.
"""

from __future__ import annotations

import importlib
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
TASK_DIR = HERE.parent
PARAMS = HERE / "params.scad"

LINE = re.compile(r"^\s*(\$?\w+)\s*=\s*(.+?);\s*//\s*(task\.py|fab):\s*(.*)$")
SAFE = re.compile(r"^[\w\s\.\+\-\*/\(\)\[\],]+$")
TOL = 1e-6        # mm


def load_task():
    """Import task.py the way test_scoring.py does: its folder on sys.path, then import the module ``task``."""
    if str(TASK_DIR) not in sys.path:
        sys.path.insert(0, str(TASK_DIR))
    return importlib.import_module("task")


def box(boxes, i):
    """Box ``i`` of a task.py box list as [x0, x1, y0, y1, z0, z1] in mm (the order of task.py's ``_mm``)."""
    c, s = np.asarray(boxes[i][0], float), np.asarray(boxes[i][1], float)
    lo, hi = (c - s / 2) * 1000, (c + s / 2) * 1000
    return [float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]), float(lo[2]), float(hi[2])]


def parse(path=PARAMS):
    """[(name, value expression, kind, comment)] for every assignment line; any other non-comment line is an error."""
    out = []
    for n, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("//"):
            continue
        m = LINE.match(line)
        if not m:
            raise AssertionError(f"params.scad:{n}: not a tagged assignment: {raw!r}")
        name, expr, kind, note = m.groups()
        if not SAFE.match(expr):
            raise AssertionError(f"params.scad:{n}: unsupported expression {expr!r}")
        out.append((name, expr, kind, note.strip()))
    return out


def values(entries):
    """Evaluate the assignments in order (OpenSCAD lists become Python lists)."""
    env: dict = {}
    for name, expr, _, _ in entries:
        env[name] = eval(expr, {"__builtins__": {}}, dict(env))  # noqa: S307 - SAFE-checked arithmetic only
    return env


def _close(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return a.shape == b.shape and bool(np.all(np.abs(a - b) < TOL))


def check_task_values(entries, env, T):
    ns = {name: getattr(T, name) for name in dir(T) if not name.startswith("__")}
    ns.update(box=box, rail=T._latch_rail(), np=np)
    n = 0
    for name, _, kind, note in entries:
        if kind != "task.py":
            continue
        want = eval(note, {"__builtins__": {"tuple": tuple, "list": list}}, ns)  # noqa: S307 - our own file
        want = [list(w) if isinstance(w, (tuple, list)) else w for w in want] if isinstance(want, (tuple, list)) \
            else want
        assert _close(env[name], want), f"{name} = {env[name]} in params.scad, but task.py {note} = {want}"
        n += 1
    return n


def spring_rate(wire, od, active, shear_modulus=79_000.0):
    """Rate (N/mm) of a helical compression spring: G d^4 / (8 D^3 n), D the mean diameter, music wire G = 79 GPa."""
    d_mean = od - wire
    return shear_modulus * wire ** 4 / (8 * d_mean ** 3 * active)


def check_fabrication(env, T):
    """Consistency of the fabrication choices with the task's numbers and with each other."""
    e = env
    # pins: the dowel stands PIN_IN + GAP proud of the rail's inner face, as the task's pins do, and is as wide
    assert _close(e["PIN_D"], T.PIN), "dowel pins must be as wide as the task's pins"
    assert _close(e["PIN_L"] - e["RAIL_T"] - e["PIN_BOSS"], e["RAIL_IN"] - (e["HALF_W"] - e["PIN_IN"]))
    assert e["PIN_BOSS"] >= 0
    assert (e["HOLE"] - e["PIN_D"]) / 2 == 0.75, "0.75 mm clearance round each pin, as in the task"
    # guide rods: inside the rail's segments, clear of the pins' pads and of the pull tab
    pad = e["PIN_D"] / 2 + 1.5
    for x in e["ROD_X"]:
        assert any(s[0] <= x - e["BOSS_W"] / 2 and x + e["BOSS_W"] / 2 <= s[1] for s in e["RAIL_SEGS"]), x
        assert all(abs(x - p) >= e["BOSS_W"] / 2 + pad for p in e["PIN_X"]), x
        assert x - e["MOUNT_W"] / 2 > e["TAB_X"][1] + 10, "a mount must leave the tab's fingers 10 mm"
        assert x + e["TONGUE_W"] / 2 <= e["CAR_L"] / 2
    # the rod's blind bore stops short of the rail's inner face, and both bores fit inside the rail and the mount
    blind_end = e["RAIL_OUT"] + e["BOSS_L"] - e["ROD_EMBED"]
    assert blind_end - e["RAIL_IN"] >= 0.8, "leave a wall between the rod bore and the drive"
    assert e["RAIL_Z0"] < e["ROD_Z"] - e["ROD_D"] / 2 - 1 and e["ROD_Z"] + e["ROD_D"] / 2 + 1 < e["RAIL_H"]
    assert e["ROD_Z"] - e["SPRING_POCKET_D"] / 2 > 1 and e["ROD_Z"] + e["SPRING_POCKET_D"] / 2 < e["MOUNT_H"] - 1
    # travel: the boss meets the mount after exactly the task's travel (its joint limit)
    assert _close(e["MOUNT_Y0"] - (e["RAIL_OUT"] + e["BOSS_L"]), T.RAIL_TRAVEL * 1000)
    # rod length: blind end to 1 mm past the mount with the rail closed
    assert _close(blind_end + e["ROD_L"], e["MOUNT_Y1"] + e["ROD_PROUD"])
    # spring: the pair's rate matches the task's within 10 %; it never goes solid; it fits the rod and the pocket
    k = spring_rate(e["SPRING_WIRE"], e["SPRING_OD"], e["SPRING_ACTIVE"])
    assert abs(k * e["N_SPRINGS"] - T.RAIL_RATE / 1000) / (T.RAIL_RATE / 1000) < 0.10, k
    solid = (e["SPRING_ACTIVE"] + 2) * e["SPRING_WIRE"]
    assert solid < e["SPRING_POCKET"] - 1, "the spring must not go solid at full travel"
    assert e["SPRING_OD"] - 2 * e["SPRING_WIRE"] > e["ROD_D"] + 0.5, "spring bore must clear the rod"
    assert e["SPRING_POCKET_D"] - e["SPRING_OD"] >= 0.5
    closed = e["SPRING_POCKET"] + e["RAIL_TRAVEL"]
    assert _close(e["SPRING_FREE"] - closed, e["SPRING_PRELOAD"])
    f_closed = e["N_SPRINGS"] * e["SPRING_RATE"] * e["SPRING_PRELOAD"]
    f_open = e["N_SPRINGS"] * e["SPRING_RATE"] * (e["SPRING_PRELOAD"] + e["RAIL_TRAVEL"])
    sim_open = T.RAIL_RATE * T.RAIL_TRAVEL
    assert f_closed <= 0.15 and abs(f_open - sim_open) <= 0.1, (f_closed, f_open, sim_open)
    # pulling the tab fully open must not drag the loose carrier (friction 0.8 on carrier + rail)
    drag = 0.8 * (T.CAR_MASS + T._latch_rail().mass) * 9.81
    assert f_open < 0.75 * drag, (f_open, drag)
    # fits, floor and stops
    assert e["SLIDE_CLR"] > 0 and e["POCKET_CLR"] > 0
    assert _close(e["BASE_T"] + e["SLIDE_CLR"], e["RAIL_Z0"]), "the rail clears the base web by SLIDE_CLR"
    assert e["BASE_T"] < e["STOP_H"] <= e["RAIL_BAR_H"] and e["STOP_H"] < e["SEAT_Z"]
    assert e["SLAB_H"] < e["SEAT_Z"] and e["SLAB_Y"] < e["HALF_W"] - 2
    assert e["RIB_X"][0][1] <= e["SLAB_X"][0] and e["SLAB_X"][1] <= e["RIB_X"][1][0]
    assert e["SLAB_POCKET"][2] < e["SLAB_H"]
    assert e["SLAB_POCKET"][0] + e["POCKET_CLR"] < e["SLAB_X"][1] - e["SLAB_X"][0] - 2
    assert e["SLAB_POCKET"][1] + e["POCKET_CLR"] < 2 * e["SLAB_Y"] - 2
    assert e["BEZEL_POCKET"][0] + e["POCKET_CLR"] < e["BEZEL_X"][1] - e["BEZEL_X"][0] - 1.2
    assert e["BEZEL_POCKET"][2] + e["POCKET_CLR"] < e["CAR_H"] - 1.5
    assert e["FLOOR_POCKET"][0] + e["POCKET_CLR"] < e["FLOOR_X"][1] - e["FLOOR_X"][0] - 2
    assert e["FLOOR_POCKET"][2] + e["POCKET_CLR"] < e["FLOOR_H"] - 0.5
    # the seated drive's rear end is flush with the carrier's rear edge
    assert _close(e["SEAT_X"] + e["SSD_L"] / 2, e["CAR_L"] / 2)
    return k, solid, f_closed, f_open, drag


def report(env, k, solid, f_closed, f_open, drag):
    steel = 7.85e-3   # g/mm^3
    ballast = sum(np.prod(env[p]) * steel for p in ("SLAB_POCKET", "BEZEL_POCKET", "FLOOR_POCKET"))
    ssd_vol = env["SSD_L"] * env["SSD_W"] * env["SSD_H"] - 4 * env["HOLE"] ** 2 * env["HOLE_DEPTH"]
    print(f"spring: {env['N_SPRINGS']} x {env['SPRING_WIRE']} mm wire, OD {env['SPRING_OD']} mm, "
          f"{env['SPRING_ACTIVE']} active coils -> {k:.3f} N/mm each (pair {env['N_SPRINGS'] * k:.3f}; task "
          f"{env['RAIL_RATE']}), free {env['SPRING_FREE']} mm, solid {solid:.1f} mm")
    print(f"rail force: {f_closed:.2f} N closed, {f_open:.2f} N at {env['RAIL_TRAVEL']} mm "
          f"(task: 0 and {env['RAIL_RATE'] * env['RAIL_TRAVEL']:.2f} N); carrier drag about {drag:.1f} N")
    print(f"guide rods: 2 x {env['ROD_D']} mm x {env['ROD_L']} mm; dowel pins: 4 x {env['PIN_D']} x {env['PIN_L']} mm")
    print(f"steel ballast plates: about {ballast:.0f} g; SSD dummy volume {ssd_vol / 1000:.1f} cm^3 "
          f"(solid PLA about {ssd_vol * 1.24e-3:.0f} g; target {env['SSD_MASS_G']} g)")


def main() -> int:
    T = load_task()
    entries = parse()
    env = values(entries)
    n = check_task_values(entries, env, T)
    k, solid, f_closed, f_open, drag = check_fabrication(env, T)
    print(f"params.scad: {n} values match task.py; {len(entries) - n} fabrication choices consistent")
    report(env, k, solid, f_closed, f_open, drag)
    return 0


if __name__ == "__main__":
    sys.exit(main())
