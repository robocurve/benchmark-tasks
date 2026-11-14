"""Offline checks of the subfloor flatness scorer and the task text, no simulator.

Part 1 builds hand-made states for every tier, cap and failure, and adversarial cases: doing nothing, a report without
measuring, a guess that is measured afterwards, the wedge forced under the straightedge's end, the straightedge rocking
on a high spot (the wedge under its middle), a reading at a shallower strip, the wrong pocket, a hand left on, a tip
that only touches the face, the straightedge outside the lines, and a new trial.

Part 2 checks that the numbers in the text match. The scripted demo is tested end to end in test_demo.py.

    PYTHONPATH=<yam_arena stubs or the real package> python3 test_task.py      (FLATNESS_CASE is set per case here)
"""
import importlib
import math
import os
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import yamkit as kit  # noqa: E402

DOWN = (1.0, 0.0, 0.0, 0.0)  # gripper pointing down (its +z toward the fingertips)
AWAY = {"left_gripper": (0.20, 0.25, 0.16), "right_gripper": (0.20, -0.25, 0.16)}
FAILS = []


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        FAILS.append(what)


def load(case):
    os.environ["FLATNESS_CASE"] = str(case)
    for m in ("task", "demo_loop"):
        sys.modules.pop(m, None)
    import task  # noqa: F401
    return importlib.reload(sys.modules["task"])


def st(pos, quat=(0, 0, 0, 1), vel=(0, 0, 0), half=0.01):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=p - half, high=p + half)


def qz(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def qx(a):
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None


class World:
    """Hand-placed parts. se: straightedge center (x, y) set on the floor, or a full pose; wedge: (tip_x past the near
    edge, y); puck: a pocket name or the start ring."""

    def __init__(self, M):
        self.M = M
        self.se = {"pos": np.array(M.SE_START, float), "quat": (0, 0, 0, 1)}
        self.wedge = {"pos": np.array(M.W_START, float), "quat": (0, 0, 0, 1)}
        self.puck = np.array(M.PUCK_START, float)
        self.grip = dict(AWAY)
        self.open = {"left_gripper": 1.0, "right_gripper": 1.0}

    def set_se(self, x=0.35, y=0.0, lift=0.0, roll=0.0):
        """On its edge on the floor (resting on the highest strips under it), optionally lifted or rolled about x."""
        M = self.M
        z = max(float(M.floor_max(y - M.SE_L / 2, y + M.SE_L / 2)), M.FLOOR) + M.SE_H / 2 + lift
        self.se = {"pos": np.array([x, y, z]), "quat": qx(roll)}

    def set_wedge_under(self, y, extra=0.0, s=None):
        """Wedge at y pushed until its thickness under the near edge fills the gap there (plus `extra` m of length
        forced further, which in this hand-made state is ignored by the floor but lifts nothing), or s given."""
        M = self.M
        edge = self.se["pos"][0] - M.SE_T / 2
        base = M.floor_max(y - M.W_W / 2, y + M.W_W / 2)
        gap = (self.se["pos"][2] - M.SE_H / 2) - base
        if s is None:
            s = max(0.0, (gap - M.W_TIP) / M.W_SLOPE) + extra
        tip = edge + s
        self.wedge = {"pos": np.array([tip - M.TIP_X, y, base]), "quat": (0, 0, 0, 1)}

    def objects(self, held=None):
        M = self.M
        o = {"straightedge": st(self.se["pos"], self.se["quat"], half=0.03),
             "wedge": st(self.wedge["pos"], self.wedge["quat"]),
             "tag_puck": st(self.puck)}
        for g, p in self.grip.items():
            o[g] = st(p, DOWN, half=0.02)
            kit._OPENING[g] = self.open[g]
        return o


def run(M, w, steps=25, mutate=None):
    """Feed `steps` control steps of the same world; returns the last score."""
    s = 0.0
    for i in range(steps):
        if mutate:
            mutate(i)
        s = M.score(w.objects())
    return s


def part1(M, case):
    d, ycm = M.CASES[case]
    y = ycm / 100.0 + 0.01  # a station 1 cm off the core strip's center: still on the 6 cm core
    print(f"-- case {case}: {d} mm at y {ycm} cm ({M.VERDICT})")
    other = "FAIL" if M.VERDICT == "PASS" else "PASS"

    fresh(); w = World(M)
    check(run(M, w) == 0.0, "nothing done = 0")

    fresh(); w = World(M); w.puck = np.array([*M.POCKETS[M.VERDICT], M.TOP + M.BOARD_T])
    check(run(M, w) == 0.0, "report without measuring, nothing else = 0")

    fresh(); w = World(M); w.set_se()
    check(run(M, w) == 0.25, "straightedge set between the lines, both ends down = 0.25")

    fresh(); w = World(M); w.set_se(x=0.31)
    check(run(M, w) == 0.0, "straightedge outside the pencil lines = 0")

    fresh(); w = World(M); w.set_se(roll=math.radians(0.5))
    check(run(M, w) == 0.0, "straightedge rolled 0.5 deg about its length (one end 2.7 mm up) = 0")

    fresh(); w = World(M); w.set_se(); w.set_wedge_under(y)
    check(run(M, w) == 0.75, "wedge in the deepest gap, reading right, no report = 0.75")
    r = M.wedge_reading(w.objects())["reading"] * 1000
    check(abs(r - d) < 0.01, f"reading {r:.2f} mm = trough {d} mm")

    w.puck = np.array([*M.POCKETS[M.VERDICT], M.TOP + M.BOARD_T])
    check(run(M, w) == 1.0, "and the puck in the correct pocket = 1.0")
    check(M.success(w.objects()), "success() true at 1.0")

    fresh(); w = World(M); w.set_se(); w.set_wedge_under(y); w.puck = np.array([*M.POCKETS[other], M.TOP + M.BOARD_T])
    check(run(M, w) == 0.5, "measured right but the wrong pocket: capped at 0.5")

    # a guess: the puck goes in the correct pocket first, the measurement comes after
    fresh(); w = World(M); w.puck = np.array([*M.POCKETS[M.VERDICT], M.TOP + M.BOARD_T])
    run(M, w, steps=5)
    w.set_se(); w.set_wedge_under(y)
    check(run(M, w) == 0.5, "report before measuring, then measured right: guess latched, capped at 0.5")

    # a shallower strip: 3 strips (6 cm) from the core's center the floor is 1.0 mm higher (or flat)
    fresh(); w = World(M); w.set_se(); ys = ycm / 100.0 + (0.07 if ycm < 0 else -0.07)
    w.set_wedge_under(ys); w.puck = np.array([*M.POCKETS[M.VERDICT], M.TOP + M.BOARD_T])
    rr = M.wedge_reading(w.objects())
    sh = run(M, w)
    if rr is not None and rr["under"] >= M.IN_GAP_MIN:
        check(sh == 0.5, f"wedge at a shallower place (reads {rr['reading'] * 1000:.1f} mm): 0.5 even with the right pocket")
    else:
        check(sh == 0.25, "wedge at a flat place cannot get under the edge: 0.25")

    # the wedge forced under one end: the straightedge sits up on it there
    fresh(); w = World(M); w.set_se(); w.se["pos"][2] += 0.0
    w.se["quat"] = qx(math.atan2(0.004, M.SE_L))  # one end 4 mm up on a wedge under it
    w.se["pos"][2] += 0.002
    w.wedge = {"pos": np.array([w.se["pos"][0] - M.SE_T / 2 + 0.035 - M.TIP_X, 0.29, M.FLOOR]), "quat": (0, 0, 0, 1)}
    check(run(M, w) == 0.0, "wedge forced under the straightedge's end (an end 4 mm up) = 0")

    # forced at the next strip out: one end stays down, the other is lifted until the end zone's inner point is
    # 0.49 mm up (inside the 0.5 mm contact allowance); the wedge sits on the strip 0.5 mm shallower and fills the
    # extra gap, so its reading lands within 0.3 mm of the depth. It must not reach 0.75.
    fresh(); w = World(M); w.set_se()
    yc = ycm / 100.0
    ys = yc + (0.03 if yc >= 0 else -0.03)  # half on the core, half on the next strip: rests on the higher one
    piv = -1.0 if ys >= 0 else 1.0  # the end that stays down is the one away from the wedge
    a = math.asin(0.00049 / (M.SE_L - M.END_ZONE))
    ang = a if piv < 0 else -a
    w.se["quat"] = qx(ang)
    w.se["pos"][2] += (M.SE_L / 2) * math.sin(a) + 1e-6
    w.set_wedge_under(ys, s=0.01)
    for _ in range(30):
        rr = M.wedge_reading(w.objects())
        w.wedge["pos"][0] += rr["top_gap"] / M.W_SLOPE
    rr = M.wedge_reading(w.objects())
    check(abs(rr["reading"] * 1000 - d) <= M.READ_TOL_MM and M.straightedge_set(w.objects()),
          f"(setup) forced at the next strip: reads {rr['reading'] * 1000:.2f} mm, straightedge still counts as set")
    check(run(M, w) == 0.5, "wedge forced at the next strip out, an end lifted under 0.5 mm: 0.5, not 0.75")
    w.puck = np.array([*M.POCKETS[M.VERDICT], M.TOP + M.BOARD_T])
    check(run(M, w) == 0.5, "and the puck then in the correct pocket: still 0.5")

    # rocking on a high spot: a block under the middle lifts both ends 1 mm
    fresh(); w = World(M); w.set_se(lift=0.001); w.set_wedge_under(y)
    check(run(M, w) == 0.0, "straightedge up on a high spot, both ends 1 mm off the floor = 0")

    # the tip only touches the face: less than 2 mm under the edge
    fresh(); w = World(M); w.set_se(); w.set_wedge_under(y, s=0.001)
    check(run(M, w) == 0.25, "wedge tip 1 mm under the edge (not seated) = 0.25")

    # wedge loose in front of the straightedge, not under it
    fresh(); w = World(M); w.set_se(); w.set_wedge_under(y, s=-0.02)
    check(run(M, w) == 0.25, "wedge lying in front of the straightedge = 0.25")

    # a hand left on the wedge at the end
    fresh(); w = World(M); w.set_se(); w.set_wedge_under(y); w.puck = np.array([*M.POCKETS[M.VERDICT], M.TOP + M.BOARD_T])
    gx = w.wedge["pos"][0] - 0.025
    w.grip["right_gripper"] = (gx, y, M.FLOOR + 0.002)
    w.open["right_gripper"] = 0.3
    check(bool(M.hands_on(w.objects())), "a gripper closed on the wedge touches it (guardkit)")
    check(run(M, w) == 0.5, "a hand left on the wedge: capped at 0.5, not zeroed")

    # not held for 1 s
    fresh(); w = World(M); w.set_se(); w.set_wedge_under(y); w.puck = np.array([*M.POCKETS[M.VERDICT], M.TOP + M.BOARD_T])
    check(run(M, w, steps=10) == 0.0, "full state for only 0.5 s scores 0 (must hold 1 s)")

    # moving wedge
    fresh(); w = World(M); w.set_se(); w.set_wedge_under(y)
    o = w.objects()
    o["wedge"] = st(w.wedge["pos"], vel=(0.02, 0, 0))
    s = 0
    for _ in range(25):
        s = M.score(o); kit._EPISODE["last"] = None; o = dict(o)
    check(s == 0.25, "wedge still sliding (2 cm/s) is not a reading = 0.25")

    # a new trial resets the record
    fresh(); w = World(M)
    check(run(M, w) == 0.0, "new trial starts at 0")


def part3(M):
    js = (HERE / "task.json").read_text() if (HERE / "task.json").exists() else ""
    rd = (HERE / "README.md").read_text() if (HERE / "README.md").exists() else ""
    for txt, name in ((js, "task.json"), (rd, "README")):
        for needle in ("1.6 mm", "0.3 mm", "0.5 mm", "5 cm", "6 cm", "1:10", "0.50 kg", "43 g", "2,400"):
            check(needle in txt, f"{name} states {needle}")
    import json
    if js:
        check(json.loads(js)["instruction"] == M.INSTRUCTION, "task.json instruction = task.py INSTRUCTION")


def part4():
    """No fixed default board: with FLATNESS_CASE unset (or not a case) each load draws a case at random."""
    seen = set()
    for raw in [None] * 20 + ["0", "x"]:
        if raw is None:
            os.environ.pop("FLATNESS_CASE", None)
        else:
            os.environ["FLATNESS_CASE"] = raw
        sys.modules.pop("task", None)
        import task as T
        seen.add(T.CASE)
        check(T.CASE in T.CASES and (T.DIP_MM, T.DIP_Y_CM) == T.CASES[T.CASE], f"unset/invalid case ({raw}) is a real case")
    check(len(seen) >= 3, f"random draw gives different boards ({sorted(seen)})")
    check(load(5).CASE == 5, "FLATNESS_CASE=5 still fixes the board")


if __name__ == "__main__":
    for c in sorted(load(1).CASES):
        M = load(c)
        part1(M, c)
    part3(load(1))
    part4()
    print("all pass" if not FAILS else f"{len(FAILS)} FAILED")
    sys.exit(1 if FAILS else 0)
