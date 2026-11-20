"""Offline checks of the wall clock task (no simulator): the keyhole geometry against the screw, the scorer's tiers and
its cheat guards, and the demo plan's reach.

    python3 test_task.py
"""
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import yamkit as kit  # noqa: E402


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w < 1e-6:
        raise ValueError
    return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def clock(pos, R=np.eye(3)):
    p = np.asarray(pos, float)
    return {"clock": ObjectState(pos=p, quat_xyzw=quat(R), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - 0.1,
                                 high=p + 0.1)}


HUNG = (M.XW, M.SCREW_Y, M.SCREW_Z - M.SLOT_TOP)


def run(states, links=None):
    """Score a sequence of clock states as one trial (arms far away unless `links` gives a gap)."""
    kit._EPISODE["count"] = kit._EPISODE.get("count", 0) + 1
    M.LINK_POSES.clear()
    M._L = None
    s = 0.0
    orig = M.arm_gap
    if links is not None:
        M.arm_gap = lambda objects: links
    try:
        for i, o in enumerate(states):
            kit._EPISODE["last"] = i
            s = M.score(o)
    finally:
        M.arm_gap = orig
    return s


def geometry():
    hs = M.SCREW_Z - M.TOP
    check(M.SHANK_R < M.SLOT_HW < M.HEAD_R < M.HOLE_R, "shank fits the slot, head only the hole")
    check(M.SHANK_L > M.PLATE_T, "the bare shank is longer than the plate is thick (the plate fits behind the head)")
    check(M.HEAD_H + M.SHANK_L - M.PLATE_T < M.POCKET + M.PLATE_T, "the pocket takes the head")
    check(M.CLOCK_START[2] + M.R < M.SCREW_Z - M.HEAD_R, "the clock starts below the screw (screw not covered)")
    check(abs(hs - (M.HUNG_Z + M.SLOT_TOP)) < 1e-9, "screw height set for the clock to hang at HUNG_Z")
    # the keyhole's opening at the narrow end: is any plate point inside the shank circle?
    pts = np.concatenate([np.asarray(c.hull) + np.asarray(c.center) for c in M._PLATE])
    dn = np.hypot(pts[:, 1], pts[:, 2] - M.SLOT_TOP)
    check(dn.min() >= M.SLOT_HW - 1e-6, "no plate vertex inside the slot's narrow end")
    dh = np.hypot(pts[:, 1], pts[:, 2] - M.HOLE_Z)
    check(dh.min() >= M.HOLE_R * math.cos(math.pi / 12) - 1e-4, "the hole is open (chords at most 0.2 mm in)")


def tiers():
    check(M.in_narrow_end(clock(HUNG)), "hung pose reads as in the narrow end")
    hung = [clock(HUNG)] * (M.HOLD_STEPS + 3)
    check(run(hung) == 1.0, "hung, clear, still 2 s: 1.0")
    check(run(hung, links=0.0) == 0.5, "hung but held: 0.5")
    check(run([clock(HUNG)] * (M.HOLD_STEPS - 2)) == 0.5, "hung under 2 s: 0.5")
    # head in the hole, not dropped (held there)
    hole = (M.XW, M.SCREW_Y, M.SCREW_Z - M.HOLE_Z)
    check(M.head_behind(clock(hole)) and not M.in_narrow_end(clock(hole)), "head in the hole: through, not hung")
    check(run([clock(hole)] * 50, links=0.0) == 0.5, "head through the hole, held: 0.5")
    # cheat (a): back pressed on the head, not through (clock 5 mm off the wall): not through
    out = (M.XW - M.PROUD, M.SCREW_Y, M.SCREW_Z - M.SLOT_TOP)
    check(not M.head_behind(clock(out)), "cheat: clock resting on the head's face is not through")
    check(run([clock(out)] * 50, links=0.0) == 0.25, "covering the screw, held on the wall: 0.25")
    # cheat (b): held in place = arms not clear -> never 1.0 (covered above)
    # start state and doing nothing
    start = clock(M.CLOCK_START)
    check(run([start] * 100) == 0.0, "doing nothing: 0")
    # side miss by 6 mm: not through
    miss = (M.XW, M.SCREW_Y + 0.006, M.SCREW_Z - M.SLOT_TOP)
    check(not M.head_behind(clock(miss)), "6 mm to the side: not through")
    # fell: tipped face down after hanging: 0 latched
    fell = clock((M.XW - 0.1, 0, M.TOP + 0.03), ry(math.radians(60)))
    check(run(hung + [fell] + hung) == 0.0, "fell at any moment: 0 (latched)")
    tipped = clock((M.XW - 0.1, 0, M.TOP + 0.2), ry(math.radians(60)))
    check(run([tipped] * 5, links=0.0) == 0.0 and not M._L.fell, "tilted while held: not a fall")
    check(run([tipped] * 5, links=0.0) == 0.0 and run([tipped] + hung) == 0.0, "tilted with no arm touching: fell")
    # partial: head in the wide hole, released clear and still: still 0.5, never 1.0 (not in the narrow end)
    hole = (M.XW, M.SCREW_Y, M.SCREW_Z - M.HOLE_Z)
    check(run([clock(hole)] * (M.HOLD_STEPS + 5)) == 0.5, "head in the wide end, clear and still: 0.5 (not hung)")
    # miss: dropped back onto the table upright under the screw: 0
    check(run([clock(M.CLOCK_START)] * 50) == 0.0, "dropped back on the table: 0")
    # swinging round the screw (in its own plane) is not a fall
    sw = M.SCREW_Z
    Rr = rx(math.radians(10))
    o = np.array([M.XW, M.SCREW_Y, sw]) - Rr @ np.array([0, 0, M.SLOT_TOP])
    check(M.in_narrow_end(clock(o, Rr)) and M.face_tilt(clock(o, Rr)) < 1, "hung 10 deg off level still hung")
    # standing on the table under the screw, shank-level coincidence: not hung (off-table check)
    low = (M.XW, M.SCREW_Y, M.TOP + M.APOTHEM)
    check(M.low_z(clock(low)) < M.OFF_TABLE, "standing on the table reads as on the table")


def reach():
    import os
    for v in ("", "nothing", "hold", "partial", "miss"):
        os.environ["CLOCK_VARIANT"] = v
        print("variant", repr(v))
        reach_one()
    os.environ.pop("CLOCK_VARIANT")


def reach_one():
    import yam_arena.demo as demo
    plans = M.demo()
    for side, base in (("left", demo.LEFT_BASE_POS), ("right", demo.RIGHT_BASE_POS)):
        plan = demo.ArmPlan(plans[side], base)
        worst = 0.0
        for k in range(int(plan.duration / 0.05)):
            a, _ = plan.action7(k * 0.05)
            pos = plan.target(k * 0.05)[0]
            p, _ = demo.ik.tcp_pose(a[:6], base)
            worst = max(worst, float(np.linalg.norm(p - pos)))
        check(worst < 0.003, f"{side} demo reachable (worst IK miss {worst * 1000:.1f} mm)")


if __name__ == "__main__":
    geometry()
    tiers()
    reach()
    print("all passed")
    sys.exit(0)
