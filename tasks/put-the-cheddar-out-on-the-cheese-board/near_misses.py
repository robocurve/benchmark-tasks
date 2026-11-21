"""Near misses and unintended strategies, driven step by step through the task's own whole-trial record.

check.py runs CASES (a do-nothing state scores 0). `python near_misses.py` runs the full list below and exits 1 if
any case does not give its expected score.
"""
import math
import sys

import numpy as np

import lots
import task

CASES = [("nothing done (the record has not started)", {}, 0.0, False)]

TOP, DT, T = task.TOP, 0.05, task.SLAB_T
FAR = {"left": np.array([[0.0, 0.6, 1.3]]), "right": np.array([[0.05, -0.6, 1.3]])}  # both grippers parked well away
ON_SLICER_Z = TOP + task.SL_H + task.BAR_H / 2
UP, FLAT = 0.0, math.pi / 2


def quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def quat_x(roll):
    return (math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2))


def start(lot="A"):
    lots.use(lot)
    cur = lots.current()
    st = {}
    for o in task.OBJECTS:
        if o.name in cur.place:
            st[o.name] = o.state(pos=cur.pos(o.name), quat_xyzw=quat_z(cur.yaw_of(o.name)) if o.name in cur.yaw else None)
        else:
            st[o.name] = o.state()
    return st


def put(st, piece, first_centre, yaw=0.0, quat=None, vel=None):
    """Move a rigid piece (a list of slab numbers): its first slab's centre goes to ``first_centre``."""
    q = quat_z(yaw) if quat is None else quat
    r = task.rot(q)
    out = dict(st)
    for i in piece:
        pos = np.asarray(first_centre, float) + r @ np.array([(i - piece[0]) * T, 0.0, 0.0])
        out[task.SLAB_NAMES[i]] = task.SLABS[i].state(pos=tuple(pos), quat_xyzw=q, lin_vel=vel)
    return out


def arm(st, q):
    out = dict(st)
    out[task.SLICER.name] = task.SLICER.state(joints={"arm": q})
    return out


def on_slicer(st, cut_at=0.060, piece=None, turned=False):
    """The bar (or a piece) laid across the slot, the wire's plane ``cut_at`` from the mouldy end. ``turned``: the
    mouldy end is on the far side (as lot B lays it)."""
    piece = list(range(task.N_SLAB)) if piece is None else piece
    if turned:
        x0 = task.X_SLOT + cut_at - (piece[0] + 0.5) * T
        return put(st, piece, (x0, 0.0, ON_SLICER_Z), yaw=math.pi)
    x0 = task.X_SLOT - cut_at + (piece[0] + 0.5) * T
    return put(st, piece, (x0, 0.0, ON_SLICER_Z))


def swing(st, q0=UP, q1=FLAT, n=60):
    return [arm(st, q0 + (q1 - q0) * k / n) for k in range(n + 1)]


def on_board(st, piece, dz=0.0, dx=0.0, dy=0.0, lot="A"):
    b = st[task.BOARD.name].pos
    x = b[0] + dx - (len(piece) - 1) * T / 2
    return put(st, piece, (x, b[1] + dy, b[2] + task.BOARD_H + task.BAR_H / 2 + dz))


def in_tub(st, piece, dz=0.0):
    t = st[task.TUB.name].pos
    x = t[0] - (len(piece) - 1) * T / 2
    return put(st, piece, (x, t[1], t[2] + task.TUB_WALL + task.BAR_H / 2 + dz))


def trial(steps, hold_s=1.2, clouds=FAR, lot="A"):
    """Start the record at the lot's layout, play ``steps`` (states, or (state, clouds) pairs), hold the last."""
    first = start(lot)
    task._arm_record(first)
    t = 0.0
    seq = [s if isinstance(s, tuple) else (s, clouds) for s in steps]
    seq = seq + [seq[-1]] * int(round(hold_s / DT))
    for st, cl in seq:
        t += DT
        task._update(st, cl, t)
    return task.score(seq[-1][0])


NEAR, CLEAN = list(range(0, 12)), list(range(12, 24))


def cut_sequence(lot="A", cut_at=0.060):
    """Lay the bar across the slot and swing the wire down through it; returns the states."""
    st = on_slicer(start(lot), cut_at, turned=(lot == "B"))
    return swing(st)


def near_point(st, name_or_slab, gap, axis=2):
    """One gripper point ``gap`` above (axis 2) or beside a slab."""
    s = st[task.SLAB_NAMES[name_or_slab]]
    p = np.array(s.pos, float)
    half = (T / 2, task.BAR_W / 2, task.BAR_H / 2)[axis]
    p[axis] += half + gap
    return p


def run():
    results = []

    def want(label, got, expect):
        results.append((label, abs(got - expect) < 1e-9, got, expect))

    # -- do nothing, and the naive errand ------------------------------------------------------------------------
    s0 = start("A")
    want("nothing moved", trial([s0]), 0.0)
    whole = list(range(24))
    want("naive: the whole bar put on the cheese board", trial([on_board(s0, whole)]), 0.0)
    want("naive, patch turned face down on the cheese board",
         trial([put(s0, whole, (s0[task.BOARD.name].pos[0] - 0.06, s0[task.BOARD.name].pos[1],
                                s0[task.BOARD.name].pos[2] + task.BOARD_H + task.BAR_H / 2), quat=quat_x(math.pi))]), 0.0)
    want("whole bar carried over the cheese board 20 cm up, then binned",
         trial([on_board(s0, whole, dz=0.20), in_tub(s0, whole)]), 0.0)
    want("whole bar binned without passing over the cheese board", trial([in_tub(s0, whole)]), 0.25)

    # -- the intended sequence and its levels ----------------------------------------------------------------------
    seq = cut_sequence("A")
    after = seq[-1]
    served = on_board(after, CLEAN)
    done = in_tub(served, NEAR)
    want("cut 60 mm from the mouldy end, clean half on the board, offcut in the tub", trial(seq + [done]), 1.0)
    want("the same, but still for only 0.95 s: only the cut, made over 1 s ago, counts", trial(seq + [done], hold_s=0.95), 0.25)
    want("clean half on the board, offcut left on the slicer", trial(seq + [served]), 0.75)
    want("offcut in the tub, clean half left on the slicer", trial(seq + [in_tub(after, NEAR)]), 0.5)
    want("cut made, nothing moved", trial(seq + [after]), 0.25)
    want("cut made, both halves binned", trial(seq + [in_tub(in_tub(after, NEAR), CLEAN, dz=0.032)]), 0.25)
    want("wire brought half way down and raised again: no cut", trial(swing(seq[0], UP, math.radians(60)) + [seq[0]]), 0.0)
    want("served half 5 mm above the board (held, not put down)", trial(seq + [in_tub(on_board(after, CLEAN, dz=0.005), NEAR)]), 0.5)
    want("served half 2.9 mm above the board", trial(seq + [in_tub(on_board(after, CLEAN, dz=0.0029), NEAR)]), 1.0)
    want("served half hanging off the board's edge (a slab's centre outside)",
         trial(seq + [in_tub(on_board(after, CLEAN, dx=0.02 + task.BOARD_X / 2), NEAR)]), 0.5)
    want("offcut held 80 mm up over the tub, not in it", trial(seq + [in_tub(served, NEAR, dz=0.08)]), 0.75)
    moving = put(done, NEAR, done[task.SLAB_NAMES[0]].pos, vel=(0.0, 0.0, 0.02))
    want("offcut still moving in the tub", trial(seq + [moving]), 0.0)

    # -- the 25 mm margin, on both sides ---------------------------------------------------------------------------
    for cut_at, first, expect in ((0.055, 11, 1.0), (0.050, 10, 0.0), (0.040, 8, 0.0)):
        sq = cut_sequence("A", cut_at)
        a = sq[-1]
        far, near = list(range(first, 24)), list(range(0, first))
        want(f"cut {cut_at * 1000:.0f} mm from the mouldy end ({cut_at * 1000 - 27.5:.1f} mm from the patch), far piece served",
             trial(sq + [in_tub(on_board(a, far), near)]), expect)
    sq = cut_sequence("A", 0.020)
    want("wire brought down through the patch", trial(sq + [in_tub(on_board(sq[-1], list(range(4, 24))), list(range(0, 4)))]), 0.0)
    sq = swing(on_slicer(s0, 0.020), UP, math.radians(50))
    want("wire lowered onto the patch and raised again, then a good cut and delivery",
         trial(sq + [on_slicer(s0, 0.060)] + cut_sequence("A") + [done]), 0.0)
    # two cuts: 40 mm (too close), then 60 mm; only the last piece is served
    sq1 = cut_sequence("A", 0.040)
    a1 = sq1[-1]
    a1 = arm(a1, UP)
    moved = on_slicer(put(a1, list(range(0, 8)), (0.30, 0.15, TOP + task.BAR_H / 2)), 0.060, piece=list(range(8, 24)))
    sq2 = swing(moved)
    fin = in_tub(in_tub(on_board(sq2[-1], CLEAN), list(range(0, 8))), list(range(8, 12)), dz=0.032)
    want("second cut 60 mm from the mouldy end after a first one at 40 mm; last piece served, the rest binned",
         trial(sq1 + [a1, moved] + sq2 + [fin]), 1.0)

    # -- cuts that must not count ----------------------------------------------------------------------------------
    sq = cut_sequence("A", 0.0527)
    want("wire 52.7 mm from the mouldy end (25.2 mm from the patch), far piece served",
         trial(sq + [in_tub(on_board(sq[-1], list(range(10, 24))), list(range(0, 10)))]), 0.0)
    sq = cut_sequence("A", 0.0530)
    want("wire 53.0 mm from the mouldy end (25.5 mm from the patch), far piece served",
         trial(sq + [in_tub(on_board(sq[-1], list(range(11, 24))), list(range(0, 11)))]), 1.0)
    # the bar pushed down through the standing wire, the arm never moved
    standing = arm(s0, math.radians(70))
    drop = [put(standing, whole, (task.X_SLOT - 0.0575, 0.0, ON_SLICER_Z + 0.10 * (1 - k / 40))) for k in range(41)]
    trial(drop)
    want("bar pushed down through the standing wire: joins that let go", float(len(task._rec["cuts"])), 0.0)
    apart = in_tub(on_board(drop[-1], CLEAN), NEAR)
    want("bar pushed down through the standing wire, then the halves set out as if cut", trial(drop + [apart]), 0.0)
    want("bar pushed through the standing wire, then the arm brought down the rest of the way",
         float(len(task._rec["cuts"])) if trial(drop + swing(drop[-1], math.radians(70), FLAT)) == 0.0 else 9.0, 0.0)
    slide = [put(arm(s0, math.radians(75)), whole, (task.X_SLOT - 0.0575, -0.15 + 0.15 * k / 40, ON_SLICER_Z)) for k in range(41)]
    trial(slide)
    want("bar slid sideways through the half-lowered wire: joins that let go", float(len(task._rec["cuts"])), 0.0)
    under = arm(on_slicer(s0), FLAT)
    trial([under] + swing(under, FLAT, UP))
    want("bar laid over the lowered wire and the arm raised up through it: joins that let go", float(len(task._rec["cuts"])), 0.0)
    # the bar lying askew across the slot

    def askew(deg, wire_at=0.060):
        a = math.radians(deg)
        st = put(s0, whole, (task.X_SLOT - (wire_at - T / 2) * math.cos(a), -(wire_at - T / 2) * math.sin(a), ON_SLICER_Z), yaw=a)
        return swing(st)

    trial(askew(30))
    want("bar lying 30 degrees askew across the slot, arm swung down: joins that let go", float(len(task._rec["cuts"])), 0.0)
    trial(askew(12))
    want("bar 12 degrees askew: joins that let go", float(len(task._rec["cuts"])), 0.0)
    sq = askew(8)
    trial(sq)
    want("bar 8 degrees askew, wire crossing its middle 60 mm from the mouldy end: the join that lets go (mm)",
         1000 * T * float(min(task._rec["cuts"] or {0})), 55.0)
    want("bar 8 degrees askew, cut 55 mm from the mouldy end at its nearest edge, far piece served",
         trial(sq + [in_tub(on_board(sq[-1], list(range(11, 24))), list(range(0, 11)))]), 1.0)
    sq = askew(9, 0.057)
    want("bar 9 degrees askew, the wire's nearest edge 52.6 mm from the mouldy end, far piece served",
         trial(sq + [in_tub(on_board(sq[-1], list(range(10, 24))), list(range(0, 10)))]), 0.0)
    want("clean half resting on the crackers (4 mm above the board), offcut binned",
         trial(seq + [in_tub(on_board(after, CLEAN, dz=0.004, dy=0.125), NEAR)]), 0.5)
    # the arm jiggled up and down in place while the bar is pushed down through the wire
    jig = [put(arm(s0, math.radians(70) + (0.06 if k % 2 else 0.0)), whole,
               (task.X_SLOT - 0.0575, 0.0, ON_SLICER_Z + 0.10 - 0.20 * k / 160)) for k in range(161)]
    trial(jig, hold_s=0.1)
    want("arm jiggled 3.4 degrees up and down in place while the bar is pushed down through the wire: joins that let go",
         float(len(task._rec["cuts"])), 0.0)
    # the bar 25 degrees askew, the wire passing just beyond its mouldy end and through its corner (no cut is possible)
    for sgn in (1, -1):
        ya = math.radians(25 * sgn)
        st = put(s0, whole, (task.X_SLOT - (-0.005 - T / 2) * math.cos(ya), -(-0.005 - T / 2) * math.sin(ya), ON_SLICER_Z), yaw=ya)
        want(f"bar {25 * sgn} degrees askew, the wire swung through its mouldy corner, then a perfect finish",
             trial(swing(st) + seq + [done]), 0.0)

    # -- grippers ------------------------------------------------------------------------------------------------------
    def hands(st, slab, gap, axis=2, arm_name="right"):
        c = dict(FAR)
        c[arm_name] = np.vstack([FAR[arm_name], near_point(st, slab, gap, axis)])
        return c

    want("gripper 4 mm above the served half at the end", trial(seq + [done], clouds=hands(done, 18, 0.004)), 0.0)
    want("a gripper face 0.5 mm from the served half", trial(seq + [done], clouds=hands(done, 18, 0.0005)), 0.0)
    want("gripper 6 mm above the served half", trial(seq + [done], clouds=hands(done, 18, 0.006)), 1.0)
    want("gripper 4 mm from the offcut in the tub", trial(seq + [done], clouds=hands(done, 6, 0.004)), 0.0)
    want("gripper state unknown", trial(seq + [done], clouds=None), 0.0)
    touch_patch = hands(s0, 3, 0.0005)
    want("a gripper touches the mould patch, then a perfect run", trial([(s0, touch_patch)] + seq + [done]), 0.0)
    want("a gripper grips the bar's side beside the patch (not the patch), then a perfect run",
         trial([(s0, hands(s0, 3, 0.0, axis=1))] + seq + [done]), 1.0)
    dirty = hands(after, 6, 0.0, axis=1)
    then_clean = hands(after, 18, 0.0, axis=1)
    want("one gripper holds the offcut, then touches the half that is served",
         trial(seq + [(after, dirty), (after, then_clean), done]), 0.0)
    # a gripper touches the mouldy piece beside the patch and then, before a second cut, clean cheese of that same piece
    sq1 = cut_sequence("A", 0.080)
    up1 = arm(sq1[-1], UP)
    moved = on_slicer(put(up1, list(range(16, 24)), (0.30, 0.15, task.TOP + task.BAR_H / 2)), 0.060, piece=list(range(0, 16)))
    sq2 = swing(moved)
    fin = in_tub(in_tub(on_board(sq2[-1], list(range(12, 16))), list(range(0, 12))), list(range(16, 24)), dz=0.032)
    want("cut at 80 mm; one gripper touches slab 3 then slab 13 of the mouldy piece; cut at 60 mm; slabs 12-15 served",
         trial(sq1 + [up1, (up1, hands(up1, 3, 0.0, axis=1)), (up1, hands(up1, 13, 0.0, axis=1)), moved] + sq2 + [fin]), 0.0)
    want("one gripper holds the served half first, then the offcut",
         trial(seq + [(after, then_clean), (after, dirty), done]), 1.0)
    want("left gripper holds the offcut, right gripper the served half",
         trial(seq + [(after, hands(after, 6, 0.0, axis=1, arm_name="left")), (after, then_clean), done]), 1.0)

    # -- other wrong ends ----------------------------------------------------------------------------------------------
    upside = put(s0, whole, (task.X_SLOT - 0.0575, 0.0, ON_SLICER_Z), quat=quat_x(math.pi))
    want("bar laid patch-down on the slicer board", trial([upside] + seq + [done]), 0.0)
    off = put(done, NEAR, (0.9, 0.0, TOP - 0.2))
    want("offcut dropped off the table", trial(seq + [off]), 0.0)
    want("served half on the cheese board, offcut put on the board too", trial(seq + [on_board(served, NEAR, dy=0.08)]), 0.0)

    # -- lot B: the patch is at the other end; the board and the tub change places --------------------------------
    b0 = start("B")
    seqb = cut_sequence("B")
    afterb = seqb[-1]
    good = in_tub(on_board(afterb, CLEAN), NEAR)
    want("lot B: near half (no patch) on the board, far half in the tub", trial(seqb + [good], lot="B"), 1.0)
    # the habit that scores 1.0 in lot A: the far half goes on the board, the near half in the tub
    far_first = afterb[task.SLAB_NAMES[0]].pos  # in lot B the far half is the mouldy one
    habit = in_tub(on_board(afterb, NEAR), CLEAN)
    want("lot B: lot A's habit (far half served, near half binned)", trial(seqb + [habit], lot="B"), 0.0)
    want("lot B: nothing moved", trial([b0], lot="B"), 0.0)
    lots.use(None)

    bad = 0
    for label, ok, got, expect in results:
        print(("ok   " if ok else "FAIL ") + f"{label}: {got} (expected {expect})")
        bad += not ok
    print(f"\n{len(results) - bad} of {len(results)} cases as expected")
    return bad


if __name__ == "__main__":
    sys.exit(1 if run() else 0)
