"""Offline checks of the drop-box scorer, layout and demo, no simulator. Each control step is built by hand: the
envelopes' poses as the simulator reports them, the hopper's angle, the hands where given.

Covers: the start scores 0; all three envelopes in the box with the hopper shut and the hands away scores 1.0 and ends
the trial; the hopper left open caps the score at 0.5; envelopes still lying in the open hopper's V, on the box's top,
on the table, held in a hand, moving, poking through the face, under the box's floor or through its top do not count;
an envelope that fell to the table outside the box counts for nothing for the rest of the trial, even if it is then
posted; a hand still on the hopper's pull withholds the 1.0; a new trial starts afresh; the hopper's geometry clears the
box over its whole swing; the springs' torque matches the Lee Spring rating; no pxr or Isaac module is imported by
loading the task or by demo(); the thresholds in the README and task.json match the code.

    python3 test_task.py
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.0, 0.45, M.TOP + 0.30), "right_gripper": (0.0, -0.45, M.TOP + 0.30)}


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def quat(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    v = np.zeros(4)
    v[i] = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k])) / 2
    v[j] = (R[j, i] + R[i, j]) / (4 * v[i])
    v[k] = (R[k, i] + R[i, k]) / (4 * v[i])
    v[3] = (R[k, j] - R[j, k]) / (4 * v[i])
    return v


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


FLAT = ry(math.pi / 2)  # an envelope lying flat (its thickness vertical)


def st(pos, R=np.eye(3), half=0.03, vel=(0.0, 0.0, 0.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat(R), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=p - half, high=p + half)


def box(x, y, z):
    """World point from box-frame coordinates."""
    return np.asarray(M.BOX_POS) + np.array([x, y, z])


def flat_in_box(x, y, k=0):
    """An envelope lying flat on the compartment's floor (stacked k deep), its middle at box-frame (x, y)."""
    return box(x, y, M.IN_Z[0] + M.ENV[0] * (k + 0.5)), FLAT


START = {n: (M.env_start(i)[0], ry(M.LEAN)) for i, n in enumerate(M.ENVS)}
GOOD = {"letter_a": flat_in_box(0.15, 0.0, 0), "letter_b": flat_in_box(0.16, 0.01, 1),
        "letter_c": flat_in_box(0.17, -0.01, 2)}


def frame(envs, hopper_deg=0.0, hands=AWAY, vel=None):
    kit._JOINTS["dropbox/hopper"] = kit.JointState(pos=-math.radians(hopper_deg), vel=0.0, latched=False)
    objs = {"dropbox": st(M.BOX_POS, half=0.3), "rack": st(M.RACK_POS)}
    for n, (p, R) in envs.items():
        objs[n] = st(p, R, vel=(vel or {}).get(n, (0.0, 0.0, 0.0)))
    for g, p in hands.items():
        objs[g] = st(p, np.diag([1.0, -1.0, -1.0]), 0.01)
        kit._OPENING[g] = 1.0
    return objs


def hopper_com():
    """The hopper's centre of mass (link frame), its mass spread over its colliders' volumes."""
    tot, mom = 0.0, np.zeros(3)
    for c in M.HOPPER.colliders:
        if c.hull is None:
            v, cm = float(np.prod(c.size)), np.asarray(c.center, float)
        else:
            H = np.asarray(c.hull, float) + np.asarray(c.center, float)
            o = H.mean(axis=0)
            v, cmv = 0.0, np.zeros(3)
            for f in c.faces:
                a, b, d = H[list(f)]
                t = abs(np.dot(a - o, np.cross(b - o, d - o))) / 6.0
                v += t
                cmv += t * (a + b + d + o) / 4.0
            cm = cmv / v
        tot += v
        mom += v * cm
    return mom / tot


def run(frames):
    M._L = M.Ledger()
    s = ok = None
    for f in frames:
        s, ok = M.score(f), M.success(f)
    return s, ok


def hold(envs, hopper_deg=0.0, n=30, hands=AWAY):
    return [frame(envs, hopper_deg, hands) for _ in range(n)]


def with_(**kw):
    out = dict(GOOD)
    out.update(kw)
    return out


def main():
    # -- the layout and its published sizes
    check(abs(M.BOX_W - 17.5 * 0.0254) < 1e-4, "face 17 1/2 in wide (Florence 4C11S-HOP drawing)")
    check(abs(2 * M.IN_HW - 12 * 0.0254) < 1e-4 and abs(M.IN_D - 15 * 0.0254) < 1e-4,
          "compartment 12 in wide, 15 in deep (drawing note 3)")
    check(abs(M.HOP_IN - 11.375 * 0.0254) < 5e-4, "between the cheeks: the drawing's 11 3/8 in largest bundle")
    check(abs(M.ENV[1] - 9.5 * 0.0254) < 2e-4 and abs(M.ENV[2] - 4.125 * 0.0254) < 2e-4, "#10 envelope 9.5 x 4.125 in")
    check(M.HOP_IN > M.ENV[1] + 0.04, "the hopper is over 4 cm wider than an envelope's length")
    check(abs(M.SPRING_K * math.radians(90) / M.SPRINGS - 0.790668) < 1e-4,
          "each spring gives its rated 790.668 N*mm (6.998 lb-in) at 90 degrees of wind")
    check(M.PRELOAD_DEG + M.OPEN_DEG <= 90.0, "the springs never wind past their rated 90 degrees")
    # the pull at the bar's middle, square to the door, from the springs and the hopper's own weight (its mass spread
    # over its colliders' volumes, as the simulator spreads it): the README's check numbers
    lever = math.hypot(M.BAR_X, M.BAR_Z)
    com = hopper_com()
    for deg, lo, hi in ((0.0, 4.5, 6.0), (M.OPEN_DEG, 6.5, 8.5)):
        a = -math.radians(deg)
        x = com[0] * math.cos(a) + com[2] * math.sin(a)
        net = M.SPRING_K * math.radians(M.PRELOAD_DEG + deg) + x * M.HOP_KG * 9.81
        f = net / lever
        print(f"     pull at the bar's middle, {deg:.0f} deg open: {f:.2f} N (springs + weight {net:.3f} N*m)")
        check(lo <= f <= hi, f"README's pull check at {deg:.0f} degrees: {lo}-{hi} N (got {f:.2f})")

    # -- the start and doing nothing
    s, ok = run(hold(START))
    check(s == 0.0 and not ok, f"start state / doing nothing scores 0 (got {s})")
    s, ok = run(hold(START, hopper_deg=44.0))
    check(s == 0.0, f"hopper held open, nothing posted: 0 (got {s})")

    # -- the full job
    s, ok = run(hold(GOOD))
    check(s == 1.0 and ok, f"all three in, hopper shut, hands away: 1.0 and the trial ends (got {s}, {ok})")
    s, ok = run(hold(GOOD, n=M.HOLD_STEPS - 1))
    check(s == 0.75 and not ok, f"held under 1 s: 0.75, not yet a success (got {s})")
    s, _ = run(hold(GOOD, hopper_deg=10.0))
    check(s == 0.5, f"all three in but the hopper left open 10 degrees: capped at 0.5 (got {s})")
    s, _ = run(hold(GOOD, hopper_deg=M.SHUT_DEG + 0.3))
    check(s == 0.5, f"hopper 1.3 degrees open (jammed): not shut, 0.5 (got {s})")
    s, ok = run(hold(GOOD, hopper_deg=M.SHUT_DEG - 0.3))
    check(s == 1.0, f"hopper within 0.8 degrees: shut (got {s})")
    gap = 0.001 + M.DOOR_H * math.sin(math.radians(M.SHUT_DEG))
    check(gap <= 0.004, f"at the shut limit the door's top edge is within 4 mm of the face ({gap * 1000:.1f} mm)")
    two = with_(letter_c=START["letter_c"])
    s, _ = run(hold(two))
    check(s == 0.5, f"two posted, one left in the rack, hopper shut: 0.5 (got {s})")
    one = with_(letter_b=START["letter_b"], letter_c=START["letter_c"])
    s, _ = run(hold(one))
    check(s == 0.25, f"one posted: 0.25 (got {s})")

    # -- a hand on the hopper's pull at the end: the 1.0 is withheld
    pull_pt = M.bar_point(0.0)  # the hand's tool point on the bar
    s, ok = run(hold(GOOD, hands={"left_gripper": AWAY["left_gripper"], "right_gripper": tuple(pull_pt)}))
    check(s == 0.75 and not ok, f"all in, hopper shut but the right hand still on the pull: 0.75 (got {s})")

    # -- envelopes not in the box
    a = -math.radians(44.0)
    in_v = box(-0.05, 0.0, 0.06), ry(a)  # lying against the open door inside the V
    s, _ = run(hold(with_(letter_a=in_v), hopper_deg=44.0))
    check(s == 0.5, f"one still in the open hopper's V, two in: 0.5 (capped; the V is outside the box) (got {s})")
    s, _ = run(hold(with_(letter_a=in_v), hopper_deg=0.0))
    check(s == 0.5, f"... the same with the hopper read as shut: the V envelope still does not count (got {s})")
    on_top = box(0.2, 0.0, M.TOP_UP + 0.001), FLAT
    s, _ = run(hold(with_(letter_a=on_top)))
    check(s == 0.5, f"one lying on the box's top: does not count (got {s})")
    s, _ = run(hold(with_(letter_a=(box(0.04, 0.0, -0.06), FLAT))))
    check(s == 0.5, f"one lying across the face's back plane (half outside): does not count (got {s})")
    under = box(0.15, 0.0, M.IN_Z[0] - 0.01), FLAT
    s, _ = run(hold(with_(letter_a=under)))
    check(s == 0.5, f"one under the compartment's floor: does not count (got {s})")
    above = box(0.15, 0.0, M.IN_Z[1] + 0.002), FLAT
    s, _ = run(hold(with_(letter_a=above)))
    check(s == 0.5, f"one through the box's top: does not count (got {s})")
    beside = box(0.15, M.IN_Y + 0.01, -0.05), FLAT
    s, _ = run(hold(with_(letter_a=beside)))
    check(s == 0.5, f"one through a side wall: does not count (got {s})")

    # -- held or moving
    s, _ = run(hold(GOOD, hands={"left_gripper": tuple(GOOD["letter_a"][0]), "right_gripper": AWAY["right_gripper"]}))
    check(s < 1.0, f"a hand still on an envelope in the box: no 1.0 (got {s})")
    fr = [frame(GOOD, vel={"letter_a": (0.0, 0.1, 0.0)}) for _ in range(30)]
    s, _ = run(fr)
    check(s == 0.5, f"one still sliding: does not count (got {s})")

    # -- fell outside: counts for nothing for the rest of the trial
    table = (np.array([0.25, 0.10, M.TOP + M.ENV[0] / 2]), FLAT)
    s, _ = run(hold(with_(letter_a=table), n=3) + hold(GOOD))
    check(s == 0.5, f"one fell to the table, then picked up and posted: it still counts for nothing (got {s})")
    floor = (np.array([0.25, 0.10, M.TOP - 0.3]), FLAT)
    s, _ = run(hold(with_(letter_a=floor), n=3) + hold(GOOD))
    check(s == 0.5, f"one fell off the table, then posted: counts for nothing (got {s})")
    rack_again = START
    s, _ = run(hold(rack_again, n=3) + hold(GOOD))
    check(s == 1.0, f"in the rack (on its base, not the table) is not 'fell outside' (got {s})")
    inside_floor = GOOD
    s, _ = run(hold(inside_floor, n=3) + hold(GOOD))
    check(s == 1.0, "the compartment's floor is not 'outside'")

    # -- new trial
    run(hold(with_(letter_a=table), n=3))
    kit._EPISODE["count"] += 1
    s, _ = run(hold(GOOD))
    check(s == 1.0, f"a new trial starts afresh (got {s})")

    # -- geometry over the swing: no hopper collider sample inside a box collider
    def samples(c):
        if c.hull is not None:
            P = np.asarray(c.hull, float)
            out = [P, P.mean(axis=0)[None]]
            for i in range(len(P)):
                for j in range(i + 1, len(P)):
                    for t in (0.25, 0.5, 0.75):
                        out.append((P[i] * t + P[j] * (1 - t))[None])
            return np.vstack(out) + np.asarray(c.center, float)
        g = np.linspace(-0.5, 0.5, 5)
        G = np.array(np.meshgrid(g, g, g)).reshape(3, -1).T * np.asarray(c.size, float)
        return G + np.asarray(c.center, float)

    def inside(P, b, tol=0.0005):
        if b.hull is None:
            return np.all(np.abs(P - np.asarray(b.center)) < np.asarray(b.size) / 2 - tol, axis=1)
        H = np.asarray(b.hull, float) + np.asarray(b.center, float)
        ok = np.ones(len(P), bool)
        c = H.mean(axis=0)
        for f in b.faces:
            a, bb, cc = H[list(f)]
            n = np.cross(bb - a, cc - a)
            n = n / (np.linalg.norm(n) + 1e-12)
            if np.dot(c - a, n) > 0:
                n = -n
            ok &= (P - a) @ n < -tol
        return ok

    hits, worst = 0, []
    for deg in np.arange(0.0, M.OPEN_DEG + 0.1, 1.0):
        R = ry(-math.radians(deg))
        for ci, c in enumerate(M.HOPPER.colliders):
            P = samples(c) @ R.T
            for bj, b in enumerate(M.DROPBOX.colliders):
                n = int(inside(P, b).sum())
                if n:
                    worst.append((deg, ci, bj))
                hits += n
    check(hits == 0, f"the hopper clears the box over its whole swing ({hits} overlaps, first {worst[:3]})")
    # the curved back closes the open hopper: nothing between the floor's far end and the curved back wider than 6 mm
    for deg in (20.0, 30.0, 42.0, M.OPEN_DEG):
        a = -math.radians(deg)
        fx = M._SB[0] * math.cos(a) + M._SB[1] * math.sin(a)
        fz = -M._SB[0] * math.sin(a) + M._SB[1] * math.cos(a)
        psi = math.degrees(math.atan2(fx, fz))
        r = math.hypot(fx, fz)
        check(M.BACK_PSI[0] <= psi <= M.BACK_PSI[1] and M.BACK_R - r < 0.008,
              f"open {deg:.0f} deg: the floor's end lies against the curved back ({(M.BACK_R - r) * 1000:.1f} mm)")
    a = 0.0
    psi_shut = math.degrees(math.atan2(M._SB[0], M._SB[1]))
    check(psi_shut > M.BACK_PSI[1] + 8, f"shut: the floor's end is {psi_shut - M.BACK_PSI[1]:.0f} deg below the curved "
          "back's lower end (letters slide off under it)")
    hull_n = sum(1 for c in M.HOPPER.colliders if c.hull is not None)
    usd = kit._mech_usda(M.DROPBOX) if hasattr(kit, "_mech_usda") else ""
    if usd:
        check(usd.count("convexHull") >= hull_n and "radius = 0.0" not in usd,
              f"every hull collider is written as a convex hull ({usd.count('convexHull')} >= {hull_n})")

    # -- demo: the stack goes in clear of the door
    c = M.cross_point()
    check(c[2] > M.BOX_POS[2], "the posting path starts above the hinge axis")

    # -- no pxr / Isaac at load or in demo()
    M.demo()
    loaded = [m for m in sys.modules if m == "pxr" or m.startswith("pxr.") or m.startswith("isaaclab") or m.startswith("omni")]
    check(not loaded, f"no pxr/Isaac module imported by loading the task or demo() ({loaded[:3]})")

    # -- text matches the code
    for p in (HERE / "README.md", HERE / "task.json"):
        if p.exists():
            t = p.read_text()
            check("0.8 degrees" in t, f"{p.name}: shut within 0.8 degrees")
            check("0.25" in t and "0.5" in t, f"{p.name}: 0.25 per envelope, capped at 0.5 while open")
    tj = HERE / "task.json"
    if tj.exists():
        j = json.loads(tj.read_text())
        check(j["instruction"] == M.INSTRUCTION, "task.json instruction == task.py INSTRUCTION")
        lim = {"title": 120, "instruction": 1000, "context": 1000, "setup": 4000, "scoring": 4000, "fidelity": 500,
               "scores": 300, "reset_notes": 1500}
        for k, n in lim.items():
            check(len(j.get(k, "")) <= n, f"task.json {k} within {n} characters ({len(j.get(k, ''))})")
    print("all passed")


if __name__ == "__main__":
    main()
