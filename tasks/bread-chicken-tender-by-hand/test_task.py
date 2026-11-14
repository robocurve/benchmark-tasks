"""Offline checks of the breading scorer and demo, no simulator. The coat is driven the way the simulator hook drives
it (coatkit.Coat.physics with the pan's force, the pressers and the grips), and the scorer is fed hand-built object
states. Covers every tier and the adversarial cases: doing nothing, dropping it into the breading and never pressing,
pressing without a hand (the pan alone), pressing only one face, regripping after the coat at a fresh spot, many
grips, the tender dropped on the table on the way (latched 0), coated but left in the breading pan, on the rack but
moving or for less than 1 s, a hand left on it at the end (capped at 0.5), a new trial, the demo's coat sequence, the
demo plan, and the numbers in task.json and the README.

    python3 test_task.py
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import coatkit as ck  # noqa: E402
import yamkit as kit  # noqa: E402

X0 = M.PAN_X
HALF = M.TENDER_SIZE[2] / 2
IN_MARINADE = np.array([X0, M.Y_MARINADE, M.FLOOR + HALF])
IN_BED = np.array([X0, M.Y_BREADING, M.FLOOR + HALF])
ON_RACK = np.array([X0, M.Y_RACK, M.GRATE_TOP + HALF])
ON_TABLE = np.array([X0, 0.32, M.TOP + HALF])
AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.30, M.TOP + 0.30)}
FLAT = np.eye(3)


def rx(a):
    return kit._rot_xyz(a, 0.0)


def grip_rot(yaw):
    """The arena's straight-down grip at this yaw (yaw 0 closes the fingers across world y)."""
    return np.asarray(grip_rotation(yaw), float)


FLIP = rx(math.pi)
HALF_OPEN = M.TENDER_SIZE[1] / 2 - 0.0003  # pads squeezing the 3.5 cm width


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=FLAT, v=(0.0, 0.0, 0.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=ck.quat(np.asarray(rot)), lin_vel=np.asarray(v, float), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M._L = M.Ledger()
    ck.reset()


def objs(tpos, trot=FLAT, held_by=None, v=(0.0, 0.0, 0.0)):
    """One control step's objects: the tender at tpos; a gripper closed on its middle if held_by names one."""
    o = {}
    for key, (_, origin) in zip(M.PART_KEYS, M.PARTS):  # the three parts, lying straight
        o[key] = st(np.asarray(tpos, float) + np.asarray(trot) @ np.asarray(origin), trot, v)
    for g, p in AWAY.items():
        o[g] = st(p, grip_rot(M.ACROSS))
        kit._OPENING[g] = 1.0
    if held_by:
        o[held_by] = st(np.asarray(tpos) + np.array([M.NEAR_GRIP, 0.0, M.CENTER_TO_TCP]), grip_rot(M.ACROSS))
        kit._OPENING[held_by] = HALF_OPEN / 0.0475
    return o


def run(tpos, trot=FLAT, n=1, **k):
    s = None
    for _ in range(n):
        s = M.score(objs(tpos, trot, **k))
    return s


# -- driving the coat as the physics hook does ---------------------------------------------------------------------
def coat():
    return ck.state()


def press(pos, rot, stations=M.STATIONS, force=3.0):
    q = ck.quat(rot)
    for s in stations:
        tcp = np.asarray(pos) + np.array([s, 0.0, 0.02])
        for _ in range(M.PRESS_STEPS + 2):
            coat().physics(pos, q, force, [tcp], [])
        coat().physics(pos, q, 0.49, [], [])


def grip(pos, rot, x, yaw=M.ACROSS, roll=0.0, tcp=None, steps=3):
    g_rot = rx(roll) @ grip_rot(yaw)
    t = np.asarray(pos) + np.array([x, 0.0, M.CENTER_TO_TCP]) if tcp is None else np.asarray(tcp)
    g = ck.Grip(tuple(t), g_rot, HALF_OPEN)
    for _ in range(steps):
        coat().physics(pos, ck.quat(rot), 0.0, [], [g])


def demo_coat():
    """The demo's contacts in order: three presses, the right's near-end grip, the turn (the left takes the far
    half while the right still holds the near end, both rolled), three presses on the other face, the right's
    final grip at the same near end."""
    press(IN_BED, FLAT)
    grip(IN_BED, FLAT, M.NEAR_GRIP)
    tc = np.array([X0, 0.008, M.FLIP_Z])  # rolled onto its edge, 8 mm to +y of the right hand's tool point
    grip(tc, rx(math.pi / 2), 0.0, tcp=(X0 + M.NEAR_GRIP, 0.0, M.FLIP_Z), roll=math.pi / 2)
    grip(tc, rx(math.pi / 2), 0.0, yaw=M.ACROSS_L, tcp=(X0 + M.FAR_GRIP, M.HAND_Y, M.FLIP_Z), roll=-math.pi / 2)
    press(IN_BED, FLIP)
    grip(IN_BED, FLIP, M.NEAR_GRIP)


# -- tests -------------------------------------------------------------------------------------------------------------
def test_nothing():
    fresh()
    check(run(IN_MARINADE, n=60) == 0.0, "doing nothing (tender left in the marinade pan) scores 0")
    check(not M._L.dropped, "lying in the marinade pan is not dropped")


def test_demo_sequence_full():
    fresh()
    demo_coat()
    c = coat().coverage()
    b = M.bare_area() * 1e4
    check(b <= 8.5, f"the demo's grips and presses leave {b:.1f} cm^2 bare (<= 8.5, 1.5 under the 10 cm^2 line)")
    s = run(ON_RACK, n=M.HOLD_STEPS + 1)
    check(s == 1.0, "demo coat, on the rack, released and still for 1 s: 1.0")
    check(M.success(objs(ON_RACK)), "success() holds")


def test_one_second_needed():
    fresh()
    demo_coat()
    check(run(ON_RACK, n=M.HOLD_STEPS) == 0.0, "on the rack for under 1 s (its first step only sets the pose to compare with): 0")
    fresh()
    demo_coat()
    check(run(ON_RACK, n=40, v=(0.03, 0.0, 0.0)) == 0.0, "on the rack but sliding (3 cm/s): 0")


def test_hand_left_on_caps():
    fresh()
    demo_coat()
    s = run(ON_RACK, n=M.HOLD_STEPS + 1, held_by="right_gripper")
    check(s == 0.5, f"a hand still on it at the end caps a full coat at 0.5 (got {s})")
    check(not M.success(objs(ON_RACK, held_by="right_gripper")), "success() needs the hand off")


def test_dropped_in_never_pressed():
    fresh()
    for _ in range(40):
        coat().physics(IN_BED, ck.quat(FLAT), 0.49, [], [])  # lying in the flour under its own weight
    for _ in range(2):
        coat().physics(IN_BED, ck.quat(FLAT), 15.0, [], [])  # the impact of being dropped in
    for _ in range(40):  # turned over in the flour, never pressed
        coat().physics(IN_BED, ck.quat(FLIP), 0.49, [], [])
    check(coat().coverage() == 0.0, "dropped into the breading and turned, never pressed: nothing sticks")
    check(run(ON_RACK, n=M.HOLD_STEPS + 5) == 0.0, "... and on the rack scores 0")


def test_pan_force_without_hand():
    fresh()
    for _ in range(50):
        coat().physics(IN_BED, ck.quat(FLAT), 5.0, [], [])
    check(coat().coverage() == 0.0, "a large pan force with no gripper pushing down sets nothing")


def test_one_face_only():
    fresh()
    press(IN_BED, FLAT)
    grip(IN_BED, FLAT, M.NEAR_GRIP)
    c = coat().coverage()
    s = run(ON_RACK, n=M.HOLD_STEPS + 1)
    check(s == 0.25, f"one face pressed ({c * 100:.1f} % coated): 0.25 (got {s})")


def test_one_station_each_face():
    fresh()
    press(IN_BED, FLAT, stations=(0.0,))
    press(IN_BED, FLIP, stations=(0.0,))
    grip(IN_BED, FLIP, M.NEAR_GRIP)
    c = coat().coverage()
    s = run(ON_RACK, n=M.HOLD_STEPS + 1)
    check(s == 0.25, f"both faces pressed at one spot only ({c * 100:.1f} % coated): 0.25 (got {s})")


def test_regrip_after_coat():
    fresh()
    demo_coat()
    grip(IN_BED, FLIP, 0.03)  # an extra grip after breading, at a fresh spot
    c = coat().coverage()
    s = run(ON_RACK, n=M.HOLD_STEPS + 1)
    check(s == 0.75, f"one extra grip at a fresh spot after the coat ({M.bare_area() * 1e4:.1f} cm^2 bare): 0.75 (got {s})")
    fresh()
    demo_coat()
    grip(IN_BED, FLIP, M.NEAR_GRIP)
    grip(IN_BED, FLIP, M.NEAR_GRIP)
    check(run(ON_RACK, n=M.HOLD_STEPS + 1) == 1.0, "regripping exactly the same spot costs nothing more: 1.0")


def test_many_grips():
    fresh()
    demo_coat()
    for x in (-0.01, 0.015, 0.04):
        grip(IN_BED, FLIP, x)
    s = run(ON_RACK, n=M.HOLD_STEPS + 1)
    check(s == 0.75, f"three extra grips along it ({M.bare_area() * 1e4:.1f} cm^2 bare): 0.75 (got {s})")
    fresh()
    demo_coat()
    for x in (-0.01, 0.015, 0.04):
        grip(IN_BED, FLIP, x)
        grip(IN_BED, FLAT, x, yaw=M.ACROSS, tcp=IN_BED + np.array([x, 0.0, -0.002]))  # lower down the sides too
    g_rot = rx(math.pi / 2) @ grip_rot(M.ACROSS)
    for x in (-0.03, 0.0, 0.03):  # and pinched across the broad faces
        g = ck.Grip((X0 + x, 0.0, IN_BED[2]), g_rot, HALF - 0.0003)
        for _ in range(3):
            coat().physics(IN_BED, ck.quat(FLIP), 0.0, [], [g])
    s = run(ON_RACK, n=M.HOLD_STEPS + 1)
    check(s <= 0.5, f"handled all over after breading ({M.bare_area() * 1e4:.1f} cm^2 bare): at most 0.5 (got {s})")


def test_light_carrying_pinch_wipes():
    law = M.LAW
    pose = {"r": (np.zeros(3), np.eye(3), 0.017)}
    f = {("r", "tip_left"): np.array([0.25, 0.0, 0.0]), ("r", "tip_right"): np.array([-0.25, 0.0, 0.0])}
    check(len(ck.classify(f, pose, law)[1]) == 1, "a 0.25 N pinch (enough to carry the tender at friction 1.0) is a grip")
    f = {("r", "tip_left"): np.array([0.0, 0.0, -0.5]), ("r", "tip_right"): np.array([0.0, 0.0, -0.5])}
    check(len(ck.classify(f, pose, law)[1]) == 0, "a fist pushing down is still a press, not a grip")


def test_grip_across_broad_faces():
    fresh()
    demo_coat()
    before = coat().coverage()
    g_rot = rx(math.pi / 2) @ grip_rot(M.ACROSS)  # a side grip pinching the top and bottom faces
    g = ck.Grip((X0, 0.0, IN_BED[2]), g_rot, HALF - 0.0003)
    p, n, _ = coat().world(IN_BED, ck.quat(FLIP))
    under = ck.under_pads(p, n, g, M.LAW.pad_reach)
    for _ in range(3):
        coat().physics(IN_BED, ck.quat(FLIP), 0.0, [], [g])
    check(under.any() and coat().coverage() < before - 0.05,
          f"a pinch on the broad faces wipes them too ({before * 100:.1f} -> {coat().coverage() * 100:.1f} %)")


def test_dropped_on_table_latched():
    fresh()
    demo_coat()
    run(IN_BED, FLIP, n=5)
    run(ON_TABLE, FLIP, n=3)
    check(M._L.dropped, "the tender on the table off every pan is dropped (latched)")
    check(run(ON_RACK, FLIP, n=M.HOLD_STEPS + 5) == 0.0, "... and stays 0 even back on the rack with a full coat")
    fresh()
    run(ON_TABLE, n=1)
    run(IN_BED, n=5)
    check(not M._L.dropped, "one control step on the table (a physics pop) is not a drop")
    fresh()
    run(np.array([X0, 0.0, M.TOP - 0.30]), n=2)
    check(M._L.dropped, "fallen off the table: dropped")


def test_left_in_breading():
    fresh()
    demo_coat()
    check(run(IN_BED, FLIP, n=60) == 0.0, "fully coated but left in the breading pan: 0")
    held_over = ON_RACK + np.array([0.0, 0.0, 0.03])
    check(run(held_over, FLIP, n=60, held_by="right_gripper") == 0.0, "held 3 cm over the rack: 0 (not on it)")


def test_new_trial_resets():
    fresh()
    demo_coat()
    run(ON_RACK, n=M.HOLD_STEPS + 1)
    kit._EPISODE["count"] += 1
    check(run(IN_MARINADE, n=5) == 0.0 and coat().coverage() == 0.0, "a new trial starts bare and scores 0")


def test_tiers_and_caps():
    cm2 = 1e-4
    check([M.tier(b * cm2) for b in (0, 10, 10.1, 25, 25.1, 50, 50.1, 85, 85.1, 122.5)]
          == [1.0, 1.0, 0.75, 0.75, 0.5, 0.5, 0.25, 0.25, 0, 0], "tiers at 10, 25, 50 and 85 cm^2 bare")
    check(abs(coat().total_area() - 122.5e-4) < 1e-9, "the tender's surface is 122.5 cm^2")
    check(M.HELD_CAP == 0.5 and M.HOLD_STEPS == 20, "held cap 0.5, 1 s hold")


def test_demo_plan():
    d = M.demo()
    t = [sum(w.duration for w in d[a]) for a in ("left", "right")]
    check(abs(t[0] - t[1]) < 1e-6, f"both arms' plans end together ({t[0]:.1f} s)")
    check(t[0] * 20 <= 1600 and 1600 <= M.RUN_LIMIT_STEPS, f"demo {t[0] * 20:.0f} steps fits --max-steps 1600")
    low = [f"{a} '{w.label}'" for a in d for w in d[a] if abs(getattr(w, "roll", 0.0)) < 1e-9
           and w.pos[2] - M.FIST_TIP < M.FLOOR + 0.001]  # top-down: fingertips 1.28 cm below the tool point
    check(not low, f"every top-down waypoint keeps the fingertips above the pan floor {low}")
    rolls = [round(math.degrees(w.roll)) for a in d for w in d[a]]
    check(90 in rolls and -90 in rolls, "the turn uses one roll each way (+90 right, -90 left)")
    check(all(abs(w.pos[1]) <= 0.30 and 0.15 <= w.pos[0] <= 0.40 for a in d for w in d[a]), "all waypoints in reach")


def test_drawn_tender():
    """The drawn tenderloin: coat cells land on its surface, and it covers the box's sides where the hands hold it."""
    c = coat()
    pts, nrm = M.tender_surface(c.c, c.n)
    check(np.abs(M._inside(pts)).max() < 0.02, "every coat cell is moved onto the drawn surface")
    check(np.allclose(np.linalg.norm(nrm, axis=1), 1.0) and ((nrm * c.n).sum(1) > 0).all(),
          "the drawn surface's normals point out the same way as the box faces'")
    check(np.linalg.norm(pts - c.c, axis=1).max() < 0.012, "no cell moves more than 1.2 cm (the tips)")
    L, W, T = M.TENDER_SIZE
    for x0, x1 in ((M.NEAR_GRIP - 0.0132, M.NEAR_GRIP + 0.0132), (M.FAR_GRIP - 0.0132, M.FAR_GRIP + 0.0132)):
        xs = np.linspace(x0, x1, 30)
        gap = W / 2 - M.tender_half_width(xs)
        check(gap.max() < 0.0035, f"the drawn sides reach within 3.5 mm of the box's where a fingertip holds it "
                                  f"({x0 * 100:.1f} to {x1 * 100:.1f} cm; {gap.max() * 1000:.1f} mm)")
    check(M.TENDER_LONG > L and M.TENDER_THICK > T and M.TENDER_THICK / 2 - T / 2 <= 0.0025,
          "the drawn shape is longer and thicker than the box, by at most 2.5 mm each face")
    check(len(M.BREADING_PAN.meshes) == 8, "the breading pan draws the flour and two shades of crumbs")


def test_soft_tender():
    """Three parts on two hinges; held level by one end part, the rest droops a little (10 to 20 degrees at the
    first hinge), and the drive is implicit (PhysX's own), so the light parts can't ring."""
    g = 9.81
    t1 = M.PART_MASS * g * (M.CUT + 2 * M.CUT + M.PART_LEN / 2)  # the middle and far parts, 1.9 and 5.8 cm out
    a1 = math.degrees(t1 / M.BEND_K)
    check(10 <= a1 <= 20, f"held by one end, the first hinge droops {a1:.0f} degrees")
    links = M.TENDER.links
    check(len(links) == 2 and all(lk.law is M._BEND and lk.axis == "Y" for lk in links),
          "two hinges across the width, each with an implicit stiffness law")
    check(abs(M.PART_MASS * 3 - M.TENDER_MASS) < 1e-12, "the parts weigh 50 g together")
    q = M.TENDER.links[0].law.drive("", 0.2, 0.0, {}, {})
    check(q[0] == 0.0 and q[1] == M.BEND_K and q[2] == 0.0, "the law drives toward straight with no explicit torque")
    fresh()
    o = objs(ON_RACK)
    M.score(o)
    check(not M._L.still, "the first step is never still (no earlier pose)")
    M.score(objs(ON_RACK))
    check(M._L.still, "a tender at rest is still")
    o = objs(ON_RACK)
    o["tender/far"] = st(np.asarray(o["tender/far"].pos) + np.array([0, 0, 0.002]), FLAT)
    M.score(o)
    check(not M._L.still, "one part moving 2 mm in a step (4 cm/s) is not still")


def test_text_numbers():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    check(j["instruction"] == M.INSTRUCTION, "task.json instruction = INSTRUCTION")
    check(j["run_limit"].startswith("2,400 steps"), "run limit 2,400 steps")
    text = " ".join(str(v) for v in j.values()) + " " + readme
    for s in ("2.0 N", "25 ms", "3 cm", "0.2 N", "10 cm^2", "25 cm^2", "50 cm^2", "85 cm^2", "122.5 cm^2", "11.5",
              "3.5", "1.4 cm", "50 g",
              "1 cm deep", "1 s", "1.6 cm", "24.4 x 16.8 x 2.5 cm"):
        check(s in text, f"text states {s!r}")
    for k in ("title", "instruction", "context", "setup", "scoring", "fidelity", "scores"):
        lim = dict(title=120, instruction=1000, context=1000, setup=4000, scoring=4000, fidelity=500, scores=300)[k]
        check(len(j[k]) <= lim, f"{k} within {lim} characters ({len(j[k])})")
    check(not re.search(r"\\b(?:90|80) %", text), "no stale percentage tiers in the text")


if __name__ == "__main__":
    n = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            n += 1
    print(f"{n} tests passed")
