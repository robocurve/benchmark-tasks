"""Offline checks of the scrub-the-hub scorer: step-by-step histories of the pad, the syringe, its plunger and the
grippers, with near misses. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
HUB = m.HUB
bad = 0

Q0 = np.array([0.0, 0.0, 0.0, 1.0])
QX90 = np.array([math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)])  # pad stood up: its broad face toward +y
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.22), "right_gripper": (0.16, -0.26, m.TOP + 0.22)}
SYR_TRAY = m.SYR_START


def qx(deg):
    a = math.radians(deg) / 2
    return np.array([math.sin(a), 0.0, 0.0, math.cos(a)])


def qz(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, 0.0, math.sin(a), math.cos(a)])


def st(pos, q=Q0, v=(0, 0, 0), w=(0, 0, 0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.array(v, float), ang_vel=np.array(w, float),
                       low=p - 0.01, high=p + 0.01)


def pad_on_face(gap=0.0, dx=0.0, dz=0.0085, q=QX90):
    """The pad stood up, its broad face `gap` short of the access face (negative: pressed in)."""
    return (HUB[0] + dx, m.HUB_TIP_Y - m.PAD[2] / 2 - gap, HUB[2] + dz), q


def pad_leaning(deg, pen, dz=0.0085):
    """The pad stood up but leaning `deg` off square (about x), its face reaching `pen` past the access face where it
    meets the disc (negative: short of it)."""
    q = qx(90 + deg)
    r = m._rot(q)
    k = int(np.argmax(np.abs(r[1, :])))
    n = r[:, k] * math.copysign(1.0, r[1, k])
    h = m.PAD[k] / 2
    cz = HUB[2] + dz
    cy = (pen - m.HUB_R * math.hypot(n[0], n[2]) - h - n[2] * (cz - HUB[2])) / n[1] + HUB[1]
    return (HUB[0], cy, cz), q


def syr_at(tip_gap=None, radial=0.0, q=Q0, pos=None):
    """The syringe lying along +y with its tip end `tip_gap` short of the access face."""
    if pos is not None:
        return pos, q
    return (HUB[0] + radial, m.HUB_TIP_Y - tip_gap - m.SYR_L / 2, HUB[2]), q


ON_PAD = m.PAD[0] / (2 * 0.04695)  # the opening with both fingers on the pad's 3.5 cm sides (0.37)


def step(pad=None, pad_v=(0.02, 0, 0), pad_w=(0, 0, 0), held=True, syr=None, plunger=None,
         grip_open=ON_PAD, grip_at=(0.0, 0.0, 0.0), grip_q=Q0):
    objs = {}
    for k, v in PARK.items():
        objs[k] = st(v)
        kit._OPENING[k] = 1.0
    if pad is None:
        pad = (m.PAD_START, Q0)
        pad_v = (0, 0, 0)
        held = False
    objs["pad"] = st(pad[0], pad[1], v=pad_v, w=pad_w)
    if held:
        # pointing up, its fingers closing along x across the pad's 3.5 cm width (the pad stood up: its 6.5 cm along z)
        objs["right_gripper"] = st(np.array(pad[0]) + grip_at, grip_q)
        kit._OPENING["right_gripper"] = grip_open
    s = syr if syr is not None else (SYR_TRAY, Q0)
    objs["syringe"] = st(s[0], s[1])
    if plunger is not None:  # depth pushed in (m) since the start
        _, axis = m._syringe_tip(objs["syringe"])
        objs["syringe/plunger"] = st(np.array(s[0]) + axis * (m.PLUNGER_ANCHOR + plunger), s[1])
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, kwargs) pairs, or (steps, kwargs, fn) where fn(i) returns per-step kwargs. A fresh trial."""
    kit._EPISODE["count"] += 1
    out = step()
    for fr in frames:
        n, kw = fr[0], fr[1]
        for i in range(n):
            k = dict(kw)
            if len(fr) > 2:
                k.update(fr[2](i))
            out = step(**k)
    return out


def check(name, got, want):
    global bad
    ok = abs(got - want) < 1e-9 if isinstance(want, float) else got == want
    print(("ok  " if ok else "FAIL") + f" {name}: {got} (want {want})")
    if not ok:
        bad += 1


S = m.SCRUB_STEPS
FACE = {"pad": pad_on_face()}
SEATED = syr_at(0.0)


def scrub_alt(i):  # back-and-forth strokes across the face
    return {"pad_v": (0.02 if (i // 10) % 2 == 0 else -0.02, 0, 0)}


def push(total, n=60):
    return lambda i: {"plunger": total * (i + 1) / n}


# do nothing
check("do nothing scores 0", run([(400, {})])[0], 0.0)

# scrub on the access face
check("15 s face scrub alone scores 0.25", run([(S, FACE, scrub_alt)])[0], 0.25)
check("14.95 s of scrub scores 0", run([(S - 1, FACE, scrub_alt)])[0], 0.0)
check("scrub time adds up over separate bouts", run([(150, FACE), (50, {}), (150, FACE)])[0], 0.25)
check("twisting the pad on the face counts", run([(S, dict(FACE, pad_v=(0, 0, 0), pad_w=(0, 3.0, 0)))])[0], 0.25)
check("pressed 2 mm into the face still counts", run([(S, {"pad": pad_on_face(gap=-0.002)})])[0], 0.25)

# reviewer's case: rubbing the connector's top, not the access face (the old rule counted this)
top = ((m.HUB_X, m.HUB_TIP_Y + 0.010, m.AXIS_Z + m.HUB_R + m.PAD[2] / 2), Q0)
check("rubbing the connector's top scores 0", run([(S + 50, {"pad": top})])[0], 0.0)
check("rubbing the top, then a syringe 8 mm off, scores 0",
      run([(S + 50, {"pad": top}), (40, {"syr": syr_at(0.008)})])[0], 0.0)

# near misses on the face
check("hovering 2 mm off the face scores 0", run([(S + 20, {"pad": pad_on_face(gap=0.002)})])[0], 0.0)
check("pad held still on the face scores 0", run([(S + 20, dict(FACE, pad_v=(0, 0, 0.004)))])[0], 0.0)
check("pad pushed straight into the face scores 0", run([(S + 20, dict(FACE, pad_v=(0, 0.05, 0)))])[0], 0.0)
check("pad not held (fingers open) scores 0", run([(S + 20, dict(FACE, grip_open=0.9))])[0], 0.0)

# holding the pad is judged by contact: both fingers on it, their opening stopped by its 3.5 cm width
check("pad held by its end, 3.1 cm from its middle, counts (contact, not nearness to the middle)",
      run([(S, dict(FACE, grip_at=(0.0, 0.0, 0.031)), scrub_alt)])[0], 0.25)
# the old rule's case: closed to the pad's width with its tool point 2.3 cm from the pad's middle, but the hand
# pointing at the pad's back face (-y) with its fingertips 5 mm short of it
check("hand closed to the pad's width, fingertips 5 mm off its back face (no contact), scores 0",
      run([(S + 20, dict(FACE, grip_at=(0.0, -(m.PAD[2] / 2 + 0.005 + 0.0128), 0.0), grip_q=qx(-90)))])[0], 0.0)
check("fingers closed to the pad's width, their ends 3 mm below the pad's lower end (no contact), scores 0",
      run([(S + 20, dict(FACE, grip_at=(0.0, 0.0, -(m.PAD[1] / 2 + 0.0128 + 0.003))))])[0], 0.0)
check("open hand round the pad, fingers 3 mm off each side, scores 0",
      run([(S + 20, dict(FACE, grip_open=(m.PAD[0] + 0.006) / (2 * 0.04695)))])[0], 0.0)
check("one finger pressed into the pad's side, the other 7 mm off it, scores 0",
      run([(S + 20, dict(FACE, grip_open=0.39, grip_at=(0.006, 0.0, 0.0)))])[0], 0.0)
check("empty closed fist pressed on the pad scores 0", run([(S + 20, dict(FACE, grip_open=0.0))])[0], 0.0)
check("fingers squeezed 2 mm into the pad still hold it",
      run([(S, dict(FACE, grip_open=(m.PAD[0] - 0.004) / (2 * 0.04695)), scrub_alt)])[0], 0.25)
check("pad tilted 45 degrees (an edge on the face) scores 0",
      run([(S + 20, {"pad": pad_on_face(q=qx(45))})])[0], 0.0)
check("pad leaning 10 degrees, its face on the disc's rim, counts",
      run([(S, {"pad": pad_leaning(10, 0.001)}, scrub_alt)])[0], 0.25)
lean_short = pad_leaning(-10, -0.004)
r_ = m._rot(lean_short[1])
assert lean_short[0][1] + np.sum(np.abs(r_[1, :]) * np.array(m.PAD) / 2) > m.HUB_TIP_Y + 0.0015  # far corner 2 mm past
check("pad leaning 10 degrees, far corner past the face but the face 4 mm short of the disc, scores 0",
      run([(S + 20, {"pad": lean_short}, scrub_alt)])[0], 0.0)
check("pad leaning 25 degrees on the rim scores 0", run([(S + 20, {"pad": pad_leaning(25, 0.001)}, scrub_alt)])[0], 0.0)
check("pad off to the side, face not covered, scores 0", run([(S + 20, {"pad": pad_on_face(dx=0.015)})])[0], 0.0)
check("pad's edge face (1.2 cm) cannot cover the face reliably: off by 1 mm scores 0",
      run([(S + 20, {"pad": ((HUB[0], m.HUB_TIP_Y - m.PAD[0] / 2, HUB[2] + 0.001), qz(90))})])[0], 0.0)

# connection
check("reviewer's case: syringe 8 mm off the hub at rest scores 0.25",
      run([(S, FACE), (60, {"syr": syr_at(0.008)})])[0], 0.25)
check("syringe seated 0.5 s after the scrub scores 0.5", run([(S, FACE), (m.SEAT_STEPS, {"syr": SEATED})])[0], 0.5)
check("seated only 0.45 s scores 0.25", run([(S, FACE), (m.SEAT_STEPS - 1, {"syr": SEATED})])[0], 0.25)
# seated means the luer tip touching the port (contact), not a tip hovering near it (review, Nov 12)
check("tip hovering 1.5 mm off the port, centered and lined up, for 2 s scores 0.25 (not touching)",
      run([(S, FACE), (40, {"syr": syr_at(0.0015)})])[0], 0.25)
check("tip hovering 0.8 mm off the port for 2 s scores 0.25 (past the 0.5 mm contact gap)",
      run([(S, FACE), (40, {"syr": syr_at(0.0008)})])[0], 0.25)
check("tip touching, 0.4 mm short of the face (the resting gap), 0.5 s scores 0.5",
      run([(S, FACE), (m.SEAT_STEPS, {"syr": syr_at(0.0004)})])[0], 0.5)
check("tip pressed 1 mm into the port, 0.5 s scores 0.5",
      run([(S, FACE), (m.SEAT_STEPS, {"syr": syr_at(-0.001)})])[0], 0.5)
check("tip 4 mm past the face (through the solid port: a solver pop, not a seat) scores 0.25",
      run([(S, FACE), (40, {"syr": syr_at(-0.004)})])[0], 0.25)
check("touching 0.45 s, hovering 1 mm off for a step, touching 0.45 s again: not 0.5 s in a row (0.25)",
      run([(S, FACE), (m.SEAT_STEPS - 1, {"syr": SEATED}), (1, {"syr": syr_at(0.001)}),
           (m.SEAT_STEPS - 1, {"syr": SEATED})])[0], 0.25)
check("plunger pushed 3.1 cm with the tip hovering 1 mm off the port after seating does not count (0.5)",
      run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}), (60, {"syr": syr_at(0.001)}, push(0.031))])[0], 0.5)
check("seated 2 mm off the axis scores 0.25", run([(S, FACE), (40, {"syr": syr_at(0.0, radial=0.002)})])[0], 0.25)
check("seated but 8 degrees off line scores 0.25",
      run([(S, FACE), (40, {"syr": syr_at(0.0, q=qz(8))})])[0], 0.25)
check("seated, then pulled off and left, still 0.5 (it was connected)",
      run([(S, FACE), (20, {"syr": SEATED}), (40, {"syr": syr_at(0.03)})])[0], 0.5)

# flush


r = run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}), (60, {"syr": SEATED}, push(0.031))])
check("seated and plunger pushed 3.1 cm scores 1.0", r[0], 1.0)
check("... and success() is true", r[1], True)
check("plunger pushed only 2.5 cm scores 0.5",
      run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}), (60, {"syr": SEATED}, push(0.025))])[0], 0.5)
check("plunger pushed before seating does not count (0.5)",
      run([(S, FACE, lambda i: {"plunger": 0.04 * i / S}), (20, {"syr": SEATED, "plunger": 0.04})])[0], 0.5)
check("plunger pushed with the syringe pulled 5 mm off the face does not count (0.5)",
      run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}),
           (60, {"syr": syr_at(0.005)}, push(0.04))])[0], 0.5)
check("plunger jiggled in and out never adds up to a flush (0.5)",
      run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}),
           (200, {"syr": SEATED}, lambda i: {"plunger": 0.004 if i % 2 else 0.0})])[0], 0.5)
check("a full flush then drawn back 1 cm stays 1.0",
      run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}), (60, {"syr": SEATED}, push(0.031)),
           (20, {"syr": SEATED}, lambda i: {"plunger": 0.031 - 0.0005 * (i + 1)})])[0], 1.0)
r = run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}), (60, {"syr": SEATED}, push(0.031))])
v = m._H.vol
check("3.1 cm pushed while seated: 5.17 mL gone, all of it in the cylinder, none spilled",
      (round(v["gone"], 2), round(v["cylinder"], 2), round(v["spilled"], 2)), (5.17, 5.17, 0.0))
check("2 mm pushed on the tray, then 2.8 cm seated: 5 mL gone, 4.67 mL collected, 0.33 mL spilled (1.0)",
      run([(5, {"plunger": 0.0}), (10, {}, lambda i: {"plunger": 0.0002 * (i + 1)}), (S, FACE),
           (20, {"syr": SEATED, "plunger": 0.002}),
           (60, {"syr": SEATED}, lambda i: {"plunger": 0.002 + 0.028 * (i + 1) / 60})])[0], 1.0)
check("2.7 cm seated, then 6 mm pushed with the tip pulled off: 4.5 mL collected but 1 mL spilled (0.5)",
      run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}), (60, {"syr": SEATED}, push(0.027)),
           (20, {"syr": syr_at(0.005)}, lambda i: {"plunger": 0.027 + 0.0003 * (i + 1)})])[0], 0.5)
check("2.9 cm seated (4.83 mL): short of 5 mL gone (0.5)",
      run([(S, FACE), (20, {"syr": SEATED, "plunger": 0.0}), (60, {"syr": SEATED}, push(0.029))])[0], 0.5)
fk = m.fk
check("computed plunger force at 0.5 mL/s is about 3.6 N", round(fk.plunger_force(0.5), 1), 3.6)
check("... at 1 mL/s about 4.8 N", round(fk.plunger_force(1.0), 1), 4.8)
check("... at 2 mL/s about 7.1 N", round(fk.plunger_force(2.0), 1), 7.1)
check("unseated, the flow adds under 0.02 N at 2 mL/s", fk.pressure_force(2.0, seated=False) < 0.02, True)
check("scene without a plunger tops out at 0.5", run([(S, FACE), (40, {"syr": SEATED})])[0], 0.5)

# touching before the scrub
check("syringe touching the face before the scrub, scrubbed later, scores 0",
      run([(5, {"syr": syr_at(0.001)}), (S, FACE), (20, {"syr": SEATED}), (60, {"syr": SEATED}, push(0.04))])[0], 0.0)
check("syringe's side touching the face before the scrub scores 0",
      run([(5, {"syr": ((HUB[0] - m.SYR_W / 2 - m.HUB_R - 0.001, m.HUB_TIP_Y - 0.03, HUB[2]), Q0)}),
           (S, FACE), (20, {"syr": SEATED})])[0], 0.0)
check("syringe 1 cm from the face before the scrub (no contact) is not a fault",
      run([(30, {"syr": syr_at(0.010)}), (S, FACE), (20, {"syr": SEATED})])[0], 0.5)
check("syringe seated mid-scrub (14 s) scores 0",
      run([(280, FACE), (20, dict(FACE, syr=SEATED)), (40, FACE)])[0], 0.0)

# a reset starts a new history
run([(5, {"syr": syr_at(0.001)})])
check("after a reset the old fault is gone", run([(S, FACE), (20, {"syr": SEATED})])[0], 0.5)

print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
