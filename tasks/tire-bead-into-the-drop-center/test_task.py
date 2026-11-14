"""Offline checks of the tire-bead task: the last section's slack law, and the scorer on hand-made joint, lever and
gripper states played as short trials (no simulator).

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
HOME = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
D = math.radians
DOWN = -m.DROP
NONE = {}  # no sector down
OPP3 = {"s210": DOWN, "s270": DOWN, "s330": DOWN}  # the three away from the valve, in the channel
LEVER_REST = (m.LEVER_POS[0], m.LEVER_POS[1], m.TOP + 0.004)


def st(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def wheel_pt(r, deg, h):
    a = D(deg)
    return (m.WX + r * math.cos(a), m.WY + r * math.sin(a), m.TOP + m.ZR + h)


def lever_with_tip(p):
    """Lever pose (yaw 0) with its spoon tip at world point p."""
    return tuple(np.asarray(p, float) - np.asarray(m.LEVER_TIP, float))


def step(sectors=NONE, flap=m.FLAP_START, valve=m.VALVE_START, lever=LEVER_REST, grippers=None, opening=(0.0, 0.0)):
    for n in m.SECTORS:
        kit._JOINTS[f"wheel/{n}"] = kit.JointState(pos=sectors.get(n, 0.0), vel=0.0, latched=False)
    kit._JOINTS["wheel/flap"] = kit.JointState(pos=flap, vel=0.0, latched=False)
    kit._JOINTS["wheel/valve"] = kit.JointState(pos=valve, vel=0.0, latched=False)
    kit._OPENING.update({"left_gripper": opening[0], "right_gripper": opening[1]})
    g = dict(HOME)
    g.update(grippers or {})
    o = {"wheel": st((m.WX, m.WY, m.TOP + m.ZR)), "lever": st(lever)}
    o.update({k: st(v, (1.0, 0.0, 0.0, 0.0)) for k, v in g.items()})  # pointing down
    return o


def trial(steps):
    """A fresh trial: a first step at the start (grippers recorded as home), the given steps (each (n, kwargs)), then
    the last state twice more (so the arms read still). Returns (score, success)."""
    kit._EPISODE["count"] += 1
    m.success(step())
    o = None
    for n, kw in steps:
        for _ in range(n):
            o = step(**kw)
            m.success(o)
    o = step(**kw)
    m.success(o)
    o = step(**kw)
    return m.score(o), m.success(o)


HELD = OPP3 | {"s150": DOWN}  # the three far arcs in the channel and the one beside the last section (150) held down
done = dict(sectors=OPP3, flap=0.0, valve=0.0)  # rolled over; the held arc let go and back up on its shoulder


TWO = {"left_gripper": wheel_pt(0.118, 90, 0.022 + m.PAD_TIP_PAST_TCP),  # rolling the last section
       "right_gripper": wheel_pt(0.116, 170, 0.016 + m.PAD_TIP_PAST_TCP)}  # holding the arc beside it down


def rolling(kw, secs=0.5, start=m.FLAP_START):
    """The last section rolled from `start` to kw's flap at an even pace over `secs` seconds (a careful hand takes
    about half a second; the demo takes 3.2 s), one control step at a time."""
    n = max(1, round(secs / m.DT))
    end = kw.get("flap", 0.0)
    return [(1, dict(kw, flap=start + (end - start) * k / n)) for k in range(1, n + 1)]


def roll(flap=0.0, valve=0.0, hands=TWO, secs=0.5):
    """Slack made, valve set, the last section rolled over while the arc beside it is held down."""
    return [(3, dict(sectors=OPP3)), (3, dict(sectors=OPP3, valve=valve))] + rolling(
        dict(sectors=HELD, valve=valve, flap=flap, grippers=hands), secs)


made = roll() + [(2, done)]
pinch_270 = lever_with_tip(wheel_pt(0.099, 270, 0.002))  # in the channel under the seated bead
under_lip = lever_with_tip(wheel_pt(0.104, 92, 0.016))  # under the raised last section's lip
beside_270 = lever_with_tip(wheel_pt(0.125, 270, 0.035))  # 3 cm off, over the tire
on_top = lever_with_tip(wheel_pt(0.105, 270, m.Z_FLANGE + 0.004))  # lying on the tire, over the flange
flat_on_dropped = lever_with_tip(wheel_pt(0.099, 270, m.Z_BEAD - m.DROP + 0.003))  # lying on an arc pushed down
near_valve = {"left_gripper": wheel_pt(m.R_BED - 0.02, 90, 0.0)}
pressing = {"left_gripper": wheel_pt(0.118, 90, 0.026 + m.PAD_TIP_PAST_TCP)}
hover_open = {"left_gripper": wheel_pt(0.12, 90, 0.0285 + 0.005 + m.PAD_TIP_PAST_TCP)}
above = {"left_gripper": wheel_pt(0.12, 90, 0.0285 + 0.03 + m.PAD_TIP_PAST_TCP)}
one_arm = [(3, dict(sectors=OPP3)), (2, dict(sectors=HELD)), (2, dict(sectors=OPP3, valve=0.0))]  # s30 pushed, let go

# Review, Nov 12: the score reads the outcome (bead on, valve straight, tube not pinched, hands off), not the technique.
# That the last section needs the slack, and so a hand holding the arc beside it down while the other rolls it, is
# the wheel's own law (LastSection, checked at the bottom), not a scoring rule: offline, joint states that the law
# would never allow (a section rolled over by one hand) score by their outcome.
CASES = [
    ("do nothing", [(5, {})], 0.0),
    ("slack made (three arcs in the channel), last section not rolled over", [(5, dict(sectors=OPP3, valve=0.0))],
     0.25),
    ("two arcs in the channel", [(5, dict(sectors={"s270": DOWN, "s330": DOWN}))], 0.25),
    ("one arc in the channel only", [(5, dict(sectors={"s270": DOWN}))], 0.0),
    ("arcs pushed only half way into the channel", [(5, dict(sectors={n: DOWN / 2 for n in m.SECTORS}))], 0.0),
    ("last section pressed without slack: stopped at the flange's edge", rolling(dict(flap=m.BLOCK_AT, valve=0.0))
     + [(5, dict(flap=m.BLOCK_AT, valve=0.0))], 0.0),
    ("the right way: slack, arc beside held, rolled over, valve straight, let go", made, 1.0),
    ("swapped hands (left holds the arc, right rolls)",
     [(3, dict(sectors=OPP3))] + rolling(dict(sectors=HELD, valve=0.0, flap=0.0, grippers={
         "left_gripper": TWO["right_gripper"], "right_gripper": TWO["left_gripper"]})) + [(2, done)], 1.0),
    ("the arc beside held down by the lever, the other hand rolling (an allowed technique)",
     [(3, dict(sectors=OPP3))] + rolling(dict(sectors=HELD, valve=0.0, flap=0.0, grippers=TWO,
                                              lever=lever_with_tip(np.asarray(wheel_pt(0.116, 170, 0.016 + 0.004))
                                                                   + np.asarray(m.LEVER_TIP) - (0.03, 0, 0))))
     + [(2, done)], 1.0),
    ("pressed 1 degree past the stop without slack at first, done properly later (outcome counts)",
     rolling(dict(flap=m.BLOCK_AT + D(1.0))) + [(2, dict(flap=m.BLOCK_AT + D(1.0)))] + made, 1.0),
    ("bead over, valve still leaning 20 degrees", roll(valve=m.VALVE_START) + [(2, dict(done, valve=m.VALVE_START))],
     0.5),
    ("valve at 9.5 degrees (straight enough)", roll() + [(2, dict(done, valve=D(9.5)))], 1.0),
    ("valve at 10.5 degrees", roll() + [(2, dict(done, valve=D(10.5)))], 0.5),
    ("valve pushed past straight to -15 degrees", roll() + [(2, dict(done, valve=D(-15)))], 0.5),
    ("flap 3 degrees short (lip on the flange top), slack arcs still down", roll(flap=D(-3.0))
     + [(2, dict(done, flap=D(-3.0)))], 0.25),
    ("flap 1.5 degrees short (lip under the flange top)", roll(flap=D(-1.5)) + [(2, dict(done, flap=D(-1.5)))], 1.0),
    ("done, a closed hand still pressing the bead", made + [(2, dict(done, grippers=pressing))], 0.5),
    # contact, not nearness (the old 1 cm zone capped these hovers at 0.5)
    ("done, an open hand hovering 5 mm over the tire (not touching)", made + [(2, dict(done, grippers=hover_open,
                                                                                       opening=(1.0, 0.0)))], 1.0),
    ("done, closed fingertips 5 mm over the tire's top (not touching)",
     made + [(2, dict(done, grippers={"left_gripper": wheel_pt(0.133, 90, 0.0285 + 0.005 + m.PAD_TIP_PAST_TCP)}))],
     1.0),
    ("done, closed fingertips 1 mm over the tire's top (within the 2 mm contact tolerance)",
     made + [(2, dict(done, grippers={"left_gripper": wheel_pt(0.133, 90, 0.0285 + 0.001 + 0.0128)}))], 0.5),
    ("done, a closed hand 5 mm beside the straight valve stem (not touching)",
     made + [(2, dict(done, grippers={"left_gripper": wheel_pt(m.R_BED - 0.035, 90 + math.degrees(
         (m.VALVE_R + 0.010 + 0.005) / (m.R_BED - 0.035)), 0.0)}))], 1.0),
    ("done, a hand at the valve", made + [(2, dict(done, grippers=near_valve))], 0.5),
    ("done, closed fingers resting on the tread's outer edge", made
     + [(2, dict(done, grippers={"right_gripper": wheel_pt(0.155, 0, 0.0)}))], 0.5),
    ("done, a hand 3 cm over the tire, let go, not home (arm position is not scored)",
     made + [(2, dict(done, grippers=above))], 1.0),
    ("lever deep in the channel under the seated bead (pinch), then done by hand",
     [(3, dict(lever=pinch_270))] + made, 0.0),
    ("lever tip 2 mm inside the bead seat line under the raised lip (reaches the tube)",
     [(3, dict(lever=lever_with_tip(wheel_pt(m.R_SEAT - 0.002, 92, 0.012))))] + made, 0.0),
    ("pinch, then everything else right: still 0 (irreversible)", [(1, dict(lever=pinch_270)), (1, {})] + made, 0.0),
    ("careful lever: tip hooking the raised last section's lip over the flange, then done",
     [(3, dict(lever=under_lip))] + made, 1.0),
    ("careful lever: tip on the shoulder under a seated arc's bead, then done",
     [(3, dict(lever=lever_with_tip(wheel_pt(m.R_SEAT + 0.002, 270, m.Z_BEAD - 0.002))))] + made, 1.0),
    ("lever in for one sampled step, out at once (still a pinch)",
     [(1, dict(lever=beside_270)), (1, dict(lever=pinch_270)), (1, dict(lever=beside_270)), (1, {})] + made, 0.0),
    ("lever lying on the tire over the flange (not under the bead)", roll() + [(3, dict(done, lever=on_top))], 1.0),
    ("lever's flat side pressing an arc pushed into the channel (tip on the bead, not under it)",
     [(3, dict(sectors=OPP3, lever=flat_on_dropped))] + made, 1.0),
    ("lever swung fast across the channel in one step (sweep)",
     [(1, dict(lever=lever_with_tip(wheel_pt(0.080, 270, 0.0)))), (1, dict(lever=lever_with_tip(wheel_pt(0.118, 270, 0.0)))),
      (1, {})] + made, 0.0),
    ("lever carried fast over the tire, 4 cm above it, in one step (not under the bead)",
     roll() + [(1, dict(done, lever=lever_with_tip(wheel_pt(0.17, 250, 0.07)))),
               (1, dict(done, lever=lever_with_tip(wheel_pt(0.03, 250, 0.07)))), (2, done)], 1.0),
    ("brisk roll: the last section rolled over in 0.2 s (7.8 cm/s at the lip)", roll(secs=0.2) + [(2, done)], 1.0),
    ("the demo's pace: rolled over in 3.2 s", roll(secs=3.2) + [(2, done)], 1.0),
    ("strike: the last section struck from its start to seated in 0.1 s (15.5 cm/s), then left right: capped at 0.5",
     roll(secs=0.1) + [(2, done)], 0.5),
    ("strike, and the valve left leaning: 0.5 (the cap and the valve level agree)",
     roll(valve=m.VALVE_START, secs=0.1) + [(2, dict(done, valve=m.VALVE_START))], 0.5),
    ("a lever tip in the tube's space, then a strike: 0 (the pinch rule, not the speed)",
     [(3, dict(lever=pinch_270))] + roll(secs=0.1) + [(2, done)], 0.0),
    ("strike that stops 3 degrees short, slack arcs still down: 0.25 (under the cap)",
     [(3, dict(sectors=OPP3))] + rolling(dict(sectors=HELD, valve=0.0, flap=D(-3.0), grippers=TWO), 0.05,
                                         start=m.FLAP_START) + [(2, dict(done, flap=D(-3.0)))], 0.25),
    ("just under the limit: rolled over in 0.15 s (10.3 cm/s at the lip), no cap", roll(secs=0.15) + [(2, done)], 1.0),
    ("a strike that stops 3 degrees short (21.7 cm/s), then pressed in slowly: still capped at 0.5 (irreversible)",
     [(3, dict(sectors=OPP3))] + rolling(dict(sectors=HELD, valve=0.0, flap=D(-17.0), grippers=TWO), 0.05,
                                         start=m.FLAP_START)
     + rolling(dict(sectors=HELD, valve=0.0, flap=D(-3.0), grippers=TWO), 0.05, start=D(-17.0))
     + rolling(dict(sectors=HELD, valve=0.0, flap=0.0, grippers=TWO), 1.0, start=D(-3.0)) + [(2, done)], 0.5),
    ("a press into the hard stop without slack in one step (10.8 cm/s), then done properly: allowed",
     rolling(dict(flap=m.BLOCK_AT), 0.1) + [(2, dict(flap=m.BLOCK_AT))] + made, 1.0),
    ("a strike into the hard stop without slack in one step (21.7 cm/s), then done properly: capped at 0.5",
     rolling(dict(flap=m.BLOCK_AT), 0.05) + [(2, dict(flap=m.BLOCK_AT))] + made, 0.5),
    ("a strike into the hard stop without slack, nothing else done: 0 (a strike alone never earns credit)",
     rolling(dict(flap=m.BLOCK_AT), 0.05) + [(3, dict(flap=m.BLOCK_AT))], 0.0),
    ("lever hooking the lip while the last section is rolled over at a careful pace",
     [(3, dict(sectors=OPP3))] + rolling(dict(sectors=HELD, valve=0.0, flap=0.0, grippers=TWO, lever=under_lip))
     + [(2, done)], 1.0),
    ("the snap-in itself: from 2.5 degrees short to seated in one step is not counted",
     roll(flap=-m.FLAP_SEAT_ZONE - 1e-6) + [(1, dict(sectors=HELD, valve=0.0, flap=0.0, grippers=TWO)), (2, done)],
     1.0),
    ("lever moved next to the wheel, tip outside the tire", roll() + [(3, dict(done, lever=lever_with_tip(
        wheel_pt(0.17, 120, -0.02))))], 1.0),
]

bad = 0
for name, seq, want in CASES:
    got, _ = trial(seq)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)


# Success, and a reset (kit.episode() changes) clearing the pinch latch and the strike cap.
sc, su = trial(made)
trial([(3, dict(lever=pinch_270))])
sc2, _ = trial(made)
trial(roll(secs=0.1))
sc3, _ = trial(made)

# The slack law: it reaches the last section only through an unbroken run of arcs in the channel from one of its ends.
law = m.LastSection()
far = {n: 0.0 for n in m.SECTORS} | OPP3


def blocked(J, q=m.FLAP_START):
    mem = {}
    law.drive("wheel/flap", q, 0.0, J, mem)
    return mem["blocked"], mem["limits"]


b_far, lim_far = blocked(far)
b_two = blocked({n: 0.0 for n in m.SECTORS} | {"s330": DOWN, "s30": DOWN})[0]
b_held = blocked(far | {"s30": DOWN})[0]
b_min = blocked({n: 0.0 for n in m.SECTORS} | {"s30": DOWN, "s330": DOWN, "s270": DOWN})[0]
b_other = blocked({n: 0.0 for n in m.SECTORS} | {"s150": DOWN, "s210": DOWN, "s270": DOWN})[0]
b_half = blocked({n: -m.DROP / 2 for n in m.SECTORS})[0]
b_mid = blocked(far, D(-4.0))[0]  # pushed past the stop without slack, not yet seated: the stop is put back
b_seated = blocked(far, D(-1.0))[0]  # seated, then the held arc let go: it stays in
t, k, _, _ = law.drive("wheel/flap", D(-2.0), 0.0, far | {"s30": DOWN}, {})
creep = m.Creep().drive("wheel/s30", -m.DROP, 0.0, {}, {})
lk = {l.name: l for l in m.OBJECTS[0].links}


flap_xy = [np.array(m._at(m.LAST_AT + d, r)) for d, (r, _) in m.FLAP_PRESS]
side_xy = [np.array(c.center[:2]) for n in m.NEXT_TO_LAST for c in lk[n].colliders]
gap = min(np.linalg.norm(a - b) for a in flap_xy for b in side_xy) - 0.010 - 0.011
for flag, name in ((sc == 1.0 and su, "done and let go: 1.0 and success"),
                   (sc2 == 1.0, "a reset clears the previous trial's pinch"),
                   (sc3 == 1.0, "a reset clears the previous trial's strike cap"),
                   (m.STRIKE_CAP == 0.5 and not hasattr(m, "SLAM_SPEED"), "a strike caps at 0.5, never a speed-only 0"),
                   (abs(m.LIP_ARM - 0.0443) < 0.0005, "the bead lip is 4.4 cm from the hinge line"),
                   (not hasattr(m, "REQUIRE_ARMS_HOME") and not hasattr(m, "ONE_HAND") and not hasattr(m, "FORCED"),
                    "no arm-home, one-hand or forced rule left in the scorer"),
                   (b_far and lim_far == (m.FLAP_LO, m.BLOCK_AT), "three far arcs down, none beside held: blocked at the edge"),
                   (b_two, "the arc beside plus one far arc (8.4 mm): blocked"),
                   (not b_held, "three far arcs and the arc beside held (16.8 mm): it rolls over"),
                   (not b_min and not b_other, "the arc beside and the next two on either side (12.6 mm): it rolls over"),
                   (b_half, "all five only half-way down (out of the click zone): no slack reaches it"),
                   (b_mid, "pushed past the stop without slack, short of the seat zone: the stop is put back"),
                   (not b_seated, "seated, then the held arc let go: it stays in"),
                   (t == 0.0 and k > 0, "within 2.5 degrees of seated it is pulled in"),
                   (creep[2] > 3.0, "an arc beside the last section is pushed back up (3.6 N at the floor)"),
                   (m.LastSection.sets_limits, "the law declares it narrows its limits (released at reset)"),
                   (gap > 2 * 0.04695, "no one hand (9.4 cm open) spans an arc beside and the last section"),
                   (2 * m.SLACK_PER_SECTOR < m.SLACK_NEED <= 3 * m.SLACK_PER_SECTOR, "10 mm needs three 4.2 mm arcs"),
                   (m.lip_height(D(-2.7)) > m.Z_FLANGE > m.lip_height(D(-2.6)), "the lip crosses the flange top at 2.6 degrees"),
                   (abs(m.VALVE_LEN * math.sin(m.VALVE_TOL) - 0.0056) < 0.0001, "10 degrees = 5.6 mm at the tip")):
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)
print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
