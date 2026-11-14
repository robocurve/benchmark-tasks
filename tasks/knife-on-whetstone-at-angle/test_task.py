"""Offline checks of the whetstone scorer: hand-made knife poses and gripper positions step by step, no simulator.

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


def axrot(a, t):
    return m._axrot(a, math.radians(t))


def quat(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


def st(pos, q=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def knife_rot(heading, phi):
    """The knife turned to `heading`, then `phi` degrees about its own handle axis (+ tips the edge down on face A;
    180 - a lies face B down with the edge down by a)."""
    r0 = axrot((0, 0, 1), heading)
    return axrot(r0 @ np.array([0, 1.0, 0]), phi) @ r0


ON = m.HANDLE_T / (2 * abs(m.FINGER_OPEN))  # the opening with both fingers on the handle's 1.8 cm thickness


def grip_rot(r):
    """The gripper's rotation for the demo's grip on a knife at rotation r: fingers closing across the handle's
    thickness (the knife's z), approach along the handle toward the tip (the knife's y)."""
    return np.column_stack([r[:, 2], r[:, 0], r[:, 1]])


def frame(pos, r, grip="right", opening=ON, grippers=None, at=(0.0, 0.0, 0.0)):
    """One control step: knife at (pos, r); the right gripper on the handle (grip="right", shifted `at` in the
    knife's frame), or `grippers` given (name -> position, or (position, rotation))."""
    g = {k: (v, np.eye(3)) for k, v in HOME.items()}
    if grippers is not None:
        g.update({k: (v if isinstance(v[0], (tuple, np.ndarray)) and len(v) == 2 else (v, np.eye(3)))
                  for k, v in grippers.items()})
    elif grip == "right":
        g["right_gripper"] = (tuple(np.asarray(pos) + r @ (np.array([0, m.GRIP_Y, 0]) + at)), grip_rot(r))
    kit._OPENING["right_gripper"] = opening
    kit._OPENING["left_gripper"] = 1.0
    objs = {"knife": st(pos, quat(r)), "board": st((*m.BOARD_POS, m.TOP)), "stone": st((*m.STONE_POS, m.TOP)),
            **{k: st(p, quat(q)) for k, (p, q) in g.items()}}
    ok = m.success(objs)
    return m.score(objs), ok


def on_stone_pos(r, a, lift=0.0, v=None):
    """Knife position putting the edge point `v` along the blade (default: the demo's, 9 cm past the bolster) at `a`
    along the stone's length, its lowest edge point `lift` above its top (0: resting on it)."""
    v = m.STONE_V + m.GRIP_Y if v is None else v
    target = np.r_[m.STONE_POS + a * m.S_U[:2], m.TOP + m.STONE_TOP + lift]
    p = target - r @ np.array([m.edge_x(v), v, 0.0])
    lows = [(p + r @ np.array([m.edge_x(y), y, s * m.BLADE_T / 2]))[2] for y in m.EDGE_SAMPLES for s in (-1, 1)]
    p[2] += (m.TOP + m.STONE_TOP + lift) - min(lows)
    return p


def strokes(face, n, angle=17.0, lead=True, lift=0.0, a0=None, travel=0.065, v=None, pitch=0.0, **kw):
    """n strokes on `face` ('A' pushes along +u, 'B' pulls), each lifted 2 cm between; `v`: which part of the blade
    is over the stone (m along the blade from the bolster; default the demo's middle); `pitch`: degrees tip-down."""
    phi = angle if face == "A" else 180.0 - angle
    r = knife_rot(m.STONE_HEADING, phi)
    r = axrot(r @ np.array([1.0, 0, 0]), -pitch if face == "A" else pitch) @ r
    sign = (1 if face == "A" else -1) * (1 if lead else -1)
    a0 = (-0.005 * sign) if a0 is None else a0
    out = []
    for _ in range(n):
        for k in range(21):
            out.append(frame(on_stone_pos(r, a0 + sign * travel * k / 20, lift, v), r, **kw))
        p = on_stone_pos(r, a0 + sign * travel, lift, v) + [0, 0, 0.02]
        out.append(frame(p, r, **kw))
        out.append(frame(on_stone_pos(r, a0, lift, v) + [0, 0, 0.02], r, **kw))
    return out


SECTIONS = (0.035, 0.0825, 0.140)  # heel, middle and tip of the blade over the stone's middle, for strokes in place


def sweep(face, n, angle=17.0, lead=True, lift=0.0, pitch=0.0, **kw):
    """n heel-to-tip strokes on `face`, as the demo makes them: the heel set on the stone, the knife drawn 8 cm along
    it (face A pushed, face B pulled, edge leading; `lead`=False reverses them) while sliding along its own length so
    the stone runs from heel to tip, then lifted 2 cm and brought back. `pitch`: degrees tip-down throughout."""
    phi = angle if face == "A" else 180.0 - angle
    r = knife_rot(m.STONE_HEADING, phi)
    r = axrot(r @ np.array([1.0, 0, 0]), -pitch if face == "A" else pitch) @ r
    sign = (1 if face == "A" else -1) * (1 if lead else -1)
    a0, dw, run = -0.04 * sign, 0.02, 0.08
    path = ([(m.DEMO_HEEL, a0 + sign * dw * k / 5) for k in range(6)]
            + [(m.DEMO_HEEL + (m.DEMO_SWEEP_V - m.DEMO_HEEL) * k / 20, a0 + sign * (dw + (run - 2 * dw) * k / 20))
               for k in range(1, 21)]
            + [(m.DEMO_SWEEP_V + (m.DEMO_TIP - m.DEMO_SWEEP_V) * k / 5, a0 + sign * (run - dw + dw * k / 5))
               for k in range(1, 6)])
    out = []
    for _ in range(n):
        out += [frame(on_stone_pos(r, a, lift, v), r, **kw) for v, a in path]
        out.append(frame(on_stone_pos(r, a0 + sign * run, lift, m.DEMO_TIP) + [0, 0, 0.02], r, **kw))
        out.append(frame(on_stone_pos(r, a0, lift, m.DEMO_HEEL) + [0, 0, 0.02], r, **kw))
    return out


def sharpen(face, n=m.PER_SIDE + 1, **kw):
    """A face sharpened as the demo does it: PER_SIDE + 1 heel-to-tip strokes."""
    sweep(face, n, **kw)


def start_pos():
    return np.array(m.KNIFE_START), knife_rot(m.PARK_HEADING, 0.0)


def put_down(heading=m.PARK_HEADING, phi=0.0, release=True, home=True):
    r = knife_rot(heading, phi)
    p = np.array(m.KNIFE_START)
    for _ in range(12):  # carried over to the board, well above everything
        frame(p + [0, 0, 0.08], r)
    if phi:
        p[2] += 0.02
    frame(p, r)
    if release:
        frame(p, r, grippers=HOME if home else {"right_gripper": (0.30, -0.12, m.TOP + 0.15)})
        return frame(p, r, grippers=HOME if home else {"right_gripper": (0.30, -0.12, m.TOP + 0.15)})
    return frame(p, r)


def new_trial():
    kit._EPISODE["count"] += 1
    p, r = start_pos()
    frame(p, r, grippers=HOME)  # first step: arms home, knife on the board


def check(name, got, want):
    assert abs(got - want) < 1e-9, f"{name}: score {got}, expected {want}"
    print(f"ok  {name}: {got}")


# 1. Do nothing.
new_trial()
p, r = start_pos()
for _ in range(30):
    s, _ = frame(p, r, grippers=HOME)
check("do nothing", s, 0.0)

# 2. The full job: 11 heel-to-tip strokes per face (face A pushed, face B pulled, edge leading), 17 degrees, put
# down flat edge away.
new_trial()
sharpen("A")
sharpen("B")
s, ok = put_down()
check(f"full job (worked {m.coverage('A')} and {m.coverage('B')} of 16 points)", s, 1.0)
assert ok, "success() should be true at the end of the full job"

# 2b. Review, Nov 12: repeated strokes on the same part of the edge (all with the blade's middle on the stone) cover
# about 6 of the 16.5 cm: 0.5, not full credit.
new_trial()
strokes("A", 11)
strokes("B", 11)
check(f"11 strokes per face on the same 6 cm (worked {m.coverage('A')} and {m.coverage('B')})", put_down()[0], 0.5)
# Under the floor of 10 strokes per face: 9 heel-to-tip strokes per face score nothing, however well spread; 10 do.
new_trial()
sharpen("A", n=m.PER_SIDE - 1)
sharpen("B", n=m.PER_SIDE - 1)
check("9 heel-to-tip strokes per face: under 10 per face", put_down()[0], 0.0)
new_trial()
sharpen("A", n=m.PER_SIDE)
sharpen("B", n=m.PER_SIDE)
check(f"10 heel-to-tip strokes per face (worked {m.coverage('A')} and {m.coverage('B')})", put_down()[0], 1.0)
# One heel-to-tip stroke plus nine more over the middle: 10 strokes per face, but the heel's and the tip's points have
# had one stroke's travel each, under the 10 cm that works a point: 0.5.
new_trial()
sharpen("A", n=1)
strokes("A", 9)
sharpen("B", n=1)
strokes("B", 9)
check(f"10 strokes per face, heel and tip stroked once (worked {m.coverage('A')} and {m.coverage('B')})",
      put_down()[0], 0.5)
new_trial()
sharpen("A")
strokes("B", 11, v=0.13)
check("face A all along, face B only at the tip", put_down()[0], 0.5)


# 2d. A back-and-forth stroke kept on the stone (Kramer: "move the blade straight back and forth"): each edge-leading
# pass of 5 cm or more is a stroke, the edge-trailing returns count for nothing. 11 round trips with the middle on the
# stone: 11 strokes per face, one place on the edge (0.5); the same with sweeps along the blade: 1.0.
def back_and_forth(face, n, travel=0.065, v=None, sweep_v=False):
    phi = 17.0 if face == "A" else 163.0
    r = knife_rot(m.STONE_HEADING, phi)
    sign = 1 if face == "A" else -1
    a0 = -0.005 * sign
    for i in range(n):
        for k in list(range(21)) + list(range(19, 0, -1)):
            vv = (m.DEMO_HEEL + (m.DEMO_TIP - m.DEMO_HEEL) * k / 20) if sweep_v else v
            frame(on_stone_pos(r, a0 + sign * travel * k / 20, 0.0, vv), r)
    frame(on_stone_pos(r, a0, 0.0, v) + [0, 0, 0.02], r)


new_trial()
back_and_forth("A", 11)
back_and_forth("B", 11)
cnt = m._totals()[0]
check(f"back and forth on the stone, middle only ({cnt['A']} and {cnt['B']} strokes)", put_down()[0], 0.5)
assert cnt == {"A": 11, "B": 11}, cnt
new_trial()
back_and_forth("A", 11, sweep_v=True)
back_and_forth("B", 11, sweep_v=True)
check("back and forth on the stone, sweeping heel to tip", put_down()[0], 1.0)
# A knife held on the stone at the right angle and jiggled 3 mm to and fro (a hand trembling, or solver jitter) for
# 30 s makes no strokes and works nothing: travel is counted net, per pass.
new_trial()
for face, phi in (("A", 17.0), ("B", 163.0)):
    r = knife_rot(m.STONE_HEADING, phi)
    frame(on_stone_pos(r, 0.0, 0.0) + [0, 0, 0.02], r)  # brought over the stone, then set down on it
    for k in range(600):
        frame(on_stone_pos(r, 0.003 * (k % 2), 0.0), r)
    frame(on_stone_pos(r, 0.0, 0.0) + [0, 0, 0.02], r)
cnt = m._totals()[0]
check(f"jiggled 3 mm on the stone for 30 s per face ({cnt['A']} and {cnt['B']} strokes, "
      f"{m.coverage('A')} points worked)", put_down()[0], 0.0)
assert cnt == {"A": 0, "B": 0} and m.coverage("A") == 0, (cnt, m.coverage("A"))

# 2c. Review, Nov 12: strokes hovering above the stone (the old scorer counted up to 3 mm) earn nothing.
for lift in (0.003, 0.002, 0.0008):
    new_trial()
    sharpen("A", lift=lift)
    sharpen("B", lift=lift)
    check(f"hovering {lift * 1000:.1f} mm above the stone", put_down()[0], 0.0)
new_trial()
sharpen("A", lift=0.0004)
sharpen("B", lift=0.0004)
check("resting 0.4 mm up (solver jitter) still touches", put_down()[0], 1.0)

# 3. Same, but the arm not back home: arm position is not scored.
new_trial()
sharpen("A")
sharpen("B")
check("arms not home", put_down(home=False)[0], 1.0)
assert not hasattr(m, "REQUIRE_ARMS_HOME") and not hasattr(m, "HOME_TOL")

# 4. Knife still held at the end (no release): 0.75.
new_trial()
sharpen("A")
sharpen("B")
check("still held at the end", put_down(release=False)[0], 0.75)

# 5. Flat strokes (5 degrees, scratching the face): no strokes count.
new_trial()
strokes("A", 4, angle=5.0)
strokes("B", 4, angle=5.0)
check("flat strokes", put_down()[0], 0.0)

# 6. Too steep (30 degrees): a steeper micro-bevel, not the 15-20 degree edge. Those touches do not count; there is no
# zero for the trial (the real check is the angle card, which shows the angle, not damage).
new_trial()
sharpen("A", angle=30.0)
sharpen("B", angle=30.0)
check("steep strokes only", put_down()[0], 0.0)
new_trial()
sharpen("A")
sharpen("B")
strokes("A", 1, angle=30.0)
check("steep stroke after good ones (it just does not count)", put_down()[0], 1.0)


def steep_ending(face, n, v=None):
    """n touches on `face`, each a full in-band stroke that then tips past 22 degrees for 3 cm before lifting off."""
    sign = 1 if face == "A" else -1
    for _ in range(n):
        for k in range(21):
            r = knife_rot(m.STONE_HEADING, 17.0 if face == "A" else 163.0)
            frame(on_stone_pos(r, -0.005 * sign + sign * 0.065 * k / 20, 0.0, v), r)
        for k in range(1, 7):
            r = knife_rot(m.STONE_HEADING, 30.0 if face == "A" else 150.0)
            frame(on_stone_pos(r, sign * (0.06 + 0.005 * k), 0.0, v), r)
        frame(on_stone_pos(r, sign * 0.09, 0.0, v) + [0, 0, 0.02], r)


new_trial()
sharpen("A")
steep_ending("B", 4)
check("every face-B touch ends too steep: none count", put_down()[0], 0.25)

# 7. One face only: 0.25.
new_trial()
sharpen("A")
check("one side only", put_down()[0], 0.25)

# 8. Two strokes per face: not enough.
new_trial()
strokes("A", 2)
strokes("B", 2)
check("two strokes per side", put_down()[0], 0.0)

# 9. Edge-trailing strokes only (lifted between, or the light return of a back-and-forth stroke): none count.
new_trial()
sweep("A", 11, lead=False)
sweep("B", 11, lead=False)
check("edge trailing", put_down()[0], 0.0)

# 10. Hovering 1 cm above the stone: no contact, no strokes.
new_trial()
strokes("A", 4, lift=0.01)
strokes("B", 4, lift=0.01)
check("hovering", put_down()[0], 0.0)

# 11. The knife dragged with the gripper open (pushed, not held), and with the other arm's open gripper.
new_trial()
strokes("A", 4, opening=1.0)
strokes("B", 4, opening=1.0)
check("dragged, gripper open", put_down()[0], 0.0)
new_trial()
strokes("A", 4, grip=None, grippers={"left_gripper": (0.40, -0.05, m.TOP + 0.05)})
strokes("B", 4, grip=None, grippers={"left_gripper": (0.40, -0.05, m.TOP + 0.05)})
check("dragged by the other arm", put_down()[0], 0.0)

# 11b. Holding is judged by contact (Nov 12): both fingers on the handle with their opening stopped by it. A hand
# half open round the handle (each finger 1.4 cm off it; the old rule, at most 60 % open with the tool point within
# 2 cm of the axis, called this held), or one finger on it and the other 5 mm off, does not hold the knife; a grip
# near the handle's end, 3 cm behind the demo's, does.
half_open = 0.5
new_trial()
sharpen("A", opening=half_open)
sharpen("B", opening=half_open)
check("strokes with the hand half open round the handle, not touching it", put_down()[0], 0.0)
new_trial()
sharpen("A", at=(0.0, 0.0, 0.009 + 0.005 - ON * abs(m.FINGER_OPEN)))
sharpen("B", at=(0.0, 0.0, 0.009 + 0.005 - ON * abs(m.FINGER_OPEN)))
check("strokes with one finger on the handle, the other 5 mm off it", put_down()[0], 0.0)
new_trial()
sharpen("A", opening=0.0)
sharpen("B", opening=0.0)
check("strokes with an empty closed fist on the handle", put_down()[0], 0.0)
new_trial()
sharpen("A", at=(0.0, -0.03, 0.0))
sharpen("B", at=(0.0, -0.03, 0.0))
check("strokes gripping the handle near its end", put_down()[0], 1.0)

# 11c. Let go is judged by contact: after laying the knife down, the fingers opened wide round the handle without
# touching it (the tool point 1.25 cm from the handle's middle) have let go; an open finger resting on the handle has
# not (0.75).
def let_go(opening, at=(0.0, 0.0, 0.0)):
    put_down(release=False)
    r = knife_rot(m.PARK_HEADING, 0.0)
    p = np.array(m.KNIFE_START)
    frame(p, r, opening=opening, at=at)
    return frame(p, r, opening=opening, at=at)


new_trial()
sharpen("A")
sharpen("B")
check("let go: fingers opened wide round the handle, not touching it", let_go(1.0)[0], 1.0)
new_trial()
sharpen("A")
sharpen("B")
check("not let go: an open finger resting on the handle's top",
      let_go(1.0, at=(0.0, 0.0, m.HANDLE_T / 2 - abs(m.FINGER_OPEN)))[0], 0.75)
new_trial()
sharpen("A")
sharpen("B")
check("let go: the same finger 3 mm above the handle",
      let_go(1.0, at=(0.0, 0.0, m.HANDLE_T / 2 - abs(m.FINGER_OPEN) + 0.003))[0], 1.0)

# 12. Put down with the edge towards the robot, or left on the stone: 0.75.
new_trial()
sharpen("A")
sharpen("B")
check("edge towards the robot", put_down(heading=m.PARK_HEADING + 180.0)[0], 0.75)
new_trial()
sharpen("A")
sharpen("B")
r = knife_rot(m.STONE_HEADING, 17.0)
p = on_stone_pos(r, 0.0)
frame(p, r, grippers=HOME)
check("left on the stone", frame(p, r, grippers=HOME)[0], 0.75)

# 13. Rolled off the end: a stroke carried past the stone's end with the edge dropping. That touch does not count;
# there is no zero for the trial.
def rolled(face, n, v=None):
    """n full strokes on `face`, each carried on past the stone's end with the edge dropping, then lifted."""
    sign = 1 if face == "A" else -1
    r = knife_rot(m.STONE_HEADING, 17.0 if face == "A" else 163.0)
    for _ in range(n):
        for k in range(21):
            frame(on_stone_pos(r, -0.005 * sign + sign * 0.085 * k / 20, 0.0, v), r)
        for k in range(1, 6):
            frame(on_stone_pos(r, sign * (0.09 + 0.006 * k), -0.002 * k, v), r)
        for _ in range(12):
            frame(on_stone_pos(r, sign * 0.12, 0.03, v), r)


new_trial()
sharpen("A")
strokes("B", 6)
rolled("B", 5)
check("six good face-B strokes and five rolled off the end", put_down()[0], 0.25)
new_trial()
sharpen("A")
strokes("B", 6)
strokes("B", 5)
check("the same eleven face-B strokes lifted off instead", put_down()[0], 0.5)
new_trial()
sharpen("A")
sharpen("B")
rolled("A", 1)
check("rolled off the end after a full job (it just does not count)", put_down()[0], 1.0)

# 14. A new trial (kit.episode() moves on) forgets the last one's strokes.
new_trial()
sharpen("A")
sharpen("B")
put_down()
new_trial()
p, r = start_pos()
check("reset after a full job", frame(p, r, grippers=HOME)[0], 0.0)

# 15. Pitched, not angled (reviewer's case): the knife rolled only 0.5 to 1 degree but pitched 15 to 17 degrees tip-down,
# dragged 7 cm edge first with only the tip on the stone. The old face-tilt angle read this as 15 to 17 degrees; the
# angle across the blade is about 1 degree, the length axis is not level and only the tip touches: no strokes.
def tip_pos(r, a, lift=0.0):
    """Knife position putting the lowest edge point at `a` along the stone's middle, `lift` above its top."""
    pts = [r @ np.array([m.edge_x(y), y, s * m.BLADE_T / 2]) for y in m.EDGE_SAMPLES for s in (-1, 1)]
    low = min(pts, key=lambda v: v[2])
    return np.r_[m.STONE_POS + a * m.S_U[:2], m.TOP + m.STONE_TOP + lift] - low


def pitched(face, roll, pitch, n=11, travel=0.07):
    """n edge-leading strokes on `face`, the knife rolled `roll` about its handle axis and pitched `pitch` tip-down
    (negative: heel-down)."""
    r = knife_rot(m.STONE_HEADING, roll if face == "A" else 180.0 - roll)
    r = axrot(r @ np.array([1.0, 0, 0]), -pitch if face == "A" else pitch) @ r
    sign = 1 if face == "A" else -1
    for _ in range(n):
        for k in range(21):
            frame(tip_pos(r, -0.005 * sign + sign * travel * k / 20), r)
        frame(tip_pos(r, sign * travel) + [0, 0, 0.02], r)
        frame(tip_pos(r, 0.0) + [0, 0, 0.02], r)


for roll, pitch in ((1.0, 17.0), (1.0, -17.0), (0.5, 15.0)):
    new_trial()
    pitched("A", roll, pitch)
    pitched("B", roll, pitch)
    check(f"pitched {pitch} deg, rolled {roll} deg, tip only", put_down()[0], 0.0)

# 16. Correct angle (17 deg across the blade) with the blade's length level or pitched 1.5 deg: 3 cm or more of the
# edge lies within 1 mm of the stone and every stroke counts, but all on one part of the edge (0.5). Pitched 3 deg
# heel-down, only about 2 cm of edge is within 1 mm: no strokes. Pitched 3 deg tip-down, the edge's rise near the tip
# (6 mm over its last 3 cm) lies along the stone, as when a sharpener lifts the handle for the tip: 4 cm on the stone,
# strokes count, still one place (0.5).
for pitch, want in ((0.0, 0.5), (1.5, 0.5), (-3.0, 0.0), (3.0, 0.5)):
    new_trial()
    pitched("A", 17.0, pitch)
    pitched("B", 17.0, pitch)
    check(f"17 deg across the blade, pitched {pitch} deg, one place on the edge", put_down()[0], want)

# 17. Correct angle but pitched 8 deg either way (2 cm of edge or less on the stone, length not level): no strokes.
for pitch in (8.0, -8.0):
    new_trial()
    pitched("A", 17.0, pitch)
    pitched("B", 17.0, pitch)
    check(f"17 deg across the blade, pitched {pitch} deg", put_down()[0], 0.0)

# 18. The scorer's edge points lie on the colliders' edge: the straight boxes end at the straight edge, and each turned
# box's outer face runs between two knots of the rising edge, so the points along the tip's rise touch what the
# physics touches.
blade = m._knife().colliders[1:]
for c in blade:
    a = math.radians(c.rot_z)
    ax, ay = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])  # the box's x and y axes
    out = [np.array(c.center[:2]) + ax * c.size[0] / 2 + ay * s * c.size[1] / 2 for s in (-1, 1)]
    on_edge = all(abs(x - m.edge_x(y)) < 1e-6 for x, y in out)
    assert on_edge or all(x < m.edge_x(y) - 0.001 for x, y in out), f"collider at y {c.center[1]:.3f} past the edge"
    assert on_edge or not c.rot_z, "a turned box off the edge"
assert sum(1 for c in blade if c.rot_z) == len(m.EDGE_KNOTS) - 1
print(f"ok  edge colliders follow the edge (tip {1000 * (m.EDGE_X - m.edge_x(m.BLADE_Y[1])):.0f} mm above the straight "
      f"edge; the last scorer point {1000 * (m.EDGE_X - m.edge_x(m.EDGE_SAMPLES[-1])):.1f} mm)")

# 19. The last scorer point, 16.2 cm (3 mm short of the point, 4.9 mm up the tip's rise), is on the stone only with the
# handle lifted for the tip, as a sharpener does: strokes in place (4 each with the heel, middle and tip over the
# stone) with the blade level work 15 of the 16 points; with the handle lifted 2.5 deg for the tip (inside the 5 deg
# level rule) all 16. The demo's heel-to-tip strokes do not lift the handle (see task.py, DEMO_TIP_YAW).
for tip_pitch, want in ((0.0, 15), (2.5, 16)):
    new_trial()
    for face in ("A", "B"):
        for v in SECTIONS:
            strokes(face, 4, v=v, pitch=tip_pitch if v == SECTIONS[-1] else 0.0)
    s, _ = put_down()
    got = (m.coverage("A"), m.coverage("B"))
    assert got == (want, want) and s == 1.0, f"tip strokes pitched {tip_pitch} deg: worked {got}, score {s}"
    print(f"ok  tip strokes pitched {tip_pitch} deg tip-down: worked {got[0]} and {got[1]} of 16 points, score {s}")

print("all pass")
