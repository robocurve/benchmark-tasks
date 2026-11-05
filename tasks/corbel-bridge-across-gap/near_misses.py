"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None)}, expected score, expected success).
A course is given by its tip: how far the brick's inner end is from the midline (y = 0); the pier faces are 90 mm
from the midline, so reach = 90 mm minus the top tip.
"""

import math

from task import BRICK, EDGE, PIER_TOP, PIER_X, TOP

H, L = BRICK[2], BRICK[1]


def course(side, k, tip, x=PIER_X, lift=0.0):
    """A brick in course k (0 on the pier) of the left or right side, its inner end `tip` from the midline."""
    s = 1.0 if side == "l" else -1.0
    return (x, s * (tip + L / 2), PIER_TOP + k * H + H / 2 + lift)


def side(s, tips, names=None):
    names = names or [f"brick_{s}{i + 1}" for i in range(len(tips))]
    return {n: (course(s, k, t), None, None) for k, (n, t) in enumerate(zip(names, tips))}


DEMO = (0.073, 0.050, 0.0015)
BRIDGE = {**side("l", DEMO), **side("r", DEMO)}
MOVING = (0.0, 0.0, -0.05)


def turn(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


def tilt(deg):
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def top_turned(s, end, deg, k=2):
    """Course k of side s turned `deg` about the vertical, its inner end face's centre `end` from the midline."""
    th = math.radians(deg)
    sg = 1.0 if s == "l" else -1.0
    return ((PIER_X, sg * (end + L / 2 * math.cos(th)), PIER_TOP + k * H + H / 2), None, None, turn(deg))


def ry(deg):
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def both(tip3):
    return {**side("l", DEMO[:2] + (tip3,)), **side("r", DEMO[:2] + (tip3,))}


CASES = [
    ("demo bridge: top tips 3 mm apart, each side 88.5 mm out", BRIDGE, 1.0, True),
    ("top tips 4.8 mm apart", both(0.0024), 1.0, True),
    ("top tips 6 mm apart (87 mm out each)", both(0.003), 0.5, False),
    ("tips 4 mm apart but the right side reaches only 78 mm",
     {**side("l", (0.069, 0.044, -0.008)), **side("r", (0.073, 0.050, 0.012))}, 0.5, False),
    ("tips 3 mm apart but offset 25 mm front to back",
     {**side("l", DEMO), **{n: ((PIER_X + 0.025, y, z), None, None) for n, ((_, y, z), _s, _v) in side("r", DEMO).items()}},
     0.5, False),
    ("spare brick on top of the left top course as a counterweight",
     {**BRIDGE, "brick_l4": ((PIER_X, 0.09, PIER_TOP + 3 * H + H / 2), None, None)}, 1.0, True),
    ("left side built, right side two courses (40 mm out)", {**side("l", DEMO), **side("r", DEMO[:2])}, 0.25, False),
    ("both sides 60 mm out, not meeting", both(0.030), 0.5, False),
    ("both sides 59 mm out", both(0.031), 0.0, False),
    ("both sides two courses only (40 mm out)", {**side("l", DEMO[:2]), **side("r", DEMO[:2])}, 0.0, False),
    ("left courses stepped too far: the three-brick stack's centre of mass 4.5 mm past the pier's edge",
     {**side("l", (0.050, 0.025, 0.0015)), **side("r", DEMO)}, 0.0, False),
    ("left top brick 80 mm past the course below (its centre past that brick's end), right not built",
     side("l", (0.073, 0.050, -0.030)), 0.0, False),
    ("left top brick held 5 mm above its course", {**BRIDGE, "brick_l3": (course("l", 2, 0.0015, lift=0.005), None, None)},
     0.0, False),
    ("a spare brick held in the air away from the bridge", {**BRIDGE, "brick_l4": ((0.2, 0.3, TOP + 0.1), None, None)},
     0.0, False),
    ("bridge built, a spare brick dropped on the table in the gap",
     {**BRIDGE, "brick_r4": ((PIER_X, 0.0, TOP + H / 2), None, None)}, 0.0, False),
    ("a brick stood on end on the table in the gap as a prop",
     {**BRIDGE, "brick_l4": ((PIER_X, 0.0, TOP + L / 2), (BRICK[0], BRICK[2], BRICK[1]), None)}, 0.0, False),
    ("spare brick dropped on the table outside the gap (ignored)",
     {**BRIDGE, "brick_l4": ((0.45, 0.25, TOP + H / 2), None, None)}, 1.0, True),
    ("demo bridge but a top brick still moving", {**BRIDGE, "brick_r3": (course("r", 2, 0.0015), None, MOVING)}, 0.0, False),
    ("demo bridge with every brick turned 1.7 degrees (as the trace leaves them)",
     {n: (pos, None, None, turn(1.7)) for n, (pos, _s, _v) in BRIDGE.items()}, 1.0, True),
    ("demo bridge, left top brick turned 6 degrees", {**BRIDGE, "brick_l3": (course("l", 2, 0.0015), None, None, turn(6.0))},
     0.0, False),
    ("demo bridge, right top brick tilted 4 degrees", {**BRIDGE, "brick_r3": (course("r", 2, 0.0015), None, None, tilt(4.0))},
     0.0, False),
    ("left top brick held 1.5 mm above its course (it would drop)",
     {**BRIDGE, "brick_l3": (course("l", 2, 0.0015, lift=0.0015), None, None)}, 0.0, False),
    ("left top brick 0.8 mm above its course (within the 1 mm contact tolerance)",
     {**BRIDGE, "brick_l3": (course("l", 2, 0.0015, lift=0.0008), None, None)}, 1.0, True),
    ("a spare counterweight held 2 mm above the left top course",
     {**BRIDGE, "brick_l4": ((PIER_X, 0.09, PIER_TOP + 3 * H + H / 2 + 0.002), None, None)}, 0.0, False),
    ("capstone centred across two-course sides (their ends 50 mm from the middle)",
     {**side("l", DEMO[:2]), **side("r", DEMO[:2]), "brick_l3": ((PIER_X, 0.0, PIER_TOP + 2 * H + H / 2), None, None)},
     1.0, True),
    ("capstone 40 mm off centre across two-course sides (its centre of mass outside its supports)",
     {**side("l", DEMO[:2]), **side("r", DEMO[:2]), "brick_l3": ((PIER_X, 0.040, PIER_TOP + 2 * H + H / 2), None, None)},
     0.0, False),
    ("left top brick on its 40 mm side (rolled 90 degrees)",
     {**BRIDGE, "brick_l3": ((PIER_X, 0.0015 + L / 2, PIER_TOP + 2 * H + 0.020), None, None, ry(90))}, 0.0, False),
    ("equal 30 mm steps (centre of mass 0.5 mm inside the pier's edge)",
     {**side("l", (0.0615, 0.0315, 0.0015)), **side("r", (0.0615, 0.0315, 0.0015))}, 1.0, True),
    ("two courses per side, ends 60 and 1.5 mm from the middle", {**side("l", (0.060, 0.0015)), **side("r", (0.060, 0.0015))},
     1.0, True),
    # Turned bricks: the scorer uses each brick's true end faces, not its widened bounding box.
    ("both top bricks turned 4.9 degrees, end faces truly 7 mm apart",
     {**side("l", DEMO), **side("r", DEMO), "brick_l3": top_turned("l", 0.0035, 4.9), "brick_r3": top_turned("r", 0.0035, 4.9)},
     0.5, False),
    ("both top bricks turned 3 degrees, end faces 4.8 mm apart",
     {**side("l", DEMO), **side("r", DEMO), "brick_l3": top_turned("l", 0.0024, 3), "brick_r3": top_turned("r", 0.0024, 3)},
     1.0, True),
    ("both top bricks turned 3 degrees, end faces 5.2 mm apart",
     {**side("l", DEMO), **side("r", DEMO), "brick_l3": top_turned("l", 0.0026, 3), "brick_r3": top_turned("r", 0.0026, 3)},
     0.5, False),
    ("top bricks turned 3 degrees, ends 4 mm apart, right side 80.2 mm out",
     {**side("l", DEMO), **side("r", DEMO), "brick_l3": top_turned("l", -0.0058, 3), "brick_r3": top_turned("r", 0.0098, 3)},
     1.0, True),
    ("top bricks turned 3 degrees, ends 4 mm apart, right side 79.8 mm out",
     {**side("l", DEMO), **side("r", DEMO), "brick_l3": top_turned("l", -0.0062, 3), "brick_r3": top_turned("r", 0.0102, 3)},
     0.5, False),
    ("top bricks turned 3 degrees, both sides 60.2 mm out",
     {**side("l", DEMO), **side("r", DEMO), "brick_l3": top_turned("l", 0.0298, 3), "brick_r3": top_turned("r", 0.0298, 3)},
     0.5, False),
    ("top bricks turned 3 degrees, both sides 59.8 mm out",
     {**side("l", DEMO), **side("r", DEMO), "brick_l3": top_turned("l", 0.0302, 3), "brick_r3": top_turned("r", 0.0302, 3)},
     0.0, False),
    ("bricks stacked on the piers with no overhang", {**side("l", (0.09, 0.09, 0.09)), **side("r", (0.09, 0.09, 0.09))},
     0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        """As the simulator reports it: a rotated object's bounding box is the world-aligned box around it."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        half = np.abs(task._rot(q)) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            pos, size, vel = spec[:3]
            st[name] = state(pos, sizes[name] if size is None else size, vel, spec[3] if len(spec) > 3 else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
