"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Give a quaternion for every tilted or turned case.
"""

import math

from task import BOXES, CARD_SIZE, PAD_TOP, PADS, RIM_Z, SIZE, T, TARGET, TOP


def rot(axis, deg):
    s = math.sin(math.radians(deg) / 2)
    return (s * axis[0], s * axis[1], s * axis[2], math.cos(math.radians(deg) / 2))


X, Z = (1, 0, 0), (0, 0, 1)
MOVING = (0.0, 0.0, -0.05)


def card(pad, dz=0.0, vel=None, quat=None):
    px, py = PADS[pad]
    spec = ((px, py, PAD_TOP + CARD_SIZE[2] / 2 + dz), None, vel)
    return {"red_card": spec if quat is None else (*spec, quat)}


def packed(names=("blue_a", "blue_b", "blue_c")):
    return {n: ((*TARGET[n], TOP + T / 2), None, None) for n in names}


OB = BOXES["orange"]
ORANGE_IN = ((OB[0], OB[1], TOP + T / 2), None, None)
FULL = {**packed(), **card("orange")}
# Leaning at 20 deg against the outside of the orange box's near wall, bottom edge on the table.
_h = SIZE["orange_a"]
LEAN_20 = ((OB[0] - OB[2] / 2 - 0.005 - 0.020, -0.100, TOP + 0.5 * (_h[1] * math.sin(math.radians(20)) + T * math.cos(math.radians(20)))),
           None, None, rot(X, 20))


def tilt_on_floor(name, deg, x, y):
    """A block tilted deg about x with its lowest corner on the table."""
    sx, sy, sz = SIZE[name]
    a = math.radians(deg)
    return ((x, y, TOP + 0.5 * (sy * math.sin(a) + sz * math.cos(a))), None, None, rot(X, deg))


def tilt_long(name, deg, x, y):
    """A block tilted deg about y (along its front-to-back length) with its lowest edge on the table."""
    sx, sy, sz = SIZE[name]
    a = math.radians(deg)
    return ((x, y, TOP + 0.5 * (sx * math.sin(a) + sz * math.cos(a))), None, None, rot((0, 1, 0), deg))


CASES = [
    ("blue packed, orange flagged", FULL, 1.0, True),
    ("always flag: card on the orange pad, nothing packed", card("orange"), 0.5, False),
    ("always pack: blue packed, no card", packed(), 0.5, False),
    ("blue packed, but the card on the blue pad", {**packed(), **card("blue")}, 0.0, False),
    ("card on the blue pad, nothing packed", card("blue"), 0.0, False),
    ("two blue packed, orange flagged", {**packed(("blue_a", "blue_c")), **card("orange")}, 0.5 + 0.5 * 2 / 3, False),
    ("one blue packed, orange flagged", {**packed(("blue_c",)), **card("orange")}, 0.5 + 0.5 / 3, False),
    ("orange flagged, one orange block packed flat in its box",
     {**FULL, "orange_d": ORANGE_IN}, 0.75, False),
    ("orange flagged, an orange block on edge in its box (above the rim)",
     {**FULL, "orange_a": ((OB[0], OB[1], TOP + 0.040), None, None, rot(X, 90))}, 0.5, False),
    ("orange flagged, orange blocks stacked in the box",
     {**FULL, "orange_d": ORANGE_IN, "orange_c": ((OB[0], OB[1], TOP + 1.5 * T), None, None)}, 0.5, False),
    ("orange packed as far as it goes, no card", {**packed(), "orange_a": ORANGE_IN}, 0.5, False),
    ("a blue block on edge in the blue box (above the rim): only that block loses credit",
     {**FULL, "blue_b": ((*TARGET["blue_b"], TOP + 0.030), None, None, rot(X, 90))}, 0.5 + 0.5 * 2 / 3, False),
    ("blue C propped 2 deg along its length (its top 34.9 mm, above the rim)",
     {**FULL, "blue_c": tilt_long("blue_c", 2.0, *TARGET["blue_c"])}, 0.5 + 0.5 * 2 / 3, False),
    ("blue C propped 1.5 deg along its length (top 33.6 mm, below the rim)",
     {**FULL, "blue_c": tilt_long("blue_c", 1.5, *TARGET["blue_c"])}, 1.0, True),
    ("an orange block lying flat on the blue flag pad (pad counts as table)",
     {**FULL, "orange_d": ((PADS["blue"][0], PADS["blue"][1], PAD_TOP + T / 2), None, None)}, 1.0, True),
    ("a blue block exactly 5 deg off flat in its box",
     {**FULL, "blue_c": tilt_on_floor("blue_c", 5.0, *TARGET["blue_c"])}, 1.0, True),
    ("a blue block 6 deg off flat (resting on a neighbour's edge)",
     {**FULL, "blue_c": tilt_on_floor("blue_c", 6.0, *TARGET["blue_c"])}, 0.5 + 0.5 * 2 / 3, False),
    ("a blue block turned 3 deg about the vertical in its box",
     {**FULL, "blue_c": ((*TARGET["blue_c"], TOP + T / 2), None, None, rot(Z, 3))}, 1.0, True),
    ("a blue block held 1.5 mm above the box floor",
     {**FULL, "blue_a": ((*TARGET["blue_a"], TOP + T / 2 + 0.0015), None, None)}, 0.5 + 0.5 * 2 / 3, False),
    ("a blue block exactly 1.0 mm above the box floor",
     {**FULL, "blue_a": ((*TARGET["blue_a"], TOP + T / 2 + 0.001), None, None)}, 1.0, True),
    ("a blue block still moving in its box",
     {**FULL, "blue_a": ((*TARGET["blue_a"], TOP + T / 2), None, MOVING)}, 0.5 + 0.5 * 2 / 3, False),
    ("a blue block packed in the orange box", {**packed(("blue_a", "blue_c")), **card("orange"),
                                               "blue_b": ORANGE_IN}, 0.5 + 0.5 * 2 / 3, False),
    ("an orange block packed in the blue box (all blue packed)",
     {**FULL, "orange_d": ((BOXES["blue"][0], BOXES["blue"][1] - 0.01, TOP + T / 2 + 0.0), None, None)}, 0.5, False),
    ("an orange block leaning 20 deg against the outside of its box", {**FULL, "orange_a": LEAN_20}, 0.5, False),
    ("an orange block outside, exactly 5 deg off flat",
     {**FULL, "orange_a": tilt_on_floor("orange_a", 5.0, 0.160, -0.100)}, 1.0, True),
    ("an orange block outside, 6 deg off flat",
     {**FULL, "orange_a": tilt_on_floor("orange_a", 6.0, 0.160, -0.100)}, 0.5, False),
    ("an orange block moved elsewhere on the table, flat (allowed)",
     {**FULL, "orange_a": ((0.20, -0.02, TOP + T / 2), None, None, rot(Z, 40))}, 1.0, True),
    ("an orange block knocked off the table", {**FULL, "orange_b": ((0.16, -0.205, TOP - 0.30), None, MOVING)},
     0.5, False),
    ("card exactly 1.0 mm above the orange pad", {**packed(), **card("orange", dz=0.001)}, 1.0, True),
    ("card 1.5 mm above the orange pad (held)", {**packed(), **card("orange", dz=0.0015)}, 0.5, False),
    ("card on the orange pad on its side", {**packed(), **card("orange", dz=(CARD_SIZE[0] - CARD_SIZE[2]) / 2,
                                                         quat=rot((0, 1, 0), 90))}, 1.0, True),
    ("card on the orange pad, still moving", {**packed(), **card("orange", vel=MOVING)}, 0.5, False),
    ("card just off the orange pad", {**packed(), "red_card": ((PADS["orange"][0] + 0.045, PADS["orange"][1],
                                                               TOP + CARD_SIZE[2] / 2), None, None)}, 0.5, False),
    ("card dropped into the orange box", {**packed(), "red_card": ((OB[0], OB[1], TOP + CARD_SIZE[2] / 2), None, None)},
     0.5, False),
]


if __name__ == "__main__":
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "factory"))
    import check  # the factory checker's own state() builder

    import task

    sizes = {o.name: check.size_of(o) for o in task.OBJECTS}
    start = check.start_states(check.load(Path(__file__).with_name("task.py")))
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, size, vel, *quat) in changes.items():
            st[name] = check.state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if ok == want_success and abs(got - want) < 1e-9 else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    raise SystemExit(bad)
