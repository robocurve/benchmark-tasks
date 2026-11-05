"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Objects not listed stay where they started. The cases run in order and the scorer keeps trial history
(whether the storage box or plant pot has been moved), so a case that moves one affects the cases after it, as in a
real trial. The record clears only on a scene reset: every item back within 0.5 mm of its start and at rest, reached
by a jump of more than 20 mm in one step. RESET below is such a reset.
"""

import numpy as np

from task import BIN, CARTON, CHAIR, HANDLE_STRIP, HINGE, MOVES, POT, ROOM_X0, STORAGE, TOP

MOVING = (0.0, 0.0, -0.05)
H = {o.name: (o.size[2] if hasattr(o, "size") else o.height) for o in (CHAIR, BIN, CARTON, STORAGE, POT)}
HALF_XY = {CHAIR.name: 0.025, BIN.name: 0.025, CARTON.name: 0.020, STORAGE.name: 0.025, POT.name: 0.020}


def yaw_q(deg):
    a = np.radians(deg) / 2
    return (0.0, 0.0, float(np.sin(a)), float(np.cos(a)))


def tilt_q(deg):
    a = np.radians(deg) / 2
    return (float(np.sin(a)), 0.0, 0.0, float(np.cos(a)))


def at(o, x, y, up=0.0, vel=None, q=None):
    """Item o standing on the floor (its base `up` above the table) with its centre over (x, y)."""
    spec = ((x, y, TOP + up + H[o.name] / 2), None, vel)
    return {o.name: spec if q is None else (*spec, q)}


def tilted(o, x, y, deg):
    """Item o tilted about x by `deg`, its lowest corner on the floor."""
    a = np.radians(deg)
    hy = HALF_XY[o.name]
    z = TOP + abs(np.cos(a)) * H[o.name] / 2 + abs(np.sin(a)) * hy
    return {o.name: ((x, y, z), None, None, tilt_q(deg))}


def moved(*names):
    """The demo's end positions for the named blockers."""
    out = {}
    for o, (x, y) in MOVES:
        if o.name in names:
            out.update(at(o, x, y))
    return out


ALL = moved("chair", "waste_bin", "carton")
RESET = [("(scene reset: chair seen 50 mm up)", at(CHAIR, *CHAIR.pos[:2], up=0.050), 0.0, False),
         ("(scene reset: back at the start in one step)", {}, 0.0, False)]
# The handle ends up just past the open door's line, in the strip HANDLE_STRIP; a 40 mm carton centred over the
# strip's middle, its near side 5 mm (clear) or 4.9 mm (in the way) beyond the strip's far edge.
SX = (HANDLE_STRIP[0] + HANDLE_STRIP[1]) / 2

CASES = [
    ("all three blockers moved clear (demo end)", ALL, 1.0, True),
    ("chair and bin cleared, carton still in the swing", moved("chair", "waste_bin"), 2 / 3, False),
    ("only the chair cleared", moved("chair"), 1 / 3, False),
    ("only the carton cleared", moved("carton"), 1 / 3, False),
    ("nothing moved", {}, 0.0, False),
    ("all cleared but the carton still sliding", {**ALL, **at(CARTON, *MOVES[2][1], vel=MOVING)}, 0.0, False),
    # The handle strip: past the open door's line, but where the handle ends up.
    ("carton 5 mm past the open door's line, where the handle ends up", {**ALL, **at(CARTON, SX, HINGE[1] + 0.025)},
     2 / 3, False),
    ("carton exactly 5 mm beyond the handle strip", {**ALL, **at(CARTON, SX, HANDLE_STRIP[3] + 0.025)}, 1.0, True),
    ("carton 4.9 mm beyond the handle strip", {**ALL, **at(CARTON, SX, HANDLE_STRIP[3] + 0.0249)}, 2 / 3, False),
    # The arc: carton on the 45-degree line from the hinge, its nearest corner 5 mm / 4.9 mm outside the radius.
    ("carton corner exactly 5 mm outside the arc",
     {**ALL, **at(CARTON, *(HINGE - (0.155 + 0.020 * np.sqrt(2)) / np.sqrt(2)))}, 1.0, True),
    ("carton corner 4.9 mm outside the arc",
     {**ALL, **at(CARTON, *(HINGE - (0.1549 + 0.020 * np.sqrt(2)) / np.sqrt(2)))}, 2 / 3, False),
    # Rotation: a turned chair's corners reach further than its square box.
    ("chair turned 45 degrees at its demo spot", {**ALL, **at(CHAIR, 0.18, 0.18, q=yaw_q(45))}, 1.0, True),
    ("chair beyond the end of the arc, square: 15 mm clear", {**ALL, **at(CHAIR, 0.21, HINGE[1])}, 1.0, True),
    ("same spot turned 45 degrees: its corner is 4.6 mm from the arc",
     {**ALL, **at(CHAIR, 0.21, HINGE[1], q=yaw_q(45))}, 2 / 3, False),
    ("chair moved but still in the swing", {**ALL, **at(CHAIR, 0.33, 0.12)}, 2 / 3, False),
    ("chair moved behind the hinge, over the left tape", {**ALL, **at(CHAIR, 0.33, 0.29 - 0.025)}, 0.0, False),
    ("chair moved behind the hinge, inside the room", {**ALL, **at(CHAIR, 0.29, 0.28 - 0.025)}, 1.0, True),
    # Standing and on the floor.
    ("bin cleared but tilted 15 degrees", {**ALL, **tilted(BIN, 0.18, 0.08, 15)}, 0.0, False),
    ("bin cleared, tilted 8 degrees (still standing)", {**ALL, **tilted(BIN, 0.18, 0.08, 8)}, 1.0, True),
    ("bin knocked over, lying clear of the swing", {**ALL, BIN.name: ((0.18, 0.08, TOP + 0.025), None, None,
                                                                         tilt_q(90))}, 0.0, False),
    ("chair base exactly 1 mm above the floor", {**ALL, **at(CHAIR, 0.18, 0.18, up=0.001)}, 1.0, True),
    ("chair held 1.5 mm above the floor", {**ALL, **at(CHAIR, 0.18, 0.18, up=0.0015)}, 0.0, False),
    ("chair held 30 mm up, clear of the swing", {**ALL, **at(CHAIR, 0.18, 0.18, up=0.030)}, 0.0, False),
    ("carton stacked on the chair", {**ALL, **at(CARTON, 0.18, 0.18, up=H["chair"])}, 0.0, False),
    # Inside the room.
    ("carton edge exactly on the near tape line", {**ALL, **at(CARTON, ROOM_X0 + 0.020, -0.03)}, 1.0, True),
    ("carton 1 mm over the near tape", {**ALL, **at(CARTON, ROOM_X0 + 0.019, -0.03)}, 0.0, False),
    ("bin pushed out of the room toward the robot", {**ALL, **at(BIN, 0.05, 0.08)}, 0.0, False),
    ("bin pushed off the table", {**ALL, BIN.name: ((0.18, 0.08, TOP - 0.30), None, None)}, 0.0, False),
    # The items clear of the swing: 10 mm of movement is allowed, more fails the whole trial.
    ("all cleared, storage box nudged exactly 10 mm", {**ALL, **at(STORAGE, STORAGE.pos[0] - 0.010, STORAGE.pos[1])},
     1.0, True),
    ("all cleared, plant pot nudged 8 mm and turned 20 degrees",
     {**ALL, **at(POT, POT.pos[0] - 0.008, POT.pos[1], q=yaw_q(20))}, 1.0, True),
    ("all cleared, storage box nudged 9.9 mm toward the hinge: now in the door's way",
     {**ALL, **at(STORAGE, *(np.asarray(STORAGE.pos[:2]) + 0.0099 / np.sqrt(2)))}, 0.0, False),
    *RESET,
    ("all cleared, storage box pushed 10.1 mm", {**ALL, **at(STORAGE, STORAGE.pos[0] - 0.0101, STORAGE.pos[1])},
     0.0, False),
    ("then the storage box put back exactly", ALL, 0.0, False),
    *RESET,
    ("all cleared, plant pot lifted 15 mm and set back", {**ALL, **at(POT, *POT.pos[:2], up=0.015)}, 0.0, False),
    ("then the pot back on its spot", ALL, 0.0, False),
    *RESET,
    ("all cleared, plant pot tipped 12 degrees in place", {**ALL, **tilted(POT, *POT.pos[:2], 12)}, 0.0, False),
    *RESET,
    ("clear the whole corner: storage box and pot moved too",
     {**ALL, **at(STORAGE, 0.18, -0.12), **at(POT, 0.25, -0.15)}, 0.0, False),
    *RESET,
    ("misread the hinge side: storage box moved, blockers left", at(STORAGE, 0.18, -0.12), 0.0, False),
    *RESET,
    ("demo end again after a reset", ALL, 1.0, True),
]


def _run() -> int:
    """Score every case in order with task.py's own score() and success(); print and count mismatches."""
    import task
    from yam_arena.task_file import Box, ObjectState

    def size(o):
        return np.asarray(o.size, float) if isinstance(o, Box) else np.array([2 * o.radius] * 2 + [o.height])

    def state(pos, sz, vel=None, quat=None):
        """As the simulator reports it: a rotated object's world-aligned bounding box is wider than the object."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        x, y, z, w = q
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        half = np.abs(r) @ (np.asarray(sz, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: size(o) for o in task.OBJECTS}
    start = {o.name: state(o.pos, sizes[o.name]) for o in task.OBJECTS}
    task.success(start), task.score(start)  # a new trial: clears the history
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, sz, vel, *quat) in changes.items():
            st[name] = state(pos, sizes[name] if sz is None else sz, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if ok == want_success and abs(got - want) < 1e-9 else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
