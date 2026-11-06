"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready Part state}, expected
score, expected success).
"""

import math

import numpy as np

from task import FLOOR_Z, PIN_H, PIN_R, PIN_START, PX, PY, STATION, TOP, socket_xy


def arm(deg):
    return STATION.state(joints={"swing": math.radians(deg)})


def pin_in(deg, tilt=0.0, lift=0.0, vel=None, dxy=(0.0, 0.0)):
    """The pin standing in the socket at ``deg``: foot centre on the hole centre (+ dxy), leaning ``tilt`` deg
    about the world x axis, its lowest point ``lift`` above the socket floor."""
    a = math.radians(tilt)
    c = np.array((PX, PY)) + socket_xy(deg) + np.asarray(dxy)
    axis = np.array([0.0, -math.sin(a), math.cos(a)])
    low_rel = PIN_R * math.sin(a)  # lowest rim point sits this far below the foot centre
    foot = np.array([c[0], c[1], FLOOR_Z + lift + low_rel])
    centre = foot + PIN_H / 2 * axis
    quat = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))
    return (tuple(centre), None, vel, quat)


ON_PAD = 67.0  # the knob is over the pad for arm angles 56.6 to 78.5 deg
LYING = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))  # pin on its side, axis along y

CASES = [
    # full and partial credit
    ("pin in the 115 socket, arm swung onto the pad", {"stop_pin": pin_in(115), "stop_plate": arm(ON_PAD)}, 1.0,
     True),
    ("pin in the 115 socket, knob just on the pad's near edge (57 deg)",
     {"stop_pin": pin_in(115), "stop_plate": arm(57)}, 1.0, True),
    ("pin in the 115 socket, knob just on the pad's far edge (78 deg)",
     {"stop_pin": pin_in(115), "stop_plate": arm(78)}, 1.0, True),
    ("pin in the 115 socket leaning 15 deg, arm on the pad",
     {"stop_pin": pin_in(115, tilt=15), "stop_plate": arm(ON_PAD)}, 1.0, True),
    ("pin in the 115 socket pushed 4 mm toward a wall, arm on the pad",
     {"stop_pin": pin_in(115, dxy=(0.004, -0.004)), "stop_plate": arm(ON_PAD)}, 1.0, True),
    ("pin foot exactly 1 mm above the floor, arm on the pad",
     {"stop_pin": pin_in(115, lift=0.001), "stop_plate": arm(ON_PAD)}, 1.0, True),
    ("pin in the 115 socket, arm not swung", {"stop_pin": pin_in(115)}, 0.5, False),
    ("pin in the 115 socket, arm at 50 deg, short of the pad", {"stop_pin": pin_in(115), "stop_plate": arm(50)},
     0.5, False),
    ("pin in the 115 socket, arm left against the pin at 97 deg", {"stop_pin": pin_in(115), "stop_plate": arm(97)},
     0.5, False),
    ("pin in the 115 socket, arm exactly at 115 deg (not past it)",
     {"stop_pin": pin_in(115), "stop_plate": arm(115)}, 0.5, False),
    ("pin in the 115 socket, arm just past it at 115.01 deg",
     {"stop_pin": pin_in(115), "stop_plate": arm(115.01)}, 0.0, False),
    ("pin in the 70 socket (stops the arm at about 52 deg), arm not swung", {"stop_pin": pin_in(70)}, 0.0, False),
    ("pin in the 70 socket, arm against it at 52 deg", {"stop_pin": pin_in(70), "stop_plate": arm(52)}, 0.0,
     False),
    ("pin in the 70 socket, arm exactly at 70 deg", {"stop_pin": pin_in(70), "stop_plate": arm(70)}, 0.0, False),
    ("pin in the 70 socket, arm just past it at 70.01 deg", {"stop_pin": pin_in(70), "stop_plate": arm(70.01)}, 0.0,
     False),
    ("pin in the 30 socket, arm not swung", {"stop_pin": pin_in(30)}, 0.0, False),
    # wrong holes and wrong sides
    ("pin in the 160 socket (range crosses the walkway), arm on the pad",
     {"stop_pin": pin_in(160), "stop_plate": arm(ON_PAD)}, 0.0, False),
    ("pin in the 160 socket, arm not swung", {"stop_pin": pin_in(160)}, 0.0, False),
    ("arm swung past the 115 socket first, then the pin fitted behind it",
     {"stop_pin": pin_in(115), "stop_plate": arm(130)}, 0.0, False),
    ("pin in the 70 socket after the arm was swung past it to 100 deg",
     {"stop_pin": pin_in(70), "stop_plate": arm(100)}, 0.0, False),
    # not standing in a socket
    ("pin held 1.5 mm above the 115 socket floor, arm on the pad",
     {"stop_pin": pin_in(115, lift=0.0015), "stop_plate": arm(ON_PAD)}, 0.0, False),
    ("pin held 5 mm up in the 115 socket, arm on the pad",
     {"stop_pin": pin_in(115, lift=0.005), "stop_plate": arm(ON_PAD)}, 0.0, False),
    ("pin in the 115 socket but still moving", {"stop_pin": pin_in(115, vel=(0.0, 0.0, -0.05)),
                                               "stop_plate": arm(ON_PAD)}, 0.0, False),
    ("pin leaning 25 deg in the 115 socket", {"stop_pin": pin_in(115, tilt=25), "stop_plate": arm(ON_PAD)}, 0.0,
     False),
    ("pin standing on the plate between sockets, in the arm's path",
     {"stop_pin": pin_in(92.5), "stop_plate": arm(ON_PAD)}, 0.0, False),
    ("pin lying on the plate across the arm's path",
     {"stop_pin": ((PX + 0.06, PY + 0.01, FLOOR_Z + PIN_R), None, None, LYING), "stop_plate": arm(ON_PAD)},
     0.0, False),
    ("pin left on the table, arm swung onto the pad", {"stop_plate": arm(ON_PAD)}, 0.0, False),
    ("pin knocked over on the table", {"stop_pin": ((PIN_START[0], PIN_START[1], TOP + PIN_R), None, None, LYING)},
     0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        """As the simulator reports it: a rotated object's world-aligned bounding box is wider than the object."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        x, y, z, w = q
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        half = np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    def size(o):
        return np.array([2 * o.radius, 2 * o.radius, o.height]) if hasattr(o, "radius") else np.asarray(o.size)

    start = {o.name: o.state() if hasattr(o, "state") else state(o.pos, size(o)) for o in task.OBJECTS}
    sizes = {o.name: size(o) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            if isinstance(spec, ObjectState):
                st[name] = spec
                continue
            pos, sz, vel, *quat = spec
            st[name] = state(pos, sizes[name] if sz is None else sz, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
