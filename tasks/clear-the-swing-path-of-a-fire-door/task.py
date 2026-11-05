"""A scale model of a room on the station table. The table top is the room's floor. Across the far side stands a
light grey wall with a red fire door in it, closed. The door's black hinge post is at its left end (the robot's
left) and a silver handle is on the room side near its right end. Dark grey floor tape marks the room's other three
edges. Five loose items stand on the floor near the door: a blue block (a chair), a green cylinder (a waste bin), a
yellow carton, a purple storage box and a terracotta cylinder (a plant pot).
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# World frame: x away from the arm bases, y to the robot's left, z up (m).
WALL_X = 0.40  # centre plane of the wall and the closed door (10 mm thick)
WALL_T, WALL_H = 0.010, 0.120
DOOR_Y0, DOOR_Y1 = 0.05, 0.20  # the closed door spans this; the hinge is at DOOR_Y1
HINGE = np.array((WALL_X, DOOR_Y1))
DOOR_W = DOOR_Y1 - DOOR_Y0  # 150 mm: the radius the free edge sweeps
CLEAR = 0.005  # an item must stay at least 5 mm from the swept area
# The room's floor: from the tape on the near side to the wall's room-side face, between the side tapes.
ROOM_X0, ROOM_X1 = 0.10, WALL_X - WALL_T / 2
ROOM_Y0, ROOM_Y1 = -0.22, 0.28
TAPE_W, TAPE_T = 0.010, 0.001

HANDLE_POS = (WALL_X - WALL_T / 2 - 0.006, DOOR_Y0 + 0.025, TOP + 0.065)  # on the room side, near the free end
HANDLE_SIZE = (0.012, 0.040, 0.012)
RED, BLACK, SILVER = (0.72, 0.10, 0.08), (0.05, 0.05, 0.05), (0.78, 0.78, 0.80)
WALL_GREY, TAPE_GREY = (0.82, 0.82, 0.80), (0.25, 0.25, 0.27)

FIXED = [
    Box("fire_door", pos=(WALL_X, (DOOR_Y0 + DOOR_Y1) / 2, TOP + WALL_H / 2), size=(WALL_T, DOOR_W, WALL_H),
        color=RED),
    Cylinder("hinge_post", pos=(WALL_X, DOOR_Y1, TOP + WALL_H / 2), radius=0.008, height=WALL_H, color=BLACK),
    Box("door_handle", pos=HANDLE_POS, size=HANDLE_SIZE, color=SILVER),
    # Wall to the right of the door, and the short return to the left of the hinge post.
    Box("wall_right", pos=(WALL_X, (ROOM_Y0 - 0.01 + DOOR_Y0) / 2, TOP + WALL_H / 2),
        size=(WALL_T, DOOR_Y0 - ROOM_Y0 + 0.01, WALL_H), color=WALL_GREY),
    Box("wall_left", pos=(WALL_X, (DOOR_Y1 + 0.008 + ROOM_Y1 + 0.01) / 2, TOP + WALL_H / 2),
        size=(WALL_T, ROOM_Y1 + 0.01 - DOOR_Y1 - 0.008, WALL_H), color=WALL_GREY),
    # Floor tape, just outside the room's near and side edges.
    Box("tape_near", pos=(ROOM_X0 - TAPE_W / 2, (ROOM_Y0 + ROOM_Y1) / 2, TOP + TAPE_T / 2),
        size=(TAPE_W, ROOM_Y1 - ROOM_Y0 + 2 * TAPE_W, TAPE_T), color=TAPE_GREY),
    Box("tape_right", pos=((ROOM_X0 + ROOM_X1) / 2, ROOM_Y0 - TAPE_W / 2, TOP + TAPE_T / 2),
        size=(ROOM_X1 - ROOM_X0, TAPE_W, TAPE_T), color=TAPE_GREY),
    Box("tape_left", pos=((ROOM_X0 + ROOM_X1) / 2, ROOM_Y1 + TAPE_W / 2, TOP + TAPE_T / 2),
        size=(ROOM_X1 - ROOM_X0, TAPE_W, TAPE_T), color=TAPE_GREY),
]

# Items in the door's way at the start.
CHAIR = Box("chair", pos=(0.350, 0.165, TOP + 0.030), size=(0.050, 0.050, 0.060), mass=0.15, color=(0.15, 0.30, 0.85))
BIN = Cylinder("waste_bin", pos=(0.345, 0.085, TOP + 0.035), radius=0.025, height=0.070, mass=0.10,
               color=(0.20, 0.60, 0.25))
CARTON = Box("carton", pos=(0.275, 0.175, TOP + 0.025), size=(0.040, 0.040, 0.050), mass=0.08,
             color=(0.90, 0.80, 0.20))
# Items near the door but clear of its swing.
STORAGE = Box("storage_box", pos=(0.260, 0.060, TOP + 0.020), size=(0.050, 0.050, 0.040), mass=0.10,
              color=(0.55, 0.25, 0.65))
POT = Cylinder("plant_pot", pos=(0.340, 0.255, TOP + 0.025), radius=0.020, height=0.050, mass=0.10,
               color=(0.80, 0.40, 0.20))

BLOCKERS = (CHAIR, BIN, CARTON)
KEEP = (STORAGE, POT)
ITEMS = (*BLOCKERS, *KEEP)
SPEC = {o.name: o for o in ITEMS}
OBJECTS = [*FIXED, *ITEMS]

UPRIGHT_DEG = 10.0  # standing: its vertical axis within 10 degrees of vertical
FLOOR_TOL = 0.001  # on the floor: its lowest point at most 1 mm above the table top
KEEP_MOVE = 0.010  # an item clear of the swing must never move its centre more than 10 mm
RESET_TOL, RESET_JUMP = 0.0005, 0.020


def _rot(st) -> np.ndarray:
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(st) -> float:
    return float(np.degrees(np.arccos(np.clip(_rot(st)[2, 2], -1.0, 1.0))))


def _outline(st, spec) -> np.ndarray:
    """Every corner (box) or rim point (cylinder) of the item in its true pose, seen from above: (n, 2)."""
    if isinstance(spec, Box):
        h = np.asarray(spec.size) / 2
        local = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    else:
        a = np.linspace(0, 2 * np.pi, 72, endpoint=False)
        ring = np.c_[spec.radius * np.cos(a), spec.radius * np.sin(a)]
        local = np.vstack([np.c_[ring, np.full(len(a), z)] for z in (-spec.height / 2, spec.height / 2)])
    return (local @ _rot(st).T + np.asarray(st.pos))[:, :2]


def _hull(p: np.ndarray) -> np.ndarray:
    """Convex hull, counter-clockwise (monotone chain)."""
    pts = sorted(set(map(tuple, np.round(p, 9))))

    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def half(seq):
        out = []
        for q in seq:
            while len(out) >= 2 and turn(out[-2], out[-1], q) <= 0:
                out.pop()
            out.append(q)
        return out

    lower, upper = half(pts), half(reversed(pts))
    return np.array(lower[:-1] + upper[:-1])


def _handle_strip() -> tuple[float, float, float, float]:
    """The handle sticks out on the room side, so in the last few degrees of the swing it passes beyond the open
    door's line. The box (x0, x1, y0, y1) around every handle point that leaves the quarter disc that way."""
    gx, gy = np.meshgrid(np.linspace(-0.5, 0.5, 13), np.linspace(-0.5, 0.5, 41))
    pts = np.c_[HANDLE_POS[0] + gx.ravel() * HANDLE_SIZE[0], HANDLE_POS[1] + gy.ravel() * HANDLE_SIZE[1]] - HINGE
    out = []
    for phi in np.radians(np.linspace(0, 90, 901)):  # opening angle; turning clockwise seen from above
        c, s_ = np.cos(phi), np.sin(phi)
        q = np.c_[c * pts[:, 0] + s_ * pts[:, 1], -s_ * pts[:, 0] + c * pts[:, 1]] + HINGE
        out.append(q[q[:, 1] > HINGE[1]])
    q = np.vstack(out)
    return (round(float(q[:, 0].min()), 4), round(float(q[:, 0].max()), 4), float(HINGE[1]),
            round(float(q[:, 1].max()), 4))


HANDLE_STRIP = _handle_strip()  # (0.254, 0.295, 0.2, 0.217)


def _zone_distance_pts(p: np.ndarray) -> np.ndarray:
    """Distance from each point to the area the door sweeps: the quarter disc of radius 150 mm about the hinge
    axis, between the closed door's line and the line from the hinge straight into the room, plus the strip just
    past the open door's line that its handle sweeps at the end of the swing."""
    u = HINGE[0] - p[:, 0]  # into the room from the wall
    v = HINGE[1] - p[:, 1]  # from the hinge toward the door's free end
    r = np.hypot(u, v)
    disc = np.where(
        (u >= 0) & (v >= 0), np.maximum(r - DOOR_W, 0.0),
        np.where((u < 0) & (v >= 0), np.hypot(u, np.maximum(v - DOOR_W, 0.0)),
                 np.where((v < 0) & (u >= 0), np.hypot(v, np.maximum(u - DOOR_W, 0.0)), r)))
    x0, x1, y0, y1 = HANDLE_STRIP
    strip = np.hypot(np.maximum.reduce([x0 - p[:, 0], p[:, 0] - x1, np.zeros(len(p))]),
                     np.maximum.reduce([y0 - p[:, 1], p[:, 1] - y1, np.zeros(len(p))]))
    return np.minimum(disc, strip)


_a = np.linspace(0, np.pi / 2, 91)
_sx, _sy = np.linspace(HANDLE_STRIP[0], HANDLE_STRIP[1], 21), np.linspace(HANDLE_STRIP[2], HANDLE_STRIP[3], 9)
_ZONE_EDGE = np.vstack([
    HINGE - DOOR_W * np.c_[np.sin(_a), np.cos(_a)],  # the arc
    HINGE - np.c_[np.zeros(31), np.linspace(0, DOOR_W, 31)],  # the closed door's line
    HINGE - np.c_[np.linspace(0, DOOR_W, 31), np.zeros(31)],  # the fully open door's line
    np.c_[_sx, np.full(21, HANDLE_STRIP[3])], np.c_[np.full(9, HANDLE_STRIP[0]), _sy],  # the handle strip
    np.c_[np.full(9, HANDLE_STRIP[1]), _sy],
])


def _inside(poly: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Points inside a counter-clockwise convex polygon."""
    e = np.roll(poly, -1, axis=0) - poly
    rel = p[:, None, :] - poly[None, :, :]
    return np.all(e[None, :, 0] * rel[:, :, 1] - e[None, :, 1] * rel[:, :, 0] >= 0, axis=1)


def zone_distance(st, spec) -> float:
    """Gap (m) between the item's outline seen from above and the door's swept area; 0 if they overlap."""
    poly = _hull(_outline(st, spec))
    if np.any(_inside(poly, _ZONE_EDGE)):
        return 0.0
    edges = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.0005)) + 1)
        edges.append(a + np.linspace(0, 1, n)[:, None] * (b - a))
    return float(_zone_distance_pts(np.vstack(edges)).min())


def _blocks(st, spec) -> bool:
    return zone_distance(st, spec) < CLEAR - EPS


def _standing_in_room(st, spec) -> bool:
    """Upright, its lowest point within 1 mm of the table top, its whole outline inside the taped room, and at rest."""
    p = _outline(st, spec)
    return bool(
        _tilt_deg(st) <= UPRIGHT_DEG + EPS
        and TOP - FLOOR_TOL - EPS <= st.low[2] <= TOP + FLOOR_TOL + EPS
        and np.all((p[:, 0] >= ROOM_X0 - EPS) & (p[:, 0] <= ROOM_X1 + EPS)
                   & (p[:, 1] >= ROOM_Y0 - EPS) & (p[:, 1] <= ROOM_Y1 + EPS))
        and st.resting()
    )


# Trial history: whether an item clear of the door's swing has been moved. The scorer sees one step at a time, so
# it is kept here. It clears only on a scene reset: every item within 0.5 mm of its start and at rest, reached by a
# jump of more than 20 mm in one control step for some item. Putting things back by hand does not clear it.
_HISTORY = {"disturbed": False, "prev": None}


def _update(objects) -> None:
    now = {o.name: np.asarray(objects[o.name].pos, float).copy() for o in ITEMS}
    prev, _HISTORY["prev"] = _HISTORY["prev"], now
    jumped = prev is None or any(np.linalg.norm(now[n] - prev[n]) > RESET_JUMP for n in now)
    home = all(np.linalg.norm(now[o.name] - np.asarray(o.pos)) <= RESET_TOL + EPS and objects[o.name].resting()
               for o in ITEMS)
    if jumped and home:
        _HISTORY["disturbed"] = False
    for o in KEEP:
        st = objects[o.name]
        if (np.linalg.norm(now[o.name] - np.asarray(o.pos)) > KEEP_MOVE + EPS
                or _tilt_deg(st) > UPRIGHT_DEG + EPS):
            _HISTORY["disturbed"] = True


def _cleared(objects) -> int:
    """How many of the three items that started in the door's way are now clear of it. -1 if the trial fails."""
    _update(objects)
    if _HISTORY["disturbed"]:
        return -1
    if not all(_standing_in_room(objects[o.name], o) for o in ITEMS):
        return -1
    if any(_blocks(objects[o.name], o) for o in KEEP):  # a nudge under 10 mm that still put it in the way
        return -1
    return sum(not _blocks(objects[o.name], o) for o in BLOCKERS)


def success(objects):
    """All three items that were in the door's way are now at least 5 mm clear of its swept area; every item
    stands upright on the floor inside the room and at rest; the storage box and plant pot were never moved and
    are not in the door's way."""
    return _cleared(objects) == len(BLOCKERS)


def distance(objects):
    return float(sum(max(0.0, CLEAR - zone_distance(objects[o.name], o)) for o in BLOCKERS))


def score(objects):
    """1/3 for each of the chair, the bin and the carton now clear of the swing (1.0 for all three), provided every
    item stands upright on the floor in the room at rest and the storage box and plant pot are out of the way; 0 if
    not, and 0 for the whole trial once the storage box or plant pot has moved more than 10 mm or tipped."""
    n = _cleared(objects)
    return 0.0 if n <= 0 else (1.0 if n == len(BLOCKERS) else n / len(BLOCKERS))


# ---- scripted demo (left arm) ----------------------------------------------------------------------------
TIP_PAST_TCP = 0.013
GRIP_BELOW_TOP = 0.018  # finger tips 18 mm below each item's top
DROP = 0.007  # release with the item's base 7 mm above the floor
TRAVEL_Z = TOP + 0.14
MOVES = ((CHAIR, (0.180, 0.180)), (BIN, (0.180, 0.080)), (CARTON, (0.160, -0.030)))
# Fingers part open: pads 66 mm apart, outer faces about 100 mm apart, so they fit between neighbouring items.
OPEN_PART = -0.033


def _height(o) -> float:
    return o.size[2] if isinstance(o, Box) else o.height


def demo():
    w = []
    for o, (x, y) in MOVES:
        grip_z = TOP + _height(o) - GRIP_BELOW_TOP + TIP_PAST_TCP
        px, py = o.pos[:2]
        w += [
            Waypoint(1.5, (px, py, TRAVEL_Z), OPEN_PART, f"over the {o.name}"),
            Waypoint(1.5, (px, py, grip_z), OPEN_PART, "descend"),
            Waypoint(0.8, (px, py, grip_z), FINGER_CLOSED, "grip"),
            Waypoint(1.2, (px, py, TRAVEL_Z), FINGER_CLOSED, "lift"),
            Waypoint(1.8, (x, y, TRAVEL_Z), FINGER_CLOSED, "carry"),
            Waypoint(1.2, (x, y, grip_z + DROP), FINGER_CLOSED, "lower"),
            Waypoint(0.6, (x, y, grip_z + DROP), OPEN_PART, "release"),
            Waypoint(1.0, (x, y, TRAVEL_Z), OPEN_PART, "up"),
        ]
    return {"left": w}
