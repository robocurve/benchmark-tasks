"""On the station table: two identical dark grey power strips (PDUs), fixed, each 20 cm long front to back, 6 cm wide
and 4 cm tall, with three square sockets (3.2 cm across, 2.5 cm deep, 6 cm apart) in a row along its length. The left
strip (PDU A) lies in front of the left arm, its centre 30 cm in front of the arm bases and 27 cm left of centre; the
right strip (PDU B) mirrors it in front of the right arm. Between the arms, near them, a fixed black staging tray
(15 cm front to back, 12 cm wide, 1 cm thick) holds six plug heads (2.5 x 2.5 x 4.5 cm, 50 g each) standing upright
in two columns of three, 15, 19.5 and 24 cm in front of the arm bases and 3 cm either side of centre. Each colour is
one server's two power plugs. The left column holds, from the arms outward, red, red, green; the right column green,
blue, blue. Done, each strip holds one red, one green and one blue plug, each down in its own socket.
"""

import math

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The power strips: a 15 mm base plate, and walls 25 mm tall around three 32 mm square sockets in a row along x.
PDU_X, PDU_A_Y, PDU_B_Y = 0.30, 0.27, -0.27
PDU_LEN, PDU_W, PDU_H, PDU_BASE_T = 0.20, 0.06, 0.04, 0.015
SOCKET = 0.032  # square opening
SOCKET_DEPTH = PDU_H - PDU_BASE_T  # 25 mm
SOCKET_X = (0.24, 0.30, 0.36)  # socket centres, 6 cm apart
SOCKET_FLOOR = TOP + PDU_BASE_T
PDU_TOP = TOP + PDU_H
PDU_GREY = (0.24, 0.25, 0.28)
STRIPS = {"A": PDU_A_Y, "B": PDU_B_Y}


def _strip(prefix, y):
    wall_t = (PDU_W - SOCKET) / 2  # 14 mm side walls
    wz = PDU_TOP - SOCKET_DEPTH / 2
    parts = [
        Box(f"{prefix}_base", pos=(PDU_X, y, TOP + PDU_BASE_T / 2), size=(PDU_LEN, PDU_W, PDU_BASE_T), color=PDU_GREY),
        Box(f"{prefix}_side_left", pos=(PDU_X, y + SOCKET / 2 + wall_t / 2, wz), size=(PDU_LEN, wall_t, SOCKET_DEPTH),
            color=PDU_GREY),
        Box(f"{prefix}_side_right", pos=(PDU_X, y - SOCKET / 2 - wall_t / 2, wz), size=(PDU_LEN, wall_t, SOCKET_DEPTH),
            color=PDU_GREY),
    ]
    edges = [PDU_X - PDU_LEN / 2]
    for sx in SOCKET_X:
        edges += [sx - SOCKET / 2, sx + SOCKET / 2]
    edges.append(PDU_X + PDU_LEN / 2)
    for i in range(0, len(edges), 2):  # the solid parts between and beyond the sockets
        x0, x1 = edges[i], edges[i + 1]
        parts.append(Box(f"{prefix}_divider_{i // 2}", pos=((x0 + x1) / 2, y, wz), size=(x1 - x0, SOCKET, SOCKET_DEPTH),
                         color=PDU_GREY))
    return parts


PDU_A = _strip("pdu_a", PDU_A_Y)
PDU_B = _strip("pdu_b", PDU_B_Y)

# The staging tray on the midline, where both arms reach.
TRAY_X, TRAY_SIZE, TRAY_T = 0.195, (0.15, 0.12), 0.010
TRAY_TOP = TOP + TRAY_T
TRAY = Box("staging_tray", pos=(TRAY_X, 0.0, TOP + TRAY_T / 2), size=(*TRAY_SIZE, TRAY_T), color=(0.08, 0.08, 0.09))

# The plugs: rigid plug heads standing in for the cord ends.
PLUG_W, PLUG_H, PLUG_KG = 0.025, 0.045, 0.05
ROW_X = (0.15, 0.195, 0.24)  # 4.5 cm apart, from the arms outward
COL_Y = (0.03, -0.03)  # the left and right columns, 6 cm apart
COLOURS = {"red": (0.85, 0.10, 0.10), "green": (0.10, 0.62, 0.20), "blue": (0.12, 0.30, 0.90)}
PLUG_START = {
    "red_1": (ROW_X[0], COL_Y[0]), "red_2": (ROW_X[1], COL_Y[0]), "green_1": (ROW_X[2], COL_Y[0]),
    "green_2": (ROW_X[0], COL_Y[1]), "blue_1": (ROW_X[1], COL_Y[1]), "blue_2": (ROW_X[2], COL_Y[1]),
}  # each arm's own column alone puts both reds on PDU A and both blues on PDU B
PLUG_NAMES = tuple(PLUG_START)
PAIRS = {c: (f"{c}_1", f"{c}_2") for c in COLOURS}
PLUGS = [
    Box(name, pos=(x, y, TRAY_TOP + PLUG_H / 2 + 0.001), size=(PLUG_W, PLUG_W, PLUG_H), mass=PLUG_KG,
        color=COLOURS[name.split("_")[0]])
    for name, (x, y) in PLUG_START.items()
]  # each starts 1 mm above the tray, and settles

OBJECTS = [*PDU_A, *PDU_B, TRAY, *PLUGS]
SUCCESS_HOLD_S = 2.5  # all six plugged in and still for 2.5 s

SEAT_XY_TOL = 0.005  # a seated plug's centre is within 5 mm of the socket's centre in x and in y
SEATED_Z = SOCKET_FLOOR + PLUG_H / 2  # a plug's centre height when it stands on the socket floor
SEAT_Z_TOL = 0.005  # and its centre within 5 mm of SEATED_Z (its top at most 2.5 cm above the strip's top)
SEAT_TILT_DEG = 10.0  # and its long axis is within 10 degrees of vertical, either end up
SPLIT_CREDIT, ONE_SEATED_CREDIT, BALANCED_CREDIT = 0.3, 0.1, 0.1


def _tilt_deg(state) -> float:
    """Angle between the object's long (z) axis and the vertical, either end up: a plug head is a plain box in one
    colour, so one stood on its other end looks and fits the same."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _seat(state):
    """The socket a plug is down in, as (strip, index), or None: its centre within SEAT_XY_TOL of the socket's
    centre, its centre height within SEAT_Z_TOL of SEATED_Z, its long axis within SEAT_TILT_DEG of vertical (either
    end up). Judged on the centre,
    not the bounding box: the simulator pads a loose object's bounding box by its 5 mm contact offset."""
    x, y, z = (float(v) for v in state.pos)
    if abs(z - SEATED_Z) > SEAT_Z_TOL or _tilt_deg(state) > SEAT_TILT_DEG:
        return None
    for strip, sy in STRIPS.items():
        for i, sx in enumerate(SOCKET_X):
            if abs(x - sx) <= SEAT_XY_TOL and abs(y - sy) <= SEAT_XY_TOL:
                return (strip, i)
    return None


def score(objects):
    """Partial credit (success scores 1.0): per colour (server), 0.3 when both its plugs are seated, one in each
    strip, or 0.1 when exactly one is seated; 0.1 more when all six sockets hold a plug. 0 overall if both plugs of
    any colour are seated in the same strip."""
    seats = {n: _seat(objects[n]) for n in PLUG_NAMES}
    total = 0.0
    for a, b in PAIRS.values():
        sa, sb = seats[a], seats[b]
        if sa and sb:
            if sa[0] == sb[0]:
                return 0.0  # both cords on one feed: the server loses power when that feed is cut
            total += SPLIT_CREDIT
        elif sa or sb:
            total += ONE_SEATED_CREDIT
    if len({s for s in seats.values() if s}) == len(STRIPS) * len(SOCKET_X):
        total += BALANCED_CREDIT
    return round(total, 6)


def success(objects):
    """Every server on both strips, three plugs per strip; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in PLUG_NAMES)


# The demo: the arms take turns at the tray. Each grips a plug straight down across its width (fingers closing along
# world y), carries it to its own strip and lowers it into a socket. Each arm takes one red, one green and one blue,
# so it reaches into the other column once.
GRIP_OPEN = -0.022  # finger joint: the pads open 44 mm, so the open fingertips clear the other column's plugs
GRIP_Z = 0.040  # TCP above a plug's bottom: the pads cover its top 18 mm, the tips stay 7 mm above the strip
DROP = 0.006  # each plug is let go this far above the socket floor
TRAVEL_Z = TOP + 0.14  # a carried plug's bottom clears the standing plugs by 4.5 cm
PICK_Z = TRAY_TOP + GRIP_Z
PLACE_Z = SOCKET_FLOOR + DROP + GRIP_Z
ENTRY_Z = PDU_TOP + 0.008 + GRIP_Z  # the plug's bottom 8 mm above the strip: stop and settle before going in
# The arm lags sideways while the TCP moves fast vertically (a plug lowered in 1 s ended 7-10 mm toward the arm and
# caught the socket's near rim), so the plug stops above the socket and eases in slowly.
PLAN = {
    "left": [("red_1", ("A", 0)), ("blue_1", ("A", 1)), ("green_1", ("A", 2))],
    "right": [("red_2", ("B", 0)), ("green_2", ("B", 1)), ("blue_2", ("B", 2))],
}
CYCLE_GAP_S = 2.0  # each arm waits above its strip between plugs, while the other arm is at the tray
RIGHT_WAIT_S = 6.65  # the right arm starts half a cycle (13.3 s) after the left
LEFT_PARK = (0.15, 0.32, TRAVEL_Z)  # in front of each arm's base, clear of the strips and the tray
RIGHT_PARK = (0.15, -0.32, TRAVEL_Z)


def _socket_xy(seat):
    strip, i = seat
    return (SOCKET_X[i], STRIPS[strip])


def _cycle(name, seat):
    p, q = PLUG_START[name], _socket_xy(seat)
    return [
        Waypoint(1.5, (*p, TRAVEL_Z), GRIP_OPEN, f"to {name}"),
        Waypoint(1.2, (*p, PICK_Z), GRIP_OPEN, "descend"),
        Waypoint(0.6, (*p, PICK_Z), FINGER_CLOSED, "close"),
        Waypoint(1.0, (*p, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (*q, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.0, (*q, ENTRY_Z), FINGER_CLOSED, "lower to just above the strip"),
        Waypoint(0.6, (*q, ENTRY_Z), FINGER_CLOSED, "settle above the socket"),
        Waypoint(2.0, (*q, PLACE_Z), FINGER_CLOSED, "ease into the socket"),
        Waypoint(0.3, (*q, PLACE_Z), FINGER_CLOSED, "hold"),
        Waypoint(0.5, (*q, PLACE_Z), GRIP_OPEN, "release"),
        Waypoint(0.8, (*q, TRAVEL_Z), GRIP_OPEN, "lift clear"),
    ]


def _arm(jobs):
    out = []
    for k, (name, seat) in enumerate(jobs):
        out += _cycle(name, seat)
        if k < len(jobs) - 1:
            out.append(Waypoint(CYCLE_GAP_S, (*_socket_xy(seat), TRAVEL_Z), GRIP_OPEN, "wait for the other arm"))
    return out


def demo():
    left = _arm(PLAN["left"])
    left.append(Waypoint(2.0, LEFT_PARK, GRIP_OPEN, "park"))
    right = [Waypoint(RIGHT_WAIT_S, (PDU_X, PDU_B_Y, TRAVEL_Z), GRIP_OPEN, "wait above PDU B")]
    right += _arm(PLAN["right"])
    right.append(Waypoint(2.5, RIGHT_PARK, GRIP_OPEN, "park"))
    return {"left": left, "right": right}
