"""On the station table, in the left arm's lane, a 1 g bench analogue of tool control at closeout on a spacecraft
integration floor. A fixed white shadow board, 30 mm thick, carries five solid dark grey silhouettes (called outlines):
a wrench, a socket, an adapter, pliers and side cutters. The pliers and cutters outlines are the same 66 x 36 mm
rectangle, so shape and size do not tell them apart. A brass chit lies on the cutters outline, which means that tool
is signed out. The pliers outline is bare with no chit, so the pliers may be inside the vehicle and that outline must
be flagged. Five loose items stand in a row on the table: the wrench, the socket, the adapter, a stray bracket that
matches no outline (foreign object debris) and a red tag. Each tool goes on its own outline, the bracket goes in the
yellow FOD bin, and the red tag goes on the pliers outline, not on the excused one. The board is as tall as the
tallest loose item and the bin walls are taller than the bracket, so nothing scores without being lifted. Every
test uses each item's centre and quaternion; the lowest point comes from the body's own extents, not the
axis-aligned bounds.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/shadow-board-closeout/shadow_board_closeout.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Put each tool on its matching outline, the stray part that matches no outline in the yellow FOD bin, and the red tag on the outline whose tool is missing. A brass chit means that tool is signed out." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
BOARD_XY = (0.280, 0.305)
BOARD = (0.100, 0.280, 0.030)  # spans x 0.230 to 0.330 and y 0.165 to 0.445; as tall as the tallest loose item
BTOP = TOP + BOARD[2]  # the board's top face
LINE_T = 0.001  # every outline is a solid 1 mm dark grey silhouette (a filled inlay) on the board top
CHIT = (0.030, 0.024, 0.003)
CHIT_XY = (0.280, 0.190)  # on the cutters outline at its centre, resting on the inlay: the cutters are signed out

OUT_X = 0.280  # every outline is centred on this line
O_WRENCH = (OUT_X, 0.410)  # box 76 x 26 mm, long along x
O_SOCKET = (OUT_X, 0.355)  # disc 42 mm across
O_ADAPTER = (OUT_X, 0.300)  # square 46 x 46 mm
EMPTY_OUT = (0.066, 0.036)  # the pliers and the cutters outlines: one rectangle, long along x, so shape and size
# cannot tell the two empty outlines apart and only the chit does
O_PLIERS = (OUT_X, 0.240)  # its tool is not in the scene and there is no chit: the outline to flag
O_CUTTERS = (OUT_X, 0.190)  # its tool is signed out (the chit on it); the empty outline nearest the tag

BIN_XY = (0.140, 0.545)
BIN_IN = (0.130, 0.100)  # the open pocket between the walls, along x and along y; the table is its floor
BIN_T, BIN_H = 0.006, 0.035  # walls taller than the 30 mm bracket, so it cannot be pushed or tipped in

WRENCH = (0.020, 0.070, 0.020)  # long along y at the start: a quarter turn from its outline
SOCKET_R, SOCKET_H = 0.018, 0.030
ADAPTER = (0.040, 0.040, 0.025)
ADAPTER_YAW = math.radians(30.0)
BRACKET = (0.018, 0.036, 0.030)
TAG = (0.024, 0.050, 0.020)
ROW_X = 0.150
STARTS = {"wrench": (ROW_X, 0.405), "socket": (ROW_X, 0.330), "bracket": (ROW_X, 0.265), "adapter": (ROW_X, 0.195), "tag": (ROW_X, 0.120)}

WHITE = (0.92, 0.92, 0.90)
LINE = (0.20, 0.20, 0.22)
BRASS = (0.78, 0.62, 0.25)
YELLOW = (0.90, 0.80, 0.10)
STEEL = (0.55, 0.56, 0.58)
DARK_STEEL = (0.35, 0.35, 0.37)
BLUE = (0.20, 0.35, 0.70)
RED = (0.85, 0.10, 0.10)


def _bin_walls():
    cx, cy = BIN_XY
    ix, iy = BIN_IN
    z = TOP + BIN_H / 2
    side = (ix + 2 * BIN_T, BIN_T, BIN_H)
    end = (BIN_T, iy, BIN_H)
    return [
        Box("bin_wall_out", pos=(cx, cy + iy / 2 + BIN_T / 2, z), size=side, color=YELLOW),
        Box("bin_wall_in", pos=(cx, cy - iy / 2 - BIN_T / 2, z), size=side, color=YELLOW),
        Box("bin_wall_near", pos=(cx - ix / 2 - BIN_T / 2, cy, z), size=end, color=YELLOW),
        Box("bin_wall_far", pos=(cx + ix / 2 + BIN_T / 2, cy, z), size=end, color=YELLOW),
    ]


_line_z = BTOP + LINE_T / 2
FIXTURES = [
    Box("shadow_board", pos=(*BOARD_XY, TOP + BOARD[2] / 2), size=BOARD, color=WHITE),
    Box("outline_wrench", pos=(*O_WRENCH, _line_z), size=(0.076, 0.026, LINE_T), color=LINE),
    Cylinder("outline_socket", pos=(*O_SOCKET, _line_z), radius=0.021, height=LINE_T, color=LINE),
    Box("outline_adapter", pos=(*O_ADAPTER, _line_z), size=(0.046, 0.046, LINE_T), color=LINE),
    Box("outline_pliers", pos=(*O_PLIERS, _line_z), size=(*EMPTY_OUT, LINE_T), color=LINE),
    Box("outline_cutters", pos=(*O_CUTTERS, _line_z), size=(*EMPTY_OUT, LINE_T), color=LINE),
    Box("signout_chit", pos=(*CHIT_XY, BTOP + LINE_T + CHIT[2] / 2), size=CHIT, color=BRASS),
    *_bin_walls(),
]
LOOSE = [
    Box("wrench", pos=(*STARTS["wrench"], TOP + WRENCH[2] / 2), size=WRENCH, mass=0.045, color=STEEL),
    Cylinder("socket", pos=(*STARTS["socket"], TOP + SOCKET_H / 2), radius=SOCKET_R, height=SOCKET_H, mass=0.050, color=DARK_STEEL),
    Box("adapter", pos=(*STARTS["adapter"], TOP + ADAPTER[2] / 2), size=ADAPTER, mass=0.055, color=BLUE, yaw=ADAPTER_YAW),
    Box("bracket", pos=(*STARTS["bracket"], TOP + BRACKET[2] / 2), size=BRACKET, mass=0.025, color=STEEL),
    Box("tag", pos=(*STARTS["tag"], TOP + TAG[2] / 2), size=TAG, mass=0.012, color=RED),
]
OBJECTS = [*LOOSE, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

FLAT = math.cos(math.radians(5.0))  # a body axis within 5 degrees of vertical
BOARD_TOL = 0.004  # lowest point to the board top (a part resting on a 1 mm outline is 1 mm up)
HOME_TOL = 0.008  # a tool's centre to its outline's centre
TAG_TOL = 0.015  # the tag's centre to the pliers outline's centre
YAW_TOL = math.radians(10.0)
FLOOR_BAND = (-0.005, 0.010)  # the bracket's lowest point on the bin floor (the table)
POINTS = {"tool": 3, "bracket": 3, "tag": 8}  # out of 20: 0.15 per tool, 0.15 for the bracket, 0.40 for the tag
TOOLS = {"wrench": O_WRENCH, "socket": O_SOCKET, "adapter": O_ADAPTER}


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _bottom(name, state) -> float:
    """The lowest point from the centre and the rotation: a box's lowest corner, or a cylinder's lowest rim point.
    The axis-aligned bounds of a turned cylinder come from its box corners and read too low, so they are not used."""
    rot = _rot(state.quat_xyzw)
    if name == "socket":
        uz = abs(float(rot[2, 2]))
        return float(state.pos[2]) - (SOCKET_H / 2 * uz + SOCKET_R * math.sqrt(max(0.0, 1.0 - uz * uz)))
    half = np.asarray({"wrench": WRENCH, "adapter": ADAPTER, "bracket": BRACKET, "tag": TAG}[name]) / 2
    return float(state.pos[2]) - float(np.abs(rot[2]) @ half)


def _square(rot, axes) -> bool:
    """Resting square: one of the given body axes within 5 degrees of vertical, either end up."""
    return any(abs(float(rot[2, a])) >= FLAT for a in axes)


def _yaw_off(vec, period) -> float:
    """Angle of a body axis's horizontal projection from world x, folded to the outline's symmetry period."""
    a = math.atan2(float(vec[1]), float(vec[0])) % period
    return min(a, period - a)


def _near(state, xy, tol) -> bool:
    return math.hypot(float(state.pos[0]) - xy[0], float(state.pos[1]) - xy[1]) <= tol


def _on_board(name, state) -> bool:
    """Resting square on the board top (or on an outline inlay), not on a tool, not on the table."""
    return abs(_bottom(name, state) - BTOP) <= BOARD_TOL


def _home(name, state) -> bool:
    """A tool on its own outline: at rest, square on the board, centre within 8 mm of the outline's centre, and
    turned to match it (the wrench's long axis within 10 degrees of the outline's, either way round; the adapter's
    edges within 10 degrees of the outline's, any quarter turn; the socket upright, any yaw)."""
    if not state.resting() or not _on_board(name, state) or not _near(state, TOOLS[name], HOME_TOL):
        return False
    rot = _rot(state.quat_xyzw)
    if name == "wrench":  # either 20 mm face down; the long axis is body y
        return _square(rot, (0, 2)) and _yaw_off(rot[:, 1], math.pi) <= YAW_TOL
    if name == "adapter":  # a 40 x 40 face down, either way up
        return _square(rot, (2,)) and _yaw_off(rot[:, 0], math.pi / 2) <= YAW_TOL
    return _square(rot, (2,))  # the socket stands on either end


def _inside_bin(state) -> bool:
    return abs(float(state.pos[0]) - BIN_XY[0]) < BIN_IN[0] / 2 and abs(float(state.pos[1]) - BIN_XY[1]) < BIN_IN[1] / 2


def _in_bin(name, state) -> bool:
    """Any loose item in the bin: centre inside the inner walls seen from above, lowest point under the wall tops."""
    return _inside_bin(state) and _bottom(name, state) < TOP + BIN_H


def _binned(objects) -> bool:
    """The bracket at rest on the bin floor, and nothing else in the bin: a tool or the tag binned as debris voids it."""
    st = objects["bracket"]
    on_floor = FLOOR_BAND[0] <= _bottom("bracket", st) - TOP <= FLOOR_BAND[1]
    alone = not any(_in_bin(n, objects[n]) for n in ("wrench", "socket", "adapter", "tag"))
    return bool(st.resting() and _inside_bin(st) and on_floor and alone)


def _tagged(objects) -> bool:
    """The red tag at rest, square on the board (not on a tool), centre within 15 mm of the pliers outline's centre."""
    st = objects["tag"]
    return bool(st.resting() and _on_board("tag", st) and _square(_rot(st.quat_xyzw), (0, 1, 2)) and _near(st, O_PLIERS, TAG_TOL))


def _bracket_on_board(objects) -> bool:
    """The bracket anywhere over the board's footprint at or above its top. The board is the tool count: a part that
    is not its tool sitting on it can make a bare outline read as filled and hide the missing tool, so it voids the
    closeout, while the bracket left on the table is only unbinned."""
    st = objects["bracket"]
    x, y = float(st.pos[0]), float(st.pos[1])
    over = abs(x - BOARD_XY[0]) <= BOARD[0] / 2 and abs(y - BOARD_XY[1]) <= BOARD[1] / 2
    return bool(over and _bottom("bracket", st) >= BTOP - BOARD_TOL)


def _points(objects) -> int:
    if _bracket_on_board(objects):
        return 0
    pts = sum(POINTS["tool"] for n in TOOLS if _home(n, objects[n]))
    pts += POINTS["bracket"] if _binned(objects) else 0
    pts += POINTS["tag"] if _tagged(objects) else 0
    return pts


def success(objects) -> bool:
    """Three tools home, the bracket alone in the bin, the tag on the pliers outline, all at rest; holds for 1 s."""
    return _points(objects) == 20


def distance(objects) -> float:
    """Metres still to go: each tool to its outline, the bracket to the bin centre, the tag to the pliers outline."""
    goals = {**TOOLS, "bracket": BIN_XY, "tag": O_PLIERS}
    return sum(math.hypot(float(objects[n].pos[0]) - x, float(objects[n].pos[1]) - y) for n, (x, y) in goals.items())


def score(objects) -> float:
    """0.15 per tool home, 0.15 for the bracket alone on the bin floor, 0.40 for the tag on the pliers outline, so
    1.0 only with all five; 0 whatever else is right if the bracket is anywhere on the board, because the board is
    the tool count and a part that is not its tool sitting on it can make a bare outline read as filled and hide the
    missing tool."""
    return _points(objects) / 20.0


TRAVEL = TOP + 0.145
TCP_UP = 0.013  # the TCP sits 13 mm above the pad tips
TIP_UP = {"wrench": 0.005, "socket": 0.006, "adapter": 0.006, "bracket": 0.006, "tag": 0.005}  # pad tips over the part's bottom
WIDTH = {"wrench": WRENCH[0], "socket": 2 * SOCKET_R, "adapter": ADAPTER[0], "bracket": BRACKET[0], "tag": TAG[0]}
RELEASE = 0.005  # the part's bottom over its resting surface when the fingers open
BOARD_ENTRY = 0.010  # the pause over the board: part bottom 15 mm over the outlines, 12 mm over the chit
BIN_ENTRY = 0.040  # the pause over the bin: bracket bottom 10 mm over the 35 mm walls
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
ALONG_X = math.pi / 2  # gripper yaw that closes the fingers along x (yaw 0 closes them along y)
# name, place, entry, pick yaw, place yaw. Every pick closes along x or near it, so the row's neighbours (spaced
# along y) sit beside the pads. The wrist's last joint turns by about the base joint's angle minus the yaw and stops
# at 120 degrees either way, so the outboard parts are taken at +90 and the inboard ones at -90 (the same closing
# line). The wrench turns a quarter in the carry and is set down closing along y, before anything stands on the
# socket outline under the open fingers; the adapter closes across its faces at 30 degrees and is turned square;
# every other part keeps its yaw.
PLAN = (
    ("wrench", (*O_WRENCH, BTOP + LINE_T), BOARD_ENTRY, ALONG_X, 0.0),
    ("socket", (*O_SOCKET, BTOP + LINE_T), BOARD_ENTRY, ALONG_X, ALONG_X),
    ("bracket", (*BIN_XY, TOP), BIN_ENTRY, ALONG_X, ALONG_X),
    ("adapter", (*O_ADAPTER, BTOP + LINE_T), BOARD_ENTRY, ADAPTER_YAW - math.pi / 2, -ALONG_X),
    ("tag", (*O_PLIERS, BTOP + LINE_T), BOARD_ENTRY, -ALONG_X, -ALONG_X),
)


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), FINGER_OPEN, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.6, (px, py, pz), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), FINGER_OPEN, "retreat", end),
    ]


def demo():
    """Left arm only, five top-down pick and place moves at 14.5 cm travel height, in row order from outboard: the
    wrench onto its outline turned a quarter, the socket onto its disc, the bracket into the bin, the adapter onto
    its square turned 30 degrees and the tag onto the pliers outline. Each part is released 5 mm above its resting surface. 81.0 s of plan; the
    episode ends on success about 1 s after the last release (the scripted policy would otherwise stop after a 3 s
    hold, at step 1,680 at 20 Hz)."""
    left = []
    for name, (qx, qy, surface), entry, yaw, place_yaw in PLAN:
        sx, sy = STARTS[name]
        grip_z = TIP_UP[name] + TCP_UP
        left += _pick_place((sx, sy, TOP + grip_z), (qx, qy, surface + RELEASE + grip_z), width=WIDTH[name], entry=entry, travel_z=TRAVEL, yaw=yaw, place_yaw=place_yaw)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def qmul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])

    def quat(yaw=0.0, tilt=0.0, axis=0):  # tilt about a body axis (0 x, 1 y), then yaw about the vertical
        q_tilt = np.zeros(4)
        q_tilt[axis] = math.sin(tilt / 2)
        q_tilt[3] = math.cos(tilt / 2)
        return qmul(np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]), q_tilt)

    dims = {"wrench": WRENCH, "adapter": ADAPTER, "bracket": BRACKET, "tag": TAG}

    def part(name, x, y, bottom=TOP, yaw=0.0, tilt=0.0, axis=0, vel=(0.0, 0.0, 0.0)):
        """A part with its lowest point at ``bottom``; low/high from the rotated box corners, as the simulator
        reports them."""
        q = quat(yaw, tilt, axis)
        rot = _rot(q)
        if name == "socket":
            half = np.abs(rot) @ np.array([SOCKET_R, SOCKET_R, SOCKET_H / 2])
            uz = abs(float(rot[2, 2]))
            up = SOCKET_H / 2 * uz + SOCKET_R * math.sqrt(max(0.0, 1.0 - uz * uz))
        else:
            half = np.abs(rot) @ (np.asarray(dims[name]) / 2)
            up = half[2]
        pos = np.array([x, y, bottom + up])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    LIFT = BTOP + LINE_T  # resting on an outline inlay
    start = {n: part(n, x, y, yaw=ADAPTER_YAW if n == "adapter" else 0.0) for n, (x, y) in STARTS.items()}
    home = {
        "wrench": part("wrench", *O_WRENCH, bottom=LIFT, yaw=math.pi / 2),  # long axis (body y) along world x
        "socket": part("socket", *O_SOCKET, bottom=LIFT),
        "adapter": part("adapter", *O_ADAPTER, bottom=LIFT),
        "bracket": part("bracket", *BIN_XY),
        "tag": part("tag", *O_PLIERS, bottom=LIFT),
    }

    def scene(*names, **moved):
        """The named parts at home, the moved ones where given, the rest on their marks."""
        return {**start, **{n: home[n] for n in names}, **moved}

    ALL = ("wrench", "socket", "adapter", "bracket", "tag")
    TOOL3 = ("wrench", "socket", "adapter")
    ox, oy = O_PLIERS
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(*ALL), 1.0, True),
        "only the tag on the pliers outline": (scene("tag"), 0.40, False),
        "only the wrench home": (scene("wrench"), 0.15, False),
        "only the socket home": (scene("socket"), 0.15, False),
        "only the adapter home": (scene("adapter"), 0.15, False),
        "only the bracket binned": (scene("bracket"), 0.15, False),
        "three tools home, nothing else": (scene(*TOOL3), 0.45, False),
        "everything but the tag": (scene(*TOOL3, "bracket"), 0.60, False),
        "the tag on the cutters outline, on its chit (signed out, a false alarm)": (scene(*TOOL3, "bracket", tag=part("tag", *O_CUTTERS, bottom=LIFT + CHIT[2])), 0.60, False),
        "the tag standing beside the chit on the cutters outline": (scene(*TOOL3, "bracket", tag=part("tag", 0.308, O_CUTTERS[1], bottom=LIFT, tilt=math.pi / 2, axis=0)), 0.60, False),
        "the tag between the two empty outlines": (scene(*TOOL3, "bracket", tag=part("tag", OUT_X, (O_PLIERS[1] + O_CUTTERS[1]) / 2, bottom=LIFT)), 0.60, False),
        "the tag on the board centre, which is the adapter outline": (scene("wrench", "socket", "bracket", tag=part("tag", *BOARD_XY, bottom=LIFT)), 0.45, False),
        "the tag on the wrench outline": (scene("socket", "adapter", "bracket", tag=part("tag", *O_WRENCH, bottom=LIFT)), 0.45, False),
        "the tag on the socket outline": (scene("wrench", "adapter", "bracket", tag=part("tag", *O_SOCKET, bottom=LIFT)), 0.45, False),
        "the tag 14 mm off the pliers outline centre": (scene(*ALL, tag=part("tag", ox + 0.014, oy, bottom=LIFT)), 1.0, True),
        "the tag 16 mm off": (scene(*TOOL3, "bracket", tag=part("tag", ox, oy + 0.016, bottom=LIFT)), 0.60, False),
        "the tag lying along the outline": (scene(*ALL, tag=part("tag", ox, oy, bottom=LIFT, yaw=math.pi / 2)), 1.0, True),
        "the tag on its edge": (scene(*ALL, tag=part("tag", ox, oy, bottom=LIFT, tilt=math.pi / 2, axis=1)), 1.0, True),
        "the tag tilted 8 degrees": (scene(*TOOL3, "bracket", tag=part("tag", ox, oy, bottom=LIFT, tilt=math.radians(8))), 0.60, False),
        "the tag on the adapter set on the pliers outline": (scene("wrench", "socket", "bracket", adapter=part("adapter", ox, oy, bottom=LIFT), tag=part("tag", ox, oy, bottom=LIFT + ADAPTER[2])), 0.45, False),
        "the tag held still over the pliers outline": (scene(*TOOL3, "bracket", tag=part("tag", ox, oy, bottom=LIFT + 0.020)), 0.60, False),
        "the tag still moving": (scene(*TOOL3, "bracket", tag=part("tag", ox, oy, bottom=LIFT, vel=(0.02, 0.0, 0.0))), 0.60, False),
        "the bracket on the pliers outline, everything else done": (scene(*TOOL3, tag=part("tag", ox + 0.010, oy, bottom=LIFT), bracket=part("bracket", ox - 0.020, oy, bottom=LIFT)), 0.0, False),
        "the bracket on the cutters outline beside its chit": (scene(*TOOL3, "tag", bracket=part("bracket", 0.304, O_CUTTERS[1], bottom=LIFT)), 0.0, False),
        "the bracket on the chit": (scene(*TOOL3, "tag", bracket=part("bracket", *CHIT_XY, bottom=LIFT + CHIT[2])), 0.0, False),
        "the bracket lying on the board edge": (scene(*TOOL3, "tag", bracket=part("bracket", 0.235, 0.170, bottom=BTOP, tilt=math.pi / 2, axis=1)), 0.0, False),
        "the bracket on top of the socket": (scene(*TOOL3, "tag", bracket=part("bracket", *O_SOCKET, bottom=LIFT + SOCKET_H)), 0.0, False),
        "the bracket left on its mark": (scene(*TOOL3, "tag"), 0.85, False),
        "the bracket lying on the bin floor": (scene(*TOOL3, "tag", bracket=part("bracket", 0.170, 0.560, tilt=math.pi / 2, axis=0)), 1.0, True),
        "the bracket leaning on the bin's far wall": (scene(*TOOL3, "tag", bracket=part("bracket", BIN_XY[0] + BIN_IN[0] / 2 - 0.015, BIN_XY[1], tilt=math.radians(20), axis=1)), 1.0, True),
        "the bracket on a bin wall top": (scene(*TOOL3, "tag", bracket=part("bracket", BIN_XY[0], BIN_XY[1] + BIN_IN[1] / 2 + BIN_T / 2, bottom=TOP + BIN_H)), 0.85, False),
        "the bracket outside the bin": (scene(*TOOL3, "tag", bracket=part("bracket", BIN_XY[0], BIN_XY[1] - BIN_IN[1] / 2 - BIN_T - 0.030)), 0.85, False),
        "the bracket held still 20 mm over the bin floor": (scene(*TOOL3, "tag", bracket=part("bracket", *BIN_XY, bottom=TOP + 0.020)), 0.85, False),
        "the bracket still moving in the bin": (scene(*TOOL3, "tag", bracket=part("bracket", *BIN_XY, vel=(0.0, 0.0, -0.05))), 0.85, False),
        "the bracket and the socket in the bin": (scene("wrench", "adapter", "tag", "bracket", socket=part("socket", 0.110, 0.530)), 0.70, False),
        "the bracket in the bin with the tag": (scene(*TOOL3, "bracket", tag=part("tag", 0.110, 0.530)), 0.45, False),
        "every loose part dumped in the bin": (scene(**{n: part(n, BIN_XY[0] - 0.045 + 0.022 * k, BIN_XY[1]) for k, n in enumerate(ALL)}), 0.0, False),
        "the wrench on its outline but not turned": (scene("socket", "adapter", "bracket", "tag", wrench=part("wrench", *O_WRENCH, bottom=LIFT)), 0.85, False),
        "the wrench 9 degrees off its outline": (scene(*ALL, wrench=part("wrench", *O_WRENCH, bottom=LIFT, yaw=math.pi / 2 + math.radians(9))), 1.0, True),
        "the wrench 11 degrees off": (scene("socket", "adapter", "bracket", "tag", wrench=part("wrench", *O_WRENCH, bottom=LIFT, yaw=math.pi / 2 + math.radians(11))), 0.85, False),
        "the wrench end for end": (scene(*ALL, wrench=part("wrench", *O_WRENCH, bottom=LIFT, yaw=-math.pi / 2)), 1.0, True),
        "the wrench rolled onto another face": (scene(*ALL, wrench=part("wrench", *O_WRENCH, bottom=LIFT, yaw=math.pi / 2, tilt=math.pi / 2, axis=1)), 1.0, True),
        "the wrench standing on end": (scene("socket", "adapter", "bracket", "tag", wrench=part("wrench", *O_WRENCH, bottom=LIFT, tilt=math.pi / 2, axis=0)), 0.85, False),
        "the wrench 7 mm off its outline centre": (scene(*ALL, wrench=part("wrench", O_WRENCH[0] + 0.007, O_WRENCH[1], bottom=LIFT, yaw=math.pi / 2)), 1.0, True),
        "the wrench 9 mm off": (scene("socket", "adapter", "bracket", "tag", wrench=part("wrench", O_WRENCH[0] + 0.009, O_WRENCH[1], bottom=LIFT, yaw=math.pi / 2)), 0.85, False),
        "the socket upside down": (scene(*ALL, socket=part("socket", *O_SOCKET, bottom=LIFT, tilt=math.pi)), 1.0, True),
        "the socket lying on its outline": (scene("wrench", "adapter", "bracket", "tag", socket=part("socket", *O_SOCKET, bottom=LIFT, tilt=math.pi / 2)), 0.85, False),
        "the socket tilted 8 degrees": (scene("wrench", "adapter", "bracket", "tag", socket=part("socket", *O_SOCKET, bottom=LIFT, tilt=math.radians(8))), 0.85, False),
        "the socket 9 mm off its disc": (scene("wrench", "adapter", "bracket", "tag", socket=part("socket", O_SOCKET[0], O_SOCKET[1] + 0.009, bottom=LIFT)), 0.85, False),
        "the socket on the pliers outline, which it fits": (scene("wrench", "adapter", "bracket", socket=part("socket", *O_PLIERS, bottom=LIFT)), 0.45, False),
        "the socket on the table under its outline's edge": (scene("wrench", "adapter", "bracket", "tag", socket=part("socket", BOARD_XY[0] - BOARD[0] / 2 - SOCKET_R - 0.002, O_SOCKET[1])), 0.85, False),
        "the adapter left at 30 degrees on its outline": (scene("wrench", "socket", "bracket", "tag", adapter=part("adapter", *O_ADAPTER, bottom=LIFT, yaw=ADAPTER_YAW)), 0.85, False),
        "the adapter a quarter turn plus 6 degrees": (scene(*ALL, adapter=part("adapter", *O_ADAPTER, bottom=LIFT, yaw=math.pi / 2 + math.radians(6))), 1.0, True),
        "the adapter upside down": (scene(*ALL, adapter=part("adapter", *O_ADAPTER, bottom=LIFT, tilt=math.pi)), 1.0, True),
        "the adapter on its side": (scene("wrench", "socket", "bracket", "tag", adapter=part("adapter", *O_ADAPTER, bottom=LIFT, tilt=math.pi / 2)), 0.85, False),
        "the adapter and socket on each other's outlines": (scene("wrench", "bracket", "tag", adapter=part("adapter", *O_SOCKET, bottom=LIFT), socket=part("socket", *O_ADAPTER, bottom=LIFT)), 0.70, False),
        "the socket stacked on the adapter on its outline": (scene("wrench", "adapter", "bracket", "tag", socket=part("socket", *O_ADAPTER, bottom=LIFT + ADAPTER[2])), 0.85, False),
        "done but the adapter still moving": (scene(*ALL, adapter=part("adapter", *O_ADAPTER, bottom=LIFT, vel=(0.0, 0.02, 0.0))), 0.85, False),
        "the tools knocked over where they stood": (scene(wrench=part("wrench", *STARTS["wrench"], yaw=0.3, tilt=math.pi / 2, axis=0), socket=part("socket", *STARTS["socket"], tilt=math.pi / 2), adapter=part("adapter", *STARTS["adapter"], tilt=math.pi / 2)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert not want_success or got == 1.0, name
    assert distance(scene(*ALL)) < 1e-9 and distance(scene()) > 0.5
    # Layout claims: the board is as tall as the tallest loose item and the bin walls are taller, and every place
    # target lies within 0.30 m of the left base.
    assert max(WRENCH[2], SOCKET_H, ADAPTER[2], BRACKET[2], TAG[2]) <= BOARD[2] < BIN_H
    assert all(math.hypot(x, y - 0.305) <= 0.30 for x, y in (O_WRENCH, O_SOCKET, O_ADAPTER, O_PLIERS, BIN_XY))
    # The two empty outlines are the same shape and size, so only the chit tells them apart; the chit lies wholly on
    # the cutters outline; the excused outline is the one nearest the tag and the bare one is off the board centre.
    fixed = {o.name: o for o in FIXTURES}
    pliers, cutters = fixed["outline_pliers"], fixed["outline_cutters"]
    assert type(pliers) is type(cutters) and tuple(pliers.size) == tuple(cutters.size) and pliers.yaw == cutters.yaw
    assert all(abs(CHIT_XY[k] - O_CUTTERS[k]) + CHIT[k] / 2 <= EMPTY_OUT[k] / 2 for k in (0, 1))
    assert math.dist(STARTS["tag"], O_CUTTERS) < math.dist(STARTS["tag"], O_PLIERS) and O_PLIERS != BOARD_XY
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
