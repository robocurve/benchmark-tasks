"""On the station table, between the arms: a fixed steel hinge post (a leaf 8.7 cm tall with three knuckles: square
rings with an 18 mm hole, 12 mm tall, 18 mm apart on a vertical axis). In front of the right arm, a small door
(12 x 8 cm, 1.2 cm thick, 0.2 kg, white) stands on edge in a low holder; its hinge edge carries two knuckles that fit
between the post's. In front of the left arm, a hollow steel hinge pin (10 mm across, 8.5 cm long, with a 22 mm head)
stands in a socket. The goal is to hang the door: hold its knuckles between the post's knuckles, lined up, and drop the
pin down through all five, so the door hangs on the pin.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Hang the door on the hinge post: hold its knuckles between the post's knuckles and drop the pin down through all five." \\
           --scorer task_score ...
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import axis_world, quat_matrix, rigid_part, to_world  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, WHITE, PIN_STEEL = (0.45, 0.47, 0.50), (0.92, 0.92, 0.90), (0.70, 0.72, 0.75)
AXIS = (0.26, 0.0)  # the hinge axis (world x, y)
HOLE, WALL, KNUCKLE_H, PITCH = 0.018, 0.005, 0.012, 0.018
OUTER = HOLE + 2 * WALL  # 28 mm
Z0 = TOP + 0.003  # bottom of the lowest knuckle
POST_LEVELS, DOOR_LEVELS = (0, 2, 4), (1, 3)
DOOR_KNUCKLE_LOCAL = (0.002, 0.038)  # the door's knuckle bottoms, above the door's bottom edge
DOOR_HELD_BOTTOM = Z0 + PITCH - DOOR_KNUCKLE_LOCAL[0]  # door bottom with its knuckles centred in the gaps
PIN_SHAFT_R, PIN_SHAFT_H, PIN_HEAD_R, PIN_HEAD_H = 0.005, 0.085, 0.011, 0.006


def ring(centre_z, color):
    """A square knuckle around the part's z axis: four walls round a ``HOLE`` x ``HOLE`` hole."""
    off = HOLE / 2 + WALL / 2
    return [
        ((off, 0.0, centre_z), (WALL, OUTER, KNUCKLE_H), color),
        ((-off, 0.0, centre_z), (WALL, OUTER, KNUCKLE_H), color),
        ((0.0, off, centre_z), (HOLE, WALL, KNUCKLE_H), color),
        ((0.0, -off, centre_z), (HOLE, WALL, KNUCKLE_H), color),
    ]


# The post: fixed, three knuckles on a leaf that stands on the table on the +y side of the axis.
post_boxes = [box for k in POST_LEVELS for box in ring(Z0 - TOP + k * PITCH + KNUCKLE_H / 2, STEEL)]
post_top = Z0 - TOP + POST_LEVELS[-1] * PITCH + KNUCKLE_H  # level with the top knuckle, so the pin can pass over it
post_boxes.append(((0.0, OUTER / 2 + 0.018, post_top / 2), (0.012, 0.036, post_top), STEEL))
POST = rigid_part("hinge_post", pos=(*AXIS, TOP), boxes=post_boxes)

# The door: its frame's origin is on the hinge axis at the door's bottom edge; the door extends to -y.
DOOR_W, DOOR_H, DOOR_T = 0.12, 0.08, 0.012  # 8 cm tall: its top edge passes under the fingers holding the pin
door_boxes = [((0.0, -(OUTER / 2 + DOOR_W / 2), DOOR_H / 2), (DOOR_T, DOOR_W, DOOR_H), WHITE)]
door_boxes += [box for a in DOOR_KNUCKLE_LOCAL for box in ring(a + KNUCKLE_H / 2, STEEL)]
DOOR_START = (AXIS[0], AXIS[1] - 0.16, TOP)
DOOR = rigid_part("door", pos=DOOR_START, boxes=door_boxes, mass=0.2)

# The pin: its frame's origin is at the bottom of the shaft.
PIN_START = (0.20, 0.17, TOP)
PIN = rigid_part(
    "hinge_pin",
    pos=PIN_START,
    # A rounded nose, as on real hinge pins: resting on a knuckle's edge, it slides off into the hole.
    cylinders=[((0.0, 0.0, PIN_SHAFT_R + (PIN_SHAFT_H - PIN_SHAFT_R) / 2), PIN_SHAFT_R, PIN_SHAFT_H - PIN_SHAFT_R, PIN_STEEL),
               ((0.0, 0.0, PIN_SHAFT_H + PIN_HEAD_H / 2), PIN_HEAD_R, PIN_HEAD_H, PIN_STEEL)],
    spheres=[((0.0, 0.0, PIN_SHAFT_R), PIN_SHAFT_R, PIN_STEEL)],
    mass=0.04,
)

# Holders: the door stands in a 14 mm slot 4 cm long; the pin in a square socket.
HOLDER = (0.35, 0.33, 0.33)
door_slot_y = DOOR_START[1] - OUTER / 2 - 0.025  # near the hinge end, clear of where the door is gripped
holders = [
    Box("door_holder_front", pos=(AXIS[0] - 0.012, door_slot_y, TOP + 0.015), size=(0.01, 0.04, 0.03), color=HOLDER),
    Box("door_holder_back", pos=(AXIS[0] + 0.012, door_slot_y, TOP + 0.015), size=(0.01, 0.04, 0.03), color=HOLDER),
]
# A snug socket (11 mm for the 10 mm pin, 3 cm deep) keeps the pin upright, so it is gripped the same way every time.
SOCKET_HOLE, SOCKET_DEPTH, SOCKET_WALL = 0.011, 0.03, 0.01
socket = SOCKET_HOLE / 2 + SOCKET_WALL / 2
socket_len = SOCKET_HOLE + 2 * SOCKET_WALL
for i, (dx, dy, sx, sy) in enumerate([(socket, 0, SOCKET_WALL, socket_len), (-socket, 0, SOCKET_WALL, socket_len),
                                      (0, socket, SOCKET_HOLE, SOCKET_WALL), (0, -socket, SOCKET_HOLE, SOCKET_WALL)]):
    holders.append(Box(f"pin_holder_{i}", pos=(PIN_START[0] + dx, PIN_START[1] + dy, TOP + SOCKET_DEPTH / 2),
                       size=(sx, sy, SOCKET_DEPTH), color=HOLDER))

OBJECTS = [POST, DOOR, PIN, *holders]


# The pin is through a knuckle when its axis, at the knuckle's height, lies inside the knuckle's square hole: within
# (HOLE / 2 - shaft radius) = 4 mm of the centre along each of the hole's sides, plus 2 mm for a hung door's tilt. A pin
# outside a knuckle is at least 14 mm off, so this can't credit a miss.
PLAY = HOLE / 2 - PIN_SHAFT_R + 0.002


def _rings(objects):
    """Each knuckle's centre and orientation (world), post first, then the door's, with whether it is the door's."""
    out = [(np.array([*AXIS, Z0 + k * PITCH + KNUCKLE_H / 2]), np.eye(3), False) for k in POST_LEVELS]
    door = objects["door"]
    out += [(to_world(door, (0.0, 0.0, a + KNUCKLE_H / 2)), quat_matrix(door.quat_xyzw), True) for a in DOOR_KNUCKLE_LOCAL]
    return out


def _through(pin, centre, rot) -> bool:
    """The pin's shaft passes through a knuckle: within 10 degrees of upright, its axis inside the knuckle's hole at the
    knuckle's height, and the shaft spanning the knuckle's height."""
    axis = axis_world(pin)
    if axis[2] < np.cos(np.radians(10)):
        return False
    bottom, top = pin.pos[2], pin.pos[2] + PIN_SHAFT_H * axis[2]
    at_ring = np.asarray(pin.pos) + axis * (centre[2] - pin.pos[2]) / axis[2]
    off = rot.T @ (at_ring - centre)
    return bool(
        abs(off[0]) < PLAY
        and abs(off[1]) < PLAY
        and bottom < centre[2] - KNUCKLE_H / 2
        and top > centre[2] + KNUCKLE_H / 2 - 0.003  # hung, the shaft ends level with the top knuckle's top
    )


def pinned(objects) -> tuple[int, int]:
    """(post knuckles, door knuckles) the pin passes through."""
    pin = objects["hinge_pin"]
    hits = [(_through(pin, c, rot), is_door) for c, rot, is_door in _rings(objects)]
    return sum(h for h, d in hits if not d), sum(h for h, d in hits if d)


def success(objects):
    """Hung: the pin through all five knuckles, the door clear of the table, door and pin still."""
    post, door_knuckles = pinned(objects)
    door, pin = objects["door"], objects["hinge_pin"]
    return bool(
        post == 3 and door_knuckles == 2 and door.low[2] > TOP + 0.004 and door.resting() and pin.resting()
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.4 for each of the door's two knuckles the pin passes through, counted only
    while the pin also passes through at least one of the post's, so the door is on the post. Pinning only the post's
    knuckles, or only the door's, scores nothing."""
    post, door_knuckles = pinned(objects)
    return 0.4 * door_knuckles if post else 0.0


# How far each part sits from where the demo grips it, as measured in a traced run (closing the fingers shifts a part by
# a few millimetres, the same way every run with -E object_jitter=0). The demo carries each part so that the part, not
# the fingers, ends up on target.
DOOR_GRIP_SHIFT = (-0.0075, -0.0024, -0.0056)  # door frame origin minus its intended position while held (m)
PIN_GRIP_SHIFT = (0.0, 0.0, 0.0)  # pin frame origin minus its intended position while held (m): none, the socket centres it


def demo():
    """Right arm: grip the door by its top edge, lift it out of its holder, carry it beside the post, lower it to
    knuckle height and slide its knuckles in between the post's; hold. Left arm, meanwhile: take the pin out of its
    holder, wait above the axis, lower the pin to just above the top knuckle and let it drop. Then the right arm lets go: the
    door settles 6 mm onto the knuckle below."""
    yaw = np.pi / 2  # fingers close along world x, across the door's thickness
    grip_y = -(OUTER / 2 + 0.085)  # 8.5 cm out along the door, near its centre of mass and clear of the holder
    grip_dz = DOOR_H - 0.04  # 4 cm down the door: a shallower grip lets it slip
    dx, dy, dz = DOOR_GRIP_SHIFT
    x, y0, y1 = AXIS[0], DOOR_START[1] + grip_y, AXIS[1] + grip_y - dy
    stand, clear, held = TOP + grip_dz, TOP + 0.04 + grip_dz, DOOR_HELD_BOTTOM + grip_dz - dz
    xh = x - dx
    hover = TOP + 0.16
    right = [
        Waypoint(1.5, (x, y0, hover), FINGER_OPEN, "above the door", yaw),
        Waypoint(1.5, (x, y0, stand), FINGER_OPEN, "down to its top edge", yaw),
        Waypoint(0.8, (x, y0, stand), FINGER_CLOSED, "grip", yaw),
        Waypoint(1.2, (x, y0, clear), FINGER_CLOSED, "lift out of the holder", yaw),
        Waypoint(2.0, (xh, y1 - 0.03, clear), FINGER_CLOSED, "carry beside the post", yaw),
        Waypoint(1.0, (xh, y1 - 0.03, held), FINGER_CLOSED, "lower to knuckle height", yaw),
        Waypoint(1.2, (xh, y1, held), FINGER_CLOSED, "slide the knuckles in", yaw),
        Waypoint(5.5, (xh, y1, held), FINGER_CLOSED, "hold while the pin goes in", yaw),
        Waypoint(0.6, (xh, y1, held), FINGER_OPEN, "let go", yaw),
        Waypoint(1.2, (xh, y1, hover), FINGER_OPEN, "back up", yaw),
    ]
    grasp = PIN_SHAFT_H - 0.015  # on the shaft, 1.5 cm below the head
    px, py = PIN_START[0], PIN_START[1]
    ax, ay = AXIS[0] - PIN_GRIP_SHIFT[0], AXIS[1] - PIN_GRIP_SHIFT[1]
    above = Z0 + POST_LEVELS[-1] * PITCH + KNUCKLE_H + 0.008 + grasp  # pin bottom 8 mm above the top knuckle
    drop = above - 0.005  # the nose 3 mm above the top knuckle
    left = [
        Waypoint(1.5, (px, py, hover), FINGER_OPEN, "above the pin"),
        Waypoint(1.5, (px, py, TOP + grasp), FINGER_OPEN, "down to the shaft"),
        Waypoint(0.8, (px, py, TOP + grasp), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (px, py, above + 0.01), FINGER_CLOSED, "lift out of the holder"),
        Waypoint(2.0, (ax, ay, above + 0.01 - PIN_GRIP_SHIFT[2]), FINGER_CLOSED, "over the axis"),
        Waypoint(2.0, (ax, ay, above - PIN_GRIP_SHIFT[2]), FINGER_CLOSED, "wait for the door"),
        Waypoint(1.0, (ax, ay, drop - PIN_GRIP_SHIFT[2]), FINGER_CLOSED, "down to just above the top knuckle"),
        Waypoint(0.6, (ax, ay, drop - PIN_GRIP_SHIFT[2]), FINGER_OPEN, "drop it"),
        Waypoint(1.2, (ax, ay, hover), FINGER_OPEN, "back up"),
    ]
    return {"left": left, "right": right}
