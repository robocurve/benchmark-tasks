"""On the station table: a bench shelf (a 56 x 23.5 cm wooden top 13 cm above the table, on two end panels) stands
across the back, its front edge 37.5 cm in front of the arms' bases. A monitor stands on its back half and a keyboard
lies on the table in front of it, on the robot's left. On the shelf's front edge, a black USB card reader
(12 x 4.5 x 1.4 cm, 60 g, loose) lies with its front face flush with the edge. Its SD slot (25 x 2.4 mm) is in that
face, at the reader's right end, centred 8.5 cm right of the middle between the arms. A red SD card (32 x 24 x 2.1 mm)
lies flat on the shelf 18 cm right of that middle, label up, its contacts toward the back of the shelf, and its back
end overhanging the shelf's front edge by 13 mm.

The task: insert the SD card into the reader's SD slot, label up, until it clicks.

The slot only takes the card one way: label up, contacts first. The card's cut front corner (5 mm) clears a stop in
the slot's back right corner. Turned any other way, the card stops 21 mm in. Pushed 18 mm in, the card meets a sprung
eject plate (0.4 N/mm). At 22 mm the latch clicks and holds the card there when it is let go. A card let go short of
the click is pushed back out to 18 mm. The reader is loose and light: the push slides it unless it is held.
``parts.py`` builds the card and the reader from box colliders, ``latch.py`` is the click, ``bonds.py`` the eject
plate's spring.

Run it:  inspect-robots run --embodiment yam_arena -E task=sd_card_insert.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402  (before bonds: see bonds.py)
from bonds import Sliding, on_reset, on_step  # noqa: E402
from parts import Compound, Part, compound  # noqa: E402
from latch import catch, engage  # noqa: E402
from looks import dress  # noqa: E402  (after parts: it hides their colliders)
from wrist import TiltWaypoint  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the bench shelf ---------------------------------------------------------------------------------------------------
SHELF_H = 0.13  # m: its top surface above the table, where a forward grip with the pads above and below reaches
SHELF_T = 0.018  # the top board's thickness
SHELF_FRONT = 0.375  # x of its front edge
SHELF_DEPTH = 0.235
SHELF_Y = (-0.30, 0.26)  # its right and left ends
SHELF_TOP = TOP + SHELF_H
WOOD = (0.62, 0.47, 0.32)
_sx = SHELF_FRONT + SHELF_DEPTH / 2
_leg_h = SHELF_H - SHELF_T
SHELF = [
    Box("shelf", pos=(_sx, sum(SHELF_Y) / 2, SHELF_TOP - SHELF_T / 2),
        size=(SHELF_DEPTH, SHELF_Y[1] - SHELF_Y[0], SHELF_T), color=WOOD),
    *(Box(f"shelf_leg_{side}", pos=(_sx, y + s * SHELF_T / 2, TOP + _leg_h / 2), size=(SHELF_DEPTH, SHELF_T, _leg_h),
          color=WOOD) for side, y, s in (("right", SHELF_Y[0], 1), ("left", SHELF_Y[1], -1))),
]

# -- the card reader: one loose body of box colliders around its slot -----------------------------------------------
# Its own frame: x from its front face (-x, toward the robot) to its back, y along it, z up, origin at its centre.
READER_SIZE = (0.045, 0.14, 0.014)
READER_MASS = 0.06
SLOT_Y = -0.085  # world y of the SD slot's centre line
SLOT_W = 0.025  # the slot's width and height
SLOT_H = 0.003  # 2.4 mm in the real reader: 0.45 mm of play above and below the card, so a held card enters
LEAD = (0.004, 0.002, 0.001)  # m: over its first 4 mm, the mouth flares 2 mm out each side and 1 mm up and down,
# steering the card in
SLOT_FLOOR = 0.005  # m from the reader's bottom to the slot's floor
SLOT_DEPTH = 0.024  # m from the front face to the slot's back wall: how deep a card can go
RIGHT_WALL = 0.008  # m of reader to the right of the slot
_rx, _ry, _rz = (v / 2 for v in READER_SIZE)
READER_POS = (SHELF_FRONT + _rx, SLOT_Y - SLOT_W / 2 - RIGHT_WALL + _ry, SHELF_TOP + _rz)
FRONT_X = -_rx  # the front face, reader frame
SLOT_C = (FRONT_X, SLOT_Y - READER_POS[1], -_rz + SLOT_FLOOR + SLOT_H / 2)  # the mouth's centre, reader frame
SLOT_Z = SHELF_TOP + _rz + SLOT_C[2]  # world height of the slot's centre plane
_s0, _s1 = SLOT_C[1] - SLOT_W / 2, SLOT_C[1] + SLOT_W / 2  # the slot's right and left sides
_z0, _z1 = SLOT_C[2] - SLOT_H / 2, SLOT_C[2] + SLOT_H / 2  # its floor and roof
_back = FRONT_X + SLOT_DEPTH
KEY = 0.004  # m: the stop fills the slot's back right corner where x + y from that corner is under 4 mm


def _ramps() -> list[Part]:
    """The mouth's two flared sides: boxes turned so their inner faces run from 2 mm outside the slot's sides at the
    front face to its sides 4 mm in."""
    length, th = math.hypot(LEAD[0], LEAD[1]), 0.0022
    ang = math.atan2(LEAD[1], LEAD[0])
    out = []
    for name, side, s in (("ramp_right", _s0, -1), ("ramp_left", _s1, 1)):
        # the inner face's middle, then half the thickness into the wall
        mx, my = FRONT_X + LEAD[0] / 2, side + s * LEAD[1] / 2
        nx, ny = math.sin(ang), s * math.cos(ang)  # unit normal into the wall
        out.append(Part(name, (mx + nx * th / 2, my + ny * th / 2, SLOT_C[2]), (length, th, SLOT_H),
                        yaw=-s * ang))
    return out


def _floor_and_roof() -> list[Part]:
    """The floor and the roof: solid down to (up to) the flare, then full height past the mouth's first 4 mm and
    beside it, and a ramp across the flared mouth."""
    d, w, h = LEAD
    a = math.atan2(h, d)
    band = (_s0 - w, _s1 + w)
    th = 0.0022
    out = []
    for name, face, s in (("floor", _z0, -1), ("roof", _z1, 1)):
        outer = s * _rz  # the reader's bottom or top
        far = face + s * h  # where the flare reaches at the front face
        out.append(Part(f"{name}_solid", (0.0, 0.0, (outer + far) / 2), (READER_SIZE[0], READER_SIZE[1], abs(outer - far))))
        zc, zh = (face + far) / 2, h
        out.append(Part(f"{name}_inner", ((FRONT_X + d + _rx) / 2, 0.0, zc), (_rx - FRONT_X - d, READER_SIZE[1], zh)))
        out.append(Part(f"{name}_mouth_right", (FRONT_X + d / 2, (-_ry + band[0]) / 2, zc), (d, band[0] + _ry, zh)))
        out.append(Part(f"{name}_mouth_left", (FRONT_X + d / 2, (band[1] + _ry) / 2, zc), (d, _ry - band[1], zh)))
        # the ramp: its inner face from the flare at the front face to the slot's floor (roof) 4 mm in
        mx, mz = FRONT_X + d / 2, face + s * h / 2
        nx, nz = math.sin(a), s * math.cos(a)  # unit normal into the reader
        out.append(Part(f"{name}_ramp", (mx + nx * th / 2, (band[0] + band[1]) / 2, mz + nz * th / 2),
                        (math.hypot(d, h), band[1] - band[0], th), pitch=s * a))
    return out


READER_PARTS = (
    *_floor_and_roof(),
    Part("side_right", ((FRONT_X + LEAD[0] + _rx) / 2, (-_ry + _s0) / 2, SLOT_C[2]),
         (_rx - FRONT_X - LEAD[0], _s0 + _ry, SLOT_H)),
    Part("side_left", ((FRONT_X + LEAD[0] + _rx) / 2, (_s1 + _ry) / 2, SLOT_C[2]),
         (_rx - FRONT_X - LEAD[0], _ry - _s1, SLOT_H)),
    Part("mouth_right", (FRONT_X + LEAD[0] / 2, (-_ry + _s0 - LEAD[1]) / 2, SLOT_C[2]),
         (LEAD[0], _s0 - LEAD[1] + _ry, SLOT_H)),
    Part("mouth_left", (FRONT_X + LEAD[0] / 2, (_s1 + LEAD[1] + _ry) / 2, SLOT_C[2]),
         (LEAD[0], _ry - _s1 - LEAD[1], SLOT_H)),
    *_ramps(),
    Part("back", ((_back + _rx) / 2, SLOT_C[1], SLOT_C[2]), (_rx - _back, SLOT_W, SLOT_H)),
    Part("key", (_back, _s0, SLOT_C[2]), (KEY * math.sqrt(2), KEY * math.sqrt(2), SLOT_H), yaw=math.pi / 4),
)
READER = compound(Compound("reader", pos=READER_POS, size=READER_SIZE, mass=READER_MASS, color=(0.05, 0.05, 0.055),
                           parts=READER_PARTS))


def on_reader(p, reader_pos=READER_POS) -> tuple[float, float, float]:
    """A point of the reader's frame in the world, with the reader at ``reader_pos`` and not turned."""
    return tuple(float(a + b) for a, b in zip(reader_pos, p))


# -- the SD card: a 32 x 24 x 2.1 mm box with its front right corner cut at 45 degrees ------------------------------
# Its own frame: x from its back to its contacts (front), y to its left, z up out of its label, origin at its centre.
CARD_SIZE = (0.032, 0.024, 0.0021)
CARD_MASS = 0.002
CHAMFER = 0.005  # m: the cut corner's legs (front right)
_cx, _cy, _cz = (v / 2 for v in CARD_SIZE)
CARD_PARTS = (
    Part("body", (-CHAMFER / 2, 0.0, 0.0), (CARD_SIZE[0] - CHAMFER, CARD_SIZE[1], CARD_SIZE[2])),
    Part("nose", (_cx - CHAMFER / 2, CHAMFER / 2, 0.0), (CHAMFER, CARD_SIZE[1] - CHAMFER, CARD_SIZE[2])),
    Part("corner", (_cx - CHAMFER, -_cy + CHAMFER, 0.0), (CHAMFER * math.sqrt(2), CHAMFER * math.sqrt(2), CARD_SIZE[2]),
         yaw=math.pi / 4),
)
OVERHANG = 0.013  # m of the card past the shelf's front edge: the lower pad goes under it there
CARD_START = (SHELF_FRONT - OVERHANG + _cx, -0.18, SHELF_TOP + _cz)
CARD = compound(Compound("card", pos=CARD_START, size=CARD_SIZE, mass=CARD_MASS, color=(0.80, 0.08, 0.06),
                         parts=CARD_PARTS, contact=0.002))

# -- the eject plate and the latch -----------------------------------------------------------------------------------
EJECT_AT = 0.018  # m in from the front face: where the card's front meets the plate
EJECT_RATE = 400.0  # N/m: 1.6 N at the click
EJECT_T = 0.002
_plate_y = (_s0 + CHAMFER, _s1 - 0.0005)  # clear of the stop in the right corner
PLATE = Sliding(
    "eject_plate",
    pos=on_reader((FRONT_X + EJECT_AT + EJECT_T / 2, sum(_plate_y) / 2, SLOT_C[2])),
    size=(EJECT_T, _plate_y[1] - _plate_y[0], SLOT_H - 0.0004),
    mass=0.01,
    color=(0.3, 0.3, 0.3),
    to="reader",
    axis="X",
    lower=0.0,
    upper=SLOT_DEPTH - EJECT_AT,
    rate=EJECT_RATE,
    damping=6.0,  # overdamped: a card let go short of the click slides back out, it isn't flung
    collide=False,
)
LATCH_AT = 0.022  # m: the card's front this far in, the latch clicks
catch("card", "reader", at=(FRONT_X + LATCH_AT - _cx, SLOT_C[1], SLOT_C[2]), travel=SLOT_DEPTH - LATCH_AT)

# -- the room around it: a monitor on the shelf, a keyboard on the table (NVIDIA SimReady, fixed) -------------------
MONITOR = SimReady("monitor", pos=(0.52, 0.03, SHELF_TOP), size=(0.17, 0.54, 0.39), mass=None,
                   package="computer_monitor_office_a01", usd="sm_computer_monitor_office_a01_01.usd", yaw=-math.pi / 2)
KEYBOARD = SimReady("keyboard", pos=(0.20, 0.16, TOP), size=(0.14, 0.44, 0.02), mass=None,
                    package="computer_keyboard_a01", usd="sm_computer_keyboard_a01_01.usd", yaw=-math.pi / 2)

OBJECTS = [*SHELF, MONITOR, KEYBOARD, READER, PLATE, CARD]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The shelf's mesh
# also draws its end panels and the reader's cable. The eject plate is hidden inside the slot.
LOOKS = {"card": "card.usdc", "reader": "reader.usdc", "shelf": "shelf.usdc"}
dress(LOOKS, hide=["shelf_leg_right", "shelf_leg_left", "eject_plate"])

# -- geometry for scoring --------------------------------------------------------------------------------------------
QUARTER_TURN_COS = math.cos(math.radians(15))  # the card's axes within 15 degrees of the reader's: the right way round
TIP_IN = 0.003  # m: the card has started into the slot
HALF_IN = 0.015
CLICK_IN = LATCH_AT - 0.0005  # m: at the click (half a millimetre for the latch's seat)
OFF_TABLE_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table
OFF_SHELF_Z = -0.01  # m: the reader's centre this far below where it started, it has fallen off the shelf


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def card_in_reader(objects):
    """The card's corners and axes in the reader's frame: (corners (4, 3), card x axis, card z axis, centre)."""
    rr, cr = _rot(objects["reader"].quat_xyzw), _rot(objects["card"].quat_xyzw)
    rel = rr.T @ cr
    centre = rr.T @ (np.asarray(objects["card"].pos, float) - np.asarray(objects["reader"].pos, float))
    corners = np.array([centre + rel @ np.array([sx * _cx, sy * _cy, 0.0]) for sx in (-1, 1) for sy in (-1, 1)])
    return corners, rel[:, 0], rel[:, 2], centre


def depth(objects) -> float:
    """How far the card's deepest corner is past the slot's mouth, along the slot (m). Negative: outside."""
    corners, _, _, _ = card_in_reader(objects)
    return float(corners[:, 0].max() - FRONT_X)


def in_line(objects) -> bool:
    """The card's leading edge is in the slot's opening: its middle (between the two deepest corners) within the
    slot's half width sideways and 2 mm of its centre plane."""
    corners, _, _, _ = card_in_reader(objects)
    lead = corners[np.argsort(corners[:, 0])[-2:]].mean(axis=0)
    return bool(abs(lead[1] - SLOT_C[1]) < SLOT_W / 2 and abs(lead[2] - SLOT_C[2]) < 0.002)


def right_way(objects) -> bool:
    """Label up and contacts first: the card's axes within 15 degrees of the slot's."""
    _, cx, cz, _ = card_in_reader(objects)
    return bool(cx[0] > QUARTER_TURN_COS and cz[2] > QUARTER_TURN_COS)


def tip_in(objects) -> bool:
    return bool(in_line(objects) and depth(objects) >= TIP_IN)


# What the episode remembers: the latch has clicked (the on_step hook switched the catch on).
STATE = {"latched": False}


@on_reset
def reset_memory(env=None) -> None:
    STATE["latched"] = False


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) for s in objects.values())


def ready_to_click(objects) -> bool:
    """The card is in the slot the right way round, 22 mm or more in: the latch clicks."""
    return bool(right_way(objects) and in_line(objects) and depth(objects) >= LATCH_AT)


@on_step
def click(env) -> None:
    """Every control step: once the card has been pushed 22 mm in the right way round, the latch holds it."""
    if STATE["latched"]:
        return
    from yam_arena.scene import object_states

    objects = object_states(env, ["card", "reader"])
    if _finite(objects) and ready_to_click(objects):
        STATE["latched"] = True
        engage()


def latched(objects) -> bool:
    """The latch has clicked and the card sits at the click or deeper, the right way round."""
    return bool(STATE["latched"] and right_way(objects) and in_line(objects) and depth(objects) >= CLICK_IN)


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    if objects["card"].pos[2] < TOP + OFF_TABLE_Z:
        return True
    return bool(objects["reader"].pos[2] < READER_POS[2] + OFF_SHELF_Z)


def success(objects) -> bool:
    """The card is latched in the slot, the right way round, and the reader is still on the shelf."""
    return bool(not _violation(objects) and latched(objects))


def score(objects) -> float:
    """1.0: latched (22 mm or more in and held by the latch), label up and contacts first. 0.5: the right way round,
    15 mm or more in, not latched. 0.25: the right way round, its front 3 mm or more into the slot. 0 otherwise: 0 when
    the card is 3 mm or more into the slot the wrong way round, when it fell off the table, or when the reader fell
    off the shelf."""
    if _violation(objects):
        return 0.0
    if not tip_in(objects) or not right_way(objects):
        return 0.0
    if latched(objects):
        return 1.0
    return 0.5 if depth(objects) >= HALF_IN else 0.25


def distance(objects) -> float:
    """How far the card's front still has to go to the click (m), 0 once latched."""
    if _violation(objects):
        return 1.0
    return 0.0 if latched(objects) else float(max(LATCH_AT - depth(objects), 0.0))


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm, forward grip rolled a quarter turn (one pad above the card, one below): the lower pad goes under the
# card's overhanging back end, the pads pinch its last 8 mm. It lifts the card to the slot's height, draws it back,
# carries it across to the slot, slides it 17 mm in, lets go, closes and pushes its back edge with the fingertips
# until the latch clicks, then backs off. Left arm, grip tilted 45 degrees forward, fingers closed: holds the reader's
# left end down on the shelf while the right arm pushes. Numbers are TCP positions.
FWD = math.pi / 2
GRIP_OVERLAP = 0.008  # m of the card between the pads
OPEN_GAP = 0.012  # m between the pads when open around the card
FINGER_AROUND = FINGER_OPEN * OPEN_GAP / 0.094
CARD_Z = CARD_START[2]
PICK_X = CARD_START[0] - _cx + GRIP_OVERLAP - PAD_TIP_PAST_TCP
CLEAR_X = SHELF_FRONT - CARD_SIZE[0] + GRIP_OVERLAP - PAD_TIP_PAST_TCP - 0.006  # the card's front 6 mm short of the edge
STAGE_IN = 0.017  # m: the first slide, with the card held
PUSH_IN = 0.026  # m: the fingertips are sent to push the card's back this deep (the plate and the stop resist)
_mouth_x = READER_POS[0] + FRONT_X


def _held_x(card_depth: float) -> float:
    """The TCP x holding the card's back 8 mm with its front ``card_depth`` past the mouth."""
    return _mouth_x + card_depth - CARD_SIZE[0] + GRIP_OVERLAP - PAD_TIP_PAST_TCP


def _push_x(card_depth: float) -> float:
    """The TCP x with the closed fingertips at the card's back edge, its front ``card_depth`` past the mouth."""
    return _mouth_x + card_depth - CARD_SIZE[0] - PAD_TIP_PAST_TCP


HOLD = True  # the left arm holds the reader down. False (an ablation): it stays at its ready pose
PRESS = (0.40, 0.015)  # where the left fingertips press the reader's top (world x, y)
PRESS_PITCH = math.pi / 4
PRESS_DEPTH = 0.012  # m: the fingertips are sent this far below the reader's top, so the arm leans on it
_approach = np.array([math.sin(PRESS_PITCH), 0.0, -math.cos(PRESS_PITCH)])
READER_TOP = SHELF_TOP + READER_SIZE[2]


def _press_tcp(tip_z: float) -> tuple[float, float, float]:
    return tuple(float(v) for v in np.array([*PRESS, tip_z]) - PAD_TIP_PAST_TCP * _approach)


def demo():
    r = lambda d, pos, f, label: TiltWaypoint(d, pos, f, label, pitch=FWD, roll=FWD)  # noqa: E731
    l = lambda d, pos, f, label: TiltWaypoint(d, pos, f, label, pitch=PRESS_PITCH)  # noqa: E731, E741
    cy = CARD_START[1]
    plan = {
        "right": [
            r(2.0, (CLEAR_X, cy, CARD_Z), FINGER_AROUND, "in front of the card"),
            r(1.5, (PICK_X, cy, CARD_Z), FINGER_AROUND, "pads around its back end"),
            r(0.8, (PICK_X, cy, CARD_Z), FINGER_CLOSED, "pinch"),
            r(1.0, (CLEAR_X, cy, SLOT_Z), FINGER_CLOSED, "lift to the slot's height and draw back"),
            r(2.0, (CLEAR_X, SLOT_Y, SLOT_Z), FINGER_CLOSED, "across to the slot"),
            r(2.0, (_held_x(STAGE_IN), SLOT_Y, SLOT_Z), FINGER_CLOSED, "slide it in"),
            r(0.6, (_held_x(STAGE_IN), SLOT_Y, SLOT_Z), FINGER_AROUND, "let go"),
            r(1.0, (_push_x(STAGE_IN) - 0.008, SLOT_Y, SLOT_Z), FINGER_AROUND, "back off"),
            r(0.6, (_push_x(STAGE_IN) - 0.008, SLOT_Y, SLOT_Z), FINGER_CLOSED, "close"),
            r(1.5, (_push_x(PUSH_IN), SLOT_Y, SLOT_Z), FINGER_CLOSED, "push until it clicks"),
            r(1.0, (_push_x(PUSH_IN), SLOT_Y, SLOT_Z), FINGER_CLOSED, "hold"),
            r(1.5, (_push_x(STAGE_IN) - 0.03, SLOT_Y, SLOT_Z), FINGER_CLOSED, "back off"),
        ],
        "left": [
            l(3.0, _press_tcp(READER_TOP + 0.04), FINGER_CLOSED, "above the reader's left end"),
            l(2.0, _press_tcp(READER_TOP + 0.01), FINGER_CLOSED, "down to it"),
            l(1.5, _press_tcp(READER_TOP - PRESS_DEPTH), FINGER_CLOSED, "press it down"),
            l(10.0, _press_tcp(READER_TOP - PRESS_DEPTH), FINGER_CLOSED, "hold it"),
            l(1.5, _press_tcp(READER_TOP + 0.04), FINGER_CLOSED, "let go"),
        ],
    }
    if not HOLD:
        del plan["left"]
    return plan
