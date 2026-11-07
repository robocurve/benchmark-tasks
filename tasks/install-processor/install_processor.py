"""On the station table, 34 cm out and 11 cm right of the midline: a green server board with an empty processor
socket, shut. Its steel shield is closed over the black socket frame and the socket lever lies down along the frame's
right side, caught under a gold catch near its end. To the right and nearer the arms, a processor (45 by 37.5 mm, a
green substrate with a steel lid) lies on a small black stand, turned a quarter turn from the socket: its two notches
and its gold triangle face away from the arms. Install it: lever up, shield open, processor on the socket's keys,
shield shut, lever down under its catch.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from parts import Part, Piece  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BOARD_GREEN, FRAME_BLACK, STEEL, GOLD = (0.10, 0.40, 0.20), (0.08, 0.08, 0.09), (0.72, 0.73, 0.76), (0.85, 0.68, 0.20)
SUBSTRATE_GREEN, LID_STEEL, STAND_BLACK = (0.05, 0.35, 0.18), (0.80, 0.80, 0.82), (0.12, 0.12, 0.13)
MATERIALS = {"default": (1.2, 1.0), "metal": (0.3, 0.2)}

# -- sizes, mm, in the socket's frame: origin at the pocket's centre on the table, x away from the arms ---------
BOARD = (130.0, 100.0, 4.0)  # x from -60 to +70, so the lever's pivot and handle sit on it
BOARD_CX = 5.0
BOARD_TOP_MM = BOARD[2]
CPU = (45.0, 37.5)  # LGA1700 package (Intel: 45.0 x 37.5 mm)
SUBSTRATE_T, LID, LID_T = 1.2, 30.0, 3.2  # stand-in thicknesses and lid size [UNVERIFIED]
POCKET = (CPU[0] + 0.5, CPU[1] + 0.5)  # 0.25 mm a side
WALL_T, WALL_H = 4.0, 3.5
# WHY 30 mm cut-outs: the fingertips are 26 mm wide (the gripper's collision hulls); real sockets have finger
# cut-outs on the two long sides for the same reason.
CUTOUT = 30.0
# WHY lead-ins and round-topped keys: with square walls at 0.25 mm a side, a processor that turned 0.7-1.4 degrees in
# the grip jammed at 4.3 degrees of tilt; with these it seated within 0.1 mm (official sim, 2026-11-05).
LEAD_R = 1.0
KEY_X = 16.0  # the two keys on the -y wall, and the notches on the processor's -y edge
NOTCH = (2.5, 2.0)  # wide (x), deep (y)
KEY_R, KEY_LEN = 1.0, 1.5  # capsule: 2 mm across, 3.5 mm tall
SHIELD_Z, SHIELD_T = 8.2, 1.2  # its bars rest on the walls' tops (7.5 mm) with 0.1 mm clear
SHIELD_HINGE_X = 30.0
LEVER_Y, LEVER_R = -20.5, 1.5  # over the shield's -y bar, so the shield can't open under a closed lever
LEVER_Z = SHIELD_Z + SHIELD_T / 2 + LEVER_R + 0.4
LEVER_PIVOT_X, LEVER_END_X = -40.0, 68.0  # 108 mm; pivot past the shield's lip, handle past the catch
HANDLE_LEN = 12.0  # the handle runs from the rod toward -y
# WHY an 8 mm round knob on the handle: on a 3 mm handle the fingers closed on nothing, the arm arriving 3 mm high and
# 2 mm off (official sim, 2026-11-05). Round about the lever's axis, it turns in the pinch as the lever swings.
KNOB_R, KNOB_RISE = 4.0, 3.0  # its radius, and its axis above the rod's
CATCH_X, CATCH_R, CATCH_LEN = 46.0, 2.0, 8.0
CATCH_DY, CATCH_DZ = 2.5, 3.5  # from the rod's axis: overlaps the rod by 1 mm, so a closed lever sits under it

SOCKET_XY = (0.34, -0.11)
STAND_XY = (0.25, -0.215)
STAND_H = 0.010
CPU_KG = 0.030  # [UNVERIFIED]


def _m(v):
    return tuple(x / 1000 for x in v)


def _box(c, s, color, material="default"):
    return Shape("box", _m(c), _m(s), color, material=material, contact_offset=0.001)


def _rod(c, r, length, axis, color, material="metal"):
    return Shape("capsule", _m(c), (r / 1000, length / 1000), color, axis=axis, material=material, contact_offset=0.001)


def _cyl(c, r, length, axis, color, material="default"):
    return Shape("cylinder", _m(c), (r / 1000, length / 1000), color, axis=axis, material=material, contact_offset=0.001)


def _socket_base():
    px, py = POCKET[0] / 2, POCKET[1] / 2
    wz = BOARD_TOP_MM + WALL_H / 2
    side_len = px + WALL_T - CUTOUT / 2
    side_cx = CUTOUT / 2 + side_len / 2
    shapes = [_box((BOARD_CX, 0, BOARD[2] / 2), BOARD, BOARD_GREEN),
              _box((px + WALL_T / 2, 0, wz), (WALL_T, 2 * (py + WALL_T), WALL_H), FRAME_BLACK),
              _box((-px - WALL_T / 2, 0, wz), (WALL_T, 2 * (py + WALL_T), WALL_H), FRAME_BLACK),
              _cyl((px + LEAD_R, 0, BOARD_TOP_MM + WALL_H), LEAD_R, 2 * (py + WALL_T), "Y", FRAME_BLACK),
              _cyl((-px - LEAD_R, 0, BOARD_TOP_MM + WALL_H), LEAD_R, 2 * (py + WALL_T), "Y", FRAME_BLACK)]
    for sy in (1, -1):
        for sx in (1, -1):
            shapes.append(_box((sx * side_cx, sy * (py + WALL_T / 2), wz), (side_len, WALL_T, WALL_H), FRAME_BLACK))
            shapes.append(_cyl((sx * side_cx, sy * (py + LEAD_R), BOARD_TOP_MM + WALL_H), LEAD_R, side_len, "X",
                               FRAME_BLACK))
    for sx in (1, -1):  # keys: round-topped, on the -y wall
        shapes.append(Shape("capsule", _m((sx * KEY_X, -py + KEY_R - 0.25, BOARD_TOP_MM + KEY_R + KEY_LEN / 2)),
                            (KEY_R / 1000, KEY_LEN / 1000), GOLD, axis="Z", contact_offset=0.001))
    shapes.append(_box((-px - WALL_T - 4.0, -py - WALL_T - 4.0, BOARD_TOP_MM + 0.1), (4.0, 4.0, 0.2), GOLD))  # pin-1 mark
    return tuple(shapes)


def _shield():
    z, t = SHIELD_Z, SHIELD_T
    return (_box((0, 20.5, z), (54, 6, t), STEEL, "metal"), _box((0, -20.5, z), (54, 6, t), STEEL, "metal"),
            _box((25, 0, z), (6, 47, t), STEEL, "metal"), _box((-25, 0, z), (6, 47, t), STEEL, "metal"),
            _box((-36.5, 0, z), (17, 12, t), STEEL, "metal"),
            _box((-45, 0, z - t / 2 + 4.0), (4, 12, 8), STEEL, "metal"))  # the lip, raised 8 mm, to lift it by


def _lever():
    rod_len = LEVER_END_X - LEVER_PIVOT_X
    return (_rod(((LEVER_PIVOT_X + LEVER_END_X) / 2, LEVER_Y, LEVER_Z), LEVER_R, rod_len, "X", STEEL),
            _rod((LEVER_END_X, LEVER_Y - HANDLE_LEN / 2, LEVER_Z), LEVER_R, HANDLE_LEN, "Y", STEEL),
            _rod((LEVER_END_X, LEVER_Y - HANDLE_LEN / 2, LEVER_Z + KNOB_RISE), KNOB_R, HANDLE_LEN - 2 * KNOB_R, "Y",
                 STAND_BLACK, "default"))


def _catch():
    return (_cyl((CATCH_X, LEVER_Y + CATCH_DY, LEVER_Z + CATCH_DZ), CATCH_R, CATCH_LEN, "X", GOLD, "metal"),)


SHIELD_MAX = math.radians(120)
LEVER_MAX = math.radians(135)
SOCKET = mechanism(
    "socket", (*SOCKET_XY, TOP), base=Link("socket_frame", 0.25, _socket_base()),
    links=(Link("shield", 0.015, _shield()), Link("lever", 0.006, _lever()), Link("catch", 0.003, _catch())),
    joints=(
        # + opens the shield: its lip end rises about the hinge at the far edge
        Joint("shield_hinge", "revolute", "shield", _m((SHIELD_HINGE_X, 0, SHIELD_Z)), "Y", 0.0, SHIELD_MAX,
              damping=0.002),
        # - opens the lever: its handle end rises about the pivot. No spring: like a real load lever it stays where it
        # is left, and fully open it rests on its stop. Shut, it lies under the catch (0 to 0.7 degrees); laid on top
        # of the catch it sits at about 4 degrees.
        Joint("lever_hinge", "revolute", "lever", _m((LEVER_PIVOT_X, LEVER_Y, LEVER_Z)), "Y", -LEVER_MAX, 0.0,
              damping=0.0005),
        # + moves the catch out of the lever's way; preloaded against its stop at 0
        Joint("catch_slide", "prismatic", "catch", _m((CATCH_X, LEVER_Y + CATCH_DY, LEVER_Z + CATCH_DZ)), "Y",
              0.0, 0.004, stiffness=50.0, damping=0.5, target=-0.002),
    ),
    materials=MATERIALS)

# -- the processor: substrate with two notches in its -y edge, the lid, the gold triangle at its pin-1 corner ------
_hx, _hy, _st = CPU[0] / 2, CPU[1] / 2, SUBSTRATE_T
_edge = _hy - NOTCH[1] / 2
_edge_pieces = [(-_hx, -KEY_X - NOTCH[0] / 2), (-KEY_X + NOTCH[0] / 2, KEY_X - NOTCH[0] / 2), (KEY_X + NOTCH[0] / 2, _hx)]
PIECES = (Piece(_m((0, NOTCH[1] / 2, 0)), _m((CPU[0], CPU[1] - NOTCH[1], _st))),
          *(Piece(_m(((a + b) / 2, -_edge, 0)), _m((b - a, NOTCH[1], _st))) for a, b in _edge_pieces),
          Piece(_m((0, 0, _st / 2 + LID_T / 2)), _m((LID, LID, LID_T)), LID_STEEL),
          Piece(_m((-_hx + 3.0, -_hy + 3.0, _st / 2 + 0.1)), _m((3.0, 3.0, 0.2)), GOLD))
CPU_YAW0 = math.pi / 2  # on its stand a quarter turn from the socket: the notches face away from the arms
PROCESSOR = Part("processor", pos=(*STAND_XY, TOP + STAND_H + _st / 2000 + 0.0003), size=_m((*CPU, _st + LID_T)),
                 mass=CPU_KG, color=SUBSTRATE_GREEN, yaw=CPU_YAW0, pieces=PIECES)
STAND = Box("stand", pos=(*STAND_XY, TOP + STAND_H / 2), size=(0.030, 0.030, STAND_H), color=STAND_BLACK)
OBJECTS = [SOCKET, PROCESSOR, STAND]

# -- scoring --------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 1.0
BOARD_TOP = TOP + BOARD_TOP_MM / 1000
SEAT_Z = 0.0005  # seated: the processor's lowest point within this of the board
SEAT_XY = 0.0008  # its centre within this of the pocket's (the pocket allows 0.25 mm; a jammed one is farther)
SEAT_TILT_DEG = 1.0
SEAT_YAW_DEG = 2.0  # turned within this of the socket's orientation (the keys only take one way round)
STILL = 0.01  # m/s
SHIELD_SHUT_DEG = 3.0
# WHY 2 degrees: under the catch the lever sits at 0 to 0.7 degrees; laid on top of the catch, at about 4.
LEVER_SHUT_DEG = 2.0


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _seated(cpu) -> bool:
    r = _rot(cpu.quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, r[2, 2]))))
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    centre = np.asarray(cpu.pos, float)[:2] - np.asarray(SOCKET_XY)
    return (float(cpu.low[2]) - BOARD_TOP <= SEAT_Z and tilt <= SEAT_TILT_DEG and abs(yaw) <= SEAT_YAW_DEG
            and float(np.hypot(*centre)) <= SEAT_XY and float(np.linalg.norm(cpu.lin_vel)) < STILL)


def score(objects) -> float:
    """0 unless the processor is seated on its keys; 0.4 seated; 0.7 seated with the shield shut over it; 1.0 with
    the lever also down under its catch."""
    if not _seated(objects["processor"]):
        return 0.0
    joints = objects["socket"].joints
    if abs(math.degrees(joints["shield_hinge"])) > SHIELD_SHUT_DEG:
        return 0.4
    if abs(math.degrees(joints["lever_hinge"])) > LEVER_SHUT_DEG:
        return 0.7
    return 1.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo (right arm) ------------------------------------------------------------------------------------
SX, SY = SOCKET_XY
SHIELD_HINGE_W = np.array([SX + SHIELD_HINGE_X / 1000, SY, TOP + SHIELD_Z / 1000])
LIP_FROM_HINGE = np.array([-0.075, 0.0, 0.0034])  # the lip's centre from the shield's hinge, shut
LIP_GRIP_Z = 0.009  # TCP above the lip's centre: the fingertips at its lower edge when shut
PINCH_LIP = -0.003  # the lip is 12 mm across y
LEVER_PIVOT_W = np.array([SX + LEVER_PIVOT_X / 1000, SY + LEVER_Y / 1000, TOP + LEVER_Z / 1000])
HANDLE_FROM_PIVOT = np.array([(LEVER_END_X - LEVER_PIVOT_X) / 1000, -HANDLE_LEN / 2000, KNOB_RISE / 1000])
HANDLE_GRIP_Z = PAD_TIP_PAST_TCP - KNOB_R / 1000 + 0.001  # fingertips 1 mm above the knob's bottom
PINCH_HANDLE = -0.002  # the knob is 8 mm across x
ACROSS_X = math.pi / 2  # fingers closing along x
GRIP_LID = -0.012  # the lid is 30 mm across
RELEASE_LID = -0.019
CPU_GRIP_Z = (STAND_H + SUBSTRATE_T / 1000 + 0.0005) + PAD_TIP_PAST_TCP  # TCP above the table, CPU on its stand
CPU_SEAT_Z = (BOARD_TOP_MM / 1000 + SUBSTRATE_T / 1000 + 0.0005) + PAD_TIP_PAST_TCP
TRAVEL = TOP + 0.10
# WHY 1.25 mm past the socket's centre along x: the arm sets the gripped processor down about 1.2 mm short (toward the
# bases). Aims from 0.25 mm short to 2.5 mm past seated it; 0.5 mm short and 3 mm past did not (official sim, no-frames
# runs, 2026-11-06). 1.25 mm is the middle of that window.
PLACE_X = SX + 0.00125
PRESS = 0.0015  # past shut, to push the lever under its catch


def _turn_y(v, deg):
    t = math.radians(deg)
    return np.array([v[0] * math.cos(t) + v[2] * math.sin(t), v[1], -v[0] * math.sin(t) + v[2] * math.cos(t)])


def lip_tcp(deg):
    return tuple(SHIELD_HINGE_W + _turn_y(LIP_FROM_HINGE, deg) + np.array([0.0, 0.0, LIP_GRIP_Z]))


def handle_tcp(deg, press=0.0):
    """TCP pinching the handle with the lever at ``deg`` (negative opens it)."""
    return tuple(LEVER_PIVOT_W + _turn_y(HANDLE_FROM_PIVOT, deg) + np.array([0.0, 0.0, HANDLE_GRIP_Z - press]))


def _above(p, dz):
    return (p[0], p[1], p[2] + dz)


def demo():
    lever_open = [-12, -30, -50, -70, -90, -110, -125]
    shield_open = [10, 25, 40, 55, 70, 85, 100, 115]
    right = [
        # 1. the lever: pinch its handle and pull it up out from under the catch, then all the way open
        Waypoint(2.0, _above(handle_tcp(0), 0.04), FINGER_OPEN, "over the lever's handle", ACROSS_X),
        Waypoint(1.5, handle_tcp(0), FINGER_OPEN, "down to the handle", ACROSS_X),
        Waypoint(0.8, handle_tcp(0), PINCH_HANDLE, "pinch the handle", ACROSS_X),
        *[Waypoint(0.7, handle_tcp(d), PINCH_HANDLE, f"lever up {-d}", ACROSS_X) for d in lever_open],
        Waypoint(0.8, handle_tcp(-125), FINGER_OPEN, "let the lever go", ACROSS_X),
        Waypoint(1.0, _above(handle_tcp(-125), 0.03), FINGER_OPEN, "up", ACROSS_X),
        # 2. the shield: pinch its lip and swing it open past upright onto its stop
        Waypoint(2.0, _above(lip_tcp(0), 0.05), FINGER_OPEN, "over the shield's lip"),
        Waypoint(1.5, lip_tcp(0), FINGER_OPEN, "down to the lip"),
        Waypoint(0.8, lip_tcp(0), PINCH_LIP, "pinch the lip"),
        *[Waypoint(0.6, lip_tcp(d), PINCH_LIP, f"shield open {d}") for d in shield_open],
        Waypoint(0.8, lip_tcp(115), FINGER_OPEN, "let the shield go"),
        Waypoint(1.2, _above(lip_tcp(115), 0.04), FINGER_OPEN, "up"),
        # 3. the processor: pinch the lid, turn it a quarter turn back, lower it through the cut-outs onto the keys
        Waypoint(2.0, (*STAND_XY, TRAVEL), FINGER_OPEN, "over the processor", CPU_YAW0),
        Waypoint(1.5, (*STAND_XY, TOP + CPU_GRIP_Z), FINGER_OPEN, "down to the lid", CPU_YAW0),
        Waypoint(0.8, (*STAND_XY, TOP + CPU_GRIP_Z), GRIP_LID, "grip the lid", CPU_YAW0),
        Waypoint(1.2, (*STAND_XY, TRAVEL), GRIP_LID, "lift", CPU_YAW0),
        Waypoint(2.0, (PLACE_X, SY, TRAVEL), GRIP_LID, "carry and turn: notches to the keys", 0.0),
        Waypoint(1.2, (PLACE_X, SY, TOP + 0.030), GRIP_LID, "lower", 0.0),
        Waypoint(1.5, (PLACE_X, SY, TOP + CPU_SEAT_Z), GRIP_LID, "seat on the keys", 0.0),
        Waypoint(0.8, (PLACE_X, SY, TOP + CPU_SEAT_Z), RELEASE_LID, "let go", 0.0),
        Waypoint(1.2, (PLACE_X, SY, TRAVEL), RELEASE_LID, "up", 0.0),
        # 4. the shield shut, the way it opened
        Waypoint(1.5, _above(lip_tcp(120), 0.04), FINGER_OPEN, "over the open lip"),
        Waypoint(1.2, lip_tcp(120), FINGER_OPEN, "down to the lip"),
        Waypoint(0.8, lip_tcp(120), PINCH_LIP, "pinch the lip"),
        *[Waypoint(0.6, lip_tcp(d), PINCH_LIP, f"shield shut {d}") for d in (105, 90, 75, 60, 45, 30, 15, 3)],
        Waypoint(0.8, lip_tcp(3), FINGER_OPEN, "let the shield go"),
        Waypoint(1.2, _above(lip_tcp(3), 0.05), FINGER_OPEN, "up"),
        # 5. the lever down and under its catch. WHY from 135: let go at 125, it falls back onto its stop at 135
        Waypoint(1.5, _above(handle_tcp(-135), 0.04), FINGER_OPEN, "over the open lever", ACROSS_X),
        Waypoint(1.2, handle_tcp(-135), FINGER_OPEN, "down to the handle", ACROSS_X),
        Waypoint(0.8, handle_tcp(-135), PINCH_HANDLE, "pinch the handle", ACROSS_X),
        *[Waypoint(0.7, handle_tcp(d), PINCH_HANDLE, f"lever down {-d}", ACROSS_X)
          for d in (-120, -100, -80, -60, -40, -20, -8, -4)],
        Waypoint(1.2, handle_tcp(0, PRESS), PINCH_HANDLE, "press it under the catch", ACROSS_X),
        Waypoint(0.8, handle_tcp(0, PRESS), FINGER_OPEN, "let go", ACROSS_X),
        Waypoint(1.5, _above(handle_tcp(0), 0.06), FINGER_OPEN, "clear", ACROSS_X),
        Waypoint(2.0, (0.20, -0.25, TOP + 0.15), FINGER_OPEN, "out of the way"),
    ]
    left = [Waypoint(1.0, (0.22, 0.30, TOP + 0.15), FINGER_OPEN, "stay clear")]
    return {"left": left, "right": right}
