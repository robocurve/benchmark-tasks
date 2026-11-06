"""On the station table, a memory chip lies upside down (pins up) on a pink anti-static foam block ahead of the left
arm. Ahead of the right arm, a circuit board is fixed to the table with an empty socket for the chip: two slots, one
for each row of pins, with sloped (lead-in) mouths. A notch at one end of the chip marks pin 1, and a notch printed on
the board at one end of the socket shows which way round it goes.

Task: pick the chip up, hand it over in the air to the other hand so that it is turned pins down, and seat it in the
socket, notch to the marked end, then let go. A hand that takes the chip as it lies, from above, holds it with its pins
pointing back at the hand: turned pins down in that hand, the chip has the hand underneath it, where it cannot go into
the socket. So it is turned over by passing it to the other hand.

Run it:  inspect-robots run --embodiment yam_arena -E task=hand_over_a_memory_chip_and_socket_it.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the chip: its origin at the centre of its package; its z axis points the way its pins do (up, at the start) ----
CHIP_L, CHIP_W, CHIP_T, CHIP_MASS = 0.070, 0.032, 0.012, 0.030
PINS_PER_ROW, PIN_PITCH = 8, 0.0065
PIN_ROW_Y = 0.007  # the two rows of pins, either side of the chip's long axis
PIN_X, PIN_Y, PIN_LEN = 0.003, 0.002, 0.008  # each pin: along the chip, across it, how far it sticks out
PIN_XS = [(k - (PINS_PER_ROW - 1) / 2) * PIN_PITCH for k in range(PINS_PER_ROW)]
PINS_HALF = max(PIN_XS) + PIN_X / 2  # 24.25 mm: the pin rows' half-length

# -- the foam block it lies on (pins up), ahead of the left arm ------------------------------------------------------
FOAM_XY, FOAM_SIZE = (0.24, 0.12), (0.046, 0.060, 0.025)  # shorter than the chip: a hand can grip its two ends

# -- the board and its socket (one fixed body; origin on the table under the board's centre) -----------------------
BOARD_XY, BOARD_SIZE = (0.29, -0.13), (0.15, 0.13, 0.003)
BOARD_TOP = TOP + 0.0005 + BOARD_SIZE[2]
SOCK_XY = (0.28, -0.12)  # the socket's centre (world)
SOCK_H, FUNNEL, FLOOR = 0.012, 0.003, 0.002  # its height, the depth of the sloped mouths, the slots' floor
SLOT_IN, SLOT_OUT = 0.004, 0.010  # each slot runs |y| 4..10 mm from the socket's centre line: 6 mm for a 2 mm pin
SLOT_HALF_L = PINS_HALF + 0.002  # 2 mm of play at either end
SOCK_HALF_W, SOCK_HALF_L = 0.013, SLOT_HALF_L + 0.0037  # the socket's outline: 26 x 60 mm
NOTCH_SIGN = +1.0  # pin 1 / the notch: the chip's +x end, and the socket's forward (+x) end

GREEN, GOLD, BLACK, WHITE = (0.05, 0.33, 0.16), (0.85, 0.68, 0.25), (0.06, 0.06, 0.07), (0.93, 0.93, 0.90)
PINK, GREY = (0.95, 0.52, 0.68), (0.55, 0.55, 0.57)

# -- scoring --------------------------------------------------------------------------------------------------------
SEATED_Z = BOARD_TOP + SOCK_H + CHIP_T / 2  # the chip's centre when it is seated (resting on the socket's top)
SEAT_XY_TOL, SEAT_Z_TOL = 0.003, 0.0025
SEAT_TILT, SEAT_YAW = math.radians(6.0), math.radians(6.0)
LET_GO = 0.05  # both grippers' fingertip centres (TCPs) at least this far from the chip's centre
AIR_TILT, AIR_HEIGHT = math.radians(20.0), 0.05  # "turned over in the air": pins down within 20 deg, 5 cm up
SUCCESS_HOLD_S = 1.0


def _chip() -> Body:
    shapes = [BoxShape((0.0, 0.0, 0.0), (CHIP_L, CHIP_W, CHIP_T), BLACK)]
    for y in (-PIN_ROW_Y, PIN_ROW_Y):
        for x in PIN_XS:
            shapes.append(BoxShape((x, y, CHIP_T / 2 + PIN_LEN / 2), (PIN_X, PIN_Y, PIN_LEN), GOLD))
    # pin 1: a half-round notch across the +x end, seen from both faces, and a dot on the top (the side without pins)
    shapes.append(CylShape((NOTCH_SIGN * CHIP_L / 2, 0.0, 0.0), 0.0055, CHIP_T + 0.0008, (0.80, 0.80, 0.80),
                           collide=False))
    shapes.append(CylShape((NOTCH_SIGN * (CHIP_L / 2 - 0.009), 0.009, -CHIP_T / 2 - 0.0003), 0.0022, 0.0006, WHITE,
                           collide=False))
    # a printed label on the top
    shapes.append(BoxShape((-0.004, 0.0, -CHIP_T / 2 - 0.0002), (0.034, 0.016, 0.0004), GREY, collide=False))
    pos = (FOAM_XY[0], FOAM_XY[1], TOP + FOAM_SIZE[2] + CHIP_T / 2 + 0.0005)
    return Body.build("chip", pos=pos, mass=CHIP_MASS, shapes=shapes, friction=(0.5, 0.4), color=BLACK)


def _socket(cx: float, cy: float, z0: float) -> list:
    """The socket's shapes, in the board's frame, its base centred at (cx, cy, z0)."""
    lo = SOCK_H - FUNNEL
    th = 0.0015  # thickness of the sloped plates (their top faces lie on the slopes)

    def slope(p1, p2, normal, width):
        n = np.asarray(normal, float) / np.linalg.norm(normal)
        a = np.asarray(p1, float) + [cx, cy, z0] - n * th / 2
        b = np.asarray(p2, float) + [cx, cy, z0] - n * th / 2
        return plate(a, b, width, th, n, BLACK)

    def box(centre, size, color=BLACK, collide=True):
        return BoxShape((cx + centre[0], cy + centre[1], z0 + centre[2]), size, color, collide=collide)

    run = 2 * SLOT_HALF_L
    shapes = [box((0.0, 0.0, FLOOR / 2), (2 * SOCK_HALF_L, 2 * SOCK_HALF_W, FLOOR)),  # floor
              box((0.0, 0.0, lo / 2), (run, 2 * SLOT_IN, lo)),  # centre ridge, up to the mouths
              box((0.0, 0.0, SOCK_H - FUNNEL / 2), (run, 0.002, FUNNEL))]  # its top, 2 mm wide
    for s in (-1.0, 1.0):
        shapes += [
            slope((0.0, s * SLOT_IN, lo), (0.0, s * 0.001, SOCK_H), (0.0, s, 1.0), run),  # ridge's sloped side
            box((0.0, s * (SLOT_OUT + SOCK_HALF_W) / 2, lo / 2), (run, SOCK_HALF_W - SLOT_OUT, lo)),  # side rail
            slope((0.0, s * SLOT_OUT, lo), (0.0, s * SOCK_HALF_W, SOCK_H), (0.0, -s, 1.0), run),  # rail's slope
            box((s * (SLOT_HALF_L + SOCK_HALF_L) / 2, 0.0, lo / 2), (SOCK_HALF_L - SLOT_HALF_L, 2 * SOCK_HALF_W, lo)),
            slope((s * SLOT_HALF_L, 0.0, lo), (s * (SLOT_HALF_L + FUNNEL), 0.0, SOCK_H), (-s, 0.0, 1.0),
                  2 * SOCK_HALF_W),  # end wall's slope
        ]
    return shapes


def _board() -> Body:
    bx, by = BOARD_XY
    t = BOARD_SIZE[2]
    z0 = 0.0005 + t  # the board's top, in the body frame
    cx, cy = SOCK_XY[0] - bx, SOCK_XY[1] - by
    shapes = [BoxShape((0.0, 0.0, 0.0005 + t / 2), BOARD_SIZE, GREEN)]
    shapes += _socket(cx, cy, z0)
    # printed outline round the socket, with the pin-1 notch at its forward end
    m, w = 0.003, 0.0012
    hl, hw = SOCK_HALF_L + m, SOCK_HALF_W + m
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((cx, cy + s * hw, z0 + 0.0002), (2 * hl, w, 0.0004), WHITE, collide=False))
        shapes.append(BoxShape((cx + s * hl, cy, z0 + 0.0002), (w, 2 * hw, 0.0004), WHITE, collide=False))
    shapes.append(CylShape((cx + NOTCH_SIGN * hl, cy, z0 + 0.0003), 0.0055, 0.0006, WHITE, collide=False))
    shapes.append(CylShape((cx + NOTCH_SIGN * (hl - 0.006), cy + hw - 0.006, z0 + 0.0003), 0.0018, 0.0006, WHITE,
                           collide=False))  # pin 1 dot
    # copper traces from the socket's sides to the board's edges
    for s in (-1.0, 1.0):
        for x in PIN_XS:
            y0, y1 = cy + s * (hw + 0.003), s * (BOARD_SIZE[1] / 2 - 0.004)
            shapes.append(BoxShape((cx + x, (y0 + y1) / 2, z0 + 0.0001), (0.0012, abs(y1 - y0), 0.0002), GOLD,
                                   collide=False))
    # a few other parts, well away from the socket
    for dx, dy in ((-0.055, 0.045), (-0.055, -0.045)):
        shapes.append(BoxShape((dx, dy, z0 + 0.001), (0.014, 0.010, 0.002), BLACK))
    for dx, dy in ((0.058, 0.050), (0.058, -0.050), (0.045, 0.050)):
        shapes.append(CylShape((dx, dy, z0 + 0.004), 0.003, 0.008, (0.15, 0.25, 0.55)))
    return Body.build("board", pos=(bx, by, TOP), mass=0.2, shapes=shapes, kinematic=True, friction=(0.3, 0.3),
                      color=GREEN)


BOARD = _board()
CHIP = _chip()
OBJECTS = [BOARD, Box("foam", pos=(*FOAM_XY, TOP + FOAM_SIZE[2] / 2), size=FOAM_SIZE, color=PINK), CHIP]


# -- scoring ------------------------------------------------------------------------------------------------------
# on_step remembers whether the chip has been turned pins down in the air (the handover) and where the grippers are;
# score() reads that and whether the chip is seated now.

_EPISODE = {"turned": False, "tcps": None}
_DEBUG = {"on": bool(os.environ.get("CHIP_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(turned=False, tcps=None)
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pins_down_tilt(rot) -> float:
    """Angle (rad) between the way the chip's pins point and straight down."""
    return float(math.acos(max(-1.0, min(1.0, -rot[2, 2]))))


@on_step
def _watch(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["chip"].data.root_link_pose_w)[0]
    pos, rot = c[:3] - origin, rotation(c[3:7])
    _EPISODE["tcps"] = _tcps(base, origin)
    if _pins_down_tilt(rot) <= AIR_TILT and pos[2] >= TOP + AIR_HEIGHT:
        _EPISODE["turned"] = True
    if _DEBUG["on"]:  # CHIP_DEBUG=1: twice a second (at 20 Hz) on stderr
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            d = (pos - [SOCK_XY[0], SOCK_XY[1], SEATED_Z]) * 1000
            yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
            tips = [np.round((t - pos) * 1000).astype(int).tolist() for t in _EPISODE["tcps"]]
            print(f"[chip] t={_DEBUG['steps'] / 20:5.1f}s pos={np.round(pos * 100, 1).tolist()} vs seat(mm)="
                  f"{np.round(d, 1).tolist()} pins_down_tilt={math.degrees(_pins_down_tilt(rot)):5.1f} yaw={yaw:6.1f} "
                  f"turned={_EPISODE['turned']} tcp-chip(mm) L,R={tips}", file=sys.stderr, flush=True)


def _state(objects) -> dict:
    chip = objects["chip"]
    pos, rot = np.asarray(chip.pos, float), rotation(chip.quat_xyzw)
    long_axis = rot[:, 0]
    yaw_off = math.atan2(abs(long_axis[1]), abs(long_axis[0]))  # the chip's length against the socket's
    seated = (abs(pos[0] - SOCK_XY[0]) <= SEAT_XY_TOL and abs(pos[1] - SOCK_XY[1]) <= SEAT_XY_TOL
              and abs(pos[2] - SEATED_Z) <= SEAT_Z_TOL and _pins_down_tilt(rot) <= SEAT_TILT and yaw_off <= SEAT_YAW)
    right_way = long_axis[0] * NOTCH_SIGN > 0  # the notch toward the socket's marked end
    tcps = _EPISODE["tcps"]
    let_go = tcps is None or all(np.linalg.norm(t - pos) >= LET_GO for t in tcps)
    return {"seated": seated, "right_way": right_way, "let_go": let_go, "turned": _EPISODE["turned"]}


def _points(s) -> float:
    if s["seated"] and s["right_way"]:
        return 1.0 if s["let_go"] else 0.5
    return 0.25 if s["turned"] else 0.0


def score(objects) -> float:
    """1.0 with the chip seated in its socket, pins down, notch to the socket's marked end, and both grippers let go;
    0.5 seated so but still held; 0.25 if it has at least been turned pins down in the air; else 0.0."""
    return _points(_state(objects))


def success(objects) -> bool:
    """Full score with the chip still."""
    return _points(_state(objects)) >= 1.0 and objects["chip"].resting()


# -- scripted demo --------------------------------------------------------------------------------------------------
# The left hand comes straight down on the chip and grips its two ends (fingers closing along the chip), lifts it and
# turns its gripper to point at the right hand, which turns the chip on edge, pins toward the left hand. The right hand
# slides in level from its side, fingers closing up and down on the chip's long edges; the left lets go and backs
# away. The right hand turns its gripper to point straight down (the chip is now pins down), carries it over the
# socket, lowers it in, lets go and lifts away.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


DOWN_L = grip_rotation(-math.pi / 2)  # left hand straight down, fingers closing along x (the chip's length)
SIDE_L = _rx(-math.pi / 2) @ DOWN_L  # pointing -y (at the right hand), fingers closing along x
SIDE_R = _ry(math.pi / 2) @ _rx(math.pi / 2) @ grip_rotation(math.pi / 2)  # pointing +y, fingers closing along z
DOWN_R = _rx(-math.pi / 2) @ SIDE_R  # straight down, fingers closing along y: the same grip, turned over

HAND_AT = np.array([0.26, 0.0, TOP + 0.20])  # the chip's centre at the handover
CHIP_START = np.array(CHIP.pos)
L_GRIP = CHIP_START + [0.0, 0.0, CHIP_T / 2]  # left TCP on the pin side of the package
L_HAND = HAND_AT + [0.0, CHIP_T / 2, 0.0]  # ... still there at the handover (the pins now point +y)
R_HAND = HAND_AT - [0.0, CHIP_T / 2, 0.0]  # right TCP on the package's top (the side without pins)
R_SEAT = np.array([SOCK_XY[0], SOCK_XY[1], SEATED_Z + CHIP_T / 2])  # ... and there, once the chip is seated
ABOVE, IN_FROM, BACK = 0.04, 0.07, 0.08
PRESS = 0.002  # lower the chip this far past seated: the soft arm presses it home
R_FIX = np.array([0.010, 0.0015, 0.0])  # where the right hand misses the socket by, holding the chip (sag): aim the other way
R_YAW = math.radians(-3.5)  # how far the chip hangs turned in the right hand, about the vertical
LEAD_S, LEAD_STEPS = 3.0, 15

L_TIMES = [1.2, 0.8, 0.3, 1.0, 2.5]  # left: down round the chip, grip, settle, lift, carry to the handover
R_TIMES = [1.2, 0.8, 0.4]  # right: slide in, grip, settle (then the left lets go)
L_LET_GO = [0.6, 1.2]  # left: open, back away
WAIT = 0.4


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _left() -> list[Waypoint]:
    out = _lead_in("left", L_GRIP + [0.0, 0.0, ABOVE], DOWN_L, "over the chip")
    out += [_w(L_TIMES[0], L_GRIP, FINGER_OPEN, "down round its ends", DOWN_L),
            _w(L_TIMES[1], L_GRIP, FINGER_CLOSED, "grip it by its ends", DOWN_L),
            _w(L_TIMES[2], L_GRIP, FINGER_CLOSED, "settle", DOWN_L),
            _w(L_TIMES[3], L_GRIP + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it off the foam", DOWN_L),
            _w(L_TIMES[4], L_HAND, FINGER_CLOSED, "hold it out on edge to the right hand, pins toward me", SIDE_L),
            _w(sum(R_TIMES) + WAIT, L_HAND, FINGER_CLOSED, "hold it while the right hand takes it", SIDE_L),
            _w(L_LET_GO[0], L_HAND, FINGER_OPEN, "let go", SIDE_L),
            _w(L_LET_GO[1], L_HAND + [0.0, BACK, 0.0], FINGER_OPEN, "back away", SIDE_L),
            _w(12.0, L_HAND + [0.0, BACK, 0.0], FINGER_OPEN, "wait", SIDE_L)]
    return out


def _right() -> list[Waypoint]:
    down = _rz(R_YAW) @ DOWN_R
    seat = R_SEAT + R_FIX
    start = R_HAND - [0.0, IN_FROM, 0.0]
    out = _lead_in("right", start, SIDE_R, "level, beside the handover point")
    out += [_w(sum(L_TIMES) + WAIT, start, FINGER_OPEN, "wait for the chip", SIDE_R),
            _w(R_TIMES[0], R_HAND, FINGER_OPEN, "slide in round its long edges", SIDE_R),
            _w(R_TIMES[1], R_HAND, FINGER_CLOSED, "grip it", SIDE_R),
            _w(R_TIMES[2] + sum(L_LET_GO), R_HAND, FINGER_CLOSED, "hold it while the left hand lets go", SIDE_R),
            _w(3.0, seat + [0.0, 0.0, ABOVE], FINGER_CLOSED, "turn it pins down, over the socket", down),
            _w(1.0, seat + [0.0, 0.0, 0.008], FINGER_CLOSED, "pins into the mouths", down),
            _w(1.2, seat - [0.0, 0.0, PRESS], FINGER_CLOSED, "press it home", down),
            _w(0.5, seat - [0.0, 0.0, PRESS], FINGER_CLOSED, "settle", down),
            _w(0.6, seat, FINGER_OPEN, "let go", down),
            _w(1.2, seat + [0.0, 0.0, ABOVE + 0.03], FINGER_OPEN, "lift away", down),
            _w(3.0, seat + [0.0, 0.0, ABOVE + 0.03], FINGER_OPEN, "hold still", down)]
    return out


def demo():
    return {"left": _left(), "right": _right()}
