"""On the station table, in front of the robot and a little to its right: a small green circuit board (8 x 6 cm, 1.6 mm
thick) stands upright in a black board holder that clamps its bottom edge. The board is turned 45 degrees: its
components side faces forward and to the robot's left, its solder side back and to the robot's right. On the
components side, near the board's left end and 1.5 cm below its top edge, sits a red 5 mm LED whose two legs pass
through two holes in the board. It is loose: it has slid 3 mm back out of the board, and nothing holds it in. On the
solder side, the LED's two legs end in two tinned pads (2 mm across, 2.54 mm apart). The board also carries a
resistor, a capacitor, a pin header and a small chip, already soldered. On the robot's right, a cold soldering iron
(22 cm, blue handle, steel barrel, 4 mm tip) rests in its stand: its tip and barrel inside a steel spiral, its handle
on a rest, pointing the same way as the board's components side faces.

The task: press the LED flush against the board with one hand and touch the iron's tip to one of its pads for 2 s
with the other, then put the iron back in its stand. The iron's tip must not touch the LED, the other parts' joints,
the table or a gripper, and the LED must not come out of its holes.

``bonds.py`` puts the LED on a sliding joint (its legs in their holes), ``compound.py`` builds the iron and the LED
from several box colliders, ``fingers.py`` reads where the grippers' fingers are, and ``looks.py`` draws the parts.

Run it:  inspect-robots run --embodiment yam_arena -E task=hold_led_solder_pad.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import Sliding, on_reset, on_step, release, released  # noqa: E402
from compound import compound  # noqa: E402
from looks import dress  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the frame of the board, its holder and the iron's stand ----------------------------------------------------------
# All of them are turned 45 degrees (TURN): their own +y is N, forward and to the robot's left, and their own +x is U,
# forward and to the robot's right. The board's components side faces N, its solder side -N, and the iron points
# along N, tip first, in its stand and at the board alike. So the left arm presses the LED from the far side of the
# board, the right arm holds the iron on the near side, and each stays in its own half.
TURN = -math.pi / 4
U = np.array([math.cos(TURN), math.sin(TURN), 0.0])
N = np.array([-math.sin(TURN), math.cos(TURN), 0.0])
Z = np.array([0.0, 0.0, 1.0])


def at(origin, u: float, n: float) -> tuple[float, float]:
    """The table point ``u`` along U and ``n`` along N from ``origin`` (x, y)."""
    return (float(origin[0] + u * U[0] + n * N[0]), float(origin[1] + u * U[1] + n * N[1]))


# -- the board and its holder --------------------------------------------------------------------------------------
BOARD_XY = (0.34, -0.015)
BOARD_SIZE = (0.08, 0.0016, 0.06)  # along U, thick (along N), tall
BOARD_T = BOARD_SIZE[1]
BOARD_BOTTOM = TOP + 0.022  # 1.2 cm of it inside the holder's jaws
BOARD_Z = BOARD_BOTTOM + BOARD_SIZE[2] / 2
BOARD = Box("board", pos=(*BOARD_XY, BOARD_Z), size=BOARD_SIZE, yaw=TURN, color=(0.08, 0.36, 0.17))

HOLDER_BASE = (0.12, 0.07, 0.012)
JAW = (0.10, 0.008, 0.022)
JAW_TOP = TOP + HOLDER_BASE[2] + JAW[2]
HOLDER = [
    Box("holder_base", pos=(*BOARD_XY, TOP + HOLDER_BASE[2] / 2), size=HOLDER_BASE, yaw=TURN, color=(0.1, 0.1, 0.11)),
    *(Box(f"holder_jaw_{side}", pos=(*at(BOARD_XY, 0.0, s * (BOARD_T / 2 + JAW[1] / 2)), TOP + HOLDER_BASE[2] + JAW[2] / 2),
          size=JAW, yaw=TURN, color=(0.1, 0.1, 0.11)) for side, s in (("front", 1), ("back", -1))),
]

# -- the LED: a 5 mm red LED, its legs in two holes, slid 3 mm back out of the board ----------------------------------
# One box spans the LED's body (5.8 mm flange, 8.6 mm tall) and its two legs (4.6 mm past the body's back). Only the
# body collides (compound.py), so the legs pass through the board. A sliding joint along N (bonds.py) is the legs in
# their holes: the LED moves in and out freely (a little friction), from flush against the board to 12 mm out, and a
# knock of 8 N or 0.05 N m breaks it loose.
LED_U, LED_Z = -0.015, BOARD_Z + 0.015  # where its axis meets the board
LED_BODY = (0.0058, 0.0086, 0.0058)
LED_LEGS = 0.0046  # m of leg behind the body: 3 mm out of the solder side when the LED is flush
LED_LEN = LED_BODY[1] + LED_LEGS
LED_GAP = 0.003  # m: how far it has slid out at the start
FLUSH_N = BOARD_T / 2 + LED_BODY[1] - LED_LEN / 2  # the box's centre along N when the LED is flush
BODY_DN = LED_LEN / 2 - LED_BODY[1] / 2  # the body's centre, from the box's centre, along its own +y
LED_OUT = 0.010  # m out from flush: the legs have left their holes
LED = compound(Sliding(
    "led", pos=(*at(BOARD_XY, LED_U, FLUSH_N + LED_GAP), LED_Z), size=(LED_BODY[0], LED_LEN, LED_BODY[2]), yaw=TURN,
    mass=0.005, color=(0.85, 0.05, 0.04), axis="Y", lower=-(LED_GAP + 0.0005), upper=0.012 - LED_GAP, rate=0.0,
    damping=0.2, hold=8.0, twist=0.05,
), [("body", (0.0, BODY_DN, 0.0), LED_BODY)])

PAD_PITCH = 0.00254
PADS = np.array([[*at(BOARD_XY, LED_U + s * PAD_PITCH / 2, -BOARD_T / 2), LED_Z] for s in (-1, 1)])

# The board's other parts, already soldered: their bodies on the components side as boxes (u, v from the board's
# centre, size along U, N and up) and their joints on the solder side (u, v).
PARTS = {
    "resistor": dict(body=((0.017, -0.009), (0.0063, 0.0025, 0.0025)), joints=[(0.0119, -0.009), (0.0221, -0.009)]),
    "capacitor": dict(body=((-0.020, -0.008), (0.005, 0.011, 0.005)), joints=[(-0.021, -0.008), (-0.019, -0.008)]),
    "header": dict(body=((0.028, 0.017), (0.0051, 0.0085, 0.0025)), joints=[(0.0267, 0.017), (0.0293, 0.017)]),
    "chip": dict(body=((0.015, 0.010), (0.0094, 0.0040, 0.0064)),
                 joints=[(0.015 + k * 0.00254, 0.010 + s * 0.0038) for k in (-1.5, -0.5, 0.5, 1.5) for s in (-1, 1)]),
}
JOINTS = np.array([[*at(BOARD_XY, u, -BOARD_T / 2), BOARD_Z + v] for p in PARTS.values() for u, v in p["joints"]])
BODIES = [(np.array([*at(BOARD_XY, u, BOARD_T / 2 + size[1] / 2), BOARD_Z + v]), np.array(size) / 2)
          for p in PARTS.values() for (u, v), size in [p["body"]]]

# -- the soldering iron: one 22 x 2 x 2 cm box, colliding as a handle, a barrel and a tip ---------------------------
STAND_XY = (0.36, -0.28)  # the iron's centre when it lies in its stand
IRON_Z = TOP + 0.055
IRON_SIZE = (0.02, 0.22, 0.02)  # across, long (its own +y, the tip's end), tall
HANDLE = (0.02, 0.11, 0.02)
BARREL = (0.008, 0.065, 0.008)
TIP = (0.004, 0.045, 0.004)
IRON_PARTS = [
    ("handle", (0.0, -IRON_SIZE[1] / 2 + HANDLE[1] / 2, 0.0), HANDLE),
    ("barrel", (0.0, -IRON_SIZE[1] / 2 + HANDLE[1] + BARREL[1] / 2, 0.0), BARREL),
    ("tip", (0.0, IRON_SIZE[1] / 2 - TIP[1] / 2, 0.0), TIP),
]
TIP_LOCAL = np.array([0.0, IRON_SIZE[1] / 2, 0.0])  # the tip's point, the centre of its end
IRON = compound(Box("iron", pos=(*STAND_XY, IRON_Z), size=IRON_SIZE, yaw=TURN, mass=0.06, color=(0.1, 0.3, 0.8)),
                IRON_PARTS)

# -- the stand: a cast base, a spiral round the tip and barrel, a rest under the handle ------------------------------
# Parts given along the iron (ly, from the iron's centre toward its tip) and across it (lx). The barrel lies on the
# spiral's bottom and the handle on the rest, on either side of the iron's centre of mass.
COIL_Y = (0.01, 0.12)  # the spiral's ends along the iron: the tip's end lies 1 cm inside
COIL_R = 0.012  # m: the spiral's inside, from its axis
COIL_FLOOR = IRON_Z - BARREL[2] / 2
COIL_AXIS_Z = COIL_FLOOR + COIL_R
REST_Y = (-0.095, -0.075)
REST_TOP = IRON_Z - HANDLE[2] / 2
LIP_UP = 0.005  # m: the rest's lips stand this far above its top, either side of the handle
STAND_BASE = (0.07, 0.19, 0.012)
BASE_TOP = TOP + STAND_BASE[2]
WIRE = 0.003


def _stand(name, lx, ly, z0, z1, size_xy, color=(0.62, 0.63, 0.65)):
    return Box(f"stand_{name}", pos=(*at(STAND_XY, lx, ly), (z0 + z1) / 2), size=(*size_xy, z1 - z0), yaw=TURN,
               color=color)


_cy, _cl = sum(COIL_Y) / 2, COIL_Y[1] - COIL_Y[0]
_ry, _rl = sum(REST_Y) / 2, REST_Y[1] - REST_Y[0]
STAND = [
    _stand("base", 0.0, -0.005, TOP, BASE_TOP, STAND_BASE[:2], (0.12, 0.12, 0.13)),
    _stand("rest", 0.0, _ry, BASE_TOP, REST_TOP, (0.016, _rl)),
    *(_stand(f"rest_lip_{k}", s * 0.015, _ry, BASE_TOP, REST_TOP + LIP_UP, (0.004, _rl)) for k, s in (("a", 1), ("b", -1))),
    _stand("coil_floor", 0.0, _cy, COIL_FLOOR - WIRE, COIL_FLOOR, (2 * COIL_R + 2 * WIRE, _cl)),
    _stand("coil_top", 0.0, _cy, COIL_FLOOR + 2 * COIL_R, COIL_FLOOR + 2 * COIL_R + WIRE, (2 * COIL_R + 2 * WIRE, _cl)),
    *(_stand(f"coil_side_{k}", s * (COIL_R + WIRE / 2), _cy, COIL_FLOOR, COIL_FLOOR + 2 * COIL_R, (WIRE, _cl))
      for k, s in (("a", 1), ("b", -1))),
    _stand("coil_post", 0.0, 0.05, BASE_TOP, COIL_FLOOR - WIRE, (0.008, 0.01)),
]

OBJECTS = [BOARD, *HOLDER, *STAND, LED, IRON]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The board's
# mesh draws its parts, the holder's base its jaws, and the stand's base the rest of the stand.
LOOKS = {"board": "board.usdc", "led": "led.usdc", "iron": "iron.usdc", "holder_base": "holder.usdc",
         "stand_base": "stand.usdc"}
dress(LOOKS, hide=[b.name for b in (*HOLDER[1:], *STAND[1:])])

# -- scoring ---------------------------------------------------------------------------------------------------------
PAD_REACH = 0.002  # m: the tip's point this close to a pad's centre touches it
FLUSH_TOL = 0.001  # m: the LED this close to flush is flush
HELD_REACH = 0.004  # m: a finger this close to the LED's body is on it (the finger meshes' points are 2 mm apart)
SOLDER_S = 2.0  # s of contact without a break
LED_TOUCH = 0.002  # m: the tip this close to the LED's body (on the components side) touched it
ZONE = 0.01  # m: the no-touch zone round the other parts' joints and bodies
TABLE_TOUCH = 0.002  # m: the tip's point this close to the table touched it
FINGER_TOUCH = 0.003  # m: the tip's point this close to a finger touched it
OFF_AXIS = 0.002  # m: the LED this far off its holes' axis has come out
HANDLE_REST = np.array([0.0, sum(REST_Y) / 2, -HANDLE[2] / 2])  # the handle's underside over the rest (iron frame)
REST_REACH = 0.015

# What the episode remembers: how long the contact has lasted without a break, whether it ever happened, whether it
# reached 2 s (soldered), and why the trial failed.
STATE = {"contact_s": 0.0, "touched": False, "soldered": False, "failed": ""}


@on_reset
def reset_memory(env=None) -> None:
    STATE.update(contact_s=0.0, touched=False, soldered=False, failed="")


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(objects[n].pos)) and np.all(np.isfinite(objects[n].quat_xyzw))
               for n in ("led", "iron"))


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def tip(objects) -> np.ndarray:
    """The iron's tip point, from the iron's centre and turn."""
    return np.asarray(objects["iron"].pos) + _rot(objects["iron"]) @ TIP_LOCAL


def board_coords(p) -> np.ndarray:
    """A point as (u along the board, n out of its components side, height above the board's centre)."""
    d = np.asarray(p, float) - np.array([*BOARD_XY, BOARD_Z])
    return np.array([d @ U, d @ N, d[2]])


def led_out(objects) -> float:
    """How far the LED stands out from flush (m), along its holes."""
    return float(board_coords(objects["led"].pos)[1] - FLUSH_N)


def led_fell(objects) -> bool:
    """The LED has left its holes: pulled out 10 mm, knocked 2 mm off their axis, or turned 10 degrees."""
    b = board_coords(objects["led"].pos)
    turned = math.degrees(math.acos(np.clip(_rot(objects["led"])[:, 1] @ N, -1.0, 1.0)))
    return bool(led_out(objects) >= LED_OUT or abs(b[0] - LED_U) > OFF_AXIS or abs(b[2] + BOARD_Z - LED_Z) > OFF_AXIS
                or turned > 10.0)


def _box_distance(points, centre, half, rot=None) -> np.ndarray:
    local = np.atleast_2d(np.asarray(points, float)) - centre
    if rot is not None:
        local = local @ rot
    return np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)


def _led_body(objects):
    led = objects["led"]
    rot = _rot(led)
    return np.asarray(led.pos) + rot @ np.array([0.0, BODY_DN, 0.0]), np.array(LED_BODY) / 2, rot


def led_held(objects, fingers) -> bool:
    """A gripper finger within 4 mm of the LED's body."""
    if fingers is None or len(fingers) == 0:
        return False
    return bool(_box_distance(fingers, *_led_body(objects)).min() < HELD_REACH)


def on_pad(objects) -> bool:
    """The tip's point within 2 mm of either of the LED's pads' centres."""
    return bool(np.linalg.norm(PADS - tip(objects), axis=1).min() < PAD_REACH)


def hazard(objects, fingers=None) -> str:
    """Why the trial fails now, or "": the tip on the LED, in the no-touch zone, on the table or a finger, or the LED
    out of its holes."""
    if led_fell(objects):
        return "the LED came out of its holes"
    t = tip(objects)
    if board_coords(t)[1] > 0 and _box_distance(t, *_led_body(objects))[0] < LED_TOUCH:
        return "the tip touched the LED"
    if np.linalg.norm(JOINTS - t, axis=1).min() < ZONE or min(_box_distance(t, c, h)[0] for c, h in BODIES) < ZONE:
        return "the tip entered the no-touch zone"
    if t[2] < TOP + TABLE_TOUCH:
        return "the tip touched the table"
    if fingers is not None and len(fingers) and np.linalg.norm(np.asarray(fingers) - t, axis=1).min() < FINGER_TOUCH:
        return "the tip touched a gripper"
    return ""


def observe(objects, fingers=None, dt: float = 0.05) -> None:
    """One control step of the episode's memory: the failure rules, and the 2 s contact timer."""
    if STATE["failed"] or not _finite(objects):
        return
    STATE["failed"] = hazard(objects, fingers)
    if STATE["failed"] or STATE["soldered"]:
        return
    contact = on_pad(objects) and abs(led_out(objects)) <= FLUSH_TOL and led_held(objects, fingers)
    STATE["contact_s"] = STATE["contact_s"] + dt if contact else 0.0
    STATE["touched"] = STATE["touched"] or contact
    if STATE["contact_s"] >= SOLDER_S - 1e-6:
        STATE["soldered"] = True


@on_step
def watch(env) -> None:
    """Every control step: where the LED, the iron and the fingers are, into the episode's memory. An LED pulled 10 mm
    out has left its holes: its joint lets go and it drops."""
    from fingers import finger_points
    from yam_arena.scene import object_states

    objects = object_states(env, ["led", "iron"])
    if _finite(objects) and led_out(objects) >= LED_OUT and not released("led"):
        release("led")
    observe(objects, finger_points(env), float(env.unwrapped.step_dt))


def in_stand(objects) -> bool:
    """The tip's point inside the spiral, between its ends, and the handle's underside within 1.5 cm of the rest."""
    t = tip(objects)
    d = t - np.array([*STAND_XY, 0.0])
    lx, ly = d @ U, d @ N
    inside = abs(lx) <= COIL_R and abs(t[2] - COIL_AXIS_Z) <= COIL_R and COIL_Y[0] <= ly <= COIL_Y[1]
    handle = np.asarray(objects["iron"].pos) + _rot(objects["iron"]) @ HANDLE_REST
    rest = np.array([*at(STAND_XY, 0.0, HANDLE_REST[1]), REST_TOP])
    return bool(inside and np.linalg.norm(handle - rest) < REST_REACH)


def success(objects) -> bool:
    """Soldered, then the iron lies still in its stand."""
    if not _finite(objects) or STATE["failed"] or not STATE["soldered"]:
        return False
    return bool(in_stand(objects) and objects["iron"].resting())


def score(objects) -> float:
    """1.0: success. 0.5: soldered, the iron not still in its stand. 0.25: the tip touched a pad while the LED was
    held flush, but not for 2 s. 0 otherwise, and for the rest of the trial once a failure rule is broken."""
    if not _finite(objects):
        return 0.0
    if not STATE["failed"]:
        STATE["failed"] = hazard(objects)
    if STATE["failed"]:
        return 0.0
    if STATE["soldered"]:
        return 1.0 if in_stand(objects) and objects["iron"].resting() else 0.5
    return 0.25 if STATE["touched"] else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Left arm, fingers closed, gripper turned so its fingers close along N: down beside the LED on the components side,
# then the right finger's outer face pushes the LED 3 mm in, flush, and leans on it until the iron has left. Right arm,
# gripper turned across the iron: grip the handle at the iron's centre of mass, lift it 11 mm, pull it back out of the
# spiral along its length, carry it over to the solder side, set the tip's end on the pad and hold 3 s, then take the
# same way back and slide it into the spiral. The iron keeps its turn the whole way, so the right wrist never turns.
# Numbers are TCP positions.
YAW_TRIM = math.radians(-4.0)  # the right wrist lags 4 degrees about the vertical: sent this far off, the iron points along N
L_YAW, R_YAW = TURN, TURN + math.pi / 2 + YAW_TRIM
HOLD_LED = True  # False: the left arm stays back (the one-handed shortcut, which scores 0)
FACE_OFF = 0.006  # m: the TCP behind the right finger's outer face, at the LED's top edge
PRESS_L = 0.002  # m: the TCP is sent this far past where the face meets the flush LED
LED_END_N = BOARD_T / 2 + LED_BODY[1]  # the LED's end, flush
L_Z = LED_Z + 0.006
L_PRESS_N = LED_END_N + FACE_OFF - PRESS_L
L_BACK = 0.03

GRIP_LY = -0.045  # m along the iron from its centre: the handle, over the centre of mass
TIP_FROM_GRIP = IRON_SIZE[1] / 2 - GRIP_LY
LIFT = 0.011  # m: the fingertips then clear the rest's lips, and the barrel the spiral's top
WITHDRAW = 0.125  # m back along the iron: the tip's end 2.5 cm behind the spiral's mouth
TRAVEL_Z = TOP + 0.10  # the carried iron's handle clears the spiral by 1.2 cm
AIM = (0.0, 0.0017)  # m: the tip's aim off the near pad's centre (along U, up): the held iron sags 1.7 mm (tuned)
PRESS = 0.002  # m: the TCP is sent this far past where the tip meets the board
PAD_AIM = PADS[0]
APPROACH = 0.03
HOLD_S = 3.0


def _r(n: float, z: float, u: float = 0.0) -> tuple[float, float, float]:
    """A right-arm TCP on the line through the near pad along N: ``n`` from the solder side, ``u`` along the board."""
    p = PAD_AIM + (u + AIM[0]) * U + n * N
    return (float(p[0]), float(p[1]), z + AIM[1])


def demo():
    w = Waypoint
    gx, gy = at(STAND_XY, 0.0, GRIP_LY)
    ox, oy = at(STAND_XY, 0.0, GRIP_LY - WITHDRAW)
    zc, zl = IRON_Z, IRON_Z + LIFT
    contact = -TIP_FROM_GRIP + PRESS
    right = [
        w(2.0, (gx, gy, zc + 0.06), FINGER_OPEN, "over the handle", R_YAW),
        w(1.2, (gx, gy, zc), FINGER_OPEN, "down round the handle", R_YAW),
        w(0.8, (gx, gy, zc), FINGER_CLOSED, "close", R_YAW),
        w(0.8, (gx, gy, zl), FINGER_CLOSED, "lift", R_YAW),
        w(2.0, (ox, oy, zl), FINGER_CLOSED, "out of the spiral", R_YAW),
        w(1.0, (ox, oy, TRAVEL_Z), FINGER_CLOSED, "up", R_YAW),
        w(2.5, _r(contact - APPROACH, TRAVEL_Z - AIM[1]), FINGER_CLOSED, "over to the board", R_YAW),
        w(1.0, _r(contact - APPROACH, LED_Z), FINGER_CLOSED, "down to the pad's height", R_YAW),
        w(1.5, _r(contact, LED_Z), FINGER_CLOSED, "tip on the pad", R_YAW),
        w(HOLD_S, _r(contact, LED_Z), FINGER_CLOSED, "hold", R_YAW),
        w(1.0, _r(contact - APPROACH, LED_Z), FINGER_CLOSED, "tip off", R_YAW),
        w(1.0, _r(contact - APPROACH, TRAVEL_Z - AIM[1]), FINGER_CLOSED, "up again", R_YAW),
        w(2.5, (ox, oy, TRAVEL_Z), FINGER_CLOSED, "back behind the spiral", R_YAW),
        w(1.0, (ox, oy, zl), FINGER_CLOSED, "down to the spiral's axis", R_YAW),
        w(2.0, (gx, gy, zl), FINGER_CLOSED, "into the spiral", R_YAW),
        w(1.0, (gx, gy, zc), FINGER_CLOSED, "set it down", R_YAW),
        w(0.6, (gx, gy, zc), FINGER_OPEN, "let go", R_YAW),
        w(1.0, (gx, gy, zc + 0.06), FINGER_OPEN, "retreat", R_YAW),
    ]
    t_off = sum(s.duration for s in right[:11])  # the tip is off the pad
    lx, ly = at(BOARD_XY, LED_U, L_PRESS_N + L_BACK)
    px, py = at(BOARD_XY, LED_U, L_PRESS_N)
    left = [
        w(2.5, (lx, ly, L_Z + 0.06), FINGER_CLOSED, "over the LED's side", L_YAW),
        w(1.5, (lx, ly, L_Z), FINGER_CLOSED, "down beside it", L_YAW),
        w(2.0, (px, py, L_Z), FINGER_CLOSED, "push it flush", L_YAW),
        w(t_off + 0.5 - 6.0, (px, py, L_Z), FINGER_CLOSED, "hold it flush", L_YAW),
        w(1.0, (lx, ly, L_Z), FINGER_CLOSED, "let go", L_YAW),
        w(1.0, (lx, ly, L_Z + 0.06), FINGER_CLOSED, "up", L_YAW),
    ]
    if not HOLD_LED:
        left = [w(left[0].duration, left[0].pos, FINGER_CLOSED, "stay back", L_YAW)]
    return {"right": right, "left": left}
