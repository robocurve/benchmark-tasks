"""On the station table, between the two arms: a cordless drill (22.5 cm long, 22.8 cm tall, green and black) lies on
its left side with its handle toward the robot and its chuck toward the right arm. An orange 18 V battery pack
(7 x 5 x 5.8 cm, 450 g) sits on the handle's foot, nearest the robot, on the left arm's side. It slides off along the
handle, toward the robot, and is locked on until both of its release buttons are pressed at once: two 1 cm square
black buttons, one on each face across the pack (left and right as the robot sees them), 3 mm of travel, 5 N springs.
A blue parts tray (20 x 15 cm, 4 cm walls) stands on the robot's left.

The task: hold the drill with the right hand, squeeze both release buttons with the left gripper (a parallel
gripper closing across the pack presses both), slide the battery off the handle and put it in the tray.

The pack slides on the handle against 10 N of friction, so a drill that is not held slides along with it. Once it has
slid 4 cm it is off the handle and free. The lock lets go only while both buttons are 1.5 mm or more in, and clicks
back if the buttons are let go within 2 mm of the seat. ``bonds.py`` makes the slide, the lock and the buttons, and
``simready.py`` loads the drill (NVIDIA SimReady ``powerTool_drill_a01``) without its moulded-on battery.

Run it:  inspect-robots run --embodiment yam_arena -E task=drill_battery_release.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402  (before bonds: see bonds.py)
from bonds import Sliding, Slotted, joint_enabled, on_reset, on_step, set_joint  # noqa: E402
from looks import dress  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every full-success condition held STILL_S without a break (timed below)
STILL_S = 1.0  # s: the placed, let-go, still pack must stay so this long, timed every control step
CONTROL_HZ = 20  # the control rate the task is run at, for offline checks (a run reads its own step time)

# -- the drill -------------------------------------------------------------------------------------------------------
# The SimReady drill stands upright in its file: chuck along +x, its left side +y, handle down to the battery at
# z = 0. Laid on its left side with the handle toward the robot, its axes go to the table's as x -> -y, y -> -z,
# z -> +x. Positions on the drill below are in its file's frame (metres).
DRILL_QUAT = (-0.5, 0.5, -0.5, 0.5)  # x, y, z, w
DRILL_R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # the same turn as a matrix
DRILL_SIDE = 0.0291  # m: the drill's left side, its lowest point once laid down, from its file's centre plane
DRILL_ORIGIN = (0.155, 0.04, TOP + DRILL_SIDE + 0.0005)  # where its file's origin goes on the table
DRILL_MASS = 1.2  # kg, without the battery
# The drill's own battery is moulded on (part of its body's mesh): these point ranges are its shells, removed at
# spawn. Pinned to SimReady revision 9d4556bf (simready.REVISION).
DRILL_BODY_MESH = "Geometry/body_01_obj_00/body_01_mesh_00"
DRILL_BATTERY_POINTS = (105596, ((5938, 8821), (12665, 20889), (22570, 32343), (67798, 68342)))
DRILL = SimReady(
    "drill",
    pos=DRILL_ORIGIN,
    package="powerTool_drill_a01",
    usd="sm_powerTool_drill_a01_01.usd",
    quat_xyzw=DRILL_QUAT,
    mass=DRILL_MASS,
    strip=((DRILL_BODY_MESH, *DRILL_BATTERY_POINTS),),
)
FOOT_Z = 0.0477  # the handle's foot, its end without the battery (file frame, z up the handle)
FOOT_X = (-0.045, 0.013)  # the foot's extent along the drill (file frame x)
DRILL_LOW = (-0.0972, -0.0289, 0.0477)  # the stripped drill's bounds, file frame (the chuck reaches x = 0.128)
DRILL_HIGH = (0.1280, 0.0291, 0.2280)


def on_drill(p) -> tuple[float, float, float]:
    """A point of the drill's file frame on the table, with the drill at its start."""
    return tuple(float(v) for v in np.asarray(DRILL_ORIGIN) + DRILL_R @ np.asarray(p, float))


# -- the battery pack: on the foot, sliding toward the robot (world -x) ----------------------------------------------
BATTERY_SIZE = (0.05, 0.07, 0.058)  # along the slide (the handle), across the buttons, thick (the drill's width)
BATTERY_MASS = 0.45
FOOT_GAP = 0.0022  # m between the pack's top and the foot it slides on, so the two don't touch at rest
_bat_in_drill = ((FOOT_X[0] + FOOT_X[1]) / 2, 0.0, FOOT_Z - FOOT_GAP - BATTERY_SIZE[0] / 2)
BATTERY_START = on_drill(_bat_in_drill)
TRAVEL = 0.04  # m: slid this far, the pack is off the foot and free
SLIDE_FRICTION = 10.0  # N: the pack's grip on the foot while it slides
BATTERY = Slotted(
    "battery",
    pos=BATTERY_START,
    size=BATTERY_SIZE,
    mass=BATTERY_MASS,
    color=(0.95, 0.40, 0.05),
    to="drill",
    axis="X",
    lower=-0.06,  # past TRAVEL: the slide is switched off at TRAVEL, before it can stop the pack
    upper=0.0,
    friction=SLIDE_FRICTION,
)

# -- the release buttons: one on each face across the pack -----------------------------------------------------------
BUTTON_SIZE = (0.01, 0.003, 0.01)  # 1 cm square, standing 3 mm proud of the pack: flush when pressed in fully
BUTTON_TRAVEL = 0.003
BUTTON_FORCE = 5.0  # N at full travel
BUTTON_RATE = BUTTON_FORCE / BUTTON_TRAVEL
PRESSED = 0.0015  # m in (half its travel): a button counts as pressed
BUTTON_OFFSET = BATTERY_SIZE[1] / 2 + BUTTON_SIZE[1] / 2  # each button's centre from the pack's, across it
BUTTONS = [
    Sliding(
        f"button_{side}",
        pos=(BATTERY_START[0], BATTERY_START[1] + s * BUTTON_OFFSET, BATTERY_START[2]),
        size=BUTTON_SIZE,
        mass=0.005,
        color=(0.05, 0.05, 0.05),
        to="battery",
        axis="Y",
        lower=-BUTTON_TRAVEL if s > 0 else 0.0,  # pressed is toward the pack
        upper=0.0 if s > 0 else BUTTON_TRAVEL,
        rate=BUTTON_RATE,
        damping=2.0,
        collide=False,  # it sinks into the pack's face
    )
    for side, s in (("left", 1), ("right", -1))
]
BUTTON_NAMES = tuple(b.name for b in BUTTONS)

# -- the tray: 20 x 15 cm, 4 cm walls, on the robot's left -------------------------------------------------------------
TRAY_XY = (0.19, 0.25)
TRAY_OUTER = (0.20, 0.15)
TRAY_WALL = 0.008
TRAY_WALL_H = 0.04
TRAY_FLOOR_T = 0.006
TRAY_FLOOR_Z = TOP + TRAY_FLOOR_T
TRAY_BLUE = (0.10, 0.22, 0.48)
_tx, _ty = TRAY_XY
_hx, _hy = TRAY_OUTER[0] / 2 - TRAY_WALL / 2, TRAY_OUTER[1] / 2 - TRAY_WALL / 2
_wz = TOP + TRAY_WALL_H / 2
TRAY = [
    Box("tray_floor", pos=(_tx, _ty, TOP + TRAY_FLOOR_T / 2), size=(*TRAY_OUTER, TRAY_FLOOR_T), color=TRAY_BLUE),
    Box("tray_wall_near", pos=(_tx - _hx, _ty, _wz), size=(TRAY_WALL, TRAY_OUTER[1], TRAY_WALL_H), color=TRAY_BLUE),
    Box("tray_wall_far", pos=(_tx + _hx, _ty, _wz), size=(TRAY_WALL, TRAY_OUTER[1], TRAY_WALL_H), color=TRAY_BLUE),
    Box("tray_wall_right", pos=(_tx, _ty - _hy, _wz), size=(TRAY_OUTER[0] - 2 * TRAY_WALL, TRAY_WALL, TRAY_WALL_H),
        color=TRAY_BLUE),
    Box("tray_wall_left", pos=(_tx, _ty + _hy, _wz), size=(TRAY_OUTER[0] - 2 * TRAY_WALL, TRAY_WALL, TRAY_WALL_H),
        color=TRAY_BLUE),
]

OBJECTS = [DRILL, BATTERY, *BUTTONS, *TRAY]

# What the boxes look like: the pack and the tray are meshes modelled in Blender (looks/make_looks.py) on the boxes'
# colliders. The tray's mesh draws its walls too.
LOOKS = {"battery": "battery.usdc", "tray_floor": "tray.usdc"}
dress(LOOKS, hide=[b.name for b in TRAY[1:]])

# -- scoring ---------------------------------------------------------------------------------------------------------
MOVED = 0.01  # m slid along the handle: the pack has started to come off
SEPARATED = 0.05  # m from its seat on the foot: the pack is off the drill
DRILL_KEPT = 0.10  # m: the drill's centre must stay within this of its start (in the plane of the table)
INNER_HALF = (TRAY_OUTER[0] / 2 - TRAY_WALL, TRAY_OUTER[1] / 2 - TRAY_WALL)  # m: the tray's 18.4 x 13.4 cm floor
FLOOR_TOUCH = 0.0015  # m: the pack's four lowest corners this close to the floor (or less) lie on it
RELEASED_GAP = 0.003  # m: every gripper finger at least this far from the pack and its buttons: let go
OFF_TABLE_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table
RELATCH = 0.002  # m: buttons let go within this of the seat, the lock clicks back

# The drill's centre (its bounding box's) and the pack's at the start. Each reset measures them in the simulation,
# these are the same worked out from the file (and are what the offline tests use).
_drill_centre = on_drill((np.asarray(DRILL_LOW) + np.asarray(DRILL_HIGH)) / 2)
START = {"drill": np.asarray(_drill_centre), "drill_quat": np.asarray(DRILL_QUAT), "battery": np.asarray(BATTERY_START)}

# What the episode remembers: both buttons were pressed together at some step, the pack has slid off its foot,
# whether every finger was clear of the pack at the last step (from on_step: nothing is clear until a step has run),
# and how long every full-success condition has held without a break (s, timed at each control step; any lapse,
# such as a finger touching the pack again or the pack moving, sets it back to 0).
STATE = {"squeezed": False, "off": False, "released": False, "placed_s": 0.0}


@on_reset
def reset_memory(env=None) -> None:
    STATE["squeezed"] = STATE["off"] = STATE["released"] = False
    STATE["placed_s"] = 0.0
    if env is not None:
        from yam_arena.scene import object_states

        objects = object_states(env, ["drill", "battery"])
        if _finite(objects):
            START.update(drill=objects["drill"].pos.copy(), drill_quat=objects["drill"].quat_xyzw.copy(),
                         battery=objects["battery"].pos.copy())


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) for s in objects.values())


def _matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def button_depth(objects, name: str) -> float:
    """How far the button is pressed into the pack (m), measured across the pack."""
    pack = objects["battery"]
    across = _matrix(pack.quat_xyzw).T @ (objects[name].pos - pack.pos)
    return float(BUTTON_OFFSET - abs(across[1]))


def both_pressed(objects) -> bool:
    """Both release buttons 1.5 mm or more in at this step."""
    return all(button_depth(objects, name) >= PRESSED for name in BUTTON_NAMES)


def _seat(objects):
    """Where the pack's centre sits on the drill as the drill now lies, and the slide's outward direction."""
    turn = _matrix(objects["drill"].quat_xyzw) @ _matrix(START["drill_quat"]).T
    seat = objects["drill"].pos + turn @ (START["battery"] - START["drill"])
    return seat, turn @ np.array([-1.0, 0.0, 0.0])


def slide(objects) -> float:
    """How far the pack has slid off its seat along the handle (m)."""
    seat, out = _seat(objects)
    return float((objects["battery"].pos - seat) @ out)


def separation(objects) -> float:
    """How far the pack's centre is from its seat on the drill (m)."""
    seat, _ = _seat(objects)
    return float(np.linalg.norm(objects["battery"].pos - seat))


def drill_moved(objects) -> float:
    """How far the drill's centre has moved from its start, in the plane of the table (m)."""
    return float(np.linalg.norm((objects["drill"].pos - START["drill"])[:2]))


def _pack_corners(pack) -> np.ndarray:
    """The pack's eight corners (8, 3), from its pose and its 7 x 5 x 5.8 cm size."""
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    return pack.pos + (signs * np.asarray(BATTERY_SIZE) / 2) @ _matrix(pack.quat_xyzw).T


def in_tray(objects) -> bool:
    """The pack lies on the tray's floor, inside its walls: all eight corners within the inner 18.4 x 13.4 cm of the
    walls, and the four lowest corners within 1.5 mm of the floor (a flat face down, not leaning or held up)."""
    corners = _pack_corners(objects["battery"])
    inside = np.all(np.abs(corners[:, :2] - np.asarray(TRAY_XY)) <= np.asarray(INNER_HALF))
    height = np.sort(corners[:, 2]) - TRAY_FLOOR_Z
    return bool(inside and height[0] >= -FLOOR_TOUCH and height[3] <= FLOOR_TOUCH)


def finger_gap(objects, fingers) -> float:
    """The closest any finger point comes to the pack with its buttons (m): 0 inside or touching it. ``fingers`` is
    (n, 3) points on the grippers' finger meshes, from ``fingers.py``."""
    if fingers is None or len(fingers) == 0:
        return math.inf
    pack = objects["battery"]
    half = np.array([BATTERY_SIZE[0], BATTERY_SIZE[1] + 2 * BUTTON_SIZE[1], BATTERY_SIZE[2]]) / 2
    local = (np.asarray(fingers, float) - pack.pos) @ _matrix(pack.quat_xyzw)
    return float(np.min(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)))


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return bool(min(objects["drill"].pos[2], objects["battery"].pos[2]) < TOP + OFF_TABLE_Z)


def _remember(objects) -> None:
    if both_pressed(objects):
        STATE["squeezed"] = True


@on_step
def work_the_lock(env) -> None:
    """The lock lets go while both buttons are pressed and clicks back when they are let go near the seat. Slid
    4 cm, the pack leaves the foot: its slide is switched off for the rest of the episode. Every step the fingers
    are checked against the pack: let go when none is within 3 mm of it."""
    from fingers import finger_points
    from yam_arena.scene import object_states

    objects = object_states(env, ["drill", "battery", *BUTTON_NAMES])
    if not _finite(objects):
        STATE["placed_s"] = 0.0
        return
    STATE["released"] = finger_gap(objects, finger_points(env)) >= RELEASED_GAP
    tick(objects, _step_s(env))
    if STATE["off"]:
        return
    s = slide(objects)
    if s >= TRAVEL:
        STATE["off"] = True
        set_joint("battery", "lock", False)
        set_joint("battery", "slide", False)
    elif both_pressed(objects):
        set_joint("battery", "lock", False)
    elif s < RELATCH and not joint_enabled("battery", "lock"):
        set_joint("battery", "lock", True)


def _done(objects) -> bool:
    return bool(separation(objects) >= SEPARATED and in_tray(objects) and STATE["released"]
                and objects["battery"].resting())


def _met(objects) -> bool:
    """Every full-success condition as things stand: squeezed at some step, the pack off the drill, let go by every
    finger, lying still on the tray's floor inside its walls, the drill within 10 cm of its start, no violation."""
    return bool(not _violation(objects) and STATE["squeezed"] and _done(objects)
                and drill_moved(objects) < DRILL_KEPT)


def _step_s(env) -> float:
    """The control step (s) of a run, 1 / CONTROL_HZ when the env does not say."""
    dt = getattr(getattr(env, "unwrapped", env), "step_dt", None)
    return float(dt) if dt else 1.0 / CONTROL_HZ


def tick(objects, dt: float) -> None:
    """Once per control step (from the step hook): add the step to the time every full-success condition has held
    without a break, or start the count over when one fails."""
    _remember(objects)
    STATE["placed_s"] = STATE["placed_s"] + dt if _met(objects) else 0.0


def success(objects) -> bool:
    """Every full-success condition (``_met``) holds now and has held without a break for STILL_S (1 s, 20 control
    steps), timed by the step hook: the pack off the drill, let go, still on the tray's floor inside its walls."""
    if _violation(objects):
        return False
    _remember(objects)
    return bool(_met(objects) and STATE["placed_s"] >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0: success (placed, let go and still for 1 s without a break). 0.5: the pack is off the drill (5 cm or more
    from its seat) but not placed (held, in the air, on the table, leaning, still moving, or placed for less than
    1 s), or it is placed but the drill moved 10 cm or more. 0.25: both buttons have been pressed together at some
    step and the pack has slid 1 cm or more along the handle. 0 otherwise, and 0 if the drill or the pack fell off
    the table."""
    if _violation(objects):
        return 0.0
    _remember(objects)
    if not STATE["squeezed"]:
        return 0.0
    if success(objects):
        return 1.0
    if separation(objects) >= SEPARATED:
        return 0.5
    return 0.25 if slide(objects) >= MOVED else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm: fingers closing along x (a quarter turn), straight down round the drill's clutch ring (just behind the
# chuck), holds the drill there until the pack is off. Left arm: fingers closing along y across the pack, over the
# two buttons, closes (which presses both), pulls 7 cm along the handle toward the robot, lifts, carries the pack to
# the tray, lowers it and lets go. Numbers are TCP positions. The arms are softly position controlled: the pull is
# sent 7 cm so the pack slides past its 4 cm of travel against the slide's friction.
HOLD = True  # the right arm holds the drill (False: it stays up, the shortcut of pulling on a loose drill)
RING_IN_DRILL = (0.055, 0.0, 0.197)  # the clutch ring's centre, file frame
RING_XY = on_drill(RING_IN_DRILL)[:2]
GRIP_TIPS_UP = 0.008  # m: the pad tips stop this far above the table
HOLD_Z = TOP + GRIP_TIPS_UP + 0.002 + PAD_TIP_PAST_TCP
PACK_Z = TOP + GRIP_TIPS_UP + PAD_TIP_PAST_TCP
PULL = 0.07
TRAVEL_Z = TOP + 0.12
CARRY_Z = TOP + 0.11  # the pack's bottom 9 cm up: over the tray's walls
_pack_below_tcp = PACK_Z - (BATTERY_START[2] - BATTERY_SIZE[2] / 2)  # the pack's bottom under the TCP, as gripped
PLACE_Z = TRAY_FLOOR_Z + 0.004 + _pack_below_tcp
HOLD_YAW = math.pi / 2


def demo():
    w = Waypoint
    rx, ry = RING_XY
    bx, by = BATTERY_START[:2]
    tx, ty = TRAY_XY
    hold_z = HOLD_Z if HOLD else TRAVEL_Z
    return {
        "right": [
            w(1.5, (rx, ry, TRAVEL_Z), FINGER_OPEN, "over the clutch ring", HOLD_YAW),
            w(1.5, (rx, ry, hold_z), FINGER_OPEN, "down round it", HOLD_YAW),
            w(0.8, (rx, ry, hold_z), FINGER_CLOSED, "hold the drill", HOLD_YAW),
            w(4.2, (rx, ry, hold_z), FINGER_CLOSED, "keep holding", HOLD_YAW),
            w(0.6, (rx, ry, hold_z), FINGER_OPEN, "let go", HOLD_YAW),
            w(1.0, (rx, ry, TRAVEL_Z), FINGER_OPEN, "retreat", HOLD_YAW),
        ],
        "left": [
            w(1.5, (bx, by, TRAVEL_Z), FINGER_OPEN, "over the pack"),
            w(1.5, (bx, by, PACK_Z), FINGER_OPEN, "down across the buttons"),
            w(0.8, (bx, by, PACK_Z), FINGER_CLOSED, "squeeze both buttons"),
            w(0.5, (bx, by, PACK_Z), FINGER_CLOSED, "wait"),
            w(2.5, (bx - PULL, by, PACK_Z + 0.002), FINGER_CLOSED, "slide it off"),
            w(1.2, (bx - PULL, by, CARRY_Z), FINGER_CLOSED, "lift"),
            w(1.8, (tx, ty, CARRY_Z), FINGER_CLOSED, "carry"),
            w(1.2, (tx, ty, PLACE_Z), FINGER_CLOSED, "lower"),
            w(0.6, (tx, ty, PLACE_Z), FINGER_OPEN, "release"),
            w(1.0, (tx, ty, TRAVEL_Z), FINGER_OPEN, "retreat"),
        ],
    }
