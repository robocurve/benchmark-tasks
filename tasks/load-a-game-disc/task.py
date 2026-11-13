"""On the station table: a games console standing upright, with a slot-loading disc drive whose slot is in its front
face, and beside it, lying open on a storage box, a game case with the game's disc on its hub.

The disc has to go from its case into the console. A game case holds its disc on a hub with spring teeth: the disc
comes off only while the hub's centre button is pressed (the teeth draw in), and lifting it without pressing the
button just lifts the case with it. The disc is 120 mm across, wider than the gripper opens, and 1.2 mm thick, so it
can only be pinched at its edge, which the case's thumb cut-out leaves free. The console's slot is 5 mm wide behind a
flared mouth: the disc has to go in upright, square to the slot and centred, until the drive takes it (7 cm in).

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable DISC_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import disckit as K  # noqa: E402
from disckit import Case, Disc, Fixed, body_pose, on_reset, on_step, write_joint, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Take the game disc out of its case and put it into the console."

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("DISC_LOT", "A")
LOTS = {"A": dict(hub=(0.430, 0.210)),
        "B": dict(hub=(0.420, 0.220))}
if LOT not in LOTS:
    raise ValueError(f"DISC_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
HUB_XY = np.array(LOTS[LOT]["hub"])

# -- the storage box the case lies on ----------------------------------------------------------------------------------
STAND_H = 0.085
TRAY = (0.135, 0.171)                   # the case's tray half (x, y); the cover half lies beyond the spine on +y
CASE_MASS = 0.070

# -- the console: standing upright, slot in its front face (facing the arms) -------------------------------------------
FACE_X = 0.470                          # the front face
SLOT_Y = 0.060                          # the slot's centre line
SLOT_Z = 0.100                          # height of the slot's middle (and of a loaded disc's centre) above the table
SLOT_W, SLOT_H, BAY = 0.005, 0.132, 0.135   # slot width and height; the drive bay behind it is as deep as this
MOUTH_W, MOUTH_D = 0.012, 0.008         # the slot's flared mouth: this wide at the face, narrowing to SLOT_W this deep
CONSOLE = dict(width=0.104, depth=0.260, height=0.300)

# -- the disc and its drive ------------------------------------------------------------------------------------------
DISC_MASS = 0.016
TAKE_AT = 0.070                         # the drive takes the disc once its leading edge is this far in ...
TAKE_TILT = math.radians(10)            # ... standing within this of upright (square to the slot) ...
TAKE_OFF = (0.0025, 0.012)              # ... and centred: across the slot, and up and down
SEATED = FACE_X + 0.072                 # where the drive draws the disc's centre to (its trailing edge 12 mm inside)
DRAW = 0.004                            # m per control step the drive draws it in (8 cm/s)
IN_SLOT = 0.020                         # leading edge this far into the slot: half credit
OFF_HUB = 0.010                         # disc centre this far from its place on the hub: off the hub
DROPPED = 0.003                         # disc's lowest point this close to the table, outside the console: dropped
PRESSED = -0.0015                       # the hub button pressed this far: the teeth draw in
UPRIGHT_Q = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))   # xyzw: disc axis along world -y, upright in the x-z plane


def disc_home() -> np.ndarray:
    """The disc's centre on the hub at the start."""
    return np.array([HUB_XY[0], HUB_XY[1], TOP + STAND_H + K.FLOOR + K.SEAT_H + K.DISC_T / 2])


def _stand() -> Fixed:
    x0, x1 = HUB_XY[0] - TRAY[0] / 2 + 0.0275, HUB_XY[0] + TRAY[0] / 2 + 0.010   # front edge behind the thumb cut-out
    y0, y1 = HUB_XY[1] - TRAY[1] / 2 - 0.010, HUB_XY[1] + TRAY[1] / 2 + 0.014 + TRAY[1] + 0.010
    return Fixed("storage_box", pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + STAND_H / 2),
                 size=(x1 - x0, y1 - y0, STAND_H), color=(0.62, 0.55, 0.46), roughness=0.8)


def _console() -> list:
    """The console's body as fixed blocks round the slot and the drive bay behind it."""
    w, d, h = CONSOLE["width"], CONSOLE["depth"], CONSOLE["height"]
    y0, y1 = SLOT_Y - w / 2, SLOT_Y + w / 2
    sy0, sy1 = SLOT_Y - SLOT_W / 2, SLOT_Y + SLOT_W / 2
    z0, z1 = SLOT_Z - SLOT_H / 2, SLOT_Z + SLOT_H / 2
    black, white = (0.06, 0.06, 0.07), (0.92, 0.92, 0.93)
    parts = []

    def block(name, xa, xb, ya, yb, za, zb, color=black, rough=0.5):
        parts.append(Fixed(name, pos=((xa + xb) / 2, (ya + yb) / 2, TOP + (za + zb) / 2),
                           size=(xb - xa, yb - ya, zb - za), color=color, roughness=rough,
                           static_friction=0.4, dynamic_friction=0.3))

    x0, x1 = FACE_X, FACE_X + d
    xm = x0 + MOUTH_D                                                   # behind the flared mouth
    m = MOUTH_W / 2 + 0.003
    block("console_left", xm, x1, sy1, y1, 0.0, h)                      # the slot's two sides, full height
    block("console_right", xm, x1, y0, sy0, 0.0, h)
    block("console_face_l", x0, xm, SLOT_Y + m, y1, 0.0, h)              # the face beside the mouth
    block("console_face_r", x0, xm, y0, SLOT_Y - m, 0.0, h)
    block("console_under_slot", x0, x1, SLOT_Y - m, SLOT_Y + m, 0.0, z0)
    block("console_over_slot", x0, x1, SLOT_Y - m, SLOT_Y + m, z1, h)
    run = math.hypot(MOUTH_D, (MOUTH_W - SLOT_W) / 2)
    ang = math.atan2((MOUTH_W - SLOT_W) / 2, MOUTH_D)
    for side in (1, -1):                                                # the mouth's two sloping lips
        mid = np.array([x0 + MOUTH_D / 2, SLOT_Y + side * (MOUTH_W + SLOT_W) / 4])
        normal = np.array([math.sin(ang), side * math.cos(ang)])
        c = mid + normal * 0.002
        q = (0.0, 0.0, math.sin(-side * ang / 2), math.cos(-side * ang / 2))
        parts.append(Fixed(f"console_lip_{'lr'[side < 0]}", pos=(float(c[0]), float(c[1]), TOP + SLOT_Z),
                           size=(run + 0.002, 0.004, SLOT_H), color=(0.03, 0.03, 0.03), roughness=0.4, quat=q,
                           static_friction=0.3, dynamic_friction=0.2))
    block("console_bay_back", x0 + BAY, x1, sy0, sy1, z0, z1)
    block("console_panel_l", x0 + 0.004, x1 - 0.010, y1, y1 + 0.006, 0.015, h - 0.010, white, 0.35)   # side panels
    block("console_panel_r", x0 + 0.004, x1 - 0.010, y0 - 0.006, y0, 0.015, h - 0.010, white, 0.35)
    block("console_feet", x0 + 0.02, x1 - 0.02, y0 - 0.006, y1 + 0.006, -0.0005, 0.0005, (0.2, 0.2, 0.2))
    block("console_light", x0 - 0.0005, x0, sy1 + 0.004, sy1 + 0.007, z1 + 0.02, h - 0.03, (0.35, 0.6, 1.0), 0.2)
    return parts


OBJECTS = [
    _stand(),
    *_console(),
    Case("game_case", pos=(float(HUB_XY[0]), float(HUB_XY[1]), TOP + STAND_H),
         size=(TRAY[0], 2 * TRAY[1] + 0.014, 0.012), mass=CASE_MASS, tray=TRAY),
    Disc("disc", pos=tuple(float(v) for v in disc_home()), size=(0.12, 0.12, K.DISC_T), mass=DISC_MASS),
]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(off_hub=False, in_slot=False, taking=False, taken=False, dropped=False, teeth_in=False,
                  draw_x=None, pressed_max=0.0, depth_max=-1.0)


_clear()


def disc_geometry(pos, quat):
    """From the disc's pose: its axis, its lowest point's height, and how far its leading edge is into the slot."""
    axis = K._rotation(quat)[:, 2]
    low = pos[2] - K.DISC_R * math.sqrt(max(0.0, 1 - axis[2] ** 2)) - K.DISC_T / 2 * abs(axis[2])
    front = pos[0] + K.DISC_R * math.sqrt(max(0.0, 1 - axis[0] ** 2))
    return axis, low, front - FACE_X


def centred(pos, axis) -> bool:
    return (abs(pos[1] - SLOT_Y) <= TAKE_OFF[0] and abs(pos[2] - (TOP + SLOT_Z)) <= TAKE_OFF[1]
            and abs(axis[1]) >= math.cos(TAKE_TILT))


@on_reset
def _reset(environment) -> None:
    _clear()
    for i in range(K.TEETH):
        write_joint(environment, "game_case", f"tooth_{i}", limits=(0.0, 0.0), position=0.0)
    write_joint(environment, "game_case", "button", position=0.0)


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    case = object_states(environment, ["game_case"])["game_case"]
    button = case.joints["button"]
    s["pressed_max"] = min(s["pressed_max"], button)
    teeth_in = button <= PRESSED
    if teeth_in != s["teeth_in"]:
        s["teeth_in"] = teeth_in
        lim = (-K.TOOTH_STROKE, -K.TOOTH_STROKE) if teeth_in else (0.0, 0.0)
        for i in range(K.TEETH):
            write_joint(environment, "game_case", f"tooth_{i}", limits=lim)

    pos, quat = body_pose(environment, "disc")
    axis, low, depth = disc_geometry(pos, quat)
    s["depth_max"] = max(s["depth_max"], depth)
    hub_home = np.asarray(case.pos) + np.array([0.0, 0.0, K.FLOOR + K.SEAT_H + K.DISC_T / 2])
    s["off_hub"] = s["off_hub"] or float(np.linalg.norm(pos - hub_home)) > OFF_HUB
    inside = depth > 0.0 and centred(pos, axis)
    if depth >= IN_SLOT and abs(pos[1] - SLOT_Y) <= 0.004:
        s["in_slot"] = True
    if not inside and not s["taking"] and low <= TOP + DROPPED:
        s["dropped"] = True
    if not s["taking"] and depth >= TAKE_AT and centred(pos, axis):
        s["taking"], s["draw_x"] = True, float(pos[0])
    if s["taking"]:                                                     # the drive draws the disc in and holds it
        s["draw_x"] = min(SEATED, s["draw_x"] + DRAW)
        write_pose(environment, "disc", (s["draw_x"], SLOT_Y, TOP + SLOT_Z), UPRIGHT_Q)
        s["taken"] = s["draw_x"] >= SEATED


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """0 if the disc ever touched the table outside the console (dropped). Otherwise 1.0 once the drive has taken
    it; 0.5 if it went at least 2 cm into the slot; 0.25 if it came off the hub; 0 otherwise."""
    s = _STATE
    if s["dropped"]:
        return 0.0
    if s["taken"]:
        return 1.0
    if s["in_slot"]:
        return 0.5
    return 0.25 if s["off_hub"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    s = _STATE
    return {"off_hub": s["off_hub"], "in_slot": s["in_slot"], "taking": s["taking"], "taken": s["taken"],
            "dropped": s["dropped"], "button_min_mm": round(1000 * s["pressed_max"], 2),
            "depth_max_mm": round(1000 * s["depth_max"], 1), "score": score(objects)}


def demo(objects, seed):
    return {}
