"""On the station table: a small sand mould in front of the arms. A brown drag (the lower half) has two long walls with a
4 cm cavity between them, and a ledge at each end of the cavity (the prints). Four black guide posts stand around it. A
long grey core lies on two rests beyond the drag, and a long blue cope (the upper half) lies on two rests in front of
it, nearer the arms. The core goes along the cavity with each end on its ledge; then the
cope goes down between the four posts and sits flat on the drag. The core and cope are loose; the drag, posts and
rests are fixed.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=set_the_core_and_close_the_mould.py -E object_jitter=0 \\
           --instruction "Set the grey core in the mould with each end on its ledge, then lower the blue cope
           flat between the posts." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from belimits import Rect, _rot, bottom, corners, inside, is_airborne, tilt_deg
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Success must not fire until the hands have left the cope (about 25 s), then it holds 5 s: it fires near 30 s.
SUCCESS_HOLD_S = 5.0

BROWN, BLACK = (0.33, 0.19, 0.09), (0.05, 0.05, 0.05)
GREY, BLUE, REST = (0.62, 0.63, 0.66), (0.12, 0.30, 0.88), (0.25, 0.62, 0.30)

# The drag: two walls 1.5 cm thick (x 0.185..0.20 and 0.24..0.255), 5 cm tall, y +-0.155, end blocks closing the cavity
# at y +-(0.14..0.155), and a 1 cm print ledge inside each end block (y +-(0.115..0.14)).
DRAG_TOP = 0.05
LEDGE_TOP = 0.01
CAVITY_X = (0.20, 0.24)
MID_X = 0.22
WALL_NEAR = Box("wall_near", pos=(0.1925, 0.0, TOP + 0.025), size=(0.015, 0.31, 0.05), color=BROWN)
WALL_FAR = Box("wall_far", pos=(0.2475, 0.0, TOP + 0.025), size=(0.015, 0.31, 0.05), color=BROWN)
END_L = Box("end_block_left", pos=(MID_X, 0.1475, TOP + 0.025), size=(0.04, 0.015, 0.05), color=BROWN)
END_R = Box("end_block_right", pos=(MID_X, -0.1475, TOP + 0.025), size=(0.04, 0.015, 0.05), color=BROWN)
LEDGE_L = Box("print_left", pos=(MID_X, 0.1275, TOP + 0.005), size=(0.04, 0.025, 0.01), color=BROWN)
LEDGE_R = Box("print_right", pos=(MID_X, -0.1275, TOP + 0.005), size=(0.04, 0.025, 0.01), color=BROWN)

# Four guide posts, 1.5 x 1.5 x 9 cm. Their inner faces are 1 cm clear of the cope on every side.
POST_H = 0.09
POSTS = [
    Box(f"post_{n}", pos=(px, py, TOP + POST_H / 2), size=(0.015, 0.015, POST_H), color=BLACK)
    for n, (px, py) in zip(("near_left", "near_right", "far_left", "far_right"),
                           ((0.1675, 0.1725), (0.1675, -0.1725), (0.2725, 0.1725), (0.2725, -0.1725)))
]
POST_AREA = Rect((0.175, 0.265), (-0.165, 0.165))  # inside the four posts' inner faces

# The core: 2.5 x 26 x 4 cm, on two rests 1 cm tall. Seated on the ledges its underside is 1 cm up and its top is level
# with the drag top; each end has 1 cm of play to the end block and 0.75 cm of play to each wall.
CORE_SIZE = (0.025, 0.26, 0.04)
CORE_REST_X = 0.32
CORE_RESTS = [Box(f"core_rest_{n}", pos=(CORE_REST_X, y, TOP + 0.005), size=(0.04, 0.04, 0.01), color=REST)
              for n, y in (("left", 0.08), ("right", -0.08))]
CORE = Box("core", pos=(CORE_REST_X, 0.0, TOP + 0.01 + CORE_SIZE[2] / 2), size=CORE_SIZE, mass=0.08, color=GREY)

# The cope: 7 x 31 x 3 cm, on two rests 2 cm tall in front of the drag. Closed, its underside rests on the drag.
COPE_SIZE = (0.07, 0.31, 0.03)
COPE_REST_X = 0.125
COPE_RESTS = [Box(f"cope_rest_{n}", pos=(COPE_REST_X, y, TOP + 0.01), size=(0.05, 0.05, 0.02), color=REST)
              for n, y in (("left", 0.09), ("right", -0.09))]
COPE = Box("cope", pos=(COPE_REST_X, 0.0, TOP + 0.02 + COPE_SIZE[2] / 2), size=COPE_SIZE, mass=0.4, color=BLUE)

OBJECTS = [WALL_NEAR, WALL_FAR, END_L, END_R, LEDGE_L, LEDGE_R, *POSTS, *CORE_RESTS, CORE, *COPE_RESTS, COPE]

# Scoring limits (m, degrees).
CORE_X_BAND = (0.205, 0.235)  # core centre
CORE_Y_MAX = 0.012
CORE_Z_TOL = 0.004  # underside within this of the ledges' top
CORE_TILT = 3.0
CORE_NEAR_Y = 0.02  # "inside the cavity footprint": centre this close to the mould's centre line
COPE_Z_TOL = 0.0025  # each end's underside within this of the drag top
COPE_TILT = 2.0
RELEASE_DIST = 0.02  # both gripper TCPs at least this far outside the cope's box
HELD_CLEARANCE = 0.01  # a part this far above every surface it could rest on counts as not placed
OFF_TABLE = 0.02


def _end_bottoms(s, spec):
    """The lowest corner of each end (y side) of the cope: tilt about either axis shows up here."""
    pts = corners(s, spec)
    hi = pts[:, 1] > s.pos[1]
    return float(pts[hi, 2].min()), float(pts[~hi, 2].min())


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go of the cope."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_mould_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._mould_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown.
    The approach axis is taken from the gripper link to the midpoint of the two finger links."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(cope) -> bool:
    """Both gripper TCPs at least RELEASE_DIST outside the cope's (oriented) box. True when no env is attached."""
    half = np.array(COPE.size) / 2
    r = _rot(cope.quat_xyzw)
    for g in grippers():
        local = r.T @ (np.asarray(g, float) - cope.pos)
        out = np.maximum(np.abs(local) - half, 0.0)
        if float(np.linalg.norm(out)) < RELEASE_DIST:
            return False
    return True


def _tilt(s) -> float:
    """Tilt from level, a part turned upside down counts as level (both parts are plain bars)."""
    t = tilt_deg(s)
    return min(t, 180.0 - t)


def _off_table(s, spec) -> bool:
    return bottom(s, spec) < TOP - OFF_TABLE


def _core_seated(core) -> bool:
    return bool(
        CORE_X_BAND[0] <= core.pos[0] <= CORE_X_BAND[1]
        and abs(core.pos[1]) <= CORE_Y_MAX
        and abs(bottom(core, CORE) - (TOP + LEDGE_TOP)) <= CORE_Z_TOL
        and _tilt(core) <= CORE_TILT
        and core.resting()
    )


def _core_in_cavity(core) -> bool:
    """Over the cavity and resting low, but not seated: its underside at most 2 cm above the table (1 cm over the
    ledges). A core lying across the walls, or held higher, is not placed."""
    b = bottom(core, CORE)
    low = b - (TOP + LEDGE_TOP) <= HELD_CLEARANCE
    return bool(
        CORE_X_BAND[0] <= core.pos[0] <= CORE_X_BAND[1] and abs(core.pos[1]) <= CORE_NEAR_Y and low and core.resting()
    )


def _cope_supports(core):
    """Heights the cope can rest at: the table, its rests, the drag top, and the core's top when the core is under it."""
    return [TOP, TOP + 0.02, TOP + DRAG_TOP, float(core.high[2])]


def _cope_in_posts(cope, core) -> bool:
    """Whole outline between the four posts, resting (not held up, not still moving), hands clear of the cope."""
    return bool(
        released(cope)
        and inside(cope, COPE, POST_AREA)
        and not is_airborne(cope, COPE, _cope_supports(core), HELD_CLEARANCE)
        and cope.resting()
    )


def _cope_closed(cope, core) -> bool:
    lo_a, lo_b = _end_bottoms(cope, COPE)
    return bool(
        _cope_in_posts(cope, core)
        and abs(lo_a - (TOP + DRAG_TOP)) <= COPE_Z_TOL
        and abs(lo_b - (TOP + DRAG_TOP)) <= COPE_Z_TOL
        and _tilt(cope) <= COPE_TILT
    )


def score(objects):
    """1.0 core seated and cope closed; 0.7 core seated, cope between the posts but proud or tilted; 0.5 core seated,
    cope anywhere else; 0.2 core over the cavity but not seated (underside at most 2 cm above the table), cope not closed; 0 otherwise
    (a cope closed over an unseated core, either part off the table, a core lying across the walls). A moving or held
    cope (a gripper TCP within 2 cm of its box) counts as cope anywhere else; a moving or held core is not placed."""
    core, cope = objects["core"], objects["cope"]
    if _off_table(core, CORE) or _off_table(cope, COPE):
        return 0.0
    if _core_seated(core):
        if _cope_closed(cope, core):
            return 1.0
        if _cope_in_posts(cope, core):
            return 0.7
        return 0.5
    if _cope_closed(cope, core):
        return 0.0
    return 0.2 if _core_in_cavity(core) else 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """Core centre to its seat plus the cope's underside to the drag top, in plan view and height (m)."""
    core, cope = objects["core"], objects["cope"]
    d_core = float(np.hypot(core.pos[0] - MID_X, core.pos[1]))
    d_cope = float(np.hypot(cope.pos[0] - MID_X, cope.pos[1])) + abs(bottom(cope, COPE) - (TOP + DRAG_TOP))
    return d_core + d_cope


# The robot keeps out too: over the drag, no part of either arm goes below the drag's rim (nothing reaches into the
# cavity or the closing gap), checked offline with the URDF. The scorer itself only sees the objects.
ARM_KEEP_OUT = [
    {"name": "drag below its rim", "x": (0.185, 0.255), "y": (-0.155, 0.155), "z_top": TOP + DRAG_TOP},
]

# --- demo -------------------------------------------------------------------------------------------------------
YAW_LEFT, YAW_RIGHT = -np.pi / 2, np.pi / 2  # fingers close along x, across the narrow core and cope (either sign grips the same; each arm holds its own)
TRAVEL_Z = TOP + 0.14
CORE_GRIP_Z = TOP + 0.053  # pad tips 1 cm below the core's top
CORE_SET_Z = TOP + 0.067  # pad tips 4 mm over the drag top; the core's underside is 14 mm over the ledges
COPE_GRIP_Z = TOP + 0.048  # pad tips 1.5 cm below the cope's top
COPE_CARRY_Z = TOP + 0.135  # cope underside 1.7 cm over the posts' tops
COPE_SET_Z = TOP + 0.08  # cope underside 2 mm over the drag
COPE_GRIP = -0.031  # 8 mm narrower than the 70 mm cope: firm but short of the 50 N cap, so the part cannot slide onto one finger
CORE_GRIP = -0.0085  # 8 mm narrower than the 25 mm core
GRIP_Y = 0.10
PARK_Y = 0.30  # final hand position, outside the posts (y 0.1675) and clear of the top view


def _core_phase():
    """Left arm: take the core at its middle, carry it over the cavity and let it drop onto the ledges."""
    (cx, cy), place = (CORE_REST_X, 0.0), (MID_X, 0.0)
    return [
        Waypoint(2.5, (cx, cy, TRAVEL_Z), FINGER_OPEN, "above the core", YAW_LEFT),
        Waypoint(1.8, (cx, cy, CORE_GRIP_Z), FINGER_OPEN, "down to the core", YAW_LEFT),
        Waypoint(0.8, (cx, cy, CORE_GRIP_Z), CORE_GRIP, "grip", YAW_LEFT),
        Waypoint(1.4, (cx, cy, TRAVEL_Z), CORE_GRIP, "lift", YAW_LEFT),
        Waypoint(1.8, (*place, TRAVEL_Z), CORE_GRIP, "over the cavity", YAW_LEFT),
        Waypoint(2.0, (*place, CORE_SET_Z), CORE_GRIP, "lower", YAW_LEFT),
        Waypoint(0.8, (*place, CORE_SET_Z), FINGER_OPEN, "let the core drop", YAW_LEFT),
        Waypoint(1.2, (*place, TRAVEL_Z), FINGER_OPEN, "clear", YAW_LEFT),
    ]


def _cope_phase(y_sign):
    """One arm of the pair: the same timing on both sides, so the cope stays level."""
    y = y_sign * GRIP_Y
    YAW = YAW_LEFT if y_sign > 0 else YAW_RIGHT
    return [
        Waypoint(2.0, (COPE_REST_X, y, COPE_CARRY_Z), FINGER_OPEN, "above the cope", YAW),
        Waypoint(1.6, (COPE_REST_X, y, COPE_GRIP_Z), FINGER_OPEN, "down to the cope", YAW),
        Waypoint(0.8, (COPE_REST_X, y, COPE_GRIP_Z), COPE_GRIP, "grip", YAW),
        Waypoint(1.4, (COPE_REST_X, y, COPE_CARRY_Z), COPE_GRIP, "lift", YAW),
        Waypoint(2.2, (MID_X, y, COPE_CARRY_Z), COPE_GRIP, "over the mould", YAW),
        Waypoint(2.8, (MID_X, y, COPE_SET_Z), COPE_GRIP, "lower slowly", YAW),
        Waypoint(0.8, (MID_X, y, COPE_SET_Z), FINGER_OPEN, "let go", YAW),
        Waypoint(1.6, (MID_X, y, COPE_CARRY_Z), FINGER_OPEN, "clear", YAW),
        Waypoint(1.5, (MID_X, y_sign * PARK_Y, COPE_CARRY_Z), FINGER_OPEN, "park to the side", YAW),
    ]


def demo():
    """The left arm sets the core on the ledges (the right arm waits above the cope's right end). Then both arms take
    the cope by its two ends, carry it together over the posts, lower it onto the drag and let go."""
    core_phase = _core_phase()
    core_s = sum(w.duration for w in core_phase)
    wait = Waypoint(core_s, (COPE_REST_X, -GRIP_Y, COPE_CARRY_Z), FINGER_OPEN, "wait over the cope", YAW_RIGHT)
    hold = Waypoint(2.0, (COPE_REST_X, -GRIP_Y, COPE_CARRY_Z), FINGER_OPEN, "above the cope", YAW_RIGHT)
    left = core_phase + _cope_phase(+1)
    right = [wait, hold] + _cope_phase(-1)[1:]
    return {"left": left, "right": right}
