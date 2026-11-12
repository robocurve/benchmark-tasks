"""Replace the CR2032 system (CMOS/RTC) battery on a server system board.

A section of server system board (FR-4, 1.6 mm) sits on four 6 mm standoffs in a steel chassis tray. On it is a
horizontal CR2032 socket: a fixed retaining hook on the side nearest the arms (-x) and a sprung spring-steel latch
on the far side (+x) whose lip overhangs the cell's top edge, with a sprung negative contact under the cell that
lifts the cell's latch side when the latch is pushed back. A spent CR2032 is latched in it, "+" face up. A new
CR2032 lies "+" face up on a staging boss on the left; an ESD disposal tray for the spent cell is on the right.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl
from yam_joints import Link, Slide
from yam_mesh import MeshMechanism, prism
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"

# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ---------------------------------------------------------------------------------------------------------------
# Layout (metres; world x forward, y left, z up; arm bases at (0, +-0.305))
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# Chassis tray, standoffs, board
TRAY_X, TRAY_Y = (0.060, 0.270), (-0.090, 0.090)
TRAY_T, TRAY_WALL_H = 1.0 * MM, 12.0 * MM
STANDOFF_H = 6.0 * MM
BOARD_T = 1.6 * MM
BOARD_X, BOARD_Y = (0.085, 0.245), (-0.075, 0.075)
BOARD_TOP = TOP + TRAY_T + STANDOFF_H + BOARD_T

# Socket (holder frame: origin at the socket centre on the board's top face)
HX, HY = 0.150, 0.0
SEAT_TOP = 1.0 * MM          # top of the moulded seat
CELL_D, CELL_H = 20.0 * MM, 3.2 * MM
CELL_R = CELL_D / 2
CAP_R, CAP_H = 8.0 * MM, 0.3 * MM   # negative cap (smaller raised disc on the "-" face)
CELL_MASS = 0.0030
SEATED_BOTTOM = 1.3 * MM     # cell bottom when latched (contact spring pushes it up against the lips)
LIP_UNDER = SEATED_BOTTOM + CELL_H   # 4.5 mm: underside of the hook lip and the latch lip
SEATED_Z = BOARD_TOP + SEATED_BOTTOM + CELL_H / 2
HOOK_TIP_X = -9.2 * MM       # fixed hook lip reaches 0.8 mm over the cell edge
LATCH_TIP_X = 9.2 * MM       # latch lip reaches 0.8 mm over the cell edge at rest
LATCH_TRAVEL = 1.5 * MM
LATCH_K, LATCH_REST = 400.0, -1.0 * MM        # 0.4 N preload closed, 1.0 N at full travel
CONTACT_TRAVEL = 3.0 * MM
CONTACT_K, CONTACT_REST = 200.0, 4.0 * MM     # 0.8 N on a latched cell, 0.2 N at full lift
CONTACT_X = 2.0 * MM

LATCH_OUTLINE_XZ = ((9.2, 4.5), (12.0, 4.5), (12.0, 6.5), (10.7, 6.5), (9.2, 5.0))  # mm, 45-degree lead-in


def _latch_chamfer():
    # prism(axis="y") maps outline (u, v) to (x=u, z=-v) and extrudes along y about y = 0.
    outline = [(x * MM, -z * MM) for x, z in LATCH_OUTLINE_XZ]
    return prism(outline, 7.0 * MM, axis="y", name="latch_lip", collision="convex")


SOCKET_BLACK, STEEL = (0.06, 0.06, 0.07), (0.78, 0.78, 0.80)
HOLDER = MeshMechanism("holder", (HX, HY, BOARD_TOP), [
    cyl((0, 0, 0.5 * MM), 9.0 * MM, 1.0 * MM),                                  # moulded seat
    box((-9.2 * MM, 0, 0.5 * MM), (2.4 * MM, 8.0 * MM, 1.0 * MM)),              # seat under the hook
    box((-11.05 * MM, 0, 2.6 * MM), (1.5 * MM, 6.0 * MM, 5.2 * MM)),            # hook post
    box((-9.75 * MM, 0, 4.85 * MM), (1.1 * MM, 6.0 * MM, 0.7 * MM)),            # hook lip, underside 4.5 mm
    box((8.2 * MM, 8.2 * MM, 1.5 * MM), (1.6 * MM, 1.6 * MM, 3.0 * MM)),        # four locating posts
    box((8.2 * MM, -8.2 * MM, 1.5 * MM), (1.6 * MM, 1.6 * MM, 3.0 * MM)),
    box((-8.2 * MM, 8.2 * MM, 1.5 * MM), (1.6 * MM, 1.6 * MM, 3.0 * MM)),
    box((-8.2 * MM, -8.2 * MM, 1.5 * MM), (1.6 * MM, 1.6 * MM, 3.0 * MM)),
    box((11.4 * MM, 0, 0.5 * MM), (4.8 * MM, 9.0 * MM, 1.0 * MM)),              # latch guide floor
    box((14.4 * MM, 0, 4.3 * MM), (1.2 * MM, 9.0 * MM, 8.6 * MM)),              # housing wall behind the latch
], [
    Link("latch", [
        box((11.1 * MM, 0, 2.85 * MM), (1.8 * MM, 7.0 * MM, 3.3 * MM)),         # latch leg
        box((11.5 * MM, 0, 7.55 * MM), (1.0 * MM, 7.0 * MM, 2.1 * MM)),         # upturned release tab
    ], Slide("x", (11.1 * MM, 0, 2.85 * MM), 0.0, LATCH_TRAVEL, damping=1.0, spring=LATCH_K,
             rest=LATCH_REST, armature=0.005), mass=0.0005, color=STEEL, friction=(0.2, 0.15)),
    Link("contact", [
        box((CONTACT_X, 0, 1.05 * MM), (4.0 * MM, 5.0 * MM, 0.5 * MM)),        # negative contact spring tip
    ], Slide("z", (CONTACT_X, 0, 1.05 * MM), 0.0, CONTACT_TRAVEL, damping=0.3, spring=CONTACT_K,
             rest=CONTACT_REST, armature=0.002), mass=0.0003, color=(0.85, 0.72, 0.35), friction=(0.2, 0.15)),
], color=SOCKET_BLACK, friction=(0.4, 0.35), contact_offset=0.0003, meshes={"latch": (_latch_chamfer(),)})

# Chassis tray with standoffs, and the board with a PCIe x16 slot body beside the socket
_SO = [(x, y) for x in (BOARD_X[0] + .006, BOARD_X[1] - .006) for y in (BOARD_Y[0] + .006, BOARD_Y[1] - .006)]
CHASSIS = Part("chassis", (0, 0, TOP), [
    box(((TRAY_X[0] + TRAY_X[1]) / 2, 0, TRAY_T / 2), (TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0], TRAY_T)),
    box((TRAY_X[0] + .0005, 0, TRAY_WALL_H / 2), (.001, TRAY_Y[1] - TRAY_Y[0], TRAY_WALL_H)),
    box((TRAY_X[1] - .0005, 0, TRAY_WALL_H / 2), (.001, TRAY_Y[1] - TRAY_Y[0], TRAY_WALL_H)),
    box(((TRAY_X[0] + TRAY_X[1]) / 2, TRAY_Y[0] + .0005, TRAY_WALL_H / 2), (TRAY_X[1] - TRAY_X[0], .001, TRAY_WALL_H)),
    box(((TRAY_X[0] + TRAY_X[1]) / 2, TRAY_Y[1] - .0005, TRAY_WALL_H / 2), (TRAY_X[1] - TRAY_X[0], .001, TRAY_WALL_H)),
    *[cyl((x, y, TRAY_T + STANDOFF_H / 2), .00275, STANDOFF_H, color=(.80, .66, .30)) for x, y in _SO],
], color=(.55, .57, .60), contact_offset=0.0005)
PCIE_X = HX - 0.030
BOARD = Part("board", (0, 0, BOARD_TOP), [
    box(((BOARD_X[0] + BOARD_X[1]) / 2, 0, -BOARD_T / 2), (BOARD_X[1] - BOARD_X[0], BOARD_Y[1] - BOARD_Y[0], BOARD_T)),
    box((PCIE_X, 0, .005625), (.0075, .089, .01125), color=(.05, .05, .05)),
], color=(.06, .32, .16), contact_offset=0.0005)

# Staging boss for the new cell (left) and the disposal tray for the spent cell (right)
STAND_XY = (0.150, 0.125)
STAND_PLATE_H, BOSS_R, BOSS_H = 3.0 * MM, 6.0 * MM, 8.0 * MM
STAND_TOP = TOP + STAND_PLATE_H + BOSS_H
STAND = Part("stand", (STAND_XY[0], STAND_XY[1], TOP), [
    box((0, 0, STAND_PLATE_H / 2), (.040, .040, STAND_PLATE_H)),
    cyl((0, 0, STAND_PLATE_H + BOSS_H / 2), BOSS_R, BOSS_H),
], color=(.10, .12, .16), contact_offset=0.0005)
BIN_XY = (0.150, -0.130)
BIN_L, BIN_W, BIN_H, BIN_T = .060, .050, .020, .002
BIN_IN_X = (BIN_XY[0] - BIN_L / 2 + BIN_T, BIN_XY[0] + BIN_L / 2 - BIN_T)
BIN_IN_Y = (BIN_XY[1] - BIN_W / 2 + BIN_T, BIN_XY[1] + BIN_W / 2 - BIN_T)
BIN_FLOOR = TOP + BIN_T
BIN = Part("bin", (BIN_XY[0], BIN_XY[1], TOP), [
    box((0, 0, BIN_T / 2), (BIN_L, BIN_W, BIN_T)),
    box((-(BIN_L - BIN_T) / 2, 0, BIN_H / 2), (BIN_T, BIN_W, BIN_H)),
    box(((BIN_L - BIN_T) / 2, 0, BIN_H / 2), (BIN_T, BIN_W, BIN_H)),
    box((0, -(BIN_W - BIN_T) / 2, BIN_H / 2), (BIN_L - 2 * BIN_T, BIN_T, BIN_H)),
    box((0, (BIN_W - BIN_T) / 2, BIN_H / 2), (BIN_L - 2 * BIN_T, BIN_T, BIN_H)),
], color=(.75, .15, .10), contact_offset=0.0005)


def _cell(name, pos, color):
    # Origin at mid-height; "+" can face up (local +z), "-" cap is the smaller disc underneath.
    return Part(name, pos, [
        cyl((0, 0, (CAP_H) / 2), CELL_R, CELL_H - CAP_H),
        cyl((0, 0, -CELL_H / 2 + CAP_H / 2), CAP_R, CAP_H, color=(.62, .62, .64)),
    ], mass=CELL_MASS, color=color, friction=(0.5, 0.4), contact_offset=0.0003)


OLD_CELL = _cell("old_cell", (HX, HY, SEATED_Z), (.42, .40, .36))
NEW_CELL = _cell("new_cell", (STAND_XY[0], STAND_XY[1], STAND_TOP + CELL_H / 2), (.55, .60, .70))
OBJECTS = [CHASSIS, BOARD, HOLDER, STAND, BIN, OLD_CELL, NEW_CELL]
PRESSED_PARTS = ("holder", "new_cell")

# ---------------------------------------------------------------------------------------------------------------
# Scoring
SEAT_XY_TOL = 1.5 * MM     # cell centre from the socket centre
SEAT_Z_TOL = 0.8 * MM      # cell centre height from the latched height
SEAT_TILT = 5.0            # degrees; also means "+" face up
LATCH_ENGAGED = 0.5 * MM   # latch lip still over the cell edge (0.8 mm overlap at rest)
SOCKET_XY = 5.0 * MM       # partial: new cell in the socket, "+" up, some part below the lips
SOCKET_LOW = 2.5 * MM
SOCKET_TILT = 45.0
HOLDER_ZONE = 0.013        # a cell centre this close to the socket centre is in the socket, not loose
LOOSE_LOW = 0.012          # loose cell resting this low over the board or tray floor
POSE_TOL = 0.002


def _xy_dist(c):
    return math.hypot(float(c.origin[0]) - HX, float(c.origin[1]) - HY)


def seated(c, holder):
    return (_xy_dist(c) <= SEAT_XY_TOL and abs(float(c.origin[2]) - SEATED_Z) <= SEAT_Z_TOL
            and c.tilt_deg() <= SEAT_TILT and float(holder.joints["latch"]) <= LATCH_ENGAGED)


def in_socket(c):
    return (_xy_dist(c) <= SOCKET_XY and float(c.low[2]) <= BOARD_TOP + SOCKET_LOW
            and c.tilt_deg() <= SOCKET_TILT)


def in_bin(c):
    lo, hi = np.asarray(c.low), np.asarray(c.high)
    return bool(lo[0] >= BIN_IN_X[0] - POSE_TOL and hi[0] <= BIN_IN_X[1] + POSE_TOL
                and lo[1] >= BIN_IN_Y[0] - POSE_TOL and hi[1] <= BIN_IN_Y[1] + POSE_TOL
                and lo[2] >= BIN_FLOOR - POSE_TOL and hi[2] <= TOP + BIN_H)


def loose_on_board(c):
    """A released cell lying on the board or tray floor outside the socket (a short-circuit hazard)."""
    x, y = float(c.origin[0]), float(c.origin[1])
    return bool(TRAY_X[0] <= x <= TRAY_X[1] and TRAY_Y[0] <= y <= TRAY_Y[1] and _xy_dist(c) > HOLDER_ZONE
                and float(c.low[2]) <= BOARD_TOP + LOOSE_LOW and released(c))


def off_table(c):
    return float(c.origin[2]) < TOP - 0.030


TEST_PAST_HAZARD = False  # offline tests only: a hazard recorded earlier in the episode


STILL_MOVE = 0.5 * MM      # stillness: the cell moves less than this ...
STILL_TURN = 1.0           # ... and turns less than this (degrees) over the last STILL_S seconds
_HIST = {"episode": None, "step": None, "cells": {}}


def cell_still(c, key):
    """Stillness from the cell's pose over the last STILL_S seconds (contact-chatter-proof); velocity offline."""
    if _RUN["t"] is None:
        return c.resting()
    if _HIST["episode"] != _RUN["episode"]:
        _HIST.update(episode=_RUN["episode"], step=None, cells={})
    hist = _HIST["cells"].setdefault(key, [])
    t = float(_RUN["t"])
    if not hist or hist[-1][0] != t:
        hist.append((t, np.asarray(c.origin, float).copy(), c.rotation()[:, 2].copy()))
    while hist and hist[0][0] < t - STILL_S - 1e-6:
        hist.pop(0)
    if hist[0][0] > t - STILL_S + 0.06:
        return False   # less than a full window observed
    p, z = hist[-1][1], hist[-1][2]
    return all(np.linalg.norm(q - p) <= STILL_MOVE and
               math.degrees(math.acos(float(np.clip(np.dot(zq, z), -1, 1)))) <= STILL_TURN for _, q, zq in hist)


def score(objects):
    old, new, holder = objects["old_cell"], objects["new_cell"], objects["holder"]
    hazard = ever("cell loose on the board", loose_on_board(old) or loose_on_board(new)) or bool(TEST_PAST_HAZARD)
    dropped = ever("cell off the table", off_table(old) or off_table(new))
    old_done = in_bin(old) and released(old) and cell_still(old, "old")
    new_rel = released(new) and cell_still(new, "new")
    new_seated = seated(new, holder) and new_rel
    seated_s = held_for("new cell seated", new_seated)
    new_in = in_socket(new) and new_rel
    if hazard or dropped:
        return 0.0
    s = 0.25 if old_done else 0.0
    if new_seated and seated_s + 1e-9 >= STILL_S:
        s += 0.75
    elif new_seated or new_in:
        s += 0.25
    return s


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------------------------------------------------------
# Demo: authored layout and elapsed time only
PAD_TIP = 0.013               # pad tips below the TCP
LIGHT_GRIP = -0.0090          # pads aimed 18 mm apart on a 20 mm cell: a light pinch the cell can pivot in
OPEN_34 = -0.017              # 34 mm between the pads
SQUEEZE_OPEN = -0.0040        # 8 mm between the pads before squeezing the latch tab
SQUEEZE = -0.0005
YAW_X = math.pi / 2           # fingers close along world x


def _w(dt, x, y, z, f, label, yaw=0.0):
    return Waypoint(dt, (float(x), float(y), float(z)), float(f), label, float(yaw))


def demo():
    B = BOARD_TOP
    sq_x = HX + 12.8 * MM
    right = [
        _w(1.5, sq_x, HY, B + .040, SQUEEZE_OPEN, "over the latch", YAW_X),
        _w(1.5, sq_x, HY, B + 6.8 * MM + PAD_TIP, SQUEEZE_OPEN, "pads either side of latch tab and housing", YAW_X),
        _w(1.0, sq_x, HY, B + 6.8 * MM + PAD_TIP, SQUEEZE, "squeeze latch back", YAW_X),
        _w(0.5, sq_x, HY, B + 6.8 * MM + PAD_TIP, SQUEEZE, "hold latch back", YAW_X),
        _w(1.5, sq_x, HY, B + 10.8 * MM + PAD_TIP, SQUEEZE, "slide up off the tab, cell pops", YAW_X),
        _w(1.0, sq_x, HY, B + .045, SQUEEZE, "rise", YAW_X),
        _w(0.6, sq_x, HY, B + .045, OPEN_34, "open", YAW_X),
        _w(1.5, HX, HY, B + .045, OPEN_34, "over the cell"),
        _w(1.5, HX, HY, B + 3.2 * MM + PAD_TIP, OPEN_34, "pads beside the raised cell"),
        _w(1.0, HX, HY, B + 3.2 * MM + PAD_TIP, LIGHT_GRIP, "pinch cell rim"),
        _w(2.0, HX, HY, B + .050, LIGHT_GRIP, "lift out from under the hook"),
        _w(2.0, BIN_XY[0], BIN_XY[1], TOP + .060, LIGHT_GRIP, "carry to the disposal tray"),
        _w(1.0, BIN_XY[0], BIN_XY[1], TOP + .040, LIGHT_GRIP, "lower into the tray"),
        _w(1.0, BIN_XY[0], BIN_XY[1], TOP + .040, -0.020, "let go of the spent cell"),
        _w(1.0, BIN_XY[0], BIN_XY[1], TOP + .100, -0.020, "rise"),
        _w(1.5, .110, -.220, TOP + .120, -0.020, "park"),
    ]
    t_right = sum(w.duration for w in right)
    sx, sy = STAND_XY
    grip_tip = STAND_TOP + 0.3 * MM
    left_pick = [
        _w(1.5, sx, sy, TOP + .060, OPEN_34, "over the new cell"),
        _w(1.5, sx, sy, grip_tip + PAD_TIP, OPEN_34, "pads beside the new cell"),
        _w(1.0, sx, sy, grip_tip + PAD_TIP, LIGHT_GRIP, "pinch new cell rim"),
        _w(1.5, sx, sy, TOP + .060, LIGHT_GRIP, "lift"),
    ]
    park_left = [_w(1.5, .110, .220, TOP + .120, OPEN_34, "park clear of the right arm")]
    wait = t_right - 1.5
    # The grip holds the cell 0.3 mm above its bottom face: pad tips = cell bottom + 0.3 mm.
    tip = lambda bottom: B + bottom + 0.3 * MM + PAD_TIP  # noqa: E731
    left = park_left + [_w(wait, .110, .220, TOP + .120, OPEN_34, "wait for the socket to clear")] + left_pick + [
        _w(2.0, HX + 2.0 * MM, HY, B + .030, LIGHT_GRIP, "over the socket, offset toward the latch"),
        _w(1.5, HX + 2.0 * MM, HY, tip(5.0 * MM), LIGHT_GRIP, "lower onto the latch tab, cell tilts"),
        _w(1.5, HX - 2.5 * MM, HY, tip(2.8 * MM), LIGHT_GRIP, "slide the low edge under the hook"),
        _w(0.8, HX - 2.5 * MM, HY, tip(2.8 * MM), OPEN_34, "let go"),
        _w(1.0, HX - 2.5 * MM, HY, B + .030, OPEN_34, "rise"),
        _w(1.0, HX + 5.5 * MM, HY, B + .030, FINGER_CLOSED, "close over the latch side", -YAW_X),
        _w(1.5, HX + 5.5 * MM, HY, B + LIP_UNDER - 8.0 * MM + PAD_TIP, FINGER_CLOSED, "press until the latch snaps", -YAW_X),
        _w(1.0, HX + 5.5 * MM, HY, B + LIP_UNDER - 8.0 * MM + PAD_TIP, FINGER_CLOSED, "hold press", -YAW_X),
        _w(1.0, HX + 5.5 * MM, HY, B + .040, FINGER_CLOSED, "rise clear", -YAW_X),
        _w(1.5, .110, .220, TOP + .120, FINGER_CLOSED, "park"),
        _w(3.5, .110, .220, TOP + .120, FINGER_CLOSED, "hold clear"),
    ]
    return {"left": left, "right": right}
