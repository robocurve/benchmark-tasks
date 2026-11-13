"""Cut in a wall above a skirting board with a paint edger, its guide wheels riding the board's top edge.

On the table: a section of drywall stands across the robot's path, its painted face toward the robot, with a white
skirting board nailed along its foot. A loaded paint edger (a flat paint pad on a head with two guide wheels along
one long edge, held by a palm grip on its back) stands on its wheels in a rest to the left of the wall, pad facing
the wall. Two green tape tabs on the board mark a 300 mm stretch.

Cutting in paints the strip of wall right above the board, where a roller cannot reach. The edger is set on the
wall with its pad flat on the wall and its wheels on the board's top edge, then run along the board: the wheels hold
the pad's lower edge just clear of the board, so the pad paints the wall right down to the board and never touches
the board. Let the wheels ride up and a strip of bare wall is left above the board; let the pad tip or drop and it
smears paint on the board.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl  # noqa: E402
from yam_joints import Hinge, Link, Mechanism  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"

# Release-and-stillness block, verbatim from the parts kit's docs (release_stillness_block.py).
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


# ---------------------------------------------------------------- layout (metres; x forward, y left, z up)
TOP = TABLE_TOP[SCENE]
DEMO_PACE = 1.0
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 2400

# Colours (sRGB)
WALL_PAINT = (0.76, 0.72, 0.64)    # the wall's existing paint, a pale warm grey
TRIM_WHITE = (0.96, 0.96, 0.94)    # white semi-gloss trim paint
PAINT_BLUE = (0.20, 0.40, 0.75)    # the new wall paint
TAPE_GREEN = (0.45, 0.78, 0.25)    # green painter's tape
PINE = (0.86, 0.70, 0.45)
REST_GREY = (0.55, 0.57, 0.60)
EDGER_RED = (0.78, 0.10, 0.12)
GRIP_BLACK = (0.10, 0.10, 0.11)

# Wall section: 1/2 in (12.7) gypsum drywall, its face the plane x = WALL_X toward the robot, 380 long, 200 tall,
# screwed to a pine 2x4 frame (38 x 89) behind it; fixed to the table.
WALL_X = 0.250
DRYWALL_T = 0.0127
WALL_Y0, WALL_Y1 = -0.060, 0.320
WALL_H = 0.200
STUD_T, STUD_D = 0.038, 0.089


def _b(x0, x1, y0, y1, z0, z1, color=None):
    """A box from its extents (part frame)."""
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color=color)


_FX0, _FX1 = DRYWALL_T, DRYWALL_T + STUD_D
WALL = Part("wall", (WALL_X, 0.0, TOP), [
    _b(0.0, DRYWALL_T, WALL_Y0, WALL_Y1, 0.0, WALL_H),                                   # 0 drywall, painted face
    _b(_FX0, _FX1, WALL_Y0, WALL_Y1, 0.0, STUD_T, color=PINE),                           # 1 bottom plate
    _b(_FX0, _FX1, WALL_Y0, WALL_Y1, WALL_H - STUD_T, WALL_H, color=PINE),               # 2 top plate
    _b(_FX0, _FX1, WALL_Y0, WALL_Y0 + STUD_T, STUD_T, WALL_H - STUD_T, color=PINE),      # 3 end stud
    _b(_FX0, _FX1, WALL_Y1 - STUD_T, WALL_Y1, STUD_T, WALL_H - STUD_T, color=PINE),      # 4 end stud
], color=WALL_PAINT, friction=(0.40, 0.35), contact_offset=0.001)

# Skirting board: primed MDF, square edge, 11/16 x 3-1/2 in (17.5 x 89), white semi-gloss, nailed along the wall's
# foot for the wall's full length. Two green tape tabs on its face mark the stretch to cut in (y = 0 to 300).
BOARD_T, BOARD_H = 0.0175, 0.089
BOARD_X = WALL_X - BOARD_T            # the board's face
STRETCH = (0.000, 0.300)
TAB_W, TAB_H, TAB_T = 0.015, 0.025, 0.0002
SKIRTING = Part("skirting", (WALL_X, 0.0, TOP), [
    _b(-BOARD_T, 0.0, WALL_Y0, WALL_Y1, 0.0, BOARD_H),                                    # 0 board
    _b(-BOARD_T - TAB_T, -BOARD_T, STRETCH[0] - TAB_W, STRETCH[0], BOARD_H - TAB_H, BOARD_H, color=TAPE_GREEN),
    _b(-BOARD_T - TAB_T, -BOARD_T, STRETCH[1], STRETCH[1] + TAB_W, BOARD_H - TAB_H, BOARD_H, color=TAPE_GREEN),
], color=TRIM_WHITE, friction=(0.30, 0.25), contact_offset=0.001)

# ---------------------------------------------------------------- the paint edger
# Shur-Line Edger Pro (4-3/4 x 3-3/4 in pad), in its own frame E: origin at the centre of the pad's face, x out of
# the face, y along the pad's long edge, z along the short edge toward the end away from the guide wheels.
# The red head carries the pad on its face and the two guide wheels at the corners of its lower long edge; the black
# palm grip on its back turns on a swivel about the axis normal to the pad (the head can turn in the pad's plane).
PAD_L, PAD_H, PAD_T = 0.121, 0.095, 0.004       # pad face 4-3/4 x 3-3/4 in; pile on its backing (estimate)
HEAD_L, HEAD_H, HEAD_T = 0.117, 0.093, 0.009
WHEEL_R, WHEEL_W = 0.007, 0.005                 # guide wheels, axles normal to the pad (estimate from photos)
WHEEL_X, WHEEL_Y = -0.009, 0.0515               # wheel mid-plane behind the pad face; wheels at y = +-WHEEL_Y
WHEEL_GAP = 0.0015                              # the pad's lower edge stands this far above the wheels' rolling line
WHEEL_LINE = -(PAD_H / 2 + WHEEL_GAP)           # the rolling line's z in E (-0.049)
HEAD_BACK = -(PAD_T + HEAD_T)                   # the head's back face, x = -0.013
SWIVEL = (HEAD_BACK, 0.0, 0.008)                # where the grip turns on the head
GRIP_A = (-0.051, HEAD_BACK, -0.029, 0.029, 0.008, 0.0475)   # the grip's upper part: flats 58 apart (x0 x1 y0 y1 z0 z1)
GRIP_B = (-0.041, HEAD_BACK, -0.022, 0.022, -0.030, 0.008)   # its lower, narrower part
TCP_IN_E = np.array([-0.037, 0.0, 0.030])       # where the demo's hand closes on the grip
GRIP_MASS, HEAD_MASS = 0.030, 0.070             # 100 g in all, with about 10 g of paint on the pad (estimate)

EDGER_LINK_SHAPES = (
    _b(-PAD_T, 0.0, -PAD_L / 2, PAD_L / 2, -PAD_H / 2, PAD_H / 2, color=PAINT_BLUE),    # 0 pad, loaded with paint
    _b(HEAD_BACK, -PAD_T, -HEAD_L / 2, HEAD_L / 2, -HEAD_H / 2, HEAD_H / 2),             # 1 red head
    cyl((WHEEL_X, WHEEL_Y, WHEEL_LINE + WHEEL_R), WHEEL_R, WHEEL_W, axis="x", color=GRIP_BLACK),   # 2 guide wheel
    cyl((WHEEL_X, -WHEEL_Y, WHEEL_LINE + WHEEL_R), WHEEL_R, WHEEL_W, axis="x", color=GRIP_BLACK),  # 3 guide wheel
    _b(HEAD_BACK - 0.003, HEAD_BACK, 0.0525, 0.0585, -0.015, 0.015, color=GRIP_BLACK),   # 4 pad-release tab
    _b(HEAD_BACK - 0.003, HEAD_BACK, -0.0585, -0.0525, -0.015, 0.015, color=GRIP_BLACK),  # 5 pad-release tab
    _b(HEAD_BACK - 0.004, HEAD_BACK, -0.020, 0.020, -0.046, -0.038, color=GRIP_BLACK),   # 6 guide-lift slide
)
WHEEL_SHAPES = (2, 3)                          # indices in the head link: they get the wheels' material
WHEEL_FRICTION = (0.05, 0.05)                  # free-turning wheels, modelled as low-friction rims

# The rest (made for the trial): grey-painted 12 mm plywood base with a pine ledge. The edger stands on its two
# wheels on the base, its grip's lower end on the ledge, pad facing the wall and touching nothing.
X_HOME, Y_HOME = 0.237, 0.415                  # the pad face centre's x, y when the edger stands in the rest
REST_T = 0.012
LEDGE_H = GRIP_B[4] - WHEEL_LINE               # 0.019: the grip's lower end above the wheels' rolling line
REST = Part("rest", (X_HOME, Y_HOME, TOP), [
    _b(-0.060, -0.005, -0.080, 0.080, 0.0, REST_T),                     # 0 base
    _b(-0.005, 0.025, -0.080, 0.080, 0.0, REST_T - 0.003),              # 1 base, 3 mm lower under the pad
    _b(-0.055, -0.023, -0.040, 0.040, REST_T, REST_T + LEDGE_H, color=PINE),  # 2 ledge under the grip
], color=REST_GREY, friction=(0.50, 0.40), contact_offset=0.001)

EDGER_HOME = np.array([X_HOME, Y_HOME, TOP + REST_T - WHEEL_LINE])   # E's origin standing in the rest
EDGER = Mechanism(
    "edger", EDGER_HOME + [0.0, 0.0, 0.0005],
    [_b(*GRIP_A),                                                                  # 0 grip, upper part (held)
     _b(*GRIP_B),                                                                  # 1 grip, lower part
     cyl((-0.032, 0.0, GRIP_A[5] + 0.0003), 0.0115, 0.0006, axis="z", color=(0.03, 0.03, 0.03))],  # 2 pole socket
    [Link("head", EDGER_LINK_SHAPES, Hinge("x", SWIVEL, -90.0, 90.0, damping=0.002), HEAD_MASS,
          color=EDGER_RED, friction=(0.40, 0.35))],
    mass=GRIP_MASS, color=GRIP_BLACK, friction=(0.60, 0.50), contact_offset=0.001, grip=0)

OBJECTS = [WALL, SKIRTING, REST, EDGER]

# ---------------------------------------------------------------- paint: cells on the wall face and the board
CELL = 0.005
COL_Y = np.arange(WALL_Y0 + CELL / 2, WALL_Y1 - 1e-9, CELL)                 # 76 columns along the wall
WALL_ROW_Z = np.arange(BOARD_H + CELL / 2, WALL_H - 1e-9, CELL)              # 22 rows, 89 to 199 above the table
TOP_ROW_X = BOARD_X + (np.arange(3) + 0.5) * BOARD_T / 3                     # board top: 3 rows across its depth
FACE_ROW_Z = (np.arange(18) + 0.5) * BOARD_H / 18                            # board face: 18 rows


def _grid(a, b):
    A, B = np.meshgrid(a, b, indexing="ij")
    return A.ravel(), B.ravel()


_wz, _wy = _grid(WALL_ROW_Z, COL_Y)
_tx, _ty = _grid(TOP_ROW_X, COL_Y)
_fz, _fy = _grid(FACE_ROW_Z, COL_Y)
WALL_CELLS = np.column_stack((np.full(_wz.size, WALL_X), _wy, TOP + _wz))
TOP_CELLS = np.column_stack((_tx, _ty, np.full(_tx.size, TOP + BOARD_H)))
FACE_CELLS = np.column_stack((np.full(_fz.size, BOARD_X), _fy, TOP + _fz))
CELLS = np.vstack((WALL_CELLS, TOP_CELLS, FACE_CELLS))
N_WALL = len(WALL_CELLS)
TRIM = slice(N_WALL, len(CELLS))                                              # every board cell
BAND_TOP = BOARD_H + 0.040                                                    # the scored band: 40 above the board
_in_stretch = (WALL_CELLS[:, 1] > STRETCH[0]) & (WALL_CELLS[:, 1] < STRETCH[1])
BAND = np.flatnonzero(_in_stretch & (WALL_CELLS[:, 2] < TOP + BAND_TOP))     # 8 rows x 60 = 480 cells
CUT = np.flatnonzero(_in_stretch & (WALL_CELLS[:, 2] < TOP + BOARD_H + CELL))  # the lowest row: 60 cells

# The pad paints a surface point that lies within its face's outline (plus D_EDGE) and no more than D_FACE in front
# of the rigid face (the pile reaches that far past the pressed face) or inside the pile's thickness.
D_FACE, D_EDGE = 0.0020, 0.0005


def pad_frame(edger):
    """(face centre, rotation) of the pad in the world, from the head link's reported pose."""
    return np.asarray(edger.link_to_world("head", (0.0, 0.0, 0.0)), float), np.asarray(edger.link_rotation("head"))


def pad_paints(p0, R, cells=CELLS):
    """Which cells the pad, its face centre at p0 and its axes R (columns: normal, long edge, short edge), paints."""
    q = (np.asarray(cells) - p0) @ R
    return ((q[:, 0] >= -PAD_T) & (q[:, 0] <= D_FACE)
            & (np.abs(q[:, 1]) <= PAD_L / 2 + D_EDGE) & (np.abs(q[:, 2]) <= PAD_H / 2 + D_EDGE))


def coverage(painted):
    """(band fraction, cut-line fraction, painted board cells)."""
    painted = np.asarray(painted, bool)
    return float(painted[BAND].mean()), float(painted[CUT].mean()), int(painted[TRIM].sum())


# ---------------------------------------------------------------- the paint record (every control step)
TEST_PAINT = None
_PAINT = {"episode": None, "token": None, "cells": np.zeros(len(CELLS), bool), "new": False}


def reset_paint():
    _PAINT.update(episode=_RUN["episode"], token=None, cells=np.zeros(len(CELLS), bool), new=True)


def paint_step(objects):
    """Add this control step's paint to the record (once per step; nothing offline)."""
    if _RUN["t"] is None:
        return
    if _PAINT["episode"] != _RUN["episode"]:
        reset_paint()
    token = (_RUN["episode"], _RUN["step"])
    if token == _PAINT["token"]:
        return
    _PAINT["token"] = token
    edger = objects["edger"]
    p0, R = pad_frame(edger)
    hit = pad_paints(p0, R)
    if np.any(hit & ~_PAINT["cells"]):
        _PAINT["cells"] = _PAINT["cells"] | hit
        _PAINT["new"] = True
    if token[1] % 20 == 0:
        _diagnostic(edger, p0, R)


def _diagnostic(edger, p0, R):
    corners = {k: p0 + R @ np.array([0.0, sy * PAD_L / 2, sz * PAD_H / 2])
               for k, (sy, sz) in {"tl": (1, 1), "tr": (-1, 1), "bl": (1, -1), "br": (-1, -1)}.items()}
    gaps = " ".join(f"{k}={(WALL_X - c[0]) * 1000:.2f}" for k, c in corners.items())
    wheels = [edger.link_to_world("head", (WHEEL_X, s * WHEEL_Y, WHEEL_LINE)) for s in (1, -1)]
    band, cut, trim = coverage(_PAINT["cells"])
    print(f"EDGER t={_RUN['t']:.2f} face_mm=({p0[0] * 1000:.1f},{p0[1] * 1000:.1f},{(p0[2] - TOP) * 1000:.1f}) "
          f"wall_gap_mm[{gaps}] pad_bottom_above_board_mm={(min(corners['bl'][2], corners['br'][2]) - TOP - BOARD_H) * 1000:.2f} "
          f"wheels_above_board_mm=({(wheels[0][2] - TOP - BOARD_H) * 1000:.2f},{(wheels[1][2] - TOP - BOARD_H) * 1000:.2f}) "
          f"wheel_x_mm=({wheels[0][0] * 1000:.1f},{wheels[1][0] * 1000:.1f}) swivel_deg={edger.angle_deg('head'):.2f} "
          f"grip_tilt_deg={edger.tilt_deg():.2f} band={band:.3f} cut={cut:.3f} trim_cells={trim}", flush=True)


# ---------------------------------------------------------------- end state
def wheel_points(edger):
    return [np.asarray(edger.link_to_world("head", (WHEEL_X, s * WHEEL_Y, WHEEL_LINE)), float) for s in (1, -1)]


def in_rest(edger):
    """Standing in the rest: both wheels on its base, the pad's face over the base's front and facing the wall."""
    p0, R = pad_frame(edger)
    wheels_down = all(-0.004 <= w[2] - (TOP + REST_T) <= 0.003 for w in wheel_points(edger))
    placed = -0.010 <= p0[0] - X_HOME <= 0.008 and abs(p0[1] - Y_HOME) <= 0.030
    facing = R[0, 0] >= math.cos(math.radians(15)) and edger.tilt_deg() <= 8.0
    return bool(wheels_down and placed and facing)


def dropped(edger):
    """The pad on the table, or the edger off the table."""
    p0, R = pad_frame(edger)
    pad_low = min((p0 + R @ np.array([x, sy * PAD_L / 2, sz * PAD_H / 2]))[2]
                  for x in (-PAD_T, 0.0) for sy in (-1, 1) for sz in (-1, 1))
    return bool(pad_low < TOP + 0.001 or edger.low[2] < TOP - 0.020)


def score(objects):
    paint_step(objects)
    edger = objects["edger"]
    painted = TEST_PAINT if TEST_PAINT is not None else _PAINT["cells"]
    band, cut, trim_cells = coverage(painted)
    lost = ever("dropped", dropped(edger))
    full = band >= 0.95 and cut >= 0.95 and trim_cells == 0
    finished = in_rest(edger) and released(edger) and edger.still(max_speed=0.01, max_joint_speed=0.05)
    still_s = held_for("finished", finished and full and not lost)
    if lost:
        return 0.0
    if trim_cells:
        return 0.25 if band >= 0.40 else 0.0
    if full:
        return 1.0 if still_s >= STILL_S else 0.75
    if band >= 0.80 and cut >= 0.80:
        return 0.5
    if band >= 0.40:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- offline fixtures (tests only; the demo never uses them)
def _rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def stroke_paint(y0=0.250, y1=0.050, gap=0.0, lift=0.0, pitch=0.0, roll=0.0, step=0.002, painted=None):
    """The paint a pad leaves swept from face-centre y0 to y1 along the wall: its face `gap` off the wall, its wheels
    `lift` above the board's top, tipped `pitch` degrees about a long edge (positive: the upper edge on the wall and
    the lower edge away from it; negative: the reverse) and `roll` degrees in the wall's plane."""
    out = np.zeros(len(CELLS), bool) if painted is None else np.array(painted, bool)
    R = _rot("y", pitch) @ _rot("x", roll)
    pivot = np.array([0.0, 0.0, PAD_H / 2 if pitch > 0 else -PAD_H / 2])   # the long edge that stays on the wall
    n = max(2, int(round(abs(y1 - y0) / step)) + 1)
    for y in np.linspace(y0, y1, n):
        edge = np.array([WALL_X - gap, y, TOP + BOARD_H - WHEEL_LINE + lift + pivot[2]])
        out |= pad_paints(edge - R @ pivot, R)
    return out


def no_paint():
    return np.zeros(len(CELLS), bool)


def dab_paint(points, gap=0.0, lift=0.0):
    """The pad set flat on the wall at each face-centre y in `points` (no stroke)."""
    out = np.zeros(len(CELLS), bool)
    for y in points:
        out |= stroke_paint(y, y, gap=gap, lift=lift)
    return out


def board_smear(y=0.150, drop=0.001):
    """The pad's lower edge dragged along the board's top `drop` below the wheels' proper line (pad fallen onto it)."""
    return stroke_paint(y + 0.020, y - 0.020, lift=-WHEEL_GAP - drop)


def edger_state(face_origin=None, yaw_deg=0.0, swivel_deg=0.0, vel=(0.0, 0.0, 0.0)):
    """The edger's offline state with its grip frame at `face_origin` (E's origin; default: standing in the rest)."""
    origin = EDGER_HOME if face_origin is None else np.asarray(face_origin, float)
    s = EDGER.offline_state(origin, math.radians(yaw_deg), {"head": math.radians(swivel_deg)})
    if np.any(np.asarray(vel)):
        from dataclasses import replace
        s = replace(s, lin_vel=np.asarray(vel, float))
    return s


# ---------------------------------------------------------------- scripted demo (left arm; the right arm waits)
PRESS_WALL = 0.0020      # pad commanded this far into the wall (about 0.8 N with the arm's give)
PRESS_DOWN = 0.0048      # wheels commanded this far into the board's top (about 2 to 3.5 N)
STROKE_Y = (0.250, 0.050)  # pad centre's path: the 121 mm pad covers y = -10.5 to 310.5
STROKE_STEPS, STROKE_STEP_S = 20, 0.60
Z_CARRY = BOARD_H - WHEEL_LINE + 0.014          # E's height (above the table) while carrying: wheels 14 above the board
Z_ON_BOARD = BOARD_H - WHEEL_LINE                # E's height with the wheels on the board's top
X_OFF_WALL = WALL_X - 0.014                      # pad face 14 in front of the wall while approaching
X_CLEAR = BOARD_X - 0.0065                       # pad face in front of the board's face while leaving
V0 = -TCP_IN_E                                   # E minus the TCP, nominal
_CLAMP = 0.020                                   # the grip slides about 17 mm across the fingers as it leaves the rest


def _edger_now():
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, ["edger"])["edger"]
    except Exception as e:  # noqa: BLE001 - an unreadable pose falls back to the open-loop plan
        print(f"EDGER_READ failed: {e!r}", flush=True)
        return None


class LazyWaypoint(Waypoint):
    """A Waypoint whose target and yaw are worked out when it is first played (they may use a pose read then)."""

    def __init__(self, duration, where, finger, label, yaw_fn=None, keep=False):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.where, self.finger, self.label, self._yaw_fn = where, finger, label, yaw_fn
        self._pos = None

    @property
    def pos(self):
        if self._pos is None:
            self._pos = tuple(float(v) for v in self.where())
        return self._pos

    @property
    def yaw(self):
        return 0.0 if self._yaw_fn is None else float(self._yaw_fn())


def demo():
    """Left arm: edger out of its rest, onto the wall above the board, one stroke along the board, back in its rest."""
    plan, clock = [], [0.0]
    aim = {"start": None, "v": None, "yaw": 0.0, "v_rest": None, "yaw_rest": 0.0}
    cmd = {"tcp": None, "yaw": 0.0}      # the hand's commanded target and yaw at the last waypoint evaluated

    def start_shift():
        """Where the edger actually stands, minus where the rest puts it (x, y; read once, before the pick)."""
        if aim["start"] is None:
            aim["start"] = np.zeros(3)
            e = _edger_now()
            if e is not None:
                d = np.asarray(e.origin, float) - EDGER_HOME
                d[2] = 0.0
                aim["start"] = np.clip(d, -0.015, 0.015)
                print(f"EDGER_START shift_mm=({d[0] * 1000:.2f},{d[1] * 1000:.2f})", flush=True)
        return aim["start"]

    def read_hand(key_v, key_yaw):
        """Read once, at the start of a move: the pad's face centre relative to the commanded hand, in the hand's
        unturned frame, and the hand yaw that turns the pad square to the wall."""
        if aim[key_v] is None:
            aim[key_v], aim[key_yaw] = V0.copy(), cmd["yaw"]
            e = _edger_now()
            if e is not None and cmd["tcp"] is not None:
                p0, R = pad_frame(e)
                c = cmd["yaw"]
                v = _rot("z", -math.degrees(c)) @ (p0 - np.asarray(cmd["tcp"], float))
                yaw = math.atan2(R[1, 0], R[0, 0])
                aim[key_v] = V0 + np.clip(v - V0, -_CLAMP, _CLAMP)
                aim[key_yaw] = float(np.clip(c - yaw, -math.radians(3), math.radians(3)))
                print(f"EDGER_IN_HAND {key_v} offset_mm=({(v - V0)[0] * 1000:.2f},{(v - V0)[1] * 1000:.2f},"
                      f"{(v - V0)[2] * 1000:.2f}) pad_yaw_deg={math.degrees(yaw):.2f} "
                      f"hand_yaw_deg={math.degrees(aim[key_yaw]):.2f}", flush=True)
        return aim[key_v], aim[key_yaw]

    def tcp_for(E, key_v=None, key_yaw=None):
        """The hand target that puts the pad's face centre at E (world)."""
        if key_v is None:
            return np.asarray(E, float) - V0
        v, yaw = read_hand(key_v, key_yaw)
        return np.asarray(E, float) - _rot("z", math.degrees(yaw)) @ v

    def add(dt, where, label, finger=FINGER_CLOSED, yaw_key=None, keep=False):
        def fn(w=where, k=yaw_key):
            p = np.asarray(w(), float)
            cmd["tcp"], cmd["yaw"] = p, (0.0 if k is None else aim[k])
            return p
        yaw_fn = None if yaw_key is None else (lambda k=yaw_key: aim[k])
        plan.append(LazyWaypoint(dt, fn, finger, label, yaw_fn, keep))
        clock[0] += plan[-1].duration

    def E(x, y, z):
        return np.array([x, y, TOP + z])

    home = lambda dz=0.0: EDGER_HOME + start_shift() + [0.0, 0.0, dz]  # noqa: E731  (where it stands now)
    rest = lambda dz=0.0: EDGER_HOME + [0.0, 0.0, dz]  # noqa: E731  (where the rest holds it)
    # The ready pose holds the left hand 51 mm above the wall's left end: rise and come back first.
    add(1.4, lambda: np.array([0.150, 0.420, 0.175]) + [0, 0, TOP], "back from the wall", FINGER_OPEN)
    add(1.4, lambda: home(0.040) - V0, "over the edger", FINGER_OPEN)
    add(1.2, lambda: home() - V0, "hand down beside the grip", FINGER_OPEN)
    add(0.8, lambda: home() - V0, "close on the grip", keep=True)
    add(1.8, lambda: home(Z_CARRY - (REST_T - WHEEL_LINE)) - V0, "lift out of the rest")
    add(2.2, lambda: tcp_for(E(X_OFF_WALL, STROKE_Y[0], Z_CARRY)), "carry to the wall")
    v, yw = "v", "yaw"
    add(1.2, lambda: tcp_for(E(WALL_X + PRESS_WALL, STROKE_Y[0], Z_CARRY), v, yw), "pad onto the wall",
        yaw_key=yw, keep=True)
    add(0.4, lambda: tcp_for(E(WALL_X + PRESS_WALL, STROKE_Y[0], Z_CARRY), v, yw), "settle on the wall",
        yaw_key=yw, keep=True)
    add(1.6, lambda: tcp_for(E(WALL_X + PRESS_WALL, STROKE_Y[0], Z_ON_BOARD - PRESS_DOWN), v, yw),
        "slide down onto the board", yaw_key=yw, keep=True)
    add(0.6, lambda: tcp_for(E(WALL_X + PRESS_WALL, STROKE_Y[0], Z_ON_BOARD - PRESS_DOWN), v, yw),
        "settle on the board", yaw_key=yw, keep=True)
    for i in range(1, STROKE_STEPS + 1):
        y = STROKE_Y[0] + (STROKE_Y[1] - STROKE_Y[0]) * i / STROKE_STEPS
        add(STROKE_STEP_S, lambda y=y: tcp_for(E(WALL_X + PRESS_WALL, y, Z_ON_BOARD - PRESS_DOWN), v, yw),
            "cut in", yaw_key=yw)
    add(0.4, lambda: tcp_for(E(WALL_X + PRESS_WALL, STROKE_Y[1], Z_ON_BOARD - PRESS_DOWN), v, yw), "dwell",
        yaw_key=yw)
    add(0.8, lambda: tcp_for(E(WALL_X + PRESS_WALL, STROKE_Y[1], Z_CARRY - 0.002), v, yw), "wheels up off the board",
        yaw_key=yw)
    add(0.8, lambda: tcp_for(E(X_CLEAR, STROKE_Y[1], Z_CARRY - 0.002), v, yw), "pad off the wall", yaw_key=yw)
    add(2.6, lambda: tcp_for(E(X_HOME, Y_HOME, Z_CARRY - 0.002), v, yw), "carry back", yaw_key=yw)
    vr, yr = "v_rest", "yaw_rest"
    add(1.0, lambda: tcp_for(rest(0.030), vr, yr), "over the rest", yaw_key=yr)
    add(1.4, lambda: tcp_for(rest(0.0015), vr, yr), "lower into the rest", yaw_key=yr, keep=True)
    add(0.8, lambda: tcp_for(rest(0.0015), vr, yr), "let go", FINGER_OPEN, yaw_key=yr, keep=True)
    add(1.0, lambda: tcp_for(rest(0.0015), vr, yr) + [0.0, 0.0, 0.060], "hand up", FINGER_OPEN, yaw_key=yr)
    add(1.2, lambda: tcp_for(rest(0.0015), vr, yr) + [-0.050, -0.040, 0.080], "park", FINGER_OPEN)
    final = plan[-1]
    plan.append(LazyWaypoint(3.5, final.where, FINGER_OPEN, "stillness tail", keep=True))
    return {"left": plan}


import paint_runtime  # noqa: E402

paint_runtime.install(sys.modules.get(__name__) or globals())
