"""On the station table, a dental X-ray sensor holder (an XCP-style paralleling instrument) is half assembled:

- The bite block (blue plastic, 12 g) sits paddle-down in a grey tray slot: a 35 x 45 x 4 mm backing paddle with
  the digital X-ray sensor (black, 30 x 40 x 6 mm, 20 g) already in its pocket, and a 45 mm bite plate standing up
  from the paddle's edge on the robot's right, topped by a 12 x 20 x 20 mm connector block with two 3.4 mm holes
  (a moulded lead-in at the top) for the indicator arm's prongs.
- The indicator arm (stainless steel flat bar, 5 x 2.5 mm, 20 g) stands in a grey storage stand on the robot's
  right, its two 3 mm x 18 mm prongs in the stand's holes: from the prongs a crossbar runs to an 8.5 cm upright
  with a grey plastic grip hub (10 x 14 x 20 mm) 5 cm up.
- The aiming ring (yellow plastic, 60 mm across, 5 mm thick, 8 g) lies flat on the table; a lug on its rim has a
  22 mm sleeve that fits the arm's upright, and a 12 mm-tall finger tab beyond it.

Finishing the assembly means: push the arm's prongs fully into the connector's holes, then slide the aiming ring
down the arm's upright until it rests on the grip hub, level, with its centre over the sensor. The bite block is
light; it must end upright where it started, with the sensor still in its pocket.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts  # noqa: E402,F401  (hooks yam_arena so the multi-shape parts are simulated)
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_parts import Part, box, cyl  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402

SCENE = "station"

# Release-and-stillness block, as in the accepted clip-pulse-oximeter-on-patient-finger task.
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


_POSE_PREV = {}


def pose_quiet(key, points, tol=0.0005):
    """Still: every given point moved under 0.5 mm since the previous control step (pose differences, because
    contact chatter makes the simulator's reported velocities unreliable). Offline it is always true."""
    if _RUN["t"] is None:
        return True
    _episode_memory()
    step = _RUN["step"]
    pts = np.asarray(points, float)
    prev = _POSE_PREV.get(key)
    if prev is not None and prev[0] == step:
        return prev[2]
    quiet = bool(prev is not None and prev[0] == step - 1 and np.all(np.abs(pts - prev[1]) < tol))
    _POSE_PREV[key] = (step, pts, quiet)
    return quiet


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]



TOP = TABLE_TOP[SCENE]
BLUE, STEEL, BLACK, YELLOW, GREY = (0.25, 0.45, 0.85), (0.75, 0.76, 0.78), (0.1, 0.1, 0.1), (0.95, 0.82, 0.15), \
    (0.4, 0.4, 0.42)
CO = 0.0005             # contact offset: close-fitting parts (m)


def rect_minus(x0, x1, y0, y1, holes):
    """Rectangles covering [x0,x1]x[y0,y1] minus the given rectangular holes (grid decomposition)."""
    xs = sorted({x0, x1, *[h[0] for h in holes], *[h[1] for h in holes]})
    ys = sorted({y0, y1, *[h[2] for h in holes], *[h[3] for h in holes]})
    out = []
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            cx, cy = (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2
            if not any(h[0] < cx < h[1] and h[2] < cy < h[3] for h in holes):
                out.append((xs[i], xs[i + 1], ys[j], ys[j + 1]))
    return out


# --- Layout: the kit frame. +x runs from the paddle to the bite plate; the kit is turned so that +x points to the
# robot's right (-y), and its parts' frames share that turn.
KIT_YAW = -math.pi / 2
BB_XY = (0.28, 0.02)            # bite block (paddle centre) on the table


def to_world(p, origin=BB_XY, yaw=KIT_YAW):
    c, s = math.cos(yaw), math.sin(yaw)
    return (origin[0] + c * p[0] - s * p[1], origin[1] + s * p[0] + c * p[1])


# --- Bite block (frame: paddle centre on the table) --------------------------------------------------------------
PADDLE = (0.035, 0.045, 0.004)
SENSOR = (0.030, 0.040, 0.006)          # along the pocket, across it, thick
RAIL_Y, RAIL = 0.022, (0.035, 0.003, 0.009)
LIP_Y, LIP = 0.0195, (0.035, 0.004, 0.002)
BP = (0.004, 0.020, 0.045)              # bite plate
BP_X = PADDLE[0] / 2 + BP[0] / 2
CONN = (0.012, 0.020, 0.020)            # connector block on top of the bite plate
CONN_Z0 = PADDLE[2] + BP[2]
PRONG_R, PRONG_DY, PRONG_L = 0.0015, 0.004, 0.018
HOLE = 0.0032                           # prong holes: 3.2 mm for 3.0 mm prongs (0.1 mm a side: rigid bodies cannot interfere,
                                        # so this stands in for the real 2.95 mm press-fit hole)
LEAD = (0.0080, 0.003)                  # each hole opens to 8 mm at the top over 3 mm: a moulded lead-in


def _bite_block():
    s = [box((0, 0, PADDLE[2] / 2), PADDLE, BLUE)]
    for sy in (-1, 1):
        s.append(box((0, sy * RAIL_Y, PADDLE[2] + RAIL[2] / 2), RAIL, BLUE))
        s.append(box((0, sy * LIP_Y, PADDLE[2] + 0.0065 + LIP[2] / 2), LIP, BLUE))
    s.append(box((BP_X, 0, PADDLE[2] + BP[2] / 2), BP, BLUE))
    holes = [(BP_X - HOLE / 2, BP_X + HOLE / 2, sy * PRONG_DY - HOLE / 2, sy * PRONG_DY + HOLE / 2) for sy in (-1, 1)]
    deep = CONN[2] - LEAD[1]
    for a, b, c, d in rect_minus(BP_X - CONN[0] / 2, BP_X + CONN[0] / 2, -CONN[1] / 2, CONN[1] / 2, holes):
        s.append(box(((a + b) / 2, (c + d) / 2, CONN_Z0 + deep / 2), (b - a, d - c, deep), BLUE))
    # lead-in: the top 3 mm of the connector has one wide opening over both holes (8 mm across them, 16 mm along)
    lead = [(BP_X - LEAD[0] / 2, BP_X + LEAD[0] / 2, -PRONG_DY - LEAD[0] / 2, PRONG_DY + LEAD[0] / 2)]
    for a, b, c, d in rect_minus(BP_X - CONN[0] / 2, BP_X + CONN[0] / 2, -CONN[1] / 2, CONN[1] / 2, lead):
        s.append(box(((a + b) / 2, (c + d) / 2, CONN_Z0 + deep + LEAD[1] / 2), (b - a, d - c, LEAD[1]), BLUE))
    return s


BITE_BLOCK = Part("bite_block", (*BB_XY, TOP + 0.0005), _bite_block(), mass=0.012, yaw=KIT_YAW, contact_offset=CO,
                  friction=(0.4, 0.35))

# --- Sensor ------------------------------------------------------------------------------------------------------
SENSOR_SEATED = (PADDLE[0] / 2 - SENSOR[0] / 2, 0.0)          # kit frame: against the bite plate, centred
SENSOR_PART = Part("sensor", (*to_world(SENSOR_SEATED), TOP + 0.0005 + PADDLE[2] + SENSOR[2] / 2 + 0.0003),
                   [box((0, 0, 0), SENSOR, BLACK)], mass=0.020, yaw=KIT_YAW, contact_offset=CO, friction=(0.5, 0.45))

# --- Indicator arm (frame: midpoint between the prong tips) ------------------------------------------------------
RING_LUG = 0.048                         # the ring's slot centre from the ring's centre
ROD_X = SENSOR_SEATED[0] + RING_LUG - BP_X     # upright position along the crossbar (arm frame)
BAR_W, BAR_T = 0.005, 0.0025             # flat bar
BAR_Z = PRONG_L + BAR_T / 2
ROD_L = 0.085
HUB, HUB_Z = (0.010, 0.014, 0.020), 0.050   # plastic grip hub on the upright (10 thick along the crossbar, 14 wide,
                                             # 20 tall), its centre 5 cm above the crossbar; the ring rests on it
GREY_HUB = (0.55, 0.55, 0.6)


def _arm_shapes():
    s = [cyl((0, sy * PRONG_DY, PRONG_L / 2), PRONG_R, PRONG_L, "z", STEEL) for sy in (-1, 1)]
    s.append(box((0, 0, BAR_Z), (BAR_W, 2 * PRONG_DY + 2 * PRONG_R, BAR_T), STEEL))       # bridge over the prongs
    s.append(box((ROD_X / 2, 0, BAR_Z), (ROD_X + BAR_W, BAR_W, BAR_T), STEEL))             # crossbar
    s.append(box((ROD_X, 0, BAR_Z + ROD_L / 2), (BAR_T, BAR_W, ROD_L), STEEL))             # upright
    s.append(box((ROD_X, 0, BAR_Z + HUB_Z), HUB, GREY_HUB))                                 # plastic grip hub
    return s


STAND_XY = (0.20, -0.16)
STAND_H, STAND_HOLE_D = 0.030, 0.018


def _stand():
    holes = [(-HOLE / 2, HOLE / 2, sy * PRONG_DY - HOLE / 2, sy * PRONG_DY + HOLE / 2) for sy in (-1, 1)]
    s = [box(((a + b) / 2, (c + d) / 2, STAND_H - STAND_HOLE_D / 2), (b - a, d - c, STAND_HOLE_D), GREY)
         for a, b, c, d in rect_minus(-0.015, 0.015, -0.012, 0.012, holes)]
    s.append(box((0, 0, (STAND_H - STAND_HOLE_D) / 2), (0.03, 0.024, STAND_H - STAND_HOLE_D), GREY))
    return s


STAND = Part("arm_stand", (*STAND_XY, TOP), _stand(), mass=None, yaw=KIT_YAW, contact_offset=CO)
ARM_START = (STAND_XY[0], STAND_XY[1], TOP + STAND_H - STAND_HOLE_D + 0.0005)
ARM = Part("indicator_arm", ARM_START, _arm_shapes(), mass=0.020, yaw=KIT_YAW, contact_offset=CO, friction=(0.3, 0.25))

# --- Aiming ring (frame: ring centre; +x toward the slot) ----------------------------------------------------------
RING_R0, RING_R1, RING_T = 0.023, 0.030, 0.005
SLOT = (0.0035, 0.0060)                  # slot for the 2.5 x 5 mm upright: 0.5 mm play each way
RING_START = (0.37, -0.10)
RING_START_YAW = math.radians(-145)      # its tab points back toward the robot, 55 degrees off the kit's direction
RING_SEGMENTS = 24
TAB_H = 0.012                            # the tab stands up as a 6 mm-thick, 12 mm-tall finger grip
LUG_TAB = 0.042                          # the lug runs on 4.2 cm past the slot's centre: a finger tab
SLEEVE = 0.022                           # the slot is a 22 mm tall sleeve, which keeps the ring square to the arm
LUG_GRIP = RING_LUG + 0.026              # where the demo pinches the tab (pads clear of the sleeve)


def _ring_shapes():
    """The ring as 24 straight segments (each an axis-aligned box in the part frame, so the ring is a 24-gon whose
    corner segments are slightly squared off), plus the lug with its slot."""
    s = []
    w, r = RING_R1 - RING_R0, (RING_R0 + RING_R1) / 2
    seg = 2 * RING_R1 * math.tan(math.pi / RING_SEGMENTS) * 1.05
    for k in range(RING_SEGMENTS):
        a = 2 * math.pi * (k + 0.5) / RING_SEGMENTS
        size = (w, seg, RING_T) if abs(math.cos(a)) > abs(math.sin(a)) else (seg, w, RING_T)
        s.append(box((r * math.cos(a), r * math.sin(a), 0), size, YELLOW))
    slot = (RING_LUG - SLOT[0] / 2, RING_LUG + SLOT[0] / 2, -SLOT[1] / 2, SLOT[1] / 2)
    for a, b, c, d in rect_minus(RING_R1 - 0.001, RING_LUG + 0.007, -0.007, 0.007, [slot]):       # the sleeve
        s.append(box(((a + b) / 2, (c + d) / 2, (SLEEVE - RING_T) / 2), (b - a, d - c, SLEEVE), YELLOW))
    s.append(box(((RING_LUG + 0.007 + RING_LUG + LUG_TAB) / 2, 0, (TAB_H - RING_T) / 2), (LUG_TAB - 0.007, 0.006, TAB_H), YELLOW))  # tab
    return s


RING = Part("aiming_ring", (*RING_START, TOP + RING_T / 2 + 0.0005), _ring_shapes(), mass=0.008,
            yaw=RING_START_YAW, contact_offset=CO, friction=(0.4, 0.35))

# --- Tray slot: a recess the paddle sits in, with a retaining lip over its far edge (kit frame) ------------------
SLOT_GAP, SLOT_WALL, SLOT_H = 0.0003, 0.006, 0.0075
LIP_IN, LIP_Z0 = 0.003, PADDLE[2] + 0.0006


def _tray_slot():
    x0, x1 = -PADDLE[0] / 2 - SLOT_GAP, PADDLE[0] / 2 + SLOT_GAP + 0.003
    y0, y1 = -PADDLE[1] / 2 - SLOT_GAP, PADDLE[1] / 2 + SLOT_GAP
    w = SLOT_WALL
    s = [box((x0 - w / 2, 0, SLOT_H / 2), (w, y1 - y0 + 2 * w, SLOT_H), GREY),                    # far wall
         box(((x0 + x1) / 2, y1 + w / 2, SLOT_H / 2), (x1 - x0, w, SLOT_H), GREY),                # side walls
         box(((x0 + x1) / 2, y0 - w / 2, SLOT_H / 2), (x1 - x0, w, SLOT_H), GREY),
         box((x0 + LIP_IN / 2, 0, LIP_Z0 + (SLOT_H - LIP_Z0) / 2), (LIP_IN, 2 * (RAIL_Y - 0.004), SLOT_H - LIP_Z0),
             GREY)]                                                                                # retaining lip
    return s


TRAY_SLOT = Part("tray_slot", (*BB_XY, TOP), _tray_slot(), mass=None, yaw=KIT_YAW, contact_offset=CO)

OBJECTS = [TRAY_SLOT, BITE_BLOCK, SENSOR_PART, STAND, ARM, RING]


# --- Scoring -----------------------------------------------------------------------------------------------------
BLOCK_UPRIGHT_DEG, BLOCK_MOVED = 5.0, 0.02
SENSOR_TOL, SENSOR_TILT = 0.002, 5.0
ARM_TOL_XY, ARM_TOL_Z, ARM_TILT = 0.0015, 0.0015, 8.0
RING_ON_TOL, RING_CENTRE_TOL, RING_LEVEL, RING_REST = 0.003, 0.010, 8.0, 0.004
# 10 mm: a standard 7 cm round cone (35 mm radius) still covers the whole 30 x 40 mm sensor (25 mm half-diagonal)
# when its centre line is within 35 - 25 = 10 mm of the sensor's centre.


def _o(st):
    return np.asarray(getattr(st, "origin", st.pos), float)


def _mat(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _tilt(q):
    return math.degrees(math.acos(float(np.clip(_mat(q)[2, 2], -1.0, 1.0))))


def kit_point(objects, p, z=0.0):
    """A point given in the bite block's frame, in world coordinates (follows the block wherever it is)."""
    b = objects[BITE_BLOCK.name]
    return _o(b) + _mat(b.quat_xyzw) @ np.array([p[0], p[1], z])


def block_ok(objects):
    o = _o(objects[BITE_BLOCK.name])
    return (_tilt(objects[BITE_BLOCK.name].quat_xyzw) <= BLOCK_UPRIGHT_DEG
            and math.hypot(o[0] - BB_XY[0], o[1] - BB_XY[1]) <= BLOCK_MOVED)


def sensor_seated(objects):
    s = objects[SENSOR_PART.name]
    tgt = kit_point(objects, SENSOR_SEATED, PADDLE[2] + SENSOR[2] / 2)
    return bool(np.linalg.norm(_o(s) - tgt) <= SENSOR_TOL and _tilt(s.quat_xyzw) <= SENSOR_TILT)


def arm_target(objects):
    """Where the prong tips' midpoint sits with the arm's bridge resting on the connector."""
    return kit_point(objects, (BP_X, 0.0), CONN_Z0 + CONN[2] - PRONG_L)


def arm_seated(objects):
    a = objects[ARM.name]
    d = _o(a) - arm_target(objects)
    return bool(math.hypot(d[0], d[1]) <= ARM_TOL_XY and abs(d[2]) <= ARM_TOL_Z and _tilt(a.quat_xyzw) <= ARM_TILT)


def upright_foot(objects):
    a = objects[ARM.name]
    return _o(a) + _mat(a.quat_xyzw) @ np.array([ROD_X, 0.0, BAR_Z])


def ring_slot(objects):
    r = objects[RING.name]
    return _o(r) + _mat(r.quat_xyzw) @ np.array([RING_LUG, 0.0, 0.0])


def beam_miss(objects):
    """How far from the sensor's centre the aiming line lands: the line through the ring's centre, perpendicular to
    the ring (the X-ray beam's axis once the cone is lined up with the ring), followed down to the sensor's face."""
    r, sen = objects[RING.name], objects[SENSOR_PART.name]
    c, n = _o(r), _mat(r.quat_xyzw)[:, 2]
    s_c, s_n = _o(sen), _mat(sen.quat_xyzw)[:, 2]
    denom = float(np.dot(n, s_n))
    if abs(denom) < 1e-6:
        return float("inf")
    hit = c + n * float(np.dot(s_c - c, s_n)) / denom
    return float(np.linalg.norm(hit - s_c))


def ring_fitted(objects):
    """The upright passes through the ring's sleeve, the ring is level within 8 degrees and slid down the upright
    until it rests on the grip hub (sleeve within 4 mm of the hub's top), and the aiming line through its centre meets the sensor within 10 mm
    of the sensor's centre (otherwise the X-ray cone would cut off part of the image)."""
    r = objects[RING.name]
    foot, slot = upright_foot(objects), ring_slot(objects)
    axis = _mat(objects[ARM.name].quat_xyzw)[:, 2]
    on_axis = foot + axis * (slot[2] - foot[2]) / max(axis[2], 0.5)   # the upright at the slot's height
    on = math.hypot(*(slot - on_axis)[:2]) <= RING_ON_TOL and slot[2] > foot[2]
    rest = foot[2] + HUB_Z + HUB[2] / 2 + RING_T / 2          # slot centre when the ring sits on the hub
    down = rest - 0.002 <= slot[2] <= rest + RING_REST
    centred = beam_miss(objects) <= RING_CENTRE_TOL
    return bool(on and down and centred and _tilt(r.quat_xyzw) <= RING_LEVEL)




def score(objects):
    """0.5 for the indicator arm seated; 1.0 with the aiming ring also fitted; either only once both grippers are at
    least 3 cm clear of the arm and the ring and both have been still for 1 s. 0 if the bite block has tipped or
    moved more than 2 cm, or the sensor has been knocked out of its pocket."""
    if not block_ok(objects) or not sensor_seated(objects):
        return 0.0
    if not arm_seated(objects):
        return 0.0
    arm, ring = objects[ARM.name], objects[RING.name]
    clear = released(arm) and released(ring)
    still = pose_quiet("arm and ring", [_o(arm), upright_foot(objects), _o(ring), ring_slot(objects)])
    if held_for("released and still", clear and still) + 1e-9 < STILL_S:
        return 0.0
    return 1.0 if ring_fitted(objects) else 0.5


def success(objects):
    return score(objects) == 1.0


# --- Closed-loop scripted demonstration (run with cabledemo.py) -------------------------------------------------
STEADY_FAR = (-0.016, RAIL_Y + 0.004)     # press here (rail's outer edge, far end) while the arm goes in
STEADY_NEAR = (0.012, -(RAIL_Y + 0.004))  # press here (other rail, bite-plate end) while the sensor goes in


def _near_yaw(a):
    """The equivalent grip yaw (a parallel gripper is symmetric under a half turn) within +-90 degrees."""
    while a > math.pi / 2:
        a -= math.pi
    while a < -math.pi / 2:
        a += math.pi
    return a


def demo_script(ctx):
    """Steadying and working hands swap with the side each part comes from. Sensor (on the left): the right hand
    presses the paddle's bite-plate end down while the left pinches the sensor by its ends, starts it into the
    pocket, lets go and pushes it home with closed fingertips. Indicator arm (on the right): the left hand presses
    the paddle's far end while the right lifts the arm out of its stand by its grip hub and lowers the prongs into
    the connector's holes, servoing on the prongs' measured position. Aiming ring: the right hand picks it up by its
    rim, turns it so its slot lines up with the upright, threads it on and lowers it onto the grip hub."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, R = ctx.left, ctx.right
    press_z = TOP + PADDLE[2] + RAIL[2] + 0.013 - 0.001        # closed fingertips 1 mm into a rail: a light press

    def press(arm, kit_xy, t_hold):
        p = to_world(kit_xy)
        yield from arm.move((*p, press_z + 0.04), yaw=math.pi / 2, finger=FINGER_CLOSED, duration=1.8)
        yield from arm.move((*p, press_z), yaw=math.pi / 2, duration=1.0)
        yield from arm.wait(t_hold)
        yield from arm.move((*p, press_z + 0.05), yaw=math.pi / 2, duration=0.8)

    def hold_until(arm, kit_xy, flag):
        p = to_world(kit_xy)
        yield from arm.move((*p, press_z + 0.04), yaw=math.pi / 2, finger=FINGER_CLOSED, duration=1.8)
        yield from arm.move((*p, press_z), yaw=math.pi / 2, duration=1.0)
        while not flag[0]:
            yield

    def servo(arm, get_err, z, yaw, steps=60, gain=0.6, tol=0.0004):
        cmd = np.array(arm.tcp(), float)
        for _ in range(steps):
            e = get_err()
            if math.hypot(e[0], e[1]) <= tol:
                return
            cmd[:2] += gain * np.asarray(e[:2])
            yield from arm.move((cmd[0], cmd[1], z), yaw=yaw, duration=0.1)

    arm_done = [False]

    def steady_for_arm():          # the tray slot holds the bite block; the left arm stays clear of the small part
        while True:
            yield

    def indicator_arm_and_ring():
        st = ctx.states()
        a0 = _o(st[ARM.name])
        hub = a0 + _mat(st[ARM.name].quat_xyzw) @ np.array([ROD_X, 0.0, BAR_Z + HUB_Z])
        gz = hub[2] + HUB[2] / 2 - 0.005 + 0.013 - 0.006       # pads cover the hub's upper part
        yield from R.wait(1.0)
        yield from R.move((hub[0], hub[1], gz + 0.04), yaw=0.0, finger=FINGER_OPEN, duration=2.0)
        yield from R.move((hub[0], hub[1], gz), yaw=0.0, duration=1.0)
        yield from R.grip(FINGER_CLOSED)
        yield from R.move((hub[0], hub[1], gz + 0.035), yaw=0.0, duration=1.2)
        tgt = arm_target(ctx.states())
        dz_grip = gz - a0[2]
        hover = tgt[2] + PRONG_L + 0.012 + dz_grip

        def arm_err():
            st_ = ctx.states()
            return arm_target(st_) - _o(st_[ARM.name])

        off = hub[:2] - a0[:2]
        yield from R.move((tgt[0] + off[0], tgt[1] + off[1], hover), yaw=0.0, duration=2.5)
        yield from servo(R, arm_err, hover, 0.0, steps=80, tol=0.0003)
        for k in range(1, 13):
            z = hover - (PRONG_L + 0.014) * k / 12
            yield from servo(R, arm_err, z, 0.0, steps=15, tol=0.0003)
            yield from R.move((*R.tcp()[:2], z), yaw=0.0, duration=0.3)
        yield from R.grip(FINGER_OPEN, duration=0.8)
        yield from R.wait(0.5)
        yield from R.move((*R.tcp()[:2], R.tcp()[2] + 0.05), yaw=0.0, duration=2.0)
        clear_z = TOP + CONN_Z0 + BAR_Z + ROD_L + 0.025                # above the upright's top before moving off
        yield from R.move((*R.tcp()[:2], clear_z), yaw=0.0, duration=1.2)
        arm_done[0] = True
        # aiming ring: pinch the tab beyond its slot, fingers closing across the tab
        st = ctx.states()
        r0, rm = _o(st[RING.name]), _mat(st[RING.name].quat_xyzw)
        ring_yaw0 = math.atan2(rm[1, 0], rm[0, 0])
        tab = r0 + rm @ np.array([LUG_GRIP, 0.0, 0.0])
        yaw_r = _near_yaw(ring_yaw0)                   # yaw 0 closes along world y: here across the tab
        rz = TOP + TAB_H + 0.004
        yield from R.move((tab[0], tab[1], clear_z), yaw=yaw_r, finger=FINGER_OPEN, duration=2.0)
        yield from R.move((tab[0], tab[1], rz + 0.04), yaw=yaw_r, duration=1.2)
        yield from R.move((tab[0], tab[1], rz), yaw=yaw_r, duration=1.0)
        yield from R.grip(FINGER_CLOSED)
        st = ctx.states()
        up_top = upright_foot(st) + _mat(st[ARM.name].quat_xyzw) @ np.array([0.0, 0.0, ROD_L])
        turn = KIT_YAW - ring_yaw0
        while turn > math.pi:
            turn -= 2 * math.pi
        while turn < -math.pi:
            turn += 2 * math.pi
        carry_z = up_top[2] + 0.012 + (rz - (TOP + RING_T / 2))
        yield from R.move((tab[0], tab[1], carry_z), yaw=yaw_r, duration=1.5)
        v = ring_slot(ctx.states())[:2] - np.asarray(R.tcp()[:2])
        c, s_ = math.cos(turn), math.sin(turn)
        v = np.array([c * v[0] - s_ * v[1], s_ * v[0] + c * v[1]])
        goal = up_top[:2] - v
        yaw_f = yaw_r + turn
        yield from R.move((goal[0], goal[1], carry_z), yaw=yaw_f, duration=3.0)

        def ring_err():
            """Slot centre to the upright's axis at the slot's height (the arm leans a few degrees in its holes)."""
            st_ = ctx.states()
            slot_, foot_ = ring_slot(st_), upright_foot(st_)
            axis = _mat(st_[ARM.name].quat_xyzw)[:, 2]
            on_axis = foot_ + axis * (slot_[2] - foot_[2]) / max(axis[2], 0.5)
            return on_axis[:2] - slot_[:2]

        yield from servo(R, ring_err, carry_z, yaw_f, steps=80, gain=0.4, tol=0.0003)
        rest = carry_z - 0.012 - (ROD_L - (HUB_Z + HUB[2] / 2 + RING_T / 2)) - 0.002   # down onto the grip hub
        n = 12
        for k in range(1, n + 1):                     # thread it on: centre, then step down, centre, step down...
            z = carry_z - (carry_z - rest) * k / n
            yield from servo(R, ring_err, R.tcp()[2], yaw_f, steps=40, gain=0.4, tol=0.0004)
            yield from R.move((*R.tcp()[:2], z), yaw=yaw_f, duration=0.35)
        yield from R.grip(FINGER_OPEN)
        yield from R.move((*R.tcp()[:2], R.tcp()[2] + 0.04), yaw=yaw_f, duration=0.8)
        yield from R.wait(3.0)

    yield from ctx.both(indicator_arm_and_ring(), steady_for_arm())
