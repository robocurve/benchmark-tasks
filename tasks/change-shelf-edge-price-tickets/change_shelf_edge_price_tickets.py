"""The base shelf of a store gondola faces the robot: a steel shelf deck 120 mm above the table with three food
cans standing on it, one per facing. A clear PVC shelf-edge ticket strip (39 mm ticket height) is taped along the
shelf's front edge. Its clear front is joined to the strip's back along the bottom and springs shut against the
tickets; a narrow grip edge runs along its top. Three white price tickets (60 x 39 mm card, 0.3 mm) stand in the
strip, one under each can, each with a coloured band matching its can. The middle and right cans have new prices:
their two new tickets stand in a small ticket stand on the table at the right, and a waste tray for the old ones is
on the table in front of the shelf. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl
from yam_joints import Hinge, Link, Mechanism
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena import demo as _motion
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



# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

SHELF_H = 0.120                  # shelf deck top above the table
Z_DECK = TOP + SHELF_H
XF = 0.150                       # front face of the shelf's front flange (the strip is taped to it)
SECTION = 0.400                  # length of the shelf section and of the strip (y -0.2 .. +0.2)
DECK_DEPTH = 0.370
PRODUCT_Y = {"left": 0.060, "mid": -0.040, "right": -0.140}  # facing centre lines (the right arm does the ticket work)
CHANGED = ("mid", "right")       # the two products whose price changes

# Ticket strip, in the strip frame: origin at (XF, 0, Z_DECK); x forward, z up.
STRIP_H = 0.042                  # overall profile height (39 mm ticket height)
TICKET_H, TICKET_W = 0.039, 0.060
TICKET_T = 0.0003                # 300 gsm white ticket card
TICKET_MASS = 0.0007             # 300 g/m2 x 60 x 39 mm
XB = -0.0010                     # front face of the strip's back wall (1.0 mm PVC back on the shelf flange)
CHANNEL = 0.0010                 # back wall to the inside of the clear front, closed
XP = XB - CHANNEL                # inside face of the clear front, closed
FRONT_T = 0.0008                 # clear front thickness
Z_FLOOR = -TICKET_H              # channel floor (bottom fold); a seated ticket's top is flush with the back wall
BEAD = 0.0008                    # the front's top edge is rolled inward: 0.2 mm from the back wall, closed
BEAD_Z = (Z_FLOOR + 0.0240, Z_FLOOR + 0.0275)
RAIL_Z = (Z_FLOOR + 0.0215, Z_FLOOR + 0.0295)                # top edge of the clear front (8 mm band)
# The clear front is 29.5 mm tall: a seated ticket's top 9.5 mm stands above its rolled edge, against the back wall.
LIP = 0.0060                     # grip edge: 6 mm flat tab along the top, 1 mm thick
PIVOT = (XP - FRONT_T / 2, 0.0, Z_FLOOR)
FRONT_OPEN_DEG = -36.0           # hinge stop, fully open (negative = top edge toward the robot)
FRONT_SPRING = 0.060             # N m / rad
FRONT_PRELOAD_DEG = 10.0         # spring rest 10 deg past closed: 0.31 N at the bead with tickets in
FRONT_MASS = 0.013               # 400 x 29.5 x 0.8 mm clear PVC
_BEAD_H = (BEAD_Z[0] + BEAD_Z[1]) / 2 - Z_FLOOR
FRONT_START_DEG = -math.degrees((TICKET_T - (CHANNEL - BEAD)) / _BEAD_H)   # pushed open by the seated tickets

CLEAR = (0.72, 0.80, 0.86)
STEEL = (0.82, 0.83, 0.85)
WHITE = (0.96, 0.96, 0.94)
GREY = (0.45, 0.47, 0.50)
TIN_COLOR = {"left": (0.95, 0.80, 0.10), "mid": (0.80, 0.10, 0.10), "right": (0.15, 0.55, 0.20)}


def _b(x0, x1, y0, y1, z0, z1, color=None):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color=color)


HALF = SECTION / 2
STRIP_BASE = [
    _b(XB, 0.0, -HALF, HALF, -STRIP_H, 0.0),                                 # back wall (taped to the flange)
    _b(XP - FRONT_T, XB, -HALF, HALF, -STRIP_H, Z_FLOOR),                    # bottom fold / channel floor
]
STILE = 0.005
FRONT_SHAPES = [
    _b(XP - FRONT_T, XP, -HALF, HALF, Z_FLOOR + 0.0002, Z_FLOOR + 0.0052),   # lower edge of the clear front
    _b(XP - FRONT_T, XP, -HALF, HALF, RAIL_Z[0], RAIL_Z[1]),                 # upper edge
    _b(XP, XP + BEAD, -HALF, HALF, BEAD_Z[0], BEAD_Z[1]),                    # rolled-in bead that grips tickets
    _b(XP - FRONT_T - LIP, XP - FRONT_T, -HALF, HALF, RAIL_Z[1] - 0.0010, RAIL_Z[1]),   # grip edge
    _b(XP - FRONT_T, XP, -HALF, -HALF + STILE, Z_FLOOR + 0.0052, RAIL_Z[0]),            # end stiles
    _b(XP - FRONT_T, XP, HALF - STILE, HALF, Z_FLOOR + 0.0052, RAIL_Z[0]),
    _b(XP - FRONT_T, XP, -HALF + STILE, HALF - STILE, Z_FLOOR + 0.0052, RAIL_Z[0]),   # clear window (see below)
]
WINDOW_SHAPE = len(FRONT_SHAPES) - 1
STRIP = Mechanism(
    "strip", (XF, 0.0, Z_DECK), STRIP_BASE,
    [Link("front", FRONT_SHAPES,
          Hinge("y", PIVOT, FRONT_OPEN_DEG, 0.0, damping=0.002, spring=FRONT_SPRING, rest=FRONT_PRELOAD_DEG,
                initial=FRONT_START_DEG, armature=1e-5),
          mass=FRONT_MASS, color=CLEAR, friction=(0.20, 0.15))],
    color=CLEAR, friction=(0.20, 0.15), contact_offset=0.0005)

# Gondola base shelf section (fixed): 1.5 mm deck, 42 mm front flange, base plinth set back 15 mm.
SHELF = Part("shelf", (XF, 0.0, Z_DECK), [
    _b(0.0, DECK_DEPTH, -HALF, HALF, -0.0015, 0.0),
    _b(0.0, 0.0015, -HALF, HALF, -STRIP_H, 0.0),
    _b(0.015, DECK_DEPTH, -HALF, HALF, -SHELF_H, -STRIP_H),
], color=STEEL, friction=(0.4, 0.35), contact_offset=0.001)

# Food cans: 3 x 4-7/16 in (76 x 113 mm) No. 300 cans, 15 oz net, 0.47 kg, fronts 110 mm behind the strip.
CAN_R, CAN_H, CAN_MASS = 0.0381, 0.1127, 0.47
CAN_X = XF + 0.110 + CAN_R
CANS = [Part(f"can_{k}", (CAN_X, y, Z_DECK + CAN_H / 2), [cyl((0, 0, 0), CAN_R, CAN_H)], CAN_MASS, TIN_COLOR[k],
             friction=(0.4, 0.35), contact_offset=0.001) for k, y in PRODUCT_Y.items()]

# Tickets: ticket frame origin at the card's centre; -x is the printed face, y along the 60 mm width, z up.
BAND_Z = (-TICKET_H / 2 + 0.006, -TICKET_H / 2 + 0.022)     # product name and price band, printed face only


def _ticket(name, pos, product):
    shapes = [box((0, 0, 0), (TICKET_T, TICKET_W, TICKET_H), color=WHITE),
              box((-TICKET_T / 2 - 0.00005, 0, sum(BAND_Z) / 2), (0.0001, TICKET_W - 0.008, BAND_Z[1] - BAND_Z[0]),
                  color=TIN_COLOR[product])]
    return Part(name, pos, shapes, TICKET_MASS, WHITE, friction=(0.80, 0.70), contact_offset=0.0005)


SEAT_X = XF + XB - TICKET_T / 2                              # a seated ticket's centre plane
SEAT_Z = Z_DECK + Z_FLOOR + TICKET_H / 2
TICKET_LEFT = _ticket("ticket_left", (SEAT_X, PRODUCT_Y["left"], SEAT_Z), "left")
OLD = {k: _ticket(f"old_{k}", (SEAT_X, PRODUCT_Y[k], SEAT_Z), k) for k in CHANGED}

# Ticket stand (fixed, grey ABS): two 1.0 mm slots along y, 25 mm apart, walls 15 mm high.
XS, YS = 0.075, -0.175
SLOT_X = {"right": -0.0125, "mid": 0.0125}                 # the right can's new ticket stands in the front slot
SLOT_W, STAND_FLOOR, STAND_WALL = 0.0010, 0.004, 0.015
STAND_L = 0.064


def _stand_shapes():
    shapes = [_b(-0.025, 0.025, -0.036, 0.036, 0.0, STAND_FLOOR)]
    edges = sorted([-0.025, 0.025] + [c + s * SLOT_W / 2 for c in SLOT_X.values() for s in (-1, 1)])
    for x0, x1 in zip(edges[0::2], edges[1::2]):
        shapes.append(_b(x0, x1, -STAND_L / 2, STAND_L / 2, STAND_FLOOR, STAND_WALL))
    for s in (-1, 1):
        y0, y1 = sorted((s * STAND_L / 2, s * 0.036))
        shapes.append(_b(-0.025, 0.025, y0, y1, STAND_FLOOR, STAND_WALL))
    return shapes


STAND = Part("ticket_stand", (XS, YS, TOP), _stand_shapes(), color=GREY, friction=(0.3, 0.25), contact_offset=0.0005)
NEW = {k: _ticket(f"new_{k}", (XS + SLOT_X[k] + SLOT_W / 2 - TICKET_T / 2, YS, TOP + STAND_FLOOR + TICKET_H / 2), k)
       for k in CHANGED}

# Waste tray (fixed, grey ABS): 100 x 120 mm outside, 25 mm walls, 3 mm floor.
XW, YW = 0.050, -0.050
TRAY_IN = (0.047, 0.057)
TRAY_WALL_Z, TRAY_FLOOR = 0.025, 0.003
TRAY = Part("waste_tray", (XW, YW, TOP), [
    _b(-0.050, 0.050, -0.060, 0.060, 0.0, TRAY_FLOOR),
    _b(-0.050, -0.047, -0.060, 0.060, TRAY_FLOOR, TRAY_WALL_Z),
    _b(0.047, 0.050, -0.060, 0.060, TRAY_FLOOR, TRAY_WALL_Z),
    _b(-0.047, 0.047, -0.060, -0.057, TRAY_FLOOR, TRAY_WALL_Z),
    _b(-0.047, 0.047, 0.057, 0.060, TRAY_FLOOR, TRAY_WALL_Z),
], color=(0.30, 0.32, 0.36), friction=(0.4, 0.35), contact_offset=0.001)

# Fingers touch the strip, the shelf flange's top and the tickets on purpose (presses and friction drags).
PRESSED_PARTS = ("strip", "shelf", "ticket_left", "old_mid", "old_right", "new_mid", "new_right")
OBJECTS = [SHELF, STRIP, *CANS, TICKET_LEFT, *OLD.values(), STAND, *NEW.values(), TRAY]

# ------------------------------------------------------------------ scoring
SEAT_TOL = 0.002          # ticket's lowest corner within 2 mm of the channel floor
UPRIGHT_DEG = 5.0         # ticket plane within 5 deg of vertical and of the strip
CENTRE_TOL = 0.015        # ticket centre within 15 mm (along the shelf) of its can's centre line
CHANNEL_TOL = 0.0010      # ticket plane between the back wall and the closed front, 1.0 mm allowance
FRONT_SHUT_DEG = -2.0     # clear front within 2 deg of shut (it rests at -0.2 deg on a ticket)
FACE_DEG = 30.0           # printed face toward the aisle: face normal within 30 deg of -x
CAN_TILT_DEG = 20.0       # sticky zero: any can tipped more than 20 deg ...
CAN_DROP = 0.020          # ... or more than 20 mm below the deck (off the shelf)
STEP_M = 0.0005           # stillness: tickets, cans and the front move under 0.5 mm per control step
TEST_STICKY = ()          # offline tests: sticky keys that already fired earlier in the episode


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def _corners(t):
    R = t.rotation()
    return [t.to_world((sx * TICKET_T / 2, sy * TICKET_W / 2, sz * TICKET_H / 2))
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def seated(t, product, objects):
    """Ticket upright in the strip's channel, bottomed, printed face out, centred under `product`'s can, and the
    clear front shut on it."""
    R = t.rotation()
    p = np.asarray(t.origin, float)
    up = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))
    face = math.degrees(math.acos(float(np.clip(R[0, 0], -1.0, 1.0))))       # printed face (local -x) vs world -x
    low = min(c[2] for c in _corners(t))
    x_ok = XF + XP - CHANNEL_TOL <= p[0] <= XF + XB + CHANNEL_TOL
    floor = Z_DECK + Z_FLOOR
    shut = objects["strip"].angle_deg("front") >= FRONT_SHUT_DEG
    return bool(x_ok and abs(low - floor) <= SEAT_TOL and up <= UPRIGHT_DEG and face <= FACE_DEG
                and abs(p[1] - PRODUCT_Y[product]) <= CENTRE_TOL and shut)


def in_strip(t):
    """Any part of the ticket still down in the channel (below the back wall's top, over the channel)."""
    p = np.asarray(t.origin, float)
    low = min(c[2] for c in _corners(t))
    return bool(XF + XP - 0.003 <= p[0] <= XF + 0.002 and low < Z_DECK - 0.002 and abs(p[1]) <= HALF)


def in_tray(t):
    p = np.asarray(t.origin, float)
    return bool(abs(p[0] - XW) <= TRAY_IN[0] and abs(p[1] - YW) <= TRAY_IN[1]
                and TOP + TRAY_FLOOR - 0.002 <= p[2] <= TOP + TRAY_WALL_Z)


def can_down(c):
    return c.tilt_deg() > CAN_TILT_DEG or c.origin[2] < Z_DECK + CAN_H / 2 - CAN_DROP


def ticket_names():
    return ["ticket_left"] + [f"old_{k}" for k in CHANGED] + [f"new_{k}" for k in CHANGED]


def credits(objects):
    """Per product: old ticket out of the strip and in the waste tray (0.25); the right new ticket seated under
    the product with its old ticket gone from the strip (0.25)."""
    out = {}
    for k in CHANGED:
        old, new = objects[f"old_{k}"], objects[f"new_{k}"]
        old_out = not in_strip(old) and in_tray(old)
        new_in = seated(new, k, objects) and not in_strip(old)
        out[k] = (bool(old_out), bool(new_in))
    return out


_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS = None
_QUIET = False


def at_rest(objects):
    """Pose change between consecutive control steps (contact chatter makes reported speeds unreliable)."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS, _QUIET
    names = ticket_names() + [c.name for c in CANS]
    if _RUN["t"] is None:
        return bool(all(objects[n].resting() for n in names) and abs(objects["strip"].joint_vel["front"]) < 0.05)
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS, _POSE_TOKEN, _QUIET = None, None, False
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"], float(_RUN["t"]))
    if token != _POSE_TOKEN:
        pose = np.r_[np.ravel([_corners(objects[n])[::7] for n in ticket_names()]),
                     np.ravel([objects[c.name].origin for c in CANS]),
                     0.0385 * objects["strip"].joints["front"]]
        _QUIET = False
        if _PREVIOUS is not None:
            step, old = _PREVIOUS
            _QUIET = bool(token[0] == step + 1 and np.all(np.abs(pose - old) < STEP_M))
        _PREVIOUS = (token[0], pose)
        _POSE_TOKEN = token
    return _QUIET


DEBUG_TCP = True
_WINDOW_HIDDEN = {"done": False}


def _hide_clear_window():
    """The clear window collides like the real transparent sheet but is not drawn, so the tickets behind it show
    (the renderer has no transparent material). Visibility does not change physics."""
    if _WINDOW_HIDDEN["done"] or _RUN["env"] is None:
        return
    _WINDOW_HIDDEN["done"] = True
    try:
        import isaaclab.sim as sim_utils
        from pxr import UsdGeom
        stage = sim_utils.get_current_stage()
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.endswith(f"/front/shape_{WINDOW_SHAPE}") and "strip" in path:
                UsdGeom.Imageable(prim).MakeInvisible()
                print("hid clear window", path, flush=True)
    except Exception as exc:  # rendering detail only; never break scoring
        print("clear window not hidden:", exc, flush=True)


def score(objects):
    _hide_clear_window()
    if DEBUG_TCP and _RUN["t"] is not None and _RUN["step"] % 10 == 0:
        hands = grippers()
        if hands:
            print("TCPDBG", _RUN["step"], {k: (np.round(v[0], 4).tolist(), round(v[1], 3)) for k, v in hands.items()},
                  flush=True)
    knocked = sticky("a can knocked over or off the shelf", any(can_down(objects[c.name]) for c in CANS))
    left_ok = seated(objects["ticket_left"], "left", objects)
    clear = all(released(objects[n]) for n in ticket_names())
    quiet = at_rest(objects)
    done = clear and quiet
    seconds = held_for("tickets released and still", done)
    if knocked:
        return 0.0
    if not (done and seconds + 1e-9 >= STILL_S) or not left_ok:
        return 0.0
    return float(sum(0.25 * a + 0.25 * b for a, b in credits(objects).values()))


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo (authored layout and time only)
# The right hand does the ticket work; the left hand holds the clear front open by pressing its grip edge down.
# Right grip yaw +90 deg: fingers close along world x (across a ticket's thickness). Left yaw 0: the closed
# fingertips' 14 mm width lies along x, over the grip edge.
PAD = 0.013                       # pad tips below the TCP
R_YAW, L_YAW = math.pi / 2, 0.0
FINGER_T = 0.0035                 # one fingertip's thickness at the pad tip
L_Y = 0.160                      # the left hand opens the front beyond the left can's ticket (70 mm clear of it)
DEMO_PACE = 1.0


def _rot_y(deg):
    a = math.radians(deg)
    return np.array(((math.cos(a), 0, math.sin(a)), (0, 1, 0), (-math.sin(a), 0, math.cos(a))))


def front_point(p_strip, deg):
    """World position of a point of the clear front (strip frame at q = 0) with the front at `deg`."""
    pivot = np.array(PIVOT)
    q = pivot + _rot_y(deg) @ (np.asarray(p_strip, float) - pivot)
    return np.array((XF, 0.0, Z_DECK)) + q


LIP_TIP = (XP - FRONT_T - LIP, 0.0, RAIL_Z[1])


def lip_press(deg, depth=0.004):
    """Left TCP pressing the front 2-3 mm behind the grip edge's tip, `depth` below it, front at `deg`."""
    tip = front_point(LIP_TIP, deg)
    return np.array((tip[0] - 0.0035, L_Y, tip[2] - depth + PAD))


OPEN_STEPS = (-6.0, -14.0, -24.0, -34.0)    # pressed to its 36 deg stop
# Tilted-wrist waypoints: a float yaw (for the offline checker's coarse replay) carrying the full grip rotation for
# the arm's IK, the pattern the oximeter and hoist demos use.
class GripOrientation(float):
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_tkt_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


_grip_rotation._tkt_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation

# Slide-up: the left hand eases the front 8 deg open (the rolled edge lets go of the tickets but still keeps them
# upright); the right hand's closed fingertips, wrist pitched 15 deg so the shelf-side fingertip's lower corner leads,
# press the 9.5 mm of the old ticket that shows above the front against the back wall and slide it 20 mm up. The
# front then closes on its lower part and holds it raised, ready to be pinched.
EASE_STEPS = (-4.0, -8.0)
PULL_EASE = (-5.0, -12.0)        # the rolled edge well off the pinched ticket while it is pulled out
DRAG_TILT_DEG = 15.0
R_DRAG = _rot_y(-DRAG_TILT_DEG) @ _original_grip_rotation(R_YAW)
TIP_CORNER = _rot_y(-DRAG_TILT_DEG) @ np.array((FINGER_T, 0.0, -PAD))   # shelf-side fingertip's lower corner
# Corner below the ticket's top: the arm reaching the farther middle ticket rides ~1-2 mm higher, so aim lower there.
DRAG_DEPTH = {"right": 0.006, "mid": 0.0075}
# Commanded past the ticket's face into the back wall; the stiffer near reach (right ticket) needs less to slide it
# without tipping its top back over the wall.
DRAG_PUSH = {"right": 0.009, "mid": 0.012}
DRAG_UP = 0.020
PINCH_TIPS = 0.003                # pinch the raised ticket with the pad tips 3 mm above the back wall
SEAT_PLANE = SEAT_X               # world x of a seated ticket's centre plane
INSERT_PLANE = XF + XB - 0.0045   # carried ticket 4.5 mm in front of the back wall
NEW_PINCH = 0.010                 # pads cover the top 10 mm of a new ticket
STAND_TCP_Z = TOP + STAND_FLOOR + TICKET_H - NEW_PINCH + PAD
INSERT_TIPS = 0.003               # release the new ticket with the pad tips 3 mm above the back wall
PARK_L, PARK_R = (0.10, 0.25, TOP + 0.16), (0.10, -0.25, TOP + 0.16)
OPEN_HALF = -0.015                # 30 mm between the pads


def demo():
    plan = {"left": [], "right": []}
    st = dict(l=np.array(lip_press(0.0)) + (0, 0, 0.04), lf=FINGER_CLOSED,
              r=np.array((0.12, -0.10, Z_DECK + 0.08)), rf=FINGER_CLOSED)

    def add(dt, label, **change):
        for k, v in change.items():
            st[k] = np.asarray(v, float) if k in ("l", "r") else v
        plan["left"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in st["l"]), float(st["lf"]), label, L_YAW))
        plan["right"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in st["r"]), float(st["rf"]), label,
                                      st.get("rr", R_YAW)))

    def open_front(steps=OPEN_STEPS):
        add(1.2, "left: fingertips onto the grip edge", l=lip_press(0.0, depth=-0.001))
        for deg in steps:
            add(0.8, f"left: press the grip edge down, front open {-deg:.0f} deg", l=lip_press(deg))
        add(0.5, "left: hold the front open")

    def close_front(steps=OPEN_STEPS):
        for deg in steps[-2::-1] + (0.0,):
            add(0.8, "left: let the front close", l=lip_press(deg, depth=0.002))
        add(0.8, "left: lift off the grip edge", l=lip_press(0.0) + (0, 0, 0.03))

    add(2.5, "move over the shelf")
    for k in ("right", "mid"):
        y = PRODUCT_Y[k]
        face = SEAT_PLANE - TICKET_T / 2                       # world x of the seated ticket's printed face
        drag_x = face - TIP_CORNER[0]
        drag_z = Z_DECK - DRAG_DEPTH[k] - TIP_CORNER[2]
        add(2.0, f"right: over the old {k} ticket, wrist pitched", r=(drag_x - 0.004, y, drag_z + 0.030),
            rf=FINGER_CLOSED, rr=GripOrientation(R_DRAG))
        open_front(EASE_STEPS)
        add(1.2, "right: fingertips onto the ticket's face above the front", r=(drag_x - 0.002, y, drag_z))
        add(1.0, "right: press it against the back wall", r=(drag_x + DRAG_PUSH[k], y, drag_z))
        add(0.5, "right: settle")
        add(2.5, "right: slide the old ticket up", r=(drag_x + DRAG_PUSH[k], y, drag_z + DRAG_UP))
        add(0.5, "right: hold it up")
        close_front(EASE_STEPS)
        add(0.6, "right: part the fingertips (let go of any card edge between them)", rf=-0.004)
        add(0.8, "right: ease off the ticket", r=(drag_x - 0.012, y, drag_z + DRAG_UP))
        add(0.8, "right: rise above it", r=(drag_x - 0.012, y, Z_DECK + 0.080))
        add(1.0, "right: wrist straight down, open", r=(SEAT_PLANE, y, Z_DECK + 0.070), rf=OPEN_HALF, rr=R_YAW)
        add(1.2, "right: pads either side of the raised ticket", r=(SEAT_PLANE, y, Z_DECK + PINCH_TIPS + PAD))
        add(1.0, "right: pinch the old ticket", rf=FINGER_CLOSED)
        add(0.4, "right: settle the pinch")
        open_front(PULL_EASE)                                  # ease the rolled edge off the raised ticket
        add(1.5, "right: pull the old ticket out", r=(SEAT_PLANE, y, Z_DECK + 0.065))
        close_front(PULL_EASE)
        add(1.2, "right: back away from the shelf", r=(0.10, y, Z_DECK + 0.060))
        add(1.6, "right: over the waste tray", r=(XW, YW, TOP + 0.085))
        add(0.8, "right: drop the old ticket", rf=OPEN_HALF)
        sx = XS + SLOT_X[k] + SLOT_W / 2 - TICKET_T / 2
        add(1.8, f"right: over the new {k} ticket", r=(sx, YS, STAND_TCP_Z + 0.050))
        add(1.2, "right: pads around its top", r=(sx, YS, STAND_TCP_Z))
        add(1.0, "right: pinch the new ticket", rf=FINGER_CLOSED)
        add(0.4, "right: settle the pinch")
        add(1.5, "right: lift it out of the stand", r=(sx, YS, STAND_TCP_Z + 0.070))
        carry_z = Z_DECK + 0.008 + (TICKET_H - NEW_PINCH) + PAD
        add(2.2, "right: carry it over the strip", r=(INSERT_PLANE, y, carry_z))
        open_front()
        add(2.0, "right: lower it into the open strip", r=(INSERT_PLANE, y, Z_DECK + INSERT_TIPS + PAD))
        add(0.8, "right: let go", rf=-0.006)
        add(1.0, "right: rise clear", r=(INSERT_PLANE, y, Z_DECK + 0.050))
        close_front()
        add(0.6, "right: close the fingers", rf=FINGER_CLOSED)
        add(1.0, "right: over the new ticket's top", r=(SEAT_PLANE, y, Z_DECK + 0.020))
        add(1.2, "right: press it down to the floor", r=(SEAT_PLANE, y, Z_DECK - 0.003 + PAD))
        add(0.5, "right: hold")
        add(1.0, "right: rise", r=(SEAT_PLANE - 0.010, y, Z_DECK + 0.060))
    add(1.5, "both: park", l=PARK_L, r=PARK_R, lf=FINGER_OPEN, rf=FINGER_OPEN)
    add(3.5, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
