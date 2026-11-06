"""An orange aluminium holder and a grey panel with two steel keyhole studs.

The panel is fixed 34 cm forward and 18 cm left of the arm-base midline. The holder
starts upright in a fixed dark blue U-channel 22 cm forward and 20 cm left.
"""

import sys, pathlib; sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl

import math
import numpy as np

from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP


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
    """Remember a satisfied condition until the next episode."""
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1200

PANEL_T, PANEL_W, PANEL_H = 0.012, 0.260, 0.130
PANEL_BACK_X, PANEL_Y = 0.346, 0.180
PANEL_FACE_X = PANEL_BACK_X - PANEL_T
PANEL_X = PANEL_BACK_X - PANEL_T / 2
PANEL_Z = TOP + PANEL_H / 2
STUD_SPACING, STUD_Z = 0.100, TOP + 0.125
STUD_YS = (PANEL_Y - STUD_SPACING / 2, PANEL_Y + STUD_SPACING / 2)
SHANK_R, SHANK_L = 0.0025, 0.016
HEAD_R, HEAD_T = 0.006, 0.003
HEAD_BACK_X = PANEL_FACE_X - SHANK_L
HEAD_FRONT_X = HEAD_BACK_X - HEAD_T
SHANK_X, HEAD_X = PANEL_FACE_X - SHANK_L / 2, HEAD_BACK_X - HEAD_T / 2
CONTACT_OFFSET = 0.001
PANEL_GREY, STUD_STEEL = (0.28, 0.30, 0.33), (0.42, 0.45, 0.49)
PANEL = Part("panel", (PANEL_X, PANEL_Y, PANEL_Z), [
    box((0, 0, 0), (PANEL_T, PANEL_W, PANEL_H)),
    *[shape for y in STUD_YS for shape in (
        cyl((SHANK_X - PANEL_X, y - PANEL_Y, STUD_Z - PANEL_Z), SHANK_R, SHANK_L, axis="x", color=STUD_STEEL),
        cyl((HEAD_X - PANEL_X, y - PANEL_Y, STUD_Z - PANEL_Z), HEAD_R, HEAD_T, axis="x", color=STUD_STEEL),
    )],
], color=PANEL_GREY, contact_offset=CONTACT_OFFSET)

PLATE_T, PLATE_W, PLATE_H = 0.004, 0.160, 0.090
ENTRY_W, ENTRY_H, ENTRY_Z = 0.024, 0.024, 0.014
SLOT_W, SLOT_L = 0.008, 0.012
ENTRY_BOTTOM, ENTRY_TOP = ENTRY_Z - ENTRY_H / 2, ENTRY_Z + ENTRY_H / 2
SLOT_BOTTOM, SLOT_TOP = ENTRY_TOP, ENTRY_TOP + SLOT_L
HOLE_YS = (-STUD_SPACING / 2, STUD_SPACING / 2)
HANDLE_X, HANDLE_W, HANDLE_H = 0.020, 0.160, 0.020
HANDLE_Z = PLATE_H / 2 + HANDLE_H / 2
BRACKET_KG, BRACKET_ORANGE = 0.120, (0.90, 0.30, 0.06)

REST_X, REST_Y = 0.220, 0.200
REST_D, REST_W, REST_H = 0.030, 0.180, 0.050
REST_SLOT, REST_FLOOR_T = 0.012, 0.004
REST_WALL_T = (REST_D - REST_SLOT) / 2
REST_WALL_H = REST_H - REST_FLOOR_T
REST_WALL_X = REST_SLOT / 2 + REST_WALL_T / 2
REST_BLUE = (0.08, 0.16, 0.34)
REST = Part("rest", (REST_X, REST_Y, TOP), [
    box((0, 0, REST_FLOOR_T / 2), (REST_D, REST_W, REST_FLOOR_T)),
    *[box((s * REST_WALL_X, 0, REST_FLOOR_T + REST_WALL_H / 2),
          (REST_WALL_T, REST_W, REST_WALL_H)) for s in (-1, 1)],
], color=REST_BLUE, contact_offset=CONTACT_OFFSET)
START_GAP = 0.001
# Centre the plate with 4 mm per side; its bottom settles 1 mm onto the floor.
START_X = REST_X
START_Z = TOP + REST_FLOOR_T + PLATE_H / 2 + START_GAP
START_ORIGIN = (START_X, REST_Y, START_Z)


def _plate_strips():
    """Disjoint rectangular strips: square entries with narrower slots ABOVE them.

    Lowering the holder moves the fixed shanks up in its frame. The final top web
    is 7 mm tall; the handle joins that web along the plate's top edge.
    """
    strips = []
    for bottom, top, width in (
        (-PLATE_H / 2, ENTRY_BOTTOM, 0.0),
        (ENTRY_BOTTOM, ENTRY_TOP, ENTRY_W),
        (SLOT_BOTTOM, SLOT_TOP, SLOT_W),
        (SLOT_TOP, PLATE_H / 2, 0.0),
    ):
        cursor = -PLATE_W / 2
        gaps = [(y - width / 2, y + width / 2) for y in HOLE_YS] if width else []
        for left, right in [*gaps, (PLATE_W / 2, PLATE_W / 2)]:
            if left > cursor:
                strips.append(box((0, (cursor + left) / 2, (bottom + top) / 2),
                                  (PLATE_T, left - cursor, top - bottom)))
            cursor = right
    return strips


BRACKET = Part("bracket", START_ORIGIN, [
    box((0, 0, HANDLE_Z), (HANDLE_X, HANDLE_W, HANDLE_H)),
    *_plate_strips(),
], mass=BRACKET_KG, color=BRACKET_ORANGE, contact_offset=CONTACT_OFFSET, grip=0)
OBJECTS = [PANEL, BRACKET, REST]

# The 4 mm plate sits in the 16 mm shank span with 6 mm clearance on each side.
GOAL_X, GOAL_Y = (HEAD_BACK_X + PANEL_FACE_X) / 2, PANEL_Y
ENTRY_HEIGHT = STUD_Z - ENTRY_Z
LOCKED_STUD_LOCAL_Z = SLOT_TOP - SHANK_R
LOCKED_HEIGHT = STUD_Z - LOCKED_STUD_LOCAL_Z
GOAL_ORIGIN = (GOAL_X, GOAL_Y, LOCKED_HEIGHT)

UPRIGHT_DEG = 5.0
OFF_TABLE = 0.020
NEAR_PANEL = 0.050
PEN_TOL = 0.002  # metres of solver penetration allowed at contacting surfaces
# Shank centres inside the keyhole openings, in the holder frame (where each
# shank axis crosses the plate mid-plane).
OPEN_Z_MIN = ENTRY_BOTTOM + SHANK_R                # 0.0045: shank on the entry bottom
OPEN_Z_MAX = LOCKED_STUD_LOCAL_Z + PEN_TOL         # 0.0375: bearing, 2 mm penetration
ENTRY_Y_TOL = ENTRY_W / 2 - SHANK_R                # 0.0095: shank against an entry side
ALIGN_Y_TOL = 0.003                                # slots (1.5 mm clearance + contact)
# Bearing: each shank centre at most 0.5 mm below its slot's closed end
# (or up to PEN_TOL into it); a still, released holder rests on both.
LOCK_GAP_TOL = 0.0005
LOCK_Z_MIN = LOCKED_STUD_LOCAL_Z - LOCK_GAP_TOL    # 0.0350
REST_SPEED, REST_SPIN = 0.010, 0.500
LIFT_CREDIT, INSERT_CREDIT, LOCK_CREDIT = 0.25, 0.50, 1.0
NUM_EPS = 1e-9


def _tilt_deg(bracket):
    return math.degrees(math.acos(float(np.clip(bracket.rotation()[2, 2], -1, 1))))


def _plate_bounds(bracket):
    """Bounds of the plate alone, from feature points in the root frame."""
    corners = np.array([(x, y, z) for x in (-PLATE_T / 2, PLATE_T / 2)
                        for y in (-PLATE_W / 2, PLATE_W / 2)
                        for z in (-PLATE_H / 2, PLATE_H / 2)])
    world = np.asarray(bracket.origin) + corners @ bracket.rotation().T
    return world.min(axis=0), world.max(axis=0)


def _plate_in_shank_span(bracket):
    lo, hi = _plate_bounds(bracket)
    return bool(lo[0] >= HEAD_BACK_X - PEN_TOL - NUM_EPS
                and hi[0] <= PANEL_FACE_X + PEN_TOL + NUM_EPS)


def _slot_pair_squared(bracket):
    # Explicit lock alignment rule: each slot's yaw-induced x displacement
    # from the holder centre plane is at most the 3 mm alignment allowance.
    # This is stricter than collision fit with the long shanks, and symmetric
    # under the equivalent 0/180-degree orientations.
    return bool(abs(bracket.rotation()[0, 1]) * STUD_SPACING / 2
                <= ALIGN_Y_TOL + NUM_EPS)


def _shanks_local(bracket):
    """Each shank axis (along world x) where it crosses the plate mid-plane, in
    the holder frame, paired with its opening's local y. A 180-degree yaw is
    equivalent for this symmetric, two-sided holder, so pair by local y order."""
    rot = bracket.rotation()
    origin = np.asarray(bracket.origin)
    normal = rot[:, 0]
    if abs(normal[0]) < 0.5:
        return None
    points = []
    for y in STUD_YS:
        x = origin[0] - (normal[1] * (y - origin[1]) + normal[2] * (STUD_Z - origin[2])) / normal[0]
        points.append(rot.T @ (np.array((x, y, STUD_Z)) - origin))
    return list(zip(sorted(points, key=lambda p: p[1]), HOLE_YS))


def _shank_in_opening(point, hole_y):
    """Shank centre inside the entry window or the slot above it (local frame)."""
    z, dy = point[2], abs(point[1] - hole_y)
    if not (OPEN_Z_MIN - NUM_EPS <= z <= OPEN_Z_MAX + NUM_EPS):
        return False
    return dy <= (ENTRY_Y_TOL if z < SLOT_BOTTOM else ALIGN_Y_TOL) + NUM_EPS


def _shank_bearing(point, hole_y):
    """Shank centre at its slot's closed end: at most 0.5 mm below it."""
    return bool(LOCK_Z_MIN - NUM_EPS <= point[2] <= OPEN_Z_MAX + NUM_EPS
                and abs(point[1] - hole_y) <= ALIGN_Y_TOL + NUM_EPS)


def score(objects):
    """Highest tier; full credit needs one second of released, resting engagement."""
    bracket = objects["bracket"]
    origin = np.asarray(bracket.origin)
    lo, hi = BRACKET.aabb(origin, bracket.rotation())
    plate_lo, plate_hi = _plate_bounds(bracket)
    invalid = (_tilt_deg(bracket) > UPRIGHT_DEG + NUM_EPS
               or lo[2] < TOP + OFF_TABLE - NUM_EPS
               or plate_hi[0] > PANEL_FACE_X + PEN_TOL
               or origin[0] > PANEL_FACE_X
               or plate_lo[1] < PANEL_Y - PANEL_W / 2 - NUM_EPS
               or plate_hi[1] > PANEL_Y + PANEL_W / 2 + NUM_EPS)
    # Distance to the finite panel face, measured at the plate centre / rear face.
    dy = max(abs(origin[1] - PANEL_Y) - PANEL_W / 2, 0.0)
    dz = max(TOP - origin[2], origin[2] - (TOP + PANEL_H), 0.0)
    dx = PANEL_FACE_X - plate_hi[0]
    staged = not invalid and math.sqrt(dx * dx + dy * dy + dz * dz) <= NEAR_PANEL + NUM_EPS
    shanks = _shanks_local(bracket)
    inserted = bool(staged and _plate_in_shank_span(bracket) and shanks
                    and all(_shank_in_opening(p, y) for p, y in shanks))
    hung = (inserted and all(_shank_bearing(p, y) for p, y in shanks)
            and _slot_pair_squared(bracket) and released(bracket)
            and bracket.resting(REST_SPEED, REST_SPIN))
    still_s = held_for("hung", hung)
    if not staged:
        return 0.0
    if not inserted:
        return LIFT_CREDIT
    if hung and still_s >= STILL_S:
        return LOCK_CREDIT
    return INSERT_CREDIT


def success(objects):
    """Full credit; the harness requires another SUCCESS_HOLD_S of success."""
    return bool(score(objects) == LOCK_CREDIT)


GRIP_YAW = math.pi / 2
TCP_ABOVE_HANDLE = 0.007  # pads cover 16 mm of the 20 mm bar
GRIP_LOCAL_Z = HANDLE_Z + TCP_ABOVE_HANDLE
PICK_TCP = (START_X, REST_Y, START_Z - START_GAP + GRIP_LOCAL_Z)
LIFT_M = 0.060  # clear the 50 mm channel before any sideways travel
LIFT_TCP_Z = PICK_TCP[2] + LIFT_M
CARRY_X, APPROACH_X = PANEL_FACE_X - 0.034, PANEL_FACE_X - 0.016
PASS_OVERSHOOT_M = 0.003  # command 3 mm past GOAL_X: the arm lags, and the plate still has 3 mm to the panel face
PASS_X = GOAL_X + PASS_OVERSHOOT_M
ENTRY_TCP_Z = ENTRY_HEIGHT + GRIP_LOCAL_Z
HANG_COMP_M = 0.003
INSERT_TCP_Z = ENTRY_TCP_Z + HANG_COMP_M  # the holder rides ~3 mm low in the pads: carry it higher so the heads sit centred in the entries
LOWER_M = 0.014
RELEASE_TCP_Z = INSERT_TCP_Z - LOWER_M
RETREAT_TCP_Z = TOP + 0.176  # open pad tips 8.5 mm above the seated handle
RETREAT_BACK = (PASS_X - 0.060, GOAL_Y, RETREAT_TCP_Z)
LEFT_PARK = (0.220, 0.300, RETREAT_TCP_Z)
RIGHT_PARK = (0.180, -0.300, TOP + 0.150)

# Goal bracket box: low=(0.316, 0.100, 0.792), high=(0.336, 0.260, 0.902).
# Point-to-box distance = norm(max(low - TCP, TCP - high, 0)), in metres:
# release 0.007500; vertical retreat 0.021500; -x retreat 0.051684;
# park and hold clear 0.106199. Open pads rise clear before the -x travel.
# Planar reach = hypot(TCP.x, TCP.y - base.y), left base.y=0.305:
# hover/descend/close/lift/hold lifted 0.243772; carry 0.325000;
# approach/settle before insertion 0.341686; pass/settle after insertion/
# lower/release/vertical retreat 0.351946; -x retreat 0.296624;
# park/hold clear 0.220057. Right park/wait: base.y=-0.305, reach 0.180069.
# Every waypoint is within 0.36 m. Clear by the -x endpoint at 19.3 s;
# then park for 2 s and hold for 3 s (5 s beyond the 0.045 m clearance).


def demo():
    # Reuse the helper's hover/descend/close/60 mm lift, then make each insertion
    # direction explicit. APPROACH_X is only the pre-insertion waypoint.
    left = pick_place_waypoints(PICK_TCP, (CARRY_X, GOAL_Y, INSERT_TCP_Z),
                                travel_z=LIFT_TCP_Z, yaw=GRIP_YAW)[:4]
    # The helper has no individual lift-duration parameter: settle at the lifted
    # pose for one second before carrying away from the channel.
    left.append(Waypoint(1.0, left[-1].pos, FINGER_CLOSED, "hold lifted", GRIP_YAW))
    left += [
        Waypoint(2.5, (CARRY_X, GOAL_Y, INSERT_TCP_Z), FINGER_CLOSED, "carry to entry height", GRIP_YAW),
        Waypoint(1.5, (APPROACH_X, GOAL_Y, INSERT_TCP_Z), FINGER_CLOSED, "approach +x", GRIP_YAW),
        Waypoint(0.8, (APPROACH_X, GOAL_Y, INSERT_TCP_Z), FINGER_CLOSED, "settle before insertion", GRIP_YAW),
        Waypoint(2.5, (PASS_X, GOAL_Y, INSERT_TCP_Z), FINGER_CLOSED, "pass both heads +x", GRIP_YAW),
        Waypoint(1.0, (PASS_X, GOAL_Y, INSERT_TCP_Z), FINGER_CLOSED, "settle after insertion", GRIP_YAW),
        Waypoint(1.8, (PASS_X, GOAL_Y, RELEASE_TCP_Z), FINGER_CLOSED, "lower 14 mm", GRIP_YAW),
        Waypoint(0.8, (PASS_X, GOAL_Y, RELEASE_TCP_Z), FINGER_OPEN, "release", GRIP_YAW),
        Waypoint(1.2, (PASS_X, GOAL_Y, RETREAT_TCP_Z), FINGER_OPEN, "retreat vertically", GRIP_YAW),
        Waypoint(1.2, RETREAT_BACK, FINGER_OPEN, "retreat -x", GRIP_YAW),
        Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park", GRIP_YAW),
        Waypoint(3.0, LEFT_PARK, FINGER_OPEN, "hold clear", GRIP_YAW),
    ]
    right = [Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park clear"),
             Waypoint(sum(w.duration for w in left) - 1.5, RIGHT_PARK, FINGER_OPEN, "wait clear")]
    return {"left": left, "right": right}
