"""On the station table: a foam play mat (60 x 16 x 1 cm, fixed) runs across the front of the table between the
arms, and a grey open toy bin (14 x 14 cm inside, walls 6 cm above its floor, fixed) stands just beyond the mat's
middle. Seven loose toys stand in one row along the mat, hazards and safe toys mixed, none colour-coded:

  right                                                                              left
  y -24        -10         -3        +4         +11           +18          +25   (cm)
  rubber ball  wooden peg  cube      block      plastic box   wooden spool steel ball
  34 mm        28 x 60 mm  20 mm     60 mm      20.9x20.9x24  29.6 x 22 mm 16 mm
  safe         safe        HAZARD    safe       HAZARD        HAZARD       HAZARD

A part is a choking hazard under 16 CFR 1501 if it fits entirely inside the small-parts test cylinder (31.7 mm
wide, 25.4 to 57.1 mm deep, sloped floor). small_parts_fit.py checks every toy against that fixture; the
nearest calls clear it by at least 1.0 mm either way.

Run it:  inspect-robots run --embodiment yam_arena -E task=clear_choking_hazards.py -E object_jitter=0 \\
           --scorer task_score --instruction "A toddler under 3 will play on this mat. ..." ...
"""

import math
import sys

import numpy as np

from yam_arena import YAM_URDF
from yam_arena.analysis import link_hulls
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_JOINTS, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SPAWN_GAP = 0.001  # loose objects spawn this far above what they rest on and settle onto it

# -- fixed: play mat and toy bin -----------------------------------------------------------------------------

MAT = Box("play_mat", pos=(0.17, 0.0, TOP + 0.005), size=(0.16, 0.60, 0.01), color=(0.70, 0.86, 0.72))
MAT_TOP = TOP + MAT.size[2]

BIN_XY = (0.34, 0.0)
BIN_INNER = 0.14  # square, inside the walls
WALL_T, WALL_H, FLOOR_T = 0.01, 0.07, 0.01  # walls stand on the table; the rim is 6 cm above the bin floor
FLOOR_TOP = TOP + FLOOR_T
RIM = TOP + WALL_H
BIN_GREY = (0.38, 0.39, 0.41)
_bx, _by = BIN_XY
_bi = BIN_INNER / 2
_wz = TOP + WALL_H / 2
BIN = [
    Box("bin_floor", pos=(_bx, _by, TOP + FLOOR_T / 2), size=(BIN_INNER, BIN_INNER, FLOOR_T), color=BIN_GREY),
    Box("bin_wall_far", pos=(_bx + _bi + WALL_T / 2, _by, _wz), size=(WALL_T, BIN_INNER + 2 * WALL_T, WALL_H),
        color=BIN_GREY),
    Box("bin_wall_near", pos=(_bx - _bi - WALL_T / 2, _by, _wz), size=(WALL_T, BIN_INNER + 2 * WALL_T, WALL_H),
        color=BIN_GREY),
    Box("bin_wall_left", pos=(_bx, _by + _bi + WALL_T / 2, _wz), size=(BIN_INNER, WALL_T, WALL_H), color=BIN_GREY),
    Box("bin_wall_right", pos=(_bx, _by - _bi - WALL_T / 2, _wz), size=(BIN_INNER, WALL_T, WALL_H), color=BIN_GREY),
]

# -- loose: seven toys in a row on the mat, x = 17 cm --------------------------------------------------------

ROW_X = 0.17


def _on_mat(half_height: float) -> float:
    return MAT_TOP + half_height + SPAWN_GAP


# Hazards: each fits entirely inside the small-parts cylinder (see small_parts_fit.py).
STEEL_BALL = Sphere("steel_ball", pos=(ROW_X, 0.25, _on_mat(0.008)), radius=0.008, mass=0.0168,
                    color=(0.80, 0.81, 0.83))  # 16 mm steel ball
SMALL_CUBE = Box("wooden_cube", pos=(ROW_X, -0.03, _on_mat(0.010)), size=(0.02, 0.02, 0.02), mass=0.0056,
                 color=(0.85, 0.20, 0.18))  # 20 mm
PLASTIC_BOX = Box("plastic_box", pos=(ROW_X, 0.11, _on_mat(0.012)), size=(0.0209, 0.0209, 0.024), mass=0.011,
                  color=(0.10, 0.62, 0.62))  # 20.9 x 20.9 x 24 mm, standing on a square face
SPOOL = Cylinder("wooden_spool", pos=(ROW_X, 0.18, _on_mat(0.011)), radius=0.0148, height=0.022, mass=0.0106,
                 color=(0.90, 0.45, 0.65))  # 29.6 mm across, 22 mm tall, standing on an end
# Safe toys: none fits inside the cylinder.
RUBBER_BALL = Sphere("rubber_ball", pos=(ROW_X, -0.24, _on_mat(0.017)), radius=0.017, mass=0.023,
                     color=(0.95, 0.55, 0.12))  # 34 mm
PEG = Cylinder("wooden_peg", pos=(ROW_X, -0.10, _on_mat(0.030)), radius=0.014, height=0.06, mass=0.026,
               color=(0.50, 0.30, 0.70))  # 28 mm across, 60 mm tall, standing on an end
BLOCK = Box("wooden_block", pos=(ROW_X, 0.04, _on_mat(0.030)), size=(0.06, 0.06, 0.06), mass=0.15,
            color=(0.18, 0.36, 0.80))  # 60 mm

HAZARDS = [SMALL_CUBE, PLASTIC_BOX, SPOOL, STEEL_BALL]
SAFE = [RUBBER_BALL, PEG, BLOCK]
TOYS = HAZARDS + SAFE

OBJECTS = [MAT, *BIN, *TOYS]

# Scoring thresholds (m, s).
DROPPED = 0.05  # a toy's lowest point this far below the table top: it fell off the table
ON_MAT = 0.01  # a toy's lowest point within this of the mat's top: resting on the mat
RELEASED = 0.03  # every fingertip of both grippers at least this far from every toy: let go
PARTIAL_CAP = 0.9  # partial credit tops out here; 1.0 needs success() (all correct, still, let go) for 1 s
SUCCESS_HOLD_S = 1.0


# -- geometry ------------------------------------------------------------------------------------------------


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=float))


WALL_TOL = 0.002  # a toy resting against an inner wall is inside (contact can leave it a hair past the face)


def extent(state, spec) -> tuple[np.ndarray, np.ndarray]:
    """The toy's exact world-aligned extent (low, high corners, m) from its pose and true shape. The simulator's
    own bounding box is padded (by up to 5 mm a side for the steel ball), so the score doesn't use it."""
    centre, rot = np.asarray(state.pos, dtype=float), _rot(state.quat_xyzw)
    if isinstance(spec, Sphere):
        half = np.full(3, spec.radius)
    elif isinstance(spec, Cylinder):
        axis = np.abs(rot[:, 2])  # the cylinder's axis in world coordinates
        half = spec.height / 2 * axis + spec.radius * np.sqrt(np.clip(1.0 - axis**2, 0.0, 1.0))
    else:
        half = np.abs(rot) @ (np.asarray(spec.size) / 2)
    return centre - half, centre + half


def in_bin(state, spec) -> bool:
    """Wholly inside the bin: within the inner walls (to WALL_TOL), no higher than the rim, and down in the bin
    (its lowest point within 5 mm of the floor or another toy on it). The walls are solid, so a toy can only
    reach past them by being on or over them, and then it sticks out by more than a wall's thickness or sits
    above the rim."""
    lo, hi = extent(state, spec)
    return bool(
        lo[0] >= _bx - _bi - WALL_TOL and hi[0] <= _bx + _bi + WALL_TOL
        and lo[1] >= _by - _bi - WALL_TOL and hi[1] <= _by + _bi + WALL_TOL
        and hi[2] <= RIM and lo[2] >= FLOOR_TOP - 0.005
    )


def on_mat(state, spec) -> bool:
    """Resting on the mat: the centre of its footprint over the mat and its lowest point within 1 cm of the
    mat's top (lying on its side counts; standing on another toy does not)."""
    lo, _ = extent(state, spec)
    cx, cy = float(state.pos[0]), float(state.pos[1])
    mx, my = MAT.pos[0], MAT.pos[1]
    over = abs(cx - mx) <= MAT.size[0] / 2 and abs(cy - my) <= MAT.size[1] / 2
    return bool(over and lo[2] <= MAT_TOP + ON_MAT)


def fell(state, spec) -> bool:
    return bool(extent(state, spec)[0][2] < TOP - DROPPED)


def distance_to(points: np.ndarray, state, spec) -> np.ndarray:
    """Each point's distance (m) from a toy's surface; 0 inside it."""
    local = (np.asarray(points) - np.asarray(state.pos)) @ _rot(state.quat_xyzw)
    if isinstance(spec, Sphere):
        return np.maximum(np.linalg.norm(local, axis=1) - spec.radius, 0.0)
    if isinstance(spec, Cylinder):
        radial = np.maximum(np.hypot(local[:, 0], local[:, 1]) - spec.radius, 0.0)
        axial = np.maximum(np.abs(local[:, 2]) - spec.height / 2, 0.0)
        return np.hypot(radial, axial)
    excess = np.maximum(np.abs(local) - np.asarray(spec.size) / 2, 0.0)
    return np.linalg.norm(excess, axis=1)


def tally(objects) -> dict:
    """The end state the score is read from."""
    return {
        "fell": [t.name for t in TOYS if fell(objects[t.name], t)],
        "binned": [t.name for t in HAZARDS if in_bin(objects[t.name], t)],
        "misplaced": [t.name for t in SAFE if not on_mat(objects[t.name], t)],
    }


# -- the robot -----------------------------------------------------------------------------------------------
#
# The task-file hooks get only the objects. To check that the toys were let go, success() reads the arms' joint
# positions from the yam_arena embodiment that calls it (found up the call stack) and runs forward kinematics.
# If the state can't be found, it raises instead of guessing.

ROBOTS = ("left_robot", "right_robot")


def _fingertips_in_link() -> dict[str, np.ndarray]:
    """Each finger's tip in its own link frame: the point of its collision hulls furthest along the gripper's
    approach axis."""
    poses = forward_kinematics(YAM_URDF, {})
    to_gripper = np.linalg.inv(poses["gripper"])
    tips = {}
    for link in ("tip_left", "tip_right"):
        verts = np.vstack([hull.vertices for hull in link_hulls()[link]])
        in_gripper = (to_gripper @ poses[link] @ np.c_[verts, np.ones(len(verts))].T).T[:, :3]
        far = in_gripper[:, 2] >= in_gripper[:, 2].max() - 0.001
        tips[link] = verts[far].mean(axis=0)
    return tips


FINGERTIPS = _fingertips_in_link()


def _embodiment():
    """The yam_arena embodiment whose step is calling this hook."""
    frame = sys._getframe(1)
    while frame is not None:
        owner = frame.f_locals.get("self")
        if owner is not None and hasattr(owner, "_task_step") and hasattr(owner, "_t"):
            return owner
        frame = frame.f_back
    raise RuntimeError(
        "clear_choking_hazards checks release only inside a yam_arena embodiment's step "
        "(inspect-robots run --embodiment yam_arena)"
    )


def fingertip_positions(embodiment) -> np.ndarray:
    """World positions (m, the objects' frame) of both grippers' four fingertips."""
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"clear_choking_hazards can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"clear_choking_hazards: the robot has no joints {missing}")
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
        root = robot.data.root_link_pose_w.torch[0].detach().cpu().numpy().astype(float)
        to_world = np.eye(4)
        to_world[:3, :3] = quat_xyzw_to_matrix(root[3:7])
        to_world[:3, 3] = root[:3] - origin
        links = forward_kinematics(YAM_URDF, dict(zip(names, q.tolist(), strict=True)))
        for link, tip in FINGERTIPS.items():
            points.append((to_world @ links[link] @ np.r_[tip, 1.0])[:3])
    return np.array(points)


def released(objects, tips: np.ndarray) -> bool:
    return all(float(np.min(distance_to(tips, objects[t.name], t))) >= RELEASED for t in TOYS)


# -- hooks ---------------------------------------------------------------------------------------------------


def all_correct(objects) -> bool:
    t = tally(objects)
    return not t["fell"] and len(t["binned"]) == len(HAZARDS) and not t["misplaced"]


def success(objects):
    """Every hazard in the bin, every safe toy on the mat, nothing fallen, every toy still, and every fingertip
    at least 3 cm from every toy. Held for SUCCESS_HOLD_S in a row, this ends the trial with 1.0."""
    if not all_correct(objects) or not all(objects[t.name].resting() for t in TOYS):
        return False
    return released(objects, fingertip_positions(_embodiment()))


def distance(objects):
    """Metres, summed over the hazards not yet in the bin, from each to the bin's centre at floor height."""
    goal = np.array([_bx, _by, FLOOR_TOP + 0.01])
    return float(sum(np.linalg.norm(np.asarray(objects[t.name].pos) - goal)
                     for t in HAZARDS if not in_bin(objects[t.name], t)))


def score(objects):
    """0 if any toy fell off the table. Otherwise 0.9 x max(0, H/4 - W/3): H of the 4 hazards wholly in the bin,
    W of the 3 safe toys not resting on the mat (in the bin or anywhere else). Doing nothing scores 0; so does
    binning everything. 1.0 comes from success() holding for 1 s."""
    t = tally(objects)
    if t["fell"]:
        return 0.0
    return PARTIAL_CAP * max(0.0, len(t["binned"]) / len(HAZARDS) - len(t["misplaced"]) / len(SAFE))


# -- demo ----------------------------------------------------------------------------------------------------

# Both arms grip with the fingers closing along x, across the row, so the open fingers never reach a neighbour.
# The two grips are the same up to a half turn; each arm uses the one inside its joint limits.
YAW = {"right": math.pi / 2, "left": -math.pi / 2}
TRAVEL_Z = TOP + 0.13  # a held toy's lowest point clears the bin's rim and the 7 cm tall toys by about 4 cm
MIN_TCP = MAT_TOP + 0.0135  # the fingertips 0.5 mm above the mat (they reach 13 mm past the TCP)
DROP_ABOVE_FLOOR = 0.0  # each hazard set down on the bin floor, not dropped: a dropped steel ball rolls on and on
PRE_OPEN = 0.008  # each finger this far from the toy as the grip comes down
CLOSE_S = 1.5  # slow close: the fingers meet the toy at low speed
GRIP_SETTLE_S = 0.6  # the fingers trail their command: hold the grip before lifting
# Grip force follows how far past the toy's surface the fingers are commanded. Shutting them fully crushes a
# toy of a few grams and the contact spits it out; this much gives a firm, bounded squeeze.
SQUEEZE = 0.002
SET_DOWN_S = 0.5  # held on the bin floor this long before letting go, so the toy is still when released
OPEN_SETTLE_S = 0.4  # the fingers trail their command: let them open fully before rising, or they spin a ball
# Measured in simulation: at the mat, each arm's fingertips close about 3 mm short in x and 2.5 mm toward the
# arm's own base in y. An off-centre squeeze pushes a light toy out of the grip, so the picks aim past it.
REACH_FIX = {"right": (0.003, 0.0025), "left": (0.003, -0.0025)}


def _half_height(spec) -> float:
    if isinstance(spec, Sphere):
        return spec.radius
    if isinstance(spec, Cylinder):
        return spec.height / 2
    return spec.size[2] / 2


def _width(spec) -> float:
    """Across the fingers (which close along x)."""
    if isinstance(spec, (Sphere, Cylinder)):
        return 2 * spec.radius
    return spec.size[0]


def _pick_place(arm: str, spec, drop: tuple[float, float]) -> list[Waypoint]:
    """Top-down pick and place with a gentle grip: the fingers come down opened only PRE_OPEN wider than the toy
    on each side, close slowly to SQUEEZE inside its surface and hold before lifting, so a light toy is held
    firmly instead of being knocked or squeezed out from between them."""
    centre_z = spec.pos[2] - SPAWN_GAP
    tcp = max(centre_z, MIN_TCP)
    below = tcp - (centre_z - _half_height(spec))  # TCP height above the toy's lowest point
    fx, fy = REACH_FIX[arm]
    px, py = spec.pos[0] + fx, spec.pos[1] + fy
    qx, qy, qz = drop[0], drop[1], FLOOR_TOP + DROP_ABOVE_FLOOR + below
    ajar = max(FINGER_OPEN, -(_width(spec) / 2 + PRE_OPEN))  # finger joint: each finger PRE_OPEN off the toy
    grip = min(FINGER_CLOSED, -(_width(spec) / 2 - SQUEEZE))  # each finger SQUEEZE inside the toy's surface
    yaw = YAW[arm]
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), ajar, "to the toy", yaw),
        Waypoint(1.5, (px, py, tcp), ajar, "descend", yaw),
        Waypoint(CLOSE_S, (px, py, tcp), grip, "close slowly", yaw),
        Waypoint(GRIP_SETTLE_S, (px, py, tcp), grip, "grip settle", yaw),
        Waypoint(1.2, (px, py, TRAVEL_Z), grip, "lift", yaw),
        Waypoint(1.8, (qx, qy, TRAVEL_Z), grip, "carry", yaw),
        Waypoint(1.2, (qx, qy, qz), grip, "lower", yaw),
        Waypoint(SET_DOWN_S, (qx, qy, qz), grip, "set down", yaw),
        Waypoint(0.6, (qx, qy, qz), ajar, "release", yaw),
        Waypoint(OPEN_SETTLE_S, (qx, qy, qz), ajar, "fingers clear", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), ajar, "retreat", yaw),
    ]


# Where each hazard is set down: one per quarter of the bin, so none lands on another. With the fingers
# closing along x, the open fingers stay clear of the walls at x 31 and 37 cm.
DROP_SPOTS = {
    "wooden_cube": (_bx + 0.03, _by - 0.035),  # right arm: the far quarter on its side
    "steel_ball": (_bx - 0.03, _by - 0.035),  # left arm: the near right quarter (it can't reach the far one)
    "plastic_box": (_bx - 0.03, _by + 0.035),
    "wooden_spool": (_bx + 0.03, _by + 0.035),
}

PARK_LEFT, PARK_RIGHT = (0.20, 0.22, TRAVEL_Z), (0.20, -0.22, TRAVEL_Z)  # over each arm's end of the mat


def _park(arm: str, pos, seconds: float = 1.5, label: str = "park") -> Waypoint:
    """Over the arm's own end of the mat, clear of the bin and of the other arm."""
    return Waypoint(seconds, pos, FINGER_OPEN, label, YAW[arm])


def demo():
    """Right arm first: the 20 mm cube into the bin's far right quarter, then it parks over its end of the
    mat. Meanwhile the left arm waits over its own end of the mat; then it sets the plastic box, the spool and
    the steel ball each in its own quarter of the bin and parks. One arm over the bin at a time."""
    right = [*_pick_place("right", SMALL_CUBE, DROP_SPOTS["wooden_cube"]), _park("right", PARK_RIGHT)]
    wait = sum(w.duration for w in right)
    left = [
        _park("left", PARK_LEFT, wait, "wait over its end of the mat"),
        *_pick_place("left", PLASTIC_BOX, DROP_SPOTS["plastic_box"]),
        *_pick_place("left", SPOOL, DROP_SPOTS["wooden_spool"]),
        *_pick_place("left", STEEL_BALL, DROP_SPOTS["steel_ball"]),
        _park("left", PARK_LEFT),
        _park("left", PARK_LEFT, 2.0, "hold still"),
    ]
    return {"right": right, "left": left}

