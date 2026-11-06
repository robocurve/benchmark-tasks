"""On the station table: the front of a commercial tumble dryer's lint drawer housing (fixed) sits on the right,
its opening facing the arms. The drawer's white lint screen (16 x 7 x 1.5 cm, loose) has been pulled out and lies
flat on the table in front of the opening, its long side pointing into the housing, with a grey pad of lint
(9 x 5.5 x 1.6 cm, loose) on it. An open black waste bin (fixed) stands in the middle of the table.

  seen from above                         x (cm, forward)
                    [ housing, opening at x 34, stop at x 45 ]       y -12
     [bin x 20, y 0]     [ screen x 12..28, lint on it ]             y -12

Clean the screen first: the lint goes in the bin. Only then slide the screen back into the housing until it meets
the stop. Putting the screen back with the lint still on it scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=clean_lint_drawer.py -E object_jitter=0 \\
           --scorer task_score --instruction "Clean the dryer's lint screen: ..." ...
"""

import sys

import numpy as np

from yam_arena import YAM_URDF
from yam_arena.analysis import link_hulls
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_JOINTS, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SPAWN_GAP = 0.001  # loose objects spawn this far above what they rest on and settle onto it

# -- fixed: the lint drawer housing, opening at x = 34 cm, centred y = -12 cm ---------------------------------

PY = -0.12  # the housing's (and the screen's) centre line, along x
MOUTH_X = 0.34  # the opening's front face
STOP_X = 0.45  # the back stop's front face: a fully seated screen's far end touches it
DEPTH = STOP_X - MOUTH_X  # 11 cm
SLOT_W = 0.08  # inside width (y)
SLOT_H = 0.022  # inside height above the table: the screen is 1.5 cm thick, 7 mm of clearance
WALL_T = 0.01
ROOF_T = 0.01
HOUSING_GREY = (0.78, 0.79, 0.80)
CABINET_GREY = (0.62, 0.64, 0.66)

_wall_len = DEPTH + WALL_T  # the side walls run from the opening to the back of the stop
_wall_cx = MOUTH_X + _wall_len / 2
HOUSING = [
    Box("housing_wall_left", pos=(_wall_cx, PY + SLOT_W / 2 + WALL_T / 2, TOP + SLOT_H / 2),
        size=(_wall_len, WALL_T, SLOT_H), color=HOUSING_GREY),
    Box("housing_wall_right", pos=(_wall_cx, PY - SLOT_W / 2 - WALL_T / 2, TOP + SLOT_H / 2),
        size=(_wall_len, WALL_T, SLOT_H), color=HOUSING_GREY),
    Box("housing_stop", pos=(STOP_X + WALL_T / 2, PY, TOP + SLOT_H / 2),
        size=(WALL_T, SLOT_W, SLOT_H), color=HOUSING_GREY),
    Box("housing_roof", pos=(_wall_cx, PY, TOP + SLOT_H + ROOF_T / 2),
        size=(_wall_len, SLOT_W + 2 * WALL_T, ROOF_T), color=HOUSING_GREY),
    # The dryer cabinet above the drawer, set back 3 cm from the opening so the gripper has room at the mouth.
    Box("dryer_cabinet", pos=(MOUTH_X + 0.03 + 0.05, PY, TOP + SLOT_H + ROOF_T + 0.03),
        size=(0.10, 0.14, 0.06), color=CABINET_GREY),
]

# -- fixed: an open waste bin, x = 20 cm, y = 0 ----------------------------------------------------------------

BIN_X, BIN_Y = 0.20, 0.0
BIN_INNER = 0.10
BIN_WALL_T, BIN_WALL_H, BIN_FLOOR_T = 0.01, 0.06, 0.01  # walls stand on the table; the rim is 5 cm above the floor
BIN_FLOOR_TOP = TOP + BIN_FLOOR_T
BIN_RIM = TOP + BIN_WALL_H
BIN_BLACK = (0.08, 0.08, 0.09)


def _bin() -> list[Box]:
    bi, wz = BIN_INNER / 2, TOP + BIN_WALL_H / 2
    across, along = (BIN_WALL_T, BIN_INNER + 2 * BIN_WALL_T, BIN_WALL_H), (BIN_INNER, BIN_WALL_T, BIN_WALL_H)
    return [
        Box("bin_floor", pos=(BIN_X, BIN_Y, TOP + BIN_FLOOR_T / 2), size=(BIN_INNER, BIN_INNER, BIN_FLOOR_T),
            color=BIN_BLACK),
        Box("bin_wall_far", pos=(BIN_X + bi + BIN_WALL_T / 2, BIN_Y, wz), size=across, color=BIN_BLACK),
        Box("bin_wall_near", pos=(BIN_X - bi - BIN_WALL_T / 2, BIN_Y, wz), size=across, color=BIN_BLACK),
        Box("bin_wall_left", pos=(BIN_X, BIN_Y + bi + BIN_WALL_T / 2, wz), size=along, color=BIN_BLACK),
        Box("bin_wall_right", pos=(BIN_X, BIN_Y - bi - BIN_WALL_T / 2, wz), size=along, color=BIN_BLACK),
    ]


# -- loose: the lint screen and the lint pad on it ---------------------------------------------------------------

SCREEN_SIZE = (0.16, 0.07, 0.015)  # long side along x
SCREEN_MASS = 0.12
SCREEN_X0 = 0.20  # its far end 6 cm short of the opening
SCREEN = Box("lint_screen", pos=(SCREEN_X0, PY, TOP + SCREEN_SIZE[2] / 2 + SPAWN_GAP), size=SCREEN_SIZE,
             mass=SCREEN_MASS, color=(0.96, 0.96, 0.95))

LINT_SIZE = (0.09, 0.055, 0.016)
LINT_MASS = 0.015
LINT = Box("lint_pad", pos=(SCREEN_X0, PY, TOP + SCREEN_SIZE[2] + LINT_SIZE[2] / 2 + 2 * SPAWN_GAP),
           size=LINT_SIZE, mass=LINT_MASS, color=(0.55, 0.55, 0.57))

OBJECTS = [*HOUSING, *_bin(), SCREEN, LINT]
SPECS = {o.name: o for o in (SCREEN, LINT)}

# Scoring thresholds (m, rad).
DROPPED = 0.05  # lowest point this far below the table top: it fell off the table
WALL_TOL = 0.002  # a lint pad resting against an inner bin wall is inside
LINT_LIFT = 0.01  # the lint's lowest point within this of the screen's top face, over it: still on the screen
DIRTY_DEPTH = 0.02  # screen pushed this far into the opening with the lint still on it: hard 0
SEATED_TOL = 0.005  # the screen's far end within this of the stop: seated
SQUARE_TOL = np.radians(5.0)  # the screen's long axis within this of the housing's axis, and lying flat
RELEASED = 0.03  # every fingertip of both grippers at least this far from the screen and the lint: let go
PARTIAL_CAP = 0.9  # partial credit tops out here; 1.0 needs success() held for 1 s
SUCCESS_HOLD_S = 1.0


# -- geometry ------------------------------------------------------------------------------------------------


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=float))


def extent(state, spec) -> tuple[np.ndarray, np.ndarray]:
    """The box's exact world-aligned extent (low, high corners, m) from its pose and size."""
    half = np.abs(_rot(state.quat_xyzw)) @ (np.asarray(spec.size) / 2)
    centre = np.asarray(state.pos, dtype=float)
    return centre - half, centre + half


def fell(state, spec) -> bool:
    return bool(extent(state, spec)[0][2] < TOP - DROPPED)


def lint_in_bin(objects) -> bool:
    """Wholly inside the bin: within its inner walls (to WALL_TOL), no part above the rim, its lowest point within
    5 mm of the bin floor."""
    lo, hi = extent(objects[LINT.name], LINT)
    bi = BIN_INNER / 2 + WALL_TOL
    return bool(hi[2] <= BIN_RIM and lo[2] <= BIN_FLOOR_TOP + 0.005
                and lo[0] >= BIN_X - bi and hi[0] <= BIN_X + bi and lo[1] >= BIN_Y - bi and hi[1] <= BIN_Y + bi)


def screen_in_bin(objects) -> bool:
    """Any part of the screen inside the bin's outer footprint below its rim: it was thrown away."""
    lo, hi = extent(objects[SCREEN.name], SCREEN)
    bo = BIN_INNER / 2 + BIN_WALL_T
    return bool(lo[2] < BIN_RIM and hi[0] > BIN_X - bo and lo[0] < BIN_X + bo
                and hi[1] > BIN_Y - bo and lo[1] < BIN_Y + bo)


def lint_on_screen(objects) -> bool:
    """The lint's centre over the screen's upper face (in the screen's own frame) and its lowest point no more
    than LINT_LIFT above that face."""
    s = objects[SCREEN.name]
    rot = _rot(s.quat_xyzw)
    local = (np.asarray(objects[LINT.name].pos) - np.asarray(s.pos)) @ rot
    half = np.asarray(SCREEN.size) / 2
    up = 1.0 if rot[2, 2] >= 0 else -1.0
    over = abs(local[0]) <= half[0] and abs(local[1]) <= half[1] and up * local[2] > 0
    face_z = extent(s, SCREEN)[1][2]
    return bool(over and extent(objects[LINT.name], LINT)[0][2] <= face_z + LINT_LIFT)


def screen_axis(state) -> np.ndarray:
    return _rot(state.quat_xyzw)[:, 0]


def insertion(objects) -> float:
    """How far (m, 0 to DEPTH) the screen's far end is inside the opening, counted only while it is in the slot:
    its far end within the slot's width and lying on the table under the roof. The far end is whichever end of the
    long axis points into the housing."""
    s = objects[SCREEN.name]
    axis = screen_axis(s)
    if axis[0] < 0:
        axis = -axis
    far = np.asarray(s.pos) + axis * SCREEN.size[0] / 2
    lo, _ = extent(s, SCREEN)
    if abs(far[1] - PY) > SLOT_W / 2 or lo[2] > TOP + SLOT_H:
        return 0.0
    return float(np.clip(far[0] - MOUTH_X, 0.0, DEPTH))


def square_and_flat(state) -> bool:
    axis = screen_axis(state)
    yaw = abs(np.arctan2(axis[1], abs(axis[0])))
    tilt = np.arccos(np.clip(abs(_rot(state.quat_xyzw)[2, 2]), -1.0, 1.0))
    return bool(yaw <= SQUARE_TOL and tilt <= SQUARE_TOL)


def seated(objects) -> bool:
    return insertion(objects) >= DEPTH - SEATED_TOL and square_and_flat(objects[SCREEN.name])


def hard_zero(objects) -> str | None:
    """The reason for a hard 0, if any."""
    if any(fell(objects[n], s) for n, s in SPECS.items()):
        return "fell off the table"
    if screen_in_bin(objects):
        return "screen in the bin"
    if lint_on_screen(objects) and insertion(objects) >= DIRTY_DEPTH:
        return "screen put back with the lint on it"
    return None


# -- the robot (release check, as in the team's other tasks) ------------------------------------------------------

ROBOTS = ("left_robot", "right_robot")


def _fingertips_in_link() -> dict[str, np.ndarray]:
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
    frame = sys._getframe(1)
    while frame is not None:
        owner = frame.f_locals.get("self")
        if owner is not None and hasattr(owner, "_task_step") and hasattr(owner, "_t"):
            return owner
        frame = frame.f_back
    raise RuntimeError("clean_lint_drawer checks release only inside a yam_arena embodiment's step "
                       "(inspect-robots run --embodiment yam_arena)")


def fingertip_positions(embodiment) -> np.ndarray:
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"clean_lint_drawer can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"clean_lint_drawer: the robot has no joints {missing}")
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
        root = robot.data.root_link_pose_w.torch[0].detach().cpu().numpy().astype(float)
        to_world = np.eye(4)
        to_world[:3, :3] = quat_xyzw_to_matrix(root[3:7])
        to_world[:3, 3] = root[:3] - origin
        links = forward_kinematics(YAM_URDF, dict(zip(names, q.tolist(), strict=True)))
        for link, tip in FINGERTIPS.items():
            points.append((to_world @ links[link] @ np.r_[tip, 1.0])[:3])
    return np.array(points)


def distance_to(points: np.ndarray, state, spec) -> np.ndarray:
    local = (np.asarray(points) - np.asarray(state.pos)) @ _rot(state.quat_xyzw)
    excess = np.maximum(np.abs(local) - np.asarray(spec.size) / 2, 0.0)
    return np.linalg.norm(excess, axis=1)


def released(objects, tips: np.ndarray) -> bool:
    return all(float(np.min(distance_to(tips, objects[n], s))) >= RELEASED for n, s in SPECS.items())


# -- hooks ---------------------------------------------------------------------------------------------------


def score(objects):
    """0 if the screen or the lint fell off the table, the screen is in the bin, or the screen is 2 cm or more into
    the opening with the lint still on it. Otherwise 0.9 x (0.35 L + 0.65 L d): L = 1 when the lint is wholly in
    the bin, d = the screen's insertion depth as a fraction of the 11 cm slot. Sliding the screen in without
    binning the lint earns nothing. 1.0 comes from success() holding for 1 s."""
    if hard_zero(objects):
        return 0.0
    lint = 1.0 if lint_in_bin(objects) else 0.0
    return PARTIAL_CAP * (0.35 * lint + 0.65 * lint * insertion(objects) / DEPTH)


def all_done(objects) -> bool:
    return hard_zero(objects) is None and lint_in_bin(objects) and seated(objects)


def success(objects):
    """The lint wholly in the bin, the screen seated against the stop, square and flat, both still and every
    fingertip at least 3 cm from both. Held for SUCCESS_HOLD_S, this ends the trial with 1.0."""
    if not all_done(objects) or not all(objects[n].resting() for n in SPECS):
        return False
    return released(objects, fingertip_positions(_embodiment()))


def distance(objects):
    """Metres: the lint's distance to the bin's centre at floor height (if not in it) plus the screen's far end's
    distance to the stop (if not seated)."""
    total = 0.0
    if not lint_in_bin(objects):
        goal = np.array([BIN_X, BIN_Y, BIN_FLOOR_TOP + LINT_SIZE[2] / 2])
        total += float(np.linalg.norm(np.asarray(objects[LINT.name].pos) - goal))
    if not seated(objects):
        total += DEPTH - insertion(objects) + (0.0 if insertion(objects) > 0 else
                                               abs(objects[SCREEN.name].pos[0] + SCREEN.size[0] / 2 - MOUTH_X))
    return total


# -- demo (right arm only; the left arm stays parked) --------------------------------------------------------------

YAW = {"left": 0.0, "right": 0.0}  # fingers close along y, across the lint's 5.5 cm and the screen's 7 cm width
FINGER_REACH = 0.0135  # the fingertips reach this far below the TCP
TRAVEL_Z = TOP + 0.13  # a held lint pad clears the bin's 6 cm walls by about 5 cm
PRE_OPEN = 0.008
SQUEEZE = 0.002
CLOSE_S, GRIP_SETTLE_S, OPEN_SETTLE_S = 1.5, 0.6, 0.4
REACH_FIX = (0.003, 0.0025)  # aim past the right arm's sag
GRIP_FROM_END = 0.02  # the screen is gripped this far in from its near end, which stays 5 cm outside the opening
SLIDE_LIFT = 0.003  # the screen's underside this far above the table while it slides in


def _fingers(width: float) -> tuple[float, float]:
    return max(FINGER_OPEN, -(width / 2 + PRE_OPEN)), min(FINGER_CLOSED, -(width / 2 - SQUEEZE))


def _lint() -> list[Waypoint]:
    """Lift the lint pad off the screen (fingertips 2 mm above the screen's face) and drop it in the bin."""
    fx, fy = REACH_FIX
    px, py = LINT.pos[0] + fx, LINT.pos[1] + fy
    tcp = TOP + SCREEN_SIZE[2] + FINGER_REACH + 0.002
    below = tcp - (TOP + SCREEN_SIZE[2])  # TCP above the lint's underside
    qz = BIN_FLOOR_TOP + below + 0.004
    ajar, grip = _fingers(LINT_SIZE[1])
    y = YAW["right"]
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), ajar, "to the lint", y),
        Waypoint(1.5, (px, py, tcp), ajar, "descend", y),
        Waypoint(CLOSE_S, (px, py, tcp), grip, "close slowly", y),
        Waypoint(GRIP_SETTLE_S, (px, py, tcp), grip, "grip settle", y),
        Waypoint(1.2, (px, py, TRAVEL_Z), grip, "lift", y),
        Waypoint(2.0, (BIN_X + fx, BIN_Y + fy, TRAVEL_Z), grip, "over the bin", y),
        Waypoint(1.5, (BIN_X + fx, BIN_Y + fy, qz), grip, "lower into the bin", y),
        Waypoint(0.6, (BIN_X + fx, BIN_Y + fy, qz), ajar, "release", y),
        Waypoint(OPEN_SETTLE_S, (BIN_X + fx, BIN_Y + fy, qz), ajar, "fingers clear", y),
        Waypoint(1.2, (BIN_X + fx, BIN_Y + fy, TRAVEL_Z), ajar, "out of the bin", y),
    ]


def _screen() -> list[Waypoint]:
    """Grip the screen near its near end, lift it 3 mm, slide it straight in along the slot until its far end meets
    the stop, set it down, open and back out."""
    fx, fy = REACH_FIX
    near0 = SCREEN_X0 - SCREEN_SIZE[0] / 2
    px0 = near0 + GRIP_FROM_END + fx
    px1 = (STOP_X - SCREEN_SIZE[0]) + GRIP_FROM_END + fx + 0.004  # push 4 mm past contact so it seats
    py = PY + fy
    tcp = TOP + FINGER_REACH + 0.0005
    slide = tcp + SLIDE_LIFT
    ajar, grip = _fingers(SCREEN_SIZE[1])
    y = YAW["right"]
    return [
        Waypoint(1.5, (px0, py, TRAVEL_Z), ajar, "to the screen", y),
        Waypoint(1.5, (px0, py, tcp), ajar, "descend", y),
        Waypoint(CLOSE_S, (px0, py, tcp), grip, "close slowly", y),
        Waypoint(GRIP_SETTLE_S, (px0, py, tcp), grip, "grip settle", y),
        Waypoint(0.6, (px0, py, slide), grip, "lift 3 mm", y),
        Waypoint(4.0, (px1, py, slide), grip, "slide into the slot", y),
        Waypoint(0.6, (px1, py, tcp), grip, "set down", y),
        Waypoint(0.6, (px1, py, tcp), ajar, "release", y),
        Waypoint(OPEN_SETTLE_S, (px1, py, tcp), ajar, "fingers clear", y),
        Waypoint(1.2, (px1 - 0.03, py, TOP + 0.10), ajar, "back away and up", y),
    ]


PARK = {"left": (0.14, 0.27, TOP + 0.10), "right": (0.14, -0.27, TOP + 0.10)}


def _span(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """The right arm bins the lint, then slides the clean screen home. The left arm stays parked."""
    right = [*_lint(), *_screen(), Waypoint(1.5, PARK["right"], FINGER_OPEN, "park", YAW["right"])]
    hold = _span(right) + 2.0
    right.append(Waypoint(hold - _span(right), PARK["right"], FINGER_OPEN, "hold still", YAW["right"]))
    left = [Waypoint(hold, PARK["left"], FINGER_OPEN, "parked", YAW["left"])]
    return {"left": left, "right": right}
