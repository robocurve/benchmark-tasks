"""On the station table: a tabletop ironing board spanning both arms (64 cm along the table, 28 cm deep, 2.5 cm
thick, fixed). A dark-blue steam iron (20 x 8 x 10 cm, 1.2 kg) stands soleplate down on the board between the
arms, its long axis pointing away from them. A light-blue shirt, folded flat (24 x 16 cm), lies on the board's
left end, 3 cm to the left of the iron. Past the board's left end, in front of the left arm, is a grey silicone
iron rest (26 x 14 cm) with a 2 cm raised lip around a 24 x 12 cm pad. The shirt lies between the iron and the
rest. The iron and the shirt are loose; the board and the rest are fixed.

The iron is hot: move it onto the rest, soleplate down, without letting it touch the shirt, and let go of it.

Run it:  inspect-robots run --embodiment yam_arena -E task=park_hot_iron.py --scorer task_score \\
           --instruction "The iron is hot. Move it from the ironing board onto the grey heat-resistant rest, \\
soleplate down, without letting it touch the shirt lying on the board, and let go of it." ...
"""

import math
import sys

import numpy as np

from yam_arena import YAM_URDF
from yam_arena.analysis import link_hulls
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SPAWN_GAP = 0.001  # loose objects spawn this far above what they rest on and settle onto it

# Tabletop ironing board: a wooden board under a cotton cover, its folding legs left out. y -38 to +26 cm.
BOARD = Box("ironing_board", pos=(0.30, -0.06, TOP + 0.0125), size=(0.28, 0.64, 0.025), color=(0.78, 0.75, 0.68))
BOARD_TOP = TOP + BOARD.size[2]

# The iron as one rigid box: soleplate face down (its body -z), long axis along x. y -2 to +6 cm.
IRON = Box(
    "iron",
    pos=(0.30, 0.02, BOARD_TOP + 0.05 + SPAWN_GAP),
    size=(0.20, 0.08, 0.10),
    mass=1.2,
    color=(0.16, 0.24, 0.55),
)

# A cotton shirt, folded flat, as a thin rigid sheet, between the iron and the rest. y +9 to +25 cm.
SHIRT = Box(
    "shirt",
    pos=(0.30, 0.17, BOARD_TOP + 0.0025 + SPAWN_GAP),
    size=(0.24, 0.16, 0.005),
    mass=0.2,
    color=(0.55, 0.72, 0.92),
)

# Silicone iron rest: a pad on the table inside a raised lip. Inner pocket 24 x 12 cm, outer y +28 to +42 cm.
REST_XY = (0.30, 0.35)
REST_INNER = (0.24, 0.12)
LIP_W, LIP_H, PAD_H = 0.01, 0.02, 0.008
PAD_TOP = TOP + PAD_H
REST_GREY = (0.45, 0.46, 0.48)
_rx, _ry = REST_XY
_ix, _iy = REST_INNER[0] / 2, REST_INNER[1] / 2
_LIP_Z = TOP + LIP_H / 2
_END_LIP = (LIP_W, REST_INNER[1] + 2 * LIP_W, LIP_H)  # across the pocket's near and far ends
_SIDE_LIP = (REST_INNER[0], LIP_W, LIP_H)  # along its left and right sides
REST = [
    Box("rest_pad", pos=(_rx, _ry, TOP + PAD_H / 2), size=(*REST_INNER, PAD_H), color=REST_GREY),
    Box("rest_lip_far", pos=(_rx + _ix + LIP_W / 2, _ry, _LIP_Z), size=_END_LIP, color=REST_GREY),
    Box("rest_lip_near", pos=(_rx - _ix - LIP_W / 2, _ry, _LIP_Z), size=_END_LIP, color=REST_GREY),
    Box("rest_lip_left", pos=(_rx, _ry + _iy + LIP_W / 2, _LIP_Z), size=_SIDE_LIP, color=REST_GREY),
    Box("rest_lip_right", pos=(_rx, _ry - _iy - LIP_W / 2, _LIP_Z), size=_SIDE_LIP, color=REST_GREY),
]

OBJECTS = [BOARD, IRON, SHIRT, *REST]

# Scoring thresholds (m, deg, s).
SHIRT_MOVE_MAX = 0.01  # any point of the shirt moving further than this from its start fails the trial
TOUCH_GAP = 0.002  # the iron closer than this to the shirt counts as touching it
LIFT = 0.03  # the iron's lowest point this far above the board's top: 0.25
FLAT_DEG = 10.0  # soleplate down: the iron's up axis within this of vertical
ON_PAD = 0.01  # the iron's lowest point within this of the pad's top: resting on the rest, not held above it
DROPPED = 0.05  # the iron's lowest point this far below the table top: it fell off the table
RELEASED = 0.03  # every fingertip of both grippers at least this far from the iron's surface: let go
SUCCESS_HOLD_S = 1.0


# -- geometry ------------------------------------------------------------------------------------------------


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=float))


def _corners(state, size) -> np.ndarray:
    half = np.asarray(size) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return np.asarray(state.pos) + (signs * half) @ _rot(state.quat_xyzw).T


def _boxes_within(a, a_size, b, b_size, gap: float) -> bool:
    """Whether two oriented boxes come within ``gap`` of each other (separating-axis test, ``b`` grown by gap)."""
    ra, rb = _rot(a.quat_xyzw), _rot(b.quat_xyzw)
    ha, hb = np.asarray(a_size) / 2, np.asarray(b_size) / 2 + gap
    d = np.asarray(b.pos) - np.asarray(a.pos)
    axes = [ra[:, i] for i in range(3)] + [rb[:, j] for j in range(3)]
    axes += [np.cross(ra[:, i], rb[:, j]) for i in range(3) for j in range(3)]
    for axis in axes:
        n = np.linalg.norm(axis)
        if n < 1e-9:
            continue  # parallel edges: covered by the face axes
        axis = axis / n
        reach = np.sum(ha * np.abs(ra.T @ axis)) + np.sum(hb * np.abs(rb.T @ axis))
        if abs(d @ axis) > reach:
            return False
    return True


def _distance_to_box(points: np.ndarray, state, size) -> np.ndarray:
    """Each point's distance (m) from an oriented box's surface; 0 inside it."""
    local = (np.asarray(points) - np.asarray(state.pos)) @ _rot(state.quat_xyzw)
    excess = np.maximum(np.abs(local) - np.asarray(size) / 2, 0.0)
    return np.linalg.norm(excess, axis=1)


def _tilt_deg(state) -> float:
    """Angle between the iron's up axis (away from the soleplate) and vertical."""
    return math.degrees(math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0))))


def _clear_of_board(iron) -> bool:
    """No part of the iron's footprint over the board."""
    bx, by = BOARD.pos[0], BOARD.pos[1]
    hx, hy = BOARD.size[0] / 2, BOARD.size[1] / 2
    return bool(iron.high[0] < bx - hx or iron.low[0] > bx + hx or iron.high[1] < by - hy or iron.low[1] > by + hy)


def _in_rest(iron) -> bool:
    """Fully inside the lip, soleplate down within FLAT_DEG of flat, and resting on the pad."""
    inside = (
        iron.low[0] >= _rx - _ix
        and iron.high[0] <= _rx + _ix
        and iron.low[1] >= _ry - _iy
        and iron.high[1] <= _ry + _iy
    )
    return bool(inside and _tilt_deg(iron) <= FLAT_DEG and iron.low[2] <= PAD_TOP + ON_PAD)


# -- the robot -----------------------------------------------------------------------------------------------
#
# The task-file hooks get only the objects. Two things the score needs come from the yam_arena embodiment that
# calls them, found up the call stack: its episode clock (to tell trials apart) and the arms' joint positions
# (to check the iron was let go). Both are read the way yam_arena's own stage tests read them. If either can't
# be found, scoring raises instead of guessing.

ROBOTS = ("left_robot", "right_robot")


def _fingertips_in_link() -> dict[str, np.ndarray]:
    """Each finger's tip in its own link frame: the point of its collision hulls furthest along the gripper's
    approach axis (13 mm past the TCP)."""
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
        "park_hot_iron scores only inside a yam_arena embodiment's step (inspect-robots run --embodiment yam_arena)"
    )


def fingertip_positions(embodiment) -> np.ndarray:
    """World positions (m, the same frame as the objects) of both grippers' four fingertips, by forward
    kinematics on the URDF from the simulated joint positions and arm base poses."""
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"park_hot_iron can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"park_hot_iron: the robot has no joints {missing}")
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
        root = robot.data.root_link_pose_w.torch[0].detach().cpu().numpy().astype(float)
        to_world = np.eye(4)
        to_world[:3, :3] = quat_xyzw_to_matrix(root[3:7])
        to_world[:3, 3] = root[:3] - origin
        links = forward_kinematics(YAM_URDF, dict(zip(names, q.tolist(), strict=True)))
        for link, tip in FINGERTIPS.items():
            points.append((to_world @ links[link] @ np.r_[tip, 1.0])[:3])
    return np.array(points)


# -- per-trial record ----------------------------------------------------------------------------------------
#
# The score depends on the whole trial: touching the shirt or moving it at any time fails it, and partial
# credit is the furthest stage reached. The record restarts when the embodiment's episode clock goes back (every
# reset sets it to 0).

_TRIAL = {"t": None, "shirt0": None, "failed": None, "stage": 0.0, "tip_gap": None}


def _update(objects) -> dict:
    embodiment = _embodiment()
    t = float(embodiment._t)
    if _TRIAL["t"] is None or t < _TRIAL["t"]:
        _TRIAL.update(shirt0=_corners(objects["shirt"], SHIRT.size), failed=None, stage=0.0)
    elif t == _TRIAL["t"]:
        return _TRIAL  # success() and score() of the same step
    _TRIAL["t"] = t
    iron, shirt = objects["iron"], objects["shirt"]
    _TRIAL["tip_gap"] = float(np.min(_distance_to_box(fingertip_positions(embodiment), iron, IRON.size)))
    if _TRIAL["failed"] is None:
        moved = np.max(np.linalg.norm(_corners(shirt, SHIRT.size) - _TRIAL["shirt0"], axis=1))
        if moved > SHIRT_MOVE_MAX:
            _TRIAL["failed"] = f"shirt moved {moved * 100:.1f} cm"
        elif _boxes_within(iron, IRON.size, shirt, SHIRT.size, TOUCH_GAP):
            _TRIAL["failed"] = "iron touched the shirt"
        elif iron.low[2] < TOP - DROPPED:
            _TRIAL["failed"] = "iron fell off the table"
    if _TRIAL["failed"] is None:
        if iron.low[2] >= BOARD_TOP + LIFT:
            _TRIAL["stage"] = max(_TRIAL["stage"], 0.25)
        if _clear_of_board(iron):
            _TRIAL["stage"] = max(_TRIAL["stage"], 0.5)
    return _TRIAL


# -- hooks ---------------------------------------------------------------------------------------------------


def success(objects):
    """Parked: no failure so far, the iron soleplate down inside the rest's lip, on the pad and still, and
    every fingertip at least 3 cm from it. Held for SUCCESS_HOLD_S in a row, this ends the trial with 1.0."""
    trial = _update(objects)
    iron = objects["iron"]
    released = trial["tip_gap"] >= RELEASED
    return bool(trial["failed"] is None and _in_rest(iron) and iron.resting() and released)


def distance(objects):
    """Metres from the iron's centre to the centre of the rest's pocket at resting height."""
    goal = np.array([_rx, _ry, PAD_TOP + IRON.size[2] / 2])
    return float(np.linalg.norm(np.asarray(objects["iron"].pos) - goal))


def score(objects):
    """0 once the shirt has moved over 1 cm, the iron has touched it, or the iron has fallen off the table.
    Otherwise the furthest stage reached: 0.25 lifted 3 cm above the board, 0.5 fully off the board.
    1.0 comes from success() holding for 1 s."""
    trial = _update(objects)
    return 0.0 if trial["failed"] is not None else trial["stage"]


# -- demo ----------------------------------------------------------------------------------------------------

GRIP_BELOW_TOP = 0.03  # the TCP this far below the iron's top: the pads hold its top 4.3 cm
TRAVEL_Z = TOP + 0.17  # the highest a straight-down grip reaches over both the iron and the rest
RELEASE_GAP = 0.005  # the soleplate let go this far above the pad
PLACE_REACH = 0.012  # aim this much further out than the pocket's centre: the loaded arm falls short in x


def demo():
    """Left arm: grip the iron across its 8 cm width at its centre, lift it (the 1.2 kg load sags the arm
    about 2.5 cm below its target), carry it left over the shirt to above the rest, lower it into the pocket,
    let go and rise back to the travel height."""
    grip = IRON.size[2] / 2 - GRIP_BELOW_TOP  # TCP above the iron's centre
    pick = (IRON.pos[0], IRON.pos[1], IRON.pos[2] - SPAWN_GAP + grip)
    place = (_rx + PLACE_REACH, _ry, PAD_TOP + RELEASE_GAP + IRON.size[2] / 2 + grip)
    return {"left": pick_place_waypoints(pick, place, travel_z=TRAVEL_Z, speed=0.6)}
