"""On the station table: three open laundry containers stand side by side across the back of the work area, each
16 x 16 cm inside with walls 4 cm above its floor (fixed): a white basket on the left, a grey mesh bag in the
middle and a blue basket on the right. Six folded garments (8 x 5 x 2.5 cm, loose) lie in one row in front of
them, long side pointing away from the arms. Two of them carry a red hand-wash tag (2.5 x 2.5 cm, loose) lying
on top near their far end:

  right                                                                     left
  y -20         -12            -4        +4        +12            +20   (cm)
  green         green + TAG    white     navy      white + TAG    white

White items go in the white basket and coloured items in the blue basket, except that a tagged item goes in the
mesh bag whatever its colour. The tags have to stay on their garments.

Run it:  inspect-robots run --embodiment yam_arena -E task=sort_tagged_laundry.py -E object_jitter=0 \\
           --scorer task_score --instruction "Sort this laundry into the three containers: ..." ...
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

# -- fixed: three open containers in a row, x = 30 cm ---------------------------------------------------------

BOX_X = 0.30
BOX_INNER = 0.16  # square, inside the walls
WALL_T, WALL_H, FLOOR_T = 0.01, 0.05, 0.01  # walls stand on the table; the rim is 4 cm above the floor
FLOOR_TOP = TOP + FLOOR_T
RIM = TOP + WALL_H
CONTAINERS = {  # name: (centre y, colour)
    "white_basket": (0.20, (0.93, 0.93, 0.91)),
    "mesh_bag": (0.0, (0.36, 0.37, 0.39)),
    "blue_basket": (-0.20, (0.18, 0.38, 0.80)),
}


def _container(name: str, cy: float, color) -> list[Box]:
    bi, wz = BOX_INNER / 2, TOP + WALL_H / 2
    across, along = (WALL_T, BOX_INNER + 2 * WALL_T, WALL_H), (BOX_INNER, WALL_T, WALL_H)
    return [
        Box(f"{name}_floor", pos=(BOX_X, cy, TOP + FLOOR_T / 2), size=(BOX_INNER, BOX_INNER, FLOOR_T), color=color),
        Box(f"{name}_wall_far", pos=(BOX_X + bi + WALL_T / 2, cy, wz), size=across, color=color),
        Box(f"{name}_wall_near", pos=(BOX_X - bi - WALL_T / 2, cy, wz), size=across, color=color),
        Box(f"{name}_wall_left", pos=(BOX_X, cy + bi + WALL_T / 2, wz), size=along, color=color),
        Box(f"{name}_wall_right", pos=(BOX_X, cy - bi - WALL_T / 2, wz), size=along, color=color),
    ]


# -- loose: six folded garments in a row, x = 14 cm, and two tags ----------------------------------------------

ROW_X = 0.14
GARMENT = (0.08, 0.05, 0.025)  # folded flat, long side along x
GARMENT_MASS = 0.06
WHITE, NAVY, GREEN = (0.95, 0.95, 0.93), (0.12, 0.16, 0.38), (0.18, 0.52, 0.28)
TAG = (0.025, 0.025, 0.004)
TAG_MASS = 0.003
TAG_RED = (0.86, 0.08, 0.08)
TAG_FROM_CENTRE = 0.025  # the tag lies this far out along the garment's long side, away from the grip


def _garment(name: str, y: float, color) -> Box:
    return Box(name, pos=(ROW_X, y, TOP + GARMENT[2] / 2 + SPAWN_GAP), size=GARMENT, mass=GARMENT_MASS, color=color)


def _tag(name: str, garment: Box) -> Box:
    z = TOP + GARMENT[2] + TAG[2] / 2 + 2 * SPAWN_GAP
    return Box(name, pos=(garment.pos[0] + TAG_FROM_CENTRE, garment.pos[1], z), size=TAG, mass=TAG_MASS,
               color=TAG_RED)


WHITE_SHIRT = _garment("white_shirt", 0.20, WHITE)
WHITE_BLOUSE = _garment("white_blouse", 0.12, WHITE)  # tagged
NAVY_SHIRT = _garment("navy_shirt", 0.04, NAVY)
WHITE_TOWEL = _garment("white_towel", -0.04, WHITE)
GREEN_SWEATER = _garment("green_sweater", -0.12, GREEN)  # tagged
GREEN_SHIRT = _garment("green_shirt", -0.20, GREEN)
GARMENTS = [WHITE_SHIRT, WHITE_BLOUSE, NAVY_SHIRT, WHITE_TOWEL, GREEN_SWEATER, GREEN_SHIRT]

BLOUSE_TAG = _tag("blouse_tag", WHITE_BLOUSE)
SWEATER_TAG = _tag("sweater_tag", GREEN_SWEATER)
TAGS = {"blouse_tag": WHITE_BLOUSE, "sweater_tag": GREEN_SWEATER}  # tag: the garment it belongs on

# Where each garment belongs.
TARGET = {
    "white_shirt": "white_basket",
    "white_towel": "white_basket",
    "navy_shirt": "blue_basket",
    "green_shirt": "blue_basket",
    "white_blouse": "mesh_bag",
    "green_sweater": "mesh_bag",
}

OBJECTS = [*(b for n, (cy, c) in CONTAINERS.items() for b in _container(n, cy, c)), *GARMENTS, BLOUSE_TAG,
           SWEATER_TAG]

# Scoring thresholds (m).
SPECS = {o.name: o for o in [*GARMENTS, BLOUSE_TAG, SWEATER_TAG]}
DROPPED = 0.05  # lowest point this far below the table top: it fell off the table
WALL_TOL = 0.002  # a garment resting against an inner wall is inside (contact can leave it a hair past the face)
TAG_LIFT = 0.01  # a tag's lowest point more than this above its garment's top face: it is off the garment
RELEASED = 0.03  # every fingertip of both grippers at least this far from every garment and tag: let go
PARTIAL_CAP = 0.9  # partial credit tops out here; 1.0 needs success() (all correct, still, let go) for 1 s
SUCCESS_HOLD_S = 1.0


# -- geometry ------------------------------------------------------------------------------------------------


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=float))


def extent(state, spec) -> tuple[np.ndarray, np.ndarray]:
    """The box's exact world-aligned extent (low, high corners, m) from its pose and size. The simulator's own
    bounding box is padded, so the score doesn't use it."""
    half = np.abs(_rot(state.quat_xyzw)) @ (np.asarray(spec.size) / 2)
    centre = np.asarray(state.pos, dtype=float)
    return centre - half, centre + half


def container_of(state, spec) -> str | None:
    """The container the garment is wholly inside: within its inner walls (to WALL_TOL), no part above the rim,
    and down in it (lowest point within 5 mm of the floor, or resting on another garment there). The walls are
    solid, so a garment can only reach past them by being on or over them, and then it sticks out by more than a
    wall's thickness or sits above the rim."""
    lo, hi = extent(state, spec)
    if hi[2] > RIM or lo[2] < FLOOR_TOP - 0.005:
        return None
    bi = BOX_INNER / 2 + WALL_TOL
    for name, (cy, _) in CONTAINERS.items():
        if lo[0] >= BOX_X - bi and hi[0] <= BOX_X + bi and lo[1] >= cy - bi and hi[1] <= cy + bi:
            return name
    return None


def tag_on(tag_state, tag_spec, garment_state, garment_spec) -> bool:
    """The tag still lies on its own garment: its centre over the garment's upper face (in the garment's own
    frame, so this holds wherever the garment is, and whichever face is up) and its lowest point no more than
    TAG_LIFT above that face."""
    rot = _rot(garment_state.quat_xyzw)
    local = (np.asarray(tag_state.pos) - np.asarray(garment_state.pos)) @ rot
    half = np.asarray(garment_spec.size) / 2
    up = 1.0 if rot[2, 2] >= 0 else -1.0  # which of the garment's z faces is on top
    over_face = abs(local[0]) <= half[0] and abs(local[1]) <= half[1] and up * local[2] > 0
    face_z = extent(garment_state, garment_spec)[1][2]
    return bool(over_face and extent(tag_state, tag_spec)[0][2] <= face_z + TAG_LIFT)


def fell(state, spec) -> bool:
    return bool(extent(state, spec)[0][2] < TOP - DROPPED)


def distance_to(points: np.ndarray, state, spec) -> np.ndarray:
    """Each point's distance (m) from a box's surface; 0 inside it."""
    local = (np.asarray(points) - np.asarray(state.pos)) @ _rot(state.quat_xyzw)
    excess = np.maximum(np.abs(local) - np.asarray(spec.size) / 2, 0.0)
    return np.linalg.norm(excess, axis=1)


def tally(objects) -> dict:
    """The state the score is read from."""
    where = {g.name: container_of(objects[g.name], g) for g in GARMENTS}
    return {
        "fell": [n for n, s in SPECS.items() if fell(objects[n], s)],
        "tag_off": [t for t, g in TAGS.items() if not tag_on(objects[t], SPECS[t], objects[g.name], g)],
        "correct": [n for n, c in where.items() if c == TARGET[n]],
        "wrong": [n for n, c in where.items() if c is not None and c != TARGET[n]],
    }


# -- the robot -----------------------------------------------------------------------------------------------
#
# The task-file hooks get only the objects. To check that the garments were let go, success() reads the arms'
# joint positions from the yam_arena embodiment that calls it (found up the call stack) and runs forward
# kinematics. If the state can't be found, it raises instead of guessing.

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
        "sort_tagged_laundry checks release only inside a yam_arena embodiment's step "
        "(inspect-robots run --embodiment yam_arena)"
    )


def fingertip_positions(embodiment) -> np.ndarray:
    """World positions (m, the objects' frame) of both grippers' four fingertips."""
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"sort_tagged_laundry can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"sort_tagged_laundry: the robot has no joints {missing}")
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
    return all(float(np.min(distance_to(tips, objects[n], s))) >= RELEASED for n, s in SPECS.items())


# -- hooks ---------------------------------------------------------------------------------------------------


def all_correct(objects) -> bool:
    t = tally(objects)
    return not t["fell"] and not t["tag_off"] and len(t["correct"]) == len(GARMENTS)


def success(objects):
    """Every garment wholly in its container, both tags on their garments, nothing fallen, everything still,
    and every fingertip at least 3 cm from every garment and tag. Held for SUCCESS_HOLD_S, this ends the trial
    with 1.0."""
    if not all_correct(objects) or not all(objects[n].resting() for n in SPECS):
        return False
    return released(objects, fingertip_positions(_embodiment()))


def distance(objects):
    """Metres, summed over the garments not yet in their container, from each to that container's centre at
    floor height."""
    total = 0.0
    for g in GARMENTS:
        if container_of(objects[g.name], g) != TARGET[g.name]:
            goal = np.array([BOX_X, CONTAINERS[TARGET[g.name]][0], FLOOR_TOP + GARMENT[2] / 2])
            total += float(np.linalg.norm(np.asarray(objects[g.name].pos) - goal))
    return total


def score(objects):
    """0 if any garment or tag fell off the table, or a tag is not on its own garment. Otherwise
    0.9 x max(0, C/6 - W/3): C of the 6 garments wholly in the right container, W wholly in a wrong one. Doing
    nothing scores 0; so do sorting by colour alone and putting everything in one container. 1.0 comes from
    success() holding for 1 s."""
    t = tally(objects)
    if t["fell"] or t["tag_off"]:
        return 0.0
    return PARTIAL_CAP * max(0.0, len(t["correct"]) / len(GARMENTS) - len(t["wrong"]) / 3)


# -- demo ----------------------------------------------------------------------------------------------------

YAW = {"left": 0.0, "right": 0.0}  # the fingers close along y, across the garment's 5 cm width
TRAVEL_Z = TOP + 0.10  # a held garment's lowest point clears the 5 cm rims by about 3.5 cm
MIN_TCP = TOP + 0.0135  # the fingertips 0.5 mm above the table (they reach 13 mm past the TCP)
PRE_OPEN = 0.008  # each finger this far from the garment as the grip comes down
CLOSE_S = 1.5  # slow close: the fingers meet the garment at low speed
GRIP_SETTLE_S = 0.6  # the fingers trail their command: hold the grip before lifting
SQUEEZE = 0.002  # each finger commanded this far inside the garment's surface: a firm, bounded grip
SET_DOWN_S = 0.5  # held on the floor this long before letting go, so the garment is still when released
OPEN_SETTLE_S = 0.4  # the fingers trail their command: let them open fully before rising
REACH_FIX = {"right": (0.003, 0.0025), "left": (0.003, -0.0025)}  # pick targets aim past the arm's sag
SLOT = 0.032  # two garments side by side in a container, their centres this far either side of its middle


def _pick_place(arm: str, spec, drop: tuple[float, float]) -> list[Waypoint]:
    """Top-down pick and place with a gentle grip across the garment's width at its centre: the fingers come
    down opened PRE_OPEN wider than it on each side, close slowly to SQUEEZE inside its surface and hold before
    lifting, then set it down on the container floor and open."""
    centre_z = spec.pos[2] - SPAWN_GAP
    tcp = max(centre_z, MIN_TCP)
    below = tcp - (centre_z - spec.size[2] / 2)  # TCP height above the garment's lowest point
    fx, fy = REACH_FIX[arm]
    px, py = spec.pos[0] + fx, spec.pos[1] + fy
    qx, qy, qz = drop[0], drop[1], FLOOR_TOP + below
    width = spec.size[1]
    ajar = max(FINGER_OPEN, -(width / 2 + PRE_OPEN))
    grip = min(FINGER_CLOSED, -(width / 2 - SQUEEZE))
    yaw = YAW[arm]
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), ajar, "to the garment", yaw),
        Waypoint(1.5, (px, py, tcp), ajar, "descend", yaw),
        Waypoint(CLOSE_S, (px, py, tcp), grip, "close slowly", yaw),
        Waypoint(GRIP_SETTLE_S, (px, py, tcp), grip, "grip settle", yaw),
        Waypoint(1.2, (px, py, TRAVEL_Z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), grip, "carry", yaw),
        Waypoint(1.2, (qx, qy, qz), grip, "lower", yaw),
        Waypoint(SET_DOWN_S, (qx, qy, qz), grip, "set down", yaw),
        Waypoint(0.6, (qx, qy, qz), ajar, "release", yaw),
        Waypoint(OPEN_SETTLE_S, (qx, qy, qz), ajar, "fingers clear", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), ajar, "retreat", yaw),
    ]


def _slot(container: str, side: int) -> tuple[float, float]:
    """Container's left (+1) or right (-1) half."""
    return BOX_X, CONTAINERS[container][0] + side * SLOT


PARK = {"left": (0.14, 0.27, TRAVEL_Z), "right": (0.14, -0.27, TRAVEL_Z)}  # beside the row, over each arm's end


def _park(arm: str, seconds: float = 1.5, label: str = "park") -> Waypoint:
    return Waypoint(seconds, PARK[arm], FINGER_OPEN, label, YAW[arm])


def _span(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """Three rounds. 1: the right arm takes the navy shirt from the middle of the row to the blue basket while
    the left arm takes the white shirt from its end to the white basket. 2: the left arm takes the tagged blouse
    to the mesh bag's left half while the right arm takes the green shirt to the blue basket. 3: the left arm
    reaches across for the white towel (the right arm waits parked), then parks while the right arm takes the
    tagged sweater to the mesh bag's right half. One arm in the middle of the table at a time."""
    l1 = _pick_place("left", WHITE_SHIRT, _slot("white_basket", +1))
    r1 = _pick_place("right", NAVY_SHIRT, _slot("blue_basket", +1))
    l2 = _pick_place("left", WHITE_BLOUSE, _slot("mesh_bag", +1))
    r2 = _pick_place("right", GREEN_SHIRT, _slot("blue_basket", -1))
    l3 = _pick_place("left", WHITE_TOWEL, _slot("white_basket", -1))
    r3 = _pick_place("right", GREEN_SWEATER, _slot("mesh_bag", -1))
    left = [*l1, *l2, *l3, _park("left")]
    right = [*r1, *r2, _park("right", _span(l3) + 1.5, "wait while the left arm works the middle"), *r3,
             _park("right")]
    hold = max(_span(left), _span(right)) + 2.0
    left.append(_park("left", hold - _span(left), "hold still"))
    right.append(_park("right", hold - _span(right), "hold still"))
    return {"left": left, "right": right}
