"""On a felt desk mat: standing wooden domino tiles (12 cm tall, 6 cm apart). A trunk of two tiles starts near the
robot and ends at a wide splitter tile, which starts two parallel branches running away from the robot. The right
branch ends at a red target tile, the left branch at a glass flower vase. Both the target and the vase stand beyond
the arms' reach.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
H, THICK, WIDE, PITCH = 0.12, 0.012, 0.05, 0.06
WOOD, RED = (0.82, 0.68, 0.46), (0.80, 0.12, 0.12)
SPLIT_X, BRANCH_X0, BRANCH_Y0 = 0.26, 0.32, 0.07
DOWN_DEG = 25  # a fallen tile leans on the next at 30 to 50 deg; standing tiles are upright


def _tile(name, x, y, yaw, width=WIDE, color=WOOD):
    return Box(name, pos=(float(x), float(y), TOP + 0.003 + H / 2), size=(THICK, width, H), mass=0.05, color=color,
               yaw=float(yaw))


def _branch(side, n=6):
    """Tile centres and yaw along one straight branch, from just past the splitter to beyond the arms' reach."""
    return [((BRANCH_X0 + i * PITCH, side * BRANCH_Y0), 0.0) for i in range(n)]


TRUNK = [_tile("trunk_1", 0.14, 0.0, 0.0), _tile("trunk_2", 0.20, 0.0, 0.0), _tile("splitter", SPLIT_X, 0.0, 0.0, 0.19)]
RIGHT = _branch(-1)  # toward the target
LEFT = _branch(1)  # toward the vase
RIGHT_TILES = [_tile(f"right_{i + 1}", *xy, yaw, color=RED if i == len(RIGHT) - 1 else WOOD) for i, (xy, yaw) in enumerate(RIGHT)]
LEFT_TILES = [_tile(f"left_{i + 1}", *xy, yaw) for i, (xy, yaw) in enumerate(LEFT)]
TARGET = RIGHT_TILES[-1].name
(vx, vy), _ = LEFT[-1]
VASE = Cylinder("vase", pos=(float(vx + 0.07), float(vy), TOP + 0.003 + 0.11), radius=0.022, height=0.22, mass=0.05,
                color=(0.55, 0.75, 0.85))
DRESSING = [
    Box("desk_mat", pos=(0.41, 0.0, TOP + 0.0015), size=(0.68, 0.46, 0.003), color=(0.15, 0.35, 0.25)),
    Asset("desk_cube", "rubiks_cube_hot3d_robolab", xy=(0.14, 0.34), extent=0.057),
    Asset("desk_marker", "dry_erase_marker_ycb_robolab", xy=(0.12, -0.34), extent=0.12),
]
TILES = [*TRUNK, *RIGHT_TILES, *LEFT_TILES]
OBJECTS = [*DRESSING, *TILES, VASE]
START = {o.name: np.array(o.pos[:2]) for o in (*TILES, VASE)}
_vase_hit = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _track(objects):
    """Remember, for the whole trial, whether the vase was knocked."""
    global _vase_hit
    if all(np.linalg.norm(objects[n].pos[:2] - xy) < 0.005 for n, xy in START.items()):
        _vase_hit = False
    _vase_hit = _vase_hit or _tilt_deg(objects["vase"]) > 20


def score(objects):
    """0 if the vase was ever knocked (tilted past 20 deg). Red target down (tilted past 25 deg): 1.0. Only part of
    the red branch down: 0.5. Otherwise 0. There is no success(), so the trial runs on and a later fall of the vase
    still counts."""
    _track(objects)
    if _vase_hit:
        return 0.0
    if _tilt_deg(objects[TARGET]) > DOWN_DEG:
        return 1.0
    return 0.5 if any(_tilt_deg(objects[t.name]) > DOWN_DEG for t in RIGHT_TILES[:-1]) else 0.0


def demo():
    """Right arm, fingers closed: presses the top edge of the second tile on the red branch forward along the branch,
    so only that branch falls, then moves clear so the top camera sees the red tile fall."""
    (x, y), yaw = RIGHT[1]
    u = np.array([np.cos(yaw), np.sin(yaw)])
    z = TOP + 0.003 + H - 0.012
    start, end = np.array([x, y]) - 0.022 * u, np.array([x, y]) + 0.03 * u
    steps = [
        (1.5, (*start, TOP + 0.14), "above the tile"),
        (1.5, (*start, z), "down behind its top edge"),
        (1.5, (*end, z), "press it forward"),
        (1.0, (*end, TOP + 0.14), "up"),
        (2.0, (0.20, -0.28, TOP + 0.20), "move clear of the camera's view"),
    ]
    return {"right": [Waypoint(d, p, FINGER_CLOSED, label) for d, p, label in steps]}
