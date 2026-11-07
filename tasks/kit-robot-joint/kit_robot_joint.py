"""On the station table, in front of the left arm: three parts of a small robot joint stand in a row nearer the arm, and
a dark kit tray with three shaped pockets lies further out. The parts, all 4 cm tall: an orange servo motor (6 x 2.5 cm,
its long side pointing away from the arm), a red bracket block (4 x 4 cm) and a blue bearing block (2.5 x 2.5 cm). Each
pocket fits one part with 4 mm to spare on each side and is 8 mm deep; the servo motor's pocket is long side to side,
so the motor has to be turned a quarter turn to go in. The bearing block would also fit either of the other pockets.
The tray is fixed; the parts are loose.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x forward from the arms, y to the left, z up (m); the arm bases stand at x 0, y +-0.305.
PART_H, PART_KG = 0.04, 0.06
PARTS = {  # name: (footprint along x, along y at the start), start x, y, colour
    "servo_motor": ((0.060, 0.025), (0.15, 0.26), (0.95, 0.50, 0.10)),
    "bracket_block": ((0.040, 0.040), (0.15, 0.17), (0.85, 0.15, 0.15)),
    "bearing_block": ((0.025, 0.025), (0.15, 0.08), (0.15, 0.35, 0.90)),
}
CLEARANCE, WALL, WALL_H = 0.004, 0.004, 0.008
TRAY_XY, TRAY_SIZE, TRAY_T = (0.28, 0.17), (0.10, 0.27), 0.005
FLOOR_Z = TOP + TRAY_T  # the pockets' floor: the tray's top
POCKETS = {  # part: (pocket centre, turned a quarter turn?)
    "servo_motor": ((0.28, 0.26), True),
    "bracket_block": ((0.28, 0.17), False),
    "bearing_block": ((0.28, 0.08), False),
}
TRAY_DARK = (0.18, 0.22, 0.30)


def pocket_inner(part):
    """The pocket's inner size along x and y (m)."""
    (fx, fy), _, _ = PARTS[part]
    _, turned = POCKETS[part]
    if turned:
        fx, fy = fy, fx
    return fx + 2 * CLEARANCE, fy + 2 * CLEARANCE


def _pocket_walls(part):
    (cx, cy), _ = POCKETS[part]
    ix, iy = pocket_inner(part)
    z = FLOOR_Z + WALL_H / 2
    ox = ix / 2 + WALL / 2
    oy = iy / 2 + WALL / 2
    return [
        Box(f"{part}_pocket_n", pos=(cx, cy + oy, z), size=(ix + 2 * WALL, WALL, WALL_H), color=TRAY_DARK),
        Box(f"{part}_pocket_s", pos=(cx, cy - oy, z), size=(ix + 2 * WALL, WALL, WALL_H), color=TRAY_DARK),
        Box(f"{part}_pocket_e", pos=(cx + ox, cy, z), size=(WALL, iy, WALL_H), color=TRAY_DARK),
        Box(f"{part}_pocket_w", pos=(cx - ox, cy, z), size=(WALL, iy, WALL_H), color=TRAY_DARK),
    ]


OBJECTS = [
    Box("kit_tray", pos=(*TRAY_XY, TOP + TRAY_T / 2), size=(*TRAY_SIZE, TRAY_T), color=TRAY_DARK),
    *[wall for part in PARTS for wall in _pocket_walls(part)],
    *[Box(part, pos=(*xy, TOP + PART_H / 2 + 0.0005), size=(*foot, PART_H), mass=PART_KG, color=colour)
      for part, (foot, xy, colour) in PARTS.items()],
]

# A part is in its pocket when it stands upright (its 4 cm height within 10 degrees of vertical, either way up: a plain
# block looks the same upside down) on the pocket floor (lowest corner within 4 mm of it) with all four bottom corners
# inside the pocket's inner edges. The bracket is a 4 cm cube, so it is upright on any face. The motor on its end
# would fit its pocket but is the wrong way round, so it doesn't count. Corners come from each part's centre and
# orientation, not from Arena's low/high, which grow when a part turns.
UPRIGHT_AXES = {"servo_motor": (2,), "bracket_block": (0, 1, 2), "bearing_block": (2,)}  # local axes that may be vertical
MAX_TILT_DEG = 10.0
CONTACT_SLACK = 0.0005  # a part pressed against a pocket wall can read this far past its inner face
MAX_GAP = 0.004


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _corners(part, state):
    (fx, fy), _, _ = PARTS[part]
    r = _rot(state.quat_xyzw)
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                local = (sx * fx / 2, sy * fy / 2, sz * PART_H / 2)
                out.append(tuple(float(state.pos[i]) + sum(r[i][j] * local[j] for j in range(3)) for i in range(3)))
    return out


def _tilt_deg(part, state) -> float:
    """The angle from vertical to the nearest of the part's axes that may stand vertical, either way up."""
    row = _rot(state.quat_xyzw)[2]
    return math.degrees(math.acos(min(1.0, max(abs(row[j]) for j in UPRIGHT_AXES[part]))))


def _in_pocket(part, state) -> bool:
    tilt = _tilt_deg(part, state)
    corners = _corners(part, state)
    bottom = sorted(corners, key=lambda c: c[2])[:4]
    if tilt > MAX_TILT_DEG or abs(bottom[0][2] - FLOOR_Z) > MAX_GAP:
        return False
    (cx, cy), _ = POCKETS[part]
    ix, iy = pocket_inner(part)
    return all(abs(c[0] - cx) <= ix / 2 + CONTACT_SLACK and abs(c[1] - cy) <= iy / 2 + CONTACT_SLACK for c in bottom)


def success(objects):
    """Every part in its own pocket, all still; must hold for 1 s."""
    return all(_in_pocket(part, objects[part]) and objects[part].resting() for part in PARTS)


def distance(objects):
    """The farthest part's centre from its pocket's centre (m)."""
    return max(math.dist(objects[part].pos[:2], POCKETS[part][0]) for part in PARTS)


def score(objects):
    """Partial credit (success scores 1.0): one third for each part in its own pocket. A part in another part's pocket
    earns nothing, and usually costs the part that belonged there too."""
    return sum(_in_pocket(part, objects[part]) for part in PARTS) / len(PARTS)


GRIP_BELOW_TOP = 0.012  # finger pads end 15 mm above a part's bottom: 7 mm over the pocket walls once it is seated
_CLEAR_Z = TOP + PART_H + 0.015 + (PART_H - GRIP_BELOW_TOP)  # a carried part's bottom 1.5 cm over standing parts
ORDER = ("bracket_block", "bearing_block", "servo_motor")  # the motor last: its pocket is next to the bracket's
# WHY: the left arm holds each part off where it aims, by a different amount over each pocket. Traced on the official
# stack (2026-11-03), the part gripped just above the pocket floor: the bracket 3.8 mm short in x and 14.3 mm short in
# y; the motor 3.9 mm short in x, 2.1 mm past in y and turned 4.4 degrees past its quarter turn (both twist about 4.5
# degrees in the grip as the fingers close); the bearing block on target. With 4 mm to spare, the first two came down
# on a pocket wall. Aiming past by that much centred them while held, but setting them down on the floor then pushed
# them 3 to 6 mm sideways against a wall (still gripped, the arm kept pressing as it sank), so they are let go above
# the floor instead (RELEASE_ABOVE). Measured 5 to 14 mm above the floor in three traces, the error repeats to 0.3 mm:
# the bracket 3.8 mm short in x and 14.5 mm short in y, the motor 4.2 mm short in x and 1.8 mm past in y. The turns
# correct the twist each part gets in the grip. Aim: x, y (m) and turn (deg).
PLACE_AIM = {"servo_motor": (0.0042, -0.0018, -2.6), "bracket_block": (0.0038, 0.0145, -0.8),
             "bearing_block": (0.0, 0.0, -2.7)}
# WHY: let go with the part's bottom 4 mm above the pocket floor: already 4 mm inside the 8 mm walls, it drops straight
# in, and the arm never presses it onto the floor (which pushed it sideways).
RELEASE_ABOVE = 0.004


def _kit(part):
    """Pick ``part`` where it stands, carry it over its pocket (turning a quarter turn if the pocket is turned), lower
    it straight down into the pocket, let go 4 mm above the floor and go straight back up."""
    _, (sx, sy), _ = PARTS[part]
    (px, py), turned = POCKETS[part]
    ax, ay, a_deg = PLACE_AIM[part]
    px, py = px + ax, py + ay
    grip = PART_H - GRIP_BELOW_TOP
    pick_z = TOP + grip
    place_z = FLOOR_Z + grip + RELEASE_ABOVE
    yaw = (math.pi / 2 if turned else 0.0) + math.radians(a_deg)
    return [
        Waypoint(1.5, (sx, sy, _CLEAR_Z), FINGER_OPEN, f"{part}: above"),
        Waypoint(1.5, (sx, sy, pick_z), FINGER_OPEN, f"{part}: down"),
        Waypoint(0.8, (sx, sy, pick_z), FINGER_CLOSED, f"{part}: grip"),
        Waypoint(1.5, (sx, sy, _CLEAR_Z), FINGER_CLOSED, f"{part}: lift"),
        Waypoint(2.5, (px, py, _CLEAR_Z), FINGER_CLOSED, f"{part}: carry", yaw),
        Waypoint(0.5, (px, py, _CLEAR_Z), FINGER_CLOSED, f"{part}: settle", yaw),
        Waypoint(1.5, (px, py, place_z), FINGER_CLOSED, f"{part}: lower", yaw),
        Waypoint(0.8, (px, py, place_z), FINGER_OPEN, f"{part}: let go", yaw),
        Waypoint(1.0, (px, py, _CLEAR_Z), FINGER_OPEN, f"{part}: up", yaw),
        Waypoint(1.0, (px, py, _CLEAR_Z), FINGER_OPEN, f"{part}: straighten", 0.0),
    ]


def demo():
    """The left arm alone kits the bracket block, then the bearing block, then the servo motor (turned a quarter turn on
    the way, so its long side lies along its pocket)."""
    return {"left": [w for part in ORDER for w in _kit(part)]}
