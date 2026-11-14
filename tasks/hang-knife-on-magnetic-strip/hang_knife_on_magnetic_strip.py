"""Restaurant prep station: put the chef's knife back on the magnetic knife strip on the backsplash. The knife lies
flat on the cutting board; the strip only holds a blade laid flat against it, so the knife has to be lifted by its
handle, turned a quarter turn so the blade's face meets the wall, and pressed flat onto the strip with the handle
clear of it: hung upright with the handle below the strip, or laid along it with the handle beyond its end."""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import trialkit as tk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BASE = TOP + 0.003
WALL_X = 0.52
STRIP_Y0, STRIP_Y1, STRIP_Z, STRIP_H = 0.175, 0.575, 0.26, 0.035  # strip runs along y; face at x = STRIP_FACE
STRIP_FACE = WALL_X - 0.016
BOARD = (0.30, 0.16, 0.02)  # centre x, y and thickness of the cutting board
HANDLE_L, HANDLE_W, HANDLE_T = 0.12, 0.024, 0.018
BLADE_L, BLADE_W, BLADE_T = 0.20, 0.045, 0.002
KNIFE_XY = (0.30, 0.10)  # handle centre; the knife lies along y with the blade toward +y
CATCH_GAP, CATCH_FLAT = 0.004, 0.99  # the magnet catches a blade within 4 mm of the strip, its face within ~8 deg
CATCH_AREA, FULL_AREA = 4e-4, 12e-4  # blade area over the strip (m^2) for the magnet to catch it, and for a full score
STEEL, BLACK, WOOD, TILE, GROUT = (0.80, 0.81, 0.83), (0.10, 0.10, 0.11), (0.62, 0.45, 0.28), (0.92, 0.93, 0.92), \
    (0.70, 0.72, 0.72)

tk.parts("knife", ("box", (0.0, HANDLE_L / 2 + BLADE_L / 2, HANDLE_T / 2 - BLADE_T / 2 - 0.004),
                   (BLADE_W, BLADE_L, BLADE_T), STEEL, True))
tk.parts("backsplash", *[("box", (-0.0105, 0.0, -0.25 + 0.05 * k), (0.001, 1.20, 0.003), GROUT, False)
                         for k in range(1, 10)])
_strip_mid = (STRIP_Y0 + STRIP_Y1) / 2
tk.parts("strip", *[p for y in (0.44, 0.52) for p in (
    ("box", (-0.0095, y - _strip_mid, 0.06), (0.002, 0.035, 0.16), STEEL, False),
    ("box", (-0.0095, y - _strip_mid, -0.075), (0.02, 0.022, 0.11), BLACK, False))])
tk.parts("board", *[("cyl", (0.075, 0.07 + 0.03 * k, BOARD[2] / 2 + 0.004), (0.014, 0.008),
                     (0.93, 0.48, 0.12), False) for k in range(3)])

OBJECTS = [
    Box("counter", pos=(0.34, 0.0, TOP + 0.0015), size=(0.52, 0.80, 0.003), color=(0.78, 0.78, 0.76)),
    Box("backsplash", pos=(WALL_X + 0.01, 0.20, BASE + 0.25), size=(0.02, 1.20, 0.50), color=TILE),
    Box("strip", pos=(WALL_X - 0.008, _strip_mid, BASE + STRIP_Z), size=(0.016, STRIP_Y1 - STRIP_Y0, STRIP_H),
        color=BLACK),
    Box("board", pos=(BOARD[0], BOARD[1], BASE + BOARD[2] / 2), size=(0.24, 0.36, BOARD[2]), color=WOOD),
    Box("towel", pos=(0.44, -0.20, BASE + 0.012), size=(0.14, 0.10, 0.024), color=(0.95, 0.95, 0.93)),
    Cylinder("deli_tub", pos=(0.46, -0.08, BASE + 0.04), radius=0.055, height=0.08, color=(0.90, 0.92, 0.92)),
    Box("knife", pos=(*KNIFE_XY, BASE + BOARD[2] + HANDLE_T / 2), size=(HANDLE_W, HANDLE_L, HANDLE_T), mass=0.2,
        color=BLACK),
]

T = {"n": 0, "held": False, "lifted": False, "dropped": False, "area": 0.0}


@tk.on_reset
def _new_trial(env, rng):
    T.update(n=0, held=False, lifted=False, dropped=False, area=0.0)


def _quat_rot(qx, qy, qz, qw):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def _knife_pose():
    """The knife body's origin (task frame) and rotation, read from the simulator."""
    base = tk.env().unwrapped
    pose = base.scene["knife"].data.root_pose_w.torch[0].cpu().numpy()
    return pose[:3] - base.scene.env_origins[0].cpu().numpy(), _quat_rot(*pose[3:7]), pose[3:7]


def _blade(origin, r):
    """Blade centre, face normal, and its four corners."""
    centre = origin + r @ np.array([0.0, HANDLE_L / 2 + BLADE_L / 2, HANDLE_T / 2 - BLADE_T / 2 - 0.004])
    corners = [centre + r @ np.array([sw * BLADE_W / 2, sl * BLADE_L / 2, 0.0])
               for sw, sl in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return centre, r @ np.array([0.0, 0.0, 1.0]), corners


def _area_on_strip(corners):
    """Area of the blade's outline, seen face-on in the wall's plane, that lies over the strip's face."""
    poly = [(c[1], c[2]) for c in corners]
    z0, z1 = BASE + STRIP_Z - STRIP_H / 2, BASE + STRIP_Z + STRIP_H / 2
    for axis, edge, keep_above in ((0, STRIP_Y0, True), (0, STRIP_Y1, False), (1, z0, True), (1, z1, False)):
        inside = (lambda p: p[axis] >= edge) if keep_above else (lambda p: p[axis] <= edge)
        out = []
        for a, b in zip(poly, poly[1:] + poly[:1]):
            if inside(a):
                out.append(a)
            if inside(a) != inside(b):
                t = (edge - a[axis]) / (b[axis] - a[axis])
                out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        poly = out
        if not poly:
            return 0.0
    return 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1])))


def _in_hand(knife):
    """A closed gripper within 6 cm of the knife's handle."""
    return any(np.linalg.norm(tcp - knife.pos) < 0.06 and opening < 0.5 for tcp, opening in tk.grippers().values())


def _catch(origin, q):
    """The magnet: a fixed joint from the strip to the knife at the knife's pose of the moment."""
    from pxr import Gf, UsdPhysics

    stage, path = tk._PATHS["knife"]
    root = path.rsplit("/", 1)[0]
    strip = np.array([WALL_X - 0.008, (STRIP_Y0 + STRIP_Y1) / 2, BASE + STRIP_Z])
    joint = UsdPhysics.FixedJoint.Define(stage, f"{root}/strip/magnet")
    joint.CreateBody0Rel().SetTargets([f"{root}/strip"])
    joint.CreateBody1Rel().SetTargets([path])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, origin - strip)))
    joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))


def score(objects):
    """0.25 once the knife is lifted 5 cm off the board; 0.5 when the magnet holds it (blade face within 4 mm of the
    strip and within about 8 degrees of the wall, with at least 4 cm^2 of it over the strip's face); 1.0 when it
    hangs still with at least 12 cm^2 of blade over the strip, laid along the strip or hung upright. 0 if the knife
    is never lifted, or is dropped: falling faster than 0.6 m/s (about a 2 cm fall) while neither gripper nor
    strip holds it."""
    knife = objects["knife"]
    T["n"] += 1
    if knife.low[2] > BASE + BOARD[2] + 0.05:
        T["lifted"] = True
    if T["lifted"] and not T["held"] and knife.lin_vel[2] < -0.6 and not _in_hand(knife):
        T["dropped"] = True
    origin, r, q = _knife_pose()
    centre, normal, corners = _blade(origin, r)
    flat = abs(normal[0])
    axis = r @ np.array([0.0, 1.0, 0.0])
    lean = math.degrees(math.atan2(axis[1], -axis[2]))  # blade direction from straight down, + toward +y
    gap = STRIP_FACE - (centre[0] + BLADE_T / 2 * flat)
    area = _area_on_strip(corners)
    if not T["held"] and not T["dropped"] and gap < CATCH_GAP and flat > CATCH_FLAT and area >= CATCH_AREA:
        T["held"] = True
        _catch(origin, q)
    if T["held"]:
        T["area"] = area
    if os.environ.get("KNIFE_DEBUG") and T["n"] % 10 == 0:
        print(f"KNIFE t={T['n'] / 20:.1f} pos={np.round(knife.pos - [0, 0, BASE], 3)} flat={flat:.3f} "
              f"gap={gap * 1000:.1f}mm area={area * 1e4:.1f}cm2 lean={lean:.0f} lifted={T['lifted']} held={T['held']} "
              f"dropped={T['dropped']}", flush=True)
    if T["dropped"] or not T["lifted"]:
        return 0.0
    if T["held"]:
        return 1.0 if T["area"] >= FULL_AREA and knife.resting() else 0.5
    return 0.25


def demo():
    """The left arm grips the handle from above, lifts the knife and turns it a quarter turn so it stands upright,
    blade up and its face toward the wall, presses the blade flat across the strip with the handle below it, lets
    go and backs away. KNIFE_VARIANT scripts the other runs checked in the README ("along" lays the knife along the
    strip)."""
    variant = os.environ.get("KNIFE_VARIANT", "upright")
    kx, ky = KNIFE_XY
    grip = BASE + BOARD[2] + HANDLE_T / 2 + 0.013
    lift = BASE + 0.20
    upright = variant == "upright"  # the demo; "along" lays it along the strip instead
    yaw = -0.12 if upright else math.pi / 2  # -0.12 rad cancels the 7 degree lean the arm leaves at this reach
    roll = tk.Tilt(yaw, 1.1 if variant == "half_roll" else 1.57, 0.0)
    hz = BASE + STRIP_Z + 0.02  # the arm sags about 4 cm at this tilted reach, leaving the blade on the strip
    if upright:
        hz = BASE + STRIP_Z - STRIP_H / 2 - HANDLE_L / 2 - 0.01  # handle below the strip, blade standing across it
    face_x = STRIP_FACE - (HANDLE_T / 2 - 0.004) + 0.002
    hy = ky + {"handle_on_strip": 0.10, "tip": -0.16}.get(variant, 0.0) + (0.20 if upright else 0.0)
    left = [
        Waypoint(2.0, (kx, ky, lift), FINGER_OPEN, "above the handle", math.pi / 2),
        Waypoint(1.5, (kx, ky, grip), FINGER_OPEN, "round the handle", math.pi / 2),
        Waypoint(0.8, (kx, ky, grip), FINGER_CLOSED, "grip", math.pi / 2),
        Waypoint(1.2, (kx, ky, lift), FINGER_CLOSED, "lift", math.pi / 2),
    ]
    if variant == "leave":
        left = []
    elif variant == "drop":
        left += [Waypoint(1.0, (kx, ky, lift), FINGER_OPEN, "let go in the air", math.pi / 2)]
    else:
        left += [
            Waypoint(2.5, (face_x - 0.06, hy, hz), FINGER_CLOSED, "roll it upright in front of the strip", roll),
            Waypoint(1.5, (face_x, hy, hz), FINGER_CLOSED, "press the blade flat on the strip", roll),
            Waypoint(0.8, (face_x, hy, hz), FINGER_OPEN, "let go", roll),
            Waypoint(1.2, (face_x - 0.08, hy, hz), FINGER_OPEN, "back off", roll),
        ]
    left.append(Waypoint(2.0, (0.08, 0.30, TOP + 0.18), FINGER_OPEN, "out of the camera's view"))
    return {"left": left}
