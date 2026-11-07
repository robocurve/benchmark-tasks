"""On the station table, right of the midline: a fixed blue driveshaft tube yoke (Spicer 1310 size) lies with its tube
pointing away from the robot and its two ears toward it, 38 cm out from the bases. The ears stand upright 59.8 mm
apart, each 11 mm thick, with a 27 mm cross hole (the bearing cap bore, caps removed) on one line across the table,
61 mm above it. 11 cm closer to the robot, an orange universal-joint cross (four 15 mm steel journals, 73 mm tip to
tip, 200 g) stands on one journal in a black holder, with the pair that goes into the yoke lying flat.

That pair is 73 mm tip to tip, longer than the 59.8 mm between the ears, so the cross cannot come down level between
them. One journal goes in at an angle and is pushed deep through its ear's hole, the cross is swung level so the other
journal lines up with the other hole, and it is slid back until both journals are in their holes by the same depth.
"""

import math
from pathlib import Path

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HERE = Path(__file__).resolve().parent


def _register():
    """Add the yoke and the cross (USD files next to this one, written by make_meshes.py) to Arena's asset registry,
    so ``Asset`` can name them. Outside the simulator (tests, checks) there is nothing to register."""
    try:
        from isaaclab_arena.assets.registries import AssetRegistry
    except ImportError:
        return
    reg = AssetRegistry()
    for name in ("ujoint_yoke", "ujoint_cross"):
        if not reg.is_registered(name, ensure_loaded=False):
            reg.register(type(name, (), {"name": name, "tags": ["object"], "usd_path": str(HERE / f"{name}.usda")}), name)


_register()

# Geometry (make_meshes.py). The yoke's frame origin is on the table under the middle of its cross holes' axis, which
# runs along world y. The cross's frame origin is on one journal's tip, its centre HALF_SPAN above.
REST = 0.001                          # assets spawn this far above the table; the fixed yoke stays there
BORE = np.array([0.38, -0.16])        # middle of the cross holes' axis, world x and y
BORE_Z = TOP + REST + 0.060           # the cross holes' axis, world height
YOKE_C, YOKE_EXT = (0.049, 0.0), 0.142  # bounding-box centre and widest side of the yoke, in its own frame
BORE_R = 0.02697 / 2                  # cross hole (bearing cap bore) radius
GAP = 0.0598                          # between the ears' inner faces
EAR_T = 0.011
JOURNAL_R = 0.0075
HALF_SPAN = 0.0365                    # cross centre to a journal's tip
HUB = 0.013
CROSS_EXT, CROSS_MASS = 2 * HALF_SPAN, 0.2
# The cross starts with the journals that go into the yoke turned START_YAW from the holes' axis (the angle the demo
# brings it in at)
START_YAW = math.radians(76.0)
HOLD = np.array([0.27, -0.16])        # holder centre, world: the standing journal's axis
HOLE_A, HOLE_N = 0.016, 0.019         # holder hole along the flat pair and across it
WALL, HOLDER_H = 0.006, 0.016
BLACK = (0.08, 0.08, 0.09)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def _holder():
    """Four walls round the standing journal: 0.5 mm clear along the flat pair (the pad plane, where a closing grip
    cannot square a lean) and 2 mm across it."""
    r = _rz(START_YAW)
    walls = [((HOLE_N + 2 * WALL, WALL), (0.0, HOLE_A / 2 + WALL / 2)), ((HOLE_N + 2 * WALL, WALL), (0.0, -HOLE_A / 2 - WALL / 2)),
             ((WALL, HOLE_A), (HOLE_N / 2 + WALL / 2, 0.0)), ((WALL, HOLE_A), (-HOLE_N / 2 - WALL / 2, 0.0))]
    out = []
    for k, ((sx, sy), off) in enumerate(walls):
        x, y = HOLD + r @ np.array(off)
        out.append(Box(f"holder_{k}", size=(sx, sy, HOLDER_H), pos=(float(x), float(y), TOP + HOLDER_H / 2),
                       color=BLACK, yaw=START_YAW))
    return out


OBJECTS = [
    Asset("yoke", "ujoint_yoke", xy=tuple(BORE + np.array(YOKE_C)), extent=YOKE_EXT),
    Asset("cross", "ujoint_cross", xy=tuple(HOLD), extent=CROSS_EXT, mass=CROSS_MASS, yaw=START_YAW),
] + _holder()

# Tolerances
IN_DEPTH = 0.001          # a journal is in its hole once its tip is this far past the ear's inner face
IN_RADIAL = 0.008         # ... with its tip's centre within this of the hole's axis (a journal in the hole: 6 mm)
CENTRED_DEPTH = 0.0035    # both journals this deep: the cross within 3.1 mm of centred (6.6 mm each when centred)
SQUARE_DEG = 5.0          # the flat pair's axis from the holes' axis: a journal in line with its hole


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def journals(o):
    """For the cross's flat pair, per ear (+1 the left ear at +y, -1 the right): (tip's depth past that ear's inner
    face (m), tip centre's distance from the holes' axis (m)); the pair's angle from the holes' axis (deg); and whether
    the cross's centre is between the ears' inner faces. ``o.pos`` is the cross's centre (its bounding-box centre)."""
    r = _rot(o.quat_xyzw)
    # WHY: the two pairs are the same, so either can be the one that goes in: score the pair nearer the holes' axis
    a = r[:, 1] if abs(r[1, 1]) >= abs(r[1, 2]) else r[:, 2]
    if a[1] < 0:
        a = -a
    c = np.asarray(o.pos, dtype=float)
    out = {}
    for s in (1, -1):
        tip = c + s * HALF_SPAN * a
        depth = s * (tip[1] - BORE[1]) - GAP / 2
        radial = math.hypot(tip[0] - BORE[0], tip[2] - BORE_Z)
        out[s] = (depth, radial)
    angle = math.degrees(math.acos(float(np.clip(a[1], -1.0, 1.0))))
    between = abs(c[1] - BORE[1]) < GAP / 2
    return out, angle, between


def _in(d, between):
    """A journal in its hole from the inside: the cross between the ears, the tip past the ear's inner face and in line
    with the hole. (Outboard of an ear, a journal can reach into that ear's hole from outside.)"""
    depth, radial = d
    return between and depth >= IN_DEPTH and radial <= IN_RADIAL


def installed(objects):
    """Both journals in their holes, each at least CENTRED_DEPTH deep, the pair in line with the holes."""
    j, angle, between = journals(objects["cross"])
    return all(_in(j[s], between) and j[s][0] >= CENTRED_DEPTH for s in (1, -1)) and angle <= SQUARE_DEG


def success(objects):
    """Installed and still. Holds for 1 s."""
    return installed(objects) and objects["cross"].resting()


def distance(objects):
    j, _, _ = journals(objects["cross"])
    return float(sum(max(0.0, CENTRED_DEPTH - j[s][0]) + max(0.0, j[s][1] - IN_RADIAL) for s in (1, -1)))


def score(objects):
    """Partial credit (success, held for 1 s, scores 1.0): 0.75 installed; 0.5 both journals in their holes (off centre
    or turned); 0.25 one journal in its hole; else 0. A cross pushed deep into one ear with its other journal out of its
    hole scores 0.25."""
    if installed(objects):
        return 0.75
    j, _, between = journals(objects["cross"])
    n = sum(_in(j[s], between) for s in (1, -1))
    return {2: 0.5, 1: 0.25, 0: 0.0}[n]


# Demo ----------------------------------------------------------------------------------------------------------------
# WHY: each sim finger is capped at 50 N. Aimed 3 mm apart the pads press 6 mm into the 15 mm journal at about 30 N,
# under the cap, so the grip centres the journal.
FINGER = -0.0015
GRIP_ABOVE = HUB + 0.015   # TCP above the cross's centre: pad tips 2 mm over the hub
YAW0 = -math.pi / 2        # the grip yaw with the flat pair along the holes' axis (fingers close along world x)
# The in-plane path, from a plan of the cross's outline against the ears in the plane of the holes' axis
# (x, y of the cross's centre from the middle of the holes (m), the pair's angle from the holes' axis (deg)): it comes
# down clear of the ears at 76 deg, swings in with its right journal entering the right hole, ends level with the cross
# 11 mm right of centre (that journal 17.6 mm past its ear's inner face), then slides back to centred. Every point along
# it keeps 1.9 mm or more between the cross and the ears, their tangs and the crotch.
PATH = [(-0.030, -0.011, 76.0), (-0.014, -0.008, 28.0), (0.0, -0.011, 0.0), (0.0, 0.0, 0.0)]
# WHY: held still between the ears, the cross sat 4.4 to 6 mm low, 0.5 to 2.6 mm toward the robot, 2 mm to the right and
# turned 2 to 3 deg past the aim (official run, rows still for 0.5 s): the arm sags under the cross and the grip turns it
# a little. Low, its journal rode on the bottom of the right hole and dragged; turned, the left journal met the left ear
# beside its hole and the cross stopped 6.4 mm short of centred. Aim off by that much.
AIM = np.array([0.002, 0.0015, 0.005])
AIM_YAW = math.radians(-2.7)


def demo(delay=0.0):
    """The right arm grips the top journal, lifts the cross out of its holder, carries it over the yoke and lowers it
    between the ears turned 76 degrees. It then swings it level while pushing its right journal through the right hole,
    slides it back to centred and lets go."""
    hi = TOP + 0.19
    yaw_s = YAW0 + START_YAW
    hx, hy = HOLD
    gz = TOP + REST + HALF_SPAN + GRIP_ABOVE
    w = [Waypoint(2.0 + delay, (hx, hy, hi), FINGER_OPEN, "over the cross", yaw_s),
         Waypoint(1.5, (hx, hy, hi), FINGER_OPEN, "settle", yaw_s),
         Waypoint(2.0, (hx, hy, gz), FINGER_OPEN, "down", yaw_s),
         Waypoint(0.8, (hx, hy, gz), FINGER, "grip", yaw_s),
         Waypoint(2.0, (hx, hy, gz + 0.03), FINGER, "out of the holder", yaw_s),
         Waypoint(1.0, (hx, hy, hi), FINGER, "lift", yaw_s)]
    cz = BORE_Z + GRIP_ABOVE

    def at(x, y, z, a):
        """TCP target for the cross's centre at (x, y) from the holes' middle, height z, the pair at a deg."""
        return (BORE[0] + x + AIM[0], BORE[1] + y + AIM[1], z + AIM[2]), YAW0 + math.radians(a) + AIM_YAW

    x0, y0, a0 = PATH[0]
    for dur, z, label in ((2.0, hi, "over the yoke"), (1.5, hi, "settle"), (3.0, cz, "down between the ears"),
                          (1.5, cz, "settle")):
        p, yaw = at(x0, y0, z, a0)
        w.append(Waypoint(dur, p, FINGER, label, yaw))
    for (x, y, a), label in zip(PATH[1:], ("swing in", "level, journal deep", "slide back to centre")):
        p, yaw = at(x, y, cz, a)
        w += [Waypoint(4.0, p, FINGER, label, yaw), Waypoint(1.0, p, FINGER, "settle", yaw)]
    p, yaw = at(0.0, 0.0, cz, 0.0)
    w += [Waypoint(0.8, p, FINGER_OPEN, "let go", yaw),
          Waypoint(1.5, (p[0], p[1], TOP + 0.15), FINGER_OPEN, "up", yaw)]
    return {"right": w}
