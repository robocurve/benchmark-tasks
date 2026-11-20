"""Scoop a urine clump out of a cat's litter box with a slotted scoop, sift the loose litter back, and tip the clump
into the waste bin.

On the station table stands a Van Ness small cat litter pan (14.4 x 10.75 x 4 in) filled 2 in deep with Happy & Polly
Clumping Natural Tofu Cat Litter (strips 2 mm across, 1-2 cm long). Near its front lies one urine clump, about golf-ball size (Litter-Robot:
"about 1 to 2 inches across"), its top just showing above the litter. A Van Ness plastic litter scoop (11 x 4.88 x
2.25 in, five 1/2 in slots in its floor) rests in the pan, its basket on the litter and its handle over the pan's
right-hand rim; an mDesign 1.25 gal round wastebasket stands to the left of the pan.

What makes it hard: the clump sits in loose litter that gives way and flows. The scoop's lip has to go into the litter
beside the clump and under it, deeper than the clump reaches, then come up with the clump on its floor; lifted, the
loose litter has to run out through the slots (shaken, over the pan, so it falls back in) while the clump, rounded and
heavier than the litter it sat in, stays on the tilted, moving floor; then the scoop is turned over the bin to tip
the clump in. The clump may not be touched with the fingers.

Simulation: the litter is about 55,000 rigid spheres 3.9-4.9 mm across (soilkit), each the volume of one tofu
strip; the clump and the scoop are rigid bodies (partkit); the pan's walls and the bin's are fixed boxes.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import partkit  # noqa: E402
import soilkit  # noqa: E402
from partkit import part  # noqa: E402
from scenekit import Shape, on_reset, on_step, robot_links  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Scoop the clump out of the litter box with the scoop, sift the loose litter back into the box, and "
               "tip the clump into the bin.")

MM = 0.001
INCH = 25.4 * MM
TOP = TABLE_TOP[SCENE]
PREP = bool(os.environ.get("PREP"))  # preparing the litter's start layout: a fixed block holds the clump's place


def _q_axis(axis, angle):
    """(x, y, z, w) quaternion for a rotation of ``angle`` about ``axis``."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    s = np.sin(angle / 2)
    return (float(a[0] * s), float(a[1] * s), float(a[2] * s), float(np.cos(angle / 2)))


def _q_mul(a, b):
    """Product of two (x, y, z, w) quaternions."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# -- the litter pan: Van Ness small cat pan, 14.4 x 10.75 x 4 in (PetValu, Mudbay). Its walls taper; here they are
# straight, at the inside size the tapered pan has halfway up the litter (34.0 x 24.7 cm), its long side across the
# table (along y). Fixed (it is heavy with litter and stands on a rubber mat in real trials).
PAN_XY = (0.300, 0.040)
PX, PY = PAN_XY
PAN_L, PAN_W, PAN_H, PAN_T = 247 * MM, 340 * MM, 4.0 * INCH, 3 * MM  # inside sizes along x and y
PAN_BLUE = (0.25, 0.45, 0.75)
FLOOR_Z = TOP + PAN_T  # the pan's inside floor
_ox, _oy = PAN_L / 2 + PAN_T / 2, PAN_W / 2 + PAN_T / 2
PAN = [
    Shape("pan_floor", pos=(PX, PY, FLOOR_Z - 6 * MM), size=(PAN_L + 2 * PAN_T, PAN_W + 2 * PAN_T, 12 * MM),
          color=PAN_BLUE),  # 3 mm above the table, thicker below it so nothing falls through
    Shape("pan_wall_far", pos=(PX + _ox, PY, TOP + PAN_H / 2), size=(PAN_T, PAN_W + 2 * PAN_T, PAN_H), color=PAN_BLUE),
    Shape("pan_wall_near", pos=(PX - _ox, PY, TOP + PAN_H / 2), size=(PAN_T, PAN_W + 2 * PAN_T, PAN_H), color=PAN_BLUE),
    Shape("pan_wall_left", pos=(PX, PY + _oy, TOP + PAN_H / 2), size=(PAN_L, PAN_T, PAN_H), color=PAN_BLUE),
    Shape("pan_wall_right", pos=(PX, PY - _oy, TOP + PAN_H / 2), size=(PAN_L, PAN_T, PAN_H), color=PAN_BLUE),
]
RIM = TOP + PAN_H

# -- the litter: tofu clumping litter, strips 2 mm across (Cature, Michu, Happy & Polly: "2mm diameter") and 1-2 cm
# long (Fortress CL102: "cut into a length of 1-2cm"), 2 in deep (Chewy: "2-3 inches"). Each strip (31-63 mm^3) is a
# sphere of the same volume, 3.9-4.9 mm across. Bulk
# density of tofu litter about 0.42 g/cm^3 (Weelitter: 6 L, 2.5 kg); with the spheres packed about 60 %, a grain is
# 700 kg/m^3.
DEPTH = 2.0 * INCH
LEVEL = FLOOR_Z + DEPTH  # the litter's levelled surface
R_MIN, R_MAX = 1.95 * MM, 2.45 * MM
_FILL = 1.45 * DEPTH  # the loose lattice it settles from (work/scoop-litter-clump/prepare_litter.sh)
LITTER = soilkit.Soil("litter", pos=(PX, PY, FLOOR_Z + _FILL / 2), size=(PAN_L - 1 * MM, PAN_W - 1 * MM, _FILL),
                      r_min=R_MIN, r_max=R_MAX, density=700.0, friction=(0.6, 0.5), angular_damping=4.0,
                      colors=((0.86, 0.80, 0.66), (0.80, 0.73, 0.58), (0.90, 0.85, 0.73), (0.75, 0.68, 0.54)),
                      start_file=os.environ.get("LITTER_START", "assets/litter_start.npz"))

# -- the clump: a puck 44 mm across and 34 mm tall (golf-ball size), 35 g: about 30 cm^3 of litter (12 g at 0.42
# g/cm^3) holding one urination (about 23 ml: 18-28 ml/kg/day for a 4 kg cat, three to four times a day). Its top 6 mm
# above the litter, its bottom 23 mm above the pan's floor (it settles about 5 mm once loose).
CLUMP_R, CLUMP_H, CLUMP_MASS = 22 * MM, 34 * MM, 0.035
CLUMP_XY = (0.335, 0.100)  # towards the pan's far wall (the hands working the scoop stay clear of the near wall),
# and 110 mm from the left wall, so the scoop's lip can go well past it
CLUMP_Z = LEVEL + 6 * MM - CLUMP_H / 2  # its centre
CLUMP_COLOR = (0.62, 0.52, 0.30)
if PREP:
    CLUMP = Shape("clump", pos=(CLUMP_XY[0], CLUMP_XY[1], CLUMP_Z), size=(2 * CLUMP_R, 2 * CLUMP_R, CLUMP_H),
                  color=CLUMP_COLOR)
else:
    CLUMP = part("clump", pos=(CLUMP_XY[0], CLUMP_XY[1], CLUMP_Z),
                 boxes=[((0.0, 0.0, 0.0), (30 * MM, 30 * MM, 30 * MM), CLUMP_COLOR)],  # inside the puck (partkit needs one)
                 cylinders=[((0.0, 0.0, 0.0), CLUMP_R, CLUMP_H, CLUMP_COLOR, "Z")], mass=CLUMP_MASS,
                 color=CLUMP_COLOR, friction=0.9, combine="max")  # a wet clump sticks to the plastic scoop

# -- the scoop: Van Ness plastic litter scoop, 11.00 x 4.88 x 2.25 in (Aquarium Depot, "regular"); slots in its
# floor 1/2 in wide and about 4 in long (Chewy answers). Part frame: x forward along the scoop, origin at the middle of
# the basket's floor, on the floor's underside plane + 1.25 mm. The basket is 5 in long (Chewy answers: "scoop
# length is 5 inches"; the 4 in slots and a solid band at each end) with 30 mm walls on the sides and back, open
# at the front; the handle rises from the back wall's top to the scoop's full 2.25 in at its end. 70 g of polypropylene (not published: a 2.5 mm shell of this size).
SC_L, SC_W, SC_D = 11.00 * INCH, 4.88 * INCH, 2.25 * INCH
BASKET_L, WALL_H, SHELL = 5.0 * INCH, 30 * MM, 2.5 * MM
SLOT_W, SLOT_N, SLOT_L = 0.5 * INCH, 5, 4.0 * INCH
RIB_W = (SC_W - SLOT_N * SLOT_W) / (SLOT_N + 1)
BAND_L = (BASKET_L - SLOT_L) / 2
HANDLE_W, HANDLE_T = 22 * MM, 12 * MM
SCOOP_MASS = 0.070
SCOOP_GREEN = (0.30, 0.65, 0.45)
_hx0, _hx1 = -BASKET_L / 2, BASKET_L / 2 - SC_L  # the handle runs from the back wall back to the scoop's end
_ztop0, _ztop1 = WALL_H - SHELL / 2, SC_D - SHELL / 2  # its top edge there (part frame)
HANDLE_RISE = float(np.arctan2(_ztop1 - _ztop0, _hx0 - _hx1))


def handle_centre(x):
    """The handle's centre line at ``x`` (part frame)."""
    return np.array([x, 0.0, _ztop0 + (_hx0 - x) * np.tan(HANDLE_RISE) - HANDLE_T / 2])


def _scoop_boxes():
    out = []
    for k in range(SLOT_N + 1):  # the ribs between the slots
        y = -SC_W / 2 + RIB_W / 2 + k * (RIB_W + SLOT_W)
        out.append(((0.0, y, 0.0), (SLOT_L, RIB_W, SHELL), SCOOP_GREEN))
    for side in (-1, 1):  # the solid bands at the front lip and at the back
        out.append((((BASKET_L - BAND_L) / 2 * side, 0.0, 0.0), (BAND_L, SC_W, SHELL), SCOOP_GREEN))
    zc = WALL_H / 2 - SHELL / 2
    for side in (-1, 1):
        out.append(((0.0, side * (SC_W / 2 - SHELL / 2), zc), (BASKET_L, SHELL, WALL_H), SCOOP_GREEN))
    out.append(((-BASKET_L / 2 + SHELL / 2, 0.0, zc), (SHELL, SC_W, WALL_H), SCOOP_GREEN))
    mid = handle_centre((_hx0 + _hx1) / 2)
    length = (_hx0 - _hx1) / np.cos(HANDLE_RISE)
    q = _q_axis((0, 1, 0), HANDLE_RISE)
    out.append((tuple(float(v) for v in mid), (float(length), HANDLE_W, HANDLE_T), SCOOP_GREEN,
                (q[3], q[0], q[1], q[2])))
    return out


SCOOP_BOXES = _scoop_boxes()
GRIP_X = -0.195  # where the right hand holds the handle (part frame): near its end, outside the pan's rim
RIM_EDGE_Y = PY - PAN_W / 2 - PAN_T  # the pan's right-hand rim (outer edge), which the handle rests on
SCOOP_X = float(os.environ.get("SCOOP_X", "0.300"))  # the scoop's middle across the pan at the start
SCOOP_Y = -0.030  # the basket's middle at the start (it points left, +y, the handle over the right-hand rim)


def _rot_y(a) -> np.ndarray:
    return np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])


def _rot_x(a) -> np.ndarray:
    return np.array([[1.0, 0.0, 0.0], [0.0, np.cos(a), -np.sin(a)], [0.0, np.sin(a), np.cos(a)]])


YAW = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # the scoop's x along +y


def scoop_rot(pitch=0.0, roll=0.0) -> np.ndarray:
    """The scoop's rotation: pointing +y, ``pitch`` nose-down, ``roll`` about its length (rad)."""
    return YAW @ _rot_y(pitch) @ _rot_x(roll)


def _handle_bottom_z(origin, pitch, y):
    """The handle's underside at world ``y`` (scoop pointing +y, pitched)."""
    xs = np.linspace(_hx1, _hx0, 400)
    pts = np.array([handle_centre(x) - (0, 0, HANDLE_T / 2) for x in xs]) @ scoop_rot(pitch).T + origin
    return float(np.interp(y, pts[:, 1], pts[:, 2]))


def scoop_rest():
    """(origin, pitch) of the scoop at rest: its front lip on the litter, its handle on the right-hand rim."""
    lip = np.array([BASKET_L / 2, 0.0, -SHELL / 2])
    best = None
    for pitch in np.radians(np.linspace(0.0, 25.0, 501)):
        z0 = LEVEL - 1 * MM - (scoop_rot(pitch) @ lip)[2]
        o = np.array([SCOOP_X, SCOOP_Y, z0])
        gap = _handle_bottom_z(o, pitch, RIM_EDGE_Y) - RIM
        if best is None or abs(gap) < abs(best[2]):
            best = (o, float(pitch), gap)
    return best[0], best[1]


REST_ORIGIN, REST_PITCH = scoop_rest()
# spawned level, its handle 1 mm above the rim; it settles nose-down onto the litter in the first half second
SCOOP0 = np.array([SCOOP_X, SCOOP_Y, RIM + 1 * MM - _handle_bottom_z(np.zeros(3), 0.0, RIM_EDGE_Y - SCOOP_Y)])
# Centre of mass: in the basket, near its back (the basket's floor and walls are most of the plastic; the handle is a
# thin ribbed strip, modelled solid here only to give the fingers a 22 mm grip): the scoop lies flat on its floor.
HANDLE_MASS = 0.018  # of the 70 g: the handle is a thin ribbed strip, most of the plastic is in the basket
SCOOP_COM = (0.0, 0.0, 10 * MM)  # the basket's own (with the handle's 18 g the whole scoop's is about 36 mm behind it)
# Materials: the basket is bare polypropylene (dynamic friction 0.26, Plastics International's PP homopolymer sheet;
# static 0.3), the lower value winning against the litter and the clump (the floor slides under them); the handle takes yam-arena's own grip
# material for held objects (1.2 static, 1.0 dynamic, standing in for the rubber finger pads), the higher value
# winning against the fingers. The handle is its own body welded to the basket (see partkit's attach_to).
SCOOP = part("scoop", pos=tuple(float(v) for v in SCOOP0), yaw=float(np.pi / 2), boxes=SCOOP_BOXES[:-1],
             mass=SCOOP_MASS - HANDLE_MASS, color=SCOOP_GREEN, friction=0.3, com=SCOOP_COM, torsional_patch=0.010,
             box_materials=[(range(len(SCOOP_BOXES) - 1), 0.3, 0.26, "min")])
HANDLE = part("scoop_handle", pos=tuple(float(v) for v in SCOOP0), yaw=float(np.pi / 2), boxes=SCOOP_BOXES[-1:],
              mass=HANDLE_MASS, color=SCOOP_GREEN, friction=1.2, combine="max", torsional_patch=0.010,
              attach_to="scoop")

# -- the bin: mDesign small round wastebasket, 1.25 gal: 7.74 in across the top, 5.73 in across the inside bottom,
# 9.12 in tall (Target listing). Twelve flat sides; it stands on the table (taped down in real trials).
BIN_XY = (0.300, 0.325)  # left of the pan
BIN_H = 9.12 * INCH
BIN_R_TOP, BIN_R_BOT, BIN_T = 7.74 * INCH / 2 - 3 * MM, 5.73 * INCH / 2, 3 * MM
BIN_GREY = (0.85, 0.85, 0.87)


def _bin():
    phi = np.arctan2(BIN_R_TOP - BIN_R_BOT, BIN_H)
    r_mid = (BIN_R_TOP + BIN_R_BOT) / 2
    width = 2 * (r_mid + BIN_T) * np.tan(np.pi / 12) + 2 * MM
    out = []
    for k in range(12):
        a = k * np.pi / 6
        r = r_mid + BIN_T / 2 / np.cos(phi)
        q = _q_mul(_q_axis((0, 0, 1), a), _q_axis((0, 1, 0), phi))
        out.append(Shape(f"bin_side{k:02d}", pos=(BIN_XY[0] + r * np.cos(a), BIN_XY[1] + r * np.sin(a), TOP + BIN_H / 2),
                         size=(BIN_T, float(width), float(BIN_H / np.cos(phi))), rot_xyzw=q, color=BIN_GREY))
    return out


BIN = _bin()

OBJECTS = [*PAN, LITTER, CLUMP, *BIN] if PREP else [*PAN, LITTER, CLUMP, SCOOP, HANDLE, *BIN]  # no scoop while the litter settles
if os.environ.get("NO_LITTER"):  # for testing grasps quickly: no litter (the scoop then rests on the pan's floor)
    _top = LEVEL - 1 * MM
    OBJECTS = [o for o in OBJECTS if o is not LITTER] + [
        Shape("litter_slab", pos=(PX, PY, (FLOOR_Z + _top) / 2), size=(PAN_L, PAN_W, _top - FLOOR_Z),
              color=(0.86, 0.80, 0.66))]

# -- scoring ---------------------------------------------------------------------------------------------------------
LITTER_IN_BIN_MAX = 0.015  # kg of loose litter that may go into the bin with the clump
LITTER_OUT_MAX = 0.015  # kg of litter that may end up neither in the pan nor in the bin (table, floor, scoop)
LIFTED = 10 * MM  # "scooped": the clump on the scoop, its bottom this far above the litter's level
FINGER_REACH = 88 * MM  # the fingers run this far from the tip links' origins (their roots) to their ends
FINGER_HALF = 5 * MM  # half a finger's thickness, for "touching the clump"
_STATE: dict = {"touched": False, "scooped": False, "t": 0.0, "last": -1.0}


def grain_mass(r):
    return LITTER.density * 4 / 3 * np.pi * np.asarray(r) ** 3


def in_pan(p) -> np.ndarray:
    p = np.asarray(p)
    return ((np.abs(p[:, 0] - PX) < PAN_L / 2) & (np.abs(p[:, 1] - PY) < PAN_W / 2)
            & (p[:, 2] > FLOOR_Z - 5 * MM) & (p[:, 2] < RIM + 5 * MM))


def bin_radius(z):
    return BIN_R_BOT + (BIN_R_TOP - BIN_R_BOT) * np.clip((np.asarray(z) - TOP) / BIN_H, 0.0, 1.0)


def in_bin(p) -> np.ndarray:
    p = np.atleast_2d(np.asarray(p, float))
    d = np.hypot(p[:, 0] - BIN_XY[0], p[:, 1] - BIN_XY[1])
    return (d < bin_radius(p[:, 2])) & (p[:, 2] > TOP - 5 * MM) & (p[:, 2] < TOP + BIN_H)


def litter_tally(grains):
    """(kg in the bin, kg neither in the pan nor in the bin)."""
    p, r = grains
    m = grain_mass(r)
    b, k = in_bin(p), in_pan(p)
    return float(m[b].sum()), float(m[~b & ~k].sum())


def clump_bottom(clump) -> float:
    """The clump's lowest point (z)."""
    axis = clump.rotation()[:, 2]
    return float(clump.pos[2] - abs(axis[2]) * CLUMP_H / 2 - np.hypot(axis[0], axis[1]) * CLUMP_R)


def on_scoop(clump, scoop) -> bool:
    """The clump's centre over the basket's floor, inside its walls (the scoop's frame)."""
    c = scoop.to_local(clump.pos)
    return bool(abs(c[0]) < BASKET_L / 2 and abs(c[1]) < SC_W / 2 and -5 * MM < c[2] < WALL_H + CLUMP_H)


def scooped_now(clump, scoop) -> bool:
    """Scooped: the clump on the scoop with its bottom 1 cm or more above the litter's level."""
    return clump_bottom(clump) > LEVEL + LIFTED and on_scoop(clump, scoop)


def score(objects, grains=None, touched=None, scooped=None) -> float:
    """0 if a finger ever touched the clump, or if the clump was never scooped (on the scoop, its bottom 1 cm above
    the litter's level). Once scooped: 1.0 the clump in the bin with at most 15 g of loose litter, and at most 15 g of
    litter left anywhere but the pan and the bin; 0.5 the clump in the bin, with more litter in the bin or out; 0.25
    the clump not in the bin. The pan is fixed, as it is taped down on a real trial."""
    touched = _STATE["touched"] if touched is None else touched
    scooped = _STATE["scooped"] if scooped is None else scooped
    if touched or not scooped:
        return 0.0
    clump = objects["clump"]
    if in_bin(clump.pos)[0]:
        in_b, out = litter_tally(grains if grains is not None else soilkit.grains("litter"))
        return 1.0 if in_b <= LITTER_IN_BIN_MAX and out <= LITTER_OUT_MAX else 0.5
    return 0.25


def success(objects) -> bool:
    return score(objects) == 1.0 and float(np.linalg.norm(objects["clump"].lin_vel)) < 0.01


def finger_points(links) -> np.ndarray:
    """Points along both fingers of one arm, from their roots (tip link origins) to their ends."""
    root = (links["tip_left"] + links["tip_right"]) / 2
    wrist = links.get("gripper", links.get("link_6"))
    d = root - wrist
    d = d / max(np.linalg.norm(d), 1e-9)
    s = np.linspace(0.3, 1.0, 8) * FINGER_REACH
    return np.array([links[k] + d * t for k in ("tip_left", "tip_right") for t in s])


def touches(clump, points) -> bool:
    local = (np.asarray(points) - clump.pos) @ clump.rotation()
    return bool(np.any((np.hypot(local[:, 0], local[:, 1]) < CLUMP_R + FINGER_HALF)
                       & (np.abs(local[:, 2]) < CLUMP_H / 2 + FINGER_HALF)))


def _fingers(scoop, links) -> str:
    """The right hand's finger roots in the scoop's frame, from the grip point (mm), for the log."""
    g = handle_centre(GRIP_X)
    tl, tr = (scoop.to_local(links[k]) - g for k in ("tip_left", "tip_right"))
    return f"fingers_l={np.round(tl * 1000).astype(int).tolist()} r={np.round(tr * 1000).astype(int).tolist()}"


@on_reset
def _reset(environment) -> None:
    _STATE.update(touched=False, scooped=False, t=0.0, last=-1.0)


@on_step
def _watch(environment) -> None:
    if PREP:
        return
    try:
        clump = partkit._part_state(environment, CLUMP)
        links = robot_links(environment)
        if not _STATE["touched"]:
            _STATE["touched"] = any(touches(clump, finger_points(links[side])) for side in ("left", "right"))
        scoop = partkit._part_state(environment, SCOOP)
        if not _STATE["scooped"]:
            _STATE["scooped"] = scooped_now(clump, scoop)
        _STATE["t"] += environment.unwrapped.step_dt
        if os.environ.get("LITTER_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
            _STATE["last"] = _STATE["t"]
            in_b, out = litter_tally(soilkit.grains("litter")) if not os.environ.get("NO_LITTER") else (0.0, 0.0)
            c = scoop.to_local(clump.pos)
            pitch = np.degrees(np.arcsin(-scoop.rotation()[2, 0]))
            roll = np.degrees(np.arctan2(scoop.rotation()[2, 1], scoop.rotation()[2, 2]))
            print(f"LOG t={_STATE['t']:.1f} clump=({clump.pos[0]:.3f},{clump.pos[1]:.3f},"
                  f"{(clump_bottom(clump) - LEVEL) * 1000:+.0f}mm over level) in_scoop=({c[0] * 1000:.0f},"
                  f"{c[1] * 1000:.0f},{c[2] * 1000:.0f})mm scoop=({scoop.pos[0]:.3f},{scoop.pos[1]:.3f},"
                  f"{(scoop.pos[2] - TOP) * 1000:.0f}mm) pitch={pitch:.0f} roll={roll:.0f} "
                  f"litter_bin={in_b * 1000:.1f}g out={out * 1000:.1f}g touched={_STATE['touched']} "
                  f"scooped={_STATE['scooped']} score={score({'clump': clump}):.2f} "
                  f"{_fingers(scoop, links['right'])}", flush=True)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


# -- the scripted demo ---------------------------------------------------------------------------------------------
# The hand holds the handle from the side, its approach level and square to the handle (towards the far side of the
# table), the fingers closing on the handle's top and underside. The fingers' inner faces open towards the palm (a
# V), so a hand reaching along the handle holds it only at the finger tips, and the litter's push back along the
# handle slides it into the V and out of the pinch; held across the fingers, the push runs along the pads' 26.4 mm
# width, and the pads' width along the handle resists the basket tipping in the hand.
DIG_PITCH = float(os.environ.get("DIG_PITCH", "60"))  # deg, nose down: the scoop held steep, pushed in along its
# handle (so the litter pushes back along the handle, not across the grip)
BELOW = float(os.environ.get("BELOW", "0.010"))  # the lip goes this far below the clump's bottom (m), in the plunge
# and the swing under it: shallow, so little litter rides on the scoop (the clump settles to 33 mm under the level)
OVER = float(os.environ.get("OVER", "0.0"))  # the plunge is aimed this far on along the handle past that point (m)
GAP = float(os.environ.get("GAP", "0.015"))  # the lip goes down this far behind the clump's edge (m)
REACH = float(os.environ.get("REACH", "0.080"))  # then swings under it to this far past its centre (m)
SWING_PITCH = float(os.environ.get("SWING_PITCH", "30"))  # deg, nose down, under the clump: with the lip on the
# floor, a flatter scoop's handle would sit below the 4 in rim (level: 53 mm below it; 20 deg: 37 mm above it)
DUMP_PITCH = np.radians(float(os.environ.get("DUMP_PITCH", "45")))  # nose down this far over the bin: the clump
# slides out over the open front (a turn about the hand's own approach axis)
GRIP_TILT = np.radians(float(os.environ.get("GRIP_TILT", "10")))  # the approach tilted down about the handle
GRIP_SIGN = float(os.environ.get("GRIP_SIGN", "-1"))  # which way round the fingers close (for the wrist's range)
_H = np.array([np.cos(HANDLE_RISE), 0.0, -np.sin(HANDLE_RISE)])  # along the handle, towards the basket (scoop frame)
_N = np.array([np.sin(HANDLE_RISE), 0.0, np.cos(HANDLE_RISE)])  # the handle's top side


def grip_rot(tilt=None) -> np.ndarray:
    """The hand's rotation in the scoop's frame: approach along -y (the world's +x at the start), tilted down by
    ``tilt`` about the handle, the fingers closing square to the handle and to the approach."""
    t = GRIP_TILT if tilt is None else tilt
    a = np.cos(t) * np.array([0.0, -1.0, 0.0]) - np.sin(t) * _N
    import tiltkit

    return tiltkit.rotation(a, GRIP_SIGN * np.cross(a, _H))


def tcp_for(origin, pitch=0.0, roll=0.0, tilt=None, gx=None):
    """The hand's pose (position, rotation) holding the scoop with its basket origin at ``origin``, pointing +y,
    pitched ``pitch`` nose-down and turned ``roll`` about its length (rad)."""
    Rs = scoop_rot(pitch, roll)
    return np.asarray(origin, float) + Rs @ handle_centre(GRIP_X if gx is None else gx), Rs @ grip_rot(tilt)


def lip(origin, pitch):
    """The front lip's lower edge in the station frame."""
    return np.asarray(origin, float) + scoop_rot(pitch) @ np.array([BASKET_L / 2, 0.0, -SHELL / 2])


def demo():
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit

    park_l, park_r = (0.15, 0.30, TOP + 0.18), (0.20, -0.30, TOP + 0.20)
    if PREP:
        n = float(os.environ.get("PREP_S", "8"))
        return {"left": [W(n, park_l, O, "wait", 0.0)], "right": [W(n, park_r, O, "wait", 0.0)]}
    tiltkit.install()

    def at(t, origin, pitch, finger, label, joint=False, roll=0.0, back=0.0, tilt=0.0, gx=None):
        p, R = tcp_for(origin, pitch, roll, tilt, gx)
        p = p - R[:, 2] * back  # ``back`` along the approach, behind the grip
        return W(t, tuple(float(v) for v in p), finger, label, tiltkit.Tilt(R, joint=joint))

    def origin_for_lip(lip_yz, pitch, x):
        return np.array([x, lip_yz[0], lip_yz[1]]) - scoop_rot(pitch) @ np.array([BASKET_L / 2, 0.0, -SHELL / 2])

    C = -float(os.environ.get("SQUEEZE_GAP", "0.008")) / 2  # the fingers' target: 2 mm per side past the 12 mm
    # handle (the scoop is light: a fully closed target squeezes it out of the pads)
    C2 = -float(os.environ.get("SQUEEZE2_GAP", "0.005")) / 2  # the dig: about 17 N a finger on the handle
    cy = CLUMP_XY[1]
    cx = DIG_X  # the scoop's middle across the pan while it digs
    r0, rp = REST_ORIGIN, REST_PITCH
    dig_p = np.radians(DIG_PITCH)
    # down along the handle, the lip ending just behind the clump's edge, just below its bottom
    lip_z = CLUMP_Z - CLUMP_H / 2 - 5 * MM - BELOW  # below the clump's bottom once it has settled
    lip_end = np.array([cx, cy - CLUMP_R - GAP, lip_z])
    # pushed in along the basket's floor (edge first), not along the handle: the handle is 9 deg off the floor, so
    # a push along it ploughs the whole floor's underside through the litter, far out from the hand, and levers
    # the handle out of the pinch (the pads hold about 0.6 N m of twist)
    d = scoop_rot(dig_p) @ (_H if os.environ.get("PLUNGE") == "handle" else np.array([1.0, 0.0, 0.0]))
    run = (LEVEL + 10 * MM - lip_end[2]) / -d[2]
    lip_in = lip_end - d * run
    o_in = origin_for_lip(lip_in[1:], dig_p, cx)
    o_dig = origin_for_lip((lip_end + d * OVER)[1:], dig_p, cx)
    # then swing flatter, the lip pushed forward under the clump, and lift at once
    lev_p = np.radians(SWING_PITCH)
    o_under = origin_for_lip((cy + REACH, lip_z), lev_p, cx)
    lift_p = np.radians(-8)
    # then straight up at the same tilt, out of the litter (both hands move alike; turning the scoop while the litter
    # holds it twists it in the pinches), and only then, in the air, turned about the bottom of the back wall so the
    # floor levels and tips back a little (the clump rolls to the back wall)
    heel = np.array([-BASKET_L / 2 + 5 * MM, 0.0, 0.0])  # on the floor, at the back wall (part frame)
    o_lift1 = o_under + np.array([0.0, 0.0, LEVEL - lip_z + float(os.environ.get("LIFT_CLEAR", "0.035"))])
    rise = float(o_lift1[2] - o_under[2])
    mid = np.zeros(3)  # the basket's middle (part frame), lifted straight up while the scoop levels about it
    flat = np.radians(float(os.environ.get("LIFT_END_PITCH", "5")))

    def jig_pitch(z):
        return float(lev_p + (flat - lev_p) * min(max(z / rise, 0.0), 1.0))

    def jig_origin(z):
        c = o_under + scoop_rot(lev_p) @ mid + np.array([0.0, 0.0, z])
        return c - scoop_rot(jig_pitch(z)) @ mid
    lift_jig, z = [], 0.0  # up 8 mm, down 4 mm, every 0.2 s, until the scoop is clear
    while z < rise:
        z = min(z + 8 * MM, rise)
        lift_jig.append(z)
        if z < rise:
            z -= 4 * MM
            lift_jig.append(z)
    top = jig_origin(rise)
    P = top + scoop_rot(flat) @ heel

    def about(rise, pitch):
        return P + np.array([0.0, 0.0, rise]) - scoop_rot(pitch) @ heel

    o_lift = about(15 * MM, np.radians(0))
    o_tip = about(40 * MM, lift_p)
    o_up = np.array([o_tip[0], o_tip[1], max(o_tip[2], RIM + 45 * MM)])  # straight on up, over the pan
    # tipped over the bin's near half (the right arm reaches no further), the lip 20 mm above its top
    o_dump = origin_for_lip((BIN_XY[1] - 60 * MM, TOP + BIN_H + 20 * MM), DUMP_PITCH, BIN_XY[0])
    o_bin = o_dump + np.array([0.0, -40 * MM, 50 * MM])
    ga = GRIP_X
    shake = []
    for k in range(4):
        for dx in (12 * MM, -12 * MM):
            shake.append(at(0.35, o_up + (dx, 0, 0), lift_p, C2, f"shake {k + 1}", gx=ga))
    right = [
        at(3.0, r0, rp, O, "beside the handle, level with it", joint=True, back=0.05, tilt=GRIP_TILT),
        at(2.0, r0, rp, O, "in over the handle, fingers above and below it", tilt=GRIP_TILT),
        at(1.0, r0, rp, -0.015, "close to 30 mm", tilt=GRIP_TILT),
        at(float(os.environ.get("CLOSE_S", "2.0")), r0, rp, -0.007, "slowly to just clear of the handle (14 mm)",
           tilt=GRIP_TILT),
        at(1.5, r0, rp, C, "squeeze it slowly (a fast close knocks the light scoop away)", tilt=GRIP_TILT),
        at(2.0, r0 + (0, 0, 0.03), rp, C, "lift it straight up off the litter and the rim, slowly", tilt=GRIP_TILT),
        at(1.5, r0 + (0, 0, 0.06), np.radians(5), C, "and on up"),
        at(1.0, r0 + (0, 0, 0.06), np.radians(5), C2, "squeeze harder now it hangs free"),
        at(6.0, o_in - d * 0.02, dig_p, C2, "turn steep slowly (a quick turn swings the scoop round in the pinch), lip over the litter close behind the clump", joint=True),
        at(1.0, o_in, dig_p, C2, "lip on the litter"),
    ]
    right += [
        at(4.0, o_dig, dig_p, C2, "push it in along its floor, edge first, to just below the clump's bottom", gx=ga),
        at(float(os.environ.get("SWING_S", "8.0")), o_under, lev_p, C2, "swing flatter, slowly, the lip forward under the clump", gx=ga),
        *[at(0.2, jig_origin(z), jig_pitch(z), C2, "straight up out of the litter, jigging it and levelling it as it comes: the litter runs off and out through the slots and lets go of the basket", gx=ga) for z in lift_jig],
        at(1.5, o_lift, 0.0, C2, "level it in the air, turning about the back wall", gx=ga),
        at(1.5, o_tip, lift_p, C2, "tip back so the clump rolls to the back wall", gx=ga),
        at(2.0, o_up, lift_p, C2, "on up over the pan", gx=ga),
        *shake,
        at(1.5, o_up, lift_p, C2, "hold over the pan: the last litter drops back", gx=ga),
        at(3.5, o_bin, lift_p, C2, "over the bin", joint=True, gx=ga),
        at(2.5, o_dump, DUMP_PITCH, C2, "tip it nose down: the clump slides out into the bin", gx=ga),
        *[at(0.3, o_dump + np.array([0.0, dy, 0.0]), DUMP_PITCH, C2, "jog it: the sticky clump lets go", gx=ga)
          for dy in (12 * MM, -12 * MM, 12 * MM, -12 * MM, 12 * MM, 0.0)],
        at(1.5, o_dump, DUMP_PITCH, C2, "hold", gx=ga),
        at(2.0, o_bin, lift_p, C2, "turn it back", gx=ga),
    ]
    if TWO_HANDS:
        right, left = _second_hand(right, at, park_l, O, ga, C)
    else:
        left = [W(sum(w.duration for w in right), park_l, O, "wait", 0.0)]
    if os.environ.get("DEMO_UPTO"):  # for testing: only the first N moves (of the right arm), then hold
        n = int(os.environ["DEMO_UPTO"])
        t_cut = sum(w.duration for w in right[:n])
        right = right[:n]
        right.append(W(3.0, right[-1].pos, right[-1].finger, "hold", right[-1].yaw))
        keep, acc = [], 0.0
        for w in left:
            if acc >= t_cut:
                break
            keep.append(w)
            acc += w.duration
        left = keep + [W(3.0 + max(0.0, t_cut - acc), keep[-1].pos, keep[-1].finger, "hold", keep[-1].yaw)]
    return {"left": left, "right": right}


# the scoop digs here across the pan: the clump (44 mm) well inside its 124 mm width, its sides more than 20 mm from
# the pan's walls (grains jam a narrower gap and hold the scoop fast)
DIG_X = float(os.environ.get("DIG_X", "0.320"))  # it ends about 15 mm further on: the handle eases out of the pinches
TWO_HANDS = os.environ.get("TWO_HANDS", "1") == "1"
LEFT_GRIP_X = float(os.environ.get("LEFT_GRIP_X", "-0.095"))  # the left hand's grip on the handle, near the basket
LIFT_LOAD = float(os.environ.get("LIFT_LOAD", "0.8"))  # kg
LOAD_X = float(os.environ.get("LOAD_X", "0.040"))  # where it bears (part frame): towards the lip
LIFT_LABELS = ("straight up", "level it", "tip back")
HOLD_AT = 7  # the right hand's waypoint (it holds the scoop up, free) after which the left hand takes hold too


def _second_hand(right, at, park_l, O, ga, C):
    """Both hands on the handle: the left takes hold near the basket while the right holds the scoop up, then both
    follow the same scoop path (straight-line moves: two arms in joint-space moves would fight over the scoop). Two
    pinches 85 mm apart hold the scoop's tipping far better than one: a single pinch on the thin handle resists
    only about 0.5 N m."""
    from yam_arena.demo import Waypoint as W

    import tiltkit

    G = handle_centre(GRIP_X)
    w7 = right[HOLD_AT]
    Rs7 = w7.yaw.rot @ grip_rot(0.0).T
    o7 = np.array(w7.pos) - Rs7 @ G
    p7 = float(np.degrees(np.arcsin(-Rs7[2, 0])))
    beside = at(1.5, o7, np.radians(p7), O, "left: down beside the handle near the basket", back=0.06, gx=LEFT_GRIP_X)
    above = W(3.0, (beside.pos[0], beside.pos[1], beside.pos[2] + 0.08), O, "left: above it, clear of the scoop",
              tiltkit.Tilt(beside.yaw.rot, joint=True))
    grab = [
        above,
        beside,
        at(2.0, o7, np.radians(p7), O, "left: in over the handle", gx=LEFT_GRIP_X),
        at(1.0, o7, np.radians(p7), -0.015, "left: close to 30 mm", gx=LEFT_GRIP_X),
        at(2.0, o7, np.radians(p7), -0.007, "left: slowly to just clear of the handle", gx=LEFT_GRIP_X),
        at(1.5, o7, np.radians(p7), C, "left: squeeze it", gx=LEFT_GRIP_X),
    ]
    t_before = sum(w.duration for w in right[:HOLD_AT + 1])
    t_grab = sum(w.duration for w in grab)
    left = [W(t_before, park_l, O, "wait", 0.0), *grab]
    new_right = right[:HOLD_AT + 1] + [W(t_grab, w7.pos, w7.finger, "hold while the left hand takes hold", w7.yaw)]
    # the litter and clump on the floor weigh on the basket (about LIFT_LOAD kg, with the litter's hold on it as it
    # comes out); the hand nearer the basket carries more than all of it, the other presses down (a lever): both are
    # aimed past their arms' give (tiltkit's load), or the scoop tips nose-down as it comes up
    span = LEFT_GRIP_X - GRIP_X
    k_l, k_r = (LOAD_X - GRIP_X) / span, (LOAD_X - LEFT_GRIP_X) / span  # a load at LOAD_X on a beam held at both grips
    for w in right[HOLD_AT + 1:]:
        R = w.yaw.rot
        gxw = ga  # every later right waypoint holds the handle at ga
        Rs = R @ grip_rot(0.0).T
        o = np.array(w.pos) - Rs @ handle_centre(gxw)
        pl = o + Rs @ handle_centre(LEFT_GRIP_X)
        lifting = w.label.startswith(LIFT_LABELS)
        new_right.append(W(w.duration, w.pos, w.finger, w.label,
                           tiltkit.Tilt(R, joint=False, load=(-LIFT_LOAD * k_r, (0, 0, 0)) if lifting else None)))
        left.append(W(w.duration, tuple(float(v) for v in pl), w.finger, "left: " + w.label,
                      tiltkit.Tilt(R, joint=False, load=(LIFT_LOAD * k_l, (0, 0, 0)) if lifting else None)))
    return new_right, left


# -- self test -------------------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built states."""
    from partkit import PartState

    def state(pos, R=np.eye(3)):
        w = np.sqrt(max(1.0 + np.trace(R), 1e-12)) / 2
        q = np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
        p = np.asarray(pos, float)
        return PartState(pos=p, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p, sliders={})

    p0, r0 = soilkit.lattice(LITTER)
    m0 = grain_mass(r0)

    def litter(to_bin=0.0, to_table=0.0):
        """The lattice with ``to_bin`` kg moved into the bin and ``to_table`` kg onto the table."""
        p = p0.copy()
        cum = np.cumsum(m0)
        nb = int(np.searchsorted(cum, to_bin))
        nt = int(np.searchsorted(cum, to_bin + to_table)) - nb
        p[:nb] = (BIN_XY[0], BIN_XY[1], TOP + 20 * MM)
        p[nb:nb + nt] = (0.25, -0.25, TOP + 2 * MM)
        return p, r0

    home = state((CLUMP_XY[0], CLUMP_XY[1], CLUMP_Z))
    binned = state((BIN_XY[0], BIN_XY[1], TOP + CLUMP_R))
    on_table = state((0.25, -0.25, TOP + CLUMP_R))
    cases = {
        "untouched, still in the litter": (home, litter(), False, False, 0.0),
        "scooped, then dropped back in the pan": (home, litter(), False, True, 0.25),
        "scooped, dropped on the table": (on_table, litter(), False, True, 0.25),
        "pushed onto the table, never scooped": (on_table, litter(), False, False, 0.0),
        "in the bin with litter, never scooped (pan tipped)": (binned, litter(0.5, 0.2), False, False, 0.0),
        "in the bin, sifted, but never scooped": (binned, litter(), False, False, 0.0),
        "in the bin, sifted": (binned, litter(0.005, 0.005), False, True, 1.0),
        "in the bin, 14 g litter with it": (binned, litter(0.014, 0.0), False, True, 1.0),
        "in the bin with a whole unsifted scoopful": (binned, litter(0.120, 0.0), False, True, 0.5),
        "in the bin, 30 g spilt on the table": (binned, litter(0.0, 0.030), False, True, 0.5),
        "in the bin, but picked with the fingers": (binned, litter(), True, True, 0.0),
    }
    for name, (clump, g, touched, scooped, want) in cases.items():
        got = score({"clump": clump}, g, touched, scooped)
        assert abs(got - want) < 1e-9, (name, got, want)
    # the starting scene against the rules: the clump in the litter, not scooped, not in the bin
    assert not in_bin(home.pos)[0] and clump_bottom(home) < LEVEL and clump_bottom(home) > FLOOR_Z
    # "scooped": the clump lying on the basket's floor, raised clear, counts; raised beside the scoop does not
    R = scoop_rot(0.0)
    raised = state((CLUMP_XY[0], CLUMP_XY[1], LEVEL + LIFTED + CLUMP_H / 2 + 5 * MM))
    under = state(raised.pos - R @ np.array([0.0, 0.0, SHELL / 2 + CLUMP_H / 2]), R)
    assert scooped_now(raised, under) and not scooped_now(home, state(home.pos - R @ (0, 0, CLUMP_H / 2), R))
    assert not scooped_now(raised, state(raised.pos + (0.0, -BASKET_L, 0.0), R))
    assert not scooped_now(raised, state(raised.pos + (0.0, 0.0, -0.10), R))
    # finger check: a finger beside the clump touches it, one 2 cm away does not
    side = np.array([CLUMP_XY[0] + CLUMP_R + 3 * MM, CLUMP_XY[1], CLUMP_Z])
    assert touches(home, side[None]) and not touches(home, (side + (0.02, 0, 0))[None])
    # sizes: a grain falls through a slot, the clump does not; the clump fits in the basket and the bin
    assert 2 * R_MAX < SLOT_W < 2 * CLUMP_R and 2 * CLUMP_R < SC_W - 2 * SHELL and 2 * CLUMP_R < 2 * BIN_R_BOT
    assert abs(RIB_W * (SLOT_N + 1) + SLOT_W * SLOT_N - SC_W) < 1e-9
    # layout: the scoop, the pan and the bin do not overlap
    assert BIN_XY[1] - BIN_R_TOP - BIN_T > PY + PAN_W / 2 + PAN_T  # the bin stands clear of the pan, to its left
    # the scoop at rest: basket in the pan, clear of the clump, its handle over the right-hand rim, gripped outside it
    assert SCOOP_Y - BASKET_L / 2 > PY - PAN_W / 2 and SCOOP_Y + BASKET_L / 2 < CLUMP_XY[1] - CLUMP_R - 20 * MM
    assert SCOOP_Y + GRIP_X < RIM_EDGE_Y - 20 * MM and 0 < REST_PITCH < np.radians(20)
    # the lip stays clear of the pan's floor through the dig
    import tiltkit

    tiltkit.install()
    G = handle_centre(GRIP_X)
    for w in demo()["right"]:
        Rs = w.yaw.rot @ grip_rot(0.0).T
        o = np.array(w.pos) - Rs @ G
        assert lip(o, np.arcsin(-Rs[2, 0]))[2] > FLOOR_Z + 5 * MM, w.label
    return len(cases) + 5


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
    p, r = soilkit.start_layout(LITTER)
    print(f"litter: {len(p)} grains, {grain_mass(r).sum() * 1000:.0f} g; scoop handle rise "
          f"{np.degrees(HANDLE_RISE):.1f} deg, rib {RIB_W * 1000:.1f} mm")
