"""Close out a locked-case showing: set the watch box back on its marked spot on the showcase floor, slide the inner glass door shut, then the
outer one, and push the keyless plunger lock in until it latches.

A single-watch case (9.7 x 7.6 x 7.1 cm) goes back in a countertop showcase, 24 x 12 x 18 in (61 x 30.5 x 45.7 cm; Webstaurant 89129359 locking countertop display case), aluminium frame and 1/4 in
glass, stands on the counter with its sliding back doors toward the arms. Its two bypassing glass doors, 31 x 40 cm each, run in a double track.
A CRL KML41-style keyless plunger lock sits in the outer door near the overlap: pushed in 12 mm against its spring, it latches, its tip standing
beside the inner door's edge so that neither door can slide.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --mode knock,near_tier,doors_open,no_lock,inner_open,hand_on
"""

from __future__ import annotations

import dataclasses
import math
import re
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ('Close up the showcase after the showing. Put the watch box back on its marked spot on the lower step, at the front of the case, without knocking over or shifting the '
               'watches standing on the higher step; slide the inner glass door shut, then the outer glass door, and push the plunger lock on the outer door in until it latches. Then take your hands away.')
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 2 s hold: score() and success() agree
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the case, station metres (x forward from the arm bases, y left, z up from the counter). Doors face the arms.
CASE_X = (0.370, 0.675)  # back (door side) to front: 30.5 cm deep
CASE_Y = (-0.305, 0.305)  # 61 cm wide
CASE_H = 0.457  # 18 in
FLOOR_Z = 0.040  # the display deck's top: a 4 cm aluminium base
TOPRAIL_Z = 0.447  # under side of the top rail and glass top
GLASS_T = 0.006  # 1/4 in
OUTER_X = (0.376, 0.382)  # the outer (front-track) door's glass, nearer the arms
INNER_X = (0.390, 0.396)  # the inner (back-track) door's glass
DOOR_Z = (0.046, 0.446)  # each door 40 cm tall, standing in the bottom track
DOOR_W = 0.310
INNER_CLOSED_Y = (-0.300, 0.010)  # the inner door shut: the right half; it opens toward +y
OUTER_CLOSED_Y = (-0.010, 0.300)  # the outer door shut: the left half; it opens toward -y. The two overlap 2 cm
INNER_OPEN = 0.060  # the inner door starts 6 cm open
OUTER_OPEN = 0.290  # the outer door starts slid fully aside, in front of the inner door
DOOR_MASS = 1.9  # 0.31 x 0.40 m x 6 mm glass at 2,500 kg/m^3
DOOR_FRICTION = 3.0  # N: the slide's friction in its track (assumed, 2 to 5 N for glass doors on plastic shoes)

# -- the outer door's pull and lock, at the door's shut place (door frame)
PULL_Y, PULL_X, PULL_R, PULL_Z = 0.060, 0.351, 0.0075, (0.170, 0.320)  # a 15 mm round bar pull, 15 cm long, on 25 mm standoffs
LOCK_Y, LOCK_Z = 0.025, 0.120  # the plunger's axis: 15 mm past the inner door's edge, low on the door
HOUSING_R, HOUSING_L = 0.012, 0.012  # the lock's barrel on the door's arm side
HEAD_R, HEAD_L = 0.008, 0.017  # the plunger's 16 mm head, standing 17 mm out of the barrel
TRAVEL = 0.012  # 12 mm in to latch
SPRING = 830.0  # N/m: 10 N at full travel (CRL's figure for the KML41 class, 8 to 10 N; assumed)
LATCH_AT = 0.0115  # the catch takes it this far in

# -- the watch box: a single-watch case with a pillow (Prestige & Fancy, PU leather, 3.8 x 3 x 2.8 in, Walmart 5341909840), rigid, 150 g (assumed),
# its long side along x; its origin at the centre of its base
BOX = np.array([0.097, 0.076, 0.071])
BOX_MASS = 0.15
BOX_START = np.array([0.300, 0.420])  # on the counter beside the case's left end
SPOT = np.array([0.525, 0.170])  # the 12 x 12 cm marked spot on the lower tier (the deck), its centre 15.5 cm in from the door line
SPOT_W = 0.120

# -- the display: a two-step stair inside the case, the higher step nearer the arms (the doors), the lower one (the deck) toward the customer.
# The higher step is a white-velvet-covered riser 2 in high and 2.5 in deep across the case (a velvet riser cut to the case's inside width), on the
# black velvet deck: white on black, so the step reads from every camera
RISER_X = (0.400, 0.464)
RISER_Y = (-0.292, 0.292)
RISER_H = 0.051
# Four watches stand on the riser, each on a clear C-ring watch stand (sold in 10-packs, 2.2 in D x 1.5 in W x 3.9 in H): a 5.6 x 3.8 cm base, a short
# upright and a C-ring the strap wraps round, the watch's head at the front, facing the customer.
# The box passes between the two left stands, 18 cm apart centre to centre: a 14.2 cm gap for the 7.6 cm box and the fingers either side
STAND_X = 0.432
STAND_YS = (0.260, 0.080, -0.080, -0.230)
STAND_D, STAND_W, STAND_H = 0.056, 0.038, 0.099
STAND_MASS = 0.15  # 25 g of plastic and a 125 g steel watch (assumed)
STAND_MOVED = 0.015  # void: a stand's base more than 1.5 cm from where it stood, or
STAND_TIPPED = 20.0  # tilted more than 20 degrees

# -- the scorer's thresholds
ON_SPOT = 0.025  # the box's centre within 2.5 cm of the spot's centre on both axes (the whole box then lies on the 12 cm spot but for 1 cm)
SHUT = 0.005  # a door within 5 mm of its closed stop
BOX_UPRIGHT = 10.0
TIPPED = 45.0  # the box on a side, or
FALL_STEP = 0.008  # the box dropped: its base 8 mm lower than one control step (50 ms) before. A free fall of 1.6 cm or more always
# puts 8 mm into one step; the demo lowers the box at most 5.6 mm a step. Either voids the run
LET_GO = 0.005  # every point of both grippers at least 5 mm off the box
HANDS_OFF = 0.010  # every point of both grippers and wrists at least 1 cm outside the case's outline (with its pull and lock)

LOOKS = {
    "riser": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=1.0),  # white velvet
    "acrylic": m.Look(glass=(0.90, 0.95, 0.97)),
    "strap": m.Look(color=m.rgb(0.04, 0.04, 0.045), roughness=0.5),  # black leather
    "strap2": m.Look(color=m.rgb(0.28, 0.13, 0.05), roughness=0.5),  # dark brown leather
    "bezel": m.Look(color=m.rgb(0.03, 0.03, 0.04), metallic=0.3, roughness=0.3),  # black ceramic bezel
    "steel": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.25),
    "dial": m.Look(color=m.rgb(0.04, 0.10, 0.36), roughness=0.3),  # dark blue dial
    "dial2": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.3),  # white dial
    "alu": m.Look(color=m.rgb(0.80, 0.81, 0.82), metallic=0.85, roughness=0.3),
    "glass": m.Look(glass=(0.93, 0.97, 0.98)),
    "deck": m.Look(color=m.rgb(0.03, 0.03, 0.035), roughness=1.0),  # black velvet
    "tape": m.Look(color=m.rgb(0.95, 0.95, 0.92), roughness=0.6),
    "chrome": m.Look(color=m.rgb(0.85, 0.86, 0.88), metallic=0.95, roughness=0.15),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.4),
    "box": m.Look(color=m.rgb(0.55, 0.10, 0.13), roughness=0.55),  # burgundy leatherette
    "gold": m.Look(color=m.rgb(0.80, 0.65, 0.30), metallic=0.8, roughness=0.3),
    "track": m.Look(color=m.rgb(0.25, 0.25, 0.26), roughness=0.6),
}


def _bx(x, y, z) -> m.Mesh:
    return m.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _cbox(name, x, y, z) -> str:
    return m.col_box(name, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


# -- the case's frame (fixed): base, deck, posts, side and front glass, top, the tracks --------------------------------------------------------------

def _frame():
    x, y = CASE_X, CASE_Y
    parts, cols = [], []
    parts.append(("base", _bx(x, y, (0.0, FLOOR_Z - 0.003)), "alu"))
    parts.append(("deck", _bx((x[0] + 0.030, x[1] - 0.006), (y[0] + 0.006, y[1] - 0.006), (FLOOR_Z - 0.003, FLOOR_Z)), "deck"))
    cols.append(_cbox("base_c", x, y, (0.0, FLOOR_Z - 0.003)))
    cols.append(_cbox("deck_c", (x[0] + 0.030, x[1] - 0.006), (y[0] + 0.006, y[1] - 0.006), (FLOOR_Z - 0.003, FLOOR_Z)))
    # the bottom track: three lips either side of the two door bands
    for k, lx in enumerate(((x[0], OUTER_X[0] - 0.002), (OUTER_X[1] + 0.002, INNER_X[0] - 0.002), (INNER_X[1] + 0.002, x[0] + 0.030))):
        parts.append((f"lip{k}", _bx(lx, y, (FLOOR_Z - 0.003, FLOOR_Z + 0.012)), "track"))
        cols.append(_cbox(f"lip{k}_c", lx, y, (FLOOR_Z - 0.003, FLOOR_Z + 0.012)))
    # the top: rail over the doors and the glass top
    parts.append(("top_rail", _bx((x[0], x[0] + 0.030), y, (TOPRAIL_Z, CASE_H)), "alu"))
    parts.append(("top_glass", _bx((x[0] + 0.030, x[1]), y, (CASE_H - GLASS_T, CASE_H)), "glass"))
    cols.append(_cbox("top_c", x, y, (TOPRAIL_Z, CASE_H)))
    # the front and side glass, and the four corner posts (the back posts' colliders leave the door bands free)
    parts.append(("front_glass", _bx((x[1] - GLASS_T, x[1]), (y[0] + 0.012, y[1] - 0.012), (FLOOR_Z, TOPRAIL_Z)), "glass"))
    cols.append(_cbox("front_c", (x[1] - GLASS_T, x[1]), y, (FLOOR_Z, TOPRAIL_Z)))
    for s, yy in (("l", (y[1] - GLASS_T, y[1])), ("r", (y[0], y[0] + GLASS_T))):
        parts.append((f"side_{s}", _bx((x[0] + 0.030, x[1] - 0.012), yy, (FLOOR_Z, TOPRAIL_Z)), "glass"))
        cols.append(_cbox(f"side_{s}_c", (x[0] + 0.030, x[1]), yy, (FLOOR_Z, TOPRAIL_Z)))
        py = (y[1] - 0.012, y[1]) if s == "l" else (y[0], y[0] + 0.012)
        parts.append((f"post_f{s}", _bx((x[1] - 0.012, x[1]), py, (FLOOR_Z, TOPRAIL_Z)), "alu"))
        parts.append((f"post_b{s}", _bx((x[0], x[0] + 0.030), py, (FLOOR_Z, TOPRAIL_Z)), "alu"))
        cols.append(_cbox(f"post_b{s}_c0", (x[0], OUTER_X[0] - 0.002), py, (FLOOR_Z, TOPRAIL_Z)))
        cols.append(_cbox(f"post_b{s}_c1", (INNER_X[1] + 0.002, x[0] + 0.030), py, (FLOOR_Z, TOPRAIL_Z)))
    # the higher step: the white velvet riser
    parts.append(("riser", _bx(RISER_X, RISER_Y, (FLOOR_Z, FLOOR_Z + RISER_H)), "riser"))
    cols.append(_cbox("riser_c", RISER_X, RISER_Y, (FLOOR_Z, FLOOR_Z + RISER_H)))
    # the marked spot: white tape round a 12 cm square on the deck
    sx, sy, w, t = SPOT[0], SPOT[1], SPOT_W / 2, 0.006
    for k, (ax, ay) in enumerate((((sx - w, sx + w), (sy - w, sy - w + t)), ((sx - w, sx + w), (sy + w - t, sy + w)),
                                  ((sx - w, sx - w + t), (sy - w, sy + w)), ((sx + w - t, sx + w), (sy - w, sy + w)))):
        parts.append((f"tape{k}", _bx(ax, ay, (FLOOR_Z, FLOOR_Z + 0.0005)), "tape"))
    return parts, cols


# -- the doors and the lock ------------------------------------------------------------------------------------------------------------------------

def _door(name, gx, ys, extra=(), extra_cols=()) -> tuple:
    parts = [(f"{name}_glass", _bx(gx, ys, DOOR_Z), "glass"),
             (f"{name}_shoe", _bx(gx, ys, (DOOR_Z[0], DOOR_Z[0] + 0.012)), "track")]
    cols = [_cbox(f"{name}_c", gx, ys, (DOOR_Z[0] + 0.010, DOOR_Z[1] - 0.004))]
    return parts + list(extra), cols + list(extra_cols)


def _links():
    # the inner door, authored where it starts (6 cm open): joint 0 is the start, -INNER_OPEN is shut
    iy = (INNER_CLOSED_Y[0] + INNER_OPEN, INNER_CLOSED_Y[1] + INNER_OPEN)
    ip, ic = _door("inner", INNER_X, iy)
    inner = m.slider("inner", "Y", -INNER_OPEN, DOOR_W - 0.020 - INNER_OPEN, ip, ic, mass=DOOR_MASS, spring=0.0, damping=4.0, friction=DOOR_FRICTION,
                     effort=50.0, armature=0.0, inertia=(DOOR_MASS * 0.31 ** 2 / 12, DOOR_MASS * 0.40 ** 2 / 12, DOOR_MASS * 0.31 ** 2 / 12))
    # the outer door, authored where it starts (slid aside 29 cm): +OUTER_OPEN is shut
    d = -OUTER_OPEN
    oy = (OUTER_CLOSED_Y[0] + d, OUTER_CLOSED_Y[1] + d)
    pull = m.cylinder(PULL_R, PULL_Z[1] - PULL_Z[0], (PULL_X, PULL_Y + d, (PULL_Z[0] + PULL_Z[1]) / 2))
    extra = [("pull", pull, "chrome")]
    extra_cols = [m.col_cylinder("pull_c", PULL_R, PULL_Z[1] - PULL_Z[0], (PULL_X, PULL_Y + d, (PULL_Z[0] + PULL_Z[1]) / 2))]
    for k, zz in enumerate((PULL_Z[0] + 0.015, PULL_Z[1] - 0.015)):
        sx = (PULL_X, OUTER_X[0])
        so = m.cylinder(0.005, sx[1] - sx[0], ((sx[0] + sx[1]) / 2, PULL_Y + d, zz), axis="x")
        extra.append((f"standoff{k}", so, "chrome"))
        extra_cols.append(m.col_hull(f"standoff{k}_c", so))
    hx = (OUTER_X[0] - HOUSING_L, OUTER_X[0])
    housing = m.cylinder(HOUSING_R, HOUSING_L, ((hx[0] + hx[1]) / 2, LOCK_Y + d, LOCK_Z), axis="x")
    back = m.cylinder(HOUSING_R, 0.006, (OUTER_X[1] + 0.003, LOCK_Y + d, LOCK_Z), axis="x")  # the clamp nut behind the glass
    extra += [("lock_barrel", housing, "chrome"), ("lock_nut", back, "chrome")]
    extra_cols.append(m.col_hull("lock_barrel_c", housing))
    op, oc = _door("outer", OUTER_X, oy, extra, extra_cols)
    outer = m.slider("outer", "Y", 0.0, OUTER_OPEN, op, oc, mass=DOOR_MASS + 0.15, spring=0.0, damping=4.0, friction=DOOR_FRICTION,
                     effort=50.0, armature=0.0, inertia=(DOOR_MASS * 0.31 ** 2 / 12, DOOR_MASS * 0.40 ** 2 / 12, DOOR_MASS * 0.31 ** 2 / 12))
    # the plunger, on the outer door: its head stands out toward the arms; it goes in along +x
    head_x = (hx[0] - HEAD_L, hx[0])
    head = m.cylinder(HEAD_R, HEAD_L, ((head_x[0] + head_x[1]) / 2, LOCK_Y + d, LOCK_Z), axis="x")
    cap = m.cylinder(HEAD_R - 0.002, 0.001, (head_x[0] - 0.0005, LOCK_Y + d, LOCK_Z), axis="x")
    plunger = m.slider("plunger", "X", 0.0, TRAVEL, [("head", head, "chrome"), ("cap", cap, "black")], [m.col_hull("head_c", head)],
                       at=(head_x[1], LOCK_Y + d, LOCK_Z), mass=0.02, spring=SPRING, damping=2.0, friction=0.0, effort=40.0, parent="outer")
    return [inner, outer, plunger]


def _case():
    parts, cols = _frame()
    case = m.mechanism("showcase", (0.0, 0.0, TOP), _links(), LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.4, 0.3), solver_iters=32)
    # Clear glass lets the light in: the renderer's glass would otherwise cast a full shadow and leave the display inside in the dark
    build = case.build
    no_shadow = re.compile(r'(def Mesh "(?:top_glass|front_glass|side_l|side_r|inner_glass|outer_glass)" \(\s*prepend apiSchemas = \["MaterialBindingAPI"\]\s*\)\s*\{)')
    return dataclasses.replace(case, build=lambda: no_shadow.sub(r'\1\n            bool primvars:doNotCastShadows = 1', build()))


def _box():
    b = BOX
    parts = [("base", m.box((b[0], b[1], b[2] - 0.022), (0.0, 0.0, (b[2] - 0.022) / 2)), "box"),
             ("lid", m.box((b[0] + 0.002, b[1] + 0.002, 0.022), (0.0, 0.0, b[2] - 0.011)), "box"),
             ("crest", m.box((0.030, 0.012, 0.0006), (0.0, 0.0, b[2] + 0.0003)), "gold")]
    return m.free("box", (BOX_START[0], BOX_START[1], TOP + 0.0005), parts, [m.col_box("box_c", tuple(b), (0.0, 0.0, b[2] / 2))], LOOKS, mass=BOX_MASS,
                  friction=(0.6, 0.5))


def _stand(k: int, y: float):
    """A watch on its C-ring stand. Origin at the centre of the stand's base, on the riser. The strap wraps the ring; the watch's head sits on the
    ring's front, turned up 55 degrees, its dial toward the customer and up (as watches are shown on C-ring stands)."""
    d, w = STAND_D, STAND_W
    ring_r, ring_z, ring_l = 0.026, STAND_H - 0.045, 0.020  # the head's top 9.9 cm up, as tall as before
    up = 55.0  # the head's place on the ring, degrees above the front
    base = m.box((d, w, 0.006), (0.0, 0.0, 0.003))
    post = m.box((0.012, 0.026, ring_z - 0.006 - ring_r + 0.004), (0.0, 0.0, (0.006 + ring_z - ring_r + 0.004) / 2))
    ring = m.cylinder(ring_r, ring_l, (0.0, 0.0, ring_z), axis="y")
    strap = m.cylinder(ring_r + 0.003, ring_l - 0.002, (0.0, 0.0, ring_z), axis="y")

    def on_ring(mesh: m.Mesh, out: float) -> m.Mesh:  # a disc built along +z, set on the ring's front `out` from its axis and turned up
        return mesh.moved((0.0, 0.0, out)).turned(90.0 - up, "y").moved((0.0, 0.0, ring_z))

    head = on_ring(m.cylinder(0.021, 0.011, (0.0, 0.0, 0.0)), ring_r + 0.0085)  # a 42 mm steel case
    bezel = on_ring(m.cylinder(0.0205, 0.0025, (0.0, 0.0, 0.0)), ring_r + 0.0150)
    dial = on_ring(m.cylinder(0.0160, 0.0010, (0.0, 0.0, 0.0)), ring_r + 0.0165)
    parts = [("base", base, "acrylic"), ("post", post, "acrylic"), ("ring", ring, "acrylic"), ("strap", strap, "strap" if k % 2 == 0 else "strap2"),
             ("head", head, "steel"), ("bezel", bezel, "bezel"), ("dial", dial, "dial" if k % 2 == 0 else "dial2")]
    cols = [m.col_box("base_c", (d, w, 0.006), (0.0, 0.0, 0.003)), m.col_hull("post_c", post), m.col_hull("strap_c", strap), m.col_hull("head_c", head)]
    return m.free(f"stand{k}", (STAND_X, y, TOP + FLOOR_Z + RISER_H + 0.0005), parts, cols, LOOKS, mass=STAND_MASS, friction=(0.5, 0.4))


OBJECTS = [_case(), _box()] + [_stand(k, y) for k, y in enumerate(STAND_YS)]


# -- the scorer -------------------------------------------------------------------------------------------------------------------------------------

def _palm_points() -> np.ndarray:
    xs, ys, zs = np.linspace(-0.05, 0.05, 17), np.linspace(-0.033, 0.033, 12), np.linspace(-0.01, 0.064, 13)
    return np.array([(x, y, z) for x in xs for y in ys for z in zs if x in (xs[0], xs[-1]) or y in (ys[0], ys[-1]) or z in (zs[0], zs[-1])])


PALM = _palm_points()  # the palm and housing's whole surface on a grid of about 6 mm, in the `gripper` body's frame
_FY, _FZ = np.meshgrid(np.linspace(-0.018, 0.018, 7), np.linspace(0.06, 0.147, 16))


def gripper_points(opening: float) -> np.ndarray:
    """Points on one gripper in its `gripper` body's frame: the palm and housing, and both fingers from root to tip."""
    f = [np.column_stack([np.full(_FY.size, sx * (opening + 0.0045)), _FY.ravel(), _FZ.ravel()]) for sx in (-1.0, 1.0)]
    return np.vstack([PALM] + f)


def hand_points(env) -> np.ndarray:
    """Both grippers (palm, housing, fingers) and each wrist link down to the gripper, in the station frame. Off the simulator: none."""
    if env is None or not hasattr(env, "unwrapped"):
        return np.zeros((0, 3))
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        P = m._np(robot.data.body_link_pose_w)[0]
        g = P[robot.find_bodies("gripper")[0][0]]
        w = P[robot.find_bodies("link5")[0][0], :3] - origin
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        out.append(gripper_points(opening) @ m.quat_matrix(g[3:7]).T + g[:3] - origin)
        out.append(np.array([w + (g[:3] - origin - w) * t for t in np.linspace(0.0, 1.0, 6)]))
    return np.vstack(out)


def box_gap(pts: np.ndarray, lo, hi) -> float:
    """The smallest distance from any point to an axis-aligned box [lo, hi]; negative depth inside is read as 0."""
    if len(pts) == 0:
        return math.inf
    d = np.maximum(np.maximum(np.asarray(lo) - pts, pts - np.asarray(hi)), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


CASE_LO = np.array([PULL_X - PULL_R, CASE_Y[0], TOP])  # the case's outline, from the pull's and the lock head's face (the nearer, 34.4 cm) to its front
CASE_HI = np.array([CASE_X[1], CASE_Y[1], TOP + CASE_H])


def moment(c: dict) -> float:
    """The scoring text on one moment's facts."""
    if c["failed"] or not c["on_spot"]:
        return 0.0
    if not c["inner_shut"]:
        return 0.25
    if not c["outer_shut"]:
        return 0.5
    if c["latched"] and c["hands_off"]:
        return 1.0
    return 0.75


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.tipped = gk.Latch("the watch box tipped onto a side or was dropped")
        self.knocked = gk.Latch("a watch stand on the higher step was knocked over or moved")
        self.latched = False
        self.end = gk.EndHold(HOLD_STEPS)
        self.score = 0.0
        self.box_z: float | None = None  # the box's height one step before
        self.report: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        case, box = m.state(env, "showcase"), m.state(env, "box")
        qi, qo, qp = case.joint("inner"), case.joint("outer"), case.joint("plunger")
        inner_shut = qi <= -INNER_OPEN + SHUT
        outer_shut = qo >= OUTER_OPEN - SHUT
        # the catch: the plunger pushed to full travel with both doors at their stops (its tip then clears the inner door's edge) latches and
        # holds it in; latched, its tip stands beside the inner door's edge, so neither door can slide: both are held where they are
        if not self.latched and env is not None and hasattr(env, "unwrapped") and qp >= LATCH_AT and inner_shut and outer_shut:
            self.latched = True
            for link in ("plunger", "inner", "outer"):
                m.lock(env, "showcase", link, stiffness=20000.0, effort=400.0)
            print(f"[showcase] latched at step {self.steps}: plunger {1000 * qp:.1f} mm in, inner {1000 * (qi + INNER_OPEN):.1f} mm, outer "
                  f"{1000 * (OUTER_OPEN - qo):.1f} mm off their stops", flush=True)
        latched = self.latched and qp >= LATCH_AT  # read again each step from the plunger's place
        tilt = box.tilt_deg()
        fell = self.box_z is not None and self.box_z - float(box.pos[2]) > FALL_STEP
        self.box_z = float(box.pos[2])
        self.tipped.trip(tilt > TIPPED or fell)
        stands = [m.state(env, f"stand{k}") for k in range(len(STAND_YS))]
        moved = [float(np.hypot(st.pos[0] - STAND_X, st.pos[1] - y)) for st, y in zip(stands, STAND_YS)]
        self.knocked.trip(any(d > STAND_MOVED for d in moved) or any(st.tilt_deg() > STAND_TIPPED for st in stands))
        hp = hand_points(env)
        loc = box.to_local(hp) if len(hp) else hp
        let_go = box_gap(loc, (-BOX[0] / 2, -BOX[1] / 2, 0.0), (BOX[0] / 2, BOX[1] / 2, BOX[2])) >= LET_GO
        base_z = float(box.pos[2] - TOP)
        on_spot = bool(abs(box.pos[0] - SPOT[0]) <= ON_SPOT and abs(box.pos[1] - SPOT[1]) <= ON_SPOT and -0.003 <= base_z - FLOOR_Z <= 0.006
                       and tilt <= BOX_UPRIGHT and let_go)
        hands_off = box_gap(hp, CASE_LO, CASE_HI) >= HANDS_OFF
        c = {"failed": bool(self.tipped) or bool(self.knocked), "on_spot": on_spot, "inner_shut": inner_shut, "outer_shut": outer_shut, "latched": latched, "hands_off": hands_off}
        mo = moment(c)
        self.end.step(mo == 1.0)
        self.score = 1.0 if self.end.done else (0.75 if mo == 1.0 else mo)
        self.report = {"step": self.steps, "score": self.score, "moment": mo, "inner_mm": round(1000 * (qi + INNER_OPEN), 1),
                       "outer_mm": round(1000 * (OUTER_OPEN - qo), 1), "plunger_mm": round(1000 * qp, 1), "latched": latched,
                       "box_mm": np.round(1000 * (box.pos - [SPOT[0], SPOT[1], TOP + FLOOR_Z]), 1).tolist(), "tilt": round(tilt, 1),
                       "let_go": let_go, "hands_off": hands_off, "tipped": bool(self.tipped),
                       "knocked": bool(self.knocked), "stand_mm": [round(1000 * d, 1) for d in moved],
                       "stand_tilt": [round(st.tilt_deg(), 1) for st in stands]}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.end.done


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------------------------

BOX_TCP = 0.050  # the tool point this high above the box's base when held: the pads on its upper sides
BOX_SQUEEZE = 20.0  # N a finger on the box
CARRY = 0.120  # the box's base this high over the counter while carried (plus 2 cm): 5 cm over the riser (9.1 cm), level with the watch stands
INNER_PUSH_X = 0.397  # the left hand's closed fingers in the inner door's track, 6 mm clear of the outer glass
INNER_PUSH_Z = 0.210  # 6 cm higher than a bare deck needs: the hand's housing then passes over the watch stands behind the glass
TILT = 45.0  # the box grip, degrees from straight down
PUSH_TILT = 50.0  # the inner-door push
PULL_GRIP_Z = 0.245
PLUNGER_FORCE = 16.0  # N: above the spring's 10 N at full travel


def demo_script(ctx):
    """The left hand takes the watch box from above, carries it through the opening and sets it on its spot, lets go and lifts clear. With its
    fingers closed and pointing down, it then pushes the inner door's glass edge 6 cm to its stop, and draws back. The right hand takes the outer
    door's bar pull, slides the door 29 cm to its stop, lets go, and pushes the plunger's head in with its closed fingertips until it latches.
    Both hands draw back out of the case's outline.

    Check modes (TASK_DEMO): `knock`: the box carried in 7 cm off the gap, into a watch stand; `near_tier`: the box set down on the riser between the stands, the rest done; `drop_high`: the box let go 4 cm above the deck on its spot, the rest done;
    `doors_open`: the box on its spot, both doors left open; `no_lock`: all but the plunger; `inner_open`: the inner door is left open, the outer shut and the plunger pushed; `hand_on`: all done, the right
    fingertips left on the plunger's head."""
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode or "full"
    for _ in range(5):
        yield
    if mode == "none":
        yield from l.wait(3.0)
        return
    tg = d.tilted(TILT, 0.0)  # fingers pointing forward and down, jaws closing along y: the wrist trails low and back, under the top rail
    ax = tg.rot[:, 2]  # the approach axis
    bx = np.array([BOX_START[0], BOX_START[1], TOP + BOX_TCP])
    yield from l.move(bx - 0.10 * ax, grip=tg, finger=d.FINGER_OPEN, t=2.0, label="left: behind and over the box")
    yield from l.move(bx, t=1.2, label="left: round the box")
    yield from l.close(BOX[1], BOX_SQUEEZE, t=0.6, label="left: hold the box")
    yield from l.move(bx + [0.0, 0.0, CARRY + 0.02], t=1.0, label="left: lift")
    cz = TOP + CARRY + BOX_TCP + 0.02  # the box's base 14 cm up: over the riser (9.1 cm), level with the watch stands, between them
    gy = SPOT[1] - 0.07 if mode == "knock" else SPOT[1]  # `knock`: the box carried 7 cm off the gap, into the stand on its right
    px, floor = (STAND_X, FLOOR_Z + RISER_H) if mode == "near_tier" else (SPOT[0], FLOOR_Z)  # `near_tier`: set down on the riser, between the stands
    yield from l.move(np.array([0.26, BOX_START[1], cz]), t=1.0, label="left: back, clear of the case's corner post")
    yield from l.move(np.array([0.28, gy, cz]), t=1.5, label="left: to the opening, in line with the gap between the stands")
    yield from l.move(np.array([px, gy, cz]), t=2.5, label="left: in through the opening, between the stands")
    print(f"[showcase] in: {TRIAL.report}", flush=True)
    if gy != SPOT[1]:
        yield from l.move(np.array([px, SPOT[1], cz]), t=1.0, label="left: across to the spot")
    high = 0.04 if mode == "drop_high" else 0.0  # `drop_high`: let go with the box's base 4 cm above the deck
    yield from l.move(np.array([px, SPOT[1], TOP + floor + BOX_TCP + 0.01 + high]), t=1.5, label="left: down toward the spot")
    for _ in range(0 if high else 20):  # lower 1.5 mm a step until the box sits on the deck
        if float(ctx.state("box").pos[2] - TOP) - floor <= 0.002:
            break
        yield from l.move(l.at() - [0.0, 0.0, 0.0015], t=0.1, label="left: set the box down")
    yield from l.wait(0.3)
    yield from l.open(t=0.5, label="left: let go of the box")
    yield from l.move(l.at() - 0.06 * ax, t=0.8, label="left: back off the box along the fingers")
    yield from l.move(l.at(), finger=d.FINGER_CLOSED, t=0.4, label="left: close the fingers, narrow for the gap")
    yield from l.move(np.array([l.at()[0], SPOT[1], cz]), t=0.8, label="left: up to the gap's height")
    yield from l.move(np.array([0.28, SPOT[1], cz]), t=2.0, label="left: back out between the stands")
    if mode == "doors_open":
        yield from l.move(np.array([0.20, 0.30, TOP + 0.25]), t=1.5, label="left: clear")
        yield from l.wait(4.0)
        print(f"[showcase] {mode} end: {TRIAL.report}", flush=True)
        return
    if mode != "inner_open":
        pg = d.tilted(PUSH_TILT, -90.0)  # fingers pointing -y and down at the edge, jaws closing along x
        y0 = INNER_CLOSED_Y[1] + INNER_OPEN
        tip = 0.010  # the closed fingertips this far ahead of the tool point, along -y
        yield from l.move(np.array([0.28, y0 + tip + 0.04, TOP + INNER_PUSH_Z]), grip=pg, finger=d.FINGER_CLOSED, t=1.5, label="left: fingers closed, level")
        yield from l.move(np.array([INNER_PUSH_X, y0 + tip + 0.03, TOP + INNER_PUSH_Z]), t=1.2, label="left: in the inner door's track, beside its edge")
        yield from l.move(np.array([INNER_PUSH_X, INNER_CLOSED_Y[1] + tip - 0.003, TOP + INNER_PUSH_Z]), t=2.5, label="left: push the inner door shut")
        yield from l.wait(0.3)
        yield from l.move(np.array([INNER_PUSH_X, INNER_CLOSED_Y[1] + tip + 0.03, TOP + INNER_PUSH_Z]), t=0.6, label="left: off the edge")
        yield from l.move(np.array([0.28, INNER_CLOSED_Y[1] + tip + 0.05, TOP + INNER_PUSH_Z]), t=1.0, label="left: out")
    yield from l.move(np.array([0.20, 0.30, TOP + 0.25]), grip=tg, t=1.5, label="left: clear")
    lv = d.level(0.0)
    pz = TOP + PULL_GRIP_Z
    py0 = PULL_Y - OUTER_OPEN
    yield from r.move(np.array([PULL_X - 0.07, py0, pz]), grip=lv, finger=-0.020, t=2.0, label="right: to the pull")
    yield from r.move(np.array([PULL_X, py0, pz]), t=1.0, label="right: round the pull")
    yield from r.close(2 * PULL_R, 20.0, t=0.5, label="right: hold the pull")
    yield from r.move(np.array([PULL_X, PULL_Y + 0.004, pz]), t=3.5, label="right: slide the outer door shut")
    yield from r.wait(0.3)
    yield from r.move(r.at(), finger=-0.020, t=0.4, label="right: let go of the pull")
    yield from r.move(r.at() - [0.07, 0.0, 0.0], t=0.7, label="right: off the pull")
    if mode == "no_lock":
        yield from r.move(np.array([0.18, -0.25, TOP + 0.30]), t=1.5, label="right: clear")
        yield from r.wait(4.0)
        print(f"[showcase] {mode} end: {TRIAL.report}", flush=True)
        return
    hx = OUTER_X[0] - HOUSING_L - HEAD_L  # the head's face
    yield from r.move(np.array([hx - 0.045, LOCK_Y, TOP + LOCK_Z]), finger=d.FINGER_CLOSED, t=1.5, label="right: fingers closed, before the plunger")
    yield from r.move(np.array([hx - 0.016, LOCK_Y, TOP + LOCK_Z]), t=0.8, label="right: to the head")
    res = yield from r.press((1.0, 0.0, 0.0), PLUNGER_FORCE, 0.030, speed=0.02, settle=0.6, label="right: push the plunger in")
    print(f"[showcase] press {res}; {TRIAL.report}", flush=True)
    yield from r.wait(0.3)
    if mode == "hand_on":
        yield from r.wait(4.0)
        print(f"[showcase] {mode} end: {TRIAL.report}", flush=True)
        return
    yield from r.move(np.array([hx - 0.06, LOCK_Y, TOP + LOCK_Z]), t=0.8, label="right: off the plunger")
    yield from r.move(np.array([0.18, -0.25, TOP + 0.30]), t=1.5, label="right: clear")
    yield from r.wait(3.0)
    print(f"[showcase] {mode} end: {TRIAL.report}", flush=True)
