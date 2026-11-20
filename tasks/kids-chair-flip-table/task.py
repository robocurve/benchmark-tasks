"""Put a preschool chair up on the table: Flash Furniture Whitney YU-YCX-003 stackable school chair and a Community Playthings
Woodcrest rectangular table C241 on its 12 in legs.

Chair (Flash Furniture 2-pack listings 2-YU-YCX-003-*: webstaurantstore.com 354CX003GRN, Flash's Salsify product page
2-YU-YCX-003-RED-GG, quill.com 2YUYCX003BLUE): one moulded polypropylene shell with four splayed plastic legs, no metal;
12.5 in W x 14 in D x 20 in H overall, seat 10 x 10 in, seat height 10.75 in, back 11 in wide rising from the seat's rear edge.
NOT CONFIRMED: mass per chair (the 2-pack pages list "weight 6" with no unit and a 10 lb shipping weight; 1.4 kg used),
shell thickness (12 mm assumed), leg section (25 mm assumed).
Table (communityplaythings.com C241 page, dimension image c241_a881, product guide KX91): solid maple top 30 x 22 in,
1 1/8 in thick, heights 12/14/16/18/20 in (12 in = leg A881), no apron (from the maker's photos). NOT CONFIRMED: leg section
(45 mm assumed), table mass (fixed here).

The robot bench stands for the classroom floor; the table stands on it, its long side across the arms. The chair starts
upright on the floor in front of the table, facing the arms. Both hands pinch the seat at its two side edges, lift, turn it
over through 180 degrees about the line between the hands, and lower it seat down onto the tabletop with its back hanging
over the table's near edge, as classroom chairs are put up.
Debug:  python demokit.py task.py OUT --no-record --size 640x360 --stills 10 --mode full   (checks: --mode drop,none)
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = (
    "Put the child's chair up on the table: lift it from the floor with both hands, turn it over and set it down gently, seat "
    "down with its legs up, the seat's back edge on the table's near edge or up to 3 cm past it, without dropping it, "
    "knocking the table with it or sliding it along the top. Then let go and move your hands away.")
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
IN2M = 0.0254

# chair: Whitney YU-YCX-003
SEAT_Z, SEAT_T, SEAT_X, SEAT_Y = 10.75 * IN2M, 0.012, 10 * IN2M, 10 * IN2M
BACK_W, BACK_TOP, BACK_T = 11 * IN2M, 20 * IN2M, 0.012
FOOT_X, FOOT_Y, LEG_R = 14 * IN2M / 2 - 0.0125, 12.5 * IN2M / 2 - 0.0125, 0.0125
CHAIR_MASS = 1.4
# table: Woodcrest C241, 12 in
TABLE_X, TABLE_Y, TABLE_H, TOP_T, LEG = 22 * IN2M, 30 * IN2M, 12 * IN2M, 1.125 * IN2M, 0.045

X0 = float(os.environ.get("X0", "0.10"))                 # chair origin (floor point under the seat centre) at the start
TX = X0 + 7 * IN2M + 0.02                                 # the table's near edge

FINGER_T = 0.009                                          # a YAM finger's thickness across the jaw (check_score: a seat pinch's lower finger)
H_STOP = FINGER_T + 0.0005
IN = 0.018                                                # a seat-edge pinch (check_score's hand-built grips): the TCP this far in from the side edge

LOOKS = {"chair": m.Look(m.rgb(0.10, 0.35, 0.75), roughness=0.45), "wood": m.Look(m.rgb(0.85, 0.70, 0.50), roughness=0.55)}


def _table():
    parts, cols = [], []
    tc = (TABLE_X / 2, 0.0, TABLE_H - TOP_T / 2)
    parts.append(("top", m.box((TABLE_X, TABLE_Y, TOP_T), tc), "wood"))
    cols.append(m.col_box("top_c", (TABLE_X, TABLE_Y, TOP_T), tc))
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        c = (TABLE_X / 2 + sx * (TABLE_X / 2 - 0.05), sy * (TABLE_Y / 2 - 0.05), (TABLE_H - TOP_T) / 2)
        parts.append((f"leg{i}", m.box((LEG, LEG, TABLE_H - TOP_T), c), "wood"))
        cols.append(m.col_box(f"leg{i}_c", (LEG, LEG, TABLE_H - TOP_T), c))
    return m.fixed("woodcrest_table", (TX, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


def _chair():
    """Chair frame: origin on the floor under the seat centre, front at +x (turned 180 deg so it faces the arms)."""
    parts, cols = [], []
    sc = (0.0, 0.0, SEAT_Z - SEAT_T / 2)
    parts.append(("seat", m.box((SEAT_X, SEAT_Y, SEAT_T), sc), "chair"))
    cols.append(m.col_box("seat_c", (SEAT_X, SEAT_Y, SEAT_T), sc))
    bc = (-SEAT_X / 2 - BACK_T / 2, 0.0, (SEAT_Z - SEAT_T + BACK_TOP) / 2)
    parts.append(("back", m.box((BACK_T, BACK_W, BACK_TOP - SEAT_Z + SEAT_T), bc), "chair"))
    cols.append(m.col_box("back_c", (BACK_T, BACK_W, BACK_TOP - SEAT_Z + SEAT_T), bc))
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        top = (sx * (SEAT_X / 2 - 0.02), sy * (SEAT_Y / 2 - 0.02), SEAT_Z - SEAT_T)
        foot = (sx * FOOT_X, sy * FOOT_Y, LEG_R)
        leg = m.tube([foot, top], LEG_R, sides=10)
        parts.append((f"leg{i}", leg, "chair"))
        cols.append(m.col_hull(f"leg{i}_c", leg))
    return m.free("chair", (X0, 0.0, TOP + 0.002), parts, cols, LOOKS, mass=CHAIR_MASS, friction=(0.45, 0.35), yaw_deg=180.0)


OBJECTS = [_table(), _chair()]


# -- poses ----------------------------------------------------------------------------------------


def _R(q):
    from yam_arena.urdf import quat_xyzw_to_matrix
    return quat_xyzw_to_matrix(np.asarray(q, float))


def chair_pose(objects):
    s = objects["chair"]
    return np.asarray(s.pos, float), _R(s.quat_xyzw)


def landed(objects) -> dict:
    """Seat-down check from the chair's pose: the seat's up axis against world down, legs up, back over the near edge."""
    p, R = chair_pose(objects)
    up = R[:, 2]                                  # the chair's own up (seat top normal when upright)
    tilt = math.degrees(math.acos(float(np.clip(-up[2], -1, 1))))  # 0 when the seat faces straight down
    s = objects["chair"]
    low, high = np.asarray(s.low), np.asarray(s.high)
    pitch = math.degrees(math.atan2(up[0], -up[2]))     # about y: + when the seat's far side is low
    roll = math.degrees(math.atan2(up[1], -up[2]))      # about x
    return {"seat_down_tilt_deg": round(tilt, 1), "pitch_deg": round(pitch, 1), "roll_deg": round(roll, 1), "pos": [round(float(v), 3) for v in p], "legs_up": bool(up[2] < -0.95), "low_z_over_table": round(float(low[2] - TOP), 3),
            "yaw_deg": round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 1)}


# -- scorer ---------------------------------------------------------------------------------------------------------------------------------------

SURF = TOP + TABLE_H                      # the tabletop's face
TOL_DOWN = 5.0                            # seat-down: the seat face within 5 degrees of straight down
EDGE_TOL = 0.03                           # the seat's rear edge on the table's near edge line or up to 3 cm past it
CLEAR = 0.03                              # every fingertip this far from the chair's surface
LINK_CLEAR = 0.05                         # and every arm link's axis (upper arm, forearm, wrist, palm) this far from it: about a link's half-width
CONTACT_AIR = 0.002                       # a two-hand hold counts only with each pad on the chair (within 2 mm of its surface; the honest grip reads 1.5 mm or less), not hovering within 3 mm
BOTH_MIN = 0.1                            # ... and held by both hands in contact for this long without a break
TURN_BAND = (35.0, 145.0)                 # the seat's face this far from down: the turn proper
TURN_GAP = 0.1                            # in the turn, at most this long in all without both hands in contact
SOLO_MAX = 0.1                            # lifted off the floor or the top: both hands hold it within this long of its leaving them
LIFT_CLEAR = 0.01                         # a set-down ends when the seat is lifted this far clear of the top: the next one is a new try
LIFT_MIN = 0.08                           # lifted: the chair's lowest point this far off the floor
AIR = 0.015                               # trace only: a part of the chair within this of the floor or the top
SUP = 0.0015                              # a part of the chair within 1.5 mm of the top is on it (contact)
SUP_FLOOR = 0.005                         # and within 5 mm of the floor: a splayed leg's end stands on the rim of its cut face, a few mm under its centre
FALL_MAX = 0.012                          # dropped: no hand holds it, no part of it is on the top or the floor, and it falls more than 12 mm
V_TOUCH = 0.5                             # set down too hard: a part of it reaches the top or the floor moving down faster than 0.5 m/s
TOL_T = 0.003                             # struck the table: a part's surface within 3 mm of the tabletop slab (its face or its edges)
PLACE_DEG = 30.0                          # the seat's face may touch the top only while it faces down within 30 degrees: the placement
SLIDE_MAX = 0.03                          # the seat may slide this far along the top after touching it
HOLD_STEPS = 40                           # 2 s at 20 Hz
SETTLE_STEPS = 20
TRACE = os.environ.get("CHAIR_TRACE", "0") != "0"
SEAT_GRIP = gk.Grip(half=SEAT_T / 2, across="z", centre=(0.0, 0.0, SEAT_Z - SEAT_T / 2), lo=(-0.14, -0.14, -math.inf), hi=(0.14, 0.14, math.inf))
# key points of the chair, in its frame: the seat's face corners, the four feet, the back's two top corners
SEAT_FACE = np.array([[sx * SEAT_X / 2, sy * SEAT_Y / 2, SEAT_Z] for sx in (-1, 1) for sy in (-1, 1)])
FEET = np.array([[sx * FOOT_X, sy * FOOT_Y, LEG_R] for sx in (-1, 1) for sy in (-1, 1)])
BACK_TIPS = np.array([[-SEAT_X / 2 - BACK_T / 2, sy * BACK_W / 2, BACK_TOP] for sy in (-1, 1)])
KEY = np.vstack([SEAT_FACE, SEAT_FACE - [0, 0, SEAT_T], FEET, BACK_TIPS])
REAR_MID = np.array([-SEAT_X / 2, 0.0, SEAT_Z])
REAR_C = np.array([[-SEAT_X / 2, sy * SEAT_Y / 2, SEAT_Z] for sy in (-1, 1)])   # the seat face's two rear corners


def _over_top(p) -> np.ndarray:
    p = np.atleast_2d(p)
    return (p[:, 0] >= TX) & (p[:, 0] <= TX + TABLE_X) & (np.abs(p[:, 1]) <= TABLE_Y / 2)


def seat_down_deg(chair) -> float:
    """How far the seat's face is from straight down, in degrees (0: seat flat, legs up; 180: upright)."""
    return math.degrees(math.acos(float(np.clip(-chair.rotation()[2, 2], -1.0, 1.0))))


SEAT_GRID = np.array([[x, y, SEAT_Z] for x in np.linspace(-SEAT_X / 2, SEAT_X / 2, 9) for y in np.linspace(-SEAT_Y / 2, SEAT_Y / 2, 9)])


def resting(chair) -> bool:
    """Some part of the chair is within AIR of the floor, or the seat's face is within AIR of the top (over it, or at its edge)."""
    w = chair.to_world(KEY)
    sup = np.where(_over_top(w) & (w[:, 2] > SURF - 0.05), SURF, TOP)
    if bool(np.any(w[:, 2] - sup < AIR)):
        return True
    g = chair.to_world(SEAT_GRID)
    near = (g[:, 0] >= TX - 0.005) & (g[:, 0] <= TX + TABLE_X) & (np.abs(g[:, 1]) <= TABLE_Y / 2)
    return bool(np.any(near & (g[:, 2] - SURF < AIR)))


def on_top(chair) -> bool:
    """Seat down on the tabletop: seat's face within TOL_DOWN of straight down, its centre over the top, the corners over the top within a few mm of the face."""
    if seat_down_deg(chair) > TOL_DOWN or not bool(_over_top(chair.pos)[0]):
        return False
    w = chair.to_world(SEAT_FACE)
    over = w[_over_top(w)]
    return bool(len(over) >= 2 and np.all((over[:, 2] - SURF > -0.004) & (over[:, 2] - SURF < 0.026)))   # 26 mm: a seat 25.4 cm deep tilted 5 degrees, plus 4 mm


def edge_gap(chair) -> float:
    """The seat's rear edge (where the back rises) from the table's near edge line, along x: negative past the edge."""
    return float(chair.to_world(REAR_MID)[0, 0] - TX)


def rear_on_edge(chair) -> bool:
    """The seat's whole rear edge on the table's near edge line or up to 3 cm past it: both rear corners of the seat's face in that band (so the
    rear edge lies along the near edge, within 7 degrees), and both within the table's width (not past its side edges)."""
    w = chair.to_world(REAR_C)
    g = w[:, 0] - TX
    return bool(np.all((g >= -EDGE_TOL) & (g <= 0.0)) and np.all(np.abs(w[:, 1]) <= TABLE_Y / 2))


_SEAT_BOX = (np.array([-SEAT_X / 2, -SEAT_Y / 2, SEAT_Z - SEAT_T]), np.array([SEAT_X / 2, SEAT_Y / 2, SEAT_Z]))
_BACK_BOX = (np.array([-SEAT_X / 2 - BACK_T, -BACK_W / 2, SEAT_Z - SEAT_T]), np.array([-SEAT_X / 2, BACK_W / 2, BACK_TOP]))


def surface_dist(chair, points) -> float:
    """The least distance from world points to the chair's whole surface: the seat plate and the back plate as solid boxes, each leg as a tube
    of its radius along its axis (0 or less: on or in it)."""
    q = chair.to_local(np.atleast_2d(np.asarray(points, float)))
    if not len(q):
        return math.inf
    d = [np.linalg.norm(np.maximum(np.maximum(lo - q, 0.0), q - hi), axis=1) for lo, hi in (_SEAT_BOX, _BACK_BOX)]
    for top, foot in LEG_ENDS:
        s = foot - top
        u = np.clip((q - top) @ s / float(s @ s), 0.0, 1.0)
        d.append(np.linalg.norm(q - (top + u[:, None] * s), axis=1) - LEG_R)
    return float(np.min(d))


ARM_CHAIN = ("link2", "link3", "link4", "link5", "gripper")


def arm_points(env) -> np.ndarray:
    """Points along both arms' links (station frame): the axes from link2 through the wrist to the gripper, and the palm up to the fingers' base."""
    if env is None:
        return np.zeros((0, 3))
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    pts = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        P = m._np(robot.data.body_link_pose_w)[0]
        chain = [P[robot.find_bodies(n)[0][0], :3] - origin for n in ARM_CHAIN]
        g = P[robot.find_bodies("gripper")[0][0]]
        chain.append(g[:3] - origin + m.quat_matrix(g[3:7]) @ np.array([0.0, 0.0, 0.10]))
        for a, b in zip(chain[:-1], chain[1:]):
            pts += [a + (b - a) * t for t in np.linspace(0.0, 1.0, 6)]
    return np.array(pts)


LADDER = gk.Ladder(("lifted", 0.25), ("turned", 0.5), ("on_top", 0.75), ("placed", 1.0))

# Every part of the chair as points in its own frame, each with the radius of the part around it: the seat plate (its faces and edges, from its
# middle plane), each leg along its whole length from the foot to its top under the seat, and the back plate from the seat's face to its top edge.
_STEP = 0.005                             # part points every 5 mm or less, each with its part's radius: a touch between points reads within 1 mm of its gap
_n = lambda L: int(math.ceil(L / _STEP)) + 1
SEAT_PTS = np.array([[x, y, SEAT_Z - SEAT_T / 2] for x in np.linspace(-SEAT_X / 2, SEAT_X / 2, _n(SEAT_X)) for y in np.linspace(-SEAT_Y / 2, SEAT_Y / 2, _n(SEAT_Y))])
LEG_PTS = np.vstack([[foot + (top - foot) * u for u in np.linspace(0.0, 1.0 - LEG_R / np.linalg.norm(foot - top), _n(np.linalg.norm(foot - top)))] for top, foot in   # to a radius short of its top, where the seat plate takes over
                     [(np.array([np.sign(f[0]) * (SEAT_X / 2 - 0.02), np.sign(f[1]) * (SEAT_Y / 2 - 0.02), SEAT_Z - SEAT_T]), f) for f in FEET]])
BACK_PTS = np.array([[-SEAT_X / 2 - BACK_T / 2, y, z] for y in np.linspace(-BACK_W / 2, BACK_W / 2, _n(BACK_W)) for z in np.linspace(SEAT_Z, BACK_TOP, _n(BACK_TOP - SEAT_Z))])
PARTS = [("seat", SEAT_PTS, SEAT_T / 2), ("leg", LEG_PTS, LEG_R), ("back", BACK_PTS, BACK_T / 2)]
ALL_PTS = np.vstack([p for _, p, _ in PARTS])
ALL_R = np.concatenate([np.full(len(p), r) for _, p, r in PARTS])
N_FACE = len(SEAT_PTS)
PART_OF = np.concatenate([[n] * len(p) for n, p, _ in PARTS])
IS_BACK = PART_OF == "back"
SLAB_LO, SLAB_HI = np.array([TX, -TABLE_Y / 2, SURF - TOP_T]), np.array([TX + TABLE_X, TABLE_Y / 2, SURF])   # the tabletop slab
# the chair's centre of mass in its own frame, from its parts at one density (seat and back plates, four legs): 5.1 cm behind the seat's middle
COM = np.array([-0.051, 0.0, 0.278])


class _LegView:
    """One leg as a part of its own for the hold test: points in the leg's frame (its axis z, from its top under the seat to its foot)."""

    def __init__(self, chair, top, foot):
        self.chair, self.top = chair, top
        u = (foot - top) / np.linalg.norm(foot - top)
        x = np.cross([0.0, 1.0, 0.0], u)
        x /= np.linalg.norm(x)
        self.R = np.column_stack([x, np.cross(u, x), u])   # the leg frame's axes in the chair's frame

    def to_local(self, points):
        return (self.chair.to_local(points) - self.top) @ self.R


def _leg_ends():
    return [(np.array([sx * (SEAT_X / 2 - 0.02), sy * (SEAT_Y / 2 - 0.02), SEAT_Z - SEAT_T]), np.array([sx * FOOT_X, sy * FOOT_Y, LEG_R]))
            for sx in (-1, 1) for sy in (-1, 1)]


LEG_ENDS = _leg_ends()
_LEG_LEN = float(np.linalg.norm(LEG_ENDS[0][1] - LEG_ENDS[0][0]))
# a leg pinched across its width, either way round it (the fingertip points spread 3 cm along the fingers, so the leg's round section is tested as
# its two flat faces across the jaws, each pad within 3 mm of the leg's surface)
LEG_GRIPS = (gk.Grip(half=LEG_R, across="x", lo=(-math.inf, -0.035, 0.0), hi=(math.inf, 0.035, _LEG_LEN)),
             gk.Grip(half=LEG_R, across="y", lo=(-0.035, -math.inf, 0.0), hi=(0.035, math.inf, _LEG_LEN)))


BACK_X = -SEAT_X / 2 - BACK_T / 2
BACK_GRIP = gk.Grip(half=BACK_T / 2, across="x", centre=(BACK_X, 0.0, (SEAT_Z + BACK_TOP) / 2), lo=(-math.inf, -0.16, SEAT_Z), hi=(math.inf, 0.16, BACK_TOP + 0.005))


def _tight(g):
    return dataclasses.replace(g, air=CONTACT_AIR)


_SEAT_C, _BACK_C = _tight(SEAT_GRIP), _tight(BACK_GRIP)
_LEG_C = tuple(_tight(g) for g in LEG_GRIPS)


def held_by(tips, chair, contact: bool = False) -> tuple:
    """The hands that hold the chair now: each hand's two fingers closed on opposite faces of the seat plate or of the back plate, or round one
    leg (gatekit's grasp test)."""
    sg, bg, lgs = (_SEAT_C, _BACK_C, _LEG_C) if contact else (SEAT_GRIP, BACK_GRIP, LEG_GRIPS)
    hands = set(gk.holding(tips, chair, sg, resting=False)) | set(gk.holding(tips, chair, bg, resting=False))
    for top, foot in LEG_ENDS:
        for g in lgs:
            hands |= set(gk.holding(tips, _LegView(chair, top, foot), g, resting=False))
    if contact:   # and each finger's nearest point (its pad, the point less PAD_INSET) must be within CONTACT_AIR of the chair's surface: a hand closed beside it does not hold it
        t = gk.all_tips(tips)
        hands = {h for h in hands if all(surface_dist(chair, t[gk.HANDS[h]][a]) - gk.PAD_INSET <= CONTACT_AIR for a in (slice(0, 6), slice(6, 12)))}
    return tuple(h for h in ("left", "right") if h in hands)


_TLEG = [(np.array([TX + TABLE_X / 2 + sx * (TABLE_X / 2 - 0.05) - LEG / 2, sy * (TABLE_Y / 2 - 0.05) - LEG / 2, TOP]),
          np.array([TX + TABLE_X / 2 + sx * (TABLE_X / 2 - 0.05) + LEG / 2, sy * (TABLE_Y / 2 - 0.05) + LEG / 2, SURF - TOP_T])) for sx in (1, -1) for sy in (1, -1)]


def to_legs(w) -> np.ndarray:
    """Each world point's distance to the nearest of the table's four legs."""
    return np.min([np.linalg.norm(np.maximum(np.maximum(lo - w, 0.0), w - hi), axis=1) for lo, hi in _TLEG], axis=0)


def to_slab(w) -> np.ndarray:
    """Each world point's distance to the tabletop slab (its face, its edges, its underside): 0 inside it."""
    return np.linalg.norm(np.maximum(np.maximum(SLAB_LO - w, 0.0), w - SLAB_HI), axis=1)


def gaps(chair):
    """Each part point's surface gap to the tabletop slab and to the floor (negative: pressed in)."""
    w = chair.to_world(ALL_PTS)
    return w, to_slab(w) - ALL_R, w[:, 2] - TOP - ALL_R


def point_v(chair, w) -> np.ndarray:
    """Each world point's velocity: the body's centre-of-mass velocity plus its turn rate across the lever from the centre of mass."""
    c = chair.to_world(COM[None, :])[0]
    return np.asarray(chair.lin_vel, float)[None, :] + np.cross(np.asarray(chair.ang_vel, float), w - c)


def point_vz(chair, w) -> np.ndarray:
    return point_v(chair, w)[:, 2]


def into_slab(w, v) -> np.ndarray:
    """Each point's speed toward the tabletop slab, along the line to its nearest point on the slab."""
    n = np.clip(w, SLAB_LO, SLAB_HI) - w
    L = np.linalg.norm(n, axis=1)
    return np.where(L > 1e-6, np.einsum("nk,nk->n", v, n) / np.maximum(L, 1e-6), 0.0)


class Trial:
    """The trial's record: lifted, turned, on the top, placed, each read from the state now (the end of the run is what is scored); and the
    latched failures (a failure stays failed until the next reset). Partial credit is the end state's: a chair lifted and turned, then put back
    on the floor, scores 0.

    The chair's motion is read after every physics step (240 Hz in the harness), with the hands' grasp read from the fingertips at that step:
    lifted without both hands (off the floor or the top, no part within 15 mm of either, and not held by both hands in its first 0.1 s in the air), dropped (an
    unsupported fall of more than 12 mm), set down too hard (any part reaching the top or the floor faster than 0.5 m/s), struck the table (any
    point along a leg's whole length or on the back, or on the seat plate while its face is more than 30 degrees from down, within 3 mm of
    the tabletop slab while the chair is held or moving; the back in front of the near edge only while the seat faces more than 30
    degrees from down, or arriving there faster than 0.5 m/s). The ladder, the drag along the top and the end hold are read once per control step."""

    def __init__(self):
        self.dropped, self.hard, self.struck, self.dragged, self.onehand = (gk.Latch(w) for w in ("chair dropped", "set down too hard", "chair struck the table", "chair dragged along the top", "lifted without both hands"))
        self.settle, self.hold = gk.EndHold(SETTLE_STEPS), gk.EndHold(HOLD_STEPS)
        self.env, self.hooked, self.armed = None, False, False
        self.reset()

    def reset(self, env=None):
        self.n = self.phys_n = self.phys_per_step = 0
        self.was_held = self.rest_top = False
        self.hands = ()
        self.touch0 = self.fall_ref = self.prev_on = self.prev_vz = self.prev_near = self.prev_vin = None
        self.fall = self.vz_peak = self.touch_v = self.solo_t = self.both_t = self.turn_gap = 0.0
        self.lift_both = False
        self.why = {}
        self.last = {}
        for latch in self.latches():
            latch.reset()
        self.settle.reset()
        self.hold.reset()
        if env is not None:
            self.env = env
            self._hook(env)
            self.armed = True

    def _hook(self, env):
        """Run `motion` after every physics step: the harness steps physics 12 times per control step and updates the scene's data after each."""
        scene = env.unwrapped.scene
        if getattr(scene, "_chair_hooked", False):
            self.hooked = True
            return
        update = scene.update

        def updated(*a, **k):
            out = update(*a, **k)
            if TRIAL.armed and TRIAL.env is not None:
                TRIAL.phys_n += 1
                TRIAL.motion(m.state(TRIAL.env, "chair"), m.fingertips(TRIAL.env))
            return out

        scene.update = updated
        scene._chair_hooked = True
        self.hooked = True

    def latches(self):
        return (self.dropped, self.hard, self.struck, self.dragged, self.onehand)

    def motion(self, chair, tips):
        """One physics step's read of the chair (in check_score, one control step's)."""
        dt = d.DT / 12 if (self.hooked and self.armed) else d.DT
        hands = held_by(tips, chair)
        self.hands = hands
        self.was_held |= bool(hands)
        w, slab, floor = gaps(chair)
        on = (slab < SUP) | (floor < SUP_FLOOR)
        self.rest_top = bool((slab < SUP).any() and not (floor < SUP_FLOOR).any())
        v = point_v(chair, w)
        vz = v[:, 2]
        # lifted without both hands: each time the chair leaves the floor or the top (no part within 15 mm of either), both hands must hold it
        # within its first 0.1 s in the air, as the instruction says ("lift it ... with both hands"); a hand may let go and re-grip once it is up
        both_now = len(held_by(tips, chair, contact=True)) == 2
        self.both_t = self.both_t + dt if both_now else 0.0
        aloft = not bool(((slab < AIR) | (floor < AIR)).any())
        if aloft:
            self.solo_t += dt
            self.lift_both |= self.both_t >= BOTH_MIN
        else:
            self.solo_t, self.lift_both = 0.0, False
        bad_lift = aloft and self.solo_t > SOLO_MAX and not self.lift_both
        # from lift-off to the end of the turn (the seat's face within TURN_BAND[0] of down) both hands must hold it (in contact) while nothing of the chair rests on the floor or the top; a break of more than TURN_GAP in all latches
        sd = seat_down_deg(chair)
        if not on.any() and sd > TURN_BAND[0] and not both_now:
            self.turn_gap += dt
        # turned over on the floor: the seat's face passes TURN_BAND[1] on its way from up toward down while any part of the chair is within 5 mm of the floor
        floor_turn = bool((floor < SUP_FLOOR).any()) and sd < TURN_BAND[1]
        bad_lift = bad_lift or self.turn_gap > TURN_GAP or floor_turn
        if bad_lift and not self.onehand:
            print(f"[chair] lifted without both hands: held by {hands}, {self.solo_t:.2f} s in the air (phys {self.phys_n})", flush=True)
        self.onehand.trip(bad_lift)
        # set down too hard: a part arrives on the top or the floor (outside 1.5 mm at the last read, inside now) moving down faster than V_TOUCH
        if self.prev_on is not None:
            arrive = on & ~self.prev_on
            if arrive.any():
                va = float(max(-vz[arrive].min(), -self.prev_vz[arrive].min()))
                self.touch_v = max(self.touch_v, va)
                self.hard.trip(self.was_held and va > V_TOUCH)
        self.prev_on, self.prev_vz = on, vz
        # dropped: no hand holds it and no part of it is on the top or the floor; the fall is read from where its centre of mass was at that moment
        c = float(chair.to_world(COM[None, :])[0, 2])
        if self.was_held and not hands and not on.any():
            if self.fall_ref is None:
                self.fall_ref = c
            self.fall = max(self.fall, self.fall_ref - c)
            self.dropped.trip(self.fall_ref - c > FALL_MAX)
        else:
            self.fall_ref = None
        # struck the table: any part within 3 mm of the tabletop slab while the chair is held or moving, but the seat plate while its face is down
        # within 30 degrees (the placing), and the back in front of the near edge while the seat faces down within 30 degrees (where it hangs once
        # placed) unless it arrives there faster than V_TOUCH
        moving = float(np.linalg.norm(chair.lin_vel)) > 0.02 or float(np.linalg.norm(chair.ang_vel)) > 0.2
        near = slab < TOL_T
        hit = near.copy()
        front = IS_BACK & (w[:, 0] < TX)
        vin = into_slab(w, v)
        knock = False
        if seat_down_deg(chair) <= PLACE_DEG:
            hit[:N_FACE] = False
            hit[front] = False
            if self.prev_near is not None:
                arr = front & near & ~self.prev_near
                knock = bool(arr.any() and max(float(vin[arr].max()), float(self.prev_vin[arr].max())) > V_TOUCH)
        self.prev_near, self.prev_vin = near, vin
        lg = to_legs(w) - ALL_R
        hit |= lg < TOL_T                                   # any part against a table leg
        bad = bool((hands or moving) and (hit.any() or knock))
        if bad and not self.struck:
            k = int(np.argmax(hit)) if hit.any() else int(np.argmin(np.where(front & near, slab, np.inf)))
            self.why = {"struck": PART_OF[k], "point": [round(float(x), 3) for x in w[k]], "gap_mm": round(1000 * float(min(slab[k], lg[k])), 1), "table_leg": bool(lg[k] < slab[k]), "into_mps": round(float(vin[k]), 3), "knock": knock, "phys": self.phys_n}
            print(f"[chair] struck the table: {self.why}", flush=True)
        self.struck.trip(bad)

    def step(self, env):
        self.n += 1
        chair = m.state(env, "chair")
        tips = m.fingertips(env)
        if not (self.hooked and self.armed):     # no physics-step hook (check_score's hand-built trials): read the motion once per control step
            self.motion(chair, tips)
        hands = self.hands
        both = len(hands) == 2
        w = chair.to_world(KEY)
        down = seat_down_deg(chair)
        # dragged: the seat's centre more than 3 cm, while the seat is on the top (a corner within 4 mm of it), from where this set-down began;
        # lifting the seat 1 cm clear of the top ends the set-down, and the next touch starts a new one
        seat = chair.to_world(SEAT_FACE)
        over = _over_top(seat)
        centre = chair.to_world(np.array([[0.0, 0.0, SEAT_Z]]))[0]
        if np.any(over & (seat[:, 2] < SURF + 0.004)):
            if self.touch0 is None:
                self.touch0 = centre[:2].copy()
            self.dragged.trip(float(np.linalg.norm(centre[:2] - self.touch0)) > SLIDE_MAX)
        elif not np.any(over & (seat[:, 2] < SURF + LIFT_CLEAR)):
            self.touch0 = None
        still = float(np.linalg.norm(chair.lin_vel)) < 0.01 and float(np.linalg.norm(chair.ang_vel)) < 0.1
        landed = on_top(chair) and not hands and still
        tip_gap = surface_dist(chair, tips)
        link_gap = surface_dist(chair, arm_points(env))
        clear = tip_gap > CLEAR and link_gap > LINK_CLEAR
        edge = rear_on_edge(chair)
        self.settle.step(landed)
        self.hold.step(landed and edge and clear)
        # the tiers are read from the state now (the end of the run is what is scored); only the failures latch
        up_now = (both and float(w[:, 2].min()) - TOP >= LIFT_MIN) or self.rest_top     # held 8 cm up by both hands, or resting on the top, off the floor
        on_now = self.settle.count >= SETTLE_STEPS
        self.last = {"lifted": up_now, "turned": up_now and down <= PLACE_DEG, "on_top": on_now, "placed": on_now and self.hold.count >= HOLD_STEPS}
        if TRACE and self.n % 10 == 0:
            print(f"[chair] step {self.n}: held by {hands}, low {1000 * (float(w[:, 2].min()) - TOP):.0f} mm, seat {down:.1f} deg from down, edge gap {1000 * edge_gap(chair):.0f} mm, still {still}, clear {clear}, "
                  f"nearest fingertip {1000 * tip_gap:.0f} mm, nearest link axis {1000 * link_gap:.0f} mm, aloft {self.solo_t:.2f} s (both {self.lift_both}), fall {1000 * self.fall:.0f} mm, touch {self.touch_v:.2f} m/s, phys/step {self.phys_n}, "
                  f"onehand {bool(self.onehand)}, dropped {bool(self.dropped)}, hard {bool(self.hard)}, struck {bool(self.struck)}, dragged {bool(self.dragged)}, settle {self.settle.count}, hold {self.hold.count}; score {self.score()}", flush=True)

    def score(self) -> float:
        failed = any(bool(x) for x in self.latches())
        return LADDER.score(self.last, failed=failed) if self.last else 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


def score(objects) -> float:
    return TRIAL.score()


# -- demo -----------------------------------------------------------------------------------------
#
# (The old whole-path demo, GRIP=leg; the default is the split demo below: seat pinch for the turn, then the back plate's side edges.) Both hands pinch the chair's two rear legs, a quarter of the way down from the seat, with the fingers pointing in from the sides and the jaws
# closing across each leg (the pads lie along it, so the leg cannot tip between them). The chair is lifted, turned over through half a turn about
# the line between the hands, carried over the table and lowered seat down by feedback on its observed height until the seat rests on the top.
# Legs up, the grips are 7 cm over the top, so the gripper bodies stay clear of it; the jaws open with the chair already resting on its seat.

GRIP = os.environ.get("GRIP", "split")                     # "split": seat pinch for the turn, then the rear legs; "leg": the two rear legs; "back": the back plate's side edges
H_BACK = float(os.environ.get("H", "0.10"))                # back grip: this far above the seat's face
U_GRIP = float(os.environ.get("U_GRIP", "0.25"))          # the grip's place along a rear leg, from the seat (0) to the foot (1)
D_DEMO = int(os.environ.get("DIR", "-1"))                 # -1: the chair's top turns toward the arms, away from the table
XT2, ZR2, ZHI2 = (float(os.environ.get(k, v)) for k, v in (("XT", "0.13"), ("ZR", "0.30"), ("ZHI", "0.46")))   # the grip line during the turn (station frame, z above the bench)
EDGE_T = float(os.environ.get("EDGE_T", "0.015"))          # landed: the seat's rear edge this far outside the table's near edge line
ZL = float(os.environ.get("ZL", "0.03"))                   # the open-loop lowering ends with the seat this far over the top; feedback takes it on
FINE_DZ = 0.0025
V_FAST, V_SLOW, V_ZONE = 0.04, 0.01, 0.02                 # lowering speed: 4 cm/s, then 1 cm/s for the last 2 cm
BETA = float(os.environ.get("BETA", "0"))                  # the fingers' tilt in the pads' plane (deg)
GRIP_FORCE = float(os.environ.get("FORCE", "35"))
SEEDS = [[j1, 1.0, 1.0, j4, j5, j6] for j1 in (-0.3, 0.3) for j4 in (-1.5, -0.6, 0.6, 1.5) for j5 in (-1.2, 1.2) for j6 in (-2.0, -1.0, 1.0, 2.0)]


def _Ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rear_leg(sy):
    """A rear leg's top (2 cm in from the seat's corner, under the seat) and foot, in the chair's frame; sy = +1 or -1 (the chair's left or right)."""
    return np.array([-(SEAT_X / 2 - 0.02), sy * (SEAT_Y / 2 - 0.02), SEAT_Z - SEAT_T]), np.array([-FOOT_X, sy * FOOT_Y, LEG_R])


def _grip_frames(p, R):
    """Each hand's grip point and hand rotation on the chair standing at (p, R): the left hand takes the rear leg at world +y."""
    out = {}
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        if GRIP == "back":                                  # the back plate's side edge, H_BACK above the seat's face, the TCP 16 mm in
            ys = [s * (BACK_W / 2 - 0.016) for s in (1, -1)]
            y = max(ys, key=lambda v: sgn * float((p + R @ np.array([BACK_X, v, 0.0]))[1]))
            c = p + R @ np.array([BACK_X, y, SEAT_Z + H_BACK])
            j = R @ np.array([1.0, 0.0, 0.0])
        else:
            tops = [rear_leg(s) for s in (1, -1)]
            top, foot = max(tops, key=lambda tf: sgn * float((p + R @ tf[0])[1]))
            c = p + R @ (top + U_GRIP * (foot - top))
            l = R @ (foot - top)
            l /= np.linalg.norm(l)
            j = np.array([-l[2], 0.0, l[0]])
            j /= np.linalg.norm(j)
        b = math.radians(BETA)                              # the approach turned about the jaw axis, in the pads' plane: the leg stays along the pads
        a0 = np.array([0.0, -sgn, 0.0])
        a = a0 * math.cos(b) + sgn * np.cross(j, a0) * math.sin(b)   # BETA > 0: the fingers point down and in, for either hand
        out[side] = (c, j, a, sgn)
    return out


def _hand_G(j, a, jaw):
    jj = jaw * j
    return np.column_stack([jj, np.cross(a, jj), a])


def _waypoints(p, R):
    """The grip line's waypoints (A, theta): lift straight up to ZR, turn through half a turn while the line goes to (XT, ZHI), carry over the table, lower."""
    g = _grip_frames(p, R)
    A0 = 0.5 * (g["left"][0] + g["right"][0])
    face0 = p + R @ np.array([0.0, 0.0, SEAT_Z])
    rear0 = p + R @ np.array([-SEAT_X / 2, 0.0, SEAT_Z])
    z_land = SURF + (face0[2] - A0[2])                     # turned over, the seat's face is on the top
    x_land = TX - EDGE_T + (rear0[0] - A0[0])              # and its rear edge EDGE_T outside the near edge
    a1 = np.array([A0[0], A0[1], ZR2 + TOP])               # straight up to ZR, then the turn
    a2 = np.array([XT2, A0[1], ZHI2 + TOP])
    a3 = np.array([x_land, A0[1], a2[2]])
    a4 = np.array([x_land, A0[1], z_land + ZL])
    wps = [(A0, 0.0)] + [(A0 + (a1 - A0) * k / 6, 0.0) for k in range(1, 7)]
    for k in range(1, 25):
        f = k / 24
        s = f * f * (3 - 2 * f)
        wps.append((a1 + (a2 - a1) * s, D_DEMO * math.pi * s))
    wps += [(a2 + (a3 - a2) * k / 8, D_DEMO * math.pi) for k in range(1, 9)]
    wps += [(a3 + (a4 - a3) * k / 8, D_DEMO * math.pi) for k in range(1, 9)]
    n_fine = int(round((ZL + 0.03) / FINE_DZ))             # the fine path: on down to 3 cm under the landing (the arms sag under the load)
    wps += [(a4 - [0.0, 0.0, FINE_DZ * k], D_DEMO * math.pi) for k in range(1, n_fine + 1)]
    return wps, g, A0, z_land


def path_clearance(p, R, wps, sub=4):
    """The chair's every part point along the planned path (each step cut in `sub`): its least surface gap to the tabletop slab while the seat faces
    more than 30 degrees from down (or any part but the seat's face), and to the floor after the lift. No simulator."""
    _, _, A0, _ = _waypoints(p, R)
    P0 = (R @ ALL_PTS.T).T + p
    worst_t, worst_f, where = 9.0, 9.0, None
    for (Aa, ta), (Ab, tb) in zip(wps[6:46], wps[7:47]):
        for u in np.linspace(0, 1, sub, endpoint=False):
            A, th = Aa + (Ab - Aa) * u, ta + (tb - ta) * u
            w = A + ((P0 - A0) @ _Ry(th).T)
            gt = to_slab(w) - ALL_R
            down = math.degrees(math.acos(float(np.clip(-(_Ry(th) @ R)[2, 2], -1, 1))))
            if down <= PLACE_DEG:
                gt[:N_FACE] = 9.0
                gt[IS_BACK & (w[:, 0] < TX)] = 9.0
            gf = w[:, 2] - TOP - ALL_R
            k = int(np.argmin(gt))
            if gt[k] < worst_t:
                worst_t, where = float(gt[k]), (PART_OF[k], round(math.degrees(th), 1), [round(float(v), 3) for v in w[k]])
            worst_f = min(worst_f, float(gf.min()))
    return worst_t, worst_f, where


def _solve_path(side, poses):
    """One arm along the whole path of (position, rotation) poses, from several seeds, each solve seeded by the last; the best branch."""
    from yam_arena import ik, station
    base = np.asarray(station.BASES[side], float)
    starts = []
    for s in SEEDS:
        q, pe, re_ = ik.solve(poses[0][0], ik.matrix_to_quat_xyzw(poses[0][1]), np.array(s), base_pos=base, iters=80)
        if pe < 0.002 and re_ < 0.02 and not any(np.abs(q - o).max() < 0.2 for o in starts):
            starts.append(q)
    best = None
    for q in starts:
        worst, wdeg, step, rows, errs = 0.0, 0.0, 0.0, [], []
        for pos, rot in poses:
            qq, pe, re_ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=base, iters=50)
            step, q = max(step, float(np.degrees(np.abs(qq - q)).max())), qq
            worst, wdeg = max(worst, pe * 1000), max(wdeg, math.degrees(re_))
            rows.append(qq)
            errs.append((round(pe * 1000, 1), round(math.degrees(re_), 1)))
        if best is None or worst + wdeg + 0.2 * step < best[0] + best[1] + 0.2 * best[2]:
            best = (worst, wdeg, step, np.array(rows), errs)
    return best


ERRS = {}
REF = os.environ.get("REF", "path_ref.json")


def _ref_path():
    """The joint branch for each arm along the planned path, found offline by a search over all IK branches, or None."""
    f = pathlib.Path(__file__).resolve().parent / REF
    if not f.exists():
        return None
    return json.loads(f.read_text())


def _solve_seeded(side, poses, ref):
    """One arm along the path, each solve seeded by the offline branch at that waypoint and by the last solve; the closer to the branch wins."""
    from yam_arena import ik, station
    base = np.asarray(station.BASES[side], float)
    rows, errs, worst, wdeg, step = [], [], 0.0, 0.0, 0.0
    q_prev = None
    for k, (pos, rot) in enumerate(poses):
        seed = np.asarray(ref[min(k, len(ref) - 1)], float)
        cands = [ik.solve(pos, ik.matrix_to_quat_xyzw(rot), s, base_pos=base, iters=80) for s in ([seed] if q_prev is None else [seed, q_prev])]
        q, pe, re_ = min(cands, key=lambda c: c[1] * 1000 + math.degrees(c[2]) + 0.05 * float(np.degrees(np.abs(c[0] - seed)).max()))
        if q_prev is not None:
            step = max(step, float(np.degrees(np.abs(q - q_prev)).max()))
        q_prev = q
        worst, wdeg = max(worst, pe * 1000), max(wdeg, math.degrees(re_))
        rows.append(q)
        errs.append((round(pe * 1000, 1), round(math.degrees(re_), 1)))
    return worst, wdeg, step, np.array(rows), errs



def plan_demo(p, R):
    """Joint paths for both arms for the chair standing at (p, R) as observed. Returns the waypoints and, per side, (joints, miss mm, miss deg,
    largest joint step deg, grasp rotation, sgn, approach)."""
    wps, g, A0, z_land = _waypoints(p, R)
    out = {}
    ref = _ref_path()
    for side in ("left", "right"):
        c, j, a, sgn = g[side]
        off = c - A0
        if ref is not None and side in ref:             # the joint branch found offline; each solve is seeded by its waypoint there
            G = _hand_G(j, a, ref[side]["jaw"])
            res = _solve_seeded(side, [(A + _Ry(th) @ off, _Ry(th) @ G) for A, th in wps], ref[side]["path"])
            out[side] = (res[3], res[0], res[1], res[2], G, sgn, a)
            ERRS[side] = res[4]
            continue
        best = None
        for jaw in (1.0, -1.0):
            G = _hand_G(j, a, jaw)
            res = _solve_path(side, [(A + _Ry(th) @ off, _Ry(th) @ G) for A, th in wps])
            if res is not None and (best is None or res[0] + res[1] + 0.2 * res[2] < best[0][0] + best[0][1] + 0.2 * best[0][2]):
                best = (res, G)
        res, G = best
        out[side] = (res[3], res[0], res[1], res[2], G, sgn, a)
        ERRS[side] = res[4]
    return wps, out


def _jgo(arm, q_goal, t, finger=None):
    q0 = arm.q.copy()
    f = arm.finger if finger is None else finger
    n = max(1, round(t * d.CONTROL_HZ))
    for k in range(1, n + 1):
        s = k / n
        s = s * s * (3 - 2 * s)
        arm.q = q0 + (np.asarray(q_goal) - q0) * s
        arm.cmd = np.r_[arm.q, float(np.clip(f / d.FINGER_OPEN, 0.0, 1.0))]
        yield
    arm.finger = f


def _jpath(arm, qs, t):
    """Through joint waypoints qs (rows), t seconds in all, straight in joint space between them."""
    per = t / max(1, len(qs))
    for q in qs:
        yield from _jgo(arm, q, per)


def _ik(arm, pos, rot, seed):
    from yam_arena import ik
    q, pe, re_ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(seed, float), base_pos=arm.base, iters=120)
    return q, pe * 1000, math.degrees(re_)


def _qat(qs, lam):
    """The joint vector a fraction `lam` (in waypoints) along the joint path qs (rows), straight in joint space between rows."""
    lam = float(np.clip(lam, 0.0, len(qs) - 1))
    i = min(int(lam), len(qs) - 2)
    u = lam - i
    return qs[i] + (qs[i + 1] - qs[i]) * u


def _send(arm, q, finger=None):
    arm.q = np.asarray(q, float)
    if finger is not None:
        arm.finger = finger
    arm.cmd = np.r_[arm.q, float(np.clip(arm.finger / d.FINGER_OPEN, 0.0, 1.0))]


def _sides(ctx):
    """The seat face's height over the top at each hand's side (the mean of its two corners on that side), the lowest corner, the chair's
    vertical speed, its pitch and roll: all read from the chair's pose."""
    st = ctx.states()
    p, R = chair_pose(st)
    w = p + (R @ SEAT_FACE.T).T
    z = w[:, 2] - SURF
    h = {"left": float(z[w[:, 1] > p[1]].mean()), "right": float(z[w[:, 1] <= p[1]].mean())}
    lk = landed(st)
    return h, float(z.min()), float(m.state(ctx.emb._env, "chair").lin_vel[2]), lk["pitch_deg"], lk["roll_deg"]


def _set_down(ctx, arms, plan, i0, log):
    """Lower the chair the last few centimetres by feedback: each arm goes on down its own joint path (solved before the run, straight down in
    2.5 mm steps) by the seat face's height read at its own side, 4 cm/s and then 1 cm/s for the last 2 cm, until that side's face is on the top
    (its corners within 1 mm). Then both hands wait for it to settle."""
    dt = 1.0 / d.CONTROL_HZ
    qs = {s: plan[s][0][i0:] for s in arms}
    off = {s: arms[s].q - qs[s][0] for s in arms}
    lam = {s: 0.0 for s in arms}
    done = {s: False for s in arms}
    trace = []
    for k in range(int(10.0 / dt)):
        h, lo, vz, pitch, roll = _sides(ctx)
        if k % 2 == 0:
            trace.append([k, round(1000 * lo, 1), round(1000 * h["left"], 1), round(1000 * h["right"], 1), round(vz, 3), pitch, roll])
        for s in arms:
            if done[s]:
                continue
            e = h[s] - 0.0005
            if e <= 0.0 or lam[s] >= len(qs[s]) - 1:
                done[s] = True
                log.setdefault("down_stop", {})[s] = {"side_mm": round(1000 * h[s], 1), "step": k, "lam": round(lam[s], 1)}
                continue
            v = V_SLOW if e < V_ZONE else V_FAST
            lam[s] = min(lam[s] + min(v * dt, e) / FINE_DZ, len(qs[s]) - 1)
            _send(arms[s], _qat(qs[s], lam[s]) + off[s])
        if all(done.values()):
            break
        yield
    for _ in range(6):
        yield
    print("[chair] set-down trace (step, lowest corner mm, left side mm, right side mm, chair vz m/s, pitch, roll; heights over the top): " + json.dumps(trace), flush=True)


def _release(ctx, arms, plan, log):
    """Open the jaws, back each hand out along its approach 7 cm, then lift it 6 cm."""
    from yam_arena import ik
    yield from ctx.together(*(_jgo(arms[s], arms[s].q, 0.8, d.FINGER_OPEN) for s in arms))
    q_out, q_up = {}, {}
    for s in arms:
        ph, Rh = ik.tcp_pose(arms[s].q, arms[s].base)
        q_out[s], mm, deg = _ik(arms[s], ph - 0.07 * Rh[:, 2], Rh, arms[s].q)
        away = [-0.08, 0.0, 0.0] if GRIP == "back" else [0.0, 0.0, 0.06]   # a back grip sits under the top's edge: away toward the arms
        q_up[s] = _ik(arms[s], ph - 0.07 * Rh[:, 2] + away, Rh, q_out[s])[0]
        log.setdefault("back_out_ik", {})[s] = [round(mm, 1), round(deg, 1)]
    yield from ctx.together(*(_jgo(arms[s], q_out[s], 1.4) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], q_up[s], 1.0) for s in arms))


# -- the split demo (GRIP=split, the default): seat pinch for the turn, the far edge set on the top, the right hand then the left moved to the back plate's side edges (LEG_APPR=back), laid flat --

S_XT, S_ZR, S_ZHI = (float(os.environ.get(k, v)) for k, v in (("S_XT", "0.02"), ("S_ZR", "0.50"), ("S_ZHI", "0.55")))   # the seat grip line in the turn
PHI_TIP = math.radians(float(os.environ.get("PHI", "22")))  # the far edge on the top, the near side this far up: the gripper bodies stay off the top
H0 = float(os.environ.get("H0", "0.06"))                     # the open-loop lowering ends with the seat this far over the top
U_LEG = float(os.environ.get("U_LEG", "0.18"))               # the leg grip's place, from the seat (0) to the foot (1)
LEG_APPR = os.environ.get("LEG_APPR", "back")                   # x: fingers pointing away from the arms, jaw along y; side: in from the chair's side
LEG_TILT = float(os.environ.get("LEG_TILT", "0"))            # the fingers pitched down (+) or up (-)
FIRST = os.environ.get("FIRST", "right")                     # the hand that moves to its leg first
FAR = np.array([SEAT_X / 2, 0.0, SEAT_Z])                    # the seat face's front edge: the far side once turned
REF5 = os.environ.get("REF5", "path_split.json")


def _ref5():
    f = pathlib.Path(__file__).resolve().parent / REF5
    return json.loads(f.read_text()) if f.exists() else {}


def _seat_grips(p, R, ref):
    """Each hand pinches the seat's side edge on its own side through the centre of mass's line, jaw vertical, fingers in from the side."""
    out = {}
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        ys = max((SEAT_Y / 2 - IN, -(SEAT_Y / 2 - IN)), key=lambda v: sgn * float((p + R @ np.array([COM[0], v, 0.0]))[1]))
        c = p + R @ np.array([COM[0], ys, SEAT_Z - SEAT_T / 2])
        a = np.array([0.0, -sgn, 0.0])
        G = _hand_G(np.array([0.0, 0.0, 1.0]), a, float(ref.get("jaw_" + side, -sgn)))
        out[side] = (R.T @ (c - p), R.T @ G, a)
    return out


def _split_poses(p, R, grips, zr=None, zhi=None):
    """The chair's poses (origin, rotation): lift, half-turn about the line between the hands, carry, lower to H0 over the landing, tip onto the
    far edge. Returns the poses and the index where the tip starts. zr, zhi: the grip line's height in the turn (default S_ZR, S_ZHI)."""
    cw = {s: p + R @ grips[s][0] for s in grips}
    A0 = 0.5 * (cw["left"] + cw["right"])
    a1 = np.array([A0[0], A0[1], (S_ZR if zr is None else zr) + TOP])
    a2 = np.array([S_XT, A0[1], (S_ZHI if zhi is None else zhi) + TOP])
    Rl = _Ry(math.pi) @ R
    r, f = Rl @ np.array([-SEAT_X / 2, 0.0, SEAT_Z]), Rl @ np.array([0.0, 0.0, SEAT_Z])
    wps = [(A0, 0.0)] + [(A0 + (a1 - A0) * k / 6, 0.0) for k in range(1, 7)]
    for k in range(1, 25):
        u = k / 24
        s = u * u * (3 - 2 * u)
        wps.append((a1 + (a2 - a1) * s, math.pi * s))
    poses = [(A + _Ry(th) @ (p - A0), _Ry(th) @ R) for A, th in wps]
    po = poses[-1][0]
    pl = np.array([TX - EDGE_T - r[0], po[1], SURF + H0 - f[2]])
    p3 = np.array([pl[0], po[1], po[2]])                     # over the landing at the turn's height
    poses += [(po + (p3 - po) * k / 8, Rl) for k in range(1, 9)]
    poses += [(p3 + (pl - p3) * k / 10, Rl) for k in range(1, 11)]
    i_tip = len(poses)
    e0 = pl + Rl @ FAR
    e1 = np.array([e0[0], e0[1], SURF + 0.001])
    for k in range(1, 13):
        s = k / 12
        Rk = _Ry(PHI_TIP * s) @ Rl
        poses.append((e0 + (e1 - e0) * s - Rk @ FAR, Rk))
    return poses, i_tip


def _leg_grip(side, p, R):
    """The hand's pose on the rear leg on its own side for the chair at (p, R), in the chair's frame (point, rotation) and in the world."""
    sgn = 1.0 if side == "left" else -1.0
    if LEG_APPR == "back":                                   # the back plate's side edge, U_LEG below the seat face (the back hangs down), 16 mm in
        ys = max((BACK_W / 2 - 0.016, -(BACK_W / 2 - 0.016)), key=lambda v: sgn * float((p + R @ np.array([BACK_X, v, 0.0]))[1]))
        c = p + R @ np.array([BACK_X, ys, SEAT_Z + U_LEG])
        jj = R @ np.array([1.0, 0.0, 0.0])
        a0 = np.array([0.0, -sgn, 0.0]) - jj[1] * jj
        a0 /= np.linalg.norm(a0)
        t = math.radians(LEG_TILT)
        a = a0 * math.cos(t) + np.cross(jj, a0) * math.sin(t)
        G = np.column_stack([jj, np.cross(a, jj), a])
        return (R.T @ (c - p), R.T @ G), (c, G)
    top, foot = max((rear_leg(s) for s in (1, -1)), key=lambda tf: sgn * float((p + R @ tf[0])[1]))
    c = p + R @ (top + U_LEG * (foot - top))
    a = np.array([1.0, 0, 0]) if LEG_APPR == "x" else np.array([0, -sgn, 0])
    t = math.radians(LEG_TILT)
    a = a * math.cos(t) + np.array([0, 0, -1.0]) * math.sin(t)
    jw = np.array([0, 1.0, 0]) if LEG_APPR == "x" else np.array([1.0, 0, 0])
    jj = jw - np.dot(jw, a) * a
    jj /= np.linalg.norm(jj)
    G = np.column_stack([jj, np.cross(a, jj), a])
    return (R.T @ (c - p), R.T @ G), (c, G)


def _pitch_about_y(R, Rl):
    """The chair's turn about world y from the flat landed rotation Rl to R, in radians (+: the near side up)."""
    zl, zt = Rl[:, 2], R[:, 2]
    return math.atan2(float(np.cross(zl, zt)[1]), float(zl @ zt))


def _seeded(side, poses, ref):
    """Each solve seeded by the offline branch at the same fraction of the path, and by the last solve; the closer to the branch wins."""
    if not ref:
        res = _solve_path(side, poses)
        return res[3], res[0], res[1], res[2]
    n, m_ = len(poses), len(ref)
    rr = [ref[min(m_ - 1, round(k * (m_ - 1) / max(1, n - 1)))] for k in range(n)]
    w, wd, st, rows, _ = _solve_seeded(side, poses, rr)
    return rows, w, wd, st


def _go_cart(arm, side, targets, t, finger=None):
    """Through Cartesian poses (pos, rot), each IK seeded by the last, t seconds in all."""
    q = arm.q.copy()
    qs = []
    for pos, rot in targets:
        q = _ik(arm, pos, rot, q)[0]
        qs.append(q)
    per = t / max(1, len(qs))
    for q in qs:
        yield from _jgo(arm, q, per, finger)


def _near_h(ctx):
    """The seat face's near-side corners (its rear edge, by the table's near edge): their mean height over the top; and the far corners'."""
    p, R = chair_pose(ctx.states())
    w = p + (R @ SEAT_FACE.T).T
    order = np.argsort(w[:, 0])
    return float(w[order[:2], 2].mean() - SURF), float(w[order[2:], 2].mean() - SURF)


SLIDE_HOLD = 0.005                                           # the lay-down waits and pushes back while the seat has slid more than this


def _centre(ctx):
    p, R = chair_pose(ctx.states())
    return (p + R @ np.array([0.0, 0.0, SEAT_Z]))[:2]


def _slide_mm(ctx, slide):
    return None if slide["ref"] is None else round(1000 * float(np.linalg.norm(_centre(ctx) - slide["ref"])), 1)


def _hold_slide(ctx, arm, slide, stop):
    """While the other hand moves, this hand holds the chair where it first rested: each step it pushes back any observed slide along x."""
    while not stop["done"]:
        if slide["ref"] is not None and os.environ.get("HOLD_REGRASP", "1") == "1":
            dx = float(slide["ref"][0] - _centre(ctx)[0])
            slide["max_mm"] = max(slide["max_mm"], 1000 * abs(dx))
            if abs(dx) > 0.002:
                from yam_arena import ik
                ph, Rh = ik.tcp_pose(arm.q, arm.base)
                _send(arm, _ik(arm, ph + [float(np.clip(0.4 * dx, -0.0015, 0.0015)), 0.0, 0.0], Rh, arm.q)[0])
        yield


def _regrasp(ctx, arm, s, grips, ref, leg_local, f_leg, log, stop):
    """One hand: open, out along its approach, up, round to the back plate (or leg), in along the grip's approach, close."""
    from yam_arena import ik
    yield from _jgo(arm, arm.q, 0.6, d.FINGER_OPEN)
    ph, Rh = ik.tcp_pose(arm.q, arm.base)
    a = grips[s][2]
    yield from _go_cart(arm, s, [(ph - 0.04 * a, Rh), (ph - 0.08 * a, Rh), (ph - 0.08 * a + [0, 0, 0.08], Rh)], 1.4)
    pc, Rc = chair_pose(ctx.states())                    # the chair as it rests now
    leg_local[s], (c, G) = _leg_grip(s, pc, Rc)
    lref = ref.get("leg_" + s)
    q_pre = _ik(arm, c - 0.10 * G[:, 2], G, np.asarray(lref[0], float) if lref else arm.q)[0]
    ph2 = ik.tcp_pose(arm.q, arm.base)[0]
    pre_p = c - 0.10 * G[:, 2]
    if LEG_APPR == "back":                               # round in front of the table's near edge, then down to the back's height
        vias = [np.array([c[0] - 0.10, pre_p[1], ph2[2]]), np.array([c[0] - 0.10, pre_p[1], c[2]])]
    else:
        vias = [np.array([min(ph2[0], c[0] - 0.12), ph2[1], max(ph2[2], c[2] + 0.02)])]
    qv = q_pre
    for vp in vias:
        qv = _ik(arm, vp, G, qv)[0]
        yield from _jgo(arm, qv, 1.3)
    yield from _jgo(arm, q_pre, 1.0)
    yield from _go_cart(arm, s, [(c - G[:, 2] * (0.10 - 0.02 * j), G) for j in range(1, 6)], 1.5)
    yield from arm.wait(0.3)
    pc, Rc = chair_pose(ctx.states())                    # correct the TCP onto the grip point as the chair and the hand are observed now
    _, (c2, G2) = _leg_grip(s, pc, Rc)
    p_obs = ik.tcp_pose(np.asarray(ctx.observed(s)[:6], float), arm.base)[0]
    p_cmd = ik.tcp_pose(arm.q, arm.base)[0]
    log.setdefault("regrasp_err_mm", {})[s] = [round(1000 * float(v), 1) for v in (c2 - p_obs)]
    if os.environ.get("REGRASP_FIX", "0") == "1":        # off by default: no correction move once the hand is at the plate
        yield from _go_cart(arm, s, [(p_cmd + (c2 - p_obs), G2)], 0.5)
    yield from _jgo(arm, arm.q, 0.8, f_leg)
    yield from arm.wait(0.4)
    stop["done"] = True


LEFT_BACK = os.environ.get("LEFT_BACK", "0") == "1"          # 0: after the tip the left hand lets go of the seat and stands clear; the right hand and the far edge lay the chair down
CLEAR_OUT = float(os.environ.get("CLEAR_OUT", "0.045"))       # a hand backs out until each of its fingertip points is this far from the chair (the scorer's gate is 3 cm)


def _hand_gap(ctx, s):
    """The least distance from this hand's 12 fingertip points to the chair's surface."""
    t = np.array(m.fingertips(ctx.emb._env), float)[gk.HANDS[s]]
    return surface_dist(m.state(ctx.emb._env, "chair"), t)


def _back_out(ctx, arm, s, then, log, stop=None):
    """Open; back out along the approach in 1.5 cm steps until every fingertip of this hand is CLEAR_OUT from the chair (read each step, at least
    3 cm, at most 9 cm); then the moves in `then` (world offsets, each in 1.5 cm steps); then read the gap again."""
    from yam_arena import ik
    yield from _jgo(arm, arm.q, 0.8, d.FINGER_OPEN)
    ph, Rh = ik.tcp_pose(arm.q, arm.base)
    a = Rh[:, 2]
    extra = 2                                                # once clear, 3 cm more out along the approach
    for k in range(8):
        if k >= 2 and _hand_gap(ctx, s) > CLEAR_OUT:
            extra -= 1
            if extra < 0:
                break
        ph = ph - 0.015 * a
        yield from _go_cart(arm, s, [(ph, Rh)], 0.3)
    for off in then:
        off = np.asarray(off, float)
        n = max(1, int(math.ceil(np.linalg.norm(off) / 0.015)))
        for _ in range(n):
            g0 = _hand_gap(ctx, s)
            ph_try = ph + off / n
            yield from _go_cart(arm, s, [(ph_try, Rh)], 0.3)
            if _hand_gap(ctx, s) < min(g0, CLEAR_OUT):       # this way brings the hand nearer the chair: back to the last pose, and no further this way
                yield from _go_cart(arm, s, [(ph, Rh)], 0.3)
                break
            ph = ph_try
    yield from arm.wait(0.3)
    log.setdefault("clear_mm", {})[s] = round(1000 * _hand_gap(ctx, s), 1)
    print(f"[chair] {s} backed out: {log['clear_mm'][s]} mm from the chair", flush=True)
    if stop is not None:
        stop["done"] = True


def _tip(ctx, arms, qs, far_plan, slide):
    """Tip the chair onto the seat's far edge along the joint rows qs, by feedback on the far side's observed height: 2 cm/s, 1 cm/s in the last
    2 cm; once the edge is down, on along the rows until the near side is up. Records where the seat first rests in slide["ref"]; returns the
    path fraction reached."""
    dt = 1.0 / d.CONTROL_HZ
    lam, touched = 0.0, False
    for k in range(int(8.0 / dt)):
        hn, hf = _near_h(ctx)
        if hf < 0.0015 and not touched:
            touched = True
            slide["ref"] = _centre(ctx)                      # where the seat first rests: the slide is read from here on
        if touched:
            lam = min(lam + 0.25, len(far_plan) - 1)   # the edge is on the top: the near side comes up to PHI about it
        else:
            v = 0.01 if hf < 0.02 else 0.02
            i = min(int(lam), len(far_plan) - 2)
            drop = max(1e-4, far_plan[i] - far_plan[i + 1])
            lam = min(lam + v * dt / drop, len(far_plan) - 1)
        for s in arms:
            _send(arms[s], _qat(qs[s], lam))
        if lam >= len(far_plan) - 1:
            break
        yield
    return lam


def _palm_rest(ctx, log, soft=False):
    """Lazy route (palm): after the set-down, the left hand comes back and rests its closed fingertips 3 mm over the upturned seat's underside,
    5 cm in from both of its middle lines on the near left, fingers pointing down, coming in from 40 cm above, and stays there."""
    from yam_arena import ik
    arm = ctx.left
    p, R = chair_pose(ctx.states())
    quarters = p + (R @ np.array([[sx * 0.05, sy * 0.05, SEAT_Z - SEAT_T] for sx in (-1, 1) for sy in (-1, 1)]).T).T
    q = quarters[np.argmax(quarters[:, 1] - quarters[:, 0])]
    G = _hand_G(np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, -1.0]), 1.0)
    ph = ik.tcp_pose(arm.q, arm.base)[0]
    yield from _go_cart(arm, "left", [(np.array([ph[0], ph[1], q[2] + 0.40]), G), (q + [0.0, 0.0, 0.40], G), (q + [0.0, 0.0, 0.25], G), (q + [0.0, 0.0, 0.15], G)], 4.0, d.FINGER_CLOSED)
    target = q + np.array([0.0, 0.0, 0.10])
    for k in range(40):
        t = np.array(m.fingertips(ctx.emb._env), float)[gk.HANDS["left"]]
        gap = float(t[:, 2].min() - q[2])
        if gap < (0.012 if soft else 0.004):
            break
        target = target - [0.0, 0.0, min(0.005, gap - (0.010 if soft else 0.003))]
        yield from _go_cart(arm, "left", [(target, G)], 0.25)
    yield from arm.wait(4.0)
    log["palm_gap_mm"] = round(1000 * _hand_gap(ctx, "left"), 1)
    print(f"[chair] palm: left fingertips {log['palm_gap_mm']} mm from the chair", flush=True)


def _demo_split(ctx):
    from yam_arena import ik
    ref = _ref5()
    p0, R0 = chair_pose(ctx.states())
    grips = _seat_grips(p0, R0, ref)
    # floorflip (lazy route): both hands hold it but the turn's grip line is only 14 cm up, so the chair's legs and back turn over on the floor
    # backknock (lazy route): the turn made 14 cm lower, so the back swings into the table's near face half way through the turn
    poses, i_tip = _split_poses(p0, R0, grips, *((0.36, 0.39) if ctx.mode == "backknock" else (0.14, None) if ctx.mode == "floorflip" else (None, None)))
    arms = {"left": ctx.left, "right": ctx.right}
    if ctx.mode == "onehand":
        arms = {"left": ctx.left}
    plan = {}
    for s in arms:
        cl, Gl, a = grips[s]
        if s == "right" and ctx.mode in ("ghostseat", "ghostleg"):    # lazy routes: the right hand stands beside the chair, its tips 10 mm (ghostseat) or 15 mm (ghostleg; 20 mm is out of the arm's IK reach) outside the seat's side edge
            cl = cl - R0.T @ a * (IN + (0.010 if ctx.mode == "ghostseat" else 0.015))
        hp = [(p + R @ cl, R @ Gl) for p, R in poses]
        plan[s] = _seeded(s, hp, ref.get("seat_" + s))
    log = {"plan": {s: {"miss_mm": round(v[1], 1), "deg": round(v[2], 1), "max_step_deg": round(v[3], 1)} for s, v in plan.items()}}
    print("[chair] split plan", log["plan"], flush=True)
    # stand off 8 cm out along each approach and 10 cm up, then in to the seat's edge
    pre = {}
    for s in arms:
        q0 = plan[s][0][0]
        pg, Rg = ik.tcp_pose(q0, arms[s].base)
        a = grips[s][2]
        q_out = _ik(arms[s], pg - 0.08 * a, Rg, q0)[0]
        pre[s] = (_ik(arms[s], pg - 0.08 * a + [0, 0, 0.10], Rg, q_out)[0], q_out)
    yield from ctx.together(*(_jgo(arms[s], pre[s][0], 3.0, d.FINGER_OPEN) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], pre[s][1], 1.2) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], plan[s][0][0], 1.5) for s in arms))
    f_seat = float(np.clip(-(SEAT_T / 2 - GRIP_FORCE / d.GRIPPER["stiffness"]), d.FINGER_OPEN, d.FINGER_CLOSED))
    # hoverlift (lazy route): the right jaws stop 2.8 mm off the seat, no grip; hoverlift_19: 1.9 mm off (inside the 2 mm the text counts as contact)
    f_hover = float(np.clip(-(SEAT_T / 2 + (0.0019 if ctx.mode == "hoverlift_19" else 0.0 if ctx.mode in ("ghostseat", "ghostleg") else 0.0028)), d.FINGER_OPEN, d.FINGER_CLOSED))
    yield from ctx.together(*(_jgo(arms[s], arms[s].q, 0.8, f_hover if (ctx.mode in ("hoverlift", "hoverlift_19", "ghostseat", "ghostleg") and s == "right") else f_seat) for s in arms))
    yield from ctx.together(*(arms[s].wait(0.5) for s in arms))
    print(f"[chair] held at the close by {held_by(m.fingertips(ctx.emb._env), m.state(ctx.emb._env, 'chair'))}", flush=True)
    n_lift, n_turn, n_carry = 7, 31, 39
    stages = [(1, n_lift, 1.5, "lift"), (n_lift, n_turn, 7.0, "turn"), (n_turn, n_carry, 2.4, "carry"), (n_carry, i_tip, 2.0, "lower")]
    if ctx.mode == "onehand_carry":              # lazy route: both lift; the right lets go and the left alone carries it through the first third of the turn; the right takes it again
        n_mid = n_lift + 8
        stages = [(1, n_lift, 1.5, "lift"), (n_lift, n_mid, 2.3, "carry1"), (n_mid, n_turn, 4.7, "turn"), (n_turn, n_carry, 2.4, "carry"), (n_carry, i_tip, 2.0, "lower")]
    for a_, b_, t, lab in stages:
        yield from ctx.together(*(_jpath(arms[s], plan[s][0][a_:b_], t) for s in arms))
        if ctx.mode == "onehand_carry" and lab == "lift":
            yield from _jgo(ctx.right, ctx.right.q, 0.8, d.FINGER_OPEN)
        if ctx.mode == "onehand_carry" and lab == "carry1":
            yield from _jgo(ctx.right, ctx.right.q, 0.8, f_seat)
        smp = {"after": lab, **landed(ctx.states())}
        log.setdefault("samples", []).append(smp)
        print(f"[chair] after {lab}: {smp}", flush=True)
        if ctx.mode == "onehand_turn" and lab == "lift":      # lazy route: both hands lift it, then the right lets go and the left alone turns and carries it
            yield from _jgo(ctx.right, ctx.right.q, 0.8, d.FINGER_OPEN)
            arms = {"left": ctx.left}
    if ctx.mode in ("onehand", "onehand_turn"):
        yield from ctx.left.wait(4.0)
        return
    if ctx.mode == "drop":                      # lazy route: open the jaws where the open-loop lowering ends, the seat H0 over the top
        yield from ctx.together(*(_jgo(arms[s], arms[s].q, 0.8, d.FINGER_OPEN) for s in arms))
        yield from ctx.together(*(_go_cart(arms[s], s, [(ik.tcp_pose(arms[s].q, arms[s].base)[0] - 0.08 * grips[s][2], ik.tcp_pose(arms[s].q, arms[s].base)[1])], 1.2) for s in arms))
        yield from ctx.together(*(arms[s].wait(4.0) for s in arms))
        return
    if ctx.mode == "backknock":
        yield from ctx.together(*(arms[s].wait(4.0) for s in arms))
        return
    dt = 1.0 / d.CONTROL_HZ
    far_plan = [float((p + R @ FAR)[2] - SURF) for p, R in poses[i_tip - 1:]]
    if ctx.mode == "retry":                     # a second try: the far edge set down 5 cm to the left, lifted clear, moved back, then the honest set-down
        shifted = [(p + np.array([0.0, 0.05, 0.0]), R) for p, R in poses[i_tip - 1:]]
        qs2 = {s: _seeded(s, [(p + R @ grips[s][0], R @ grips[s][1]) for p, R in shifted], [list(q) for q in plan[s][0][i_tip - 1:]])[0] for s in arms}
        yield from ctx.together(*(_jgo(arms[s], qs2[s][0], 1.5) for s in arms))
        first = {"ref": None, "max_mm": 0.0}
        lam1 = yield from _tip(ctx, arms, qs2, far_plan, first)
        yield from ctx.together(*(arms[s].wait(0.6) for s in arms))
        log["retry_first_touch"] = {"centre": None if first["ref"] is None else [round(float(v), 3) for v in first["ref"]], **landed(ctx.states())}
        print(f"[chair] retry: first set-down {log['retry_first_touch']}", flush=True)
        n_up = int(math.ceil(lam1)) + 1
        yield from ctx.together(*(_jpath(arms[s], list(reversed(qs2[s][:n_up])), 2.5) for s in arms))
        yield from ctx.together(*(_jgo(arms[s], plan[s][0][i_tip - 1], 1.5) for s in arms))
        yield from ctx.together(*(arms[s].wait(0.5) for s in arms))
    # tip onto the far edge, by feedback on the far side's observed height: 2 cm/s, 1 cm/s in the last 2 cm, on along the path once it is down
    qs = {s: plan[s][0][i_tip - 1:] for s in arms}
    slide = {"ref": None, "max_mm": 0.0}
    yield from _tip(ctx, arms, qs, far_plan, slide)
    yield from ctx.together(*(arms[s].wait(0.6) for s in arms))
    pt, Rt = chair_pose(ctx.states())
    hn, hf = _near_h(ctx)
    log["tipped"] = {"near_mm": round(1000 * hn, 1), "far_mm": round(1000 * hf, 1), **landed(ctx.states()),
                     "tcp_z_mm": {s: round(1000 * (ik.tcp_pose(np.asarray(ctx.observed(s)[:6], float), arms[s].base)[0][2] - SURF), 1) for s in arms}}
    print(f"[chair] tipped: {log['tipped']}", flush=True)
    # each hand in turn: open, out along its approach, up, round to in front of its rear leg, in along the leg grip's approach, close
    f_leg = float(np.clip(-(LEG_R - GRIP_FORCE / d.GRIPPER["stiffness"]), d.FINGER_OPEN, d.FINGER_CLOSED))
    order = [FIRST, "left" if FIRST == "right" else "right"]
    leg_local = {}
    for s in order:
        arm = arms[s]
        other = arms["left" if s == "right" else "right"]
        stop = {"done": False}
        if s == "left" and not LEFT_BACK:                # the left hand lets go of the seat, out along its approach, then up and toward the arms
            yield from ctx.together(_back_out(ctx, arm, s, [[0.0, 0.0, 0.10], [-0.06, 0.0, 0.0]], log, stop), _hold_slide(ctx, other, slide, stop))
            continue
        yield from ctx.together(_regrasp(ctx, arm, s, grips, ref, leg_local, f_leg, log, stop), _hold_slide(ctx, other, slide, stop))
        print(f"[chair] {s} on its leg: held by {held_by(m.fingertips(ctx.emb._env), m.state(ctx.emb._env, 'chair'))}, slide {_slide_mm(ctx, slide)} mm, {landed(ctx.states())}", flush=True)
    # lay it flat about the far edge, by feedback on the near side's observed height: 2 cm/s, 1 cm/s in the last 2 cm; then 1.5 cm further down
    pt, Rt = chair_pose(ctx.states())
    Rl = _Ry(math.pi) @ R0
    phi = _pitch_about_y(Rt, Rl)
    e = pt + Rt @ FAR
    lay = []
    for k in range(25):
        Rk = _Ry(-phi * k / 24) @ Rt
        lay.append((e - Rk @ FAR, Rk))
    lay += [(lay[-1][0] - [0, 0, 0.0025 * k], lay[-1][1]) for k in range(1, 7)]
    near_plan = [float(min((p + (R @ REAR_C.T).T)[:, 2]) - SURF) for p, R in lay]
    lq = {}
    arms = {s: arms[s] for s in arms if s in leg_local}             # the hands on the back plate lay it down; a hand that stood clear stays where it is
    for s in arms:
        ph, Rh = ik.tcp_pose(arms[s].q, arms[s].base)                # each hand where it is commanded now, in the chair's observed frame
        cl, Gl = Rt.T @ (ph - pt), Rt.T @ Rh
        res = _seeded(s, [(p + R @ cl, R @ Gl) for p, R in lay], [list(arms[s].q)])
        lq[s] = res[0]
        log.setdefault("lay_plan", {})[s] = [round(float(v), 1) for v in res[1:]]
    lam, off_x = 0.0, 0.0
    trace = []
    for k in range(int(16.0 / dt)):
        hn, hf = _near_h(ctx)
        if k % 4 == 0:
            trace.append([k, round(1000 * hn, 1), round(1000 * hf, 1), _slide_mm(ctx, slide), round(1000 * off_x, 1)])
        if hn < 0.0008 or lam >= len(lay) - 1:
            break
        dx = float(slide["ref"][0] - _centre(ctx)[0]) if slide["ref"] is not None else 0.0   # the seat's observed slide along x since it first rested
        slide["max_mm"] = max(slide["max_mm"], 1000 * abs(dx))
        if abs(dx) > 0.002:                                  # push it back, 1.5 mm a step at most
            off_x += float(np.clip(0.4 * dx, -0.0015, 0.0015))
        if abs(dx) <= SLIDE_HOLD:                            # lower only while the slide is under 5 mm; 2 cm/s, 1 cm/s under 3 cm, 5 mm/s under 1 cm
            v = 0.005 if hn < 0.01 else (0.01 if hn < 0.03 else 0.02)
            i = min(int(lam), len(lay) - 2)
            drop = max(5e-4, near_plan[i] - near_plan[i + 1])
            lam = min(lam + min(v * dt, hn) / drop, len(lay) - 1)
        for s in arms:
            q_l = _qat(lq[s], lam)
            if abs(off_x) > 1e-4:
                ph, Rh = ik.tcp_pose(q_l, arms[s].base)
                q_l = _ik(arms[s], ph + [off_x, 0.0, 0.0], Rh, arms[s].q)[0]
            _send(arms[s], q_l)
        yield
    log["lay_trace"] = trace
    log["slide_max_mm"] = round(slide["max_mm"], 1)
    yield from ctx.together(*(arms[s].wait(0.6) for s in arms))
    # open, back out along the approach until clear of the chair (read each step), then toward the arms; a back grip sits under the top's edge,
    # below the up-pointing rear legs, so it does not go up
    then = [[-0.06, 0.0, 0.0]] if LEG_APPR == "back" else [[0.0, 0.0, 0.06]]
    # palm (lazy route): while the right hand backs out, the left comes back and rests its fingertips on the upturned seat
    yield from ctx.together(*(_back_out(ctx, arms[s], s, then, log) for s in arms), *((_palm_rest(ctx, log, ctx.mode == "palm_soft"),) if ctx.mode in ("palm", "palm_soft") else ()))
    yield from ctx.together(*(arms[s].wait(3.0) for s in arms))
    final = landed(ctx.states())
    final["edge_gap_mm"] = round(1000 * edge_gap(m.state(ctx.emb._env, "chair")), 1)
    final["hand_gap_mm"] = {s: round(1000 * _hand_gap(ctx, s), 1) for s in ("left", "right")}
    log["final"] = final
    print("[chair] LOG " + json.dumps(log), flush=True)


def demo_script(ctx):
    from yam_arena import ik
    if ctx.mode == "none":                      # the arms wait
        yield from ctx.together(ctx.left.wait(15.0), ctx.right.wait(15.0))
        return
    if GRIP == "split":
        yield from _demo_split(ctx)
        return
    p0, R0 = chair_pose(ctx.states())           # where the chair stands, as observed
    wps, plan = plan_demo(p0, R0)
    wt, wf, where = path_clearance(p0, R0, wps)
    log = {"plan": {s: {"miss_mm": round(v[1], 1), "deg": round(v[2], 1), "max_step_deg": round(v[3], 1)} for s, v in plan.items()},
           "path_clear_mm": {"table": round(1000 * wt, 1), "floor": round(1000 * wf, 1), "nearest": where}}
    print("[chair] plan", log["plan"], log["path_clear_mm"], flush=True)
    arms = {"left": ctx.left, "right": ctx.right}
    if ctx.mode == "onehand":                   # lazy route: the left hand alone does the whole path; the right arm stays at its ready pose
        arms = {"left": ctx.left}
    # stand-off 8 cm out along each approach and 10 cm higher, then in along the approach to the leg
    pre = {}
    for s in arms:
        qs, *_r, G, sgn, a = plan[s]
        pg, _ = ik.tcp_pose(qs[0], arms[s].base)
        q_out, mm1, _ = _ik(arms[s], pg - 0.08 * a, G, qs[0])
        q_hi, mm2, _ = _ik(arms[s], pg - 0.08 * a + [0, 0, 0.10], G, q_out)
        pre[s] = (q_hi, q_out)
        print(f"[chair] {s} stand-off miss {mm1:.1f} / {mm2:.1f} mm", flush=True)
    yield from ctx.together(*(_jgo(arms[s], pre[s][0], 3.0, d.FINGER_OPEN) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], pre[s][1], 1.2) for s in arms))
    yield from ctx.together(*(_jgo(arms[s], plan[s][0][0], 1.5) for s in arms))
    f_close = float(np.clip(-((BACK_T / 2 if GRIP == "back" else LEG_R) - GRIP_FORCE / d.GRIPPER["stiffness"]), d.FINGER_OPEN, d.FINGER_CLOSED))
    yield from ctx.together(*(_jgo(arms[s], arms[s].q, 0.8, f_close) for s in arms))
    yield from ctx.together(*(arms[s].wait(0.5) for s in arms))
    for s in arms:
        po = ik.tcp_pose(np.asarray(ctx.observed(s)[:6], float), arms[s].base)[0]
        pc = ik.tcp_pose(plan[s][0][0], arms[s].base)[0]
        print(f"[chair] {s} at the close: TCP observed {np.round(po, 3).tolist()} planned {np.round(pc, 3).tolist()} (miss {1000 * np.linalg.norm(po - pc):.0f} mm); held by {held_by(m.fingertips(ctx.emb._env), m.state(ctx.emb._env, 'chair'))}", flush=True)

    # each hand's rotation in the chair's frame at the close: the chair's turn in the grip (slip) is read against it
    hand_R = lambda s: ik.tcp_pose(ctx.observed(s)[:6], arms[s].base)[1]
    held = {s: R0.T @ hand_R(s) for s in arms}
    track = {"slip_deg": 0.0}

    def watched(gen):
        for _ in gen:
            p, R = chair_pose(ctx.states())
            for s in arms:
                dR = (R.T @ hand_R(s)) @ held[s].T
                track["slip_deg"] = max(track["slip_deg"], math.degrees(math.acos(float(np.clip((np.trace(dR) - 1) / 2, -1, 1)))))
            yield

    n0, n1, n2, n3 = 7, 31, 39, 47
    for a, b, t, lab in [(1, n0, 1.5, "lift"), (n0, n1, 7.0, "turn"), (n1, n2, 2.4, "carry"), (n2, n3, 1.6, "lower")]:
        yield from watched(ctx.together(*(_jpath(arms[s], plan[s][0][a:b], t) for s in arms)))
        sample = {"after": lab, "slip_deg": round(track["slip_deg"], 1), **landed(ctx.states())}
        log.setdefault("samples", []).append(sample)
        print(f"[chair] after {lab}: {sample}", flush=True)
    if ctx.mode == "onehand":
        yield from ctx.left.wait(4.0)
        return
    if ctx.mode == "drop":                      # lazy route: open the jaws where the open-loop lowering ends, the seat ZL over the top
        yield from ctx.together(*(arms[s].wait(0.5) for s in arms))
        yield from _release(ctx, arms, plan, log)
    else:
        yield from watched(_set_down(ctx, arms, plan, n3 - 1, log))
        yield from _release(ctx, arms, plan, log)
    yield from ctx.together(*(arms[s].wait(3.0) for s in arms))
    final = landed(ctx.states())
    p, R = chair_pose(ctx.states())
    final["edge_gap_mm"] = round(1000 * edge_gap(m.state(ctx.emb._env, "chair")), 1)
    log.update({"slip_deg": round(track["slip_deg"], 1), "final": final})
    print("[chair] LOG " + json.dumps(log), flush=True)
