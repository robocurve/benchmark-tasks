"""A Haply MinVerse desktop haptic controller stands on its folding feet at the back of the desk: a grey head
lying flat on two side posts over a black base bar, with its passive two-link arm reaching forward from the head.
The arm ends in the gimbal, a black 14 mm tube that turns freely about its own axis; its tip is a fork with a round
cutout for the stylus's ball. The arm is unpowered and moves wherever it is pushed. The black VerseGrip stylus, a
pen-shaped handle with a 9 mm steel ball on its front end, lies loose on the desk to the right of the controller.
Both arms start at their ready pose with the grippers open.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
from functools import lru_cache

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402  (before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s released-and-still hold is counted inside score()
INSTRUCTION = "Fit the VerseGrip stylus onto the MinVerse's gimbal, then let go and move both hands clear."
MM = 1e-3
EPS = 1e-6
D = math.radians

BLACK, DARK, GREY = (0.05, 0.05, 0.06), (0.16, 0.16, 0.17), (0.50, 0.52, 0.55)
STEEL, WHITE, CYAN = (0.78, 0.79, 0.81), (0.93, 0.93, 0.90), (0.20, 0.80, 0.90)
Z = np.array([0.0, 0.0, 1.0])


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _ft(v):
    """Plain Python floats (Isaac rejects NumPy scalars in poses)."""
    return tuple(float(x) for x in v)


def _quat(m):
    return tuple(float(v) for v in parts._matrix_to_quat(np.asarray(m, float)))


# -- where things are (world, m; the controller never moves) ---------------------------------------------------------
# Gimbal frame: u along the tube toward its tip, n out of the cutout (straight up when the gimbal's turn is 0),
# w = n x u. The ball's seat (where the ball's centre sits when the stylus is fitted) is the frame's origin and lies
# on the tube's turning axis.
BETA = D(180.0)  # the gimbal tube points at the bench arms (world heading)
U = np.array([math.cos(BETA), math.sin(BETA), 0.0])
W = np.cross(Z, U)
M = np.column_stack([U, W, Z])  # gimbal frame -> world, at turn 0
SEAT = np.array([0.300, 0.030, TOP + 0.070])

TUBE_R = 0.007  # gimbal tube radius (14 mm, estimate from maker photographs)
TUBE_BACK = 0.070  # tube from 6.8 mm to 70 mm behind the seat
R_PIVOT = SEAT - U * TUBE_BACK  # forearm end: the gimbal's bearing
ELBOW = R_PIVOT - U * 0.048 + Z * 0.042
SHOULDER = ELBOW - U * 0.047 - Z * 0.024
ORIGIN = np.array([*(SHOULDER - U * 0.058)[:2], TOP])  # part frame origin: on the desk under the head's middle
BALL_R = 0.0045  # 9 mm steel ball connector (estimate, scaled from the maker's photograph)

# The arm is unpowered: each joint sticks until pushed past a small torque, then turns and stays where it is left
# (motor and capstan drag; estimates). Stiffness (N m/rad) and the slip it takes to break away (rad).
STICK = {"swing": (5.0, 0.01), "shoulder": (20.0, 0.01), "elbow": (12.0, 0.01), "turn": (0.4, 0.01)}
ARM_JOINTS = tuple(STICK)
JAW_K, JAW_PRELOAD, JAW_TRAVEL = 50.0, 0.60, 0.0035  # N/m, N, m: the sprung tine (magnet bias, estimate)


def gpt(du, dw, dn):
    """World point at (du, dw, dn) mm in the gimbal frame (turn 0)."""
    return SEAT + M @ (np.array([du, dw, dn], float) * MM)


def _local(p):
    return tuple(float(v) for v in np.asarray(p, float) - ORIGIN)


def gbox(u0, u1, w0, w1, n0, n1, color=BLACK, friction=None):
    """A box spanning [u0,u1] x [w0,w1] x [n0,n1] mm in the gimbal frame."""
    c = gpt((u0 + u1) / 2, (w0 + w1) / 2, (n0 + n1) / 2)
    size = ((u1 - u0) * MM, (w1 - w0) * MM, (n1 - n0) * MM)
    return parts.box(size, at=_local(c), color=color, quat=_quat(M), friction=friction)


def gwedge(section, axis, a0, a1, color=BLACK, friction=None):
    """A convex prism: a polygon in the (w, n) plane (axis "u", extruded u a0..a1) or in the (u, n) plane
    (axis "w", extruded w a0..a1), all in mm of the gimbal frame."""
    pts = []
    for a in (a0, a1):
        for p, q in section:
            pts.append(gpt(a, p, q) if axis == "u" else gpt(p, a, q))
    k = len(section)
    faces = [tuple(range(k - 1, -1, -1)), tuple(range(k, 2 * k))]
    faces += [(i, (i + 1) % k, k + (i + 1) % k, k + i) for i in range(k)]
    return parts.hull([_local(p) for p in pts], faces, color=color, friction=friction)


def bar(a, b, width, thick, color=BLACK, collide=True):
    """A box from point a to point b (world), its width along w."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    x = _unit(b - a)
    y = W
    z = np.cross(x, y)
    return parts.box((float(np.linalg.norm(b - a)), width, thick), at=_local((a + b) / 2), color=color,
                     quat=_quat(np.column_stack([x, y, z])), collide=collide)


def dbox(cu, cw, cz, su, sw, sz, color=BLACK, collide=True):
    """A box in the controller's own frame (u forward toward the arm's tip, w, z up), metres from ORIGIN."""
    c = ORIGIN + U * cu + W * cw + Z * cz
    return parts.box((su, sw, sz), at=_local(c), color=color, quat=_quat(M), collide=collide)


# -- the MinVerse: body, then the passive arm (swing, shoulder, elbow), the gimbal (turn) and its sprung tine (jaw) -
FRIC_METAL = (0.30, 0.25)  # anodised and steel parts against the steel ball (estimate)

_BODY = [
    dbox(0.000, 0.000, 0.010, 0.030, 0.240, 0.020),  # base bar 240 mm (folded envelope 240 x 120 x 40 mm)
    dbox(0.000, 0.104, 0.003, 0.120, 0.020, 0.006),  # folding feet, unfolded fore and aft
    dbox(0.000, -0.104, 0.003, 0.120, 0.020, 0.006),
    dbox(0.000, 0.108, 0.055, 0.030, 0.016, 0.070),  # side posts that carry the head's hinge
    dbox(0.000, -0.108, 0.055, 0.030, 0.016, 0.070),
    dbox(0.000, 0.000, 0.096, 0.095, 0.200, 0.032, GREY),  # the head, lying flat, status light away from the user
    dbox(-0.0480, 0.000, 0.104, 0.002, 0.170, 0.005, WHITE, collide=False),  # status light (white: idle)
    dbox(0.000, 0.125, 0.030, 0.026, 0.026, 0.060),  # the inkwell (calibration holder) at the right end
    dbox(0.000, 0.125, 0.0605, 0.016, 0.016, 0.002, DARK, collide=False),
]

_SWING = parts.Hinge(
    "swing", shapes=(parts.cylinder(0.010, 0.030, at=_local(SHOULDER), color=BLACK, axis="z"),),
    pivot=_local(SHOULDER), axis=_ft(Z), lower=-0.7, upper=0.7, mass=0.02, spring=STICK["swing"][0],
    damping=0.002, max_effort=1.0)
_SHOULDER = parts.Hinge(
    "shoulder", shapes=(bar(SHOULDER, ELBOW, 0.012, 0.010),
                        bar(SHOULDER + Z * 0.009, ELBOW + Z * 0.009, 0.003, 0.003, STEEL, collide=False)),
    pivot=_local(SHOULDER), axis=_ft(W), lower=-0.8, upper=0.8, mass=0.025, spring=STICK["shoulder"][0],
    damping=0.002, max_effort=1.0, parent="swing")
_ELBOW = parts.Hinge(
    "elbow", shapes=(bar(ELBOW, R_PIVOT - U * 0.004, 0.010, 0.008),
                     bar(ELBOW + Z * 0.008, R_PIVOT - U * 0.004 + Z * 0.008, 0.003, 0.003, STEEL, collide=False),
                     parts.cylinder(0.0085, 0.018, at=_local(ELBOW), color=BLACK, axis="x" if abs(W[0]) > 0.9 else "y"),
                     gbox(-80.0, -72.0, -7.0, 7.0, -7.0, 7.0)),  # bearing housing behind the gimbal
    pivot=_local(ELBOW), axis=_ft(W), lower=-0.8, upper=0.8, mass=0.025, spring=STICK["elbow"][0],
    damping=0.002, max_effort=1.0, parent="shoulder")

# The gimbal's fork (mm, gimbal frame): a pocket 9.6 mm square around the seat, closed below (floor), behind (back
# wall), on one side (fixed tine) and in front by two posts with a 4.8 mm slot for the stylus's neck; the other side
# is the sprung tine (jaw). Its top is a funnel of 45 degree chamfers.
_FORK = (
    parts.prism(TUBE_R / math.cos(math.pi / 16), (TUBE_BACK - 0.0068), 16, at=_local(gpt(-(TUBE_BACK * 1000 + 6.8) / 2, 0, 0)),
                color=BLACK, quat=_quat(np.column_stack([U, W, Z]) @ np.array(
                    [[1, 0, 0], [0, math.cos(math.pi / 16), -math.sin(math.pi / 16)],
                     [0, math.sin(math.pi / 16), math.cos(math.pi / 16)]]))),
    gbox(-6.8, 3.0, -7.0, 4.6, -7.0, -4.7, friction=FRIC_METAL),  # floor (ends short of the posts)
    gbox(-6.8, 6.3, -7.0, -4.8, -4.7, 3.0, friction=FRIC_METAL),  # fixed tine
    gbox(-6.8, -4.8, -4.8, 4.6, -4.7, 3.0, friction=FRIC_METAL),  # back wall
    gbox(4.8, 6.3, -4.8, -2.4, -4.7, 3.0, friction=FRIC_METAL),  # front post (fixed side)
    gbox(4.8, 6.3, 2.4, 4.6, -4.7, 3.0, friction=FRIC_METAL),  # front post (tine side)
    gwedge(((-4.8, 3.0), (-9.0, 3.0), (-9.0, 7.2)), "u", -6.8, 6.3, friction=FRIC_METAL),  # chamfer, fixed side
    gwedge(((-4.8, 3.0), (-9.0, 3.0), (-9.0, 7.2)), "w", -4.8, 4.6, friction=FRIC_METAL),  # chamfer, back
    gwedge(((4.8, 3.0), (8.3, 3.0), (8.3, 6.5)), "w", -4.8, -2.4, friction=FRIC_METAL),  # chamfers on the posts
    gwedge(((4.8, 3.0), (8.3, 3.0), (8.3, 6.5)), "w", 2.4, 4.6, friction=FRIC_METAL),
)
_TURN = parts.Hinge(
    "turn", shapes=_FORK, pivot=_local(SEAT), axis=_ft(U), lower=-math.pi, upper=math.pi, mass=0.015,
    spring=STICK["turn"][0], damping=0.0005, max_effort=0.1, parent="elbow")
# The sprung tine: a plate beside the ball with a lip over it; its chamfered top lets a ball pressed down into the
# cutout push it aside, and its lip then closes over the ball. Its pull toward closed stands in for the magnets.
_JAW = parts.Slider(
    "jaw", shapes=(gbox(-4.6, 4.6, 4.8, 7.0, -4.6, 4.1, friction=FRIC_METAL),
                   gwedge(((2.2, 4.1), (7.0, 4.1), (7.0, 8.9)), "u", -4.0, 4.0, friction=FRIC_METAL)),
    axis=_ft(W), travel=JAW_TRAVEL, mass=0.008, spring=JAW_K, damping=0.3, max_effort=5.0,
    target=-JAW_PRELOAD / JAW_K, parent="turn")

MINVERSE = parts.Part(
    "minverse", pos=_ft(ORIGIN), mass=None, shapes=_BODY, joints=(_SWING, _SHOULDER, _ELBOW, _TURN, _JAW),
    friction=(0.6, 0.5), contact_offset=0.001)

# -- the VerseGrip stylus: ball, neck, nose and a 21 mm moulded body, 180 mm overall, 80 g (estimates) -------------
# Part frame: the ball's centre is the origin; the body runs along -x.
_ROLL16 = np.array([[1, 0, 0], [0, math.cos(math.pi / 16), -math.sin(math.pi / 16)],
                    [0, math.sin(math.pi / 16), math.cos(math.pi / 16)]])
STYLUS_LEN = 0.1645  # ball centre to the tail's centre of curvature
BODY_R = 0.0105
_STYLUS_SHAPES = (
    parts.sphere(BALL_R, at=(0.0, 0.0, 0.0), color=STEEL),
    parts.cylinder(0.002, 0.010, at=(-0.0085, 0.0, 0.0), color=STEEL, axis="x"),
    parts.cylinder(0.0055, 0.006, at=(-0.0165, 0.0, 0.0), color=BLACK, axis="x"),
    parts.cylinder(0.0085, 0.006, at=(-0.0225, 0.0, 0.0), color=BLACK, axis="x"),
    parts.prism(BODY_R / math.cos(math.pi / 16), 0.139, 16, at=(-0.095, 0.0, 0.0), color=BLACK, quat=_quat(_ROLL16)),
    parts.sphere(0.0100, at=(-STYLUS_LEN, 0.0, 0.0), color=BLACK),
    parts.box((0.012, 0.007, 0.002), at=(-0.040, 0.0, BODY_R), color=DARK, collide=False),  # buttons A and B
    parts.box((0.012, 0.007, 0.002), at=(-0.055, 0.0, BODY_R), color=DARK, collide=False),
    parts.cylinder(0.003, 0.002, at=(-0.075, 0.0, BODY_R), color=DARK, collide=False),  # calibrate
    parts.box((0.004, 0.0015, 0.001), at=(-0.112, 0.0, BODY_R), color=CYAN, collide=False),  # status light
    parts.cylinder(0.0035, 0.002, at=(-0.128, 0.0, BODY_R), color=DARK, collide=False),  # power
)
STYLUS_A = (0.290, -0.150, TOP + BODY_R + 0.0024)  # ball centre, lot A (stylus along x, ball toward +x)
STYLUS_B = (0.200, -0.095, TOP + BODY_R + 0.0024)  # lot B (stylus along y, ball toward +y)
STYLUS = parts.Part("versegrip", pos=STYLUS_A, mass=0.080, shapes=_STYLUS_SHAPES, friction=(0.6, 0.5),
                    contact_offset=0.001)

OBJECTS = [MINVERSE, STYLUS]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", turn=40.0, place={"versegrip": STYLUS_A}, yaw={"versegrip": 0.0}),
    lots.Lot("B", turn=0.0, place={"versegrip": STYLUS_B}, yaw={"versegrip": math.pi / 2}),
)

# -- scoring thresholds ---------------------------------------------------------------------------------------------
SEATED = 0.0015  # ball centre within 1.5 mm of the seat: down in the fork below the tine's lip (perched, it sits 8 mm up)
JAW_SHUT = 0.0005  # the sprung tine within 0.5 mm of closed
MOUTH_SIDE, MOUTH_UP = 0.0035, 0.010  # ball in the cutout's mouth: within 3.5 mm of its centre line, at most 10 mm up
HAND_CLEAR, HAND_FREE = 0.030, 0.005
STILL_POS, STILL_ROT = 0.0010, D(2.0)
HOLD_S = 1.0
OFF_DESK = TOP - 0.05
DESK_GAP = 0.005  # the gimbal must never be pushed down to within 5 mm of the desk (bracing it on the desk)


def gimbal_frame(objects):
    """(seat centre, rotation whose columns are u, w, n) where the gimbal is now."""
    p, q = objects[MINVERSE.name].links["turn"]
    r = parts._quat_to_matrix(q) @ M
    return np.asarray(p, float), r


def ball_in_gimbal(objects):
    """The ball centre in the gimbal frame (m): (du, dw, dn)."""
    seat, r = gimbal_frame(objects)
    return r.T @ (np.asarray(objects[STYLUS.name].pos, float) - seat)


def seated(objects):
    rel = ball_in_gimbal(objects)
    jaw = float(objects[MINVERSE.name].joints["jaw"])
    return float(np.linalg.norm(rel)) <= SEATED + EPS and jaw <= JAW_SHUT + EPS


def in_mouth(objects):
    du, dw, dn = ball_in_gimbal(objects)
    return math.hypot(du, dw) <= MOUTH_SIDE + EPS and -0.0010 - EPS <= dn <= MOUTH_UP + EPS


GIMBAL_R = 0.012  # rounded segment round the tube and fork (the fork's chamfers reach 11.5 mm from the axis)


def capsules(objects):
    """(a, b, radius): the stylus, the gimbal (tube and fork) and the arm's two links as rounded segments, world."""
    st = objects[STYLUS.name]
    p, r = np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)
    seat, g = gimbal_frame(objects)
    links = objects[MINVERSE.name].links
    ep, eq = links["elbow"]
    sp, sq = links["shoulder"]
    ep, sp = np.asarray(ep, float), np.asarray(sp, float)
    return [(p, p + r @ np.array([-STYLUS_LEN, 0.0, 0.0]), BODY_R),
            (seat + g[:, 0] * 0.0063, seat - g[:, 0] * TUBE_BACK, GIMBAL_R),
            (ep, ep + parts._quat_to_matrix(eq) @ (R_PIVOT - ELBOW), 0.009),
            (sp, sp + parts._quat_to_matrix(sq) @ (ELBOW - SHOULDER), 0.009)]


def gimbal_low(objects):
    """Height of the gimbal's lowest surface above the desk (m), from its rounded segment."""
    seat, g = gimbal_frame(objects)
    return min(float((seat + g[:, 0] * 0.0063)[2]), float((seat - g[:, 0] * TUBE_BACK)[2])) - GIMBAL_R - TOP


def _seg_dist(points, a, b, radius):
    ab = b - a
    t = np.clip((points - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(points - (a + np.outer(t, ab)), axis=1) - radius


def hand_gap(points, objects):
    best = 1.0
    for a, b, rad in capsules(objects):
        lo, hi = np.minimum(a, b) - rad - 0.1, np.maximum(a, b) + rad + 0.1
        keep = np.all((points >= lo) & (points <= hi), axis=1)
        if np.any(keep):
            best = min(best, float(_seg_dist(points[keep], a, b, rad).min()))
    return best


def _pose_vec(objects):
    st = objects[STYLUS.name]
    p, q = objects[MINVERSE.name].links["turn"]
    return np.r_[np.asarray(st.pos, float), np.asarray(st.quat_xyzw, float), np.asarray(p, float), np.asarray(q, float)]


def _moved(a, b):
    """(largest position change m, largest turn rad) between two pose vectors."""
    dp = max(float(np.linalg.norm(a[0:3] - b[0:3])), float(np.linalg.norm(a[7:10] - b[7:10])))
    dr = 0.0
    for i in (3, 10):
        dr = max(dr, 2.0 * math.acos(min(1.0, abs(float(a[i:i + 4] @ b[i:i + 4])))))
    return dp, dr


# -- the trial record (written only by the simulator's step and reset hooks) ------------------------------------------
def _fresh():
    return dict(known=False, invalid=False, why="", t=None, anchor=None, ref=None, held_clear=0.0, held_free=0.0,
                gap=None, seated=False, mouth=False, stamp=None)


REC = _fresh()


def stamp(objects):
    return tuple(round(float(v), 9) for v in _pose_vec(objects)) + (round(float(objects[MINVERSE.name].joints["jaw"]), 9),)


def observe(objects, hands, t):
    """Update the record from one control step's states and the hands' surface points (None: unknown)."""
    try:
        vec = _pose_vec(objects)
        ok = bool(np.all(np.isfinite(vec)))
    except (KeyError, TypeError, ValueError, AttributeError):
        ok = False
    dt = None if REC["t"] is None else float(t) - REC["t"]
    if not ok or hands is None or (dt is not None and not 0.0 < dt <= 0.2):
        REC.update(known=False, invalid=True, why="state", held_clear=0.0, held_free=0.0, t=float(t), stamp=None)
        return
    if float(objects[STYLUS.name].low[2]) < OFF_DESK:
        REC.update(invalid=True, why="stylus off the desk")
    if gimbal_low(objects) < DESK_GAP - EPS:
        REC.update(invalid=True, why="gimbal pressed onto the desk")
    is_seated, mouth = seated(objects), in_mouth(objects)
    gap = hand_gap(hands, objects)
    ref = REC["ref"]
    still = False
    if ref is not None:
        dp, dr = _moved(vec, ref)
        still = dp <= STILL_POS + EPS and dr <= STILL_ROT + EPS
    free, clear = gap >= HAND_FREE - EPS, gap >= HAND_CLEAR - EPS
    if is_seated and free and still and dt is not None:
        REC["held_free"] += dt
        REC["held_clear"] = REC["held_clear"] + dt if clear else 0.0
    else:
        REC["held_free"], REC["held_clear"], REC["ref"] = 0.0, 0.0, vec
    if is_seated and free and not clear:
        REC["held_clear"] = 0.0
    REC.update(known=True, t=float(t), stamp=stamp(objects), gap=gap, seated=is_seated, mouth=mouth)


def tier():
    if REC["seated"]:
        if REC["held_clear"] >= HOLD_S - EPS:
            return 1.0
        if REC["held_free"] >= HOLD_S - EPS:
            return 0.75
        return 0.5
    if REC["mouth"]:
        return 0.25
    return 0.0


def score(objects):
    """0 with an unknown, stale or invalid record; otherwise the row the end state reaches (see tier())."""
    try:
        if not REC["known"] or REC["invalid"] or REC["stamp"] != stamp(objects):
            return 0.0
        return float(tier())
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# -- simulator hooks: hand samples, the arm's drag, the record --------------------------------------------------------
@lru_cache(maxsize=1)
def hand_samples():
    """Points on every collision face of the gripper, both fingers and the wrist link (link frames), <= 3 mm apart."""
    from yam_arena import analysis

    out = {}
    for name in ("gripper", "tip_left", "tip_right", "link5"):
        cloud = []
        for h in analysis.link_hulls().get(name, []):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(0)
                u = face[0] - c
                if np.linalg.norm(u) < 1e-9:
                    continue
                u /= np.linalg.norm(u)
                v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / 0.003))
                    for i in range(n + 1):
                        for k in range(n - i + 1):
                            cloud.append(c + (a - c) * (i / n) + (b - c) * (k / n))
        if not cloud:
            raise ValueError("missing hand geometry")
        pts = np.asarray(cloud)
        _, idx = np.unique(np.round(pts / 0.0015).astype(np.int64), axis=0, return_index=True)
        out[name] = pts[np.sort(idx)]
    return out


def hand_points(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    clouds = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        bodies = parts._np(asset.data.body_link_pose_w)[0]
        if bodies.ndim != 2 or bodies.shape[1] != 7 or not np.isfinite(bodies).all():
            raise ValueError("unknown robot body poses")
        for name, pts in hand_samples().items():
            pose = bodies[asset.body_names.index(name)]
            clouds.append(pts @ parts._quat_to_matrix(pose[3:]).T + pose[:3] - origin)
    return np.concatenate(clouds)


def drag(objects):
    """The arm's joints stick until pushed past their break-away torque, then follow (anchors trail by the slip)."""
    q = {k: float(objects[MINVERSE.name].joints[k]) for k in ARM_JOINTS}
    anc = REC["anchor"]
    out = {}
    for k, (_, slip) in STICK.items():
        a = min(max(anc[k], q[k] - slip), q[k] + slip)
        out[k] = round(a, 6)
    return out


def _apply_drag(sim, anchors):
    old = REC.get("anchor") or {}
    for k, v in anchors.items():
        if old.get(k) != v:
            sim.set_target(MINVERSE.name, k, v)
    REC["anchor"] = dict(anchors)


@parts.on_reset
def _reset(sim):
    REC.clear()
    REC.update(_fresh())
    REC["anchor"] = {k: 0.0 for k in ARM_JOINTS}


_lots_apply = lots._apply


def _apply_lot(emb):
    """Each trial: the stylus where the lot puts it, the gimbal turned to the lot's start, the arm still, then settle."""
    _lots_apply(emb)
    lot = lots.current()
    sim = parts.Sim(emb)
    start = {"swing": 0.0, "shoulder": 0.0, "elbow": 0.0, "turn": D(float(lot.turn))}
    for name, value in start.items():
        sim.write_joint(MINVERSE.name, name, value)
    sim.write_joint(MINVERSE.name, "jaw", 0.0)
    sim.set_target(MINVERSE.name, "jaw", _JAW.target)  # write_joint also moved the drive target; restore the pull
    REC.clear()
    REC.update(_fresh())
    REC["anchor"] = {}
    _apply_drag(sim, start)
    base = emb._base
    for _ in range(max(1, round(lots.SETTLE_S / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_lot


@parts.on_step
def _step(sim):
    objects = sim.states()
    t = float(sim.t)
    try:
        hands = hand_points(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        hands = None
    try:
        if REC["anchor"] is None or not REC["anchor"]:
            REC["anchor"] = {k: float(objects[MINVERSE.name].joints[k]) for k in ARM_JOINTS}
        _apply_drag(sim, drag(objects))
    except (KeyError, TypeError, ValueError, AttributeError) as e:
        REC.update(known=False, invalid=True, why="arm drag: " + str(e))
    observe(objects, hands, t)
    if os.environ.get("TRACE_OUT"):
        try:
            _trace_row(objects, t)
        except Exception as e:  # noqa: BLE001  (a trace must never stop a trial)
            print("trace row failed:", e)


def _trace_row(objects, t):
    j = objects[MINVERSE.name].joints
    rel = ball_in_gimbal(objects)
    seat, _ = gimbal_frame(objects)
    row = dict(t=round(t, 3), lot=lots.current().name, ball_mm=[round(float(v) * 1000, 2) for v in rel],
               jaw_mm=round(float(j["jaw"]) * 1000, 3), turn_deg=round(math.degrees(float(j["turn"])), 2),
               swing=round(float(j["swing"]), 4), shoulder=round(float(j["shoulder"]), 4),
               elbow=round(float(j["elbow"]), 4), seat=[round(float(v), 4) for v in seat],
               stylus=[round(float(v), 4) for v in objects[STYLUS.name].pos],
               tilt_deg=round(tilt.tilt_of(objects[STYLUS.name].quat_xyzw), 2),
               seated=bool(REC["seated"]), mouth=bool(REC["mouth"]), low_mm=round(gimbal_low(objects) * 1000, 1),
               gap=None if REC["gap"] is None else round(float(REC["gap"]), 4), invalid=bool(REC["invalid"]),
               held_free=round(float(REC["held_free"]), 2), held_clear=round(float(REC["held_clear"]), 2),
               why=str(REC["why"]), score=float(score(objects)))
    with open(pathlib.Path(os.environ["TRACE_OUT"]).with_name("mechanism.jsonl"), "a") as f:
        f.write(json.dumps(row) + "\n")


# -- the scripted demo: open loop from the published layout -----------------------------------------------------------
BRACE_ARM, INSERT_ARM = "left", "right"
BRACE_BACK = 0.050  # brace grip on the tube, this far behind the seat
GRIP_FWD = 0.085  # stylus held this far along its body from the ball's centre
GRIP_TCP_UP = 0.0055  # the gripper's centre point above the stylus's axis when picked from the desk
BRACE_GRIP, STYLUS_GRIP = -0.004, -0.0075  # finger targets: 3 mm past the 14 mm tube and the 21 mm body
PUSH = 0.020  # ball commanded this far below the seat (the arms give under load)
LEAN = 20.0  # degrees the stylus is held leaned (ball end up) near the gimbal: keeps the two wrists apart
PARK = {"left": (0.18, 0.28, TOP + 0.16), "right": (0.18, -0.28, TOP + 0.16)}


def _yaw_for(direction, arm):
    """Grip yaw that closes the fingers across a part lying along ``direction``. The stock grip (yaw 0) closes them
    along world y; a right wrist turns +90 degrees more easily than -90, a left one the other way."""
    a = math.atan2(direction[1], direction[0])
    options = [y for y in (a, a + math.pi, a - math.pi, a - 2 * math.pi) if -D(100.0) <= y <= D(100.0)]
    options.sort(key=lambda y: (abs(abs(y) - math.pi / 2) < 0.2 and (y < 0) == (arm == "right"), abs(y)))
    return options[0]


def demo():
    lot = lots.current()
    turn0 = float(lot.turn)
    yaw_b = _yaw_for(U, BRACE_ARM)
    # Leaning the gripped tube toward +w turns it by +angle about u: grip it leaned as the gimbal stands (read from
    # where its cutout faces), then stand the grip up, which turns the cutout to face straight up.
    side = W if turn0 >= 0 else -W
    lean_heading = math.atan2(side[1], side[0])
    turn0 = abs(turn0)
    P = SEAT - U * BRACE_BACK
    a0 = side * math.sin(D(turn0)) - Z * math.cos(D(turn0))  # the leaned approach (toward the fingertips)

    def B(dt, p, f, label, lean):
        return tilt.Waypoint(float(dt), tuple(float(v) for v in p), float(f), label, yaw=float(yaw_b),
                             tilt=float(lean), heading=float(lean_heading))

    t_brace = 2.5 + 1.5 + 0.8 + 0.4 + 2.0
    brace = [
        B(2.5, P - a0 * 0.045, FINGER_OPEN, "above the gimbal tube", turn0),
        B(1.5, P, FINGER_OPEN, "down over the tube", turn0),
        B(0.8, P, BRACE_GRIP, "grip the tube", turn0),
        B(0.4, P, BRACE_GRIP, "settle", turn0),
        B(2.0, P, BRACE_GRIP, "turn the cutout up", 0.0),
    ]
    # the stylus: pick from the desk, carry, ball down into the cutout, press, let go
    pos0 = np.asarray(lot.pos(STYLUS.name), float)
    yaw0 = float(lot.yaw.get(STYLUS.name, 0.0))
    ax0 = np.array([math.cos(yaw0), math.sin(yaw0), 0.0])  # from the tail toward the ball
    g0 = pos0 - ax0 * GRIP_FWD
    pick_yaw = _yaw_for(ax0, INSERT_ARM)
    ins_yaw = _yaw_for(U, INSERT_ARM)
    lift = TOP + 0.13
    lean_h = math.atan2(-U[1], -U[0])  # the stylus held leaned: its ball end up, its tail down toward the bench arms
    r_lean = tilt.rotation(ins_yaw, LEAN, lean_h) @ tilt.rotation(ins_yaw, 0.0, 0.0).T

    def tcp(ball):
        """Gripper point that puts the ball at ``ball`` with the stylus held leaned LEAN degrees."""
        return np.asarray(ball, float) + r_lean @ (U * GRIP_FWD + Z * GRIP_TCP_UP)

    def S_(dt, p, f, label, yaw, lean=0.0):
        return tilt.Waypoint(float(dt), tuple(float(v) for v in p), float(f), label, yaw=float(yaw),
                             tilt=float(lean), heading=float(lean_h))

    zg = TOP + BODY_R + GRIP_TCP_UP
    ins = [
        S_(2.0, (g0[0], g0[1], zg + 0.05), FINGER_OPEN, "above the stylus", pick_yaw),
        S_(1.5, (g0[0], g0[1], zg), FINGER_OPEN, "down to the stylus", pick_yaw),
        S_(0.8, (g0[0], g0[1], zg), STYLUS_GRIP, "grip the body", pick_yaw),
        S_(0.4, (g0[0], g0[1], zg), STYLUS_GRIP, "settle", pick_yaw),
        S_(1.5, (g0[0], g0[1], lift), STYLUS_GRIP, "lift", pick_yaw),
    ]
    t_ins = sum(w.duration for w in ins)
    near = SEAT + Z * 0.040 + U * 0.040  # ball short of the gimbal, toward the bench arms
    ins += [S_(max(2.0, t_brace + 0.4 - t_ins), tcp(near), STYLUS_GRIP, "carry toward the gimbal", ins_yaw, LEAN)]
    ins += [
        S_(1.5, tcp(SEAT + Z * 0.015), STYLUS_GRIP, "ball over the cutout", ins_yaw, LEAN),
        S_(3.0, tcp(SEAT - Z * PUSH), STYLUS_GRIP, "press it in", ins_yaw, LEAN),
        S_(1.0, tcp(SEAT - Z * PUSH), STYLUS_GRIP, "hold", ins_yaw, LEAN),
        S_(1.0, tcp(SEAT), STYLUS_GRIP, "ease off", ins_yaw, LEAN),
        S_(0.8, tcp(SEAT), FINGER_OPEN, "let go", ins_yaw, LEAN),
        S_(1.5, tcp(SEAT) + U * 0.060, FINGER_OPEN, "slide back off the stylus", ins_yaw, LEAN),
        S_(1.0, tcp(SEAT) + U * 0.090 + Z * 0.04, FINGER_OPEN, "lift clear", ins_yaw, LEAN),
    ]
    t_free = sum(w.duration for w in ins)
    brace += [
        B(max(0.5, t_free - t_brace), P, BRACE_GRIP, "hold the gimbal", 0.0),
        B(0.8, P, FINGER_OPEN, "let go", 0.0),
        B(1.5, P + Z * 0.05, FINGER_OPEN, "lift clear", 0.0),
        B(2.0, PARK[BRACE_ARM], FINGER_OPEN, "away", 0.0),
        B(4.0, PARK[BRACE_ARM], FINGER_OPEN, "still", 0.0),
    ]
    ins += [
        S_(0.8 + 1.5 + 1.0, tcp(SEAT) + U * 0.090 + Z * 0.04, FINGER_OPEN, "wait", ins_yaw, LEAN),
        S_(2.0, PARK[INSERT_ARM], FINGER_OPEN, "away", 0.0),
    ]
    end = sum(w.duration for w in brace)
    ins += [S_(max(0.5, end - sum(w.duration for w in ins)), PARK[INSERT_ARM], FINGER_OPEN, "still", 0.0)]
    if os.environ.get("VERSEGRIP_DEMO") == "onehand":  # the one-hand check: same stylus motions, gimbal not held
        brace = [B(end, PARK[BRACE_ARM], FINGER_OPEN, "stand clear (one-hand check)", 0.0)]
    return {BRACE_ARM: brace, INSERT_ARM: ins}
