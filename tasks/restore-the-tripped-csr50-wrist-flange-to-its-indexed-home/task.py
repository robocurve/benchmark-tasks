"""A Universal Robots UR3e arm is bolted to the far side of the bench, stopped and braked in a service pose with its
wrist reaching toward the robot. On its tool flange is a Zimmer CSR50 pneumatic collision sensor (crash protector):
a silver housing 79 mm across and, below it, a 63 mm compensating flange with the standard four-hole tool pattern on
its face. The end tool has been taken off, so the flange's face is bare. A crash has tripped the sensor: the flange
stands about 12 mm out of the housing and is turned well away from home. A small black triangle is marked on the
housing's rim and another on the flange's rim; they point at each other when the flange is home. Compressed air is
connected. Both arms start at their ready pose with the grippers open.
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
INSTRUCTION = ("The collision sensor on the stopped UR3e's wrist has tripped. Reset it by hand: put its flange back "
               "home, where the triangle on the flange's rim meets the triangle on the housing, then let go and move "
               "your hands clear.")
MM = 1e-3
EPS = 1e-6
D = math.radians

UR_GREY, UR_BLUE, UR_DARK = (0.80, 0.81, 0.82), (0.36, 0.52, 0.66), (0.20, 0.21, 0.22)
SILVER, STEEL, BLACK = (0.74, 0.75, 0.77), (0.58, 0.59, 0.61), (0.04, 0.04, 0.04)
HOSE_BLUE, CABLE = (0.15, 0.35, 0.80), (0.08, 0.08, 0.09)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _quat(m):
    return tuple(float(v) for v in parts._matrix_to_quat(np.asarray(m, float)))


# -- the UR3e: published Denavit-Hartenberg parameters, stopped at fixed joint angles -----------------------------
UR_BASE = np.array([0.660, -0.040, TOP])
UR_Q_DEG = (-35.9, -81.8, 133.5, -61.7, 92.4, 0.0)  # base, shoulder, elbow, wrist 1, 2, 3 as the pendant shows them
UR_D = (0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921)
UR_A = (0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0)
UR_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
SHOULDER_OFF, ELBOW_OFF = 0.120, 0.093  # upper-arm and forearm planes off the base axis (estimates)


def ur_frames(q_deg=UR_Q_DEG):
    t = np.eye(4)
    t[:3, 3] = UR_BASE
    out = [t.copy()]
    for th, d, a, al in zip(map(math.radians, q_deg), UR_D, UR_A, UR_ALPHA):
        ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
        t = t @ np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])
        out.append(t.copy())
    return out


FR = ur_frames()
O = [f[:3, 3] for f in FR]
Z = [f[:3, 2] for f in FR]

# -- the CSR50 on the UR3e's tool flange ---------------------------------------------------------------------------
# Zimmer CSR50 (datasheet): installation height 71 mm, housing 79 mm across, compensating flange 63 mm across with
# 12 mm of it below the housing; 12.5 mm axial and 12.5 degree angular travel, 360 degrees about its axis; 0.66 kg.
EZ = _unit(Z[6])  # the sensor's axis, out of the robot (toward the bench's arms)
EX = _unit(np.cross([0.0, 0.0, 1.0], EZ))
EY = np.cross(EZ, EX)  # uppermost direction across the axis: both triangles are marked here
R_CSR = np.column_stack([EX, EY, EZ])
SEAT = O[6] + 0.059 * EZ  # centre of the housing's seat face (the flange's back face at home)
F0 = SEAT + 0.012 * EZ  # centre of the flange's free face at home
APPROACH = -EZ
HEADING = math.atan2(APPROACH[1], APPROACH[0])
LEAN = 90.0 - math.degrees(math.asin(max(-1.0, min(1.0, -APPROACH[2]))))  # degrees off straight down
R_H, H_LEN, GAP = 0.0395, 0.0585, 0.0001  # housing radius and length; seat face 0.1 mm behind the home back face
R_F, F_THK = 0.0315, 0.012
TRAVEL, TILT_MAX = 0.0125, D(12.5)
TURN_LIM = D(175.0)
TRI_W, TRI_H = 0.008, 0.007  # the triangle marks: 8 mm wide, 7 mm tall (estimate)


_ALONG_Z = _quat(parts._x_to((0.0, 0.0, 1.0)))


def _disc(r, length, z, color, collide=True, x=0.0, y=0.0, sides=36):
    """A round body along the sensor's axis (a many-sided prism, so it reads as round on the close wrist cameras)."""
    return parts.prism(r / math.cos(math.pi / sides), length, sides, at=(x, y, z), color=color, collide=collide,
                       quat=_ALONG_Z)


def _tri(r, z_tip, z_base, sign):
    """A triangle mark on a rim of radius r at the +y side, tip at z_tip pointing toward z_base's opposite."""
    rr = r + 0.0004
    return ((0.0, rr, z_tip), (-TRI_W / 2, rr, z_base), (TRI_W / 2, rr, z_base))


def _csr_body():
    shapes = [_disc(R_H, H_LEN, -GAP - H_LEN / 2, SILVER)]
    # M8 proximity switch in its holder on the underside, and the M5 air fitting on the side (with hose and cable).
    shapes += [parts.box((0.014, 0.010, 0.018), at=(0.0, -(R_H + 0.004), -0.030), color=STEEL),
               parts.cylinder(0.004, 0.026, at=(0.0, -(R_H + 0.013), -0.030), color=BLACK, axis="y"),
               parts.cylinder(0.0022, 0.030, at=(0.0, -(R_H + 0.026 + 0.015), -0.030), color=CABLE, axis="y",
                              collide=False),
               parts.cylinder(0.005, 0.012, at=(R_H + 0.006, 0.0, -0.040), color=STEEL, axis="x"),
               parts.cylinder(0.002, 0.040, at=(R_H + 0.032, 0.0, -0.040), color=HOSE_BLUE, axis="x", collide=False)]
    return shapes


def _flange_shapes():
    out = [_disc(R_F, F_THK, F_THK / 2, STEEL)]
    face = F_THK + 0.0002
    out.append(_disc(0.01575, 0.0006, face - 0.0002, UR_DARK, collide=False))
    for k in range(4):  # ISO 9409-1-50-4-M6: four M6 holes on a 50 mm circle, a 6 mm dowel hole between two of them
        a = D(45.0 + 90.0 * k)
        out.append(_disc(0.0025, 0.0006, face - 0.0002, BLACK, collide=False, x=0.025 * math.cos(a),
                         y=0.025 * math.sin(a), sides=16))
    out.append(_disc(0.003, 0.0006, face - 0.0002, BLACK, collide=False, x=-0.025, sides=16))
    return out


def _carrier(name, z):
    return (parts.cylinder(0.006, 0.002, at=(0.0, 0.0, z), color=STEEL, collide=False),)


CSR = parts.Part(
    "csr50", pos=tuple(float(v) for v in SEAT), quat=_quat(R_CSR), shapes=_csr_body(),
    friction=(0.8, 0.7), contact_offset=0.002,
    visuals=(parts.Visual("housing_mark", _tri(R_H, -0.0015, -0.0085, 1), ((0, 1, 2),), BLACK, "base"),
             parts.Visual("flange_mark", _tri(R_F, 0.0015, 0.0085, 1), ((0, 1, 2),), BLACK, "turn")),
    joints=[
        # The piston and flange slide out along the axis (0 = home), tip about two axes and turn freely about it.
        parts.Slider("axial", shapes=_carrier("axial", -0.010), axis=(0.0, 0.0, 1.0), travel=TRAVEL, mass=0.05,
                     spring=0.0, damping=8.0, max_effort=125.0, friction=0.0),
        parts.Hinge("tilt_x", shapes=_carrier("tilt_x", -0.004), pivot=(0.0, 0.0, 0.0), axis=(1.0, 0.0, 0.0),
                    lower=-TILT_MAX, upper=TILT_MAX, mass=0.02, spring=0.0, damping=0.02, max_effort=4.0,
                    parent="axial"),
        parts.Hinge("tilt_y", shapes=_carrier("tilt_y", -0.002), pivot=(0.0, 0.0, 0.0), axis=(0.0, 1.0, 0.0),
                    lower=-TILT_MAX, upper=TILT_MAX, mass=0.02, spring=0.0, damping=0.02, max_effort=4.0,
                    parent="tilt_x"),
        parts.Hinge("turn", shapes=tuple(_flange_shapes()), pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0),
                    lower=-TURN_LIM, upper=TURN_LIM, mass=0.17, spring=0.0, damping=0.03, max_effort=4.0,
                    parent="tilt_y"),
    ],
)


def _seg_prism(p0, p1, r, color, sides=28):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = _unit(p1 - p0)
    rot = parts._x_to(ax)
    return parts.prism(r / math.cos(math.pi / sides), float(np.linalg.norm(p1 - p0)), sides,
                       at=tuple(float(v) for v in (p0 + p1) / 2 - UR_BASE), color=color, quat=_quat(rot))


def _ur_segments():
    """(start, end, radius, colour) of each round body of the stopped arm, world frame."""
    sh, el = SHOULDER_OFF, ELBOW_OFF
    segs = [
        (O[0] + [0, 0, 0.016], O[0] + [0, 0, 0.105], 0.045, UR_GREY),  # base housing
        (O[0] + [0, 0, 0.105], O[1] + [0, 0, 0.045], 0.045, UR_BLUE),  # shoulder pan joint
        (O[1] - Z[1] * 0.045, O[1] + Z[1] * (sh + 0.035), 0.045, UR_BLUE),  # shoulder lift joint
        (O[1] + Z[1] * sh, O[2] + Z[1] * sh, 0.032, UR_GREY),  # upper arm
        (O[2] + Z[2] * (sh - el - 0.030), O[2] + Z[2] * (sh + 0.035), 0.0375, UR_BLUE),  # elbow joint
        (O[2] + Z[2] * (sh - el), O[3] + Z[2] * (sh - el), 0.027, UR_GREY),  # forearm
        (O[3] + Z[3] * (sh - el - 0.030), O[3] + Z[3] * (UR_D[3] + 0.032), 0.0315, UR_BLUE),  # wrist 1
        (O[4] - Z[4] * 0.032, O[4] + Z[4] * (UR_D[4] - 0.030), 0.0315, UR_BLUE),  # wrist 2
        (O[5] - Z[5] * 0.032, O[6] - Z[5] * 0.0005, 0.0315, UR_BLUE),  # wrist 3, ending at the tool flange
    ]
    return segs


UR_SEGS = _ur_segments()
UR3E = parts.Part(
    "ur3e", pos=tuple(float(v) for v in UR_BASE),
    shapes=[parts.cylinder(0.064, 0.016, at=(0.0, 0.0, 0.008), color=UR_GREY)]
    + [_seg_prism(a, b, r, c) for a, b, r, c in UR_SEGS],
)

OBJECTS = [UR3E, CSR]
DEMO_TOUCHES = {"csr50"}  # the fingertips are commanded past the flange's home to load the push (the arm gives way)

# -- the tripped start: out at the axial stop, turned 75 degrees one way or the other --------------------------------
START_AXIAL = TRAVEL
LOTS = lots.declare(OBJECTS, lots.Lot("A", turn=75.0), lots.Lot("B", turn=-75.0))

# -- the sensor's mechanism (on_step): vented and loose, the seat's lands and lead-in, pressure back, ready ----------
LAND = 0.0025  # off the mark, the flange's seat balls ride on the lands this far out (estimate)
LAND_OFF = 0.0035  # and leave them once more than this far out
INDEX = D(5.0)  # lead-in: within 5 degrees of the mark the balls find their grooves
GUIDE_DEPTH = 0.003  # the grooves' lead-in acts within 3 mm of home
K_LAND, K_GUIDE = 40000.0, 0.5  # N/m on the lands (125 N cap); N m/rad of the grooves' centring on turn and tilt
CAPTURE_GAP, CAPTURE_TURN = 0.0008, D(2.5)  # seated within 0.8 mm all round on the mark: membrane closed, pressure builds
PRESSURE_S = 1.0  # time to rebuild pressure once closed (estimate)
TRIP_AX, TRIP_TILT, TRIP_TURN = 0.0015, D(2.0), D(4.0)  # a seated flange forced this far trips again
K_AX_READY, K_ROT_READY = 20000.0, 30.0  # seated and pressurised: N/m (125 N cap), N m/rad (4 N m cap)
# Vented, the flange stays where it is left: its seals' friction, a stick-slip element per joint (stiffness and the
# slip it takes to break away): 1.5 N along the axis, 0.05 N m to turn or tip it (estimates).
STICK = {"axial": (3000.0, 0.0005), "tilt_x": (2.0, 0.025), "tilt_y": (2.0, 0.025), "turn": (2.0, 0.025)}
JOINTS = ("axial", "tilt_x", "tilt_y", "turn")

# -- scoring thresholds ------------------------------------------------------------------------------------------------
LINED_UP = D(5.0)  # triangles together within 5 degrees (2.7 mm along the 63 mm rim)
FLUSH_GAP, FLUSH_TURN = 0.0010, D(2.5)  # flush: 1 mm or less all round, on the mark
HAND_CLEAR = 0.020  # every hand surface this far from the sensor (housing and flange) for the final hold
STILL_AX, STILL_ROT = 0.0003, D(0.3)
HOLD_S = 1.0


def rim_gap(j):
    """Largest gap (m) between the flange's back face and its seat, all round: out along the axis plus the tilt at
    the 63 mm rim."""
    return max(0.0, float(j["axial"])) + R_F * math.sin(tilt_of(j))


def tilt_of(j):
    """Angle (rad) between the flange's axis and the housing's axis."""
    c = math.cos(float(j["tilt_x"])) * math.cos(float(j["tilt_y"]))
    return math.acos(max(-1.0, min(1.0, c)))


def _R(q):
    return parts._quat_to_matrix(q)


# -- the trial record (written only by the simulator's step and reset hooks) ------------------------------------------


def _fresh():
    return dict(known=False, invalid=False, why="", t=None, mode="vented", lands=False, guide=False,
                cap_t=0.0, indexed=False, flush=False, held=0.0, ref=None, clear=False, still=False, stamp=None,
                hand_gap=None, drives=None, anchor=None)


REC = _fresh()


def stamp(objects):
    j = objects[CSR.name].joints
    return tuple(round(float(j[k]), 9) for k in ("axial", "tilt_x", "tilt_y", "turn"))


def mechanism(objects, dt):
    """Advance the sensor's state from the flange's joints; returns the drives it should have now."""
    j = objects[CSR.name].joints
    ax, tl, err = float(j["axial"]), tilt_of(j), abs(float(j["turn"]))
    if REC["mode"] == "ready":
        if ax > TRIP_AX + EPS or tl > TRIP_TILT + EPS or err > TRIP_TURN + EPS:
            REC.update(mode="vented", cap_t=0.0)
    else:
        if rim_gap(j) <= CAPTURE_GAP + EPS and err <= CAPTURE_TURN + EPS:
            REC["cap_t"] += dt
            if REC["cap_t"] >= PRESSURE_S - EPS:
                REC["mode"] = "ready"
        else:
            REC["cap_t"] = 0.0
    q = {k: float(j[k]) for k in JOINTS}
    if REC["anchor"] is None or REC["mode"] == "ready":
        REC["anchor"] = dict(q)
    else:
        for k, (_, slip) in STICK.items():  # the seal sticks until pulled more than its slip, then follows
            a = REC["anchor"][k]
            REC["anchor"][k] = min(max(a, q[k] - slip), q[k] + slip)
    if REC["mode"] == "vented":
        if not REC["lands"] and err > INDEX + EPS and ax < LAND - EPS:
            REC["lands"] = True
        elif REC["lands"] and (ax > LAND_OFF + EPS or err <= INDEX + EPS):
            REC["lands"] = False
        REC["guide"] = ax <= GUIDE_DEPTH + EPS and err <= INDEX + EPS
    else:
        REC["lands"] = REC["guide"] = False
    if REC["mode"] == "ready":
        return (("axial", K_AX_READY, 0.0), ("tilt_x", K_ROT_READY, 0.0), ("tilt_y", K_ROT_READY, 0.0),
                ("turn", K_ROT_READY, 0.0))
    anc = REC["anchor"]
    out = [("axial", K_LAND, LAND) if REC["lands"] else ("axial", STICK["axial"][0], anc["axial"])]
    for k in ("tilt_x", "tilt_y", "turn"):
        out.append((k, K_GUIDE, 0.0) if REC["guide"] else (k, STICK[k][0], anc[k]))
    return tuple((k, float(kk), round(float(v), 7)) for k, kk, v in out)


def _cyl_dist(points, centre, axis, half_len, radius):
    """Distance from each point to a solid finite cylinder (negative inside)."""
    v = points - centre
    z = v @ axis
    rr = np.linalg.norm(v - np.outer(z, axis), axis=1)
    dr, dz = rr - radius, np.abs(z) - half_len
    out = np.sqrt(np.maximum(dr, 0.0) ** 2 + np.maximum(dz, 0.0) ** 2)
    inside = (dr < 0) & (dz < 0)
    out[inside] = np.maximum(dr, dz)[inside]
    return out


def csr_cylinders(objects):
    """(centre, axis, half length, radius) of the housing and of the flange where it is now."""
    st = objects[CSR.name]
    p, r = np.asarray(st.pos, float), _R(st.quat_xyzw)
    hz = r[:, 2]
    out = [(p + hz * (-GAP - H_LEN / 2), hz, H_LEN / 2, R_H)]
    lp, lq = st.links["turn"]
    lr = _R(lq)
    out.append((np.asarray(lp, float) + lr[:, 2] * (F_THK / 2), lr[:, 2], F_THK / 2, R_F))
    return out


def _near(points, cyls, reach=0.10):
    best = reach
    for c, a, hl, r in cyls:
        lo, hi = c - (hl + r + reach), c + (hl + r + reach)
        keep = np.all((points >= lo) & (points <= hi), axis=1)
        if np.any(keep):
            best = min(best, float(_cyl_dist(points[keep], c, a, hl, r).min()))
    return best


def joints_now(objects):
    j = objects[CSR.name].joints
    return np.array([float(j["axial"]), float(j["tilt_x"]), float(j["tilt_y"]), float(j["turn"])])


def observe(objects, hands, t):
    """Update the trial record from one control step's states and hand surface points (None: unknown)."""
    try:
        q = joints_now(objects)
        ok = bool(np.all(np.isfinite(q)))
    except (KeyError, TypeError, ValueError, AttributeError):
        ok = False
    dt = None if REC["t"] is None else float(t) - REC["t"]
    if not ok or hands is None or (dt is not None and not 0.0 < dt <= 0.2):
        REC.update(known=False, invalid=True, held=0.0, t=float(t), stamp=None)
        return
    j = objects[CSR.name].joints
    err = abs(float(j["turn"]))
    # 1. Progress reached at any time.
    if err <= LINED_UP + EPS:
        REC["indexed"] = True
    if rim_gap(j) <= FLUSH_GAP + EPS and err <= FLUSH_TURN + EPS:
        REC["flush"] = True
    # 2. The end hold: ready, every hand surface clear of the sensor, the flange still.
    gap = _near(hands, csr_cylinders(objects))
    REC["hand_gap"] = gap
    clear = gap >= HAND_CLEAR - EPS
    ref = REC["ref"]
    still = ref is not None and abs(q[0] - ref[0]) <= STILL_AX + EPS and np.all(np.abs(q[1:] - ref[1:]) <= STILL_ROT + EPS)
    if REC["mode"] == "ready" and clear and still and dt is not None:
        REC["held"] += dt
    else:
        REC["held"], REC["ref"] = 0.0, q
    REC.update(known=True, t=float(t), stamp=stamp(objects), clear=clear, still=still)


def tier():
    if REC["mode"] == "ready":
        return 1.0 if REC["held"] >= HOLD_S - EPS else 0.75
    if REC["flush"]:
        return 0.5
    if REC["indexed"]:
        return 0.25
    return 0.0


def score(objects):
    """0 with an unknown or stale record; otherwise the highest tier reached (see tier())."""
    try:
        if not REC["known"] or REC["invalid"] or REC["stamp"] != stamp(objects):
            return 0.0
        return float(tier())
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# -- simulator hooks: hand samples, the sensor's drives, the record --------------------------------------------------


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
            clouds.append(pts @ _R(pose[3:]).T + pose[:3] - origin)
    return np.concatenate(clouds)


def _drive(sim, drives):
    if drives == REC.get("drives"):
        return
    for name, k, target in drives:
        sim.set_stiffness(CSR.name, name, k)
        sim.set_target(CSR.name, name, target)
    REC["drives"] = drives


VENTED = (("axial", 0.0, 0.0), ("tilt_x", 0.0, 0.0), ("tilt_y", 0.0, 0.0), ("turn", 0.0, 0.0))


@parts.on_reset
def _reset(sim):
    REC.clear()
    REC.update(_fresh())
    _drive(sim, VENTED)


_lots_apply = lots._apply


def _apply_lot(emb):
    """Each trial: put the flange in the lot's tripped start (out at its stop, turned), vented, then settle."""
    _lots_apply(emb)
    lot = lots.current()
    sim = parts.Sim(emb)
    for name, value in (("axial", START_AXIAL), ("tilt_x", 0.0), ("tilt_y", 0.0), ("turn", D(float(lot.turn)))):
        sim.write_joint(CSR.name, name, value)
    REC.clear()
    REC.update(_fresh())
    start = {"axial": START_AXIAL, "tilt_x": 0.0, "tilt_y": 0.0, "turn": D(float(lot.turn))}
    _drive(sim, tuple((k, STICK[k][0], v) for k, v in start.items()))
    REC["anchor"] = dict(start)
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
        dt = 0.0 if REC["t"] is None else max(0.0, t - REC["t"])
        _drive(sim, mechanism(objects, dt))
    except (KeyError, TypeError, ValueError, AttributeError) as e:
        REC.update(known=False, invalid=True, why="sensor drive: " + str(e))
    observe(objects, hands, t)
    if os.environ.get("TRACE_OUT"):
        j = objects[CSR.name].joints
        row = dict(t=round(t, 3), lot=lots.current().name, axial_mm=round(float(j["axial"]) * 1000, 3),
                   tilt_deg=round(math.degrees(tilt_of(j)), 3), turn_deg=round(math.degrees(float(j["turn"])), 3),
                   mode=REC["mode"], lands=REC["lands"], guide=REC["guide"], cap_t=round(REC["cap_t"], 2),
                   indexed=REC["indexed"], flush=REC["flush"], rim_gap_mm=round(rim_gap(j) * 1000, 3), hand_gap=REC["hand_gap"],
                   held=round(REC["held"], 2), why=REC["why"], score=score(objects))
        with open(pathlib.Path(os.environ["TRACE_OUT"]).with_name("mechanism.jsonl"), "a") as f:
            f.write(json.dumps(row) + "\n")


# -- the scripted demo: open loop from the published layout ----------------------------------------------------------
Z_TCP = 0.0140  # TCP 14 mm out from the seat face along the axis: the fingertips stop 1.7 mm short of the housing
GRIP = -0.0285  # finger target: 3 mm past the 63 mm rim on each side
PUSH_OVER = 0.008  # commanded past home along the axis, for the arm's give
OUT_CLEAR = 0.070


def tcp_at(axial, extra=0.0):
    return SEAT + EZ * (axial + Z_TCP + extra)


def _W(dt, p, finger, label, roll):
    return tilt.Waypoint(float(dt), tuple(float(v) for v in p), float(finger), label, yaw=float(roll),
                         tilt=float(LEAN), heading=float(HEADING))


def demo():
    lot = lots.current()
    turn0 = D(float(lot.turn))
    r0, r1 = turn0 / 2, -turn0 / 2  # the grip turns with the flange: start half one way, end half the other
    left = [
        _W(2.0, tcp_at(START_AXIAL, OUT_CLEAR), FINGER_OPEN, "line up on the axis", r0),
        _W(1.6, tcp_at(START_AXIAL), FINGER_OPEN, "slide over the flange", r0),
        _W(0.8, tcp_at(START_AXIAL), GRIP, "grip the rim", r0),
        _W(0.4, tcp_at(START_AXIAL), GRIP, "settle", r0),
        _W(3.0, tcp_at(START_AXIAL), GRIP, "turn to the mark", r1),
        _W(0.4, tcp_at(START_AXIAL), GRIP, "settle", r1),
        _W(2.0, tcp_at(0.0, -PUSH_OVER), GRIP, "push home", r1),
        _W(2.2, tcp_at(0.0, -PUSH_OVER), GRIP, "hold while the air re-seats it", r1),
        _W(0.8, tcp_at(0.0, -PUSH_OVER), FINGER_OPEN, "let go", r1),
        _W(1.5, tcp_at(0.0, OUT_CLEAR), FINGER_OPEN, "back off along the axis", r1),
        tilt.Waypoint(2.0, (0.18, 0.30, TOP + 0.16), FINGER_OPEN, "hands away"),
        tilt.Waypoint(2.0, (0.18, 0.30, TOP + 0.16), FINGER_OPEN, "still"),
    ]
    right = [tilt.Waypoint(2.0, (0.18, -0.30, TOP + 0.16), FINGER_OPEN, "stand clear"),
             tilt.Waypoint(sum(w.duration for w in left) - 2.0, (0.18, -0.30, TOP + 0.16), FINGER_OPEN, "wait")]
    return {"left": left, "right": right}
