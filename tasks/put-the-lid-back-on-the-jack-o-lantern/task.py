"""A carved pumpkin (a jack-o'-lantern) stands on the bench in front of the robot, its carved face toward one side of
the bench. Its lid, cut with the stem on it, rests on the opening in the top. The lid and the opening were cut with
the knife leaning inward, so their edges are matching cones, and one place round the cut has a key: a notch cut in
the lid's edge and a tooth left on the opening's rim. Where the pumpkin faces and how the lid lies change between
trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402  (only for its waypoint rotation; the grip is straight down)
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
S45 = math.sqrt(0.5)

SKIN_C = (0.93, 0.47, 0.08)  # pumpkin skin
RIB_C = (0.78, 0.36, 0.05)
FLESH_C = (0.98, 0.78, 0.36)  # cut face colour
DARK_C = (0.10, 0.06, 0.03)  # carved openings (dark inside)
STEM_C = (0.42, 0.36, 0.20)


def _f(v):
    """Plain Python floats: a position of NumPy floats crashes the simulator at reset."""
    return tuple(float(x) for x in v)


# -- the cut (m). Both cones are 45 degrees: the cut surface's radius grows by 1 mm for each 1 mm up. ---------------
R_LID = 0.0325  # lid's cut edge at its top face: 65 mm across
H_LID = 0.012  # lid thickness at the cut: its bottom edge is 41 mm across
CLEAR = 0.0005  # radial clearance of the knife cut
R_HOLE = R_LID + CLEAR  # opening's cut edge at the rim: 66 mm across
H_WALL = 0.015  # pumpkin wall at the top: the opening's cut runs 15 mm down, to 36 mm across
KEY_D = 0.008  # the key: tooth 8 mm deep (radially) and 10 mm wide; notch 2 mm wider each side and 2 mm deeper
KEY_W = 0.010
NOTCH_W = KEY_W + 0.004
NOTCH_GAP = 0.002

# -- the pumpkin, own frame: origin on the bench under the opening's axis, +x behind the face (the key side), z up --
Z_RIM = 0.105  # rim of the opening (cut top) above the bench
PUMPKIN_R = 0.073  # 146 mm across at the widest
PUMPKIN_KG = 0.80  # estimate for a hollowed pumpkin of this size; weigh it
LID_KG = 0.030  # ballasted lid mass (foam plus washer); weigh it


def _cone_box(phi, r0, z_lo, z_hi, t, width, side, color, y_off=0.0):
    """A flat facet of the 45-degree cut. The cut surface has radius r0 + z; the facet's face lies on it, tangent at
    angle phi, from height z_lo to z_hi; the box is t thick behind the surface (side -1: outward, into the pumpkin
    wall; +1: inward, into the lid). ``y_off`` moves the facet sideways (tangentially)."""
    zm = 0.5 * (z_lo + z_hi)
    rm = r0 + zm
    nr, nz = -S45 * side * t / 2, S45 * side * t / 2  # half the thickness along the inward-up normal (r, z)
    c, s = math.cos(phi), math.sin(phi)
    x, y = (rm + nr) * c - y_off * s, (rm + nr) * s + y_off * c
    return parts.box(((z_hi - z_lo) / S45, width, t), at=(x, y, zm + nz), color=color, yaw=phi, pitch=-math.pi / 4)


def _ring_box(phi, r_in, r_out, z_lo, z_hi, width, color, collide=True):
    rm = 0.5 * (r_in + r_out)
    return parts.box((r_out - r_in, width, z_hi - z_lo), at=(rm * math.cos(phi), rm * math.sin(phi), 0.5 * (z_lo + z_hi)),
                     color=color, yaw=phi, collide=collide)


N_SEG = 24


def _pumpkin_shapes():
    out = []
    # the opening's cut: 24 facets 8 mm thick behind the cone surface, from 2 mm below the wall to the rim
    t = 0.008
    w = 2 * (R_HOLE + 0.002) * math.tan(math.pi / N_SEG) + 0.001
    for k in range(N_SEG):
        out.append(_cone_box(2 * math.pi * k / N_SEG, R_HOLE, -H_WALL, 0.0, t, w, -1, FLESH_C))
    # the tooth at the back (+x): the cut surface stands 8 mm further in over a 10 mm width, the whole wall deep
    out.append(_cone_box(0.0, R_HOLE - KEY_D, -H_WALL, 0.0, t + KEY_D * S45, KEY_W, -1, FLESH_C))
    # the top skin round the opening, 4 mm thick, out to the shoulder
    for k in range(N_SEG):
        phi = 2 * math.pi * (k + 0.5) / N_SEG
        out.append(_ring_box(phi, R_HOLE, 0.0585, -0.004, 0.0, 2 * 0.0585 * math.tan(math.pi / N_SEG) + 0.0005, SKIN_C))
    # the rest is a 15 mm wall: 24 staves round the bulge, a flat bottom (collision), and the rounded look (no collision)
    z0 = Z_RIM
    for k in range(N_SEG):
        phi = 2 * math.pi * (k + 0.5) / N_SEG
        out.append(_ring_box(phi, 0.044, 0.0585, 0.012 - z0, -0.004, 2 * 0.0585 * math.tan(math.pi / N_SEG) + 0.0005,
                             SKIN_C))
    out.append(parts.cylinder(0.052, 0.012, at=(0.0, 0.0, 0.006 - z0), color=FLESH_C))
    for r, z_lo, z_hi in ((0.050, 0.000, 0.010), (0.064, 0.010, 0.024), (0.071, 0.024, 0.040), (PUMPKIN_R, 0.040, 0.072),
                          (0.069, 0.072, 0.082), (0.063, 0.082, 0.088)):
        out.append(parts.cylinder(r, z_hi - z_lo, at=(0.0, 0.0, 0.5 * (z_lo + z_hi) - z0), color=SKIN_C, collide=False))
    out.append(parts.cylinder(0.040, 0.001, at=(0.0, 0.0, 0.0885 - z0), color=DARK_C, collide=False))  # inside: dark
    for k in range(10):  # ribs
        phi = 2 * math.pi * (k + 0.5) / 10
        out.append(_ring_box(phi, PUMPKIN_R - 0.004, PUMPKIN_R + 0.0015, 0.030 - z0, 0.080 - z0, 0.004, RIB_C,
                             collide=False))
    # the carved face, toward -x: two eyes, a nose and a mouth (dark cut-outs, drawn on the skin; no collision)
    fx = -(PUMPKIN_R + 0.001)
    for y in (-0.024, 0.024):
        out.append(parts.box((0.004, 0.018, 0.018), at=(fx, y, 0.068 - z0), color=DARK_C, roll=math.pi / 4,
                             collide=False))
    out.append(parts.box((0.004, 0.010, 0.010), at=(fx, 0.0, 0.052 - z0), color=DARK_C, roll=math.pi / 4,
                         collide=False))
    out.append(parts.box((0.004, 0.056, 0.010), at=(fx, 0.0, 0.034 - z0), color=DARK_C, collide=False))
    for y in (-0.014, 0.014):
        out.append(parts.box((0.004, 0.008, 0.006), at=(fx, y, 0.041 - z0), color=SKIN_C, collide=False))
    return out


def _lid_shapes():
    """The lid, own frame: origin at the middle of its top face, +x toward its notch, z up."""
    out = []
    t = 0.003
    top = -t * S45  # facets stop so their inner top edge is level with the top face
    w = 2 * R_LID * math.tan(math.pi / N_SEG) + 0.001
    half = 0.5 * NOTCH_W + 0.5 * w + 0.0005
    for k in range(N_SEG):
        phi = 2 * math.pi * (k + 0.5) / N_SEG
        if abs((R_LID - H_LID) * math.sin(phi)) < half and math.cos(phi) > 0:
            continue  # the notch
        out.append(_cone_box(phi, R_LID, -H_LID, top, t, w, 1, FLESH_C))
    # the notch: its floor (the cut moved 8.5 mm in) and two side walls, on the lower part of the cut
    out.append(_cone_box(0.0, R_LID - KEY_D - NOTCH_GAP, -H_LID, top, t, NOTCH_W, 1, FLESH_C))
    tw = (KEY_D + NOTCH_GAP) * S45 + t
    for s in (-1, 1):
        out.append(_cone_box(0.0, R_LID - 0.0016, -H_LID, -tw * S45, tw, 0.003, 1, FLESH_C,
                             y_off=s * (0.5 * NOTCH_W + 0.0015)))  # 1.6 mm in: the cone curves away at 7.5 mm aside
    # the top skin: a ring of 4 mm plates, open over the notch, and a centre plug under the stem
    r_in, r_out = 0.011, R_LID - 2 * t * S45
    for k in range(N_SEG):
        phi = 2 * math.pi * (k + 0.5) / N_SEG
        c, s = math.cos(phi), math.sin(phi)
        if c > 0 and abs(r_out * s) < 0.5 * NOTCH_W + 0.002 + 0.5 * 2 * r_out * math.tan(math.pi / N_SEG):
            out.append(_ring_box(phi, r_in, R_LID - KEY_D - NOTCH_GAP - 0.0035, -0.004, 0.0,
                                 2 * r_out * math.tan(math.pi / N_SEG) + 0.001, SKIN_C))
            continue
        out.append(_ring_box(phi, r_in, r_out, -0.004, 0.0, 2 * r_out * math.tan(math.pi / N_SEG) + 0.001, SKIN_C))
    out.append(parts.cylinder(0.012, 0.004, at=(0.0, 0.0, -0.002), color=SKIN_C))
    out.append(parts.cylinder(0.0075, 0.026, at=(0.0, 0.0, 0.013), color=STEM_C))  # the stem: 15 mm across, 26 mm tall
    return out


STEM_H = 0.026
SEAT_DROP = 0.0  # a keyed lid seats with its top face level with the rim (MuJoCo check: within 0.1 mm)

# -- layout (world: x away from the robot, y to its left, z up). The left arm does the work. -----------------------
P_XY = (0.300, 0.110)


def _lid_start(p_yaw, wrong_deg):
    """Lid start: over the opening, turned ``wrong_deg`` from keyed, 10 mm up (it settles onto the tooth)."""
    return (P_XY[0], P_XY[1], TOP + Z_RIM + 0.010), p_yaw + math.radians(wrong_deg)


# lot: pumpkin yaw (face direction is pumpkin -x), lid start wrong angle (degrees, + = counterclockwise from above)
LOT_SPEC = {"A": (0.0, 110.0), "B": (math.radians(90.0), -75.0), "C": (math.radians(-55.0), 155.0)}

_lp, _ly = _lid_start(*LOT_SPEC["A"])
LID = parts.Part("lid", pos=_f(_lp), yaw=float(_ly), mass=LID_KG, shapes=_lid_shapes(), friction=(0.6, 0.5),
                 contact_offset=0.001, com=(0.0, 0.0, -0.003), inertia=(6.0e-6, 6.0e-6, 1.0e-5))
PUMPKIN = parts.Part("pumpkin", pos=_f((P_XY[0], P_XY[1], TOP + Z_RIM)), yaw=0.0, mass=PUMPKIN_KG,
                     shapes=_pumpkin_shapes(), friction=(0.6, 0.5), contact_offset=0.001,
                     com=(0.0, 0.0, 0.050 - Z_RIM), inertia=(0.0021, 0.0021, 0.0026))
OBJECTS = [PUMPKIN, LID]


def _lot(name):
    p_yaw, wrong = LOT_SPEC[name]
    lp, ly = _lid_start(p_yaw, wrong)
    return lots.Lot(name, place={"pumpkin": PUMPKIN.pos, "lid": _f(lp)}, yaw={"pumpkin": p_yaw, "lid": float(ly)})


LOTS = lots.declare(OBJECTS, *(_lot(n) for n in LOT_SPEC))
SUCCESS_HOLD_S = 1.5  # the 1 s hold is enforced in score() itself; this only keeps a run going past success


# -- scoring geometry ------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _wrap_deg(a) -> float:
    return (float(a) + 180.0) % 360.0 - 180.0


def measures(objects) -> dict:
    """The lid in the pumpkin's frame: its top centre's height above the rim (m), its distance off the opening's axis
    (m), its tilt from the pumpkin's axis and the key error (the notch's direction from the tooth's, about the axis)
    (degrees)."""
    lid, pk = objects["lid"], objects["pumpkin"]
    rp, rl = _rot(pk.quat_xyzw), _rot(lid.quat_xyzw)
    p = rp.T @ (np.asarray(lid.pos, float) - np.asarray(pk.pos, float))
    r = rp.T @ rl
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    key = math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))
    return dict(height=float(p[2]) + SEAT_DROP, off=float(math.hypot(p[0], p[1])), tilt=tilt_deg, key=_wrap_deg(key))


KEY_TOL = 8.0  # degrees
HEIGHT_TOL = 0.002  # lid top within 2 mm of level with the rim
TILT_TOL = 3.0  # degrees
OFF_TOL = 0.003  # lid centre within 3 mm of the opening's axis


def seated(objects) -> bool:
    m = measures(objects)
    return bool(abs(m["key"]) <= KEY_TOL + EPS and abs(m["height"]) <= HEIGHT_TOL + EPS and m["tilt"] <= TILT_TOL + EPS
                and m["off"] <= OFF_TOL + EPS)


def fallen_in(objects) -> bool:
    """The lid has gone down through the opening: its top centre more than 20 mm below the rim, inside the cut."""
    m = measures(objects)
    return bool(m["height"] < -0.020 - EPS and m["off"] < R_HOLE + 0.010)


# -- trial record ------------------------------------------------------------------------------------------------------
CLEAR_M = 0.005  # hands more than 5 mm from the lid
STILL_M = 0.001  # nothing moves more than 1 mm during the hold
HOLD_S = 1.0
PUMPKIN_SHOVE, PUMPKIN_TURN = 0.010, 5.0  # the pumpkin pushed more than 10 mm or turned/tipped more than 5 degrees
_rec = parts.__dict__.setdefault("jack_lid_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", gap=None, since=None, held=False, snap=None, t=0.0, pk0=None,
                shoved=False, fell=False, worst_shove=0.0, worst_turn=0.0)


if not _rec:
    reset_record()


def _snap(objects) -> np.ndarray:
    lid, pk = objects["lid"], objects["pumpkin"]
    rl, rp = _rot(lid.quat_xyzw), _rot(pk.quat_xyzw)
    pts = [np.asarray(lid.pos) + rl @ p for p in (np.zeros(3), np.array([R_LID, 0, 0]), np.array([0, R_LID, 0]),
                                                  np.array([0, 0, STEM_H]))]
    pts += [np.asarray(pk.pos) + rp @ p for p in (np.zeros(3), np.array([PUMPKIN_R, 0, -0.05]),
                                                 np.array([0, PUMPKIN_R, -0.05]))]
    return np.concatenate(pts)


def _pumpkin_move(objects):
    pk = objects["pumpkin"]
    p0, r0 = _rec["pk0"]
    r = _rot(pk.quat_xyzw)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r0.T @ r) - 1) / 2))))
    return float(np.linalg.norm(np.asarray(pk.pos, float) - p0)), ang


def observe(objects, gap, t) -> None:
    """One control step. ``gap``: closest sampled hand point to the lid (m), or None when the hands could not be read,
    which ends all credit for the trial."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        lid, pk = objects["lid"], objects["pumpkin"]
        vals = [*lid.pos, *lid.quat_xyzw, *pk.pos, *pk.quat_xyzw]
        if not all(math.isfinite(float(v)) for v in vals):
            raise ValueError("lid or pumpkin state unknown")
        if _rec["pk0"] is None:
            _rec["pk0"] = (np.asarray(pk.pos, float).copy(), _rot(pk.quat_xyzw))
        d, ang = _pumpkin_move(objects)
        _rec["worst_shove"] = max(_rec["worst_shove"], d)
        _rec["worst_turn"] = max(_rec["worst_turn"], ang)
        if d > PUMPKIN_SHOVE + EPS or ang > PUMPKIN_TURN + EPS:
            _rec["shoved"] = True
        if fallen_in(objects):
            _rec["fell"] = True
        if gap is None or not math.isfinite(gap):
            raise ValueError("hand state unknown")
        ok = seated(objects)
        snap = _snap(objects)
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    _rec["gap"] = float(gap)
    if not ok or gap <= CLEAR_M + EPS:
        _rec.update(since=None, held=False, snap=None)
    elif _rec["since"] is None or float(np.abs(snap - _rec["snap"]).max()) > STILL_M + EPS:
        _rec.update(since=float(t), held=False, snap=snap)
    else:
        _rec["held"] = float(t) - _rec["since"] >= HOLD_S - EPS


def score(objects):
    """0 for the whole trial if the hands, the lid or the pumpkin could not be read at any step, if the pumpkin was
    ever pushed more than 10 mm or turned or tipped more than 5 degrees, or if the lid ever fell in through the
    opening. Otherwise 1.0 when, with every gripper and fingertip more than 5 mm from the lid and nothing moving more
    than 1 mm for 1 s, the lid is keyed and seated: its notch within 8 degrees of the tooth, its top within 2 mm of
    level with the rim, tilted under 3 degrees and within 3 mm of the opening's axis. 0 otherwise."""
    try:
        if not _rec["known"] or _rec["shoved"] or _rec["fell"] or _rec["steps"] == 0 or not _rec["held"]:
            return 0.0
        if _rec["gap"] is None or _rec["gap"] <= CLEAR_M + EPS:
            return 0.0
        if not seated(objects):
            return 0.0
        if _rec["snap"] is None or float(np.abs(_snap(objects) - _rec["snap"]).max()) > STILL_M + EPS:
            return 0.0
        return 1.0
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        m = measures(objects)
        return abs(m["key"]) / 180.0 + abs(m["height"]) * 20 + m["off"] * 20
    except (KeyError, ValueError, TypeError, AttributeError):
        return 1.0


# -- hands: hull faces sampled every 3 mm ------------------------------------------------------------------------------
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step=0.003) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            ii, jj = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing="ij")
            keep = (ii + jj) <= m
            s_, t_ = ii[keep][:, None] / m, jj[keep][:, None] / m
            pts.append(c + (a - c) * s_ + (b - c) * t_)
    pts = np.concatenate(pts)
    _, idx = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(idx)]])


def link_points(name) -> np.ndarray:
    if name not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[name] = np.concatenate([_hull_face_points(h) for h in hs])
    return _FACES[name]


def lid_boxes(lid):
    """World boxes of every lid shape (the round plug and stem count as their boxes)."""
    return [(c, r, s) for _, c, r, s in LID.world_boxes({"base": (np.asarray(lid.pos, float), _rot(lid.quat_xyzw))})]


def hand_gap(objects, pts) -> float:
    """Closest hand point to the lid or its stem (m; negative inside)."""
    from yam_arena.analysis import box_signed_distance

    lid = objects["lid"]
    pts = pts[np.linalg.norm(pts - np.asarray(lid.pos, float), axis=1) < 0.30]
    if len(pts) == 0:
        return 1.0
    return float(min(box_signed_distance(pts, c, r, s).min() for c, r, s in lid_boxes(lid)))


def _hand_points(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                out.append(link_points(n) @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


# -- simulator hooks ---------------------------------------------------------------------------------------------------
@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["last_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}
    g = None
    try:
        pts = _hand_points(sim)
        g = hand_gap(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("lid.jsonl")) if os.environ.get("TRACE_OUT") else None
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            m = measures(objects)
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), seated=seated(objects),
                       **{k: round(v, 5) for k, v in m.items()}, lid=np.round(np.asarray(objects["lid"].pos), 4).tolist(),
                       gap=_rec["gap"], held=_rec["held"], shove=round(_rec["worst_shove"], 4),
                       turn=round(_rec["worst_turn"], 2), fell=_rec["fell"], known=_rec["known"], reason=_rec["reason"])
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            a = base.scene["left_robot"]
            poses = parts._np(a.data.body_link_pose_w)[0]
            row["left_g"] = np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist()
            q = parts._np(a.data.joint_pos)[0]
            row["left_q"] = np.round(q, 3).tolist()
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the published layout (which way the face points; where the lid's notch is) ------------------
F_GRIP, F_LET = -0.0045, -0.0200  # fingers across the 15 mm stem
PERCH = 0.005  # a lid resting on the tooth stands about this much proud (published start, settled)
PINCH = 0.024  # TCP this far above the lid's top face: the pads hold the upper 15 mm of the stem
LIFT = 0.012  # the lid is lifted this far to turn it clear of the tooth


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def demo():
    lot = lots.current()
    p_yaw = lot.yaw.get("pumpkin", 0.0)
    l_yaw = lot.yaw.get("lid", LID.yaw)
    turn = math.radians(_wrap_deg(math.degrees(p_yaw - l_yaw)))  # the notch's way round to the tooth, shortest
    x, y = P_XY
    rim = TOP + Z_RIM
    # The perched lid leans along the tooth's direction (its tooth side is propped up), so the fingers close along
    # that direction: the pads then stand the stem upright as they pinch it. Of the two such grips, take the one
    # whose turn keeps the wrist nearer its middle.
    g0 = min((p_yaw - math.pi / 2 + k * math.pi for k in (-1, 0, 1)),
             key=lambda g: max(abs(g), abs(g + turn)))
    g0 = min(g0, math.radians(86.0))  # the wrist's reach at this spot ends just short of +90 degrees
    g1 = g0 + turn
    gm = g0 + turn / 2
    grab = (x, y, rim + PERCH + PINCH)
    up = (x, y, rim + PERCH + PINCH + LIFT)
    low = (x, y, rim - SEAT_DROP + PINCH + 0.004)
    seat = (x, y, rim - SEAT_DROP + PINCH)  # the pinch height on a seated lid's stem
    # after the drop, square it up: pinch the stem four times, turning a quarter turn between pinches, so the pads
    # stand the stem upright both ways and centre it, and the lid settles level in the cut
    g2 = g1 + math.pi / 2 if g1 < 0 else g1 - math.pi / 2
    W = Waypoint
    left = [
        W(2.0, (x, y, rim + 0.070), FINGER_OPEN, "over the stem", g0),
        W(1.5, grab, FINGER_OPEN, "fingers either side of the stem", g0),
        W(0.8, grab, F_GRIP, "pinch the stem", g0),
        W(0.4, grab, F_GRIP, "settle", g0),
        W(1.0, up, F_GRIP, "lift the lid off the tooth", g0),
        W(1.5, up, F_GRIP, "turn the notch round", gm),
        W(1.5, up, F_GRIP, "turn the notch round to the tooth", g1),
        W(1.0, low, F_GRIP, "lower it to just above the cut", g1),
        W(0.5, low, F_GRIP, "settle", g1),
        W(0.8, low, F_LET, "let go: it drops into the cut", g1),
        W(0.6, seat, F_LET, "fingers down beside the stem", g1),
        W(0.6, seat, F_GRIP, "pinch to stand the stem upright", g1),
        W(0.6, seat, F_LET, "let go", g1),
        W(1.0, seat, F_LET, "turn the open fingers a quarter turn", g2),
        W(0.6, seat, F_GRIP, "pinch across the other way", g2),
        W(0.6, seat, F_LET, "let go", g2),
        W(1.0, seat, F_LET, "turn the open fingers back", g1),
        W(0.6, seat, F_GRIP, "pinch again", g1),
        W(0.6, seat, F_LET, "let go", g1),
        W(1.0, seat, F_LET, "a quarter turn round again", g2),
        W(0.6, seat, F_GRIP, "pinch again across", g2),
        W(0.6, seat, F_LET, "let go", g2),
        W(1.2, (x, y, rim + 0.070), F_LET, "up and away", g2),
    ]
    rest, ryaw = _rest_pose("left")
    left += [W(2.0, rest, F_LET, "back to the rest pose", ryaw), W(3.0, rest, F_LET, "wait", ryaw)]
    right = [W(sum(w.duration for w in left), _rest_pose("right")[0], FINGER_OPEN, "wait", _rest_pose("right")[1])]
    plans = {"left": left, "right": right}
    for arm, plan in plans.items():
        rest, ryaw = _rest_pose(arm)
        plan[:0] = [W(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw),
                    W(0.5, rest, FINGER_OPEN, "settle at the rest pose", ryaw)]
    return plans
