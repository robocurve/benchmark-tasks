"""A front-loading home video game console (grey, 256 mm wide) stands on a wooden shelf board with its front toward
the robot and its hinged lid shut. A game cartridge with a red label is loaded inside it. A second cartridge with a
blue label lies flat, label up, on one of two small wooden display risers in front of the console, its label end
toward the robot and overhanging the riser; the other riser is empty. Both arms start at their ready pose with the
grippers open.
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
SUCCESS_HOLD_S = 0.1  # the 1 s release-and-still hold is counted inside score()
INSTRUCTION = ("Swap the game in the console for the spare one: put the old game on the empty stand, load the spare "
               "and leave the console closed, ready to play.")
MM = 1e-3
EPS = 1e-6

GREY, LIGHT, DARK, BLACK = (0.62, 0.62, 0.64), (0.78, 0.78, 0.79), (0.22, 0.22, 0.24), (0.08, 0.08, 0.09)
RED, BLUE, WOOD = (0.80, 0.16, 0.12), (0.12, 0.30, 0.82), (0.66, 0.50, 0.32)

# -- console: Nintendo Entertainment System NES-001, 256 W x 203 D x 89 H mm ---------------------------------------
# Console frame: origin at the middle of the front edge on the bench, x into the console (away from the robot), y to
# the robot's left, z up. Inside sizes are estimates (see README).
DEPTH, HALF_W, HEIGHT = 203 * MM, 128 * MM, 89 * MM
SILL = 40 * MM  # top of the recessed black front strip and of the chamber floor
RECESS = 8 * MM  # the black strip sits 8 mm behind the lid's front face
CH_W = 71 * MM  # chamber half-width
ROOF_X = 68 * MM  # the lid covers the chamber's front 68 mm from above
REAR_X = 160 * MM
LID_HINGE = np.array([ROOF_X, 0.0, 87.5 * MM])
LID_OPEN = 1.92  # rad (110 degrees): the lid's open stop, past its balance point (about 103 degrees)
LID_HALF = 70.5 * MM
LEDGE = 8 * MM  # the lid's lower edge stands 8 mm proud of its face as a finger ledge

# Cartridge: NES Game Pak, 120 W x 133 L x 20 thick mm, about 100 g. Pak frame: origin at its centre, x along its
# length toward the connector end, y across, z up out of the label face.
PAK = np.array([133 * MM, 120 * MM, 20 * MM])
PAK_MASS = 0.100
PAK_FRONT = 6 * MM  # label end of a loaded pak, behind the console front face
PAK_Z0 = 46 * MM  # underside of a loaded pak with the tray down
PAK_C0 = np.array([PAK_FRONT + PAK[0] / 2, 0.0, PAK_Z0 + PAK[2] / 2])  # loaded pak centre, tray down

# Tray (the carrier that holds the pak on the 72-pin connector): turns about a pivot at the connector.
PIVOT = np.array([149 * MM, 0.0, 56 * MM])
RAISED = 0.10  # rad: popped up, front about 14 mm higher
PRESS_LO = -0.030  # rad: bottom of the push-push over-travel (front 4.3 mm below "down")
CATCH = -0.018  # rad: a press past this toggles the push-push catch (front 2.6 mm below "down")
TRAY_K, TRAY_REST = 2.5, 0.24  # N m/rad toward 0.24 rad: about 5.5 N at the label end to press it past the catch
STOP_X = PAK_FRONT + PAK[0] + 0.5 * MM  # connector stop face (tray frame, tray down)

X0, Y0 = 0.345, 0.0  # world position of the console's front-edge middle
RISER = 30 * MM  # the console stands on a 30 mm shelf board


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _box(lo, hi, color, **kw):
    lo, hi = np.minimum(lo, hi).astype(float), np.maximum(lo, hi).astype(float)
    return parts.box(tuple(hi - lo), tuple((hi + lo) / 2), color, **kw)


def _shell():
    m = MM
    s = [
        _box((RECESS, -HALF_W, 0), (DEPTH, HALF_W, SILL), DARK),  # lower body with the black front strip
        _box((0, -HALF_W, SILL), (DEPTH, -CH_W, HEIGHT), GREY),  # right of the chamber
        _box((0, CH_W, SILL), (DEPTH, HALF_W, HEIGHT), GREY),  # left of the chamber
        _box((ROOF_X, -CH_W, 86 * m), (DEPTH, CH_W, HEIGHT), GREY),  # top behind the lid
        _box((REAR_X, -CH_W, SILL), (DEPTH, CH_W, 86 * m), DARK),  # board and rear of the chamber
        _box((RECESS - 0.5 * m, 20 * m, 12 * m), (RECESS, 60 * m, 26 * m), LIGHT, collide=False),  # controller ports
        _box((RECESS - 0.5 * m, -100 * m, 16 * m), (RECESS, -62 * m, 26 * m), LIGHT, collide=False),  # power, reset
    ]
    return s


def _flare(sy):
    """A short guide wall at the tray's mouth, angled 23 degrees out toward the robot."""
    a = math.atan2(6.0, 14.0)
    q = parts._matrix_to_quat(parts._rot((0, 0, 1), -sy * a))
    return parts.box((15.2 * MM, 3 * MM, 14 * MM), (13 * MM, sy * 65.5 * MM, 53 * MM), DARK, quat=tuple(q))


def _tray_shapes():
    m = MM
    s = []
    for sy in (1, -1):
        s.append(_box((12 * m, sy * 54 * m, 42 * m), (STOP_X - 0.5 * m, sy * 63 * m, PAK_Z0), BLACK))  # rail floor
        s.append(_box((20 * m, sy * 61.5 * m, PAK_Z0), (STOP_X - 0.5 * m, sy * 64.5 * m, 60 * m), BLACK))  # rail wall
        s.append(_flare(sy))
        q = tuple(parts._matrix_to_quat(_rot_y(math.radians(-25.0))))
        s.append(parts.box((9 * m, 9 * m, 3 * m), (8.5 * m, sy * 58.5 * m, 43.2 * m), BLACK, quat=q))  # rail lead-in
    s.append(_box((STOP_X, -50 * m, PAK_Z0), (STOP_X + 13 * m, 50 * m, 62 * m), BLACK))  # 72-pin connector
    return s


def _lid_shapes():
    m = MM
    return [
        _box((0, -LID_HALF, 86 * m), (ROOF_X, LID_HALF, HEIGHT), GREY),  # top
        _box((0, -LID_HALF, 42.5 * m), (2.5 * m, LID_HALF, 86 * m), GREY),  # front face
        _box((-LEDGE, -LID_HALF, 42.5 * m), (0, LID_HALF, 45.5 * m), GREY),  # finger ledge along its lower edge
    ]


CONSOLE = parts.Part(
    "console", pos=(X0, Y0, TOP + RISER), shapes=_shell(), friction=(0.35, 0.3), contact_offset=1 * MM,
    joints=[
        parts.Hinge("tray", shapes=tuple(_tray_shapes()), pivot=tuple(PIVOT), axis=(0.0, 1.0, 0.0), lower=PRESS_LO,
                    upper=0.0, mass=0.060, spring=TRAY_K, damping=0.05, max_effort=5.0, target=TRAY_REST),
        parts.Hinge("lid", shapes=tuple(_lid_shapes()), pivot=tuple(LID_HINGE), axis=(0.0, 1.0, 0.0), lower=0.0,
                    upper=LID_OPEN, mass=0.048, damping=0.01),
    ])


def _pak(name, pos, label):
    m = MM
    hx, hy, hz = PAK / 2
    shapes = [
        parts.box(tuple(PAK), (0, 0, 0), GREY),
        parts.box((97 * m, 55 * m, 0.4 * m), (-8 * m, 0, hz + 0.2 * m), label, collide=False),  # label
        parts.box((10 * m, 112 * m, 0.4 * m), (-hx + 6 * m, 0, hz + 0.2 * m), DARK, collide=False),  # grip ridges
        parts.box((0.4 * m, 100 * m, 6 * m), (hx + 0.2 * m, 0, -hz + 4 * m), BLACK, collide=False),  # connector slot
    ]
    return parts.Part(name, pos=tuple(float(v) for v in pos), mass=PAK_MASS, shapes=shapes, friction=(0.35, 0.3),
                      contact_offset=1 * MM)


ORIGIN = np.array([X0, Y0, TOP + RISER])
OLD = _pak("old_game", ORIGIN + PAK_C0, RED)
# The shelf board the console stands on: 30 mm thick, flush with the console's recessed black front strip.
SHELF = parts.Part("shelf", pos=(X0, Y0, TOP), shapes=[_box((RECESS, -160 * MM, 0), (260 * MM, 160 * MM, RISER), WOOD)],
                   friction=(0.5, 0.45))

# Display risers ("stands" in the code): pine blocks 80 deep x 100 wide x 60 high; a pak lies on one with 50 mm of its
# label end overhanging, as a game on a shelf edge, so it can be taken by its 20 mm thickness.
STAND = np.array([80 * MM, 100 * MM, 60 * MM])
STAND_Y = 0.150
PARK_FRONT = 0.205  # world x of a parked pak's label end
STAND_X = PARK_FRONT + 50 * MM + STAND[0] / 2


def stand_part(name, sy):
    return parts.Part(name, pos=(STAND_X, sy * STAND_Y, TOP), shapes=[parts.box(tuple(STAND), (0, 0, STAND[2] / 2), WOOD)],
                      friction=(0.5, 0.45))


STANDS = {"left_stand": stand_part("left_stand", 1), "right_stand": stand_part("right_stand", -1)}


def parked_pos(sy):
    return (PARK_FRONT + PAK[0] / 2, sy * STAND_Y, TOP + STAND[2] + PAK[2] / 2)


SPARE = _pak("spare_game", parked_pos(1), BLUE)
OBJECTS = [CONSOLE, OLD, SPARE, SHELF, *STANDS.values()]
SCORED = (OLD.name, SPARE.name)
DEMO_TOUCHES = {"console"}

LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={SPARE.name: parked_pos(1)}, spare_side=1),  # a zero move forces the settle and fresh frame
    lots.Lot("B", place={SPARE.name: parked_pos(-1)}, spare_side=-1),
)

# -- geometry of a state ------------------------------------------------------------------------------------------


def _R(q):
    return parts._quat_to_matrix(np.asarray(q, float))


def _finite(st):
    try:
        v = np.r_[np.asarray(st.pos, float), np.asarray(st.quat_xyzw, float), np.asarray(st.lin_vel, float),
                  np.asarray(st.ang_vel, float)]
    except (TypeError, ValueError, AttributeError):
        return False
    return v.shape == (13,) and bool(np.isfinite(v).all()) and np.linalg.norm(st.quat_xyzw) > 1e-6


def console_frame(console):
    return np.asarray(console.pos, float), _R(console.quat_xyzw)


def tray_frame(console):
    """(origin, rotation) of the tray in world: maps tray-frame points (console frame with the tray down) to world."""
    p, r = console_frame(console)
    q = float(console.joints["tray"])
    rt = _rot_y(q)
    return p + r @ (PIVOT - rt @ PIVOT), r @ rt


def pak_in_tray(console, pak):
    """The pak's centre relative to its loaded home in the tray frame (m) and its turn from the loaded pose (deg)."""
    o, r = tray_frame(console)
    rel = r.T @ (np.asarray(pak.pos, float) - o) - PAK_C0
    turn = r.T @ _R(pak.quat_xyzw)
    ang = math.degrees(math.acos(float(np.clip((np.trace(turn) - 1) / 2, -1, 1))))
    return rel, ang


SEAT_ALONG, SEAT_ACROSS, SEAT_UP, SEAT_ANG = 3 * MM, 3 * MM, 2 * MM, 5.0


def seated(console, pak):
    rel, ang = pak_in_tray(console, pak)
    return bool(-SEAT_ALONG - EPS <= rel[0] <= 0.5 * MM + EPS and abs(rel[1]) <= SEAT_ACROSS + EPS
                and abs(rel[2]) <= SEAT_UP + EPS and ang <= SEAT_ANG + EPS)


DOWN_TOL = 0.008  # rad: front within about 1.1 mm of the down reference
SHUT_TOL = math.radians(2.0)  # lid's front edge within about 3 mm of shut


def locked_down(console, rec):
    return rec["latch"] == "locked" and abs(float(console.joints["tray"])) <= DOWN_TOL + EPS


def lid_shut(console):
    return float(console.joints["lid"]) <= SHUT_TOL + EPS


def pry_deg(console, pak):
    """How far the pak's length is tipped out of the tray's plane (degrees): prising it up or down at one end."""
    _, r = tray_frame(console)
    x = _R(pak.quat_xyzw)[:, 0]
    return math.degrees(math.asin(float(np.clip(abs(x @ r[:, 2]), 0.0, 1.0))))


def corners(pak):
    h = PAK / 2
    c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    return c @ _R(pak.quat_xyzw).T + np.asarray(pak.pos, float)


def in_chamber(console, pak):
    """True while any corner of the pak is inside the console's cartridge chamber."""
    p, r = console_frame(console)
    c = (corners(pak) - p) @ r
    return bool(np.any((c[:, 0] > 0.0) & (c[:, 0] < REAR_X) & (np.abs(c[:, 1]) < CH_W) & (c[:, 2] > SILL - 5 * MM)
                       & (c[:, 2] < HEIGHT + 5 * MM)))


def on_stand(pak, stand):
    """Lying flat (within 10 degrees) on the stand's top, centre over the top face, underside within 3 mm of it."""
    r = _R(pak.quat_xyzw)
    tilt_d = math.degrees(math.acos(float(np.clip(abs(r[2, 2]), -1, 1))))
    sp = np.asarray(stand.pos, float)
    c = np.asarray(pak.pos, float)
    bottom = float(corners(pak)[:, 2].min())
    return bool(tilt_d <= 10.0 + EPS and abs(c[0] - sp[0]) <= STAND[0] / 2 + EPS and abs(c[1] - sp[1]) <= STAND[1] / 2 + EPS
                and abs(bottom - (TOP + STAND[2])) <= 3 * MM + EPS)


def parked(objects):
    return any(on_stand(objects[OLD.name], objects[n]) for n in STANDS)


def tier(objects, rec):
    """The value of this state on the ladder, before the release, stillness and whole-trial gates."""
    console = objects[CONSOLE.name]
    if not parked(objects):
        return 0.0
    if not seated(console, objects[SPARE.name]):
        return 0.25
    if not locked_down(console, rec):
        return 0.5
    if not lid_shut(console):
        return 0.75
    return 1.0


# -- hands clear ---------------------------------------------------------------------------------------------------
CLEAR = 8 * MM  # sampled hand surface to a scored part: 5 mm clear plus 3 mm for the 3 mm sampling


def part_boxes(objects):
    """(centre, rotation, size) boxes over both paks and the console's moving lid and tray."""
    out = []
    for n in SCORED:
        b = objects[n]
        out.append((np.asarray(b.pos, float), _R(b.quat_xyzw), PAK.copy()))
    console = objects[CONSOLE.name]
    links = {k: (np.asarray(v[0], float), _R(v[1])) for k, v in console.links.items() if k in ("lid", "tray")}
    out += [(c, r, s) for _, c, r, s in CONSOLE.world_boxes(links)]
    return out


def hands_clear(clouds, objects):
    if not clouds:
        return False
    pts = []
    for p in clouds:
        p = np.asarray(p, float)
        if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
            return False
        pts.append(p)
    pts = np.concatenate(pts)
    for c, r, s in part_boxes(objects):
        lo = c - (np.abs(r) @ (s / 2)) - CLEAR - 0.01
        hi = c + (np.abs(r) @ (s / 2)) + CLEAR + 0.01
        near = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
        if not len(near):
            continue
        local = np.abs((near - c) @ r) - s / 2
        if np.linalg.norm(np.maximum(local, 0.0), axis=1).min() <= CLEAR + EPS:
            return False
    return True


# -- the trial record ---------------------------------------------------------------------------------------------
# Kept per trial and cleared only by the simulator's reset callback (never by object positions).
STILL_V, DRIFT_POS, DRIFT_DEG, HOLD_S = 3 * MM, 0.5 * MM, 1.0, 1.0
PRY_MM, PRY_DEG = 5 * MM, 15.0
ARRIVE_FROM = 10 * MM  # the spare "arrives" at the connector when it slides home from more than 10 mm short
DOWN_ZONE = 0.05  # rad: the carrier counts as down (caught or held down) below this


def moved_in_tray(console, pak):
    """Largest displacement (m) of any corner of the pak from its loaded home, in the carrier's frame."""
    o, r = tray_frame(console)
    h = PAK / 2
    c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    now = (c @ _R(pak.quat_xyzw).T + np.asarray(pak.pos, float) - o) @ r
    return float(np.linalg.norm(now - (c + PAK_C0), axis=1).max())


def _fresh():
    return dict(known=False, invalid=False, zero=None, latch="locked", armed=True, released=False,
                spare_away=True, held=0.0, t=None, value=None, ref=None, stamp=None, clear=False, still=False)


REC = _fresh()


def stamp(objects):
    out = []
    for n in SCORED:
        st = objects[n]
        out += [*np.round(np.asarray(st.pos, float), 9), *np.round(np.asarray(st.quat_xyzw, float), 9)]
    c = objects[CONSOLE.name]
    out += [round(float(c.joints["tray"]), 9), round(float(c.joints["lid"]), 9)]
    return tuple(float(v) for v in out)


def poses(objects):
    c = objects[CONSOLE.name]
    return (np.array([np.asarray(objects[n].pos, float) for n in SCORED]), [_R(objects[n].quat_xyzw) for n in SCORED],
            np.array([float(c.joints["tray"]), float(c.joints["lid"])]))


def drifted(ref, now):
    if ref is None:
        return True
    if np.linalg.norm(now[0] - ref[0], axis=1).max() > DRIFT_POS + EPS:
        return True
    for a, b in zip(ref[1], now[1]):
        if math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))) > DRIFT_DEG + EPS:
            return True
    return bool(np.degrees(np.abs(now[2] - ref[2])).max() > DRIFT_DEG + EPS)


def slow(objects):
    return all(np.linalg.norm(objects[n].lin_vel) <= STILL_V + EPS for n in SCORED)


def latch_update(rec, q):
    """The push-push catch (a declared mechanism): pressed past CATCH while caught, it lets the tray rise; pressed
    past CATCH again after rising, it catches the tray down."""
    if rec["latch"] == "locked":
        if rec["armed"] and q <= CATCH + EPS:
            rec.update(latch="free", armed=False, released=True)
        elif not rec["armed"] and q >= -0.005:
            rec["armed"] = True
    else:
        if not rec["armed"] and q >= RAISED - 0.02:
            rec["armed"] = True
        elif rec["armed"] and q <= CATCH + EPS:
            rec.update(latch="locked", armed=False)


def observe(objects, clouds, t):
    """Update the trial record from one control step's states and hand surface samples (None: unknown)."""
    try:
        ok = all(_finite(objects[n]) for n in (CONSOLE.name, *SCORED))
        c = objects[CONSOLE.name]
        q, lid = float(c.joints["tray"]), float(c.joints["lid"])
        ok = ok and math.isfinite(q) and math.isfinite(lid)
    except (KeyError, TypeError, ValueError, AttributeError):
        ok = False
    dt = None if REC["t"] is None else float(t) - REC["t"]
    if not ok or clouds is None or (dt is not None and not 0.0 < dt <= 0.2):
        REC.update(known=False, invalid=True, held=0.0, t=float(t), stamp=None)
        return
    latch_update(REC, q)
    old, spare = objects[OLD.name], objects[SPARE.name]
    # Whole-trial rules.
    if not REC["released"] and moved_in_tray(c, old) > PRY_MM + EPS:
        REC["zero"] = REC["zero"] or "old game moved before the catch was released"
    for pak in (old, spare):
        if in_chamber(c, pak) and pry_deg(c, pak) > PRY_DEG + EPS:
            REC["zero"] = REC["zero"] or f"{pak is old and 'old' or 'spare'} game tilted against the tray in the chamber"
    rel_sp, _ = pak_in_tray(c, spare)
    if rel_sp[0] < -ARRIVE_FROM:
        REC["spare_away"] = True
    home = (rel_sp[0] >= -SEAT_ALONG - EPS and abs(rel_sp[1]) <= SEAT_ACROSS + EPS and abs(rel_sp[2]) <= 10 * MM)
    if home and REC["spare_away"]:
        REC["spare_away"] = False
        if q < DOWN_ZONE:
            REC["zero"] = REC["zero"] or "spare slid onto the connector with the carrier down"
    value = tier(objects, REC)
    clear = hands_clear(clouds, objects)
    now = poses(objects)
    calm = slow(objects)
    if clear and calm and value == REC["value"] and dt is not None and not drifted(REC["ref"], now):
        REC["held"] += dt
    else:
        REC["held"], REC["ref"] = 0.0, now
    REC.update(known=True, t=float(t), value=value, stamp=stamp(objects), clear=clear, still=calm)


def score(objects):
    """Ladder value of this state: 0.25 old game on a stand, 0.5 spare fully in, 0.75 spare caught down, 1.0 lid shut,
    only with every hand clear and everything still for the last 1 s, and 0 for good after any whole-trial fault."""
    try:
        if not REC["known"] or REC["invalid"] or REC["zero"] or REC["stamp"] != stamp(objects):
            return 0.0
        if not (REC["clear"] and REC["still"] and REC["held"] >= HOLD_S - EPS):
            return 0.0
        return float(tier(objects, REC))
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# -- simulator hooks ----------------------------------------------------------------------------------------------


@lru_cache(maxsize=1)
def hand_samples():
    """Points on every collision face of each gripper link (link frame), at most 3 mm apart."""
    from yam_arena import analysis

    out = {}
    for name in ("gripper", "tip_left", "tip_right"):
        cloud = []
        for h in analysis.link_hulls().get(name, []):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(0)
                u = face[0] - c
                u /= np.linalg.norm(u)
                v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / 0.003))
                    for i in range(n + 1):
                        for j in range(n - i + 1):
                            cloud.append(c + (a - c) * (i / n) + (b - c) * (j / n))
        if not cloud:
            raise ValueError("missing hand geometry")
        pts = np.asarray(cloud)
        _, keep = np.unique(np.floor(pts / 0.0025).astype(np.int64), axis=0, return_index=True)
        out[name] = pts[np.sort(keep)]
    return out


def hand_clouds(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    clouds = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses_ = parts._np(asset.data.body_link_pose_w)[0]
        if poses_.ndim != 2 or poses_.shape[1] != 7 or not np.isfinite(poses_).all():
            raise ValueError("unknown robot body poses")
        for name, pts in hand_samples().items():
            pose = poses_[asset.body_names.index(name)]
            clouds.append(pts @ _R(pose[3:]).T + pose[:3] - origin)
    return clouds


_LIMIT = {"upper": None}


def _tray_upper(sim, upper):
    import torch

    if _LIMIT["upper"] == upper:
        return
    _LIMIT["upper"] = upper
    base = sim.env.unwrapped
    asset = base.scene[CONSOLE.name]
    ids = asset.find_joints(["tray"], preserve_order=True)[0]
    lim = torch.tensor([[[PRESS_LO, upper]]], dtype=torch.float32, device=base.device)
    asset.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


@parts.on_reset
def _reset(sim):
    REC.clear()
    REC.update(_fresh())
    _LIMIT["upper"] = None
    _tray_upper(sim, 0.0)
    sim.set_target(CONSOLE.name, "tray", TRAY_REST)


@parts.on_step
def _step(sim):
    objects = sim.states()
    try:
        clouds = hand_clouds(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        clouds = None
    observe(objects, clouds, float(sim.t))
    _tray_upper(sim, 0.0 if REC["latch"] == "locked" else RAISED)
    sim.set_target(CONSOLE.name, "tray", TRAY_REST)  # the carrier spring: always pushing the tray up
    if os.environ.get("TRACE_OUT"):
        c = objects[CONSOLE.name]
        row = dict(t=round(float(sim.t), 3), lot=lots.current().name, tray=round(float(c.joints["tray"]), 4),
                   lid=round(float(c.joints["lid"]), 4), latch=REC["latch"], armed=REC["armed"],
                   released=REC["released"], zero=REC["zero"], value=REC["value"], held=round(REC["held"], 2),
                   clear=REC["clear"], still=REC["still"], score=score(objects))
        for n in SCORED:
            rel, ang = pak_in_tray(c, objects[n])
            row[n] = dict(rel_mm=np.round(rel * 1000, 1).tolist(), ang=round(ang, 1),
                          pos=np.round(objects[n].pos, 4).tolist(), seated=seated(c, objects[n]),
                          v=round(float(np.linalg.norm(objects[n].lin_vel)), 4))
        row["parked"] = parked(objects)
        try:
            from yam_arena.constants import TCP_OFFSET

            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            tcp = {}
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
                tcp[arm] = np.round(pose[:3] - origin + _R(pose[3:]) @ np.array(TCP_OFFSET), 4).tolist()
            row["tcp"] = tcp
        except (ValueError, AttributeError, KeyError, IndexError):
            pass
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("console.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# -- scripted demo: open loop from the published layout of this trial ---------------------------------------------
# Horizontal grip: fingertips point into the console (+x) and close top-to-bottom on the pak's 20 mm thickness.
# The left arm does this on the robot's right of the midline and the right arm on its left (where each reaches).
GRIP_YAW = {"left": math.pi / 2, "right": -math.pi / 2}
GRIP_SIDE = {"left": -1, "right": 1}  # side of the midline each arm takes a pak on
GRIP_Y = 0.030  # grip line 30 mm off the pak's centre line, toward that arm's working side
GRIP_IN = 34 * MM  # TCP this far behind the label end (fingertips 47 mm in)
PINCH = -0.0075  # finger target: 15 mm between pads on the 20 mm pak
OPEN_PAK = -0.016  # 32 mm between pads around the pak (6 mm each side; the chamber floor is 20 mm below it)
LIFT = 3 * MM  # pak held this far above the rail line while it slides in
CARRY_DZ = 5.5 * MM  # a held pak hangs about 5.5 mm below the commanded grip (measured on the GPU); aim that much higher
PRESS_DEEP = 35 * MM  # presses aim this far below the pak top at the bottom of the over-travel (the arm is soft in z)
COMP = {"left": np.zeros(3), "right": np.zeros(3)}  # carry offsets, measured on the GPU
def _ready():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    out = {a: ik.tcp_pose(np.array(READY, float), np.array(station.BASES[a], float)) for a in ("left", "right")}
    return {a: np.asarray(p, float) for a, (p, _) in out.items()}, np.asarray(out["left"][1], float)


PARK, R_PARK = _ready()  # each arm parks at its own ready pose (hand high over its base line)
TRAVEL_Z = TOP + 0.095


def _W(t, p, f, label, rot=None):
    if rot is None:
        return tilt.Waypoint(float(t), tuple(float(v) for v in p), float(f), label)
    w = tilt.Waypoint(float(t), tuple(float(v) for v in p), float(f), label)
    w.yaw = tilt._TiltedYaw(0.0, rot)
    return w


def r_grip(arm, pitch=0.0):
    """Horizontal grip, fingertips pitched down by ``pitch`` rad (to match the raised tray)."""
    return tilt.rotation(GRIP_YAW[arm], 90.0 - math.degrees(pitch), 0.0)


def r_lean(deg):
    return tilt.rotation(0.0, deg, 0.0)


def world(p):
    return ORIGIN + np.asarray(p, float)


def tray_point(p, q):
    """Console-frame position of tray-frame point p with the tray at angle q."""
    return PIVOT + _rot_y(q) @ (np.asarray(p, float) - PIVOT)


def pak_grip_tcp(arm, q, slide=0.0, lift=0.0):
    """TCP for a pak in the tray at angle q, drawn ``slide`` m out along the tray and ``lift`` m above its rails."""
    local = np.array([PAK_FRONT + GRIP_IN - slide, GRIP_SIDE[arm] * GRIP_Y, PAK_Z0 + PAK[2] / 2 + lift])
    return world(tray_point(local, q)) + COMP[arm]


def stand_grip_tcp(arm, sy, lift=0.0):
    c = np.asarray(parked_pos(sy))
    return np.array([PARK_FRONT + GRIP_IN, c[1] + GRIP_SIDE[arm] * GRIP_Y, c[2] + lift]) + COMP[arm]


class Plan:
    """Builds both arms' waypoint lists; while one arm acts the other holds its last pose."""

    def __init__(self):
        self.w = {"left": [], "right": []}
        self.last = {"left": (PARK["left"], FINGER_OPEN, R_PARK), "right": (PARK["right"], FINGER_OPEN, R_PARK)}

    def go(self, arm, t, p, f, label, rot=None):
        self.w[arm].append(_W(t, p, f, label, rot))
        self.last[arm] = (np.asarray(p, float), f, rot)
        other = "right" if arm == "left" else "left"
        p0, f0, r0 = self.last[other]
        self.w[other].append(_W(t, p0, f0, "wait", r0))

    def both(self, t, label):
        for arm in ("left", "right"):
            p0, f0, r0 = self.last[arm]
            self.w[arm].append(_W(t, p0, f0, label, r0))


def open_lid(P, arm):
    """Lift the lid by its finger ledge with the top of the closed fingertips, the fingers pointing into the console
    (closed top to bottom, so their upper face is nearly flat), following the ledge's arc past the balance point;
    the lid then falls back onto its open stop."""
    rot = r_grip(arm)
    corner = np.array([0.012, 0.0, 0.004])  # upper front of the closed fingertips, from the TCP
    y = GRIP_SIDE[arm] * 0.040
    ledge = np.array([-3.0 * MM, 0.0, 42.5 * MM]) - LID_HINGE  # the ledge's underside, 3 mm in front of the lid face

    def tcp(deg, into=1.5 * MM):
        r = _rot_y(math.radians(deg))
        e = LID_HINGE + r @ ledge
        n = r @ np.array([0.0, 0.0, -1.0])  # the ledge underside's outward normal
        return world(e - n * into + np.array([0.0, y, 0.0])) - corner

    below = tcp(0.0, into=-3.0 * MM)
    P.go(arm, 2.5, below + np.array([-0.04, 0.0, 0.0]), 0.0, "in front of the lid ledge, below it", rot)
    P.go(arm, 1.5, below, 0.0, "under the ledge", rot)
    P.go(arm, 1.0, tcp(0.0), 0.0, "up against the ledge", rot)
    for deg in (4, 10, 20, 32, 45, 58, 70, 82, 93, 101, 107):
        P.go(arm, 0.9, tcp(deg), 0.0, f"lift the lid to {deg} degrees", rot)
    end = tcp(107)
    P.go(arm, 1.2, end + np.array([-0.035, 0.0, -0.01]), 0.0, "back away from the open lid", rot)
    P.go(arm, 2.0, PARK[arm], FINGER_OPEN, "park", R_PARK)


PRESS_TCP_IN = 20 * MM  # press: TCP this far behind the label end, closed fingertips flat on the pak's top
PRESS_UNDER = 9.0 * MM  # underside of the closed fingers below the TCP where they cross the pak's label end edge


def press_points(arm, q):
    """(TCP over the label end with the fingers just touching, with the tray at angle q) for a press."""
    edge = tray_point((PAK_FRONT, GRIP_SIDE[arm] * GRIP_Y, PAK_Z0 + PAK[2]), q)
    return world((PAK_FRONT + PRESS_TCP_IN, edge[1], edge[2] + PRESS_UNDER))


def press(P, arm, label, from_q, release):
    """Press the loaded pak's label end down with the underside of the closed fingers (fingers pointing into the
    console), past the catch, then rise slowly so the tray can come up under them if the catch let go."""
    rot = r_grip(arm)
    start = press_points(arm, from_q)
    bottom = press_points(arm, PRESS_LO) - np.array([0.0, 0.0, PRESS_DEEP])
    up = press_points(arm, RAISED) + np.array([0.0, 0.0, 0.010])
    P.go(arm, 1.5, start + np.array([0.0, 0.0, 0.015]), 0.0, f"fingers over the label end for the {label}", rot)
    P.go(arm, 2.0, bottom, 0.0, f"press for the {label}", rot)
    P.go(arm, 1.5, bottom, 0.0, "hold", rot)
    P.go(arm, 2.5, up, 0.0, "rise and let the tray come up" if release else "rise off the caught tray", rot)


def held(p):
    """Command for a TCP that carries a pak: aim CARRY_DZ higher so the pak itself ends where planned."""
    return np.asarray(p, float) + np.array([0.0, 0.0, CARRY_DZ])


def take_old(P, arm, park_side):
    """Grip the popped-up old pak by its label end, draw it straight out along the tray and lay it on a stand."""
    rot_t, rot_h = r_grip(arm, RAISED), r_grip(arm)
    P.go(arm, 3.0, press_points(arm, 0.0) + np.array([-0.050, 0.0, 0.015]), 0.0, "in front of the console", rot_h)
    press(P, arm, "release", 0.0, True)
    g = pak_grip_tcp(arm, RAISED)
    back = pak_grip_tcp(arm, RAISED, slide=0.055)
    P.go(arm, 1.5, np.array([back[0], back[1], press_points(arm, RAISED)[2] + 0.010]), 0.0, "back out over the label end", rot_t)
    P.go(arm, 1.5, back, OPEN_PAK, "line up on the old pak's label end", rot_t)
    P.go(arm, 2.0, g, OPEN_PAK, "fingers over and under the label end", rot_t)
    P.go(arm, 1.0, g, PINCH, "pinch the old pak", rot_t)
    P.go(arm, 0.5, g, PINCH, "settle", rot_t)
    out = pak_grip_tcp(arm, RAISED, slide=0.150)
    P.go(arm, 4.0, out, PINCH, "draw it straight out along the tray", rot_t)
    clear = held(out + np.array([-0.030, 0.0, 0.012]))
    P.go(arm, 1.5, clear, PINCH, "clear of the console, level it", rot_h)
    s = stand_grip_tcp(arm, park_side)
    P.go(arm, 3.0, (s[0], s[1], clear[2]), PINCH, "carry over the stand", rot_h)
    P.go(arm, 1.5, held(s + np.array([0, 0, 0.004])), PINCH, "lower onto the stand", rot_h)
    P.go(arm, 1.0, held(s + np.array([0, 0, 0.004])), OPEN_PAK, "let go", rot_h)
    P.go(arm, 1.5, s + np.array([-0.052, 0, 0.004]), OPEN_PAK, "back out", rot_h)
    P.go(arm, 1.5, s + np.array([-0.052, 0, 0.060]), OPEN_PAK, "rise", rot_h)
    P.go(arm, 1.5, np.array([s[0] - 0.040, 0.5 * s[1], TOP + 0.13]), OPEN_PAK, "toward the middle", rot_h)
    P.go(arm, 2.0, PARK[arm], FINGER_OPEN, "park", R_PARK)


def load_spare(P, arm, spare_side):
    """Pick the spare off its stand, slide it into the raised tray to the connector and let go."""
    rot_t, rot_h = r_grip(arm, RAISED), r_grip(arm)
    s = stand_grip_tcp(arm, spare_side)
    P.go(arm, 1.5, np.array([s[0] - 0.040, 0.5 * s[1], TOP + 0.13]), OPEN_PAK, "toward the spare", rot_h)
    P.go(arm, 1.5, s + np.array([-0.052, 0, 0.060]), OPEN_PAK, "above the spare's label end", rot_h)
    P.go(arm, 1.5, s + np.array([-0.052, 0, 0.0]), OPEN_PAK, "line up on the spare's label end", rot_h)
    P.go(arm, 2.0, s, OPEN_PAK, "fingers over and under it", rot_h)
    P.go(arm, 1.0, s, PINCH, "pinch the spare", rot_h)
    P.go(arm, 0.5, s, PINCH, "settle", rot_h)
    P.go(arm, 1.5, held(s + np.array([-0.030, 0, 0.015])), PINCH, "lift it off", rot_h)
    entry = held(pak_grip_tcp(arm, RAISED, slide=0.150, lift=LIFT))
    P.go(arm, 3.0, entry + np.array([-0.030, 0, 0.012]), PINCH, "carry in front of the console", rot_h)
    P.go(arm, 1.5, entry, PINCH, "line up with the tray", rot_t)
    home = held(pak_grip_tcp(arm, RAISED, slide=-0.002, lift=LIFT))
    P.go(arm, 4.5, home, PINCH, "slide it in to the connector", rot_t)
    P.go(arm, 0.5, home, PINCH, "settle", rot_t)
    P.go(arm, 1.0, home, OPEN_PAK, "let go", rot_t)
    out = pak_grip_tcp(arm, RAISED, slide=0.070, lift=0.0)
    P.go(arm, 1.5, out, OPEN_PAK, "draw the fingers out", rot_t)
    P.go(arm, 1.0, out, 0.0, "close the fingers", rot_t)
    press(P, arm, "catch", RAISED, False)
    P.go(arm, 1.5, press_points(arm, 0.0) + np.array([-0.060, 0.0, 0.020]), 0.0, "back away", rot_h)
    P.go(arm, 2.0, PARK[arm], FINGER_OPEN, "park", R_PARK)


def close_lid(P, arm):
    """Press the open lid's finger ledge (uppermost and in front of the hinge when the lid is open) down from above,
    past the balance point, then lift away; the lid swings shut under its own weight."""
    y = -GRIP_SIDE[arm] * 0.040
    rot = r_lean(30.0)
    low_corner = rot @ np.array([0.0, 0.0, 0.013]) - 0.0077 * (rot @ np.array([1.0, 0.0, 0.0]))  # lowest fingertip edge
    pt = np.array([-LEDGE, 0.0, 44 * MM]) - LID_HINGE  # the finger ledge's front face

    def tcp(deg, into=2.0 * MM):
        r = _rot_y(math.radians(deg))
        p = LID_HINGE + r @ pt
        n = r @ np.array([-1.0, 0.0, 0.0])  # the ledge face's outward normal (up when the lid is open)
        return world(p - n * into + np.array([0.0, y, 0.0])) - low_corner

    open_deg = math.degrees(LID_OPEN)
    P.go(arm, 2.5, tcp(open_deg, into=-0.025), 0.0, "over the open lid's ledge", rot)
    P.go(arm, 1.5, tcp(open_deg, into=-0.004), 0.0, "down onto the ledge", rot)
    for a in (open_deg, 104, 98, 92, 86):
        P.go(arm, 1.0, tcp(a), 0.0, f"press the lid to {a:.0f} degrees", rot)
    P.go(arm, 1.5, tcp(86) + np.array([-0.02, 0.0, 0.04]), 0.0, "lift away", rot)
    P.go(arm, 2.0, PARK[arm], FINGER_OPEN, "park", R_PARK)


def demo():
    lot = lots.current()
    sp = lot.spare_side  # the stand the spare is on: +1 the robot's left, -1 its right (visible in the cameras)
    old_arm = "left" if sp > 0 else "right"  # the arm whose working side holds the empty stand
    new_arm = "right" if sp > 0 else "left"
    P = Plan()
    P.both(0.5, "start")
    open_lid(P, new_arm)
    take_old(P, old_arm, -sp)  # starts with the release press
    load_spare(P, new_arm, sp)  # ends with the catch press
    close_lid(P, old_arm)
    P.both(4.0, "hands clear, hold")
    return P.w
