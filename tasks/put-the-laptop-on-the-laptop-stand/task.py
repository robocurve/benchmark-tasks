"""A desk (the station table). On the left, an open 13 inch laptop lies propped on three stacked hardcover books, its
screen showing a static illustrative upload image. On the right stands an empty wooden laptop stand. Both supports are narrower
than the laptop, so its base overhangs them at the sides. Where the laptop lies on the books changes between
trials. Nobody is in the scene.
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
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
D = math.radians

CHASSIS = (0.16, 0.16, 0.17)  # ThinkPad black
KEYS = (0.07, 0.07, 0.08)
PAD = (0.22, 0.22, 0.23)
SCREEN = (0.88, 0.90, 0.93)
BEZEL = (0.05, 0.05, 0.06)
BAR_BG = (0.70, 0.72, 0.76)
BAR = (0.10, 0.45, 0.85)
LOGO = (0.85, 0.10, 0.10)
BOOK1 = (0.20, 0.30, 0.55)
BOOK2 = (0.55, 0.20, 0.18)
BOOK3 = (0.22, 0.42, 0.28)
PAGES = (0.93, 0.91, 0.84)
BEECH = (0.80, 0.66, 0.46)


def _qy(a: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _qz(a: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _roty(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# -- the laptop: Lenovo ThinkPad X1 Nano Gen 3 (13 in) ---------------------------------------------------------------
# Lenovo PSREF: 293.2 x 208.0 x 14.4 mm (non-touch); "starting at" about 0.97 kg (reviews weigh 0.96 to 1.0 kg). Part
# frame: origin at the middle of the base's underside, z up, +x from the keyboard's front edge toward the hinge,
# +y to the user's left. Modelled as a base 10.0 mm thick (0.60 kg) and a lid 4.4 mm thick (0.37 kg) on a hinge along
# the base's back top edge (the split of thickness and mass is an estimate). The lid is drawn open 110 degrees from
# closed (leaning 20 degrees back past upright); the hinge holds it there with up to 1.5 N m, a stand-in for the
# friction hinge (estimate: 13 in laptop hinges hold roughly 0.5 to 1.5 N m).
LAP_W, LAP_D = 0.2932, 0.2080  # width (y), depth (x)
BASE_T, LID_T = 0.0100, 0.0044
BASE_MASS, LID_MASS = 0.60, 0.37
OPEN = D(110.0)
PIVOT = (LAP_D / 2, 0.0, BASE_T)
R_OPEN = _roty(OPEN)
LID_SPRING, LID_HOLD = 20.0, 1.5  # N m/rad, N m
LID_LIMIT = D(25.0)  # the lid may not move more than this from where it starts (closed by 85 degrees = asleep)


def _lid_at(local) -> tuple[float, float, float]:
    """Part-frame position, with the lid open, of a point given relative to the hinge with the lid closed."""
    p = np.asarray(PIVOT, float) + R_OPEN @ np.asarray(local, float)
    return tuple(float(v) for v in p)


def _base_shapes():
    out = [parts.box((LAP_D, LAP_W, BASE_T), at=(0.0, 0.0, BASE_T / 2), color=CHASSIS)]
    z = BASE_T + 0.0003
    out += [
        parts.box((0.100, 0.270, 0.0005), at=(0.030, 0.0, z), color=KEYS, collide=False),  # keyboard
        parts.box((0.055, 0.100, 0.0005), at=(-0.065, 0.0, z), color=PAD, collide=False),  # touchpad
        parts.box((0.008, 0.008, 0.0005), at=(0.030, 0.0, z + 0.0002), color=LOGO, collide=False),  # TrackPoint
    ]
    return out


def _lid_shapes():
    q = _qy(OPEN)
    out = [parts.box((LAP_D, LAP_W, LID_T), at=_lid_at((-LAP_D / 2, 0.0, LID_T / 2)), color=CHASSIS, quat=q)]
    # looks only, on the screen face (the closed lid's underside): bezel, screen, upload window and progress bar
    zf = -0.0004
    out += [
        parts.box((LAP_D - 0.006, LAP_W - 0.006, 0.0005), at=_lid_at((-LAP_D / 2, 0.0, zf)), color=BEZEL, quat=q,
                  collide=False),
        parts.box((LAP_D - 0.022, LAP_W - 0.020, 0.0005), at=_lid_at((-LAP_D / 2 - 0.002, 0.0, zf - 0.0004)),
                  color=SCREEN, quat=q, collide=False),
        parts.box((0.070, 0.170, 0.0005), at=_lid_at((-0.110, 0.0, zf - 0.0008)), color=(0.97, 0.97, 0.98), quat=q,
                  collide=False),
        parts.box((0.012, 0.140, 0.0005), at=_lid_at((-0.118, 0.0, zf - 0.0012)), color=BAR_BG, quat=q,
                  collide=False),
        parts.box((0.012, 0.088, 0.0005), at=_lid_at((-0.118, 0.026, zf - 0.0016)), color=BAR, quat=q,
                  collide=False),
        parts.box((0.008, 0.120, 0.0005), at=_lid_at((-0.092, 0.0, zf - 0.0012)), color=(0.25, 0.25, 0.30), quat=q,
                  collide=False),
    ]
    return out


# -- the books: three hardcover books, 235 x 155 x 35 mm each, stacked flat (the laptop's makeshift riser) -----------
# They lie lengthwise front to back; the laptop's front edge overhangs their front edge by 55 mm, as the stand's does.
BOOK_L, BOOK_W, BOOK_T = 0.235, 0.155, 0.035
BOOKS_TOP_H = 3 * BOOK_T  # 105 mm
OVERHANG = 0.055  # the base's front edge in front of its support's front edge
LAP_A = (0.41, 0.120)  # the laptop's middle in lot A
BOOKS_FRONT = LAP_A[0] - LAP_D / 2 + OVERHANG  # 0.361
BOOKS_C = (BOOKS_FRONT + BOOK_L / 2, 0.120)


def _books_shapes():
    out = []
    for k, col in enumerate((BOOK1, BOOK2, BOOK3)):
        z0 = k * BOOK_T
        out.append(parts.box((BOOK_L, BOOK_W, BOOK_T), at=(0.0, 0.0, z0 + BOOK_T / 2), color=col))
        # page edges on the side facing the robot
        out.append(parts.box((0.0006, BOOK_W - 0.012, BOOK_T - 0.006), at=(-BOOK_L / 2 - 0.0003, 0.002,
                                                                          z0 + BOOK_T / 2), color=PAGES,
                             collide=False))
    return out


# -- the laptop stand: a plain beech riser, top 150 mm deep and 160 mm wide, 120 mm tall (drawing in the README) ---
STAND_D, STAND_W, STAND_H = 0.150, 0.160, 0.120
STAND_FRONT = BOOKS_FRONT
STAND_C = (STAND_FRONT + STAND_D / 2, -0.140)
STAND_TOP = TOP + STAND_H
TARGET = (STAND_FRONT - OVERHANG + LAP_D / 2, STAND_C[1])  # where the laptop's middle ends up in the demo


def _stand_shapes():
    t = 0.018  # top board and two side boards
    return [
        parts.box((STAND_D, STAND_W, t), at=(0.0, 0.0, STAND_H - t / 2), color=BEECH),
        parts.box((STAND_D, t, STAND_H - t), at=(0.0, STAND_W / 2 - t / 2, (STAND_H - t) / 2), color=BEECH),
        parts.box((STAND_D, t, STAND_H - t), at=(0.0, -STAND_W / 2 + t / 2, (STAND_H - t) / 2), color=BEECH),
    ]


# -- layout -------------------------------------------------------------------------------------------------------------
LAYOUT = {
    "A": dict(at=LAP_A, yaw=0.0),
    "B": dict(at=(0.415, 0.108), yaw=D(6.0)),
}
BOOKS_TOP = TOP + BOOKS_TOP_H

LID = parts.Hinge("lid", shapes=tuple(_lid_shapes()), pivot=PIVOT, axis=(0.0, 1.0, 0.0), lower=-OPEN,
                  upper=D(65.0), mass=LID_MASS, spring=LID_SPRING, damping=0.05, max_effort=LID_HOLD)
LAPTOP = parts.Part("laptop", pos=(*LAYOUT["A"]["at"], BOOKS_TOP), yaw=LAYOUT["A"]["yaw"], mass=BASE_MASS,
                    shapes=tuple(_base_shapes()), joints=(LID,), friction=(0.6, 0.5), contact_offset=0.002)
BOOKS = parts.Part("books", pos=(*BOOKS_C, TOP), shapes=tuple(_books_shapes()), friction=(0.6, 0.5),
                   contact_offset=0.002)
STAND = parts.Part("stand", pos=(*STAND_C, TOP), shapes=tuple(_stand_shapes()), friction=(0.6, 0.5),
                   contact_offset=0.002)

OBJECTS = [LAPTOP, BOOKS, STAND]
LID_COLLIDE = tuple(s for s in LID.shapes if s.collide)
BASE_COLLIDE = tuple(s for s in LAPTOP.shapes if s.collide)


def _lot(name: str) -> lots.Lot:
    L = LAYOUT[name]
    return lots.Lot(name, place={"laptop": (*L["at"], BOOKS_TOP)}, yaw={"laptop": L["yaw"]})


LOTS = lots.declare(OBJECTS, _lot("A"), _lot("B"))
SUCCESS_HOLD_S = 1.5


# -- geometry helpers ---------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def shape_gap(pts, shape, origin, rot) -> float:
    """Closest signed distance (m, negative inside) from any point to one box of a link whose frame is at world
    ``origin``/``rot`` (shape centres given relative to that frame's origin)."""
    c = np.asarray(origin, float) + rot @ np.asarray(shape.at, float)
    r = rot @ shape.R
    loc = (np.asarray(pts, float) - c) @ r
    q = np.abs(loc) - np.asarray(shape.size, float) / 2
    out = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    return float(out.min())


def lid_pose(st):
    """World (origin, rotation) of the lid link: its frame sits at the hinge and turns with the lid."""
    if "lid" in getattr(st, "links", {}):
        p, q = st.links["lid"]
        return np.asarray(p, float), _rot(q)
    r = _rot(st.quat_xyzw)
    jr = _roty(float(st.joints.get("lid", 0.0)))
    return np.asarray(st.pos, float) + r @ np.asarray(PIVOT, float), r @ jr


def lid_gap(pts, st) -> float:
    if len(pts) == 0:
        return math.inf
    o, r = lid_pose(st)
    piv = np.asarray(PIVOT, float)
    best = math.inf
    for s in LID_COLLIDE:
        c = o + r @ (np.asarray(s.at, float) - piv)
        rr = r @ s.R
        loc = (np.asarray(pts, float) - c) @ rr
        q = np.abs(loc) - np.asarray(s.size, float) / 2
        d = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
        best = min(best, float(d.min()))
    return best


def base_gap(pts, st) -> float:
    if len(pts) == 0:
        return math.inf
    o, r = np.asarray(st.pos, float), _rot(st.quat_xyzw)
    return min(shape_gap(pts, s, o, r) for s in BASE_COLLIDE)


def tilt_of(st) -> float:
    """Radians the base's z axis leans off the vertical."""
    return math.acos(max(-1.0, min(1.0, float(_rot(st.quat_xyzw)[2, 2]))))


def lid_angle(st) -> float:
    """Radians the lid has turned from where it starts (positive: further open, negative: toward closed)."""
    return float(st.joints.get("lid", 0.0))


def base_corners(st) -> np.ndarray:
    """World positions of the base's eight corners."""
    r, p = _rot(st.quat_xyzw), np.asarray(st.pos, float)
    pts = np.array([[sx * LAP_D / 2, sy * LAP_W / 2, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, BASE_T)])
    return p + pts @ r.T


def base_low(st) -> float:
    return float(base_corners(st)[:, 2].min())


def _in_rect(xy, c, half_x, half_y) -> bool:
    return abs(float(xy[0]) - c[0]) <= half_x + EPS and abs(float(xy[1]) - c[1]) <= half_y + EPS


def base_centre(st) -> np.ndarray:
    """World position of the middle of the base's underside (the part's origin)."""
    return np.asarray(st.pos, float)


def on_books(st) -> bool:
    return bool(_in_rect(base_centre(st)[:2], BOOKS_C, BOOK_L / 2, BOOK_W / 2) and base_low(st) <= BOOKS_TOP + 0.004)


FACING_MAX = D(15.0)
OVER_MIN, OVER_MAX = 0.020, 0.090  # the front edge's overhang past the stand's front edge, every stable placement


def facing(st) -> float:
    """Radians the laptop is turned from square (keyboard toward the robot), seen from above."""
    r = _rot(st.quat_xyzw)
    return abs(math.atan2(r[1, 0], r[0, 0]))


def on_stand(st) -> bool:
    """The laptop rests on the stand the right way round: turned at most 15 degrees from square (keyboard toward the
    robot); its front edge 20 to 90 mm in front of the stand's front edge, measured at the base's middle (every
    placement in that range keeps its centre of mass over the stand); its middle within 25 mm side to side of the
    stand's middle; the middle of its underside within 3 mm of the stand's top; the base level to within 5
    degrees."""
    c = base_centre(st)
    over = STAND_FRONT - (c[0] - LAP_D / 2)
    return bool(facing(st) <= FACING_MAX + EPS and OVER_MIN - EPS <= over <= OVER_MAX + EPS
                and abs(c[1] - STAND_C[1]) <= 0.025 + EPS
                and -0.002 - EPS <= c[2] - STAND_TOP <= 0.003 + EPS and tilt_of(st) <= D(5.0) + EPS)


def over_stand(st) -> bool:
    """Held over the stand: the base's middle above the stand's top outline (in top view), the middle of its
    underside no lower than the stand's top."""
    c = base_centre(st)
    return bool(_in_rect(c[:2], STAND_C, STAND_D / 2, STAND_W / 2) and c[2] >= STAND_TOP - 0.002 - EPS)


def on_desk(st) -> bool:
    """On the padded desk: the lowest base corner is within 5 mm of the contact plane."""
    return base_low(st) <= TOP + 0.005 + EPS


def over_books(st) -> bool:
    """Any part of the base's outline over the books' outline, seen from above (separating-axis test of the two
    rectangles; touching edges count as over)."""
    r = _rot(st.quat_xyzw)
    corners = base_corners(st)[[0, 2, 4, 6], :2]  # the four bottom corners (x, y)
    books = np.array([[BOOKS_C[0] + sx * BOOK_L / 2, BOOKS_C[1] + sy * BOOK_W / 2] for sx in (-1, 1) for sy in (-1, 1)])
    for axis in (np.array([1.0, 0.0]), np.array([0.0, 1.0]), r[:2, 0], r[:2, 1]):
        n = float(np.linalg.norm(axis))
        if n < 1e-9:
            continue
        a_, b_ = corners @ (axis / n), books @ (axis / n)
        if a_.max() < b_.min() - EPS or b_.max() < a_.min() - EPS:
            return False
    return True


def off_books(st) -> bool:
    """Lifted clear of the books: the base's underside 10 mm or more above the books' top, or no part of the base
    over the books seen from above; and not on the desk."""
    return bool((base_low(st) >= BOOKS_TOP + 0.010 - EPS or not over_books(st)) and not on_desk(st))


# -- the trial record ---------------------------------------------------------------------------------------------------
TOUCH = 0.001  # geometric contact tolerance; a lid brush alone is allowed
PINCH_SPAN = 0.010  # opposite-face contact witnesses must be within one 10 mm patch
CLEAR = 0.030  # let go: every sampled hand point at least 30 mm from the laptop
HOLD_S = 1.0
STILL_M = 0.001
TIP_MAX = D(25.0)
FALL_V = 0.02  # m/s: downward motion during an observed unsupported, unheld interval
FALL_M = 0.002  # m: position noise allowance for the same interval
SUPPORT_GAP = 0.003  # m: support contact tolerance, including contact give
_rec = parts.__dict__.setdefault("laptop_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", zero=False, why="", gaps=None, snap=None, since=None, held=False,
                t=0.0, lifted=False, prev=None, lid_anchor=None, contacts=None)


if not _rec:
    reset_record()


def _snapshot(objects) -> np.ndarray:
    st = objects["laptop"]
    o, r = lid_pose(st)
    lid = []
    for shape in LID_COLLIDE:
        c = o + r @ (np.asarray(shape.at) - np.asarray(PIVOT))
        rr = r @ shape.R
        h = np.asarray(shape.size) / 2
        lid.extend(c + rr @ (h * np.array([x, y, z]))
                   for x in (-1, 1) for y in (-1, 1) for z in (-1, 1))
    return np.concatenate([base_corners(st), np.asarray(lid)])


def end_state(objects) -> bool:
    return bool(on_stand(objects["laptop"]))


def _clip_bottom(poly, axis, bound, sign):
    """Clip a 3-D bottom-face polygon by one vertical half-plane."""
    out = []
    if len(poly) == 0:
        return np.empty((0, 3))
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        da, db = sign * (a[axis] - bound), sign * (b[axis] - bound)
        ina, inb = da <= EPS, db <= EPS
        if ina:
            out.append(a)
        if ina != inb:
            out.append(a + (b - a) * da / (da - db))
    return np.asarray(out).reshape((-1, 3))


def _support_at(st, centre, half_x, half_y, height):
    """Some bottom-face area touches this actual horizontal support, not just its infinite plane."""
    poly = base_corners(st)[[0, 2, 6, 4]]
    for axis, c, h in ((0, centre[0], half_x), (1, centre[1], half_y)):
        poly = _clip_bottom(poly, axis, c + h, 1)
        poly = _clip_bottom(poly, axis, c - h, -1)
    if not len(poly):
        return False
    low = float(poly[:, 2].min())
    return height - 0.002 - EPS <= low <= height + SUPPORT_GAP + EPS


def supported(st):
    """Contact with the books, stand or padded desk, regardless of whether it is a target placement."""
    from yam_arena import station

    return any((_support_at(st, BOOKS_C, BOOK_L / 2, BOOK_W / 2, BOOKS_TOP),
                _support_at(st, STAND_C, STAND_D / 2, STAND_W / 2, STAND_TOP),
                _support_at(st, station.TABLE_CENTER[:2], station.TABLE_SIZE[0] / 2,
                            station.TABLE_SIZE[1] / 2, TOP)))


def observe(objects, contacts, t) -> None:
    """One measured control step. Missing or malformed observations fail closed permanently.

    contacts comes from contact_observation(), not an action target or a policy-declared grasp.
    A sampled lid pinch or unsupported lid hold is a zero; a brush alongside base support is allowed.
    An observed unheld/unsupported descent is a drop. Inter-frame release/landing is unresolved at 20 Hz.
    """
    _rec["steps"] += 1
    try:
        t = float(t)
        if not math.isfinite(t) or (_rec["prev"] is not None and t <= _rec["prev"][0]):
            raise ValueError("missing or non-increasing observation time")
        st = objects["laptop"]
        for name in ("books", "stand"):
            fixed = objects.get(name)
            for field, shape in (("pos", (3,)), ("quat_xyzw", (4,)), ("lin_vel", (3,)), ("ang_vel", (3,))):
                value = np.asarray(getattr(fixed, field, None), float)
                if value.shape != shape or not np.isfinite(value).all():
                    raise ValueError("fixed support state unknown: " + name)
            if np.linalg.norm(fixed.quat_xyzw) <= EPS:
                raise ValueError("fixed support rotation unknown: " + name)
        p, q = np.asarray(st.pos, float), np.asarray(st.quat_xyzw, float)
        v, w = np.asarray(st.lin_vel, float), np.asarray(st.ang_vel, float)
        if p.shape != (3,) or q.shape != (4,) or v.shape != (3,) or w.shape != (3,) or \
                not all(np.isfinite(a).all() for a in (p, q, v, w)) or np.linalg.norm(q) <= EPS or \
                "lid" not in st.joints or not math.isfinite(float(st.joints["lid"])):
            raise ValueError("object pose, velocity or lid joint unknown")
        if "lid" not in st.links or len(st.links["lid"]) != 2:
            raise ValueError("lid link pose unknown")
        lp, lq = (np.asarray(a, float) for a in st.links["lid"])
        if lp.shape != (3,) or lq.shape != (4,) or not np.isfinite(lp).all() or \
                not np.isfinite(lq).all() or np.linalg.norm(lq) <= EPS:
            raise ValueError("lid link pose unknown")
        if contacts is None or not all(k in contacts for k in
                ("lid_gap", "hand_gap", "lid_contact", "base_support", "display_pinch")):
            raise ValueError("arm contact state unknown")
        if not all(math.isfinite(float(contacts[k])) for k in ("lid_gap", "hand_gap")) or \
                not all(isinstance(contacts[k], (bool, np.bool_)) for k in
                        ("lid_contact", "base_support", "display_pinch")):
            raise ValueError("arm contact state malformed")
        snap = _snapshot(objects)
        if not np.isfinite(snap).all():
            raise ValueError("object state unknown")
        gaps = (float(contacts["lid_gap"]), float(contacts["hand_gap"]))
        surface = supported(st)
        free = not surface and not contacts["base_support"]
        if contacts["base_support"]:
            _rec["lid_anchor"] = None
        elif contacts["lid_contact"]:
            _rec["lid_anchor"] = float(p[2]) if _rec["lid_anchor"] is None else min(_rec["lid_anchor"], float(p[2]))
        elif surface or contacts["base_support"]:
            _rec["lid_anchor"] = None
        display_lift = contacts["lid_contact"] and not contacts["base_support"] and \
            (free or (_rec["lid_anchor"] is not None and float(p[2]) > _rec["lid_anchor"] + 0.005 + EPS))
        prev = _rec["prev"]
        falling = float(v[2]) < -FALL_V or (prev is not None and float(p[2]) < prev[1] - FALL_M - EPS)
        # Preserve an observed free-flight interval through its landing; support at impact cannot erase it.
        dropped = falling and (free or (prev is not None and prev[2]))
        if not _rec["zero"]:
            if contacts["display_pinch"]:
                _rec.update(zero=True, why="opposing fingertips gripped the display")
            elif display_lift:
                _rec.update(zero=True, why="the laptop was lifted or supported by the display without base-hand support")
            elif abs(lid_angle(st)) > LID_LIMIT + EPS:
                _rec.update(zero=True, why="the lid turned more than 25 degrees")
            elif tilt_of(st) > TIP_MAX:
                _rec.update(zero=True, why="the laptop tilted more than 25 degrees")
            elif float(p[2]) < TOP - 0.02:
                _rec.update(zero=True, why="the laptop left the desk")
            elif dropped:
                _rec.update(zero=True, why="observed unsupported descent without base-hand support")
        _rec["contacts"] = dict(contacts, surface_support=surface, free=free, falling=falling)
        _rec["t"] = t
        _rec["prev"] = (t, float(p[2]), free)
        if off_books(st):
            _rec["lifted"] = True
        done = end_state(objects) and gaps[1] >= CLEAR - EPS
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    _rec["gaps"] = gaps
    if _rec["snap"] is None or float(np.linalg.norm(snap - _rec["snap"], axis=1).max()) > STILL_M + EPS:
        _rec["snap"] = snap
        _rec["since"] = t if done else None
    elif not done:
        _rec["since"] = None
    elif _rec["since"] is None:
        _rec["since"] = t
    _rec["held"] = _rec["since"] is not None and t - _rec["since"] >= HOLD_S - EPS


# -- scoring ------------------------------------------------------------------------------------------------------------
def score(objects):
    """Whole-trial zeros: opposing fingertips gripping the display; a display-only hold unsupported by a surface;
    a display-only lift of more than 5 mm while contact persists; lid movement or base tilt over 25 degrees;
    observed unheld unsupported descent; leaving the desk; unknown
    observations. Incidental lid contact and controlled padded-desk set-downs are allowed. End-state scores and
    the 30 mm clearance / 1 s stillness requirement remain unchanged. A laptop left on the desk scores 0.
    """
    try:
        if not _rec["known"] or _rec["zero"] or _rec["steps"] == 0 or _rec["gaps"] is None:
            return 0.0
        st = objects["laptop"]
        if abs(lid_angle(st)) > LID_LIMIT + EPS:
            return 0.0
        if on_stand(st):
            if _rec["held"] and end_state(objects) and _rec["gaps"][1] >= CLEAR - EPS:
                return 1.0
            return 0.75
        if over_stand(st):
            return 0.5
        if off_books(st):
            return 0.25
        return 0.0
    except (KeyError, ValueError, TypeError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        return float(np.linalg.norm(base_centre(objects["laptop"])[:2] - np.asarray(STAND_C)))
    except Exception:  # noqa: BLE001
        return 2.0


# -- the arms as point clouds: hull faces sampled every 3 to 5 mm ----------------------------------------------------
HAND_LINKS = ("link5", "gripper", "tip_left", "tip_right")
ARM_LINKS = ("link3", "link4") + HAND_LINKS  # the lid rule counts the forearm too
_FACES: dict = {}


def _hull_face_points(hull, step) -> np.ndarray:
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


def link_points(name, step=0.003) -> np.ndarray:
    key = (name, step)
    if key not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[key] = np.concatenate([_hull_face_points(h, step) for h in hs])
    return _FACES[key]


STEP = {"link3": 0.008, "link4": 0.008, "link5": 0.005, "gripper": 0.005, "tip_left": 0.003, "tip_right": 0.003}


def hand_points(link_poses: dict, links=HAND_LINKS) -> np.ndarray:
    return np.concatenate([link_points(name, STEP[name]) @ m[:3, :3].T + m[:3, 3]
                           for (_, name), m in link_poses.items() if name in links])


def contact_observation(objects, clouds):
    """Contact proxies from measured link hull points, retaining arm and fingertip identity.

    Front/back mean the two broad faces of the articulated display slab, including its free rim.
    A base-side graze is not base support. No force or damage measurement is claimed.
    """
    expected = {(arm, link) for arm in ("left_robot", "right_robot") for link in ARM_LINKS}
    if clouds is None or set(clouds) != expected:
        return None
    if any(np.asarray(p).ndim != 2 or np.asarray(p).shape[1] != 3 or len(p) == 0 or
           not np.isfinite(p).all() for p in clouds.values()):
        return None
    st = objects["laptop"]
    hands = np.concatenate([p for (_, link), p in clouds.items() if link in HAND_LINKS])
    arms = np.concatenate(list(clouds.values()))
    lid_g = lid_gap(arms, st)
    hand_g = min(lid_gap(hands, st), base_gap(hands, st))
    local = (hands - np.asarray(st.pos, float)) @ _rot(st.quat_xyzw)
    under = (np.abs(local[:, 0]) <= LAP_D / 2 + EPS) & (np.abs(local[:, 1]) <= LAP_W / 2 + EPS) & \
            (np.abs(local[:, 2]) <= SUPPORT_GAP + EPS)
    o, r = lid_pose(st)
    sides = []
    for (_, link), points in clouds.items():
        if link not in ("tip_left", "tip_right"):
            continue
        face = [[], []]
        for shape in LID_COLLIDE:
            c = o + r @ (np.asarray(shape.at, float) - np.asarray(PIVOT, float))
            local = (points - c) @ (r @ shape.R)
            h = np.asarray(shape.size, float) / 2
            inside = (np.abs(local[:, 0]) <= h[0] + TOUCH + EPS) & \
                     (np.abs(local[:, 1]) <= h[1] + TOUCH + EPS)
            for i, sign in enumerate((-1, 1)):
                hit = inside & (np.abs(local[:, 2] - sign * h[2]) <= TOUCH + EPS)
                face[i].extend(local[hit, :2])
        sides.append([np.asarray(f).reshape((-1, 2)) for f in face])
    def opposing(a, b):
        return bool(len(a) and len(b) and np.any(
            np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2) <= PINCH_SPAN + EPS))
    pinch = any(opposing(a[0], b[1]) or opposing(a[1], b[0])
                for i, a in enumerate(sides) for b in sides[i + 1:])
    return dict(lid_gap=lid_g, hand_gap=hand_g, lid_contact=bool(lid_g <= TOUCH + EPS),
                base_support=bool(np.any(under)), display_pinch=bool(pinch))


def _arm_poses(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ARM_LINKS:
                if not np.isfinite(poses[i]).all() or np.linalg.norm(poses[i, 3:]) <= EPS:
                    return None
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                out[(arm, n)] = m
    if len(out) != 2 * len(ARM_LINKS) or not all(np.isfinite(m).all() for m in out.values()):
        return None
    return out


@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["last_t"] = None


_stock_apply = lots._apply


def _apply_lot(emb) -> bool:
    moved = _stock_apply(emb)
    reset_record()
    _rec["last_t"] = None
    return moved


lots._apply = _apply_lot


@parts.on_step
def _step(sim) -> None:
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001
        objects = {}
    g = None
    try:
        poses = _arm_poses(sim)
        if poses is not None:
            clouds = {(arm, link): link_points(link, STEP[link]) @ m[:3, :3].T + m[:3, 3]
                      for (arm, link), m in poses.items()}
            g = contact_observation(objects, clouds)
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = (str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("laptop.jsonl")) if os.environ.get("TRACE_OUT")
           else os.environ.get("LAPTOP_LOG"))
    if log and objects:
        try:
            st = objects["laptop"]
            r = _rot(st.quat_xyzw)
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), zero=_rec["zero"],
                       why=_rec["why"], pos=np.round(st.pos, 4).tolist(), tilt=round(math.degrees(tilt_of(st)), 1),
                       yaw=round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 1),
                       lid=round(math.degrees(lid_angle(st)), 1), low=round(base_low(st) - TOP, 4),
                       on_stand=on_stand(st), over=over_stand(st), off_books=off_books(st),
                       gaps=None if _rec["gaps"] is None else [round(v, 4) for v in _rec["gaps"]],
                       held=_rec["held"], known=_rec["known"], reason=_rec["reason"],
                       contacts=_rec["contacts"], velocity=np.round(st.lin_vel, 4).tolist())
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                a = base.scene[arm]
                poses = parts._np(a.data.body_link_pose_w)[0]
                row[arm] = np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist()
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the layout the top camera shows ------------------------------------------------------------
# Both arms together. Each comes in from the front, leaned 80 degrees forward with its fingers closing up and down
# (10 degrees off vertical), and pinches the base's front edge where it overhangs the books, 90 mm either side of the
# laptop's middle (pads up to 43 mm in from the edge, 165 mm and more in front of the hinge, far from the lid).
# Together they lift the front about 85 mm (the back edge stays on the books until the front is about 70 mm up; the laptop
# then hangs about 15 degrees nose up in the pinch), wait, carry it to above the stand, lower it until the base rests
# on the stand, open, draw straight back toward the robot and go back to rest.
# With LAPTOP_DEMO=naive the right arm instead comes in from the right side and pinches the lid's right side edge a
# quarter of the way up the screen (holding it by the display), lifts it 30 mm, holds it, puts it back and lets
# go; the left arm stays at rest. That scores 0.
GRIP_IN = 0.030  # TCP this far in from the base's front edge (pad tips 43 mm in)
GRIP_Y = 0.090  # each pinch this far to the side of the laptop's middle
TILT_DEG = 80.0
OPEN_PART = -0.020  # fingers opened 40 mm while coming in, clear of the desk and the books
LIFT = 0.030
CARRY_Z = 0.065  # pinch this far above the stand's top while crossing (the base hangs nose up)
SAG = 0.004  # the wrists sag a little under the load; aim this much high while holding


def _w(d, p, finger, label, rot):
    return Waypoint(d, tuple(float(v) for v in p), finger, label, tilt._TiltedYaw(0.0, rot))


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def _front_grip(arm: str, heading: float) -> np.ndarray:
    """Grip leaned TILT_DEG from straight down toward ``heading`` (rad, world), fingers closing in the vertical
    plane through that direction. The two arms use opposite yaws (the same grip, turned 180 degrees) to stay in
    their wrists' range."""
    yaw = heading - math.pi / 2 if arm == "left" else heading + math.pi / 2
    return tilt.rotation(yaw, TILT_DEG, heading)


def _lid_grip(heading: float) -> np.ndarray:
    """Grip pointing horizontally toward ``heading`` with the fingers closing horizontally across it (across the
    lid's thickness when it comes in from the laptop's side)."""
    return tilt.rotation(heading, 90.0, heading)


def demo():
    seen = lots.current()
    lx, ly, _ = seen.pos("laptop")
    yaw = seen.yaw_of("laptop")
    c, s = math.cos(yaw), math.sin(yaw)
    ex, ey = np.array([c, s]), np.array([-s, c])  # the laptop's +x (toward the hinge) and +y (its left)
    mid = np.array([lx, ly])
    dest = np.array(TARGET)
    naive = os.environ.get("LAPTOP_DEMO") == "naive"
    plans = {}
    for arm, side in (("left", 1.0), ("right", -1.0)):
        if naive and arm == "left":
            rest, ryaw = _rest_pose(arm)
            plans[arm] = [Waypoint(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw),
                          Waypoint(26.0, rest, FINGER_OPEN, "stay at rest", ryaw)]
            continue
        if naive:  # the lid's right side edge, a quarter of the way up the screen, taken from the side
            lp = np.asarray(PIVOT, float) + R_OPEN @ np.array([-LAP_D / 4, 0.0, LID_T / 2])
            off = np.array([float(lp[0]), -(LAP_W / 2 - 0.015)])
            out = np.array([float(lp[0]), -(LAP_W / 2 + 0.050)])
            dz = float(lp[2])
            g0, gd = _lid_grip(yaw + math.pi / 2), _lid_grip(math.pi / 2)
            squeeze = FINGER_CLOSED
        else:
            off = np.array([-LAP_D / 2 + GRIP_IN, side * GRIP_Y])
            out = np.array([-LAP_D / 2 - 0.030, side * GRIP_Y])
            dz = BASE_T / 2
            g0, gd = _front_grip(arm, yaw), _front_grip(arm, 0.0)
            squeeze = FINGER_CLOSED  # 5 mm past each face of the 10 mm base: about 25 N a finger
        p0 = mid + off[0] * ex + off[1] * ey
        q0 = mid + out[0] * ex + out[1] * ey
        pd = dest + off
        qd = dest + out
        z0 = BOOKS_TOP + dz
        zd = STAND_TOP + dz
        zc = STAND_TOP + CARRY_Z + dz
        if naive:  # take it by the lid's edge, lift it 30 mm off the books, hold, put it back and let go
            work = [
                _w(2.5, (*q0, z0 + 0.030), OPEN_PART, "beside the lid", g0),
                _w(1.2, (*q0, z0), OPEN_PART, "down to a quarter of the way up the lid", g0),
                _w(1.8, (*p0, z0), OPEN_PART, "around the lid's edge", g0),
                _w(0.8, (*p0, z0), squeeze, "pinch the lid", g0),
                _w(1.5, (*p0, z0 + LIFT), squeeze, "lift the laptop by the lid", g0),
                _w(3.0, (*p0, z0 + LIFT), squeeze, "hold it up", g0),
                _w(1.5, (*p0, z0), squeeze, "put it back", g0),
                _w(0.8, (*p0, z0), OPEN_PART, "let go", g0),
                _w(1.5, (*q0, z0), OPEN_PART, "out", g0),
            ]
            rest, ryaw = _rest_pose(arm)
            work += [Waypoint(2.0, rest, FINGER_OPEN, "back to the rest pose", ryaw)]
            work[:0] = [Waypoint(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw)]
            plans[arm] = work
            continue
        work = [
            _w(2.5, (*q0, z0 + 0.030), OPEN_PART, "in front of the edge", g0),
            _w(1.2, (*q0, z0), OPEN_PART, "down to its height", g0),
            _w(1.8, (*p0, z0), OPEN_PART, "in over and under the edge", g0),
            _w(0.8, (*p0, z0), squeeze, "pinch", g0),
            _w(0.6, (*p0, z0 + SAG), squeeze, "take the weight", g0),
            _w(1.5, (*p0, z0 + LIFT + SAG), squeeze, "lift together", g0),
            _w(1.2, (*p0, zc + SAG), squeeze, "up to carry height", g0),
            _w(1.2, (*p0, zc + SAG), squeeze, "steady, until the back edge is off the books", g0),
            _w(4.0, (*pd, zc + SAG), squeeze, "carry it over the stand", gd),
            _w(0.8, (*pd, zc + SAG), squeeze, "steady", gd),
            _w(2.0, (*pd, zd + 0.008 + SAG), squeeze, "lower toward the stand", gd),
            _w(1.5, (*pd, zd), squeeze, "set it down", gd),
            _w(0.8, (*pd, zd), OPEN_PART, "let go", gd),
            _w(1.8, (*qd, zd), OPEN_PART, "draw back", gd),
            _w(1.0, (*qd, zd + 0.06), OPEN_PART, "up", gd),
        ]
        rest, ryaw = _rest_pose(arm)
        work += [Waypoint(2.0, rest, FINGER_OPEN, "back to the rest pose", ryaw),
                 Waypoint(3.0, rest, FINGER_OPEN, "wait", ryaw)]
        work[:0] = [Waypoint(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw),
                    Waypoint(0.5, rest, FINGER_OPEN, "settle at the rest pose", ryaw)]
        plans[arm] = work
    return plans
