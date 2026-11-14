"""On the station table: a sealed steel food can (a standard 15 oz can, 7.6 cm wide and 11.3 cm tall) stands in
front of the arms. An ordinary two-handle rotary can opener (the "swing" pattern: two 15 to 16 cm handles hinged at the
head, a cutting wheel on the upper jaw, a toothed feed wheel and a wing key on the lower) lies flat on the table to
its left, handles open, key up. The robot must open the can: set the opener's head on the rim, squeeze the handles
shut, turn the key until the lid is cut all the way round, then take the opener off and put it down on the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from openerkit import (Hinge, clamp, env, gripper_points, on_reset, on_step, part, set_clamp,  # noqa: E402
                       set_clamp_target, set_weld, weld)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Open the can with the can opener: clamp it on the rim, turn the key until the lid is cut all the way "
               "round, then take the opener off and put it on the table.")

STEEL = (0.70, 0.71, 0.73)
TIN = (0.80, 0.81, 0.83)
DARK = (0.25, 0.26, 0.28)
RED = (0.72, 0.12, 0.10)
CREAM = (0.93, 0.88, 0.70)
BLACK = (0.08, 0.08, 0.09)
HANDLE = (0.10, 0.32, 0.62)   # vinyl-coated handles
GRIP = (0.09, 0.09, 0.10)     # the plastic grips drawn over the handles
CHROME = (0.80, 0.81, 0.83, 0.8)
DULL = (0.50, 0.51, 0.53, 0.6)
HID = {"hidden": True}        # collides, drawn by the meshes below instead

# -- the can (a 300 x 407 "15 oz" food can), frame: origin on the table under its axis -------------------------------
CAN_XY = (0.42, -0.04)
R_BODY = 0.0365               # body wall
R_SEAM = 0.0381               # over the double seam (3 inch nominal)
R_INNER = 0.0335              # inside the seam, where the lid's countersink starts
H_SEAM = 0.1127               # top of the seam (4 7/16 inch)
SEAM_H = 0.0035               # the seam bead's height
FILL_TOP = 0.1035             # the contents (and headspace floor)
LID_R, LID_T = 0.0330, 0.0015
LID_TOP = H_SEAM - 0.0032     # the lid sits 3.2 mm below the seam top (countersink)
CAN_MASS, LID_MASS = 0.47, 0.008


def _ring(r_in, r_out, z0, z1, n, color):
    """A ring of ``n`` boxes, radii r_in..r_out, heights z0..z1."""
    rm, w = (r_in + r_out) / 2, 2 * math.pi * r_out / n * 1.05
    return [((rm * math.cos(a), rm * math.sin(a), (z0 + z1) / 2), (r_out - r_in, w, z1 - z0), color, {"yaw": a})
            for a in (2 * math.pi * (k + 0.5) / n for k in range(n))]


CAN_BOXES = (_ring(R_INNER, R_SEAM, H_SEAM - SEAM_H, H_SEAM, 36, TIN)
             + _ring(R_INNER, R_BODY, FILL_TOP, H_SEAM - SEAM_H, 36, TIN)
             + [((-R_BODY - 0.0006, 0.0, 0.053), (0.002, 0.012, 0.080), CREAM, {"visual": True})])   # stripe on label
CAN_CYLS = (
    ((0.0, 0.0, FILL_TOP / 2), R_BODY, FILL_TOP, TIN),
    ((0.0, 0.0, 0.002), R_SEAM, 0.004, TIN),                                    # bottom seam
    ((0.0, 0.0, 0.053), R_BODY + 0.0005, 0.080, RED, "Z", {"visual": True}),    # paper label
)
CAN = part("can", pos=(*CAN_XY, TOP), boxes=CAN_BOXES, cylinders=CAN_CYLS, mass=CAN_MASS, color=TIN, friction=0.5)
LID = part("lid", pos=(*CAN_XY, TOP + LID_TOP - LID_T / 2), mass=LID_MASS, color=TIN, friction=0.5,
           cylinders=(((0.0, 0.0, 0.0), LID_R, LID_T, TIN),))
weld(LID, CAN, "")

# -- the opener, frame: origin at the rim point under its head when seated (the seam top), x along the handles (away
# from the head), y outward from the can (the key side), z up as it is used ------------------------------------------
SHOE_Z = 0.001                    # the shoe's underside is this far above the frame origin; seated, it rests on the seam
PIVOT = (-0.016, 0.014, 0.006)    # the jaws' hinge, axis along y
GRIP_T = 0.10                     # where the demo's gripper holds the handles (middle of the grip, along x)
OPEN_ANGLE = math.radians(25)     # the handles open this far at most; the upper hinge reads 0 there, 25 deg shut
SPRING_PAST = math.radians(15)    # the opening spring's rest angle is this far past the open stop, so it holds them
                                  # open: shut, it pushes with SPRING * 40 deg
SPRING = 2.9                      # N m/rad: holding them shut takes 2.0 N m, about 15 N at the middle of the grip
KEY_DAMP = 0.15                   # N m s/rad at the key: the effort of cutting (about 0.3 N m at 2 rad/s)
KEY_C = (0.0, 0.048, -0.006)      # centre of the wing key; its axis is y, through the feed wheel
FEED_R, RIM_R = 0.006, 0.0375     # feed wheel and the seam it drives (under the bead): radii, m
RATIO = FEED_R / RIM_R            # the can turns 0.16 of a turn for each turn of the key

OPENER_BOXES = (
    ((0.006, 0.015, -0.005), (0.048, 0.022, 0.034), STEEL, HID),        # head, outside the seam
    ((0.000, -0.003, 0.003), (0.024, 0.014, 0.004), STEEL),             # shoe resting on the seam top (z 1-5 mm)
    ((0.110, 0.014, -0.0115), (0.160, 0.016, 0.009), HANDLE, HID),      # lower handle
    ((0.008, 0.008, 0.000), (0.010, 0.006, 0.006), DARK, {"visual": True, **HID}),   # feed wheel guard
)
OPENER_CYLS = (
    ((0.0, 0.004, -0.006), FEED_R, 0.004, DARK, "Y", {"visual": True, **HID}), # toothed feed wheel, under the bead
    ((0.0, 0.031, -0.006), 0.004, 0.012, STEEL, "Y"),                         # key shaft
)

# -- the opener's look (drawn only; it collides as the boxes and cylinders above) -----------------------------------


def _rounded(corners, r, m=6):
    """A convex outline (2D, counter-clockwise): the corners rounded off with radius r, m points per corner."""
    out = []
    for i, (x, y) in enumerate(corners):
        (px, py), (nx, ny) = corners[i - 1], corners[(i + 1) % len(corners)]
        a0, a1 = math.atan2(px - x, y - py), math.atan2(x - nx, ny - y)
        a1 += 2 * math.pi if a1 < a0 else 0.0
        out += [(x + r * math.cos(a0 + (a1 - a0) * j / (m - 1)), y + r * math.sin(a0 + (a1 - a0) * j / (m - 1)))
                for j in range(m)]
    return out


def _rect(w, h, r):
    return _rounded([(-w / 2 + r, -h / 2 + r), (w / 2 - r, -h / 2 + r), (w / 2 - r, h / 2 - r), (-w / 2 + r, h / 2 - r)], r)


def _gear(n, r_tip, r_root):
    return [(rr * math.cos(2 * math.pi * (k + f) / n), rr * math.sin(2 * math.pi * (k + f) / n))
            for k in range(n) for f, rr in ((0.0, r_root), (0.2, r_tip), (0.5, r_tip), (0.7, r_root))]


def _circle(r, n=24):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _loft(sections, color):
    """A solid through closed 3D sections of equal point count, capped at both ends (fan from each end's centre)."""
    n, pts = len(sections[0]), [p for s in sections for p in s]
    faces = [(i * n + k, i * n + (k + 1) % n, (i + 1) * n + (k + 1) % n, (i + 1) * n + k)
             for i in range(len(sections) - 1) for k in range(n)]
    for s in (sections[0], sections[-1]):
        c = len(pts)
        pts += [tuple(np.mean(s, 0))] + list(s)
        faces += [(c, c + 1 + k, c + 1 + (k + 1) % n) for k in range(n)]
    return tuple(tuple(float(v) for v in p) for p in pts), tuple(faces), color


def _bar(x_sections, cy, cz, color):
    """Along x: sections (x, width in y, height in z, corner radius)."""
    return _loft([[(x, cy + u, cz + v) for u, v in _rect(w, h, r)] for x, w, h, r in x_sections], color)


def _plate_y(outlines, cx, cz, color):
    """Across y: sections (y, outline in (x, z) about (cx, cz))."""
    return _loft([[(cx + u, y, cz + v) for u, v in o] for y, o in outlines], color)


def _disc_y(cx, cz, y0, y1, r, color, teeth=0, r_root=0.0):
    o = _gear(teeth, r, r_root) if teeth else _circle(r)
    return _plate_y([(y0, o), (y1, o)], cx, cz, color)


def _grip(cy, cz):   # a plastic grip over the rear two thirds of a handle: the collision box's section, rounded
    return _bar([(0.075, 0.0145, 0.0075, 0.003), (0.078, 0.016, 0.009, 0.0035), (0.183, 0.016, 0.009, 0.0035),
                 (0.187, 0.015, 0.008, 0.0035), (0.190, 0.0125, 0.0055, 0.0025)], cy, cz, GRIP)


_HEAD = [(-0.014, -0.014), (0.0, -0.0185), (0.0265, -0.0145), (0.0265, 0.004), (0.008, 0.0085), (-0.014, 0.006)]
BODY_MESHES = (
    _grip(0.014, -0.0115),
    _bar([(0.020, 0.013, 0.004, 0.0015), (0.080, 0.013, 0.004, 0.0015)], 0.014, -0.0115, CHROME),   # bare steel
    _plate_y([(0.0235, _rounded(_HEAD, 0.0035)), (0.026, _rounded(_HEAD, 0.0035))], 0.0, 0.0, CHROME),  # outer plate
    _plate_y([(0.006, _rounded(_HEAD, 0.0035)), (0.0085, _rounded(_HEAD, 0.0035))], 0.0, 0.0, CHROME),  # inner plate
    _plate_y([(0.0085, _rounded(_HEAD, 0.0005)), (0.0235, _rounded(_HEAD, 0.0005))], 0.0, 0.0, DULL),
    _disc_y(0.0, -0.006, 0.0015, 0.0055, FEED_R, DULL, teeth=18, r_root=0.0050),    # toothed feed wheel
    _disc_y(0.0, -0.006, 0.0, 0.0015, 0.0025, CHROME),
    _disc_y(-0.010, 0.0035, 0.026, 0.0275, 0.0025, CHROME),                          # rivets
    _disc_y(0.0185, -0.0075, 0.026, 0.0275, 0.0022, CHROME),
)
UPPER_MESHES = (   # shut
    _grip(0.014, -0.0005),
    _bar([(0.028, 0.013, 0.004, 0.0015), (0.080, 0.013, 0.004, 0.0015)], 0.014, -0.0005, CHROME),
    _plate_y([(y, _rounded([(0.028, -0.002), (0.040, -0.002), (0.040, 0.010), (0.028, 0.010)], 0.0015))
              for y in (0.009, 0.019)], 0.0, 0.0, CHROME),
    _bar([(-0.018, 0.017, 0.005, 0.002), (-0.0165, 0.020, 0.007, 0.0025), (0.042, 0.020, 0.007, 0.0025)],
         0.0, 0.0095, CHROME),                                                         # jaw arm
    _plate_y([(-0.0046, _circle(0.0062)), (-0.0040, _circle(0.007)),
              (-0.0034, _circle(0.0062))], 0.0, 0.0, CHROME),                          # cutting wheel, 14 mm
    _disc_y(0.0, 0.0, -0.0030, -0.0008, 0.0058, DULL, teeth=14, r_root=0.0049),        # its gear
    _disc_y(0.0, 0.0, -0.0058, -0.0046, 0.0022, CHROME),
)


def _key_part(corners, r, color):   # a part of the wing key: outline in (x, y), 8 mm thick in z, edges rounded 1 mm
    return _loft([[(u, v, z) for u, v in _rounded(corners, rr)] for z, rr in
                  ((-0.010, r - 0.001), (-0.009, r), (-0.003, r), (-0.002, r - 0.001))], color)


KEY_MESHES = tuple(_key_part([(cx - 0.0015, 0.0475), (cx + 0.0015, 0.0475), (cx + 0.0015, 0.0485),
                              (cx - 0.0015, 0.0485)], 0.0075, BLACK) for cx in (-0.0185, 0.0185)) + (
    _key_part([(-0.011, 0.0425), (0.011, 0.0425), (0.011, 0.0495), (-0.011, 0.0495)], 0.0025, BLACK),
    _disc_y(0.0, -0.006, 0.036, 0.040, 0.003, BLACK),
)


def _open_pose(boxes, cyls, meshes):
    """The upper handle's shapes, given shut, turned about the hinge to its open stop: its joint is 0 there."""
    a, px, pz = OPEN_ANGLE, PIVOT[0], PIVOT[2]

    def turn(c):
        dx, dz = c[0] - px, c[2] - pz      # a turn of -a about y lifts the handle end
        return (px + dx * math.cos(a) - dz * math.sin(a), c[1], pz + dx * math.sin(a) + dz * math.cos(a))

    return (tuple((turn(c), size, col, {**(opt[0] if opt else {}), "pitch": -a}) for c, size, col, *opt in boxes),
            tuple((turn(c[0]), *c[1:]) for c in cyls),
            tuple((tuple(turn(q) for q in pts), faces, col) for pts, faces, col in meshes))


_UB, _UC, _UM = _open_pose((
    ((0.115, 0.014, -0.0005), (0.150, 0.016, 0.009), HANDLE, HID),    # upper handle
    ((0.012, 0.0, 0.0095), (0.060, 0.020, 0.007), STEEL, HID),        # jaw arm over the rim
), (
    ((0.0, -0.004, 0.0), 0.007, 0.002, BLACK, "Y", {"visual": True, **HID}),  # cutting wheel, inside the rim
), UPPER_MESHES)
UPPER = Hinge("upper", anchor=PIVOT, axis="Y", lower=0.0, upper=OPEN_ANGLE, start=0.0, target=-SPRING_PAST,
              rate=SPRING, damping=0.02, mass=0.045, armature=1e-4, boxes=_UB, cylinders=_UC, meshes=_UM)
KEY = Hinge("key", anchor=(0.0, 0.040, -0.006), axis="Y", damping=KEY_DAMP, mass=0.015, armature=2e-4,
            friction=0.2, boxes=(
                ((0.0, 0.048, -0.006), (0.055, 0.016, 0.008), DARK, HID), # the wings, 5.5 cm across, 8 mm thick
            ), cylinders=(
                ((0.0, 0.042, -0.006), 0.0035, 0.004, DARK, "Y"),         # hub
            ), meshes=KEY_MESHES)

# lying flat on the table, key up: opener x -> world +y, opener y -> world +z, opener z -> world +x
_LYING = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
_USE = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # seated at the can's front: y -> -x


def _quat(m):
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w)


OPENER_START = (0.30, 0.10, TOP + 0.0105)   # its frame origin; it rests on its shoe and the handles' undersides
OPENER = part("opener", pos=OPENER_START, boxes=OPENER_BOXES, cylinders=OPENER_CYLS, mass=0.12,
              hinges=(UPPER, KEY), color=STEEL, friction=0.8, quat=_quat(_LYING), meshes=BODY_MESHES)
# the clamp: the wheels closed on the rim hold the opener to the can and let them turn only about the can's axis
clamp(OPENER, CAN, local_self=(0.0, -R_SEAM, SHOE_Z), local_other=(0.0, 0.0, H_SEAM), stiffness=20.0, damping=0.5,
      max_torque=3.0)

OBJECTS = [CAN, LID, OPENER]

# -- clamping and cutting ----------------------------------------------------------------------------------------
READY_OPEN = math.radians(20)     # the head was set on the rim with the handles at least this far open
CLOSED = math.radians(3)          # squeezed within this of shut: the wheels close on the rim, the cutting wheel pierces
LET_GO = math.radians(6)          # opened past this, the wheels leave the rim
SEAT_RADIAL, SEAT_HEIGHT, SEAT_ALONG, SEAT_TILT = 0.003, 0.003, 0.006, math.radians(8)
RIM_BINS = 180                    # the rim, tracked in 2 degree steps
KNOCKED_DEG, UPRIGHT_DEG, MOVED_M = 30.0, 10.0, 0.05
CLEAR_M = 0.02
_S: dict = {}


@on_reset
def _reset(environment=None) -> None:
    _S.update(cut=np.zeros(RIM_BINS, bool), clamped=False, ready=False, pierced=False, released=False, theta=None,
              key=None, key_raw=None, key_u=0.0, key0=0.0, theta0=0.0, knocked=False, start=None, step=0, scored=None, last=(0.0, False))
    if environment is not None:
        set_weld("lid", True)
        set_clamp("opener", False)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _poses(environment):
    from openerkit import _SPECS, _part_state

    return _part_state(environment, _SPECS["opener"]), _part_state(environment, _SPECS["can"])


def seat_error(op, can) -> tuple[np.ndarray, float]:
    """The can's axis point at seam height in the opener's frame minus where it is when seated, and the tilt (rad)
    between the opener's up axis and the can's."""
    axis_pt = can.to_world((0.0, 0.0, H_SEAM))
    err = op.to_local(axis_pt) - np.array([0.0, -R_SEAM, SHOE_Z])
    tilt = math.acos(float(np.clip(op.rotation()[:, 2] @ can.rotation()[:, 2], -1.0, 1.0)))
    return err, tilt


def handle_opening(op) -> float:
    """How far the handles are open (rad): 0 shut, 25 degrees at the stop."""
    return OPEN_ANGLE - op.joints["upper"]


def rel_angle(op, can) -> float:
    """The opener's turn about the can's axis, relative to the can (rad)."""
    r = can.rotation().T @ op.rotation()
    return math.atan2(r[1, 0], r[0, 0])


@on_step
def _clamp_and_cut(environment) -> None:
    _S["step"] += 1
    op, can = _poses(environment)
    opening, raw = handle_opening(op), op.joints["key"]
    # the joint reading wraps round; follow the key's turn step by step
    if _S["key_raw"] is not None:
        _S["key_u"] += _wrap(raw - _S["key_raw"])
    _S["key_raw"] = raw
    key = _S["key_u"]
    err, tilt = seat_error(op, can)
    seated = (abs(err[0]) < SEAT_ALONG and abs(err[1]) < SEAT_RADIAL and abs(err[2]) < SEAT_HEIGHT
              and tilt < SEAT_TILT)
    theta = rel_angle(op, can)
    if not _S["clamped"]:
        if seated and opening > READY_OPEN:
            _S["ready"] = True
        elif not seated:
            _S["ready"] = False
        if _S["ready"] and opening < CLOSED:
            _S.update(clamped=True, pierced=True, ready=False, key0=key, theta0=theta, key=key, theta=theta)
            set_clamp_target("opener", theta)
            set_clamp("opener", True)
        return
    if opening > LET_GO:                     # let go: the wheels open off the rim
        _S.update(clamped=False, ready=False)
        set_clamp("opener", False)
        return
    # the feed wheel turns the rim past the head: the drive follows the key at the wheels' ratio
    set_clamp_target("opener", _S["theta0"] + RATIO * (key - _S["key0"]))
    d_key = key - _S["key"]
    d_theta = _wrap(theta - _S["theta"])
    _S["key"], _S["theta"] = key, theta
    # cut only where the rim really passed the cutting wheel in the cutting direction, driven by the key
    # (key turned clockwise seen from its end: its angle about the opener's y falls, and so does theta)
    if d_key < 0 and d_theta < 0 and not _S["released"]:
        step = max(d_theta, RATIO * d_key)   # both negative: the smaller turn
        a0, a1 = theta - step, theta
        k0, k1 = math.floor(math.degrees(a1) / 2), math.ceil(math.degrees(a0) / 2)
        for k in range(k0, max(k1, k0 + 1)):
            _S["cut"][k % RIM_BINS] = True
    if not _S["released"] and _S["cut"].all():
        _S["released"] = True
        set_weld("lid", False)


_reset()


def cut_fraction() -> float:
    return float(_S["cut"].mean())


def _box_gap(p, low, high) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(low - p, p - high), 0.0)))


def opener_put_down(op, can) -> bool:
    """The opener lies on the table: lowest point within 1.5 cm of the table, nothing above 8 cm, still, and its
    bounding box at least 1 cm from the can's side."""
    gap = math.hypot(*np.maximum(np.maximum(op.low[:2] - can.pos[:2], can.pos[:2] - op.high[:2]), 0.0))
    return bool(op.low[2] < TOP + 0.015 and op.high[2] < TOP + 0.08 and op.resting(0.01, 0.3)
                and gap > R_SEAM + 0.01)


def hands_off(op, can) -> bool:
    environment = env()
    if environment is None:
        return False
    for p in gripper_points(environment):
        if _box_gap(p, op.low, op.high) < CLEAR_M:
            return False
        r = math.hypot(p[0] - can.pos[0], p[1] - can.pos[1]) - R_SEAM
        h = max(p[2] - (can.pos[2] + H_SEAM), can.pos[2] - p[2], 0.0)
        if math.hypot(max(r, 0.0), h) < CLEAR_M:
            return False
    return True


def status(objects) -> dict:
    op, can = objects["opener"], objects["can"]
    if _S["start"] is None:
        _S["start"] = can.pos[:2].copy()
    tilt = math.degrees(math.acos(float(np.clip(can.rotation()[2, 2], -1.0, 1.0))))
    moved = float(np.linalg.norm(can.pos[:2] - _S["start"]))
    _S["knocked"] |= tilt > KNOCKED_DEG
    return dict(tilt=tilt, moved=moved, cut=cut_fraction(), clamped=_S["clamped"], pierced=_S["pierced"],
                released=_S["released"], knocked=_S["knocked"])


def _update(objects) -> tuple[float, bool]:
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    st = status(objects)
    op, can = objects["opener"], objects["can"]
    if st["knocked"]:
        result = (0.0, False)
    elif st["released"]:
        done = (not st["clamped"] and st["tilt"] < UPRIGHT_DEG and st["moved"] < MOVED_M and opener_put_down(op, can)
                and hands_off(op, can))
        result = (1.0, True) if done else (0.75, False)
    elif st["cut"] >= 0.5:
        result = (0.5, False)
    elif st["pierced"]:
        result = (0.25, False)
    else:
        result = (0.0, False)
    _S["last"] = result
    return result


def success(objects) -> bool:
    """The lid cut all the way round, the opener off the can and lying still on the table, the can upright within
    5 cm of its start, both grippers 2 cm clear; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if the can was ever tipped past 30 degrees. Else 1.0 as in success(); 0.75 the lid cut all the way round;
    0.5 at least half the rim cut; 0.25 the opener was clamped on the rim (lid pierced); 0 otherwise."""
    return _update(objects)[0]


def demo():
    """No waypoint plan: the full-score demo is closed loop (demo.py). An empty plan lets ``--policy yam_scripted``
    run the task with both arms holding still (the do-nothing check)."""
    return {}
