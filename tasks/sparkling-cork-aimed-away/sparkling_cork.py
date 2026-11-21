"""Open a half-bottle of sparkling wine at a set dinner table: stopper in the dish, bottle back on its coaster, the
stopper never aimed at the glass lamp above or at the guest across the table, and never let go of before it is out.

A 375 mL sparkling-wine bottle (foil and wire hood already off) stands on a stainless wine coaster under a glass pendant
lamp. A plastic sparkling-wine stopper is pushed into its neck. Across the table a guest (a seated mannequin) has a
plate and a champagne flute; to the right the table is empty up to the room wall. A small dish waits on the right.

Mechanism: the stopper is its own body, held in the neck by a PhysX D6 joint (bodykit Thread) that keeps it on the
bottle's axis and lets it slide out along that axis and turn about it. The fit is friction: the slide's drive is re-aimed
every control step at where the stopper is now, stiff, and capped at the pull force (PULL_N when seated, falling with the
length of shank left inside), so the stopper stays where it is unless a gripper pulls it harder than that, by contact.
When the shank's end passes the lip, the joint is switched off and the stopper is a loose body. That moment is "the
exit": the scorer records where the bottle's axis points and whether a gripper is closed on the stopper.

Frame: x forward from the arm bases, y left, z up, metres. All sizes, masses and places: layout.py.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
import layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape, SphereShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
CONTROL_HZ = 20.0

GLASS_C, LABEL_C, INK_C = (0.10, 0.30, 0.14), (0.93, 0.88, 0.72), (0.12, 0.10, 0.08)
STOPPER_C, STEEL_C, DISH_C = (0.97, 0.97, 0.95), (0.72, 0.73, 0.75), (0.95, 0.93, 0.88)
PLATE_C, FLUTE_C, MAT_C = (0.98, 0.98, 0.97), (0.80, 0.88, 0.92), (0.55, 0.20, 0.18)
LAMP_C, ROD_C, WALL_C = (0.97, 0.97, 0.96), (0.10, 0.10, 0.10), (0.88, 0.86, 0.80)
SKIN_C, SHIRT_C, CHAIR_C = (0.85, 0.72, 0.60), (0.85, 0.15, 0.15), (0.45, 0.30, 0.18)
GOLD_C, WIRE_C, RED_C, HAIR_C = (0.85, 0.68, 0.20), (0.75, 0.75, 0.78), (0.90, 0.05, 0.05), (0.30, 0.20, 0.12)

# a 5 x 7 block font for the label's word, printed in dark ink round the bottle's front (the side facing the robot)
FONT = {"B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
        "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
        "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
        "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100")}


def _word(text: str, z_top: float, cell: float, r: float, centre_angle: float) -> list:
    """Block letters on the label: each row's run of lit cells is one thin box on the bottle's surface."""
    out, n = [], len(text) * 6 - 1
    for i, ch in enumerate(text):
        for row, bits in enumerate(FONT[ch]):
            k = 0
            while k < 5:
                if bits[k] == "1":
                    j = k
                    while j < 5 and bits[j] == "1":
                        j += 1
                    arc = (i * 6 + (k + j) / 2 - n / 2) * cell  # arc length from the word's middle
                    a = centre_angle + arc / r
                    out.append(BoxShape((r * math.cos(a), r * math.sin(a), z_top - (row + 0.5) * cell),
                                        (0.0006, (j - k) * cell, cell), INK_C,
                                        quat_xyzw=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2)), collide=False))
                    k = j
                else:
                    k += 1
    return out


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _cyl(r, z0, z1, color, xy=(0.0, 0.0), collide: bool = True) -> CylShape:
    return CylShape((xy[0], xy[1], (z0 + z1) / 2), r, z1 - z0, color, axis="Z", collide=collide)


# -- shapes -------------------------------------------------------------------------------------------------------------
def _bottle_shapes() -> list:
    s = [_cyl(L.BOTTLE_R, 0.0, L.BODY_TOP, GLASS_C)]
    z0 = L.BODY_TOP
    for z1, r in L.SHOULDER:
        s.append(_cyl(r, z0, z1, GLASS_C))
        z0 = z1
    bore_z = L.BOTTLE_H - L.BORE_DEPTH  # the neck is solid glass below the bore and a tube round it
    s += [_cyl(L.NECK_R, z0, bore_z, GLASS_C)]
    s += _tube(L.NECK_R, L.BORE_R, bore_z, L.LIP_Z[0], GLASS_C) + _tube(L.LIP_R, L.BORE_R, *L.LIP_Z, GLASS_C)
    # the paper label round the body, and a dark band printed on it (no collision)
    s += [_cyl(L.BOTTLE_R + 0.0005, 0.040, 0.105, LABEL_C, collide=False),
          _cyl(L.BOTTLE_R + 0.0008, 0.098, 0.102, GOLD_C, collide=False),
          _cyl(0.0163, 0.165, 0.226, GOLD_C, collide=False)]  # the gold foil collar left on the neck
    s += _word("BRUT", 0.0625, 0.0022, L.BOTTLE_R + 0.0009, math.pi)
    return s


def _tube(r_out, r_in, z0, z1, color, n: int = 12) -> list:
    """A glass tube from z0 to z1 (bottle frame): n slabs whose inner faces are r_in from the axis."""
    out = []
    wall = r_out - r_in
    for k in range(n):
        a = 2 * math.pi * k / n
        c = ((r_in + wall / 2) * math.cos(a), (r_in + wall / 2) * math.sin(a), (z0 + z1) / 2)
        w = 2 * r_out * math.tan(math.pi / n) + 0.0005
        out.append(BoxShape(c, (wall, w, z1 - z0), color, quat_xyzw=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2))))
    return out


def _stopper_shapes() -> list:
    # the ribbed shank's collider is 0.25 mm inside the bore; its grip on the bore is the fit below (FIT)
    return [_cyl(L.BORE_R - 0.00025, -L.SHANK_LEN, 0.0, STOPPER_C), _cyl(L.FLANGE_R, 0.0, L.FLANGE_T, STOPPER_C),
            _cyl(L.KNOB_R, L.FLANGE_T, L.FLANGE_T + L.KNOB_H, STOPPER_C),
            SphereShape((0.0, 0.0, L.FLANGE_T + L.KNOB_H - L.KNOB_R + 0.0015), L.KNOB_R, STOPPER_C, collide=False)]  # domed


def _ring(r_out, wall, h, color, floor: float, n: int = 16) -> list:
    """A round open-topped vessel: a floor disc and n wall slabs round it (origin at the middle of its base)."""
    s = [_cyl(r_out, 0.0, floor, color)]
    for k in range(n):
        a = 2 * math.pi * k / n
        c = ((r_out - wall / 2) * math.cos(a), (r_out - wall / 2) * math.sin(a), h / 2)
        w = 2 * r_out * math.tan(math.pi / n) + 0.001
        s.append(BoxShape(c, (wall, w, h), color, quat_xyzw=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2))))
    return s


def _flute_shapes() -> list:  # the child's tumbler
    return [_cyl(L.FLUTE_BASE_R, 0.0, 0.008, FLUTE_C)] + _ring(L.FLUTE_BOWL_R, 0.003, L.FLUTE_H, FLUTE_C, 0.008, n=12)


def _plate_shapes() -> list:
    return [_cyl(L.PLATE_R + 0.02, 0.0, 0.003, MAT_C), _cyl(L.PLATE_R, 0.003, L.PLATE_H, PLATE_C)]


def _guest_shapes() -> list:
    """A seated child on a chair at the table's left side, facing the table (-y) (frame: under the seat's middle, on
    the floor). Sizes: layout.py."""
    seat, hw, hd = L.SEAT_Z, L.SEAT_W / 2, L.SEAT_D / 2
    s = [_box((-hw, -hd, seat - 0.04), (hw, hd, seat), CHAIR_C),
         _box((-hw, hd, seat), (hw, hd + 0.04, L.CHAIR_H), CHAIR_C)]
    for sx in (-hw + 0.02, hw - 0.02):
        for sy in (-hd + 0.02, hd - 0.02):
            s.append(_box((sx - 0.02, sy - 0.02, 0.0), (sx + 0.02, sy + 0.02, seat - 0.04), CHAIR_C))
    hy = L.HEAD_XY[1] - L.GUEST_XY[1]  # the head's offset toward the table
    sh = L.SHOULDER_Z
    s += [_box((-0.12, -0.12, seat), (0.12, 0.08, seat + 0.10), SHIRT_C),  # hips and thighs
          _box((-0.13, hy - 0.06, seat + 0.08), (0.13, hy + 0.08, sh), SHIRT_C),  # torso, shoulders 26 cm
          _box((0.13, hy - 0.05, sh - 0.22), (0.18, hy + 0.04, sh), SHIRT_C),  # upper arms
          _box((-0.18, hy - 0.05, sh - 0.22), (-0.13, hy + 0.04, sh), SHIRT_C),
          _box((0.10, -0.33, TOP + 0.001), (0.16, hy - 0.06, TOP + 0.045), SKIN_C),  # forearms on the table
          _box((-0.16, -0.33, TOP + 0.001), (-0.10, hy - 0.06, TOP + 0.045), SKIN_C),
          _cyl(0.035, sh - 0.01, TOP + L.HEAD_Z - 0.06, SKIN_C, xy=(0.0, hy)),  # neck
          SphereShape((0.0, hy, TOP + L.HEAD_Z), L.HEAD_R, SKIN_C),
          SphereShape((0.0, hy + 0.012, TOP + L.HEAD_Z + 0.014), L.HEAD_R, HAIR_C, collide=False)]  # hair
    return s


def _trim_shapes() -> list:
    """The foil capsule (gold, on its side) and the wire hood (ring, four legs, cap) taken off the bottle (estimates)."""
    s = [CylShape((0.0, 0.0, 0.0165), 0.0165, 0.060, GOLD_C, axis="X"),
         _cyl(0.0145, 0.0, 0.002, WIRE_C, xy=(0.0, 0.065), collide=False),
         _cyl(0.008, 0.024, 0.027, WIRE_C, xy=(0.0, 0.065), collide=False)]
    for k in range(4):
        a = k * math.pi / 2
        s.append(bodykit.segment((0.0145 * math.cos(a), 0.065 + 0.0145 * math.sin(a), 0.001),
                                 (0.007 * math.cos(a), 0.065 + 0.007 * math.sin(a), 0.025), 0.0012, WIRE_C,
                                 collide=False))
    return s


def _lamp_shapes() -> list:
    """A drum pendant shade (rim LAMP_BOTTOM above the table) on a rod to the ceiling, a bulb inside."""
    z0, z1 = TOP + L.LAMP_BOTTOM, TOP + L.LAMP_BOTTOM + L.LAMP_TALL
    return [_cyl(L.LAMP_R, z0, z1, LAMP_C), _cyl(0.006, z1, 2.4, ROD_C),
            _cyl(L.LAMP_R + 0.001, z0 - L.BAND, z0 + 0.01, RED_C, collide=False),  # red 2 cm band below the rim
            _cyl(0.006 + L.BAND, z1, z1 + 0.035, RED_C, collide=False),  # red foam sleeve, 2 cm thick, on the rod
            _cyl(0.03, z0 - 0.001, z0 + 0.04, (1.0, 0.98, 0.85), collide=False)]


def _wall_shapes() -> list:
    return [_box((-0.25, -0.02, 0.0), (1.10, 0.0, 1.9), WALL_C)]


# -- the stopper's fit --------------------------------------------------------------------------------------------------
FIT = "stopper_fit"
FIT_K = 2.0e4  # N/m: the friction hold's stiffness (stiff: the stopper does not creep under a pull below the cap)
FIT_DAMP = 20.0  # N s/m
PULL_N = float(os.environ.get("CORK_PULL_N", L.PULL_N))  # (a development override for probes; the task uses L.PULL_N)


def _build() -> list:
    cx, cy = L.COASTER_XY
    floor = L.COASTER_FLOOR
    coaster = Body.build("coaster", pos=(cx, cy, TOP), mass=0.25,
                         shapes=_ring(L.COASTER_R, L.COASTER_WALL, L.COASTER_RIM_H, STEEL_C, floor),
                         color=STEEL_C, friction=(0.6, 0.5), solver_iterations=16)
    bottle = Body.build("bottle", pos=(cx, cy, TOP + floor), mass=L.BOTTLE_MASS, shapes=_bottle_shapes(),
                        color=GLASS_C, friction=L.BOTTLE_FRICTION, solver_iterations=32, com=L.BOTTLE_COM,
                        contact_offset=0.002, damping=(0.02, 0.3))
    stopper = Body.build("stopper", pos=(cx, cy, TOP + floor + L.BOTTLE_H), mass=L.STOPPER_MASS,
                         shapes=_stopper_shapes(), color=STOPPER_C, friction=(0.9, 0.8), solver_iterations=32,
                         contact_offset=0.001)
    bodykit.thread(FIT, "bottle", "stopper", anchor0=(0.0, 0.0, L.BOTTLE_H), axis0=(0.0, 0.0, 1.0),
                   anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 0.0, 1.0), slide_limits=(0.0, L.EXIT_AT + 0.003),
                   turn_damping=L.TWIST_DAMP, slide_stiffness=FIT_K, slide_damping=FIT_DAMP,
                   slide_max_force=PULL_N)
    dish = Body.build("dish", pos=(L.DISH_XY[0], L.DISH_XY[1], TOP), mass=0.30,
                      shapes=_ring(L.DISH_R, 0.006, L.DISH_H, DISH_C, L.DISH_FLOOR), color=DISH_C,
                      friction=(0.6, 0.5), solver_iterations=16)
    flute = Body.build("flute", pos=(L.FLUTE_XY[0], L.FLUTE_XY[1], TOP), mass=0.17, shapes=_flute_shapes(),
                       color=FLUTE_C, friction=(0.6, 0.5), solver_iterations=16, com=(0.0, 0.0, 0.06))
    plate = Body.build("plate", pos=(L.PLATE_XY[0], L.PLATE_XY[1], TOP), mass=0.6, shapes=_plate_shapes(),
                       color=PLATE_C, kinematic=True)
    guest = Body.build("guest", pos=(L.GUEST_XY[0], L.GUEST_XY[1], 0.0), mass=20.0, shapes=_guest_shapes(),
                       color=SHIRT_C, kinematic=True)
    pendant = Body.build("pendant", pos=(L.LAMP_XY[0], L.LAMP_XY[1], 0.0), mass=1.0, shapes=_lamp_shapes(),
                         color=LAMP_C, kinematic=True)
    trim = Body.build("trim", pos=(L.TRIM_XY[0], L.TRIM_XY[1], TOP), mass=0.01, shapes=_trim_shapes(),
                      color=GOLD_C, kinematic=True)
    wall = Body.build("wall", pos=(0.0, L.WALL_Y, 0.0), mass=10.0, shapes=_wall_shapes(), color=WALL_C,
                      kinematic=True)
    return [coaster, bottle, stopper, dish, flute, plate, guest, pendant, trim, wall]


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
SPEC = {o.name: o for o in OBJECTS}
GUEST_HEAD = np.array([L.HEAD_XY[0], L.HEAD_XY[1], TOP + L.HEAD_Z])
LAMP_MID = np.array([L.LAMP_XY[0], L.LAMP_XY[1], TOP + L.LAMP_BOTTOM + L.LAMP_TALL / 2])


# -- reading the scene ------------------------------------------------------------------------------------------------
def axis(st) -> np.ndarray:
    return st["bottle"].rotation()[:, 2].copy()


def lip(st) -> np.ndarray:
    return np.asarray(st["bottle"].pos, float) + axis(st) * L.BOTTLE_H


def travel(st) -> float:
    """How far the stopper has come out of the neck (m, along the bottle's axis; 0 seated)."""
    return float(st["bottle"].to_local(st["stopper"].pos)[2]) - L.BOTTLE_H


def tilt_deg(st, name: str) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(st[name].rotation()[2, 2])))))


def _ang(a, b) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = float(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _dheading(h1: float, h2: float) -> float:
    return abs((h1 - h2 + 180.0) % 360.0 - 180.0)


def aim(st) -> dict:
    """Where the bottle's axis points: tilt from vertical, heading seen from above, the angle to the lamp's shade, and
    the heading differences to the guest's plate and back to the robot (the midpoint of its arm bases)."""
    ax, lp = axis(st), lip(st)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ax[2])))))
    head = L.heading_deg(ax[0], ax[1])
    to_guest = L.heading_deg(L.HEAD_XY[0] - lp[0], L.HEAD_XY[1] - lp[1])
    to_self = L.heading_deg(-lp[0], -lp[1])
    return dict(tilt=tilt, heading=head, lamp_deg=_ang(ax, LAMP_MID - lp), guest_deg=_dheading(head, to_guest),
                self_deg=_dheading(head, to_self), head_deg=_ang(ax, GUEST_HEAD - lp))


def aim_safe(a: dict) -> bool:
    return (L.TILT_RANGE[0] <= a["tilt"] <= L.TILT_RANGE[1] and a["lamp_deg"] >= L.LAMP_CONE
            and a["guest_deg"] >= L.GUEST_CONE and a["self_deg"] >= L.SELF_CONE)


def _grippers(base) -> list[tuple[np.ndarray, np.ndarray, float]]:
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(r)) and math.isfinite(half)):
            raise ValueError("a gripper pose is not finite")
        out.append((p, r, half))
    return out


def finger_points(p, r, half: float) -> list[np.ndarray]:
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


KNOB_MID = np.array([0.0, 0.0, L.FLANGE_T + L.KNOB_H / 2])  # stopper frame


def holding(st, grips) -> bool:
    """Is a gripper closed on the stopper's knob: the knob's middle between its two pads (within the pads' length and
    width) and the fingers closed to within 3 mm of the knob's width?"""
    c = st["stopper"].rotation() @ KNOB_MID + np.asarray(st["stopper"].pos, float)
    for p, r, half in grips:
        loc = np.asarray(r, float).T @ (c - np.asarray(p, float))
        if (abs(loc[0]) <= half + 0.003 and abs(loc[1]) <= 0.012 and -0.035 <= loc[2] <= 0.013
                and half <= L.KNOB_R + 0.003):
            return True
    return False


def _shape_boxes(name: str) -> list[tuple[np.ndarray, np.ndarray]]:
    return [(np.asarray(sh.centre, float), sh.half_extent()) for sh in SPEC[name].shapes if getattr(sh, "collide", True)]


_SHAPES = {n: _shape_boxes(n) for n in ("bottle", "stopper", "pendant")}


def clearance(st, points, name: str) -> float:
    best = float("inf")
    for p in points:
        loc = st[name].to_local(p)
        for c, h in _SHAPES[name]:
            d = np.maximum(0.0, np.abs(loc - c) - h)
            best = min(best, float(np.linalg.norm(d)))
    return best


def in_dish(st) -> bool:
    loc = st["dish"].to_local(st["stopper"].pos)
    r = float(np.hypot(loc[0], loc[1]))
    return r <= L.DISH_R - 0.006 - 0.008 and -0.005 <= loc[2] - L.DISH_FLOOR <= 0.030


def on_coaster(st) -> bool:
    loc = st["coaster"].to_local(st["bottle"].pos)
    r = float(np.hypot(loc[0], loc[1]))
    return r <= L.COASTER_R - L.COASTER_WALL - L.BOTTLE_R + 0.002 and -0.004 <= loc[2] - L.COASTER_FLOOR <= 0.008


# -- episode state, the score -----------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["CORK_DEBUG"], "w") if os.environ.get("CORK_DEBUG") else None
CLEAR_STEPS = 20  # 1 s
GUESTS = ("flute",)


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, st=None, exit=None, hist_b=[], hist_s=[], start=None, fit_on=True, report={}, hand_err=False,
                anchor=0.0, lamp=False, free=0, flew=False)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def _set_fit(st) -> None:
    """The friction hold (static and sliding friction): the slide's spring is anchored where the stopper last slipped
    to. It holds the stopper there with up to F = PULL_N x the share of the shank still inside (at least 15 %); once a
    pull stretches it past F / FIT_K (0.6 mm at 12 N), the anchor follows, so the spring never pulls back harder than
    F and the stopper slides against F. The drive's force cap is set to F as well."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    tr = max(0.0, travel(st))
    cap = PULL_N * max(0.15, max(0.0, L.SHANK_LEN - tr) / L.SHANK_LEN)
    _EP["anchor"] = max(_EP["anchor"], tr - cap / FIT_K)
    bodykit.set_slide_target(FIT, _EP["anchor"])
    UsdPhysics.DriveAPI(get_current_stage().GetPrimAtPath(bodykit._THREAD_PATHS[FIT]), "transX").GetMaxForceAttr().Set(
        float(cap))


def evaluate(st, ex, hist_b, hist_s, start, lamp: bool = False) -> dict:
    moved = {n: (float(np.linalg.norm(np.asarray(st[n].pos) - start[n])) if start else 0.0) for n in GUESTS}
    guest_ok = moved["flute"] <= L.GUEST_MOVE and tilt_deg(st, "flute") <= 10.0
    on_table = float(st["bottle"].pos[2]) >= TOP - 0.02
    clear_b = len(hist_b) >= CLEAR_STEPS and min(hist_b[-CLEAR_STEPS:]) >= L.CLEAR
    clear_s = len(hist_s) >= CLEAR_STEPS and min(hist_s[-CLEAR_STEPS:]) >= L.CLEAR
    tilt = tilt_deg(st, "bottle")
    dish = in_dish(st) and clear_s
    coaster = on_coaster(st) and tilt <= L.UPRIGHT_DEG and clear_b
    out = ex is not None
    safe = out and ex["held"] and ex["safe"]
    score = 0.0
    if safe and guest_ok and on_table and not lamp:
        score = 0.4 + (0.3 if dish else 0.0) + (0.3 if coaster else 0.0)
    return dict(score=round(score, 3), stopper_out=out, exit=ex, lamp_touched=lamp, travel_mm=round(travel(st) * 1000, 1),
                stopper_in_dish=in_dish(st), stopper_released=clear_s, bottle_on_coaster=on_coaster(st),
                bottle_tilt=round(tilt, 1), bottle_released=clear_b, flute_moved_mm=round(moved["flute"] * 1000, 1),
                flute_tilt=round(tilt_deg(st, "flute"), 1), bottle_on_table=on_table,
                min_clear_mm=[round(min(h[-CLEAR_STEPS:]) * 1000, 1) if h else None for h in (hist_b, hist_s)])


def _launch(env, st) -> None:
    """The disclosed gas rule (layout.py): the fit is switched off and the stopper is set moving out along the neck's
    axis at FLY_SPEED (the bottle's own velocity is left out: a hand holds it nearly still)."""
    import torch

    bodykit.set_thread_on(FIT, False)
    _EP["fit_on"] = False
    _EP["flew"] = True
    obj = env.unwrapped.scene["stopper"]
    vel = torch.zeros_like(obj.data.default_root_vel.torch)
    vel[:, :3] = torch.tensor(axis(st) * L.FLY_SPEED, dtype=vel.dtype, device=vel.device)
    obj.write_root_velocity_to_sim_index(root_velocity=vel)
    if _DBG is not None:
        _DBG.write(f"{_EP['step']:4d} FLY: the fit gave with no hand on the stopper; it leaves at {L.FLY_SPEED} m/s\n")


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    try:
        grips = _grippers(env.unwrapped)
        pts = [q for p, r, h in grips for q in finger_points(p, r, h)]
        cb, cs = clearance(st, pts, "bottle"), clearance(st, pts, "stopper")
        near_lamp = clearance(st, pts, "pendant") < L.LAMP_TOUCH
        held = holding(st, grips)
    except Exception as err:  # noqa: BLE001  (fails closed: not released, and not holding the stopper)
        cb = cs = 0.0
        near_lamp = True
        held = False
        if _DBG is not None and not _EP["hand_err"]:
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    tr = travel(st)
    _EP["free"] = 0 if held else _EP.get("free", 0) + 1
    if _EP["exit"] is None and tr >= L.EXIT_AT - 0.0005:
        a = aim(st)
        _EP["exit"] = dict(step=_EP["step"], held=held, safe=aim_safe(a), flew=_EP.get("flew", False),
                           **{k: round(v, 1) for k, v in a.items()}, lip=[round(float(v), 4) for v in lip(st)])
        if _DBG is not None:
            _DBG.write(f"{_EP['step']:4d} EXIT {_EP['exit']}\n")
    if _EP["fit_on"]:
        if tr >= L.EXIT_AT - 0.0005:
            bodykit.set_thread_on(FIT, False)  # out of the neck: a loose body from now on
            _EP["fit_on"] = False
        elif _EP["anchor"] >= L.GIVE_AT and _EP["free"] >= L.FREE_STEPS:
            _launch(env, st)  # the gas: the fit has given and nothing holds the stopper
        else:
            _set_fit(st)
    for key, v in (("hist_b", cb), ("hist_s", cs)):
        _EP[key].append(v)
        if len(_EP[key]) > CLEAR_STEPS:
            _EP[key].pop(0)
    _EP["held"] = held
    if near_lamp and not _EP["lamp"]:
        _EP["lamp"] = True
        if _DBG is not None:
            _DBG.write(f"{_EP['step']:4d} LAMP touched (a finger within 20 mm of the lamp)\n")
    _EP["step"] += 1
    if _DBG is not None and _EP["step"] % 2 == 0:
        _debug(st)


def report(objects=None) -> dict:
    st = _EP.get("st")
    if st is None:
        return dict(score=0.0)
    return evaluate(st, _EP["exit"], _EP["hist_b"], _EP["hist_s"], _EP.get("start"), _EP["lamp"])


def score(objects) -> float:
    """Read at the end of the trial (see task.json "scoring"). 0 unless the stopper has come out of the neck and, at the
    moment its shank's end passed the lip, a gripper was closed on it and the bottle's axis was tilted 30 to 60 degrees
    from vertical, at least 25 degrees off the line to the lamp's shade, and headed at least 45 degrees off the line to
    the guest's plate and off the line back to the robot; also 0 if the flute moved over 20 mm or tilted over 10
    degrees, the bottle left the table, or a finger came within 20 mm of the lamp at any time. Then 0.4, plus 0.3 with the stopper resting in the dish and no finger within
    50 mm of it for the last 1 s, plus 0.3 with the bottle standing in the coaster (tilt at most 10 degrees) and no
    finger within 50 mm of it for the last 1 s."""
    rep = report(objects)
    _EP["report"] = rep
    if _DBG is not None:
        _DBG.write(f"score report: {rep}\n")
        _DBG.flush()
    return float(rep["score"])


def distance(objects) -> float:
    st = _EP.get("st")
    if st is None:
        return 1.0
    return float(np.linalg.norm(np.asarray(st["stopper"].pos, float)[:2] - np.asarray(L.DISH_XY)))


def _debug(st) -> None:
    b, s = st["bottle"].pos, st["stopper"].pos
    a = aim(st)
    _DBG.write(f"{_EP['step']:4d} tr={travel(st) * 1000:6.2f}mm fit={int(_EP['fit_on'])} held={int(_EP['held'])} "
               f"spring={FIT_K * max(0.0, travel(st) - _EP['anchor']) if _EP['fit_on'] else 0.0:5.1f}N "
               f"tilt={a['tilt']:5.1f} head={a['heading']:6.1f} lamp={a['lamp_deg']:5.1f} guest={a['guest_deg']:5.1f} "
               f"self={a['self_deg']:5.1f} b=({b[0]:.4f},{b[1]:.4f},{b[2] - TOP:.4f}) "
               f"s=({s[0]:.4f},{s[1]:.4f},{s[2] - TOP:.4f}) cb={_EP['hist_b'][-1]:.3f} cs={_EP['hist_s'][-1]:.3f} "
               f"flute=({st['flute'].pos[0]:.4f},{st['flute'].pos[1]:.4f}) coaster=({st['coaster'].pos[0]:.4f},"
               f"{st['coaster'].pos[1]:.4f})\n")
    _DBG.flush()


# -- the fixed start --------------------------------------------------------------------------------------------------
def poses() -> dict:
    cx, cy = L.COASTER_XY
    ident = (0.0, 0.0, 0.0, 1.0)
    e = 0.0005
    return {
        "coaster": (np.array([cx, cy, TOP + e]), ident),
        "bottle": (np.array([cx, cy, TOP + L.COASTER_FLOOR + e]), ident),
        "stopper": (np.array([cx, cy, TOP + L.COASTER_FLOOR + e + L.BOTTLE_H]), ident),
        "dish": (np.array([L.DISH_XY[0], L.DISH_XY[1], TOP + e]), ident),
        "flute": (np.array([L.FLUTE_XY[0], L.FLUTE_XY[1], TOP + e]), ident),
        "plate": (np.array([L.PLATE_XY[0], L.PLATE_XY[1], TOP]), ident),
        "guest": (np.array([L.GUEST_XY[0], L.GUEST_XY[1], 0.0]), ident),
        "pendant": (np.array([L.LAMP_XY[0], L.LAMP_XY[1], 0.0]), ident),
        "trim": (np.array([L.TRIM_XY[0], L.TRIM_XY[1], TOP]), ident),
        "wall": (np.array([0.0, L.WALL_Y, 0.0]), ident),
    }


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put every body at its one fixed place (no seed changes it), seat the stopper, let them settle."""
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    _blank()
    for name, (pos, quat) in poses().items():
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, float), dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    bodykit.set_slide_target(FIT, 0.0)
    bodykit.set_thread_on(FIT, True)  # the stopper is pushed into the neck
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    st = _scene.object_states(emb._env, NAMES)
    _EP["st"] = st
    _EP["start"] = {n: np.asarray(st[n].pos, float).copy() for n in GUESTS}
    if _DBG is not None:
        _DBG.write(f"seed {seed}: fixed layout, stopper travel {travel(st) * 1000:.2f} mm\n")
    return emb._observation()


# -- the scripted demo (reads object poses from the simulator to plan; moves the parts only through the grippers) ------
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "" (full), letgo, flyaway, nodish, nothing, upright, guest (README)
AIM_HEADING = float(os.environ.get("CORK_HEADING", "-95"))  # degrees: the bottle tilts toward the empty right side
TILT = math.radians(45.0)  # the AAO angle
APPROACH = math.radians(float(os.environ.get("CORK_APPROACH", "-10")))  # the bottle hand's heading
GRIP_Z = 0.090  # the body grip, 90 mm above the bottle's base (its centre of mass is at 85 mm)
LIFT = 0.035  # lift the base 35 mm: it clears the coaster's 25 mm rim while tilted
BODY_OPEN, BODY_SQUEEZE = -0.046, -(L.BOTTLE_R - 0.006)
KNOB_OPEN, KNOB_SQUEEZE = -0.022, -(L.KNOB_R - 0.006)
PULL = 0.035  # the stopper hand draws back 35 mm along the axis: the 25 mm modelled stem is out with 10 mm to spare
TWIST = math.radians(float(os.environ.get("CORK_TWIST", "30")))
LETGO = float(os.environ.get("CORK_LETGO_MM", "8")) / 1000.0  # the letgo variant's hand travel before letting go


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def rest(arm: str, seconds: float) -> Waypoint:
    p, r = ready_pose(arm)
    side = 1.0 if arm == "left" else -1.0
    return Waypoint(seconds, tuple((p + np.array([-0.12, 0.04 * side, 0.0])).tolist()), FINGER_OPEN,
                    "back to the ready pose", r)


def demo():
    """Both arms wait at the ready pose; the real plan is made on the first control step (corkdemo.py)."""
    return {arm: [park(arm, 0.5, label="look")] for arm in ("left", "right")}


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) / np.linalg.norm(close)
    x = x - z * float(x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _rot(ax, a: float) -> np.ndarray:
    ax = np.asarray(ax, float) / np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def _wp(sec, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(np.asarray(pos, float).tolist()), float(finger), label, np.asarray(rot, float))


def plan_from_state(st) -> dict[str, list[Waypoint]]:
    """The left hand lifts the bottle by its body and tilts it 45 degrees toward the empty side; the right hand closes on
    the stopper from above along the axis, twists and draws it out, still closed on it, and puts it in the dish; the
    left hand stands the bottle back in the coaster."""
    if VARIANT == "nothing":
        return {arm: [park(arm, 30.0, label="wait")] for arm in ("left", "right")}
    if VARIANT in ("upright", "guest"):
        return _careless_plan(st)
    b = np.asarray(st["bottle"].pos, float)
    tilt = TILT
    h = math.radians(AIM_HEADING)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])  # the tilt axis: the top goes toward h
    d = np.array([math.cos(APPROACH), math.sin(APPROACH), 0.0])
    r0 = _frame(d, (math.sin(APPROACH), -math.cos(APPROACH), 0.0))
    g0 = b + np.array([0.0, 0.0, GRIP_Z]) - 0.004 * d
    g1 = g0 + np.array([0.0, 0.0, LIFT])
    rt = _rot(u, tilt) @ r0
    ax = _rot(u, tilt) @ np.array([0.0, 0.0, 1.0])
    lip_p = (g1 + 0.004 * d) + ax * (L.BOTTLE_H - GRIP_Z)  # the axis passes 4 mm beyond the TCP along the approach
    up3 = np.array([0.0, 0.0, 0.03])
    left = [_wp(3.0, g0 - 0.10 * d + up3, BODY_OPEN, "beside the bottle", r0),
            _wp(1.0, g0 - 0.05 * d + up3, BODY_OPEN, "beside the bottle", r0),
            _wp(1.5, g0, BODY_OPEN, "fingers either side of the bottle", r0),
            _wp(0.8, g0, BODY_SQUEEZE, "hold the bottle", r0),
            _wp(1.5, g1, BODY_SQUEEZE, "lift the bottle", r0)]
    for k in (1, 2, 3):
        left.append(_wp(1.0, g1, BODY_SQUEEZE, "tilt the bottle", _rot(u, tilt * k / 3) @ r0))
    left.append(_wp(0.5, g1, BODY_SQUEEZE, "hold it tilted", rt))
    t_tilted = sum(w.duration for w in left)
    # the stopper hand: fingers along the axis, closing across it
    c = _rot(ax, 0.0) @ u
    r1 = _frame(-ax, c)
    knob = lip_p + ax * 0.018  # the fingertips (13 mm past the TCP) end 5 mm above the lip, on the knob
    right = [park("right", max(0.5, t_tilted - 3.0), label="wait"),
             _wp(2.5, knob + 0.08 * ax, KNOB_OPEN, "above the stopper", r1),
             _wp(1.5, knob, KNOB_OPEN, "fingers either side of the stopper", r1),
             _wp(0.8, knob, KNOB_SQUEEZE, "hold the stopper", r1)]
    if VARIANT in ("letgo", "flyaway"):
        # twist it and start to draw it (LETGO_MM of hand travel: the fit is loaded but has not slipped), then let go of
        # it and draw the hand back: the stopper is left in the neck
        right += [_wp(1.0, knob + 0.004 * ax, KNOB_SQUEEZE, "twist the stopper", _rot(ax, TWIST) @ r1),
                  _wp(2.5, knob + LETGO * ax, KNOB_SQUEEZE, "draw the stopper", _rot(ax, TWIST) @ r1),
                  _wp(0.8, knob + LETGO * ax, KNOB_OPEN, "let go of the stopper", _rot(ax, TWIST) @ r1)
                  if VARIANT == "letgo" else  # flyaway: the hand opens while it is still drawing, as the fit gives
                  _wp(0.8, knob + (LETGO + 0.008) * ax, KNOB_OPEN, "draw the stopper", _rot(ax, TWIST) @ r1),
                  _wp(1.5, knob + 0.07 * ax, KNOB_OPEN, "back off", _rot(ax, TWIST) @ r1)]
    else:
        right += [_wp(1.0, knob + 0.004 * ax, KNOB_SQUEEZE, "twist the stopper", _rot(ax, TWIST) @ r1),
                  _wp(3.0, knob + PULL * ax, KNOB_SQUEEZE, "draw the stopper", _rot(ax, TWIST) @ r1),
                  _wp(1.0, knob + (PULL + 0.04) * ax, KNOB_SQUEEZE, "draw the stopper", _rot(ax, TWIST) @ r1)]
        rd = _frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
        dish = np.asarray(st["dish"].pos, float)
        if VARIANT == "nodish":
            dish = dish + np.array([0.10, 0.0, 0.0])  # beside the dish, on the table
        drop = 0.048  # TCP height over the dish floor: the hanging stopper's stem end 5 mm above it
        right += [_wp(2.5, dish + np.array([0.0, 0.0, 0.14]), KNOB_SQUEEZE, "carry the stopper", rd),
                  _wp(1.5, dish + np.array([0.0, 0.0, L.DISH_FLOOR + drop]), KNOB_SQUEEZE, "lower the stopper", rd),
                  _wp(0.8, dish + np.array([0.0, 0.0, L.DISH_FLOOR + drop]), KNOB_OPEN, "let go of the stopper", rd),
                  _wp(1.2, dish + np.array([0.0, 0.0, 0.14]), KNOB_OPEN, "back off up", rd)]
    right += [rest("right", 2.5), rest("right", 1.0)]
    t_free = sum(w.duration for w in right[:-2]) - (5.0 if VARIANT not in ("letgo", "flyaway") else 0.0)
    hold = max(0.5, t_free - t_tilted)
    left.append(_wp(hold, g1, BODY_SQUEEZE, "hold it tilted", rt))
    for k in (2, 1, 0):
        left.append(_wp(1.0, g1, BODY_SQUEEZE, "stand the bottle up", _rot(u, tilt * k / 3) @ r0))
    down = g0 + np.array([0.0, 0.0, 0.001])
    left += [_wp(0.8, g1, BODY_SQUEEZE, "stand the bottle up", r0),
             _wp(1.5, down, BODY_SQUEEZE, "set the bottle down", r0),
             _wp(1.0, down, BODY_SQUEEZE, "set down", r0),
             _wp(0.8, down, BODY_OPEN, "let go of the bottle", r0),
             _wp(1.0, down + up3, BODY_OPEN, "back off", r0),
             _wp(1.5, down - 0.10 * d + np.array([0.0, 0.0, 0.08]), BODY_OPEN, "back off up", r0),
             rest("left", 2.5), rest("left", 1.0)]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    print(f"corkdemo: variant '{VARIANT}', bottle ({b[0]:.3f}, {b[1]:.3f}), heading {math.degrees(h):.0f}, "
          f"plan {max(tl, tr):.1f} s", flush=True)
    return {"left": left, "right": right}


def _careless_plan(st) -> dict[str, list[Waypoint]]:
    """Two part-done plans that reach the exit with the aim wrong (README). The stopper hand closes across the knob
    (fingers along u, normal to the tilt plane) with an approach in the tilt plane, BETA degrees off the axis.
    upright: the left hand holds the bottle standing in its coaster; the right hand pulls the stopper straight up and
    puts it in the dish; the left hand lets go. guest: the right hand lifts the bottle, carries it 10 cm nearer the robot
    and tilts it 45 degrees toward the child (heading 35); the left hand pulls the stopper out along the axis and keeps
    it; both hands hold on to the end."""
    b = np.asarray(st["bottle"].pos, float)
    up3 = np.array([0.0, 0.0, 0.03])
    if VARIANT == "upright":
        sarm, barm, h, tilt, beta, carry = "right", "left", -90.0, 0.0, 60.0, np.zeros(3)
    else:
        sarm, barm, h, tilt, beta, carry = "left", "right", 35.0, 45.0, -110.0, np.array([-0.10, -0.02, 0.0])
    h, tilt = math.radians(h), math.radians(tilt)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    d = np.array([math.cos(APPROACH), math.sin(APPROACH), 0.0])
    r0 = _frame(d, (math.sin(APPROACH), -math.cos(APPROACH), 0.0))
    g0 = b + np.array([0.0, 0.0, GRIP_Z]) - 0.004 * d
    g1 = g0 + (np.array([0.0, 0.0, LIFT]) + carry if tilt else 0.0)
    bot = [_wp(3.0, g0 - 0.10 * d + up3, BODY_OPEN, "beside the bottle", r0),
           _wp(1.0, g0 - 0.05 * d + up3, BODY_OPEN, "beside the bottle", r0),
           _wp(1.5, g0, BODY_OPEN, "fingers either side of the bottle", r0),
           _wp(0.8, g0, BODY_SQUEEZE, "hold the bottle", r0)]
    if tilt:
        bot += [_wp(1.5, g0 + np.array([0.0, 0.0, LIFT]), BODY_SQUEEZE, "lift the bottle", r0),
                _wp(2.0, g1, BODY_SQUEEZE, "carry the bottle", r0)]
        for k in (1, 2, 3):
            bot.append(_wp(1.0, g1, BODY_SQUEEZE, "tilt the bottle", _rot(u, tilt * k / 3) @ r0))
    rt = _rot(u, tilt) @ r0
    ax = _rot(u, tilt) @ np.array([0.0, 0.0, 1.0])
    lip_p = (g1 + 0.004 * d) + ax * (L.BOTTLE_H - GRIP_Z)
    t_ready = sum(w.duration for w in bot)
    appr = _rot(u, math.radians(beta)) @ (-ax)
    r1 = _frame(appr, u)
    tcp = lip_p + 0.011 * ax - 0.004 * appr
    sto = [park(sarm, max(0.5, t_ready - 3.0), label="wait"),
           _wp(2.5, tcp - 0.08 * appr, KNOB_OPEN, "beside the stopper", r1),
           _wp(1.5, tcp, KNOB_OPEN, "fingers either side of the stopper", r1),
           _wp(0.8, tcp, KNOB_SQUEEZE, "hold the stopper", r1),
           _wp(3.0, tcp + PULL * ax, KNOB_SQUEEZE, "draw the stopper", r1),
           _wp(1.0, tcp + (PULL + 0.04) * ax, KNOB_SQUEEZE, "draw the stopper", r1)]
    if VARIANT == "upright":
        rd = _frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
        dish = np.asarray(st["dish"].pos, float)
        sto += [_wp(2.5, dish + np.array([0.0, 0.0, 0.14]), KNOB_SQUEEZE, "carry the stopper", rd),
                _wp(1.5, dish + np.array([0.0, 0.0, L.DISH_FLOOR + 0.048]), KNOB_SQUEEZE, "lower the stopper", rd),
                _wp(0.8, dish + np.array([0.0, 0.0, L.DISH_FLOOR + 0.048]), KNOB_OPEN, "let go of the stopper", rd),
                _wp(1.2, dish + np.array([0.0, 0.0, 0.14]), KNOB_OPEN, "back off up", rd),
                rest(sarm, 2.5), rest(sarm, 1.0)]
        t_out = sum(w.duration for w in sto[:6])
        bot += [_wp(max(0.5, t_out - t_ready + 1.0), g0, BODY_SQUEEZE, "hold the bottle", r0),
                _wp(0.8, g0, BODY_OPEN, "let go of the bottle", r0),
                _wp(1.0, g0 + up3, BODY_OPEN, "back off", r0),
                _wp(1.5, g0 - 0.10 * d + np.array([0.0, 0.0, 0.08]), BODY_OPEN, "back off up", r0),
                rest(barm, 2.5), rest(barm, 1.0)]
    else:
        sto.append(_wp(8.0, tcp + (PULL + 0.04) * ax, KNOB_SQUEEZE, "keep the stopper", r1))
        t_out = sum(w.duration for w in sto)
        bot.append(_wp(max(0.5, t_out - t_ready), g1, BODY_SQUEEZE, "hold it tilted", rt))
    tl, tr = sum(w.duration for w in bot), sum(w.duration for w in sto)
    print(f"corkdemo: variant '{VARIANT}', bottle ({b[0]:.3f}, {b[1]:.3f}), plan {max(tl, tr):.1f} s", flush=True)
    return {barm: bot, sarm: sto}


import corkdemo  # noqa: E402  (the demo's control hooks for yam_scripted)

corkdemo.install()
corkdemo_task = True
