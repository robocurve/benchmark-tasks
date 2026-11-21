"""A bathroom spray cleaner, put away where a toddler cannot get at it: nozzle turned to OFF, bottle in the latched
cabinet.

A 16 oz trigger-spray bottle of bathroom cleaner (printed label, hazard mark, "KEEP OUT OF REACH OF CHILDREN") stands on
the bathroom counter with its nozzle on SPRAY. A toddler tower stands against the counter by the open cleaning caddy;
the child's tumbler and toothbrush are on the counter. A counter-top cabinet with a self-latching child-safety latch
stands open beside the bottle; a second bottle of the same cleaner is already inside it. The nozzle is a square cap that
turns about the spray axis and clicks into four positions a quarter turn apart: OFF, SPRAY, OFF, STREAM.

Mechanisms: the nozzle is its own body on a hinge about the spray axis. A detent spring (a joint drive re-aimed every
control step at the nearest click, with a stiffness set so its torque follows the detent's profile) pulls it into the
nearest click with at most DETENT_TORQUE; between clicks it is free. The cabinet door is its own body on a vertical
hinge with a stop at 0 (shut) and 110 degrees, and a little drag. When the door comes within LATCH_DEG of shut, the
latch's hook catches: from then on a stiff spring holds the door shut. Nothing moves the nozzle or the door except
contact.

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
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
CONTROL_HZ = 20.0

WHITE, BLACK, CADDY_C = (0.93, 0.93, 0.91), (0.08, 0.08, 0.09), (0.16, 0.16, 0.17)
OFF_C, SPRAY_C, STREAM_C = (0.85, 0.10, 0.08), (0.95, 0.95, 0.95), (0.15, 0.35, 0.90)
LABEL_C, INK_C, HAZ_C = (0.98, 0.85, 0.20), (0.05, 0.05, 0.05), (0.85, 0.08, 0.06)
CAB_C, DOOR_C, LATCH_C = (0.90, 0.90, 0.88), (0.86, 0.86, 0.84), (0.55, 0.55, 0.58)
CUP_C, BRUSH_C, WOOD_C = (0.98, 0.62, 0.15), (0.30, 0.75, 0.35), (0.78, 0.62, 0.42)


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _cyl(r, z0, z1, color, xy=(0.0, 0.0), collide: bool = True) -> CylShape:
    return CylShape((xy[0], xy[1], (z0 + z1) / 2), r, z1 - z0, color, axis="Z", collide=collide)


def _qz(a: float) -> tuple[float, float, float, float]:
    """xyzw quaternion of a turn ``a`` (rad) about z."""
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


NOZ_C = (L.NOZ_FRONT_X + L.NOZ_LEN / 2, 0.0, L.NOZ_AXIS_Z)  # the nozzle body's origin, on the spray axis (bottle frame)


def _trigger_box():
    """The trigger as an oriented box in the bottle frame: centre, axes (along, through, across) and half sizes."""
    p1, p2, width, thick = (np.asarray(L.TRIGGER[0], float), np.asarray(L.TRIGGER[1], float), L.TRIGGER[2],
                            L.TRIGGER[3])
    d = (p2 - p1) / np.linalg.norm(p2 - p1)
    n = np.array([-d[2], 0.0, d[0]])  # in the x-z plane, perpendicular to the trigger's length
    if n[0] < 0:
        n = -n  # thickness goes back (+x) from the front face
    c = (p1 + p2) / 2 + n * thick / 2
    ax = np.column_stack([d, n, [0.0, 1.0, 0.0]])
    ax[:, 2] *= np.sign(np.linalg.det(ax))  # right-handed
    return c, ax, np.array([np.linalg.norm(p2 - p1) / 2, thick / 2, width / 2])


# -- the printed label (a 3 x 5 pixel font, each run of pixels a thin box on the bottle's round side) ------------------
FONT = {
    "A": ("010", "101", "111", "101", "101"), "B": ("110", "101", "110", "101", "110"),
    "C": ("011", "100", "100", "100", "011"), "D": ("110", "101", "101", "101", "110"),
    "E": ("111", "100", "110", "100", "111"), "F": ("111", "100", "110", "100", "100"),
    "H": ("101", "101", "111", "101", "101"), "I": ("111", "010", "010", "010", "111"),
    "K": ("101", "110", "100", "110", "101"), "L": ("100", "100", "100", "100", "111"),
    "M": ("101", "111", "111", "101", "101"), "N": ("111", "101", "101", "101", "101"),
    "O": ("111", "101", "101", "101", "111"), "P": ("110", "101", "110", "100", "100"),
    "R": ("110", "101", "110", "101", "101"), "T": ("111", "010", "010", "010", "010"),
    "U": ("101", "101", "101", "101", "111"), " ": ("000",) * 5,
}
LABEL_AZ = math.pi  # the label faces the robot (-x), under the nozzle


def _tangent_box(s: float, z: float, w: float, h: float, color, r: float, t: float = 0.0006) -> BoxShape:
    """A thin box on the round side at arc position ``s`` (m, + to the viewer's right) and height ``z`` (its middle)."""
    a = LABEL_AZ + s / r
    c = ((r + t / 2) * math.cos(a), (r + t / 2) * math.sin(a), z)
    return BoxShape(c, (t, w, h), color, collide=False, quat_xyzw=_qz(a))


def _text(line: str, z_top: float, px: float, color, r: float) -> list:
    cols = 4 * len(line) - 1
    s0 = -cols * px / 2
    out = []
    for i, ch in enumerate(line):
        rows = FONT[ch]
        for k, row in enumerate(rows):
            j = 0
            while j < 3:
                if row[j] == "1":
                    e = j
                    while e < 3 and row[e] == "1":
                        e += 1
                    s = s0 + (4 * i + (j + e) / 2) * px
                    out.append(_tangent_box(s, z_top - (k + 0.5) * px, (e - j) * px, px, color, r + 0.0006))
                    j = e
                else:
                    j += 1
    return out


def _label() -> list:
    """BATHROOM / CLEANER, a hazard diamond with "!", and KEEP OUT OF / REACH OF / CHILDREN, on a yellow label."""
    r = L.BOTTLE_R
    out = [_tangent_box(s, 0.075, 0.0071, 0.118, LABEL_C, r) for s in np.arange(-0.0355, 0.0356, 0.0071)]
    out += _text("BATHROOM", 0.129, 0.0020, INK_C, r)
    out += _text("CLEANER", 0.115, 0.0020, INK_C, r)
    # the hazard diamond: a red square turned 45 degrees with a white middle and a black "!"
    a = LABEL_AZ
    for size, color, t in ((0.020, HAZ_C, 0.0012), (0.015, WHITE, 0.0018)):
        c = ((r + t / 2) * math.cos(a), (r + t / 2) * math.sin(a), 0.083)
        q = bodykit._quat_from_matrix(np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0],
                                                [0.0, 0.0, 1.0]]) @ np.array(
            [[1.0, 0.0, 0.0], [0.0, math.cos(math.pi / 4), -math.sin(math.pi / 4)],
             [0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]]))
        out.append(BoxShape(c, (t, size, size), color, collide=False, quat_xyzw=q))
    out.append(_tangent_box(0.0, 0.0855, 0.0022, 0.0060, INK_C, r + 0.0016))
    out.append(_tangent_box(0.0, 0.0782, 0.0022, 0.0020, INK_C, r + 0.0016))
    out += _text("KEEP OUT OF", 0.063, 0.0012, INK_C, r)
    out += _text("REACH OF", 0.054, 0.0012, INK_C, r)
    out += _text("CHILDREN", 0.045, 0.0012, INK_C, r)
    return out


def _bottle_shapes(with_nozzle_marks: str | None = None) -> list:
    """The bottle, its label and the sprayer's fixed parts (bottle frame). ``with_nozzle_marks``: also add the nozzle
    cap with the given word on top (for the second bottle, whose nozzle is not a separate body)."""
    r, dz = L.BOTTLE_R, L.NECK_DZ
    s0, s1, s2 = L.SHOULDER
    s = [_cyl(r, 0.0, s0, WHITE), _cyl(0.028, s0, s1, WHITE), _cyl(0.020, s1, s2, WHITE),
         _cyl(0.0145, s2, L.CLOSURE_Z[0], WHITE), _cyl(L.CLOSURE_R, *L.CLOSURE_Z, BLACK)]
    hw = L.SHROUD_HW
    noz_back = L.NOZ_FRONT_X + L.NOZ_LEN
    s += [_box((-0.012, -0.0125, L.CLOSURE_Z[1]), (0.014, 0.0125, 0.273 + dz), BLACK),  # the pump body
          _box((noz_back, -hw, 0.272 + dz), (0.010, hw, L.SHROUD_TOP), BLACK),  # the shroud, nozzle end
          _box((0.010, -0.014, 0.268 + dz), (0.0357, 0.014, 0.288 + dz), BLACK),  # its tail
          _box((-0.036, -0.006, 0.266 + dz), (-0.012, 0.006, 0.273 + dz), BLACK)]  # the trigger's hinge web
    c, ax, half = _trigger_box()
    s.append(BoxShape(tuple(c.tolist()), tuple((2 * half).tolist()), BLACK, quat_xyzw=bodykit._quat_from_matrix(ax)))
    if with_nozzle_marks:
        s += [BoxShape(NOZ_C, (L.NOZ_LEN, 2 * L.NOZ_HALF, 2 * L.NOZ_HALF), BLACK)]
        s += _marks(np.array(NOZ_C), with_nozzle_marks)
    return s + _label()


def _marks(c, top: str) -> list:
    """Thin printed labels on the nozzle's four side faces (no collision). ``top`` is the word on the top face; the two
    faces a quarter turn away say OFF; the bottom face says the other word."""
    words = {"SPRAY": SPRAY_C, "STREAM": STREAM_C, "OFF": OFF_C}
    if top == "SPRAY":
        up, down, side = "SPRAY", "STREAM", "OFF"
    else:  # OFF on top: SPRAY and STREAM face the sides
        up, down, side = "OFF", "OFF", None
    h, t, ln = L.NOZ_HALF, 0.0006, L.NOZ_LEN * 0.8
    out = [_box(c + (-ln / 2, -h * 0.8, h), c + (ln / 2, h * 0.8, h + t), words[up], collide=False),
           _box(c + (-ln / 2, -h * 0.8, -h - t), c + (ln / 2, h * 0.8, -h), words[down], collide=False)]
    for sy, word in ((1.0, side or "SPRAY"), (-1.0, side or "STREAM")):
        y0, y1 = sorted((sy * h, sy * (h + t)))
        out.append(_box(c + (-ln / 2, y0, -h * 0.8), c + (ln / 2, y1, h * 0.8), words[word], collide=False))
    return out


def _nozzle_shapes() -> list:
    return [BoxShape((0.0, 0.0, 0.0), (L.NOZ_LEN, 2 * L.NOZ_HALF, 2 * L.NOZ_HALF), BLACK),
            _box((-L.NOZ_LEN / 2 - 0.0005, -0.0012, -0.0012), (-L.NOZ_LEN / 2, 0.0012, 0.0012), (0.4, 0.4, 0.4),
                 collide=False)] + _marks(np.zeros(3), "SPRAY")


def _cabinet_shapes() -> list:
    """The EKET carcass (frame: its footprint's middle on the counter): floor, top, two sides, back; open at the front
    (-x), where the door shuts against it. The latch's catch is a small block inside the right side at the front."""
    p, hd, hw, h = L.PANEL, L.CAB_D / 2, L.CAB_W / 2, L.CAB_H
    return [_box((-hd, -hw, 0.0), (hd, hw, p), CAB_C), _box((-hd, -hw, h - p), (hd, hw, h), CAB_C),
            _box((-hd, -hw, p), (hd, -hw + p, h - p), CAB_C), _box((-hd, hw - p, p), (hd, hw, h - p), CAB_C),
            _box((hd - p, -hw + p, p), (hd, hw - p, h - p), CAB_C),
            _box((-hd, -hw + p, 0.20), (-hd + 0.012, -hw + p + 0.012, 0.23), LATCH_C, collide=False)]


def _door_shapes() -> list:
    """The door (frame: on the hinge line at mid height; shut it spans -y from the hinge, its inner face at +x). The
    latch's hook is a small block on its inner face near the free edge."""
    p, w, hh = L.PANEL, L.CAB_W, L.CAB_H / 2
    return [_box((0.0, -w, -hh), (p, 0.0, hh), DOOR_C),
            _box((p, -w + p + 0.004, 0.20 - hh), (p + 0.010, -w + p + 0.024, 0.23 - hh), LATCH_C, collide=False)]


def _caddy_shapes() -> list:
    w, f = L.CADDY_WALL, L.CADDY_FLOOR
    hx, hy, hz = L.CADDY_W / 2, L.CADDY_L / 2, L.COMP_H + f
    end = (L.CADDY_L - L.COMP_L) / 2
    cw = L.CENTER_W / 2
    s = [_box((-hx, -hy, 0.0), (hx, hy, f), CADDY_C)]
    for sx in (-1.0, 1.0):
        x0, x1 = sorted((sx * hx, sx * (hx - w)))
        s.append(_box((x0, -hy, 0.0), (x1, hy, hz), CADDY_C))  # outer long walls
        x0, x1 = sorted((sx * cw, sx * (cw - w)))
        s.append(_box((x0, -hy, 0.0), (x1, hy, hz), CADDY_C))  # the two walls of the middle divider
    for sy in (-1.0, 1.0):
        y0, y1 = sorted((sy * hy, sy * (hy - end)))
        s.append(_box((-hx, y0, 0.0), (hx, y1, hz), CADDY_C))  # end walls (with the hand holds)
        y0, y1 = sorted((sy * 0.150, sy * 0.170))
        s.append(_box((-cw, y0, hz), (cw, y1, L.HANDLE_TOP), CADDY_C))  # handle posts
    s.append(_box((-0.012, -0.170, L.HANDLE_TOP - 0.030), (0.012, 0.170, L.HANDLE_TOP), CADDY_C))  # the handle grip
    return s


def _cup_shapes() -> list:
    return [_cyl(L.CUP_R, 0.0, L.CUP_H, CUP_C), _box((-0.004, -0.003, L.CUP_H - 0.01), (0.004, 0.003, L.CUP_H + 0.075),
                                                     BRUSH_C)]


def _tower_shapes() -> list:
    """The toddler tower (frame: its footprint's middle on the floor): four posts, the platform, rails round the top."""
    hx, hy, h, t = L.TOWER_W / 2, L.TOWER_D / 2, L.TOWER_H, 0.03
    s = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            x0, x1 = sorted((sx * hx, sx * (hx - t)))
            y0, y1 = sorted((sy * hy, sy * (hy - t)))
            s.append(_box((x0, y0, 0.0), (x1, y1, h), WOOD_C))
    s.append(_box((-hx + t, -hy + t, L.TOWER_STEP - 0.018), (hx - t, hy - t, L.TOWER_STEP), WOOD_C))
    for z in (h - 0.03, 0.62):
        s += [_box((-hx, -hy, z), (hx, -hy + t, z + 0.03), WOOD_C), _box((-hx, hy - t, z), (hx, hy, z + 0.03), WOOD_C),
              _box((-hx, -hy, z), (-hx + t, hy, z + 0.03), WOOD_C), _box((hx - t, -hy, z), (hx, hy, z + 0.03), WOOD_C)]
    return s


DOOR_ANCHOR = (L.DOOR_HINGE[0], L.DOOR_HINGE[1], TOP + L.CAB_H / 2)
DOOR_START = _qz(-math.radians(L.DOOR_OPEN_DEG))


def _build() -> list:
    bx, by = L.BOTTLE_XY
    cabinet = Body.build("cabinet", pos=(L.CAB_XY[0], L.CAB_XY[1], TOP), mass=L.CAB_MASS, shapes=_cabinet_shapes(),
                         color=CAB_C, friction=(0.5, 0.4), solver_iterations=16)
    door = Body.build("door", pos=DOOR_ANCHOR, mass=L.DOOR_MASS, shapes=_door_shapes(), color=DOOR_C,
                      friction=(0.6, 0.5), solver_iterations=16, start_xyzw=DOOR_START, ignore=("cabinet",))
    bodykit.hinge("door_turn", "cabinet", "door", anchor=DOOR_ANCHOR, axis="Z", limits_deg=(-L.DOOR_MAX_DEG, 0.0),
                  damping=L.DOOR_DAMP)
    bottle = Body.build("bottle", pos=(bx, by, TOP), mass=L.BOTTLE_MASS, shapes=_bottle_shapes(), color=WHITE,
                        friction=L.BOTTLE_FRICTION, solver_iterations=32, com=L.BOTTLE_COM, contact_offset=0.002)
    noz_w = (bx + NOZ_C[0], by + NOZ_C[1], TOP + NOZ_C[2])
    nozzle = Body.build("nozzle", pos=noz_w, mass=L.NOZ_MASS, inertia=L.NOZ_INERTIA, shapes=_nozzle_shapes(),
                        color=BLACK, friction=(1.0, 0.9), solver_iterations=32, contact_offset=0.001,
                        ignore=("bottle",))
    bodykit.hinge("nozzle_turn", "bottle", "nozzle", anchor=noz_w, axis="X", spring=_k_small(),
                  damping=L.NOZ_FRICTION_DAMP, target_deg=0.0)
    caddy = Body.build("caddy", pos=(L.CADDY_XY[0], L.CADDY_XY[1], TOP), mass=L.CADDY_MASS, shapes=_caddy_shapes(),
                       color=CADDY_C, friction=(0.6, 0.5), solver_iterations=16)
    ox, oy = L.OTHER_XY
    other = Body.build("bottle_2", pos=(ox, oy, TOP + L.FLOOR_TOP), mass=L.BOTTLE_MASS,
                       shapes=_bottle_shapes("OFF"), color=WHITE, friction=L.BOTTLE_FRICTION, solver_iterations=16,
                       com=L.BOTTLE_COM)
    cup = Body.build("cup", pos=(L.CUP_XY[0], L.CUP_XY[1], TOP), mass=0.05, shapes=_cup_shapes(), color=CUP_C,
                     kinematic=True)
    tower = Body.build("tower", pos=(L.TOWER_XY[0], L.TOWER_XY[1], 0.0), mass=5.0, shapes=_tower_shapes(),
                       color=WOOD_C, kinematic=True)
    return [cabinet, door, bottle, nozzle, caddy, other, cup, tower]


def _k_small() -> float:
    """The detent spring's stiffness at a click (N m per degree): the slope of DETENT_TORQUE * sin(4 * error)."""
    return 4.0 * L.DETENT_TORQUE * math.pi / 180.0


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
SPEC = {o.name: o for o in OBJECTS}


# -- reading the scene ------------------------------------------------------------------------------------------------
def nozzle_angle(st) -> float:
    """The nozzle's turn against the bottle about the spray axis, degrees in (-180, 180]: 0 SPRAY, +-90 OFF, 180
    STREAM."""
    r = st["bottle"].rotation().T @ st["nozzle"].rotation()
    return math.degrees(math.atan2(r[2, 1], r[1, 1]))


def nozzle_off(st) -> bool:
    return abs(abs(nozzle_angle(st)) - 90.0) <= L.OFF_WITHIN


def setting(st) -> str:
    a = abs(nozzle_angle(st))
    return "OFF" if abs(a - 90.0) <= 45.0 else ("SPRAY" if a < 45.0 else "STREAM")


def door_open_deg(st) -> float:
    """How far the door stands open (degrees; 0 shut)."""
    r = st["cabinet"].rotation().T @ st["door"].rotation()
    return -math.degrees(math.atan2(r[1, 0], r[0, 0]))


def tilt_deg(st, name: str) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(st[name].rotation()[2, 2])))))


def in_cabinet(st) -> bool:
    """Is the bottle's base wholly inside the cabinet's walls (behind the shut door's line), resting on its floor?"""
    loc = st["cabinet"].to_local(st["bottle"].pos)
    if not (-0.005 <= loc[2] - L.FLOOR_TOP <= 0.012):
        return False
    hd, hw, p, r = L.CAB_D / 2, L.CAB_W / 2, L.PANEL, L.BOTTLE_R
    return (-hd + r - 0.002 <= loc[0] <= hd - p - r + 0.002) and abs(loc[1]) <= hw - p - r + 0.002


def compartment(st) -> str | None:
    """Which caddy compartment the bottle's base is fully inside ("near", "far"), resting on its floor; else None."""
    loc = st["caddy"].to_local(st["bottle"].pos)
    if not (-0.005 <= loc[2] - L.CADDY_FLOOR <= 0.012):
        return None
    room_x = L.COMP_W / 2 - L.BOTTLE_R
    room_y = L.COMP_L / 2 - L.BOTTLE_R
    for name, cx in (("near", -(L.CENTER_W / 2 + L.COMP_W / 2)), ("far", L.CENTER_W / 2 + L.COMP_W / 2)):
        if abs(loc[0] - cx) <= room_x + 0.002 and abs(loc[1]) <= room_y + 0.002:
            return name
    return None


def finger_points(p, r, half: float) -> list[np.ndarray]:
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list:
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
        out.extend(finger_points(p, r, half))
    if not all(np.all(np.isfinite(p)) for p in out):
        raise ValueError("a gripper position is not finite")
    return out


def _shape_boxes(name: str) -> list[tuple[np.ndarray, np.ndarray]]:
    """Each colliding shape of a body as a box (centre, half sizes) in the body frame (a turned box: its bounding box)."""
    return [(np.asarray(sh.centre, float), sh.half_extent()) for sh in SPEC[name].shapes if getattr(sh, "collide", True)]


def clearance(st, points) -> float:
    """Smallest distance (m) from the finger points to the bottle's and its nozzle's parts (each part's box)."""
    best = float("inf")
    for name in ("bottle", "nozzle"):
        boxes = _SHAPES[name]
        for p in points:
            loc = st[name].to_local(p)
            for c, h in boxes:
                d = np.maximum(0.0, np.abs(loc - c) - h)
                best = min(best, float(np.linalg.norm(d)))
    return best


_SHAPES = {n: _shape_boxes(n) for n in ("bottle", "nozzle")}
TRIG_C, TRIG_AX, TRIG_HALF = _trigger_box()
TRIGGER_GAP = 0.002  # a finger point this close to the trigger counts as a finger on it


def on_trigger(st, points) -> bool:
    """Is any finger point within TRIGGER_GAP of the trigger (bottle frame)?"""
    for p in points:
        loc = TRIG_AX.T @ (st["bottle"].to_local(p) - TRIG_C)
        d = np.maximum(0.0, np.abs(loc) - TRIG_HALF)
        if float(np.linalg.norm(d)) <= TRIGGER_GAP:
            return True
    return False


# -- the detent and the latch -------------------------------------------------------------------------------------------
def detent_torque(angle_deg: float) -> float:
    """The detent's restoring torque (N m) at a nozzle angle: DETENT_TORQUE * sin(4 * (click - angle)), zero at every
    click and half-way between clicks, at most DETENT_TORQUE 22.5 degrees from a click."""
    click = 90.0 * round(angle_deg / 90.0)
    return L.DETENT_TORQUE * math.sin(math.radians(4.0 * (click - angle_deg)))


def _set_detent(angle_deg: float) -> None:
    """Re-aim the hinge drive at the nearest click with the stiffness that gives detent_torque() here."""
    click = 90.0 * round(angle_deg / 90.0)
    err = click - angle_deg
    k = _k_small() if abs(err) < 0.5 else detent_torque(angle_deg) / err
    bodykit.set_hinge_drive("nozzle_turn", click, max(k, 0.0), L.NOZ_FRICTION_DAMP)


# -- episode state, the score -----------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["SPRAY_DEBUG"], "w") if os.environ.get("SPRAY_DEBUG") else None
HANDS_CLEAR = 0.05  # every finger point at least 50 mm from the bottle ...
CLEAR_STEPS = 20  # ... for the last 1 s of the trial
UPRIGHT_DEG, FALLEN_DEG = 15.0, 45.0
OTHER_MOVE, OTHER_TILT = 0.020, 10.0  # the second bottle: moved at most 20 mm, tilted at most 10 degrees
CADDY_MOVE, CAB_MOVE = 0.050, 0.030  # the caddy moved at most 50 mm, the cabinet at most 30 mm
SHUT_DEG = 0.8  # at the end the latched door is shut: its free edge within 5 mm (0.8 degrees at 35 cm)
MOVED = ("bottle_2", "caddy", "cabinet")


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, clear_hist=[], st=None, clear=float("inf"), start=None, trigger_breach=False, latched=False,
               report={})


_blank()


@bodykit.on_reset
def _reset() -> None:
    start = _EP.get("start")
    _blank()
    _EP["start"] = start


def evaluate(st, clear_hist, start, breach: bool, latched: bool) -> dict:
    ang = nozzle_angle(st)
    off = nozzle_off(st)
    tilt = tilt_deg(st, "bottle")
    inside = in_cabinet(st)
    door = door_open_deg(st)
    shut = latched and door <= SHUT_DEG
    on_counter = float(st["bottle"].pos[2]) >= TOP - 0.01
    clear = len(clear_hist) >= CLEAR_STEPS and min(clear_hist[-CLEAR_STEPS:]) >= HANDS_CLEAR
    moved = {n: (float(np.linalg.norm(np.asarray(st[n].pos) - start[n])) if start else 0.0) for n in MOVED}
    other_ok = (moved["bottle_2"] <= OTHER_MOVE and tilt_deg(st, "bottle_2") <= OTHER_TILT
                and moved["caddy"] <= CADDY_MOVE and moved["cabinet"] <= CAB_MOVE)
    gate = off and not breach and tilt <= FALLEN_DEG and on_counter and clear and other_ok
    score = 0.0
    if gate:
        if inside and shut:
            score = 1.0 if tilt <= UPRIGHT_DEG else 0.8
        elif inside:
            score = 0.3
        else:
            score = 0.1
    return dict(score=score, nozzle_deg=round(ang, 1), setting=setting(st), nozzle_off=off, trigger_touched=breach,
                in_cabinet=inside, door_open_deg=round(door, 1), latched=latched, tilt_deg=round(tilt, 1),
                compartment=compartment(st), on_counter=on_counter, hands_clear=clear,
                min_clear_mm=round(min(clear_hist[-CLEAR_STEPS:]) * 1000, 1) if clear_hist else None,
                moved_mm={n: round(v * 1000, 1) for n, v in moved.items()},
                bottle_2_tilt=round(tilt_deg(st, "bottle_2"), 1),
                bottle_xy=[round(float(v), 4) for v in st["bottle"].pos[:2]])


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    ang = nozzle_angle(st)
    _set_detent(ang)
    door = door_open_deg(st)
    if not _EP["latched"] and door <= L.LATCH_DEG:
        _EP["latched"] = True  # the hook catches; from now on it holds the door shut
        bodykit.set_hinge_drive("door_turn", 0.0, L.LATCH_SPRING, L.DOOR_DAMP)
        if _DBG is not None:
            _DBG.write(f"{_EP['step']:4d} LATCHED (door {door:.2f} deg)\n")
    try:
        pts = _hand_points(env.unwrapped)
        clear = clearance(st, pts)
        pressed = on_trigger(st, pts)
    except Exception as err:  # noqa: BLE001  (fails closed: not released, and a finger on the trigger)
        clear, pressed = 0.0, True
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    if pressed and not nozzle_off(st):
        if not _EP["trigger_breach"] and _DBG is not None:
            _DBG.write(f"{_EP['step']:4d} TRIGGER touched with the nozzle at {ang:.1f} deg\n")
        _EP["trigger_breach"] = True
    _EP["clear"] = clear
    _EP["clear_hist"].append(clear)
    if len(_EP["clear_hist"]) > CLEAR_STEPS:
        _EP["clear_hist"].pop(0)
    _EP["step"] += 1
    if _DBG is not None and _EP["step"] % 2 == 0:
        _debug(st, ang)


def report(objects=None) -> dict:
    st = _EP.get("st")
    if st is None:
        return dict(score=0.0)
    return evaluate(st, _EP["clear_hist"], _EP.get("start"), _EP["trigger_breach"], _EP["latched"])


def score(objects) -> float:
    """Read at the end of the trial (see task.json "scoring"). 0 unless the nozzle is within 20 degrees of an OFF
    click, no finger point came within 2 mm of the trigger while the nozzle was not OFF, the bottle is tilted at most
    45 degrees and still at or above the counter, every finger point has been at least 50 mm from the bottle for the
    last 1 s, the second bottle moved at most 20 mm and tilted at most 10 degrees, the caddy moved at most 50 mm and
    the cabinet at most 30 mm. Then: bottle in the cabinet (base inside its walls, on its floor) with the door latched
    and shut: 1.0 (tilt at most 15 degrees) or 0.8 (15 to 45); in the cabinet, door not latched: 0.3; anywhere else:
    0.1."""
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
    p = np.asarray(st["bottle"].pos, float)
    return float(np.linalg.norm(p[:2] - np.asarray(L.PLACE_XY)))


def _debug(st, ang) -> None:
    b = st["bottle"].pos
    _DBG.write(f"{_EP['step']:4d} noz={ang:7.2f} tq={detent_torque(ang):+.4f} off={int(nozzle_off(st))} "
               f"b=({b[0]:.4f},{b[1]:.4f},{b[2] - TOP:.4f}) tilt={tilt_deg(st, 'bottle'):5.1f} "
               f"cab={int(in_cabinet(st))} door={door_open_deg(st):6.2f} latch={int(_EP['latched'])} "
               f"clear={_EP['clear']:.3f} trig={int(_EP['trigger_breach'])} "
               f"b2=({st['bottle_2'].pos[0]:.4f},{st['bottle_2'].pos[1]:.4f}) "
               f"cabinet=({st['cabinet'].pos[0]:.4f},{st['cabinet'].pos[1]:.4f}) "
               f"caddy=({st['caddy'].pos[0]:.4f},{st['caddy'].pos[1]:.4f})\n")
    _DBG.flush()


# -- the fixed start --------------------------------------------------------------------------------------------------
def poses() -> dict:
    bx, by = L.BOTTLE_XY
    ident = (0.0, 0.0, 0.0, 1.0)
    return {
        "cabinet": (np.array([L.CAB_XY[0], L.CAB_XY[1], TOP + 0.0005]), ident),
        "door": (np.array([DOOR_ANCHOR[0], DOOR_ANCHOR[1], DOOR_ANCHOR[2] + 0.0005]), DOOR_START),
        "bottle": (np.array([bx, by, TOP + 0.0005]), ident),
        "nozzle": (np.array([bx + NOZ_C[0], by, TOP + 0.0005 + NOZ_C[2]]), ident),
        "caddy": (np.array([L.CADDY_XY[0], L.CADDY_XY[1], TOP + 0.0005]), ident),
        "bottle_2": (np.array([L.OTHER_XY[0], L.OTHER_XY[1], TOP + L.FLOOR_TOP + 0.001]), ident),
        "cup": (np.array([L.CUP_XY[0], L.CUP_XY[1], TOP]), ident),
        "tower": (np.array([L.TOWER_XY[0], L.TOWER_XY[1], 0.0]), ident),
    }


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put every body at its one fixed place (no seed changes it), let them settle, return the first observation."""
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
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    st = _scene.object_states(emb._env, NAMES)
    _EP["st"] = st
    _EP["start"] = {n: np.asarray(st[n].pos, float).copy() for n in MOVED}
    if _DBG is not None:
        _DBG.write(f"seed {seed}: fixed layout\n")
        _debug(st, nozzle_angle(st))
    return emb._observation()


# -- the scripted demo (reads object poses from the simulator to plan; moves the parts only through the grippers) ------
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "" (full), caddy, spray, dooropen, nothing (README)
NOZ_OPEN, NOZ_SQUEEZE = -0.020, -(L.NOZ_HALF - 0.002)  # 2 mm past contact on each side of the 15.2 mm cap
NECK_OPEN, NECK_SQUEEZE = -0.032, -(L.CLOSURE_R - 0.003)  # 3 mm past contact on the 31.5 mm closure
TURN_DEG = float(os.environ.get("SPRAY_TURN_DEG", "95"))  # the wrist turns this far: 5 degrees past the click
TURN_SIGN = float(os.environ.get("SPRAY_TURN_SIGN", "1"))
PICK_AZ = math.radians(L.PICK_AZ_DEG)
DOOR_UP = float(os.environ.get("SPRAY_DOOR_UP", "-1"))  # which way the closed fingers' plane faces while pushing


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def rest(arm: str, seconds: float) -> Waypoint:
    """The end pose: the ready pose drawn back 12 cm and out 4 cm."""
    p, r = ready_pose(arm)
    side = 1.0 if arm == "left" else -1.0
    return Waypoint(seconds, tuple((p + np.array([-0.12, 0.04 * side, 0.0])).tolist()), FINGER_OPEN,
                    "back to the ready pose", r)


def demo():
    """Both arms wait at the ready pose; the real plan is made on the first control step (spraydemo.py)."""
    return {arm: [park(arm, 0.5, label="look")] for arm in ("left", "right")}


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) / np.linalg.norm(close)
    x = x - z * float(x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _rx(a: float) -> np.ndarray:
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _wp(sec, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(np.asarray(pos, float).tolist()), float(finger), label, np.asarray(rot, float))


def _az(a: float) -> np.ndarray:
    """The neck grip's frame for a heading ``a`` (0 along +x, pi/2 along +y): fingers level, closing across."""
    d = np.array([math.cos(a), math.sin(a), 0.0])
    return _frame(d, (math.sin(a), -math.cos(a), 0.0))


def nozzle_moves(arm: str, b_p: np.ndarray) -> list[Waypoint]:
    """Pinch the nozzle's two side faces from the front (fingers along +x, the spray axis), turn the wrist a quarter
    turn about that axis, let go and back straight out."""
    tcp = b_p + np.array([L.NOZ_FRONT_X - L.NOZ_TCP_BACK, 0.0, L.NOZ_AXIS_Z])
    r0 = _frame((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))  # closing across y
    sign = TURN_SIGN
    back = tcp - np.array([0.06, 0.0, 0.0])
    moves = [
        _wp(3.0, back + np.array([0.0, 0.0, 0.03]), NOZ_OPEN, "in front of the nozzle", r0),
        _wp(0.8, back, NOZ_OPEN, "in front of the nozzle", r0),
        _wp(1.6, tcp, NOZ_OPEN, "fingers either side of the nozzle", r0),
        _wp(0.8, tcp, NOZ_SQUEEZE, "pinch the nozzle", r0),
    ]
    for k in (1, 2, 3):
        moves.append(_wp(0.8, tcp, NOZ_SQUEEZE, "turn the nozzle", _rx(sign * math.radians(TURN_DEG) * k / 3) @ r0))
    r1 = _rx(sign * math.radians(90.0)) @ r0
    moves += [
        _wp(0.6, tcp, NOZ_SQUEEZE, "turn the nozzle", r1),  # back to the click: the detent holds it there
        _wp(0.8, tcp, NOZ_OPEN, "let go of the nozzle", r1),
        _wp(1.2, back, NOZ_OPEN, "back off", r1),
    ]
    return moves


def _neck(b_p: np.ndarray, a: float = PICK_AZ) -> np.ndarray:
    """The neck grip's TCP: on the closure, 4 mm short of the axis along the heading."""
    return b_p + np.array([-0.004 * math.cos(a), -0.004 * math.sin(a), sum(L.CLOSURE_Z) / 2])


def neck_pick(b_p: np.ndarray) -> list[Waypoint]:
    r = _az(PICK_AZ)  # from the front right, closing across
    neck = _neck(b_p)
    d = r[:, 2]
    return [
        _wp(2.5, neck - 0.08 * d + np.array([0.0, 0.0, 0.04]), NECK_OPEN, "beside the neck", r),
        _wp(0.8, neck - 0.08 * d, NECK_OPEN, "beside the neck", r),
        _wp(1.8, neck, NECK_OPEN, "fingers either side of the neck", r),
        _wp(0.8, neck, NECK_SQUEEZE, "hold the neck", r),
    ]


def carry(b_p: np.ndarray, target_xy, lift: float, floor: float) -> list[Waypoint]:
    """Lift the bottle, carry it over ``target_xy``, set it down on a floor ``floor`` above the counter, let go and
    back out along the heading."""
    r = _az(PICK_AZ)
    d = r[:, 2]
    neck = _neck(b_p)
    up = neck + np.array([0.0, 0.0, lift])
    tgt = np.array([target_xy[0], target_xy[1], 0.0]) + (neck - b_p) * np.array([1.0, 1.0, 0.0])
    over = np.array([tgt[0], tgt[1], up[2]])
    down = np.array([tgt[0], tgt[1], TOP + floor + sum(L.CLOSURE_Z) / 2 + 0.004])
    return [
        _wp(1.5, up, NECK_SQUEEZE, "lift the bottle", r),
        _wp(3.5, over, NECK_SQUEEZE, "carry the bottle", r),
        _wp(0.8, over, NECK_SQUEEZE, "steady", r),  # spraydemo re-aims here from the bottle's pose
        _wp(1.5, down, NECK_SQUEEZE, "set the bottle down", r),
        _wp(0.6, down, NECK_SQUEEZE, "set down", r),
        _wp(0.8, down, NECK_OPEN, "let go of the bottle", r),
        _wp(1.5, down - 0.12 * d, NECK_OPEN, "back off", r),
        _wp(1.0, down - 0.20 * d + np.array([0.0, 0.0, 0.06]), NECK_OPEN, "back off up", r),
    ]


def door_push() -> list[Waypoint]:
    """The left hand's closed fingertips push the door's outer face shut, 25 cm from the hinge, along its arc."""
    hx, hy = L.DOOR_HINGE
    rad, z = L.DOOR_PUSH_R, TOP + L.DOOR_PUSH_Z
    moves = []

    def at(th_deg: float, out: float):
        t = math.radians(th_deg)
        n = np.array([-math.cos(t), math.sin(t), 0.0])  # the outer face's normal
        c = np.array([hx - rad * math.sin(t), hy - rad * math.cos(t), z])
        return c + n * (0.013 + out), _frame(-n, (0.0, 0.0, DOOR_UP))

    th0 = L.DOOR_OPEN_DEG
    p, r = at(th0, 0.06)
    moves += [_wp(3.0, p + np.array([0.0, 0.0, 0.05]), 0.0, "to the door", r), _wp(1.0, p, 0.0, "to the door", r)]
    p, r = at(th0, -0.002)
    moves.append(_wp(1.2, p, 0.0, "touch the door", r))
    for th in np.arange(th0 - 10.0, -0.1, -10.0).tolist() + [0.0, -3.0]:
        p, r = at(th, -0.004)
        moves.append(_wp(0.7, p, 0.0, "push the door shut", r))
    moves.append(_wp(0.5, p, 0.0, "push the door shut", r))
    p, r = at(0.0, 0.08)
    moves.append(_wp(1.2, p, 0.0, "back off", r))
    return moves


def place_correction(st) -> np.ndarray:
    """How far the bottle's base is from over the place (world xy, m; z 0)."""
    p = np.asarray(st["bottle"].pos, float)
    if VARIANT == "caddy":
        want = np.asarray(st["caddy"].pos, float) + st["caddy"].rotation() @ np.array(
            [-(L.CENTER_W / 2 + L.COMP_W / 2), 0.0, 0.0])
    else:
        want = np.array([L.PLACE_XY[0], L.PLACE_XY[1], 0.0]) + (np.asarray(st["cabinet"].pos) - _EP["start"]["cabinet"])
    return np.array([want[0] - p[0], want[1] - p[1], 0.0])


def plan_from_state(st) -> dict[str, list[Waypoint]]:
    """Both hands' plans, from the bottle's start pose. The right hand turns the nozzle, then picks the bottle by the
    neck and stands it in the cabinet; the left hand then pushes the door shut."""
    b_p = np.asarray(st["bottle"].pos, float)
    b_p = np.array([b_p[0], b_p[1], TOP])
    if VARIANT == "nothing":
        return {arm: [park(arm, 30.0, label="wait")] for arm in ("left", "right")}
    right = [] if VARIANT == "spray" else nozzle_moves("right", b_p)
    if right:
        right.append(park("right", 2.0, label="via the ready pose"))  # resets the arm's pose before the neck grip
    right += neck_pick(b_p)
    if VARIANT == "caddy":
        right += carry(b_p, L.SLOT_XY, L.CADDY_LIFT, L.CADDY_FLOOR)
    else:
        right += carry(b_p, L.PLACE_XY, L.LIFT, L.FLOOR_TOP)
    right += [rest("right", 2.5), rest("right", 1.0)]
    t_out = sum(w.duration for w in right) - 3.5  # the right hand is out of the cabinet
    if VARIANT in ("caddy", "dooropen"):
        left = [park("left", 1.0, label="wait")]
    else:
        left = [park("left", t_out - 2.0, label="wait")] + door_push() + [rest("left", 2.5), rest("left", 1.0)]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    print(f"spraydemo: variant '{VARIANT}', bottle ({b_p[0]:.3f}, {b_p[1]:.3f}), plan {max(tl, tr):.1f} s", flush=True)
    return {"left": left, "right": right}


import spraydemo  # noqa: E402  (the demo's control hooks for yam_scripted)

spraydemo.install()
spraydemo_task = True
