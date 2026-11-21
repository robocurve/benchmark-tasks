"""livekit: scene, read-only measurements, connector physics and per-physics-substep monitors of the merged LiPo
charging task for Isaac Lab-Arena / yam_arena (PhysX). Base: the live-banana-lead task's scene (charge lead's XT60
mated to the pack and seated in a fixed bench clip, both banana plugs live and loose on the mat). Added from the
balance-charge task: the XT60 sprung-contact friction element, four sprung charger keys, the charger state model
(program preset to LiPo BALANCE 2.2 A 3S) and its LCD.

Never writes object poses, joint states or scores. Physics it adds:
  - bounded axial retention (<= 0.5 N, full depth only) of the banana plugs, C5 and XH plugs (reaction on the charger);
  - XT60 contact friction: a D6 joint (every DOF free) between the fixed clip and the pack's XT60 with a velocity
    drive (target 0) along the mating axis, force capped at XT60_FRICTION_N while the halves are coaxial and inside
    the contact engagement, 0 otherwise: it resists pulling out and pushing in alike and never pulls a plug in;
  - key plungers: prismatic, 1.2 mm travel, preloaded springs.
Gripper force: each YAM finger capped at GRIP_N.
"""
from __future__ import annotations

import math
import os
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
from partkit import (Hinge, env, finger_points, on_physics, on_pre_reset, on_reset, on_spawn, on_step, part,  # noqa: E402
                     set_grip_force, set_pad_friction, set_physics_dt, set_screen)
from charger import KEYS, Charger, KeySwitches  # noqa: E402
from task_scoring import ChargeScorer  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

DEBUG = set(filter(None, os.environ.get("CHARGE_DEBUG", "").split(",")))  # dev experiments only; never in evidence runs
PHYSICS_DT = 1.0 / 960.0  # 0.15 mm banana pin clearance: smaller step than the station's 1/240 s
set_physics_dt(PHYSICS_DT)
GRIP_N = 40.0  # N per finger (real YAM)
set_grip_force(GRIP_N)
# YAM finger pads: the arm asset has no pad material (PhysX default 0.5). Rubber pad on the plug's nylon/heat-shrink:
# 0.8 static / 0.7 dynamic, an engineering ESTIMATE (rubber on dry concrete 0.6-0.85, on dry asphalt 0.9 static).
_pad = os.environ.get("CHARGE_PAD_FRICTION", "0.8,0.7")
PAD_FRICTION = None if _pad == "none" else tuple(map(float, _pad.split(",")))
set_pad_friction(PAD_FRICTION)
XT60_FRICTION_N = 18.0  # N, extraction / insertion (low end of the Amass XT60 TDS 1.8-4.5 kgf; pair gauge-checked at 15-20 N)
# N s/m: the drive slips at F/D below the cap; 5e6 keeps the slip under a lead's ~2 N pull to 0.4 um/s (dry friction)
XT60_FRICTION_DAMPING = float(os.environ.get("CHARGE_XT60_DAMPING", "5e6"))
XT60_ENGAGE_M, XT60_ENGAGE_LATERAL_M, XT60_ENGAGE_ANGLE_DEG = 0.007, 0.0006, 6.0
GRIPPER_LINKS = ("gripper", "tip_left", "tip_right")
GRIPPER_CONTACT_N = 0.05  # N: a gripper link touching a plug or wire above this counts as a touch
WIRE_ZONE_M = 0.05  # wire within 5 cm of each banana plug
CLEARANCE_M = 0.030

TOP = TABLE_TOP["station"]
MAT_TOP = TOP + 0.002
RISER_HEIGHT = 0.090
CHARGER = np.array([0.25, 0.06, TOP + 0.0205 + RISER_HEIGHT])
CHARGER_YAW = math.pi / 2
RC = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # charger frame -> world
BALANCE_PORT = np.array([0.068, 0.025, -0.004])  # 3S, charger frame; the other XH ports are drawn only
OUTPUT_PORTS = {"positive": np.array([0.073, -0.02, -0.004]), "negative": np.array([0.073, -0.04, -0.004])}
POWER_PORT = np.array([-0.068, 0.0, -0.001])
RS_POWER = np.diag([-1.0, -1.0, 1.0])  # C5 inlet frame in the charger frame (faces charger -X)
CLIP = np.array([0.270, 0.227, TOP + 0.017])
RCLIP = np.diag([-1.0, -1.0, 1.0])  # clip frame -> world (opening toward world -X)
BATTERY = np.array([0.1765, 0.200, MAT_TOP + 0.0131])  # front face 3 cm behind and 2.7 cm beside the XT60 sleeve
BANANA_REST = {"negative": np.array([0.290, 0.262, MAT_TOP + 0.0042]),
               "positive": np.array([0.360, 0.306, MAT_TOP + 0.0042])}
HARNESS_LAY = {"negative": (0.230, 0.4215), "positive": (0.224, 0.4174)}
SEATED_OFFSET = {"balance": 0.0011, "power": 0.0005, "main": 0.0008}
XT60 = np.array([0.0155, 0.016, 0.0081])
SLEEVE = np.array([0.016, 0.013, 0.006])  # heat-shrink over the male XT60's solder cups
XH = np.array([0.008, 0.0125, 0.0057])
C5_BODY = ((-0.003, 0.0, 0.00075), (0.018, 0.018, 0.0165))  # three-lobe C5 envelope, inlet frame (centre, size)

RED, BLACK, GOLD, BLUE = (0.70, 0.03, 0.03), (0.06, 0.06, 0.06), (0.80, 0.65, 0.20), (0.08, 0.30, 0.65)
DARK, WHITE, YELLOW, GREY = (0.08, 0.08, 0.08), (0.88, 0.88, 0.84), (0.95, 0.72, 0.06), (0.65, 0.65, 0.68)

# banana jack: 4.3 mm bore (apothem 2.15 mm, 8 staves), 14 mm deep, face at the port origin, axis charger +X
R_IN, WALL, L_SLEEVE = 0.00215, 0.0023, 0.014
PIN_R, PIN_LEN = 0.002, 0.012  # pin 4.0 mm, 12 mm overall incl. the 2 mm-radius rounded nose
SHORT_GAP = 0.0001  # metal-to-metal contact: geometric gap below 0.1 mm
PIN_GAP = 0.0005  # the two live pins may not come within 0.5 mm of each other
LIVE_OVERLAP = 0.010  # XT60 housings still telescoped (< 10 mm withdrawn) => live
LEADS = ("positive", "negative")
OTHER = {"positive": "negative", "negative": "positive"}


# ---------------------------------------------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------------------------------------------
def resample(points, max_link):
    """Smooth polyline through waypoints at uniform spacing (reset routing only)."""
    from scipy.interpolate import CubicSpline
    p = np.asarray(points, float)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    dense = CubicSpline(s, p, bc_type="natural")(np.linspace(0, s[-1], 2000))
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    n = max(2, int(np.ceil(arc[-1] / max_link)))
    return np.array([np.interp(np.linspace(0, arc[-1], n + 1), arc, dense[:, k]) for k in range(3)]).T


def _perp(t):
    t = t / np.linalg.norm(t)
    a = np.cross(t, [0.0, 0.0, 1.0])
    if np.linalg.norm(a) < 0.3:
        a = np.cross(t, [1.0, 0.0, 0.0])
    a /= np.linalg.norm(a)
    return a, np.cross(t, a)


def cable(prefix, points, radius, mass, ei, damping, color, parent=""):
    """Chain of revolute links along ``points`` (part frame). Each link bends about one axis normal to it, alternating
    between the two normals, with a spring of EI / link length toward the authored (reset) shape."""
    pts = np.asarray(points, float)
    lengths = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    hinges = []
    for i in range(len(pts) - 1):
        t = pts[i + 1] - pts[i]
        a, b = _perp(t)
        # native capsule links (cylinder links resting on the mat crept sideways in PhysX)
        hinges.append(Hinge(f"{prefix}{i}", tuple(pts[i]), axis=tuple(a if i % 2 == 0 else b),
                            capsules=((tuple((pts[i] + pts[i + 1]) / 2), radius, max(lengths[i] - 2 * radius, 1e-4),
                                       color, tuple(t)),),
                            parent=parent, rate=ei / lengths[i], damping=damping, mass=mass * lengths[i] / lengths.sum()))
        parent = hinges[-1].name
    return hinges


def plug_link(name, rear, u, parent, mass, boxes=(), cylinders=(), spheres=(), relief_r=0.0016, rate=2e-3):
    """Strain relief (two bending hinges normal to the plug axis ``u``) and the plug body at the cable end ``rear``."""
    a, b = _perp(np.asarray(u, float))
    rear = tuple(float(v) for v in rear)
    return [Hinge(f"{name}_relief", rear, axis=tuple(a), spheres=((rear, relief_r, DARK),), parent=parent, rate=rate,
                  damping=5e-5, mass=0.0003),
            Hinge(name, rear, axis=tuple(b), parent=f"{name}_relief", rate=rate, damping=5e-5, mass=mass,
                  boxes=tuple(boxes), cylinders=tuple(cylinders), spheres=tuple(spheres))]


def oriented_box(centre, size, r, origin):
    """Axis-aligned box (centre, size) of a frame with signed-permutation rotation ``r`` -> part-frame box."""
    c = np.asarray(origin, float) + r @ np.asarray(centre, float)
    s = np.abs(r) @ np.asarray(size, float)
    return tuple(c.tolist()), tuple(s.tolist())


# ---------------------------------------------------------------------------------------------------------------
# charger (free rigid body; boxes in the charger frame) + measurement copies of its conductors
# ---------------------------------------------------------------------------------------------------------------
CONDUCTORS = []  # (name, kind, centre, half, rot) in the charger frame; kind: case|jack_positive|jack_negative|...
_charger_boxes = []


def _cbox(name, centre, size, color, kind=None, rot=None):
    centre = tuple(float(v) for v in centre)
    size = tuple(float(v) for v in size)
    if rot is None:
        _charger_boxes.append((centre, size, color))
        R = np.eye(3)
    else:
        _charger_boxes.append((centre, size, color, ("X", float(rot))))
        c, s = math.cos(rot), math.sin(rot)
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if kind:
        CONDUCTORS.append((name, kind, np.array(centre), np.array(size) / 2, R))


def _side_wall(name, x, holes):
    ys = sorted(set([-0.0675, 0.0675] + [v for h in holes for v in h[:2]]))
    zs = sorted(set([-0.017, 0.015] + [v for h in holes for v in h[2:]]))
    for y0, y1 in zip(ys, ys[1:]):
        for z0, z1 in zip(zs, zs[1:]):
            y, z = (y0 + y1) / 2, (z0 + z1) / 2
            if any(h[0] < y < h[1] and h[2] < z < h[3] for h in holes):
                continue
            _cbox(f"{name}_{y:.4f}_{z:.4f}", (x, y, z), (0.001, y1 - y0, z1 - z0), BLUE, "case")


# aluminium case, TREATED as conductive (bare vs anodised not verified)
_cbox("electronics_core", (0, 0, -0.001), (0.100, 0.100, 0.020), DARK, "case")
_cbox("case_top", (0, 0, 0.0145), (0.144, 0.135, 0.001), BLUE, "case")
_cbox("case_bottom", (0, 0, -0.0165), (0.144, 0.135, 0.001), BLUE, "case")
for _s in (-1, 1):
    _cbox(f"case_flank{_s}", (0, _s * 0.067, -0.001), (0.144, 0.001, 0.030), BLUE, "case")
_bp = BALANCE_PORT
_holes_out = [[_bp[1] - XH[1] / 2 - 0.0018, _bp[1] + XH[1] / 2 + 0.0018, _bp[2] - 0.0044, _bp[2] + 0.0044]]
for _o in OUTPUT_PORTS.values():
    _holes_out.append([_o[1] - 0.0048, _o[1] + 0.0048, _o[2] - 0.0048, _o[2] + 0.0048])
_side_wall("output_wall", 0.0715, _holes_out)
_c5c, _c5s = np.array(C5_BODY[0]), np.array(C5_BODY[1])
_c5_in_lo, _c5_in_hi = _c5c - _c5s / 2 - 0.0002, _c5c + _c5s / 2 + 0.0002  # cavity 0.2 mm clear, inlet frame
_c5_out_lo, _c5_out_hi = _c5_in_lo - 0.0015, _c5_in_hi + 0.0015
_pp = POWER_PORT
_side_wall("input_wall", -0.0715, [[_pp[1] - _c5_out_hi[1], _pp[1] - _c5_out_lo[1],
                                     _pp[2] + _c5_out_lo[2], _pp[2] + _c5_out_hi[2]]])
for _x in (-0.055, 0.055):  # rubber feet
    for _y in (-0.05, 0.05):
        _cbox(f"foot_{_x}_{_y}", (_x, _y, -0.017), (0.012, 0.012, 0.002), DARK)
# banana jacks: 8 metal staves around the bore + back plate
for _name, _o in OUTPUT_PORTS.items():
    _w = 2 * (R_IN + WALL) * math.tan(math.pi / 8) + 0.0002
    _rc = R_IN + WALL / 2
    for _k in range(8):
        _a = _k * math.pi / 4
        _cbox(f"{_name}_stave{_k}", _o + (-L_SLEEVE / 2, -_rc * math.sin(_a), _rc * math.cos(_a)), (L_SLEEVE, _w, WALL),
              RED if _name == "positive" else BLACK, f"jack_{_name}", rot=_a)
    _cbox(f"{_name}_back", _o + (-L_SLEEVE - 0.0005, 0, 0), (0.001, 0.009, 0.009), DARK, f"jack_{_name}")
# 3S XH balance socket (nylon): walls 0.1 mm clear of the housing (the omitted header pins locate a real XH plug; with
# 0.25 mm the light lead tilted the preset plug ~4 deg), stop 4 mm behind the port origin
_cbox("balance_stop", _bp + (-0.009, 0, 0), (0.010, XH[1] + 0.003, XH[2] + 0.003), WHITE)
for _s in (-1, 1):
    _cbox(f"balance_side{_s}", _bp + (-0.002, _s * (XH[1] / 2 + 0.00085), 0), (0.012, 0.0015, XH[2] + 0.0032), WHITE)
    _cbox(f"balance_tb{_s}", _bp + (-0.002, 0, _s * (XH[2] / 2 + 0.00085)), (0.012, XH[1] + 0.0032, 0.0015), WHITE)
CONDUCTORS.append(("balance_pins", "balance_pin", _bp + (-0.0015, 0, 0), np.array([0.0025, 0.006, 0.0025]), np.eye(3)))
# C5 inlet (charger -X wall): cavity 0.3 mm clear of the plug envelope, stop at the pin bases (inlet x = -12 mm)
for _c, _sz in [(((-0.015), 0.0, (_c5_out_lo[2] + _c5_out_hi[2]) / 2),
                 (0.006, _c5_out_hi[1] - _c5_out_lo[1], _c5_out_hi[2] - _c5_out_lo[2]))]:
    _cc, _ss = oriented_box(_c, _sz, RS_POWER, POWER_PORT)
    _cbox("power_stop", _cc, _ss, DARK)
_x0, _x1 = -0.012, 0.0035
for _s, (_lo, _hi) in enumerate([(_c5_out_lo[1], _c5_in_lo[1]), (_c5_in_hi[1], _c5_out_hi[1])]):
    _cc, _ss = oriented_box(((_x0 + _x1) / 2, (_lo + _hi) / 2, (_c5_out_lo[2] + _c5_out_hi[2]) / 2),
                            (_x1 - _x0, _hi - _lo, _c5_out_hi[2] - _c5_out_lo[2]), RS_POWER, POWER_PORT)
    _cbox(f"power_wall_y{_s}", _cc, _ss, DARK)
for _s, (_lo, _hi) in enumerate([(_c5_out_lo[2], _c5_in_lo[2]), (_c5_in_hi[2], _c5_out_hi[2])]):
    _cc, _ss = oriented_box(((_x0 + _x1) / 2, (_c5_out_lo[1] + _c5_out_hi[1]) / 2, (_lo + _hi) / 2),
                            (_x1 - _x0, _c5_out_hi[1] - _c5_out_lo[1], _hi - _lo), RS_POWER, POWER_PORT)
    _cbox(f"power_wall_z{_s}", _cc, _ss, DARK)
_cc, _ss = oriented_box(((_x0 + _x1) / 2, (_c5_in_lo[1] + _c5_in_hi[1]) / 2, (_c5_in_lo[2] + _c5_in_hi[2]) / 2),
                        (_x1 - _x0, _c5_in_hi[1] - _c5_in_lo[1], _c5_in_hi[2] - _c5_in_lo[2]), RS_POWER, POWER_PORT)
CONDUCTORS.append(("mains_pins", "mains_pin", np.array(_cc), np.array(_ss) / 2, np.eye(3)))
_charger_visuals = [((-0.008, 0.043, 0.0151), (0.105, 0.033, 0.0004), (0.10, 0.30, 0.62)),  # brand panel
                    ((0.0, 0.007, 0.0152), (0.066, 0.018, 0.0006), (0.10, 0.25, 0.12)),  # LCD
                    ((0.0725, -0.030, 0.006), (0.0004, 0.030, 0.005), WHITE),  # OUTPUT label strip
                    ((0.0725, -0.020, -0.0115), (0.0004, 0.004, 0.0015), RED),  # + mark under the + jack
                    ((0.0725, -0.040, -0.0115), (0.0004, 0.004, 0.0015), BLACK)]  # - mark
_charger_visuals += [((0.0722, y, z), (0.0004, w, 0.0057), WHITE)  # 2S/4S/5S/6S XH headers (not modelled as sockets)
                     for y, z, w in [(0.0105, -0.004, 0.010), (0.042, -0.004, 0.015), (0.0395, -0.0125, 0.0175),
                                     (0.01675, -0.0125, 0.020)]]
_charger_visuals = [v for v in _charger_visuals if v[2] != (0.10, 0.25, 0.12)]  # the LCD is a textured screen
KEY_POS = {k: np.array([-0.045 + 0.030 * i, -0.043, 0.0165]) for i, k in enumerate(KEYS)}  # charger frame
KEY_TRAVEL, KEY_STIFFNESS, KEY_PRELOAD = 0.0012, 2000.0, 0.0005  # m, N/m, m (1 N preload against the top stop)
LCD = ((0.0, 0.007, 0.0152), (0.066, 0.018))
LCD_DIR = pathlib.Path(os.environ.get("TMPDIR", "/tmp")) / "charge_lcd"


def lcd_png(lines):
    from PIL import Image, ImageDraw, ImageFont
    LCD_DIR.mkdir(parents=True, exist_ok=True)
    path = LCD_DIR / (("_".join(lines) or "off").replace(" ", "-").replace("/", "-").replace(":", "") + ".png")
    if not path.exists():
        im = Image.new("RGB", (512, 128), (12, 32, 100) if any(lines) else (15, 20, 25))
        d = ImageDraw.Draw(im)
        for i, line in enumerate(lines):
            d.text((8, i * 64), line, font=ImageFont.load_default(size=46), fill=(225, 240, 255))
        im.save(path)
    return str(path)


_keys = tuple(Hinge(f"key_{k}", tuple(p), axis="Z", linear=True, lower=-KEY_TRAVEL, upper=0.0, start=0.0,
                    target=KEY_PRELOAD, rate=KEY_STIFFNESS, damping=0.5, mass=0.002,
                    cylinders=((tuple(p), 0.004, 0.003, GREY, "Z"),)) for k, p in KEY_POS.items())
CHARGER_PART = part("charger", CHARGER, boxes=_charger_boxes, mass=0.65, yaw=CHARGER_YAW, color=BLUE, friction=0.5,
                    contact_offset=0.0002, visuals=_charger_visuals, hinges=_keys,
                    screen=(LCD[0], LCD[1], lcd_png(Charger().display())))
RISER = part("riser", (0.25, 0.06, MAT_TOP + RISER_HEIGHT / 2), boxes=(((0, 0, 0), (0.114, 0.124, RISER_HEIGHT), DARK),),
             mass=0.8, color=DARK, friction=1.2)
MAT = part("mat", (0.30, 0.02, TOP + 0.001), boxes=(((0, 0, 0), (0.48, 0.62, 0.002), (0.35, 0.36, 0.36)),), mass=None,
           color=(0.35, 0.36, 0.36), friction=1.0)

# ---------------------------------------------------------------------------------------------------------------
# bench clip holding the harness XT60 (female), opening toward world -X; kinematic
# ---------------------------------------------------------------------------------------------------------------
_clip_boxes = [oriented_box((-0.012, 0, -0.010), (0.04, 0.03, 0.010), RCLIP, (0, 0, 0)) + (DARK,)]
_iy, _iz = XT60[1] / 2 + 0.00025, XT60[2] / 2 + 0.00025  # 0.25 mm shell clearance (AMASS XT60 outlines, as in the balance-charge task)
for _s in (-1, 1):
    _clip_boxes.append(oriented_box((-0.002, _s * (_iy + 0.00075), 0), (0.012, 0.0015, 2 * _iz + 0.003), RCLIP, (0, 0, 0))
                       + (YELLOW,))
    _clip_boxes.append(oriented_box((-0.002, 0, _s * (_iz + 0.00075)), (0.012, 2 * _iy + 0.003, 0.0015), RCLIP, (0, 0, 0))
                       + (YELLOW,))
_clip_boxes.append(oriented_box(((-0.011 - XT60[0] / 2) / 2, 0, 0), (0.011 - XT60[0] / 2, 2 * _iy + 0.003, 2 * _iz + 0.003),
                                RCLIP, (0, 0, 0)) + (YELLOW,))  # stop: full depth = plug centre at the clip origin
# lead-in chamfer at the female shroud mouth (as moulded on XT60 housings; same 1.5 mm deep, 0.8 mm per side opening
# as the balance-charge task's female XT60): four 1.5 mm plates sloping outward from the wall ends (clip x 4 -> 5.5 mm)
_CH_L, _CH_W = 0.0015, 0.0008
_cl, _cang = math.hypot(_CH_L, _CH_W), math.atan2(_CH_W, _CH_L)
for _s in (-1, 1):
    _d = np.array([math.cos(_cang), _s * math.sin(_cang)])  # face direction (clip x, y or z)
    _n = np.array([-math.sin(_cang), _s * math.cos(_cang)])  # outward normal of the face
    for _k, _inner, _span in ((1, _iy, 2 * _iz + 0.003), (2, _iz, 2 * _iy + 0.003)):
        _mid = np.array([0.004, _s * _inner]) + _d * _cl / 2 + _n * 0.00075
        _c = np.zeros(3)
        _c[0], _c[_k] = _mid
        _size = [_cl, 0.0015, _span] if _k == 1 else [_cl, _span, 0.0015]
        _rot = ("Z", float(_s * _cang)) if _k == 1 else ("Y", float(_s * _cang))  # clip-frame turn, mapped by RCLIP
        _clip_boxes.append((tuple((RCLIP @ _c).tolist()), tuple(_size), YELLOW, _rot))
CLIP_PART = part("clip", CLIP, boxes=_clip_boxes, mass=None, color=YELLOW, friction=0.6, contact_offset=0.0003)
XT60_CAVITY = (np.array([(-0.00775 + 0.004) / 2, 0, 0]), np.array([(0.004 + 0.00775) / 2, _iy, _iz]))  # clip frame

# ---------------------------------------------------------------------------------------------------------------
# harness leads: fixed at the clip, 300 mm AWG14, ending in open 4 mm banana plugs (pins toward the robot, -X)
# ---------------------------------------------------------------------------------------------------------------
LEAD_POINTS = {}
TOUCH_LINKS = {}  # lead -> links a gripper may not touch while live: the plug and the wire within 5 cm of it


def harness_lay(name):
    """Literal flat U-shaped lay of one 300 mm lead (same waypoints as ../src/scene.py harness_lay)."""
    y0, xturn = HARNESS_LAY[name]
    rear = BANANA_REST[name] + np.array([0.020, 0, 0])
    lie = MAT_TOP + 0.0017
    s = CLIP + RCLIP @ np.array([-0.018, 0, 0.002])
    s[1] = y0
    r = (rear[1] - y0) / 2
    pts = [s, s + [0.012, 0, -0.004], [s[0] + 0.035, y0, lie]]
    for a in np.linspace(-np.pi / 2, np.pi / 2, 7):
        pts.append([xturn + r * np.cos(a), y0 + r + r * np.sin(a), lie])
    pts += [rear + [0.012, 0, -0.001], rear + [0.004, 0, 0], rear]
    points = resample(pts, 0.0075)
    points[:, 2] = np.maximum(points[:, 2], MAT_TOP + 0.0017)
    return points


def lead_part(name):
    pts = harness_lay(name)
    LEAD_POINTS[name] = pts
    root = pts[0]
    local = pts - root
    color = RED if name == "positive" else BLACK
    mass, damp = (0.0234, 2e-4) if "heavycable" in DEBUG else (0.0078, 2e-5)
    hinges = cable("c", local, 0.0016, mass, 2e-6, damp, color)
    rear = local[-1]
    # pin + rounded nose = one native capsule (axis from the insulator face to the nose-sphere centre, r 2 mm); its rear
    # hemisphere lies inside the handle. (PhysX cylinder pins stopped ~1 mm short of the shoulder.)
    hinges += plug_link("plug", rear, (1, 0, 0), hinges[-1].name, 0.008,
                        cylinders=((tuple(rear - [0.010, 0, 0]), 0.004, 0.020, color, "X"),))
    hinges[-1] = Hinge(**{**hinges[-1].__dict__, "capsules": ((tuple(rear - [0.025, 0, 0]), PIN_R, 0.010, GOLD, "X"),)})
    seg = np.linalg.norm(np.diff(local, axis=0), axis=1)
    mid_from_plug = np.cumsum(seg[::-1])[::-1] - seg / 2
    TOUCH_LINKS[name] = ["plug", "plug_relief"] + [f"c{i}" for i in np.nonzero(mid_from_plug <= WIRE_ZONE_M)[0]]
    return part(f"lead_{name}", tuple(root), boxes=(((0.0, 0.0, 0.0), (0.006, 0.006, 0.006), DARK),), mass=None,
                hinges=tuple(hinges), color=color, friction=1.0, contact_offset=0.0003, torsional_patch=0.001)


LEAD_PARTS = {n: lead_part(n) for n in LEADS}

# ---------------------------------------------------------------------------------------------------------------
# AC C5 cord: mat clamp -> PRESET seated C5 plug in the charger inlet
# ---------------------------------------------------------------------------------------------------------------
_RSW = RC @ RS_POWER  # inlet frame -> world (inlet +X = world -Y)
POWER_SOCKET_W = CHARGER + RC @ POWER_PORT
_plug_p = POWER_SOCKET_W + _RSW @ np.array([SEATED_OFFSET["power"], 0, 0])
_rear = _plug_p + _RSW @ np.array([0.032, 0, 0])
SUPPORT = np.array([0.215, -0.215, MAT_TOP + 0.0027])
_lie = MAT_TOP + 0.0027
_pw = resample([SUPPORT, [0.225, -0.16, _lie], [0.24, -0.11, _lie], [0.25, -0.079, _lie + 0.004], [0.25, -0.068, TOP + 0.03],
                [0.25, -0.064, _rear[2] - 0.035], [0.25, -0.056, _rear[2] - 0.006], _rear + [0, -0.008, 0], _rear], 0.012)
_pw[:, 2] = np.maximum(_pw[:, 2], MAT_TOP + 0.0027)
_pl = _pw - SUPPORT
_L = float(np.linalg.norm(np.diff(_pw, axis=0), axis=1).sum())
_ac = cable("c", _pl, 0.0026, 0.045 * _L, 2e-5, 1e-4, DARK)
_po = _plug_p - SUPPORT  # plug origin (inlet reference), part frame
_ac += plug_link("plug", _pl[-1], _RSW[:, 0], _ac[-1].name, 0.010, relief_r=0.0026, rate=5e-3,
                 boxes=[oriented_box(C5_BODY[0], C5_BODY[1], _RSW, _po) + (DARK,),
                        oriented_box((0.016, 0, 0), (0.008, 0.018, 0.016), _RSW, _po) + (DARK,)],
                 cylinders=((tuple(_po + _RSW @ [0.026, 0, 0]), 0.0026, 0.012, DARK, tuple(_RSW[:, 0])),))
AC_PART = part("ac_cord", tuple(SUPPORT), boxes=(((0, 0, -0.0011), (0.018, 0.016, 0.0032), DARK),), mass=None,
               hinges=tuple(_ac), color=DARK, friction=0.8, contact_offset=0.0003)
POWER_REF = _po - _pl[-1]  # plug origin relative to the plug link anchor (part frame at rest)

# ---------------------------------------------------------------------------------------------------------------
# battery (free) with its 6 cm XT60 lead (male XT60 + solder-cup sleeve, PRESET mated in the clip) and 3S XH lead
# ---------------------------------------------------------------------------------------------------------------
_bat_hinges = []
_start = BATTERY + np.array([0.0385, -0.008, 0.0])
_xt_center = CLIP + RCLIP @ np.array([SEATED_OFFSET["main"], 0, 0])
_xt_end = _xt_center + RCLIP @ np.array([XT60[0] / 2 + SLEEVE[0], 0, 0])
_wp = [_start, _start + [0.008, 0, -0.004], [0.232, 0.207, MAT_TOP + 0.0017], _xt_end + [-0.008, 0, -0.002], _xt_end]
_m = resample(_wp, 0.008)
_m[:, 2] = np.maximum(_m[:, 2], MAT_TOP + 0.0017)
_bat_hinges += cable("m", _m - BATTERY, 0.0016, 0.012, 2e-6, 2e-5, (0.70, 0.08, 0.04))
MAIN_LEAD_END = (_bat_hinges[-1].name, _m[-1] - _m[-2])  # last lead link and the lead end in its frame
# the male XT60 with its sleeve is its own rigid body (not a lead link): jointed to the lead end, and carrying the
# contact-friction element against the clip (both joints authored in _xt60_joints)
XT60_PART = part("xt60_male", tuple(_xt_center), mass=0.0055, color=YELLOW, friction=0.8, contact_offset=0.0003,
                 boxes=(((0.0, 0.0, 0.0), tuple(XT60.tolist()), YELLOW),
                        (tuple(RCLIP @ [XT60[0] / 2 + SLEEVE[0] / 2, 0, 0]), tuple(SLEEVE.tolist()), (0.12, 0.12, 0.12))))
_bstart = BATTERY + np.array([0.0385, 0.008, 0.0])
_xh_center = CHARGER + RC @ (BALANCE_PORT + [SEATED_OFFSET["balance"], 0, 0])
_xh_end = _xh_center + RC @ np.array([XH[0] / 2, 0, 0])
_b = resample([_bstart, _bstart + [0.008, 0, -0.008], [0.228, 0.190, MAT_TOP + 0.0017], [0.226, 0.155, MAT_TOP + 0.055],
               [0.225, 0.141, _xh_end[2] - 0.012], _xh_end + [0, 0.004, 0], _xh_end], 0.008)
_b[:, 2] = np.maximum(_b[:, 2], MAT_TOP + 0.0011)
_hc = _xh_center - BATTERY
_bat_hinges += cable("b", _b - BATTERY, 0.001, 0.0024, 4e-7, 1e-5, (0.2, 0.2, 0.2))
_bat_hinges += plug_link("xh", _b[-1] - BATTERY, RC[:, 0], _bat_hinges[-1].name, 0.0008, relief_r=0.001, rate=5e-4,
                         boxes=[oriented_box((0, 0, 0), XH, RC, _hc) + (WHITE,)])
XH_REF = _hc - (_b[-1] - BATTERY)
BATTERY_PART = part("battery", BATTERY, boxes=(((0, 0, 0), (0.077, 0.034, 0.026), (0.18, 0.18, 0.20)),), mass=0.1277,
                    hinges=tuple(_bat_hinges), color=(0.18, 0.18, 0.20), friction=0.8, contact_offset=0.0003,
                    visuals=(((0, 0, 0.0131), (0.071, 0.030, 0.0004), WHITE),))

OBJECTS = [MAT, RISER, CHARGER_PART, CLIP_PART, LEAD_PARTS["negative"], LEAD_PARTS["positive"], AC_PART, BATTERY_PART,
           XT60_PART]

# Reference points on each connector's moving link, in its part frame at rest relative to the link anchor
# (part frames are world-aligned for every part except the charger): (part, link, reference offset, rest rotation)
PLUGS = {
    "positive": ("lead_positive", "plug", np.array([-0.020, 0, 0]), np.eye(3)),  # insulator front face, +X = handle
    "negative": ("lead_negative", "plug", np.array([-0.020, 0, 0]), np.eye(3)),
    "power": ("ac_cord", "plug", POWER_REF, _RSW),
    "main": ("xt60_male", "body", np.zeros(3), RCLIP),
    "balance": ("battery", "xh", XH_REF, RC),
}


# ---------------------------------------------------------------------------------------------------------------
# read-only measurements (pure functions of poses)
# ---------------------------------------------------------------------------------------------------------------
def rot_angle_deg(r):
    return float(np.degrees(math.acos(float(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))))


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class Poses:
    """Snapshot of the bodies the scorer needs (env frame): name -> (pos, 3x3 rotation)."""

    def __init__(self, items):
        self.items = items

    def plug(self, name):
        """(reference point, plug-frame rotation) of a connector: plug frame +X points back toward its cable."""
        _, _, ref, r0 = PLUGS[name]
        p, r = self.items[name]
        return p + r @ ref, r @ r0

    def charger(self):
        return self.items["charger"]


def socket_frames(poses):
    cp, cr = poses.charger()
    rcw = cr  # charger body rotation already includes the yaw
    out = {n: (cp + rcw @ o, rcw) for n, o in OUTPUT_PORTS.items()}
    out["balance"] = (cp + rcw @ BALANCE_PORT, rcw)
    out["power"] = (cp + rcw @ POWER_PORT, rcw @ RS_POWER)
    out["main"] = (CLIP.copy(), RCLIP)
    return out


def insertion(poses, frames, plug, socket, axial=(-0.0025, 0.0015), lateral=0.0004, axis_only=True):
    pp, pr = poses.plug(plug)
    sp, sr = frames[socket]
    rel = sr.T @ (pp - sp)
    angle = (float(np.degrees(math.acos(float(np.clip(sr[:, 0] @ pr[:, 0], -1, 1))))) if axis_only
             else rot_angle_deg(sr.T @ pr))
    return dict(relative=rel.tolist(), angle_deg=angle,
                connected=bool(axial[0] <= rel[0] <= axial[1] and max(abs(rel[1:])) < lateral and angle < 5))


def xt60_state(poses, frames):
    pp, pr = poses.plug("main")
    sp, sr = frames["main"]
    rel = sr.T @ (pp - sp)
    angle = rot_angle_deg(sr.T @ pr)
    return dict(relative=rel.tolist(), angle_deg=angle,
                seated=bool(-0.0005 <= rel[0] <= 0.0012 and max(abs(rel[1:])) < 0.00065 and angle < 5),
                live=bool(rel[0] < LIVE_OVERLAP and math.hypot(rel[1], rel[2]) < 0.003 and angle < 20))


def pin_segment(poses, lead):
    """The pin's metal as a capsule: axis from the insulator front face to the nose-sphere centre, radius 2 mm."""
    p, r = poses.plug(lead)
    u = r[:, 0]
    return p, p - u * (PIN_LEN - PIN_R)


def seg_seg_distance(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = float(np.clip((b * f - c * e) / den, 0, 1)) if den > 1e-15 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t, s = 0.0, float(np.clip(-c / a, 0, 1))
    elif t > 1:
        t, s = 1.0, float(np.clip((b - c) / a, 0, 1))
    return float(np.linalg.norm(p1 + d1 * s - (p2 + d2 * t)))


def _point_box(pts, centre, half, rot):
    loc = (pts - centre) @ rot  # into the box frame
    q = np.abs(loc) - half
    outside = np.linalg.norm(np.maximum(q, 0), axis=-1)
    inside = np.minimum(np.max(q, axis=-1), 0)
    return outside + inside


def capsule_box_distance(a, b, radius, centre, half, rot):
    """Exact distance (negative inside) from capsule (segment a-b, radius) to an oriented box: the point-to-box
    distance is convex along the segment, so a ternary search on it is exact to the tolerance."""
    lo, hi = 0.0, 1.0
    for _ in range(40):
        m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
        d = _point_box(np.array([a + (b - a) * m1, a + (b - a) * m2]), centre, half, rot)
        if d[0] < d[1]:
            hi = m2
        else:
            lo = m1
    return float(_point_box((a + (b - a) * (lo + hi) / 2)[None], centre, half, rot)[0]) - radius


_CB = None


def _conductor_arrays():
    global _CB
    if _CB is None:
        _CB = (np.array([c[2] for c in CONDUCTORS]), np.array([c[3] for c in CONDUCTORS]),
               np.array([c[4] for c in CONDUCTORS]))
    return _CB


def conductor_touches(poses, lead):
    """Charger-frame conductors (case, the wrong jack, balance and mains pins) and the XT60 contacts touched by the
    LIVE pin of ``lead`` (gap < 0.1 mm). Its own jack is the intended conductor."""
    cp, cr = poses.charger()
    a, b = pin_segment(poses, lead)
    la, lb = cr.T @ (a - cp), cr.T @ (b - cp)
    out = []
    if np.all(np.abs(la) <= CHARGER_REACH) or np.all(np.abs(lb) <= CHARGER_REACH):
        out += _charger_touches(lead, la, lb)
    if min(np.linalg.norm(a - CLIP), np.linalg.norm(b - CLIP)) < 0.04:
        cc, ch = XT60_CAVITY
        gap = capsule_box_distance(RCLIP.T @ (a - CLIP), RCLIP.T @ (b - CLIP), PIN_R, cc, ch, np.eye(3))
        if gap < SHORT_GAP:
            out.append(dict(lead=lead, geom=f"{lead}_pin", other="xt60_contacts", kind="xt60_contact", gap_m=gap))
    return out


CHARGER_REACH = np.array([0.0725, 0.0685, 0.0175]) + 0.015  # case half extents + 15 mm (charger frame)


def _charger_touches(lead, la, lb):
    centres, halves, rots = _conductor_arrays()
    samples = la + (lb - la) * np.linspace(0, 1, 13)[:, None]
    loc = np.einsum("bij,bsi->bsj", rots, samples[None] - centres[:, None])  # (boxes, samples, 3) in box frames
    q = np.abs(loc) - halves[:, None]
    coarse = (np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)).min(-1) - PIN_R
    out = []
    for i in np.nonzero(coarse < 0.002)[0]:
        name, kind, centre, half, rot = CONDUCTORS[i]
        if kind == f"jack_{lead}":
            continue
        gap = capsule_box_distance(la, lb, PIN_R, centre, half, rot)
        if gap < SHORT_GAP:
            k = "wrong_jack" if kind.startswith("jack_") else "charger_case" if kind == "case" else kind
            out.append(dict(lead=lead, geom=f"{lead}_pin", other=name, kind=k, gap_m=gap))
    return out


def wrong_entry_now(poses, frames, lead):
    pp, pr = poses.plug(lead)
    sp, sr = frames[OTHER[lead]]
    rel = sr.T @ (pp - sp)
    axis = float(np.degrees(math.acos(float(np.clip(sr[:, 0] @ pr[:, 0], -1, 1)))))
    depth = PIN_LEN - rel[0]
    return dict(tip_depth_m=float(depth), lateral_m=float(math.hypot(rel[1], rel[2])),
                entered=bool(depth >= 0.002 and math.hypot(rel[1], rel[2]) < 0.0025 and axis < 30))


class SubstepMonitor:
    """Accumulates short-circuit evidence over EVERY physics substep while the XT60 is live (read-only)."""

    def __init__(self):
        self.run_min_live_gap = float("inf")
        self.substeps = 0
        self.live_substeps = 0
        self.reset()

    def reset(self):
        self.contacts = {}
        self.live_any = False
        self.entered = {n: False for n in LEADS}
        self.min_live_gap = float("inf")

    def sample(self, poses, t):
        self.substeps += 1
        frames = socket_frames(poses)
        if not xt60_state(poses, frames)["live"]:
            return
        self.live_any = True
        self.live_substeps += 1
        a1, b1 = pin_segment(poses, "positive")
        a2, b2 = pin_segment(poses, "negative")
        gap = seg_seg_distance(a1, b1, a2, b2) - 2 * PIN_R
        self.min_live_gap = min(self.min_live_gap, gap)
        self.run_min_live_gap = min(self.run_min_live_gap, gap)
        if gap < PIN_GAP:
            self.contacts.setdefault(("positive_pin", "negative_pin"), dict(
                lead="positive", geom="positive_pin", other="negative_pin", kind="other_pin", gap_m=gap, t=t,
                substep=True))
        for lead in LEADS:
            for c in conductor_touches(poses, lead):
                self.contacts.setdefault((c["geom"], c["other"]), dict(c, t=t, substep=True))
            if not self.entered[lead] and wrong_entry_now(poses, frames, lead)["entered"]:
                self.entered[lead] = True

    def flush(self):
        out = dict(live_any=self.live_any, contacts=list(self.contacts.values()), entered=dict(self.entered),
                   min_live_pin_gap_m=None if self.min_live_gap == float("inf") else float(self.min_live_gap))
        self.reset()
        return out


def moved(pose, start):
    return dict(shift_m=float(np.linalg.norm(pose[0] - start[0])), turn_deg=rot_angle_deg(start[1].T @ pose[1]))


def measurements(poses, start, substep=None):
    """Scoring inputs. ``substep`` (Runtime.flush) adds what happened at any physics substep of this control step."""
    frames = socket_frames(poses)
    rows = {n: insertion(poses, frames, n, n) for n in LEADS}
    xt60 = xt60_state(poses, frames)
    bal = insertion(poses, frames, "balance", "balance", axial=(-0.0005, 0.0015), lateral=0.00065, axis_only=False)
    a1, b1 = pin_segment(poses, "positive")
    a2, b2 = pin_segment(poses, "negative")
    gap = seg_seg_distance(a1, b1, a2, b2) - 2 * PIN_R
    contacts = []
    if xt60["live"]:
        for lead in LEADS:
            contacts += conductor_touches(poses, lead)
        if gap < PIN_GAP:
            contacts.append(dict(lead="positive", geom="positive_pin", other="negative_pin", kind="other_pin", gap_m=gap))
    sub = substep or dict(live_any=False, contacts=[], gripper_contacts=[], keys_pressed=False, apart_running=False,
                          min_live_pin_gap_m=None)
    seen = {(c["geom"], c["other"]) for c in contacts}
    contacts = [c for c in sub["contacts"] if (c["geom"], c["other"]) not in seen] + contacts
    return dict(live=bool(xt60["live"] or sub["live_any"]), xt60=xt60, positive=rows["positive"], negative=rows["negative"],
                crossed_positive=insertion(poses, frames, "positive", "negative"),
                crossed_negative=insertion(poses, frames, "negative", "positive"),
                power=insertion(poses, frames, "power", "power", axis_only=False),
                balance=dict(relative=bal["relative"], angle_deg=bal["angle_deg"], seated=bal["connected"]),
                metal_contacts=contacts, gripper_contacts=sub["gripper_contacts"], pin_gap_m=float(gap),
                min_live_pin_gap_step_m=sub["min_live_pin_gap_m"], keys_pressed=sub["keys_pressed"],
                xt60_apart_while_running=sub["apart_running"],
                charger_moved=moved(poses.items["charger"], start["charger"]),
                riser_moved=moved(poses.items["riser"], start["riser"]))


# ---------------------------------------------------------------------------------------------------------------
# bounded axial retention of the banana, C5 and XH plugs (verbatim law of the live-lead task)
# ---------------------------------------------------------------------------------------------------------------
class AxialContact:
    def __init__(self, capture_max=0.0021, lateral=0.00065, stiffness=1000.0, plastic=False):
        self.anchor = None
        self.previous = None
        self.force = 0.0
        self.activations = 0
        self.capture_max, self.lateral, self.stiffness, self.plastic = capture_max, lateral, stiffness, plastic
        self.deepest = None

    def axial_force(self, x, y, z, angle, dt):
        v = 0.0 if self.previous is None else (x - self.previous) / dt
        self.previous = x
        aligned = abs(y) < self.lateral and abs(z) < self.lateral and angle < np.radians(5)
        if self.anchor is None and aligned and -0.0005 <= x <= self.capture_max:
            self.anchor = x
            self.deepest = x
            self.activations += 1
        if self.anchor is not None:
            self.deepest = min(self.deepest, x)
            lost = (x - self.deepest > 0.002 or x < -0.0005) if self.plastic else abs(x - self.anchor) > 0.002
            if not aligned or lost:
                self.anchor = None
        raw = 0.0 if self.anchor is None else -self.stiffness * (x - self.anchor) - 0.02 * v
        self.force = float(np.clip(raw, -0.5, 0.5))
        if self.plastic and self.anchor is not None and abs(raw) > 0.5:
            self.anchor = x + (self.force + 0.02 * v) / self.stiffness
        return self.force


def new_retention():
    wiring = dict(capture_max=0.0015, lateral=0.0004, stiffness=2000.0, plastic=True)
    return {"balance": AxialContact(capture_max=0.0015, lateral=0.00065, stiffness=2000.0, plastic=True),
            "power": AxialContact(**wiring), "positive": AxialContact(**wiring), "negative": AxialContact(**wiring)}


def retention_forces(poses, retention, dt):
    """Equal and opposite axial forces (bounded 0.5 N) between each captured plug and the charger."""
    frames = socket_frames(poses)
    out, reaction = {}, np.zeros(3)
    for name, contact in retention.items():
        pp, pr = poses.plug(name)
        sp, sr = frames[name]
        rel = sr.T @ (pp - sp)
        angle = (math.acos(float(np.clip(sr[:, 0] @ pr[:, 0], -1, 1))) if name in LEADS
                 else math.radians(rot_angle_deg(sr.T @ pr)))
        f = contact.axial_force(*rel, angle, dt) * sr[:, 0]
        out[name] = (f, (pp + sp) / 2)
        reaction -= f
    out["charger"] = (reaction, poses.charger()[0])
    return out


# ---------------------------------------------------------------------------------------------------------------
# XT60 contact friction: D6 joint clip -> pack XT60, velocity drive on the mating axis, cap set every substep
# ---------------------------------------------------------------------------------------------------------------
FRICTION_JOINT = {}


@on_spawn
def _xt60_joints(stage, spec, prim_path, paths):
    """Friction element: a D6 joint (every DOF free) from the fixed clip to the male XT60 body with a velocity drive
    (target 0) on the clip's mating axis, cap set by set_friction. Lead end: a D6 joint from the last lead link to the
    sleeve's rear, translations locked, bending and twist on light springs (strain relief)."""
    if spec.name != "xt60_male":
        return
    from pxr import Gf, Sdf, UsdPhysics
    env = prim_path.rsplit("/", 1)[0]
    q = Gf.Quatf(0.0, 0.0, 0.0, 1.0)  # RCLIP: 180 deg about z, so the joint X axis is the clip's mating axis
    j = UsdPhysics.Joint.Define(stage, f"{paths['']}/xt60_friction")
    j.CreateBody0Rel().SetTargets([Sdf.Path(env + "/clip/body")])
    j.CreateBody1Rel().SetTargets([Sdf.Path(paths[""])])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, _xt_center - CLIP)))
    j.CreateLocalRot0Attr(q)
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(q)
    j.CreateCollisionEnabledAttr(True)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "transX")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(XT60_FRICTION_DAMPING)
    d.CreateMaxForceAttr(0.0)
    d.CreateTargetVelocityAttr(0.0)
    FRICTION_JOINT["path"] = str(j.GetPrim().GetPath())
    link, end = MAIN_LEAD_END
    c = UsdPhysics.Joint.Define(stage, f"{paths['']}/lead_end")
    c.CreateBody0Rel().SetTargets([Sdf.Path(f"{env}/battery/{link}")])
    c.CreateBody1Rel().SetTargets([Sdf.Path(paths[""])])
    c.CreateLocalPos0Attr(Gf.Vec3f(*map(float, end)))
    c.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    c.CreateLocalPos1Attr(Gf.Vec3f(*map(float, _xt_end - _xt_center)))
    c.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    c.CreateExcludeFromArticulationAttr(True)
    for dof in ("transX", "transY", "transZ"):
        lim = UsdPhysics.LimitAPI.Apply(c.GetPrim(), dof)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)  # low > high: locked
    for dof in ("rotX", "rotY", "rotZ"):
        lim = UsdPhysics.LimitAPI.Apply(c.GetPrim(), dof)
        lim.CreateLowAttr(-40.0)
        lim.CreateHighAttr(40.0)
        dr = UsdPhysics.DriveAPI.Apply(c.GetPrim(), dof)
        dr.CreateTypeAttr("force")
        dr.CreateStiffnessAttr(2e-3 * math.pi / 180)
        dr.CreateDampingAttr(5e-5 * math.pi / 180)
        dr.CreateTargetPositionAttr(0.0)


def xt60_engaged(x):
    rel = x["relative"]
    return bool(-0.001 <= rel[0] <= XT60_ENGAGE_M and max(abs(rel[1]), abs(rel[2])) < XT60_ENGAGE_LATERAL_M
                and x["angle_deg"] < XT60_ENGAGE_ANGLE_DEG)


def set_friction(engaged):
    if engaged == STATE["engaged"]:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics
    prim = get_current_stage().GetPrimAtPath(FRICTION_JOINT["path"])
    UsdPhysics.DriveAPI.Get(prim, "transX").GetMaxForceAttr().Set(float(XT60_FRICTION_N if engaged else 0.0))
    STATE["engaged"] = engaged
    STATE["engage_events"] += 1


# ---------------------------------------------------------------------------------------------------------------
# runtime: PhysX tensor views, per-substep hook (monitors, keys, charger, friction gate), per-control-step scoring
# ---------------------------------------------------------------------------------------------------------------
BODIES = {"charger": ("charger", "body"), "positive": ("lead_positive", "plug"), "negative": ("lead_negative", "plug"),
          "power": ("ac_cord", "plug"), "main": ("xt60_male", "body"), "balance": ("battery", "xh"),
          "battery": ("battery", "body"), "riser": ("riser", "body"),
          **{f"key_{k}": ("charger", f"key_{k}") for k in KEYS}}
STATE = {}
RESET_SNAPSHOT = {}  # body positions after the first reset of the process (reset diagnostics)


def _fresh_state():
    return dict(scorer=ChargeScorer(), monitor=SubstepMonitor(), retention=new_retention(), start=None, steps=0,
                sim_t=0.0, veto_event=None, trace=[], first_full=None, view=None, nonfinite=None, score=0.0, hook_s=0.0,
                charger=Charger(), switches=KeySwitches(), engaged=None, engage_events=0, lcd=None, sub=_fresh_sub())


def _fresh_sub():
    return dict(gripper_contacts={}, keys_pressed=False, apart_running=False, travel_max={k: 0.0 for k in KEYS},
                touch_max_n=0.0)


STATE.update(_fresh_state())


class View:
    """Torch PhysX views: scored bodies (articulation links included) for poses and retention forces, and a contact
    view of the live plugs and their wire (within 5 cm) against every gripper link of both arms."""

    def __init__(self, environment):
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        self.torch = torch
        base = environment.unwrapped
        self.device = base.device
        stage = get_current_stage()
        found, touch, grip = {}, {}, []
        want = {v: k for k, v in BODIES.items()}
        zone = {(f"lead_{lead}", link): f"{lead}:{link}" for lead in LEADS for link in TOUCH_LINKS[lead]}
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            parts = path.split("/")
            if "env_0" not in parts:
                continue
            key = tuple(parts[-2:])
            if key in want and want[key] not in found:
                found[want[key]] = path
            if key in zone and zone[key] not in touch:
                touch[zone[key]] = path
        for arm in ("left_robot", "right_robot"):
            links = base.scene[arm].root_view.link_paths
            links = links[0] if links and isinstance(links[0], (list, tuple)) else links
            grip += [p for p in links if p.rsplit("/", 1)[1] in GRIPPER_LINKS]
        missing = (set(BODIES) - set(found)) | (set(zone.values()) - set(touch))
        if missing or len(grip) != 2 * len(GRIPPER_LINKS):
            raise RuntimeError(f"livekit: bodies not found on the stage: {sorted(missing)}; gripper links {grip}")
        self.names = list(BODIES)
        sid = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.sim = tensors.create_simulation_view("torch", stage_id=sid)
        self.sim.set_subspace_roots("/")
        self.view = self.sim.create_rigid_body_view([found[n] for n in self.names])
        order = list(self.view.prim_paths)
        self.index = {n: order.index(found[n]) for n in self.names}
        self.ids = torch.arange(self.view.count, dtype=torch.int32, device=self.device)
        self.origin = _np(base.scene.env_origins[0])
        from partkit import pad_readback
        STATE["pad_readback"] = {k: list(v) for k, v in pad_readback(stage).items()}
        self.touch_names = sorted(touch)
        self.grip_paths = grip
        self.contacts = self.sim.create_rigid_contact_view([touch[n] for n in self.touch_names],
                                                           filter_patterns=[grip] * len(self.touch_names),
                                                           max_contact_data_count=1024)

    def poses(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(np.float64)
        return Poses({n: (tf[i, :3] - self.origin, quat_to_matrix(tf[i, 3:7])) for n, i in self.index.items()})

    def gripper_touches(self, dt):
        """{plug or wire link: N} touched by any gripper link during the last physics step."""
        f = self.contacts.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        mag = np.linalg.norm(f, axis=-1).sum(axis=-1)
        return {self.touch_names[i]: float(mag[i]) for i in range(len(mag))}

    def apply(self, forces):
        """forces: name -> (force, point) in the env frame."""
        f = np.zeros((self.view.count, 3), np.float32)
        p = np.zeros((self.view.count, 3), np.float32)
        for n, (force, point) in forces.items():
            f[self.index[n]] += force
            p[self.index[n]] = point + self.origin
        t = self.torch
        self.view.apply_forces_and_torques_at_position(t.as_tensor(f, device=self.device),
                                                       t.zeros((self.view.count, 3), device=self.device),
                                                       t.as_tensor(p, device=self.device), self.ids, True)


def _np(x):
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


def key_travel(poses):
    cp, cr = poses.charger()
    return {k: float(KEY_POS[k][2] - (cr.T @ (poses.items[f"key_{k}"][0] - cp))[2]) for k in KEYS}


@on_pre_reset
def _pre_reset():
    view = STATE.get("view")  # the PhysX tensor views stay valid across env resets
    if STATE.get("engaged"):
        set_friction(False)  # disarm before the env reset re-places the pack (re-armed on the first substep)
    restore_authored()
    STATE.update(_fresh_state())
    STATE["view"] = view


def restore_authored():
    """Environment reset only (never during a trial): every free part and articulated part back to its authored root
    pose and joint positions with zero velocity, before yam_arena's own reset and settle."""
    environment = env()
    if environment is None:
        return
    import torch
    base = environment.unwrapped
    origin = base.scene.env_origins[:1]
    for spec in OBJECTS:
        if spec.mass is None and not spec.hinges:
            continue  # kinematic, never moves
        a = base.scene[spec.name]
        root = torch.as_tensor(_t(a.data.default_root_state)).clone()
        root[:, :3] += origin
        a.write_root_pose_to_sim(root[:, :7])
        a.write_root_velocity_to_sim(torch.zeros_like(root[:, 7:]))
        if spec.hinges:
            jp = torch.as_tensor(_t(a.data.default_joint_pos)).clone()
            a.write_joint_state_to_sim(jp, torch.zeros_like(jp))
    STATE["restored"] = STATE.get("restored", 0) + 1


def _t(x):
    return getattr(x, "torch", x)


@on_physics
def _substep(environment, dt):
    t0 = time.perf_counter()
    try:
        _substep_body(environment, dt)
    finally:
        STATE["hook_s"] += time.perf_counter() - t0


def _substep_body(environment, dt):
    if STATE["view"] is None:
        STATE["view"] = View(environment)
    view = STATE["view"]
    poses = view.poses()
    for n, (p, r) in poses.items.items():
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(r))):
            STATE["nonfinite"] = STATE["nonfinite"] or dict(body=n, t=STATE["sim_t"])
            raise RuntimeError(f"livekit physics guard: non-finite pose of {n} at t={STATE['sim_t']:.4f}")
    frames = socket_frames(poses)
    x = xt60_state(poses, frames)
    set_friction(xt60_engaged(x))
    if STATE["start"] is not None:  # the scored run: sample the state left by the previous substep
        sub = STATE["sub"]
        STATE["monitor"].sample(poses, STATE["sim_t"])
        touched = view.gripper_touches(dt)
        sub["touch_max_n"] = max(sub["touch_max_n"], max(touched.values()))
        if x["live"]:
            for n, f in touched.items():
                if f > GRIPPER_CONTACT_N:
                    sub["gripper_contacts"].setdefault(n, dict(body=n, force_n=round(f, 3), t=round(STATE["sim_t"], 4)))
        travel = key_travel(poses)
        for k, v in travel.items():
            sub["travel_max"][k] = max(sub["travel_max"][k], v)
        closed = STATE["switches"].sample(dt, travel)
        sub["keys_pressed"] |= any(closed.values())
        ch = STATE["charger"]
        if ch.running and not x["seated"]:
            sub["apart_running"] = True
        if insertion(poses, frames, "power", "power", axis_only=False)["connected"]:  # the charger needs mains
            bal = insertion(poses, frames, "balance", "balance", axial=(-0.0005, 0.0015), lateral=0.00065,
                            axis_only=False)
            ch.step(dt, closed, x["seated"], bal["connected"])
    forces = retention_forces(poses, STATE["retention"], dt)
    if STATE.get("probe_n"):  # calibration fixture only (scripted force probe of the XT60 friction), never in a trial
        forces["main"] = (STATE["probe_n"] * RCLIP[:, 0], poses.plug("main")[0])
    if "noforce" not in DEBUG:
        view.apply(forces)
    STATE["sim_t"] += dt


@on_reset
def _after_reset(environment):
    view = STATE["view"] or View(environment)
    STATE["view"] = view
    poses = view.poses()
    STATE["start"] = {n: poses.items[n] for n in ("charger", "riser")}
    STATE["monitor"] = SubstepMonitor()
    STATE["sim_t"] = 0.0
    STATE["initial"] = measurements(poses, STATE["start"])
    snap = {n: p.tolist() for n, (p, _) in poses.items.items()}
    first = RESET_SNAPSHOT.setdefault("first", snap)
    STATE["reset_diff_mm"] = {n: round(float(np.linalg.norm(np.subtract(snap[n], first[n]))) * 1000, 3) for n in snap}
    STATE["initial_engaged"] = STATE["engaged"]
    _update_lcd()


def _update_lcd():
    lines = tuple(STATE["charger"].display())
    if lines != STATE["lcd"]:
        set_screen("charger", lcd_png(lines))
        STATE["lcd"] = lines


def clearance(environment, poses):
    """Smallest distance from any finger point (TCP and pad tips, 6 mm outside each pad) to the charger case, the pack,
    either banana plug's handle axis or either XT60 half."""
    pts = np.array(finger_points(environment))
    d = []
    for (c, r), half in ((poses.charger(), np.array([0.0725, 0.0685, 0.0175])),
                         (poses.items["battery"], np.array([0.0385, 0.017, 0.013]))):
        d.append(np.linalg.norm(np.maximum(np.abs((pts - c) @ r) - half, 0), axis=1).min())
    for lead in LEADS:
        p, r = poses.plug(lead)
        a, b = p, p + r[:, 0] * 0.020
        ab = b - a
        t = np.clip((pts - a) @ ab / (ab @ ab), 0, 1)
        d.append(np.linalg.norm(pts - (a + t[:, None] * ab), axis=1).min())
    d.append(np.linalg.norm(pts - poses.plug("main")[0], axis=1).min())
    d.append(np.linalg.norm(pts - CLIP, axis=1).min())
    return float(min(d))


def intact_now(poses):
    zmin = min(p[2] for p, _ in poses.items.values())
    return bool(zmin > TOP - 0.04), float(zmin)


@on_step
def _control_step(environment):
    t0 = time.perf_counter()
    try:
        _control_step_body(environment)
    finally:
        STATE["hook_s"] += time.perf_counter() - t0


def _control_step_body(environment):
    view = STATE["view"]
    if view is None or STATE["start"] is None:
        return
    base = environment.unwrapped
    dt = float(base.physics_dt) * int(getattr(base.cfg, "decimation", 1))
    poses = view.poses()
    mon = STATE["monitor"].flush()
    sub = STATE["sub"]
    STATE["sub"] = _fresh_sub()
    m = measurements(poses, STATE["start"], substep=dict(
        live_any=mon["live_any"], contacts=mon["contacts"], min_live_pin_gap_m=mon["min_live_pin_gap_m"],
        gripper_contacts=list(sub["gripper_contacts"].values()), keys_pressed=sub["keys_pressed"],
        apart_running=sub["apart_running"]))
    ch = STATE["charger"]
    clear = clearance(environment, poses)
    m.update(lcd_error=ch.page == "error", running=ch.running, clear=clear >= CLEARANCE_M)
    intact, zmin = intact_now(poses)
    sc = STATE["scorer"]
    before = sc.veto
    sc.step(dt, m, intact=intact)
    STATE["steps"] += 1
    STATE["score"] = sc.score
    _update_lcd()
    if sc.veto is not None and before is None:
        STATE["veto_event"] = dict(step=STATE["steps"], t=STATE["sim_t"], veto=sc.veto, measurements=m)
    if sc.score >= 1.0 and STATE["first_full"] is None:
        STATE["first_full"] = STATE["steps"]
    STATE["trace"].append(dict(
        step=STATE["steps"], t=round(STATE["sim_t"], 5), score=sc.score, veto=sc.veto, hold=round(sc.hold, 3),
        live=m["live"], xt60_seated=m["xt60"]["seated"], xt60_rel=m["xt60"]["relative"], engaged=STATE["engaged"],
        positive=m["positive"]["connected"], negative=m["negative"]["connected"],
        positive_rel=m["positive"]["relative"], negative_rel=m["negative"]["relative"],
        positive_angle=m["positive"]["angle_deg"], negative_angle=m["negative"]["angle_deg"],
        power=m["power"]["connected"], balance=m["balance"]["seated"], intact=intact, zmin=round(zmin, 4),
        pin_gap_m=m["pin_gap_m"], contacts=m["metal_contacts"][:3], gripper_contacts=m["gripper_contacts"][:3],
        gripper_touch_max_n=round(sub["touch_max_n"], 3), keys_pressed=m["keys_pressed"],
        key_travel_max_mm={k: round(v * 1000, 3) for k, v in sub["travel_max"].items()}, lcd=list(ch.display()),
        page=ch.page, clearance_m=round(clear, 4), charger_moved=m["charger_moved"], riser_moved=m["riser_moved"]))


def summary():
    mon = STATE["monitor"]
    return dict(score=STATE["score"], veto=STATE["scorer"].veto, steps=STATE["steps"], first_full_step=STATE["first_full"],
                veto_event=STATE["veto_event"], initial=STATE.get("initial"), initial_engaged=STATE.get("initial_engaged"),
                min_live_pin_gap_m=None if mon.run_min_live_gap == float("inf") else mon.run_min_live_gap,
                substeps=mon.substeps, live_substeps=mon.live_substeps, physics_dt=PHYSICS_DT, grip_n=GRIP_N,
                xt60_friction_n=XT60_FRICTION_N, engage_events=STATE["engage_events"],
                charger_events=STATE["charger"].events, pad_friction=PAD_FRICTION, pad_readback=STATE.get("pad_readback"),
                reset_diff_mm=STATE.get("reset_diff_mm"), restored=STATE.get("restored"), hook_wall_s=round(STATE["hook_s"], 2), debug=sorted(DEBUG))
