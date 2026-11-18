"""Seat a hinged optical flip mount on its 12.7 mm post, snug its collar clamp, and latch the filter flat in its detent.

Real product class: an optical-bench 90-degree hinged 25.4 mm (1-inch) filter flip mount (64 g matte-black anodized
split collar, 32 g captive brass cam-clamp lever, and 44 g spring-loaded hinged filter cell with coated bandpass glass
and side actuator tab) mounted on a 12.7 mm stainless optical post + slotted pedestal base (310 g) with a 28 g sprung
post staging detent and a 32 g sprung brass lower beam-path detent catch on an optical breadboard.

Bimanual physics: the flip mount starts 11.2 mm above the post's machined reference shoulder on a 520 N/m sprung staging
detent with its horizontal cam-clamp open (48 deg) and its spring-biased filter frame upright (88 deg) clear of the beam
port. Pressing the collar past the staging detent onto the post flat, snugging the cam lever, and pressing the
cantilevered filter frame down past the 110 N/m lower detent shifts and pitches an unbraced 310 g post assembly past
registration unless the second arm clamps the rear stabilizer boss.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit as mk  # noqa: E402  (must precede labkit so labkit wraps mechkit's import finder)
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- distinct PBR colours ---------------------------------------------------------------------------------------------
BREADBOARD_BLK = (0.13, 0.14, 0.16)
ANODIZED_BLK = (0.18, 0.20, 0.23)
ANODIZED_SLATE = (0.26, 0.30, 0.35)
RUBBER_DARK = (0.10, 0.11, 0.12)
STEEL_POST = (0.76, 0.79, 0.83)
STEEL_BASE = (0.58, 0.62, 0.66)
KNURL_DARK = (0.33, 0.36, 0.40)
BRASS_GOLD = (0.82, 0.65, 0.24)
FILTER_CYAN = (0.18, 0.72, 0.78)
BEAM_EMERALD = (0.16, 0.80, 0.52)
RETICLE_GOLD = (0.86, 0.68, 0.22)

# -- 1:1 optical-bench geometry (metres, kg, N, deg) ------------------------------------------------------------------
DECK_H = 0.015  # 15 mm optical breadboard deck height above tabletop (3 mm rubber feet + 12 mm plate)
DECK_Z = TOP + DECK_H
POST_X, POST_Y = 0.265, -0.025  # 12.7 mm optical post station on breadboard (0.08 mm IK on both arms!)
HINGE_Y = POST_Y - 0.026  # -0.051 m: horizontal hinge axis (along X) 26 mm in front of post axis
_RING_Y = -0.048  # 25.4 mm filter centre in holder frame -> world BEAM_Y = HINGE_Y - 0.048 = -0.099 m
BEAM_X, BEAM_Y = POST_X, HINGE_Y + _RING_Y  # (0.265, -0.099): collimated camera-calibration port centre
STAB_X_OFF, STAB_Y_OFF = 0.035, 0.160  # rear stabilizer boss at (0.300, +0.135): Left Arm yaw=-pi/2 -> 0.08mm IK!

POST_ASSY_MASS = 0.310  # 100 g 12.7 mm stainless post + 210 g slotted pedestal base = 310 g
KEEPER_MASS = 0.028  # 28 g sprung brass post staging detent (>= 25 g)
COLLAR_MASS = 0.064  # 64 g anodized split-collar mount body
CLAMP_MASS = 0.032  # 32 g captive brass horizontal cam-clamp lever (>= 25 g)
HOLDER_MASS = 0.044  # 44 g hinged 25.4 mm (1-inch) optical filter holder + coated glass (>= 25 g)
DETENT_MASS = 0.032  # 32 g sprung brass lower detent plunger (>= 25 g)

COLLAR_START_M = 0.0112  # 11.2 mm initial staging height resting on sprung brass detent (0 interpenetration, <1mm drop!)
COLLAR_MAX_M = 0.020
COLLAR_FRIC_N = 2.6  # 2.6 N smooth collar drag + sprung brass staging detent -> 0.0 mm idle creep!

CLAMP_OPEN_DEG = 48.0  # initial open angle of horizontal brass split-collar cam lever
CLAMP_FRIC_NM = 0.32  # calibrated hand-snug cam-clamp torque (~0.32 N m; vertical axis -> 0 gravity creep!)

HINGE_START_DEG = 88.0  # 90-degree class upright park position (completely clear of the vertical beam port)
HINGE_K_NM = 0.28  # upward return spring (~5.6 N at striker; snaps unlatched holder upright & tips unbraced post!)
DETENT_K_NM = 110.0  # 110 N/m brass lower detent plunger spring (low cam-in force at 1.5x mu, solid horizontal lock!)

SHOULDER_H = 0.034  # post reference shoulder flat height above breadboard deck
HINGE_Z = DECK_Z + SHOULDER_H + 0.012  # DECK_Z + 0.046 m when collar_slide == 0.0


def _quat_rx(deg: float) -> tuple:
    r = math.radians(deg)
    return (math.sin(r / 2.0), 0.0, 0.0, math.cos(r / 2.0))


def _quat_ry(deg: float) -> tuple:
    r = math.radians(deg)
    return (0.0, math.sin(r / 2.0), 0.0, math.cos(r / 2.0))


def _build_breadboard():
    """260 x 300 x 15 mm anodized optical breadboard with calibration beam port and pedestal datum channel."""
    bx0, by0 = POST_X, 0.005
    foot_h, plate_t = 0.003, 0.012
    boxes = [
        ((0.0, 0.0, foot_h + plate_t / 2), (0.260, 0.300, plate_t), BREADBOARD_BLK),
        # Collimated vertical camera-calibration source bezel & emerald optical aperture at (BEAM_X, BEAM_Y)
        ((BEAM_X - bx0, BEAM_Y - by0, DECK_H + 0.002), (0.044, 0.044, 0.004), ANODIZED_SLATE),
        ((BEAM_X - bx0, BEAM_Y - by0, DECK_H + 0.0045), (0.026, 0.026, 0.002), BEAM_EMERALD),
        ((BEAM_X - bx0, BEAM_Y - by0, DECK_H + 0.0058), (0.028, 0.002, 0.001), RETICLE_GOLD),
        ((BEAM_X - bx0, BEAM_Y - by0, DECK_H + 0.0058), (0.002, 0.028, 0.001), RETICLE_GOLD),
        # Side optical reference emitter housing on -X edge (outside arm paths)
        ((-0.108, BEAM_Y - by0, DECK_H + 0.018), (0.026, 0.036, 0.036), ANODIZED_BLK),
        ((-0.093, BEAM_Y - by0, DECK_H + 0.022), (0.006, 0.016, 0.016), BRASS_GOLD),
        # Precision pedestal registration channel (6.5 mm lateral clearance for +-5mm spawn jitter, open along Y!)
        ((-0.0240, 0.020, DECK_H + 0.005), (0.009, 0.120, 0.010), ANODIZED_SLATE),
        ((0.0240, 0.020, DECK_H + 0.005), (0.009, 0.120, 0.010), ANODIZED_SLATE),
        ((0.015, 0.182, DECK_H + 0.005), (0.068, 0.008, 0.010), RETICLE_GOLD),
    ]
    for fx in (-0.114, 0.114):
        for fy in (-0.132, 0.132):
            boxes.append(((fx, fy, foot_h / 2), (0.022, 0.022, foot_h), RUBBER_DARK))
    for gx in (-0.085, 0.085):
        for gy in (-0.105, -0.035, 0.035, 0.105):
            boxes.append(((gx, gy, DECK_H + 0.0005), (0.007, 0.007, 0.001), STEEL_BASE))
    return labkit.part("breadboard", (bx0, by0, TOP), fixed=True, friction=0.10, color=BREADBOARD_BLK, boxes=boxes)


BREADBOARD = _build_breadboard()
kit = mk.Kit()

# -- 1. 310 g 12.7 mm stainless optical post + slotted pedestal base (front fulcrum at y_rel = -26 mm!) ---------------
kit.part("post_assy", (POST_X, POST_Y, DECK_Z), [
    # Slotted pedestal base contacts deck over y_rel = [-0.026, +0.172] (static CoM behind -26 mm; Beat 3 tips unbraced!)
    mk.box((0.024, 0.198, 0.010), at=(0.0, 0.073, 0.005), color=STEEL_BASE),
    mk.box((0.046, 0.034, 0.010), at=(0.018, STAB_Y_OFF, 0.005), color=STEEL_BASE),
    # Cantilevered side detent arm at x_rel = +0.020 (clears vertical beam port at x_rel = 0 & suspended 14 mm high!)
    mk.box((0.008, 0.102, 0.008), at=(0.020, -0.072, 0.018), color=ANODIZED_BLK),
    mk.box((0.048, 0.012, 0.008), at=(0.0, -0.118, 0.018), color=ANODIZED_BLK),
    mk.box((0.024, 0.024, 0.024), at=(0.0, 0.0, 0.022), color=STEEL_POST),  # machined post shoulder (top z=0.034)
    mk.box((0.026, 0.026, 0.003), at=(0.0, 0.0, 0.0325), color=BRASS_GOLD),  # gold reference shoulder witness flat
    mk.box((0.0127, 0.0127, 0.064), at=(0.0, 0.0, 0.064), color=STEEL_POST),  # 12.7 mm vertical optical post
    mk.box((0.009, 0.015, 0.064), at=(0.0, 0.0, 0.064), color=STEEL_POST, yaw=math.pi / 4),
    # Rear knurled stabilizer boss at (0.300, +0.135): 28 mm X thickness, top z=0.056 (< shank bevel!) -> 0.0mm shift!
    mk.box((0.024, 0.026, 0.028), at=(STAB_X_OFF, STAB_Y_OFF, 0.024), color=STEEL_BASE),
    mk.box((0.028, 0.032, 0.020), at=(STAB_X_OFF, STAB_Y_OFF, 0.046), color=KNURL_DARK),
    # Horizontal lower stop anvil behind beam port supporting filter holder flat at 0.0 deg (top z = 0.041)
    mk.box((0.026, 0.010, 0.019), at=(0.0, -0.102, 0.0315), color=ANODIZED_SLATE),
    # Lower detent plunger guide housing at y_rel = -0.124
    mk.box((0.026, 0.014, 0.028), at=(0.0, -0.124, 0.036), color=ANODIZED_BLK),
], mass=POST_ASSY_MASS, color=STEEL_POST)

# -- 2. 28 g sprung brass staging detent on post_assy (-X side, 22-deg non-locking taper + visible indicator flag!) ----
KEEPER_ORIGIN = (POST_X - 0.0163, POST_Y, DECK_Z + SHOULDER_H + 0.011)
kit.part("collar_keeper", KEEPER_ORIGIN, [
    mk.box((0.012, 0.014, 0.014), at=(-0.008, 0.0, 0.0), color=BRASS_GOLD),
    mk.box((0.038, 0.038, 0.006), at=(-0.032, 0.060, -0.004), color=BRASS_GOLD),  # external detent release paddle
    mk.box((0.006, 0.012, 0.012), at=(0.0, 0.0, 0.0), color=BRASS_GOLD, quat=_quat_ry(-22.0)),
], mass=KEEPER_MASS, color=BRASS_GOLD)
kit.slider("keeper_slide", "post_assy", "collar_keeper", axis=(-1, 0, 0),
           lower=0.0, upper=0.010, start=0.0,
           stiffness=520.0, rest=0.0, damping=8.0, friction=0.0)

# -- 3. 64 g matte-black anodized split-collar mount body (slides 11 mm down post to z_rel = SHOULDER_H) --------------
kit.part("mount_collar", (POST_X, POST_Y, DECK_Z + SHOULDER_H), [
    mk.box((0.028, 0.028, 0.026), at=(0.0, 0.0, 0.013), color=ANODIZED_BLK),
    mk.box((0.014, 0.020, 0.016), at=(0.0, -0.018, 0.012), color=ANODIZED_BLK),
    mk.box((0.014, 0.012, 0.016), at=(0.018, 0.008, 0.014), color=ANODIZED_SLATE),
    # Upper press crown offset +6 mm toward +Y so gripper clears 88-deg upright filter frame (top z = +0.073)
    mk.box((0.028, 0.036, 0.014), at=(0.0, 0.006, 0.063), color=ANODIZED_SLATE),
    mk.box((0.030, 0.038, 0.004), at=(0.0, 0.006, 0.071), color=BRASS_GOLD),
], mass=COLLAR_MASS, color=ANODIZED_BLK)
kit.slider("collar_slide", "post_assy", "mount_collar", axis=(0, 0, 1),
           lower=0.0, upper=COLLAR_MAX_M, start=COLLAR_START_M,
           stiffness=0.0, rest=0.0, damping=6.0, friction=COLLAR_FRIC_N)

# -- 4. 32 g horizontal brass split-collar cam lever (vertical axis -> 0.00 deg gravity creep in idle!) ---------------
CLAMP_PIVOT = (POST_X + 0.024, POST_Y + 0.008, DECK_Z + SHOULDER_H + 0.016)
kit.part("clamp_lever", CLAMP_PIVOT, [
    mk.box((0.012, 0.014, 0.014), at=(0.0, 0.0, 0.0), color=BRASS_GOLD),
    mk.box((0.044, 0.022, 0.012), at=(0.026, 0.0, 0.0), color=BRASS_GOLD),
    mk.box((0.032, 0.034, 0.022), at=(0.052, 0.0, 0.0), color=KNURL_DARK),
    mk.box((0.034, 0.026, 0.004), at=(0.052, 0.0, 0.012), color=BRASS_GOLD),
], mass=CLAMP_MASS, color=BRASS_GOLD)
kit.hinge("clamp_screw", "mount_collar", "clamp_lever", anchor=CLAMP_PIVOT, axis=(0, 0, -1),
          lower_deg=0.0, upper_deg=55.0, start_deg=CLAMP_OPEN_DEG,
          stiffness=0.0, rest_deg=0.0, damping=0.22, friction=CLAMP_FRIC_NM)

# -- 5. 44 g spring-loaded 90-deg hinged 25.4 mm optical filter holder with side flip tab (-X side!) ------------------
HINGE_PIVOT = (POST_X, HINGE_Y, HINGE_Z)
_holder_boxes = [
    mk.box((0.022, 0.022, 0.008), at=(0.0, -0.017, 0.0), color=ANODIZED_BLK),
    mk.box((0.018, 0.018, 0.003), at=(0.0, _RING_Y, 0.0), color=FILTER_CYAN),
    mk.box((0.018, 0.018, 0.003), at=(0.0, _RING_Y, 0.0), color=FILTER_CYAN, yaw=math.pi / 4),
    # Side knurled press block: 36 x 42 x 20 mm (x in [-52,-16]mm clear of anvil, z in [-4,+16]mm -> zero interference!)
    mk.box((0.036, 0.042, 0.020), at=(-0.034, -0.058, 0.006), color=KNURL_DARK),
    mk.box((0.022, 0.008, 0.006), at=(0.0, -0.084, 0.0), color=BRASS_GOLD),  # striker bar: y_rel=[-0.114,-0.106], z=[-3,+3]mm
]
for _k in range(8):
    _a = _k * math.pi / 4.0
    _r = 0.0150
    _holder_boxes.append(mk.box((0.0125, 0.0045, 0.009),
                                at=(_r * math.cos(_a), _RING_Y + _r * math.sin(_a), 0.0),
                                color=ANODIZED_BLK if _k % 2 == 0 else BRASS_GOLD, yaw=_a + math.pi / 2))
kit.part("filter_holder", HINGE_PIVOT, _holder_boxes, mass=HOLDER_MASS, color=ANODIZED_BLK)
kit.hinge("flip_hinge", "mount_collar", "filter_holder", anchor=HINGE_PIVOT, axis=(-1, 0, 0),
          lower_deg=0.0, upper_deg=90.0, start_deg=HINGE_START_DEG,
          stiffness=HINGE_K_NM, rest_deg=89.0, damping=0.14, friction=0.0)

# -- 6. 32 g sprung brass lower detent plunger on post_assy (16-deg low-drag cam + horizontal catch lip!) -------------
DETENT_ORIGIN = (POST_X, POST_Y - 0.1180, HINGE_Z)
kit.part("detent_plunger", DETENT_ORIGIN, [
    mk.box((0.018, 0.012, 0.014), at=(0.0, -0.005, 0.004), color=BRASS_GOLD),
    mk.box((0.038, 0.036, 0.008), at=(0.028, -0.014, 0.002), color=BRASS_GOLD),  # +X external thumb-release paddle
    # Horizontal catch lip: y_rel in [-0.1185, -0.1105] (3.5 mm overlap on striker [-0.114, -0.106]), bottom z = +3.55 mm
    mk.box((0.020, 0.008, 0.0025), at=(0.0, 0.0035, 0.0048), color=BRASS_GOLD),
    # 16-deg-from-vertical cam ramp (cos 16 - 1.65*sin 16 = 0.51 > 0 -> effortless snap even at fric1.5!)
    mk.box((0.020, 0.006, 0.014), at=(0.0, 0.0032, 0.0105), color=BRASS_GOLD, quat=_quat_rx(16.0)),
], mass=DETENT_MASS, color=BRASS_GOLD)
kit.slider("detent_slide", "post_assy", "detent_plunger", axis=(0, -1, 0),
           lower=0.0, upper=0.014, start=0.0,
           stiffness=DETENT_K_NM, rest=0.0, damping=6.0, friction=0.0)

OBJECTS = [BREADBOARD, *kit.objects()]

# -- pure stateless end-state scoring thresholds (every number in task.json scoring appears here as a literal) ---------
POST_POS_TOL = 0.016  # 16 mm: optical post pedestal stays registered in the breadboard datum pocket
POST_Z_TOL = 0.006  # 6 mm: optical post pedestal vertical seating tolerance on breadboard deck
POST_TILT_DEG = 5.0  # 5 deg: optical post stays vertical on the breadboard
COLLAR_SEAT_TOL = 0.003  # 3 mm: split collar seated flush against the post's machined flat shoulder (starts at 11 mm)
CLAMP_SNUG_DEG = 15.0  # 15 deg: collar clamp lever swung inward snug from 48 deg open
HOLDER_PARTIAL_DEG = 45.0  # 45 deg: swung past halfway from 88 deg upright park
HOLDER_FLAT_DEG = 6.0  # 6 deg: filter holder held flat across the beam path by the lower detent
BEAM_ALIGN_TOL = 0.016  # 16 mm: 25.4 mm optical filter centred over the calibration beam aperture
CLEAR_DIST = 0.120  # 120 mm: both grippers clear of post, rear stabilizer boss, collar, clamp lever, and holder
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # 3 s continuous hold at 1.0 enforced statelessly by the runner (60 steps at 20 Hz)
PASS_SCORE = 1.0 - 1e-9


def _tilt_deg(quat_xyzw) -> float:
    r = mk.quat_to_mat(quat_xyzw)
    return math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))


def post_registered(objects: dict) -> bool:
    p, q = kit.pose(objects, "post_assy")
    return (math.hypot(float(p[0] - POST_X), float(p[1] - POST_Y)) <= POST_POS_TOL
            and abs(float(p[2] - DECK_Z)) <= POST_Z_TOL
            and _tilt_deg(q) <= POST_TILT_DEG)


def collar_seated(objects: dict) -> bool:
    return float(kit.q(objects, "collar_slide")) <= COLLAR_SEAT_TOL


def clamp_snugged(objects: dict) -> bool:
    return float(kit.q_deg(objects, "clamp_screw")) <= CLAMP_SNUG_DEG


def holder_angle_deg(objects: dict) -> float:
    return float(kit.q_deg(objects, "flip_hinge"))


def filter_in_beam(objects: dict) -> bool:
    fp = kit.point(objects, "filter_holder", (0.0, _RING_Y, 0.0))
    return (holder_angle_deg(objects) <= HOLDER_FLAT_DEG
            and math.hypot(float(fp[0] - BEAM_X), float(fp[1] - BEAM_Y)) <= BEAM_ALIGN_TOL)


def hands_clear(objects: dict, tcps=None) -> bool:
    pts = mk.tcps() if tcps is None else tcps
    if pts is None or len(pts) < 2:
        return False
    anchors = [
        kit.pose(objects, "post_assy")[0],
        kit.point(objects, "post_assy", (STAB_X_OFF, STAB_Y_OFF, 0.046)),
        kit.pose(objects, "mount_collar")[0],
        kit.point(objects, "clamp_lever", (0.048, 0.0, 0.0)),
        kit.point(objects, "filter_holder", (0.0, _RING_Y, 0.0)),
        kit.point(objects, "filter_holder", (-0.028, -0.062, 0.001)),
    ]
    return all(float(np.linalg.norm(np.asarray(t, float) - a)) > CLEAR_DIST for t in pts for a in anchors)


def components(objects: dict, tcps=None) -> dict:
    """Pure observable physical end-state predicates (zero step-accumulated score state)."""
    if "post_assy" not in objects:
        return {
            "post_registered": False, "collar_seated": False, "clamp_snugged": False,
            "holder_partial": False, "filter_in_beam": False, "hands_clear": False,
        }
    return {
        "post_registered": post_registered(objects),
        "collar_seated": collar_seated(objects),
        "clamp_snugged": clamp_snugged(objects),
        "holder_partial": holder_angle_deg(objects) <= HOLDER_PARTIAL_DEG,
        "filter_in_beam": filter_in_beam(objects),
        "hands_clear": hands_clear(objects, tcps),
    }


def score(objects: dict, tcps=None) -> float:
    """Judged from pure physical end-state geometry; Possible scores 0.0, 0.25, 0.50, 0.75, 1.0."""
    c = components(objects, tcps)
    if not c["post_registered"]:
        return 0.0
    collar_done = c["collar_seated"] and c["clamp_snugged"]
    if not collar_done and not c["holder_partial"]:
        return 0.0
    if not collar_done:
        return 0.25
    if not c["filter_in_beam"]:
        return 0.50
    if not c["hands_clear"]:
        return 0.75
    return 1.0


def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    if "post_assy" not in objects:
        return 1.0
    return float(kit.q(objects, "collar_slide") + math.radians(holder_angle_deg(objects)) * 0.05)


# -- open-loop bimanual scripted demo (scene constants only) ----------------------------------------------------------
def demo():
    """Left gripper stabilizes the rear pedestal boss; right seats the collar, snugs the clamp, and latches the filter."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    rl = pitch_demo.rotation(-math.pi / 2.0)
    r0 = pitch_demo.rotation(0.0)

    lx, ly, lz = POST_X + STAB_X_OFF, POST_Y + STAB_Y_OFF, DECK_Z + 0.052
    z_hov = TOP + 0.148

    # Beat 1: press split-collar crown straight down past sprung staging detent onto post flat shoulder
    cx, cy = POST_X, POST_Y + 0.012
    cz_top = DECK_Z + SHOULDER_H + COLLAR_START_M + 0.102
    cz_press = DECK_Z + SHOULDER_H + 0.066

    # Beat 2: push horizontal brass cam lever in +Y from 48 deg open (y = POST_Y - 0.028) to ~0 deg snugged
    kx = POST_X + 0.062
    ky_pre = POST_Y - 0.056
    ky_snug = POST_Y + 0.017
    kz_cam = DECK_Z + SHOULDER_H + 0.026

    # Beat 3: at side press block x = POST_X - 0.034, sweep 88-deg upright tab forward & press top face flat!
    fx = POST_X - 0.034
    fy_back, fz_back = HINGE_Y + 0.036, HINGE_Z + 0.064
    fy_mid, fz_mid = HINGE_Y - 0.036, HINGE_Z + 0.056
    fy_lat, fz_lat = HINGE_Y - 0.056, HINGE_Z + 0.001

    seq = [
        (0.6, (lx, ly, z_hov), o, rl, (cx, cy, z_hov), o, "hover over stabilizer boss and collar crown"),
        (0.6, (lx, ly, lz), o, rl, (cx, cy, cz_top), c, "descend left over rear boss and right above collar"),
        (0.7, (lx, ly, lz), c, rl, (cx, cy, cz_top), c, "clamp pedestal boss steady along X"),
        (0.9, (lx, ly, lz), c, rl, (cx, cy, cz_press), c, "press split collar past staging detent to post flat"),
        (0.6, (lx, ly, lz), c, rl, (cx, cy, z_hov), c, "lift straight up clear of seated collar"),
        (0.6, (lx, ly, lz), c, rl, (kx, ky_pre, z_hov), c, "align in front of open horizontal cam lever"),
        (0.6, (lx, ly, lz), c, rl, (kx, ky_pre, kz_cam), c, "descend beside open brass cam lever"),
        (0.9, (lx, ly, lz), c, rl, (kx, ky_snug, kz_cam), c, "sweep horizontal cam lever snug against collar"),
        (0.6, (lx, ly, lz), c, rl, (kx, ky_snug, z_hov), c, "lift straight up clear of snugged cam lever"),
        (0.6, (lx, ly, lz), c, rl, (fx, fy_back, z_hov), c, "align behind side flip tab of 88-deg filter holder"),
        (0.6, (lx, ly, lz), c, rl, (fx, fy_back, fz_back), c, "descend behind upright side flip tab"),
        (0.8, (lx, ly, lz), c, rl, (fx, fy_mid, fz_mid), c, "sweep filter holder forward past 45 deg"),
        (1.1, (lx, ly, lz), c, rl, (fx, fy_lat, fz_lat), c, "press filter holder flat past sprung lower detent"),
        (0.7, (lx, ly, lz), c, rl, (fx, fy_lat, z_hov), c, "lift right straight up so detent holds filter flat"),
        (0.7, (lx, ly, lz), o, rl, (fx, -0.22, z_hov), o, "release pedestal boss and swing right clear"),
        (0.7, (lx, ly, z_hov), o, rl, (0.18, -0.30, TOP + 0.17), o, "lift left straight up clear"),
        (0.8, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, "park both arms clear"),
        (4.2, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, "hold 3 s post-release"),
    ]
    left = [Waypoint(d, lp, lf, lab, rot=lr) for d, lp, lf, lr, _, _, lab in seq]
    right = [Waypoint(d, rp, rf, lab, rot=r0) for d, _, _, _, rp, rf, lab in seq]
    return {"left": left, "right": right}
