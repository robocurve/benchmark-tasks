"""On the station table: a full-scale interior door-and-jamb junction mock-up fixed at x 37.5 cm, y +5.0 cm
(a matte slate-blue 1-3/4 in exterior door slab with panel moulding and a brass deadbolt thumb-turn below, a 3 mm
dark reveal seam, and a warm-white door jamb with proud casing). Mounted across the reveal seam at height 12.5 cm
above the table is a full-size Prime-Line U 10827 satin-nickel door reinforcement lock. When locked (0 deg), its
spring-loaded extruded aluminum stop block overlaps the closed door face with a 3 mm door gap and its catch lug sits
behind the jamb bracket's rigid dark-steel anti-swing retainer arc, so pulling straight back or swinging without
retracting jams at about 11 deg. To unlock, pinch the block's recessed top and bottom finger grooves (66.2 mm span),
slide the block laterally away from the jamb against its return spring to clear the retainer arc, sustain that
retraction while swinging 90 deg outward clear of the door, let the spring seat the block into the open rest pocket
against the jamb, and release clear.

Requires -E object_jitter=0: the lock fixture is fixed at the exact layout coordinates.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from closedloop import grip_rot, placeholder_demo
import mechkit
from mechkit import Joint, Mech, box, cyl, extrude, on_reset, on_step, tube
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])  # 0.7475 m
SCORE_OVER = "end"
SUCCESS_HOLD_S = 8.0

# ---------------------------------------------------------------------------------------------------------------------
# Layout and full-scale dimensions (metres, station frame: x forward, y left, z up)
# Shifted to (0.375, +0.050, TOP) with Z_LOCK = 0.125 m so the right arm sweeps the full 90-deg arc with >8 deg
# joint-4 margin and <0.05 mm IK error while keeping all active work inside x 0.16..0.38 m, |y| <= 0.24 m.

DOOR_X = 0.375
DOOR_Y = 0.050
Z_LOCK = 0.125

DOOR_THICK = 0.04445  # 1-3/4 in exterior door slab
DOOR_W_Y = 0.160      # ly = -0.020 .. +0.140 (world y = +0.030 .. +0.190)
DOOR_H = 0.220        # z = 0 .. 0.220 (top at TOP + 0.220 = 0.9675 m)

REVEAL_Y = -0.0215    # 3 mm dark reveal between door edge (ly = -0.020) and jamb (ly = -0.023)
JAMB_DEPTH = 0.060
JAMB_W_Y = 0.107      # ly = -0.130 .. -0.023 (world y = -0.080 .. +0.027)
CASING_THICK = 0.006  # proud white colonial casing at ly = -0.130 .. -0.056 (4 mm clear of open block at ly = -0.052)
CASING_W_Y = 0.074

# Prime-Line U 10827 jamb plate and vertical hinge axis (local frame; door face at lx = 0)
FLANGE_DEPTH = 0.0556  # 2-3/16 in mounting plate depth
FLANGE_W_Y = 0.025
FLANGE_H = 0.0762      # 3.0 in height
HINGE_AT = (-0.012, -0.036, Z_LOCK)

# Externalized anti-swing retainer arc on BASE (models the U 10827 internal stop lip and open rest pocket)
RETAINER_R_IN = 0.006
RETAINER_R_OUT = 0.0255
RETAINER_H = 0.040
RETAINER_A0_DEG = 111.0
RETAINER_A1_DEG = 159.0

# Moving stop block (Prime-Line U 10827: 3 in / 76.2 mm tall, 1-3/8 in / 35.0 mm wide, 1 in / 25.0 mm projection)
BLOCK_X0 = -0.016
BLOCK_X1 = 0.009        # 3 mm gap to closed door face (lx_door = +0.012 in hinge frame)
BLOCK_DEPTH = BLOCK_X1 - BLOCK_X0  # 0.025 m
BLOCK_Y0 = 0.0285       # 3.0 mm radial gap outside RETAINER_R_OUT = 25.5 mm at s = 0
BLOCK_Y1 = 0.0635       # 35.0 mm width (1-3/8 in)
BLOCK_Y_MID = 0.5 * (BLOCK_Y0 + BLOCK_Y1)  # 0.0460 m
BLOCK_HALF_H = 0.0381   # 76.2 mm / 2
GROOVE_HALF_H = 0.0331  # 66.2 mm / 2 (5.0 mm deep recessed top and bottom finger grooves)
SHOULDER_W = 0.0030     # 3.0 mm rear y-end shoulders leaving a 29.0 mm pocket around the 15.5 mm pad tips
SHOULDER_X0 = 0.000     # shoulders occupy rear 9 mm (lx 0..+9 mm); front 16 mm fully open across 35 mm for pad entry
BEVEL = 0.0025          # 2.5 mm front-edge extrusion chamfer

LUG_Y = 0.0185          # catch lug center in hinge frame: inner face at 15.5 + s (rests on cam at s = 9.75..10.0 mm)
LUG_SIZE = (0.0054, 0.006, 0.048)

HINGE_LOW = 0.0
HINGE_HIGH = math.pi / 2.0  # 90.0 deg
HINGE_HOLD = 0.03           # N m vertical-axis hinge friction (zero gravity load, no creep)
HINGE_DAMP = 0.005
HINGE_MASS = 0.020          # kg

SLIDE_LOW = 0.0
SLIDE_HIGH = 0.015          # 15.0 mm lateral spring travel
SLIDE_SPRING = 400.0        # N/m (6.0 N at 15 mm full travel; 5.6 N at 14 mm working retraction)
SLIDE_DAMP = 8.0            # N s/m
SLIDE_MASS = 0.120          # kg (hollowed extruded aluminum stop block; full U 10827 lock ~170 g)
SLIDE_FRICTION = 0.95

# High-contrast sRGB colours for 320x176 visibility
DOOR_BLUE = (0.23, 0.35, 0.49)
DOOR_TRIM = (0.15, 0.24, 0.35)
REVEAL_DARK = (0.07, 0.08, 0.10)
JAMB_WHITE = (0.93, 0.92, 0.89)
BRASS = (0.78, 0.62, 0.23)
BRASS_BRIGHT = (0.87, 0.72, 0.30)
NICKEL_BASE = (0.70, 0.73, 0.77)
NICKEL_BLOCK = (0.84, 0.86, 0.89)
DARK_STEEL = (0.30, 0.33, 0.38)
GROOVE_DARK = (0.20, 0.22, 0.26)
SCREW_HEAD = (0.50, 0.53, 0.58)
MARK_DARK = (0.14, 0.16, 0.20)


def _ghost(shape):
    return dataclasses.replace(shape, collide=False)


# ---------------------------------------------------------------------------------------------------------------------
# Procedural mechkit geometry (no third-party assets)

RETAINER_ARC = tube(
    HINGE_AT,
    r_in=RETAINER_R_IN,
    r_out=RETAINER_R_OUT,
    height=RETAINER_H,
    color=DARK_STEEL,
    axis="z",
    sides=24,
    a0=math.radians(RETAINER_A0_DEG),
    a1=math.radians(RETAINER_A1_DEG),
)

CATCH_LUG = box((0.0, LUG_Y, 0.0), LUG_SIZE, DARK_STEEL)

# Beveled extruded aluminum stop block core (66.2 mm tall across the top/bottom finger grooves)
_CORE_XY = [
    (BLOCK_X0 + BEVEL, BLOCK_Y0),
    (BLOCK_X1, BLOCK_Y0),
    (BLOCK_X1, BLOCK_Y1),
    (BLOCK_X0 + BEVEL, BLOCK_Y1),
    (BLOCK_X0, BLOCK_Y1 - BEVEL),
    (BLOCK_X0, BLOCK_Y0 + BEVEL),
]
STOP_CORE = extrude((0.0, 0.0, 0.0), _CORE_XY, 2.0 * GROOVE_HALF_H, NICKEL_BLOCK, axis="z")

# Four 3.0 mm rear y-end groove shoulders (5.0 mm tall, rear 9 mm along x) leaving the front 16 mm completely open
# across 35 mm for clean wide-pad entry and a 29.0 mm rear pocket around the 15.5 mm pad tips for positive pull registration
_SH_X = 0.5 * (SHOULDER_X0 + BLOCK_X1)
_SH_DX = BLOCK_X1 - SHOULDER_X0         # 9.0 mm
_SH_H = BLOCK_HALF_H - GROOVE_HALF_H    # 5.0 mm
_SH_Z = GROOVE_HALF_H + 0.5 * _SH_H     # 35.6 mm
STOP_SHOULDERS = (
    box((_SH_X, BLOCK_Y0 + 0.5 * SHOULDER_W, +_SH_Z), (_SH_DX, SHOULDER_W, _SH_H), NICKEL_BLOCK),
    box((_SH_X, BLOCK_Y1 - 0.5 * SHOULDER_W, +_SH_Z), (_SH_DX, SHOULDER_W, _SH_H), NICKEL_BLOCK),
    box((_SH_X, BLOCK_Y0 + 0.5 * SHOULDER_W, -_SH_Z), (_SH_DX, SHOULDER_W, _SH_H), NICKEL_BLOCK),
    box((_SH_X, BLOCK_Y1 - 0.5 * SHOULDER_W, -_SH_Z), (_SH_DX, SHOULDER_W, _SH_H), NICKEL_BLOCK),
)

# High-contrast dark recessed groove inlays (look-only) for crisp 320x176 readability
GROOVE_INLAYS = (
    _ghost(box((-0.0035, BLOCK_Y_MID, +(GROOVE_HALF_H + 0.00025)), (0.021, 0.028, 0.0005), GROOVE_DARK)),
    _ghost(box((-0.0035, BLOCK_Y_MID, -(GROOVE_HALF_H + 0.00025)), (0.021, 0.028, 0.0005), GROOVE_DARK)),
)

# Upper and lower yoke arms bridging the hinge pin to the stop block (4.0 mm vertically clear of retainer arc and bridge)
YOKE_TOP = box((-0.001, 0.012, +0.030), (0.018, 0.033, 0.012), NICKEL_BLOCK)
YOKE_BOT = box((-0.001, 0.012, -0.030), (0.018, 0.033, 0.012), NICKEL_BLOCK)

# Look-only front door visual landmarks: raised panel moulding, brass deadbolt rose + thumb-turn, 3 mm reveal strip,
# mounting screws, and top protractor reference marks (20 deg and 90 deg) on the jamb top.
_DOOR_VISUALS = (
    # 3 mm dark reveal seam between closed door slab and white jamb
    _ghost(box((0.001, REVEAL_Y, DOOR_H / 2.0), (0.004, 0.003, DOOR_H), REVEAL_DARK)),
    # Matte door panel moulding rails on the lower/right door face
    _ghost(box((-0.0006, 0.060, 0.012), (0.0012, 0.120, 0.006), DOOR_TRIM)),
    _ghost(box((-0.0006, 0.060, 0.204), (0.0012, 0.120, 0.006), DOOR_TRIM)),
    _ghost(box((-0.0006, 0.120, DOOR_H / 2.0), (0.0012, 0.006, 0.192), DOOR_TRIM)),
    # Warm brass deadbolt escutcheon rose and vertical thumb-turn below the reinforcement lock
    _ghost(cyl((-0.0035, 0.060, 0.048), 0.027, 0.007, BRASS, axis="x", sides=24)),
    _ghost(cyl((-0.0085, 0.060, 0.048), 0.010, 0.005, BRASS_BRIGHT, axis="x", sides=16)),
    _ghost(box((-0.0135, 0.060, 0.048), (0.008, 0.007, 0.028), BRASS_BRIGHT)),
    # Mounting screws on jamb flange inner face (visible in top/wrist views)
    _ghost(cyl((0.028, -0.023, Z_LOCK + 0.025), 0.0035, 0.002, SCREW_HEAD, axis="y", sides=12)),
    _ghost(cyl((0.028, -0.023, Z_LOCK), 0.0035, 0.002, SCREW_HEAD, axis="y", sides=12)),
    _ghost(cyl((0.028, -0.023, Z_LOCK - 0.025), 0.0035, 0.002, SCREW_HEAD, axis="y", sides=12)),
    # Subtle top protractor reference lines at 20 deg and 90 deg on top of the jamb plate/post
    _ghost(box((0.028, -0.036, Z_LOCK + FLANGE_H / 2.0 + 0.0005), (0.042, 0.002, 0.001), MARK_DARK)),
)

_BASE_SHAPES = (
    # Closed 1-3/4 in exterior door slab (lx = 0 .. +0.04445, ly = -0.020 .. +0.140, z = 0 .. 0.220)
    box((DOOR_THICK / 2.0, 0.060, DOOR_H / 2.0), (DOOR_THICK, DOOR_W_Y, DOOR_H), DOOR_BLUE),
    # White door jamb post (lx = 0 .. +0.060, ly = -0.130 .. -0.023, z = 0 .. 0.220)
    box((JAMB_DEPTH / 2.0, -0.0765, DOOR_H / 2.0), (JAMB_DEPTH, JAMB_W_Y, DOOR_H), JAMB_WHITE),
    # Proud colonial jamb casing (lx = -0.006 .. 0, ly = -0.130 .. -0.056)
    box((-CASING_THICK / 2.0, -0.093, DOOR_H / 2.0), (CASING_THICK, CASING_W_Y, DOOR_H), JAMB_WHITE),
    # Rear weighted mounting foot behind door and jamb
    box((0.084, 0.005, 0.008), (0.080, 0.270, 0.016), JAMB_WHITE),
    # Prime-Line U 10827 jamb mounting flange (2-3/16 x 3 in)
    box((FLANGE_DEPTH / 2.0, -0.0355, Z_LOCK), (FLANGE_DEPTH, FLANGE_W_Y, FLANGE_H), NICKEL_BASE),
    # Rigid hinge bridge connecting mounting flange to retainer arc
    box((-0.007, -0.0375, Z_LOCK), (0.014, 0.013, RETAINER_H), DARK_STEEL),
    # Externalized rigid anti-swing retainer arc (112 .. 158 deg, r_out = 25.5 mm)
    RETAINER_ARC,
    *_DOOR_VISUALS,
)

HINGE_JOINT = Joint(
    "hinge",
    "hinge",
    (_ghost(cyl((0.0, 0.0, 0.0), 0.005, 0.074, DARK_STEEL, axis="z")),),
    axis="z",
    at=HINGE_AT,
    parent=None,
    low=HINGE_LOW,
    high=HINGE_HIGH,
    hold=HINGE_HOLD,
    damping=HINGE_DAMP,
    mass=HINGE_MASS,
)

SLIDE_JOINT = Joint(
    "slide_block",
    "slide",
    (YOKE_TOP, YOKE_BOT, CATCH_LUG, STOP_CORE, *STOP_SHOULDERS, *GROOVE_INLAYS),
    axis="y",
    at=(0.0, 0.0, 0.0),
    parent="hinge",
    low=SLIDE_LOW,
    high=SLIDE_HIGH,
    spring=SLIDE_SPRING,
    damping=SLIDE_DAMP,
    hold=0.0,
    mass=SLIDE_MASS,
    friction=SLIDE_FRICTION,
)

DOOR_LOCK = Mech(
    "door_lock",
    pos=(DOOR_X, DOOR_Y, TOP),
    mass=None,
    shapes=_BASE_SHAPES,
    joints=(HINGE_JOINT, SLIDE_JOINT),
    friction=0.60,
)

OBJECTS = [DOOR_LOCK]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring: checked every step; SCORE_OVER = "end"
# Forced-interlock zero condition (latched whole run): catch_lug penetrates retainer_arc by > 2.0 mm OR hinge angle
# enters 15.0..75.0 deg while slide_block < 9.0 mm (real trials: lock bent, forced, or damaged).
# Milestone C (latched whole run): theta >= 20.0 deg reached without forcing (physically impossible without sliding
# past the anti-swing retainer arc).
# End gate (required for any nonzero score): mechkit.released() >= 30 mm from all finger pads AND still
# (|dtheta/dt| < 2.0 deg/s and |ds/dt| < 3.0 mm/s) for the final 1.0 s continuously.
# Tiers when not forced, Milestone C latched, and end gate holds:
#   1.00 if theta >= 84.0 deg and s <= 3.0 mm (seated in open rest pocket)
#   0.50 if theta >= 78.0 deg and not 1.00
#   0.25 if Milestone C latched and neither above
#   0.00 otherwise.

FORCED_PEN_M = 0.002       # 2.0 mm penetration into retainer arc latches forced fault
FORCED_THETA_LO = 15.0     # deg
FORCED_THETA_HI = 75.0     # deg
FORCED_SLIDE_MIN_MM = 9.0  # mm

MILESTONE_C_DEG = 20.0
OPEN_FULL_DEG = 84.0
OPEN_SEATED_MM = 3.0
OPEN_PARTIAL_DEG = 78.0

RELEASE_CLEARANCE = 0.030  # 30 mm from all finger pad points
STILL_DTHETA_MAX = 2.0     # deg/s
STILL_DS_MAX = 3.0         # mm/s
END_HOLD_S = 1.0           # s continuous released and still
NOMINAL_DT = 0.05          # 20 Hz fallback for offline unit tests

_L: dict = {
    "forced": False,
    "forced_reason": None,
    "milestone_c": False,
    "still_s": 0.0,
    "prev_theta": None,
    "prev_s": None,
    "last_t": None,
    "seen": None,
}


@on_reset
def _reset(env=None) -> None:
    _L.update(
        forced=False,
        forced_reason=None,
        milestone_c=False,
        still_s=0.0,
        prev_theta=None,
        prev_s=None,
        last_t=None,
        seen=None,
    )


def _step_dt() -> float:
    t = float(mechkit.elapsed())
    prev = _L["last_t"]
    _L["last_t"] = t
    if prev is not None and t > prev + 1e-9:
        return t - prev
    return NOMINAL_DT


def lug_retainer_gap(joints: dict[str, float]) -> float:
    """Signed clearance (m) between CATCH_LUG and RETAINER_ARC at the given joint values (negative = penetration)."""
    pose = DOOR_LOCK.joint_pose("slide_block", joints)
    return float(mechkit.gap(CATCH_LUG, pose, RETAINER_ARC, None))


def _update(objects) -> None:
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    dt = _step_dt()
    st = objects["door_lock"]
    theta_deg = math.degrees(float(st.joints["hinge"]))
    s_mm = float(st.joints["slide_block"]) * 1000.0

    if not _L["forced"]:
        pen_gap = lug_retainer_gap(st.joints)
        if pen_gap < -FORCED_PEN_M:
            _L["forced"] = True
            _L["forced_reason"] = f"catch_lug penetrated retainer_arc ({pen_gap * 1000.0:.2f} mm)"
        elif FORCED_THETA_LO <= theta_deg <= FORCED_THETA_HI and s_mm < FORCED_SLIDE_MIN_MM:
            _L["forced"] = True
            _L["forced_reason"] = f"forced interlock at theta={theta_deg:.1f} deg, s={s_mm:.2f} mm"

    if not _L["forced"] and theta_deg >= MILESTONE_C_DEG:
        _L["milestone_c"] = True

    rel = mechkit.released(st, clearance=RELEASE_CLEARANCE)
    if _L["prev_theta"] is not None:
        dtheta_dt = abs(theta_deg - _L["prev_theta"]) / dt
        ds_dt = abs(s_mm - _L["prev_s"]) / dt
        still_now = dtheta_dt < STILL_DTHETA_MAX and ds_dt < STILL_DS_MAX
        if rel and still_now:
            _L["still_s"] += dt
        else:
            _L["still_s"] = 0.0
    else:
        _L["still_s"] = 0.0

    _L["prev_theta"] = theta_deg
    _L["prev_s"] = s_mm


@on_step
def _step_hook(env) -> None:
    pass


def score(objects) -> float:
    """0.00 if forced interlock latched, Milestone C (theta >= 20 deg cleanly) not reached, or end gate (released >=
    30 mm and still < 2 deg/s, < 3 mm/s for final 1.0 s) not met. Otherwise 1.00 if theta >= 84.0 deg and s <= 3.0 mm,
    0.50 if theta >= 78.0 deg, or 0.25 if Milestone C was latched."""
    _update(objects)
    if _L["forced"] or not _L["milestone_c"]:
        return 0.0
    if _L["still_s"] < END_HOLD_S - 1e-6:
        return 0.0
    st = objects["door_lock"]
    theta_deg = math.degrees(float(st.joints["hinge"]))
    s_mm = float(st.joints["slide_block"]) * 1000.0
    if theta_deg >= OPEN_FULL_DEG and s_mm <= OPEN_SEATED_MM:
        return 1.0
    if theta_deg >= OPEN_PARTIAL_DEG:
        return 0.5
    return 0.25


def success(objects) -> bool:
    """Full score (1.00, already requiring no forcing, Milestone C, >= 84 deg, s <= 3 mm, released, and still 1.0 s)."""
    return score(objects) == 1.0


# ---------------------------------------------------------------------------------------------------------------------
# Robot keep-out check for sweep.py: no arm part may enter the solid closed door slab or jamb interior.
ARM_KEEP_OUT = [
    {
        "name": "closed door slab and jamb",
        "x": (DOOR_X + 0.002, DOOR_X + 0.125),
        "y": (DOOR_Y - 0.130, DOOR_Y + 0.145),
        "z_top": TOP + DOOR_H + 0.005,
    }
]

# ---------------------------------------------------------------------------------------------------------------------
# Closed-loop scripted demo (right arm only; left arm holds READY open throughout).
# The demo reads the live hinge and slide_block joint angles of door_lock from the simulator, alongside its joint
# encoders and gripper opening, to maintain 13-14.5 mm spring retraction through the 90-degree swing. It never
# teleports objects or writes joint states.

demo = placeholder_demo

HINGE_WORLD = np.array([DOOR_X + HINGE_AT[0], DOOR_Y + HINGE_AT[1], TOP + Z_LOCK], dtype=float)
GRASP_LX = -0.005        # TCP local x in hinge frame: pad tips reach lx = +0.008 (4 mm clear of door face at +0.012)
GRASP_LY0 = BLOCK_Y_MID  # 0.0460 m: centered in the recessed top/bottom finger grooves
PITCH_HORIZ = math.pi / 2.0
ROLL_CAM_BACK = -math.pi / 2.0  # pads oppose vertically along world z; wrist camera stays on -x robot side


def _arc_tcp(theta_rad: float, slide_m: float, lx: float = GRASP_LX, z_off: float = 0.0) -> np.ndarray:
    """World TCP position riding at local (lx, GRASP_LY0 + slide_m) in the rotating hinge frame."""
    ly = GRASP_LY0 + float(slide_m)
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array(
        [
            HINGE_WORLD[0] + lx * c - ly * s,
            HINGE_WORLD[1] + lx * s + ly * c,
            HINGE_WORLD[2] + z_off,
        ],
        dtype=float,
    )


def _lock_rot(yaw_rad: float) -> np.ndarray:
    return grip_rot(yaw=yaw_rad, pitch=PITCH_HORIZ, roll=ROLL_CAM_BACK)


def demo_script(ctx):
    R = ctx.right

    # 1) Pre-grasp approach with full FINGER_OPEN (93.9 mm, 8.85 mm/side clearance over the 76.2 mm stop block)
    #    Keep every transfer <= 0.09 m/s (well under the 0.12 m/s speed envelope).
    transit = np.array([0.255, -0.100, TOP + 0.195], dtype=float)
    pre_pos = _arc_tcp(0.0, 0.0, lx=GRASP_LX - 0.052)  # 52 mm back along -x from grasp point
    yield from R.move(transit, duration=4.2, finger=FINGER_OPEN, rot=_lock_rot(0.0))
    yield from R.move(pre_pos, duration=3.6, finger=FINGER_OPEN, rot=_lock_rot(0.0))
    yield from R.hold(0.3)

    # Trim joint-PD gravity sag at pre-grasp and carry the measured offset into the horizontal advance
    yield from R.trim_to_measured(pre_pos, tol=0.0015, max_comp=0.030, step_mps=0.03, max_duration=1.2)
    sag = R.pos - R.measured_pos
    sag_z = float(np.clip(sag[2], -0.025, 0.025))

    def _pinch_groove(x_extra: float = 0.0):
        nonlocal sag, sag_z
        goal_xyz = _arc_tcp(0.0, 0.0, lx=GRASP_LX + x_extra)
        yield from R.move(goal_xyz + sag, duration=2.4, finger=FINGER_OPEN, rot=_lock_rot(0.0))
        yield from R.trim_to_measured(goal_xyz, tol=0.0015, max_comp=0.030, step_mps=0.03, max_duration=0.8)
        sag = R.pos - R.measured_pos
        sag_z = float(np.clip(sag[2], -0.025, 0.025))
        yield from R.grip(FINGER_CLOSED, duration=0.8)
        yield from R.hold(0.4)

    yield from _pinch_groove(0.0)
    # Verify grasped pad gap sits inside the 66.2 mm recessed finger grooves (61..71 mm), not clamped on the 76.2 mm lips
    in_groove = R.holding(0.060) and (R.offline or (0.061 <= R.gap() <= 0.071))
    if not in_groove:
        yield from R.grip(FINGER_OPEN, duration=0.6)
        yield from R.move(pre_pos + sag, duration=1.6, finger=FINGER_OPEN, rot=_lock_rot(0.0))
        yield from R.trim_to_measured(pre_pos, tol=0.0015, max_comp=0.030, step_mps=0.03, max_duration=0.8)
        sag = R.pos - R.measured_pos
        sag_z = float(np.clip(sag[2], -0.025, 0.025))
        yield from _pinch_groove(0.0015)

    # 2) Lateral spring retraction (+14.2 mm along +y) with closed-loop feedback on live slide_block travel.
    #    Overdrive compensates joint-5 PD compliance under the 5.7 N spring load, while live x_bias and yaw_bias
    #    keep the pad tips and stop block clear of the closed door face (offline dry_run keeps x_bias=0, yaw_bias=0).
    x_nom = float(HINGE_WORLD[0] + GRASP_LX)
    x_bias = 0.0 if ctx.offline else float(np.clip(sag[0], -0.006, 0.001))
    yaw_bias = 0.0
    s_goal = 0.0142
    s_over = 0.0
    slide_steps = max(1, round(2.4 / ctx.dt))
    for k in range(1, slide_steps + 1):
        frac = k / slide_steps
        s_ref = s_goal * (frac * frac * (3.0 - 2.0 * frac))
        st = ctx.obj("door_lock")
        s_live = float(st.joints["slide_block"])
        if not ctx.offline:
            th_live = float(st.joints["hinge"])
            ds = float(np.clip(0.35 * (s_ref - s_live), -0.0015, 0.0020))
            s_over = float(np.clip(s_over + ds, 0.0, 0.038))
            dx = float(np.clip(0.30 * (x_nom - R.measured_pos[0]), -0.0012, 0.0005))
            x_bias = float(np.clip(x_bias + dx, -0.014, 0.001))
            dyaw = float(np.clip(0.35 * (0.0 - th_live), -0.005, 0.015))
            yaw_bias = float(np.clip(yaw_bias + dyaw, 0.0, 0.07))
        s_cmd = s_ref + s_over
        R._set_cartesian(_arc_tcp(0.0, s_cmd, lx=GRASP_LX, z_off=sag_z) + np.array([x_bias, 0.0, 0.0]), _lock_rot(yaw_bias), FINGER_CLOSED)
        yield

    # Hold until slide_block cleanly reaches >= 13.5 mm before starting the 90-degree swing
    for _ in range(max(1, round(2.0 / ctx.dt))):
        st = ctx.obj("door_lock")
        s_live = float(st.joints["slide_block"])
        if not ctx.offline:
            th_live = float(st.joints["hinge"])
            ds = float(np.clip(0.40 * (s_goal - s_live), -0.0015, 0.0020))
            s_over = float(np.clip(s_over + ds, 0.0, 0.040))
            dx = float(np.clip(0.30 * (x_nom - R.measured_pos[0]), -0.0012, 0.0005))
            x_bias = float(np.clip(x_bias + dx, -0.014, 0.001))
            dyaw = float(np.clip(0.35 * (0.0 - th_live), -0.005, 0.015))
            yaw_bias = float(np.clip(yaw_bias + dyaw, 0.0, 0.07))
        R._set_cartesian(_arc_tcp(0.0, s_goal + s_over, lx=GRASP_LX, z_off=sag_z) + np.array([x_bias, 0.0, 0.0]), _lock_rot(yaw_bias), FINGER_CLOSED)
        yield
        if not ctx.offline and s_live >= 0.0135:
            break

    # 3) Coupled 90-degree outward swing around the vertical hinge pin while sustaining 13.2..14.8 mm retraction
    arc_steps = max(1, round(9.0 / ctx.dt))
    th_over = 0.0
    for k in range(1, arc_steps + 1):
        frac = k / arc_steps
        sm = frac * frac * (3.0 - 2.0 * frac)
        th_ref = HINGE_HIGH * sm
        st = ctx.obj("door_lock")
        th_live = float(st.joints["hinge"])
        s_live = float(st.joints["slide_block"])
        if not ctx.offline:
            ds = float(np.clip(0.40 * (s_goal - s_live), -0.0015, 0.0020))
            s_over = float(np.clip(s_over + ds, -0.002, 0.040))
            dth = float(np.clip(0.22 * (th_ref - th_live), -0.015, 0.020))
            th_over = float(np.clip(th_over + dth, -0.04, 0.14))
            sag_z = float(np.clip(sag_z + 0.15 * (HINGE_WORLD[2] - R.measured_pos[2]), -0.025, 0.025))
        th_cmd = float(np.clip(th_ref + th_over, 0.0, HINGE_HIGH + 0.08))
        s_cmd = float(np.clip(s_goal + s_over, 0.010, 0.054))
        xb = x_bias * math.cos(th_ref)
        rot_yaw = min(th_cmd + yaw_bias * (1.0 - sm), HINGE_HIGH)
        R._set_cartesian(_arc_tcp(th_cmd, s_cmd, lx=GRASP_LX, z_off=sag_z) + np.array([xb, 0.0, 0.0]), _lock_rot(rot_yaw), FINGER_CLOSED)
        yield

    # Hold briefly at 90 deg to ensure hinge reaches >= 89.0 deg before easing the spring home
    for _ in range(max(1, round(0.8 / ctx.dt))):
        st = ctx.obj("door_lock")
        th_live = float(st.joints["hinge"])
        s_live = float(st.joints["slide_block"])
        if not ctx.offline:
            ds = float(np.clip(0.25 * (s_goal - s_live), -0.0015, 0.0020))
            s_over = float(np.clip(s_over + ds, -0.002, 0.040))
            dth = float(np.clip(0.25 * (HINGE_HIGH - th_live), -0.015, 0.020))
            th_over = float(np.clip(th_over + dth, -0.02, 0.14))
        th_cmd = float(np.clip(HINGE_HIGH + th_over, HINGE_HIGH - 0.01, HINGE_HIGH + 0.08))
        R._set_cartesian(_arc_tcp(th_cmd, s_goal + s_over, lx=GRASP_LX, z_off=sag_z), _lock_rot(HINGE_HIGH), FINGER_CLOSED)
        yield
        if not ctx.offline and th_live >= math.radians(89.0):
            break

    # 4) Ease the block along +x (local -y at 90 deg) so the return spring seats it into the open rest pocket (s <= 1 mm)
    seat_pos = _arc_tcp(HINGE_HIGH + 0.02, 0.0, lx=GRASP_LX, z_off=sag_z)
    yield from R.move(seat_pos, duration=2.2, finger=FINGER_CLOSED, rot=_lock_rot(HINGE_HIGH))
    yield from R.hold(0.5)

    # 5) Open gripper wide, back straight off the block along -y, lift clear, park at READY, and hold 10 s
    yield from R.grip(FINGER_OPEN, duration=1.0)
    yield from R.hold(0.2)
    back_pos = _arc_tcp(HINGE_HIGH, 0.0, lx=GRASP_LX - 0.060, z_off=sag_z)  # -60 mm in world -y
    yield from R.move(back_pos, duration=1.8, finger=FINGER_OPEN, rot=_lock_rot(HINGE_HIGH))
    clear_pos = np.array([0.255, -0.150, TOP + 0.215], dtype=float)
    yield from R.move(clear_pos, duration=3.0, finger=FINGER_OPEN, rot=_lock_rot(0.0))
    yield from R.home(duration=4.5)
    yield from R.hold(10.0)
