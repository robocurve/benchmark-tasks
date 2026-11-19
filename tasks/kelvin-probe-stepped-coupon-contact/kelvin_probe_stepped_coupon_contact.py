"""Lower the Kelvin probe head onto the stepped coupon while releasing its counterbalanced transit clip.

Autonomous SDL four-point Kelvin-probe sheet-resistance characterization bench:
- `carrier_stage` (720 g): unbolted ceramic/anodized XY guide base (`204 x 128 x 12 mm` plinth, `46 mm` trunnion
  towers above mat) on a pebbled ESD `bench_mat` (mu=0.80).
- `datum_fence` (56 g): compliant V-anvil datum carriage on a short 5 mm +X seat stop (`stiffness=240 N/m`, `0.04 N`).
- `stepped_coupon` (68 g): thin-film two-step coupon (`44 mm` `+X` slide to `+40 mm` datum, `25 N/m` return spring
  toward 0, `0.02 N` friction, `16 mm` upper step on `-X`, `8 mm` lower step on `+X`, asymmetric `-Y` polarity keyway).
- `probe_arm` (130 g) & `probe_head` (55 g): counterbalanced rotary arm (`+Y` hinge, `-26..+4 deg` travel stops around
  `-22 deg` raised start to `0 deg` seated, `0.080 N*m` detent) carrying a 4-pin stepped Kelvin head (`+-4 mm` Z slider, `650 N/m`).
- `transit_clip` (56 g): weighted safety prop bonded to `carrier_stage` (`mk.bond`, `3.5 N` breakaway)
  physically blocking `probe_arm` above the coupon until lifted clear into its `(0.292, 0.175)` m mat bay.

Scored strictly from observable end-state geometry (`SUCCESS_HOLD_S = 3.0`, `SCORE_OVER = "end"`):
zero `on_step`, zero episode counters, zero code-held stiffness/target overrides.
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
import mechkit as mk  # noqa: E402  (must precede labkit so mechkit wraps sys.meta_path first)
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP
MAT_H = 0.004
DECK_Z = TOP + MAT_H

# -- distinct PBR colours ---------------------------------------------------------------------------------------------
ANOD_SLATE = (0.24, 0.27, 0.31)
CERAMIC_WT = (0.90, 0.90, 0.87)
STEEL_304 = (0.72, 0.75, 0.78)
BRASS_C360 = (0.83, 0.66, 0.26)
COPPER_PAD = (0.80, 0.48, 0.25)
AMBER_CLIP = (0.90, 0.58, 0.14)
BLUE_FILM = (0.18, 0.42, 0.72)
GREEN_FILM = (0.18, 0.56, 0.40)
RED_HANDLE = (0.78, 0.18, 0.16)
ESD_MAT = (0.18, 0.24, 0.22)

# -- 1:1 station layout (metres, kg, N, deg) --------------------------------------------------------------------------
BX, BY = 0.270, 0.000
BASE_Z0 = DECK_Z
STAGE_H = 0.012
COUPON_Z0 = BASE_Z0 + STAGE_H  # coupon underside sliding surface
HINGE_ANCHOR = (BX - 0.062, BY, BASE_Z0 + 0.052)

BASE_MASS = 0.720
FENCE_MASS = 0.056
COUPON_MASS = 0.068
ARM_MASS = 0.130
HEAD_MASS = 0.055
CLIP_MASS = 0.056

COUPON_X0 = BX - 0.022  # coupon start center in X; seated (+0.040) -> BX + 0.018
COUPON_TRAVEL = 0.044
COUPON_TARGET = 0.040
COUPON_K = 25.0  # N/m return spring toward 0.0 (1.0 N at 40 mm)

CLIP_DOCK_REL = (0.012, 0.088, 0.0)  # relative to carrier_stage origin (clip sits flush on bench_mat)
CLIP_BAY_XY = (BX + 0.022, 0.175)
CLIP_BREAK_N = 3.5

ARM_START_DEG = -22.0
ARM_K_NM = 0.0
ARM_FRIC_NM = 0.080

# Probe contact targets in carrier_stage frame when coupon_slide == COUPON_TARGET (+40 mm):
# Upper step spans x in [-0.017, +0.018], top at z_rel = STAGE_H + 0.016 (0.028 above BASE_Z0)
# Lower step spans x in [+0.018, +0.053], top at z_rel = STAGE_H + 0.008 (0.020 above BASE_Z0)
PIN_REL_X = (0.000, 0.036)  # rear upper-step pins vs front lower-step pins (in base frame)
PIN_REL_Y = (-0.012, 0.012)

_VIS_DIR = "assets/kelvin-probe-stepped-coupon-contact"

# -- Fixed ESD bench mat with clip rest bay ---------------------------------------------------------------------------
BENCH_MAT = labkit.part(
    "bench_mat",
    pos=(BX, BY, TOP),
    fixed=True,
    friction=0.80,
    color=ESD_MAT,
    boxes=[
        ((0.0, 0.0, MAT_H / 2), (0.480, 0.440, MAT_H), ESD_MAT),
        # Low-profile padded parking bay for disengaged transit clip on +Y side (y = +0.175)
        ((0.022, 0.175 - 0.032, MAT_H + 0.003), (0.072, 0.006, 0.006), ANOD_SLATE),
        ((0.022, 0.175 + 0.032, MAT_H + 0.003), (0.072, 0.006, 0.006), ANOD_SLATE),
        ((-0.011, 0.175, MAT_H + 0.003), (0.006, 0.064, 0.006), ANOD_SLATE),
        ((0.055, 0.175, MAT_H + 0.003), (0.006, 0.064, 0.006), ANOD_SLATE),
    ],
)
labkit.VISUALS["bench_mat/body"] = pathlib.Path(_HERE) / _VIS_DIR / "bench_mat.usdc"

kit = mk.Kit()

# -- 1. 720 g ceramic & anodized XY carrier stage ---------------------------------------------------------------------
kit.part("carrier_stage", (BX, BY, BASE_Z0), [
    mk.box((0.204, 0.128, STAGE_H), at=(0.008, 0.0, STAGE_H / 2), color=ANOD_SLATE),  # main machined plinth
    mk.box((0.156, 0.010, 0.018), at=(0.0, 0.032, STAGE_H + 0.009), color=CERAMIC_WT),  # +y guide rail
    mk.box((0.156, 0.010, 0.018), at=(0.0, -0.032, STAGE_H + 0.009), color=CERAMIC_WT),  # -y guide rail
    # Asymmetric polarity key rail along -y inside slot (rejects 180-deg reversed coupon loading!)
    mk.box((0.110, 0.005, 0.006), at=(0.012, -0.024, STAGE_H + 0.013), color=AMBER_CLIP),
    mk.box((0.012, 0.054, 0.024), at=(0.102, 0.0, STAGE_H + 0.012), color=STEEL_304),  # +x datum backstop
    mk.box((0.018, 0.012, 0.046), at=(-0.062, 0.034, STAGE_H + 0.023), color=ANOD_SLATE),  # +y trunnion post
    mk.box((0.018, 0.012, 0.046), at=(-0.062, -0.034, STAGE_H + 0.023), color=ANOD_SLATE),  # -y trunnion post
    # Side dock tab for transit_clip (+y edge, clears plinth top cleanly)
    mk.box((0.036, 0.012, 0.012), at=(0.012, 0.058, STAGE_H + 0.006), color=ANOD_SLATE),
], mass=BASE_MASS, color=ANOD_SLATE, visual=f"{_VIS_DIR}/carrier_stage.usdc")

# -- 2. 56 g compliant V-anvil datum fence (34x56 mm top silhouette >= 12 px) -----------------------------------------
kit.part("datum_fence", (BX + 0.072, BY, COUPON_Z0), [
    mk.box((0.034, 0.056, 0.020), at=(0.004, 0.0, 0.010), color=CERAMIC_WT),
    mk.box((0.004, 0.042, 0.016), at=(-0.015, 0.0, 0.010), color=BRASS_C360),  # +x target V-anvil contact face (+0.055)
], mass=FENCE_MASS, color=CERAMIC_WT, visual=f"{_VIS_DIR}/datum_fence.usdc")
kit.slider("datum_slide", "carrier_stage", "datum_fence", axis=(1.0, 0.0, 0.0), lower=0.0, upper=0.005,
           start=0.0, stiffness=240.0, rest=0.0, damping=6.0, friction=0.04)

# -- 3. 68 g stepped thin-film coupon (upper step on -X, lower step on +X, polarity key notch on -Y) -----------------
kit.part("stepped_coupon", (COUPON_X0, BY, COUPON_Z0), [
    # Lower front step (+X half: 8 mm tall, spans x_rel in [0, +0.035])
    mk.box((0.035, 0.046, 0.008), at=(0.0175, 0.0, 0.004), color=GREEN_FILM),
    # Upper rear step (-X half: 16 mm tall, +Y offset leaves 6 mm asymmetric keyway on -Y)
    mk.box((0.035, 0.040, 0.016), at=(-0.0175, 0.003, 0.008), color=BLUE_FILM),
    # Four copper measurement pads matching the 4-point Kelvin pin positions at x_slide = +0.040 m
    mk.box((0.010, 0.010, 0.001), at=(-0.018, -0.012, 0.0165), color=COPPER_PAD),
    mk.box((0.010, 0.010, 0.001), at=(-0.018, 0.012, 0.0165), color=COPPER_PAD),
    mk.box((0.010, 0.010, 0.001), at=(0.018, -0.012, 0.0085), color=COPPER_PAD),
    mk.box((0.010, 0.010, 0.001), at=(0.018, 0.012, 0.0085), color=COPPER_PAD),
    # Right-arm arched handle bridge over -Y rail to wide-caged lug at y = -0.078 (38 mm X slot absorbs +-10 mm jitter)
    mk.box((0.012, 0.010, 0.014), at=(0.0, -0.020, 0.017), color=STEEL_304),
    mk.box((0.012, 0.056, 0.006), at=(0.0, -0.046, 0.022), color=STEEL_304),
    mk.box((0.022, 0.014, 0.022), at=(0.0, -0.078, 0.016), color=CERAMIC_WT),
    mk.box((0.004, 0.026, 0.024), at=(-0.021, -0.078, 0.016), color=STEEL_304),
    mk.box((0.004, 0.026, 0.024), at=(0.021, -0.078, 0.016), color=STEEL_304),
], mass=0.068, color=BLUE_FILM, visual=f"{_VIS_DIR}/stepped_coupon.usdc")
kit.slider("coupon_slide", "carrier_stage", "stepped_coupon", axis=(1.0, 0.0, 0.0), lower=0.0, upper=COUPON_TRAVEL,
           start=0.0, stiffness=COUPON_K, rest=0.0, damping=1.2, friction=0.02)

# -- 4. 130 g counterbalanced rotary arm + 55 g compliant four-point probe head ---------------------------------------
kit.part("probe_arm", HINGE_ANCHOR, [
    mk.box((0.124, 0.022, 0.010), at=(0.020, 0.0, 0.0), color=STEEL_304),  # main beam (-0.042..+0.082)
    mk.box((0.022, 0.068, 0.014), at=(0.0, 0.0, 0.0), color=STEEL_304),  # trunnion crossbar
    mk.box((0.052, 0.060, 0.040), at=(-0.064, 0.0, 0.002), color=BRASS_C360),  # rear brass counterweight (~0 torque)
    # Left-arm wide press pad on +Y (y_world = +0.064 -> clears transit_clip & handles +-10 mm jitter)
    mk.box((0.018, 0.050, 0.008), at=(0.042, 0.032, 0.002), color=STEEL_304),
    mk.box((0.028, 0.022, 0.016), at=(0.042, 0.064, 0.005), color=RED_HANDLE),
    mk.box((0.004, 0.026, 0.020), at=(0.023, 0.064, 0.005), color=STEEL_304),
    mk.box((0.004, 0.026, 0.020), at=(0.061, 0.064, 0.005), color=STEEL_304),
], mass=0.130, color=STEEL_304, visual=f"{_VIS_DIR}/probe_arm.usdc")
kit.hinge("probe_hinge", "carrier_stage", "probe_arm", anchor=HINGE_ANCHOR, axis=(0.0, 1.0, 0.0),
          lower_deg=-26.0, upper_deg=4.0, start_deg=ARM_START_DEG, stiffness=ARM_K_NM,
          rest_deg=ARM_START_DEG, damping=0.03, friction=ARM_FRIC_NM)

# Probe head center in hinge frame: (0.080, 0, -0.008) -> world (BX + 0.018, BY, BASE_Z0 + 0.044) when arm at 0 deg
kit.part("probe_head", (HINGE_ANCHOR[0] + 0.080, BY, HINGE_ANCHOR[2] - 0.008), [
    mk.box((0.056, 0.038, 0.012), at=(0.0, 0.0, 0.002), color=ANOD_SLATE),  # probe block body
    # Rear step-locking shoulder heel at x_carrier = -0.024 (+X face at -0.021; drops 7 mm below upper step)
    mk.box((0.006, 0.036, 0.016), at=(-0.042, 0.0, -0.014), color=CERAMIC_WT),
    # 2 upper-step brass probe pins at x_carrier = 0.000 (x_head = -0.018), tip bottom flush on +0.0290 pad
    mk.box((0.006, 0.006, 0.014), at=(-0.018, -0.012, -0.008), color=BRASS_C360),
    mk.box((0.006, 0.006, 0.014), at=(-0.018, 0.012, -0.008), color=BRASS_C360),
    # 2 lower-step brass probe pins at x_carrier = +0.036 (x_head = +0.018), tip bottom flush on +0.0210 pad
    mk.box((0.006, 0.006, 0.022), at=(0.018, -0.012, -0.012), color=BRASS_C360),
    mk.box((0.006, 0.006, 0.022), at=(0.018, 0.012, -0.012), color=BRASS_C360),
], mass=HEAD_MASS, color=ANOD_SLATE, visual=f"{_VIS_DIR}/probe_head.usdc")
kit.slider("head_compliance", "probe_arm", "probe_head", axis=(0.0, 0.0, 1.0), lower=-0.004, upper=0.004,
           start=0.0, stiffness=650.0, rest=0.0, damping=10.0, friction=0.02)

# -- 5. 56 g weighted transit clip (flush on bench_mat at y=+0.088; 3.5 N breakaway bond; props arm at -14 deg)
kit.part("transit_clip", (BX + CLIP_DOCK_REL[0], BY + CLIP_DOCK_REL[1], BASE_Z0), [
    mk.box((0.044, 0.040, 0.010), at=(0.0, 0.0, 0.005), color=AMBER_CLIP),  # base foot flush on bench_mat
    # -Y bridge + steel safety prop 6 mm cleanly under probe_arm at -22 deg (top +0.056 vs +0.0627 -> blocks below -14 deg)
    mk.box((0.016, 0.046, 0.012), at=(-0.032, -0.022, 0.050), color=STEEL_304),
    mk.box((0.016, 0.016, 0.044), at=(-0.032, -0.008, 0.024), color=STEEL_304),
    # Wide-caged grasp lug at (BX + 0.022, +0.106) -> 32 mm X window + 14 mm grip face absorbs +-10 mm jitter
    mk.box((0.022, 0.014, 0.024), at=(0.010, 0.018, 0.022), color=AMBER_CLIP),
    mk.box((0.004, 0.026, 0.024), at=(-0.008, 0.018, 0.022), color=STEEL_304),
    mk.box((0.004, 0.026, 0.024), at=(0.028, 0.018, 0.022), color=STEEL_304),
], mass=0.056, color=AMBER_CLIP, visual=f"{_VIS_DIR}/transit_clip.usdc")
kit.bond("clip_bond", "carrier_stage", "transit_clip", break_force=CLIP_BREAK_N, break_torque=mk.INF)

OBJECTS = [BENCH_MAT, *kit.objects()]

# -- thresholds (PhysX caliper/protractor tolerances, 100% pure observable end-state) ---------------------------------
SEAT_TOL_M = 0.006
PARTIAL_TOL_M = 0.012
ARM_TILT_MAX_DEG = 6.0
PARTIAL_TILT_DEG = 12.0
BASE_XY_MAX = 0.015
BASE_TILT_MAX_DEG = 8.0
CLIP_CLEAR_M = 0.030
CLIP_BAY_XY_TOL = 0.032
CLIP_BAY_Z_TOL = 0.012
CLIP_TILT_MAX_DEG = 20.0
CLEAR_M = 0.045
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"
POSSIBLE_SCORES = (0.0, 0.4, 0.7, 1.0)


def _tilt_deg(quat) -> float:
    r = np.asarray(mk.quat_to_mat(quat), float)
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2]))))))


def _tcps() -> list[np.ndarray]:
    return [np.asarray(t, float) for t in mk.tcps()]


def components(objects) -> dict:
    """Pure observable physical end-state quantities (poses, joint positions, and gripper TCPs only)."""
    bpos, bquat = kit.pose(objects, "carrier_stage")
    cpos, cquat = kit.pose(objects, "stepped_coupon")
    hpos, hquat = kit.pose(objects, "probe_head")
    kpos, kquat = kit.pose(objects, "transit_clip")

    base_xy = float(np.linalg.norm(np.asarray(bpos, float) - np.array([BX, BY, BASE_Z0], float)))
    base_tilt = _tilt_deg(bquat)

    coupon_q = float(kit.q(objects, "coupon_slide"))
    arm_deg = float(kit.q_deg(objects, "probe_hinge"))
    coupon_err = abs(coupon_q - COUPON_TARGET)

    # Probe head pin midpoint height & alignment relative to the seated upper/lower coupon steps
    rmat = np.asarray(mk.quat_to_mat(bquat), float)
    c_in_base = rmat.T @ (np.asarray(cpos, float) - np.asarray(bpos, float))
    h_in_base = rmat.T @ (np.asarray(hpos, float) - np.asarray(bpos, float))
    head_rel_c = h_in_base - c_in_base
    # When coupon_q == +0.040 and probe_arm == 0 deg: h_in_base = (+0.018, 0, +0.044), c_in_base = (+0.018, 0, +0.012)
    pin_xy_err = float(math.hypot(head_rel_c[0], head_rel_c[1]))
    pin_z_err = float(abs((head_rel_c[2]) - 0.032))
    hmat = np.asarray(mk.quat_to_mat(hquat), float)
    head_rel_rot = rmat.T @ hmat
    head_tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, float(head_rel_rot[2, 2]))))))

    # 180-deg polarity check: coupon local +x must remain aligned with carrier_stage +x
    cmat = np.asarray(mk.quat_to_mat(cquat), float)
    coupon_facing = float((rmat.T @ cmat)[0, 0])

    # Weighted transit clip must be disengaged >= 30 mm AND seated upright inside the +Y mat rest bay
    dock_w = np.asarray(bpos, float) + rmat @ np.asarray(CLIP_DOCK_REL, float)
    kp = np.asarray(kpos, float)
    clip_sep = float(np.linalg.norm(kp - dock_w))
    clip_tilt = _tilt_deg(kquat)
    in_clip_bay = bool(
        abs(kp[0] - CLIP_BAY_XY[0]) <= CLIP_BAY_XY_TOL
        and abs(kp[1] - CLIP_BAY_XY[1]) <= CLIP_BAY_XY_TOL
        and abs(kp[2] - BASE_Z0) <= CLIP_BAY_Z_TOL
        and clip_tilt <= CLIP_TILT_MAX_DEG
    )

    tcps = _tcps()
    hands_clear = len(tcps) >= 2
    for tcp in tcps:
        for name in ("carrier_stage", "datum_fence", "stepped_coupon", "probe_arm", "probe_head", "transit_clip"):
            st = objects[name]
            lo, hi = np.asarray(st.low, float) - CLEAR_M, np.asarray(st.high, float) + CLEAR_M
            if bool(np.all(tcp >= lo) and np.all(tcp <= hi)):
                hands_clear = False

    max_err = max(coupon_err, pin_xy_err, pin_z_err)
    return {
        "base_stable": bool(base_xy <= BASE_XY_MAX and base_tilt <= BASE_TILT_MAX_DEG),
        "base_xy": base_xy,
        "base_tilt_deg": base_tilt,
        "coupon_slide_m": coupon_q,
        "coupon_facing": coupon_facing,
        "arm_deg": arm_deg,
        "pin_xy_err": pin_xy_err,
        "pin_z_err": pin_z_err,
        "max_contact_err": max_err,
        "head_tilt_deg": head_tilt,
        "clip_sep_m": clip_sep,
        "clip_tilt_deg": clip_tilt,
        "in_clip_bay": in_clip_bay,
        "clip_released": bool(clip_sep >= CLIP_CLEAR_M and in_clip_bay),
        "hands_clear": hands_clear,
    }


def score(objects) -> float:
    """Stateless end-state scorer; Possible scores: 0.0, 0.4, 0.7, 1.0."""
    c = components(objects)
    if not c["base_stable"] or c["coupon_facing"] < 0.85 or not c["clip_released"]:
        return 0.0
    if c["max_contact_err"] > PARTIAL_TOL_M or c["head_tilt_deg"] > PARTIAL_TILT_DEG:
        return 0.0
    if c["max_contact_err"] > 0.008 or c["head_tilt_deg"] > 9.0:
        return 0.4
    if c["max_contact_err"] > SEAT_TOL_M or c["head_tilt_deg"] > ARM_TILT_MAX_DEG or not c["hands_clear"]:
        return 0.7
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    c = components(objects)
    return float(c["max_contact_err"] + max(0.0, CLIP_CLEAR_M - c["clip_sep_m"]) + 0.002 * c["head_tilt_deg"])


def demo() -> dict:
    """Open-loop dual-arm sequence reading fixed scene constants only."""
    W, c, o = Waypoint, FINGER_CLOSED, FINGER_OPEN
    rl = pitch_demo.rotation(0.0)  # both arms use yaw=0.0 so elbows stay wide on +-Y (0 arm_arm contact)

    # 1. Right braces coupon/carrier along -Y while Left lifts transit_clip +4 mm off 3.5 N bond then pulls +Y into bay
    clip_pick = (BX + 0.022, 0.106, BASE_Z0 + 0.022 + PAD_TIP_PAST_TCP)
    clip_snap = (BX + 0.022, 0.106, BASE_Z0 + 0.0265 + PAD_TIP_PAST_TCP)
    clip_pull = (BX + 0.022, 0.165, BASE_Z0 + 0.0265 + PAD_TIP_PAST_TCP)
    clip_lift = (BX + 0.022, 0.165, BASE_Z0 + 0.055 + PAD_TIP_PAST_TCP)
    clip_bay = (BX + 0.022, 0.192, BASE_Z0 + 0.025 + PAD_TIP_PAST_TCP)

    # 2. Left lowers probe_arm press lug at (BX - 0.020, +0.064) from -22 deg (+0.072) to 0 deg (top +0.065)
    arm_x, arm_y = BX - 0.022, 0.064
    arm_top_open = (arm_x, arm_y, BASE_Z0 + 0.088 + PAD_TIP_PAST_TCP)
    arm_press = (arm_x + 0.002, arm_y, BASE_Z0 + 0.0620 + PAD_TIP_PAST_TCP)

    # 3. Right grips coupon lug at (COUPON_X0, -0.078) -> slides +40 mm (+8 mm target push into datum fence)
    cx0, cy0 = COUPON_X0, -0.078
    cz_pick = COUPON_Z0 + 0.016 + PAD_TIP_PAST_TCP
    cx_seat = cx0 + COUPON_TARGET + 0.008

    z_high = TOP + 0.185

    left = [
        W(1.0, (0.200, 0.190, z_high), o, "sweep wide over +Y approach lane", rot=rl),
        W(1.0, (clip_pick[0], clip_pick[1], z_high), o, "hover over transit clip lug", rot=rl),
        W(1.0, clip_pick, o, "descend onto transit clip lug", rot=rl),
        W(0.8, clip_pick, c, "clamp transit clip lug", rot=rl),
        W(0.7, clip_snap, c, "lift transit clip 4 mm to break 3.5 N bond cleanly", rot=rl),
        W(1.0, clip_pull, c, "slide transit clip +Y clear of probe arm", rot=rl),
        W(0.8, clip_lift, c, "lift transit clip clear of bay rim", rot=rl),
        W(0.8, (clip_bay[0], clip_bay[1], clip_lift[2]), c, "carry transit clip over rest bay", rot=rl),
        W(1.0, clip_bay, c, "lower transit clip into rest bay", rot=rl),
        W(0.8, clip_bay, o, "release transit clip in bay", rot=rl),
        W(0.8, (clip_bay[0], clip_bay[1], z_high), o, "lift clear of clip bay", rot=rl),
        W(1.6, (arm_x, arm_y, z_high), c, "wait over probe arm while right seats stepped coupon", rot=rl),
        W(1.0, arm_top_open, c, "lower onto raised probe arm lug", rot=rl),
        W(1.8, arm_press, c, "lower counterbalanced Kelvin head onto stepped coupon", rot=rl),
        W(1.4, arm_press, c, "hold probe seated across upper and lower steps", rot=rl),
        W(1.0, (arm_x, arm_y, z_high), c, "lift clear vertically above probe arm", rot=rl),
        W(1.1, (0.22, 0.23, z_high), o, "retreat via clear waypoint", yaw=0.0),
        W(1.2, (0.14, 0.29, TOP + 0.195), o, "park left arm >= 45 mm clear", yaw=0.0),
        W(3.6, (0.14, 0.29, TOP + 0.195), o, "hold 3.0 s scored steady state", yaw=0.0),
    ]

    right = [
        W(1.0, (0.200, -0.190, z_high), o, "sweep wide over -Y approach lane", rot=rl),
        W(1.0, (cx0, cy0, z_high), o, "hover over coupon lug", rot=rl),
        W(1.0, (cx0, cy0, cz_pick), o, "straddle coupon caging lug", rot=rl),
        W(0.8, (cx0, cy0, cz_pick), c, "grip coupon lug", rot=rl),
        W(5.6, (cx0, cy0, cz_pick), c, "brace carrier steady while left disengages transit clip", rot=rl),
        W(1.8, (cx_seat, cy0, cz_pick), c, "slide stepped coupon +40 mm to datum fence", rot=rl),
        W(5.8, (cx_seat, cy0, cz_pick), c, "hold coupon at datum while probe arm lowers", rot=rl),
        W(0.8, (cx_seat, cy0, cz_pick), o, "release coupon now latched under probe shoulder", rot=rl),
        W(1.1, (cx_seat, cy0, z_high), o, "lift clear vertically above coupon", rot=rl),
        W(1.1, (0.22, -0.23, z_high), o, "retreat via clear waypoint", yaw=0.0),
        W(1.2, (0.14, -0.29, TOP + 0.195), o, "park right arm >= 45 mm clear", yaw=0.0),
        W(3.4, (0.14, -0.29, TOP + 0.195), o, "hold 3.0 s scored steady state", yaw=0.0),
    ]
    return {"left": left, "right": right}
