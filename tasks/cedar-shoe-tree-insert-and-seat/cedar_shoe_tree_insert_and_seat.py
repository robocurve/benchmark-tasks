"""Insert and seat a twin-tube spring-loaded cedar shoe tree inside an unbolted size-10 oxford shoe on an entry mat.

Physical model (1:1 manufacturer dimensions, all dynamic bodies >= 145 g, SelfCollisions=False across links):
- Entry mat (fixed): 360 x 560 x 5 mm smooth entryway shoe tray/mat (mu = 0.12) with 2.5 mm perimeter curb.
- Men's size-10 cap-toe oxford shoe (loose, 380 g, 305 x 108 mm footprint, 70 mm heel counter, 90 mm laced saddle,
  mu = 0.12): stacked outsole, storm welt, and tan calfskin insole (z = 16 mm) bound a 278 mm internal cavity from
  the front toe stop (y = +139 mm) to the rear heel-counter wall (y = -139 mm). Tapered vamp walls enclose an 84 mm
  clear forepart chamber under a 62 mm cap-toe ceiling (rising to 72 mm under the 28 mm laced instep saddle); low-cut
  ankle quarters dip to 45.5 mm; curved rear heel counter rises to 70 mm with an 8 mm inward topline collar undercut
  (z = 52..70 mm, inner lip at y = -131 mm -> 270 mm clear throat opening).
- Twin-tube red cedar shoe tree (320 g total = 145 g contoured forepart + 175 g rounded heel slide): 292 mm uncompressed
  length, 42 mm telescoping travel along twin brass tubes with K_TUBE = 250.0 N/m (5.5 N at 22 mm compression to clear
  the 270 mm collar lip; ~3.8 N active lock preload at 15 mm compression inside the 278 mm shoe cavity).
Because 5.5 N axial compression exceeds the unheld 380 g leather outsole's ~0.45 N mat friction by >12x, a one-handed push
slides the shoe across the mat without compressing the spring >= 22 mm or seating the heel block beneath the collar undercut.
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
import mechkit as mk  # noqa: E402, F401  (must precede labkit so labkit wraps mechkit's finder)
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0
PASS_SCORE = 1.0 - 1e-9

# -- distinct PBR palette (roughness & metallic tuned on reset in Isaac Sim) ------------------------------------------
RUBBER = (0.15, 0.16, 0.18)
RUBBER_TRIM = (0.23, 0.25, 0.28)
SOLE_DARK = (0.09, 0.08, 0.07)
WELT_BROWN = (0.29, 0.17, 0.10)
ESPRESSO = (0.24, 0.13, 0.07)
CAP_BURNISH = (0.18, 0.09, 0.05)
LACE_CORD = (0.11, 0.09, 0.08)
TAN_LINING = (0.72, 0.56, 0.39)
CEDAR = (0.74, 0.39, 0.25)
CEDAR_SAP = (0.85, 0.66, 0.47)
CEDAR_SLOT = (0.34, 0.17, 0.11)
BRASS = (0.82, 0.68, 0.28)
STEEL_ROD = (0.75, 0.77, 0.80)

_PBR = {
    RUBBER: (0.86, 0.0), RUBBER_TRIM: (0.78, 0.0), SOLE_DARK: (0.74, 0.0), WELT_BROWN: (0.48, 0.0),
    ESPRESSO: (0.28, 0.04), CAP_BURNISH: (0.20, 0.06), LACE_CORD: (0.82, 0.0), TAN_LINING: (0.55, 0.0),
    CEDAR: (0.66, 0.0), CEDAR_SAP: (0.62, 0.0), CEDAR_SLOT: (0.75, 0.0),
    BRASS: (0.22, 0.88), STEEL_ROD: (0.24, 0.85),
}

# -- 1:1 manufacturer dimensions (m, kg, N) ---------------------------------------------------------------------------
MAT_X, MAT_Y = 0.310, 0.000
MAT_W, MAT_L, MAT_H = 0.360, 0.560, 0.005
MAT_TOP = TOP + MAT_H
MAT_MU = 0.12

SHOE_X, SHOE_Y = 0.250, 0.042
SHOE_MASS = 0.380  # 380 g single men's size-10 oxford
SHOE_MU = 0.12
SHOE_LEN, SHOE_BALL_W, SHOE_HEEL_W, SHOE_H = 0.305, 0.108, 0.084, 0.070
INSOLE_Z = 0.016
CAVITY_HALF_Y = 0.139  # 278 mm internal cavity: y in [-0.139, +0.139]
COLLAR_LIP_Y = -0.131  # 8 mm inward heel-counter collar undercut at z = 52..70 mm (270 mm clear throat)
COLLAR_LIP_Z = 0.052
VAMP_CEIL_Z = 0.062
LACE_Y, LACE_W, LACE_TOP_Z = 0.064, 0.028, 0.090

TREE_X, TREE_Y = 0.370, 0.005
TREE_FORE_MASS, TREE_HEEL_MASS = 0.145, 0.175  # 320 g total; both links >= 145 g (>> 25 g floor)
TREE_MU = 0.58
TREE_LEN = 0.292  # 292 mm uncompressed length: y_toe_tip (+0.070) - y_heel_rear (-0.222)
TOE_TIP_Y, HEEL_REAR_Y = 0.070, -0.222
HEEL_BLOCK_LEN, HEEL_BLOCK_W, HEEL_BLOCK_H = 0.054, 0.044, 0.032
HANDLE_Y, HANDLE_LEN, HANDLE_W, HANDLE_TOP_Z = -0.192, 0.036, 0.028, 0.072
SLIDE_MAX_M = 0.042  # 42 mm max twin-tube compression travel
COMPRESS_GATE_M = 0.022  # 22 mm physical compression required to clear the 270 mm heel-collar lip
SPRING_FORCE_AT_GATE_N = 5.5  # 5.5 N spring force at 22 mm compression
K_TUBE = 250.0  # N/m (5.5 N at 22 mm compression)
D_TUBE = 12.0


def _mat_part():
    rim = 0.0025
    boxes = [
        ((0.0, 0.0, MAT_H / 2), (MAT_W, MAT_L, MAT_H), RUBBER),
        ((0.0, MAT_L / 2 - 0.006, MAT_H + rim / 2), (MAT_W - 0.016, 0.012, rim), RUBBER_TRIM),
        ((0.0, -MAT_L / 2 + 0.006, MAT_H + rim / 2), (MAT_W - 0.016, 0.012, rim), RUBBER_TRIM),
        ((-MAT_W / 2 + 0.006, 0.0, MAT_H + rim / 2), (0.012, MAT_L - 0.016, rim), RUBBER_TRIM),
        ((MAT_W / 2 - 0.006, 0.0, MAT_H + rim / 2), (0.012, MAT_L - 0.016, rim), RUBBER_TRIM),
        ((-MAT_W / 2 + 0.020, 0.0, MAT_H - 0.0005), (0.005, MAT_L - 0.060, 0.002), RUBBER_TRIM),
        ((MAT_W / 2 - 0.020, 0.0, MAT_H - 0.0005), (0.005, MAT_L - 0.060, 0.002), RUBBER_TRIM),
    ]
    cyls = [
        ((sx * (MAT_W / 2 - 0.008), sy * (MAT_L / 2 - 0.008), (MAT_H + rim) / 2), 0.008, MAT_H + rim, RUBBER_TRIM, "Z")
        for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)
    ]
    return part("entry_mat", (MAT_X, MAT_Y, TOP), boxes=boxes, cylinders=cyls, fixed=True, color=RUBBER, friction=MAT_MU)


def _shoe_part():
    """Men's size-10 cap-toe oxford (305 x 108 mm, 70 mm counter, 90 mm saddle, 380 g) with 8 mm collar undercut."""
    boxes = [
        # Sculpted outsole waist + stacked rear heel block + storm welt + tan calfskin insole (z = 16 mm)
        ((0.0, -0.022, 0.007), (0.070, 0.130, 0.014), SOLE_DARK),
        ((0.0, -0.108, 0.007), (SHOE_HEEL_W, 0.056, 0.014), SOLE_DARK),
        ((0.0, 0.056, 0.0045), (SHOE_BALL_W, 0.092, 0.009), SOLE_DARK),
        ((0.0, 0.056, 0.0115), (0.098, 0.096, 0.005), WELT_BROWN),
        ((0.0, -0.004, 0.015), (0.066, 0.258, 0.002), TAN_LINING),
        # Burnished 50 mm tapered cap-toe nose (y in [+0.118, +0.1525], clamp zone ahead of upper brogue cap shoulder)
        ((0.0, 0.1455, 0.036), (0.050, 0.014, 0.044), CAP_BURNISH),
        ((-0.025, 0.130, 0.036), (0.006, 0.026, 0.044), CAP_BURNISH),
        ((0.025, 0.130, 0.036), (0.006, 0.026, 0.044), CAP_BURNISH),
        ((-0.036, 0.110, 0.036), (0.008, 0.018, 0.044), CAP_BURNISH, -0.32),
        ((0.036, 0.110, 0.036), (0.008, 0.018, 0.044), CAP_BURNISH, 0.32),
        ((-0.0305, 0.106, 0.046), (0.011, 0.010, 0.036), CAP_BURNISH),
        ((0.0305, 0.106, 0.046), (0.011, 0.010, 0.036), CAP_BURNISH),
        # Tapered forepart vamp walls (inner clear width >= 84 mm across y in [+0.022, +0.102])
        ((-0.048, 0.062, 0.038), (0.008, 0.080, 0.044), ESPRESSO, -0.04),
        ((0.048, 0.062, 0.038), (0.008, 0.080, 0.044), ESPRESSO, 0.04),
        # Vamp roof: 62 mm cap-toe ceiling + 72 mm instep ceiling under closed-lacing Oxford saddle
        ((0.0, 0.102, 0.065), (0.080, 0.026, 0.006), ESPRESSO),
        ((0.0, 0.128, 0.062), (0.050, 0.036, 0.006), CAP_BURNISH),
        ((0.0, 0.064, 0.075), (0.086, 0.054, 0.006), ESPRESSO),
        ((0.0, LACE_Y, 0.081), (LACE_W, 0.042, 0.014), ESPRESSO),
        ((0.0, 0.041, 0.082), (0.046, 0.008, 0.016), ESPRESSO),
        ((0.0, 0.034, 0.082), (0.034, 0.005, 0.014), TAN_LINING),
        # Swooping low-cut ankle quarter walls (dip to 45.5 mm across y in [-0.120, +0.022] -> 79 mm clear width)
        ((-0.043, -0.049, 0.029), (0.007, 0.142, 0.028), ESPRESSO, 0.03),
        ((0.043, -0.049, 0.029), (0.007, 0.142, 0.028), ESPRESSO, -0.03),
        ((-0.042, -0.049, 0.044), (0.005, 0.136, 0.003), TAN_LINING, 0.03),
        ((0.042, -0.049, 0.044), (0.005, 0.136, 0.003), TAN_LINING, -0.03),
        # Hollow multi-facet rear heel counter pocket walls (y = -0.139 m inner wall, >= 66 mm clear pocket width)
        ((0.0, -0.1455, 0.043), (0.058, 0.013, 0.054), ESPRESSO),
        ((-0.035, -0.139, 0.043), (0.007, 0.016, 0.054), ESPRESSO, -0.35),
        ((0.035, -0.139, 0.043), (0.007, 0.016, 0.054), ESPRESSO, 0.35),
        ((-0.039, -0.128, 0.043), (0.007, 0.018, 0.054), ESPRESSO, -0.08),
        ((0.039, -0.128, 0.043), (0.007, 0.018, 0.054), ESPRESSO, 0.08),
        # 8 mm inward topline collar undercut (z = 52..70 mm, inner lip at y = -0.131 m -> 270 mm clear throat)
        ((0.0, -0.135, 0.061), (0.056, 0.008, 0.018), ESPRESSO),
        ((0.0, -0.133, 0.0695), (0.054, 0.006, 0.003), TAN_LINING),
    ]
    cyls = [
        ((0.0, 0.1115, 0.007), 0.041, 0.014, WELT_BROWN, "Z"),
        ((0.0, 0.1145, 0.0035), 0.038, 0.007, SOLE_DARK, "Z"),
        ((0.0, -0.1115, 0.007), 0.042, 0.014, SOLE_DARK, "Z"),
        ((0.0, 0.144, 0.030), 0.010, 0.052, CAP_BURNISH, "X"),
        *(((0.0, 0.048 + i * 0.010, 0.088), 0.002, 0.026, LACE_CORD, "X") for i in range(4)),
    ]
    return part("shoe", (SHOE_X, SHOE_Y, MAT_TOP + 0.0005), boxes=boxes, cylinders=cyls,
                mass=SHOE_MASS, color=ESPRESSO, friction=SHOE_MU)


def _tree_part():
    """Twin-tube spring-loaded red cedar shoe tree (292 mm uncompressed, 320 g = 145 g forepart + 175 g heel slide)."""
    p90 = math.pi / 2
    fore_boxes = [
        ((0.0, 0.002, 0.010), (0.068, 0.036, 0.020), CEDAR),
        ((-0.029, 0.002, 0.018), (0.010, 0.034, 0.005), CEDAR_SAP),
        ((0.029, 0.002, 0.018), (0.010, 0.034, 0.005), CEDAR_SAP),
        ((0.0, 0.026, 0.015), (0.034, 0.038, 0.005), CEDAR, (p90, 0.10)),
        ((0.0, 0.034, 0.009), (0.036, 0.032, 0.014), CEDAR),
        ((0.0, 0.018, 0.019), (0.005, 0.064, 0.003), CEDAR_SLOT),
    ]
    fore_cyls = [
        ((0.0, 0.052, 0.010), 0.018, 0.016, CEDAR, "Z"),
        ((-0.016, -0.082, 0.016), 0.006, 0.132, BRASS, "Y"),
        ((0.016, -0.082, 0.016), 0.006, 0.132, BRASS, "Y"),
        ((-0.016, -0.024, 0.016), 0.0075, 0.010, BRASS, "Y"),
        ((0.016, -0.024, 0.016), 0.0075, 0.010, BRASS, "Y"),
    ]
    heel_boxes = [
        ((0.0, -0.181, HEEL_BLOCK_H / 2), (HEEL_BLOCK_W, 0.026, HEEL_BLOCK_H), CEDAR),
        ((0.0, -0.175, HEEL_BLOCK_H - 0.003), (HEEL_BLOCK_W - 0.010, 0.014, 0.006), CEDAR_SAP),
        ((0.0, HANDLE_Y, 0.052), (HANDLE_W, HANDLE_LEN, 0.040), CEDAR),
        ((0.0, HANDLE_Y, HANDLE_TOP_Z - 0.003), (HANDLE_W, HANDLE_LEN, 0.006), BRASS),
        ((0.0, -0.162, 0.050), (0.044, 0.008, 0.036), CEDAR),
    ]
    heel_cyls = [
        ((0.0, HEEL_REAR_Y + 0.022, HEEL_BLOCK_H / 2), 0.022, HEEL_BLOCK_H, CEDAR, "Z"),
        ((-0.016, -0.148, 0.016), 0.0045, 0.044, STEEL_ROD, "Y"),
        ((0.016, -0.148, 0.016), 0.0045, 0.044, STEEL_ROD, "Y"),
    ]
    spring_joint = Joint(
        "spring_tube", kind="slide", axis="Y", anchor=(0.0, HEEL_REAR_Y + HEEL_BLOCK_LEN / 2, 0.018),
        lower=0.0, upper=SLIDE_MAX_M, stiffness=K_TUBE, damping=D_TUBE, target=0.0, initial=0.0,
        mass=TREE_HEEL_MASS, boxes=heel_boxes, cylinders=heel_cyls,
    )
    return part("shoe_tree", (TREE_X, TREE_Y, MAT_TOP + 0.0005), boxes=fore_boxes, cylinders=fore_cyls,
                mass=TREE_FORE_MASS, joints=(spring_joint,), color=CEDAR, friction=TREE_MU)


MAT = _mat_part()
SHOE = _shoe_part()
TREE = _tree_part()
OBJECTS = (MAT, SHOE, TREE)

# -- stateless physical end-state scoring thresholds (every number matches task.json & README) ------------------------
SHOE_DRIFT_MAX_M = 0.065  # shoe stays within 65 mm horizontally of start mark on mat
SHOE_Z_TOL_M = 0.008  # shoe outsole stays within 8 mm of mat surface
SHOE_TILT_MAX_DEG = 10.0  # shoe rests upright within 10 deg
MAT_MARGIN_M = 0.020  # shoe & tree remain inside mat bounds with 20 mm margin
TOE_SEAT_MIN_Y = 0.122  # forepart tip >= 122 mm forward in shoe cavity
TOE_LAT_MAX_M = 0.018  # lateral centring <= 18 mm
INSOLE_BAND_Z = (0.010, 0.030)  # bottom points resting on 16 mm insole (10 to 30 mm above outsole bottom)
ALIGN_MAX_DEG = 12.0  # tree aligned with shoe within 12 deg
ROLL_MAX_DEG = 15.0  # tree roll within 15 deg
HEEL_SEAT_MIN_AFT_M, HEEL_SEAT_MAX_AFT_M = 0.125, 0.144
HEEL_SEAT_Y_BAND = (-HEEL_SEAT_MAX_AFT_M, -HEEL_SEAT_MIN_AFT_M)  # heel rear base 125 to 144 mm aft in counter pocket
HEEL_LAT_MAX_M = 0.018
PRELOAD_BAND_M = (0.008, 0.024)  # 8 to 24 mm active spring compression locking heel beneath 52 mm collar lip
HAND_CLEAR_M = 0.080  # both grippers' TCPs & pad tips >= 80 mm from both shoe and shoe tree

_ENV: dict = {"env": None}
_ids: dict = {}


def _apply_pbr_materials() -> None:
    """Tune Omniverse PreviewSurface roughness & metallic per material colour when running in Isaac Sim."""
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Usd, UsdShade
        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        targets = [(np.asarray(linear_rgb(c), float), r, m) for c, (r, m) in _PBR.items()]
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if prim.GetTypeName() == "Shader" and prim.GetName() == "Shader":
                sh = UsdShade.Shader(prim)
                dc = sh.GetInput("diffuseColor")
                if dc and dc.Get() is not None:
                    col = np.asarray(dc.Get(), float)
                    for tcol, rough, met in targets:
                        if float(np.max(np.abs(col - tcol))) < 0.015:
                            sh.CreateInput("roughness", dc.GetTypeName().ScalarType).Set(float(rough))
                            sh.CreateInput("metallic", dc.GetTypeName().ScalarType).Set(float(met))
                            break
    except Exception:
        pass


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])

def _tilt_deg(R: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))

def _arm_points(env):
    try:
        base = env.unwrapped
        tcps, pads = [], []
        for side in ("left", "right"):
            rob = base.scene[f"{side}_robot"]
            if side not in _ids:
                _ids[side] = rob.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(rob.data.joint_pos)[0, _ids[side]]
            p, rot = ik.tcp_pose(q, station.BASES[side])
            p, rot = np.asarray(p, float), np.asarray(rot, float)
            tcps.append(p)
            pads.append(p + rot[:, 2] * PAD_TIP_PAST_TCP)
        return tuple(tcps), tuple(pads)
    except Exception:
        return None, None

def _in_mat_bounds(p: np.ndarray) -> bool:
    return (
        abs(float(p[0]) - MAT_X) <= MAT_W / 2 - MAT_MARGIN_M
        and abs(float(p[1]) - MAT_Y) <= MAT_L / 2 - MAT_MARGIN_M
        and float(p[2]) >= TOP - 0.005
    )


def _live_arm_points():
    return _arm_points(_ENV.get("env"))

def _geom(sh, tr) -> dict:
    if sh is None or tr is None:
        return {
            "shoe_on_mat": False, "toe_seated": False, "heel_seated": False,
            "preload_locked": False, "hands_clear": False,
            "shoe_drift_m": 1.0, "shoe_tilt_deg": 90.0, "q_slide_m": 0.0,
            "toe_s": np.zeros(3), "heel_s": np.zeros(3),
        }
    Rs, ps = _rot(sh.quat_xyzw), np.asarray(sh.pos, float)
    Rt, pt = _rot(tr.quat_xyzw), np.asarray(tr.pos, float)
    joints = getattr(tr, "joints", None) or {}
    q_slide = float(joints.get("spring_tube", 0.0))

    toe_bot_w = pt + Rt @ np.array([0.0, TOE_TIP_Y, 0.002])
    heel_bot_w = pt + Rt @ np.array([0.0, HEEL_REAR_Y + q_slide, 0.000])
    toe_s = Rs.T @ (toe_bot_w - ps)
    heel_s = Rs.T @ (heel_bot_w - ps)

    R_rel = Rs.T @ Rt
    align_deg = math.degrees(math.acos(float(np.clip(R_rel[1, 1], -1.0, 1.0))))
    roll_deg = math.degrees(math.acos(float(np.clip(R_rel[0, 0], -1.0, 1.0))))

    shoe_drift = math.hypot(float(ps[0]) - SHOE_X, float(ps[1]) - SHOE_Y)
    shoe_dz = abs(float(ps[2]) - MAT_TOP)
    shoe_tilt = _tilt_deg(Rs)

    shoe_on_mat = (
        _in_mat_bounds(ps)
        and _in_mat_bounds(pt)
        and shoe_drift <= SHOE_DRIFT_MAX_M
        and shoe_dz <= SHOE_Z_TOL_M
        and shoe_tilt <= SHOE_TILT_MAX_DEG
    )
    toe_seated = (
        float(toe_s[1]) >= TOE_SEAT_MIN_Y
        and abs(float(toe_s[0])) <= TOE_LAT_MAX_M
        and INSOLE_BAND_Z[0] <= float(toe_s[2]) <= INSOLE_BAND_Z[1]
        and align_deg <= ALIGN_MAX_DEG
        and roll_deg <= ROLL_MAX_DEG
    )
    heel_seated = (
        HEEL_SEAT_Y_BAND[0] <= float(heel_s[1]) <= HEEL_SEAT_Y_BAND[1]
        and abs(float(heel_s[0])) <= HEEL_LAT_MAX_M
        and INSOLE_BAND_Z[0] <= float(heel_s[2]) <= INSOLE_BAND_Z[1]
    )
    preload_locked = PRELOAD_BAND_M[0] <= q_slide <= PRELOAD_BAND_M[1]

    clear = False
    tcps_now, pads_now = _live_arm_points()
    if tcps_now is not None and pads_now is not None and len(tcps_now) >= 2 and len(pads_now) >= 2:
        pts = list(tcps_now) + list(pads_now)
        lo_s, hi_s = np.asarray(sh.low, float), np.asarray(sh.high, float)
        lo_t, hi_t = np.asarray(tr.low, float), np.asarray(tr.high, float)
        d_min = min(
            min(float(np.linalg.norm(np.maximum(lo_s - p, np.maximum(p - hi_s, 0.0)))),
                float(np.linalg.norm(np.maximum(lo_t - p, np.maximum(p - hi_t, 0.0)))))
            for p in pts
        )
        clear = d_min >= HAND_CLEAR_M

    return {
        "shoe_drift_m": shoe_drift,
        "shoe_tilt_deg": shoe_tilt,
        "toe_s": toe_s,
        "heel_s": heel_s,
        "q_slide_m": q_slide,
        "shoe_on_mat": shoe_on_mat,
        "toe_seated": toe_seated,
        "heel_seated": heel_seated,
        "preload_locked": preload_locked,
        "hands_clear": clear,
    }


@on_reset
def _reset(env=None) -> None:
    _ENV["env"] = env
    _ids.clear()
    _apply_pbr_materials()

@on_step
def _bind_env(env) -> None:
    _ENV["env"] = env

def components(objects: dict) -> dict:
    return _geom(objects.get("shoe"), objects.get("shoe_tree"))

def score(objects: dict) -> float:
    """Stateless physical end-state scorer (Possible scores: 0.0, 0.35, 0.70, 1.0)."""
    k = components(objects)
    if not (k["shoe_on_mat"] and k["toe_seated"]):
        return 0.0
    if not (k["heel_seated"] and k["preload_locked"]):
        return 0.35
    return 1.0 if k["hands_clear"] else 0.70

def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    sh, tr = objects.get("shoe"), objects.get("shoe_tree")
    if sh is None or tr is None:
        return 1.0
    k = _geom(sh, tr)
    toe_err = max(0.0, TOE_SEAT_MIN_Y - float(k["toe_s"][1])) + abs(float(k["toe_s"][0]))
    heel_err = float(np.linalg.norm(k["heel_s"] - np.array([0.0, -0.135, INSOLE_Z])))
    return round(toe_err + heel_err, 4)


# -- scripted demo (fixed scene geometry constants only; zero simulator/RNG/state reads) -------------------------------

def _toe_tcp(toe_x: float, toe_y: float, toe_z: float, pitch_rad: float, comp_m: float = 0.0) -> tuple[float, float, float]:
    yaw90 = math.pi / 2
    r_grip0, r_grip = pitch_demo.rotation(yaw90, 0.0, 0.0), pitch_demo.rotation(yaw90, pitch_rad, 0.0)
    pad_w = np.array([toe_x, toe_y, toe_z]) + (r_grip @ r_grip0.T) @ np.array([0.0, HANDLE_Y + comp_m - TOE_TIP_Y, 0.042])
    tcp = pad_w - r_grip[:, 2] * PAD_TIP_PAST_TCP
    return (float(tcp[0]), float(tcp[1]), float(tcp[2]))


def _heel_tcp(hx: float, hy: float, hz: float, pitch_rad: float) -> tuple[float, float, float]:
    yaw90 = math.pi / 2
    r_grip0, r_grip = pitch_demo.rotation(yaw90, 0.0, 0.0), pitch_demo.rotation(yaw90, pitch_rad, 0.0)
    pad_w = np.array([hx, hy, hz]) + (r_grip @ r_grip0.T) @ np.array([0.0, HANDLE_Y - HEEL_REAR_Y, 0.044])
    tcp = pad_w - r_grip[:, 2] * PAD_TIP_PAST_TCP
    return (float(tcp[0]), float(tcp[1]), float(tcp[2]))


def demo():
    """Bimanual 4-phase trajectory from fixed scene constants only (<0.5 mm IK error across all waypoints)."""
    W = Waypoint
    o, c = FINGER_OPEN, FINGER_CLOSED
    yaw90 = math.pi / 2

    lx, ly = SHOE_X, SHOE_Y + 0.134
    lz_grip, lz_press = MAT_TOP + 0.040 + PAD_TIP_PAST_TCP, MAT_TOP + 0.036 + PAD_TIP_PAST_TCP
    park_l, park_r = (0.16, 0.33, TOP + 0.16), (0.16, -0.33, TOP + 0.16)
    p0, p_ins = 0.0, -math.radians(12.0)

    r_pick = _toe_tcp(TREE_X, TREE_Y + TOE_TIP_Y, MAT_TOP + 0.002, p0)
    r_lift = _toe_tcp(TREE_X, TREE_Y + TOE_TIP_Y, MAT_TOP + 0.088, p0)
    r_over = _toe_tcp(SHOE_X, SHOE_Y + 0.015, MAT_TOP + 0.088, p0)
    r_throat = _toe_tcp(SHOE_X, SHOE_Y + 0.030, MAT_TOP + 0.038, p_ins)
    r_mid_vamp = _toe_tcp(SHOE_X, SHOE_Y + 0.098, MAT_TOP + 0.028, p_ins, 0.002)
    r_vamp = _toe_tcp(SHOE_X, SHOE_Y + CAVITY_HALF_Y + 0.008, MAT_TOP + 0.024, p_ins, 0.008)
    # Drive heel slide forward past -131 mm collar lip (~22 mm margin for PhysX compliance), plunge onto 16 mm insole, then lock
    r_comp_hi = _heel_tcp(SHOE_X + 0.017, SHOE_Y - 0.018, MAT_TOP + 0.078, p0)
    r_arc_mid = _heel_tcp(SHOE_X + 0.017, SHOE_Y - 0.016, MAT_TOP + 0.020, p0)
    r_seat_dn = _heel_tcp(SHOE_X + 0.017, SHOE_Y - 0.020, MAT_TOP + INSOLE_Z - 0.008, p0)
    r_preload = _heel_tcp(SHOE_X + 0.008, SHOE_Y - 0.128, MAT_TOP + INSOLE_Z - 0.004, p0)
    r_up = (r_preload[0], r_preload[1], TOP + 0.16)

    right = [
        W(1.1, (r_pick[0], r_pick[1], TOP + 0.16), o, "over tree crest", yaw=yaw90),
        W(0.9, r_pick, o, "around tree crest", yaw=yaw90),
        W(0.3, r_pick, o, "settle centred around crest", yaw=yaw90),
        W(1.0, r_pick, c, "grasp tree crest", yaw=yaw90),
        W(1.1, r_lift, c, "lift tree level clear", yaw=yaw90),
        W(1.4, r_over, c, "carry level over shoe throat", yaw=yaw90),
        W(0.5, r_over, c, "settle centred over throat", yaw=yaw90),
        W(1.0, r_throat, c, "angle nose into instep mouth", yaw=yaw90, pitch=p_ins),
        W(1.0, r_mid_vamp, c, "thread diagonally under vamp", yaw=yaw90, pitch=p_ins),
        W(1.1, r_vamp, c, "seat forepart against toe stop", yaw=yaw90, pitch=p_ins),
        W(1.5, r_comp_hi, c, "compress twin-tube spring past lip", yaw=yaw90, pitch=p0),
        W(1.0, r_arc_mid, c, "drop heel inside collar lip", yaw=yaw90, pitch=p0),
        W(1.0, r_seat_dn, c, "seat heel onto insole", yaw=yaw90, pitch=p0),
        W(0.9, r_preload, c, "engage counter preload", yaw=yaw90, pitch=p0),
        W(0.6, r_preload, o, "release tree crest", yaw=yaw90, pitch=p0),
        W(0.9, r_up, o, "lift clear of shoe", yaw=yaw90, pitch=p0),
        W(1.2, park_r, o, "park right"),
        W(4.2, park_r, o, "hold 3 s post-release"),
    ]
    t_pre_push = sum(w.duration for w in right[:9])
    t_stabilize_done = sum(w.duration for w in right[:-3])
    left = [
        W(1.1, (lx, ly, TOP + 0.16), o, "over oxford cap-toe", yaw=yaw90),
        W(0.9, (lx, ly, lz_grip), o, "around cap-toe nose", yaw=yaw90),
        W(0.7, (lx, ly, lz_press), c, "clamp oxford on mat", yaw=yaw90),
        W(t_pre_push - 2.7, (lx, ly, lz_press), c, "steady oxford during throat entry", yaw=yaw90),
        W(t_stabilize_done - t_pre_push, (lx, ly - 0.018, lz_press), c, "counter-brace oxford against spring push", yaw=yaw90),
        W(0.6, (lx, ly - 0.018, lz_press), o, "open jaws on cap-toe", yaw=yaw90),
        W(0.5, (lx, ly + 0.022, lz_grip + 0.015), o, "slide forward clear of nose", yaw=yaw90),
        W(0.8, (lx, ly + 0.022, TOP + 0.16), o, "lift clear of oxford", yaw=yaw90),
        W(1.1, park_l, o, "park left"),
    ]
    tl, tr_t = sum(w.duration for w in left), sum(w.duration for w in right)
    if tl < tr_t:
        left.append(W(tr_t - tl, park_l, o, "hold 3 s post-release"))
    return {"left": left, "right": right}
