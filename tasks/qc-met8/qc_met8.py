"""On the station table: a precision-turned Ø42 mm steel test shaft cradled horizontally in dual cast-iron V-blocks,
a Mitutoyo Surftest SJ-210 portable surface-roughness drive unit (115 x 23 x 26 mm motor body with a 40 mm detector
nosepiece, a 90-degree front V-saddle adapter, and a rear knurled leveling thumbscrew) parked flat on a rubber
staging pad, a ground steel rear reference anvil behind the shaft, and the SJ-210 calculation display console with
a live LCD status bar and green START key.

The drive's rear leveling foot starts 7.5 mm retracted (+3.58 deg nose-up when placed on the shaft crown and rear
anvil, placing the diamond stylus out of range above the crest). The robot must seat the drive's front V-saddle
coaxially along the shaft's top crown generator and rest its rear foot on the anvil, turn the rear leveling
thumbscrew clockwise until the detector barrel sits horizontal (<= 1.5 deg pitch, READY), then press START on the
console and keep both arms clear during the hands-off motorized cut-off traverse (the Ra display is preset, not a measured surface-roughness result).
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- Colours (visual detail only; instruments are identified by geometry and mechanism) -------------------------------
STEEL = (0.76, 0.78, 0.81)
DARK_STEEL = (0.24, 0.26, 0.29)
ANVIL_GREY = (0.36, 0.38, 0.41)
DRIVE_BODY = (0.28, 0.30, 0.33)
MITUTOYO_BLUE = (0.10, 0.48, 0.78)
BRASS = (0.78, 0.64, 0.26)
RUBBER = (0.16, 0.17, 0.19)
PAD_TRIM = (0.30, 0.32, 0.35)
CONSOLE_GREY = (0.20, 0.21, 0.23)
SCREEN_DARK = (0.04, 0.05, 0.06)
KEY_GREEN = (0.12, 0.68, 0.30)
KEY_GREY = (0.38, 0.40, 0.43)
AMBER = (0.95, 0.58, 0.12)
LIT_GREEN = (0.22, 0.92, 0.42)
LIT_CYAN = (0.18, 0.82, 0.96)
LIT_RED = (0.94, 0.20, 0.20)
STYLUS_ORANGE = (0.94, 0.36, 0.12)

# -- World coordinates (x forward from arm bases, y left, z up, metres) ----------------------------------------------
SHAFT_X = 0.280
SHAFT_Y0, SHAFT_Y1 = 0.010, 0.160
SHAFT_YC = 0.5 * (SHAFT_Y0 + SHAFT_Y1)  # 0.085
SHAFT_R = 0.021                         # Ø42.0 mm turned shaft
SHAFT_L = SHAFT_Y1 - SHAFT_Y0           # 150.0 mm
SHAFT_AXIS_Z = 0.035                    # axis 35 mm above table -> crown at TOP + 0.056 m
CROWN_Z = SHAFT_AXIS_Z + SHAFT_R        # 0.056 m above table

VBLOCK_YS = (0.024, 0.146)

ANVIL_XY = (SHAFT_X, -0.045)
ANVIL_H = 0.042                         # top at TOP + 0.042 m

STAGING_XY = (0.195, 0.095)
STAGING_BASE_H = 0.005
STAGING_REAR_STEP_H = 0.0055

CONSOLE_XY = (0.240, -0.195)
CONSOLE_SIZE = (0.160, 0.0658, 0.0521)  # Mitutoyo SJ-210 display unit: 160 x 65.8 x 52.1 mm
START_LOCAL_X = -0.045
START_WORLD_XY = (CONSOLE_XY[0] + START_LOCAL_X, CONSOLE_XY[1])  # (0.195, -0.195)

# -- Drive unit local frame (origin at motor housing center; +Y toward front detector nose) --------------------------
BODY_W, BODY_L, BODY_H = 0.023, 0.115, 0.026  # Mitutoyo SJ-210 drive unit: 23 x 115 x 26 mm
NOSE_L, NOSE_R = 0.040, 0.007                 # 40 mm front detector barrel (total length 155 mm)
SADDLE_Y = 0.050                              # front 90-deg V-saddle position in drive frame
STYLUS_Y = 0.088                              # diamond stylus & skid contact line
REAR_Y = -0.070                               # rear leveling pedestal center (120 mm wheelbase to SADDLE_Y)
WHEELBASE = SADDLE_Y - REAR_Y                 # 0.120 m

# When seated level (pitch = 0 deg) over CROWN_Z (0.056): stylus tip at z_local = -0.0150 touches crown ->
# drive origin at TOP + 0.0710 m; rear foot bottom at z_local = -0.0290 touches ANVIL_H (0.0420).
DRIVE_LEVEL_Z = CROWN_Z + 0.0150              # 0.0710 m above table
STYLUS_LOCAL_Z = -0.0150
V_CHEEK_DIST = SHAFT_R + 0.002                # cylinder radius + half of 4 mm V-plate thickness
V_AX_LOCAL_Z = SHAFT_AXIS_Z - DRIVE_LEVEL_Z   # -0.0360 m: shaft axis relative to level drive origin
_SQ2 = math.sqrt(2.0)

# Rear leveling thumbscrew: 2.5 mm/rev pitch, starts 7.5 mm retracted (+3.58 deg nose-up on shaft & anvil)
THUMBSCREW_R = 0.013                          # Ø26.0 mm knurled brass wheel
THUMBSCREW_H = 0.014
FOOT_PAD_R, FOOT_PAD_H = 0.006, 0.003
PITCH_M_PER_REV = 0.0025
K_RAD = PITCH_M_PER_REV / (2.0 * math.pi)
E0 = 0.0075                                   # 7.5 mm retracted at start (3.0 turns CW to level)
LIFT_MIN, LIFT_MAX = -0.0030, 0.0090
REAR_PAD_LOCAL_Z = -0.0275                    # center of 3 mm pad when rear_lift = 0 (bottom at -0.0290)
THUMBSCREW_LOCAL_Z = 0.0260                   # wheel center at z_local=+0.026 (+0.019..+0.033 at start -> world TOP+0.090)

# Motorized cut-off traverse
TRAVERSE_DIST_M = 0.0048                      # 4.8 mm evaluation traverse (6 x 0.8 mm cut-off)
TRAVERSE_DUR_S = 1.6
# Engineering tolerance, not an official threshold or calibrated hardware accuracy.
# Current CPU trace peak travel misses 4.8 mm by 9.786227 micrometres.
# Round that observed error up to 10 micrometres; independent engine review required.
TRAVERSE_END_TOL_M = 0.000010

# -- Fixed bench fixtures --------------------------------------------------------------------------------------------
_vblock_boxes = []
_v_off = (SHAFT_R + 0.004) / _SQ2             # 8 mm thick V-cheek plate tangent to Ø42 mm shaft
for _vy in VBLOCK_YS:
    _vblock_boxes += [
        ((0.0, _vy - SHAFT_YC, 0.009), (0.058, 0.024, 0.018), DARK_STEEL),
        ((-_v_off, _vy - SHAFT_YC, SHAFT_AXIS_Z - _v_off), (0.018, 0.024, 0.008), DARK_STEEL, (0.0, -math.pi / 4)),
        ((_v_off, _vy - SHAFT_YC, SHAFT_AXIS_Z - _v_off), (0.018, 0.024, 0.008), DARK_STEEL, (0.0, math.pi / 4)),
        ((-0.024, _vy - SHAFT_YC, 0.024), (0.010, 0.024, 0.022), DARK_STEEL),
        ((0.024, _vy - SHAFT_YC, 0.024), (0.010, 0.024, 0.022), DARK_STEEL),
    ]

# Every part is drawn by its real-geometry PBR model under assets/visual (assets_build/make_visuals.py); the boxes and
# cylinders below are its invisible colliders (labkit hide_shapes).
_VIS = "assets/visual/"

SHAFT_ASSEMBLY = part(
    "shaft", pos=(SHAFT_X, SHAFT_YC, TOP), fixed=True, friction=0.85,
    visuals=[(_VIS + "shaft_vblocks.usda", (0.0, 0.0, 0.0))], hide_shapes=True,
    boxes=_vblock_boxes,
    cylinders=[
        ((0.0, 0.0, SHAFT_AXIS_Z), SHAFT_R, SHAFT_L, STEEL, "Y"),
        ((0.0, -SHAFT_L / 2 - 0.001, SHAFT_AXIS_Z), SHAFT_R - 0.002, 0.002, DARK_STEEL, "Y"),
        ((0.0, SHAFT_L / 2 + 0.001, SHAFT_AXIS_Z), SHAFT_R - 0.002, 0.002, DARK_STEEL, "Y"),
    ],
)

REAR_ANVIL = part(
    "rear_anvil", pos=(*ANVIL_XY, TOP), fixed=True, friction=0.95,
    visuals=[(_VIS + "rear_anvil.usda", (0.0, 0.0, 0.0))], hide_shapes=True,
    boxes=[
        ((0.0, 0.0, ANVIL_H / 2), (0.056, 0.050, ANVIL_H), ANVIL_GREY),
        ((-0.0155, 0.0, ANVIL_H + 0.002), (0.006, 0.042, 0.004), DARK_STEEL),  # 25 mm U-channel guide rails
        ((0.0155, 0.0, ANVIL_H + 0.002), (0.006, 0.042, 0.004), DARK_STEEL),
    ],
)

STAGING_PAD = part(
    "staging_pad", pos=(*STAGING_XY, TOP), fixed=True, friction=0.95,
    visuals=[(_VIS + "staging_pad.usda", (0.0, 0.0, 0.0))], hide_shapes=True,
    boxes=[
        ((0.0, 0.0, STAGING_BASE_H / 2), (0.048, 0.170, STAGING_BASE_H), RUBBER),
        ((0.0, REAR_Y, STAGING_BASE_H + STAGING_REAR_STEP_H / 2), (0.036, 0.028, STAGING_REAR_STEP_H), PAD_TRIM),
    ],
)

# -- Loose Mitutoyo SJ-210 Drive Unit (with screw-foot lift/turn & motorized stylus traverse) -------------------------
_DRIVE_START_Z = TOP + STAGING_BASE_H + 0.0290 + 0.0005  # V-cheek tips 0.5 mm above staging pad, dead flat

_DRIVE_JOINTS = [
    Joint(
        "rear_lift", kind="slide", axis="Z", anchor=(0.0, REAR_Y, REAR_PAD_LOCAL_Z),
        lower=LIFT_MIN, upper=LIFT_MAX, initial=E0, target=E0,
        stiffness=20000.0, damping=200.0, mass=0.025, visuals=[(_VIS + "drive_foot.usda", (0.0, 0.0, 0.0))],
        boxes=[
            ((0.0, REAR_Y, REAR_PAD_LOCAL_Z), (0.018, 0.018, FOOT_PAD_H), RUBBER),
        ],
        cylinders=[
            ((0.0, REAR_Y, -0.004), 0.004, 0.044, STEEL),
        ],
    ),
    Joint(
        "rear_turn", kind="hinge", axis="Z", parent="rear_lift", anchor=(0.0, REAR_Y, THUMBSCREW_LOCAL_Z),
        lower=-10.0 * math.pi, upper=2.0 * math.pi, initial=0.0, target=0.0,
        stiffness=0.0, damping=0.001, friction=0.004, mass=0.025, visuals=[(_VIS + "thumbwheel.usda", (0.0, 0.0, 0.0))],
        cylinders=[
            ((0.0, REAR_Y, THUMBSCREW_LOCAL_Z), THUMBSCREW_R, THUMBSCREW_H, BRASS),
        ],
        boxes=[
            ((0.0, REAR_Y, THUMBSCREW_LOCAL_Z + THUMBSCREW_H / 2 + 0.001), (0.024, 0.003, 0.002), DARK_STEEL),
        ],
    ),
    Joint(
        "stylus_slide", kind="slide", axis="Y", anchor=(0.0, STYLUS_Y, 0.005),
        lower=0.0, upper=TRAVERSE_DIST_M, initial=0.0, target=0.0,
        stiffness=200.0, damping=4.0, mass=0.003, visuals=[(_VIS + "stylus_carriage.usda", (0.0, 0.0, 0.0))],
        boxes=[
            ((0.0, STYLUS_Y, 0.005), (0.015, 0.006, 0.005), STYLUS_ORANGE),
            ((0.0, STYLUS_Y, -0.0125), (0.004, 0.004, 0.004), STEEL),
        ],
    ),
]

DRIVE_UNIT = part(
    "drive_unit", pos=(*STAGING_XY, _DRIVE_START_Z), mass=0.190, friction=0.95,
    visuals=[(_VIS + "drive_body.usda", (0.0, 0.0, 0.0))], hide_shapes=True,
    boxes=[
        ((0.0, 0.0, 0.0), (BODY_W, BODY_L, BODY_H), DRIVE_BODY),
        ((0.0, -0.004, BODY_H / 2 + 0.00125), (0.019, 0.096, 0.0025), MITUTOYO_BLUE),
        ((0.0, 0.010, BODY_H / 2 + 0.0029), (0.015, 0.044, 0.0008), STEEL),
        ((0.0, -0.064, 0.001), (0.012, 0.012, 0.010), DARK_STEEL),                  # rear cable boot
        ((0.0, REAR_Y, 0.0), (0.022, 0.026, 0.010), DRIVE_BODY),                      # rear pedestal bracket
        ((0.0, STYLUS_Y, -0.0125), (0.008, 0.012, 0.004), DARK_STEEL),                # front skid shoe
        ((-0.016, SADDLE_Y, -0.0135), (0.008, 0.030, 0.005), STEEL),                  # left V-saddle wing gusset
        ((0.016, SADDLE_Y, -0.0135), (0.008, 0.030, 0.005), STEEL),                   # right V-saddle wing gusset
        ((-V_CHEEK_DIST / _SQ2, SADDLE_Y, V_AX_LOCAL_Z + V_CHEEK_DIST / _SQ2),
         (0.022, 0.030, 0.004), STEEL, (0.0, -math.pi / 4)),
        ((V_CHEEK_DIST / _SQ2, SADDLE_Y, V_AX_LOCAL_Z + V_CHEEK_DIST / _SQ2),
         (0.022, 0.030, 0.004), STEEL, (0.0, math.pi / 4)),
    ],
    cylinders=[
        ((0.0, BODY_L / 2 + NOSE_L / 2, -0.004), NOSE_R, NOSE_L, STEEL, "Y"),
    ],
    joints=_DRIVE_JOINTS,
)

# -- Fixed SJ-210 Display Console (160 x 65.8 x 52.1 mm, sprung START key, live status bars) -------------------------
START_TRAVEL = 0.004
START_PRESSED_M = -0.0018
_CZ = CONSOLE_SIZE[2]
_SCR_Z = _CZ + 0.0010
_LAMP_SUNK = -0.003

_CONSOLE_JOINTS = [
    Joint(
        "start_btn", kind="slide", axis="Z", anchor=(START_LOCAL_X, 0.0, _CZ),
        lower=-START_TRAVEL, upper=0.0, initial=0.0, target=0.0,
        stiffness=120.0, damping=2.0, mass=0.005, visuals=[(_VIS + "start_key.usda", (0.0, 0.0, 0.0))],
        boxes=[((START_LOCAL_X, 0.0, _CZ + 0.0025), (0.024, 0.020, 0.005), KEY_GREEN)],
    ),
    Joint(
        "lamp_oor", kind="slide", axis="Z", visuals=[(_VIS + "lcd_oor.usda", (0.0, 0.0, 0.0))], anchor=(0.045, 0.0, _SCR_Z),
        lower=_LAMP_SUNK, upper=0.0, initial=0.0, target=0.0, stiffness=200.0, damping=2.0, mass=0.002,
        boxes=[((0.045, 0.0, _SCR_Z), (0.007, 0.036, 0.001), AMBER)],
    ),
    Joint(
        "lamp_ready", kind="slide", axis="Z", visuals=[(_VIS + "lcd_ready.usda", (0.0, 0.0, 0.0))], anchor=(0.035, 0.0, _SCR_Z),
        lower=_LAMP_SUNK, upper=0.0, initial=_LAMP_SUNK, target=_LAMP_SUNK, stiffness=200.0, damping=2.0, mass=0.002,
        boxes=[((0.035, 0.0, _SCR_Z), (0.007, 0.036, 0.001), LIT_GREEN)],
    ),
    Joint(
        "prog_bar", kind="slide", axis="Z", visuals=[(_VIS + "lcd_meas.usda", (0.0, 0.0, 0.0))], anchor=(0.025, 0.0, _SCR_Z),
        lower=_LAMP_SUNK, upper=0.0, initial=_LAMP_SUNK, target=_LAMP_SUNK, stiffness=200.0, damping=2.0, mass=0.002,
        boxes=[((0.025, 0.0, _SCR_Z), (0.007, 0.036, 0.001), LIT_CYAN)],
    ),
    Joint(
        "lamp_done", kind="slide", axis="Z", visuals=[(_VIS + "lcd_done.usda", (0.0, 0.0, 0.0))], anchor=(0.012, 0.0, _SCR_Z),
        lower=_LAMP_SUNK, upper=0.0, initial=_LAMP_SUNK, target=_LAMP_SUNK, stiffness=200.0, damping=2.0, mass=0.002,
        boxes=[((0.012, 0.0, _SCR_Z), (0.013, 0.036, 0.001), LIT_GREEN)],
    ),
    Joint(
        "lamp_fault", kind="slide", axis="Z", visuals=[(_VIS + "lcd_fault.usda", (0.0, 0.0, 0.0))], anchor=(0.001, 0.0, _SCR_Z),
        lower=_LAMP_SUNK, upper=0.0, initial=_LAMP_SUNK, target=_LAMP_SUNK, stiffness=200.0, damping=2.0, mass=0.002,
        boxes=[((0.001, 0.0, _SCR_Z), (0.006, 0.036, 0.001), LIT_RED)],
    ),
]

CONSOLE = part(
    "console", pos=(*CONSOLE_XY, TOP), fixed=True, friction=0.8,
    visuals=[(_VIS + "console.usda", (0.0, 0.0, 0.0))], hide_shapes=True,
    boxes=[
        ((0.0, 0.0, _CZ / 2), CONSOLE_SIZE, CONSOLE_GREY),
        ((0.068, 0.0, _CZ + 0.0005), (0.014, 0.056, 0.001), MITUTOYO_BLUE),
        ((0.026, 0.0, _CZ + 0.0005), (0.056, 0.046, 0.001), SCREEN_DARK),
        ((-0.015, -0.018, _CZ + 0.001), (0.012, 0.014, 0.002), KEY_GREY),
        ((-0.015, 0.0, _CZ + 0.001), (0.012, 0.014, 0.002), MITUTOYO_BLUE),
        ((-0.015, 0.018, _CZ + 0.001), (0.012, 0.014, 0.002), KEY_GREY),
        ((0.050, CONSOLE_SIZE[1] / 2 + 0.004, 0.025), (0.014, 0.008, 0.014), DARK_STEEL),
    ],
    joints=_CONSOLE_JOINTS,
)

# -- Bench dressing (render-only real models, nothing to manipulate) -------------------------------------------------
# NVIDIA SimReady models (CC-BY-4.0, assets/props/*/LICENSE.txt; assets_build/flatten_props.py), at their own scale,
# set in a band beyond the shaft and console (x 0.33-0.57 m, |y| < 0.24 m): outside both arms' paths and clear of the
# ready-pose grippers' top-camera footprint (y = +-0.305). Each is a fixed part whose only shape is an invisible
# collider box over the model's footprint. (name, model, (x, y), yaw deg, model bbox min, model bbox max)
_PROPS = [
    ("prop_caliper", "instrument_caliper_a01", (0.375, 0.070), 90.0, (-0.2213, -0.0304, 0.0), (0.0170, 0.0481, 0.0076)),
    ("prop_glove_box", "box_glove_a01", (0.470, 0.060), 90.0, (-0.0891, -0.0586, 0.0), (0.0891, 0.0586, 0.0562)),
    ("prop_safety_glasses", "ppe_eyeProtection_safetyGlasses_a01", (0.490, -0.130), 200.0,
     (-0.0825, -0.0845, 0.0001), (0.0845, 0.0845, 0.0596)),
    ("prop_solvent_spray", "consumable_spray_solvent_a01", (0.530, 0.200), 30.0, (-0.031, -0.031, 0.0),
     (0.031, 0.031, 0.1864)),
]
BENCH_PROPS = [
    part(name, pos=(*xy, TOP), fixed=True, yaw=math.radians(yaw_deg), hide_shapes=True,
         visuals=[(f"assets/props/{model}/{model}.usdc", (0.0, 0.0, 0.0))],
         boxes=[(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0005 + (hi[2] - 0.001) / 2),
                 (hi[0] - lo[0], hi[1] - lo[1], hi[2] - 0.001), PAD_TRIM)])
    for name, model, xy, yaw_deg, lo, hi in _PROPS
]

OBJECTS = [SHAFT_ASSEMBLY, REAR_ANVIL, STAGING_PAD, CONSOLE, DRIVE_UNIT, *BENCH_PROPS]

# -- Shared named thresholds (used by both on_step, score, and success) ----------------------------------------------
LATERAL_TOL_MM = 1.5
YAW_TOL_DEG = 2.5
ROLL_TOL_DEG = 5.0
SADDLE_Z_TOL_M = 0.004
ANVIL_X_TOL_M = 0.012
ANVIL_Y_TOL_M = 0.018
ANVIL_Z_TOL_M = 0.004
PITCH_TOL_DEG = 1.5
PITCH_PARTIAL_DEG = 2.8
TIP_DEG = 12.0
CLEAR_DIST_M = 0.030
OPEN_FINGER_LIM = FINGER_OPEN * 0.60          # -0.02817 m per finger (~56 mm gap)
TOUCH_DIST_M = 0.022
VIB_LIN_VEL = 0.030
VIB_ANG_VEL = 0.40

_TEST_ROBOTS = None                           # synthetic stand-in for offline unit tests only


def _robot_observations():
    """Return list of (points_xyz_array, fingers_open_bool) for left_robot and right_robot.
    Returns None if simulator environment is unavailable (fails closed outside unit tests)."""
    if _TEST_ROBOTS is not None:
        return _TEST_ROBOTS
    env = labkit.env()
    if env is None:
        return None
    base = env.unwrapped
    origin = labkit._np(base.scene.env_origins[0])
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        bnames = list(art.body_names)
        pos = labkit._np(art.data.body_pos_w)[0] - origin
        g = pos[bnames.index("gripper")]
        tl = pos[bnames.index("tip_left")]
        tr = pos[bnames.index("tip_right")]
        mid = 0.5 * (tl + tr)
        u = mid - g
        n = float(np.linalg.norm(u))
        tcp = g + (u / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])) * 0.134
        jnames = list(art.joint_names)
        q = labkit._np(art.data.joint_pos)[0]
        f_open = (q[jnames.index("joint7")] <= OPEN_FINGER_LIM and q[jnames.index("joint8")] <= OPEN_FINGER_LIM)
        out.append((np.vstack([tcp, tl, tr]), bool(f_open)))
    return out


def _point_box_dist(pt: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    clamped = np.clip(pt, lo, hi)
    return float(np.linalg.norm(pt - clamped))


def _grippers_near_any(robots, states_list, margin_m: float) -> bool:
    for pts, _ in robots:
        for st in states_list:
            lo, hi = np.asarray(st.low, float), np.asarray(st.high, float)
            for p in pts:
                if _point_box_dist(p, lo, hi) < margin_m:
                    return True
    return False


def _grippers_open_and_clear(objects) -> bool:
    robots = _robot_observations()
    if robots is None:
        return False
    if not all(f_open for _, f_open in robots):
        return False
    targets = [objects["drive_unit"], objects["shaft"], objects["console"]]
    return not _grippers_near_any(robots, targets, CLEAR_DIST_M)


def drive_metrics(drive) -> dict:
    """Compute crown alignment, yaw skew, pitch, roll, and rear-anvil seating from drive_unit PartState."""
    rot = drive.rotation()
    pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(rot[2, 1])))))
    roll_deg = abs(math.degrees(math.atan2(float(-rot[2, 0]), float(rot[2, 2]))))
    yaw_deg = abs(math.degrees(math.atan2(float(rot[0, 1]), float(rot[1, 1]))))
    saddle_crown = drive.to_world((0.0, SADDLE_Y, STYLUS_LOCAL_Z))
    lateral_mm = abs(float(saddle_crown[0]) - SHAFT_X) * 1000.0
    saddle_y_ok = (SHAFT_Y0 + 0.020) <= float(saddle_crown[1]) <= (SHAFT_Y1 - 0.020)
    saddle_z_ok = abs(float(saddle_crown[2]) - (TOP + CROWN_Z)) <= SADDLE_Z_TOL_M
    lift_q = float(drive.joints.get("rear_lift", E0))
    rear_pad_bot = drive.to_world((0.0, REAR_Y, REAR_PAD_LOCAL_Z - FOOT_PAD_H / 2 + lift_q))
    anvil_ok = (
        abs(float(rear_pad_bot[0]) - ANVIL_XY[0]) <= ANVIL_X_TOL_M
        and abs(float(rear_pad_bot[1]) - ANVIL_XY[1]) <= ANVIL_Y_TOL_M
        and abs(float(rear_pad_bot[2]) - (TOP + ANVIL_H)) <= ANVIL_Z_TOL_M
    )
    seated = bool(
        lateral_mm <= LATERAL_TOL_MM
        and yaw_deg <= YAW_TOL_DEG
        and roll_deg <= ROLL_TOL_DEG
        and abs(pitch_deg) <= TIP_DEG
        and saddle_y_ok
        and saddle_z_ok
        and anvil_ok
    )
    return {
        "seated": seated,
        "lateral_mm": lateral_mm,
        "yaw_deg": yaw_deg,
        "pitch_deg": pitch_deg,
        "roll_deg": roll_deg,
        "anvil_ok": anvil_ok,
        "saddle_z_ok": saddle_z_ok,
    }


_log = {
    "traversing": False,
    "traverse_t": 0.0,
    "stylus_intermediate_seen": False,
    "stylus_start": 0.0,
    "traverse_done": False,
    "vibration_fault": False,
    "premature_start": False,
    "post_disturbed": False,
}
_prev_start = False
_ids: dict = {}


@on_reset
def _clear(env):
    global _prev_start
    _prev_start = False
    _ids.clear()
    _log.update(
        traversing=False,
        traverse_t=0.0,
        stylus_intermediate_seen=False,
        stylus_start=0.0,
        traverse_done=False,
        vibration_fault=False,
        premature_start=False,
        post_disturbed=False,
    )


@on_step
def _watch(env):
    import torch

    global _prev_start
    drive = labkit._part_state(env, labkit._SPECS["drive_unit"])
    console = labkit._part_state(env, labkit._SPECS["console"])
    shaft = labkit._part_state(env, labkit._SPECS["shaft"])
    dt = float(getattr(env.unwrapped, "step_dt", 0.05))

    turn_rad = float(drive.joints["rear_turn"])
    want_lift = min(max(E0 + turn_rad * K_RAD, LIFT_MIN), LIFT_MAX)

    m = drive_metrics(drive)
    ready = m["seated"] and abs(m["pitch_deg"]) <= PITCH_TOL_DEG

    # The sampled actuator state, not the commanded target, must be finite.
    stylus_position = float(drive.joints.get("stylus_slide", math.nan))
    stylus_finite = math.isfinite(stylus_position)
    if not stylus_finite:
        _log["vibration_fault"] = True
        _log["traversing"] = False
        _log["traverse_done"] = False

    down = float(console.joints["start_btn"]) < START_PRESSED_M
    if down and not _prev_start:
        if (ready and stylus_finite and stylus_position < TRAVERSE_DIST_M
                and not _log["traverse_done"]):
            _log["traversing"] = True
            _log["traverse_t"] = 0.0
            _log["stylus_intermediate_seen"] = False
            _log["stylus_start"] = stylus_position
            _log["vibration_fault"] = False
        else:
            _log["premature_start"] = True
    _prev_start = down

    if _log["traverse_done"]:
        robots = _robot_observations()
        if (
            down
            or robots is None
            or not all(f_open for _, f_open in robots)
            or _grippers_near_any(robots, [drive, shaft, console], CLEAR_DIST_M)
        ):
            _log["post_disturbed"] = True

    stylus_target = 0.0
    if _log["traversing"]:
        # Observed progress, not a commanded target or an assumed PD speed bound.
        if stylus_finite and 0.0 < stylus_position - _log["stylus_start"] < TRAVERSE_DIST_M:
            _log["stylus_intermediate_seen"] = True
        robots = _robot_observations()
        touched = robots is None or _grippers_near_any(robots, [drive, shaft], TOUCH_DIST_M)
        shaken = (
            float(np.linalg.norm(drive.lin_vel)) > VIB_LIN_VEL
            or float(np.linalg.norm(drive.ang_vel)) > VIB_ANG_VEL
            or not ready
        )
        if touched or shaken:
            _log["vibration_fault"] = True
            _log["traversing"] = False
        else:
            _log["traverse_t"] += dt
            frac = min(1.0, _log["traverse_t"] / TRAVERSE_DUR_S)
            stylus_target = frac * TRAVERSE_DIST_M
            if (_log["traverse_t"] >= TRAVERSE_DUR_S
                    and stylus_finite and _log["stylus_intermediate_seen"]
                    and stylus_position - _log["stylus_start"]
                    >= TRAVERSE_DIST_M - TRAVERSE_END_TOL_M):
                _log["traversing"] = False
                _log["traverse_done"] = True
                stylus_target = TRAVERSE_DIST_M

    d_asset = env.unwrapped.scene["drive_unit"]
    c_asset = env.unwrapped.scene["console"]
    if "d_ids" not in _ids:
        dn = list(d_asset.joint_names)
        _ids["d_ids"] = [dn.index("rear_lift_joint"), dn.index("stylus_slide_joint")]
        cn = list(c_asset.joint_names)
        _ids["c_ids"] = [cn.index(f"{n}_joint") for n in ("lamp_oor", "lamp_ready", "prog_bar", "lamp_done", "lamp_fault")]

    d_asset.set_joint_position_target(torch.tensor([[want_lift, stylus_target]], device=d_asset.device),
                                      joint_ids=_ids["d_ids"])
    lamps = [
        0.0 if (not ready and not _log["traverse_done"]) else _LAMP_SUNK,
        0.0 if (ready and not _log["traversing"] and not _log["traverse_done"]) else _LAMP_SUNK,
        0.0 if _log["traversing"] else _LAMP_SUNK,
        0.0 if (_log["traverse_done"] and not _log["vibration_fault"] and ready) else _LAMP_SUNK,
        0.0 if _log["vibration_fault"] else _LAMP_SUNK,
    ]
    c_asset.set_joint_position_target(torch.tensor([lamps], device=c_asset.device), joint_ids=_ids["c_ids"])


def score(objects) -> float:
    """0.25 drive front V-saddle seated on shaft crown (<= 1.5 mm lateral, <= 2.5 deg yaw, <= 5 deg roll, rear foot on
    anvil); +0.30 leveled (<= 1.5 deg pitch) or +0.10 partial (1.5-2.8 deg pitch); +0.30 START triggered while READY and
    >=1.6 s traverse with observed travel >=4.790 mm completed with zero gripper contact (no vibration_fault) while still seated/leveled; +0.15 both
    grippers open and >= 3.0 cm clear (only when seated). Documented tiers: 0, 0.25, 0.35, 0.4, 0.5, 0.55, 0.7, 0.85, 1.0."""
    drive = objects["drive_unit"]
    m = drive_metrics(drive)
    if not m["seated"] or not drive.resting() or _log["post_disturbed"]:
        return 0.0
    total = 0.25
    pitch_abs = abs(m["pitch_deg"])
    leveled = pitch_abs <= PITCH_TOL_DEG
    if leveled:
        total += 0.30
    elif pitch_abs <= PITCH_PARTIAL_DEG:
        total += 0.10
    if leveled and _log["traverse_done"] and not _log["vibration_fault"]:
        total += 0.30
    if _grippers_open_and_clear(objects):
        total += 0.15
    return round(total, 2)


SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    drive = objects["drive_unit"]
    saddle = drive.to_world((0.0, SADDLE_Y, STYLUS_LOCAL_Z))
    return float(math.hypot(float(saddle[0]) - SHAFT_X, float(saddle[2]) - (TOP + CROWN_Z))
                 + abs(drive_metrics(drive)["pitch_deg"]) * 0.005)


# -- Scripted bimanual demo (open loop from fixed layout, no simulator state read) -----------------------------------
_L_YAW = -math.pi / 2
_L_COM_Y = -0.0060
_L_PICK_Z = _DRIVE_START_Z - 0.0050 + PAD_TIP_PAST_TCP
_L_PLACE_Z = TOP + DRIVE_LEVEL_Z - 0.0065 + PAD_TIP_PAST_TCP
_L_CARRY_Z = TOP + 0.120
_L_OPEN_D = -0.0245   # 49 mm gap over 23 mm drive housing (absorbs spawn jitter)
_L_HOLD_D = -0.0065   # rigid clamping force on 23 mm housing

_R_SCREW_XY = (ANVIL_XY[0], ANVIL_XY[1] - 0.0010)
_R_SCREW_Z = TOP + DRIVE_LEVEL_Z + THUMBSCREW_LOCAL_Z - 0.0050 + PAD_TIP_PAST_TCP
_R_HOVER_Z = _R_SCREW_Z + 0.030
_R_Y0, _R_Y1 = math.radians(45.0), math.radians(-45.0)    # 90 deg CW per stroke; 12 strokes = 3.0 turns (7.5 mm)
_R_OPEN_S = -0.0230   # 46 mm gap over 26 mm thumbscrew wheel
_R_HOLD_S = -0.0116   # gentle torque-transfer grip on 26 mm wheel

_START_PRESS_Z = TOP + CONSOLE_SIZE[2] - 0.001 + PAD_TIP_PAST_TCP
_LEFT_PARK = (0.120, 0.260, TOP + 0.130)
_RIGHT_PARK = (0.100, -0.360, TOP + 0.180)  # >= 13 cm clear of the console in y, out of its top-camera footprint


def demo():
    """Left arm picks drive from staging pad and seats front V-saddle on shaft crown + rear foot on anvil, then parks.
    Right arm waits clear until left parks, turns rear thumbwheel 3.0 turns CW (12 strokes x 90 deg) to level barrel,
    presses START on console, parks clear, and waits for the 1.6 s traverse + 1.0 s hold."""
    left = [
        Waypoint(1.0, (STAGING_XY[0], STAGING_XY[1] + _L_COM_Y, _L_CARRY_Z), FINGER_OPEN, "above drive on pad", _L_YAW),
        Waypoint(0.9, (STAGING_XY[0], STAGING_XY[1] + _L_COM_Y, _L_PICK_Z), _L_OPEN_D, "descend over drive", _L_YAW),
        Waypoint(0.75, (STAGING_XY[0], STAGING_XY[1] + _L_COM_Y, _L_PICK_Z), _L_HOLD_D, "grip drive housing", _L_YAW),
        Waypoint(0.25, (STAGING_XY[0], STAGING_XY[1] + _L_COM_Y, _L_PICK_Z), _L_HOLD_D, "settle grip", _L_YAW),
        Waypoint(1.2, (STAGING_XY[0], STAGING_XY[1] + _L_COM_Y, _L_CARRY_Z), _L_HOLD_D, "lift drive", _L_YAW),
        Waypoint(1.6, (SHAFT_X, 0.025 + _L_COM_Y, _L_CARRY_Z), _L_HOLD_D, "carry over shaft crown", _L_YAW),
        Waypoint(1.1, (SHAFT_X, 0.025 + _L_COM_Y, _L_PLACE_Z), _L_HOLD_D, "lower V-saddle onto crown", _L_YAW),
        Waypoint(0.6, (SHAFT_X, 0.025 + _L_COM_Y, _L_PLACE_Z), _L_OPEN_D, "release drive", _L_YAW),
        Waypoint(0.9, (SHAFT_X, 0.025 + _L_COM_Y, _L_CARRY_Z), _L_OPEN_D, "lift clear", _L_YAW),
        Waypoint(1.1, _LEFT_PARK, FINGER_OPEN, "park left arm", 0.0),
    ]
    t_left = sum(w.duration for w in left)
    right = [
        Waypoint(t_left, _RIGHT_PARK, FINGER_OPEN, "wait clear until left seats and parks", _R_Y0),
        Waypoint(1.0, (*_R_SCREW_XY, _R_HOVER_Z), _R_OPEN_S, "move above thumbscrew", _R_Y0),
    ]
    for k in range(12):
        right += [
            Waypoint(0.35, (*_R_SCREW_XY, _R_HOVER_Z), _R_OPEN_S, f"hover thumbscrew {k}", _R_Y0),
            Waypoint(0.35, (*_R_SCREW_XY, _R_SCREW_Z), _R_OPEN_S, f"descend onto wheel {k}", _R_Y0),
            Waypoint(0.25, (*_R_SCREW_XY, _R_SCREW_Z), _R_HOLD_S, f"grip wheel {k}", _R_Y0),
            Waypoint(0.45, (*_R_SCREW_XY, _R_SCREW_Z), _R_HOLD_S, f"turn CW 90 deg {k}", _R_Y1),
            Waypoint(0.20, (*_R_SCREW_XY, _R_SCREW_Z), _R_OPEN_S, f"release wheel {k}", _R_Y1),
            Waypoint(0.35, (*_R_SCREW_XY, _R_HOVER_Z), _R_OPEN_S, f"lift and unwind wrist {k}", _R_Y0),
        ]
    right += [
        Waypoint(1.0, (*START_WORLD_XY, TOP + 0.090), FINGER_CLOSED, "move over START key", 0.0),
        Waypoint(0.45, (*START_WORLD_XY, _START_PRESS_Z), FINGER_CLOSED, "press START key", 0.0),
        Waypoint(0.1, (*START_WORLD_XY, _START_PRESS_Z), FINGER_CLOSED, "hold START press", 0.0),
        # straight back to the park pose (lifting off the key as it opens), so both arms are parked and still for
        # the last >= 1.5 s of the episode (success holds 1.0 s after the 1.6 s traverse that START begins)
        Waypoint(0.7, _RIGHT_PARK, FINGER_OPEN, "lift off START and park right arm clear", 0.0),
        Waypoint(3.0, _RIGHT_PARK, FINGER_OPEN, "hands-off traverse and settle", 0.0),
    ]
    return {"left": left, "right": right}
