"""Snap an ESD ground cord onto a bench mat corner tab, route clear of the work zone, and plug into a ground block.

Robotics electronics bench electrostatic-discharge (ESD) assembly workcell:
- Fixed static-dissipative bench mat assembly (`esd_bench`) at MAT_CENTER=(0.228, -0.068) with a marked yellow
  PCB keep-out work zone (x = 0.155..0.255 m, y = -0.120..-0.015 m), green PCB workplate, left staging pad,
  front plug guide dock, bench corner retainer (`tab_catch`, 28 g, 14 N/m), and ground block nest at (0.375, -0.205).
- Articulated 50 g ESD mat corner flap (`snap_tab`) hinged along +X (`tab_hinge`, 0.35 N*m/rad curled +14 deg up):
  left counter-hold flattens the corner boss under `tab_catch` so the 10 mm brass male snap stud and separate 28 g
  beveled spring-jaw retainer (`snap_jaw`, 22 N/m) stay level during downward snap engagement.
- Articulated coiled ground cord (`snap_cap` 34 g -> `strain_boot` 26 g -> `cord_prox` 30 g -> `cord_dist` 30 g ->
  `plug_boot` 26 g -> `banana_plug` 34 g) starting clear of the mat; once snapped onto the stud and routed east of
  the work zone (x > 0.255 m), the banana plug drives -Y into the 120 g ground block past a separate 28 g beveled
  socket latch (`jack_latch`, 6 N/m) and holds against a ~5 N reverse pull check.
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
# Import mechkit BEFORE labkit so labkit's sys.meta_path finder wraps mechkit's in Isaac Sim.
import mechkit as mk  # noqa: E402
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- 1:1 manufacturer & workcell geometry (metres, kg, N) ----------------------------------------------------------
MAT_CENTER = (0.228, -0.068)
MAT_SIZE = (0.160, 0.118, 0.004)
MAT_TOP = TOP + MAT_SIZE[2]

WORK_X = (0.155, 0.255)  # marked yellow PCB assembly keep-out zone in X
WORK_Y = (-0.120, -0.015)  # marked yellow PCB assembly keep-out zone in Y

TAB_HINGE = (0.220, 0.018, TOP + 0.002)
TAB_HOLD_LOCAL = (-0.065, 0.088, 0.012)  # world (0.155, +0.106) when flat -> 150 mm from stud
STUD_LOCAL = (+0.075, 0.008, 0.002)  # world (0.295, +0.026) when flat (comfortable reach for right YAM)
STUD_XY = (TAB_HINGE[0] + STUD_LOCAL[0], TAB_HINGE[1] + STUD_LOCAL[1])

CAP_START_XY = (0.115, -0.060)
PLUG_START_XY = (0.255, -0.225)
GB_XY = (0.375, -0.205)
GB_Z0 = TOP + 0.003
GB_SIZE = (0.068, 0.054, 0.030)
SOCKET_Y_WORLD = GB_XY[1] + GB_SIZE[1] / 2  # +Y front face of ground block (y = -0.178 m)

SNAP_CAP_MASS = 0.034
BOOT_MASS = 0.026
CORD_LINK_MASS = 0.030
PLUG_MASS = 0.034
TAB_MASS = 0.050
JAW_MASS = 0.028
GB_MASS = 0.120

# -- distinct PBR colours -------------------------------------------------------------------------------------------
SLATE_BLUE = (0.20, 0.36, 0.56)
MAT_DARK = (0.22, 0.24, 0.28)
YELLOW_WARN = (0.92, 0.76, 0.12)
GREEN_PCB = (0.16, 0.52, 0.28)
ALU_BLUE = (0.22, 0.48, 0.80)
BRASS_GOLD = (0.82, 0.66, 0.22)
STEEL = (0.76, 0.78, 0.81)
COIL_BLACK = (0.16, 0.17, 0.19)
JACK_ORANGE = (0.86, 0.42, 0.14)
GB_BODY = (0.28, 0.30, 0.34)

kit = mk.Kit()
_mx, _my = MAT_CENTER
_gx, _gy = GB_XY
_wx_mid = (WORK_X[0] + WORK_X[1]) / 2
_wy_mid = (WORK_Y[0] + WORK_Y[1]) / 2
kit.part("esd_bench", (0.0, 0.0, TOP), [
    mk.box(MAT_SIZE, at=(_mx, _my, MAT_SIZE[2] / 2), color=SLATE_BLUE),
    mk.box((0.004, WORK_Y[1] - WORK_Y[0], 0.001), at=(WORK_X[0], _wy_mid, 0.0045), color=YELLOW_WARN),
    mk.box((0.004, WORK_Y[1] - WORK_Y[0], 0.001), at=(WORK_X[1], _wy_mid, 0.0045), color=YELLOW_WARN),
    mk.box((WORK_X[1] - WORK_X[0], 0.004, 0.001), at=(_wx_mid, WORK_Y[0], 0.0045), color=YELLOW_WARN),
    mk.box((WORK_X[1] - WORK_X[0], 0.004, 0.001), at=(_wx_mid, WORK_Y[1], 0.0045), color=YELLOW_WARN),
    mk.box((0.080, 0.094, 0.0006), at=(_wx_mid, _wy_mid, 0.0043), color=GREEN_PCB),
    mk.box((0.024, 0.024, 0.0004), at=(_wx_mid, _wy_mid, 0.0048), color=MAT_DARK),
    mk.box((0.042, 0.042, 0.005), at=(CAP_START_XY[0], CAP_START_XY[1], 0.0025), color=MAT_DARK),
    mk.box((0.066, 0.086, 0.0014), at=(PLUG_START_XY[0], PLUG_START_XY[1] - 0.012, 0.0007), color=MAT_DARK),
    mk.box((0.005, 0.032, 0.008), at=(PLUG_START_XY[0] - 0.0205, PLUG_START_XY[1] + 0.002, 0.0047), color=ALU_BLUE),
    mk.box((0.005, 0.032, 0.008), at=(PLUG_START_XY[0] + 0.0205, PLUG_START_XY[1] + 0.002, 0.0047), color=ALU_BLUE),
    mk.box((0.005, 0.022, 0.010), at=(PLUG_START_XY[0] - 0.0195, PLUG_START_XY[1] - 0.028, 0.0057), color=ALU_BLUE),
    mk.box((0.005, 0.022, 0.010), at=(PLUG_START_XY[0] + 0.0195, PLUG_START_XY[1] - 0.028, 0.0057), color=ALU_BLUE),
    mk.box((0.044, 0.005, 0.010), at=(PLUG_START_XY[0], PLUG_START_XY[1] - 0.0495, 0.0057), color=ALU_BLUE),
    # Bench pocket around 120 g common-point ground block (2 mm rear backstop + 6 mm side nest)
    mk.box((0.086, 0.068, 0.003), at=(_gx, _gy, 0.0015), color=MAT_DARK),
    mk.box((0.086, 0.006, 0.018), at=(_gx, _gy - GB_SIZE[1] / 2 - 0.005, 0.011), color=MAT_DARK),
    mk.box((0.006, 0.068, 0.018), at=(_gx - GB_SIZE[0] / 2 - 0.006, _gy, 0.011), color=MAT_DARK),
    mk.box((0.006, 0.068, 0.008), at=(_gx + GB_SIZE[0] / 2 + 0.006, _gy, 0.0055), color=MAT_DARK),
    mk.box((0.014, 0.006, 0.012), at=(_gx - 0.034, _gy + GB_SIZE[1] / 2 + 0.006, 0.0075), color=MAT_DARK),
    mk.box((0.014, 0.006, 0.008), at=(_gx + 0.034, _gy + GB_SIZE[1] / 2 + 0.006, 0.0055), color=MAT_DARK),
], mass=None, color=SLATE_BLUE)

# Articulated 50 g ESD mat corner flap curled +14 deg up at start around +X, with steep 55 deg funnel cheeks around stud
_q_fy_n = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle([1, 0, 0], math.radians(-55.0))))
_q_fy_p = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle([1, 0, 0], math.radians(55.0))))
_q_fx_p = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle([0, 1, 0], math.radians(-55.0))))
kit.part("snap_tab", TAB_HINGE, [
    mk.box((0.185, 0.098, 0.004), at=(+0.002, 0.050, 0.0), color=SLATE_BLUE),
    mk.box((0.036, 0.026, 0.024), at=TAB_HOLD_LOCAL, color=ALU_BLUE),
    mk.box((0.013, 0.013, 0.0014), at=(STUD_LOCAL[0], STUD_LOCAL[1], 0.0027), color=STEEL),
    mk.box((0.005, 0.005, 0.0056), at=(STUD_LOCAL[0], STUD_LOCAL[1], 0.0062), color=BRASS_GOLD),
    mk.box((0.014, 0.008, 0.0020), at=(STUD_LOCAL[0] - 0.010, STUD_LOCAL[1] - 0.0245, 0.0068), color=STEEL, quat=_q_fy_n),
    mk.box((0.026, 0.008, 0.0020), at=(STUD_LOCAL[0] + 0.002, STUD_LOCAL[1] + 0.0245, 0.0068), color=STEEL, quat=_q_fy_p),
    mk.box((0.008, 0.026, 0.0020), at=(STUD_LOCAL[0] + 0.0218, STUD_LOCAL[1] + 0.003, 0.0068), color=STEEL, quat=_q_fx_p),
], mass=TAB_MASS, color=SLATE_BLUE)
kit.hinge("tab_hinge", "esd_bench", "snap_tab", anchor=TAB_HINGE, axis=(1, 0, 0),
          lower_deg=0.0, upper_deg=24.0, start_deg=14.0, stiffness=0.35, rest_deg=14.0, damping=0.02, friction=0.02)

# Separate 28 g corner retainer catch (`tab_catch`) locking outer +Y edge flat ONLY when left counter-holds
_q_tc = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle([1, 0, 0], math.radians(60.0))))
kit.part("tab_catch", (TAB_HINGE[0] - 0.005, TAB_HINGE[1] + 0.1055, TAB_HINGE[2] + 0.0020), [
    mk.box((0.048, 0.030, 0.012), at=(0.0, +0.016, 0.006), color=MAT_DARK),
    mk.box((0.034, 0.009, 0.0020), at=(0.0, -0.0055, +0.0032), color=STEEL),
    mk.box((0.034, 0.012, 0.0022), at=(0.0, -0.0065, +0.0078), color=YELLOW_WARN, quat=_q_tc),
], mass=JAW_MASS, color=STEEL)
kit.slider("tab_catch_slide", "esd_bench", "tab_catch", axis=(0, 1, 0),
           lower=0.0, upper=0.009, start=0.0, stiffness=14.0, rest=0.0, damping=1.5, friction=0.002)

# Separate 28 g beveled spring-jaw catch on -X side of stud (60 deg steep bevel >> atan(mu) slides freely)
_q_jaw = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle([0, 1, 0], math.radians(60.0))))
_JAW_POS = (TAB_HINGE[0] + STUD_LOCAL[0] - 0.0273, TAB_HINGE[1] + STUD_LOCAL[1], TAB_HINGE[2] + 0.0020)
kit.part("snap_jaw", _JAW_POS, [
    mk.box((0.010, 0.032, 0.0180), at=(-0.001, 0.0, 0.0090), color=MAT_DARK),
    mk.box((0.012, 0.056, 0.0035), at=(-0.010, 0.0, 0.0090), color=YELLOW_WARN),
    mk.box((0.010, 0.030, 0.0018), at=(+0.0068, 0.0, 0.0067), color=STEEL),
    mk.box((0.013, 0.030, 0.0022), at=(+0.0082, 0.0, 0.0117), color=BRASS_GOLD, quat=_q_jaw),
], mass=JAW_MASS, color=STEEL)
kit.slider("jaw_slide", "snap_tab", "snap_jaw", axis=(-1, 0, 0),
           lower=0.0, upper=0.009, start=0.0, stiffness=22.0, rest=0.0, damping=1.8, friction=0.002)

# 120 g common-point ground block + separate 28 g socket retention latch (`jack_latch`, 56 mm indicator bar)
_q_gb_l = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle([0, 0, 1], math.radians(22.0))))
kit.part("ground_block", (_gx, _gy, GB_Z0), [
    mk.box((GB_SIZE[0], GB_SIZE[1], 0.007), at=(0.0, 0.0, 0.0035), color=GB_BODY),
    mk.box((0.034, 0.012, 0.005), at=(0.0, +GB_SIZE[1] / 2 + 0.006, 0.0025), color=GB_BODY),  # low front support shelf
    mk.box((0.0170, GB_SIZE[1], 0.029), at=(-0.0255, 0.0, 0.0215), color=GB_BODY),
    mk.box((0.0170, GB_SIZE[1], 0.029), at=(+0.0255, 0.0, 0.0215), color=GB_BODY),
    mk.box((0.034, 0.036, 0.005), at=(0.0, -0.009, 0.0270), color=GB_BODY),  # rear hood keeps plug <= 4 deg pitch
    mk.box((0.0030, 0.020, 0.026), at=(-0.0146, +0.016, 0.0200), color=YELLOW_WARN, quat=_q_gb_l),  # -X entry funnel
    mk.box((0.0030, 0.030, 0.026), at=(-0.0132, -0.007, 0.0200), color=BRASS_GOLD),  # rear socket alignment sleeve -X
    mk.box((0.0030, 0.030, 0.026), at=(+0.0132, -0.007, 0.0200), color=BRASS_GOLD),  # rear socket alignment sleeve +X
    mk.box((0.024, 0.008, 0.026), at=(0.0, -GB_SIZE[1] / 2 + 0.004, 0.0200), color=BRASS_GOLD),
    mk.box((0.017, 0.006, 0.006), at=(-0.0255, +GB_SIZE[1] / 2 - 0.003, 0.033), color=YELLOW_WARN),
    mk.box((0.017, 0.006, 0.006), at=(+0.0255, +GB_SIZE[1] / 2 - 0.003, 0.033), color=YELLOW_WARN),
], mass=GB_MASS, color=GB_BODY)

_q_jl = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle([0, 0, 1], math.radians(60.0))))
_JL_POS = (_gx + 0.0152, _gy + 0.014, GB_Z0 + 0.017)
kit.part("jack_latch", _JL_POS, [
    mk.box((0.006, 0.018, 0.011), at=(0.0, +0.004, 0.0), color=STEEL),
    mk.box((0.008, 0.005, 0.011), at=(-0.0045, -0.0010, 0.0), color=BRASS_GOLD),
    mk.box((0.018, 0.0025, 0.011), at=(-0.0045, +0.0070, 0.0), color=YELLOW_WARN, quat=_q_jl),
    mk.box((0.016, 0.058, 0.004), at=(+0.025, 0.0, +0.005), color=YELLOW_WARN),  # 58 mm outer latch indicator bar
], mass=JAW_MASS, color=BRASS_GOLD)
kit.slider("jack_slide", "ground_block", "jack_latch", axis=(1, 0, 0),
           lower=0.0, upper=0.008, start=0.0, stiffness=6.0, rest=0.0, damping=1.1, friction=0.002)

# 5-link articulated coiled ESD ground cord with 50 mm molded strain-relief boots (all objects clear mat at t=0):
# snap_cap -> (pitch boot) -> cord_prox -> (yaw elbow) -> cord_dist -> (yaw) -> plug_boot -> (pitch) -> banana_plug
_cap0 = np.array([CAP_START_XY[0], CAP_START_XY[1], TOP + 0.0052])
_elb0 = np.array([0.170, -0.235, TOP + 0.0052])
_plg0 = np.array([PLUG_START_XY[0], PLUG_START_XY[1], TOP + 0.0016])


def _yaw_to(p0, p1) -> float:
    return math.atan2(float(p1[1] - p0[1]), float(p1[0] - p0[0]))


_y_init = _yaw_to(_cap0, _elb0)
_ux, _uy = math.cos(_y_init), math.sin(_y_init)
_b1_hinge = np.array([_cap0[0] + 0.025 * _ux, _cap0[1] + 0.025 * _uy, TOP + 0.0052])
_c1_hinge = np.array([_cap0[0] + 0.068 * _ux, _cap0[1] + 0.068 * _uy, TOP + 0.0052])
_c2_hinge = np.array([_elb0[0], _elb0[1], TOP + 0.0052])
_pb_hinge = np.array([_plg0[0], _plg0[1] + 0.056, TOP + 0.0052])
_plg_hinge = np.array([_plg0[0], _plg0[1] + 0.024, TOP + 0.0052])

_y1 = _yaw_to(_c1_hinge, _c2_hinge)
_prox_len = float(np.hypot(_c2_hinge[0] - _c1_hinge[0], _c2_hinge[1] - _c1_hinge[1])) - 0.006
_y2 = _yaw_to(_c2_hinge, _pb_hinge)
_dist_len = float(np.hypot(_pb_hinge[0] - _c2_hinge[0], _pb_hinge[1] - _c2_hinge[1])) - 0.006

kit.part("snap_cap", tuple(_cap0), [
    # Lower 36 x 36 mm brass snap flange caught by snap_jaw (20 x 20 mm upper bore)
    mk.box((0.036, 0.0080, 0.0022), at=(0.0, -0.0140, 0.0011), color=BRASS_GOLD),
    mk.box((0.036, 0.0080, 0.0022), at=(0.0, +0.0140, 0.0011), color=BRASS_GOLD),
    mk.box((0.0080, 0.020, 0.0022), at=(-0.0140, 0.0, 0.0011), color=BRASS_GOLD),
    mk.box((0.0080, 0.020, 0.0022), at=(+0.0140, 0.0, 0.0011), color=BRASS_GOLD),
    # Narrow 22 x 22 mm middle skirt waist around bore + 23 x 50 mm shroud
    mk.box((0.022, 0.0030, 0.0052), at=(0.0, -0.0095, 0.0048), color=COIL_BLACK),
    mk.box((0.022, 0.0030, 0.0052), at=(0.0, +0.0095, 0.0048), color=COIL_BLACK),
    mk.box((0.0030, 0.016, 0.0052), at=(-0.0095, 0.0, 0.0048), color=COIL_BLACK),
    mk.box((0.0030, 0.016, 0.0052), at=(+0.0095, 0.0, 0.0048), color=COIL_BLACK),
    # Raised 23 x 50 mm shroud (clears snap_jaw & passes top_cam) + tall 15 x 28 x 35 mm X-pinch grip boss
    mk.box((0.023, 0.050, 0.0040), at=(0.0, 0.0, 0.0110), color=COIL_BLACK),
    mk.box((0.015, 0.028, 0.035), at=(0.0, 0.0, 0.0295), color=ALU_BLUE),
    mk.box((0.015, 0.028, 0.003), at=(0.0, 0.0, 0.0485), color=YELLOW_WARN),
], mass=SNAP_CAP_MASS, color=COIL_BLACK)

kit.part("strain_boot", tuple(_b1_hinge), [
    mk.box((0.048, 0.018, 0.010), at=(0.025, 0.0, 0.0), color=COIL_BLACK),
    mk.box((0.020, 0.022, 0.010), at=(0.015, 0.0, 0.0), color=YELLOW_WARN),
], mass=BOOT_MASS, color=COIL_BLACK, yaw=_y_init)
kit.hinge("boot_p1", "snap_cap", "strain_boot", anchor=tuple(_b1_hinge), axis=(-_uy, _ux, 0.0),
          lower_deg=-85.0, upper_deg=85.0, start_deg=0.0, stiffness=0.0, damping=0.002, friction=0.0005)

kit.part("cord_prox", tuple(_c1_hinge), [
    mk.box((_prox_len, 0.014, 0.009), at=(_prox_len / 2 + 0.003, 0.0, 0.0), color=COIL_BLACK),
    mk.box((0.024, 0.020, 0.010), at=(_prox_len * 0.5, 0.0, 0.001), color=YELLOW_WARN),
    mk.box((0.022, 0.018, 0.010), at=(_prox_len * 0.8, 0.0, 0.001), color=COIL_BLACK),
], mass=CORD_LINK_MASS, color=COIL_BLACK, yaw=_y1)
kit.hinge("cord_h1", "strain_boot", "cord_prox", anchor=tuple(_c1_hinge), axis=(0, 0, 1),
          lower_deg=-350.0, upper_deg=350.0, start_deg=0.0, stiffness=0.0, damping=0.001, friction=0.0002)

kit.part("cord_dist", tuple(_c2_hinge), [
    mk.box((_dist_len, 0.012, 0.009), at=(_dist_len / 2 + 0.003, 0.0, 0.0), color=COIL_BLACK),
    mk.box((0.022, 0.018, 0.010), at=(_dist_len * 0.2, 0.0, 0.001), color=YELLOW_WARN),
    mk.box((0.022, 0.018, 0.010), at=(_dist_len * 0.5, 0.0, 0.001), color=COIL_BLACK),
], mass=CORD_LINK_MASS, color=COIL_BLACK, yaw=_y2)
kit.hinge("cord_h2", "cord_prox", "cord_dist", anchor=tuple(_c2_hinge), axis=(0, 0, 1),
          lower_deg=-350.0, upper_deg=350.0, start_deg=0.0, stiffness=0.0, damping=0.001, friction=0.0002)

kit.part("plug_boot", tuple(_pb_hinge), [
    mk.box((0.032, 0.054, 0.008), at=(0.0, -0.008, 0.0020), color=COIL_BLACK),
    mk.box((0.034, 0.026, 0.0085), at=(0.0, -0.006, 0.0020), color=YELLOW_WARN),
], mass=BOOT_MASS, color=COIL_BLACK)
kit.hinge("cord_h3", "cord_dist", "plug_boot", anchor=tuple(_pb_hinge), axis=(0, 0, 1),
          lower_deg=-350.0, upper_deg=350.0, start_deg=0.0, stiffness=0.0, damping=0.001, friction=0.0002)

# Banana plug (`banana_plug`): hinged in pitch at rear boot, tall 15 x 28 mm X-pinch grip boss + yellow crown + locking tip (-Y)
kit.part("banana_plug", tuple(_plg0), [
    mk.box((0.017, 0.032, 0.012), at=(0.0, +0.005, 0.006), color=JACK_ORANGE),
    mk.box((0.015, 0.028, 0.034), at=(0.0, +0.004, 0.029), color=ALU_BLUE),  # 15 x 28 mm X-pinch grip boss
    mk.box((0.015, 0.028, 0.003), at=(0.0, +0.004, 0.0475), color=YELLOW_WARN),  # yellow crown strip
    mk.box((0.026, 0.004, 0.024), at=(0.0, +0.0295, 0.028), color=JACK_ORANGE),  # +Y pad-caging rear push flange
    mk.box((0.024, 0.004, 0.018), at=(0.0, -0.0205, 0.033), color=JACK_ORANGE),  # -Y pad-caging front stop flange
    mk.box((0.0045, 0.018, 0.010), at=(0.0, -0.020, 0.012), color=BRASS_GOLD),  # narrow retention neck
    mk.box((0.0170, 0.008, 0.010), at=(0.0, -0.033, 0.012), color=BRASS_GOLD),  # locking tip collar snaps past jack_latch
], mass=PLUG_MASS, color=JACK_ORANGE)
kit.hinge("boot_p2", "plug_boot", "banana_plug", anchor=tuple(_plg_hinge), axis=(1, 0, 0),
          lower_deg=-85.0, upper_deg=85.0, start_deg=0.0, stiffness=0.0, damping=0.002, friction=0.0005)

OBJECTS = [*kit.objects()]
SEATED_IN = {"ground_block": "esd_bench"}

SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"

# -- scoring (100% physical end-state readouts; literal thresholds match task.json) --------------------------------
TAB_FLAT_DEG = 4.0  # 4.0 deg max corner tab curl relative to flat bench
STUD_XY_TOL = 0.0050  # 5.0 mm max radial snap offset
SNAP_Z_TOL = 0.0030  # 3.0 mm max gap above flat stud base
JAW_CLOSED_M = 0.0012  # 1.2 mm max retainer jaw residual gap
JACK_HOME_GAP_M = 0.0030  # 3.0 mm max gap between plug shoulder and ground block front face
JACK_PARTIAL_GAP_M = 0.0180  # 18.0 mm partial entry window (unseated vs pulled out)
JACK_ALIGN_DEG = 8.0  # 8.0 deg max plug yaw/tilt misalignment relative to ground block
BLOCK_SHIFT_MAX = 0.008  # 8.0 mm max ground block shift from bench nest
CLEAR_M = 0.045  # 45 mm minimum gripper TCP clearance from mat tab, cap, cord, plug, and block


def _ang_deg(qa, qb) -> float:
    ra, rb = mk.quat_to_mat(qa), mk.quat_to_mat(qb)
    tr = float(np.trace(ra.T @ rb))
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


def _cord_clear(objects) -> bool:
    for name in ("cord_prox", "cord_dist"):
        st = objects[name]
        p, q = kit.pose(objects, name)
        rm = mk.quat_to_mat(q)
        part = kit.parts[name]
        mid_x = (part.lo[0] + part.hi[0]) / 2
        for frac in (0.25, 0.5, 0.75):
            pt = p + rm @ np.array([part.lo[0] + frac * (part.hi[0] - part.lo[0]), 0.0, 0.0])
            if (WORK_X[0] <= pt[0] <= WORK_X[1]) and (WORK_Y[0] <= pt[1] <= WORK_Y[1]):
                return False
        if float(st.low[2]) < TOP - 0.020:
            return False
        _ = mid_x
    return True


def components(objects) -> dict:
    t_p, t_q = kit.pose(objects, "snap_tab")
    rt = mk.quat_to_mat(t_q)
    stud_w = t_p + rt @ np.array([STUD_LOCAL[0], STUD_LOCAL[1], 0.0034])

    c_p, c_q = kit.pose(objects, "snap_cap")
    jaw_p, _ = kit.pose(objects, "snap_jaw")
    gb_p, gb_q = kit.pose(objects, "ground_block")
    pl_p, pl_q = kit.pose(objects, "banana_plug")
    jl_p, _ = kit.pose(objects, "jack_latch")

    tab_deg = abs(kit.q(objects, "tab_hinge") * 180.0 / math.pi)
    tab_flat = tab_deg <= TAB_FLAT_DEG
    stud_xy = float(math.hypot(c_p[0] - stud_w[0], c_p[1] - stud_w[1]))
    snap_z = float(c_p[2] - stud_w[2])
    jaw_open = max(0.0, kit.q(objects, "jaw_slide"))
    stud_snapped = tab_flat and (stud_xy <= STUD_XY_TOL) and (-0.0015 <= snap_z <= SNAP_Z_TOL) and (jaw_open <= JAW_CLOSED_M)

    cord_clear = _cord_clear(objects)
    rgb = mk.quat_to_mat(gb_q)
    rpl = mk.quat_to_mat(pl_q)
    shoulder_w = pl_p + rpl @ np.array([0.0, -0.011, 0.012])
    face_w = gb_p + rgb @ np.array([0.0, GB_SIZE[1] / 2, 0.012])
    jack_gap = float(rgb[:, 1] @ (shoulder_w - face_w))
    jack_lat = float(abs(rgb[:, 0] @ (shoulder_w - face_w)))
    jack_deg = _ang_deg(gb_q, pl_q)
    jl_open = max(0.0, kit.q(objects, "jack_slide"))
    block_shift = float(math.hypot(gb_p[0] - GB_XY[0], gb_p[1] - GB_XY[1]))
    block_in_nest = block_shift <= BLOCK_SHIFT_MAX

    jack_entered = block_in_nest and (jack_lat <= 0.008) and (jack_deg <= JACK_ALIGN_DEG) and (-0.003 <= jack_gap <= JACK_PARTIAL_GAP_M)
    jack_home = block_in_nest and (jack_lat <= 0.005) and (jack_deg <= JACK_ALIGN_DEG) and (-0.003 <= jack_gap <= JACK_HOME_GAP_M) and (jl_open <= JAW_CLOSED_M)

    tcps = [np.asarray(t, float) for t in mk.tcps()]
    guarded = [objects[n] for n in ("snap_tab", "snap_cap", "cord_prox", "cord_dist", "banana_plug", "ground_block")]
    hands_clear = len(tcps) >= 2
    for t in tcps:
        for st in guarded:
            lo, hi = np.asarray(st.low, float) - CLEAR_M, np.asarray(st.high, float) + CLEAR_M
            if bool(np.all(t >= lo) and np.all(t <= hi)):
                hands_clear = False
    _ = (jaw_p, jl_p)

    return {
        "tab_flat": tab_flat,
        "tab_deg": tab_deg,
        "stud_snapped": stud_snapped,
        "stud_xy_m": stud_xy,
        "snap_z_m": snap_z,
        "jaw_open_m": jaw_open,
        "cord_clear": cord_clear,
        "block_in_nest": block_in_nest,
        "jack_entered": jack_entered,
        "jack_home": jack_home,
        "jack_gap_m": jack_gap,
        "jack_lat_m": jack_lat,
        "jack_deg": jack_deg,
        "jl_open_m": jl_open,
        "hands_clear": hands_clear,
    }


def score(objects) -> float:
    """Judged at the end of the trial; Possible scores 0.0, 0.25, 0.50, 0.60, 0.85, 1.0."""
    c = components(objects)
    if not (c["stud_snapped"] and c["block_in_nest"]):
        return 0.0
    if c["jack_home"] and c["cord_clear"] and c["hands_clear"]:
        return 1.0
    if c["jack_home"] and c["cord_clear"]:
        return 0.85
    if c["jack_home"] and not c["cord_clear"]:
        return 0.50
    if c["cord_clear"] and c["jack_entered"]:
        return 0.60
    return 0.25


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    c = components(objects)
    return float(c["stud_xy_m"] + max(0.0, c["snap_z_m"]) + max(0.0, c["jack_gap_m"]) + (0.0 if c["cord_clear"] else 0.1))


# -- open-loop bimanual scripted demo (constants only) --------------------------------------------------------------
def demo():
    """One arm flattens and counter-holds the corner mat tab; the other arm snaps the 10 mm stud onto the stud,
    routes the cord around the +X perimeter clear of the work zone, drives the banana plug into the ground block,
    pulls ~5 N to check retention, and both park >= 45 mm clear for 3 s."""
    rl = axis_angle_matrix([1.0, 0.0, 0.0], -math.radians(18.0)) @ pitch_demo.rotation(0.0)
    rr = pitch_demo.rotation(math.pi / 2)
    r_flat = pitch_demo.rotation(math.pi / 2)
    l_safe = TOP + 0.145
    r_safe = TOP + 0.068

    lx_tab = TAB_HINGE[0] + TAB_HOLD_LOCAL[0]
    ly_tab = TAB_HINGE[1] + TAB_HOLD_LOCAL[1]
    lz_tab_hi = TAB_HINGE[2] + 0.052 + PAD_TIP_PAST_TCP
    lz_tab_hold = TOP + 0.002 + TAB_HOLD_LOCAL[2] + 0.011 + PAD_TIP_PAST_TCP - 0.005

    rx_cap0, ry_cap0 = CAP_START_XY
    rz_cap_pick = TOP + 0.0052 + 0.020 + PAD_TIP_PAST_TCP  # pad bottom +8.6 mm above shroud top
    rz_cap_press = TOP + 0.0054 + 0.020 + PAD_TIP_PAST_TCP - 0.015  # firm ~15 N snap (needs left counter-hold)

    rx_plg0 = PLUG_START_XY[0]
    ry_plg0 = PLUG_START_XY[1] + 0.004
    rz_plg_pick = TOP + 0.0016 + 0.020 + PAD_TIP_PAST_TCP  # pad bottom +8.0 mm above orange plug base

    rx_in = GB_XY[0]
    ry_pre = SOCKET_Y_WORLD + 0.048
    ry_home = SOCKET_Y_WORLD + 0.0090  # drive plug shoulder (-0.011) past latch (+0.0105) without hitting rear wall
    ry_pull = SOCKET_Y_WORLD + 0.0160  # ~5 N reverse pull-test against flat rear latch catch
    rz_in = GB_Z0 + 0.0095 + 0.020 + PAD_TIP_PAST_TCP

    left = [
        Waypoint(1.2, (lx_tab, ly_tab, l_safe), FINGER_CLOSED, "above mat corner tab", rot=rl),
        Waypoint(1.1, (lx_tab, ly_tab, lz_tab_hi), FINGER_CLOSED, "approach curled corner boss", rot=rl),
        Waypoint(1.0, (lx_tab, ly_tab, lz_tab_hold), FINGER_CLOSED, "flatten and counter-hold tab", rot=rl),
        Waypoint(9.1, (lx_tab, ly_tab, lz_tab_hold), FINGER_CLOSED, "counter-hold during 10 mm snap", rot=rl),
        Waypoint(0.7, (lx_tab, ly_tab, lz_tab_hold), FINGER_CLOSED, "dwell after tab catch locks", rot=rl),
        Waypoint(1.0, (lx_tab, ly_tab, l_safe), FINGER_OPEN, "lift left arm clear", rot=rl),
        Waypoint(1.3, (0.160, 0.300, TOP + 0.165), FINGER_OPEN, "park left arm clear", yaw=0.0),
        Waypoint(18.0, (0.160, 0.300, TOP + 0.165), FINGER_OPEN, "wait clear for routing and 3 s hold", yaw=0.0),
    ]

    right = [
        Waypoint(1.2, (rx_cap0, ry_cap0, TOP + 0.095), FINGER_OPEN, "above 10 mm snap cap", rot=rr),
        Waypoint(1.1, (rx_cap0, ry_cap0, rz_cap_pick), FINGER_OPEN, "straddle cap grip boss", rot=rr),
        Waypoint(0.9, (rx_cap0, ry_cap0, rz_cap_pick), FINGER_CLOSED, "pinch snap cap", rot=rr),
        Waypoint(1.0, (rx_cap0, ry_cap0, TOP + 0.068), FINGER_CLOSED, "lift snap cap clear", rot=rr),
        Waypoint(1.5, (0.245, -0.068, TOP + 0.068), FINGER_CLOSED, "carry cap with cord trailing south", rot=rr),
        Waypoint(1.4, (STUD_XY[0] + 0.0005, STUD_XY[1], TOP + 0.068), FINGER_CLOSED, "align high above male stud", rot=rr),
        Waypoint(1.1, (STUD_XY[0] + 0.0005, STUD_XY[1], rz_cap_press + 0.024), FINGER_CLOSED, "lower over 10 mm stud", rot=rr),
        Waypoint(1.3, (STUD_XY[0] + 0.0005, STUD_XY[1], rz_cap_press), FINGER_CLOSED, "press ~10 N past snap jaw", rot=rr),
        Waypoint(0.9, (STUD_XY[0] + 0.0005, STUD_XY[1], rz_cap_press), FINGER_CLOSED, "dwell pressed to seat snap cap", rot=rr),
        Waypoint(0.7, (STUD_XY[0] + 0.0005, STUD_XY[1], rz_cap_press), FINGER_OPEN, "release snapped cap", rot=rr),
        Waypoint(0.9, (STUD_XY[0], STUD_XY[1], TOP + 0.095), FINGER_OPEN, "clear stud", rot=rr),
        Waypoint(1.4, (rx_plg0, ry_plg0, TOP + 0.095), FINGER_OPEN, "above banana plug grip", rot=r_flat),
        Waypoint(1.1, (rx_plg0, ry_plg0, rz_plg_pick), FINGER_OPEN, "descend onto banana plug", rot=r_flat),
        Waypoint(0.9, (rx_plg0, ry_plg0, rz_plg_pick), FINGER_CLOSED, "grasp banana plug", rot=r_flat),
        Waypoint(1.0, (rx_plg0, ry_plg0, TOP + 0.092), FINGER_CLOSED, "lift banana plug clear above block", rot=r_flat),
        Waypoint(1.4, (0.415, -0.055, TOP + 0.085), FINGER_CLOSED, "sweep cord around +X work-zone channel", rot=r_flat),
        Waypoint(1.1, (0.395, -0.095, TOP + 0.085), FINGER_CLOSED, "guide elbow east along +X channel", rot=r_flat),
        Waypoint(1.3, (rx_in, ry_pre, TOP + 0.085), FINGER_CLOSED, "align high ahead of ground block", rot=r_flat),
        Waypoint(1.1, (rx_in, ry_pre, rz_in), FINGER_CLOSED, "lower plug ahead of ground block", rot=r_flat),
        Waypoint(0.4, (rx_in, ry_pre, rz_in), FINGER_CLOSED, "settle plug ahead of socket", rot=r_flat),
        Waypoint(2.2, (rx_in, ry_home, rz_in), FINGER_CLOSED, "drive ~9 N into socket latch", rot=r_flat),
        Waypoint(0.7, (rx_in, ry_home, rz_in), FINGER_CLOSED, "hold plug seated for latch lock", rot=r_flat),
        Waypoint(0.8, (rx_in, ry_pull, rz_in), FINGER_CLOSED, "pull 5 N to verify jack hold", rot=r_flat),
        Waypoint(0.7, (rx_in, ry_pull, rz_in), FINGER_OPEN, "release plug in locked socket", rot=r_flat),
        Waypoint(0.9, (rx_in, ry_pull, TOP + 0.145), FINGER_OPEN, "lift right arm clear", rot=r_flat),
        Waypoint(1.2, (0.390, -0.320, TOP + 0.165), FINGER_OPEN, "park right arm clear", yaw=0.0),
        Waypoint(3.8, (0.390, -0.320, TOP + 0.165), FINGER_OPEN, "post-release 3 s stability hold", yaw=0.0),
    ]
    return {"left": left, "right": right}
