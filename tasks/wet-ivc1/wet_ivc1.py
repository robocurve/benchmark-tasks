"""Seal a mouse IVC cage and dock it in the empty bay of a ventilated rack so its air valves connect.

Bench layout in station coordinates (TOP = station.TABLE_TOP, +X forward, +Y left):
- mat (fixed): silicone bench mat 400 x 260 x 2 mm centred at (0.200, 0.000, TOP).
- cage (loose, 600 g base + 2 x 10 g valves): GM500-class mouse IVC cage 391 x 199 x 160 mm resting on the mat at
  (0.210, 0.000, TOP + 0.002), facing +X. Side flanges (y +-0.086..+-0.0995, z 0.125..0.135) overhang the tub for
  lifting and hanging on the rack runners; grip rails in the side-handle cutouts (z 0.135..0.156).
- lid (loose, 120 g): the cage's filter top lies flat on the bench beside the cage at (0.210, +0.205). Its plug skirt
  (1 mm radial clearance in the tub mouth, 6 mm chamfered lead-in) carries the radial silicone gasket; set on and pressed
  down it sits in register on the flange. The real cage has no latches: the gasket's friction holds the top (maker
  spec sheet, README).
  ``valve_l`` / ``valve_r`` (slide X, -8..0 mm): spring-closed air-valve pistons protruding 10 mm from the cage front at
  y = +-0.050, z = 0.150 (cage frame). Only the rack's nozzles open them, when the cage is fully home.
- bottle (loose, 180 g): 260 mL polycarbonate water bottle (55 x 55 x 129 mm) upright in the cage's front pocket.
- rack (fixed): stainless rack bay centred at (0.610, 0.000, TOP). Runner ledges (top z = TOP + 0.145, 17 mm wide,
  x 0.405..0.815, inner edges y +-0.091), side walls (inner faces y +-0.108, 278 mm high), cell floor at TOP + 0.004,
  fixed rear stop face at x = 0.808 (y +-0.030), rear plenum wall x 0.817..0.833, and two silicone air nozzles
  (r 5 mm) at y +-0.050, z TOP + 0.170 with tips at x = 0.811: inlet left, outlet right.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
from labkit import GLASS_COLORS, GLASS_IOR, GLASS_TINTS, Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import demo as _demo
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN
from yam_arena.urdf import axis_angle_matrix

# -- geometry & scoring thresholds (single source of truth for score(), success(), the hook and demo()) ----

TOP = station.TABLE_TOP
CONTROL_DT = 0.05  # 20 Hz control step

MAT_POS = (0.200, 0.000, TOP)
MAT_SIZE = (0.400, 0.260, 0.002)
MAT_H = MAT_SIZE[2]

CAGE_LEN = 0.391
CAGE_HALF = CAGE_LEN / 2.0  # 0.1955
CAGE_WIDTH = 0.199
CAGE_H = 0.160
CAGE_MASS = 0.600  # base; + 2 x 10 g valves; the 120 g filter top is its own body
CAGE_SPAWN_POS = (0.210, 0.000, TOP + MAT_H)

FLANGE_Z0 = 0.125
FLANGE_Z1 = 0.135
FLANGE_Z_MID = 0.5 * (FLANGE_Z0 + FLANGE_Z1)  # 0.130
POCKET_X = 0.045
POCKET_FLOOR_Z = 0.033

# Filter top (own body, frame = the cage frame when seated): plate z 0.135..0.160 on the flange; plug skirt 1 mm inside
# the tub mouth (x +-0.180, y +-0.070) down to z 0.131, then a 6 mm chamfered lead-in to z 0.123.
LID_MASS = 0.120
LID_FRICTION = 0.5  # declared polycarbonate / silicone-gasket friction coefficient
LID_SEALED_M = 0.0015  # every plug corner within 1.5 mm of its seated place in the cage frame = in register and level
LID_SPAWN_XY = (0.245, 0.215)  # lying flat on the bench beside the cage (+y side)
LID_SPINE_X = -0.033  # lid-frame x of the 70 x 80 mm grip spine between the two recessed side-handle pockets
LID_GRIP_X = -0.016  # pinch the spine at the top's centre of mass (x -0.016): no tipping torque in the grip
LID_GRIP_Z = 0.140  # lid-frame z of the fingertips when pinching the spine
LID_CORNERS = tuple((sx * 0.175, sy * 0.065, 0.135) for sx in (-1, 1) for sy in (-1, 1))

# Air valves (declared spring: 100 N/m, 0.7 N at the 7 mm docked travel; 0.3 N seal friction)
VALVE_Y = 0.050
VALVE_Z = 0.150
VALVE_PROTRUDE = 0.010
VALVE_TRAVEL_MAX = 0.008
VALVE_K = 100.0
VALVE_FRICTION_N = 0.3
VALVE_OPEN_M = 0.005  # piston pushed in >= 5 mm = valve open

BOTTLE_W = 0.055
BOTTLE_H = 0.129
BOTTLE_HALF_H = BOTTLE_H / 2.0  # 0.0645
BOTTLE_MASS = 0.180
BOTTLE_SPAWN_POS = (
    CAGE_SPAWN_POS[0] + POCKET_X,
    0.000,
    CAGE_SPAWN_POS[2] + POCKET_FLOOR_Z + BOTTLE_HALF_H,
)

RACK_POS = (0.610, 0.000, TOP)
RUNNER_FRONT_X = 0.405
RACK_REAR_X = 0.825
RUNNER_TOP_Z = 0.145  # above TOP
STOP_WORLD_X = 0.808  # fixed rear stop face
NOZZLE_TIP_X = 0.811  # nozzle tips; a docked piston (tip at STOP + 10 mm) is pushed in 7 mm
NOZZLE_R = 0.005
CELL_FLOOR_H = 0.004

HANG_LOW_Z = RUNNER_TOP_Z - FLANGE_Z0  # 0.020 m above TOP when flanges sit on runners
HANG_Z_TOL = 0.008  # cell floor (0.004) misses by 16 mm
RUNNER_Y_TOL = 0.012
MIN_ENGAGED_X = 0.150
ON_RUNNER_TILT_DEG = 5.0
DOCK_GAP_M = 0.003  # nose within 3 mm of the stop: the open piston is pressed by its own nozzle
CAGE_UPRIGHT_DEG = 20.0
BOTTLE_TILT_DEG = 25.0
POCKET_XY_TOL = 0.022
POCKET_Z_TOL = 0.035

HAND_CLEAR_M = 0.120
HOLD_S = 3.0  # final state held this long inside score()

S_LID = 0.20
S_RUNNERS = 0.15
S_VALVE = 0.25
S_FINAL = 0.15
SCORE_LEVELS = (0.0, 0.15, 0.20, 0.35, 0.40, 0.60, 0.65, 0.85, 1.0)  # every total score() can return

SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0  # after the 3 s hold inside score(): >= 4 s with both arms parked clear
PASS_SCORE = 1.0 - 1e-9

# Demo placement & grasp constants
FINGER_STRADDLE = -0.025
GRASP_LOCAL_X = -0.025
GRASP_Y = 0.094
PLACE_CAGE_X = 0.420
PUSH_CAGE_X = STOP_WORLD_X - CAGE_HALF  # 0.6125
TIP_BELOW_TCP = 0.013
LID_CARRY_Z = 0.203  # TCP height carrying the top: plate underside 11 mm above the bottle neck
LID_RELEASE_UP = 0.014  # release the top 14 mm above its seat: the chamfered skirt is inside the tub mouth
LID_REG_TCP_X = 0.186  # open-finger TCP x (from the top's spawn centre) at the end of the squaring push
LID_RELEASE_BACK_M = 0.006  # release the top 6 mm behind register: the carried top rides ~6 mm forward on its seat
LID_PRESS_IN = 0.003  # closed fingertips pressed 3 mm into the spine top to seat the gasket

# -- colours & materials -----------------------------------------------------------------------------------

MAT_BLUE = (0.20, 0.34, 0.44)
POLY_CLEAR = (0.76, 0.80, 0.84)
LID_WHITE = (0.93, 0.94, 0.95)
BEDDING_TAN = (0.76, 0.67, 0.48)
BOTTLE_AQUA = (0.72, 0.86, 0.94)
BOTTLE_CAP_BLUE = (0.16, 0.42, 0.72)
STEEL = (0.72, 0.74, 0.77)
STEEL_DARK = (0.46, 0.49, 0.53)
BEZEL_SLATE = (0.22, 0.24, 0.28)
SILICONE = (0.80, 0.80, 0.78)

GLASS_COLORS.add(POLY_CLEAR)
GLASS_COLORS.add(BOTTLE_AQUA)

VIS = pathlib.Path(_HERE) / "assets" / "visual"
GLASS_TINTS.update({"poly": (0.93, 0.96, 0.98), "water": (0.70, 0.86, 1.0), "red": (0.90, 0.12, 0.08)})
GLASS_IOR.update({"poly": 1.586, "water": 1.333, "red": 1.586})  # polycarbonate, water

# -- objects -----------------------------------------------------------------------------------------------

MAT = part(
    "mat",
    pos=MAT_POS,
    fixed=True,
    friction=0.8,
    boxes=[((0.0, 0.0, MAT_H / 2.0), MAT_SIZE, MAT_BLUE)],
    visuals=[("", str(VIS / "mat.usda")), ("", str(VIS / "bench_props.usda"))],  # props: render-only, no collider
    hide_shapes=True,
)

# Plug-skirt chamfer: 6 mm inward over 8 mm height (53 deg from horizontal), 4 mm thick slabs
_CH_TH = math.atan2(0.006, 0.008)
_CH_LEN = math.hypot(0.006, 0.008)
_CH_DX = 0.002 * math.cos(_CH_TH)  # slab centre sits half a thickness inside its outer face
_CH_DZ = 0.002 * math.sin(_CH_TH)
# End walls (no bottle or handle above them): 13 mm lead-in over 20 mm height (57 deg from horizontal)
_CHE_TH = math.atan2(0.013, 0.020)
_CHE_LEN = math.hypot(0.013, 0.020)
_CHE_DX = 0.002 * math.cos(_CHE_TH)
_CHE_DZ = 0.002 * math.sin(_CHE_TH)
_LID_BOXES = [
    # Filter-top plate on the flange (z 0.135..0.160): rear, grip spine, front, bottle-bay sides (bay x 0.002..0.090,
    # y +-0.040, open for the bottle), and the filter sheet. Pockets x -0.068..+0.008 beside the spine are open.
    ((-0.131, 0.0, 0.1475), (0.126, 0.164, 0.025), POLY_CLEAR),
    ((LID_SPINE_X, 0.0, 0.1475), (0.070, 0.080, 0.025), POLY_CLEAR),
    ((+0.133, 0.0, 0.1475), (0.086, 0.164, 0.025), POLY_CLEAR),  # front edge x 0.176, behind the valve pistons
    ((+0.046, +0.061, 0.1475), (0.088, 0.042, 0.025), POLY_CLEAR),
    ((+0.046, -0.061, 0.1475), (0.088, 0.042, 0.025), POLY_CLEAR),
    ((-0.130, 0.0, 0.1605), (0.095, 0.100, 0.003), LID_WHITE),
    # Side plug skirt carrying the gasket, 1 mm inside the tub mouth (y +-0.070), z 0.131..0.135
    ((0.0, +0.067, 0.133), (0.350, 0.004, 0.004), POLY_CLEAR),
    ((0.0, -0.067, 0.133), (0.350, 0.004, 0.004), POLY_CLEAR),
    # End walls: one sloped face from 1 mm inside the flange mouth (x +-0.184) at z 0.135 to 14 mm inside at z 0.115
    ((+0.1765 - _CHE_DX, 0.0, 0.125 + _CHE_DZ), (0.004, 0.120, _CHE_LEN), POLY_CLEAR, (0.0, +_CHE_TH)),
    ((-0.1765 + _CHE_DX, 0.0, 0.125 + _CHE_DZ), (0.004, 0.120, _CHE_LEN), POLY_CLEAR, (0.0, -_CHE_TH)),
    # Side walls: chamfered lead-in (outer face 1 mm -> 7 mm inside the mouth over z 0.131 -> 0.123)
    ((0.0, +0.066 - _CH_DX, 0.127 + _CH_DZ), (0.004, 0.330, _CH_LEN), POLY_CLEAR, (math.pi / 2, +_CH_TH)),
    ((0.0, -0.066 + _CH_DX, 0.127 + _CH_DZ), (0.004, 0.330, _CH_LEN), POLY_CLEAR, (math.pi / 2, -_CH_TH)),
]
_LID_LOW_Z = float(labkit._extent(labkit._boxes(_LID_BOXES, "lid"), ())[0][2])
LID_SPAWN_POS = (LID_SPAWN_XY[0], LID_SPAWN_XY[1], TOP - _LID_LOW_Z + 0.0005)

LID = part(
    "lid",
    pos=LID_SPAWN_POS,
    mass=LID_MASS,
    friction=LID_FRICTION,
    visuals=[("", str(VIS / "lid.usda"))],
    hide_shapes=True,
    boxes=_LID_BOXES,
)


def _valve(name: str, y: float) -> Joint:
    return Joint(
        name=name,
        kind="slide",
        axis="X",
        lower=-VALVE_TRAVEL_MAX,
        upper=0.0,
        target=0.0,
        stiffness=VALVE_K,
        damping=1.0,
        friction=VALVE_FRICTION_N,
        mass=0.01,
        boxes=[((CAGE_HALF + VALVE_PROTRUDE - 0.0075, y, VALVE_Z), (0.015, 0.012, 0.012), SILICONE)],
    )


VALVE_L = _valve("valve_l", +VALVE_Y)
VALVE_R = _valve("valve_r", -VALVE_Y)

CAGE = part(
    "cage",
    pos=CAGE_SPAWN_POS,
    mass=CAGE_MASS,
    friction=0.85,
    joints=(VALVE_L, VALVE_R),
    visuals=[("", str(VIS / "cage.usda")),
             ("valve_l", str(VIS / "valve_l.usda")), ("valve_r", str(VIS / "valve_r.usda"))],
    hide_shapes=True,
    boxes=[
        # Tub floor and corncob bedding (tapered 148 mm lower belly for runner clearance)
        ((0.0, 0.0, 0.003), (0.360, 0.144, 0.006), POLY_CLEAR),
        ((0.0, 0.0, 0.010), (0.348, 0.132, 0.008), BEDDING_TAN),
        # Lower perimeter tub walls up to flange underside (z = 0.006 .. 0.125, outer y +-0.074)
        ((+0.182, 0.0, 0.0655), (0.004, 0.148, 0.119), POLY_CLEAR),
        ((-0.182, 0.0, 0.0655), (0.004, 0.148, 0.119), POLY_CLEAR),
        ((0.0, +0.072, 0.0655), (0.368, 0.004, 0.119), POLY_CLEAR),
        ((0.0, -0.072, 0.0655), (0.368, 0.004, 0.119), POLY_CLEAR),
        # Continuous peripheral flange rim (391 x 199 x 10 mm, z = 0.125 .. 0.135)
        ((0.0, +0.08475, FLANGE_Z_MID), (CAGE_LEN, 0.0295, 0.010), POLY_CLEAR),
        ((0.0, -0.08475, FLANGE_Z_MID), (CAGE_LEN, 0.0295, 0.010), POLY_CLEAR),
        ((+0.18975, 0.0, FLANGE_Z_MID), (0.0115, CAGE_WIDTH, 0.010), POLY_CLEAR),
        ((-0.18975, 0.0, FLANGE_Z_MID), (0.0115, CAGE_WIDTH, 0.010), POLY_CLEAR),
        # Side-handle grip rails in the side cutouts (y +-0.088..+-0.0995, z 0.135..0.156)
        ((-0.027, +0.09375, 0.1455), (0.082, 0.0115, 0.021), POLY_CLEAR),
        ((-0.027, -0.09375, 0.1455), (0.082, 0.0115, 0.021), POLY_CLEAR),
        # Bottle pocket floor & cradle walls (60 x 60 mm clear bay at x = +0.045, floor top z = 0.033)
        ((+POCKET_X, 0.0, 0.030), (0.066, 0.066, 0.006), POLY_CLEAR),
        ((+POCKET_X - 0.032, 0.0, 0.085), (0.004, 0.066, 0.104), POLY_CLEAR),
        ((+POCKET_X + 0.032, 0.0, 0.085), (0.004, 0.066, 0.104), POLY_CLEAR),
        ((+POCKET_X, +0.032, 0.085), (0.060, 0.004, 0.104), POLY_CLEAR),
        ((+POCKET_X, -0.032, 0.085), (0.060, 0.004, 0.104), POLY_CLEAR),
    ],
)

BOTTLE = part(
    "bottle",
    pos=BOTTLE_SPAWN_POS,
    mass=BOTTLE_MASS,
    friction=0.55,
    visuals=[("", str(VIS / "bottle.usda"))],
    hide_shapes=True,
    boxes=[
        ((0.0, 0.0, -0.0545), (0.053, 0.053, 0.020), BOTTLE_CAP_BLUE),
        ((0.0, 0.0, +0.0100), (BOTTLE_W, BOTTLE_W, 0.109), BOTTLE_AQUA),
        ((0.0, 0.0, +0.0650), (0.036, 0.036, 0.002), BOTTLE_CAP_BLUE),
    ],
)

_NOZ_LEN = 0.014
RACK = part(
    "rack",
    pos=RACK_POS,
    fixed=True,
    friction=0.25,
    visuals=[("", str(VIS / "rack.usda"))],
    hide_shapes=True,
    boxes=[
        # Stainless cell floor (x = 0.435 .. 0.825, top at TOP + 0.004)
        ((+0.020, 0.0, 0.002), (0.390, 0.216, CELL_FLOOR_H), STEEL_DARK),
        # Side upright walls (x = 0.435 .. 0.825, inner face +-0.108, z = 0 .. 0.278)
        ((+0.020, +0.114, 0.139), (0.390, 0.012, 0.278), STEEL),
        ((+0.020, -0.114, 0.139), (0.390, 0.012, 0.278), STEEL),
        # Rear plenum manifold wall (x = 0.817 .. 0.833)
        ((+0.215, 0.0, 0.139), (0.016, 0.240, 0.278), STEEL),
        # Fixed rear stop plate, face at x = 0.808 (0.808 .. 0.817), y +-0.030, z 0.110 .. 0.155
        ((STOP_WORLD_X + 0.0045 - RACK_POS[0], 0.0, 0.1325), (0.009, 0.060, 0.045), STEEL_DARK),
        # Elevated horizontal runner ledges (x = 0.405 .. 0.815, inner +-0.091, top at TOP + 0.145)
        ((+0.000, +0.0995, RUNNER_TOP_Z - 0.004), (0.410, 0.017, 0.008), STEEL),
        ((+0.000, -0.0995, RUNNER_TOP_Z - 0.004), (0.410, 0.017, 0.008), STEEL),
        # Rear anti-tip retainer rails (x = 0.710 .. 0.825)
        ((+0.1575, +0.0985, 0.172), (0.115, 0.016, 0.008), STEEL_DARK),
        ((+0.1575, -0.0985, 0.172), (0.115, 0.016, 0.008), STEEL_DARK),
        # Top header crossbar (x = 0.455 .. 0.530, z = 0.272..0.280)
        ((-0.1175, 0.0, 0.276), (0.075, 0.240, 0.008), BEZEL_SLATE),
    ],
    cylinders=[
        # Air nozzles: inlet (+y) and outlet (-y), tips at x = 0.811
        ((NOZZLE_TIP_X + _NOZ_LEN / 2 - RACK_POS[0], +VALVE_Y, HANG_LOW_Z + VALVE_Z), NOZZLE_R, _NOZ_LEN, SILICONE, "X"),
        ((NOZZLE_TIP_X + _NOZ_LEN / 2 - RACK_POS[0], -VALVE_Y, HANG_LOW_Z + VALVE_Z), NOZZLE_R, _NOZ_LEN, SILICONE, "X"),
    ],
)

SCENE = "station"
OBJECTS = (MAT, CAGE, LID, BOTTLE, RACK)

# -- trial memory (filled by score() and the observe-only hook; cleared on reset) ---------------------------

_ids: dict = {}
_trial: dict = {}


def _reset_trial() -> None:
    _ids.clear()
    _trial.clear()
    _trial.update(tcps=None, step=0, hold_s=0.0)


_reset_trial()


def _set_test_trial(*, tcps=None) -> None:
    """Test helper only; allows offline unit tests to supply arm TCP poses."""
    _trial["tcps"] = tcps


def _tilt_deg(part_state: PartState) -> float:
    return float(part_state.tilt_deg())


def _lid_error(cage: PartState, lid: PartState) -> float:
    """Largest distance (m) of the four plug corners from their seated places in the cage frame (register + level)."""
    return max(float(np.max(np.abs(cage.to_local(lid.to_world(c)) - np.asarray(c)))) for c in LID_CORNERS)


def _metrics(objects: dict[str, PartState]) -> dict:
    cage = objects["cage"]
    bottle = objects["bottle"]
    lid = objects["lid"]
    cage_z = float(cage.low[2] - TOP)
    cage_tilt = _tilt_deg(cage)
    nose_x = float(cage.pos[0] + CAGE_HALF)
    rear_x = float(cage.pos[0] - CAGE_HALF)
    overlap_x = min(RACK_REAR_X, nose_x) - max(RUNNER_FRONT_X, rear_x)
    upright = cage_tilt <= CAGE_UPRIGHT_DEG and float(cage.low[2]) >= TOP - 0.01

    on_runners = (
        abs(cage_z - HANG_LOW_Z) <= HANG_Z_TOL
        and abs(float(cage.pos[1])) <= RUNNER_Y_TOL
        and cage_tilt <= ON_RUNNER_TILT_DEG
        and overlap_x >= MIN_ENGAGED_X
    )
    gap = max(0.0, STOP_WORLD_X - nose_x)
    home = on_runners and gap <= DOCK_GAP_M
    lid_err = _lid_error(cage, lid)
    vl = -float(cage.joints.get("valve_l", 0.0))
    vr = -float(cage.joints.get("valve_r", 0.0))

    bottle_local = cage.to_local(bottle.pos)
    bottle_ok = (
        _tilt_deg(bottle) <= BOTTLE_TILT_DEG
        and abs(float(bottle_local[0]) - POCKET_X) <= POCKET_XY_TOL
        and abs(float(bottle_local[1])) <= POCKET_XY_TOL
        and abs(float(bottle_local[2]) - (POCKET_FLOOR_Z + BOTTLE_HALF_H)) <= POCKET_Z_TOL
    )
    return {
        "upright": upright,
        "sealed": upright and lid_err <= LID_SEALED_M,
        "on_runners": on_runners,
        "home": home,
        "valve_l_open": vl >= VALVE_OPEN_M,
        "valve_r_open": vr >= VALVE_OPEN_M,
        "bottle_ok": bottle_ok,
        "resting": bool(cage.resting() and bottle.resting() and lid.resting()),
        "gap": gap,
        "cage_z": cage_z,
        "lid_err": lid_err,
        "valve_mm": (vl * 1e3, vr * 1e3),
    }


def _point_segment_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
    return float(np.linalg.norm(p - (a + t * ab)))


def _hands_clear(objects: dict[str, PartState], tcps) -> bool:
    if tcps is None or len(tcps) < 2:
        return False
    cage = objects["cage"]
    a_cage = cage.to_world((-CAGE_HALF, 0.0, 0.0))[:2]
    b_cage = cage.to_world((+CAGE_HALF, 0.0, 0.0))[:2]
    a_rack = np.array([RUNNER_FRONT_X, 0.0])
    b_rack = np.array([RACK_REAR_X, 0.0])
    for tp in tcps:
        p = np.asarray(tp, float)[:2]
        if _point_segment_dist(p, a_cage, b_cage) < HAND_CLEAR_M:
            return False
        if _point_segment_dist(p, a_rack, b_rack) < HAND_CLEAR_M:
            return False
    return True


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            out.append(ik.tcp_pose(q, station.BASES[side])[0])
        return (out[0], out[1])
    except Exception:
        return None


@on_reset
def _on_reset(env) -> None:
    _reset_trial()


SUBACTIONS = (
    ("lid sealed", "lid"),
    ("cage on runners", "cage"),
    ("valve L open", "cage.valve_l"),
    ("valve R open", "cage.valve_r"),
    ("arms clear", "tcps"),
)


@on_step
def _watch(env) -> None:
    """Observe-only: record the arm TCPs for the clearance check and log the sub-action end states during the final
    hold. Moves nothing and writes no joint target."""
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)
    if _trial["hold_s"] < HOLD_S:
        return
    cage = labkit._part_state(env, labkit._SPECS["cage"])
    lid = labkit._part_state(env, labkit._SPECS["lid"])
    tcps = _trial["tcps"]
    ends = (
        round(_lid_error(cage, lid), 6),
        [round(float(v), 6) for v in cage.pos],
        round(float(cage.joints.get("valve_l", 0.0)), 6),
        round(float(cage.joints.get("valve_r", 0.0)), 6),
        [[round(float(v), 4) for v in t] for t in tcps] if tcps else None,
    )
    for (name, key), end in zip(SUBACTIONS, ends):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": _trial["step"], "hold_s": _trial["hold_s"]}
        print("SUBACTION_END " + json.dumps(rec), flush=True)


# -- scoring -----------------------------------------------------------------------------------------------


def _evaluate(objects: dict[str, PartState], update: bool) -> float:
    try:
        m = _metrics(objects)
    except (KeyError, TypeError, AttributeError):  # a tracked part is missing: fail closed
        m = None
    if m is None or not m["upright"]:
        if update:
            _trial["hold_s"] = 0.0
        return 0.0
    total = S_RUNNERS if m["on_runners"] else 0.0
    if m["sealed"]:
        total += S_LID
    vl = m["home"] and m["valve_l_open"]
    vr = m["home"] and m["valve_r_open"]
    total += S_VALVE * (int(vl) + int(vr))
    final_ready = (m["sealed"] and vl and vr and m["bottle_ok"] and m["resting"]
                   and _hands_clear(objects, _trial["tcps"]))
    if update:
        _trial["hold_s"] = _trial["hold_s"] + CONTROL_DT if final_ready else 0.0
    if final_ready and _trial["hold_s"] >= HOLD_S - 1e-9:
        total += S_FINAL
    return round(total, 6)


def score(objects: dict[str, PartState]) -> float:
    """Final state only. 0.15 cage hanging level on both runners; 0.20 filter top seated in register and level on the
    cage (every plug corner within 1.5 mm); 0.25 per air valve pushed in >= 5 mm by its rack nozzle with the cage home
    against the stop; 0.15 when all of that holds with the bottle upright in its pocket, cage, top and bottle at rest and
    both grippers >= 12 cm clear, held 3 s. Called once per control step; it keeps only the hold time."""
    return _evaluate(objects, update=True)


def success(objects: dict[str, PartState]) -> bool:
    """Pass iff the full score holds (same thresholds as score(); does not advance the hold timer)."""
    return _evaluate(objects, update=False) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    """Metres from the docked, sealed goal."""
    m = _metrics(objects)
    cage = objects["cage"]
    return float(m["gap"] + abs(m["cage_z"] - HANG_LOW_Z) + abs(float(cage.pos[1])) + m["lid_err"])


# -- scripted demo (fixed bench constants only; never reads _trial or the simulator) -------------------------


class Orient(float):
    """Float-compatible wrapper carrying a 3x3 wrist rotation matrix through yam_arena.demo.Waypoint."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj._rot = np.asarray(rot, float)
        return obj

    @property
    def rot(self) -> np.ndarray:
        return self._rot


_prev_grip = _demo.grip_rotation
if not getattr(_prev_grip, "_rot_duck", False):

    def _grip_rotation(yaw=0.0, _prev=_prev_grip):
        return yaw.rot if hasattr(yaw, "rot") else _prev(yaw)

    _grip_rotation._rot_duck = True
    _demo.grip_rotation = _grip_rotation


def _pitched_out(pitch_deg: float, roll_out_deg: float, sign: float) -> Orient:
    r0 = _prev_grip(0.0)
    ry = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(-pitch_deg))
    rx = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(-sign * roll_out_deg))
    return Orient(rx @ (ry @ r0))


def demo() -> dict[str, list[_demo.Waypoint]]:
    """Square the filter top on the bench (both open fingers push its front end back), pick it up by the grip spine,
    lower its skirt into the tub mouth and let the chamfers seat it on its gasket, lift the cage by its side handles onto
    the rack runners, release, push it home against the rear stop (the nozzles open both valves), then park clear."""
    W = _demo.Waypoint

    cx = CAGE_SPAWN_POS[0]
    x_reg_out = LID_SPAWN_XY[0] + 0.1845 + 0.030
    x_reg_in = LID_SPAWN_XY[0] + LID_REG_TCP_X
    y_reg = LID_SPAWN_XY[1]
    z_reg = TOP + 0.035
    x_pick, y_pick = LID_SPAWN_XY[0] + LID_GRIP_X, LID_SPAWN_XY[1]  # grip spine of the squared top
    x_lid = cx + LID_GRIP_X - LID_RELEASE_BACK_M  # the same spine over the cage, just behind register
    z_lid_pick = LID_SPAWN_POS[2] + LID_GRIP_Z + TIP_BELOW_TCP
    z_lid_above = z_lid_pick + 0.060
    z_seat = CAGE_SPAWN_POS[2] + LID_GRIP_Z + TIP_BELOW_TCP
    z_release = z_seat + LID_RELEASE_UP
    z_carry = TOP + LID_CARRY_Z
    z_press = CAGE_SPAWN_POS[2] + 0.160 + TIP_BELOW_TCP - LID_PRESS_IN

    x_start_grasp = cx + GRASP_LOCAL_X
    z_start_handle = TOP + MAT_H + 0.152
    z_start_over = TOP + MAT_H + 0.205
    z_lift = TOP + RUNNER_TOP_Z + 0.092
    x_place_grasp = PLACE_CAGE_X + GRASP_LOCAL_X
    z_place_handle = TOP + RUNNER_TOP_Z + 0.024
    z_place_up = TOP + RUNNER_TOP_Z + 0.072

    x_place_back_rim = PLACE_CAGE_X - CAGE_HALF
    z_push = TOP + RUNNER_TOP_Z + 0.007
    x_push_touch = x_place_back_rim - 0.015
    x_push_home = PUSH_CAGE_X - CAGE_HALF + 0.012
    y_push = 0.068

    out: dict[str, list[_demo.Waypoint]] = {}
    for arm, sign in (("left", +1.0), ("right", -1.0)):
        lift_orient = _pitched_out(35.0, 15.0, sign)
        push_orient = _pitched_out(25.0, 18.0, sign)
        if arm == "left":
            lid_phase = [
                W(1.0, (x_reg_out, y_reg, z_lid_above), FINGER_OPEN, "beyond the front end of the top"),
                W(0.6, (x_reg_out, y_reg, z_reg), FINGER_OPEN, "open fingers down in front of the top"),
                W(0.8, (x_reg_in, y_reg, z_reg), FINGER_OPEN, "push the top back square against both fingers"),
                W(0.4, (x_reg_in + 0.010, y_reg, z_reg), FINGER_OPEN, "back off"),
                W(0.5, (x_reg_in + 0.010, y_reg, z_lid_above), FINGER_OPEN, "lift"),
                W(0.7, (x_pick, y_pick, z_lid_above), FINGER_OPEN, "over filter top on the bench"),
                W(0.8, (x_pick, y_pick, z_lid_pick), FINGER_OPEN, "fingers into the side pockets"),
                W(0.5, (x_pick, y_pick, z_lid_pick), FINGER_CLOSED, "pinch the grip spine"),
                W(1.8, (x_pick, y_pick, z_carry), FINGER_CLOSED, "lift the top"),
                W(0.5, (x_lid, y_pick, z_carry), FINGER_CLOSED, "draw back clear of the rack"),
                W(1.4, (x_lid, 0.0, z_carry), FINGER_CLOSED, "carry over the cage"),
                W(0.8, (x_lid, 0.0, z_release), FINGER_CLOSED, "lower skirt into the tub mouth"),
                W(0.4, (x_lid, 0.0, z_release), FINGER_OPEN, "release onto the gasket"),
                W(0.4, (x_lid, 0.0, z_release), FINGER_OPEN, "let the top settle on its gasket"),
                W(0.8, (x_lid, 0.0, z_carry), FINGER_OPEN, "lift clear"),
            ]
            lid_s = sum(w.duration for w in lid_phase)
        else:  # the right arm waits outboard while the left arm seats the top
            lid_phase = [W(lid_s, (x_start_grasp, -0.200, z_start_over), FINGER_STRADDLE, "wait outboard", lift_orient)]
        out[arm] = lid_phase + [
            W(1.0, (x_start_grasp, sign * GRASP_Y, z_start_over), FINGER_STRADDLE, "over side handles", lift_orient),
            W(0.8, (x_start_grasp, sign * GRASP_Y, z_start_handle), FINGER_STRADDLE, "straddle handle rails", lift_orient),
            W(0.6, (x_start_grasp, sign * GRASP_Y, z_start_handle), FINGER_CLOSED, "pinch handle rails", lift_orient),
            W(0.9, (x_start_grasp, sign * GRASP_Y, z_lift), FINGER_CLOSED, "lift cage clear", lift_orient),
            W(1.5, (x_place_grasp, sign * GRASP_Y, z_lift), FINGER_CLOSED, "carry over runners", lift_orient),
            W(0.8, (x_place_grasp, sign * GRASP_Y, z_place_handle), FINGER_CLOSED, "set flanges on runners", lift_orient),
            W(0.5, (x_place_grasp, sign * GRASP_Y, z_place_handle), FINGER_STRADDLE, "release handle rails", lift_orient),
            W(0.7, (x_place_grasp, sign * GRASP_Y, z_place_up), FINGER_STRADDLE, "lift clear of handles", lift_orient),
            W(0.8, (0.215, sign * 0.135, TOP + 0.210), FINGER_CLOSED, "swing behind cage", push_orient),
            W(0.7, (0.202, sign * y_push, z_push), FINGER_CLOSED, "align behind rear rim", push_orient),
            W(0.5, (x_push_touch, sign * y_push, z_push), FINGER_CLOSED, "contact rear rim", push_orient),
            W(2.3, (x_push_home, sign * y_push, z_push), FINGER_CLOSED, "push home onto nozzles", push_orient),
            W(0.3, (x_push_home, sign * y_push, z_push), FINGER_CLOSED, "seat against stop", push_orient),
            W(0.4, (x_push_home - 0.055, sign * y_push, z_push), FINGER_CLOSED, "back clear of rim", push_orient),
            W(0.4, (x_push_home - 0.055, sign * y_push, TOP + 0.210), FINGER_OPEN, "lift clear of rim", push_orient),
            W(0.6, (0.250, sign * 0.140, TOP + 0.210), FINGER_OPEN, "retreat clear"),
            W(0.8, (0.140, sign * 0.320, TOP + 0.150), FINGER_OPEN, "park"),
            W(5.0, (0.140, sign * 0.320, TOP + 0.150), FINGER_OPEN, "hold parked clear"),
        ]
    return out
