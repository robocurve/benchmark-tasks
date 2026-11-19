"""Inspect and index a keyed comparator reticle carriage past an X-axis crossing beam gate and lock the vernier keeper.

Physical 1:1 optical-comparator reticle calibration substage (zero @on_step joint drives):
- Fixed pebbled ESD rubber inspection mat (`bench_mat`, 480 x 460 x 2.4 mm, mu = 0.90).
- Unbolted 1.25 kg anodized aluminium comparator reticle `stage` with brass dovetail ways along Y = -90 mm, an open
  condenser viewport, a 180-deg stepped polarity datum fence, split Z-tower gate hinge piers at (0.252, +0.035),
  and perpendicular +Y vernier keeper guide rails.
- 120 g X-axis crossing-barrier `beam_gate` (`gate_hinge`, 0..52 deg about -X in vertical Y-Z plane, 0.10 N*m/rad
  return spring, 0.003 N*m friction, zero open detent). Because its rotational axis (-X) is parallel to the reticle's
  +X motion, +X pushes against the closed gate produce 0.000 N*m opening torque: unheld (`ABLATION left_parked`)
  reticle insertion stalls dead at ~23 mm (`score == 0.0`). Held +44 deg open by left arm, its shutter blade lifts
  +42 mm clear above the reticle datum nose, then returns behind the seated carriage before keeper lock-out.
- 82 g polarity-keyed `reticle_carriage` (`reticle_slide`, 90 mm +X stroke to datum, 0.18 N slide detent) carrying a
  north datum nose (blocked at 0 deg) and a south -Y vernier skirt (`Y = -108 mm`) whose 18 mm notch aligns near +90 mm.
- 65 g perpendicular `vernier_keeper` (`keeper_slide`, 54 mm +Y stroke, 0.18 N detent) whose twin brass fingers lock
  both the seated reticle's vernier notch and the top of the closed beam gate's shutter tooth.
"""
from __future__ import annotations

import math
import pathlib
import sys
import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402,F401
import mechkit as mk  # noqa: E402
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from pitch_demo import Waypoint, hinge_turn  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP
Z_HOME = TOP + 0.002
SX, SY = 0.268, 0.000
PIV_X, PIV_Y, PIV_Z = 0.252, 0.035, 0.020
RX, RY = 0.218, -0.090
KX, KY = 0.308, -0.181

STEEL, SLATE, BLACK = (0.66, 0.69, 0.72), (0.25, 0.28, 0.32), (0.09, 0.11, 0.13)
BLUE, AMBER, GREEN = (0.14, 0.34, 0.58), (0.88, 0.56, 0.10), (0.16, 0.46, 0.34)
BRASS, WHITE, RED = (0.76, 0.60, 0.26), (0.90, 0.91, 0.88), (0.72, 0.16, 0.14)
MAT_COL = (0.15, 0.18, 0.20)

STAGE_M, GATE_M, RETICLE_M, KEEPER_M = 1.250, 0.120, 0.082, 0.065
GATE_MAX_DEG = 52.0
GATE_SPRING = 0.10  # N*m/rad (0.0122 N*m at 7 deg > 4x 0.003 N*m friction; 0.000 N*m camming under +X push)
RETICLE_STROKE = 0.090  # 90 mm +X slide -> seated center at X = 0.308 m
KEEPER_STROKE = 0.054  # 54 mm +Y perpendicular vernier keeper stroke -> seated Y = -0.127 m
HANDLE_Z = 0.065
GH_DY, GH_Z = 0.012, 0.074  # gate handle offset in beam_gate local frame -> world (0.252, +0.047, +0.074)


def _handle_rot(at_x: float, at_y: float, at_z: float, color, yaw: float = 0.0):
    """3-axis shoulder-caged handle (cheeks +-22.5 mm for >6 mm pad slack & narrow cap so housing never snags)."""
    cy, sy = math.cos(yaw), math.sin(yaw)

    def _p(lx: float, ly: float, lz: float):
        return (at_x + lx * cy - ly * sy, at_y + lx * sy + ly * cy, lz)

    return [
        mk.box((0.024, 0.018, 0.034), at=_p(0.0, 0.0, at_z), yaw=yaw, color=color),
        mk.box((0.005, 0.014, 0.034), at=_p(-0.0225, 0.0, at_z), yaw=yaw, color=SLATE),
        mk.box((0.005, 0.014, 0.034), at=_p(0.0225, 0.0, at_z), yaw=yaw, color=SLATE),
        mk.box((0.050, 0.018, 0.004), at=_p(0.0, 0.0, at_z - 0.019), yaw=yaw, color=STEEL),
        mk.box((0.014, 0.014, 0.003), at=_p(0.0, 0.0, at_z + 0.0185), yaw=yaw, color=WHITE),
    ]


kit = mk.Kit()
kit.part("stage", (SX, SY, Z_HOME), [
    mk.box((0.196, 0.284, 0.012), at=(0.0, 0.0, 0.006), color=SLATE),
    mk.box((0.184, 0.272, 0.003), at=(0.0, 0.0, 0.0135), color=STEEL),
    # Reticle dovetail ways along Y = -0.090 m (`X = 0.194..0.344`) + 180-deg stepped polarity datum stop at X = 0.328
    mk.box((0.148, 0.006, 0.010), at=(0.001, -0.072, 0.020), color=BRASS),
    mk.box((0.148, 0.006, 0.010), at=(0.001, -0.108, 0.020), color=BRASS),
    mk.box((0.008, 0.008, 0.020), at=(0.064, -0.085, 0.025), color=RED),
    mk.box((0.006, 0.008, 0.020), at=(0.067, -0.095, 0.025), color=BLACK),
    # Split X-axis gate hinge piers at (PIV_X, PIV_Y) = (0.252, +0.035) + positive closed-gate rest pad at Y = -0.038
    mk.box((0.008, 0.024, 0.018), at=(PIV_X - SX - 0.014, PIV_Y - SY, 0.024), color=STEEL),
    mk.box((0.008, 0.024, 0.018), at=(PIV_X - SX + 0.014, PIV_Y - SY, 0.024), color=STEEL),
    mk.box((0.016, 0.016, 0.005), at=(PIV_X - SX, -0.038 - SY, 0.0175), color=BLACK),
    # Perpendicular +Y vernier keeper guide rails around X = 0.308, Y = -0.181..-0.127
    mk.box((0.006, 0.068, 0.012), at=(KX - SX - 0.016, -0.149, 0.021), color=BRASS),
    mk.box((0.006, 0.068, 0.012), at=(KX - SX + 0.016, -0.149, 0.021), color=BRASS),
    *[mk.box((0.016, 0.004, 0.010), at=(x, y, 0.007), yaw=math.pi / 4 if x * y > 0 else -math.pi / 4, color=STEEL)
      for x in (-0.091, 0.091) for y in (-0.135, 0.135)],
    *[mk.box((0.008, 0.008, 0.003), at=(x, y, 0.0165), color=BRASS)
      for x in (-0.083, 0.083) for y in (-0.124, 0.124)],
], mass=STAGE_M, color=SLATE)

kit.part("beam_gate", (PIV_X, PIV_Y, Z_HOME), [
    # Pivot axle block at (0, 0, PIV_Z) + vertical north riser to handle at (0.0, GH_DY, GH_Z)
    mk.box((0.018, 0.020, 0.008), at=(0.0, 0.0, PIV_Z), color=BLUE),
    mk.box((0.016, 0.016, 0.036), at=(0.0, GH_DY / 2.0, PIV_Z + 0.022), color=BLUE),
    # Horizontal shutter arm extending south to Y = -0.060 (`y_g = -0.095`) + tooth across reticle datum nose
    mk.box((0.008, 0.100, 0.006), at=(0.0, -0.048, PIV_Z + 0.004), color=STEEL),
    mk.box((0.006, 0.014, 0.014), at=(0.0, -0.095, 0.024), color=AMBER),
    mk.box((0.010, 0.012, 0.002), at=(0.0, -0.095, 0.032), color=RED),
    *_handle_rot(0.0, GH_DY, GH_Z, BLUE, -math.pi / 2.0),
], mass=GATE_M, color=BLUE)

kit.part("reticle_carriage", (RX, RY, Z_HOME), [
    # Reticle sled at Y = -0.090 (`X = 0.210..0.226` at reset; `0.300..0.316` seated -> +45 mm east of gate tooth!)
    mk.box((0.016, 0.014, 0.016), at=(0.0, 0.0, 0.028), color=STEEL),
    # 180-deg asymmetric polarity key nose at X = 0.230 (datum 0.320) + tall handle riser at RY = -0.090
    mk.box((0.008, 0.006, 0.014), at=(0.012, 0.002, 0.028), color=AMBER),
    mk.box((0.014, 0.012, 0.004), at=(0.0, 0.0, 0.038), color=WHITE),
    mk.box((0.014, 0.014, 0.010), at=(0.0, 0.0, 0.043), color=STEEL),
    # North blocking datum lug (`Y = -0.067..-0.053, Z = 0.020..0.034`): hits X-hinged shutter tooth cleanly when unheld!
    mk.box((0.014, 0.014, 0.014), at=(0.0, 0.030, 0.027), color=AMBER),
    # Machined -Y vernier skirt (`Y = -0.108`) with 18 mm locking notch centered on `x_r = 0.0`
    mk.box((0.008, 0.006, 0.014), at=(-0.0130, -0.016, 0.027), color=BRASS),
    mk.box((0.016, 0.006, 0.014), at=(0.0170, -0.016, 0.027), color=BRASS),
    *_handle_rot(0.0, 0.0, HANDLE_Z, AMBER, 0.0),
], mass=RETICLE_M, color=STEEL)

kit.part("vernier_keeper", (KX, KY, Z_HOME), [
    mk.box((0.026, 0.024, 0.012), at=(0.0, 0.0, 0.024), color=GREEN),
    # Primary +Y vernier locking tongue (enters reticle notch at Y = -0.109..-0.103 when q_keep >= 0.048, clears sled)
    mk.box((0.008, 0.019, 0.014), at=(0.0, 0.0185, 0.027), color=BRASS),
    # Secondary west + north overhead latch finger (stays south of reticle at q=0, locks gate <= 3 deg at q=0.054)
    mk.box((0.060, 0.008, 0.007), at=(-0.026, 0.016, 0.0415), color=STEEL),
    mk.box((0.008, 0.051, 0.007), at=(-0.056, 0.0415, 0.0415), color=BRASS),
    *_handle_rot(0.0, 0.0, HANDLE_Z, GREEN, math.pi / 2.0),
], mass=KEEPER_M, color=GREEN)

kit.hinge(
    "gate_hinge", "stage", "beam_gate", anchor=(PIV_X, PIV_Y, Z_HOME + PIV_Z), axis=(-1.0, 0.0, 0.0),
    lower_deg=0.0, upper_deg=GATE_MAX_DEG, start_deg=0.0, rest_deg=0.0,
    stiffness=GATE_SPRING, damping=0.02, friction=0.003,
)
kit.slider(
    "reticle_slide", "stage", "reticle_carriage", axis=(1.0, 0.0, 0.0),
    lower=0.0, upper=RETICLE_STROKE, start=0.0, damping=1.0, friction=0.18,
)
kit.slider(
    "keeper_slide", "stage", "vernier_keeper", axis=(0.0, 1.0, 0.0),
    lower=0.0, upper=KEEPER_STROKE, start=0.0, damping=1.0, friction=0.18,
)

SLUG = "reticle-gate-vernier-keeper"
for _n in ("stage", "beam_gate", "reticle_carriage", "vernier_keeper"):
    mk.VISUALS[_n] = f"assets/{SLUG}/{_n}.usdc"
labkit.VISUALS["bench_mat/body"] = str(pathlib.Path(_HERE) / "assets" / SLUG / "bench_mat.usdc")

BENCH_MAT = labkit.part(
    "bench_mat", (0.275, 0.0, TOP), fixed=True, friction=0.90, color=MAT_COL,
    boxes=[((0.0, 0.0, 0.0012), (0.480, 0.460, 0.0024), MAT_COL)],
)
OBJECTS = [*kit.objects(), BENCH_MAT]
DYNAMIC_NAMES = tuple(kit.parts)

CLEAR = 0.045
BASE_TRAVEL, BASE_Z_MAX, BASE_TILT_DEG = 0.015, 0.010, 8.0
RETICLE_PARTIAL, RETICLE_SEATED = 0.076, 0.084  # within 14 mm / 6 mm of the 90 mm datum stop
KEEPER_ENGAGED = 0.048  # within 6 mm of full 54 mm vernier keeper stroke
GATE_CLOSED_DEG = 7.0
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"
S_FULL, S_NEAR, S_PARTIAL = 1.0, 0.7, 0.4


def _tcps():
    return [np.asarray(t, float) for t in mk.tcps()]


def components(objects: dict) -> dict:
    """Stateless final poses, joint displacements, and fail-closed dual-TCP clearance."""
    pos, quat = kit.pose(objects, "stage")
    r = mk.quat_to_mat(quat)
    tcps = _tcps()
    bounds = [(np.asarray(objects[n].low, float), np.asarray(objects[n].high, float)) for n in DYNAMIC_NAMES]
    return dict(
        reticle=float(kit.q(objects, "reticle_slide")),
        keeper=float(kit.q(objects, "keeper_slide")),
        gate_deg=float(kit.q_deg(objects, "gate_hinge")),
        base_off=float(np.linalg.norm(pos - (SX, SY, Z_HOME))),
        base_z=abs(float(pos[2] - Z_HOME)),
        base_tilt=math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0)))),
        clear=len(tcps) >= 2 and all(
            np.all(np.isfinite(t)) and all(np.linalg.norm(t - np.clip(t, lo, hi)) >= CLEAR for lo, hi in bounds)
            for t in tcps
        ),
    )


def score(objects: dict) -> float:
    """Stateless end-state score. Possible scores: 0.0, 0.4, 0.7, 1.0."""
    k = components(objects)
    if not all(math.isfinite(k[n]) for n in ("reticle", "keeper", "gate_deg", "base_off", "base_z", "base_tilt")):
        return 0.0
    if k["base_off"] > BASE_TRAVEL or k["base_z"] > BASE_Z_MAX or k["base_tilt"] > BASE_TILT_DEG:
        return 0.0
    if k["reticle"] < RETICLE_PARTIAL:
        return 0.0
    if k["reticle"] < RETICLE_SEATED or k["keeper"] < KEEPER_ENGAGED or abs(k["gate_deg"]) > GATE_CLOSED_DEG:
        return S_PARTIAL
    return S_FULL if k["clear"] else S_NEAR


def success(objects: dict) -> bool:
    return score(objects) == S_FULL


def distance(objects: dict) -> float:
    k = components(objects)
    return float(
        k["base_off"]
        + max(0.0, RETICLE_STROKE - k["reticle"])
        + max(0.0, KEEPER_STROKE - k["keeper"])
        + math.radians(max(0.0, abs(k["gate_deg"]))) * 0.095
    )


PARK_L, PARK_R = (0.14, 0.30, TOP + 0.21), (0.14, -0.30, TOP + 0.21)
RETREAT_L, RETREAT_R = (0.20, 0.23, TOP + 0.21), (0.20, -0.23, TOP + 0.21)


def demo(t_wait: float = 0.0) -> dict:
    """Open-loop bimanual plan: left tilts -X crossing gate UP & braces stage; right indexes reticle & keeper."""
    W, c, o = Waypoint, FINGER_CLOSED, -0.036
    z_grip = Z_HOME + HANDLE_Z - 0.010 + PAD_TIP_PAST_TCP
    z_gate_grip = Z_HOME + GH_Z - 0.008 + PAD_TIP_PAST_TCP
    z_high = TOP + 0.178
    high_rot = pitch_demo.rotation(0.0, pitch=math.radians(25.0))

    gh0 = (PIV_X, PIV_Y + GH_DY, z_gate_grip)
    gh0_hi = (PIV_X, PIV_Y + GH_DY, z_high)
    rot_g0 = pitch_demo.rotation(-math.pi / 2.0)
    piv_w = (PIV_X, PIV_Y, Z_HOME + PIV_Z)
    swing_rad = math.radians(44.0)
    open_wps = hinge_turn(
        gh0, rot_g0, piv_w, (-1.0, 0.0, 0.0), swing_rad,
        steps=8, duration=2.4, finger=c, label="lift beam gate clear",
    )
    gh_open, rot_g_open = open_wps[-1].pos, open_wps[-1].yaw.rotation
    close_wps = hinge_turn(
        gh_open, rot_g_open, piv_w, (1.0, 0.0, 0.0), swing_rad,
        steps=8, duration=2.2, finger=c, label="ease beam gate closed",
    )

    rot_rx = pitch_demo.rotation(0.0)
    rot_ky = pitch_demo.rotation(math.pi / 2.0)
    rx_end = RX + RETICLE_STROKE + 0.004
    ky_end = KY + KEEPER_STROKE + 0.004

    left = [
        W(1.0, gh0_hi, o, "above beam gate handle", rot=rot_g0),
        W(0.7, gh0, o, "straddle beam gate handle", rot=rot_g0),
        W(1.0, gh0, c, "grasp beam gate handle", rot=rot_g0),
        *open_wps,
        W(6.6, gh_open, c, "sustain beam gate lifted while reticle seats", rot=rot_g_open),
        *close_wps,
        W(0.5, gh0, c, "seat beam gate on rest pad", rot=rot_g0),
        W(0.6, gh0, o, "release beam gate so return spring zeroes angle", rot=rot_g0),
        W(0.8, gh0_hi, o, "lift straight clear of beam gate", rot=rot_g0),
        W(0.9, RETREAT_L, o, "high intermediate retreat left", rot=high_rot),
        W(1.1, PARK_L, o, "park clear left", rot=high_rot),
    ]

    right = [
        W(3.6, PARK_R, o, "wait at park while beam gate lifts clear", rot=high_rot),
        W(0.9, RETREAT_R, o, "stage toward reticle corridor", rot=high_rot),
        W(0.8, (RX, -0.17, z_high), o, "align south of reticle track", rot=rot_rx),
        W(0.7, (RX, RY, z_high), o, "above reticle carriage", rot=rot_rx),
        W(0.7, (RX, RY, z_grip), o, "straddle reticle carriage handle", rot=rot_rx),
        W(0.8, (RX, RY, z_grip), c, "grasp reticle carriage handle", rot=rot_rx),
        W(2.4, (rx_end, RY, z_grip), c, "drive keyed reticle carriage to +X datum", rot=rot_rx),
        W(0.4, (rx_end, RY, z_grip), c, "settle reticle carriage against datum", rot=rot_rx),
        W(0.6, (rx_end, RY, z_grip), o, "release reticle carriage handle", rot=rot_rx),
        W(0.8, (rx_end, RY, z_high), o, "lift straight clear of reticle carriage", rot=rot_rx),
        W(2.6, (KX, KY, z_high), o, "wait above vernier keeper for gate closure", rot=rot_ky),
        W(0.8, (KX, KY, z_grip), o, "straddle vernier keeper handle", rot=rot_ky),
        W(0.8, (KX, KY, z_grip), c, "grasp vernier keeper handle", rot=rot_ky),
        W(1.8, (KX, ky_end, z_grip), c, "drive vernier keeper into notch & gate latch", rot=rot_ky),
        W(0.5, (KX, ky_end, z_grip), c, "settle vernier keeper engaged", rot=rot_ky),
        W(0.7, (KX, ky_end, z_grip), o, "release vernier keeper handle", rot=rot_ky),
        W(0.8, (KX, ky_end, z_high), o, "lift straight clear of vernier keeper", rot=rot_ky),
        W(0.9, RETREAT_R, o, "high intermediate retreat right", rot=high_rot),
        W(1.1, PARK_R, o, "park clear right", rot=high_rot),
    ]

    end = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.6 + t_wait
    for arm, park in ((left, PARK_L), (right, PARK_R)):
        arm.append(W(end - sum(w.duration for w in arm), park, o, "hold clear 3 s", rot=high_rot))
    return {"left": left, "right": right}
