"""Dispense a cohesive mash portion from a thumb-lever disher into a ramekin, then return the disher to its rest tray.

Real object: a size-16 (2 oz, 54 mm bowl) stainless thumb-lever portion disher, 155 g, on a silicone bench mat
beside a tub of mash and a 95 g ceramic ramekin on a rubber coaster (README "Real-world build" for listings).

Mechanism, exactly as coded (README "How it is modelled"):
- disher: one free rigid body (0.099 kg, its rack and wiper added), 45 boxes: a 12-sided stainless bowl with the
  mouth down (54 mm across flats), a bored cap ring, a rolled rim, a collar, a 152 mm handle with a cushioned
  elastomer grip sleeve between guard flanges, and two guide rails.
- thumb rack: a prismatic joint on the disher along +y, 0.030 m travel, spring return (100 N/m: 3.0 N at full stroke,
  softened from the real lever's 12 N to stay inside the arm's push budget). Includes an outboard carriage, a
  flanged 12 mm thumb tab on the left-arm side, a bridge, and a detent block that blocks the wiper drive arm.
- wiper: a hinge about the bowl axis (0 = blocked at ~15 deg by the rack's detent block, 180 deg = swept). A light
  spring (0.008 N m/rad toward 180 deg) sweeps the wiper once the rack draws past 27 mm and slides its detent block
  clear of the drive arm's -y sweep path, so the swept end-state physically cannot be reached by shaking.
- mash plug: a free 0.065 kg stepped plug inside the bowl, held by cohesion (a `mechkit` bond released when
  thumb_rack >= 0.027 m and wiper_sweep >= 150 deg).

Scored strictly from observable physical end-state geometry (mechkit poses, wiper hinge angle, and gripper TCPs).

Run it:
  inspect-robots run --embodiment yam_arena -E task=portion_disher_level_and_sweep_release.py -E object_jitter=0 \\
    -E control_hz=20 -E cam_width=640 -E cam_height=360 --instruction "<see task.json>" --scorer task_score ...
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
import mechkit as mk  # noqa: E402, F401  (patches the task scene before labkit does)
import labkit  # noqa: E402, F401
import pitch_demo  # noqa: E402, F401
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- look: brushed steel, dark steel, black elastomer, amber acetal, porcelain, mash, rubber ----------------------
STEEL, STEEL_D = (0.63, 0.65, 0.68), (0.42, 0.44, 0.47)
ELAST, ACETAL = (0.11, 0.11, 0.12), (0.80, 0.55, 0.12)
PORC, BAND, MASH = (0.94, 0.93, 0.89), (0.16, 0.30, 0.55), (0.94, 0.87, 0.66)
MASH_LT, TUB_W, TUB_RIM = (0.98, 0.93, 0.76), (0.90, 0.90, 0.87), (0.55, 0.57, 0.60)
MAT, RUBBER = (0.17, 0.19, 0.21), (0.22, 0.24, 0.26)

# -- layout (station frame: x forward from the arm bases, y left, z above the table top) --------------------------
DX, DY = 0.30, -0.02  # disher at rest: bowl axis
RX, RY = 0.24, 0.06  # ramekin on its coaster (left-arm side)
TX, TY = 0.44, -0.16  # supply tub on its coaster

# -- disher, its own frame (origin = bowl/mash center at DISHER_Z0 above table top) -------------------------------
DISHER_Z0 = 0.128
Z_MOUTH, Z_CAP, BOWL_A, BOWL_T, RIM_A = 0.104 - DISHER_Z0, 0.154 - DISHER_Z0, 0.0297, 0.0055, 0.0335
H_Y0, H_Y1, H_Z0, H_Z1 = -0.048, -0.192, 0.130 - DISHER_Z0, 0.144 - DISHER_Z0
GRIP_Y, GRIP_Z = -0.195, 0.160 - DISHER_Z0  # raised elastomer pommel block (top-down Y-pinch, 0 deg yaw)
TAB_X, TAB_Y, TAB_Z = 0.028, 0.0725, 0.114 - DISHER_Z0  # thumb tab on left-arm side
RACK_STROKE = 0.030
PLUG_A, PLUG_Z0, PLUG_Z1 = 0.0160, 0.108 - DISHER_Z0, 0.148 - DISHER_Z0

DISHER_M, RACK_M, WIPER_M = 0.099, 0.030, 0.026
MASH_M, RAMEKIN_M, TUB_M = 0.065, 0.095, 0.280
K_RACK, K_WIPER, C_WIPER = 60.0, 0.005, 0.0015  # N/m, N m/rad, N m s/rad


def _ring(apothem, z0, z1, t, colour, n=12):
    """A hollow polygonal wall: ``n`` plates on a circle of ``apothem`` (mechkit boxes)."""
    side = 2.0 * apothem * math.tan(math.pi / n) * 1.04
    return [mk.box((side, t, z1 - z0), at=(apothem * math.cos(2 * math.pi * k / n),
                                           apothem * math.sin(2 * math.pi * k / n), (z0 + z1) / 2),
                   color=colour, yaw=2 * math.pi * k / n + math.pi / 2) for k in range(n)]


def _disc(apothem, z0, z1, colour, n=12):
    """A solid regular polygon: n/2 slabs across it."""
    side = 2.0 * apothem * math.tan(math.pi / n)
    return [mk.box((2.0 * apothem, side, z1 - z0), at=(0.0, 0.0, (z0 + z1) / 2), color=colour, yaw=k * math.pi / n)
            for k in range(0, n, 2)]


def _lring(apothem, z0, z1, t, colour, n=12):
    """The same wall for a labkit part: (centre, size, colour, yaw) tuples."""
    side = 2.0 * apothem * math.tan(math.pi / n) * 1.04
    return [((apothem * math.cos(2 * math.pi * k / n), apothem * math.sin(2 * math.pi * k / n), (z0 + z1) / 2),
             (side, t, z1 - z0), colour, 2 * math.pi * k / n + math.pi / 2) for k in range(n)]


# -- the disher, its rack, its wiper and the plug (mechkit: boxes, joints, a bonded child) ----------------------
kit = mk.Kit()
_dz = DISHER_Z0
kit.part("disher", (DX, DY, TOP + _dz), [
    *_ring(BOWL_A, Z_MOUTH, Z_CAP, BOWL_T, STEEL),  # bowl, mouth down, 54 mm across flats
    *_ring(0.0325, Z_CAP, Z_CAP + 0.006, 0.006, STEEL, n=8),  # bored cap ring, the wiper's shaft comes through
    *_ring(RIM_A, Z_MOUTH - 0.005, Z_MOUTH + 0.001, 0.006, STEEL_D, n=8),  # rolled rim
    mk.box((0.024, 0.020, 0.026), at=(0.0, -0.039, 0.141 - _dz), color=STEEL_D),  # collar
    mk.box((0.020, H_Y0 - H_Y1, H_Z1 - H_Z0), at=(0.0, (H_Y0 + H_Y1) / 2, (H_Z0 + H_Z1) / 2), color=STEEL),
    mk.box((0.044, 0.022, 0.028), at=(0.0, GRIP_Y, GRIP_Z), color=ELAST),  # raised T-pommel grip sleeve (Y-pinch)
    mk.box((0.006, 0.026, 0.030), at=(0.025, GRIP_Y, GRIP_Z), color=STEEL_D),  # +x pommel cheek guard
    mk.box((0.006, 0.026, 0.030), at=(-0.025, GRIP_Y, GRIP_Z), color=STEEL_D),  # -x pommel cheek guard
    mk.box((0.0035, 0.070, 0.006), at=(0.014, -0.075, 0.144 - _dz), color=STEEL_D),  # guide rail
    mk.box((0.0035, 0.070, 0.006), at=(-0.014, -0.075, 0.144 - _dz), color=STEEL_D),
    mk.box((0.016, 0.028, 0.003), at=(0.0, -0.068, 0.146 - _dz), color=ACETAL),  # maker's badge plate
], mass=DISHER_M, color=STEEL)
kit.part("rack", (DX, DY, TOP + _dz), [
    mk.box((0.012, 0.072, 0.008), at=(0.022, 0.068, 0.154 - _dz), color=ACETAL),  # outboard carriage bar
    mk.box((0.012, 0.016, 0.028), at=(TAB_X, TAB_Y, 0.140 - _dz), color=ACETAL),  # drop strut
    mk.box((0.012, 0.022, 0.044), at=(TAB_X, TAB_Y, TAB_Z), color=ACETAL),  # thumb tab
    mk.box((0.004, 0.018, 0.036), at=(TAB_X - 0.007, TAB_Y, TAB_Z), color=ELAST),  # rubber thumb pad
    mk.box((0.018, 0.005, 0.044), at=(TAB_X, TAB_Y + 0.0135, TAB_Z), color=STEEL_D),  # +y finger flange
    mk.box((0.018, 0.005, 0.044), at=(TAB_X, TAB_Y - 0.0135, TAB_Z), color=STEEL_D),  # -y finger flange
    mk.box((0.010, 0.030, 0.008), at=(0.020, 0.020, 0.175 - _dz), color=ACETAL),  # web to the bridge
    mk.box((0.050, 0.020, 0.008), at=(0.002, 0.020, 0.179 - _dz), color=STEEL_D),  # bridge over the cap
    mk.box((0.012, 0.016, 0.010), at=(0.015, +0.015, 0.164 - _dz), color=STEEL_D),  # detent block blocking +y wiper
], mass=RACK_M, color=ACETAL)
kit.part("wiper", (DX, DY, TOP + _dz), [
    mk.box((0.014, 0.014, 0.006), at=(0.0, 0.0, 0.162 - _dz), color=STEEL_D),  # hub plate
    mk.box((0.026, 0.008, 0.004), at=(0.013, 0.0, 0.163 - _dz), color=STEEL_D),  # spider arm
    mk.box((0.0040, 0.005, 0.054), at=(0.0240, 0.0, 0.134 - _dz), color=STEEL),  # scraper blade, clear of 16 mm plug
    mk.box((0.0045, 0.005, 0.008), at=(0.0235, 0.0, 0.111 - _dz), color=ELAST),  # blade heel (r>=0.021)
    mk.box((0.017, 0.008, 0.008), at=(0.0115, 0.0, 0.164 - _dz), color=STEEL_D),  # drive arm (r<=0.020) bearing on detent
    mk.box((0.008, 0.008, 0.012), at=(0.0, 0.0, 0.156 - _dz), color=STEEL_D),  # shaft through the cap
], mass=WIPER_M, color=STEEL_D)
kit.part("mash", (DX, DY, TOP + _dz), [
    *_disc(PLUG_A, PLUG_Z0, 0.136 - _dz, MASH),
    *_disc(0.0150, 0.136 - _dz, PLUG_Z1, MASH_LT),  # stepped top of the portion
], mass=MASH_M, color=MASH)
kit.slider("thumb_rack", "disher", "rack", axis=(0.0, 1.0, 0.0), lower=0.0, upper=RACK_STROKE, start=0.0,
           stiffness=K_RACK, damping=2.0, friction=1.0)
kit.hinge("wiper_sweep", "disher", "wiper", anchor=(DX, DY, TOP + _dz), axis=(0.0, 0.0, 1.0), lower_deg=0.0,
          upper_deg=180.0, start_deg=0.0, stiffness=K_WIPER, rest_deg=180.0, damping=C_WIPER, friction=0.001)
kit.bond("mash_adhesion", "disher", "mash", break_force=200.0)

# -- the bench: mat, rest tray, ramekin, tub and their coasters (labkit: fixed parts, boxes and cylinders) -------
COASTER_H = 0.009  # m: coaster top over the table (vessels stand on it)
MAT_P = labkit.part("mat", (0.30, 0.05, TOP), fixed=True, friction=0.9, color=MAT,
                    boxes=[((0.0, 0.0, 0.001), (0.46, 0.46, 0.002), MAT)])
REST = labkit.part("rest_tray", (DX, DY, TOP), fixed=True, friction=0.8, color=STEEL_D, boxes=[
    ((0.0, -0.085, 0.003), (0.110, 0.230, 0.006), STEEL_D),
    ((0.026, 0.0, 0.096), (0.016, 0.056, 0.006), RUBBER),   # cradle pads under bowl rim (mouth bottom 99 mm)
    ((-0.026, 0.0, 0.096), (0.016, 0.056, 0.006), RUBBER),
    ((0.0, -0.026, 0.096), (0.048, 0.014, 0.006), RUBBER),
    ((0.026, 0.0, 0.049), (0.014, 0.014, 0.088), STEEL_D),
    ((-0.026, 0.0, 0.049), (0.014, 0.014, 0.088), STEEL_D),
    ((0.0, -0.150, 0.064), (0.014, 0.014, 0.122), STEEL_D),  # post under handle neck (clear of pommel)
    ((0.0, -0.150, 0.1275), (0.024, 0.018, 0.005), RUBBER),
])
COASTER_R = labkit.part("coaster_ramekin", (RX, RY, TOP), fixed=True, friction=0.9, color=RUBBER, boxes=[
    *[((c[0], c[1], 0.0075), (s[0], s[1], 0.003), RUBBER, yaw) for c, s, _k, yaw in
      _lring(0.0465, 0.006, 0.009, 0.005, RUBBER)],
], cylinders=[((0.0, 0.0, 0.0055), 0.046, 0.007, RUBBER)])
COASTER_T = labkit.part("coaster_tub", (TX, TY, TOP), fixed=True, friction=0.9, color=RUBBER, boxes=[
    *[((c[0], c[1], 0.0075), (s[0], s[1], 0.003), RUBBER, yaw) for c, s, _k, yaw in
      _lring(0.0695, 0.006, 0.009, 0.005, RUBBER)],
], cylinders=[((0.0, 0.0, 0.0055), 0.069, 0.007, RUBBER)])
RAMEKIN = labkit.part("ramekin", (RX, RY, TOP + COASTER_H), mass=RAMEKIN_M, friction=0.8, color=PORC,
                      boxes=[*_lring(0.0375, 0.012, 0.060, 0.0045, PORC), *_lring(0.0405, 0.056, 0.064, 0.004, PORC),
                             *_lring(0.0395, 0.030, 0.040, 0.0035, BAND)],
                      cylinders=[((0.0, 0.0, 0.0025), 0.024, 0.005, PORC), ((0.0, 0.0, 0.0085), 0.0395, 0.007, PORC)])
TUB = labkit.part("tub", (TX, TY, TOP + COASTER_H), mass=TUB_M, friction=0.8, color=TUB_W,
                  boxes=[*_lring(0.0585, 0.008, 0.078, 0.005, TUB_W), *_lring(0.0615, 0.074, 0.080, 0.004, TUB_RIM),
                         ((0.0, -0.042, 0.082), (0.094, 0.010, 0.008), STEEL),
                         ((0.062, 0.0, 0.048), (0.016, 0.020, 0.040), TUB_RIM),
                         ((-0.062, 0.0, 0.048), (0.016, 0.020, 0.040), TUB_RIM)],
                  cylinders=[((0.0, 0.0, 0.004), 0.055, 0.008, TUB_W), ((0.0, 0.0, 0.019), 0.050, 0.022, MASH)])

OBJECTS = [*kit.objects(), MAT_P, REST, COASTER_R, COASTER_T, RAMEKIN, TUB]

# -- thresholds (score(), the README and task.json use these names) ----------------------------------------------
CLEAR = 0.045  # m: both grippers' tool points this far from every scored body
RAMEKIN_TRAVEL, TUB_TRAVEL = 0.020, 0.025  # m: each vessel still within this of its mark
VESSEL_TILT_DEG = 8.0  # a vessel tipped more than this has been knocked over
RACK_FULL = 0.027  # m: stroke threshold releasing mash cohesion (of 0.030 m travel)
SWEEP_MIN_DEG = 150.0  # deg: swept wiper angle at end-state confirming full actuation
PARK_XY, PARK_Z, PARK_TILT_DEG = 0.045, 0.040, 15.0  # the disher back on its rest tray
MASH_XY, MASH_Z = 0.020, (0.012, 0.070)  # plug in the ramekin: off its axis, height over the table
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"
S_FULL, S_PARKED, S_MISSED = 1.0, 0.7, 0.4


def _tcps():
    return [np.asarray(t, float) for t in mk.tcps()]


def _tilt_deg(quat) -> float:
    r = mk.quat_to_mat(quat)
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2]))))))


def _rule(objects) -> None:
    """Release mash cohesion once the thumb rack clears the detent (>= 27 mm) and the wiper sweeps >= 150 deg."""
    if "mash_adhesion" not in kit._released:
        if kit.q(objects, "thumb_rack") >= RACK_FULL and abs(kit.q_deg(objects, "wiper_sweep")) >= SWEEP_MIN_DEG:
            kit.release("mash_adhesion")


kit.on_step(_rule)


def _vessel_ok(objects, name, mark, travel) -> bool:
    p = np.asarray(objects[name].pos, float)
    return (float(math.hypot(p[0] - mark[0], p[1] - mark[1])) <= travel
            and _tilt_deg(objects[name].quat_xyzw) <= VESSEL_TILT_DEG)


def components(objects) -> dict:
    """Pure observable physical end-state quantities (poses, wiper hinge angle, and gripper TCP clearance)."""
    dpos, dquat = kit.pose(objects, "disher")
    mpos = np.asarray(objects["mash"].pos, float)
    lo = np.min([np.asarray(objects[n].low, float) for n in ("disher", "ramekin", "tub", "mash")], axis=0)
    hi = np.max([np.asarray(objects[n].high, float) for n in ("disher", "ramekin", "tub", "mash")], axis=0)
    tcps = _tcps()
    return {
        "sweep_deg": abs(float(kit.q_deg(objects, "wiper_sweep"))),
        "mash_off_axis": float(math.hypot(mpos[0] - RX, mpos[1] - RY)),
        "mash_z": float(mpos[2] - TOP),
        "ramekin_ok": _vessel_ok(objects, "ramekin", (RX, RY), RAMEKIN_TRAVEL),
        "tub_ok": _vessel_ok(objects, "tub", (TX, TY), TUB_TRAVEL),
        "park_xy": float(math.hypot(dpos[0] - DX, dpos[1] - DY)),
        "park_z": float(dpos[2] - (TOP + DISHER_Z0)),
        "park_tilt": _tilt_deg(dquat),
        "clear": len(tcps) >= 2 and all(float(np.linalg.norm(t - np.clip(t, lo, hi))) >= CLEAR for t in tcps),
    }


def score(objects) -> float:
    """Judged at trial end from physical end-state; Possible scores: 0.0, 0.4, 0.7, 1.0."""
    k = components(objects)
    if not (k["ramekin_ok"] and k["tub_ok"]):
        return 0.0
    if k["sweep_deg"] < SWEEP_MIN_DEG:
        return 0.0
    if not (k["mash_off_axis"] <= MASH_XY and MASH_Z[0] <= k["mash_z"] <= MASH_Z[1]):
        return S_MISSED
    parked = k["park_xy"] <= PARK_XY and abs(k["park_z"]) <= PARK_Z and k["park_tilt"] <= PARK_TILT_DEG
    return S_FULL if parked and k["clear"] else S_PARKED


def success(objects) -> bool:
    return score(objects) >= S_FULL - 1e-9


def distance(objects) -> float:
    k = components(objects)
    return float(k["park_xy"] + k["mash_off_axis"] + max(0.0, SWEEP_MIN_DEG - k["sweep_deg"]) * 1e-3)


# -- scripted demo (fixed scene geometry only, open loop) ---------------------------------------------------------
PARK_L, PARK_R = (0.14, 0.30, TOP + 0.16), (0.18, -0.30, TOP + 0.16)
PARK_S = 3.8
CARRY_Z = -0.008  # lower bowl gently over ramekin rim (mouth 91 mm > ramekin 73 mm)
RACK_OVERDRIVE = 0.048  # compensates ~15 mm arm compliance while drawing 30 mm spring rack


def demo(t_wait=0.0):
    """Right arm grips handle sleeve from above, carries disher over ramekin, holds steady while left arm pinches
    and draws the flanged thumb tab 30 mm so the wiper sweeps >= 150 deg and releases the mash; right arm returns
    disher to rest tray, both park >= 45 mm clear for 3 s. Constants only."""
    W, c, o = Waypoint, FINGER_CLOSED, FINGER_OPEN
    rot_pinch = pitch_demo.rotation(math.pi / 2)
    rot_park = pitch_demo.rotation(0.0)

    gx, gy = DX, DY + GRIP_Y
    gz = DISHER_Z0 + GRIP_Z + PAD_TIP_PAST_TCP - 0.002
    dx, dy = RX - DX + 0.005, RY - DY + 0.010  # +5/+10 mm compensates compliant wrist carry deflection over ramekin
    tx, ty = DX + TAB_X + dx - 0.008, DY + TAB_Y + dy - 0.012
    tz = DISHER_Z0 + TAB_Z + CARRY_Z + PAD_TIP_PAST_TCP - 0.014

    def P(x, y, z):
        return (x, y, TOP + z)

    right = [W(1.4, P(gx, gy, gz + 0.040), o, "above grip sleeve", rot=rot_park),
             W(0.9, P(gx, gy, gz), o, "straddle T-pommel sleeve", rot=rot_park),
             W(0.8, P(gx, gy, gz), c, "clamp elastomer sleeve", rot=rot_park),
             W(1.0, P(gx, gy, gz + 0.025), c, "lift disher off rest tray", rot=rot_park),
             W(1.6, P(gx + dx, gy + dy, gz + 0.025), c, "carry bowl over ramekin", rot=rot_park),
             W(1.0, P(gx + dx, gy + dy, gz + CARRY_Z), c, "hold bowl above ramekin rim", rot=rot_park),
             W(7.5, P(gx + dx, gy + dy, gz + CARRY_Z), c, "steady while thumb rack is drawn", rot=rot_park)]
    left = [W(4.5, PARK_L, o, "wait while disher moves", rot=rot_park),
            W(1.6, P(tx, ty, tz + 0.070), o, "above flanged thumb tab", rot=rot_pinch),
            W(0.9, P(tx, ty, tz), o, "straddle thumb tab", rot=rot_pinch),
            W(0.7, P(tx, ty, tz), c, "pinch thumb tab", rot=rot_pinch),
            W(2.4, P(tx, ty + RACK_OVERDRIVE, tz), c, "draw rack 30 mm to clear wiper detent", rot=rot_pinch),
            W(1.6, P(tx, ty + RACK_OVERDRIVE, tz), c, "hold for 180 deg sweep & release", rot=rot_pinch),
            W(0.6, P(tx, ty + RACK_OVERDRIVE, tz), o, "release thumb tab", rot=rot_pinch),
            W(0.9, P(tx, ty + RACK_OVERDRIVE, tz + 0.070), o, "retract up", rot=rot_pinch),
            W(1.4, PARK_L, o, "park left arm clear", rot=rot_park)]
    right += [W(1.0, P(gx + dx, gy + dy, gz + 0.030), c, "lift empty disher", rot=rot_park),
              W(1.6, P(gx, gy, gz + 0.030), c, "return over rest tray", rot=rot_park),
              W(1.0, P(gx, gy, gz + 0.002), c, "seat disher on rest tray", rot=rot_park),
              W(0.6, P(gx, gy, gz + 0.002), o, "release grip sleeve", rot=rot_park),
              W(1.0, P(gx, gy, gz + 0.045), o, "lift clear", rot=rot_park),
              W(1.4, PARK_R, o, "park right arm clear", rot=rot_park)]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    end = max(tl, tr) + PARK_S + t_wait
    left.append(W(max(0.2, end - tl), PARK_L, o, "hold clear 3 s", rot=rot_park))
    right.append(W(max(0.2, end - tr), PARK_R, o, "hold clear 3 s", rot=rot_park))
    return {"left": left, "right": right}
