"""Reverse the handing of a commercial mortise lock's latchbolt so a jamb's strike latches on it: end state = bevel
facing the other way and the jamb closed and held by the latch in its strike. (The scripted demo pulls and turns the
latch with the right arm, then pushes the jamb closed with the left.)

Real product: grade-1 commercial mortise lock case, 113 x 154 x 25 mm (4-7/16 x 6-1/16 x 1 in), 19 mm (3/4 in)
latchbolt throw. The maker's installation instructions (README provenance) give the field handing procedure:
"If necessary to change handing: 1. Pull out the latch and rotate 180 deg." -- no tool, case cover stays on.

Bench layout in station coordinates (TOP = station.TABLE_TOP, +X forward, +Y left; arm bases at y = +-0.305 m):
- lockcase (fixed): the lock case standing on its back edge in a padded bench holder, faceplate up, long axis along
  Y, centred at CASE_XY. Faceplate (armored front 32 x 203 x 3.2 mm) top at FACE_TOP; its 17.5 x 29.5 mm latchbolt
  opening is the handing key: the 16 x 28 mm head only enters it square (0 or 180 deg).
- latchbolt (fixed base, two joints): "bolt" slides along the case's front axis (Z), spring-returned to the rated
  19 mm throw, from -19 mm (retracted flush) to +PULL_LIMIT (pulled for handing); "turn" is the head's free rotation
  about the same axis. At rest 6.5 mm of the head sits inside the opening, so the head can turn only when pulled out
  more than 6.5 mm; released while turned off-square it rests on the faceplate (held out on the shoulder).
- jamb (fixed guide base, one slide "close" along X): a jamb section with the strike, face 3 mm above the faceplate,
  starting 90 mm open; closed (0) puts the strike opening over the latch. Wrong handing: the lip stops on the latch's
  square back. Reversed: the lip cams the bevel in and the latch springs into the opening.
- visual-only bench props (labkit.SCENERY): no collider, no mass, outside the robot's path.

Run it:
  inspect-robots run --embodiment yam_arena -E task=mortise_latchbolt_handing_flip.py -E object_jitter=0 \\
    -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<see task.json>" --scorer task_score ...
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
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

# -- geometry (single source of truth for score(), success(), demo()) ---------------------------------------------

SCENE = "station"
TOP = station.TABLE_TOP

CASE_XY = (0.320, -0.150)
HOLDER_FLOOR = 0.012  # holder base plate under the case's back edge
CASE_DEPTH = 0.113  # 4-7/16 in, front to back
CASE_LEN = 0.154  # 6-1/16 in, along the faceplate (world Y)
CASE_T = 0.025  # 1 in thick (world X)
FRONT_T = 0.0032  # armored front 1/8 in
FRONT_W = 0.032  # 1-1/4 in
FRONT_LEN = 0.203  # 8 in
FACE_TOP = HOLDER_FLOOR + CASE_DEPTH + FRONT_T  # part-frame z of the faceplate top (0.1282 m)

THROW = 0.019  # rated latchbolt throw (3/4 in)
HEAD_T = 0.016  # head thickness (world X at 0 deg)
HEAD_L = 0.028  # head length along the faceplate (world Y at 0 deg)
HEAD_IN = 0.0065  # head length still inside the faceplate opening at full throw (handing key engagement)
HOLE_CLEAR = 0.00075  # radial play per side in the opening
HOLE_T = HEAD_T + 2 * HOLE_CLEAR  # 17.5 mm
HOLE_L = HEAD_L + 2 * HOLE_CLEAR  # 29.5 mm
CHANNEL_DEPTH = THROW + HEAD_IN + 0.002  # guide channel below the faceplate top for full retraction
PULL_LIMIT = 0.0130  # mechanical end of the handing pull (head tail 6.5 mm clear of the faceplate before it turns)
BEVEL_LOW = 0.0020  # bevel starts 2 mm above the faceplate on the strike side and runs to a 3.5 mm top land
LAND = 0.0035

# Spring: 0.2 N/mm (unverified, no published rate): 3.8 N to push it flush, 2.6 N at the handing pull.
SPRING_K = 200.0
BOLT_MASS, HEAD_MASS = 0.020, 0.045
SPRING_TARGET = (BOLT_MASS + HEAD_MASS) * 9.81 / SPRING_K  # offsets gravity so the head rests at the full throw

# Strike side: a wooden jamb section with an ANSI 4-7/8 in strike (124 x 32 mm plate, estimated opening and lip), on a
# linear guide along X that stands in for the door swing (the last 9 cm of travel at the latch, where the arc is
# almost straight). The jamb face runs 3 mm above the faceplate (door-edge clearance). Closing = jamb moves +X onto the
# latch; its leading strike lip meets the latch's -X face first, so after the flip the bevel faces the lip and cams the
# latch in; before the flip the lip meets the square back face and jams. A light return spring (door-seal pressure,
# ~1 N at closed, estimated) pushes an unlatched jamb back open.
GAP = 0.003  # door-edge clearance between faceplate and jamb face
JAMB_Z0 = FACE_TOP + GAP  # jamb face (strike plate face), part frame
JAMB_H = 0.025
STRIKE_T = 0.0016
OPEN_X0, OPEN_X1 = -0.011, 0.009  # strike opening along X at closed, relative to the latch axis
LIP_X1 = 0.034  # leading edge of the strike lip at closed
JAMB_X0 = -0.036  # trailing end of the jamb at closed
OPEN_HALF_Y = 0.016  # strike opening half-length along Y
JAMB_Y0, JAMB_Y1 = -0.075, 0.185  # jamb extent along Y (the +Y end runs on the guide)
GUIDE_Y = 0.170  # guide rail centre along Y
TRAVEL = 0.090  # jamb open position (closing travel)
CLOSE_K = 12.0  # return spring, N/m: ~1 N at closed
PUSH_Y = 0.170  # where the closing push meets the jamb's back end face (part frame, over the guide)
PUSH_BLOCK = (0.030, 0.050, 0.020)  # hardwood push block on the jamb's back end over the guide: x, y, height
CLOSED_TOL = 0.003  # jamb within 3 mm of its stop

# -- scoring thresholds (shared by on_step, score, success) -------------------------------------------------------

PULLED_MIN = HEAD_IN + 0.0005  # head tail clear of the faceplate: the handing position (sub-action log only)
TURN_TOL_DEG = 10.0  # 180 +- 10 deg
SEAT_TOL = 0.0010  # projection within 1 mm of the rated 19 mm throw
HAND_CLEAR_M = 0.050  # grippers off the lock: tool-centre points outside the faceplate box grown by 5 cm
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
PASS_SCORE = 1.0 - 1e-9
S_TURNED, S_SEATED, S_DONE = 0.4, 0.7, 1.0

# -- colours ------------------------------------------------------------------------------------------------------

STEEL = (0.72, 0.73, 0.74)
CASE_GREY = (0.42, 0.43, 0.45)
BRASS = (0.78, 0.62, 0.32)
PAD_BLACK = (0.10, 0.10, 0.11)
WOOD = (0.62, 0.45, 0.28)
ALU = (0.66, 0.67, 0.69)


def _case_boxes():
    zc = FACE_TOP - FRONT_T  # case front face (under the armored front)
    z_ch = FACE_TOP - CHANNEL_DEPTH  # channel floor
    ht, hl = HOLE_T / 2, HOLE_L / 2
    ct, cl = CASE_T / 2, CASE_LEN / 2
    boxes = [
        # case body below the guide channel
        ((0.0, 0.0, (HOLDER_FLOOR + z_ch) / 2), (CASE_T, CASE_LEN, z_ch - HOLDER_FLOOR), CASE_GREY),
        # guide channel walls (around the head's travel)
        (((ht + ct) / 2, 0.0, (z_ch + zc) / 2), (ct - ht, HOLE_L, zc - z_ch), CASE_GREY),
        ((-(ht + ct) / 2, 0.0, (z_ch + zc) / 2), (ct - ht, HOLE_L, zc - z_ch), CASE_GREY),
        ((0.0, (hl + cl) / 2, (z_ch + zc) / 2), (CASE_T, cl - hl, zc - z_ch), CASE_GREY),
        ((0.0, -(hl + cl) / 2, (z_ch + zc) / 2), (CASE_T, cl - hl, zc - z_ch), CASE_GREY),
        # armored front with the latchbolt opening
        ((0.0, (hl + FRONT_LEN / 2) / 2, FACE_TOP - FRONT_T / 2), (FRONT_W, FRONT_LEN / 2 - hl, FRONT_T), STEEL),
        ((0.0, -(hl + FRONT_LEN / 2) / 2, FACE_TOP - FRONT_T / 2), (FRONT_W, FRONT_LEN / 2 - hl, FRONT_T), STEEL),
        (((ht + FRONT_W / 2) / 2, 0.0, FACE_TOP - FRONT_T / 2), (FRONT_W / 2 - ht, HOLE_L, FRONT_T), STEEL),
        ((-(ht + FRONT_W / 2) / 2, 0.0, FACE_TOP - FRONT_T / 2), (FRONT_W / 2 - ht, HOLE_L, FRONT_T), STEEL),
        # padded bench holder: base plate + two cheeks gripping the case's lower 45 mm
        ((0.0, 0.0, HOLDER_FLOOR / 2), (0.080, 0.190, HOLDER_FLOOR), PAD_BLACK),
        ((ct + 0.008, 0.0, HOLDER_FLOOR + 0.0225), (0.016, 0.130, 0.045), PAD_BLACK),
        ((-ct - 0.008, 0.0, HOLDER_FLOOR + 0.0225), (0.016, 0.130, 0.045), PAD_BLACK),
    ]
    return tuple(boxes)


def _head_boxes():
    """Beveled head at 0 deg, bevel facing +X, in the part frame (z relative to FACE_TOP): square back face at -X, a
    3.5 mm top land, and a flat bevel from the land down to BEVEL_LOW on the +X face (one pitched slab carries the
    bevel plane; the boxes under it stay inside the profile)."""
    f, t = FACE_TOP, HEAD_T / 2
    xl = -t + LAND
    lh = HEAD_L
    run, rise = t - xl, THROW - BEVEL_LOW
    ln = math.hypot(run, rise)
    pitch = math.atan2(rise, run)  # local +X along the bevel, down toward +X; local +Z = outward normal
    nx, nz = rise / ln, run / ln
    slab = 0.003
    mx, mz = (xl + t) / 2 - nx * slab / 2, (THROW + BEVEL_LOW) / 2 - nz * slab / 2
    x_mid = 0.0
    z_mid = THROW - (x_mid - xl) * rise / run - 0.0004  # under the bevel line at x = 0
    x_q = 0.004
    z_q = THROW - (x_q - xl) * rise / run - 0.0004
    return (
        ((0.0, 0.0, f + (-HEAD_IN + BEVEL_LOW) / 2), (HEAD_T, lh, HEAD_IN + BEVEL_LOW), BRASS),  # below the bevel
        (((-t + xl) / 2, 0.0, f + (BEVEL_LOW + THROW) / 2), (LAND, lh, THROW - BEVEL_LOW), BRASS),  # back + land
        (((-t + x_mid) / 2, 0.0, f + (BEVEL_LOW + z_mid) / 2), (x_mid + t, lh, z_mid - BEVEL_LOW), BRASS),
        (((x_mid + x_q) / 2, 0.0, f + (BEVEL_LOW + z_q) / 2), (x_q - x_mid, lh, z_q - BEVEL_LOW), BRASS),
        ((mx, 0.0, f + mz), (ln, lh, slab), BRASS, (0.0, pitch)),  # the bevel face
    )


LOCKCASE = part("lockcase", (*CASE_XY, TOP), fixed=True, friction=0.3, boxes=_case_boxes())

_BOLT = Joint(
    "bolt", kind="slide", axis="Z", anchor=(0.0, 0.0, FACE_TOP),
    cylinders=(((0.0, 0.0, FACE_TOP - 0.003), 0.005, 0.006, BRASS),),
    lower=-THROW, upper=PULL_LIMIT, stiffness=SPRING_K, damping=6.0, target=SPRING_TARGET, friction=0.15,
    initial=0.0, mass=BOLT_MASS,
)
_TURN = Joint(
    "turn", kind="hinge", axis="Z", parent="bolt", anchor=(0.0, 0.0, FACE_TOP),
    boxes=_head_boxes(), lower=-3.5, upper=3.5, stiffness=0.0, damping=0.002, friction=0.002,
    initial=0.0, mass=HEAD_MASS,
)
# The latchbolt's own fixed base: a small block on the guide-channel floor (the latch tail's guide inside the case).
LATCHBOLT = part(
    "latchbolt", (*CASE_XY, TOP), fixed=True, friction=0.3,
    boxes=(((0.0, 0.0, FACE_TOP - CHANNEL_DEPTH + 0.0007), (0.004, 0.004, 0.0014), BRASS),),
    joints=(_BOLT, _TURN),
)

def _jamb_boxes():
    """Jamb section with the strike, at the closed position, in the part frame (x relative to the latch axis)."""
    z0, z1 = JAMB_Z0, JAMB_Z0 + JAMB_H
    zs = z0 + STRIKE_T
    oy = OPEN_HALF_Y
    pocket_top = z1 - 0.003
    zc = (zs + pocket_top) / 2
    hz = pocket_top - zs
    sx0, sx1 = OPEN_X0 - 0.010, LIP_X1  # strike plate along X
    sy = 0.062  # strike plate half-length (124 mm)
    return (
        # strike plate (steel, flush in the jamb face) around the opening
        (((sx0 + OPEN_X0) / 2, 0.0, z0 + STRIKE_T / 2), (OPEN_X0 - sx0, 2 * sy, STRIKE_T), STEEL),
        (((OPEN_X1 + sx1) / 2, 0.0, z0 + STRIKE_T / 2), (sx1 - OPEN_X1, 2 * sy, STRIKE_T), STEEL),  # lip
        (((OPEN_X0 + OPEN_X1) / 2, (oy + sy) / 2, z0 + STRIKE_T / 2), (OPEN_X1 - OPEN_X0, sy - oy, STRIKE_T), STEEL),
        (((OPEN_X0 + OPEN_X1) / 2, -(oy + sy) / 2, z0 + STRIKE_T / 2), (OPEN_X1 - OPEN_X0, sy - oy, STRIKE_T), STEEL),
        # wooden jamb: behind and ahead of the opening, beside it, the pocket roof, and the parts beyond the strike
        (((JAMB_X0 + OPEN_X0) / 2, 0.0, (z0 + z1) / 2 + STRIKE_T / 2), (OPEN_X0 - JAMB_X0, 2 * sy, z1 - zs), WOOD),
        (((OPEN_X1 + LIP_X1) / 2, 0.0, (zs + z1) / 2), (LIP_X1 - OPEN_X1, 2 * sy, z1 - zs), WOOD),
        (((OPEN_X0 + OPEN_X1) / 2, (oy + sy) / 2, zc), (OPEN_X1 - OPEN_X0, sy - oy, hz), WOOD),
        (((OPEN_X0 + OPEN_X1) / 2, -(oy + sy) / 2, zc), (OPEN_X1 - OPEN_X0, sy - oy, hz), WOOD),
        (((OPEN_X0 + OPEN_X1) / 2, 0.0, (pocket_top + z1) / 2), (OPEN_X1 - OPEN_X0, 2 * sy, z1 - pocket_top), WOOD),
        (((JAMB_X0 + LIP_X1) / 2, (sy + JAMB_Y1) / 2, (z0 + z1) / 2), (LIP_X1 - JAMB_X0, JAMB_Y1 - sy, z1 - z0), WOOD),
        (((JAMB_X0 + LIP_X1) / 2, (JAMB_Y0 - sy) / 2, (z0 + z1) / 2), (LIP_X1 - JAMB_X0, -sy - JAMB_Y0, z1 - z0), WOOD),
        # push block flush with the back end, over the guide
        ((JAMB_X0 + PUSH_BLOCK[0] / 2, PUSH_Y, z1 + PUSH_BLOCK[2] / 2), PUSH_BLOCK, WOOD),
    )


def _guide_boxes():
    """Fixed linear guide under the jamb's +Y end: a rail block on the bench (part frame, latch axis at x = 0)."""
    ytop = JAMB_Z0 - 0.0015
    yc = GUIDE_Y
    return (
        ((-0.060, yc, ytop / 2), (0.200, 0.020, ytop), ALU),
        ((-0.060, yc, 0.004), (0.220, 0.060, 0.008), ALU),  # foot plate
    )


_CLOSE = Joint(
    "close", kind="slide", axis="X", anchor=(0.0, 0.0, JAMB_Z0),
    boxes=_jamb_boxes(), lower=-TRAVEL, upper=0.0, stiffness=CLOSE_K, damping=4.0, target=-TRAVEL, friction=0.1,
    initial=-TRAVEL, mass=0.6,
)
JAMB = part("jamb", (*CASE_XY, TOP), fixed=True, friction=0.2, boxes=_guide_boxes(), joints=(_CLOSE,))

OBJECTS = (LOCKCASE, LATCHBOLT, JAMB)

# Visual meshes (assets/visual, built by assets_build/build_visuals.py); the shapes above stay as invisible colliders.
_VIS = pathlib.Path(_HERE) / "assets" / "visual"
labkit.VISUALS.update({
    "lockcase/body": str(_VIS / "lockcase.usdc"),
    "latchbolt/body": str(_VIS / "latch_base.usdc"),
    "latchbolt/bolt": str(_VIS / "latch_bolt.usdc"),
    "latchbolt/turn": str(_VIS / "latch_head.usdc"),
    "jamb/body": str(_VIS / "guide.usdc"),
    "jamb/close": str(_VIS / "jamb.usdc"),
})
# Visual-only bench props (SimReady hand tools, physics stripped; assets_build/build_props.py), lying flat outside the
# arms' paths: end-cutting pliers and a claw hammer.
labkit.SCENERY["lockcase"] = [(str(pathlib.Path(_HERE) / "assets" / "props" / "bench_props.usda"), (0.0, 0.0, TOP))]

# -- trial state ---------------------------------------------------------------------------------------------------

_trial: dict = {}
_ids: dict = {}


def _fresh() -> dict:
    return {"done_s": 0.0, "tcps": None, "step": -1}


_trial.update(_fresh())


def _set_test_trial(**kw) -> None:
    """Explicit test hook for offline scoring tests (never called by demo())."""
    for k, v in kw.items():
        if k not in _trial:
            raise KeyError(k)
        _trial[k] = tuple(np.asarray(p, float) for p in v) if k == "tcps" and v is not None else v


@on_reset
def _reset(_env) -> None:
    _trial.clear()
    _trial.update(_fresh())
    _ids.clear()
    if _env is not None:
        _drive_targets(_env.unwrapped)


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
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


def _q(lb: PartState) -> float:
    return float(lb.joints.get("bolt", 0.0))


def _turn_err_deg(lb: PartState, target_deg: float) -> float:
    a = math.degrees(float(lb.joints.get("turn", 0.0)))
    return abs((a - target_deg + 180.0) % 360.0 - 180.0)


def _flipped(lb: PartState) -> bool:
    return _turn_err_deg(lb, 180.0) <= TURN_TOL_DEG


def _seated(lb: PartState) -> bool:
    return abs(_q(lb)) <= SEAT_TOL


def _jamb_q(jb: PartState) -> float:
    return float(jb.joints.get("close", -TRAVEL))


def _latched(lb: PartState, jb: PartState) -> bool:
    """Jamb at its stop (within 3 mm) and the latch seated at full throw: with the jamb closed the latch can only stand
    at 19 mm inside the strike opening (anywhere else the strike plate holds it pushed in)."""
    return _seated(lb) and _jamb_q(jb) >= -CLOSED_TOL


def _hands_clear(tcps) -> bool:
    """Both tool-centre points outside the outline of the lock and the jamb over its whole travel, grown by 5 cm on
    every side."""
    if tcps is None or len(tcps) < 2:
        return False
    cx, cy = CASE_XY
    lo = np.array([cx + JAMB_X0 - TRAVEL, cy - FRONT_LEN / 2, TOP]) - HAND_CLEAR_M
    hi = np.array([cx + LIP_X1, cy + JAMB_Y1, TOP + JAMB_Z0 + JAMB_H + PUSH_BLOCK[2]]) + HAND_CLEAR_M
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


SUBACTIONS = (
    ("pull latchbolt to handing position", "latchbolt.bolt (max during run)"),
    ("turn latchbolt 180 deg", "latchbolt.turn"),
    ("seat latchbolt at rated throw", "latchbolt.bolt"),
    ("close the jamb onto the strike", "jamb.close"),
)


def _log_subaction_end(lb: PartState, step: int, run_end: bool) -> None:
    """Observe-only: one SUBACTION_END JSON line per sub-action on every step of the success hold."""
    ends = (
        round(_trial.get("q_max", 0.0), 6),
        round(math.degrees(float(lb.joints.get("turn", 0.0))), 3),
        round(_q(lb), 6),
        round(_trial.get("jamb_q", -TRAVEL), 6),
    )
    for (name, key), end in zip(SUBACTIONS, ends):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": step, "run_end": run_end}
        print("SUBACTION_END " + json.dumps(rec), flush=True)


def _drive_targets(base) -> None:
    """Keep the springs' drive targets (Isaac resets drive targets to the default joint position): the latch spring's
    gravity offset, and the jamb return spring's open end."""
    import torch

    asset = base.scene["latchbolt"]
    if "bolt_j" not in _ids:
        _ids["bolt_j"] = [list(asset.joint_names).index(_BOLT.joint_name)]
    asset.set_joint_position_target(torch.tensor([[SPRING_TARGET]], device=asset.device), joint_ids=_ids["bolt_j"])
    jamb = base.scene["jamb"]
    if "close_j" not in _ids:
        _ids["close_j"] = [list(jamb.joint_names).index(_CLOSE.joint_name)]
    jamb.set_joint_position_target(torch.tensor([[-TRAVEL]], device=jamb.device), joint_ids=_ids["close_j"])


@on_step
def _watch(env) -> None:
    base = env.unwrapped
    lb = labkit._part_state(env, labkit._SPECS["latchbolt"])
    jb = labkit._part_state(env, labkit._SPECS["jamb"])
    _trial["jamb_q"] = _jamb_q(jb)
    if _trial["step"] % 40 == 39:  # observe-only state log every 2 s (evidence for runs that do not reach the hold)
        print("STATE " + json.dumps({"step": _trial["step"] + 1, "jamb_close_m": round(_jamb_q(jb), 4),
                                     "bolt_m": round(_q(lb), 4),
                                     "turn_deg": round(math.degrees(float(lb.joints.get("turn", 0.0))), 1)}), flush=True)
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)
    q = _q(lb)
    _trial["q_max"] = max(_trial.get("q_max", 0.0), q)
    dt = float(getattr(base, "step_dt", 0.05))
    _trial["done_s"] = _trial["done_s"] + dt if score({"latchbolt": lb, "jamb": jb}) >= PASS_SCORE else 0.0
    if _trial["done_s"] > 0.0:
        _log_subaction_end(lb, _trial["step"], _trial["done_s"] >= SUCCESS_HOLD_S - 1.5 * dt)

    _drive_targets(base)


# -- scoring -------------------------------------------------------------------------------------------------------

def score(objects: dict[str, PartState]) -> float:
    """End state only (protractor, ruler, photo); no history:
    - 0.0: bevel not reversed: turn not 180 +- 10 deg from the start (untouched, tugged and let go, left at 90 deg
      on the faceplate, turned back), whatever the jamb does.
    - 0.4: bevel reversed, but not latched: the jamb is not within 3 mm of its stop with the latch standing 19 +- 1 mm
      out into the strike opening (jamb open or sprung back, latch held in or not seated).
    - 0.7: reversed and latched, but a gripper is still on the lock or jamb (tool-centre point inside their outline
      grown by 5 cm on every side), so the jamb is not shown held by the latch alone.
    - 1.0: reversed, latched, both grippers off (the run ends as a success after 1 s of this).
    """
    lb, jb = objects["latchbolt"], objects["jamb"]
    if not _flipped(lb):
        return 0.0
    if not _latched(lb, jb):
        return S_TURNED
    if not _hands_clear(_trial["tcps"]):
        return S_SEATED
    return S_DONE


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    """Remaining turn (rad, as m-equivalent at the 14 mm head radius) + seat error (m) + jamb travel left (m)."""
    lb, jb = objects["latchbolt"], objects["jamb"]
    return float(math.radians(_turn_err_deg(lb, 180.0)) * HEAD_L / 2 + abs(_q(lb)) + abs(min(_jamb_q(jb), 0.0)))


# -- scripted demo (fixed scene geometry only) ---------------------------------------------------------------------

def demo():
    """Right arm turns the latchbolt in two 90-deg strokes (joint6 +-120 deg); then the left arm closes the jamb.
    1. Pinch the head's end faces at yaw 0, pull it 13 mm past the throw, wait until the tail has cleared the opening,
       turn to yaw +90, open: the spring sets the turned head on the faceplate (held out on the shoulder).
    2. Re-pinch at yaw -90, lift, turn back to yaw 0 (head now at 180 deg), lower it into the opening, open, park.
    3. Left arm, fingers closed, pushes the jamb's back end face 9 cm onto the latch: the strike lip cams the
       reversed bevel in, the latch springs into the strike opening; back off, park clear and hold.
    """
    W = _demo.Waypoint
    c = FINGER_CLOSED
    wide = 0.45 * FINGER_OPEN  # ~42 mm jaw around the 28 mm head
    cx, cy = CASE_XY
    face = TOP + FACE_TOP
    head_top = face + THROW
    grip0 = head_top - 0.008 + PAD_TIP_PAST_TCP  # pad tips 8 mm below the head top
    pull = 0.017  # commanded past the 13 mm pull stop: the arm's position error under the spring load
    rest_q = HEAD_IN  # turned head resting on the faceplate
    hover = head_top + PULL_LIMIT + PAD_TIP_PAST_TCP + 0.018
    YAW = math.pi / 2
    park_r = (0.16, -0.32, TOP + 0.20)
    park_l = (0.20, +0.26, TOP + 0.20)

    right = [
        W(1.2, (cx, cy, hover), wide, "over the latchbolt"),
        W(0.8, (cx, cy, grip0), wide, "lower around the head"),
        W(1.2, (cx, cy, grip0), c, "pinch the head end faces"),
        W(1.0, (cx, cy, grip0 + pull), c, "pull out past the throw"),
        W(0.6, (cx, cy, grip0 + pull), c, "tail clear of the opening"),
        W(1.2, (cx, cy, grip0 + pull), c, "turn 90 deg", yaw=YAW),
        W(0.3, (cx, cy, grip0 + pull - 0.0015), c, "set the head on the faceplate", yaw=YAW),
        W(0.4, (cx, cy, grip0 + pull - 0.0015), wide, "release", yaw=YAW),
        W(0.5, (cx, cy, hover), wide, "lift clear", yaw=YAW),
        W(1.0, (cx, cy, hover), wide, "swing the wrist to -90 deg", yaw=-YAW),
        W(0.6, (cx, cy, grip0 + rest_q), wide, "lower around the turned head", yaw=-YAW),
        W(1.2, (cx, cy, grip0 + rest_q), c, "pinch again", yaw=-YAW),
        W(0.5, (cx, cy, grip0 + pull), c, "lift off the faceplate", yaw=-YAW),
        W(1.2, (cx, cy, grip0 + pull), c, "turn the last 90 deg", yaw=0.0),
        W(0.8, (cx, cy, grip0), c, "let it into the opening", yaw=0.0),
        W(0.4, (cx, cy, grip0), wide, "release at the throw", yaw=0.0),
        W(0.5, (cx, cy, hover), wide, "lift clear", yaw=0.0),
        W(1.2, park_r, wide, "park clear"),
    ]
    t_right = sum(w.duration for w in right)
    py = cy + PUSH_Y
    back = cx + JAMB_X0  # jamb back end face at closed
    tip_z = TOP + JAMB_Z0 + 0.008 + PAD_TIP_PAST_TCP  # pad tips 8 mm above the jamb face: fingers flat on the end face
    reach = 0.012  # pad tips' front face ahead of the tool-centre point along X (closed fingers)
    push = 0.045  # commanded past the stop: the arm's position error is its pushing force
    left = [
        W(1.0, park_l, wide, "stay parked"),
        W(t_right - 3.0, park_l, wide, "stay parked"),
        W(1.5, (back - TRAVEL - 0.060, py, tip_z + 0.04), c, "behind the open jamb, fingers closed"),
        W(0.5, (back - TRAVEL - 0.060, py, tip_z + 0.04), c, "settle"),
        W(0.8, (back - TRAVEL - 0.060, py, tip_z), c, "down beside the jamb's end face"),
        W(0.4, (back - TRAVEL - 0.060, py, tip_z), c, "settle"),
        W(0.8, (back - TRAVEL - reach, py, tip_z), c, "touch the end face"),
        W(2.5, (back - reach + push, py, tip_z), c, "push the jamb onto the latch"),
        W(0.5, (back - reach + push, py, tip_z), c, "hold at the stop"),
        W(0.5, (back - reach - 0.025, py, tip_z), c, "back off"),
        W(0.6, (back - reach - 0.025, py, tip_z + 0.03), c, "lift clear"),
        W(1.5, park_l, wide, "park clear"),
        W(3.0, park_l, wide, "hold clear"),
    ]
    right.append(W(sum(w.duration for w in left) - t_right, park_r, wide, "hold clear"))
    return {"left": left, "right": right}
