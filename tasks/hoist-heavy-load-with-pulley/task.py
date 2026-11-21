"""Hoist a 6 kg kettlebell with a rope over a ratchet block, hand over hand, until its base clears a red line.

A 12 mm rope is tied to the kettlebell's handle, runs up over a 75 mm sailing ratchet block hung from an aluminium
extrusion frame, and comes back down onto the table in front of the arms. The ratchet sheave turns freely when the
rope is hauled in and locks when the load pulls back, so the rope then has to slip round the locked sheave: one arm
can hold the kettlebell while the other changes grip, but one arm cannot pull it up. Both arms must haul together,
taking turns to move one hand up the rope while the other holds.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import hoistkit  # noqa: E402
import kit  # noqa: E402
import review_cams  # noqa: E402
import ropeskin  # noqa: E402
from parts import (  # noqa: E402,F401
    AXLE_X,
    AXLE_Z,
    BELL_R,
    BLOCK,
    EYE_Z_LOCAL,
    FRAME,
    FX,
    HANDLE_Z,
    HAUL_SLOPE_RAD,
    HOIST,
    LOAD_MASS,
    LOAD_POS0,
    MAT,
    MAT_TOP,
    OBJECTS,
    P_TANGENT,
    PULLEY_AXLE,
    R_PITCH,
    RATCHET_D,
    RATCHET_EFFORT,
    RATCHET_K,
    RATCHET_ON,
    ROPE_LENGTH,
    ROPE_NODES,
    ROPE_R,
    SCENE,
    TARGET_H,
    TOP,
    BEARING_DAMPING,
    haul_point,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import DOWN_GRIP, Waypoint  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix  # noqa: E402

kit.allow_matrix_grips()

SCORE_OVER = "end"

# -- scoring constants (all checkable against the board's tape lines, a tape measure and a video) --------------------

TIERS = ((0.30, 0.75), (0.20, 0.50), (0.10, 0.25))  # highest line the base has reached -> partial credit
SUCCESS_HOLD_S = 2.0  # base above the red (40 cm) line this long without a break
DROP_LIMIT = 0.10  # base more than 10 cm below the highest it has reached: the load ran back, score 0
BELL_TOUCH = 0.020  # gripper centre this close to the kettlebell's surface counts as touching it
ROPE_TOUCH = 0.030  # gripper centre this close to the rope between the kettlebell and the block counts as gripping it

HAUL_SIGN = float(os.environ.get("HOIST_HAUL_SIGN", "-1"))  # hauling in turns the sheave joint this way
RATCHET_BACK_VEL = 0.02  # rad/s turning back (0.75 mm/s of rope) engages the pawl
RATCHET_RELEASE = 0.002  # rad pulled the hauling way past the catch ...
RATCHET_RELEASE_STEPS = 2  # ... for this many control steps in a row lifts the pawl; contact jitter doesn't


class Episode:
    def __init__(self):
        self.t = 0.0
        self.lift_h = 0.0
        self.max_lift_h = 0.0
        self.above_s = 0.0
        self.best_hold_s = 0.0
        self.touched_bell = False
        self.gripped_load_rope = False
        self.dropped = False
        self.load_pos = np.array(LOAD_POS0, float)
        self.sheave = 0.0
        self.locked = False
        self.lock_at = 0.0
        self.best_p = 0.0
        self.init = False
        self.fwd_steps = 0


EP = Episode()
_TRACE = None


def score_of(ep: Episode) -> float:
    """0, 0.25, 0.5, 0.75 or 1.0 from the episode so far."""
    if ep.touched_bell or ep.gripped_load_rope or ep.dropped:
        return 0.0
    if ep.best_hold_s >= SUCCESS_HOLD_S:
        return 1.0
    for need, val in TIERS:
        if ep.max_lift_h >= need - 1e-9:
            return val
    return 0.0


def _arm_info(env, side: str) -> tuple[np.ndarray, float]:
    """A YAM arm's gripper centre (between the finger pads' tips) in the station frame, and its finger gap (m)."""
    base = env.unwrapped
    arm = base.scene[f"{side}_robot"]
    origin = kit.as_numpy(base.scene.env_origins[0])
    names = list(arm.body_names)
    pos_w = kit.as_numpy(arm.data.body_pos_w)[0] - origin
    gi = names.index("gripper")
    gq = kit.as_numpy(arm.data.body_quat_w)[0, gi]
    tcp = pos_w[gi] + kit.rotation(gq) @ np.array(TCP_OFFSET, float)
    jnames = list(arm.joint_names)
    jp = kit.as_numpy(arm.data.joint_pos)[0]
    gap = -(float(jp[jnames.index("joint7")]) + float(jp[jnames.index("joint8")]))
    return tcp, gap


def _load_side_rope(rope_points: np.ndarray) -> np.ndarray:
    """Rope links between the kettlebell and the first link touching the sheave."""
    ax = np.array([AXLE_X, AXLE_Z])
    d = np.hypot(rope_points[:, 0] - ax[0], rope_points[:, 2] - ax[1])
    on = np.nonzero(d < R_PITCH + 0.015)[0]
    end = int(on[0]) if len(on) else len(rope_points)
    return rope_points[:end]


def _bell_distance(tcp: np.ndarray, pos: np.ndarray, rot: np.ndarray) -> float:
    """Distance from a point to the kettlebell's surface (bell sphere and handle bar), m."""
    d_bell = float(np.linalg.norm(tcp - pos)) - BELL_R
    local = rot.T @ (tcp - pos)
    hz = local - np.array([0.0, 0.0, HANDLE_Z])
    d_handle = float(np.hypot(hz[0], hz[2])) - 0.0165 if abs(hz[1]) < 0.06 else 1.0
    return min(d_bell, d_handle)


def advance(ep: Episode, dt: float, hoist_st: hoistkit.HoistState, tcps) -> None:
    """Update the episode's measures for one control step."""
    ep.t += dt
    ep.load_pos = np.asarray(hoist_st.load_pos, float)
    ep.lift_h = max(0.0, float(hoist_st.load_bottom_z) - MAT_TOP)
    ep.max_lift_h = max(ep.max_lift_h, ep.lift_h)
    if ep.lift_h < ep.max_lift_h - DROP_LIMIT:
        ep.dropped = True
    load_rope = _load_side_rope(hoist_st.rope_points)
    for tcp in tcps:
        if _bell_distance(tcp, hoist_st.load_pos, hoist_st.load_rot) < BELL_TOUCH:
            ep.touched_bell = True
        if len(load_rope) and float(np.min(np.linalg.norm(load_rope - tcp[None, :], axis=1))) < ROPE_TOUCH:
            ep.gripped_load_rope = True
    if ep.lift_h >= TARGET_H:
        ep.above_s += dt
    else:
        ep.above_s = 0.0
    ep.best_hold_s = max(ep.best_hold_s, ep.above_s)


# -- the ratchet: a pawl that lets the sheave turn only the hauling way ----------------------------------------------


def _set_pawl(env, engaged: bool, at: float = 0.0) -> None:
    """Engage the pawl at sheave angle ``at`` (a stiff spring holds the sheave there) or lift it (free sheave)."""
    import torch

    asset = env.unwrapped.scene[BLOCK.name]
    dev = asset.device
    k, d = (RATCHET_K, RATCHET_D) if engaged else (0.0, BEARING_DAMPING)
    asset.write_joint_stiffness_to_sim(torch.tensor([[k]], device=dev), joint_ids=[0])
    asset.write_joint_damping_to_sim(torch.tensor([[d]], device=dev), joint_ids=[0])
    asset.set_joint_position_target(torch.tensor([[float(at)]], device=dev), joint_ids=[0])


def _ratchet(env, ep: Episode) -> tuple[float, float]:
    """The pawl: it catches the sheave as soon as it turns back and holds it there on a stiff spring; it lifts once the
    sheave has been pulled the hauling way past the catch for a few control steps in a row."""
    asset = env.unwrapped.scene[BLOCK.name]
    th = float(kit.as_numpy(asset.data.joint_pos)[0, 0])
    w = float(kit.as_numpy(asset.data.joint_vel)[0, 0])
    try:
        ep.pawl_torque = float(kit.as_numpy(asset.data.applied_torque)[0, 0])
    except Exception:  # noqa: BLE001
        ep.pawl_torque = float("nan")
    v = HAUL_SIGN * w  # > 0: turning the hauling way
    if not ep.init:
        ep.init = True
        ep.locked = False
        _set_pawl(env, False)
    if not RATCHET_ON:
        return th, w
    if ep.locked:
        if abs(th - ep.lock_at) > math.pi:  # the reported angle wrapped (it runs over +-720 deg): catch again here
            ep.lock_at = th
            _set_pawl(env, True, th)
        ep.fwd_steps = ep.fwd_steps + 1 if HAUL_SIGN * (th - ep.lock_at) > RATCHET_RELEASE else 0
        if ep.fwd_steps >= RATCHET_RELEASE_STEPS:
            ep.locked, ep.fwd_steps = False, 0
            _set_pawl(env, False)
    elif v < -RATCHET_BACK_VEL:
        ep.locked, ep.lock_at, ep.fwd_steps = True, th, 0
        _set_pawl(env, True, th)
    return th, w


def _draw_rope(hoist_st: hoistkit.HoistState) -> None:
    eye = hoist_st.load_pos + hoist_st.load_rot @ np.array([0.0, 0.0, EYE_Z_LOCAL])
    pts, sds, tan = ropeskin.centre_line(hoist_st.rope_points, hoist_st.rope_rots, hoist_st.rope_lengths, start=eye)
    core, strand = ropeskin.skin(pts, sds, tan, ROPE_R)
    kit.set_mesh("rope_core", *core)
    kit.set_mesh("rope_strand", *strand)



# -- the grip on the rope -------------------------------------------------------------------------------------------
# PhysX contact between position-controlled fingers and a chain of rigid rope links is unreliable under load (the rope
# slips out, or the solver shoots it through both hands when two hands clamp the inextensible rope). So while a
# gripper's fingers are closed on the rope, the rope link between them is held to the gripper by a stiff
# spring-damper (a firm grip on a slightly stretchy rope); it lets go as soon as the fingers open. The same rule applies
# to every policy.
GRIP_ASSIST = os.environ.get("HOIST_GRIP_ASSIST", "1") != "0"
GRIP_GAP = (0.005, 0.0155)  # finger gap (m) that means "closed on the 12 mm rope" (fully shut on nothing: < 5 mm)
GRIP_REACH = 0.025  # a rope link this close to the gripper centre is the one between the fingers
GRIP_K, GRIP_C = float(os.environ.get("HOIST_GRIP_K", "2000")), float(os.environ.get("HOIST_GRIP_C", "40"))  # N/m, N s/m
GRIP_KR, GRIP_CR = 2.0, 0.05  # N m/rad, N m s/rad: the rope can't spin freely in the fingers

_GRIPS: dict[str, object] = {}  # side -> enabled-attribute of its current joint
_GRIP_N = [0]
_PATHS: dict[str, str] = {}


def _gripper_path(env, side: str) -> str:
    if side not in _PATHS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        stage = get_current_stage()
        import re

        root = re.sub(r"env_(\.\*|\[\^/\]\+)", "env_0", env.unwrapped.scene[f"{side}_robot"].cfg.prim_path)
        for prim in stage.Traverse():
            ps = str(prim.GetPath())
            if ps.startswith(root) and prim.GetName() == "gripper" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                _PATHS[side] = ps
                break
        _PATHS.setdefault(side, "")
    return _PATHS[side]


def _grip(env, side: str, gap: float, tcp: np.ndarray) -> None:
    if not GRIP_ASSIST:
        return
    closed = GRIP_GAP[0] < gap < GRIP_GAP[1]
    if side in _GRIPS and gap >= GRIP_GAP[1]:  # the fingers opened: let go (a hand that only squeezes harder keeps it)
        _GRIPS.pop(side).Set(False)
        return
    if side in _GRIPS or not closed:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    base = env.unwrapped
    hoist = base.scene[HOIST.name]
    names = list(hoist.body_names)
    origin = kit.as_numpy(base.scene.env_origins[0])
    pos = kit.as_numpy(hoist.data.body_pos_w)[0] - origin
    quat = kit.as_numpy(hoist.data.body_quat_w)[0]
    segs = [i for i, n in enumerate(names) if n.startswith("seg_")]
    d = np.linalg.norm(pos[segs] - tcp[None, :], axis=1)
    k = int(np.argmin(d))
    if d[k] > GRIP_REACH:
        return
    li = segs[k]
    arm = base.scene[f"{side}_robot"]
    an = list(arm.body_names)
    gi = an.index("gripper")
    gpos = kit.as_numpy(arm.data.body_pos_w)[0, gi] - origin
    gq = kit.as_numpy(arm.data.body_quat_w)[0, gi]
    rg, rl = kit.rotation(gq), kit.rotation(quat[li])
    p0 = rg.T @ (pos[li] - gpos)
    q0 = kit.quat_from_matrix(rg.T @ rl)  # xyzw
    gpath = _gripper_path(env, side)
    import re

    lpath = re.sub(r"env_(\.\*|\[\^/\]\+)", "env_0", hoist.cfg.prim_path) + f"/{names[li]}"
    if not gpath:
        return
    stage = get_current_stage()
    _GRIP_N[0] += 1
    jp = f"/World/grip_{side}_{_GRIP_N[0]}"
    j = UsdPhysics.Joint.Define(stage, jp)
    j.CreateBody0Rel().SetTargets([gpath])
    j.CreateBody1Rel().SetTargets([lpath])
    j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in p0]))
    j.CreateLocalRot0Attr(Gf.Quatf(float(q0[3]), float(q0[0]), float(q0[1]), float(q0[2])))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateExcludeFromArticulationAttr(True)
    prim = j.GetPrim()
    for dof, (kk, cc) in (("transX", (GRIP_K, GRIP_C)), ("transY", (GRIP_K, GRIP_C)), ("transZ", (GRIP_K, GRIP_C)),
                          ("rotX", (GRIP_KR, GRIP_CR)), ("rotY", (GRIP_KR, GRIP_CR)), ("rotZ", (GRIP_KR, GRIP_CR))):
        drv = UsdPhysics.DriveAPI.Apply(prim, dof)
        drv.CreateTypeAttr("force")
        rot = dof.startswith("rot")
        drv.CreateStiffnessAttr(float(kk) * (math.pi / 180.0 if rot else 1.0))
        drv.CreateDampingAttr(float(cc) * (math.pi / 180.0 if rot else 1.0))
        drv.CreateTargetPositionAttr(0.0)
    _GRIPS[side] = j.CreateJointEnabledAttr(True)


def _release_grips() -> None:
    for side in list(_GRIPS):
        try:
            _GRIPS.pop(side).Set(False)
        except Exception:  # noqa: BLE001
            pass


@kit.on_reset
def _reset():
    global EP, _TRACE
    EP = Episode()
    _release_grips()
    if _TRACE is not None:
        _TRACE.close()
    path = os.environ.get("HOIST_TRACE")
    _TRACE = open(path, "w") if path else None  # noqa: SIM115


@kit.on_step
def _observe(env):
    dt = float(env.unwrapped.step_dt)
    th, w = _ratchet(env, EP) if BLOCK in OBJECTS else (0.0, 0.0)
    EP.sheave = th
    hoist_st = hoistkit._hoist_state(env, HOIST)
    ltcp, lgap = _arm_info(env, "left")
    rtcp, rgap = _arm_info(env, "right")
    _grip(env, "left", lgap, ltcp)
    _grip(env, "right", rgap, rtcp)
    advance(EP, dt, hoist_st, (ltcp, rtcp))
    _draw_rope(hoist_st)
    review_cams.capture()
    if _TRACE is not None and (round(EP.t / dt) % 2 == 0):
        l_d = float(np.min(np.linalg.norm(hoist_st.rope_points - ltcp[None, :], axis=1)))
        r_d = float(np.min(np.linalg.norm(hoist_st.rope_points - rtcp[None, :], axis=1)))
        rec = {
            "t": round(EP.t, 3),
            "lift_cm": round(EP.lift_h * 100.0, 2),
            "max_lift_cm": round(EP.max_lift_h * 100.0, 2),
            "sheave_deg": round(math.degrees(th), 1),
            "sheave_vel": round(w, 3),
            "pawl": EP.locked,
            "pawl_at": round(math.degrees(EP.lock_at), 1),
            "pawl_nm": round(getattr(EP, "pawl_torque", float("nan")), 3),
            "grips": sorted(_GRIPS),
            "l_rope_mm": round(l_d * 1000.0, 1),
            "r_rope_mm": round(r_d * 1000.0, 1),
            "l_gap_mm": round(lgap * 1000.0, 1),
            "r_gap_mm": round(rgap * 1000.0, 1),
            "ltcp": [round(float(v), 3) for v in ltcp],
            "rtcp": [round(float(v), 3) for v in rtcp],
            "bell": [round(float(v), 3) for v in EP.load_pos],
            "hold_s": round(EP.best_hold_s, 2),
            "touched_bell": EP.touched_bell,
            "load_rope": EP.gripped_load_rope,
            "dropped": EP.dropped,
            "score": score_of(EP),
        }
        if round(EP.t / dt) % 20 == 0:
            rec["rope"] = np.round(hoist_st.rope_points[:, [0, 2]], 3).tolist()
        _TRACE.write(json.dumps(rec) + "\n")
        _TRACE.flush()


def score(state, env=None) -> float:
    return score_of(EP)


def success(state, env=None) -> bool:
    return score_of(EP) >= 1.0


def distance(state, env=None) -> float:
    return max(0.0, TARGET_H - EP.lift_h)


# -- scripted demo: two-arm hand-over-hand hauling --------------------------------------------------------------------

_DOWN_MAT = quat_xyzw_to_matrix(np.asarray(DOWN_GRIP))
TABLE_ROT = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -math.radians(float(os.environ.get("HOIST_PICK_TILT", "40")))) @ _DOWN_MAT
_SQUARE = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -HAUL_SLOPE_RAD) @ _DOWN_MAT  # approach square to the rope
_U_ROPE = np.array([math.cos(HAUL_SLOPE_RAD), 0.0, math.sin(HAUL_SLOPE_RAD)])
_SIDE = math.radians(float(os.environ.get("HOIST_SIDE_DEG", "90")))
# Each hand takes the rope from its own side, like a person hauling: the left gripper reaches in from the left and the
# right one from the right, fingers closing across the rope, so the two grippers sit on opposite sides of the rope and
# never collide even when their grips are close together.
GRIP = {
    "left": axis_angle_matrix(_U_ROPE, -_SIDE) @ _SQUARE,
    "right": axis_angle_matrix(_U_ROPE, _SIDE) @ _SQUARE,
}


def _pt(d: float, side: str, back: float = 0.0, lift: float = 0.0) -> tuple[float, float, float]:
    """Point ``d`` down the hauling line from the sheave, backed off ``back`` along ``side``'s approach axis (out
    away from the rope toward its own side) and raised ``lift`` square to the rope (up and back)."""
    a = GRIP[side][:, 2]
    n_up = np.array([-math.sin(HAUL_SLOPE_RAD), 0.0, math.cos(HAUL_SLOPE_RAD)])
    p = haul_point(d) - back * a + lift * n_up
    return tuple(float(v) for v in p)


def _d_at(z_above_table: float) -> float:
    return (P_TANGENT[2] - (TOP + z_above_table)) / math.sin(HAUL_SLOPE_RAD)


def _env_f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def demo() -> dict[str, list[Waypoint]]:
    """Both arms haul together along the taut 60-deg line, each gripping from its own side; between pulls the rear hand
    lets go, backs out to its side, moves up past the holding hand and takes a new grip higher up. ``HOIST_MODE``
    checks (never for scored trials): ``single_left`` / ``single_right`` (one arm alone tries to haul), ``hold_test``
    (one hand holds alone for 4 s), ``release_test`` (both hands let go)."""
    mode = os.environ.get("HOIST_MODE", "demo")
    left: list[Waypoint] = []
    right: list[Waypoint] = []

    def add(dur, lpos, lfin, rpos, rfin, label, lrot=None, rrot=None):
        left.append(Waypoint(dur, lpos, lfin, f"L:{label}", GRIP["left"] if lrot is None else lrot))
        right.append(Waypoint(dur, rpos, rfin, f"R:{label}", GRIP["right"] if rrot is None else rrot))

    stroke = _env_f("HOIST_STROKE", 0.14)
    d_high = _d_at(_env_f("HOIST_Z_HIGH", 0.38))
    d_mid = d_high + stroke
    d_low = d_mid + stroke
    gap = _env_f("HOIST_HAND_GAP", 0.0)  # extra spacing of the two grips along the rope
    clear = _env_f("HOIST_CLEAR", 0.07)  # how far an open hand backs out to its side
    deep = _env_f("HOIST_DEEP", 0.012)  # how far past the gripper centre the rope sits (toward the palm)
    close_s, open_s = 1.2, 0.8
    pull_s = _env_f("HOIST_PULL_S", 2.6)
    n_pulls = int(_env_f("HOIST_PULLS", 5))

    O, C = FINGER_OPEN, FINGER_CLOSED
    g_x = _env_f("HOIST_PICK_X", 0.52)
    g_pick = (g_x, 0.0, TOP + ROPE_R + 0.003)
    g_hover = (g_x, 0.0, TOP + 0.10)
    d_taut = _env_f("HOIST_D_TAUT", 0.847)  # where the picked-up rope just runs taut
    r_wait = (0.18, -0.24, TOP + 0.25)
    l_wait = (0.18, 0.24, TOP + 0.25)

    def grip_at(d, side):
        return _pt(d, side, back=-deep)

    def off_at(d, side):
        return _pt(d, side, back=clear)

    # Phase 0: the left hand picks the rope up off the table and draws it taut along the hauling line
    # (the left hand holds the rope square-on, from above, until the right hand has it; the waiting hand too)
    sq = _SQUARE
    add(1.2, g_hover, O, r_wait, O, "over rope", lrot=TABLE_ROT, rrot=sq)
    add(0.8, g_pick, O, r_wait, O, "down", lrot=TABLE_ROT, rrot=sq)
    add(close_s, g_pick, C, r_wait, O, "grip rope", lrot=TABLE_ROT, rrot=sq)
    add(0.8, (g_x - 0.04, 0.0, TOP + 0.10), C, r_wait, O, "lift rope", lrot=TABLE_ROT, rrot=sq)
    taut = tuple(float(v) for v in haul_point(d_taut))
    add(2.0, taut, C, r_wait, O, "draw taut", lrot=sq, rrot=sq)

    # the right hand takes the taut rope a little above d_mid, the left lets go, and the right hauls it tight
    snug = _env_f("HOIST_SNUG", 0.06)
    add(1.2, taut, C, off_at(d_mid - snug, "right"), O, "R beside", lrot=sq)
    add(0.6, taut, C, grip_at(d_mid - snug, "right"), O, "R onto", lrot=sq)
    add(close_s, taut, C, grip_at(d_mid - snug, "right"), C, "R grip", lrot=sq)
    add(open_s, taut, O, grip_at(d_mid - snug, "right"), C, "L open", lrot=sq)
    lift_off = tuple(float(v) for v in haul_point(d_taut) - 0.07 * sq[:, 2])
    add(1.0, lift_off, O, grip_at(d_mid, "right"), C, "L off, R hauls tight", lrot=sq)

    if mode == "single_right":
        add(pull_s, l_wait, O, grip_at(d_low, "right"), C, "R hauls alone", lrot=sq)
        add(3.0, l_wait, O, grip_at(d_low, "right"), C, "R keeps hauling", lrot=sq)
        return {"left": left, "right": right}

    add(1.0, off_at(d_high - gap, "left"), O, grip_at(d_mid, "right"), C, "L up")
    add(0.6, grip_at(d_high - gap, "left"), O, grip_at(d_mid, "right"), C, "L onto")
    add(close_s, grip_at(d_high - gap, "left"), C, grip_at(d_mid, "right"), C, "L grip")

    if mode == "single_left":
        add(open_s, grip_at(d_high - gap, "left"), C, grip_at(d_mid, "right"), O, "R open")
        add(0.8, grip_at(d_high - gap, "left"), C, r_wait, O, "R away", rrot=sq)
        add(pull_s, grip_at(d_mid, "left"), C, r_wait, O, "L hauls alone", rrot=sq)
        add(3.0, grip_at(d_mid, "left"), C, r_wait, O, "L keeps hauling", rrot=sq)
        return {"left": left, "right": right}

    hands = {"left": d_high - gap, "right": d_mid}
    front, rear = "left", "right"
    for k in range(1, n_pulls + 1):
        # both hands haul together, each one stroke down the rope
        hands = {s: hands[s] + stroke for s in hands}
        add(pull_s, grip_at(hands["left"], "left"), C, grip_at(hands["right"], "right"), C, f"pull{k}")
        add(0.5, grip_at(hands["left"], "left"), C, grip_at(hands["right"], "right"), C, f"settle{k}")
        if mode in ("hold_test", "release_test") and k == 1:
            if mode == "hold_test":  # the rear hand lets go and backs out; the front hand holds alone
                seq = [(open_s, grip_at(hands[rear], rear), O, "rear open"), (0.6, off_at(hands[rear], rear), O, "rear out"),
                       (4.0, off_at(hands[rear], rear), O, "front holds alone")]
                hb = _env_f("HOIST_HOLD_BIAS", 0.04)
                for dur, pos, fin, lbl in seq:
                    if rear == "right":
                        add(dur, grip_at(hands["left"] + hb, "left"), C, pos, fin, lbl)
                    else:
                        add(dur, pos, fin, grip_at(hands["right"] + hb, "right"), C, lbl)
            else:
                add(open_s, grip_at(hands["left"], "left"), O, grip_at(hands["right"], "right"), O, "both open")
                add(0.6, off_at(hands["left"], "left"), O, off_at(hands["right"], "right"), O, "both out")
                add(3.0, off_at(hands["left"], "left"), O, off_at(hands["right"], "right"), O, "watch")
            return {"left": left, "right": right}
        if k == n_pulls:
            break
        # change hands: the rear hand lets go, backs out to its side, and takes a new grip one stroke above the front
        new_d = hands[front] - stroke
        # the holding hand keeps pulling a little (aims a few cm further down the rope), as a person does, so the soft
        # wrist doesn't give the rope back while the other hand is off it
        hold = grip_at(hands[front] + _env_f("HOIST_HOLD_BIAS", 0.04), front)
        seq = [
            (open_s, grip_at(hands[rear], rear), O, "open"),
            (0.5, off_at(hands[rear], rear), O, "out"),
            (1.1, off_at(new_d, rear), O, "up"),
            (0.6, grip_at(new_d, rear), O, "onto"),
            (close_s, grip_at(new_d, rear), C, "grip"),
        ]
        for dur, pos, fin, lbl in seq:
            if rear == "left":
                add(dur, pos, fin, hold, C, f"sw{k}_{lbl}")
            else:
                add(dur, hold, C, pos, fin, f"sw{k}_{lbl}")
        hands[rear] = new_d
        front, rear = rear, front

    add(_env_f("HOIST_HOLD_S", 3.5), left[-1].pos, C, right[-1].pos, C, "hold")
    return {"left": left, "right": right}
