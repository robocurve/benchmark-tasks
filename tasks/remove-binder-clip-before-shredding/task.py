"""A clipped stack of folded invoices lies broad-face down on the station desk.
The robot removes its spring clip, reorients the paper and feeds it into a personal shredder.
The clip grips by friction; only the powered feed is scripted after the insertion sensor.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import shred_usd as S  # noqa: E402
import steptime  # noqa: E402
from scenekit import UsdBody, before_reset, on_reset, on_step  # noqa: E402
from yam_arena.analysis import link_hulls
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
USD = S.build()

STACK_XY = (0.15, 0.0)          # stack centre; broad face down, long side along y
STACK_T, STACK_L, STACK_H = S.STACK
FLAT_ANGLE = math.radians(85.5)
STACK_Z = STACK_H / 2 * math.cos(FLAT_ANGLE) + STACK_T / 2 * math.sin(FLAT_ANGLE)
FLAT_Q = (0.0, math.sin(FLAT_ANGLE / 2), 0.0, math.cos(FLAT_ANGLE / 2))
CLIP_XY = (STACK_XY[0] + STACK_H / 2 * math.sin(FLAT_ANGLE), 0.06)
CLIP_Z = STACK_Z + STACK_H / 2 * math.cos(FLAT_ANGLE)
SLOT_TOP = S.SHRED_TOP

SHREDDER = UsdBody("shredder", usd=USD["shredder"], pos=(0.0, 0.0, TOP), size=(0.16, 0.30, S.SHRED_TOP), friction=(0.6, 0.5))
DISH = UsdBody("clip_dish", usd=USD["dish"], pos=(0.0, 0.0, TOP), size=S.DISH, friction=(0.6, 0.5))
INVOICES = UsdBody("invoices", usd=USD["invoices"], pos=(*STACK_XY, TOP + STACK_Z + 0.0002), size=S.STACK,
                   rot_xyzw=FLAT_Q, mass=S.STACK_MASS, friction=(0.6, 0.5), solver_iters=64, torsional_patch=0.008)
CLIP = UsdBody("binder_clip", usd=USD["clip"], pos=(*CLIP_XY, TOP + CLIP_Z + 0.0002), rot_xyzw=FLAT_Q, size=(0.02, S.CLIP_L, 0.045),
               articulation=True, joint_pos=(("jaw", 0.0),), drives=(("jaw", 1000.0, 0.2, 5.0),),
               friction=(0.5, 0.45), solver_iters=64, torsional_patch=0.008)
OBJECTS = [SHREDDER, DISH, INVOICES, CLIP]

# -- the shredder and the clip, once per control step ---------------------------------------------------------------------
FEED_SPEED = 0.04                  # m/s
START_DEPTH = 0.015                # m into the slot: the auto-start sensor and the cutters
GONE_DEPTH = 0.02                  # m: the stack's upper edge this far below the top has passed the cutters
PINCH_OPEN = (0.004, 0.014)        # m: fingers closed on the 10 mm handle pair
def _clip_outline() -> np.ndarray:
    """Points on the clip's real shape (clip frame): its lips, sloped sides, back and handles, along its length."""
    xz = [(sx * 0.0023, z) for sx in (-1, 1) for z in (S.LIP_Z[0], -0.015, S.LIP_Z[1])]
    for f in np.linspace(0.0, 1.0, 6):
        xz += [(sx * (0.0028 + f * (S.SPINE_HW - 0.0028)), S.LIP_Z[1] + f * (S.SPINE_Z[1] - S.LIP_Z[1])) for sx in (-1, 1)]
    xz += [(x, S.SPINE_Z[1]) for x in np.linspace(-S.SPINE_HW, S.SPINE_HW, 5)]
    xz += [(sx * 0.004, z) for sx in (-1, 1) for z in (0.015, S.HANDLE_TOP)]
    ys = np.linspace(-S.CLIP_L / 2, S.CLIP_L / 2, 5)
    return np.array([[x, y, z] for x, z in xz for y in ys])


CLIP_PTS = _clip_outline()
GRIP_LINKS = ("gripper", "tip_left", "tip_right")
GRIP_HULLS = {name: link_hulls()[name] for name in GRIP_LINKS}

_SH: dict = {}


def _shred_reset() -> None:
    _SH.clear()
    _SH.update(feeding=False, jammed=False, shredded=False, clip_on=True, started_with_clip=False, pinched=False,
               shown=None, events=[], drive=None)


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _yaw_quat(q) -> np.ndarray:
    r = _rot(q)
    yaw = math.atan2(r[1, 1], r[0, 1]) - math.pi / 2      # the stack's long axis (local y) in the table plane
    if r[2, 2] < 0:  # either short edge may enter; the rollers must not turn the paper upside down
        return np.array([-math.sin(yaw / 2), math.cos(yaw / 2), 0.0, 0.0])
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def _pose(environment, name: str):
    base = environment.unwrapped
    p = _np(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - _np(base.scene.env_origins[0]), p[3:7]


def _rotvec(r: np.ndarray) -> np.ndarray:
    angle = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(angle))
    return axis * angle


PHYS_DT = 1.0 / 240.0              # s per physics step
SETTLE_S = 0.25                    # s: the throat squares the stack up to the slot


def _lift(environment, on: bool, extra_down: float = 0.0) -> None:
    """A constant force cancelling the stack's weight (less `extra_down`), or none."""
    import torch

    asset = environment.unwrapped.scene["invoices"]
    comp = asset.permanent_wrench_composer
    comp.reset()
    if on:
        f = torch.tensor([[[0.0, 0.0, S.STACK_MASS * 9.81 - extra_down]]], dtype=torch.float32, device=asset.device)
        comp.add_forces_and_torques_index(forces=f, torques=torch.zeros_like(f), is_global=True)


def _set_drive(environment, drive: dict | None) -> None:
    """Drive the stack along the feed or hold it (`drive`), every physics step, or let it go (None)."""
    import torch

    asset = environment.unwrapped.scene["invoices"]
    if not getattr(asset, "_shredder_drive", False):           # run _substep before every physics step
        origin = _np(environment.unwrapped.scene.env_origins[0])
        write = asset.write_data_to_sim

        def write_data_to_sim():
            write()
            _substep(asset, origin)

        asset.write_data_to_sim = write_data_to_sim
        asset._shredder_drive = True
    _SH["drive"] = drive
    _lift(environment, drive is not None)
    if drive is None:
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=asset.device))


def _substep(asset, origin) -> None:
    """Set the stack's velocity so that it follows the feed (or holds still) by the next physics step. It never jumps,
    so a clip gripping it is carried along by its grip."""
    d = _SH.get("drive")
    if not d:
        return
    import torch

    d["t"] += PHYS_DT
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    p0, q0 = d["p0"], d["q0"]
    if d["mode"] == "feed":
        s = min(1.0, d["t"] / SETTLE_S)
        target = np.array([p0[0] + s * (S.SLOT_C[0] - p0[0]), p0[1], p0[2] - FEED_SPEED * d["t"]])
        tq, vff = _slerp(q0, _yaw_quat(q0), s), np.array([0.0, 0.0, -FEED_SPEED])
    else:
        target, tq, vff = np.asarray(p0), np.asarray(q0), np.zeros(3)
    v = vff + (target - pos) / (4 * PHYS_DT)
    w = _rotvec(_rot(tq) @ _rot(quat).T) / (4 * PHYS_DT)
    v *= min(1.0, 0.3 / max(np.linalg.norm(v), 1e-9))
    w *= min(1.0, 4.0 / max(np.linalg.norm(w), 1e-9))
    asset.write_root_velocity_to_sim_index(root_velocity=torch.tensor([[*v, *w]], dtype=torch.float32, device=asset.device))


def _release_stack(environment) -> None:
    _SH["drive"] = None
    environment.unwrapped.scene["invoices"].permanent_wrench_composer.reset()


def stack_corners(pos, quat) -> np.ndarray:
    h = np.array([STACK_T, STACK_L, STACK_H]) / 2
    local = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return local @ _rot(quat).T + np.asarray(pos)


def clip_points(pos, quat) -> np.ndarray:
    return CLIP_PTS @ _rot(quat).T + np.asarray(pos)


def in_slot(p, below: float = 0.0) -> bool:
    """A point inside the slot's outline and below the shredder's top surface (by `below`)."""
    return (S.SLOT_X[0] <= p[0] <= S.SLOT_X[1] and S.SLOT_Y[0] <= p[1] <= S.SLOT_Y[1]
            and p[2] < TOP + SLOT_TOP - below)


def grippers(environment) -> list[dict]:
    """Each arm's tool point, rotation, finger opening and fingertip pad points (station frame)."""
    from scenekit import arm_joints
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.station import BASES

    out = []
    for side in ("left", "right"):
        p, r = ik.tcp_pose(arm_joints(environment, side, ARM_JOINTS), BASES[side])
        q7, q8 = arm_joints(environment, side, ["joint7", "joint8"])
        opening = float(-(q7 + q8))
        tip = p + 0.013 * r[:, 2]
        pads = [tip + sx * (opening / 2 + dx) * r[:, 0] + dy * r[:, 1]
                for sx in (-1, 1) for dx in (0.0, 0.005, 0.010)
                for dy in (-0.018, -0.009, 0.0, 0.009, 0.018)]
        robot = environment.unwrapped.scene[f"{side}_robot"]
        poses = _np(robot.data.body_link_pose_w)[0]
        origin = _np(environment.unwrapped.scene.env_origins[0])
        hull_poses = {name: (poses[robot.body_names.index(name)][:3] - origin,
                              _rot(poses[robot.body_names.index(name)][3:7])) for name in GRIP_LINKS}
        out.append({"hulls": hull_poses, "side": side, "tcp": p, "rot": r, "opening": opening, "pads": pads})
    return out


def _pinching(g: dict, cpos, cquat) -> bool:
    """Fingers closed on the folded-up handle pair, across it (both pads on it)."""
    if not PINCH_OPEN[0] <= g["opening"] <= PINCH_OPEN[1]:
        return False
    rc = _rot(cquat)
    tcp = (g["tcp"] - cpos) @ rc
    tip = tcp + 0.013 * (g["rot"][:, 2] @ rc)
    across = abs(float(g["rot"][:, 0] @ rc[:, 0]))          # the fingers close along the clip's x (across the paper)
    hx, hy, hz = S.HANDLE_BOX
    return bool(across > 0.8 and abs(tcp[0]) < 0.012 and abs(tcp[1]) < 0.02 and tip[2] < hz[1] - 0.003
                and tcp[2] + 0.03 > hz[0])


def _show(names_on: set, names_off: set) -> None:
    try:
        import omni.usd
        from pxr import UsdGeom
    except ImportError:
        return
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return
    if not _SH.get("prims"):
        _SH["prims"] = {p.GetName(): p for p in stage.Traverse()
                        if p.GetName() in ("jam_on", "jam_off", "shreds") and "/shredder/" in str(p.GetPath())}
    for name, prim in _SH["prims"].items():
        if name in names_on:
            UsdGeom.Imageable(prim).MakeVisible()
        elif name in names_off:
            UsdGeom.Imageable(prim).MakeInvisible()


def _lights() -> None:
    state = (_SH["jammed"], _SH["shredded"])
    if _SH.get("shown") == state:
        return
    on, off = set(), set()
    (on if _SH["jammed"] else off).add("jam_on")
    (off if _SH["jammed"] else on).add("jam_off")
    (on if _SH["shredded"] else off).add("shreds")
    _show(on, off)
    _SH["shown"] = state


@before_reset
def _put_back(environment) -> None:
    from scenekit import set_joint_targets, write_joint_positions

    set_joint_targets(environment, "binder_clip", {"jaw": S.JAW_CLOSED})
    write_joint_positions(environment, "binder_clip", {"jaw": 0.0})
    _release_stack(environment)
    _shred_reset()
    _lights()


def clip_overlaps_edge(spos, squat, cpos) -> bool:
    """The clip has not cleared the paper while its jaw region overlaps the clipped edge.
    Conservative during opening: the clip must be pulled clear, not merely squeezed.
    """
    rel = (np.asarray(cpos) - np.asarray(spos)) @ _rot(squat)
    return bool(abs(rel[0]) < 0.008 and abs(rel[1]) < STACK_L / 2 + S.CLIP_L / 2
                and STACK_H / 2 - 0.012 < rel[2] < STACK_H / 2 - S.LIP_Z[0] + 0.001)


@on_step
def _run_shredder(environment) -> None:
    """The clip's handles and the shredder's sensor, feed and jam, once per control step."""
    from scenekit import set_joint_targets

    sh = _SH
    spos, squat = _pose(environment, "invoices")
    cpos, cquat = _pose(environment, "binder_clip")
    gs = grippers(environment)
    sh["grippers"] = gs
    # the clip: open while a gripper pinches its handles
    sh["pinched"] = any(_pinching(g, cpos, cquat) for g in gs)
    set_joint_targets(environment, "binder_clip", {"jaw": S.JAW_OPEN if sh["pinched"] else S.JAW_CLOSED})
    # Track current overlap every step: removing and reattaching the clip cannot bypass the feed interlock.
    on = clip_overlaps_edge(spos, squat, cpos)
    if on != sh["clip_on"]:
        sh["events"].append(("clip on" if on else "clip off", _T.get("step", 0) + 1))
    sh["clip_on"] = on
    # a clip in the slot jams the shredder: the feed stops for good, and the stack stays stuck where it is
    if not sh["jammed"] and any(in_slot(p) for p in clip_points(cpos, cquat)):
        sh["jammed"] = True
        sh["events"].append(("jam", _T.get("step", 0) + 1))
        if sh["feeding"] or any(in_slot(p) for p in stack_corners(spos, squat)):
            _set_drive(environment, {"mode": "hold", "p0": spos.copy(), "q0": squat.copy(), "t": 0.0})
        sh["feeding"] = False
    # the sensor: paper 1.5 cm into the slot starts the cutters, which draw it in at 4 cm/s
    if not (sh["feeding"] or sh["jammed"] or sh["shredded"]):
        low = min(stack_corners(spos, squat), key=lambda p: p[2])
        corners = stack_corners(spos, squat)
        aligned = (abs(_rot(squat)[2, 2]) > math.cos(math.radians(12))
                   and corners[:, 0].min() >= S.SLOT_X[0] - 0.001
                   and corners[:, 0].max() <= S.SLOT_X[1] + 0.001
                   and corners[:, 1].min() >= S.SLOT_Y[0]
                   and corners[:, 1].max() <= S.SLOT_Y[1])
        if aligned and in_slot(low, START_DEPTH):
            sh["feeding"] = True
            sh["started_with_clip"] = sh["clip_on"]
            sh["events"].append(("start", _T.get("step", 0) + 1, sh["clip_on"]))
            _set_drive(environment, {"mode": "feed", "p0": spos.copy(), "q0": squat.copy(), "t": 0.0})
    if sh["feeding"] and max(stack_corners(spos, squat)[:, 2]) <= TOP + SLOT_TOP - GONE_DEPTH:
        sh["feeding"], sh["shredded"] = False, True       # past the cutters: it settles out of sight under the throat
        sh["events"].append(("shredded", _T.get("step", 0) + 1))
        _set_drive(environment, None)
    _lights()


def _slerp(a, b, t):
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a @ b < 0:
        b = -b
    q = (1 - t) * a + t * b
    return q / np.linalg.norm(q)


# -- scoring ----------------------------------------------------------------------------------------------------------------
CLEAR = 0.03                       # m: fingertips this far from the slot's opening are clear
REST = 0.01                        # m/s

_T: dict = {}


@on_reset
def _reset(_env=None) -> None:
    """Failures stay failed for the whole trial: only a reset clears them."""
    _T.clear()
    _T.update(failed=None, step=0, held=0, checked_step=-1, goal=False)


_reset()
_shred_reset()


def _slot_distance(p) -> float:
    dx = max(S.SLOT_X[0] - p[0], 0.0, p[0] - S.SLOT_X[1])
    dy = max(S.SLOT_Y[0] - p[1], 0.0, p[1] - S.SLOT_Y[1])
    dz = p[2] - (TOP + SLOT_TOP)
    return float(math.sqrt(dx * dx + dy * dy + dz * dz))


def clip_on_shredder(clip) -> bool:
    for p in clip_points(clip.pos, clip.quat_xyzw):
        if (S.SHRED_X[0] <= p[0] <= S.SHRED_X[1] and S.SHRED_Y[0] <= p[1] <= S.SHRED_Y[1]
                and p[2] > TOP + SLOT_TOP - 0.005):
            return True
    return False


def clip_held(clip) -> bool:
    """Conservative 1 cm clearance between the clip outline and the actual gripper collision hulls."""
    pts = clip_points(clip.pos, clip.quat_xyzw)
    for g in _SH.get("grippers", []):
        for name, (p, r) in g["hulls"].items():
            local = (pts - p) @ r
            for h in GRIP_HULLS[name]:
                # Every clip point must lie outside at least one hull face by 1 cm.
                # This is conservative near edges and includes the open fingers.
                if np.any(np.max(local @ h.planes[:, :3].T - h.planes[:, 3], axis=1) < 0.01):
                    return True
    return False


def clip_put_down(clip) -> bool:
    pts = clip_points(clip.pos, clip.quat_xyzw)
    on_table = (pts[:, 0].min() >= -0.19 and pts[:, 0].max() <= 0.91
                and pts[:, 1].min() >= -0.65 and pts[:, 1].max() <= 0.65
                and TOP - 0.003 <= pts[:, 2].min() <= TOP + S.DISH[2] + 0.005)
    return bool(on_table and not _SH["clip_on"] and np.linalg.norm(clip.lin_vel) < REST and not clip_held(clip)
                and not clip_on_shredder(clip))


def grippers_clear() -> bool:
    return all(_slot_distance(p) >= CLEAR for g in _SH.get("grippers", []) for p in g["pads"])


def _update(objects) -> None:
    if _T["failed"]:
        return
    if (objects["binder_clip"].pos[2] < TOP - 0.03
            or (not _SH["shredded"] and not _SH["feeding"] and objects["invoices"].pos[2] < TOP - 0.03)):
        _T["failed"] = "paper or clip fell below the desk"
    elif _SH["started_with_clip"]:
        _T["failed"] = "the shredder started with the clip on the stack"
    elif _SH["jammed"]:
        _T["failed"] = "a clip entered the slot (jam)"
    else:
        for g in _SH.get("grippers", []):
            if any(in_slot(p, 0.001) for p in g["pads"]):
                _T["failed"] = f"a {g['side']} fingertip entered the slot"


def score(objects) -> float:
    """0 for the whole trial if the shredder starts with the clip on the stack, a clip enters the slot, or a fingertip
    enters the slot. Otherwise 1.0 for the stack shredded, the clip put down off the shredder and every fingertip 3 cm
    or more from the slot; 0.3 for the clip off the stack with the stack still on the desk or in the shredder; else 0."""
    _update(objects)
    if _T["failed"]:
        return 0.0
    clip = objects["binder_clip"]
    _T["goal"] = bool(_SH["shredded"] and clip_put_down(clip) and grippers_clear())
    if _T["checked_step"] != _T["step"]:
        _T["held"] = _T["held"] + 1 if _T["goal"] else 0
        _T["checked_step"] = _T["step"]
    if _T["held"] >= 20:
        return 1.0
    p = objects["invoices"].pos
    on_desk = (_SH["shredded"] or _SH["feeding"]
               or (-0.19 <= p[0] <= 0.91 and -0.65 <= p[1] <= 0.65 and p[2] > TOP - 0.02))
    if not _SH["clip_on"] and on_desk:
        return 0.3
    return 0.0


def success(objects) -> bool:
    """The stack shredded, the clip put down and the grippers clear, with no failure; held 1 s, then the trial ends."""
    steptime.tick()  # the harness calls success() once per control step
    _T["step"] += 1
    s = score(objects)
    _log(objects, s)
    return bool(_T["goal"] and not _T["failed"])


def _log(objects, s: float) -> None:
    out = os.environ.get("TASK_SCORE_DIR")
    if not out:
        return
    os.makedirs(out, exist_ok=True)
    st, clip = objects["invoices"], objects["binder_clip"]
    gs = _SH.get("grippers", [])
    row = {"step": _T["step"], "score": s, "failed": _T["failed"], "clip_on": _SH["clip_on"], "pinched": _SH["pinched"],
           "feeding": _SH["feeding"], "jammed": _SH["jammed"], "shredded": _SH["shredded"],
           "jaw": round(clip.joints.get("jaw", 0.0), 5),
           "stack": [round(float(v), 4) for v in st.pos], "stack_q": [round(float(v), 4) for v in st.quat_xyzw],
           "clip": [round(float(v), 4) for v in clip.pos], "clip_q": [round(float(v), 4) for v in clip.quat_xyzw],
           "clip_v": round(float(np.linalg.norm(clip.lin_vel)), 4),
           "put_down": clip_put_down(clip), "on_shredder": clip_on_shredder(clip), "clear": grippers_clear(),
           "slot_dist": [round(min(_slot_distance(p) for p in g["pads"]), 4) for g in gs],
           "tcp": [[round(float(v), 4) for v in g["tcp"]] for g in gs], "opening": [round(g["opening"], 4) for g in gs],
           "events": _SH["events"], "held": _T["held"]}
    with open(os.path.join(out, "steps.jsonl"), "a") as f:
        f.write(json.dumps(row) + "\n")


# -- scripted demonstration: only physical arm actions ---------------------------------------------------------------
def demo():
    if os.environ.get("TASK_DEMO") == "nothing":
        return {"right": [Waypoint(10.0, (0.22, -0.33, TOP + 0.19), -0.0469, "wait clear", math.pi / 2)]}
    from flat_demo import make
    return make(TOP, CLIP_XY, CLIP_Z, SLOT_TOP, S.DISH_C)
