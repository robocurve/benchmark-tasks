"""Two bimanual YAM stations face each other across a table, one at each end of a plywood panel that serves as the
ceiling, its underside 760 mm above the table. Four #8 pan-head wood screws are already driven into the panel, each
head left 3/8 in (9.5 mm) below it, as the light's maker instructs. The bare steel housing of a Commercial Electric
WR484080LSL 4-ft LED wrap light (lens and channel cover off, 1,218 x 320 x 55 mm, 3.7 kg) rests back up on two padded
stands, 30 mm below the ceiling and 15 mm toward station B from its pencil outline. Each arm has a push-to-start
cordless screwdriver standing on a holder beside it. The robots are told to hang the light on the four screws by its
keyholes (raise it, bring the keyholes' round centre holes over the heads, slide it into its outline so the screws go
into the narrow slots, let it down onto the heads and let go), and then each to tighten the two screws at its own end.

Task code (fidelity note): the screw model (bit on the head, push-to-start motor and clutch, thread) in _substep() and
on_step(); the contact monitor and scoring in on_step() and score(). Run with the yam_pair embodiment (CONTRACT.md in
yam-pair-kit):

    inspect-robots run --embodiment yam_pair -E task=task.py --policy yam_pair_scripted --instruction "$INSTR" \\
        --scorer pair_task_score --max-steps 6000
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import geometry as g  # noqa: E402
from yam_pair.task import Layout, Shape, UsdBody  # noqa: E402

LAYOUT = Layout("facing", separation=g.SEPARATION)
PHYSICS_HZ = 240
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
_TASK = json.loads((HERE / "task.json").read_text()) if (HERE / "task.json").exists() else {}
INSTRUCTIONS = dict(_TASK.get("instructions", {}))
DEBUG = bool(os.environ.get("CLP_DEBUG"))
VARIANT = os.environ.get("CLP_VARIANT", "")
TORQUE_LOG = os.environ.get("CLP_TORQUE_LOG", "")
SPIKE = float(os.environ.get("CLP_SPIKE", "0") or 0)   # debug: print substeps with a joint over this fraction of rated

TOP, CEIL = g.TOP, g.CEIL
WOOD, WOOD_DARK, FOAM, PLY = (0.74, 0.60, 0.42), (0.62, 0.48, 0.32), (0.18, 0.18, 0.20), (0.80, 0.68, 0.50)
HOUSING_START = (g.START_SHIFT, 0.0, CEIL - g.START_DROP + 0.0005)
if VARIANT == "drive_test":                       # development only: the light starts hung on its four screws
    HOUSING_START = (0.0, 0.0, CEIL - (g.GAP - g.SHEET_T) + 0.0003)
ARMS = tuple(f"{st}_{side}" for st in "ab" for side in ("left", "right"))
SCREW_NAMES = tuple(g.SCREWS)


def _holder_world(arm: str):
    st, side = arm.split("_")
    return LAYOUT.to_world(st, (*g.HOLDER_LOCAL[side], 0.0))[:2]


DRIVER_START = {arm: (*_holder_world(arm), TOP + g.HOLDER_FLOOR + 0.0005) for arm in ARMS}
SCREW_JOINTS = {f"{s}_{k}": (g.PZ0 if k == "pz" else 0.0) for s in SCREW_NAMES for k in ("pz", "spin")}


def _panel() -> list:
    """The ceiling panel's collider as boxes, open (10 mm square) around each screw's shank; two rails on its top."""
    rects = [(g.PANEL_X[0], g.PANEL_X[1], g.PANEL_Y[0], g.PANEL_Y[1])]
    for hx, hy in g.SCREWS.values():
        rects = g.subtract(rects, (hx - g.HOLE_OPEN, hx + g.HOLE_OPEN, hy - g.HOLE_OPEN, hy + g.HOLE_OPEN))
    out = []
    for i, (x0, x1, y0, y1) in enumerate(rects):
        out.append(Shape(f"panel_{i}", pos=((x0 + x1) / 2, (y0 + y1) / 2, CEIL + g.PANEL_T / 2),
                         size=(x1 - x0, y1 - y0, g.PANEL_T), color=PLY))
    for i, y in enumerate((g.PANEL_Y[1] - g.RAIL[0] / 2, g.PANEL_Y[0] + g.RAIL[0] / 2)):
        out.append(Shape(f"rail_{i}", pos=(0.0, y, CEIL + g.PANEL_T + g.RAIL[1] / 2),
                         size=(g.PANEL_X[1] - g.PANEL_X[0], g.RAIL[0], g.RAIL[1]), color=WOOD))
    return out


def _frame() -> list:
    out = []
    leg_top = CEIL + g.PANEL_T + g.RAIL[1]         # up past the panel's edge to the rail's top, screwed to both
    for i, (x, y) in enumerate(g.LEG_XY):
        z0 = TOP + g.LEG_FOOT[2]
        out.append(Shape(f"leg_{i}", pos=(x, y, (z0 + leg_top) / 2), size=(g.LEG[0], g.LEG[1], leg_top - z0), color=WOOD))
        out.append(Shape(f"leg_foot_{i}", pos=(x, y, TOP + g.LEG_FOOT[2] / 2), size=g.LEG_FOOT, color=WOOD_DARK))
    bar_top = CEIL - g.START_DROP - g.HOUSING_D - g.FOAM_T      # the housing's rims rest on the foam
    for i, x in enumerate(g.STAND_X):
        post_h = bar_top - g.CROSSBAR[2] - TOP
        out.append(Shape(f"stand_post_{i}", pos=(x, 0.0, TOP + post_h / 2), size=(g.STAND_POST[0], g.STAND_POST[1],
                                                                                 post_h), color=WOOD))
        out.append(Shape(f"stand_bar_{i}", pos=(x, 0.0, bar_top - g.CROSSBAR[2] / 2), size=g.CROSSBAR,
                         color=WOOD_DARK))
        out.append(Shape(f"stand_foam_{i}", pos=(x, 0.0, bar_top + g.FOAM_T / 2),
                         size=(g.CROSSBAR[0], g.CROSSBAR[1], g.FOAM_T), color=FOAM))
        out.append(Shape(f"stand_foot_{i}", pos=(x, 0.0, TOP + 0.006), size=(0.20, 0.20, 0.012), color=WOOD_DARK))
    for arm in ARMS:
        st, side = arm.split("_")
        x, y = _holder_world(arm)
        fl = TOP + g.HOLDER_FLOOR                      # the driver's bottom, on the peg
        rt = fl - g.PEG_H
        out.append(Shape(f"holder_{arm}_riser", pos=(x, y, (TOP + rt) / 2), size=(g.RISER, g.RISER, rt - TOP), color=WOOD))
        # the peg is turned square to the hand's approach, so the fingers pass its sides
        yaw = LAYOUT.yaw_to_world(st, math.radians(g.HOLDER_YAW[side]))
        out.append(Shape(f"holder_{arm}_peg", pos=(x, y, rt + g.PEG_H / 2), size=(g.PEG_W, g.PEG_W, g.PEG_H),
                         rot_xyzw=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), color=WOOD_DARK))
    return out


OBJECTS = [
    UsdBody("housing", usd="assets/housing.usda", pos=HOUSING_START, size=(g.HOUSING_L, g.HOUSING_W, g.HOUSING_D),
            mass=g.HOUSING_MASS, friction=(0.8, 0.7), solver_iters=32),
    *(UsdBody(f"driver_{arm}", usd="assets/driver.usda", pos=DRIVER_START[arm], size=(0.045, 0.045, g.DRV_TIP),
              mass=g.DRV_MASS, friction=(0.8, 0.7), solver_iters=32, torsional_patch=0.01) for arm in ARMS),
    UsdBody("screws", usd="assets/screws.usda", pos=(0.0, 0.0, 0.0), size=(0.01, 0.01, 0.02), articulation=True,
            joint_pos=tuple(SCREW_JOINTS.items()),
            drives=((".*_pz", g.THREAD_K, g.THREAD_C, 3000.0), (".*_spin", 0.0, 0.05, g.CLUTCH)),
            friction=g.HEAD_FRICTION, friction_min=True, solver_iters=32),
    *_panel(),
    *_frame(),
]

# -- live state ------------------------------------------------------------------------------------------------------
_S: dict = {}       # scoring and monitor state (control rate)
_SUB: dict = {}     # substep state
_V: dict = {}       # PhysX views


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), np.float64)
    return np.asarray(x, np.float64)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _ids(asset, names):
    return asset.find_joints(list(names), preserve_order=True)[0]


def _write(env, fn: str, s: str, values: dict) -> None:
    """Write per-joint values to the screws articulation (joint names '<screw>_<key>')."""
    import torch

    asset = env.unwrapped.scene["screws"]
    names = [f"{s}_{k}" for k in values]
    ids = _ids(asset, names)
    t = torch.tensor([list(values.values())], dtype=torch.float32, device=asset.device)
    {"pos_target": lambda: asset.actuators.target_command.set_position_index(value=t, joint_ids=ids),
     "vel_target": lambda: asset.actuators.target_command.set_velocity_index(value=t, joint_ids=ids)}[fn]()


# -- the screw model, applied before every physics substep ------------------------------------------------------------
def _install(env) -> None:
    base = env.unwrapped
    scene = base.scene
    art = scene["screws"]
    robots = {arm: scene[f"{arm}_robot"] for arm in ARMS}
    _SUB.update(art=art, drivers={arm: scene[f"driver_{arm}"] for arm in ARMS}, robots=robots,
                jid={s: {k: int(_ids(art, [f"{s}_{k}"])[0]) for k in ("pz", "spin")} for s in SCREW_NAMES},
                origin=_np(scene.env_origins[0]), dt=float(base.physics_dt))
    if getattr(scene, "_clp_hook", False):
        return
    write = scene.write_data_to_sim

    def write_and_model(*a, **k):
        try:
            _substep(env)
        except Exception as e:                       # never take the simulation down
            _SUB["err"] = repr(e)
            if DEBUG:
                raise
        return write(*a, **k)

    scene.write_data_to_sim = write_and_model
    scene._clp_hook = True


def _driver_pose(arm: str):
    d = _SUB["drivers"][arm]
    pose = _np(d.data.root_link_pose_w)[0]
    vel = _np(d.data.root_vel_w)[0]
    return pose[:3] - _SUB["origin"], _rot(pose[3:7]), vel[:3], vel[3:6]


def _head(q, s: str) -> np.ndarray:
    """The bottom middle of screw s's head (where the bit goes in), world."""
    hx, hy = g.SCREWS[s]
    return np.array([hx, hy, CEIL + q[_SUB["jid"][s]["pz"]] - g.SCREW_L - g.HEAD_H])


def _substep(env) -> None:
    """Before every physics substep:
    - each driver's bit on the nearest screw head (task code; the bit has no collider): while the bit tip is within the
      head's radius and pressed into it, the head pushes back on the bit (a stiff spring, at most PUSH_MAX) and the
      recess's walls pull the bit toward the head's axis (a spring, at most the push);
    - the motor runs while the bit sits in the head (within 1.5 mm and 6 degrees) and pushes it PUSH_START or more; it
      turns that screw at RPM until the torque the screw needs (thread drag plus the clamp through the thread) reaches
      the clutch, which then slips; the torque acts back on the driver;
    - the thread: each screw's z joint follows its spin angle at PITCH per turn (a stiff drive).
    Also accumulates each arm's applied joint torques over the substeps."""
    S, art = _SUB, _SUB["art"]
    if "state" not in S:
        return
    q = _np(art.data.joint_pos)[0]
    omega = {s: 0.0 for s in SCREW_NAMES}
    for arm in ARMS:
        dv = S["drv"][arm]
        p_d, R_d, v_d, w_d = _driver_pose(arm)
        axis = R_d[:, 2]
        bit = p_d + axis * g.DRV_TIP
        dist = {s: float(np.linalg.norm((bit - _head(q, s))[:2])) for s in SCREW_NAMES}
        s = min(dist, key=dist.get)
        rel = bit - _head(q, s)
        lat, ax = dist[s], float(rel[2])
        tilt = float(math.degrees(math.acos(max(-1.0, min(1.0, axis[2])))))
        push, lat_f = 0.0, np.zeros(3)
        if lat <= g.HEAD_R and 0.0 < ax < 0.006:                      # the bit pressed into the head
            push = min(max(0.0, g.BIT_K * ax + g.BIT_D * float(v_d @ axis)), g.PUSH_MAX)
            # the recess's walls centre the bit: a lateral spring toward the head's axis, at most the push
            d_xy = np.array([rel[0], rel[1], 0.0])
            lat_f = -g.RECESS_K * d_xy
            n = float(np.linalg.norm(lat_f))
            if n > push:
                lat_f *= push / max(n, 1e-9)
        seated = lat <= 0.0015 and -0.0015 <= ax <= 0.004 and tilt <= 6.0
        on = seated and push >= g.PUSH_START
        st = S["state"][s]
        need = g.THREAD_DRAG + max(st["thread_f"], 0.0) * g.PITCH / (2 * math.pi * g.THREAD_EFF)
        clutch = S.get("clutch", g.CLUTCH)
        w_ax = float(w_d @ axis)
        if on and need < clutch:
            om, delivered, slipping = max(0.0, g.RPM * 2 * math.pi / 60 + w_ax), need, False
        elif on:
            om, delivered, slipping = 0.0, clutch, True
        else:
            om, delivered, slipping = 0.0, 0.0, False
        if on:
            omega[s] = max(omega[s], om)
            st["motor_steps"] += 1
            st["driver"] = arm
            st["slipping"] = slipping
        dv.update(screw=s, lat=lat, ax=ax, tilt=tilt, push=push, motor=on, delivered=delivered)
        dv["push_sum"] = dv.get("push_sum", 0.0) + push
        dv["push_n"] = dv.get("push_n", 0) + 1
        _driver_wrench(arm, push, delivered, axis, bit, lat_f)
    for s in SCREW_NAMES:
        st, j = S["state"][s], S["jid"][s]
        th = float(q[j["spin"]])
        st["theta"] += math.remainder(th - st["theta_prev"], 2 * math.pi)
        st["theta_prev"] = th
        target = g.PZ0 + g.PITCH * st["theta"] / (2 * math.pi)
        st["thread_f"] = g.THREAD_K * (target - float(q[j["pz"]]))
        _write(env, "vel_target", s, {"spin": omega[s]})
        _write(env, "pos_target", s, {"pz": min(target, g.TRAVEL + 0.004)})
    # each arm's joint torques (the drives' applied torque, from the last substep)
    for arm, robot in S["robots"].items():
        tau = getattr(robot.data, "applied_torque", None)
        if tau is None:
            continue
        t = np.abs(_np(tau)[0][:6])
        if SPIKE and np.any(t / np.asarray(g.RATED) > SPIKE):
            dq = _np(robot.data.joint_pos_target)[0][:6] - _np(robot.data.joint_pos)[0][:6]
            dv = _SUB["drv"][arm]
            print(f"[spike] step {_S.get('step')} {arm} tau {np.round(t, 2).tolist()} target-pos deg "
                  f"{np.round(np.degrees(dq), 2).tolist()} vel {np.round(_np(robot.data.joint_vel)[0][:6], 2).tolist()} "
                  f"drv push {dv['push']:.1f} lat {dv['lat'] * 1000:.1f} ax {dv['ax'] * 1000:.1f} mot {dv['motor']}",
                  flush=True)
        acc = S["tq"][arm]
        acc["sum"] += t
        acc["max"] = np.maximum(acc["max"], t)
        acc["n"] += 1


def _driver_wrench(arm: str, push: float, torque: float, axis, bit, lat_f=None) -> None:
    """The head pushing back on the bit (along the driver's axis, at its tip), the recess centring the bit (lat_f), and
    the motor's reaction torque."""
    import torch

    dv, d = _SUB["drv"][arm], _SUB["drivers"][arm]
    if push <= 0.0 and torque <= 0.0:
        if dv.get("wrench_on"):
            d.permanent_wrench_composer.reset()
            dv["wrench_on"] = False
        return
    lat_f = np.zeros(3) if lat_f is None else lat_f
    f = torch.tensor((-push * axis + lat_f).reshape(1, 1, 3), dtype=torch.float32, device=d.device)
    tq = torch.tensor((-torque * axis).reshape(1, 1, 3), dtype=torch.float32, device=d.device)
    pos = torch.tensor(np.asarray(bit + _SUB["origin"]).reshape(1, 1, 3), dtype=torch.float32, device=d.device)
    d.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=tq, positions=pos, body_ids=[0],
                                                             is_global=True)
    dv["wrench_on"] = True


# -- views and monitor --------------------------------------------------------------------------------------------------
LINKS = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
HAND = (6, 7, 8)                  # the gripper and its two fingers: the links that can hold the light


def _views(env) -> dict:
    if not _V:
        from yam_arena.robot import asset_link_paths

        psv = env.unwrapped.sim.physics_sim_view
        ns = "/World/envs/env_*"
        lp = asset_link_paths()
        arm_links = {arm: [f"{ns}/{arm[0].upper()}{arm[2:].title()}Arm{lp[k]}" for k in LINKS] for arm in ARMS}
        _V["arm_links"] = arm_links
        for arm, paths in arm_links.items():
            _V[f"housing_{arm}"] = psv.create_rigid_contact_view(f"{ns}/housing", filter_patterns=paths)
        fingers = [p for arm in ARMS for p in (arm_links[arm][i] for i in HAND)]
        all_links = [p for arm in ARMS for p in arm_links[arm]]
        drivers = [f"{ns}/driver_{arm}" for arm in ARMS]
        _V["housing_drivers"] = psv.create_rigid_contact_view(f"{ns}/housing", filter_patterns=drivers)
        for arm in ARMS:
            _V[f"held_{arm}"] = psv.create_rigid_contact_view(f"{ns}/driver_{arm}", filter_patterns=all_links)
        for s in SCREW_NAMES:
            _V[f"head_{s}"] = psv.create_rigid_contact_view(f"{ns}/screws/{s}_link_pz", filter_patterns=[f"{ns}/housing"])
        supports = [f"{ns}/{n}/geometry/mesh" for n in ("stand_foam_0", "stand_foam_1", "stand_bar_0", "stand_bar_1",
                                                         "stand_post_0", "stand_post_1", "table", "floor")]
        try:
            _V["supports"] = psv.create_rigid_contact_view(f"{ns}/housing", filter_patterns=supports)
            _V["supports_n"] = len(supports)
        except Exception as e:                       # static colliders not accepted as filters: geometric fallback
            _V["supports"] = None
            _V["supports_err"] = repr(e)
        _V["dt"] = float(env.unwrapped.physics_dt)
        _V["grip_body"] ={arm: int(_SUB["robots"][arm].find_bodies("gripper")[0][0]) for arm in ARMS}
        from yam_arena.constants import FINGER_JOINTS
        _V["fingers"] = {arm: list(_SUB["robots"][arm].find_joints(FINGER_JOINTS, preserve_order=True)[0]) for arm in ARMS}
    return _V


def _matrix(view, dt) -> np.ndarray:
    """Contact forces (filters, 3) on the view's one sensor body."""
    m = _np(view.get_contact_force_matrix(dt))
    return m.reshape(-1, 3) if m.size else np.zeros((0, 3))


def _clamp(view, dt) -> float:
    """The housing's push down on the screw's head (the head pressing the housing up), N."""
    m = _matrix(view, dt)
    return float(max(0.0, -m.sum(axis=0)[2])) if m.size else 0.0


def _reset_state(env=None) -> None:
    _S.clear()
    _S.update(step=0, dropped=False, drop_step=None, drop_end=None, events=[], last_touch_step=0, touched=False,
              peak={}, tier=0.0, ends={e: dict(free=False, z_ref=0.0, fall=0.0, recent=[]) for e in "ab"},
              grip_hist={arm: [] for arm in ARMS},
              tq_series={arm: [] for arm in ARMS}, tq_max_series={arm: [] for arm in ARMS})
    _SUB["state"] = {s: dict(theta=0.0, theta_prev=0.0, thread_f=0.0, slipping=False, motor_steps=0, driver=None,
                             hung=False, tight=False, flush=False, seated=False, clamp=0.0, gap=1.0, edge_gap=1.0,
                             along=0.0, across=0.0, turns=0.0, tight_logged=False)
                     for s in SCREW_NAMES}
    _SUB["drv"] = {arm: dict(screw=None, lat=1.0, ax=0.0, tilt=0.0, push=0.0, motor=False, delivered=0.0)
                   for arm in ARMS}
    _SUB["tq"] = {arm: dict(sum=np.zeros(6), max=np.zeros(6), n=0) for arm in ARMS}
    _S["state"] = _SUB["state"]
    _SUB["clutch"] = float(os.environ.get("CLP_CLUTCH", g.CLUTCH))


def on_build(env) -> None:
    """Visual only: the four plywood plugs with the screws' pilot holes, and the pencil outline, on the panel's
    underside."""
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    stage = omni.usd.get_context().get_stage()

    def mat(name, color, rough=0.7):
        path = f"/World/CeilingDecor/Looks/{name}"
        m = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/s")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return m

    plug, pencil = mat("plug", (0.86, 0.74, 0.55)), mat("pencil", (0.15, 0.15, 0.17))
    root = "/World/envs/env_0/CeilingDecor"
    for s, (x, y) in g.SCREWS.items():
        c = UsdGeom.Cylinder.Define(stage, f"{root}/plug_{s}")
        c.CreateRadiusAttr(g.PLUG_R)
        c.CreateHeightAttr(0.0004)
        c.AddTranslateOp().Set(Gf.Vec3d(x, y, CEIL - 0.0004))
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(plug)
    x0, x1 = -g.HOUSING_L / 2, g.HOUSING_L / 2
    y0, y1 = -g.HOUSING_W / 2, g.HOUSING_W / 2
    for k, (cx, cy, sx, sy) in enumerate(((0.0, y0, x1 - x0 + 0.03, 0.0012), (0.0, y1, x1 - x0 + 0.03, 0.0012),
                                          (x0, 0.0, 0.0012, y1 - y0 + 0.03), (x1, 0.0, 0.0012, y1 - y0 + 0.03))):
        c = UsdGeom.Cube.Define(stage, f"{root}/line_{k}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(cx, cy, CEIL - 0.0003))
        c.AddScaleOp().Set(Gf.Vec3f(sx, sy, 0.0003))
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(pencil)


VIEWS = {"side": ((0.05, -1.55, TOP + 0.50), (0.0, 0.0, TOP + 0.62)),
         "below": ((-0.36, -0.16, TOP + 0.26), (-0.41, 0.0, CEIL - 0.03)),   # setup.png: up into A's end of the housing
         "side_low": ((0.0, -1.45, TOP + 0.30), (0.0, 0.0, TOP + 0.66)),
         "a_end": ((-0.30, -0.95, TOP + 0.45), (-0.45, 0.0, TOP + 0.66)),
         "b_end": ((0.30, -0.95, TOP + 0.45), (0.45, 0.0, TOP + 0.66)),
         "a_key": ((-0.25, -0.40, TOP + 0.55), (-0.40, 0.0, TOP + 0.76)),
         "under": ((-0.15, -0.55, TOP + 0.25), (-0.35, 0.0, TOP + 0.74))}


def place_overview(env, view: str | None = None) -> None:
    """Put yam_pair's overview camera (not seen by either robot) where the ceiling does not hide the work."""
    import torch

    scene = env.unwrapped.scene
    if "overview_cam" not in scene.keys():
        return
    eye, target = VIEWS[view or os.environ.get("CLP_VIEW", "side")]
    origin = scene.env_origins[0:1]
    cam = scene["overview_cam"]
    cam.set_world_poses_from_view(torch.tensor([eye], dtype=torch.float32, device=origin.device) + origin,
                                  torch.tensor([target], dtype=torch.float32, device=origin.device) + origin)


def before_reset(env) -> None:
    _install(env)
    _reset_state(env)
    for arm in ARMS:
        _SUB["drivers"][arm].permanent_wrench_composer.reset()


def on_reset(env) -> None:
    _install(env)
    place_overview(env)
    _reset_state(env)
    _V.clear()


def _housing(env):
    h = env.unwrapped.scene["housing"]
    pose = _np(h.data.root_link_pose_w)[0]
    vel = _np(h.data.root_link_vel_w)[0]           # of the link frame, like the pose
    return pose[:3] - _SUB["origin"], _rot(pose[3:7]), vel[:3], vel[3:6]


def _screw_relation(p_h, R_h, q, s: str) -> dict:
    """Where screw s's shank lies in its keyhole (housing frame) and how its head sits under the back."""
    k = g.SCREW_KEY[s]
    kx, ky = g.KEYS[k]
    hx, hy = g.SCREWS[s]
    pz = float(q[_SUB["jid"][s]["pz"]])
    head_top = CEIL + pz - g.SCREW_L
    loc = R_h.T @ (np.array([hx, hy, head_top]) - p_h)          # the head's top middle, housing frame
    along, across = float(loc[0] - kx), float(loc[1] - ky)
    # the shank in a narrow slot, clear of the round centre hole, and the back above the head: the head holds it
    in_slot = abs(across) <= g.KEY_SLOT_W / 2 + 0.0005 and g.KEY_HOLE_D / 2 + g.SHANK_R <= abs(along) <= g.KEY_SLOT_HALF
    gap = float(-g.SHEET_T - loc[2])                              # back's underside above the head's top (+ = gap)
    return dict(along=along, across=across, gap=gap, hung=bool(in_slot and gap > -0.002), head_top=head_top, loc=loc)


def _room() -> None:
    """The room's walls, a stand-in: a screwdriver knocked more than ROOM_HALF from the middle, or through the floor, is
    stopped there, lying on the floor. Without it a driver flung off the table can roll off the edge of the (200 m)
    floor and fall without end, and the growing scene bounds wreck the cameras' ray-traced shadows (black rings)."""
    import torch
    from yam_arena.station import FLOOR_Z

    S = _S
    for arm in ARMS:
        if VARIANT == "far_driver" and arm == "b_right":          # development: test the renderer without the room
            if S["step"] == 20:
                _place_driver(arm, (0.0, 0.0, -5000.0), torch)
            continue
        if VARIANT == "fling_driver" and arm == "b_right" and S["step"] == 20:      # development: test the room
            d = _SUB["drivers"][arm]
            d.write_root_velocity_to_sim_index(root_velocity=torch.tensor([[15.0, 3.0, 4.0, 0.0, 0.0, 0.0]],
                                                                          dtype=torch.float32, device=d.device))
        p = _driver_pose(arm)[0]
        if abs(p[0]) <= g.ROOM_HALF and abs(p[1]) <= g.ROOM_HALF and p[2] >= FLOOR_Z - 0.05:
            continue
        x, y = (float(np.clip(v, 0.05 - g.ROOM_HALF, g.ROOM_HALF - 0.05)) for v in p[:2])
        _place_driver(arm, (x, y, FLOOR_Z + 0.02), torch)
        S["events"].append((S["step"], f"driver_{arm} reached the room's edge ({g.ROOM_HALF:.1f} m out) and was "
                                       f"stopped there on the floor"))


def _place_driver(arm, pos, torch) -> None:
    """Put a driver at pos (task frame) lying on its side, at rest."""
    d = _SUB["drivers"][arm]
    o = np.asarray(_SUB["origin"], float)
    pose = [pos[0] + o[0], pos[1] + o[1], pos[2] + o[2], math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    d.write_root_pose_to_sim_index(root_pose=torch.tensor([pose], dtype=torch.float32, device=d.device))
    d.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=d.device))


def on_step(env) -> None:
    S, V = _S, _views(env)
    S["step"] += 1
    dt = V["dt"]
    _room()
    art = _SUB["art"]
    q = _np(art.data.joint_pos)[0]
    p_h, R_h, v_h, w_h = _housing(env)
    L, W, D = g.HOUSING_L, g.HOUSING_W, g.HOUSING_D
    # -- who touches the housing: every link of every arm, and the screwdrivers --------------------------------------
    hand_f, body_f, hold_end, slip = {}, {}, {"a": False, "b": False}, {"a": None, "b": None}
    for arm in ARMS:
        mat = _matrix(V[f"housing_{arm}"], dt)    # forces on the housing from each link
        m = np.linalg.norm(mat, axis=1)
        hand = float(m[list(HAND)].sum()) if m.size else 0.0
        body = float(m[:HAND[0]].sum()) if m.size else 0.0
        hand_f[arm], body_f[arm] = hand, body
        if DEBUG and body > g.TOUCH_F and S["step"] % 20 == 0:     # which links other than the hand touch it
            print(f"[body] step {S['step']} {arm} " + " ".join(f"{n} {f:.1f}" for n, f in zip(LINKS, m) if f > 0.0),
                  flush=True)
        S["peak"][f"hand_{arm}"] = max(S["peak"].get(f"hand_{arm}", 0.0), hand)
        # this hand holds the end it is at while it touches the light and the light there moves with it: over the last
        # HOLD_S, the gripper's place in the light's frame moved less than HOLD_SLIP (fingers it slides out of do not)
        gp = _np(_SUB["robots"][arm].data.body_link_pos_w)[0, V["grip_body"][arm]] - _SUB["origin"]
        r_loc = R_h.T @ (gp - p_h)
        hist = S["grip_hist"][arm] = (S["grip_hist"][arm] + [(hand > g.TOUCH_F, r_loc)])[-(round(g.HOLD_S * 20) + 1):]
        if hand > g.TOUCH_F:
            from yam_arena.constants import FINGER_OPEN
            e = "a" if r_loc[0] < 0 else "b"
            moved = float(np.linalg.norm(r_loc - hist[0][1]))
            slip[e] = moved if slip[e] is None else min(slip[e], moved)
            opening = float(np.mean(_np(_SUB["robots"][arm].data.joint_pos)[0][V["fingers"][arm]]) / FINGER_OPEN)
            if (opening <= g.HOLD_OPEN and len(hist) > round(g.HOLD_S * 20) and all(c for c, _ in hist)
                    and moved < g.HOLD_SLIP):
                hold_end[e] = True
    dm = np.linalg.norm(_matrix(V["housing_drivers"], dt), axis=1)
    drv_f = {arm: float(dm[i]) if dm.size else 0.0 for i, arm in enumerate(ARMS)}
    held_drv = {arm: float(np.linalg.norm(_matrix(V[f"held_{arm}"], dt), axis=1).sum()) > g.TOUCH_F for arm in ARMS}
    touch = (any(hand_f[a] > g.TOUCH_F or body_f[a] > g.TOUCH_F for a in ARMS)
             or any(drv_f[a] > g.TOUCH_F and held_drv[a] for a in ARMS))
    S.update(hand_f=hand_f, body_f=body_f, drv_f=drv_f, touched=touch)
    if touch:
        S["last_touch_step"] = S["step"]
    # -- screws: the keyhole relation, the clamp, tight ------------------------------------------------------------
    for s in SCREW_NAMES:
        st = S["state"][s]
        rel = _screw_relation(p_h, R_h, q, s)
        st["clamp"] = _clamp(V[f"head_{s}"], dt)
        st.update(along=rel["along"], across=rel["across"], gap=rel["gap"], hung=rel["hung"])
        st["turns"] = st["theta"] / (2 * math.pi)
        st["seated"] = bool(rel["hung"] and rel["gap"] <= g.SEAT_GAP and st["clamp"] >= g.SEAT_F)
        # flush: the back's nearest long edge, in line with the screw, within 2 mm of the ceiling
        edge = p_h + R_h @ np.array([rel["loc"][0], math.copysign(W / 2, rel["loc"][1]), 0.0])
        st["edge_gap"] = float(CEIL - edge[2])
        st["flush"] = bool(st["edge_gap"] <= g.FLUSH_GAP)
        st["tight"] = bool(st["seated"] and st["flush"])
        if st["tight"] and not st["tight_logged"]:
            st["tight_logged"] = True
            S["events"].append((S["step"], f"{s} tight: clamp {st['clamp']:.0f} N, head gap {st['gap'] * 1000:.2f} mm, "
                                            f"edge gap {st['edge_gap'] * 1000:.2f} mm, {st['turns']:.1f} turns"))
    for arm in ARMS:
        dv = _SUB["drv"][arm]
        dv["push_avg"] = dv.get("push_sum", 0.0) / max(1, dv.get("push_n", 0))
        dv["push_sum"], dv["push_n"] = 0.0, 0
        S["peak"][f"push_{arm}"] = max(S["peak"].get(f"push_{arm}", 0.0), dv["push_avg"])
    # -- dropped: an end comes down FALL or more onto a stand, the table, the floor or a robot, while no gripper holds
    # that end and it is not on its screws ---------------------------------------------------------------------------
    support = 0.0
    if V.get("supports") is not None:
        support = float(np.linalg.norm(_matrix(V["supports"], dt), axis=1).sum())
    else:                                          # geometric: a rim corner at a crossbar's foam or below it
        foam = CEIL - g.START_DROP - g.HOUSING_D
        rims = [p_h + R_h @ np.array([sx * L / 2, sy * W / 2, -D]) for sx in (-1, 1) for sy in (-1, 1)]
        support = 1.0 if min(r[2] for r in rims) <= foam + 0.001 else 0.0
    on_robot = sum(body_f.values()) + sum(drv_f[a] for a in ARMS if not held_drv[a])
    S["support_f"] = support
    for e, sx in (("a", -1.0), ("b", 1.0)):
        en = S["ends"][e]
        z_end = min(float((p_h + R_h @ np.array([sx * L / 2, sy * W / 2, -D]))[2]) for sy in (-1, 1))
        on_screws = any(S["state"][s]["hung"] for s in g.STATION_SCREWS[e])
        held = hold_end[e]
        free = not held and not on_screws
        en["recent"] = (en["recent"] + [z_end])[-(round(g.FALL_LOOKBACK * 20) + 1):]
        if free and not en["free"]:                # from its highest point just before it was let go
            en["z_ref"] = max(en["recent"])
        if free:
            en["z_ref"] = max(en["z_ref"], z_end)
            en["fall"] = en["z_ref"] - z_end
        else:
            en["fall"] = 0.0
        en.update(free=free, held=held, slip=slip[e], on_screws=on_screws, z=z_end)
        if (free and en["fall"] >= g.FALL and (support > g.TOUCH_F or on_robot > g.TOUCH_F) and not S["dropped"]):
            S["dropped"], S["drop_step"], S["drop_end"] = True, S["step"], e
            S["events"].append((S["step"], f"dropped: {e.upper()}'s end came down {en['fall'] * 1000:.0f} mm onto "
                                            f"{'a stand, the table or the floor' if support > g.TOUCH_F else 'a robot'}, "
                                            f"unheld and not on its screws"))
    # -- the score ------------------------------------------------------------------------------------------------------
    own = (S["step"] - S["last_touch_step"]) >= round(g.HANG_S * 20) and not touch
    hung = {s: S["state"][s]["hung"] for s in SCREW_NAMES}
    tight = {s: S["state"][s]["tight"] for s in SCREW_NAMES}
    n_hung = sum(hung.values())
    end_tight = {e: all(tight[s] for s in g.STATION_SCREWS[e]) for e in "ab"}
    if S["dropped"] or n_hung == 0:
        tier = 0.0
    elif n_hung == 4 and own:
        tier = 1.0 if all(tight.values()) else (0.75 if any(end_tight.values()) else 0.5)
    else:
        tier = 0.25
    was_own = S.get("own", False)
    S.update(tier=tier, own=own, n_hung=n_hung)
    if n_hung == 4 and not S.get("all_hung_logged"):
        S["all_hung_logged"] = True
        S["events"].append((S["step"], "hung on all four screws (each shank in a narrow slot, the back above its head)"))
    if own and not was_own and n_hung:
        S["events"].append((S["step"], f"no robot touching it for {g.HANG_S:.0f} s; tier {tier}"))
    _torques()
    _debug(env, p_h, q)
    lo_hi = os.environ.get("CLP_TRACE_STEPS")
    if DEBUG and lo_hi and int(lo_hi.split("-")[0]) <= S["step"] <= int(lo_hi.split("-")[1]):
        print(f"[trace] step {S['step']} hx {p_h[0] * 1000:+.2f} hy {p_h[1] * 1000:+.2f} hz {(p_h[2] - CEIL) * 1000:+.2f} "
              f"vx {v_h[0] * 1000:+.1f} " + " ".join(f"{s}: cl {S['state'][s]['clamp']:.0f} gap {S['state'][s]['gap'] * 1000:+.1f} "
                                                   f"t {S['state'][s]['turns']:.2f}" for s in SCREW_NAMES)
              + " | " + " ".join(f"{a}: h {hand_f[a]:.1f} b {body_f[a]:.1f} d {drv_f[a]:.1f} p {_SUB['drv'][a]['push']:.1f} "
                                 f"m {int(_SUB['drv'][a]['motor'])}" for a in ARMS) + f" sup {support:.1f}", flush=True)


def _torques() -> None:
    """Per arm and joint: this control step's mean and largest |applied torque| over its substeps."""
    for arm in ARMS:
        acc = _SUB["tq"][arm]
        if acc["n"] == 0:
            continue
        _S["tq_series"][arm].append(acc["sum"] / acc["n"])
        _S["tq_max_series"][arm].append(acc["max"].copy())
        acc.update(sum=np.zeros(6), max=np.zeros(6), n=0)
    if TORQUE_LOG and _S["step"] % 100 == 0:
        _save_torques()


def _save_torques() -> None:
    np.savez(TORQUE_LOG, **{f"{arm}_mean": np.array(_S["tq_series"][arm]) for arm in ARMS},
             **{f"{arm}_max": np.array(_S["tq_max_series"][arm]) for arm in ARMS})


def torque_summary() -> dict:
    """Per arm and joint: peak |torque| (any substep), and the highest 1 s and 10 s moving averages of |torque|."""
    out = {}
    for arm in ARMS:
        mean = np.array(_S.get("tq_series", {}).get(arm, []))
        mx = np.array(_S.get("tq_max_series", {}).get(arm, []))
        if not len(mean):
            continue
        out[arm] = {"peak": mx.max(axis=0).tolist()}
        for name, n in (("avg1", 20), ("avg10", 200)):
            if len(mean) >= n:
                c = np.cumsum(np.vstack([np.zeros(6), mean]), axis=0)
                out[arm][name] = ((c[n:] - c[:-n]) / n).max(axis=0).tolist()
            else:
                out[arm][name] = mean.mean(axis=0).tolist()
    return out


def _debug(env, p_h, q) -> None:
    S = _S
    if not DEBUG:
        return
    for step, text in S["events"][S.get("printed", 0):]:
        print(f"[clp] event step {step}: {text}", flush=True)
    S["printed"] = len(S["events"])
    if S["step"] % 100 == 0:
        print("[rep] step", S["step"], json.dumps({k: (round(v, 4) if isinstance(v, float) else v)
                                                   for k, v in report(None, None).items()}), flush=True)
    if S["step"] % 10 == 0:
        print(f"[clp] step {S['step']} tier {S['tier']} drop {S['dropped']} touch {S['touched']} "
              f"hx {p_h[0]:+.4f} hy {p_h[1]:+.4f} hz {p_h[2] - CEIL:+.4f} "
              + " ".join(f"{s}: hung {st['hung']} along {st['along'] * 1000:+.1f} across {st['across'] * 1000:+.1f} "
                         f"gap {st['gap'] * 1000:+.2f} clamp {st['clamp']:.0f} edge {st['edge_gap'] * 1000:+.2f} "
                         f"turns {st['turns']:.2f} tight {st['tight']}" for s, st in S["state"].items())
              + " | " + " ".join(f"{arm}: hand {S['hand_f'][arm]:.1f} body {S['body_f'][arm]:.1f} "
                                 f"drv {S['drv_f'][arm]:.1f} push {_SUB['drv'][arm]['push']:.1f} "
                                 f"mot {_SUB['drv'][arm]['motor']}" for arm in ARMS)
              + " | " + " ".join(f"{e}: held {en.get('held')} slip "
                                 f"{'-' if en.get('slip') is None else format(en['slip'] * 1000, '.1f')} screws {en.get('on_screws')} "
                                 f"fall {en['fall'] * 1000:.1f} "
                                 f"z {(en.get('z', CEIL) - CEIL) * 1000:+.1f}" for e, en in S["ends"].items())
              + f" support {S['support_f']:.1f} err {_SUB.get('err')} {_V.get('supports_err', '')}"
              + " | drivers " + " ".join(f"{a} " + ",".join(f"{v:+.2f}" for v in _driver_pose(a)[0]) for a in ARMS),
              flush=True)


def report(objects, pair) -> dict:
    S = _S
    out = {"tier": S.get("tier", 0.0), "dropped": float(S.get("dropped", False)), "own": float(S.get("own", False)),
           "n_hung": float(S.get("n_hung", 0))}
    for s, st in S.get("state", {}).items():
        for k in ("hung", "seated", "flush", "tight"):
            out[f"{s}_{k}"] = float(st[k])
        for k in ("along", "across", "gap", "edge_gap", "clamp", "turns"):
            out[f"{s}_{k}"] = float(st[k])
    for k, v in S.get("peak", {}).items():
        out[f"peak_{k}"] = float(v)
    for arm, d in torque_summary().items():
        for name, vals in d.items():
            for i, v in enumerate(vals):
                out[f"tq_{arm}_{name}_j{i + 1}"] = float(v)
    out["cross_contact_peak_n"] = float(getattr(pair, "contact_peak_n", 0.0))
    if TORQUE_LOG and _S.get("tq_series"):
        _save_torques()                      # the whole run, up to its last step
    return out


def success(objects, pair) -> bool:
    """Never: every trial runs to its step limit and is scored on its end state."""
    return False


def score(objects, pair) -> float:
    """1.0: it hangs on all four screws (each shank in a narrow slot of its keyhole, the back above the head), all four
    are tight (head within 0.3 mm of the back, clamping it with 50 N or more, and the back's nearest long edge within
    2 mm of the ceiling), and no robot (any link, or a screwdriver in a hand) has touched it for the last 3 s. 0.75: the
    same, but only one end's two screws are tight. 0.5: hangs on all four on its own, no end tight. 0.25: not dropped
    and hung on at least one screw, but less. 0: dropped at any time (an end came down 10 mm or more onto a stand, the
    table, the floor or a robot while no gripper held that end and it was not on its screws), or hung on no screw."""
    return float(_S.get("tier", 0.0))


def demo(layout):
    from scripted import station_script

    return {"a": station_script("a", VARIANT), "b": station_script("b", VARIANT)}
