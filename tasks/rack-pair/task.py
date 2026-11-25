"""Two bimanual YAM robots face each other across one bench, with a small 9U open-frame rack at the bench's far end.
A 1U rackmount server (an iStarUSA D-118V2-ITX chassis, ballasted to 6.4 kg) lies flat on a foam stand between the
robots, its rack ears toward the open end of the bench. The chassis members of an iStarUSA TC-RAIL-20 sliding rail kit
are screwed to its sides and run past its rear toward the rack; the kit's two rack rails are fixed in the rack at U3.
Each robot is told: "Rack this server with the robot across from you: lift it off the stand together, slide both of
its rails into the rack's rails and push it in until its rack ears meet the posts, then let go."

score() (judged on the end state, after any drop): 1.0 home and clear; 0.75 both sides engaged but not home or not
clear; 0.5 one side engaged; 0.25 lifted together, no side engaged; 0 never lifted together, or dropped. See
README.md for the definitions; report() gives every number behind them, each arm's load and its joint torque.

Run:   inspect-robots run --embodiment yam_pair -E task=task.py --policy yam_pair_scripted --instruction "$INSTR" \\
           --scorer pair_task_score --max-steps 1200
"""

from __future__ import annotations

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from yam_pair.task import Box, Layout, UsdBody  # noqa: E402

TEXT = ("Rack this server with the robot across from you: lift it off the stand together, slide both of its rails "
        "into the rack's rails and push it in until its rack ears meet the posts, then let go.")
INSTRUCTIONS = {"a": TEXT, "b": TEXT}
LAYOUT = Layout("facing", separation=2 * g.X_STATION)
TABLE = {"size": (1.50, 2.00), "center": (0.0, 0.25)}
PHYSICS_HZ = 240
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


OBJECTS = [
    Box("mat", pos=(*g.MAT["center"], g.TABLE_TOP + g.MAT["size"][2] / 2), size=g.MAT["size"], color=(0.30, 0.30, 0.32)),
    Box("stand", pos=(*g.STAND["center"], g.TABLE_TOP + g.MAT["size"][2] + g.STAND["size"][2] / 2),
        size=g.STAND["size"], color=(0.58, 0.58, 0.60)),
    UsdBody("rack", usd="assets/rack.usda", pos=(0.0, 0.0, 0.0), size=(0.51, 0.70, 0.56),
            friction=(g.RAIL_FRICTION, g.RAIL_FRICTION), friction_min=True),
    UsdBody("server", usd="assets/server.usda", pos=g.SERVER_START, size=(0.48, 0.48, 0.044), mass=g.MASS,
            friction=(0.8, 0.7), solver_iters=32),
]

# -- the server's colliders, in its own frame (centre, size) ------------------------------------------------------------
_HW = g.BODY_W / 2
BOXES = {
    "body": ((0.0, (g.PANEL_Y[0] + g.BODY_Y[1]) / 2, g.BODY_H / 2), (g.BODY_W, g.BODY_Y[1] - g.PANEL_Y[0], g.BODY_H)),
    "ear_a": ((-(_HW + g.EAR_W / 2), -g.EAR_T / 2, g.EAR_H / 2), (g.EAR_W, g.EAR_T, g.EAR_H)),
    "ear_b": ((_HW + g.EAR_W / 2, -g.EAR_T / 2, g.EAR_H / 2), (g.EAR_W, g.EAR_T, g.EAR_H)),
    "member_a": ((-sum(g.MEMBER_X) / 2, sum(g.MEMBER_Y) / 2, g.MEMBER_ZC), (g.MEMBER_T, g.MEMBER_Y[1] - g.MEMBER_Y[0], g.MEMBER_H)),
    "member_b": ((sum(g.MEMBER_X) / 2, sum(g.MEMBER_Y) / 2, g.MEMBER_ZC), (g.MEMBER_T, g.MEMBER_Y[1] - g.MEMBER_Y[0], g.MEMBER_H)),
}
ARMS = ("a_left", "a_right", "b_left", "b_right")
GRIP_LINKS = ("gripper", "tip_left", "tip_right")


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64)


# -- geometry of a server pose (pure NumPy; used by the scorer and by the demo) ------------------------------------------
def corners(p, R, which="bottom") -> np.ndarray:
    xs, ys = (-_HW, _HW), (g.PANEL_Y[0], g.BODY_Y[1])
    z = 0.0 if which == "bottom" else g.BODY_H
    return np.array([p + R @ np.array([x, y, z]) for x in xs for y in ys])


def member_section(p, R, side: int, y_world: float | None = None):
    """Centre of a chassis member's cross-section (world) at its tip, or where it crosses ``y_world``."""
    xm = side * sum(g.MEMBER_X) / 2
    base = p + R @ np.array([xm, 0.0, g.MEMBER_ZC])
    d = R[:, 1]
    if y_world is None:
        return base + d * g.MEMBER_Y[1]
    t = (y_world - base[1]) / d[1]
    return base + d * float(np.clip(t, g.MEMBER_Y[0], g.MEMBER_Y[1]))


def _in_pocket(pt, side: int) -> bool:
    xc = sum(g.MEMBER_X) / 2
    return (abs(side * pt[0] - xc) <= 0.003 and abs(pt[2] - g.RAIL_ZC) <= 0.0035
            and g.RAIL_Y[0] - 0.001 <= pt[1] <= g.RAIL_Y[1])


def _on_rail_top(pt, side: int) -> bool:
    z_on = g.RAIL_ZC + g.POCKET_H / 2 + g.LIP_T + g.MEMBER_H / 2
    return (g.LIP_IN - 0.004 <= side * pt[0] <= g.WEB_IN + g.WEB_T + 0.002 and abs(pt[2] - z_on) <= 0.004
            and g.RAIL_Y[0] <= pt[1] <= g.RAIL_Y[1])


def engagement(p, R) -> dict:
    """Per side (-1: robot A's, +1: robot B's): how far the member's tip is past the posts' front faces, and whether
    it runs inside its rack rail (checked at the tip and at the mouth) or rides on top of it."""
    out = {}
    mouth = g.RAIL_Y[0] + g.FLARE_L
    for side in (-1, 1):
        tip = member_section(p, R, side)
        e = float(tip[1] - g.Y_FRONT)
        inside = on_top = False
        if tip[1] > mouth:
            at_mouth = member_section(p, R, side, mouth)
            inside = _in_pocket(tip, side) and _in_pocket(at_mouth, side)
            on_top = _on_rail_top(tip, side) or _on_rail_top(at_mouth, side)
        out[side] = {"e": e, "inside": inside, "on_top": on_top, "engaged": inside and e >= g.ENGAGED_MIN}
    return out


def ear_gaps(p, R) -> list[float]:
    out = []
    for side in (-1, 1):
        back = p + R @ np.array([side * (_HW + g.EAR_W / 2), 0.0, g.EAR_H / 2])
        out.append(float(max(g.Y_FRONT - back[1], 0.0)))
    return out


def euler_deg(R) -> tuple[float, float, float]:
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))     # about y (the server's long axis): x tilts
    pitch = np.degrees(np.arcsin(np.clip(R[2, 0], -1, 1)))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return float(-pitch), float(roll), float(yaw)          # (side-to-side tilt, end-to-end tilt, yaw)


def on_stand(p, R) -> bool:
    sx, sy, _ = g.STAND["size"]
    cx, cy = g.STAND["center"]
    n = R[:, 2]
    for dx in (-sx / 2, sx / 2):
        for dy in (-sy / 2, sy / 2):
            q = np.array([cx + dx, cy + dy, g.STAND_TOP])
            local = R.T @ (q - p)
            if abs(local[0]) > _HW or not (g.PANEL_Y[0] <= local[1] <= g.BODY_Y[1]):
                continue
            if abs(local[2]) <= 0.003:          # the server's underside within 3 mm of the stand top here
                return True
    del n
    return False


# -- per-step state (built in on_step, read by score/success/report) ----------------------------------------------------
_S: dict = {}
_HULLS: dict = {}


def _reset_state() -> None:
    _S.clear()
    _S.update(step=0, lifted_together=False, dropped=False, drop_ref=None, untouched=0, max_lift=0.0,
              peak_kg={a: 0.0 for a in ARMS}, cont_peak={a: 0.0 for a in ARMS}, peak_peak={a: 0.0 for a in ARMS},
              over_cont_s={a: 0.0 for a in ARMS}, cur={})


_reset_state()


def _hull_distance(hulls, p, R) -> dict[str, float]:
    """Smallest signed distance from each link's hull vertices to the server's colliders (m)."""
    from yam_arena.analysis import box_signed_distance

    out = {}
    for name, h in hulls:
        local = (h.vertices - p) @ R          # into the server frame
        d = min(float(box_signed_distance(local, np.array(c), np.eye(3), np.array(s)).min()) for c, s in BOXES.values())
        out[name] = min(out.get(name, np.inf), d)
    return out


def _arm_load(env, arm: str):
    """(vertical load the arm holds up, kg; |torque| / continuous rating per joint; / peak). The PD part of the joint
    torque (yam_arena adds the links' own gravity as a feed-forward) is turned into the tool-point wrench."""
    import armload
    from yam_arena.constants import ARM_JOINTS, SHOULDER, WRIST

    robot = env.unwrapped.scene[f"{arm}_robot"]
    ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
    q = _np(robot.data.joint_pos)[0, ids]
    qd = _np(robot.data.joint_vel)[0, ids]
    qt = _np(robot.actuators.target_command.position)[0, ids]
    kp = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    kd = np.array([SHOULDER["damping"]] * 3 + [WRIST["damping"]] * 3)
    lim = _np(robot.data.joint_effort_limits)[0, ids]                # the drives' limits as set (see on_build)
    tau_pd = np.clip(kp * (qt - q) - kd * qd, -lim, lim)
    grav = _np(robot.data.gravity_compensation_forces)[0]
    if grav.shape[0] == robot.num_joints + 6:
        grav = grav[6:]
    tau = tau_pd + grav[ids]
    side = arm.split("_")[1]
    fz = armload.support_force(q, tau_pd, side)
    return fz / 9.81, np.abs(tau) / np.array(g.CONT), np.abs(tau) / np.array(g.PEAK)


def on_build(env) -> None:
    """Cap each arm's joint drives at the motors' continuous ratings (9 N m on joints 1-3, 3 N m on 4-6). yam_arena
    adds the links' own gravity as a separate feed-forward, so the cap applies to what the arm does above holding
    itself up: a payload or a push."""
    import torch
    from yam_arena.constants import ARM_JOINTS

    for arm in ARMS:
        robot = env.unwrapped.scene[f"{arm}_robot"]
        ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        lim = torch.tensor([list(g.CONT)], dtype=torch.float32, device=robot.device)
        robot.write_joint_effort_limit_to_sim_index(limits=lim, joint_ids=ids)


def on_reset(env) -> None:
    _reset_state()


def on_step(env) -> None:
    from yam_arena.analysis import world_hulls
    from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN
    from yam_arena.runtime import link_poses

    base = env.unwrapped
    origin = _np(base.scene.env_origins)[0]
    srv = base.scene["server"]
    pose = _np(srv.data.root_link_pose_w)[0]
    p, R = pose[:3] - origin, _rot(pose[3:7])
    _S["step"] += 1
    touch, hold = {}, {}
    for arm in ARMS:
        robot = base.scene[f"{arm}_robot"]
        poses = link_poses(robot)
        for m in poses.values():
            m[:3, 3] -= origin
        d = _hull_distance(world_hulls(poses, list(GRIP_LINKS)), p, R)
        fids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        opening = float(np.clip(_np(robot.data.joint_pos)[0, fids].mean() / FINGER_OPEN, 0, 1))
        touch[arm] = min(d.values()) <= g.TOUCH
        hold[arm] = opening < g.CLOSED_BELOW and d.get("tip_left", 1) <= g.HOLD_TOUCH and d.get("tip_right", 1) <= g.HOLD_TOUCH
        kg, cont, peak = _arm_load(env, arm)
        if touch[arm]:
            _S["peak_kg"][arm] = max(_S["peak_kg"][arm], kg)
        _S["cont_peak"][arm] = max(_S["cont_peak"][arm], float(cont.max()))
        _S["peak_peak"][arm] = max(_S["peak_peak"][arm], float(peak.max()))
        if cont.max() > 1.0:
            _S["over_cont_s"][arm] += base.step_dt
        _S["cur"][arm] = {"kg": kg, "cont": float(cont.max()), "touch": touch[arm], "hold": hold[arm],
                          "opening": opening}
    bottom = corners(p, R, "bottom")
    eng = engagement(p, R)
    stand = on_stand(p, R)
    rail = any(eng[s]["inside"] or eng[s]["on_top"] for s in (-1, 1))
    supported = any(hold.values()) or stand or rail          # a hand pushing it is not holding it up
    low = float(min(bottom[:, 2].min(), corners(p, R, "top")[:, 2].min()))
    _S["max_lift"] = max(_S["max_lift"], float(bottom[:, 2].min() - g.STAND_TOP))
    if supported:
        _S["drop_ref"] = None
    elif _S["drop_ref"] is None:
        _S["drop_ref"] = low
    if (_S["drop_ref"] is not None and low < _S["drop_ref"] - g.DROP_FALL) or low < g.LANDED_Z:
        _S["dropped"] = True
    both_hold = any(hold[a] for a in ARMS if a.startswith("a")) and any(hold[a] for a in ARMS if a.startswith("b"))
    if both_hold and bottom[:, 2].min() >= g.STAND_TOP + g.LIFT_CLEAR:
        _S["lifted_together"] = True
    _S["untouched"] = 0 if any(touch.values()) else _S["untouched"] + 1
    _S["eng"], _S["gaps"], _S["pose"] = eng, ear_gaps(p, R), (p, R)
    _S["supported"], _S["on_stand"], _S["on_rail"] = supported, stand, rail


def _score() -> float:
    if not _S.get("eng") or _S["dropped"] or not _S["lifted_together"]:
        return 0.0
    eng = _S["eng"]
    n = sum(eng[s]["engaged"] for s in (-1, 1))
    clear = _S["untouched"] >= round(g.CLEAR_S * 20)
    home = n == 2 and max(_S["gaps"]) <= g.HOME_GAP
    if home and clear:
        return 1.0
    if n == 2:
        return 0.75
    if n == 1:
        return 0.5
    return 0.25


def score(objects, pair) -> float:
    return _score()


def success(objects, pair) -> bool:
    return _score() >= 1.0


def report(objects, pair) -> dict:
    if not _S.get("eng"):
        return {}
    p, R = _S["pose"]
    tilt, pitch, yaw = euler_deg(R)
    out = {"score": _score(), "lifted_together": _S["lifted_together"], "dropped": _S["dropped"],
           "supported": _S["supported"], "max_lift_mm": 1000 * _S["max_lift"],
           "engaged_a": _S["eng"][-1]["engaged"], "engaged_b": _S["eng"][1]["engaged"],
           "tip_a_mm": 1000 * _S["eng"][-1]["e"], "tip_b_mm": 1000 * _S["eng"][1]["e"],
           "inside_a": _S["eng"][-1]["inside"], "inside_b": _S["eng"][1]["inside"],
           "on_top_a": _S["eng"][-1]["on_top"], "on_top_b": _S["eng"][1]["on_top"],
           "ear_gap_a_mm": 1000 * _S["gaps"][0], "ear_gap_b_mm": 1000 * _S["gaps"][1],
           "tilt_deg": tilt, "pitch_deg": pitch, "yaw_deg": yaw, "x_mm": 1000 * p[0], "y_mm": 1000 * p[1],
           "z_mm": 1000 * (p[2] - g.TABLE_TOP), "untouched_steps": _S["untouched"],
           "cross_contact_peak_n": float(pair.contact_peak_n)}
    for arm in ARMS:
        c = _S["cur"][arm]
        out[f"{arm}_kg"] = c["kg"]
        out[f"{arm}_touch"] = c["touch"]
        out[f"{arm}_hold"] = c["hold"]
        out[f"{arm}_peak_kg"] = _S["peak_kg"][arm]
        out[f"{arm}_tau_cont_peak"] = _S["cont_peak"][arm]
        out[f"{arm}_tau_peak_ratio"] = _S["peak_peak"][arm]
        out[f"{arm}_over_cont_s"] = _S["over_cont_s"][arm]
    return {k: (float(v) if isinstance(v, (bool, np.bool_, int, float, np.integer, np.floating)) else v)
            for k, v in out.items()}


def demo(layout):
    import racker

    return racker.controllers(layout, os.environ.get("RACKPAIR_VARIANT", ""))
