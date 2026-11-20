"""Scripted full-score demo (reads object poses from the simulator: the blocks' poses, the station's pose and its lid's
hinge angle).

1. Right arm, block A: a straight-down pinch across its 25 mm width; lifted over the station's wall and carried over
   the front-right rod; lined up on the block's measured position until its hole is over the rod's tip; lowered until
   the hole's lower end is 12 mm below the tip; let go (it slides down the rod) and the hand rises.
2. Right arm, block B: the same, onto the back-right rod.
3. Left arm, lid: fingers closed, tipped 45 degrees, the fingertips push the lid's outer face toward the robot's right
   until it is past vertical; it falls shut and rests on its catches.
4. Right arm, locks: the closed fingertips press the lid's free end down, deeper by steps, until the lid is held
   within 0.2 degrees of shut; the hand rises.
5. Both arms, push: the closed fingers' flat faces on the station's front face, 80 mm either side of its middle, push
   it toward the wall (closed loop on its measured back corners) until both are within 3 mm of the wall; the hands back
   off and go home.

    python demo.py task.py demo_out [--max-steps 3600] [--no-record] [--variant NAME] [--start-view] [--close-view]
                                    [--hold S]

Variants (scorer checks), each printing its mode:
- ``loose_wall``: both blocks laid on the bench at the droppings' right end, by the wall; nothing else;
- ``one_in``: block A threaded; block B left on its mark; the lid shut and locked, the station pushed to the wall;
- ``shut_unlocked``: both threaded; the lid shut (resting on its catches, not pressed); the station pushed;
- ``off_rod``: block A threaded; block B let go inside the station off the rods; locked; pushed;
- ``no_push``: the full demo without the push;
- ``off_rod_no_push``: ``off_rod`` without the push;
- ``both_off``: both blocks let go inside the station off the rods (between the rods on each side); locked; pushed;
- ``open``: both threaded, nothing else;
- ``hold_lid``: the full demo, but the right hand stays pressing the lid at the end;
- ``nothing``: the arms stay still.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, together, wait  # noqa: E402

TRACE = bool(os.environ.get("MB_TRACE"))
GAP_MM = 94.0                       # pads' gap at grip 1.0
TIP = 0.013                         # fingertips past the tool point
FT_HALF = 0.0064                    # half the closed fingertips' thickness along the closing axis
PICK_TIP = 0.003                    # the pinch: fingertips this far above the bench
CARRY = 0.118                       # a carried block's underside this far above the bench (the walls are 100 tall)
SAFE_Z = 0.150                      # moves between places: the tool point this far above the bench
HOVER = 0.006                       # lined up with its underside this far above the rod's tip
THREAD = 0.012                      # let go with the hole's lower end this far below the rod's tip
LID_SPAN = 0.120                    # the lid pushed this far from its hinge
LID_TILT = math.radians(45.0)
PRESS_Y = -0.110                    # the lock press: this far along the lid (station frame y), at its middle
PRESS_STEP = 0.0005                 # m per control step, down
PRESS_MAX = 0.090                   # commanded at most this far below the lid's top
PUSH_Y = 0.080                      # the push: hands this far either side of the station's middle ...
PUSH_TIP_Z = 0.055                  # ... fingertips this far above the bench (the gripper body clears the lid)
PUSH_SPEED = 0.0006                 # m per control step
PUSH_LEAD = 0.006                   # the hands led this far into the front face
WALL_DONE = 0.003                   # pushed until both back corners are this close to the wall


def grip_for(width_m: float) -> float:
    return max(0.0, min(1.0, width_m * 1000 / GAP_MM))


def R_from(a, c) -> np.ndarray:
    """Gripper rotation with approach ``a`` (its z) and closing axis ``c`` (its x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def down(yaw: float) -> np.ndarray:
    return R_from([0.0, 0.0, -1.0], [math.cos(yaw), math.sin(yaw), 0.0])


def rot_z(a: float) -> np.ndarray:
    return g.rot_z(a)


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


class Scene:
    """The measured scene."""

    def __init__(self, ctx):
        st = ctx.states()
        self.task = ctx.task
        s = st["bait_station"]
        self.p0 = np.asarray(s.pos, float)
        self.R0 = ctx.task._rotation(s.quat_xyzw)
        self.lid = float(s.joints["lid_hinge"])
        self.blocks = {n: (np.asarray(st[n].pos, float), ctx.task._rotation(st[n].quat_xyzw)) for n in ("block_a", "block_b")}

    def world(self, local) -> np.ndarray:
        """A point in the station frame, world."""
        return self.p0 + self.R0 @ np.asarray(local, float)

    def corners(self) -> list:
        return [float(g.WALL_X - self.world((g.HALF_D, s * g.HALF_L, 0.0))[0]) for s in (-1, 1)]


def settle_to(arm, pos, rot, tol=0.0006, rounds=3, seconds=0.6):
    """Move to ``pos`` and correct the command by the measured tool-point error."""
    target = np.asarray(pos, float)
    cmd = target.copy()
    yield from arm.move(cmd, rot, seconds)
    for _ in range(rounds):
        yield from wait(0.2)
        err = target - tcp_actual(arm)[0]
        if TRACE:
            from scenekit import arm_joints, env
            from yam_arena.constants import ARM_JOINTS
            print(f"SETTLE {arm.side} target {np.round(target, 4).tolist()} actual {np.round(tcp_actual(arm)[0], 4).tolist()} "
                  f"q_cmd {np.round(arm.q, 3).tolist()} q_meas {np.round(arm_joints(env(), arm.side, ARM_JOINTS), 3).tolist()} "
                  f"worst {arm.worst_mm:.1f}", flush=True)
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
        yield from arm.move(cmd, rot, 0.3)


def block_in_hand(ctx, arm, name):
    p, R = tcp_actual(arm)
    pb, Rb = Scene(ctx).blocks[name]
    return R.T @ (pb - p), R.T @ Rb


def hand_for_block(rel, pb, Rb):
    """The hand pose that puts the held block at (pb, Rb)."""
    t_in, R_in = rel
    Rh = Rb @ R_in.T
    return pb - Rh @ t_in, Rh


def block_yaw(Rb) -> float:
    return math.atan2(Rb[1, 0], Rb[0, 0])


def pinch_yaw(Rb) -> float:
    """The hand's yaw for a pinch across the block's width, the one near -90 degrees (reachable for the right arm)."""
    y = block_yaw(Rb) + math.pi / 2
    return math.remainder(y + math.pi / 2, math.pi) - math.pi / 2


# -- blocks ----------------------------------------------------------------------------------------------------------------
def rise(arm, z=None, seconds=0.6):
    """Straight up to a tool point SAFE_Z above the bench (from where it is measured to be)."""
    z = g.TOP + SAFE_Z if z is None else z
    p_act, _ = tcp_actual(arm)
    if p_act[2] < z - 0.002:
        arm.pos = p_act
        yield from arm.move(np.array([p_act[0], p_act[1], z]), arm.rot, seconds)


def pick(ctx, arm, name):
    sc = Scene(ctx)
    pb, Rb = sc.blocks[name]
    R = down(pinch_yaw(Rb))
    p = np.array([pb[0], pb[1], g.TOP + PICK_TIP + TIP])
    above = np.array([p[0], p[1], g.TOP + SAFE_Z])
    yield from rise(arm)
    yield from arm.move(above, R, 1.6, grip=grip_for(0.042))
    yield from settle_to(arm, p + [0, 0, 0.015], R, rounds=2, seconds=0.5)
    yield from settle_to(arm, p, R, rounds=2, seconds=0.5)
    yield from arm.set_grip(0.0, 0.5)
    yield from wait(0.25)
    yield from arm.move(arm.pos + [0, 0, CARRY + 0.002 - PICK_TIP], R, 1.0)


def carry_over(ctx, arm, name, goal_xy, yaw=None):
    """Carry the held block (level, underside CARRY up) over ``goal_xy``; returns the in-hand pose."""
    rel = block_in_hand(ctx, arm, name)
    ctx.flags.setdefault("in_hand_mm", {})[name] = np.round(rel[0] * 1000, 1).tolist()
    pb, Rb = Scene(ctx).blocks[name]
    yaw = block_yaw(Rb) if yaw is None else yaw
    goal = np.array([goal_xy[0], goal_xy[1], g.TOP + CARRY + g.BLK_T / 2])
    ph, Rh = hand_for_block(rel, goal, rot_z(yaw))
    yield from arm.move(ph, Rh, 1.6)
    return rel, yaw


def thread(ctx, arm, name, rod):
    """Pick ``name`` and thread it onto rod ``rod`` (index into geometry.RODS)."""
    yield from pick(ctx, arm, name)
    sc = Scene(ctx)
    rod_w = sc.world((g.RODS[rod][0], g.RODS[rod][1], 0.0))
    rel, yaw = yield from carry_over(ctx, arm, name, rod_w[:2])
    Rgoal = rot_z(yaw)
    tip_z = sc.world((0, 0, g.ROD_TIP))[2]
    rec = ctx.flags.setdefault("thread", {}).setdefault(name, {})
    for attempt in range(3):
        sc = Scene(ctx)
        rod_w = sc.world((g.RODS[rod][0], g.RODS[rod][1], 0.0))
        goal = np.array([rod_w[0], rod_w[1], tip_z + HOVER + g.BLK_T / 2])
        ph, Rh = hand_for_block(rel, goal, Rgoal)
        yield from arm.move(ph, Rh, 1.0)
        cmd = ph.copy()
        err = np.zeros(3)
        for r in range(10):                                  # settle the block's hole over the rod's tip
            yield from wait(0.15)
            pb, _ = Scene(ctx).blocks[name]
            err = goal - pb
            if np.linalg.norm(err[:2]) < 0.0004:
                break
            cmd = cmd + np.r_[err[:2], 0.0]
            yield from arm.move(cmd, Rh, 0.3)
        rec[f"hover_err_mm_{attempt}"] = np.round(err * 1000, 2).tolist()
        # straight down, its measured xy held over the rod in closed loop, until the hole's lower end is THREAD below
        z_end = tip_z - THREAD + g.BLK_T / 2
        stall, jam = 0, False
        last = None
        for k in range(400):
            pb, _ = Scene(ctx).blocks[name]
            if pb[2] <= z_end:
                break
            cmd[:2] += 0.3 * (goal[:2] - pb[:2])
            cmd[2] -= 0.0003
            arm.aim(cmd, Rh)
            yield
            if last is not None and last - pb[2] < 0.00005:
                stall += 1
            else:
                stall = 0
            last = pb[2]
            if stall > 30:
                jam = True
                break
        pb, _ = Scene(ctx).blocks[name]
        rec[f"down_mm_{attempt}"] = round((pb[2] - z_end) * 1000, 2)
        if not jam:
            break
        rec[f"jam_{attempt}"] = True
        p_act, _ = tcp_actual(arm)
        arm.pos = p_act
        yield from arm.move(p_act + [0, 0, 0.012], Rh, 0.5)
        rel = block_in_hand(ctx, arm, name)
    b = ctx.task._STATE["blocks"].get(name, {})
    rec["at_release"] = dict(off_mm=round(b.get("axis_off", 0) * 1000, 2), depth_mm=round(b.get("depth", 0) * 1000, 2),
                             on_rod=b.get("on_rod"))
    yield from arm.set_grip(grip_for(0.040), 0.4)
    yield from wait(0.5)
    yield from rise(arm, seconds=1.0)
    b = ctx.task._STATE["blocks"].get(name, {})
    rec["after"] = dict(off_mm=round(b.get("axis_off", 0) * 1000, 2), depth_mm=round(b.get("depth", 0) * 1000, 2),
                        on_rod=b.get("on_rod"), rod=b.get("rod"))


def drop_inside(ctx, arm, name, local_xy):
    """Pick ``name`` and let it go 10 mm over the station's floor at ``local_xy`` (station frame), off the rods."""
    yield from pick(ctx, arm, name)
    sc = Scene(ctx)
    w = sc.world((local_xy[0], local_xy[1], 0.0))
    rel, yaw = yield from carry_over(ctx, arm, name, w[:2])
    goal = np.array([w[0], w[1], sc.world((0, 0, g.FLOOR_T))[2] + 0.010 + g.BLK_T / 2])
    ph, Rh = hand_for_block(rel, goal, rot_z(yaw))
    yield from arm.move(ph, Rh, 1.5)
    yield from arm.set_grip(grip_for(0.040), 0.4)
    yield from wait(0.5)
    yield from rise(arm, seconds=1.0)


def lay_by_wall(ctx, arm, name, xy):
    """Pick ``name`` and lay it flat on the bench at ``xy``."""
    yield from pick(ctx, arm, name)
    rel, yaw = yield from carry_over(ctx, arm, name, xy)
    goal = np.array([xy[0], xy[1], g.TOP + g.BLK_T / 2 + 0.002])
    ph, Rh = hand_for_block(rel, goal, rot_z(yaw))
    yield from arm.move(ph, Rh, 1.5)
    yield from arm.set_grip(grip_for(0.040), 0.4)
    yield from wait(0.3)
    yield from rise(arm, z=g.TOP + 0.13, seconds=1.0)


# -- the lid ---------------------------------------------------------------------------------------------------------------
def shut_lid(ctx, arm):
    """Left arm: push the open lid's outer face toward the robot's right until it is past vertical; it falls shut."""
    sc = Scene(ctx)
    th = sc.lid
    a = np.array([0.0, -math.sin(LID_TILT), -math.cos(LID_TILT)])
    R = R_from(a, [1.0, 0.0, 0.0])
    n = sc.R0 @ g.lid_rot(th) @ np.array([0.0, 0.0, 1.0])               # the lid's outer face normal
    face = sc.world(g.lid_to_station((0.0, -LID_SPAN, g.LID_SKIRT), th))
    tip = face + 0.004 * n
    tcp = tip - TIP * a
    yield from arm.move_joints(tcp + 0.04 * n + [0, 0, 0.02], R, 2.0, grip=0.0)
    yield from settle_to(arm, tcp, R, rounds=2, seconds=0.6)
    cmd = tcp.copy()
    for k in range(200):
        th = Scene(ctx).lid
        if th < math.radians(78.0):
            break
        cmd[1] -= 0.0008
        arm.aim(cmd, R)
        yield
    ctx.flags["lid_push"] = dict(steps=k, lid_deg=round(math.degrees(Scene(ctx).lid), 1),
                                 hand_y_mm=round((tcp[1] - cmd[1]) * 1000, 1))
    p_act, _ = tcp_actual(arm)
    arm.pos = p_act
    yield from arm.move(p_act + [0, 0.04, 0.03], R, 0.6)
    yield from arm.go_home(2.0)


def wait_lid_settled(ctx, seconds=6.0):
    last, still = None, 0
    for k in range(round(seconds * 20)):
        th = Scene(ctx).lid
        if last is not None and abs(th - last) < math.radians(0.05) and th < math.radians(5.0):
            still += 1
            if still >= 8:
                break
        else:
            still = 0
        last = th
        yield
    ctx.flags["lid_rest_deg"] = round(math.degrees(Scene(ctx).lid), 2)


def press_locks(ctx, arm, keep=False):
    """Right arm: press the lid's free end down, deeper by steps, until the lid is held within 0.2 degrees of shut,
    hold, then rise (``keep``: stay pressing)."""
    sc = Scene(ctx)
    R = down(math.radians(-90))
    top = sc.world((0.0, PRESS_Y, g.ST_H))
    p = top + [0, 0, TIP]
    yield from arm.move_joints(p + [0, 0, 0.05], R, 2.0, grip=0.0)
    yield from settle_to(arm, p + [0, 0, 0.004], R, rounds=2, seconds=0.6)
    cmd = arm.pos.copy()
    held, depth = 0, 0.0
    for k in range(int(PRESS_MAX / PRESS_STEP) + 60):
        th = Scene(ctx).lid
        if th <= math.radians(0.2):
            held += 1
            if held >= 10:
                break
        else:
            held = 0
            if depth < PRESS_MAX:
                cmd[2] -= PRESS_STEP
                depth = (p[2] + 0.004) - cmd[2]
        arm.aim(cmd, R)
        yield
    ctx.flags["press"] = dict(depth_mm=round(depth * 1000, 1), lid_deg=round(math.degrees(Scene(ctx).lid), 3),
                              locked_by_step=ctx.task._STATE.get("lock_record"))
    if keep:
        return
    yield from wait(0.3)
    p_act, _ = tcp_actual(arm)
    arm.pos = p_act
    yield from arm.move(p_act + [0, 0, 0.06], R, 0.8)
    yield from wait(0.5)
    ctx.flags["after_press_lid_deg"] = round(math.degrees(Scene(ctx).lid), 3)


# -- the push ----------------------------------------------------------------------------------------------------------------
def push_to_wall(ctx):
    """Both arms: the closed fingers' flat faces on the front face at y +-PUSH_Y push the station until both back
    corners are within WALL_DONE of the wall."""
    L, Rr = ctx.left, ctx.right
    sc = Scene(ctx)
    RL, RR = down(math.pi), down(0.0)

    def contact(sc, side) -> np.ndarray:
        y = PUSH_Y if side == "left" else -PUSH_Y
        f = sc.world((-g.HALF_D, y, 0.0))
        return np.array([f[0] - FT_HALF, f[1], g.TOP + PUSH_TIP_Z + TIP])

    cl, cr = contact(sc, "left"), contact(sc, "right")
    yield from together(L.move_joints(cl + [-0.03, 0, 0.06], RL, 2.0, grip=0.0),
                        Rr.move_joints(cr + [-0.03, 0, 0.06], RR, 2.0, grip=0.0))
    yield from together(L.move(cl + [-0.02, 0, 0], RL, 0.8), Rr.move(cr + [-0.02, 0, 0], RR, 0.8))
    cmdl, cmdr = L.pos.copy(), Rr.pos.copy()
    ys = {"left": cmdl[1], "right": cmdr[1]}            # the hands keep their y (their friction resists any drift)
    best, still = 1.0, 0
    for k in range(500):
        sc = Scene(ctx)
        gr, gl = sc.corners()                               # right (-y) and left (+y) back corners' gaps
        if max(gr, gl) <= WALL_DONE:
            break
        if max(gr, gl) < best - 0.0003:
            best, still = max(gr, gl), 0
        else:
            still += 1
        if still >= 30 and best <= 0.010:                   # no progress for 1.5 s, square enough: done
            break
        # each hand led PUSH_LEAD past its contact; the side further from the wall leads more
        tl, tr = contact(sc, "left"), contact(sc, "right")
        skew = gl - gr
        for cmd, t, extra, side in ((cmdl, tl, max(0.0, skew), "left"), (cmdr, tr, max(0.0, -skew), "right")):
            want = t[0] + PUSH_LEAD + min(0.012, 3.0 * extra)
            cmd[0] = min(cmd[0] + PUSH_SPEED, want) if cmd[0] < want else cmd[0]
            cmd[1] = ys[side]
        L.aim(cmdl, RL)
        Rr.aim(cmdr, RR)
        yield
    sc = Scene(ctx)
    ctx.flags["push"] = dict(steps=k, corners_mm=[round(v * 1000, 1) for v in sc.corners()])
    yield from wait(0.3)
    for arm in (L, Rr):
        p_act, _ = tcp_actual(arm)
        arm.pos = p_act
    yield from together(L.move(L.pos + [-0.06, 0, 0], RL, 0.8), Rr.move(Rr.pos + [-0.06, 0, 0], RR, 0.8))
    yield from together(L.move(L.pos + [0, 0, 0.06], RL, 0.6), Rr.move(Rr.pos + [0, 0, 0.06], RR, 0.6))


# -- the scripts ---------------------------------------------------------------------------------------------------------------
def script(ctx, variant: str):
    print(f"MODE variant={variant}", flush=True)
    L, Rr = ctx.left, ctx.right
    if variant == "loose_wall":
        yield from lay_by_wall(ctx, Rr, "block_a", (0.420, -0.215))
        yield from lay_by_wall(ctx, Rr, "block_b", (0.420, -0.165))
        yield from Rr.go_home(2.0)
    else:
        if variant == "both_off":
            yield from drop_inside(ctx, Rr, "block_a", (0.0, -0.035))
        else:
            yield from thread(ctx, Rr, "block_a", 0)
        if variant in ("off_rod", "off_rod_no_push", "both_off"):
            yield from drop_inside(ctx, Rr, "block_b", (0.0, 0.035))
        elif variant != "one_in":
            yield from thread(ctx, Rr, "block_b", 2)
        yield from Rr.go_home(2.0)
        if variant != "open":
            yield from shut_lid(ctx, L)
            yield from wait_lid_settled(ctx)
            if variant != "shut_unlocked":
                yield from press_locks(ctx, Rr, keep=(variant == "hold_lid"))
                if variant == "hold_lid":
                    yield from wait(3.0)
                    print("FLAGS", json.dumps(ctx.flags), flush=True)
                    return
                yield from Rr.go_home(2.0)
            if variant not in ("no_push", "off_rod_no_push"):
                yield from push_to_wall(ctx)
            yield from together(L.go_home(2.0), Rr.go_home(2.0))
    print("FLAGS", json.dumps(ctx.flags), flush=True)
    yield from wait(1.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    bs = " ".join(f"{n[-1]} in {int(b['inside'])} rod {b['rod']} off {b['axis_off'] * 1000:.1f} depth "
                  f"{b['depth'] * 1000:.1f} on {int(b['on_rod'])}" for n, b in S.get("blocks", {}).items())
    sc = Scene(ctx)
    return (f"lid {math.degrees(S['lid']):.2f} locked {S['locked']} safe {S.get('safe')} wall "
            f"{[round(v * 1000, 1) for v in S['wall']]} gap {S['gap_mm']} fell {S['fell']} {bs} tier {S['tier']} "
            f"st {np.round((sc.p0 - [g.ST_X, g.ST_Y, g.TOP]) * 1000, 1).tolist()} events {S['events'][-3:]}")


# the start views: (eye, target) of the top camera, m in the station frame, heights above the bench. One from the right
# of the right arm (the blocks, the pail's label and lid, the droppings' right end), one from the left of the left arm
# (the cup and the truck by the wall); both see past the arms' ready poses.
START_SHOTS = (((0.15, -0.65, 0.30), (0.33, -0.12, 0.04)), ((0.00, 0.45, 0.40), (0.30, 0.20, 0.04)))
START_SHOT_S = 1.25


def start_view(ctx, out_dir):
    """Recorded runs, before the arms move: the top camera shows the start from close by, START_SHOT_S from each of
    START_SHOTS, then goes back to its own pose. Only the recording changes; the arms stay at their ready pose."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)
    pos0 = cam.data.pos_w.torch[0:1].clone()
    quat0 = cam.data.quat_w_opengl.torch[0:1].clone()
    T = g.TOP
    steps = []
    for eye, tgt in START_SHOTS:
        e = torch.tensor([[eye[0], eye[1], T + eye[2]]], dtype=torch.float32, device=o.device) + o
        t = torch.tensor([[tgt[0], tgt[1], T + tgt[2]]], dtype=torch.float32, device=o.device) + o
        cam.set_world_poses_from_view(e, t)
        steps.append(ctx.task._STATE["step"] + 1)
        print("top camera: start view", eye, tgt, flush=True)
        yield from wait(START_SHOT_S)
    cam.set_world_poses(positions=pos0, orientations=quat0, convention="opengl")
    steps.append(ctx.task._STATE["step"] + 1)
    print(f"top camera: back to its pose {pos0.tolist()} {quat0.tolist()}", flush=True)
    ctx.flags["start_view_steps"] = steps
    (pathlib.Path(out_dir) / "start_view.json").write_text(json.dumps({"shots": steps[:-1], "back": steps[-1]}))


def close_view(ctx, gen):
    """Recorded runs: once both arms are home at the end, the top camera moves to a close view of the station (from
    the robot's side and above), so the video ends on the locked station at the wall. Only the recording changes."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)
    moved = False
    for _ in gen:
        yield
        if not moved and ctx.flags.get("close_view"):
            sc = Scene(ctx)
            t = sc.world((0.0, 0.0, g.ST_H * 0.6))
            eye = t + np.array([-0.33, -0.16, 0.30])
            cam.set_world_poses_from_view(torch.tensor(np.array([eye]), dtype=torch.float32, device=o.device) + o,
                                          torch.tensor(np.array([t]), dtype=torch.float32, device=o.device) + o)
            moved = True
            ctx.flags["close_view_step"] = ctx.task._STATE["step"]
            print("top camera: close view", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--close-view", action="store_true")
    ap.add_argument("--start-view", action="store_true", help="recorded runs: open on a close view of the start")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds after the script ends (inf: to the step limit)")
    ap.add_argument("--variant", default="full",
                    choices=["full", "loose_wall", "one_in", "shut_unlocked", "off_rod", "no_push", "off_rod_no_push", "both_off",
                             "open", "hold_lid", "nothing"])
    a = ap.parse_args()
    if a.variant == "nothing":
        def scr(ctx):
            print("MODE variant=nothing", flush=True)
            yield from wait(5.0)
    else:
        def scr(ctx):
            def body():
                if a.start_view and not a.no_record:
                    yield from start_view(ctx, a.out)
                yield from script(ctx, a.variant)
                ctx.flags["close_view"] = True
                yield from wait(2.0)
            if a.close_view and not a.no_record:
                yield from close_view(ctx, body())
            else:
                yield from body()
    res = run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
              log_every=int(os.environ.get("MB_LOG_EVERY", "20")), hold_s=a.hold)
    S = sys.modules["task"]._STATE if "task" in sys.modules else {}
    extra = {"variant": a.variant, "tier": S.get("tier"), "safety_flag": S.get("safe"), "locked": S.get("locked"),
             "lid_deg": round(math.degrees(S.get("lid", 0.0)), 3), "along_wall": S.get("along_wall"),
             "wall_mm": [round(v * 1000, 2) for v in S.get("wall", [])], "gap_mm": S.get("gap_mm"),
             "fell": S.get("fell"), "events": S.get("events"), "lock_record": S.get("lock_record"),
             "blocks": {n: {"inside": b["inside"], "on_rod": b["on_rod"], "rod": b["rod"],
                            "axis_off_mm": round(b["axis_off"] * 1000, 2), "depth_mm": round(b["depth"] * 1000, 2),
                            "tilt_deg": round(math.degrees(b["tilt"]), 2), "p_mm": np.round(b["p"] * 1000, 1).tolist()}
                        for n, b in S.get("blocks", {}).items()}}
    print("EXTRA " + json.dumps(extra, default=str), flush=True)
    (pathlib.Path(a.out) / "extra.json").write_text(json.dumps(extra, indent=2, default=str))
