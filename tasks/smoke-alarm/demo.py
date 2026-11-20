"""Scripted full-score demo (reads object poses from the simulator: the alarm's pose, its compartment's hinge angle,
the latch tab's travel and the batteries' poses). README.md describes each step.

1. Right arm, open: closed fingertips press the latch tab down until the compartment is freed and springs out past
   its catch, then push the tab's inner face outward along the compartment's arc (closed loop on the hinge angle).
2. Left arm, the pin: fingers closed, tipped 45 degrees toward the compartment, pressing on the alarm's back beside
   it (about 10 N); it holds the alarm down until the compartment is latched again.
3. Right arm, the old battery: a straight-down pinch across its width on its free half; the hand turns about the
   battery's snap end until its free end is 3 mm up (it unsnaps), lifts it out and lays it on the table.
4. Right arm, the new battery: a straight-down pinch on its card, turned so its snaps point at the contacts with the +
   stud on the side toward the alarm's centre; lowered into the pocket (closed loop on its pose), slid onto its contacts
   and pressed through the pinch until it snaps.
5. Right arm, latch: the closed fingertips push the tab's outer face back along the arc until the compartment is shut
   past its catch, then both arms withdraw.
6. Both arms, mount: side grips on the alarm's band (each hand along the band's tangent, below the board), lift, carry
   it under the ring, raise it onto the ring's face, turn it clockwise seen from below until it locks, let go.

    python demo.py task.py demo_out [--max-steps 6000] [--no-record] [--variant NAME] [--close-view]

Variants (scorer checks), each printing its mode:
- ``pull_leave``: the old battery taken out and laid on the table, the compartment left open;
- ``no_latch``: the full swap, the compartment left open;
- ``open_only``: the compartment opened, nothing else;
- ``reversed``: the new battery starts on its card turned over (label down); the same moves put it in turned over,
  then the compartment is pushed shut;
- ``latch_empty``: the old battery taken out, the empty compartment pushed shut;
- ``relatch``: the old battery taken out, the empty compartment pushed shut, opened again, the new battery snapped in,
  the compartment latched and the alarm mounted;
- ``no_mount``: the full swap and latch, the alarm left on the counter;
- ``mount_only``: the mount alone (the old battery stays in);
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
from demokit import run, together, turn, wait  # noqa: E402

DEBUG = bool(os.environ.get("SA_DEBUG"))
GAP_MM = 94.0                       # pads' gap at grip 1.0
TIP = 0.013                         # fingertips past the tool point
TAB_C = ((g.TAB_X[0] + g.TAB_X[1]) / 2, (g.TAB_Y[0] + g.TAB_Y[1]) / 2)
FT_HALF = 0.0064                    # half the closed fingertips' thickness along the closing axis
PIN_XY = (0.300, -0.015)          # on the back beside the compartment
PIN_TILT = math.radians(45.0)
PIN_DEPTH = float(os.environ.get("SA_PIN", "0.030"))      # commanded below the back (~10 N)
PRESS_DEPTH = float(os.environ.get("SA_PRESS", "0.026"))  # commanded below where the battery met the floor (press peaks ~20 N)
LEAD = math.radians(float(os.environ.get("SA_LEAD", "6")))
PUSH_X = -0.0500                    # the pushing fingertips' centre along the compartment (their 26 mm width spans
                                    # the tab and stays clear of the back beyond the free end)
TRACE = bool(os.environ.get("SA_TRACE"))
PICK_UP = 0.011                     # the new battery's pinch: fingertips this far above its bottom
LATCH_OVER = math.radians(20.0)     # the last push aims this far beyond shut (the arm gives about 3 mm per newton)
HOVER = 0.012                       # the new battery's bottom over the floor while it is lined up (walls are 8 mm)


def grip_for(width_m: float) -> float:
    return max(0.0, min(1.0, width_m * 1000 / GAP_MM))


def R_from(a, c) -> np.ndarray:
    """Gripper rotation with approach ``a`` (its z) and closing axis ``c`` (its x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def tilted(yaw: float, phi: float, toward: float) -> np.ndarray:
    """Fingers pointing down, tipped ``phi`` toward the horizontal direction ``toward``, closing axis horizontal at
    ``yaw``."""
    a = np.array([math.cos(toward) * math.sin(phi), math.sin(toward) * math.sin(phi), -math.cos(phi)])
    return R_from(a, [math.cos(yaw), math.sin(yaw), 0.0])


def down(yaw: float) -> np.ndarray:
    return R_from([0.0, 0.0, -1.0], [math.cos(yaw), math.sin(yaw), 0.0])


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
        self.alarm = st["smoke_alarm"]
        self.old, self.new = st["old_battery"], st["new_battery"]
        self.hinge = float(self.alarm.joints["hinge"])
        self.tab = float(self.alarm.joints["tab"])
        self.p0 = np.asarray(self.alarm.pos, float)
        self.R0 = ctx.task._rotation(self.alarm.quat_xyzw)
        self.yaw0 = math.atan2(self.R0[1, 0], self.R0[0, 0])

    def tray_point(self, local, z, hinge=None) -> np.ndarray:
        """A point in the compartment's frame (x, y) at height z above the table, world."""
        th = self.hinge if hinge is None else hinge
        p = g.tray_to_alarm([local[0], local[1], 0.0], th)
        w = self.p0 + self.R0 @ p
        return np.array([w[0], w[1], g.TOP + z])

    def tray_rot(self, hinge=None) -> np.ndarray:
        th = self.hinge if hinge is None else hinge
        return self.R0 @ g.rot_z(th)

    def batt(self, name):
        b = getattr(self, "old" if name == "old_battery" else "new")
        return np.asarray(b.pos, float), self.task._rotation(b.quat_xyzw)


def settle_to(arm, pos, rot, tol=0.0006, rounds=3, seconds=0.6):
    """Move to ``pos`` and correct the command by the measured tool-point error."""
    target = np.asarray(pos, float)
    cmd = target.copy()
    yield from arm.move(cmd, rot, seconds)
    for _ in range(rounds):
        yield from wait(0.2)
        err = target - tcp_actual(arm)[0]
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
        yield from arm.move(cmd, rot, 0.3)


# -- left arm: the pin ---------------------------------------------------------------------------------------------------
def pin_pose():
    """Fingers tipped PIN_TILT toward the compartment (the left forearm stays clear of the right arm's), closing axis
    along x; the lowest corner of the closed fingertips on the back, beside the compartment."""
    R = tilted(math.pi, PIN_TILT, -math.pi / 2)
    low = TIP * math.cos(PIN_TILT) + 0.0132 * math.sin(PIN_TILT)
    p = np.array([PIN_XY[0], PIN_XY[1] + TIP * math.sin(PIN_TILT), g.TOP + g.ALARM_H + low])
    return p, R


def pin_script(ctx, arm):
    """Holds the alarm down while ctx.flags["pin_want"] is set, waits above it otherwise."""
    p, R = pin_pose()
    wait_p = p + np.array([0.0, 0.03, 0.06])
    yield from arm.move_joints(wait_p, R, 2.0, grip=0.0)
    state = False
    ctx.flags["pin_state"] = state
    while not ctx.flags.get("release_pin"):
        want = bool(ctx.flags.get("pin_want"))
        if want == state:
            yield
            continue
        if want:
            yield from arm.move(p + [0, 0, 0.005], R, 0.8)
            yield from arm.move(p - [0, 0, PIN_DEPTH], R, 1.0)
        else:
            yield from arm.move(wait_p, R, 0.8)
        state = want
        ctx.flags["pin_state"] = state
    yield from arm.move(p + [0, 0, 0.05], R, 0.8)
    ctx.flags["left_free"] = True


def set_pin(ctx, want: bool):
    ctx.flags["pin_want"] = want
    while ctx.flags.get("pin_state") != want:
        yield


# -- right arm ------------------------------------------------------------------------------------------------------------
def press_tab(ctx, arm):
    sc = Scene(ctx)
    R = down(math.radians(-90))
    top = sc.tray_point(TAB_C, g.TAB_Z[1] + 0.0004 + TIP)
    yield from arm.move_joints(top + [0, 0, 0.04], R, 2.0, grip=0.0)
    yield from settle_to(arm, top + [0, 0, 0.004], R)
    cmd = arm.pos.copy()
    for k in range(200):                                       # down 6 mm/s until the tab has gone 1.6 mm
        if -Scene(ctx).tab >= 0.00155 or top[2] - cmd[2] > 0.012:
            break
        cmd[2] -= 0.0003
        arm.aim(cmd, R)
        yield
    ctx.flags["tab_cmd_mm"] = round((top[2] - cmd[2]) * 1000, 1)
    p_act, _ = tcp_actual(arm)
    arm.pos = p_act
    yield from arm.move(p_act + [0, 0, 0.012], R, 0.4)


def push_arc(ctx, arm, to: float, side: float, z: float, tol: float):
    """Push the tab along the compartment's arc to hinge angle ``to``: the closed fingertips on the tab's inner face
    (``side`` +1, pushing it open) or outer face (-1, pushing it shut), their bottoms ``z`` above the table, led LEAD
    ahead of the measured angle."""
    sc = Scene(ctx)
    yoff = (g.TAB_Y[1] + FT_HALF + 0.0012) if side > 0 else (g.TAB_Y[0] - FT_HALF - 0.0012)
    th = sc.hinge
    R = down(sc.yaw0 + th - math.pi / 2)
    xc = PUSH_X
    p = sc.tray_point((xc, yoff), z + TIP, th)
    yield from arm.move_joints(p + [0, 0, 0.03], R, 1.5, grip=0.0)          # fingers closed: one 12.8 mm block
    yield from settle_to(arm, p, R, rounds=2, seconds=0.5)
    sgn = 1.0 if to > th else -1.0
    stall = 0
    best = th
    for k in range(400):
        sc = Scene(ctx)
        th = sc.hinge
        if (to - th) * sgn <= tol:
            break
        cmd_th = th + sgn * min(LEAD, abs(to - th) + math.radians(1.0))
        y_in = yoff - side * 0.0008                           # press 0.8 mm into the face
        p = sc.tray_point((xc, y_in), z + TIP, cmd_th)
        arm.aim(p, down(sc.yaw0 + cmd_th - math.pi / 2))
        yield
        if TRACE and k % 5 == 0:
            pa, _ = tcp_actual(arm)
            print(f"PUSH k {k} th {math.degrees(th):.2f} cmd {math.degrees(cmd_th):.2f} tcp_err_mm "
                  f"{np.round((p - pa) * 1000, 1).tolist()}", flush=True)
        if (th - best) * sgn > math.radians(0.05):
            best, stall = th, 0
        else:
            stall += 1
        if stall > 60:
            break
    ctx.flags.setdefault("push_log", []).append((round(math.degrees(to), 1), round(math.degrees(Scene(ctx).hinge), 2), k))


def open_door(ctx, arm):
    yield from press_tab(ctx, arm)
    yield from push_arc(ctx, arm, g.SWING - math.radians(4.0), +1, 0.0495, math.radians(0.5))
    p_act, _ = tcp_actual(arm)
    arm.pos = p_act
    yield from arm.move(p_act + [0, 0, 0.03], arm.rot, 0.6)


def shut_door(ctx, arm):
    # coming from elsewhere (not from the pocket), first go to above the pocket, where the full demo's hand is
    sc = Scene(ctx)
    stage = sc.tray_point((g.SEAT[0], g.SEAT[1]), 0.10, sc.hinge)
    if np.linalg.norm(tcp_actual(arm)[0] - stage) > 0.06:
        yield from arm.move_joints(stage, down(sc.yaw0 + sc.hinge - math.pi / 2), 1.5)
        ctx.flags["shut_staged"] = True
    yield from push_arc(ctx, arm, math.radians(2.0), -1, 0.0400, math.radians(0.5))
    # past the catch: push on toward LATCH_OVER beyond shut until the compartment has stayed shut 0.3 s
    n_shut, over = 0, 0.0
    for k in range(160):
        sc = Scene(ctx)
        n_shut = n_shut + 1 if sc.hinge < g.LATCH_AT - math.radians(0.15) else 0
        if n_shut >= 6:
            break
        over = min(LATCH_OVER, over + math.radians(0.5))
        p = sc.tray_point((PUSH_X, g.TAB_Y[0] - FT_HALF - 0.0004), 0.0400 + TIP, -over)
        arm.aim(p, down(sc.yaw0 - over - math.pi / 2))
        yield
    ctx.flags["latch_push_deg"] = round(math.degrees(over), 1)
    yield from wait(0.2)
    p_act, _ = tcp_actual(arm)
    arm.pos = p_act
    sc = Scene(ctx)
    away = sc.R0 @ g.rot_z(sc.hinge) @ np.array([0.0, -1.0, 0.0])
    yield from arm.move(p_act + 0.012 * away, arm.rot, 0.4)
    yield from arm.move(arm.pos + [0, 0, 0.05], arm.rot, 0.6)
    ctx.flags["shut_deg"] = round(math.degrees(Scene(ctx).hinge), 3)


def take_old(ctx, arm):
    """Pinch the old battery on its free half, lift until its free end is 3 mm up (unsnapped), lift it out and lay it
    on the table."""
    sc = Scene(ctx)
    th = sc.hinge
    R = down(sc.yaw0 + th - math.pi / 2)
    p = sc.tray_point((-0.040, g.POCKET_YC), g.WALL_TOP + 0.0015 + TIP, th)
    yield from arm.move_joints(p + [0, 0, 0.04], R, 1.5, grip=grip_for(0.042))
    yield from settle_to(arm, p + [0, 0, 0.01], R)
    yield from settle_to(arm, p, R)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.3)
    # tip its free end up about the snap end (the hand turns about the snap end's bottom edge) until it is 3 mm up
    p0, R0h = arm.pos.copy(), arm.rot.copy()
    pivot = sc.tray_point((g.BLOCK_FACE, g.POCKET_YC), g.FLOOR_TOP, th)
    axis = sc.tray_rot(th) @ np.array([0.0, 1.0, 0.0])          # turning about +y_tray lifts the free end (-x_tray)
    from demokit import turn
    lifted, a = 0.0, 0.0
    for k in range(200):
        b = ctx.task._STATE["batt"].get("old_battery")
        lifted = max(b["lift"]) if b else 0.0
        if lifted >= 0.003:
            break
        a += math.radians(0.25)
        Rr = turn(axis, a)
        arm.aim(pivot + Rr @ (p0 - pivot), Rr @ R0h)
        yield
    ctx.flags["unsnap_deg"] = round(math.degrees(a), 1)
    yield from wait(0.2)
    p_act, _ = tcp_actual(arm)
    arm.pos = p_act
    yield from arm.move(p_act + [0, 0, 0.045], R0h, 1.2)
    p_act, _ = tcp_actual(arm)
    place = np.array([0.165, -0.045, g.TOP + g.BATT_T / 2 + 0.002])
    pb, Rb = Scene(ctx).batt("old_battery")
    off = p_act - pb
    ctx.flags["old_out_mm"] = np.round((pb - [0, 0, g.TOP]) * 1000, 1).tolist()
    yield from arm.move_joints(place + off + [0, 0, 0.03], R0h, 1.5)
    yield from arm.move(place + off, arm.rot, 0.8)
    yield from arm.set_grip(grip_for(0.045), 0.5)
    yield from arm.move(arm.pos + [0, 0, 0.05], arm.rot, 0.6)


def battery_in_hand(ctx, arm, name):
    p, R = tcp_actual(arm)
    pb, Rb = Scene(ctx).batt(name)
    return R.T @ (pb - p), R.T @ Rb


def hand_for_battery(rel, pb, Rb):
    """The hand pose that puts the held battery at (pb, Rb)."""
    t_in, R_in = rel
    Rh = Rb @ R_in.T
    return pb - Rh @ t_in, Rh


def insert_new(ctx, arm, flip=False):
    sc = Scene(ctx)
    pn, _ = sc.batt("new_battery")
    R = down(math.radians(-90))
    # fingertips 11 mm up the battery: seated, they stay 3 mm above the pocket's walls
    p = np.array([pn[0], pn[1], g.TOP + g.CARD[2] + PICK_UP + TIP])
    yield from arm.move_joints(p + [0, 0, 0.05], R, 2.0, grip=grip_for(0.042))
    yield from settle_to(arm, p + [0, 0, 0.012], R)
    yield from settle_to(arm, p, R)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0, 0, 0.06], R, 1.0)
    rel = battery_in_hand(ctx, arm, "new_battery")
    ctx.flags["new_in_hand_mm"] = np.round(rel[0] * 1000, 1).tolist()
    # the seated pose (0.5 mm short of the block), hovering
    sc = Scene(ctx)
    Rt = sc.tray_rot()
    Rb = Rt @ (np.diag([1.0, -1.0, -1.0]) if flip else np.eye(3))
    seat = g.SEAT - np.array([0.0010, 0.0, 0.0])

    def target(h):
        s = Scene(ctx)
        q = g.tray_to_alarm(seat + [0, 0, h], s.hinge)
        return s.p0 + s.R0 @ q

    ph, Rh = hand_for_battery(rel, target(0.03), Rb)
    yield from arm.move_joints(ph, Rh, 2.0)
    # hover with its bottom HOVER over the floor (above the pocket's walls), settle on its measured position
    goal = target(HOVER)
    ph, Rh = hand_for_battery(rel, goal, Rb)
    yield from arm.move(ph, Rh, 1.0)
    cmd = ph.copy()
    for r in range(6):
        yield from wait(0.3)
        pb, _ = Scene(ctx).batt("new_battery")
        err = goal - pb
        if np.linalg.norm(err[:2]) < 0.0003:
            break
        cmd = cmd + np.r_[err[:2], 0.0]
        yield from arm.move(cmd, Rh, 0.3)
    ctx.flags["hover_err_mm"] = np.round(err * 1000, 2).tolist()
    # straight down, its measured xy held on the goal in closed loop, until it is 0.5 mm over the floor, then let go
    for k in range(160):
        pb, _ = Scene(ctx).batt("new_battery")
        if pb[2] - (goal[2] - HOVER) < 0.0005:
            break
        cmd[:2] += 0.3 * (goal[:2] - pb[:2])
        cmd[2] -= 0.0003
        arm.aim(cmd, Rh)
        yield
    b = ctx.task._STATE["batt"]["new_battery"]
    ctx.flags["placed"] = dict(gap_mm=round(b["gap"] * 1000, 2), side_mm=round(b["side"] * 1000, 2),
                               lift_mm=[round(v * 1000, 2) for v in b["lift"]])
    # slide it in toward the contacts on the floor until its snaps are in their slots (the case 0.5 mm from the
    # block, read off its pose), then press it down through the pinch (the fingertips stay above the walls), the hand
    # commanded PRESS_DEPTH below: "push the battery in until it snaps"
    xt = Scene(ctx).tray_rot()[:, 0]
    for k in range(80):
        b = ctx.task._STATE["batt"]["new_battery"]
        if b["gap"] <= 0.0005:
            break
        cmd[:2] += 0.0001 * xt[:2]
        cmd[2] -= 0.00003
        arm.aim(cmd, Rh)
        yield
    ctx.flags["slid_in_mm"] = round(ctx.task._STATE["batt"]["new_battery"]["gap"] * 1000, 2)
    z0 = cmd[2]
    fmax, trace = 0.0, []
    for k in range(int(PRESS_DEPTH / 0.0004) + 12):           # down 8 mm/s, then hold
        if cmd[2] > z0 - PRESS_DEPTH:
            cmd[2] -= 0.0004
        arm.aim(cmd, Rh)
        yield
        b = ctx.task._STATE["batt"]["new_battery"]
        fmax = max(fmax, b["force"])
        if TRACE and k % 4 == 0:
            print(f"PRESS k {k} F {b['force']:.2f} lift {np.round(np.asarray(b['lift']) * 1000, 2).tolist()} "
                  f"gap {b['gap'] * 1000:.2f} snapped {ctx.task._STATE['snapped']}", flush=True)
    ctx.flags["press_fmax"] = round(fmax, 2)
    yield from wait(0.2)
    yield from arm.set_grip(grip_for(0.036), 0.4)
    p_act, _ = tcp_actual(arm)
    arm.pos = p_act
    yield from arm.move(p_act + [0, 0, 0.03], Rh, 0.8)


def flip_new(ctx):
    """Variant setup: the new battery turned over on its card (label down)."""
    import torch

    from scenekit import env

    base = env().unwrapped
    a = base.scene["new_battery"]
    o = base.scene.env_origins[0]
    pose = torch.tensor([[g.NEW_X, g.NEW_Y, g.TOP + g.CARD[2] + g.BATT_T / 2 + 0.0003, 1.0, 0.0, 0.0, 0.0]],
                        dtype=torch.float32, device=a.device)
    pose[0, :3] += o
    a.write_root_link_pose_to_sim_index(root_pose=pose)
    a.write_root_link_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=a.device))
    yield


# -- both arms: the alarm onto its bracket -----------------------------------------------------------------------------
SQUEEZE = float(os.environ.get("SA_SQUEEZE", "0.030"))     # each hand commanded this far into the band
GRIP_A = {"left": math.radians(130.0), "right": math.radians(310.0)}  # where each hand holds the band (alarm frame),
                                     # clear of the compartment's opening at the rim (236 to 297 degrees)
BETA = {"left": math.radians(70.0), "right": math.radians(-80.0)}     # each hand's roll from fingers-down (see side_grip)
FLIP = {"left": True, "right": False}
HC = 0.040                           # the fingertips' tapered side, its middle this far above the alarm's front
SEED = {"left": (-1.2806, 1.8996, 0.9325, -0.6419, -1.2465, -0.1722),  # IK seeds for the first approach (offline IK)
        "right": (0.2835, 2.0345, 0.5645, 1.2406, -0.7801, 1.4722)}
TWIST = math.radians(float(os.environ.get("SA_TWIST", "23")))
LEVEL_GAIN = float(os.environ.get("SA_LEVEL", "0.08"))   # levelling: each hand moves this fraction of its side's height
NF_ANG = math.atan2(5.5, 14.0)       # the closed fingertips' tapered inner side: 21.4 degrees off the approach axis
PF = np.array([0.0, 0.01045, 0.005])  # its middle, from the tool point (gripper frame)


def side_grip(aw: float, beta: float, flip: bool):
    """A side grip on the band at angle ``aw`` (world): the closed fingertips' tapered inner side flat on the band, the
    hand rolled ``beta`` about the band's normal from fingers-down (70 degrees: the fingers along the band's tangent,
    20 degrees below level). Returns the rotation and the tool point's offset from the contact."""
    u = np.array([math.cos(aw), math.sin(aw), 0.0])
    t = np.array([math.sin(aw), -math.cos(aw), 0.0])
    ez = np.array([0.0, 0.0, 1.0])
    y = math.cos(NF_ANG) * (-u) + math.sin(NF_ANG) * ez
    z = -math.cos(NF_ANG) * ez - math.sin(NF_ANG) * u
    R = turn(-u, beta) @ np.column_stack([t, y, z])
    off = R @ PF
    if flip:                         # the other fingertip's side: the same contact, the hand turned over
        R = R @ np.diag([-1.0, -1.0, 1.0])
    return R, off


def mtrace(ctx, what):
    """Mount progress: the alarm's pose and each hand's distance from its axis (the log)."""
    s = Scene(ctx)
    hands = {arm.side: tcp_actual(arm)[0] for arm in (ctx.left, ctx.right)}
    r = {k: round(float(np.linalg.norm((v - s.p0)[:2])) * 1000, 1) for k, v in hands.items()}
    z = {k: round(float(v[2] - g.TOP) * 1000, 1) for k, v in hands.items()}
    f = {}
    try:
        V = ctx.flags.get("_hand_view")
        if V is None:
            from yam_arena.robot import asset_link_paths
            from scenekit import env
            import yam_arena.constants as C
            psv = env().unwrapped.sim.physics_sim_view
            ns = "/World/envs/env_*"
            lp = asset_link_paths()
            hands = [f"{ns}/{arm}{lp[link]}" for arm in ("LeftArm", "RightArm") for link in ("gripper", "tip_left", "tip_right")]
            V = (psv.create_rigid_contact_view(f"{ns}/smoke_alarm/body", filter_patterns=hands), float(C.PHYSICS_DT))
            ctx.flags["_hand_view"] = V
        m = V[0].get_contact_force_matrix(V[1])
        m = np.asarray(m.numpy() if hasattr(m, "numpy") else m).reshape(-1, 3)
        f = {k: round(float(np.linalg.norm(m[i])), 2) for i, k in enumerate(("Lg", "Ltl", "Ltr", "Rg", "Rtl", "Rtr"))}
    except Exception as e:                                  # debugging aid only
        f = {"err": repr(e)[:80]}
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, s.R0[2, 2]))))
    print(f"MOUNT {what}: alarm {np.round((s.p0 - [0, 0, g.TOP]) * 1000, 1).tolist()} yaw {math.degrees(s.yaw0):.2f} "
          f"tilt {tilt:.2f} hand r {r} z {z} forces {f}", flush=True)
    return f


def both(ctx, fn, seconds):
    """Both arms to the poses fn(arm) -> (p, R), together, in straight lines."""
    yield from together(*(arm.move(*fn(arm), seconds) for arm in (ctx.left, ctx.right)))


def mount(ctx):
    """Grip the alarm's band from opposite sides with both hands' closed fingertips (side grips, the hands along the
    band's tangent), lift it, carry it under the bracket, raise it until its back meets the ring's face, turn it
    clockwise seen from below until it locks, let go and draw the hands out under the board."""
    ctx.flags["view"] = "mount"
    sc = Scene(ctx)
    c, yaw = sc.p0.copy(), sc.yaw0
    b = np.array([g.BRACKET_X, g.BRACKET_Y])

    def at(centre, front, dr, phi=0.0, dz=None):
        """Each hand's pose: its contact dr outside the band (negative: pressed in), HC above the alarm's front at
        height ``front`` (plus dz[side])."""
        def fn(arm):
            aw = yaw + phi + GRIP_A[arm.side]
            R, off = side_grip(aw, BETA[arm.side], FLIP[arm.side])
            r = g.ALARM_R + dr
            cp = np.array([centre[0] + r * math.cos(aw), centre[1] + r * math.sin(aw),
                           front + HC + (dz or {}).get(arm.side, 0.0)])
            return cp - off, R
        return fn

    f0 = c[2]
    # each hand up 80 mm from where it is, then (joint space) to 60 mm out from the band and 80 mm up, then straight in
    yield from together(*(arm.move(arm.pos + [0.0, 0.0, 0.080], arm.rot, 1.0, grip=0.0) for arm in (ctx.left, ctx.right)))
    yield from together(*(arm.move_joints(*at(c, f0 + 0.080, 0.060)(arm), 2.5, seeds=(SEED[arm.side],))
                          for arm in (ctx.left, ctx.right)))
    yield from both(ctx, at(c, f0 + 0.030, 0.015), 1.2)
    yield from both(ctx, at(c, f0, 0.015), 1.0)
    yield from both(ctx, at(c, f0, 0.002), 0.6)
    yield from both(ctx, at(c, f0, -SQUEEZE), 1.5)
    yield from wait(0.3)
    ctx.flags["squeeze_N"] = mtrace(ctx, "squeezed")
    yield from both(ctx, at(c, f0 + 0.005, -SQUEEZE), 0.8)
    yield from wait(0.3)
    mtrace(ctx, "lifted")
    z = f0 + 0.005
    hand_c = c[:2].copy()
    dz = {"left": 0.0, "right": 0.0}

    def steer(phi, centre_gain=0.1):
        s = Scene(ctx)
        n = s.R0[:, 2]
        topc = s.p0 + s.R0 @ np.array([0, 0, g.ALARM_H])
        for side in dz:
            a = yaw + phi + GRIP_A[side]
            h = -(n[0] * math.cos(a) + n[1] * math.sin(a)) / n[2] * g.ALARM_R   # its back's height there vs its middle
            dz[side] = float(np.clip(dz[side] - LEVEL_GAIN * h, -0.010, 0.010))
        if centre_gain:
            hand_c[:] += centre_gain * (b - topc[:2])
            d = hand_c - b
            if np.linalg.norm(d) > 0.012:
                hand_c[:] = b + d * 0.012 / np.linalg.norm(d)
        return topc, math.degrees(math.acos(min(1.0, n[2])))

    # up to 60 mm, then carry it under the ring, its back 15 mm below the ring's face, levelling it as it goes
    z_carry = g.TOP + g.MOUNT_H - g.ALARM_H - 0.015
    for k in range(30):
        steer(0.0, 0.0)
        z = f0 + 0.005 + (0.060 - 0.005) * ik_smooth((k + 1) / 30)
        for arm in (ctx.left, ctx.right):
            arm.aim(*at(hand_c, z, -SQUEEZE, 0.0, dz)(arm))
        yield
    start, n = hand_c.copy(), 100
    p0 = math.remainder(round(yaw / g.ENTRY_STEP) * g.ENTRY_STEP - yaw, 2 * math.pi)  # to the nearest entry position
    ctx.flags["entry_turn_deg"] = round(math.degrees(p0), 2)
    for k in range(n):
        s_ = ik_smooth((k + 1) / n)
        steer(p0 * s_, 0.0)
        hand_c[:] = start + (b - start) * s_
        zz = z + (z_carry - z) * s_
        for arm in (ctx.left, ctx.right):
            arm.aim(*at(hand_c, zz, -SQUEEZE, p0 * s_, dz)(arm))
        yield
    z = z_carry
    mtrace(ctx, "carried")
    for k in range(3):
        yield from wait(0.3)
        s = Scene(ctx)
        topc = s.p0 + s.R0 @ np.array([0, 0, g.ALARM_H])
        err = b - topc[:2]
        mtrace(ctx, f"under the ring {k}")
        if np.linalg.norm(err) < 0.0007 or np.linalg.norm(err) > 0.03:
            break
        hand_c[:] += err
        yield from both(ctx, at(hand_c, z, -SQUEEZE, p0, dz), 0.5)
    s = Scene(ctx)
    ctx.flags["under_ring_mm"] = np.round(((s.p0 + s.R0 @ np.array([0, 0, g.ALARM_H]))[:2] - b) * 1000, 2).tolist()
    # up until its back meets the ring's face, read off its pose: each step the hands level it and keep the middle of
    # its back under the ring's middle
    for k in range(400):
        topc, tilt = steer(p0)
        if topc[2] >= g.TOP + g.MOUNT_H - 0.0003 and tilt < 1.5:
            break
        if topc[2] < g.TOP + g.MOUNT_H - 0.0003:
            z += 0.0001
        for arm in (ctx.left, ctx.right):
            arm.aim(*at(hand_c, z, -SQUEEZE, p0, dz)(arm))
        yield
    mtrace(ctx, "at the ring")
    s = Scene(ctx)
    yaw_start = s.yaw0
    topc = s.p0 + s.R0 @ np.array([0, 0, g.ALARM_H])
    ctx.flags["seat"] = dict(gap_mm=round((g.TOP + g.MOUNT_H - topc[2]) * 1000, 2),
                             off_mm=round(float(np.linalg.norm(topc[:2] - b)) * 1000, 2),
                             tilt_deg=round(math.degrees(math.acos(min(1.0, s.R0[2, 2]))), 2))
    # 1 mm more, then turn it clockwise seen from below (+z) about its axis until it has turned TWIST - 1.5 degrees
    z += 0.001
    phi, turned = p0, 0.0
    for k in range(240):
        s = Scene(ctx)
        turned = math.remainder(s.yaw0 - yaw_start, 2 * math.pi)
        if turned >= TWIST - math.radians(1.5) or phi - p0 >= TWIST + math.radians(10.0):
            break
        if k >= 6:
            phi += math.radians(0.5)
        steer(phi)
        for arm in (ctx.left, ctx.right):
            arm.aim(*at(hand_c, z, -SQUEEZE, phi, dz)(arm))
        yield
    ctx.flags["turned_deg"] = round(math.degrees(turned), 2)
    ctx.flags["hands_turned_deg"] = round(math.degrees(phi - p0), 2)
    mtrace(ctx, "turned")
    yield from wait(0.3)
    # let go: out 20 mm, down 60 mm, then each hand clear of the board before it goes home
    yield from both(ctx, at(hand_c, z, 0.020, phi, dz), 0.6)
    yield from both(ctx, at(hand_c, z - 0.060, 0.020, phi, dz), 1.0)
    ctx.flags["view"] = "mounted"
    pl, Rl = ctx.left.pos.copy(), ctx.left.rot.copy()
    yield from together(ctx.left.move(pl + [0.0, 0.0, -0.050], Rl, 0.8))
    yield from together(ctx.left.move(pl + [-0.200, 0.0, -0.050], Rl, 1.5))
    yield from together(ctx.left.move(pl + [-0.200, 0.0, 0.150], Rl, 1.2))


def ik_smooth(s: float) -> float:
    return s * s * (3 - 2 * s)


def right_script(ctx, variant):
    arm = ctx.right
    while "pin_state" not in ctx.flags:
        yield
    if variant == "mount_only":                      # a check of the mount alone (the old battery stays in)
        return
    yield from open_door(ctx, arm)
    if variant == "open_only":
        return
    yield from set_pin(ctx, True)
    yield from take_old(ctx, arm)
    if variant == "pull_leave":
        return
    if variant in ("latch_empty", "relatch"):
        yield from shut_door(ctx, arm)
        if variant == "latch_empty":
            return
        yield from set_pin(ctx, False)
        yield from open_door(ctx, arm)
        yield from set_pin(ctx, True)
    yield from insert_new(ctx, arm, flip=(variant == "reversed"))
    if variant == "no_latch":
        return
    yield from shut_door(ctx, arm)


def script(ctx, variant: str):
    print(f"MODE variant={variant}", flush=True)
    if variant == "reversed":
        yield from flip_new(ctx)

    def right():
        yield from right_script(ctx, variant)
        ctx.flags["release_pin"] = True
        while not ctx.flags.get("left_free"):
            yield
        if variant in ("full", "relatch", "mount_only"):
            yield from mount(ctx)
        yield from together(ctx.left.go_home(2.0), ctx.right.go_home(2.0))
        ctx.flags["view"] = "side"

    yield from together(right(), pin_script(ctx, ctx.left))
    print("FLAGS", json.dumps({k: v for k, v in ctx.flags.items() if k not in ("pin_state", "pin_want", "release_pin", "left_free", "_hand_view")}), flush=True)
    yield from wait(1.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    bs = " ".join(f"{n[:3]} gap {b['gap'] * 1000:.1f} lift {b['lift'][0] * 1000:.1f}/{b['lift'][1] * 1000:.1f} "
                  f"side {b['side'] * 1000:.1f} F {b['force']:.1f}" for n, b in S.get("batt", {}).items())
    sc = Scene(ctx)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, sc.R0[2, 2]))))
    return (f"tilt {tilt:.2f} hinge {math.degrees(S['hinge']):.2f} max {math.degrees(S['hinge_max']):.1f} tab {S['tab'] * 1000:.2f} "
            f"latched {S['latched']} snapped {S['snapped']} counter {S['on_counter']} mount gap {S['mount_gap'] * 1000:.1f} "
            f"off {S['mount_off'] * 1000:.1f} twist {math.degrees(S['twist']):.1f} seated {S['seated']} locked {S['locked']} "
            f"tier {S['tier']} "
            f"alarm {np.round((sc.p0 - [g.ALARM_X, g.ALARM_Y, g.TOP]) * 1000, 1).tolist()} {bs} events {S['events'][-3:]}")


def close_view(ctx, gen):
    """Recorded runs: once the compartment is open, the top camera moves to a close view of it (from the robot's right
    and above), so the video shows the battery swap. Only the recording changes; the demo reads object poses."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)
    moved = mount_view = side_view = False

    def look(eye, t):
        cam.set_world_poses_from_view(torch.tensor(np.array([eye]), dtype=torch.float32, device=o.device) + o,
                                      torch.tensor(np.array([t]), dtype=torch.float32, device=o.device) + o)

    for _ in gen:
        yield
        if not moved and Scene(ctx).hinge > math.radians(60):
            t = np.array([g.ALARM_X + 0.005, g.ALARM_Y - 0.075, g.TOP + 0.035])
            look(t + np.array([-0.17, -0.10, 0.17]), t)
            moved = True
            ctx.flags["close_view_step"] = ctx.task._STATE["step"]
            print("top camera: close view", flush=True)
        if not mount_view and ctx.flags.get("view") == "mount":
            # the second close view: the alarm and the bracket, from the far right, below the board
            t = np.array([g.BRACKET_X - 0.05, g.BRACKET_Y - 0.05, g.TOP + 0.110])
            look(np.array([g.BRACKET_X + 0.42, g.BRACKET_Y - 0.30, g.TOP + 0.150]), t)
            mount_view = True
            ctx.flags["mount_view_step"] = ctx.task._STATE["step"]
            print("top camera: mount view", flush=True)
        if not side_view and ctx.flags.get("view") == "side":
            # the arms home: a low side view of the alarm hanging on its bracket, from the far right, near table height
            t = np.array([g.BRACKET_X, g.BRACKET_Y, g.TOP + 0.120])
            look(np.array([g.BRACKET_X + 0.30, g.BRACKET_Y - 0.30, g.TOP + 0.050]), t)
            side_view = True
            ctx.flags["side_view_step"] = ctx.task._STATE["step"]
            print("top camera: side view", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=6000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--close-view", action="store_true")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds after the script ends (inf: to the step limit)")
    ap.add_argument("--variant", default="full",
                    choices=["full", "pull_leave", "no_latch", "open_only", "reversed", "latch_empty", "relatch",
                             "no_mount", "mount_only", "nothing"])
    a = ap.parse_args()
    if a.variant == "nothing":
        def scr(ctx):
            print("MODE variant=nothing", flush=True)
            yield from wait(5.0)
    else:
        def scr(ctx):
            if a.close_view and not a.no_record:
                yield from close_view(ctx, script(ctx, a.variant))
            else:
                yield from script(ctx, a.variant)
    res = run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
              log_every=int(os.environ.get("SA_LOG_EVERY", "20")), hold_s=a.hold)
    S = sys.modules["task"]._STATE if "task" in sys.modules else {}
    extra = {"variant": a.variant, "tier": S.get("tier"), "snapped": S.get("snapped"), "latched": S.get("latched"),
             "hinge_deg": round(math.degrees(S.get("hinge", 0.0)), 3),
             "hinge_max_deg": round(math.degrees(S.get("hinge_max", 0.0)), 2), "on_counter": S.get("on_counter"), "mounted": S.get("mounted"),
             "events": S.get("events"), "snap_record": S.get("snap_record"),
             "batt": {n: {k: (np.round(np.asarray(v) * 1000, 2).tolist() if k in ("p", "lift") else
                              (round(v * 1000, 2) if k in ("gap", "side") else v)) for k, v in b.items()}
                      for n, b in S.get("batt", {}).items()}}
    print("EXTRA " + json.dumps(extra, default=str), flush=True)
    (pathlib.Path(a.out) / "extra.json").write_text(json.dumps(extra, indent=2, default=str))
