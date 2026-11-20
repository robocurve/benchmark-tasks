"""Scripted full-score demo (reads object poses and joint angles from the simulator).

Left arm, in order:
1. the door: fingers round the handle bar (a loose cage, closing along the door's normal), it follows the bar's arc
   down to the open stop, the hand turning with the door (and up to 30 degrees more, for reach) by feedback on the
   door's angle; opens and backs off;
2. the rack: pinches the front wire, the hand tipped 55 degrees, and draws it out to its stop with the pan on it;
3. each half in turn, the near one first: one fingertip down its hole and the other outside its rim (the half is
   wider than the open jaws), lifted off the plate, carried and set level on the pan's front part, outside the oven;
4. the rack: pinches the front wire again and pushes it home;
5. a look into the open oven with its wrist camera (the close-up), then the door: cages the bar on its stop and lifts
   it shut along its arc; backs off and goes to its ready pose.
Right arm: grips the Toast/Timer knob's ridge (hand level, pointing at the oven) and turns it clockwise about 55
degrees, then lets go and backs off.

    python demo.py task.py demo_out [--max-steps 3600] [--no-record] [--variant NAME]

Variants (scorer checks), each printing its mode:
- ``plate_in``: a check run: the plate, the halves on it, starts on the pan inside the shut oven (``TO_PLATE_IN=1``,
  read by task.py), and the right arm turns the knob;
- ``plate_in_off``: the same start, the knob never turned;
- ``plate_near``: the demo, but before the knob is turned the right hand pushes the plate 150 mm along the table
  toward the oven's front right corner (to about 30 mm from it);
- ``no_knob``: the demo without the knob turn;
- ``rack_only``: the maker's way, halves straight onto the rack: a check run with the pan set aside on the table at
  the start (``TO_PAN_ASIDE=1``, read by task.py);
- ``door_open``: the demo without shutting the door;
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

DEBUG = bool(os.environ.get("TO_DEBUG"))
GAP_MM = 94.0                       # pads' gap at grip 1.0
TOP = g.TOP
DOOR_Y = 0.270                      # where the left hand takes the handle bar
RACK_Y = 0.240                      # where it pinches the rack's front wire
RACK_TILT = math.radians(55.0)
RACK_OFF = float(os.environ.get("TO_RACK_OFF", "0.010"))   # the wire this far from the hand's tool point, along its
                                                         # approach (the fingertips 3 mm past it, over the rack's wires)
KNOB_TURN = math.radians(float(os.environ.get("TO_KNOB", "55")))
LOOK = os.environ.get("TO_LOOK", "1") == "1"            # a look into the open oven with the left wrist camera


def grip_for(width_m: float) -> float:
    return max(0.0, min(1.0, width_m * 1000 / GAP_MM))


def R_from(a, c) -> np.ndarray:
    """Gripper rotation with approach ``a`` (its z) and closing axis ``c`` (its x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rotz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def down(yaw: float) -> np.ndarray:
    """Pointing straight down, the closing axis at ``yaw`` from +x."""
    return rotz(yaw) @ R_from([0, 0, -1], [1, 0, 0])


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


def door_angle(ctx) -> float:
    return float(ctx.task._STATE["door"])


def rack_pos(ctx) -> float:
    return float(ctx.task._STATE["rack"])


# -- the door ------------------------------------------------------------------------------------------------------------
def door_beta(theta: float) -> float:
    """How much further than the door the hand is turned at a door angle (for reach near the open stop)."""
    return math.radians(-10.0) + math.radians(40.0) * min(1.0, max(0.0, theta / g.DOOR_OPEN))


def door_hand(theta: float, y: float = DOOR_Y, back: float = 0.0):
    """The hand caging the bar at door angle ``theta``: TCP on the bar's axis (``back`` m back along the approach),
    the approach along the door's face toward the hinge, turned ``door_beta`` further."""
    phi = -theta + door_beta(theta)
    a = np.array([-math.sin(phi), 0.0, -math.cos(phi)])
    c = np.array([math.cos(phi), 0.0, -math.sin(phi)])
    bx, bz = g.bar_centre(theta)
    return np.array([bx, y, TOP + bz]) - back * a, R_from(a, c)


def door_move(ctx, arm, target: float, lead=math.radians(14), cage=0.016, rate=math.radians(40)):
    """Carry the door to ``target`` along its arc: the command moves at ``rate`` but never more than ``lead`` ahead of
    the measured angle (the arm gives about 3 mm per newton, so the hand must lead the bar to pull it)."""
    n = 0
    cmd = door_angle(ctx)
    while n < 400:
        th = door_angle(ctx)
        err = target - th
        if abs(err) < math.radians(1.0):
            break
        sgn = 1.0 if err > 0 else -1.0
        cmd = cmd + sgn * rate * 0.05
        cmd = th + sgn * min(abs(cmd - th), lead) if sgn * (cmd - th) > 0 else cmd
        cmd = min(cmd, target + sgn * math.radians(3)) if sgn > 0 else max(cmd, target + sgn * math.radians(3))
        if DEBUG and n % 10 == 0:
            tp, _ = tcp_actual(arm)
            want, _ = door_hand(cmd)
            print(f"DOOR n {n} th {math.degrees(th):.1f} cmd {math.degrees(cmd):.1f} tcp_err_mm "
                  f"{np.round((want - tp) * 1000, 1).tolist()}", flush=True)
        p, R = door_hand(cmd)
        arm.grip = grip_for(cage)
        arm.aim(p, R)
        yield
        n += 1
    ctx.flags.setdefault("door_log", []).append((round(math.degrees(target), 1), round(math.degrees(door_angle(ctx)),
                                                                                           1), n))


def open_door(ctx, arm):
    p, R = door_hand(0.0, back=0.05)
    yield from arm.move_joints(p, R, 2.5, grip=grip_for(0.030))
    p, R = door_hand(0.0)
    yield from arm.move(p, R, 1.0)
    yield from arm.set_grip(grip_for(0.016), 0.4)
    yield from door_move(ctx, arm, g.DOOR_OPEN)
    yield from wait(0.5)
    th = door_angle(ctx)
    p, R = door_hand(th, back=0.05)
    yield from arm.set_grip(grip_for(0.030), 0.3)
    yield from arm.move(p, R, 0.8)


def close_door(ctx, arm):
    th = door_angle(ctx)
    p, R = door_hand(th, back=0.05)
    yield from arm.move_joints(p, R, 2.0, grip=grip_for(0.030))
    p, R = door_hand(th)
    yield from arm.move(p, R, 0.8)
    yield from arm.set_grip(grip_for(0.016), 0.4)
    yield from door_move(ctx, arm, 0.0)
    yield from wait(0.5)
    p, R = door_hand(0.0, back=0.06)
    yield from arm.set_grip(grip_for(0.030), 0.3)
    yield from arm.move(p, R, 0.8)


# -- the rack ------------------------------------------------------------------------------------------------------------
def rack_hand(x_wire: float, back: float = 0.0, y: float = RACK_Y):
    a = np.array([math.sin(RACK_TILT), 0.0, -math.cos(RACK_TILT)])
    c = np.array([math.cos(RACK_TILT), 0.0, math.sin(RACK_TILT)])
    wire = np.array([x_wire, y, TOP + g.RACK_Z])
    return wire - RACK_OFF * a - back * a, R_from(a, c)


def move_rack(ctx, arm, target: float):
    """Pinch the rack's front wire and slide the rack to ``target`` (0 home, -RACK_OUT out)."""
    for attempt in range(3):
        x0 = g.RACK_X0 + rack_pos(ctx)
        y = RACK_Y - 0.015 * attempt                # a retry takes the wire a little further along
        p, R = rack_hand(x0, back=0.05, y=y)
        if attempt == 0:
            yield from arm.move_joints(p, R, 2.0, grip=grip_for(0.014))
        else:
            yield from arm.move(p, R, 0.8, grip=grip_for(0.014))
        p, R = rack_hand(x0, y=y)
        yield from arm.move(p, R, 0.8)
        yield from wait(0.3)
        tp, _ = tcp_actual(arm)
        err = p - tp
        if DEBUG:
            print(f"RACK at wire (try {attempt}): want {np.round((p - [0, 0, TOP]) * 1000, 1).tolist()} err_mm "
                  f"{np.round(err * 1000, 1).tolist()} rack {rack_pos(ctx) * 1000:.1f}", flush=True)
        if np.linalg.norm(err) < 0.004:
            break
        ctx.flags.setdefault("rack_retry", []).append(np.round(err * 1000, 1).tolist())
    yield from arm.set_grip(0.0, 0.4)
    overshoot = 0.004 * (1 if target > rack_pos(ctx) else -1)
    p1, _ = rack_hand(g.RACK_X0 + target + overshoot, y=y)
    yield from arm.move(p1, R, 2.5)
    yield from wait(0.3)
    ctx.flags.setdefault("rack_log", []).append(round(rack_pos(ctx) * 1000, 1))
    yield from arm.set_grip(grip_for(0.014), 0.3)
    p2, _ = rack_hand(g.RACK_X0 + rack_pos(ctx), back=0.06, y=y)
    yield from arm.move(p2, R, 0.8)


# -- the halves ----------------------------------------------------------------------------------------------------------
def half_pose(ctx, name):
    st = ctx.states()[name]
    return np.asarray(st.pos, float), ctx.task._rotation(st.quat_xyzw)


HOLE_OPEN = 0.052                   # jaw opening for the hole grip: inner finger centred in the hole, outer clear
FINGER_T = 0.0063                   # fingertip thickness
TIP_IN = float(os.environ.get("TO_TIPIN", "0.005"))     # fingertips this far above the half's underside


def hole_grip(centre, Rh, yaw: float = -math.pi / 2):
    """The hand gripping a half through its hole: approach along the half's axis, closing along the half's plane at
    ``yaw`` (world), one fingertip centred in the hole and the other outside the rim; fingertips TIP_IN above the
    underside. Returns (TCP, rotation)."""
    from demokit import PAD_TIP

    a = -Rh[:, 2]
    c = rotz(yaw) @ np.array([1.0, 0.0, 0.0])
    R = R_from(a, c)
    xg = R[:, 0]
    # the inner finger is the one at TCP + (open/2) x_g; its centre sits on the hole's axis
    off = HOLE_OPEN / 2 + FINGER_T / 2
    tcp = np.asarray(centre, float) - off * xg + Rh[:, 2] * (TIP_IN + PAD_TIP)
    return tcp, R


def settle_to(arm, pos, rot, tol=0.0004, rounds=4, seconds=0.6):
    """Move to ``pos`` and correct the command by the measured tool-point error."""
    target = np.asarray(pos, float)
    cmd = target.copy()
    yield from arm.move(cmd, rot, seconds)
    err = np.zeros(3)
    for _ in range(rounds):
        yield from wait(0.25)
        err = target - tcp_actual(arm)[0]
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
        yield from arm.move(cmd, rot, 0.35)
    return err


def carry_half(ctx, arm, name, place_xy, surface_place):
    """Pick a half off the plate, gripped through its hole, and set it level on the pan (or the rack)."""
    c, Rh = half_pose(ctx, name)
    p, R = hole_grip(c, Rh)
    a = R[:, 2]
    yield from arm.move_joints(p - 0.08 * a, R, 2.5, grip=grip_for(HOLE_OPEN))
    yield from settle_to(arm, p - 0.025 * a, R)
    # re-aim at the half (it may have settled) and go down into the hole
    c, Rh = half_pose(ctx, name)
    p, R = hole_grip(c, Rh)
    err = yield from settle_to(arm, p - 0.008 * a, R)
    yield from arm.move(arm.pos + 0.008 * a, R, 0.5)
    tp, _ = tcp_actual(arm)
    if DEBUG:
        print(f"HALF {name} down: err_mm {np.round((p - tp) * 1000, 1).tolist()}", flush=True)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0, 0, 0.10], R, 1.2)
    # where the half sits in the hand (measured), to place its centre exactly
    tp, tR = tcp_actual(arm)
    c1, Rh1 = half_pose(ctx, name)
    rel = tR.T @ (c1 - tp)
    ctx.flags.setdefault("half_rel_mm", []).append((name, np.round(rel * 1000, 1).tolist()))
    if DEBUG:
        print(f"HALF {name} lifted: rel_mm {np.round(rel * 1000, 1).tolist()} tilt "
              f"{math.degrees(math.acos(min(1, Rh1[2, 2]))):.1f}", flush=True)
    # place: the half level (its axis up), its underside centre 2 mm over the surface at place_xy
    Rp = down(-math.pi / 2)
    target_c = np.array([place_xy[0], place_xy[1], surface_place + 0.002])
    p_place = target_c - Rp @ rel
    yield from arm.move_joints(p_place + [0, 0, 0.08], Rp, 3.0)
    if DEBUG:
        tp, tR = tcp_actual(arm)
        c2, _ = half_pose(ctx, name)
        print(f"HALF {name} over the pan: rel_mm {np.round(tR.T @ (c2 - tp) * 1000, 1).tolist()}", flush=True)
    yield from arm.move(p_place + [0, 0, 0.015], Rp, 1.0)
    yield from arm.move(p_place, Rp, 0.6)
    yield from arm.set_grip(grip_for(HOLE_OPEN), 0.5)
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0, 0, 0.08], Rp, 1.0)


# -- the knob ------------------------------------------------------------------------------------------------------------
def knob_hand(roll: float, back: float = 0.0):
    """Hand level, pointing +x at the knob's ridge, closing across it (vertical at OFF), turned ``roll`` about x."""
    from demokit import turn

    a = np.array([1.0, 0.0, 0.0])
    R = turn([1, 0, 0], roll) @ R_from(a, [0, 0, 1])
    tip_x = g.OVEN_X - (g.KNOB_DEPTH - g.RIDGE[2]) - 0.001
    from demokit import PAD_TIP

    return np.array([tip_x - PAD_TIP - back, g.KNOB_Y, TOP + g.KNOB_Z[0]]), R


def knob_chain(arm, target: float, step=math.radians(2.5)):
    """Joint angles for the hand on the knob at rolls 0 .. target + 8 degrees, solved as one continuous chain from a
    seed that keeps every pose exact (the wrist's roll range decides which branch can turn that far)."""
    from yam_arena import ik

    from demokit import Arm

    rolls = np.arange(0.0, target + math.radians(12) + 1e-9, step)
    for seed in [arm.q] + [np.array(q, float) for q in Arm.SEEDS]:
        chain, ok, q = [], True, seed
        for r in rolls:
            p, R = knob_hand(r)
            q, err, rerr = ik.solve(p, ik.matrix_to_quat_xyzw(R), q_init=q, base_pos=arm.base, iters=200)
            if err > 0.0007 or rerr > math.radians(0.7):
                ok = False
                break
            chain.append(q)
        if ok:
            return rolls, np.array(chain)
    raise RuntimeError("no knob chain")


def turn_knob(ctx, arm, target: float):
    """Grip the ridge and roll the hand clockwise (as the robot sees it), leading the knob's measured angle by up to 8
    degrees, in joint space along one IK chain; then let go and back off."""
    from yam_arena import ik

    rolls, chain = knob_chain(arm, target)
    p0, R0 = knob_hand(0.0, back=0.05)
    q_back, _, _ = ik.solve(p0, ik.matrix_to_quat_xyzw(R0), q_init=chain[0], base_pos=arm.base, iters=200)
    yield from move_q(arm, q_back, 2.5, grip=grip_for(0.026))
    yield from move_q(arm, chain[0], 1.0)
    yield from arm.set_grip(0.0, 0.5)
    cmd = 0.0
    n = 0
    while n < 300:
        k = float(ctx.task._STATE["timer"])
        if k >= target:
            break
        cmd = min(cmd + math.radians(1.5), k + math.radians(12), rolls[-1])
        i = min(len(rolls) - 1, int(cmd / (rolls[1] - rolls[0])))
        f = cmd / (rolls[1] - rolls[0]) - i if i < len(rolls) - 1 else 0.0
        arm.q = chain[i] if i == len(rolls) - 1 else chain[i] * (1 - f) + chain[i + 1] * f
        yield
        n += 1
        if DEBUG and n % 5 == 0:
            from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

            from scenekit import arm_joints, env
            qa = arm_joints(env(), arm.side, ARM_JOINTS)
            qf = arm_joints(env(), arm.side, FINGER_JOINTS)
            print(f"KNOB n {n} knob {math.degrees(k):.1f} cmd {math.degrees(cmd):.1f} q6 cmd "
                  f"{math.degrees(arm.q[5]):.1f} act {math.degrees(qa[5]):.1f} q5 {math.degrees(arm.q[4]):.1f}/"
                  f"{math.degrees(qa[4]):.1f} fingers_mm {np.round(qf * 1000, 1).tolist()}", flush=True)
    ctx.flags["knob_deg"] = round(math.degrees(float(ctx.task._STATE["timer"])), 1)
    yield from arm.set_grip(grip_for(0.026), 0.4)
    q_now = arm.q.copy()
    p1, R1 = ik.tcp_pose(q_now, arm.base)
    q_off, _, _ = ik.solve(p1 - [0.05, 0, 0], ik.matrix_to_quat_xyzw(R1), q_init=q_now, base_pos=arm.base, iters=200)
    yield from move_q(arm, q_off, 0.8)


def move_q(arm, q1, seconds: float = 2.0, grip: float | None = None):
    """Generator: a smooth joint-space move to the joint angles q1."""
    from yam_arena import ik

    q0, q1, g0 = arm.q.copy(), np.asarray(q1, float), arm.grip
    g1 = g0 if grip is None else grip
    n = max(1, round(seconds * 20))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        arm.grip = g0 + s * (g1 - g0)
        arm.q = q0 + s * (q1 - q0)
        yield
    arm.pos, arm.rot = ik.tcp_pose(arm.q, arm.base)


# -- scripts -------------------------------------------------------------------------------------------------------------
PLACE_A = (float(os.environ.get("TO_PX", "0.343")), 0.275)      # half centres on the drawn-out pan
PLACE_B = (PLACE_A[0], 0.162)


def pan_floor_out(ctx) -> float:
    return float(ctx.states()["pan"].pos[2]) + g.PAN_T


def rack_top(ctx) -> float:
    return TOP + g.RACK_Z + g.WIRE_R


def left_script(ctx, variant):
    arm = ctx.left
    yield from open_door(ctx, arm)
    yield from arm.move(arm.pos + [-0.05, 0, 0.08], arm.rot, 0.8)
    yield from move_rack(ctx, arm, -g.RACK_OUT)
    yield from place_halves(ctx, arm, rack_top if variant == "rack_only" else pan_floor_out)
    if variant == "plate_in":
        yield from carry_plate_in(ctx, arm)
    yield from move_rack(ctx, arm, 0.0)
    if LOOK:
        # the left wrist camera about 30 cm in front of the open oven, looking in at the halves on the pan (the
        # close-up)
        R = R_from([math.cos(math.radians(11)), 0.0, -math.sin(math.radians(11))], [0.0, -1.0, 0.0])
        yield from arm.move(arm.pos + [-0.06, 0.0, 0.06], arm.rot, 0.8)
        yield from arm.move_joints(np.array([0.23, g.CAV_YC, TOP + 0.15]), R, 2.0)
        yield from wait(0.5)
        ctx.flags["inside_look_step"] = int(ctx.task._STATE["step"])
        yield from wait(1.0)
    if variant != "door_open":
        yield from close_door(ctx, arm)
    ctx.flags["left_done"] = True
    yield from arm.move(arm.pos + [-0.13, 0.0, 0.03], arm.rot, 1.2)        # back off, clear of the handle
    yield from arm.go_home(2.0)


def right_script(ctx, variant):
    arm = ctx.right
    while not ctx.flags.get("left_done"):
        yield
    if variant == "plate_near":
        yield from push_plate(ctx, arm, 0.15)
    if variant not in ("no_knob",):
        yield from turn_knob(ctx, arm, KNOB_TURN)
    yield from arm.go_home(2.0)
    ctx.flags["right_done"] = True


def carry_plate_in(ctx, arm):
    """Variant: after the halves, the empty plate is pinched by its rim (its +y edge), lifted, turned a quarter turn
    and laid on the halves on the drawn-out pan, so that it goes into the oven with them."""
    pl = ctx.states()["plate"]
    c = np.asarray(pl.pos, float)
    rim = c + np.array([0.0, g.PLATE_R - 0.004, 0.0])
    R = down(-math.pi / 2)                        # closing along y, across the rim
    p = np.array([rim[0], rim[1], c[2] + 0.014 + 0.013])
    yield from arm.move_joints(p + [0, 0, 0.08], R, 2.5, grip=grip_for(0.040))
    yield from arm.move(p, R, 1.2)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.3)
    yield from arm.move(p + [0, 0, 0.15], R, 1.5)
    tp, tR = tcp_actual(arm)
    c1 = np.asarray(ctx.states()["plate"].pos, float)
    rel = tR.T @ (c1 - tp)
    ctx.flags["plate_rel_mm"] = np.round(rel * 1000, 1).tolist()
    top = max(float(ctx.states()[n].pos[2]) for n in ("half_a", "half_b")) + g.HALF_T
    Rp = down(0.0)                                # turned so the grip point is toward the robot
    target = np.array([0.366, g.CAV_YC, top + 0.008])
    p_place = target - Rp @ rel
    yield from arm.move_joints(p_place + [0, 0, 0.06], Rp, 3.0)
    yield from arm.move(p_place, Rp, 1.0)
    yield from arm.set_grip(grip_for(0.040), 0.4)
    yield from wait(0.6)
    yield from arm.move(arm.pos + [-0.03, 0, 0.07], Rp, 0.8)


def place_halves(ctx, arm, surface):
    dx = rack_pos(ctx) + g.RACK_OUT                  # the rack short of its stop by this much (0 normally)
    yield from carry_half(ctx, arm, "half_a", (PLACE_A[0] + dx, PLACE_A[1]), surface(ctx))
    yield from carry_half(ctx, arm, "half_b", (PLACE_B[0] + dx, PLACE_B[1]), surface(ctx))


def push_plate(ctx, arm, dist: float):
    """Variant: the closed fingertips push the plate along the table by its rim, ``dist`` toward the oven's front
    right corner (diagonally, +x +y)."""
    c = np.asarray(ctx.states()["plate"].pos, float)
    u = np.array([math.sqrt(0.5), math.sqrt(0.5), 0.0])
    R = down(math.pi / 4)
    p = c - (g.PLATE_R + 0.010) * u
    p[2] = c[2] + 0.014 + 0.013
    yield from arm.move_joints(p + [0, 0, 0.06], R, 2.5, grip=0.0)
    yield from arm.move(p, R, 1.0)
    yield from arm.move(p + dist * u, R, 3.5)
    yield from arm.move(arm.pos - 0.02 * u + [0, 0, 0.08], R, 1.0)


def script(ctx, variant: str):
    print(f"MODE variant={variant}", flush=True)
    yield from together(left_script(ctx, variant), right_script(ctx, variant))
    print("FLAGS", json.dumps({k: v for k, v in ctx.flags.items()}, default=str), flush=True)
    yield from wait(1.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return (f"door {math.degrees(S['door']):.1f} rack {S['rack'] * 1000:.0f} timer {math.degrees(S['timer']):.1f} "
            f"on {S['on']} plate_gap {S['plate_gap'] * 1000:.0f} min {S['plate_gap_min'] * 1000:.0f} in {S['plate_in']} "
            f"halves {[(n, h['inside'], h['resting'], h['low_mm'], h['pos']) for n, h in S['halves'].items()]} "
            f"gaps {S.get('gaps')} tier {S['tier']} rung {S['rung']} slip {S['pan_slip'] * 1000:.1f} why {S['why']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--variant", default="full",
                    choices=["full", "plate_in", "plate_in_off", "plate_near", "no_knob", "rack_only", "door_open", "nothing",
                             "knob"])
    a = ap.parse_args()
    if a.variant in ("plate_in", "plate_in_off"):
        def scr(ctx):
            # check runs: the plate, the bagel's halves on it, starts on the pan inside the oven (TO_PLATE_IN=1)
            print(f"MODE variant={a.variant} plate_in_start={os.environ.get('TO_PLATE_IN')}", flush=True)
            if a.variant == "plate_in":
                yield from turn_knob(ctx, ctx.right, KNOB_TURN)
                yield from ctx.right.go_home(2.0)
            yield from wait(2.0)
    elif a.variant == "knob":
        def scr(ctx):
            print("MODE variant=knob", flush=True)
            yield from turn_knob(ctx, ctx.right, KNOB_TURN)
            yield from wait(1.0)
    elif a.variant == "nothing":
        def scr(ctx):
            print("MODE variant=nothing", flush=True)
            yield from wait(5.0)
    else:
        def scr(ctx):
            yield from script(ctx, a.variant)
    res = run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
              log_every=int(os.environ.get("TO_LOG_EVERY", "20")), hold_s=float(os.environ.get("TO_HOLD", "3.0")))
    S = sys.modules["task"]._STATE if "task" in sys.modules else {}
    extra = {"variant": a.variant, "door_deg": round(math.degrees(S.get("door", 0)), 2),
             "door_gap_mm": round(g.DOOR_H * math.sin(S.get("door", 0)) * 1000, 2),
             "rack_mm": round(S.get("rack", 0) * 1000, 1), "timer_deg": round(math.degrees(S.get("timer", 0)), 1),
             "on_step": S.get("on_step"), "plate_gap_min_mm": round(S.get("plate_gap_min", 0) * 1000, 1),
             "plate_gap_on_min_mm": round(S.get("plate_gap_on_min", 0) * 1000, 1),
             "plate_moved_mm": round(S.get("plate_moved", 0) * 1000, 1), "pan_slip_mm": round(S.get("pan_slip", 0) * 1000, 1),
             "halves": S.get("halves"), "gaps_mm": S.get("gaps"), "zero_why": S.get("zero_why"), "why": S.get("why")}
    print("EXTRA " + json.dumps(extra, default=str), flush=True)
    (pathlib.Path(a.out) / "extra.json").write_text(json.dumps(extra, indent=2, default=str))
