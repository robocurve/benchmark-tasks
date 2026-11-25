"""The scripted demo, closed loop on demokit: it reads the cases' poses from the simulator (privileged) and the arms'
own joints. Modes (environment variable TASK_DEMO, or demokit's --mode):

- ``full``: slide C1 left and C2 right, take N by its hand holes with both hands, set it on the shelf with its centre
  just over the edge, let go and push it home;
- ``shove``: no room made; N is carried straight into the 16 cm gap and pushed home with both hands;
- ``drop``: room made, then N let go with its centre still in front of the shelf's edge;
- ``nothing``: the arms stay still.
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER
from yam_arena.urdf import axis_angle_matrix

import demokit as d
import layout as L

GRIPPER_K = GRIPPER["stiffness"]
HX, HY, HZ = (s / 2 for s in L.CASE)
TIP = 0.013                                                     # fingertips past the TCP
LOG = bool(os.environ.get("SHELF_DEBUG"))

# sliding a soup case sideways
PUSH_H = 0.39                                                   # the pushing hand's TCP height above the table
BODY = 0.051                                                    # the gripper body's side from the TCP, across the fingers
BODY_MID = 0.057                                                # the pushing hand's TCP this far ahead of the case's centre
PUSH_YAW = 14.0                                                 # turned toward the case against the wrist's give
PUSH = {"left": d.level(0), "right": d.level(0)}                # pointing +x, fingers across y, wrist camera up
SLIDE = {"left": d.level(PUSH_YAW), "right": d.level(-PUSH_YAW)}

# N pinched at its hand holes: lower finger in the hole, upper finger on the box's top
PINCH_TILT = 86.0                                               # each hand points straight inward, 4 degrees down
PINCH_IN = 0.025                                                # the fingertips this far in past the box's side
PINCH_BACK = 0.039                                              # the pinch this far toward the near end from N's centre
PINCH_F = 40.0                                                  # N per finger
PINCH_OPEN = -0.035                                             # jaws on the way in: inner faces 6.6 cm apart
STRIP = L.CASE[2] / 2 - L.HOLE_TOP                              # the wall between the hole's top edge and the box's top


def tcp_force(ctx, arm):
    """The force the arm's position drives push its TCP with (N, world), from the joint errors and the drives'
    stiffness through the position Jacobian. An estimate, for the log."""
    from yam_arena.constants import SHOULDER, WRIST

    q = ctx.observed(arm.side)[:6]
    k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    tau = k * (arm.cmd[:6] - q)
    J = np.zeros((3, 6))
    p0 = d.ik.tcp_pose(q, arm.base)[0]
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-4
        J[:, i] = (d.ik.tcp_pose(q + dq, arm.base)[0] - p0) / 1e-4
    return np.linalg.lstsq(J.T, tau, rcond=None)[0]


def _roll(g, deg):
    return d._as_grip(g.rot @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(deg)))

def _s(ctx, name):
    s = ctx.state(name)
    return np.asarray(s.pos, float), np.asarray(s.rotation(), float)

def _corners(ctx, name):
    pos, R = _s(ctx, name)
    return pos + (np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * [HX, HY, HZ]) @ R.T

def _face_y(ctx, name, sign):
    """The y of a case's side face toward the gap: its right face (sign +1, pushed left) or its left face (sign -1)."""
    c = _corners(ctx, name)
    return float(c[:, 1].min()) if sign > 0 else float(c[:, 1].max())

def _yaw(ctx, name):
    R = _s(ctx, name)[1]
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))

def _tilt(ctx, name):
    R = _s(ctx, name)[1]
    return math.degrees(math.acos(min(1.0, R[2, 2])))

def _log(text):
    print(f"[demo] {text}", flush=True)

def park(ctx, arm, t=2.0):
    """Back to a pose near ready, out of the way."""
    side = 1.0 if arm.side == "left" else -1.0
    yield from arm.move((0.22, side * 0.30, L.H(0.19)), grip=d.down(0), finger=FINGER_OPEN, t=t, label="park")

def _drive(arm, point):
    """One control step: command the TCP to ``point`` with the arm's grip."""
    mm, deg = arm._solve(np.asarray(point, float), arm.grip.rot)
    arm.worst = (max(arm.worst[0], mm), max(arm.worst[1], deg))

def push_face(ctx, arm, name, sign, goal, x0, x1, z, rate=0.025, timeout=6.0):
    """Push a case's side face to ``goal`` (y) with the side of the hand, closed loop on the face: the command moves
    across until the face is there, at up to ``rate`` m/s, while the hand drifts from x0 to x1 so that its friction
    draws the case toward the shelf's front, never toward its back."""
    c = np.array(arm.at(), float)
    n = round(timeout * d.CONTROL_HZ)
    face0 = _face_y(ctx, name, sign)
    for k in range(n):
        t = k * d.DT
        want = face0 + sign * min(abs(goal - face0), rate * t)
        err = want - _face_y(ctx, name, sign)
        c[1] += float(np.clip(0.25 * err, -d.SERVO_STEP, d.SERVO_STEP))
        c[0] = x0 + (x1 - x0) * min(1.0, t / max(0.1, abs(goal - face0) / rate))
        c[2] = z
        _drive(arm, c)
        yield
        if sign * (_face_y(ctx, name, sign) - goal) > -0.0015:
            break

def slide_aside(ctx, arm, name: str, sign: float, dist: float):
    """Slide a soup case sideways along the shelf by ``dist``: the closed hand, level, pointing forward and turned a few
    degrees toward the case (the wrist gives as much under the load), enters the gap at the case's mid-height and
    pushes the case's side face away from the gap (sign +1: toward +y) with the side of the hand, closed loop on the
    face, while drifting toward the robot so its friction draws the case forward, not back."""
    z = L.H(PUSH_H)
    face0 = _face_y(ctx, name, sign)
    pos0 = _s(ctx, name)[0]
    g = SLIDE[arm.side]
    tip_side = BODY
    x_tcp = pos0[0] + BODY_MID
    y_in = 0.0
    yield from arm.move((0.30, y_in, z), grip=g, finger=FINGER_CLOSED, t=2.0, label=f"{name}: before the gap")
    yield from arm.move((x_tcp, y_in, z), t=1.6, label=f"{name}: into the gap")
    yield from arm.servo((x_tcp, y_in, z), timeout=0.8, label=f"{name}: into the gap")
    start = ctx.t
    goal = face0 + sign * dist
    yield from arm.move((x_tcp, face0 - sign * (tip_side + 0.006), z), t=0.8, label=f"{name}: to the face")
    yield from push_face(ctx, arm, name, sign, goal, x_tcp, x_tcp - 0.015, z)
    _log(f"{name} pushed {sign * (_face_y(ctx, name, sign) - face0) * 100:.1f} cm in {ctx.t - start:.1f} s; "
         f"yaw {_yaw(ctx, name):.1f} deg; pos {np.round(_s(ctx, name)[0], 3)}")
    off = arm.tcp() - np.array([0.0, sign * 0.015, 0.0])          # from where the hand is, not where it was told
    _drive(arm, arm.tcp())
    yield
    yield from arm.move(off, t=0.5, label=f"{name}: off")
    yield from arm.move((0.30, off[1], z), t=1.4, label=f"{name}: out of the gap")

def _q_squeezed(arm, point, inward, depth):
    """Joint targets for the TCP at ``point`` with the grip, then joints 1 to 3 alone turned as if to carry the TCP
    ``depth`` further along ``inward``: the shoulder presses the hand into the case while the wrist keeps the hand's
    attitude, so both fingertips stay on the face."""
    rot = getattr(arm, "hold_rot", None)
    mm, deg = arm._solve(np.asarray(point, float), arm.grip.rot if rot is None else rot)
    arm.worst = (max(arm.worst[0], mm), max(arm.worst[1], deg))
    q = arm.q.copy()
    if depth:
        p0 = d.ik.tcp_pose(q, arm.base)[0]
        J = np.zeros((3, 3))
        for i in range(3):
            dq = np.zeros(6)
            dq[i] = 1e-4
            J[:, i] = (d.ik.tcp_pose(q + dq, arm.base)[0] - p0) / 1e-4
        q[:3] += np.linalg.lstsq(J, np.asarray(inward, float) * depth, rcond=None)[0]
    arm.cmd = np.r_[q, float(np.clip(arm.finger / FINGER_OPEN, 0.0, 1.0))]

def hold_path(ctx, path, t, depth=None, dy=0.0):
    """Both hands on N's sides along a path of hand points (x, z), a function of the fraction done, each TCP's y at its
    contact (shifted by ``dy`` over the move) and pressed in by the shoulder ``depth`` (a function of the fraction, or
    the current depth). Closed loop in x and z on the observed TCPs, so neither the squeeze nor the load's sag carries
    N off its path."""
    arms = (ctx.left, ctx.right)
    H_ = ctx.hold
    y0 = {a.side: H_[a.side]["y"] for a in arms}
    n = max(1, round(t * d.CONTROL_HZ))
    level = getattr(ctx, "level", None)
    for k in range(1, n + 16):
        f = min(1.0, k / n)
        x, z = path(d.ik.smoothstep(f))
        ctx.hold_nom = (x, z)
        if level is not None:
            # keep N level: turn both hands about the world y axis against its pitch, and raise the hand on the side its
            # roll lowers (read from N's pose: privileged)
            R = _s(ctx, "N")[1]
            pitch = math.atan2(R[0, 2], R[2, 2])
            roll = math.atan2(R[1, 2], R[2, 2])
            level["pitch"] = float(np.clip(level["pitch"] - np.clip(0.25 * pitch, -0.01, 0.01), -0.45, 0.45))
            level["roll"] = float(np.clip(level["roll"] + np.clip(0.02 * roll, -0.001, 0.001), -0.03, 0.03))
            Ry = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), level["pitch"])
        for a in arms:
            h = H_[a.side]
            if level is not None:
                a.hold_rot = Ry @ level["rot0"][a.side]
            obs = a.tcp()
            h["trim"] = np.clip(h["trim"] + np.clip(0.2 * (np.array([x, z]) - obs[[0, 2]]), -0.003, 0.003), -0.06, 0.06)
            h["y"] = y0[a.side] + dy * d.ik.smoothstep(f)
            if depth is not None:
                h["depth"] = depth(f) if callable(depth) else depth
            inward = np.array([0.0, -1.0 if a.side == "left" else 1.0, 0.0])
            dzr = 0.0 if level is None else (level["roll"] if a.side == "left" else -level["roll"])
            _q_squeezed(a, (x + h["trim"][0], h["y"], z + h["trim"][1] + dzr), inward, h["depth"])
        yield
    if LOG:
        pos, _ = _s(ctx, "N")
        _log(f"N at {np.round(pos, 3)}, tilt {_tilt(ctx, 'N'):.1f} deg, yaw {_yaw(ctx, 'N'):.1f}, hands "
             f"{np.round(ctx.left.tcp(), 3)} {np.round(ctx.right.tcp(), 3)}, force {np.round(tcp_force(ctx, ctx.left), 1)}")

def carry(ctx, points, speed=0.10):
    """Both holding hands through (hands' x, height of the hands above the table) points, keeping their squeeze."""
    for x, h in points:
        x0, z0 = ctx.hold_nom
        z1 = L.H(h)
        dist = math.hypot(x - x0, z1 - z0)
        yield from hold_path(ctx, lambda f, x0=x0, z0=z0, x=x, z1=z1: (x0 + (x - x0) * f, z0 + (z1 - z0) * f),
                             max(0.6, dist / speed))

def front_in_band(ctx, name, y_lo, y_hi):
    """The least x of a case's outline (seen from above) between y_lo and y_hi: where a hand in that band meets it."""
    c = _corners(ctx, name)
    c = c[c[:, 2] < c[:, 2].mean()][:, :2]              # the four bottom corners
    hull = c[np.argsort(np.arctan2(*(c - c.mean(0)).T[::-1]))]
    best = math.inf
    for i in range(4):
        p, q = hull[i], hull[(i + 1) % 4]
        for t in np.linspace(0.0, 1.0, 41):
            x, y = p + t * (q - p)
            if y_lo <= y <= y_hi:
                best = min(best, float(x))
    return best

def slot_y(ctx):
    """The middle of the gap between C1 and C2, read from their faces."""
    return 0.5 * (_face_y(ctx, "C1", +1) + _face_y(ctx, "C2", -1))

def push_flush(ctx, arms, ys):
    """Closed hands, level, pointing +x, against N's near face, push it in until that face is on the front-edge line."""
    z = L.H(PUSH_H)
    near = float(_corners(ctx, "N")[:, 0].min())
    yield from ctx.together(*[a.move((min(0.16, near - 0.08), y, z), grip=PUSH[a.side], finger=FINGER_CLOSED, t=2.5,
                                     label="flush: well behind N") for a, y in zip(arms, ys)])
    yield from ctx.together(*[a.move((near - TIP - 0.03, y, z), t=1.2, label="flush: behind N") for a, y in zip(arms, ys)])
    goal = L.FRONT_X

    def mover(a, y):
        def tgt(t):
            return np.array([min(goal, near + 0.04 * t), y, z])

        def rd():
            return np.array([float(_corners(ctx, "N")[:, 0].min()), y, z])

        return a.servo(tgt, tol=0.002, timeout=(goal - near) / 0.04 + 3.0, gain=0.25, read=rd, label="flush: push")

    yield from ctx.together(*[mover(a, y) for a, y in zip(arms, ys)])
    _log(f"N near face at x {float(_corners(ctx, 'N')[:, 0].min()):.4f}, pos {np.round(_s(ctx, 'N')[0], 3)}, yaw {_yaw(ctx, 'N'):.1f}")
    yield from ctx.together(*[a.move(a.at() - [0.06, 0.0, 0.0], t=0.8, label="flush: back") for a in arms])

def to_ready(arm, t=2.0):
    """Back to the ready pose in joint space (no IK), fingers open."""
    from yam_arena.constants import READY

    q0 = arm.q.copy()
    q1 = np.array(READY, float)
    n = max(1, round(t * d.CONTROL_HZ))
    for k in range(1, n + 1):
        f = d.ik.smoothstep(k / n)
        arm.q = q0 + (q1 - q0) * f
        arm.cmd = np.r_[arm.q, 1.0]
        yield
    arm.finger, arm.grip, arm.trim = FINGER_OPEN, d.down(0), np.zeros(3)
    arm.hold_rot = None

def home(ctx):
    l, r = ctx.left, ctx.right
    yield from ctx.together(to_ready(l), to_ready(r))
    yield from ctx.together(l.wait(0.2), r.wait(0.2))

def _pinch_grip(side):
    s = 1.0 if side == "left" else -1.0
    best = None
    for rl in range(-180, 180, 15):
        g = _roll(d.tilted(PINCH_TILT, -s * 90.0), float(rl))
        cam = -g.rot[:, 1]
        score = abs(g.rot[2, 0]) - cam[0]        # the jaws closing up and down, the wrist camera toward the robot
        if best is None or score > best[0]:
            best = (score, g)
    return best[1]

PINCH = {"left": _pinch_grip("left"), "right": _pinch_grip("right")}


def take_n_pinch(ctx):
    """Each hand, jaws open up and down, slides its lower finger into a hand hole and its upper finger over the box's
    top, then closes on the wall between them."""
    l, r = ctx.left, ctx.right
    pos, _ = _s(ctx, "N")
    xp = pos[0] - PINCH_BACK
    zp = pos[2] + L.HOLE_TOP + STRIP / 2

    def y_of(side, inset):
        sg = 1.0 if side == "left" else -1.0
        return pos[1] + sg * (HY + TIP - inset)

    # turn the hands low and well in front of the shelf first, so that the elbows stay clear of the cases on it; jaws
    # part open, so that the lower finger fits the hole with the upper one above the box's top
    yield from ctx.together(l.move((0.24, 0.24, zp - 0.02), grip=PINCH["left"], finger=PINCH_OPEN, t=2.5),
                            r.move((0.24, -0.24, zp - 0.02), grip=PINCH["right"], finger=PINCH_OPEN, t=2.5))
    yield from ctx.together(l.move((xp - 0.04, y_of("left", -0.06), zp), t=1.5), r.move((xp - 0.04, y_of("right", -0.06), zp), t=1.5))
    yield from ctx.together(l.move((xp, y_of("left", -0.04), zp), t=1.0), r.move((xp, y_of("right", -0.04), zp), t=1.0))
    ctx.hold = {a.side: {"trim": np.zeros(2), "y": y_of(a.side, -0.04), "depth": 0.0} for a in (l, r)}
    yield from hold_path(ctx, lambda f: (xp, zp), 0.6)
    yield from _pinch_in(ctx, xp, zp, y_of)
    width = STRIP
    fclose = float(np.clip(-(width / 2 - PINCH_F / GRIPPER_K), FINGER_OPEN, FINGER_CLOSED))
    for a in (l, r):
        a.finger = fclose
    yield from hold_path(ctx, lambda f: (xp, zp), 0.8)
    ctx.level = {"pitch": 0.0, "roll": 0.0, "rot0": {a.side: PINCH[a.side].rot for a in (l, r)}}
    if LOG:
        _log(f"pinched: N {np.round(_s(ctx, 'N')[0], 3)}, hands {np.round(l.tcp(), 3)} {np.round(r.tcp(), 3)}, "
             f"jaws {np.round(ctx.observed('left')[6], 3)} {np.round(ctx.observed('right')[6], 3)}")


def _pinch_in(ctx, xp, zp, y_of):
    n = round(1.2 * d.CONTROL_HZ)
    for k in range(1, n + 6):
        f = d.ik.smoothstep(min(1.0, k / n))
        for a in (ctx.left, ctx.right):
            h = ctx.hold[a.side]
            h["y"] = y_of(a.side, -0.04 + (0.04 + PINCH_IN) * f)
            obs = a.tcp()
            h["trim"] = np.clip(h["trim"] + np.clip(0.2 * (np.array([xp, zp]) - obs[[0, 2]]), -0.003, 0.003), -0.06, 0.06)
            _q_squeezed(a, (xp + h["trim"][0], h["y"], zp + h["trim"][1]), (0, 0, 0), 0.0)
        yield


def pinch_lift_out(ctx):
    """Up 8 cm off the table, back until N's far end is 3 cm in front of the shelf, then up above it."""
    x0, z0 = ctx.hold_nom
    pos, _ = _s(ctx, "N")
    lead = pos[0] - x0                                   # N's centre ahead of the hands
    dz = z0 - pos[2]                                     # the hands above N's centre
    clear = L.SHELF_X[0] - HX - 0.03 - lead
    yield from carry(ctx, [(x0, z0 - L.TABLE_Z + 0.08), (clear, z0 - L.TABLE_Z + 0.08),
                           (clear, L.WORK_TOP + HZ + 0.04 + dz)], speed=0.06)


def pinch_insert(ctx, y_goal, xp_rel):
    """Line N up with the slot, carry it in over the shelf until the hands are at ``xp_rel``, set it down level."""
    l, r = ctx.left, ctx.right
    pos, _ = _s(ctx, "N")
    dz = ctx.hold_nom[1] - pos[2]
    yield from hold_path(ctx, lambda f, p=ctx.hold_nom: p, 0.8, dy=y_goal - pos[1])
    for a in (l, r):
        ctx.hold[a.side]["y"] += y_goal - pos[1]
    yield from carry(ctx, [(ctx.hold_nom[0] + 0.10, L.WORK_TOP + HZ + 0.012 + dz), (xp_rel, L.WORK_TOP + HZ + 0.006 + dz)], speed=0.06)
    yield from carry(ctx, [(xp_rel, L.WORK_TOP + HZ + dz - 0.002)], speed=0.03)


def pinch_release(ctx):
    """Open the jaws, slide the fingers out of the hole and off the top, and back away."""
    l, r = ctx.left, ctx.right
    ctx.level = None
    for a in (l, r):
        a.finger = FINGER_OPEN
    yield from hold_path(ctx, lambda f, p=ctx.hold_nom: p, 0.6)
    for a in (l, r):
        a.grip = d._as_grip(a.hold_rot) if getattr(a, "hold_rot", None) is not None else a.grip
        a.hold_rot = None
    yield from ctx.together(l.move(l.at() + [0.0, 0.045, 0.0], t=1.0), r.move(r.at() + [0.0, -0.045, 0.0], t=1.0))
    yield from ctx.together(l.move(l.at() + [-0.03, 0.0, 0.04], t=0.8), r.move(r.at() + [-0.03, 0.0, 0.04], t=0.8))
    yield from ctx.together(to_ready(l, 2.5), to_ready(r, 2.5))

def script(ctx):
    mode = ctx.mode
    l, r = ctx.left, ctx.right
    if mode == "nothing":
        yield from l.wait(3.0)
        return
    room = mode in ("full", "drop")
    if room:
        yield from slide_aside(ctx, l, "C1", +1.0, 0.05)
        yield from ctx.together(park(ctx, l), slide_aside(ctx, r, "C2", -1.0, 0.05))
        yield from park(ctx, r)
    yield from take_n_pinch(ctx)
    yield from pinch_lift_out(ctx)
    if mode == "drop":
        # let go with N's centre still in front of the shelf's edge
        yield from pinch_insert(ctx, slot_y(ctx), L.SHELF_X[0] - PINCH_BACK - 0.05)
        yield from pinch_release(ctx)
        yield from home(ctx)
        return
    y_goal = slot_y(ctx)
    # the hands stop 4 mm short of the neighbours where they meet them: each hand's body and finger bases, 3.4 cm either
    # side of its TCP, lie 4.5 to 15 cm outboard of N's side
    side_y = HY + TIP - PINCH_IN
    front = min(front_in_band(ctx, "C1", y_goal + side_y + 0.045, y_goal + side_y + 0.15),
                front_in_band(ctx, "C2", y_goal - side_y - 0.15, y_goal - side_y - 0.045))
    _log(f"neighbours' fronts where the hands are: {front:.4f}")
    yield from pinch_insert(ctx, y_goal, min(0.40, front - 0.004 - 0.034))
    _log(f"set down: N {np.round(_s(ctx, 'N')[0], 3)}, tilt {_tilt(ctx, 'N'):.1f}, C1 {np.round(_s(ctx, 'C1')[0], 3)}, "
         f"C2 {np.round(_s(ctx, 'C2')[0], 3)}")
    yield from pinch_release(ctx)
    if room:
        yield from push_flush(ctx, [r], [y_goal])
    else:
        yield from push_flush(ctx, [l, r], [y_goal + 0.06, y_goal - 0.06])
    yield from home(ctx)
    yield from ctx.together(l.wait(1.5), r.wait(1.5))
