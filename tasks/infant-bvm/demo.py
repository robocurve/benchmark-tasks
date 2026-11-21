"""Scripted demo. The left arm takes the bag of the bag-valve mask by its middle (pads on its two side walls), lifts
the BVM and carries the mask over the manikin's face, nose end toward the crown. The right arm comes in from the
manikin's right, pitched down, and pinches the mask's stem and the swivel's port above it; together they lower the
mask onto the face and the right arm presses it on (a loop on the
cushion's measured compression) while the left keeps the bag where it is. The left squeezes the bag 25 mm over 1.3 s,
holds 0.3 s and lets it go, waits for the chest to fall, and squeezes again. Then the right lets go of the stem and
backs out, and the left lifts the BVM 70 mm off the face and holds it still.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--variant NAME]

Variants (scorer checks): ``offcentre`` presses the mask on 10 mm toward the chin; ``chest`` has the mask hand let
go of the stem and pinch the chest while the bag hand gives the two squeezes; ``big`` gives the second breath with a
40 mm squeeze; ``hyper`` starts with the head 25 degrees back (set on the neck; its friction holds it); ``drop``
puts the BVM down on the mat at the end instead of holding it; ``one`` gives one breath only. The demo reads object
poses (and the cushion's and the bag's joint positions).
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, wait  # noqa: E402

DEBUG = bool(os.environ.get("BVM_DEBUG"))
SQUEEZE = os.environ.get("BVM_SQUEEZE", "lr")   # lr: the side walls, gripped from above; tb: regrip from the side, top and bottom
GAP_MM = 94.0                       # pads' gap at grip 1.0
BAG_D = 2 * g.BAG_R * 1000          # 71 mm
CARRY_SQ = 8.0                      # mm commanded into the bag while carried (about 4 mm: well under any breath)
LEFT_TILT = math.radians(20.0)      # bag hand: pointing down, tilted 20 deg away from the robot (reach)
GRASP_IN = 0.010                    # ... and 10 mm from the bag's middle toward the valve (its fingers lean back)
PINCH_PITCH = math.radians(20.0)    # mask hand: fingers from the manikin's right, pointing down 20 deg
PAD_Z = 0.040                       # the pads' middle this far above the mask's plane: across the stem and the swivel
PINCH_SQ = 6.0                      # mm commanded inside the 22 mm stem
SQUEEZE_MM = 25.0                   # each breath
T_IN, T_HOLD, T_OUT = 1.3, 0.3, 0.6
TILT_GAIN = float(os.environ.get("BVM_TILT_GAIN", "0.3"))
YAW_GAIN = float(os.environ.get("BVM_YAW_GAIN", "0.0"))      # off: the pads roll on the round stem, it can't turn it   # the press's tilt trim (a plane fit to the cushion)
C_MIN = 1.35                        # mm: the press rises until the least-pressed cushion segment is in this far
C_TARGET = 2.2                      # mm of cushion compression the press aims for
ABOVE = 0.035                       # the mask is brought this far over the face before going down


def grip_for(gap_mm: float) -> float:
    return float(min(1.0, max(0.0, gap_mm / GAP_MM)))


def R_from(a, c) -> np.ndarray:
    """A gripper rotation with approach a (z) and closing axis c (x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rot(axis, ang) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


def arrive(arm, tol=0.002, timeout=2.0):
    for _ in range(round(timeout * 20)):
        if np.linalg.norm(tcp_actual(arm)[0] - arm.pos) < tol:
            return
        yield
    if DEBUG:
        print(f"arrive: {arm.side} still {np.linalg.norm(tcp_actual(arm)[0] - arm.pos) * 1000:.1f} mm off", flush=True)


class Scene:
    """Measured poses: the BVM's bodies and joints, the manikin's head and the face's mask point."""

    def __init__(self, ctx):
        from scenekit import env
        t = ctx.task
        self.b = t.body_poses(env(), "bvm")
        self.m = t.body_poses(env(), "manikin")
        self.q = t.joints(env(), "bvm")
        self.qm = t.joints(env(), "manikin")
        self.dome, self.valve, self.head = self.b["dome"], self.b["valve"], self.m["head"]
        self.face_o = t.face_point(self.head)
        self.face_R = self.head[1] @ t.LM["mask_R"]
        self.segs = np.array([self.q[s] for s in t.SEGS]) * 1000
        self.d_lr = (self.q["wall_left"] + self.q["wall_right"]) * 1000
        self.d_tb = (self.q["wall_top"] + self.q["wall_bottom"]) * 1000
        self.sq = self.d_tb if SQUEEZE == "tb" else self.d_lr


def compose(T, U):
    """(p, R) o (p, R)."""
    return T[0] + T[1] @ U[0], T[1] @ U[1]


def relative(T, U):
    """T^-1 o U."""
    return T[1].T @ (U[0] - T[0]), T[1].T @ U[1]


# -- the bag hand ---------------------------------------------------------------------------------------------------------
def bag_grasp_in_valve():
    """The left TCP in the valve's frame: on the bag's axis at its middle, pointing down and 30 deg away from the
    robot (the valve frame's +x), closing across the bag."""
    a = np.array([math.sin(LEFT_TILT), 0.0, -math.cos(LEFT_TILT)])
    return np.array([-g.BAG_MID + GRASP_IN, 0.0, g.VALVE_Z]), R_from(a, [0.0, -1.0, 0.0])


def pick_bag(ctx):
    L = ctx.left
    sc = Scene(ctx)
    T = compose(sc.valve, bag_grasp_in_valve())
    a = T[1][:, 2]
    yield from L.move_joints(T[0] - 0.07 * a, T[1], 3.0, grip=0.97)
    yield from L.move(T[0] - 0.01 * a, T[1], 1.0)
    yield from L.move(T[0], T[1], 0.8)
    yield from arrive(L, 0.002, 1.5)
    yield from L.set_grip(grip_for(BAG_D - CARRY_SQ), 0.8)
    yield from wait(0.5)


# -- the mask on the face ------------------------------------------------------------------------------------------------
class Hold:
    """Both hands on the BVM, moved as one rigid body: commanded relative to the dome (the mask)."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.T_dL = None
        self.T_dR = None
        self.off = 0.0          # press: the right hand's command this far into the face (along the mask's -z), m
        self.tilt = np.zeros(2)  # about the mask's x and y, rad
        self.yaw = 0.0          # about the mask's z, rad: keeps the mask turned as the face
        self.squeeze = CARRY_SQ  # the bag's squeeze target, mm
        self.sq_corr = 0.0
        self.shift = np.zeros(3)
        self.left_fixed = False     # the bag hand keeps its pose (world), only its grip follows the squeeze
        self.T_vL = None            # the bag hand's hold in the valve's frame (from the side)
        self.left_pos = None
        self.trim_frozen = False
        self.bag_dz = 0.0

    def grab(self, left: bool, right: bool):
        sc = Scene(self.ctx)
        if left:
            self.T_dL = relative(sc.dome, (self.ctx.left.pos.copy(), self.ctx.left.rot.copy()))
        if right:
            self.T_dR = relative(sc.dome, (self.ctx.right.pos.copy(), self.ctx.right.rot.copy()))

    def aim(self, target, press=True, sc=None):
        """Command both hands for the dome at ``target`` (p, R) turned by the tilt trim: the mask hand also with the
        press (into the face). The bag hand: while it holds the bag by its sides (from above, which can't turn the
        mask) it moves with the mask; once it holds the bag top and bottom (T_vL) it stays where it took it and only
        keeps its pads square to the bag's measured walls."""
        p, R = target
        R = R @ rot([0, 0, 1], self.yaw) @ rot([1, 0, 0], self.tilt[0]) @ rot([0, 1, 0], self.tilt[1])
        p = p + self.shift
        if self.T_dR is not None:
            q = p - R[:, 2] * (self.off if press else 0.0)
            pr, Rr = compose((q, R), self.T_dR)
            self.ctx.right.aim(pr, Rr)
        if self.T_dL is not None or self.T_vL is not None:
            if not self.left_fixed:
                if self.T_vL is not None:
                    if sc is not None:
                        self.ctx.left.aim(self.left_pos + R[:, 2] * self.bag_dz, sc.valve[1] @ self.T_vL[1])
                else:
                    pl, Rl = compose((p, R), self.T_dL)
                    self.ctx.left.aim(pl, Rl)
            sq = self.squeeze + self.sq_corr
            self.ctx.left.grip = grip_for(BAG_D - sq)

    def level(self, sc, R_target, gain=0.12):
        """The bag hand's height trim: the mask's pitch against the face (its chin end high = nose down) moves the
        bag (110 mm on the nose side) up, and the other way."""
        s = float(sc.dome[1][:, 0] @ R_target[:, 2])          # sin of the nose-down pitch
        self.bag_dz = float(np.clip(self.bag_dz + gain * 0.11 * s, -0.02, 0.02))
        return math.degrees(math.asin(max(-1.0, min(1.0, s))))

    def track_squeeze(self, sc):
        """The bag's measured squeeze follows its target (the fingers give under the walls' springs)."""
        if self.squeeze > CARRY_SQ + 0.5 and self.squeeze > sc.sq - 0.5:
            self.sq_corr = float(np.clip(self.sq_corr + 0.35 * (self.squeeze - sc.sq), 0.0, 25.0))
        else:
            self.sq_corr = max(0.0, self.sq_corr - 1.0)

    def press(self, sc, gain=0.6, tilt=True, pitch=True):
        """The right hand's press: integral on the cushion's mean compression; tilt: even it out (a plane fit); yaw:
        the mask's turn about its axis held to the face's."""
        c = sc.segs
        if tilt:
            xd, Rf = sc.dome[1][:, 0], sc.face_R
            psi = math.atan2(float(xd @ Rf[:, 1]), float(xd @ Rf[:, 0]))
            self.yaw = float(np.clip(self.yaw - YAW_GAIN * psi, -0.2, 0.2))
            self.psi = psi
        step = float(np.clip(gain * 1e-3 * (C_TARGET - c.mean()), -0.0015, 0.0015))
        self.off = float(np.clip(self.off + step, -0.004, 0.060))
        if not tilt:
            return
        cp = self.ctx.task._CP
        A = np.c_[np.ones(len(cp)), cp[:, 0], cp[:, 1]]
        coef = np.linalg.lstsq(A, c / 1000.0, rcond=None)[0]       # compression = c0 + a x + b y
        # more compressed toward +x: turn about the mask's y so +x rises (negative about y), likewise for y about x
        if pitch:
            self.tilt[1] = float(np.clip(self.tilt[1] - TILT_GAIN * coef[1], -0.12, 0.12))
        self.tilt[0] = float(np.clip(self.tilt[0] + TILT_GAIN * coef[2], -0.12, 0.12))


def carry_to_face(ctx, hold, target_shift=np.zeros(3)):
    """Lift, carry the mask over the face's mask point (the face's orientation), the right pinches the stem."""
    L = ctx.left
    yield from L.move(L.pos + np.array([0.0, 0.0, 0.07]), L.rot, 1.2)
    sc = Scene(ctx)
    T_vL = relative(sc.valve, (L.pos.copy(), L.rot.copy()))       # the bag hand in the valve's frame (as gripped)
    o, R = sc.face_o + target_shift, sc.face_R
    above = (o + R[:, 2] * ABOVE, R)
    pl, Rl = compose(above, T_vL)
    yield from L.move_joints(pl + np.array([0.0, 0.0, 0.03]), Rl, 3.0)
    yield from L.move(pl, Rl, 1.0)
    yield from wait(0.4)
    # close the loop on the mask's measured pose (position and turn)
    for _ in range(4):
        sc = Scene(ctx)
        e = above[0] - sc.dome[0]
        Re = above[1] @ sc.dome[1].T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2)))
        if np.linalg.norm(e) < 0.001 and ang < math.radians(1.0):
            break
        w = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        Rc = rot(w, ang) if np.linalg.norm(w) > 1e-9 else np.eye(3)
        yield from L.move(L.pos + e, Rc @ L.rot, 0.6)
        yield from wait(0.3)
    hold.grab(left=True, right=False)
    # the mask hand: in from the manikin's right, pitched down, pads either side of the stem
    R_ = ctx.right
    sc = Scene(ctx)
    pd, Rd = sc.dome
    a = Rd @ np.array([0.0, math.cos(PINCH_PITCH), -math.sin(PINCH_PITCH)])
    Rr = R_from(a, Rd[:, 0])
    tcp = pd + Rd @ np.array([0.0, 0.0, PAD_Z]) - a * 0.0065
    yield from R_.move_joints(tcp - 0.07 * a, Rr, 3.0, grip=0.6)
    yield from R_.move(tcp - 0.015 * a, Rr, 1.0)
    yield from R_.move(tcp, Rr, 0.8)
    yield from arrive(R_, 0.002, 1.5)
    yield from R_.set_grip(grip_for(2 * g.STEM_R * 1000 - PINCH_SQ), 0.8)
    yield from wait(0.4)
    hold.grab(left=False, right=True)
    hold.grab(left=True, right=False)


def seat(ctx, hold, target_shift=np.zeros(3), seconds=6.0, lower=True):
    """Lower the mask onto the face (both hands), then press it on: the right hand's press and tilt follow the
    cushion's compression until every segment is in and the mask sits on the face's mask point."""
    ctx.flags["view"] = "close"
    sc = Scene(ctx)
    o, R = sc.face_o + target_shift, sc.face_R
    start = sc.dome
    lp0 = None if hold.left_pos is None else hold.left_pos.copy()
    n = 30 if lower else 0
    for k in range(1, n + 1):
        s = k / n
        s = s * s * (3 - 2 * s)
        p = start[0] + s * (o + R[:, 2] * 0.004 - start[0])
        if lp0 is not None:                                  # the bag hand comes down with the mask
            hold.left_pos = lp0 + (p - start[0])
        hold.aim((p, R), press=False, sc=Scene(ctx))
        yield
    good = 0
    for k in range(round(seconds * 20)):
        sc = Scene(ctx)
        o, R = sc.face_o + target_shift, sc.face_R
        hold.press(sc, pitch=hold.T_vL is None)
        e = (o - sc.dome[0])
        hold.shift = np.clip(hold.shift + 0.3 * (e - R[:, 2] * (R[:, 2] @ e)), -0.01, 0.01)
        if hold.T_vL is not None:
            pitch = hold.level(sc, R)
        else:
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(sc.dome[1][:, 0] @ R[:, 2])))))
        hold.aim((o, R), sc=sc)
        S = ctx.task._STATE
        good = good + 1 if (S["pressed"] == g.N_SEG and sc.segs.min() >= 1.1 and S["centre_mm"] < 3.0) else 0
        if DEBUG and k % 10 == 0:
            mt = np.degrees(np.arccos(np.clip(sc.dome[1][:, 2] @ R[:, 2], -1, 1)))
            print(f"[seat] k {k} yaw {math.degrees(getattr(hold, 'psi', 0.0)):.2f}/{math.degrees(hold.yaw):.2f} pitch {pitch:.2f} bag_dz {hold.bag_dz * 1000:.1f} mask tilt {mt:.2f} off {hold.off * 1000:.1f} tilt {np.degrees(hold.tilt).round(2)} segs "
                  f"{sc.segs.round(2).tolist()} centre {S['centre_mm']:.1f} pressed {S['pressed']} head "
                  f"{S['head_deg']:.2f}", flush=True)
        yield
        if good >= 5 and k > 20:                             # seated: stop trimming, hold it there
            break


def regrip_tb(ctx, hold, target_shift=np.zeros(3)):
    """The bag hand lets go of the bag's sides and takes it from the robot's left, pads on its top and bottom walls
    (the right hand holds the mask on meanwhile)."""
    L = ctx.left
    hold.T_dL = None
    yield from L.set_grip(1.0, 0.5)
    yield from L.move(L.pos + np.array([0.0, 0.0, 0.05]), L.rot, 0.8)
    yield from L.move(L.pos + np.array([0.0, 0.09, 0.0]), L.rot, 1.0)
    sc = Scene(ctx)
    az, el = math.radians(10.0), math.radians(10.0)
    a = sc.valve[1] @ np.array([math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), -math.sin(el)])
    c = sc.valve[1][:, 2]                       # closing exactly along the bag's up: the pads square to the walls
    a = a - c * (c @ a)
    Rl = R_from(a, c)
    p = sc.valve[0] + sc.valve[1] @ np.array([-g.BAG_MID + 0.010, 0.0, g.VALVE_Z])
    yield from L.move_joints(p - 0.08 * a, Rl, 3.0, grip=0.97)
    yield from L.move(p - 0.01 * a, Rl, 1.0)
    yield from L.move(p, Rl, 0.6)
    yield from arrive(L, 0.002, 1.0)
    yield from L.set_grip(grip_for(BAG_D - CARRY_SQ), 0.6)
    yield from wait(0.3)
    sc = Scene(ctx)
    hold.T_vL = relative(sc.valve, (L.pos.copy(), L.rot.copy()))
    hold.left_pos = L.pos.copy()


def keep(ctx, hold, seconds, target_shift=np.zeros(3), until=None):
    """Hold the mask on (press loop) with the bag's squeeze at hold.squeeze, for ``seconds`` or until ``until()``."""
    for k in range(round(seconds * 20)):
        sc = Scene(ctx)
        o, R = sc.face_o + target_shift, sc.face_R
        hold.press(sc, gain=0.4, tilt=not hold.trim_frozen)
        hold.track_squeeze(sc)
        hold.aim((o, R), sc=sc)
        yield
        if until is not None and until():
            break


def breath(ctx, hold, depth=SQUEEZE_MM, target_shift=np.zeros(3)):
    """Squeeze the bag ``depth`` mm over T_IN, hold T_HOLD, let go over T_OUT (closed loop on the walls' travel), then
    wait for the chest to fall back."""
    def prof(t):
        if t < T_IN:
            s = t / T_IN
            return CARRY_SQ + (depth - CARRY_SQ) * s * s * (3 - 2 * s)
        if t < T_IN + T_HOLD:
            ctx.flags["shot"] = "squeeze" if "seated" in str(ctx.flags.get("shot")) else ctx.flags.get("shot")
            return depth
        s = min(1.0, (t - T_IN - T_HOLD) / T_OUT)
        return depth + (CARRY_SQ - depth) * s * s * (3 - 2 * s)

    n = round((T_IN + T_HOLD + T_OUT) * 20)
    from scenekit import arm_joints, env
    for k in range(n):
        hold.squeeze = prof(k / 20)
        yield from keep(ctx, hold, 0.05, target_shift)
        if DEBUG:
            sc = Scene(ctx)
            fj = arm_joints(env(), "left", ["joint7", "joint8"]) * 1000
            tl, Rl = tcp_actual(ctx.left)
            pv = Rl.T @ (sc.valve[0] - tl)
            bx = Rl.T @ sc.valve[1][:, 0]
            bz = Rl.T @ sc.valve[1][:, 2]
            walls = [round(sc.q[f"wall_{w}"] * 1000, 1) for w in ("top", "left", "bottom", "right")]
            print(f"[sq] k {k} target {hold.squeeze:.1f} corr {hold.sq_corr:.1f} fingers {np.round(fj, 1).tolist()} walls "
                  f"{walls} valve_in_tcp {np.round(pv * 1000, 1).tolist()} bag_x {np.round(bx, 3).tolist()} "
                  f"bag_z {np.round(bz, 3).tolist()} rise {ctx.task._STATE['rise'] * 1000:.2f}", flush=True)
    hold.squeeze = CARRY_SQ
    S = ctx.task._STATE
    yield from keep(ctx, hold, 5.0, target_shift, until=lambda: S["rise"] <= 0.0004)    # chest back down
    yield from keep(ctx, hold, 0.4, target_shift)


def chest_pinch(ctx, hold):
    """Variant: once the mask has sat 2.5 s, the mask hand lets go of the stem and pinches the chest beside the flag;
    the bag hand gives the two squeezes with the mask held only by the bag, then the mask hand lets go of the chest."""
    R_ = ctx.right
    yield from keep(ctx, hold, 2.5)
    hold.T_dR = None
    yield from R_.set_grip(0.6, 0.5)
    a = R_.rot[:, 2]
    yield from R_.move(R_.pos - 0.07 * a, R_.rot, 1.2)
    yield from R_.move(R_.pos + np.array([-0.05, -0.05, 0.08]), R_.rot, 1.2)
    st = ctx.task.LM["sternum"] + np.array([g.CROWN_X, 0.0, g.MAT_TOP])
    Rc = R_from([0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
    p = st + np.array([0.045, 0.0, 0.0])                  # beside the flag, on the chest
    yield from R_.move_joints(p + np.array([0.0, 0.0, 0.08]), Rc, 3.0, grip=0.5)
    yield from R_.move(p + np.array([0.0, 0.0, -0.004]), Rc, 1.2)
    yield from R_.set_grip(0.0, 0.6)                      # pinch the chest's skin
    ctx.flags["shot"] = "pinch"
    yield from breath(ctx, hold)
    yield from breath(ctx, hold)
    yield from R_.set_grip(0.5, 0.5)
    yield from R_.move(R_.pos + np.array([0.0, 0.0, 0.08]), Rc, 1.0)


def finish(ctx, hold, variant):
    """The right lets go of the stem and backs out (the left holds the bag); the left lifts the BVM 70 mm off the face
    along the mask's axis and holds it still (``drop``: puts it down on the mat beside the manikin and lets go)."""
    R_, L = ctx.right, ctx.left
    hold.T_dR = None
    yield from R_.set_grip(0.6, 0.5)
    a = R_.rot[:, 2]
    yield from R_.move(R_.pos - 0.07 * a, R_.rot, 1.2)
    yield from R_.move(R_.pos + np.array([-0.05, -0.05, 0.08]), R_.rot, 1.2)
    sc = Scene(ctx)
    up = sc.face_R[:, 2]
    hold.T_dL = None
    yield from L.move(L.pos + up * 0.070, L.rot, 2.0)
    if variant == "drop":
        yield from L.move(L.pos + np.array([0.0, 0.25, 0.0]), L.rot, 2.5)
        yield from L.move(L.pos + np.array([0.0, 0.0, -0.10]), L.rot, 1.5)
        yield from L.set_grip(1.0, 0.6)
        yield from L.move(L.pos + np.array([0.0, 0.0, 0.08]), L.rot, 1.0)
    yield from R_.go_home(2.5)


def script(ctx, variant):
    if variant == "hyper":
        yield from head_back(ctx, 25.0)
    yield from wait(0.5)
    hold = Hold(ctx)
    yield from pick_bag(ctx)
    shift = np.zeros(3)
    if variant == "offcentre":
        sc = Scene(ctx)
        shift = sc.face_R[:, 0] * 0.010                    # 10 mm toward the chin
    yield from carry_to_face(ctx, hold, shift)
    if SQUEEZE == "tb" and variant != "chest":
        yield from regrip_tb(ctx, hold, shift)
    yield from seat(ctx, hold, shift)
    ctx.flags["shot"] = "seated"
    hold.trim_frozen = True
    yield from keep(ctx, hold, 0.5, shift)
    if variant == "chest":
        yield from chest_pinch(ctx, hold)
    else:
        yield from breath(ctx, hold, target_shift=shift)
        if variant != "one":
            yield from breath(ctx, hold, 40.0 if variant == "big" else SQUEEZE_MM, target_shift=shift)
    if variant == "hyper":                                 # the mask kept on (the airway check is done)
        yield from keep(ctx, hold, 30.0, shift)
    yield from finish(ctx, hold, variant)
    yield from wait(30.0)


def head_back(ctx, angle_deg):
    """Variant setup: the head turned ``angle_deg`` back on its neck (the neck's friction then holds it), as if a hand
    had tilted it before the mask went on."""
    from scenekit import env, set_joint_targets, write_joint_positions

    q = -math.radians(angle_deg)
    write_joint_positions(env(), "manikin", {"neck": q})
    set_joint_targets(env(), "manikin", {"neck": q})
    ctx.task._STATE["neck_rest"] = q
    yield


CLOSE_VIEW = (0.106, 0.106, 0.070)     # the close view's eye from the point midway between the mask point and the flag's dot


def close_view(ctx, gen):
    """Recorded runs: when the mask goes onto the face the top camera moves to a close view of the face and the chest
    flag (from the feet and the manikin's left, 165 mm off), so the video shows the seated mask and the flag's rise.
    Only the recording changes; the demo reads object poses, not images."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)
    moved = False
    for _ in gen:
        yield
        if not moved and ctx.flags.get("view") == "close":
            face = ctx.task.face_point(Scene(ctx).head)
            dot = ctx.task.LM["sternum"] + np.array([g.CROWN_X, 0.0, g.MAT_TOP + g.DOT_UP])
            t = 0.5 * (face + dot)
            eye = torch.tensor(np.array([t + np.array(CLOSE_VIEW)]), dtype=torch.float32, device=o.device) + o
            tgt = torch.tensor(np.array([t]), dtype=torch.float32, device=o.device) + o
            cam.set_world_poses_from_view(eye, tgt)
            moved = True
            print("top camera: close view", flush=True)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    sc = Scene(ctx)
    tl, _ = tcp_actual(ctx.left)
    tr, _ = tcp_actual(ctx.right)
    from scenekit import arm_joints, env
    gl = -float(np.sum(arm_joints(env(), "left", ["joint7", "joint8"]))) * 1000
    walls = [round(sc.q[f"wall_{w}"] * 1000, 1) for w in ("top", "left", "bottom", "right")]
    return (f"gapL {gl:.1f} cmdL {ctx.left.grip * 94:.1f} walls {walls} dome {np.round(sc.dome[0] * 1000, 1).tolist()} centre {S['centre_mm']:.1f} segs {sc.segs.round(1).tolist()} "
            f"pressed {S['pressed']} seated {S['seated']} head {S['head_deg']:.2f} sq {sc.d_lr:.1f} V {S['v_bag']:.1f} "
            f"rise {S['rise'] * 1000:.2f} br {S.get('n_breath')}/{S.get('n_big')} clean {S['clean']} who {S['who']} "
            f"swivel {math.degrees(sc.q['swivel']):.1f} Lerr {np.linalg.norm(tl - ctx.left.pos) * 1000:.1f} "
            f"Rerr {np.linalg.norm(tr - ctx.right.pos) * 1000:.1f} gap {S['face_gap'] * 1000:.0f} grip {S['gripped']} "
            f"held {S['held_steps']} moved {S['moved_mm']:.1f} lift {S['lift_mm']:.1f} eye {S.get('d_eye_mm', 0):.1f} "
            f"tier {S['tier']} why {S['why']} episodes {S['breaths'].breaths + S['breaths'].too_big + S['breaths'].void}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--keep-top-cam", action="store_true", help="don't move the top camera to the close view")
    ap.add_argument("--variant", default="full",
                    choices=["full", "nothing", "offcentre", "chest", "big", "hyper", "drop", "one"])
    ap.add_argument("--variants", default="", help="comma list: run these one after another in this process (no video)")
    a = ap.parse_args()
    if a.variants:
        import variants
        pathlib.Path(a.out).mkdir(parents=True, exist_ok=True)
        variants.main(a.task, str(pathlib.Path(a.out) / "variants.json"), a.variants.split(","), a.max_steps)
        sys.exit(0)
    print("variant", a.variant, flush=True)
    if a.variant == "nothing":
        def scr(ctx):
            yield from wait(4.0)
    else:
        def scr(ctx):
            if a.no_record or a.keep_top_cam:
                yield from script(ctx, a.variant)
            else:
                yield from close_view(ctx, script(ctx, a.variant))
    run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=int(os.environ.get("BVM_LOG_EVERY", "10")), hold_s=float("inf"))
