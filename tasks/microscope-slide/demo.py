"""Scripted full-score demo. The left arm takes the slide from the bench holder from above by its two long edges (pads
on the 1 mm edges near the label end, the slide along the pads' lower slanted edges), lifts it out along the slot, turns
it flat (coverslip up, label end left) while carrying it round to the left of the stage, and lays it on the stage in
front of the seat with its hand level from the left (fingers pointing right, tipped 21 degrees down so the pads' lower
edges lie on the stage); it opens and slides the fingers out. The right arm takes the caliper's lever knob loosely
between its pads and swings the arm open 30 degrees, the trailing pad pushing the knob, and holds it. The left, fingers
closed and pointing back, pushes the slide's front edge with its fingertips in short strokes (push_strokes): at the
middle while the slide is square, at its lagging end when it is not, lifting between strokes, until its back edge lies
flush on the template's back edge. The right lets the arm return at 7 mm/s at the tip, the knob riding on its pad,
still on the knob when the tip meets the slide's right end; it gives way while the spring carries the slide onto the
jaw, then lifts off the knob. Both arms go home. (MS_PUSH=pair: the earlier two-fingertip push, which keeps the slide
square but meets its top arris with the rounded tips: a face contact under rule 1.)

    python demo.py task.py demo_out [--max-steps 3600] [--no-record] [--variant NAME] [--upto PHASE]

Variants (scorer checks): ``face`` takes the slide across its faces in the specimen area; ``snap`` lets go of the
lever with the arm open (it snaps shut on the slide); ``short`` stops the push 3 mm short of the template; ``brush``
holds the arm's tip in the slide's path so the slide brushes it during the push (a hand on the lever), then opens it 8
degrees and lets go (the arm snaps on).
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json.
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

DEBUG = bool(os.environ.get("MS_DEBUG"))
GAP_MM = 94.0                       # pads' gap at grip 1.0
SQUEEZE_MM = 3.2                    # commanded inside the part: ~8 N at the fingers' 5 N/mm
BETA = math.atan2(6.2, 15.8)        # the pads' lower edges' slant (fingertips taper 26.4 -> 14 mm wide)
PITCH = math.radians(18.0)          # push: hand pitched down so the fingertips are the fingers' lowest point
E1 = np.array([0.0, 0.0132, -0.003])    # pad's lower edge, wide end (gripper frame rel. TCP: c, b, a)
E2 = np.array([0.0, 0.0070, 0.0128])    # pad's lower edge, at the tip
TIP_FROM_LABEL = 0.025              # the pads' tip end this far from the label end
EDGE_BELOW = 0.0002                 # the pads' lower edges reach this far below the slide's underside
SET_GAP_JAW = 0.003                 # set down with the left end this far right of the jaw line
SET_BACK = 0.035                    # and the back edge this far in front of the template's back edge
RELEASE_H = 0.0003                  # let go with the slide's underside this far above the stage
KNOB_H = 0.017                      # right TCP above the stage top when holding the knob
KNOB_TILT = math.radians(45.0)      # right hand: approach tilted 45 deg from vertical,
KNOB_AZ = math.radians(60.0)        # its horizontal part pointing left and a little back (the hand comes from the right)
TIP_SPEED = 0.007                   # lever return, tip speed (m/s)
TIP_Z = 0.00005                     # push: the fingertips' lower edges this far above the stage
OPEN_TO = math.radians(30.0)        # the arm swung open this far: its tip 15 mm clear of the slide's path


def grip_for(width_m: float, squeeze: bool = True) -> float:
    return max(0.0, (width_m * 1000 - (SQUEEZE_MM if squeeze else 0.0)) / GAP_MM)


def R_from(a, c) -> np.ndarray:
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rotz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


# -- the slide in the left hand ----------------------------------------------------------------------------------------
def slide_in_hand():
    """(R_GS, p_GS): the slide's pose in the left gripper's TCP frame for the edge pinch: width across the pads (c),
    length along the pads' lower slanted edge (label end toward the pads' wide end), underside EDGE_BELOW above that
    edge, the pads' tips TIP_FROM_LABEL from the label end."""
    ex = np.array([0.0, math.sin(BETA), -math.cos(BETA)])
    ey = np.array([1.0, 0.0, 0.0])
    ez = np.cross(ex, ey)
    R = np.column_stack([ex, ey, ez])
    # unknown p = (0, pb, pa): (E1 - p).ez = -(T/2 + EDGE_BELOW); (E2 - p).ex = L/2 - TIP_FROM_LABEL
    A = np.array([[ez[1], ez[2]], [ex[1], ex[2]]])
    rhs = np.array([E1 @ ez + g.SLIDE_T / 2 + EDGE_BELOW, E2 @ ex - (g.SLIDE_L / 2 - TIP_FROM_LABEL)])
    pb, pa = np.linalg.solve(A, rhs)
    return R, np.array([0.0, pb, pa])


R_GS, P_GS = slide_in_hand()


def gripper_for_slide(p_s, R_s):
    """The left TCP pose that holds the slide at (p_s, R_s)."""
    R_g = R_s @ R_GS.T
    return p_s - R_g @ P_GS, R_g


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


def finger_gap(arm) -> float:
    from scenekit import arm_joints, env

    q = arm_joints(env(), arm.side, ["joint7", "joint8"])
    return float(-(q[0] + q[1]))


def arrive(arm, tol=0.002, timeout=2.0):
    for _ in range(round(timeout * 20)):
        if np.linalg.norm(tcp_actual(arm)[0] - arm.pos) < tol:
            return
        yield
    if DEBUG:
        print(f"arrive: {arm.side} still {np.linalg.norm(tcp_actual(arm)[0] - arm.pos) * 1000:.1f} mm off", flush=True)


def settle_to(arm, pos, rot, tol=0.0004, rounds=3, seconds=0.6):
    """Move to ``pos`` and correct the command by the measured tool-point error (gravity sag, tracking)."""
    target = np.asarray(pos, float)
    cmd = target.copy()
    yield from arm.move(cmd, rot, seconds)
    for _ in range(rounds):
        yield from wait(0.3)
        err = target - tcp_actual(arm)[0]
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
        yield from arm.move(cmd, rot, 0.4)


def slide_pose(ctx):
    s = ctx.states()["slide"]
    return np.asarray(s.pos, float), ctx.task._rotation(s.quat_xyzw)


def M(p) -> np.ndarray:
    return g.to_world(p)


# -- left arm --------------------------------------------------------------------------------------------------------------
def pick(ctx, arm, variant):
    p_s, R_s = slide_pose(ctx)
    tcp, Rg = gripper_for_slide(p_s, R_s)
    a = Rg[:, 2]
    if variant == "face":
        # across the faces in the specimen area: pads on the faces 20 mm below the label end, closing along the normal
        n = R_s[:, 2]
        lx = R_s[:, 0]
        Rg = R_from(-lx, n)
        tcp = p_s + lx * (g.SLIDE_L / 2 - 0.030) - Rg[:, 2] * 0.0
        a = Rg[:, 2]
        yield from arm.move_joints(tcp - 0.06 * a, Rg, 3.0, grip=0.35)
        yield from settle_to(arm, tcp, Rg)
        yield from arm.set_grip(grip_for(g.SLIDE_T + 0.0002), 0.8)
        return Rg
    yield from arm.move_joints(tcp - 0.06 * a, Rg, 3.0, grip=0.50)
    yield from arm.move(tcp - 0.012 * a, Rg, 1.2)
    yield from settle_to(arm, tcp, Rg)
    yield from arm.set_grip(grip_for(g.SLIDE_W), 0.8)
    yield from wait(0.4)
    return Rg


def lift_and_turn(ctx, arm, Rg):
    """Out of the slot along its axis, then (Cartesian, slerped) round to the left of the stage while turning flat,
    over at 30 mm, down to 12 mm above the set-down (path checked offline: dev/turn_path.py)."""
    p_s, R_s = slide_pose(ctx)
    yield from arm.move(arm.pos + 0.030 * R_s[:, 0], Rg, 1.5)
    tcp, Rset = set_pose(ctx)
    yield from arm.move(np.array([tcp[0], 0.15, g.STAGE_Z]), Rset, 3.5)
    yield from arm.move(np.array([tcp[0], 0.09, g.STAGE_Z + 0.03]), Rset, 1.2)
    yield from arm.move(tcp + np.array([0.0, 0.0, 0.03]), Rset, 1.5)
    yield from arrive(arm, 0.003)
    return Rset


def set_pose(ctx, lift=0.0):
    """The left TCP pose that holds the slide flat in front of the seat (underside RELEASE_H + lift above the stage)."""
    ex, ez = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    R_s = np.column_stack([ex, np.cross(ez, ex), ez])
    return gripper_for_slide(set_centre(lift), R_s)


def set_centre(lift=0.0) -> np.ndarray:
    return M((g.BACK_EDGE_X - SET_BACK - g.SLIDE_W / 2, g.JAW_Y - SET_GAP_JAW - g.SLIDE_L / 2,
              g.SLIDE_T / 2 + RELEASE_H + lift))


def set_down(ctx, arm, Rset):
    tcp, _ = set_pose(ctx)
    yield from arm.move(tcp + np.array([0.0, 0.0, 0.012]), Rset, 1.5)
    # close the loop on the slide itself (where planned in x/y, underside RELEASE_H above the stage), but never with
    # the pads' lower edges below 0.1 mm over the stage (the slide may sit a little high in the pads)
    pad_low = min((Rset @ E1)[2], (Rset @ E2)[2])
    for _ in range(4):
        yield from wait(0.25)
        p_s, R_s = slide_pose(ctx)
        tcp_m = tcp_actual(arm)[0]
        want = set_centre()
        low = (p_s + g.slide_corners() @ R_s.T)[:, 2].min()
        z_slide = tcp_m[2] + (g.STAGE_Z + RELEASE_H - low)
        z_pads = g.STAGE_Z + 0.0001 - pad_low
        err = np.array([want[0] - p_s[0], want[1] - p_s[1], max(z_slide, z_pads) - tcp_m[2]])
        step = err if np.linalg.norm(err) < 0.004 else err * 0.6
        yield from arm.move(arm.pos + step, Rset, 0.8 if abs(err[2]) > 0.002 else 0.5)
        if np.linalg.norm(err) < 0.0003:
            break
    yield from wait(0.3)
    yield from arm.set_grip(0.40, 0.8)                         # open to ~38 mm
    yield from wait(0.3)
    # slide the open fingers out along the slide's edges (to the left), then up
    yield from arm.move(arm.pos + np.array([0.0, 0.045, 0.001]), Rset, 1.5)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.05]), Rset, 1.0)


def push(ctx, arm, variant):
    """Two fingertips on the slide's front edge, 44 mm apart about its middle (fingers open, closing axis across the
    stage, hand pitched down BETA so the fingers' lower edges run level 0.12 mm over the stage): push straight back.
    Two contacts keep the slide square (one point lets it turn away)."""
    Rp = push_rot()
    bdown = -1.0 if Rp[2, 1] > 0 else 1.0
    tip_local = np.array([0.0, bdown * E2[1], E2[2]])     # the tips' lower front corner (each finger, mid-thickness)
    edge_z = (Rp @ np.array([0.0, bdown * E1[1], E1[2]]))[2]
    p_s, R_s = slide_pose(ctx)
    x_front = (p_s + g.slide_corners() @ R_s.T)[:, 0].min()
    tipv = Rp @ tip_local
    lean = (0.001 - TIP_Z) * math.tan(BETA)               # the end face meets the slide's top-front corner this far on
    start = np.array([x_front - 0.005 - lean - tipv[0], p_s[1], g.STAGE_Z + TIP_Z - edge_z])
    # out to a safe pose in front of the microscope keeping the hand's orientation, joint-space round to above the
    # push start, then down (dev/push_path2.py)
    yield from arm.move(np.array([0.24, 0.10, 1.00]), arm.rot, 1.5)
    yield from arm.move_joints(start + np.array([-0.08, 0.0, 0.10]), Rp, 2.5, grip=PUSH_GAP * 1000 / GAP_MM)
    yield from arm.move(start + np.array([0.0, 0.0, 0.006]), Rp, 1.5)
    # close the loop on the fingers' measured pose (FK): lower edges TIP_Z over the stage, square to x
    cmd, Rq, yaw = start.copy(), Rp, 0.0
    low_local = np.array([0.0, bdown * E1[1], E1[2]])
    for _ in range(4):
        yield from arm.move(cmd, Rq, 0.5)
        yield from wait(0.25)
        p, R = tcp_actual(arm)
        err = np.array([start[0] - p[0], start[1] - p[1], g.STAGE_Z + TIP_Z - (p + R @ low_local)[2]])
        yaw_err = math.atan2(R[1, 2], R[0, 2])
        if np.linalg.norm(err) < 0.0001 and abs(yaw_err) < math.radians(0.2):
            break
        cmd = cmd + err
        yaw -= yaw_err
        Rq = rotz(yaw) @ Rp
    target = g.BACK_EDGE_X - (0.003 if variant == "short" else 0.0)
    hold = {"y": start[1], "yaw": yaw}

    def gaps():
        S = ctx.task._STATE
        off = g.BACK_EDGE_X - target
        return max(S["back"]) / 1000 - off, min(S["back"]) / 1000 - off

    def step_push(cmd, v):
        p, R = tcp_actual(arm)
        ctx.flags["tip_z"] = ((p + R @ low_local)[2] - g.STAGE_Z) * 1000
        hold["yaw"] -= 0.5 * math.atan2(R[1, 2], R[0, 2])
        return cmd + np.array([v * 0.05, 0.5 * (hold["y"] - p[1]), 0.8 * (g.STAGE_Z + TIP_Z - (p + R @ low_local)[2])])

    # 20 mm/s, then 2 mm/s over the last 1.5 mm until the first end of the back edge meets the template, then on at
    # 2 mm/s until the other end does too (the blocked side's finger stops, the other squares the slide), at most 1.5 mm
    # past the first touch and only while the slide still moves
    over, still, last = 0.0, 0, slide_pose(ctx)[0]
    for k in range(600):
        far, near = gaps()
        if far <= 0.00008 or over >= 0.0015 or still >= 4 or (variant == "short" and near <= 0.00005):
            break
        v = min(0.015, 0.002 + 0.001 * k) if near > 0.0015 else 0.002     # eased in to 15 mm/s
        if near <= 0.00005:
            over += v * 0.05
        cmd = step_push(cmd, v)
        Rq = rotz(hold["yaw"]) @ Rp
        arm.aim(cmd, Rq)
        yield
        now = slide_pose(ctx)[0]
        still = still + 1 if (over > 0 and np.linalg.norm(now - last) < 0.00002) else 0
        last = now
    yield from wait(0.3)
    yield from arm.move(arm.pos + np.array([-0.002, 0.0, 0.0]), Rq, 0.4)
    yield from arm.move(arm.pos + np.array([-0.03, 0.0, 0.02]), Rq, 1.2)


def push_rot() -> np.ndarray:
    """Pointing +x, pitched BETA down (the fingers' lower edges level), closing axis along y (pointing -y: the wrist's
    reachable roll)."""
    return R_from([math.cos(BETA), 0.0, -math.sin(BETA)], [0.0, -1.0, 0.0])


PUSH_GAP = 0.040                    # pads' gap for the two-fingertip push


STROKE_PITCH = math.radians(18.0)   # "strokes" push: closed fingertips, pitched so their tips are the lowest point
STROKE_OFF = 0.014                  # push point this far from the slide's middle when correcting its yaw
YAW_BAND = math.radians(0.6)        # correct the yaw beyond this


def push_strokes(ctx, arm, variant):
    """The strict-rule push: closed fingertips (stacked: their end is flat across the fingers' height, so the contact
    normal lies in the stage plane), in short strokes. A part pushed at one point turns away, so each stroke pushes at
    the middle while the slide is square and at its lagging end (STROKE_OFF out) when it is not; between strokes the
    tip backs off and lifts before moving sideways (the pads' friction would drag the slide). Once the first end of
    the back edge meets the template, strokes at the lagging end turn the slide flush about that corner."""
    Rp = R_from([math.cos(STROKE_PITCH), 0.0, -math.sin(STROKE_PITCH)],
                [-math.sin(STROKE_PITCH), 0.0, -math.cos(STROKE_PITCH)])
    tip_local = np.array([0.0034 if Rp[2, 0] < 0 else -0.0034, 0.0, 0.0128])
    tipv = Rp @ tip_local
    lean = (0.001 - TIP_Z) * math.tan(STROKE_PITCH)
    target = g.BACK_EDGE_X - (0.003 if variant == "short" else 0.0)

    def gaps():
        S = ctx.task._STATE
        off = g.BACK_EDGE_X - target
        return max(S["back"]) / 1000 - off, min(S["back"]) / 1000 - off

    def edge_point(y_off):
        """The slide's front edge at y_off from its middle (along its length), and its yaw (+: the +y end lags)."""
        q_s, Q_s = slide_pose(ctx)
        psi = math.atan2(-Q_s[0, 0], Q_s[1, 0])
        return q_s + y_off * Q_s[:, 0] + (g.SLIDE_W / 2) * Q_s[:, 1], psi

    def tip_now():
        p, R = tcp_actual(arm)
        return p + R @ tip_local

    p0, psi0 = edge_point(0.0)
    start = np.array([p0[0] - 0.005 - lean - tipv[0], p0[1] - tipv[1], g.STAGE_Z + TIP_Z - tipv[2]])
    yield from arm.move(np.array([0.24, 0.10, 1.00]), arm.rot, 1.5, grip=0.0)
    yield from arm.move_joints(start + np.array([-0.06, 0.0, 0.08]), Rp, 2.5)
    yield from arm.move(start + np.array([0.0, 0.0, 0.006]), Rp, 1.5)
    cmd, Rq, yaw = start.copy(), Rp, 0.0
    for _ in range(4):                                     # the tip by FK: TIP_Z over the stage, end face square to x
        yield from arm.move(cmd, Rq, 0.5)
        yield from wait(0.2)
        p, R = tcp_actual(arm)
        err = (start + tipv) - (p + R @ tip_local)
        yaw_err = math.atan2(R[1, 2], R[0, 2])
        if np.linalg.norm(err) < 0.0001 and abs(yaw_err) < math.radians(0.2):
            break
        cmd, yaw = cmd + err, yaw - yaw_err
        Rq = rotz(yaw) @ Rp
    y_off = 0.0
    ctx.flags["push_started"] = True
    for stroke in range(40):
        far, near = gaps()
        if far <= 0.00008 or (variant == "short" and near <= 0.00005):
            break
        _, psi = edge_point(0.0)
        touched = near <= 0.00005
        if touched or abs(psi) > YAW_BAND:
            want_off = math.copysign(STROKE_OFF, psi)      # the lagging end
        else:
            want_off = 0.0
        pe, _ = edge_point(want_off)
        if abs(want_off - y_off) > 0.001 or stroke == 0:
            # reposition: back 1.5 mm, up 1.5 mm, across, down, 1.5 mm in front of the edge
            y_off = want_off
            yield from arm.move(cmd + np.array([-0.0015, 0.0, 0.0015]), Rq, 0.4)
            cmd = cmd + np.array([-0.0015, 0.0, 0.0015])
            tgt_tip = np.array([pe[0] - 0.0015 - lean, pe[1], g.STAGE_Z + TIP_Z + 0.0015])
            cmd = cmd + (tgt_tip - tip_now())
            yield from arm.move(cmd, Rq, 0.6)
            cmd = cmd + np.array([0.0, 0.0, -0.0015])
            yield from arm.move(cmd, Rq, 0.4)
        # advance: until the slide has gone 3 mm (1 mm when correcting), the decision changes, or it is stuck
        q_start = slide_pose(ctx)[0]
        last, stuck, moved = q_start, 0, 0.0
        limit = 0.003 if y_off == 0.0 else 0.001
        for _ in range(160):
            far, near = gaps()
            if far <= 0.00008 or (variant == "short" and near <= 0.00005):
                break
            _, psi = edge_point(0.0)
            if y_off == 0.0 and abs(psi) > YAW_BAND:
                break
            if y_off != 0.0 and math.copysign(1.0, psi) != math.copysign(1.0, y_off):
                break
            if (near <= 0.00005) != touched:
                break
            v = 0.008 if near > 0.0015 else 0.002
            tip = tip_now()
            ctx.flags["tip_z"] = (tip[2] - g.STAGE_Z) * 1000
            ctx.flags["psi"] = math.degrees(psi)
            cmd = cmd + np.array([v * 0.05, 0.5 * (pe[1] - tip[1]), 0.8 * (g.STAGE_Z + TIP_Z - tip[2])])
            arm.aim(cmd, Rq)
            yield
            now = slide_pose(ctx)[0]
            moved = float(np.linalg.norm(now - q_start))
            stuck = stuck + 1 if (moved > 0.0001 and np.linalg.norm(now - last) < 0.00002) else 0
            last = now
            if moved >= limit or stuck >= 4:
                break
    yield from wait(0.3)
    yield from arm.move(arm.pos + np.array([-0.002, 0.0, 0.0]), Rq, 0.4)
    yield from arm.move(arm.pos + np.array([-0.03, 0.0, 0.02]), Rq, 1.2)


# -- right arm ---------------------------------------------------------------------------------------------------------
def knob_pose(phi: float, carriage=(0.0, 0.0), lead: float = 0.0):
    """The right TCP pose round the lever knob at caliper angle ``phi``: the pads (gap CAPTURE) straddle the knob across
    its path; ``lead`` shifts the hand along the closing axis so the trailing pad meets the knob."""
    a = np.array([math.sin(KNOB_TILT) * math.cos(KNOB_AZ), math.sin(KNOB_TILT) * math.sin(KNOB_AZ), -math.cos(KNOB_TILT)])
    R = rotz(phi) @ R_from(a, np.cross(a, [0.0, 0.0, 1.0]))
    k = g.caliper_point(g.KNOB, phi, carriage)
    return M((k[0], k[1], KNOB_H)) + lead * opening_side(R), R


def opening_side(R) -> np.ndarray:
    """The closing axis' sign that points along the knob's opening direction (CCW about the pivot)."""
    c = R[:, 0]
    t = np.array([-(g.KNOB[1] - g.PIVOT[1]), g.KNOB[0] - g.PIVOT[0], 0.0])
    return c if c @ t > 0 else -c


CAPTURE = 0.012                     # pads' gap round the 8 mm knob (a loose capture: no clamping on the carriage)
LEAD = CAPTURE / 2 - g.KNOB_R - 0.0004   # the trailing pad 0.4 mm into the knob's side


def open_lever(ctx, arm, variant):
    """Take the knob loosely between the pads, the trailing pad against it, and swing the arm open to OPEN_TO (the
    pad pushes the knob; the spring keeps it against the pad). The command keeps the measured sag correction and
    follows the caliper's measured angle."""
    S = ctx.task._STATE
    phi0 = S["cal"]
    tcp, Rk = knob_pose(phi0, S["carriage"])
    a = Rk[:, 2]
    yield from arm.move_joints(tcp - 0.05 * a, Rk, 2.5, grip=CAPTURE * 1000 / GAP_MM)
    yield from settle_to(arm, tcp - 0.012 * a, Rk)
    corr = arm.pos - (tcp - 0.012 * a)                     # the sag correction found above
    yield from arm.move(tcp + corr, Rk, 1.0)
    p, R = knob_pose(phi0, S["carriage"], LEAD)
    yield from arm.move(p + corr, R, 0.6)
    yield from wait(0.3)
    n = 50
    for k in range(1, n + 1):
        phi = phi0 + (OPEN_TO - phi0) * (1 - math.cos(math.pi * k / n)) / 2
        p, R = knob_pose(phi, S["carriage"], LEAD)
        arm.aim(p + corr, R)
        yield
    # hold: trim the correction until the arm stands at OPEN_TO (the spring presses the knob on the pad)
    for _ in range(20):
        err = g.caliper_point(g.KNOB, OPEN_TO) - g.caliper_point(g.KNOB, S["cal"])
        corr = corr + 0.3 * np.r_[err, 0.0]
        p, R = knob_pose(OPEN_TO, S["carriage"], LEAD)
        arm.aim(p + corr, R)
        yield
    ctx.flags["lever_open"] = True
    ctx.flags["corr"] = corr


def close_lever(ctx, arm, variant):
    """Let the arm return at TIP_SPEED at its tip, the knob riding on the retreating pad, until the tip meets the slide;
    the hand goes on the same way at the same speed while the spring carries the slide onto the jaw and the arm comes to
    rest, until the pad has left the knob; then it lifts off."""
    S = ctx.task._STATE
    corr = ctx.flags["corr"].copy()
    if variant == "snap":
        # let go of the open lever: the hand rises off the knob at once and the spring snaps the arm onto the slide
        yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.03]), arm.rot, 0.25)
        yield from arm.move(arm.pos - 0.05 * arm.rot[:, 2], arm.rot, 1.0)
        yield from wait(1.0)
        return
    dphi = TIP_SPEED * 0.05 / g.CAL_R_TIP
    phi = S["cal"]
    prev = phi0 = S["cal"]
    for _ in range(800):
        if S["tip_slide"] > g.TOUCH_F and phi0 - S["cal"] > math.radians(1.0):   # a touch made by this return
            break
        # the knob rides on the retreating pad; a governor holds the command a step whenever the tip's measured speed
        # passes 1.5 x TIP_SPEED (the arm's bend and the spring's falling force make the pad's actual speed differ)
        v_meas = (prev - S["cal"]) * g.CAL_R_TIP / 0.05
        prev = S["cal"]
        if v_meas < 1.5 * TIP_SPEED:
            phi -= dphi
        p, R = knob_pose(phi, S["carriage"], LEAD)
        arm.aim(p + corr, R)
        yield
    # the tip is on the slide: the hand keeps going the same way at the same speed (straight on along the knob's closing
    # direction): the knob follows while the spring carries the slide onto the jaw, then stops; the pad then leaves it
    # gradually as the arm's bend (its load) runs out, so the arm comes to rest under control and is not pushed open
    # again. Stop once the pad has been off the knob 3 steps with the slide on the jaw (at most 30 mm).
    o = opening_side(arm.rot)
    t_close = -o / np.linalg.norm(o)
    r_knob = math.hypot(g.KNOB[0] - g.PIVOT[0], g.KNOB[1] - g.PIVOT[1])
    v_knob = TIP_SPEED * r_knob / g.CAL_R_TIP
    cmd, off, moved = arm.pos.copy(), 0, 0.0
    for _ in range(600):
        cmd = cmd + v_knob * 0.05 * t_close
        moved += v_knob * 0.05
        arm.aim(cmd, arm.rot)
        yield
        off = off + 1 if S["tip_hand"] <= g.TOUCH_F else 0
        if (off >= 3 and max(S["jaw"]) <= 0.10) or moved > 0.030:
            break
    yield from arm.move(arm.pos + 0.001 * t_close, arm.rot, 0.3)
    # off the knob: straight up (the pads' faces are vertical: they slide off the knob's top), out along the approach,
    # and away from the microscope before going home (dev/right_exit.py)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.03]), arm.rot, 1.0)
    yield from arm.move(arm.pos - 0.05 * arm.rot[:, 2], arm.rot, 1.0)
    yield from arm.move(np.array([0.22, -0.20, 1.00]), arm.rot, 1.5)


# -- the script ------------------------------------------------------------------------------------------------------------
def left_script(ctx, variant, upto):
    arm = ctx.left
    Rg = yield from pick(ctx, arm, variant)
    if upto == "pick" or variant == "face":
        yield from wait(2.0)
        yield from arm.move(arm.pos + np.array([0, 0, 0.04]), arm.rot, 1.5)
        yield from wait(3.0)
        return
    Rset = yield from lift_and_turn(ctx, arm, Rg)
    if upto == "turn":
        yield from wait(4.0)
    yield from set_down(ctx, arm, Rset)
    if upto in ("turn", "set"):
        return
    while not ctx.flags.get("lever_open"):
        yield
    yield from (push if os.environ.get("MS_PUSH") == "pair" else push_strokes)(ctx, arm, variant)
    ctx.flags["pushed"] = True
    yield from arm.move(np.array([0.22, 0.12, 1.00]), arm.rot, 1.5)
    yield from arm.go_home(2.5)


def right_script(ctx, variant, upto):
    arm = ctx.right
    if upto in ("pick", "turn", "set") or variant == "face":
        return
    while not ctx.flags.get("set_down_started"):
        yield
    yield from open_lever(ctx, arm, variant)
    if variant == "brush":
        yield from brush_then_drop(ctx, arm)
        yield from arm.go_home(2.5)
        return
    while not ctx.flags.get("pushed"):
        yield
    yield from close_lever(ctx, arm, variant)
    yield from arm.go_home(2.5)


def brush_then_drop(ctx, arm):
    """Check variant: once the push starts, the hand holds the arm with its tip 0.3 mm into the slide's path (measured
    angle, in closed loop), so the slide brushes it near the end of the push while a hand is on the lever; if it never
    touched, the hand closes the arm until it does. After the push the hand opens the arm 8 degrees off the slide and
    lets go: the arm snaps onto the slide. The landing is the snap, not the brush, so the run must be capped."""
    S = ctx.task._STATE
    corr = ctx.flags["corr"].copy()
    while not ctx.flags.get("push_started"):
        yield
    p_s, R_s = slide_pose(ctx)
    m = p_s + g.slide_corners() @ R_s.T - np.array(ctx.task.M0) - np.array([*S["carriage"], 0.0])
    right_y = m[:, 1].min()

    def tip_y(phi):
        return max(g.caliper_point((x, g.ARM_TIP[1][1]), phi)[1] for x in g.ARM_TIP[0])

    lo, hi = 0.0, OPEN_TO
    for _ in range(30):                                    # the angle putting the tip 0.3 mm into the slide's path
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if tip_y(mid) > right_y + 0.0003 else (lo, mid)
    target, phi, touched = lo, S["cal"], 0
    while not ctx.flags.get("pushed"):
        phi += float(np.clip(target - phi, -math.radians(0.5), math.radians(0.5)))
        err = g.caliper_point(g.KNOB, phi) - g.caliper_point(g.KNOB, S["cal"])
        corr = corr + 0.2 * np.r_[err, 0.0]                # hold the MEASURED angle at the target
        pk, Rk = knob_pose(phi, S["carriage"], LEAD)
        arm.aim(pk + corr, Rk)
        touched += S["tip_slide"] > g.TOUCH_F
        yield
    for _ in range(200):                                   # no brush yet: close by hand until the tip touches
        if touched >= 10:
            break
        if S["tip_slide"] > g.TOUCH_F:
            touched += 1
        else:
            phi -= math.radians(0.3)
        pk, Rk = knob_pose(phi, S["carriage"], LEAD)
        arm.aim(pk + corr, Rk)
        yield
    ctx.flags["brushed"] = touched
    for k in range(1, 21):                                 # open it 8 degrees off the slide, by hand
        pk, Rk = knob_pose(phi + math.radians(8.0) * k / 20, S["carriage"], LEAD)
        arm.aim(pk + corr, Rk)
        yield
    yield from wait(0.5)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.03]), arm.rot, 0.25)     # let go
    yield from arm.move(arm.pos - 0.05 * arm.rot[:, 2], arm.rot, 1.0)
    yield from arm.move(np.array([0.22, -0.20, 1.00]), arm.rot, 1.5)


def script(ctx, variant: str, upto: str):
    from demokit import together

    def left():
        gen = left_script(ctx, variant, upto)
        yield from gen

    def flagger():
        # the right arm starts on the lever once the left has the slide over the stage
        while True:
            S = ctx.task._STATE
            if S["step"] > 0 and ctx.left.pos[2] > g.STAGE_Z and abs(ctx.left.rot[2, 2]) < 0.6 \
                    and ctx.left.pos[1] < 0.15:
                ctx.flags["set_down_started"] = True
                return
            yield

    yield from together(left(), flagger(), right_script(ctx, variant, upto))
    yield from wait(1.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    p_s, R_s = slide_pose(ctx)
    tl, Rl = tcp_actual(ctx.left)
    rel = Rl.T @ (p_s - tl)
    m = p_s - np.array(ctx.task.M0)
    tr, _ = tcp_actual(ctx.right)
    return (f"slideM {np.round(m * 1000, 1).tolist()} up {S['up']:.3f} relL {np.round(rel * 1000, 2).tolist()} "
            f"Lerr {np.linalg.norm(tl - ctx.left.pos) * 1000:.1f} Rerr {np.round((tr - ctx.right.pos) * 1000, 1).tolist()} "
            f"tipz {ctx.flags.get('tip_z', 0):.2f} gapL {finger_gap(ctx.left) * 1000:.1f} gapR {finger_gap(ctx.right) * 1000:.1f} "
            f"grip_f {S['grip_f']:.2f} cal {math.degrees(S['cal']):.1f} car {np.round(np.array(S['carriage']) * 1000, 2).tolist()} "
            f"tip {S['tip_slide']:.2f}/{S['tip_hand']:.2f} back {S['back']} jaw {S['jaw']} corners {np.round(S['corners'] * 1000, 2).tolist()} "
            f"flat {S['flat']} held {S['held']} seated {S['seated']} under {S['under']} gap {S['gap'] * 1000:.0f} "
            f"rung {S['rung']} landing {S['landing']} brushed {ctx.flags.get('brushed')} hard {S['hard']} why {S['why']} face {S['face']} optic {S.get('optic_who')}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--variant", default="full", choices=["full", "face", "snap", "short", "brush", "nothing"])
    ap.add_argument("--upto", default="full", choices=["pick", "turn", "set", "full"])
    a = ap.parse_args()
    if a.variant == "nothing":
        def scr(ctx):
            yield from wait(5.0)
    else:
        def scr(ctx):
            yield from script(ctx, a.variant, a.upto)
    run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=int(os.environ.get("MS_LOG_EVERY", "10")))
