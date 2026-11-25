"""Scripted full-score demo (reads object poses from the simulator: the phone's pose).

1. Left arm, pick-up: the gripper tilted 20 degrees to the stand's recline, a pinch across the phone's thickness on its
   top edge, centred 16.5 mm from the screen-left edge (55 mm from the camera side, clear of the camera corner);
   lifted 45 mm along the phone so its bottom edge clears the lip, stood upright and carried to the cross, where it
   turns the camera to face 100 degrees (board 1 in its first frame, the left arm's base 7 degrees outside it).
2. Right arm, start tap: a fingertip end face on the on-screen shutter, in closed loop on the touch (0.2 s on the
   glass), then home.
3. Left arm, the sweep: the camera turned from 100 to -14 degrees in 10 s (17 deg/s at the most), the hand circling
   so that the lens stays over the cross's centre: every step the hand's command is solved for the phone's goal pose
   (lens over the cross, camera level) from the phone's pose in the hand, with a correction updated in closed loop
   from the phone's measured pose.
4. Right arm, stop tap, then home.
5. Left arm, set-down: the camera turned back, the phone tilted to the recline, lowered into the stand's slot and let
   go; the hand backs off along the phone and goes home.

    python demo.py task.py demo_out [--max-steps N] [--no-record] [--variant NAME | --variants A,B,...] [--slit]
                                    [--hold S] [--upto STAGE]

Variants (scorer checks), each printing its mode:
- ``wrist``: the same pinch, the sweep (from 104 degrees) turned by the wrist alone (the tool point fixed where it
  is when the lens is over the cross with the camera at 0 degrees);
- ``plateau``: a pinch on the top edge over the camera plateau, the sweep turned by the wrist alone (the wrist's
  axis about 3 mm from the lens);
- ``button``: the phone lifted by its width 45 mm below the top (the pads on the volume and side buttons);
- ``fast``: the sweep at 60 deg/s;
- ``short``: the stop tap with the camera at 40 degrees;
- ``nostop``: no stop tap (the phone turned back and returned);
- ``keep``: the full demo, the phone not put back (held at the end);
- ``drop``: the full capture, then the phone let go 80 mm above the mat;
- ``nothing``: the arms stay still.
Writes one MP4 per camera, setup.png (the first top-camera frame), result.json, extra.json, captures.json, and with
``--slit`` the lens's path and the arms' poses during the capture (pano_path.npz) for the panorama picture
(dev/panorama.py).
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

TRACE = bool(os.environ.get("PANO_TRACE"))
TOPZ = g.TOP
L3 = np.array([g.CROSS[0], g.CROSS[1], TOPZ + g.LENS_H])      # the lens's goal, world
PINCH_Y = -g.PW / 2 + 0.0165        # the pinch's centre along the top edge (phone y): 16.5 mm from the screen-left edge
PINCH_DEPTH = 0.003                 # the tool point this far below the top edge (fingertips 16 mm below it)
PLATEAU_Y = g.PW / 2 - g.LENS_FROM_SIDE   # --plateau: the pinch centred over the lens
OPEN_PINCH = 0.024                  # the fingers' gap for the pinch approach
C0, C1 = 100.0, -14.0               # the camera's heading at the start and the stop tap
SWEEP_S = 10.0                      # the sweep's duration (smooth start and stop): 17 deg/s at the most
TAP_TILT = {"start": 75.0, "end": 75.0}
WRIST_C0, WRIST_C_AXIS = 104.0, 0.0  # --wrist: the capture starts at 104 degrees (board 1 stays in full with the lens
                                    # 67 mm off the cross); the lens is over the cross with the camera at 0 degrees


def grip_for(width_m: float) -> float:
    return max(0.0, min(1.0, width_m * 1000 / g.GAP_MM))


def R_from(a, c) -> np.ndarray:
    """Gripper rotation with approach ``a`` (its z) and closing axis ``c`` (its x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rotvec(R) -> np.ndarray:
    a = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a))
    return v * a


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


def phone_pose(ctx):
    st = ctx.states()["phone"]
    return np.asarray(st.pos, float), ctx.task._rotation(st.quat_xyzw)


def lens_of(p, R) -> np.ndarray:
    return p + R @ g.lens_local()


def phone_for_lens(c_deg: float, lens=L3) -> tuple[np.ndarray, np.ndarray]:
    """The phone's pose (world) upright with its camera facing heading ``c_deg`` and its lens at ``lens``."""
    R = g.phone_upright(c_deg - 180.0)
    return lens - R @ g.lens_local(), R


def monitor(ctx):
    return ctx.task._STATE["mon"]


def settle_to(arm, pos, rot, tol=0.0006, rounds=3, seconds=0.6):
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


class PhoneHold:
    """The phone held in one hand. The hand is commanded so the phone's measured pose reaches a goal: the hand's
    command is solved from the phone's pose in the hand (measured when it was gripped), plus a correction in position
    and rotation updated in closed loop from the phone's measured pose (the wrist gives a little under the phone)."""

    MAX_DP, MAX_DR = 0.020, math.radians(8.0)

    def __init__(self, ctx, arm):
        self.ctx, self.arm = ctx, arm
        p, R = phone_pose(ctx)
        ph, Rh = tcp_actual(arm)
        self.rel_p, self.rel_R = Rh.T @ (p - ph), Rh.T @ R
        self.dp, self.dr = np.zeros(3), np.zeros(3)
        self.goal = (p, R)
        self.worst = 0.0

    def command(self, p_goal, R_goal):
        Rh = R_goal @ self.rel_R.T
        ph = np.asarray(p_goal, float) - Rh @ self.rel_p
        ang = float(np.linalg.norm(self.dr))
        Rc = turn(self.dr / ang, ang) if ang > 1e-9 else np.eye(3)
        # the rotation correction turns the hand about the lens, so it does not move the lens
        lens = lens_of(np.asarray(p_goal, float), R_goal)
        return Rc @ (ph - lens) + lens + self.dp, Rc @ Rh

    def error(self, p_goal, R_goal):
        p, R = phone_pose(self.ctx)
        return lens_of(np.asarray(p_goal), R_goal) - lens_of(p, R), rotvec(R_goal @ R.T)

    def update(self, gain=0.35):
        e_p, e_r = self.error(*self.goal)
        self.worst = max(self.worst, float(np.linalg.norm(e_p)))
        dp = self.dp + gain * e_p
        n = float(np.linalg.norm(dp))
        self.dp = dp if n <= self.MAX_DP else dp * self.MAX_DP / n
        dr = self.dr + gain * e_r
        n = float(np.linalg.norm(dr))
        self.dr = dr if n <= self.MAX_DR else dr * self.MAX_DR / n
        return e_p, e_r

    def track(self, p_goal, R_goal, gain=0.35):
        """One step: fold in the last step's error, then aim at this step's goal."""
        self.update(gain)
        self.goal = (np.asarray(p_goal, float), np.asarray(R_goal, float))
        self.arm.aim(*self.command(*self.goal))

    def move(self, p_goal, R_goal, seconds, gain=0.35, joints=False):
        """A smooth move of the phone to a goal pose (position straight, rotation slerped), tracked every step."""
        from yam_arena import ik

        if joints:
            p, R = self.command(p_goal, R_goal)
            yield from self.arm.move_joints(p, R, seconds)
            self.goal = (np.asarray(p_goal, float), np.asarray(R_goal, float))
            return
        p0, R0 = self.goal
        n = max(1, round(seconds * 20))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.track(p0 + s * (np.asarray(p_goal) - p0), ik.slerp_matrix(R0, R_goal, s), gain)
            yield

    def settle(self, seconds=0.6, gain=0.35):
        for _ in range(max(1, round(seconds * 20))):
            self.track(*self.goal, gain)
            yield


def pivot(hold, c_from, c_to, seconds, ramp=True, lens=L3, j6_only_tcp=None):
    """Turn the camera from heading c_from to c_to with the lens held at ``lens`` (or, with ``j6_only_tcp``, the
    hand's tool point held fixed there and the hand only turning about the vertical)."""
    from yam_arena import ik

    n = max(1, round(seconds * 20))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n) if ramp else k / n
        c = c_from + s * (c_to - c_from)
        p_g, R_g = phone_for_lens(c, lens)
        if j6_only_tcp is None:
            hold.track(p_g, R_g)
        else:
            Rh = R_g @ hold.rel_R.T
            hold.goal = (p_g, R_g)
            hold.arm.aim(j6_only_tcp, Rh)
        yield


# -- the right hand's taps ----------------------------------------------------------------------------------------------
def tap_rot(R, tilt):
    """(approach, rotation) for a tap: into the screen, tipped down by 90 - tilt degrees, fingers closing along the
    phone's width."""
    n = R[:, 0]
    down = math.radians(90.0 - tilt)
    a = -n * math.cos(down) + np.array([0.0, 0.0, -1.0]) * math.sin(down)
    return a, R_from(a, R[:, 1])


def tap_pose(ctx, arm, tilt):
    """The tool point and rotation that put the closed fingertips' touching edge on the shutter."""
    from yam_arena.analysis import fk_link_poses_world
    from yam_arena.constants import ARM_JOINTS

    p, R = phone_pose(ctx)
    n = R[:, 0]
    sh = p + R @ g.shutter_local()
    a, Rg = tap_rot(R, tilt)
    tcp = sh - a * g.TIP
    ends = ctx.task._end_faces()
    for _ in range(3):
        q = arm.solve_far(tcp, Rg)
        d = {nm: float(v) for nm, v in zip(ARM_JOINTS, q)}
        d["joint7"] = d["joint8"] = 0.0
        poses = fk_link_poses_world(d, arm.base)
        P = np.concatenate([ends[k] @ poses[k][:3, :3].T + poses[k][:3, 3] for k in ends])
        depth = (P - sh) @ (-n)                         # how far past the glass's plane, along its inward normal
        touch = P[depth >= depth.max() - 0.0005].mean(0)
        off = touch - sh
        off -= n * (off @ n)
        tcp = tcp - off + (-n) * (-(depth.max()))       # the touching edge onto the shutter, just on the glass
    return tcp, Rg, a


def tap_via(ctx, tilt):
    """The stop tap's way in, clear of the phone and the left arm (dev/probe_tap_via.py): a point 30 mm further back
    than the 30 mm pre-tap point, 40 mm up and 40 mm toward the phone's camera side, with the hand tipped 45 degrees
    down."""
    p, R = phone_pose(ctx)
    a, Rg = tap_rot(R, tilt)
    sh = p + R @ g.shutter_local()
    via = sh - a * (g.TIP + 0.060) + np.array([0.0, 0.0, 0.040]) + R[:, 1] * 0.040
    return via, tap_rot(R, 45.0)[1]


def tap(ctx, arm, tilt, label, via=None):
    """Approach 30 mm off (through ``via`` if given: joint space to it, then a straight line), then in at 0.5 mm a
    step until the monitor reads this hand's end face on the glass; hold 0.1 s; back off (the same way)."""
    tcp, Rg, a = tap_pose(ctx, arm, tilt)
    if via is None:
        yield from arm.move_joints(tcp - 0.030 * a, Rg, 2.0, grip=0.0)       # fingers closed: their end faces tap
    else:
        yield from arm.move_joints(via[0], via[1], 2.5, grip=0.0)
        yield from arm.move(tcp - 0.030 * a, Rg, 1.2)
    yield from settle_to(arm, tcp - 0.006 * a, Rg, rounds=2, seconds=0.8)
    M = monitor(ctx)
    cmd = tcp - 0.006 * a
    rec = ctx.flags.setdefault("taps", {})
    for k in range(40):
        cmd = cmd + 0.0005 * a
        arm.aim(cmd, Rg)
        yield
        if any(key.startswith(arm.side) for key in M.info.get("touching", {})):
            rec[label] = dict(step=M.step, steps_in=k + 1, at=M.info["touching"])
            break
    cmd = tcp_actual(arm)[0] - 0.0005 * a              # stop pressing: hold where the fingertips are
    arm.aim(cmd, Rg)
    yield from wait(0.1)
    for _ in range(10):                                 # off the glass at once: 2 mm a step
        cmd = cmd - 0.002 * a
        arm.aim(cmd, Rg)
        yield
    yield from arm.move(cmd - 0.015 * a, Rg, 0.4)
    if via is not None:
        yield from arm.move(via[0], via[1], 1.0)


# -- the left hand's pick-up and set-down --------------------------------------------------------------------------------
def pick(ctx, arm, y_pinch=PINCH_Y, depth=PINCH_DEPTH):
    p, R = phone_pose(ctx)
    n, u = R[:, 0], R[:, 2]
    top = p + R @ np.array([0.0, y_pinch, g.PH / 2])
    Rg = R_from(-u, n)
    yield from arm.move_joints(top + 0.050 * u, Rg, 2.5, grip=grip_for(OPEN_PINCH))
    yield from settle_to(arm, top + 0.010 * u, Rg, rounds=1, seconds=1.0)
    yield from settle_to(arm, top - depth * u, Rg, rounds=2, seconds=0.6)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.3)
    return PhoneHold(ctx, arm)


def pick_by_width(ctx, arm, below=0.045):
    """--button: the phone lifted by its width, the pads on the side edges ``below`` the top (the buttons)."""
    p, R = phone_pose(ctx)
    u, yv = R[:, 2], R[:, 1]
    at = p + R @ np.array([0.0, 0.0, g.PH / 2 - below])
    Rg = R_from(-u, -yv)                                # (closing along +y is out of the left arm's reach here)
    yield from arm.move_joints(at + (below + 0.050) * u, Rg, 2.5, grip=grip_for(0.090))
    yield from settle_to(arm, at, Rg, rounds=2, seconds=1.5)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.3)
    return PhoneHold(ctx, arm)


def lift_out(ctx, hold):
    """Lift 45 mm along the phone (out of the slot), then stand it upright, camera to 180 (screen to +x), raised."""
    p, R = phone_pose(ctx)
    u = R[:, 2]
    hold.goal = (p, R)
    yield from hold.move(p + 0.045 * u, R, 1.2)
    p1 = p + 0.045 * u
    R_up = g.phone_upright(0.0)
    yield from hold.move(np.array([p1[0] - 0.015, p1[1], TOPZ + g.LENS_H - (g.PH / 2 - g.LENS_FROM_TOP) + 0.020]), R_up,
                         1.5)
    yield from hold.settle(0.3)


def to_start(ctx, hold, c_start=C0, j6_tcp=None):
    """From upright at the stand to the lens over the cross, the camera turned to ``c_start``."""
    p_g, R_g = phone_for_lens(160.0)
    yield from hold.move(p_g + np.array([0, 0, 0.010]), R_g, 1.5, joints=True)
    yield from hold.move(p_g, R_g, 0.6)
    yield from hold.settle(0.3)
    yield from pivot(hold, 160.0, c_start, 3.0)
    yield from hold.settle(0.8)


def put_back(ctx, hold, c_now):
    """The camera turned back to 150; the phone carried round in front of the stand's back plate (never over it),
    tilted to the stand's recline 25 mm in front of its resting place, moved back onto the plate and lowered into the
    slot in closed loop, let go; the hand backs off along the phone (dev/probe_putback.py, path B)."""
    yield from pivot(hold, c_now, 150.0, 3.0)
    p_s, R_s = g.phone_in_stand()
    p_s = p_s + np.array([0, 0, TOPZ])
    n, u = R_s[:, 0], R_s[:, 2]
    p_a, _ = phone_for_lens(150.0)
    front = p_s + 0.025 * n + 0.020 * u
    yield from hold.move(p_a + np.array([0.040, 0.030, 0.0]), phone_for_lens(170.0)[1], 1.5)
    yield from hold.move(front + np.array([0.0, -0.060, 0.0]), R_s, 2.0)
    yield from hold.move(front, R_s, 1.2)
    yield from hold.settle(0.5)
    yield from hold.move(p_s + 0.002 * n + 0.020 * u, R_s, 1.2, gain=0.25)
    yield from hold.move(p_s + 0.002 * n + 0.003 * u, R_s, 1.2, gain=0.25)
    yield from hold.settle(0.6, gain=0.25)
    ctx.flags["before_release"] = phone_report(ctx)
    yield from hold.arm.set_grip(grip_for(OPEN_PINCH), 0.5)
    yield from wait(0.5)
    ph, Rh = tcp_actual(hold.arm)
    hold.arm.pos, hold.arm.rot = ph, Rh
    yield from hold.arm.move(ph - 0.045 * Rh[:, 2], Rh, 1.0)
    ctx.flags["after_release"] = phone_report(ctx)


def phone_report(ctx):
    p, R = phone_pose(ctx)
    return dict(pos_mm=np.round((p - np.array([0, 0, TOPZ])) * 1000, 1).tolist(),
                angles=[round(v, 2) for v in ctx.task.camera_angles(R)],
                lens_mm=np.round((lens_of(p, R) - L3) * 1000, 2).tolist())


# -- the lens's path (for the panorama picture only) ----------------------------------------------------------------
def with_path_log(ctx, gen, out):
    """Run ``gen``; while a capture runs (and 0.5 s either side) keep each step's phone pose and both arms' link
    poses (table frame), for dev/panorama.py to render the panorama the phone's camera saw."""
    M = monitor(ctx)
    rec = {"step": [], "pos": [], "R": [], "links": []}
    names = None
    tail = 0
    for _ in gen:
        live = M.capture is not None
        tail = 10 if live else max(0, tail - 1)
        o = ctx.task._STATE.get("obs")
        if (live or tail) and o is not None:
            if names is None:
                names = [(sd, n) for sd in ("left", "right") for n in o["poses"][sd]]
            rec["step"].append(M.step)
            rec["pos"].append(np.asarray(o["pos"]))
            rec["R"].append(np.asarray(o["R"]))
            rec["links"].append(np.array([o["poses"][sd][n] for sd, n in names]))
        yield
    if rec["step"]:
        np.savez_compressed(pathlib.Path(out) / "pano_path.npz", step=np.array(rec["step"]), pos=np.array(rec["pos"]),
                            R=np.array(rec["R"]), links=np.array(rec["links"]),
                            names=np.array([f"{sd}:{n}" for sd, n in names]))


# -- the scripts -------------------------------------------------------------------------------------------------------
def script(ctx, variant: str, upto: str):
    print(f"MODE variant={variant} upto={upto}", flush=True)
    Lt, Rt = ctx.left, ctx.right
    M = monitor(ctx)
    c0, c1 = C0, C1
    sweep_s = SWEEP_S
    if variant == "fast":
        sweep_s = (c0 - c1) / 60.0 * 1.0       # linear at 60 deg/s
    if variant == "short":
        c1 = 40.0
    if variant == "wrist":
        c0 = WRIST_C0
    # 1. pick-up
    if variant == "button":
        hold = yield from pick_by_width(ctx, Lt)
    elif variant == "plateau":
        hold = yield from pick(ctx, Lt, y_pinch=PLATEAU_Y)
    else:
        hold = yield from pick(ctx, Lt)
    ctx.flags["grip"] = phone_report(ctx)
    ik_phase(ctx, "pick")
    yield from lift_out(ctx, hold)
    ctx.flags["lifted"] = phone_report(ctx)
    ik_phase(ctx, "lift")
    if upto == "lift":
        return
    j6_tcp = None
    if variant in ("wrist", "plateau"):
        # the tool point fixed where it is when the lens is over the cross with the camera at WRIST_C_AXIS (the stop
        # tap then finds the phone 10 mm from where the demo has it)
        p_b, R_b = phone_for_lens(WRIST_C_AXIS)
        j6_tcp = hold.command(p_b, R_b)[0] if variant == "wrist" else None
    yield from to_start(ctx, hold, c0)
    ctx.flags["start_pose"] = phone_report(ctx)
    ik_phase(ctx, "to start")
    if variant == "plateau":
        j6_tcp = tcp_actual(Lt)[0]                  # the wrist's axis about 3 mm from the lens: keep the tool point
    if variant == "wrist":
        p_s, R_s = phone_for_lens(c0)
        yield from hold.arm.move(j6_tcp, R_s @ hold.rel_R.T, 1.5)
        yield from wait(0.5)
    if upto == "start":
        return
    # 2. start tap
    yield from tap(ctx, Rt, TAP_TILT["start"], "start")
    ctx.flags["after_start_tap"] = dict(capturing=M.capture is not None, step=M.step)
    yield from together(Rt.go_home(2.0), hold.settle(1.0) if j6_tcp is None else wait(1.0))
    if upto == "tap":
        return
    # 3. the sweep
    yield from pivot(hold, c0, c1, sweep_s, ramp=variant != "fast", j6_only_tcp=j6_tcp)
    if variant == "fast":
        yield from (hold.settle(2.0) if j6_tcp is None else wait(2.0))      # let the swing die down before the tap
    ctx.flags["sweep_end"] = phone_report(ctx)
    ik_phase(ctx, "start tap and sweep")
    ctx.flags["hold_worst_mm"] = round(hold.worst * 1000, 2)
    # 4. stop tap
    if variant == "nostop":
        yield from (hold.settle(2.0) if j6_tcp is None else wait(2.0))
    else:
        def hold_still():
            for _ in range(200):
                if ctx.flags.get("stop_done"):
                    return
                if j6_tcp is None:
                    hold.track(*hold.goal)
                yield

        def stop_tap():
            via = tap_via(ctx, TAP_TILT["end"]) if c1 < 0 else None
            yield from tap(ctx, Rt, TAP_TILT["end"] if c1 < 0 else 75.0, "end", via=via)
            ctx.flags["stop_done"] = True
            yield from Rt.go_home(2.0)

        yield from together(hold_still(), stop_tap())
    ctx.flags["saved"] = [{k: r[k] for k in ("full", "aligned", "err_mm", "warn", "level", "clear", "corner", "seen",
                                               "max_rate", "max_drift", "max_tilt", "max_roll", "sweep", "cover", "tier")}
                          for r in M.saved]
    if upto == "stop" or variant == "button":
        yield from hold.settle(1.0)
        return
    # 5. set-down
    if variant == "keep":
        yield from hold.settle(3.0)
        return
    if variant == "drop":
        p, R = hold.goal
        yield from hold.move(np.array([0.28, 0.18, TOPZ + 0.080 + g.PH / 2]), R, 2.0)
        yield from hold.settle(0.5)
        yield from Lt.set_grip(grip_for(OPEN_PINCH), 0.3)
        yield from wait(0.5)
        ph, Rh = tcp_actual(Lt)
        Lt.pos, Lt.rot = ph, Rh
        yield from Lt.move(ph + np.array([0, 0, 0.06]), Rh, 1.0)
        yield from Lt.go_home(2.0)
        return
    ik_phase(ctx, "stop tap")
    yield from put_back(ctx, hold, c1)
    yield from Lt.go_home(2.0)
    ik_phase(ctx, "set-down")
    ctx.flags["script_end_step"] = M.step


def ik_phase(ctx, name):
    """Each arm's worst IK miss (mm) during the phase just ended (the hand's command it could not reach)."""
    rec = ctx.flags.setdefault("ik_worst_mm", {})
    rec[name] = {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}
    ctx.left.worst_mm = ctx.right.worst_mm = 0.0


def log(ctx, step, info) -> str:
    M = ctx.task._STATE.get("mon")
    if M is None:
        return ""
    return (f"info {M.info} tier {M.tier} events {M.events[-3:]} saved "
            f"{[{k: r[k] for k in ('full', 'aligned', 'err_mm', 'warn', 'level', 'clear', 'tier')} for r in M.saved]}")


VARIANTS = ["full", "wrist", "plateau", "button", "fast", "short", "nostop", "keep", "drop", "nothing"]


CLOSE_EYE = (0.12, -0.28, 0.34)     # PANO_CLOSE_VIEW=1: the top camera's close view, from the cross (above the table)
CLOSE_TGT = (0.0, 0.0, 0.12)


def close_view(ctx):
    """Recorded runs only: move the top camera to a close view of the phone over the cross (the scene, the physics
    and the script are unchanged; only the top camera's picture is)."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)
    c = np.array([g.CROSS[0], g.CROSS[1], TOPZ])
    eye, tgt = c + np.array(CLOSE_EYE), c + np.array(CLOSE_TGT)
    cam.set_world_poses_from_view(torch.tensor(np.array([eye]), dtype=torch.float32, device=o.device) + o,
                                  torch.tensor(np.array([tgt]), dtype=torch.float32, device=o.device) + o)


def run_one(a, variant, out):
    def scr(ctx):
        ctx.flags["out"] = out
        if os.environ.get("PANO_CLOSE_VIEW"):
            close_view(ctx)
        if variant == "nothing":
            print("MODE variant=nothing", flush=True)
            gen = wait(5.0)
        else:
            gen = script(ctx, variant, a.upto)
        if a.slit:
            gen = with_path_log(ctx, gen, out)
        yield from gen
        print("FLAGS", json.dumps(ctx.flags, default=str), flush=True)

    run(a.task, out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=int(os.environ.get("PANO_LOG_EVERY", "20")), hold_s=a.hold)
    T = sys.modules["task"]
    M = T._STATE.get("mon")
    extra = {"variant": variant, "upto": a.upto, "tier": M.tier, "steps": M.step, "events": M.events,
             "saved": [{k: v for k, v in r.items() if k != "frames"} for r in M.saved], "moved": sorted(M.moved),
             "set_down": M.set_down, "fell": M.fell, "locked": M.locked, "info": M.info}
    print("EXTRA " + json.dumps(extra, default=str), flush=True)
    pathlib.Path(out).mkdir(parents=True, exist_ok=True)
    (pathlib.Path(out) / "extra.json").write_text(json.dumps(extra, indent=2, default=str))
    (pathlib.Path(out) / "captures.json").write_text(json.dumps(M.saved, default=str))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--slit", action="store_true", help="also keep the lens's path and the arms' poses for the panorama")
    ap.add_argument("--hold", type=float, default=2.0, help="seconds after the script ends (inf: to the step limit)")
    ap.add_argument("--upto", default="full", choices=["lift", "start", "tap", "stop", "full"])
    ap.add_argument("--variant", default="full", choices=VARIANTS)
    ap.add_argument("--variants", default=None,
                    help="comma-separated variants run one after another in this process (each from a reset), "
                         "each into OUT/NAME")
    a = ap.parse_args()
    if a.variants:
        for v in a.variants.split(","):
            assert v in VARIANTS, v
            run_one(a, v, str(pathlib.Path(a.out) / v))
    else:
        run_one(a, a.variant, a.out)
