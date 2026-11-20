"""Scripted full-score demo for the fold-the-glasses-into-their-case task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--cam 640x352]
    DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record     # prints the CAL numbers

The left hand takes the top rim of the left lens and lifts the glasses 25 mm off the table. The right hand pushes the
right temple round its hinge with the backs of its closed fingers until it lies along the back of the front. The left
hand sets the glasses down; the right takes the top rim of the right lens, the left lets go, and the right lifts them
again. The left hand pushes the left temple round over the right one and goes back. The right hand turns the folded
glasses lenses down, lowers them into the case far edge first and lets go; they settle flat on the case floor. It then
nudges the open lid forward past its spring's dead centre and the spring snaps it shut.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def _cols(x, z) -> np.ndarray:
    """A TCP rotation with the fingers closing along ``x`` and pointing along ``z``."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    z = z / np.linalg.norm(z)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it. ``qs`` records
    every step's joint command (for the offline clash check)."""

    def __init__(self, name: str, base_pos, solve: bool = True):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base, self.solve = name, np.asarray(base_pos, float), solve
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.worst_label = ""
        self.ends: list[tuple[np.ndarray, np.ndarray, str]] = []
        self.qs: list[tuple[np.ndarray, float, str]] = []
        self.label = "start"

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        if pe * 1000 > self.worst_mm:
            self.worst_mm, self.worst_label = pe * 1000, self.label
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, label: str = ""):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        self.label = label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        pos = np.asarray(pos, float)
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            p, r = p0 + s * (pos - p0), ik.slerp_matrix(r0, rot, s)
            if self.solve:
                self._solve(p, r)
            else:
                self.pos, self.rot = p, r
            self.qs.append((self.q.copy(), self.grip, label))
            yield
        self.ends.append((pos.copy(), np.asarray(rot, float).copy(), label))

    def path(self, poses, duration: float, label: str = ""):
        """Generator: follow a list of (pos, rot) evenly over ``duration`` (linear in the list's index)."""
        self.label = label
        n = max(1, round(duration * CONTROL_HZ))
        m = len(poses) - 1
        for k in range(1, n + 1):
            u = k / n * m
            i = min(int(u), m - 1)
            f = u - i
            from yam_arena import ik

            p = (1 - f) * poses[i][0] + f * poses[i + 1][0]
            r = ik.slerp_matrix(poses[i][1], poses[i + 1][1], f)
            if self.solve:
                self._solve(p, r)
            else:
                self.pos, self.rot = p, r
            self.qs.append((self.q.copy(), self.grip, label))
            yield
        self.ends.append((self.pos.copy(), self.rot.copy(), label))

    def set_grip(self, grip: float, duration: float = 0.8, label: str = "grip"):
        yield from self.move(self.pos, self.rot, duration, grip, label)

    def stay(self):
        self.qs.append((self.q.copy(), self.grip, self.label))


def both(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


# The demo runs open loop: it reads no simulator state. Where the glasses stand once they have settled on the table,
# and where they sit in the left hand once they are lifted, are measured once with DEMO_CALIBRATE=1 (which reads the
# simulator's poses at those moments and prints them) and fixed here. The layout is fixed (object_jitter=0) and the
# simulation deterministic.
CAL = {
    # the glasses settled on the table: origin moved (dx, dy, dz, m) and turned (yaw, pitch, roll, deg)
    "settled": (0.0, 0.0, -0.00057, 5e-05, -0.03076, -3e-05),
    "in_air": (-0.00046, 0.00041, -0.00121, 0.40519, 0.15503, 0.09706),     # the same, lifted by the left hand
    "in_air_r": (-0.00054, -0.00069, 0.0, 0.52703, 0.18456, -0.37471),      # the same, lifted again by the right hand
    "in_hand": (0.00096, 0.00118, -0.00052),     # the glasses' origin off where planned, lifted to go to the case (m)
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
_MEASURED: dict = {}

FOLD_LIFT = 0.025     # the temples are folded with the glasses held 25 mm above the table
HOLD_Y = 0.056        # the left hand holds the left lens's top rim this far left of the frame's middle
HOLD_DEPTH = 0.004    # its pad tips this far below the top of the rim
HOLD_ROLL = math.radians(15.0)   # the holding fingers lean 15 degrees out (keeps the two forearms apart)
TIPS = 0.013          # the TCP sits this far short of the pad tips
FACES = {1: 0.0085, -1: 0.010}   # closed fingers: the pad backs stand this far from the TCP along +x and -x
# Which way the fingers' closing axis (TCP x) points, to keep the wrist within its turn limits: +1 or -1 against the
# directions named where they are used.
SIGN = {"hold_left": -1, "hold_right": 1, "push_right": -1, "push_left": 1, "lid": -1}
PUSH_R = 0.060        # each temple is pushed 60 mm from its hinge
PUSH_END = {"push_right": math.radians(96.0), "push_left": math.radians(88.0)}   # how far round each push goes
PUSH_LEAN = math.radians(35.0)  # the pushing hand leans up to 35 degrees out (away from the holding hand)
PUSH_BACK = math.radians(20.0)  # and 20 degrees back, away from the way it pushes
PUSH_LOW = 0.0045     # the pushing pad tips this far below the hinge's middle: the fingers widen 20 mm above their
                      # tips, and that must pass over the front's top rim as the temple comes home
PRESS = 0.0015        # the pushing face is commanded this far into the temple
LAY_TILT = math.radians(40.0)    # lowered into the case lenses down, far edge 40 degrees low
LID_PUSH_R = 0.050    # the lid is nudged this far up from its hinge
LID_TO = math.radians(80.0)      # the nudge is commanded down to 80 degrees open (the spring flips at 95)


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    L, R = ctx.left, ctx.right
    T = task
    down = np.array([0.0, 0.0, -1.0])
    ez = np.array([0.0, 0.0, 1.0])

    cal = dict(CAL)

    def measure(key, value):
        """DEMO_CALIBRATE=1 only: record a measured offset and use it from here on (so the calibration run moves the
        way the open-loop run will with these numbers in CAL)."""
        if CALIBRATE:
            _MEASURED[key] = [round(float(v), 5) for v in value]
            cal[key] = tuple(_MEASURED[key])
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)

    if CALIBRATE:
        s = ctx.states()["glasses"]
        r = s.rotation()
        measure("settled", (*(np.asarray(s.origin) - np.array(T.GLASSES.pos)), math.degrees(math.atan2(r[1, 0], r[0, 0])),
                            math.degrees(-math.asin(r[2, 0])), math.degrees(math.atan2(r[2, 1], r[2, 2]))))
    dx, dy, dz, yaw, pitch, roll = cal["settled"]
    g0 = np.array(T.GLASSES.pos) + np.array([dx, dy, dz])
    rg = _rot_axis(ez, math.radians(yaw)) @ _rot_axis((0, 1, 0), math.radians(pitch)) @ _rot_axis((1, 0, 0),
                                                                                                math.radians(roll))

    def gw(p):   # a point in the glasses' frame, in the world (as they stand on the table)
        return g0 + rg @ np.asarray(p, float)

    # 1. Left hand: hold the top rim of the left lens, fingers closing across the front, leaning out to the left.
    def hold_pose(side, sign):
        a = rg @ np.array([0.0, -side * math.sin(HOLD_ROLL), -math.cos(HOLD_ROLL)])
        rot = _cols(sign * (rg @ np.array([1.0, 0.0, 0.0])), a)
        return gw((-T.FRONT_T / 2, side * HOLD_Y, T.FRONT_H - HOLD_DEPTH)) - TIPS * a, rot, a

    hold_l, rot_l, a_l = hold_pose(1, SIGN["hold_left"])
    hold_r, rot_r, a_r = hold_pose(-1, SIGN["hold_right"])
    yield from L.move(hold_l - 0.05 * a_l, rot_l, 3.0, grip=0.3, label="L over the left lens")
    yield from L.move(hold_l, rot_l, 1.5, label="L round the rim")
    yield from L.set_grip(0.0, 0.8, label="L holds the front")
    # ... and lifts the glasses clear of the table, so the earpiece tips do not drag on it while the temples turn
    yield from L.move(hold_l + FOLD_LIFT * ez, rot_l, 1.5, label="L lifts the glasses")
    yield from L.move(L.pos, L.rot, 0.5, label="L holds them up")
    if CALIBRATE:
        s = ctx.states()["glasses"]
        r = s.rotation()
        measure("in_air", (*(np.asarray(s.origin) - (g0 + FOLD_LIFT * ez)), math.degrees(math.atan2(r[1, 0], r[0, 0])),
                           math.degrees(-math.asin(r[2, 0])), math.degrees(math.atan2(r[2, 1], r[2, 2]))))
    dx, dy, dz, yaw, pitch, roll = cal["in_air"]
    g0 = g0 + FOLD_LIFT * ez + np.array([dx, dy, dz])
    rg = _rot_axis(ez, math.radians(yaw)) @ _rot_axis((0, 1, 0), math.radians(pitch)) @ _rot_axis((1, 0, 0),
                                                                                                math.radians(roll))
    hold_l = hold_l + FOLD_LIFT * ez        # where the left hand holds them now (as commanded)
    hold_r, rot_r, a_r = hold_pose(-1, SIGN["hold_right"])

    # 2. Right hand: fold the right temple, pushing its outer face 30 mm from the hinge with the closed fingers.
    def push_poses(side, r, end, sign, steps=24):
        hinge = np.array([T.HINGE_X, side * T.HINGE_Y, T.HINGE_Z])
        out = []
        for k in range(steps + 1):
            phi = end * k / steps
            d = rg @ np.array([math.cos(phi), -side * math.sin(phi), 0.0])    # along the temple
            n = rg @ np.array([-math.sin(phi), -side * math.cos(phi), 0.0])   # the way it folds
            # the hand leans out (away from the other hand), turned about the push direction
            t = rg @ np.array([0.0, side, 0.0])
            t = t - (t @ n) * n                          # out, as far as a turn about the push direction allows
            lean = PUSH_LEAN * float(np.linalg.norm(t))
            t = t / max(float(np.linalg.norm(t)), 1e-9)
            a = -(math.cos(lean) * ez + math.sin(lean) * t)
            # and back: its top tipped away from the push, so the fingers' wider upper part (they widen 20 mm above
            # their tips) stays clear of the front's top rim as the temple comes home
            axis = np.cross(ez, n)
            axis /= np.linalg.norm(axis)
            a = _rot_axis(axis, PUSH_BACK) @ a
            if a @ n < 0:
                a = _rot_axis(axis, -2 * PUSH_BACK) @ a
            tip = gw(hinge) + r * d - (T.TEMPLE_T / 2 + FACES[sign] - PRESS) * n
            tip[2] = gw(hinge)[2] - PUSH_LOW
            out.append((tip - TIPS * a, _cols(sign * n, a)))
        return out

    def fold(arm, side, key, name):
        poses = push_poses(side, PUSH_R, PUSH_END[key], SIGN[key])
        sx = SIGN[key] * poses[0][1][:, 0]               # the way the temple folds, at the start
        start = poses[0][0] - 0.006 * sx
        yield from arm.move(start + 0.06 * ez, poses[0][1], 3.0, grip=0.0, label=f"{name} over the temple")
        yield from arm.move(start, poses[0][1], 1.2, label=f"{name} beside it")
        yield from arm.move(poses[0][0], poses[0][1], 0.6, label=f"{name} onto it")
        yield from arm.path(poses, 6.0, label=f"{name} folds it")
        yield from arm.move(arm.pos, arm.rot, 1.0, label=f"{name} folds it")      # the arm lags a pushed path: let it catch up
        yield from arm.move(arm.pos - 0.008 * SIGN[key] * arm.rot[:, 0] + 0.002 * ez, arm.rot, 0.6,
                            label=f"{name} off it")
        yield from arm.move(arm.pos + 0.06 * ez, arm.rot, 1.2, label=f"{name} up")

    yield from fold(R, -1, "push_right", "R")

    # 3. The left hand sets the glasses back down on the table (on the rims and the earpiece tips); the right hand takes
    #    the top rim of the right lens and the left lets go; the right lifts them again.
    yield from L.move(hold_l - FOLD_LIFT * ez, rot_l, 1.5, label="L sets them down")
    yield from R.move(hold_r - FOLD_LIFT * ez - 0.05 * a_r, rot_r, 2.5, grip=0.3, label="R over the right lens")
    yield from R.move(hold_r - FOLD_LIFT * ez, rot_r, 1.5, label="R round the rim")
    yield from R.set_grip(0.0, 0.8, label="R holds the front")
    yield from L.set_grip(0.3, 0.6, label="L lets go")
    yield from L.move(L.pos - 0.05 * a_l, rot_l, 1.2, label="L off the rim")
    yield from R.move(hold_r, rot_r, 1.5, label="R lifts the glasses")
    yield from R.move(R.pos, R.rot, 0.5, label="R holds them up")
    if CALIBRATE:
        s = ctx.states()["glasses"]
        r = s.rotation()
        measure("in_air_r", (*(np.asarray(s.origin) - g0), math.degrees(math.atan2(r[1, 0], r[0, 0])),
                             math.degrees(-math.asin(r[2, 0])), math.degrees(math.atan2(r[2, 1], r[2, 2]))))
    dx, dy, dz, yaw, pitch, roll = cal["in_air_r"]
    g0 = g0 + np.array([dx, dy, dz])
    rg = _rot_axis(ez, math.radians(yaw)) @ _rot_axis((0, 1, 0), math.radians(pitch)) @ _rot_axis((1, 0, 0),
                                                                                                math.radians(roll))

    # 4. The left hand folds the left temple over the right one, and goes back.
    yield from fold(L, 1, "push_left", "L")
    yield from both(L.move(*L.home, 3.0, grip=1.0, label="L back"), R.move(R.pos, R.rot, 0.2, label="R holds"))

    # 5. Right hand: lift the folded glasses, turn them lenses down and far edge low, carry them over the case and
    #    lower them in until the far edge is on the case floor; let go.
    rel_p = R.rot.T @ (g0 - R.pos)         # the glasses' origin in the hand's frame, as planned
    rel_r = R.rot.T @ rg
    lift = R.pos + 0.05 * ez
    yield from R.move(lift, R.rot, 1.5, label="R lifts them")
    if CALIBRATE:
        s = ctx.states()["glasses"]
        measure("in_hand", np.asarray(s.origin) - (lift + R.rot @ rel_p))
    rel_p = rel_p + R.rot.T @ np.array(cal["in_hand"])

    def tcp_for(origin, rot_g):
        r = rot_g @ rel_r.T
        return origin - r @ rel_p, r

    tilt = _rot_axis((0, 1, 0), -(math.pi / 2 - LAY_TILT))     # lenses down, the far (bottom) edge LAY_TILT low
    case = np.array([T.CASE_X, T.CASE_Y, T.TOP + 0.0005])
    floor = case[2] + T.IN_Z[0]
    # where the glasses' origin goes so that their bottom edge is on the floor, 19 mm past the case's middle
    bottom = np.array([-T.FRONT_T, 0.0, 0.0])
    edge = case + np.array([0.019, 0.0, 0.0])
    origin = np.array([edge[0], case[1], floor + 0.001]) - tilt @ bottom
    turned = tcp_for(lift + R.rot @ rel_p + np.array([0.0, 0.0, 0.03]), tilt)
    over = tcp_for(origin + np.array([0.0, 0.0, 0.06]), tilt)
    lay = tcp_for(origin, tilt)
    yield from R.move(turned[0], turned[1], 3.0, label="R turns them lenses down")
    yield from R.move(over[0], over[1], 3.0, label="R carries them over the case")
    yield from R.move(lay[0], lay[1], 2.0, label="R lowers them in")
    yield from R.set_grip(0.3, 0.6, label="R lets go")
    yield from R.move(R.pos - 0.03 * R.rot[:, 2] + 0.02 * ez, R.rot, 1.0, label="R draws back")
    yield from R.move(R.pos + 0.06 * ez, R.rot, 1.0, label="R up")

    # 6. Right hand: nudge the open lid forward past its dead centre; the spring snaps it shut.
    hinge = case + np.array(T.CASE_HINGE)
    lid_pt = np.array([-LID_PUSH_R, 0.0, T.LID_H])   # on its outer face, shut frame

    def lid_pose(theta):
        rr = _rot_axis((0, 1, 0), theta)
        n = rr @ np.array([0.0, 0.0, 1.0])               # the outer face's normal
        p = hinge + rr @ lid_pt + (0.0005 + FACES[SIGN["lid"]]) * n
        p[2] -= TIPS - 0.008                             # pad tips 8 mm below the contact height
        return p

    rot_lid = _cols((-SIGN["lid"], 0.0, 0.0), down)
    lid_path = [(lid_pose(math.radians(a)), rot_lid) for a in np.linspace(math.degrees(T.OPEN), math.degrees(LID_TO), 9)]
    start = lid_path[0][0] + np.array([0.008, 0.0, 0.0])
    yield from R.move(start + 0.05 * ez, rot_lid, 3.5, grip=0.0, label="R behind the lid")
    yield from R.move(start, rot_lid, 1.2, label="R down behind it")
    yield from R.path(lid_path, 2.5, label="R nudges the lid shut")
    yield from R.move(R.pos + 0.08 * ez, rot_lid, 1.2, label="R up")
    yield from R.move(*R.home, 3.0, grip=1.0, label="R back")


def offline_plan(task, solve: bool = False):
    """The demo's moves without the simulator: each arm's list of (pos, rot, label) at the end of every move, and (with
    ``solve``) each step's joint commands."""
    from yam_arena import station

    class _Ctx:
        left = Arm("left", station.BASES["left"], solve=solve)
        right = Arm("right", station.BASES["right"], solve=solve)

    ctx = _Ctx()
    steps = 0
    for _ in script(task, ctx):
        steps += 1
        for a in (ctx.left, ctx.right):
            if len(a.qs) < steps:
                a.stay()
    return {"left": ctx.left.ends, "right": ctx.right.ends, "steps": steps, "ctx": ctx}


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 640,
        cam_height: int = 352, hold_s: float = 6.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction=emb.task.module.INSTRUCTION))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info, first_full = 0, None, False, {}, None
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        if first_full is None and float(info.get("score", 0.0) or 0.0) >= 1.0:
            first_full = steps
        snap(res.observation)
        every = int(os.environ.get("DEMO_TRACE_EVERY", "50"))
        if steps % every == 0:
            extra = ""
            if os.environ.get("DEMO_TRACE"):
                st = ctx.states()
                g, c = st["glasses"], st["glasses_case"]
                extra = (f" fold {[round(v, 1) for v in task.fold_deg(g)]} lid_gap {1000 * task.lid_gap(c):.1f}mm"
                         f" g {np.round(np.asarray(g.origin), 3).tolist()} case {np.round(np.asarray(c.origin), 3).tolist()}"
                         f" L {ctx.left.label!r} R {ctx.right.label!r}")
                base = emb._env.unwrapped
                for arm, a in (("left_robot", ctx.left), ("right_robot", ctx.right)):
                    rob = base.scene[arm]
                    pose = rob.data.body_link_pose_w[0][list(rob.body_names).index("gripper")]
                    pose = getattr(pose, "torch", pose).detach().cpu().numpy()
                    org = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy()
                    from yam_arena.constants import TCP_OFFSET
                    from yam_arena import ik as _ik
                    x, y, z, w = pose[3:7]
                    rm = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                                   [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                                   [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
                    tcp = pose[:3] - org + rm @ np.array(TCP_OFFSET)
                    extra += f" {arm[0].upper()}err {np.round(1000 * (tcp - a.pos), 1).tolist()}"
            print(f"step {steps} score {info.get('score')}{extra}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    # The trial's score is what the task file's score() reported on the last step (task_score gives 1.0 on success,
    # which score() also is then); it is never set to 1.0 here just because the episode ended.
    final = float(info.get("score", 0.0) or 0.0)
    result = {"task_score": final, "success": success, "steps": steps, "first_full_score_step": first_full,
              "script_ended_step": ended,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if CALIBRATE:
        result["CAL"] = _MEASURED
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam", default="640x352", help="camera width x height")
    a = ap.parse_args()
    w, h = (int(v) for v in a.cam.split("x"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=w, cam_height=h)
