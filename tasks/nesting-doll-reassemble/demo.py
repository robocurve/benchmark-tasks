"""Scripted full-score demo for nesting-doll-reassemble, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record]
    python demo.py task.py --ik          (no simulator: IK error of every target, step by step)

Every target comes from the task file's fixed layout and the clock, with each arm's own joint readings as feedback
(an integral trim of the tool point on the way down). One step reads a piece's pose: each top is lined up over its
spigot from its measured position before it goes down (it slips and swings 2 to 4 mm in the pinch, and the skirt
clears the spigot by 1.5 mm), and its head is pressed where it now stands.

Right arm, then left arm:
Every piece is round, so every pinch closes along x (toward and away from the arms): pinches closing along y had one
finger arrive well ahead of the other and shove the piece 20 to 30 mm, tipping it (Isaac batches 765bb241 and
401e829d), while the train-track task's x pinches moved its piece under 1 mm. The fingers open 6 mm a side and close
over 2.4 s.
1. Right: the third doll by its belly, over the second doll's bottom half, let go with the fingertips 3 mm above its
   spigot: it drops about 37 mm down the bore onto the floor (fingers cannot follow it into the bore).
2. Right: the second doll's bottom half (the third doll in it) by its belly, over the first doll's bottom half, let go
   the same way, its foot already 20 mm down the bore: it drops about 44 mm onto the first doll's floor.
3. Right: the second doll's top by its skirt band, turned a quarter turn to put its face over the apron, down over the
   second doll's spigot inside the first doll, let go 1 mm above its fit ring's stop (it settles 4 mm proud); then the
   closed fingers press down on its head, aimed 25 mm past home (the arm's push grows with its position error), which
   slides it over the snug fit onto the shoulder.
4. Left: the first doll's top by its skirt band, turned a quarter turn, down over the first doll's spigot, let go
   4 mm proud, and pressed home the same way.

Wrong attempts (DEMO_VARIANT=...):
  skip_inner  step 1 left out: the third doll stays on the table (0)
  unturned    the tops put on without the quarter turn: faces off the aprons (0.25: the third doll is in the second)
  outside     the second doll closed where it stands and left out; the first closed empty (0.5)
  hold        the last top put on but never let go (0)
  rest        both tops put on, turned, and let go, but never pressed home: they stand 4 mm proud (0.25)
Writes one MP4 per camera, setup.png and result.json; DEMO_DEBUG=1 (default) prints a trace every 10 steps.
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
VARIANTS = ("", "skip_inner", "unturned", "outside", "hold", "rest")
KI, CORR_MAX = 0.2, 0.03
XY = np.array([1.0, 1.0, 0.0])
DOWN = np.array([0.0, 0.0, -1.0])
FLIP = np.diag([-1.0, -1.0, 1.0])
SQUEEZE = float(os.environ.get("DEMO_SQUEEZE_MM", "4")) / 1000   # a pinch closes to the piece's width less this, not
                                                                  # fully shut: one finger drives harder than the other,
                                                                  # and a full close shoved the wider halves 35 mm


def cols(closing, approach):
    x, z = np.asarray(closing, float), np.asarray(approach, float)
    return np.column_stack([x, np.cross(z, x), z])


def rot_axis(axis, th):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each from where the last command left it, with an optional
    integral trim on the arm's own joint readings (forward kinematics) so the measured tool point follows the plan."""

    def __init__(self, name: str, base_pos, read=None):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.pos = np.asarray(self.pos, float)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.last_mm = 0.0
        self.read = read
        self.corr = np.zeros(3)

    def _solve(self, pos, rot, servo=None):
        from yam_arena import ik

        if servo is not None and self.read is not None:
            meas = self.read()
            if meas is not None:
                self.corr = np.clip(self.corr + KI * np.asarray(servo) * (self.pos - meas[0]), -CORR_MAX, CORR_MAX)
        q, pe, _ = ik.solve(np.asarray(pos, float) + self.corr, ik.matrix_to_quat_xyzw(rot), self.q,
                            base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.last_mm = pe * 1000
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, servo=None):
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s), servo)
            yield

    def path(self, fn, duration: float, servo=None):
        """Follow ``fn(s) -> (pos, rot)`` for s from 0 to 1 (smoothstep timing)."""
        from yam_arena import ik

        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            pos, rot = fn(ik.smoothstep(k / n))
            self._solve(pos, rot, servo)
            yield

    def hold(self, duration: float, grip: float | None = None, servo=None):
        yield from self.move(self.pos, self.rot, duration, grip, servo)

    def to_joints(self, q_target, duration: float, grip: float | None = None):
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        self.corr = np.zeros(3)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (np.asarray(q_target, float) - q0)
            self.grip = g0 + s * (g1 - g0)
            pos, rot = ik.tcp_pose(self.q, self.base)
            self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


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


def chain(*gens):
    for g in gens:
        yield from g




def grip_for(yaw: float) -> np.ndarray:
    """Pointing straight down, fingers closing along the horizontal direction ``yaw``."""
    return cols(np.array([math.cos(yaw), math.sin(yaw), 0.0]), DOWN)


def opening(width: float) -> float:
    """Grip command for a finger gap ``width`` + 12 mm (1.0 = 94 mm): 6 mm a side."""
    return min(1.0, (width + 0.012) / 0.094)


def moves(T, name: str, grip_z: float, width: float, dest_xy, dest_base_z: float, turn: float, clear_z: float):
    """Targets for carrying one piece: pinched with the TCP ``grip_z`` above its own origin, set down with its origin
    at ``dest_base_z`` over ``dest_xy``, turned ``turn`` about the vertical on the way; ``clear_z`` is the height its
    origin is carried at."""
    x, y, yaw = T.LAYOUT[name]
    top = T.TOP + 0.002
    pick = np.array([x, y, top + grip_z])
    dest = np.array([dest_xy[0], dest_xy[1], T.TOP + dest_base_z + grip_z])
    return {"hi": pick + [0, 0, 0.06], "pick": pick, "lift": np.array([x, y, T.TOP + clear_z + grip_z]),
            "over": np.array([dest[0], dest[1], T.TOP + clear_z + grip_z]), "set": dest,
            "out": dest + [0, 0, 0.04], "shut": max(0.0, (width - SQUEEZE) / 0.094), "yaw0": float(os.environ.get("DEMO_GRIP_YAW_DEG", "0")) * math.pi / 180, "turn": turn, "open": opening(width)}


def best_flip(arm, M) -> np.ndarray:
    from yam_arena import ik

    out = []
    for F in (np.eye(3), FLIP):
        q, worst = arm.q.copy(), 0.0
        Rp, Rq = grip_for(M["yaw0"]) @ F, grip_for(M["yaw0"] + M["turn"]) @ F
        poses = [(M["hi"], Rp), (M["pick"], Rp), (M["lift"], Rp)]
        poses += [(M["lift"] + s * (M["over"] - M["lift"]), ik.slerp_matrix(Rp, Rq, s)) for s in np.linspace(0, 1, 5)]
        poses += [(M["set"], Rq), (M["out"], Rq)]
        for pos, R in poses:
            q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(R), q, base_pos=arm.base, iters=200)
            worst = max(worst, pe + 0.05 * re + 0.01 * max(0.0, abs(float(q[5])) - 2.6))
        out.append((worst, F))
    return min(out, key=lambda t: t[0])[1]


def solve_full(arm, pos, R):
    """Joint angles for a pose, solved from the arm's command and from several seeds; the closest good one."""
    from yam_arena import ik
    from yam_arena.constants import READY

    rng = np.random.default_rng(3)
    seeds = [arm.q.copy(), np.array(READY, float)] + [np.array(READY, float) + rng.normal(0, 0.6, 6) for _ in range(8)]
    best = None
    for sd in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(R), sd, base_pos=arm.base, iters=300)
        key = (pe * 1000 + math.degrees(re) > 1.0, float(np.max(np.abs(q - arm.q))))
        if best is None or key < best[0]:
            best = (key, q)
    return best[1]


PRESS_OVER = float(os.environ.get("DEMO_PRESS_MM", "25")) / 1000   # the press is aimed this far past the head's seated
                     # height: the arm's push grows with its position error, and 7 mm of error (first try) pushed under
                     # the fit's 3 N
PRESS_UP = 0.030     # the closed fingers come down onto the head from this far above it


def press(arm, xy, head_z: float):
    """Closed fingers straight down onto a top's head at ``xy`` (its seated height ``head_z``), aimed PRESS_OVER past
    home, held, and back up: the axial press that slides the top over the snug fit."""
    R = arm.rot.copy()
    proud = head_z + FIT_PROUD
    above = np.array([xy[0], xy[1], proud + PRESS_UP + PAD_TIP])
    yield from arm.move(above, R, 1.2, grip=0.0)
    yield from arm.move(np.array([xy[0], xy[1], proud + 0.003 + PAD_TIP]), R, 0.8, servo=XY)
    yield from arm.move(np.array([xy[0], xy[1], head_z - PRESS_OVER + PAD_TIP]), R, 1.5, servo=XY)
    yield from arm.hold(1.0, servo=XY)
    yield from arm.move(above, R, 1.0)


LOCATE = {"fn": None}   # set by script(): name -> the piece's state (the tops are lined up over their spigot from it)
ALIGN_MM = 0.4


def align(arm, M, R):
    """Hovering with the top's rim 10 mm over the spigot, shift the hand until the top's rim centre is over its bottom
    half's axis as it now stands (within 0.4 mm): the top slips and swings in the pinch by 2 to 4 mm, the bottom halves
    shift by up to 6 mm when the inner dolls drop in, and the skirt clears the spigot by only 1.5 mm. Returns the total
    shift, applied to the rest of the carry."""
    shift = np.zeros(3)
    for _ in range(6):
        st, base = LOCATE["fn"](M["piece"]), LOCATE["fn"](M["base"])
        err = np.array([base.pos[0] - st.pos[0], base.pos[1] - st.pos[1], 0.0])
        if np.linalg.norm(err) * 1000 <= ALIGN_MM:
            break
        shift += err
        yield from arm.move(M["hover"] + shift, R, 0.6)
        yield from arm.hold(0.4)
    return shift


def carry(arm, M, hold_end: bool = False):
    F = best_flip(arm, M)
    Rp, Rq = grip_for(M["yaw0"]) @ F, grip_for(M["yaw0"] + M["turn"]) @ F
    # the free move to above the piece in joint space: a Cartesian move there can turn the wrist half a turn, which the
    # step-by-step IK does not follow (in batch 86fb2fe3 it sent the arm 26 cm off)
    yield from arm.to_joints(solve_full(arm, M["hi"], Rp), 2.5, M["open"])
    yield from arm.move(M["pick"], Rp, 1.5, servo=XY)
    yield from arm.hold(0.8, servo=XY)
    yield from arm.hold(2.4, M["shut"], servo=XY)
    yield from arm.move(M["lift"], Rp, 1.5)
    yield from arm.move(M["over"], Rq, 3.0, servo=XY)
    yield from arm.hold(0.5, servo=XY)
    shift = np.zeros(3)
    if M.get("piece") and LOCATE["fn"] is not None:
        base = LOCATE["fn"](M["base"]).pos                  # over the bottom half where it now stands
        M = dict(M, hover=np.array([base[0], base[1], M["hover"][2]]))
        yield from arm.move(M["hover"], Rq, 1.5, servo=XY)
        yield from arm.hold(0.4, servo=XY)
        shift = yield from align(arm, M, Rq)
        # set down so the top's rim, as it now hangs in the pinch, stops 1 mm over the fit ring's stop
        drop = float(LOCATE["fn"](M["piece"]).pos[2]) - (M["rim_stop_z"] + 0.001)
        put = arm.pos - np.array([0.0, 0.0, drop])
        M = dict(M, set=put, out=put + [0.0, 0.0, 0.04])
        yield from arm.move(M["set"], Rq, 2.0)
        yield from arm.hold(0.6)
    else:
        yield from arm.move(M["set"], Rq, 2.0, servo=XY)
        yield from arm.hold(0.6, servo=XY)
    if hold_end:
        yield from arm.hold(8.0, servo=XY)
        return
    yield from arm.hold(0.6, M["open"])
    yield from arm.move(M["out"], Rq, 1.2)
    if M.get("press") is not None:
        xy, head_z = M["press"]
        if M.get("piece") and LOCATE["fn"] is not None:     # the head of the top as it now stands
            st = LOCATE["fn"](M["piece"])
            xy = tuple(st.to_world(np.array([0.0, 0.0, M["head_up"]]))[:2])
        yield from press(arm, xy, head_z)


SKIRT_GRIP = 0.016   # a top is pinched by its skirt band, the TCP this far above its rim: the pads' tips stop 3 mm
                     # above the rim, clear of the bottom half's shoulder when the top is seated
PAD_TIP = 0.013      # the pads' tips are this far below the TCP
FIT_PROUD = 0.004    # a top let go on its spigot stands this far proud: its fit ring rests on the spigot's rim
LET_GO = float(os.environ.get("DEMO_LET_GO_MM", "-4")) / 1000   # the planned hover and set-down heights: the set-down is
                                     # then taken from the top's measured height (a first run let go 8 mm up and the top
                                     # landed on the ring cocked 13 degrees)


def plan(T, variant: str) -> tuple[list, list]:
    """(right arm's carries, left arm's carries)."""
    d1, d2 = T.dims(1), T.dims(2)
    b1 = T.LAYOUT["doll1_bottom"][:2]
    b2 = T.LAYOUT["doll2_bottom"][:2]
    turn = lambda top, bottom: 0.0 if variant == "unturned" else (    # noqa: E731
        (T.LAYOUT[bottom][2] - T.LAYOUT[top][2] + math.pi) % (2 * math.pi) - math.pi)

    def top_move(n, dest_xy, seat_z, clear_z):
        """Set on its spigot and let go 1 mm above the fit ring's stop, then pressed home unless ``rest``."""
        d = T.dims(n)
        M = moves(T, f"doll{n}_top", SKIRT_GRIP, 2 * d["rs"], dest_xy, seat_z + FIT_PROUD + LET_GO,
                  turn(f"doll{n}_top", f"doll{n}_bottom"), clear_z)
        foot = seat_z - d["zs"]                       # the bottom half's foot, above the table top
        M["press"] = None if variant == "rest" else (dest_xy, T.TOP + 0.002 + foot + d["H"])
        M["piece"], M["base"], M["head_up"] = f"doll{n}_top", f"doll{n}_bottom", d["H"] - d["zs"]
        M["rim_stop_z"] = T.TOP + 0.002 + seat_z + FIT_PROUD           # the top's origin where the fit ring stops it
        M["hover"] = M["set"] + [0.0, 0.0, T.SPIGOT_H + 0.010 - FIT_PROUD - LET_GO]   # the rim 10 mm over the spigot
        return M

    def drop_into(name, z_belly, width, dest_xy, rim_z):
        """Pinched with the TCP 9 mm above ``z_belly``; let go with the pads' tips 3 mm above ``rim_z``."""
        grip_z = z_belly + PAD_TIP - 0.004
        base = rim_z + 0.003 - (grip_z - PAD_TIP)
        return moves(T, name, grip_z, width, dest_xy, base, 0.0, rim_z + 0.006)   # carried with its foot over the rim

    right, left = [], []
    rim2 = d2["zs"] + T.SPIGOT_H                      # the second doll's spigot top, 52 mm
    rim1 = d1["zs"] + T.SPIGOT_H                      # the first's, 68 mm
    if variant != "skip_inner":
        # 1. the third doll by its belly, let go over the second doll's bore: it drops onto the floor
        z3 = 0.25 * T.SIZES[3][0]
        right.append(drop_into("doll3", z3, 2 * T.outer(3, z3), b2, rim2))
    if variant == "outside":
        right.append(top_move(2, b2, d2["zs"], T.FLOOR + T.SIZES[3][0] + 0.020))   # over the third doll's head
        left.append(top_move(1, b1, d1["zs"], rim1 + 0.020))
        return right, left
    # 2. the second doll's bottom (the third in it) by its upper belly, let go over the first doll's bore
    z2 = 0.25 * T.SIZES[2][0]      # its widest: the fingertips 4 mm below it, so the squeeze cannot wedge the hand up
    right.append(drop_into("doll2_bottom", z2, 2 * T.outer(2, z2), b1, rim1))
    # 3. the second doll's top onto its bottom, now on the first doll's floor; carried with its rim planned 20 mm above
    #    the third doll's head (80 mm), as it slips about 8 mm in the pinch
    right.append(top_move(2, b1, T.FLOOR + d2["zs"], 2 * T.FLOOR + T.SIZES[3][0] + 0.020))
    # 4. the first doll's top onto its bottom, carried with its rim planned 20 mm above the second doll's head (114 mm):
    #    a top slips about 8 mm down in the pinch on its tapered skirt (Isaac batch 0bcf2811), so it clears by 12 mm
    left.append(top_move(1, b1, d1["zs"], T.FLOOR + T.SIZES[2][0] + 0.020))
    return right, left


def script(task, ctx):
    from yam_arena.constants import READY as READY_Q

    variant = os.environ.get("DEMO_VARIANT", "")
    if variant not in VARIANTS:
        raise ValueError(f"unknown DEMO_VARIANT {variant!r}")
    right, left = plan(task, variant)
    if hasattr(ctx, "emb"):
        LOCATE["fn"] = lambda name: task._state(ctx.emb._env, name)
    for M in right:
        yield from carry(ctx.right, M)
    yield from ctx.right.to_joints(READY_Q, 2.0)
    for i, M in enumerate(left):
        yield from carry(ctx.left, M, hold_end=(variant == "hold" and i == len(left) - 1))
    if variant != "hold":
        yield from ctx.left.to_joints(READY_Q, 2.0)
    yield from wait(1.0)


def bases(task):
    from yam_arena import station

    up = np.array([0.0, 0.0, getattr(task, "PEDESTAL", 0.0)])
    return {k: np.asarray(v, float) + up for k, v in station.BASES.items()}


class Ctx:
    def __init__(self, emb, task):
        self.emb = emb
        self.bases = bases(task)
        self.left = Arm("left", self.bases["left"], read=lambda: self.joint_tcp("left"))
        self.right = Arm("right", self.bases["right"], read=lambda: self.joint_tcp("right"))

    def joint_tcp(self, name: str):
        """The arm's tool point and gripper rotation from its own joint positions (forward kinematics)."""
        from partkit import _np
        from yam_arena import ik
        from yam_arena.constants import ARM_JOINTS

        art = self.emb._env.unwrapped.scene[f"{name}_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        pos, rot = ik.tcp_pose(_np(art.data.joint_pos)[0][ids], self.bases[name])
        return np.asarray(pos, float), np.asarray(rot, float)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def summary(task, st) -> dict:
    """Logging only: every piece's place and what the scorer makes of the set."""
    out = {}
    for n in task.SPECS:
        p = st[n]
        out[n] = {"pos": np.round(np.asarray(p.pos) - [0, 0, task.TOP], 4).tolist(),
                  "yaw_deg": round(math.degrees(task.yaw_of(p)), 1), "tilt_deg": round(task.tilt_deg(p), 1)}
    out["levels"] = task.levels(st)
    return out


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 4.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb, task)
    gen = script(task, ctx)
    frames: dict[str, list] = {}
    debug = os.environ.get("DEMO_DEBUG", "1") != "0"
    min_gap = float("inf")
    import clearance

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    last_gap = {"mm": None}

    def row(steps, info):
        from yam_arena.scene import object_states  # noqa: F401

        st = object_states(emb._env, emb.task.names)
        return {"step": steps, "score": info.get("score"), "dolls": summary(task, st), "arm_gap_mm": last_gap["mm"],
                "violation": task._STATE["violation"], "fit_mm": task._STATE.get("fit_mm"),
                "tcp": {"L_cmd": np.round(ctx.left.pos - [0, 0, task.TOP], 3).tolist(),
                        "L_meas": np.round(ctx.joint_tcp("left")[0] - [0, 0, task.TOP], 3).tolist(),
                        "R_cmd": np.round(ctx.right.pos - [0, 0, task.TOP], 3).tolist(),
                        "R_meas": np.round(ctx.joint_tcp("right")[0] - [0, 0, task.TOP], 3).tolist(),
                        "grip": [round(float(ctx.left.grip), 2), round(float(ctx.right.grip), 2)]},
                "ik_mm": [round(ctx.left.last_mm, 1), round(ctx.right.last_mm, 1)]}

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % 5 == 0:
            try:
                gap, _ = clearance.min_clearance(ctx.left.q, ctx.left.grip, ctx.left.base, ctx.right.q,
                                                 ctx.right.grip, ctx.right.base)
                min_gap = min(min_gap, gap)
                last_gap["mm"] = round(1000 * gap, 1)
            except Exception as exc:  # noqa: BLE001 - logging only
                if steps == 5:
                    print(f"clearance unavailable: {exc!r}", flush=True)
        if debug and steps % 10 == 0:
            try:
                print("TRACE " + json.dumps(row(steps, info), default=str), flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"trace unavailable: {exc!r}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "variant": os.environ.get("DEMO_VARIANT", ""),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)},
              "min_arm_link_gap_mm": round(1000 * min_gap, 1) if min_gap < float("inf") else None,
              "violation": task._STATE["violation"], "fit_mm": task._STATE.get("fit_mm")}
    try:
        from yam_arena.scene import object_states  # noqa: F401

        result["dolls"] = summary(task, object_states(emb._env, emb.task.names))
    except Exception as exc:  # noqa: BLE001 - logging only
        result["dolls"] = repr(exc)
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, default=str), flush=True)
    return result


def ik_check(task_path: str) -> None:
    """No simulator: yam_arena IK error along each arm's whole script (both arms, default variant)."""
    import importlib

    sys.path.insert(0, str(Path(task_path).resolve().parent))
    task = importlib.import_module(Path(task_path).stem)
    B = bases(task)

    class C:
        pass

    ctx = C()
    ctx.left, ctx.right = Arm("left", B["left"]), Arm("right", B["right"])
    n = 0
    for _ in script(task, ctx):
        n += 1
    print(f"IK steps {n}: worst left {ctx.left.worst_mm:.1f} mm, right {ctx.right.worst_mm:.1f} mm", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out", nargs="?", default="demo_out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--ik", action="store_true", help="only print the IK error along the script")
    a = ap.parse_args()
    if a.ik:
        ik_check(a.task)
    else:
        run(a.task, a.out, a.max_steps, record=not a.no_record)
