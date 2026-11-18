"""Scripted full-score demo for the pipe repair clamp task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The right arm pinches the open clamp by its hinge knuckle, carries it over the pipe, centres it on the red band and
lowers it until its halves straddle the pipe and its bore sits on it. It lets go, opens its fingers wide round the two
halves, lowers them to the halves' grip pads beside the pipe and closes them: the squeeze swings both halves in under
the pipe until their feet meet and the latch snaps. It opens and moves away.

Open loop: every position comes from the task file's fixed layout. Nothing is read from the simulated clamp.

Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
Negative variants (each must score below 1):
    DEMO_VARIANT=offleak    clamps the pipe 100 mm from the leak (scores 0.5)
    DEMO_VARIANT=nosqueeze  sets the clamp on the pipe over the leak and leaves it open (scores 0.25)
DEMO_DEBUG=1 logs the clamp every 20 steps (logging only).
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
TIP = 0.013            # finger tips below the TCP
KNUCKLE_OPEN = 0.40    # about 38 mm open round the 10 mm hinge knuckle
KNUCKLE_DOWN = 0.004   # the finger tips this far below the knuckle's top
PAD_Z = -0.004         # the squeezing finger tips this far below the pipe's axis, on the halves' pads


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.off = np.zeros(3)

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, ctx=None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it). With ``ctx`` the
        arm adds up its own position error (commanded minus measured from its joints) and corrects for it, which takes
        out the sag under a carried part."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy() - self.off, self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            target = p0 + s * (np.asarray(pos, float) - p0)
            if ctx is not None:
                self.off = np.clip(self.off + 0.2 * (target - ctx.actual(self.name)), -0.02, 0.02)
            self._solve(target + self.off, ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos - self.off, self.rot, duration, grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    variant = os.environ.get("DEMO_VARIANT", "")
    top = task.TOP
    R = ctx.right
    rot = np.asarray(grip_rotation(math.pi / 2), float)         # fingers close along x
    knuckle_top = task.HINGE_Z + 0.011
    sx, sy = task.CLAMP_START
    z0 = task.CLAMP.pos[2]
    pick = np.array([sx, sy, z0 + knuckle_top - KNUCKLE_DOWN + TIP])
    yield from R.move(np.array([0.16, -0.22, top + 0.20]), rot, 2.0, grip=KNUCKLE_OPEN)
    yield from R.move(pick + [0, 0, 0.05], rot, 1.5)
    yield from R.move(pick, rot, 1.0)
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)
    held = ctx.actual(R.name) - pick
    held = held if float(np.linalg.norm(held)) < 0.02 else np.zeros(3)
    if os.environ.get("DEMO_DEBUG"):
        print(f"grasp offset mm {np.round(1000 * held, 2)} ik worst {R.worst_mm:.1f}", flush=True)

    y = task.LEAK_Y + (0.100 if variant == "offleak" else 0.0)
    seat = top + task.PIPE_Z + (task.BORE_R - task.PIPE_R)      # the bore's top resting on the pipe

    def clamp_at(zc):
        return np.array([task.PIPE_X, y, zc + knuckle_top - KNUCKLE_DOWN + TIP]) + held

    yield from R.move(pick + [0, 0, 0.08] + held, rot, 1.2, ctx=ctx)
    yield from R.move(clamp_at(seat + 0.06), rot, 2.5, ctx=ctx)
    yield from R.move(clamp_at(seat + 0.02), rot, 1.2, ctx=ctx)
    yield from wait(0.3)
    yield from R.move(clamp_at(seat - 0.001), rot, 1.5, ctx=ctx)             # down over the pipe, halves straddling it
    yield from wait(0.3)
    yield from R.set_grip(KNUCKLE_OPEN, 0.6)
    yield from R.move(R.pos - R.off + [0, 0, 0.05], rot, 1.0)
    R.off = np.zeros(3)
    if variant == "nosqueeze":
        yield from R.move(np.array([0.16, -0.22, top + 0.20]), rot, 2.0)
        yield from wait(1.5)
        return

    # squeeze the halves shut round the pipe: fingers wide open either side, down to the pads, then closed
    sq = np.array([task.PIPE_X, y, top + task.PIPE_Z + PAD_Z + TIP])
    yield from R.move(sq + [0, 0, 0.06], rot, 1.2, grip=1.0)
    yield from R.move(sq, rot, 1.5)
    yield from R.set_grip(0.0, 1.5)
    yield from wait(0.6)
    yield from R.set_grip(1.0, 0.8)
    yield from R.move(sq + [0, 0, 0.07], rot, 1.2)
    yield from R.move(np.array([0.16, -0.22, top + 0.20]), rot, 2.0)
    yield from wait(1.5)


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def actual(self, name: str) -> np.ndarray:
        """The arm's measured TCP position (world), from its own joint positions."""
        from yam_arena import ik

        arm_ids = self.emb._ids[f"{name}_robot"][0]
        q = self.emb._base.scene[f"{name}_robot"].data.joint_pos.torch[0][arm_ids].cpu().numpy().astype(np.float64)
        return ik.tcp_pose(q, self.left.base if name == "left" else self.right.base)[0]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, cam_width: int = 320,
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
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}
    debug = bool(os.environ.get("DEMO_DEBUG"))

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    def trace(step, info):
        from partkit import _part_state

        clamp = _part_state(emb._env, task.CLAMP)
        p = np.asarray(clamp.pos) - [task.PIPE_X, task.LEAK_Y, task.TOP + task.PIPE_Z]
        print(f"step {step} score {info.get('score')} | clamp off mm {np.round(1000 * p, 1)} on pipe {task.on_pipe(clamp)}"
              f" over leak {task.over_leak(clamp)} | halves deg from shut far {math.degrees(task.shut_by(clamp, 1)):.1f}"
              f" near {math.degrees(task.shut_by(clamp, -1)):.1f} latched {task.latched()}"
              f" | clear {task._STATE['clear']}", flush=True)
        rob = emb._base.scene["right_robot"]
        jn, jp = list(rob.joint_names), rob.data.joint_pos.torch[0].cpu().numpy()
        gap = 2 * max(abs(float(jp[jn.index(j)])) for j in ("joint7", "joint8") if j in jn)
        print(f"   right: cmd mm {np.round(1000 * (ctx.right.pos - [0, 0, task.TOP]), 1)} actual mm"
              f" {np.round(1000 * (ctx.actual('right') - [0, 0, task.TOP]), 1)} grip cmd {ctx.right.grip:.2f}"
              f" gap mm {1000 * gap:.1f}", flush=True)

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
        if debug and steps % 20 == 0:
            trace(steps, info)
        elif steps % 100 == 0:
            print(f"step {steps} score {info.get('score')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
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
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
