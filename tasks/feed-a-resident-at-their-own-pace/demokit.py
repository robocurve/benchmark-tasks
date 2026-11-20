"""demokit: a closed-loop scripted demo for a yam_arena task file, played in the yam_arena embodiment.

A task's demo.py defines ``script(ctx)``, a generator that moves ``ctx.left`` and ``ctx.right`` (Cartesian moves with
IK on the URDF) and yields once per control step; it may read the scene through ``ctx.states()``, the episode time
``ctx.now()`` and the task module's own state. ``run()`` plays it and writes one MP4 per camera, ``setup.png`` (the
first top-camera frame) and ``result.json``.

    python demo.py task.py demo_out [--variant NAME] [--max-steps N] [--no-record]
"""

from __future__ import annotations

import os

import argparse
import json
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ


class Arm:
    """One arm commanded in Cartesian space (IK on the URDF), each move starting where the last command left it."""

    def __init__(self, side: str):
        from yam_arena import ik, station
        from yam_arena.constants import READY

        self.side, self.base = side, np.asarray(station.BASES[side], float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.worst_mm = 0.0

    def aim(self, pos, rot) -> None:
        from yam_arena import ik

        q, err, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q_init=self.q, base_pos=self.base, iters=80)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)
        self.last_mm = err * 1000
        self.worst_mm = max(self.worst_mm, err * 1000)

    def move(self, pos, rot=None, seconds: float = 1.0, grip: float | None = None):
        """Generator: a smooth straight-line move to ``pos``/``rot``; ``grip`` 1 open .. 0 closed."""
        from yam_arena import ik

        rot = self.rot if rot is None else rot
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(seconds * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.aim(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def move_speed(self, pos, rot=None, speed: float = 0.15, grip: float | None = None, min_s: float = 0.4):
        """A move that takes as long as ``speed`` (m/s, peak about 1.5x that) needs for the distance."""
        d = float(np.linalg.norm(np.asarray(pos, float) - self.pos))
        yield from self.move(pos, rot, max(min_s, d / speed), grip)

    def set_grip(self, grip: float, seconds: float = 0.8):
        yield from self.move(self.pos, self.rot, seconds, grip)

    def go_home(self, seconds: float = 2.0, lift: float = 0.0):
        if lift:
            yield from self.move(self.pos + [0, 0, lift], self.rot, 0.8)
        yield from self.move(self.home[0], self.home[1], seconds, grip=1.0)

    def command(self) -> np.ndarray:
        return np.r_[self.q, self.grip]


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def wait_until(cond, timeout: float = 120.0):
    """Yield until ``cond()`` is true (or ``timeout`` s)."""
    for _ in range(int(timeout * CONTROL_HZ)):
        if cond():
            return
        yield


def together(*gens):
    """Run generators side by side, one step each per control step."""
    gens = [iter(g) for g in gens]
    while gens:
        for gen in list(gens):
            try:
                next(gen)
            except StopIteration:
                gens.remove(gen)
        if gens:
            yield


def down(yaw: float = 0.0) -> np.ndarray:
    """Tool frame pointing straight down, fingers closing along the world y axis turned by ``yaw``."""
    from yam_arena.demo import grip_rotation

    return grip_rotation(yaw)


class Ctx:
    def __init__(self, emb, task):
        self.emb, self.task = emb, task
        self.left, self.right = Arm("left"), Arm("right")
        self.flags: dict = {}

    def arm(self, side: str) -> Arm:
        return self.left if side == "left" else self.right

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def pos(self, name: str) -> np.ndarray:
        return np.asarray(self.states()[name].pos, float).copy()

    def now(self) -> float:
        import dynkit

        return dynkit.now()

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.command(), self.right.command()))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264", quality=8,
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, script, max_steps: int = 2400, record: bool = True, variant: str = "",
        cam_width: int = 640, cam_height: int = 352, hold_s: float = 3.0, report=None) -> dict:
    """Play ``script(ctx, variant)`` until it ends (then ``hold_s`` more seconds), the episode succeeds, or
    ``max_steps``. ``report(task)`` returns extra fields for result.json."""
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    task = emb.task.module
    sys.modules.setdefault("task", task)
    obs = emb.reset(Scene(id="demo", instruction=task.INSTRUCTION))
    ctx = Ctx(emb, task)
    ctx.obs = obs
    gen = script(ctx, variant)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        if ended is None:
            try:
                next(gen)
            except StopIteration:
                ended = steps
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        ctx.obs = res.observation
        snap(res.observation)
        if steps % int(os.environ.get("DEMO_PRINT_EVERY", "100")) == 0:
            print(f"step {steps} t {ctx.now():.1f} score {info.get('score')} {getattr(task, 'status', lambda: '')()}",
                  flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"variant": variant or "demo", "task_score": score, "success": success, "steps": steps,
              "seconds": round(steps * DT, 2),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if report is not None:
        result.update(report(task))
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"{variant or 'demo'}_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            # the first frames are drawn before the moving bodies (the person) are first posed
            imageio.imwrite(out / "setup.png", frames[top][min(2, len(frames[top]) - 1)])
    (out / f"result_{variant or 'demo'}.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


def main(script, variants=("",), report=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--variant", default="", choices=list(variants))
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=352)
    a = ap.parse_args()
    run(a.task, a.out, script, a.max_steps, record=not a.no_record, variant=a.variant, cam_width=a.cam_width,
        cam_height=a.cam_height, report=report)
