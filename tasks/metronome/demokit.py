"""demokit: scripted full-score demos for yam_arena task files.

A demo is a generator, ``script(ctx)``, that yields once per control step after setting the arms' commands. Use
``ctx.left`` and ``ctx.right`` (:class:`Arm`) for smooth Cartesian moves solved with yam_arena's IK, and
``ctx.states()`` for the measured scene, to close the loop. :func:`run` plays the script in the yam_arena
embodiment, scores it with the task file, and writes one MP4 per camera, setup.png (the first top-camera frame) and
result.json:

    from demokit import Arm, run, together, wait

    def script(ctx):
        cube = ctx.states()["red_cube"].pos
        yield from ctx.left.move(cube + [0, 0, 0.08], grip_rotation(0.0), 2.0, grip=1.0)
        ...

    run("task.py", "demo_out", script)

Keep this file next to your task file. Run demos in the Isaac Lab-Arena environment (``source <ARENA_DIR>/yam.env``).
"""

from __future__ import annotations

import json
import math
import sys
from collections.abc import Callable
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
PAD_TIP = 0.013   # yam_arena.constants.PAD_TIP_PAST_TCP: fingertips past the tool point


class Arm:
    """One arm commanded in Cartesian space (IK on the URDF), each move starting where the last command left it.
    ``pos``/``rot`` are the tool point (between the finger pads, 13 mm short of their tips) in the world frame."""

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
        """Track a pose this step (for closed-loop moves)."""
        from yam_arena import ik

        q, err, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q_init=self.q, base_pos=self.base, iters=80)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)
        self.worst_mm = max(self.worst_mm, err * 1000)

    def move(self, pos, rot=None, seconds: float = 1.0, grip: float | None = None):
        """Generator: a smooth straight-line move to ``pos``/``rot``; ``grip`` 1 open .. 0 closed (None keeps it)."""
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

    def set_grip(self, grip: float, seconds: float = 0.8):
        yield from self.move(self.pos, self.rot, seconds, grip)

    def go_home(self, seconds: float = 2.5):
        yield from self.move(self.home[0], self.home[1], seconds, grip=1.0)

    def command(self) -> np.ndarray:
        return np.r_[self.q, self.grip]


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def together(*gens):
    """Run generators side by side, one step each per control step (for example both arms at once)."""
    gens = [iter(g) for g in gens]
    while gens:
        for gen in list(gens):
            try:
                next(gen)
            except StopIteration:
                gens.remove(gen)
        if gens:
            yield


def turn(axis, angle: float) -> np.ndarray:
    """Rotation by ``angle`` about the unit vector ``axis``; compose with a grip, e.g. ``turn([0, 0, 1], a) @ rot``."""
    k = np.asarray(axis, float)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


class Ctx:
    """What a script works with: both arms, the task module and the measured scene."""

    def __init__(self, emb):
        self.emb, self.task = emb, emb.task.module
        self.left, self.right = Arm("left"), Arm("right")
        self.flags: dict = {}   # for scripts that run arms in parallel and need to signal each other

    def states(self):
        """Every task object's state after the last control step (``ObjectState``, or scenekit's ``BodyState``)."""
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.command(), self.right.command()))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, script: Callable, max_steps: int = 2400, record: bool = True,
        cam_width: int = 640, cam_height: int = 352, hold_s: float = 4.0,
        log: Callable | None = None, log_every: int = 100) -> dict:
    """Play ``script(ctx)`` in the yam_arena embodiment until the task succeeds, ``max_steps`` pass, or the script
    ends and ``hold_s`` more seconds pass. ``log(ctx, step, info)`` returns a progress line every ``log_every``
    steps. Returns (and writes) the result."""
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    sys.modules.setdefault("task", emb.task.module)
    obs = emb.reset(Scene(id="demo", instruction=getattr(emb.task.module, "INSTRUCTION", "demo")))
    ctx = Ctx(emb)
    gen = script(ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

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
        if steps % log_every == 0:
            line = log(ctx, steps, info) if log else ""
            print(f"step {steps} score {info.get('score')} {line}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    state = getattr(ctx.task, "_STATE", None)
    if isinstance(state, dict) and "why" in state:
        result["violations"] = list(state["why"])
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
