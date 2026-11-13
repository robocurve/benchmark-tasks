"""Scripted demo runner for the roll-race task (adapted from the DMX runner, itself from an accepted task demo).

    python demo.py task.py out_dir [--script script.txt] [--seeds 0 1 2] [--max-steps 4800] [--no-record]

The policy is a generator ``script(task, ctx)`` loaded from the script file. Each seed resets the scene with that
seed (which draws the hidden cylinder kinds) and runs the script once. Writes per-seed MP4s per camera, setup.png and
result.json into out_dir/seed_<n>/ and a summary.json in out_dir.
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

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)

    def aim(self, pos, rot):
        """Track a pose this step (for closed-loop moves)."""
        self._solve(pos, rot)


def reachable_yaw(arm, pos, yaw):
    """The grip yaw, or the same parallel grip turned 180 degrees, whichever the arm reaches more exactly. A wrist at
    its joint limit otherwise leaves the fingers several degrees off and they twist light parts as they close."""
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    best = None
    for y in (yaw, yaw + math.pi, yaw - math.pi):
        _, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(grip_rotation(y)), arm.q, base_pos=arm.base, iters=200)
        if best is None or pe + 0.05 * re < best[0]:
            best = (pe + 0.05 * re, y)
    return best[1]


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


def _actual_tcp(ctx, arm):
    from yam_arena import ik

    rob = ctx.emb._env.unwrapped.scene[f"{arm.name}_robot"]
    qa = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
    return ik.tcp_pose(qa, arm.base)[0]


def goto(ctx, arm, goal, rot, duration, rounds=4):
    """Move to ``goal`` (world TCP) in free air, then re-aim from the measured fingertip: the joints sag under
    gravity, so the command is shifted by the measured error until the fingertip is within 1.5 mm. The final shift
    is kept in ``arm.offset`` and applied to the short moves that follow (descend, push), which touch the levers."""
    goal = np.asarray(goal, float)
    yield from arm.move(goal + arm.offset, rot, duration)
    cmd = goal + arm.offset
    for _ in range(rounds):
        yield from wait(0.35)
        err = _actual_tcp(ctx, arm) - goal
        if np.linalg.norm(err) < 0.0015:
            break
        cmd = cmd - err
        yield from arm.move(cmd, rot, 0.4)
    arm.offset = cmd - goal


def _yaw_of(p):
    r = p.rotation()
    return math.atan2(r[1, 0], r[0, 0])


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


def run(task_path: str, out_dir: str, script_path: str, seeds, base_path: str | None = None, max_steps: int = 4800, record: bool = True,
        cam_width: int = 320, cam_height: int = 176, hold_s: float = 3.0) -> list[dict]:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    g = dict(globals())
    if base_path:   # helpers shared with the reference demo (pick, place, press, ...)
        exec(compile(Path(base_path).read_text(), base_path, "exec"), g)
    exec(compile(Path(script_path).read_text(), script_path, "exec"), g)
    script = g["script"]
    root = Path(out_dir)
    root.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, device=os.environ.get("ISAAC_DEVICE", "cuda:0"), object_jitter=0,
                               control_hz=CONTROL_HZ, cam_width=cam_width, cam_height=cam_height)
    results = []
    for seed in seeds:
        out = root / f"seed_{seed}"
        out.mkdir(parents=True, exist_ok=True)
        obs = emb.reset(Scene(id="demo", instruction="demo"), seed=int(seed))
        task = emb.task.module
        ctx = Ctx(emb)
        ctx.seed = int(seed)
        gen = script(task, ctx)
        frames: dict[str, list] = {}

        def snap(o):
            if record:
                for cam, img in o.images.items():
                    frames.setdefault(cam, []).append(np.asarray(img))

        snap(obs)
        steps, ended, success, info, error = 0, None, False, {}, None
        while steps < max_steps:
            try:
                next(gen)
            except StopIteration:
                ended = steps if ended is None else ended
            except Exception as exc:   # keep the video and the scorer record of a failed demo
                import traceback

                error = traceback.format_exc()
                print(error, flush=True)
                gen = iter(())
                ended = steps if ended is None else ended
            res = emb.step(ctx.action())
            steps += 1
            info = res.info
            snap(res.observation)
            if steps % 200 == 0:
                print(f"seed {seed} step {steps} score {info.get('score')}", flush=True)
            if os.environ.get("DEMO_DEBUG") and steps % 10 == 0 and hasattr(task, "debug_line"):
                print(f"[dbg] step {steps} " + task.debug_line(ctx.states()), flush=True)
            if res.terminated:
                success = res.termination_reason == "success"
                break
            if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
                break
        score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
        result = {"seed": int(seed), "task_score": score, "success": success, "steps": steps, "error": error,
                  "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)},
                  "script_notes": getattr(ctx, "notes", []), "scorer": task.report(ctx.states())}
        if record:
            import imageio

            for cam, fs in frames.items():
                _write_mp4(fs, out / f"demo_{cam}.mp4")
            top = next((c for c in frames if "top" in c), next(iter(frames), None))
            if top:
                imageio.imwrite(out / "setup.png", frames[top][0])
        (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
        print("RESULT", json.dumps(result, default=str), flush=True)
        results.append(result)
    (root / "summary.json").write_text(json.dumps(results, indent=2, default=str))
    return results


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--script", default="script.txt")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0])
    ap.add_argument("--max-steps", type=int, default=4800)
    ap.add_argument("--base", default=None)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.script, a.seeds, a.base, a.max_steps, record=not a.no_record)
