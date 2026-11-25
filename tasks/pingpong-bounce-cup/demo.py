"""Open-loop native-contact bounce demonstration; fixed layout and arm commands only.
DEMO_VARIANT=held, straightdrop, miss provide negative controls.
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


class Frame:
    """A fixed object frame from the task's layout constants: what the first camera frame shows."""

    def __init__(self, xy, z, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        self.pos = np.array([xy[0], xy[1], z], float)
        self.rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])

    def to_world(self, local):
        return self.pos + self.rot @ np.asarray(local, float)


def script(task, ctx):
    """Open loop, using fixed layout and own arm commands only."""
    from yam_arena.demo import grip_rotation
    R = ctx.right
    rot = grip_rotation(0.0)
    p = np.array(task.BALL_START) + [0,0,.001]
    yield from R.move(p+[0,0,.075],rot,2.0,grip=1)
    yield from R.move(p,rot,1.5)
    yield from R.set_grip(.35,1.2)
    yield from wait(.5)
    launch_x = float(os.environ.get('DEMO_SWEEP_START_X', '.12'))
    launch_z = task.BOARD_TOP + float(os.environ.get('DEMO_RELEASE_HEIGHT', '.240'))
    sweep_step = float(os.environ.get('DEMO_SWEEP_STEP', '.025'))
    lean = math.radians(float(os.environ.get('DEMO_LEAN_DEG','15')))
    c,s = math.cos(lean),math.sin(lean)
    launch_rot = np.array([[c,0,-s],[0,1,0],[s,0,c]]) @ rot
    release_step = int(os.environ.get('DEMO_RELEASE_STEP', '5'))
    yield from R.move((launch_x,-.15,launch_z),launch_rot,2.0)
    yield from wait(.5)
    variant = os.environ.get('DEMO_VARIANT','')
    if variant == 'held':
        yield from wait(3)
        return
    y = -.08 if variant == 'miss' else -.15
    if variant == 'straightdrop':
        yield from R.set_grip(1,.25)
    else:
        # Constant-speed release sweep. No ball pose, velocity, or task state is read.
        for k in range(1,release_step+2):
            R.grip = 1.0 if k >= release_step else .35
            R.aim((launch_x+sweep_step*k,y,launch_z),launch_rot)
            yield
    # Immediately sweep sideways out of the flight corridor, avoiding the rebound.
    for k in range(1,9):
        R.grip = 1.0
        R.aim((.17,-.35,task.BOARD_TOP+.190),rot)
        yield
    yield from R.move((.18,-.36,task.BOARD_TOP+.18),rot,.8,grip=1)
    yield from wait(8)


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


def run(task_path: str, out_dir: str, max_steps: int = 1600, record: bool = True, cam_width: int = 320,
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
        if steps % 1 == 0 and 150 <= steps <= 175 or steps % 5 == 0 and steps < 240 or steps % 50 == 0:
            b = ctx.states()['ball']
            print('step', steps, 'score',info.get('score'),'ball',b.pos.tolist(),'vel',b.lin_vel.tolist(), 'spin', b.ang_vel.tolist(), 'state',str(task._STATE),flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        # Keep observing natural settling until success or the full run limit.
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
    ap.add_argument("--max-steps", type=int, default=1600)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
