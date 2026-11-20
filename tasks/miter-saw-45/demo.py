"""Scripted full-score demo for the mitre-saw task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The right arm closes on the front handle from above, its fingers across the handle so the near finger squeezes the
release paddle against it (the plunger lifts). It swings the turntable through 45 degrees along the handle's arc,
turning the gripper with it, and opens: the paddle springs back and the plunger drops into the 45 degree detent. Then
the arm lifts away.

Open loop: every position comes from the task file's fixed layout and the planned angle; nothing is read from the
simulated saw.

Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
Negative variants (each must score below 1):
    DEMO_VARIANT=short   lets go at 25 degrees, short of the 45 degree detent (scores 0.25)
    DEMO_VARIANT=held    swings to 45 degrees but never lets go: the paddle stays squeezed, unlocked (scores 0.25)
DEMO_DEBUG=1 logs the table and paddle angles every 20 steps (logging only).
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
GRIP_R = 0.218         # TCP distance from the turntable's centre when the handle and squeezed paddle are held
SWING_S = 5.0          # s for the 45 degree swing
STEPS = 30             # waypoints along the arc


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


def _rz(angle: float) -> np.ndarray:
    c, s_ = math.cos(angle), math.sin(angle)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    variant = os.environ.get("DEMO_VARIANT", "")
    top = task.TOP
    cx, cy = task.SAW_XY
    z = top + (task.HANDLE_Z[0] + task.HANDLE_Z[1]) / 2
    rot0 = np.asarray(grip_rotation(math.pi / 2), float)       # closing axis along x: across the handle and paddle
    R = ctx.right

    def pose(theta):
        p = np.array([cx, cy, z]) + _rz(theta) @ np.array([-GRIP_R, 0.0, 0.0])
        return p, _rz(theta) @ rot0

    p0, r0 = pose(0.0)
    yield from R.move(p0 + [0, 0, 0.07], r0, 2.5, grip=1.0)
    yield from R.move(p0, r0, 1.5)
    yield from R.set_grip(0.0, 1.0)
    yield from wait(0.4)
    # stop a degree short of 45 and let go: the released plunger catches the detent as the table settles into it
    end = math.radians(25.0 if variant == "short" else 44.0)
    for k in range(1, STEPS + 1):
        p, r = pose(end * k / STEPS)
        yield from R.move(p, r, SWING_S / STEPS)
    yield from wait(0.5)
    # trim: the handle's angle from the arm's own measured tool point, swung on or back until it reads the target
    cmd = end
    for _ in range(12):
        a = ctx.actual(R.name)
        got = math.atan2(cy - a[1], cx - a[0])
        if os.environ.get("DEMO_DEBUG"):
            print(f"handle at {math.degrees(got):.2f} deg (commanded {math.degrees(cmd):.2f})", flush=True)
        if abs(got - end) <= math.radians(0.3):
            break
        cmd += max(-math.radians(3.0), min(end - got, math.radians(3.0)))
        p, r = pose(cmd)
        yield from R.move(p, r, 0.4)
        yield from wait(0.2)
    if variant != "held":
        yield from R.set_grip(1.0, 0.8)
        yield from R.move(R.pos - R.off + [0, 0, 0.08], R.rot, 1.5)   # straight up from where it was commanded
        yield from R.move(np.array([0.15, -0.25, top + 0.20]), rot0, 2.0)
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

        saw = _part_state(emb._env, task.SAW)
        print(f"step {step} score {info.get('score')} | table deg {math.degrees(task.table_angle(saw)):.2f}"
              f" paddle deg {math.degrees(saw.hinges.get('paddle', 0.0)):.2f} pressed {task.paddle_pressed(saw)}"
              f" locked_at {task._STATE['locked_at']} | saw moved mm"
              f" {1000 * np.linalg.norm(np.asarray(saw.pos) - task._STATE['saw0']):.1f}"
              f" | failed {task._STATE['failed']} {task._STATE['why']} clear {task._STATE['clear']}", flush=True)

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
