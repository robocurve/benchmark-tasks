"""Scripted full-score demo for the saw depth-of-cut task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The right arm, fingers closed, pushes the depth lever's thumb tab out round its pivot to free the foot plate. It opens
its fingers a little, pinches the plate's top edge near the rear and swings the plate out on its front pivot by the
angle that leaves 25 mm of blade past it, lets go, and pushes the lever back home to clamp it, then moves away.

Open loop: every position comes from the task file's fixed layout (the saw's geometry gives the swing for 25 mm).
Nothing is read from the simulated saw.

Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
Negative variants (each must score below 1):
    DEMO_VARIANT=nolock  sets 25 mm and leaves the lever out (scores 0.5)
    DEMO_VARIANT=deep    sets 35 mm and locks it (scores 0.25)
DEMO_DEBUG=1 logs the saw every 20 steps (logging only).
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
TIP_HALF = 0.008       # half the closed finger tips' length (along x of the gripper)
PUSH_IN = 0.005        # the finger tips this far into the face they push
PINCH_DOWN = 0.010     # the open finger tips this far down the foot plate's top edge
PINCH_Y = -0.112       # ... this far behind the arbor (saw frame)
OPEN_GRIP = 0.30


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


def _rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    variant = os.environ.get("DEMO_VARIANT", "")
    top = task.TOP
    R = ctx.right
    O = np.array(task.SAW_ORIGIN, float)                        # the arbor (world); the saw's axes are the world's
    piv = O + np.array(task.LEVER_PIVOT, float)
    tab_z = task.LEVER_PIVOT[2] + 0.009                          # finger tips at the thumb tab's mid-height (saw frame)
    flat = np.asarray(grip_rotation(0.0), float)

    def tab_push(theta, side):
        """TCP pushing the tab's +x face (side +1, toward the robot) or its -x face (side -1) with the lever at
        ``theta``."""
        r = _rz(theta)
        c = piv + r @ np.array([0.0, -task.LEVER_L + 0.005, 0.0])
        n = r @ np.array([1.0, 0.0, 0.0])
        tip = c + side * n * (0.005 + TIP_HALF - PUSH_IN)
        tip[2] = O[2] + tab_z
        return tip + [0.0, 0.0, TIP], r @ flat

    # 1. free the plate: push the tab out round the lever's pivot
    p, r = tab_push(0.0, 1)
    yield from R.move(np.array([0.16, -0.22, top + 0.22]), r, 2.0, grip=0.0)
    yield from R.move(p + [0.004, 0, 0.04], r, 1.5)
    yield from R.move(p + [0.004, 0, 0.0], r, 0.8)
    free = task.LEVER_FREE + math.radians(5.0)
    for k in range(1, 11):
        p, r = tab_push(free * k / 10, 1)
        yield from R.move(p, r, 0.2)
    yield from R.move(R.pos - R.off + [0, 0, 0.04], R.rot, 0.8)

    # 2. pinch the plate's top edge near the rear and swing it out on its pivot
    target = task.shoe_for(0.035 if variant == "deep" else task.TARGET)
    pv = O + np.array([task.SHOE_X, task.PIVOT_Y, 0.0])
    g0 = np.array([task.SHOE_X + task.SHOE_T / 2, PINCH_Y, task.SHOE_Z[1] - PINCH_DOWN]) - np.array(
        [task.SHOE_X, task.PIVOT_Y, 0.0])
    pinch = np.asarray(grip_rotation(math.pi / 2), float)       # fingers close across the 3 mm plate (along x)

    def plate(beta):
        return pv + _rz(beta) @ g0 + [0.0, 0.0, TIP], _rz(beta) @ pinch

    p, r = plate(0.0)
    yield from R.move(p + [0, 0, 0.05], r, 1.5, grip=OPEN_GRIP)
    yield from R.move(p, r, 1.0)
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)
    held = ctx.actual(R.name) - p
    held = held if float(np.linalg.norm(held)) < 0.02 else np.zeros(3)
    if os.environ.get("DEMO_DEBUG"):
        print(f"pinch offset mm {np.round(1000 * held, 2)} ik worst {R.worst_mm:.1f} swing {math.degrees(target):.2f} deg",
              flush=True)
    n = 24
    for k in range(1, n + 1):                                   # slowly, the arm taking out its own lag as it goes
        p, r = plate(target * k / n)
        yield from R.move(p + held, r, 0.3, ctx=ctx)
    yield from R.move(p + held, r, 1.0, ctx=ctx)
    yield from wait(0.5)
    yield from R.set_grip(OPEN_GRIP, 0.6)
    yield from R.move(R.pos - R.off + [0, 0, 0.05], R.rot, 0.8)
    R.off = np.zeros(3)                                         # the pull's lag correction no longer applies
    if variant == "nolock":
        yield from R.move(np.array([0.16, -0.22, top + 0.22]), flat, 2.0)
        yield from wait(1.5)
        return

    # 3. clamp it: push the tab back home from its other side
    yield from R.set_grip(0.0, 0.4)
    start = task.LEVER_FREE                                     # from the lever's stop: it is somewhere short of it
    p, r = tab_push(start, -1)
    yield from R.move(p + [-0.004, 0, 0.04], r, 1.5)
    yield from R.move(p + [-0.004, 0, 0.0], r, 0.8)
    for k in range(1, 15):
        p, r = tab_push(start + (math.radians(3.0) - start) * k / 14, -1)
        yield from R.move(p, r, 0.2)
    yield from wait(0.3)
    yield from R.move(R.pos - R.off + [-0.02, 0, 0.05], R.rot, 0.8)
    yield from R.move(np.array([0.16, -0.22, top + 0.22]), flat, 2.0)
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
        sh, turn = task.saw_shift(saw)
        print(f"step {step} score {info.get('score')} | depth mm {1000 * task.depth(saw):.2f} shoe deg"
              f" {math.degrees(saw.hinges.get('shoe', 0.0)):.2f} lever deg {math.degrees(saw.hinges.get('depth_lever', 0.0)):.1f}"
              f" locked {task.lever_locked(saw)} | saw moved mm {1000 * sh:.1f} turned deg {math.degrees(turn):.1f}"
              f" | failed {task._STATE['failed']} changed {task._STATE['changed']} clear {task._STATE['clear']}", flush=True)
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
