"""Scripted full-score demo for the reciprocating-saw blade task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The left arm takes the spare blade across its width between its two rests, lifts it, carries it to the saw's clamp line,
sets it level with the slot. It pushes the tang 6 mm in, presses it onto the slot's floor, and pushes on until the
shoulder meets the clamp face: the tang lifts the jaw's pointed pin, which drops into the tang's hole. It then lets go
and withdraws.

Open loop: every position comes from the task file's fixed layout. Once the blade is held, the arm reads its own tool
point from its joints and shifts later targets by the offset; nothing is read from the simulated objects.

Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
Negative variants (each must score below 1):
    DEMO_VARIANT=short   the tang stops 10 mm short of the shoulder (scores 0)
    DEMO_VARIANT=held    pushed home but never let go (scores below 1)
DEMO_DEBUG=1 logs the blade, the jaw and the grip clearance every 20 steps (logging only).
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
GRIP_DX = 0.030        # the fingers close across the blade 30 mm toward the tang from its origin, between the rests
CARRY_Z = 0.115        # carry height of the blade's origin above the table (the saw's body is 95 mm tall)
LIFT_IN = 0.0003       # aimed 0.3 mm above the floor: the blade rides 0.6 to 1.2 mm high in the fingers, inside the 3 mm slot
PUSH_SPEED = 0.010     # m/s
PRESS = 0.0012         # once the tang is in the slot the hand drops this far, to put the tang on the slot's floor


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
    arm = ctx.left
    rot = np.asarray(grip_rotation(0.0), float)          # fingers down, closing across the blade's width
    top = task.TOP
    blade_z = task.BLADE_Z0 + task.BLADE_T / 2           # the TCP grips the blade's mid-plane
    sit_z = task.BLADE_SIT_Z + LIFT_IN + task.BLADE_T / 2
    gx = GRIP_DX
    pre_x, seat_x = task.PRE_X + gx, task.SEAT_X + gx

    yield from arm.move((task.BLADE_X0 + gx, task.BLADE_Y0, top + 0.080), rot, 2.5, grip=1.0)
    yield from arm.move((task.BLADE_X0 + gx, task.BLADE_Y0, blade_z), rot, 2.0)
    yield from arm.set_grip(0.0, 1.0)
    yield from wait(0.3)
    # Where the hand really is (from its own joints) relative to the grasp point in the layout: the blade sits in the
    # fingers with this offset, so every later target is shifted by it.
    held = ctx.actual(arm.name) - np.array([task.BLADE_X0 + gx, task.BLADE_Y0, blade_z])
    if os.environ.get("DEMO_DEBUG"):
        print(f"grasp offset mm {np.round(1000 * held, 2)}", flush=True)
    yield from arm.move((task.BLADE_X0 + gx, task.BLADE_Y0, top + CARRY_Z), rot, 2.0, ctx=ctx)
    yield from arm.move(np.array((pre_x, task.SAW_Y, top + CARRY_Z)) + held, rot, 3.0, ctx=ctx)
    yield from arm.move(np.array((pre_x, task.SAW_Y, sit_z)) + held, rot, 2.0, ctx=ctx)
    # in until the tang is 6 mm into the slot (short of the jaw's pin), then press it down onto the slot's floor so
    # the pin only has to ride up the tang's thickness, then on to the end
    mid = seat_x - 0.0155
    yield from arm.move(np.array((mid, task.SAW_Y, sit_z)) + held, rot, (mid - pre_x) / PUSH_SPEED, ctx=ctx)
    yield from arm.move(np.array((mid, task.SAW_Y, sit_z - PRESS)) + held, rot, 0.6, ctx=ctx)
    end = seat_x - (0.010 if variant == "short" else 0.0)
    yield from arm.move(np.array((end, task.SAW_Y, sit_z - PRESS)) + held, rot, max(0.6, abs(end - mid) / PUSH_SPEED),
                        ctx=ctx)
    yield from wait(0.6)
    if variant != "held":
        yield from arm.set_grip(1.0, 1.0)
        yield from arm.move((end - 0.02, task.SAW_Y, top + 0.170), rot, 2.0)
        yield from arm.move((0.20, 0.20, top + 0.170), rot, 2.0)
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

        blade, saw = _part_state(emb._env, task.BLADE), _part_state(emb._env, task.SAW)
        print(f"step {step} score {info.get('score')} | blade {np.round(1000 * (blade.pos - [task.SAW_X, task.SAW_Y, task.TOP]), 1)}"
              f" up {np.round(blade.rotation()[:, 2], 3)} gap_mm {1000 * task.seated_gap(blade):.2f}"
              f" seated {task.blade_seated(blade)} jaw_deg {math.degrees(saw.hinges.get(task.JAW_HINGE, 0.0)):.1f}"
              f" | saw_moved_mm {1000 * np.linalg.norm(saw.pos - task._STATE['saw0']):.2f}"
              f" released {task._STATE['released']}", flush=True)

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
