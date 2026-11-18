"""Scripted full-score demo for the ticket-stub task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The left arm pinches the ticket's main part by its top edge and holds it down in its stand; the right arm pinches
the stub by its top edge and pulls it up, so the perforation tears along its length. The right arm carries the stub
over the stub box and lets it fall in; then both arms let go and move away, leaving the ticket in its stand.

Open loop: every position comes from the task file's fixed layout. Nothing is read from the simulated ticket.

Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
Negative variants (each must score below 1):
    DEMO_VARIANT=notear  pinches both parts and lets go without tearing (scores 0)
    DEMO_VARIANT=drop    tears the stub off and drops it on the counter beside the stand (scores 0.25)
DEMO_DEBUG=1 logs the ticket and the stub every 20 steps (logging only).
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
PINCH_DOWN = 0.012     # the finger tips this far below the card's top edge
OPEN = 0.30            # about 28 mm open round the card
LEFT_Y = 0.085         # the left hand pinches the ticket here (the two hands need about 165 mm between them)
RIGHT_Y = -0.085       # the right hand pinches the stub here
TEAR_LIFT = 0.030      # the stub pulled up this far: the perforation parts long before


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


def reachable_yaw(arm, points, yaw):
    """The grip yaw, or the same parallel grip turned 180 degrees, whichever the arm reaches more exactly at all of
    ``points`` (worst case)."""
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    best = None
    for y in (yaw, yaw + math.pi):
        worst = 0.0
        for pos in points:
            _, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(grip_rotation(y)), arm.q, base_pos=arm.base, iters=200)
            worst = max(worst, pe + 0.05 * re)
        if best is None or worst < best[0]:
            best = (worst, y)
    return best[1]


def together(*gens):
    """Run several move generators side by side, one control step each per yield."""
    gens = list(gens)
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        yield


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    variant = os.environ.get("DEMO_VARIANT", "")
    top = task.TOP
    L, R = ctx.left, ctx.right
    zt = top + task.CARD_Z0 + task.CARD_H - PINCH_DOWN + TIP
    pl = np.array([task.STAND_X, LEFT_Y, zt])
    pr = np.array([task.STAND_X, RIGHT_Y, zt])
    # fingers close across the card (along x); each arm takes whichever of the two such grips it reaches best
    rl = np.asarray(grip_rotation(reachable_yaw(L, [pl, pl + [0, 0, 0.08]], math.pi / 2)), float)
    rr = np.asarray(grip_rotation(reachable_yaw(R, [pr, pr + [0, 0, 0.08]], math.pi / 2)), float)
    yield from together(L.move(pl + [-0.02, 0, 0.08], rl, 2.0, grip=OPEN), R.move(pr + [-0.02, 0, 0.08], rr, 2.0, grip=OPEN))
    yield from together(L.move(pl + [0, 0, 0.04], rl, 1.2), R.move(pr + [0, 0, 0.04], rr, 1.2))
    yield from together(L.move(pl, rl, 1.2), R.move(pr, rr, 1.2))
    yield from together(L.set_grip(0.0, 0.8), R.set_grip(0.0, 0.8))
    yield from wait(0.3)
    if os.environ.get("DEMO_DEBUG"):
        print(f"pinched: offsets mm left {np.round(1000 * (ctx.actual('left') - pl), 2)} right"
              f" {np.round(1000 * (ctx.actual('right') - pr), 2)}", flush=True)
    if variant == "notear":
        yield from together(L.set_grip(OPEN, 0.8), R.set_grip(OPEN, 0.8))
        yield from together(L.move(pl + [0, 0, 0.08], rl, 1.2), R.move(pr + [0, 0, 0.08], rr, 1.2))
        yield from wait(1.5)
        return
    # tear: the left hand holds the ticket down, the right pulls the stub straight up along the perforation
    yield from R.move(pr + [0, 0, TEAR_LIFT], rr, 1.5)
    yield from R.move(pr + [0, -0.01, 0.08], rr, 1.2)
    if variant == "drop":
        drop = np.array([task.STAND_X - 0.06, RIGHT_Y - 0.04, top + 0.10])
    else:
        bx, by = task.STUB_BOX_XY
        drop = np.array([bx, by, top + task.STUB_BOX[2] + 0.07])
    yield from R.move(drop, rr, 2.0)
    yield from R.set_grip(OPEN, 0.6)
    yield from wait(0.5)
    yield from L.set_grip(OPEN, 0.6)
    yield from together(L.move(pl + [0, 0, 0.08], rl, 1.2), R.move(drop + [0, 0, 0.05], rr, 1.0))
    yield from together(L.move(np.array([0.16, 0.22, top + 0.22]), rl, 1.5),
                        R.move(np.array([0.16, -0.22, top + 0.22]), rr, 1.5))
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

        ticket, stub = _part_state(emb._env, task.TICKET), _part_state(emb._env, task.STUB)
        print(f"step {step} score {info.get('score')} | gap mm {1000 * task.perf_gap(ticket, stub):.2f} torn"
              f" {task._STATE['torn']} | stub mm {np.round(1000 * (np.asarray(stub.pos) - [0, 0, task.TOP]), 1)}"
              f" in box {task.stub_in_box(stub)} | ticket mm {np.round(1000 * (np.asarray(ticket.pos) - [0, 0, task.TOP]), 1)}"
              f" tilt {math.degrees(task.tilt(ticket)):.1f} in stand {task.ticket_in_stand(ticket)} clear {task._STATE['clear']}",
              flush=True)
        for nm, arm in (("left", ctx.left), ("right", ctx.right)):
            rob = emb._base.scene[f"{nm}_robot"]
            jn, jp = list(rob.joint_names), rob.data.joint_pos.torch[0].cpu().numpy()
            gap = 2 * max(abs(float(jp[jn.index(j)])) for j in ("joint7", "joint8") if j in jn)
            print(f"   {nm}: cmd mm {np.round(1000 * (arm.pos - [0, 0, task.TOP]), 1)} actual mm"
                  f" {np.round(1000 * (ctx.actual(nm) - [0, 0, task.TOP]), 1)} grip cmd {arm.grip:.2f} gap mm {1000 * gap:.1f}",
                  flush=True)

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
