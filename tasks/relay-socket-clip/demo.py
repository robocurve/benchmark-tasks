"""Scripted full-score demo for the relay-socket task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The right arm takes the new relay by its body from the foam tray, carries it over the socket and pushes it straight
down until its body sits on the socket (the blades go 6.4 mm into the contacts). It lets go and lifts away, then with
closed fingers pushes the open hold-down clip forward along its arc until it drops onto the relay, presses it down
until it snaps under its catch, and lifts away.

Open loop: every position comes from the task file's fixed layout. Once the relay is held, the arm reads its own tool
point from its joints and shifts later targets by the offset. Nothing is read from the simulated objects.

Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
Negative variants (each must score below 1):
    DEMO_VARIANT=short    pushes the relay only 3 mm in and leaves it (scores 0.25)
    DEMO_VARIANT=noclip   seats the relay but leaves the clip open (scores 0.5)
DEMO_DEBUG=1 logs the relay, the contacts and the clip every 20 steps (logging only).
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
GRIP_UP = 0.018        # the TCP this far above the relay body's underside (its middle)
CARRY_Z = 0.120        # TCP height while the relay is carried
OVERPUSH = 0.0008      # the push ends this far below flush
PUSH_TIME = 2.5


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
    rot = np.asarray(grip_rotation(0.0), float)                 # fingers close across the relay's width (y)
    tx, ty = task.TRAY_XY
    sx, sy = task.SOCKET_XY
    body0 = task.TOP + 0.004 + task.CONTACT_TRAVEL + 0.0005     # the relay body's underside in the tray

    pick = np.array([tx, ty, body0 + GRIP_UP])
    yield from R.move(pick + [0, 0, 0.07], rot, 2.5, grip=1.0)
    yield from R.move(pick, rot, 1.5)
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)
    held = ctx.actual(R.name) - pick
    if os.environ.get("DEMO_DEBUG"):
        print(f"grasp offset mm {np.round(1000 * held, 2)}", flush=True)

    def at(x, y, body_z):
        return np.array([x, y, body_z + GRIP_UP]) + held

    over = top + task.S_TOP + task.CONTACT_TRAVEL + 0.004          # blades 4 mm above the socket's top
    yield from R.move(at(tx, ty, top + CARRY_Z - GRIP_UP), rot, 1.5, ctx=ctx)
    yield from R.move(at(sx, sy, top + CARRY_Z - GRIP_UP), rot, 2.5, ctx=ctx)
    yield from R.move(at(sx, sy, over), rot, 2.0, ctx=ctx)
    home = top + task.S_TOP - OVERPUSH
    end = home + (0.0034 if variant == "short" else 0.0)
    yield from R.move(at(sx, sy, end), rot, PUSH_TIME, ctx=ctx)
    yield from wait(0.4)
    yield from R.set_grip(1.0, 0.8)
    yield from R.move(R.pos - R.off + [0, 0, 0.06], rot, 1.2)
    if variant in ("noclip", "short"):
        yield from R.move(np.array([0.15, -0.25, top + 0.20]), rot, 2.0)
        yield from wait(1.5)
        return

    # close the clip: closed fingers push its free end forward along its arc past upright; it drops onto the relay
    yield from R.set_grip(0.0, 0.5)
    piv = np.array([sx + task.PIVOT[0], sy, top + task.PIVOT[2]])
    L = task.CLIP_L

    def free_end(theta):                                          # theta: the clip's angle above flat
        return piv + np.array([-L * math.cos(theta), 0.0, L * math.sin(theta)])

    tip_below = 0.013                                             # finger tips below the TCP
    th0 = task.CLIP_OPEN
    start = free_end(th0) + np.array([0.012, 0.0, tip_below - 0.004])
    yield from R.move(start + [0, 0, 0.03], rot, 1.5)
    yield from R.move(start, rot, 1.0)
    for k in range(1, 13):
        th = th0 - math.radians(70.0) * k / 12                  # to 50 degrees: well past upright
        p = free_end(th) + np.array([0.012 * math.sin(th) + 0.004, 0.0, tip_below - 0.004 * math.cos(th)])
        yield from R.move(p, rot, 0.25)
    yield from R.move(R.pos - R.off + [0.02, 0, 0.04], rot, 0.8)
    yield from wait(1.0)                                          # it falls onto the relay
    press = piv + np.array([-0.026, 0.0, 0.002])                  # on the hood, over the relay's middle
    yield from R.move(press + [0, 0, tip_below + 0.03], rot, 1.2)
    yield from R.move(press + [0, 0, tip_below - 0.003], rot, 1.2)
    yield from wait(0.4)
    yield from R.move(press + [0, 0, tip_below + 0.05], rot, 1.0)
    yield from R.move(np.array([0.15, -0.25, top + 0.20]), rot, 2.0)
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

        sock, relay = _part_state(emb._env, task.SOCKET_PART), _part_state(emb._env, task.RELAY)
        d = task.relay_offset(relay, sock)
        print(f"step {step} score {info.get('score')} | relay off mm {np.round(1000 * d, 2)}"
              f" tilt {math.degrees(task.tilt(relay)):.1f} seated {task.seated(relay, sock)}"
              f" | contacts mm {1000 * sock.sliders.get('contacts', 0.0):.2f}"
              f" clip deg {math.degrees(task.clip_angle(sock)):.1f} latched {task._STATE['latched']}"
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
