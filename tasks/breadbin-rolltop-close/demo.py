"""Scripted full-score demo for the roll-top bread bin task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The right arm, fingers closed and its hand tipped 30 degrees forward, sets its finger tips against the back of the handle ridge on
the open tambour's front slat and pushes it along the grooves' path: forward along the top, round the rounded corner
and down the front until the tambour is shut, then moves away. No grip: the tambour follows its own grooves.

Open loop: every position comes from the task file's fixed layout (the grooves' centreline). Nothing is read from the
simulated tambour.

Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
Negative variants (each must score below 1):
    DEMO_VARIANT=short  pushes the handle 45 mm along the grooves and stops (scores 0.25)
    DEMO_VARIANT=down   presses the open handle straight down, across the grooves (scores 0)
DEMO_DEBUG=1 logs the tambour every 20 steps (logging only).
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
TIP = 0.013            # finger tips below the TCP, along the tool axis
PRESS_OVER = 0.003     # pressed this far past shut
TIP_HALF = 0.008       # half the closed finger tips' length along x
GRIP_UP = 0.008        # the pinching finger tips this far out from the slat's middle, on the ridge
OPEN_GRIP = 0.35       # about 33 mm open: room for the 8 mm ridge, clear of the top board behind it
HAND_OVER = math.radians(150.0)   # pinched round the corner to here, then pushed
PUSH_TILT = math.radians(30.0)   # tipped forward: straight down cannot reach 26 cm out at this height
PUSH_Y = -0.050        # push the ridge this far toward the right arm's side of the bin's middle
PUSH_IN = 0.004        # the closed finger tips this far into the handle's trailing face while pushing


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


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def groove_point(task, s):
    """The grooves' centreline ``s`` metres along from the open handle slat's place toward the shut end:
    (x, z) of the handle slat's pins, and the slat's pitch (0 open, -pi/2 on the front run)."""
    s0 = task.START[0] - task.ARC_C[0]                      # the top run left in front of it
    cx, cz = task.ARC_C
    if s <= s0:
        return task.START[0] - s, task.GROOVE_TOP, 0.0
    s -= s0
    arc = 0.5 * math.pi * task.ARC_R
    if s <= arc:
        a = 0.5 * math.pi + s / task.ARC_R
        return cx + task.ARC_R * math.cos(a), cz + task.ARC_R * math.sin(a), -s / task.ARC_R
    return task.GROOVE_X, cz - (s - arc), -0.5 * math.pi


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    variant = os.environ.get("DEMO_VARIANT", "")
    top = task.TOP
    R = ctx.right

    s_top = task.START[0] - task.ARC_C[0]
    s_shut = s_top + 0.5 * math.pi * task.ARC_R + (task.ARC_C[1] - task.GROOVE_BOTTOM) - (task.GROOVE_W / 2 - task.PIN_R)
    debug = bool(os.environ.get("DEMO_DEBUG"))
    tilt = _ry(-PUSH_TILT)
    down = tilt @ np.asarray(grip_rotation(0.0), float)                # closed fingers, tipped forward 30 degrees
    axis = tilt @ np.array([0.0, 0.0, -1.0])                            # from the TCP to the finger tips
    along = tilt @ np.array([1.0, 0.0, 0.0])                            # the tips' 16 mm length

    def push_tcp(s, into=PUSH_IN):
        """TCP that puts the closed finger tips ``into`` metres into the ridge's trailing face, the slat ``s`` along."""
        x, z, pitch = groove_point(task, s)
        r = _ry(pitch)
        n = r @ np.array([1.0, 0.0, 0.0])                               # the trailing face's outward normal
        face = np.array([x, PUSH_Y, top + z]) + r @ np.array([task.HANDLE[0] / 2, 0.0, task.SLAT_T / 2 + 0.003])
        tip = face + max(0.0, float(n[0])) * TIP_HALF * along - into * n   # tips centred clear of the face
        return tip - TIP * axis

    def pose(s, yaw):
        """TCP and rotation pinching the handle ridge with the slat ``s`` along the grooves; the hand turns with it."""
        x, z, pitch = groove_point(task, s)
        rot = _ry(pitch) @ np.asarray(grip_rotation(yaw), float)
        tcp = np.array([x, PUSH_Y, top + z]) + _ry(pitch) @ np.array([0.0, 0.0, task.SLAT_T / 2 + GRIP_UP + TIP])
        return tcp, rot

    s_hand = s_top + task.ARC_R * (HAND_OVER - 0.5 * math.pi)        # the hand lets go here, on the curve
    if variant == "down":                                               # presses the handle straight down instead
        p = push_tcp(0.0, -0.01) - TIP_HALF * along
        yield from R.move(p + [0, 0, 0.06], down, 2.5, grip=0.0)
        yield from R.move(p + [0, 0, -0.012], down, 2.0)
        yield from wait(0.5)
        yield from R.move(p + [0, 0, 0.08], down, 1.5)
        yield from wait(1.5)
        return

    yaw = math.pi / 2                                                    # fingers close along the groove
    p0, r0 = pose(0.0, yaw)
    yield from R.move(np.array([0.16, -0.20, top + 0.24]), r0, 2.0, grip=OPEN_GRIP)
    yield from R.move(p0 + [0, 0, 0.05], r0, 1.5)
    yield from R.move(p0, r0, 1.2)
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)
    held = ctx.actual(R.name) - p0
    held = held if float(np.linalg.norm(held)) < 0.02 else np.zeros(3)
    if debug:
        print(f"pinched: offset mm {np.round(1000 * held, 2)} ik worst {R.worst_mm:.1f}", flush=True)
    end = 0.045 if variant == "short" else s_hand
    n = max(4, round(end / 0.005))
    for k in range(1, n + 1):                                           # along the top and partway round the corner
        p, r = pose(end * k / n, yaw)
        yield from R.move(p + held, r, 0.18, ctx=ctx)
    if debug:
        print(f"drawn to s {1000 * end:.0f} mm: ik worst {R.worst_mm:.1f}", flush=True)
    yield from wait(0.3)
    yield from R.set_grip(OPEN_GRIP, 0.6)
    x, z, pitch = groove_point(task, end)
    yield from R.move(R.pos - R.off + _ry(pitch) @ np.array([0.0, 0.0, 0.04]), R.rot, 1.0)   # off the ridge
    if variant == "short":
        yield from R.move(np.array([0.16, -0.20, top + 0.24]), r0, 2.0)
        yield from wait(1.5)
        return
    # the rest round the corner and down the front: closed fingers, tipped forward, pushing on the ridge's back
    p = push_tcp(end, -0.006)
    yield from R.move(p + [0.0, 0.0, 0.03], down, 1.5, grip=0.0)
    yield from R.move(p, down, 0.8)
    n = max(4, round((s_shut - end) / 0.005))
    for k in range(1, n + 1):
        yield from R.move(push_tcp(end + (s_shut - end) * k / n), down, 0.15)
    yield from R.move(push_tcp(s_shut) - [0.0, 0.0, PRESS_OVER], down, 0.5)
    if debug:
        print(f"pushed shut: ik worst {R.worst_mm:.1f}", flush=True)
    yield from wait(0.5)
    end = s_shut
    x, z, pitch = groove_point(task, end)
    back = _ry(pitch) @ np.array([1.0, 0.0, 0.0])
    yield from R.move(R.pos - R.off + 0.02 * back + [0.0, 0.0, 0.01], down, 0.6)
    yield from R.move(np.array([0.15, 0.0, top + 0.20]), down, 1.2)
    yield from R.move(np.array([0.16, -0.20, top + 0.24]), down, 1.5)
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

        tam = _part_state(emb._env, task.TAMBOUR)
        js = [math.degrees(v) for v in tam.hinges.values()]
        print(f"step {step} score {info.get('score')} | lead mm {np.round(1000 * task.lead(tam), 1)}"
              f" to go mm {1000 * task.distance({'tambour': tam}):.1f} front {task.on_front(tam)} shut {task.shut(tam)}"
              f" | joints deg min {min(js):.1f} max {max(js):.1f} | moved {task._STATE['moved_once']}"
              f" clear {task._STATE['clear']}", flush=True)
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
