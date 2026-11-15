"""Scripted full-score demo for the side-release buckle task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The right arm presses the male half's webbing tail down on the table with its closed fingers. The left arm comes down
over the female half with its fingers open across the buckle and closes on the female's sides just behind its mouth,
where the male's two tabs stand out of the windows: closing presses both tabs in and holds the female. Still
squeezing, it draws the female straight off along the buckle, away from the held-down male. Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to
the output folder.
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


def reachable_yaw(arm, pos, yaw):
    """The grip yaw, or the same parallel grip turned 180 degrees, whichever the arm reaches more exactly."""
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


def _rot_x(angle) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


# The demo runs open loop: it reads no simulator state while it runs (DEMO_DEBUG only prints it). Every target comes
# from the published layout in task.py; the layout is fixed (object_jitter=0).
TCP_ABOVE_TIP = 0.013  # the pads' tips are 13 mm below the TCP
TILT_L = math.radians(10)  # the left hand leans 10 degrees out along the buckle (about its closing axis), which keeps
#                            its wrist clear of the right arm's
TIP_Z_L = 0.0035       # the left pads' tip centres 3.5 mm above the table (their lowest corners about 1 mm)
PINCH_Y = 0.028        # the left pads (27 mm wide) centred 28 mm behind the female's mouth: over the tabs, clear of it
L_OPEN = 52 / 94       # the left fingers come down opened 52 mm (the tabs span 44 mm), so a finger that reaches the
#                        light buckle first pushes it only a little before the other arrives
PRESS_Y = -0.148       # the right fingertips press the male's webbing tail down near its end, 148 mm from the mouth
PRESS_DEPTH = 0.025    # commanded 25 mm below the webbing's top: the compliant arm presses it down firmly
PULL = 0.075           # how far the left hand draws the female off (the male's tongue is 47 mm long)


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    T = task
    x0, y0 = T.BUCKLE_XY
    press = np.array([x0, y0 + PRESS_Y, T.TOP + 0.0016 + TCP_ABOVE_TIP - PRESS_DEPTH])
    rot_l = _rot_x(-TILT_L) @ grip_rotation(reachable_yaw(L, np.array([x0, y0 + PINCH_Y, T.TOP + 0.02]), -math.pi / 2))
    rot_r = grip_rotation(reachable_yaw(R, press, 0.0))
    pinch = np.array([x0, y0 + PINCH_Y, T.TOP + TIP_Z_L]) - TCP_ABOVE_TIP * rot_l[:, 2]   # fingers close across x
    up = np.array([0, 0, 0.07])

    # 1. Right arm: fingers closed, press the male half's webbing tail down on the table, and keep pressing.
    yield from R.move(press + up, rot_r, 2.5, grip=0.0)
    yield from R.move(press + np.array([0, 0, PRESS_DEPTH + 0.004]), rot_r, 1.2)
    yield from R.move(press, rot_r, 1.0)
    yield from wait(0.3)

    # 2. Left arm: fingers open 52 mm across the buckle, down beside the female's windows, and close: the pads press
    #    both tabs in until they stop on the female's sides, and hold the female.
    yield from L.move(pinch + up, rot_l, 2.5, grip=L_OPEN)
    yield from L.move(pinch, rot_l, 1.5)
    yield from L.set_grip(0.0, 1.5)
    yield from wait(0.5)

    # 3. Keep squeezing and draw the female straight off along the buckle, away from the held-down male.
    yield from L.move(pinch + np.array([0, PULL, 0]), rot_l, 4.0)
    yield from wait(2.0)


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
        if steps % 100 == 0 or (os.environ.get("DEMO_DEBUG") and steps % 20 == 0):
            msg = f"step {steps} score {info.get('score')}"
            if os.environ.get("DEMO_DEBUG"):
                st = ctx.states()
                m, f = st["male"], st["female"]
                pa, pb = task.pressed_in(m)
                from yam_arena import ik, station

                st14 = np.asarray(next(iter(res.observation.state.values())), float).ravel()
                for k, arm in enumerate(("left", "right")):
                    p_, _ = ik.tcp_pose(st14[7 * k:7 * k + 6], station.BASES[arm])
                    msg += f" | {arm[0]} tcp {np.round(1000 * p_, 1)} grip {st14[7 * k + 6]:.2f}"
                msg += (f" | gap {1000 * task.gap(m, f):.1f}mm tabs in {1000 * pa:.2f} {1000 * pb:.2f}mm"
                        f" | female {np.round(1000 * np.asarray(f.pos), 1)} male {np.round(1000 * np.asarray(m.pos), 1)}")
            print(msg, flush=True)
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
