"""Scripted full-score demo for the tilt-window-sash task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

Each arm takes hold of one of the sash's two tilt-latch buttons and the two lift the sash together, about its pivot
bars, from lying flat toward the arms up toward the frame. Fifteen degrees short of upright both slide their buttons
in to their stops, which draws the latch bolts into the sash, and push the sash on until it closes against the upper
sash. They let go of the buttons: the bolts snap out into the jamb slots and hold the sash upright.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
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


def _rot_y(angle) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


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


def pitched_grip(phi: float) -> np.ndarray:
    """A grip pitched ``phi`` from straight down toward +x (the window), fingers still closing along world y."""
    from yam_arena.demo import grip_rotation

    return _rot_y(-phi) @ grip_rotation(0.0)


def grip_pitch(tilt: float) -> float:
    """The grip's pitch for a sash tilt (both radians): following the button as it turns, within what the arms reach.
    Upright the button points up but the arms only reach it from the room side, pitched 45 degrees or more; lying flat
    it points at the arms but they only reach it pitched 60 degrees or less."""
    return float(np.clip(tilt, math.radians(45), math.radians(60)))


def script(task, ctx):
    """The demo as a generator: one yield per control step. Open loop: it uses only the published layout."""
    T = task
    L, R = ctx.left, ctx.right
    tips = 0.013   # PAD_TIP_PAST_TCP: the TCP sits this far behind the pad tips
    pivot = np.array(T.PIVOT)

    def button_tip(side, tilt, dy=0.0):
        """Where the pad tips go on a latch button (12 mm out from the rail, on the button's middle), with the sash at
        ``tilt`` (rad from upright) and the button slid ``dy`` along the rail."""
        local = np.array([-0.004, side * (T.LATCH_Y + 0.005) + dy, T.SASH_TOP + 0.012])
        return pivot + _rot_y(-tilt) @ local

    def tcp(side, tilt, dy=0.0, back=0.0):
        rot = pitched_grip(grip_pitch(tilt))
        axis = rot[:, 2]
        return button_tip(side, tilt, dy) - (tips + back) * axis, rot

    def go(arm, side, tilt, duration, dy=0.0, back=0.0, grip=None):
        p, r = tcp(side, tilt, dy, back)
        yield from arm.move(p, r, duration, grip)

    t0 = T.START_TILT
    # 1. Both arms: come in along the grip's axis to the latch buttons and close on them.
    yield from both(go(L, 1, t0, 2.5, back=0.05, grip=0.5), go(R, -1, t0, 2.5, back=0.05, grip=0.5))
    yield from both(go(L, 1, t0, 1.2), go(R, -1, t0, 1.2))
    yield from both(L.set_grip(0.0, 0.8), R.set_grip(0.0, 0.8))
    yield from wait(0.3)

    # 2. Lift the sash together about its pivot bars, in 3-degree steps, to 15 degrees short of upright.
    lift_to = math.radians(15)
    n = int(round(math.degrees(t0 - lift_to) / 3))
    for k in range(1, n + 1):
        t = t0 + (lift_to - t0) * k / n
        yield from both(go(L, 1, t, 0.25), go(R, -1, t, 0.25))
    yield from wait(0.3)

    # 3. Slide both buttons in, 18 mm past their 12 mm stops (the arms are compliant and lag), drawing the bolts into the sash.
    pull = T.LATCH_TRAVEL + 0.018
    yield from both(go(L, 1, lift_to, 1.0, dy=-pull), go(R, -1, lift_to, 1.0, dy=pull))
    yield from wait(0.3)

    # 4. Push the sash on, buttons held in, until it closes against the upper sash (commanded 6 degrees past: the arms lag).
    close = math.radians(-6)
    n = 6
    for k in range(1, n + 1):
        t = lift_to + (close - lift_to) * k / n
        yield from both(go(L, 1, t, 0.3, dy=-pull), go(R, -1, t, 0.3, dy=pull))
    yield from wait(0.5)

    # 5. Let go of the buttons: the springs snap the bolts out into the jamb slots. Back away.
    yield from both(L.set_grip(0.6, 0.6), R.set_grip(0.6, 0.6))
    yield from both(go(L, 1, close, 1.2, back=0.06), go(R, -1, close, 1.2, back=0.06))
    yield from wait(1.5)


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
        if steps % 100 == 0 or (os.environ.get("DEMO_DEBUG") and steps % 20 == 0 and steps > int(os.environ.get("DEMO_DEBUG_FROM", 0))):
            msg = f"step {steps} score {info.get('score')}"
            if os.environ.get("DEMO_DEBUG"):
                st = ctx.states()
                s = st["sash"]
                r = s.rotation()
                msg += (f" | tilt {task.tilt_deg(s):.2f} pivot {1000 * task.pivot_offset(s):.1f}mm"
                        f" latches {', '.join(f'{k} {1000 * v:.1f}' for k, v in s.sliders.items())}"
                        f" roll {math.degrees(math.asin(max(-1, min(1, r[2, 1])))):.2f}"
                        f" | L tcp {np.round(ctx.left.pos, 3)} R tcp {np.round(ctx.right.pos, 3)}")
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
