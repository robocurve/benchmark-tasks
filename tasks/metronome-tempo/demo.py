"""Scripted full-score demo for the metronome task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1600] [--no-record] [--cam 640x352]

The left arm pinches the sliding weight across its sides (fingers pointing at the scale, closing across the rod), slides
it up the rod from the 120 notch to the 60 notch (the pinch also holds the pendulum upright while the weight is pushed
over the notches), eases off, lets go and draws back. It then closes its fingers, puts the outside of the fingertips
against the weight's left side, swings the pendulum 40 degrees to the right along its arc, and pulls the hand straight back toward
the arms, out of the pendulum's plane, so the pendulum swings on its own. The arm goes back to where it started.
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


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def _cols(x, z) -> np.ndarray:
    """A TCP rotation with the fingers closing along ``x`` and pointing along ``z``."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    return np.column_stack([x, np.cross(z, x), z])


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it. With
    ``solve=False`` it only records where each move ends (for the offline reach check)."""

    def __init__(self, name: str, base_pos, solve: bool = True):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base, self.solve = name, np.asarray(base_pos, float), solve
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.ends: list[tuple[np.ndarray, np.ndarray, str]] = []

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, label: str = ""):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        pos = np.asarray(pos, float)
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            p, r = p0 + s * (pos - p0), ik.slerp_matrix(r0, rot, s)
            if self.solve:
                self._solve(p, r)
            else:
                self.pos, self.rot = p, r
            yield
        self.ends.append((pos.copy(), np.asarray(rot, float).copy(), label))

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip, "grip")


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


# The demo runs open loop except for one thing: it reads where the weight is from the simulator to click it into the
# 60 notch, as a person reads the scale. The arm is compliant, so while it pushes the weight over the rod's ridges the
# weight lags the commanded hand position by several millimetres, and whether it then stops in the 60 notch or one
# either side is too sensitive for an open-loop slide. So the slide is commanded open loop to stop short, in or below
# the 60 notch ("slide_lag", measured with DEMO_CALIBRATE=1), and then, while the weight is not in the 60 notch, the
# hand eases off and pushes it slowly one click at a time toward 60, stopping as soon as it has clicked over.
CAL = {
    "slide_lag": 0.0072,   # m: commanded this far past the 60 line, the weight stops a notch or so short
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
_MEASURED: dict = {}

PRELOAD = 0.008           # m: the hand pressed this far toward 60 at once before the slow final push
EASE = 0.0025             # m: the hand eased back to this far above the weight's goal before letting go
OPEN = 0.45               # grip: the pads about 42 mm apart
TIPS = 0.013              # the pad tips stand this far past the TCP
FINGER_OUT = 0.0064       # closed, each fingertip's outer face is this far from the TCP's centre plane
SWING_DEG = 40.0          # how far the pendulum is swung before it is let go


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    L = ctx.left
    T = task
    ex, ey, ez = np.eye(3)
    rot = _cols((0, -1, 0), (1, 0, 0))           # fingers pointing +x (at the scale), closing along y (across the rod)
    x_rod = T.CASE_XY[0] + T.X_ROD
    pivot = np.array([x_rod, T.CASE_XY[1], T.TOP + T.Z_PIV])
    x_pinch = x_rod - 0.015                      # the TCP: the pad tips 2 mm in front of the rod's centre plane

    def measure(key, value):
        if CALIBRATE:
            _MEASURED[key] = round(float(value), 5)
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)

    # 1. Pinch the weight across its sides, level with its middle.
    z0 = pivot[2] + T.SL_START - T.SL_H / 2
    yield from L.move((x_pinch - 0.06, pivot[1], z0), rot, 3.0, grip=OPEN, label="in front of the weight")
    yield from L.move((x_pinch, pivot[1], z0), rot, 1.5, label="pads either side of it")
    yield from L.set_grip(0.0, 0.8)
    yield from wait(0.3)

    # 2. Slide it up the rod until its top edge is on the 60 line.
    goal = T.MARK_Z[T.TARGET]
    z1 = z0 + (goal - T.SL_START) + CAL["slide_lag"]
    yield from L.move((x_pinch, pivot[1], z1), rot, 2.5, label="slide it up to 60")
    yield from wait(0.3)
    if CALIBRATE:
        s = ctx.states()
        top = T.weight_top(s["pendulum"].pos, s["pendulum"].rotation(), s["sliding_weight"].pos)
        measure("slide_lag", goal - top + CAL["slide_lag"])
    # The compliant arm still pulls up on the weight (just short of the click force); ease off to just above where the
    # weight is (still pulling up a little, so as not to push it back down a notch), so that opening the fingers does
    # not drag it over the next ridge.
    z_eased = z0 + (goal - T.SL_START) + EASE
    yield from L.move((x_pinch, pivot[1], z_eased), rot, 0.8, label="ease off")
    yield from wait(0.3)

    # Click it into the 60 notch: read the scale, and while it is not at 60, push it slowly toward 60 until it clicks
    # over a ridge, then ease off (the arm is still pushing with up to the click force) and read again.
    states = getattr(ctx, "states", None)

    def top_now():
        s = states()
        return T.weight_top(s["pendulum"].pos, s["pendulum"].rotation(), s["sliding_weight"].pos)

    for _ in range(8 if states else 0):
        top = top_now()
        notch = T.notch_at(top)
        if notch == T.TARGET:
            break
        d = 1.0 if top < goal else -1.0
        z = pivot[2] + top - T.SL_H / 2                  # the weight's middle: the hand eased to it
        yield from L.move((x_pinch, pivot[1], z), rot, 0.5, label="ease to the weight")
        z += d * PRELOAD                                 # the compliant arm takes about 9 mm to push with 3 N:
        yield from L.move((x_pinch, pivot[1], z), rot, 0.8, label="press toward 60")   # most of it at once,
        for _k in range(240):                            # then 0.5 mm/s toward 60, until it is past the ridge's peak
            z += d * 0.000025
            yield from L.move((x_pinch, pivot[1], z), rot, 1.0 / CONTROL_HZ)
            t = top_now()
            if CALIBRATE and _k % 10 == 0:
                print(f"ramp d {d:+.0f} cmd_mm {1000 * (z - pivot[2] + T.SL_H / 2):.2f} top_mm {1000 * t:.2f}", flush=True)
            if T.notch_at(t) != notch:
                # past the peak: brake at once (pull the hand back to just short of the next notch), or the arm's
                # stored push carries the weight on over the next ridge too
                yield from L.move((x_pinch, pivot[1], pivot[2] + t - T.SL_H / 2 - d * 0.002), rot,
                                  1.0 / CONTROL_HZ)
                break
        yield from wait(0.3)
        z_eased = pivot[2] + top_now() - T.SL_H / 2 + d * 0.001
        yield from L.move((x_pinch, pivot[1], z_eased), rot, 0.5, label="clicked: ease off")
        yield from wait(0.3)
    if CALIBRATE:
        measure("set_top_minus_goal", top_now() - goal)
    yield from L.set_grip(OPEN, 0.6)
    yield from L.move((x_pinch - 0.07, pivot[1], z_eased), rot, 1.2, label="let go, draw back")

    # 3. Fingers closed; the outside of the fingertips against the weight's left side, level with its middle.
    s_c = goal - T.SL_H / 2
    side = T.SL_W / 2 + FINGER_OUT + 0.001

    def at(alpha_deg, y_off):
        a = math.radians(alpha_deg)
        p = pivot + np.array([0.0, y_off * math.cos(a) - s_c * math.sin(a), y_off * math.sin(a) + s_c * math.cos(a)])
        p[0] = x_pinch
        return p

    yield from L.move(at(0.0, side + 0.03) - 0.06 * ex, rot, 2.0, grip=0.0, label="to the left of the weight")
    yield from L.move(at(0.0, side + 0.03), rot, 1.0, label="fingertips beside it")
    yield from L.move(at(0.0, side), rot, 1.0, label="against its left side")

    # 4. Swing the pendulum to the right along its arc, then pull the hand straight back, out of its plane.
    n = 10
    for k in range(1, n + 1):
        yield from L.move(at(SWING_DEG * k / n, side), rot, 0.25, label="swing it" if k == n else "")
    yield from wait(0.3)
    yield from L.move(L.pos - 0.08 * ex, rot, 0.4, label="let it go")
    yield from L.move(*L.home, 3.0, grip=1.0, label="back")


def offline_plan(task):
    """The demo's moves without the simulator: each arm's list of (pos, rot, label) at the end of every move."""
    from yam_arena import station

    class _Ctx:
        left = Arm("left", station.BASES["left"], solve=False)
        right = Arm("right", station.BASES["right"], solve=False)

    ctx = _Ctx()
    steps = sum(1 for _ in script(task, ctx))
    return {"left": ctx.left.ends, "right": ctx.right.ends, "steps": steps}


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


def run(task_path: str, out_dir: str, max_steps: int = 1600, record: bool = True, cam_width: int = 640,
        cam_height: int = 352, hold_s: float = 14.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction=emb.task.module.INSTRUCTION))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info, first_full = 0, None, False, {}, None
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        if first_full is None and float(info.get("score", 0.0) or 0.0) >= 1.0:
            first_full = steps
        snap(res.observation)
        if steps % 100 == 0:
            print(f"step {steps} score {info.get('score')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    # The trial's score is what the task file's score() reported on the last step (task_score gives 1.0 on success,
    # which score() also is then); it is never set to 1.0 here just because the episode ended.
    final = float(info.get("score", 0.0) or 0.0)
    result = {"task_score": final, "success": success, "steps": steps, "first_full_score_step": first_full,
              "script_ended_step": ended,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if CALIBRATE:
        result["CAL"] = _MEASURED
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
    ap.add_argument("--cam", default="640x352", help="camera width x height")
    a = ap.parse_args()
    w, h = (int(v) for v in a.cam.split("x"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=w, cam_height=h)
