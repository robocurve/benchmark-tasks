"""Scripted full-score demo for the cassette-burner canister task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1600] [--no-record]

The right arm picks the canister up by its middle with its fingers tilted to match the canister's turned notch and
turns its hand upright as it lifts, so that the notch faces up, lays it on the compartment's rails and pushes it into the gas inlet against the seal's
spring. While it holds it there, the left arm pushes the locking lever across behind the canister's base. Then both
let go. Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
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


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def notch_roll(can) -> float:
    """The notch's angle about the canister's axis (world x), degrees: 90 straight up, 125 as it starts."""
    lug = can.rotation() @ np.array([0.0, -1.0, 0.0])
    return math.degrees(math.atan2(lug[2], lug[1]))


# The demo runs open loop: it reads no simulator state. The grip tilt that leaves the notch straight up, and where the
# carried canister hangs in the compliant arm's fingers, were measured once with DEMO_CALIBRATE=1 (which runs the same
# moves and prints the canister's measured pose) and are fixed here. The layout is fixed (object_jitter=0) and the simulation deterministic.
CAL = {
    # the grip's tilt about the canister's axis (degrees) that leaves its notch straight up once lifted and turned back
    "tilt_deg": 38.0,
    # how far to shift the fingers so that the carried canister's centre comes over the compartment's line
    "carry_fix": (0.00045, 0.01367),
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
_MEASURED: dict = {}


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    T = task
    top = T.TOP
    sx, sy = T.STOVE_XY
    can0 = np.array([*T.CAN_XY, top + T.CAN_R])          # the published starting layout
    rot_r = grip_rotation(reachable_yaw(R, can0, 0.0))   # fingers close across the canister (world y)
    tab = np.array([sx + (-0.119), sy + 0.0025 + 0.0075 + 0.0064 + 0.003, top + 0.082])  # beside the handle's +y face
    rot_l = grip_rotation(reachable_yaw(L, tab, 0.0))    # closed fingers, their pads' 27 mm width along x
    up = np.array([0, 0, 0.06])
    ey = np.array([0, 1.0, 0])

    # 1. Right arm: grip the canister by its middle with the fingers tilted 35 degrees about its axis, so that its
    #    notch is square to them, and lift it, turning the hand back upright: the notch now faces straight up. The
    #    fingers come in along the tilted hand's axis, wide open, and close across the canister's axis, held 10 mm
    #    back so that the lower one stays clear of the table.
    tilt = math.radians(CAL["tilt_deg"])
    rot_tilt = _rot_axis((1, 0, 0), tilt) @ rot_r
    grip_pt = can0 - 0.010 * rot_tilt[:, 2]   # closing on a line through its axis, the fingertips 10 mm short
    back = -0.09 * rot_tilt[:, 2]   # backed off along the tilted hand's own approach axis
    yield from R.move(grip_pt + back + np.array([0, 0, 0.03]), rot_tilt, 2.0, grip=1.0)
    yield from R.move(grip_pt + back, rot_tilt, 0.8)
    yield from R.move(grip_pt, rot_tilt, 1.5)
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from R.move(grip_pt + np.array([0, 0, 0.08]), rot_r, 2.0)
    yield from wait(0.5)
    if CALIBRATE:
        s = ctx.states()["canister"]
        _MEASURED["notch_after_lift"] = round(notch_roll(s), 2)
        print("CAL so far = " + json.dumps(_MEASURED), flush=True)

    pre = np.array([sx - 0.025, sy + T.COMP_Y, top + T.AXIS_Z + 0.010])   # the hand 10 mm above the canister's axis
    yield from R.move(pre + np.array([0, 0, 0.07]), rot_r, 2.0)
    yield from wait(0.5)
    if CALIBRATE:   # where the canister hangs in the fingers relative to where they were sent
        c = ctx.states()["canister"].pos
        _MEASURED["carry_fix"] = [round(float(pre[0] - c[0]), 5), round(float(pre[1] - c[1]), 5)]
        print("CAL so far = " + json.dumps(_MEASURED), flush=True)
    fx, fy = _MEASURED["carry_fix"] if CALIBRATE else CAL["carry_fix"]
    pre = pre + np.array([fx, fy, 0.0])
    yield from R.move(pre + np.array([0, 0, 0.07]), rot_r, 1.0)
    yield from R.move(pre - np.array([0, 0, 0.001]), rot_r, 1.5)
    yield from wait(0.3)
    # Push it in: commanded about 20 mm past the plunger's stop, since the arm lags; the stop holds the canister.
    yield from R.move(R.pos + np.array([0.025 + 0.015 + 0.025, 0, -0.001]), rot_r, 3.5)
    yield from wait(2.0)

    # 3. Left arm: with its fingers closed, push the lever's handle across behind the canister's base (commanded 25 mm past its stop: the arm lags).
    #    The fingers come down 30 mm to the handle's left, clear of it, and then move across onto it.
    beside = tab + np.array([0, 0.030, 0])
    yield from L.move(beside + np.array([0, 0, 0.05]), rot_l, 2.5, grip=0.0)
    yield from L.move(beside, rot_l, 1.5)
    yield from L.move(L.pos - (0.030 + T.LOCK_TRAVEL + 0.025) * ey, rot_l, 3.5)

    # 4. Let go: the right arm first (the seal's spring sets the canister back against the lever), then the left.
    yield from R.set_grip(0.85, 0.8)
    yield from R.move(R.pos + up, rot_r, 1.0)
    yield from L.move(L.pos + np.array([0, 0.004, 0.0]), rot_l, 0.4)
    yield from L.move(L.pos + np.array([0, 0, 0.04]), rot_l, 0.8)


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
        if steps % 100 == 0 or (os.environ.get("DEMO_DEBUG") and steps % 20 == 0):
            msg = f"step {steps} score {info.get('score')}"
            if os.environ.get("DEMO_DEBUG"):
                st = ctx.states()
                can, stove = st["canister"], st["burner"]
                c = task.in_compartment(can, stove)
                msg += (f" | can {np.round(1000 * c, 1)}mm world {np.round(1000 * np.asarray(can.pos), 1)}"
                        f" notch {notch_roll(can):.1f} axis {task.axis_angle(can, stove):.1f}"
                        f" | lock {1000 * stove.sliders['lock']:.1f} plug {1000 * stove.sliders['plug']:.1f}"
                        f" stove moved {1000 * task.stove_moved(stove):.1f} tilt {task.stove_tilt(stove):.1f}")
            print(msg, flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps,
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
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
