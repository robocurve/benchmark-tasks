"""Scripted full-score demo for the wheelchair-armrest task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1600] [--no-record]

Both arms reach in level from the arms' side, fingers closing up and down. The right arm pinches the red release tab
and pushes it toward the back of the chair, which draws the lock's catch in; the left arm pinches the arm pad. They
lift the armrest straight up out of its sockets together, carry it toward the arms and stand it on its post feet on the
table. The right arm moves from the tab to the front of the pad, and the two arms lift it, tip it over onto its side
(pad toward the chair, posts toward the arms) while lowering it, and lay it on the table.
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
PRE = math.radians(float(os.environ.get("DEMO_PRE", 2.0)))


def _rot_y(angle) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def level_grip(flip: bool = False) -> np.ndarray:
    """TCP rotation for a level grip reaching forward (+x) with the fingers closing up and down (z); ``flip`` turns it
    180 degrees about the reach direction (the same grip)."""
    x = np.array([0.0, 0.0, -1.0 if flip else 1.0])
    z = np.array([1.0, 0.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


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




def reachable_flip(arm, pos, rot_of) -> bool:
    """Whichever of the two equivalent grips (``rot_of(False)`` or ``rot_of(True)``) the arm reaches more exactly."""
    from yam_arena import ik

    best = None
    for flip in (False, True):
        _, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot_of(flip)), arm.q, base_pos=arm.base, iters=200)
        if best is None or pe + 0.05 * re < best[0]:
            best = (pe + 0.05 * re, flip)
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



# The demo runs open loop: it reads no simulator state, only the published layout in task.py. The armrest is held in
# both grippers from the moment it leaves its sockets, so where it goes is where the arms put it.


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    L, R = ctx.left, ctx.right
    T = task
    p0 = np.array(T.ARMREST.pos, float)            # the armrest's origin (midway between its post feet) at the start
    ex, ez = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
    back = 0.06                                    # approach and leave 6 cm out along the reach

    # Grip points in the armrest's frame (TCP, 13 mm behind the finger tips along the reach).
    pad_rear = np.array([-0.025, 0.090, 0.311])    # tips 15.5 mm onto the pad's arm-side edge, at mid-thickness
    pad_front = np.array([-0.025, -0.090, 0.311])
    tab = np.array([-0.028, T.POST_Y[0], (T.TAB[4] + T.TAB[5]) / 2])   # tips 4 mm short of the post

    def at(origin, tilt, local):
        return origin + _rot_y(tilt) @ local

    fl = reachable_flip(L, at(p0, 0, pad_rear), level_grip)
    fr = reachable_flip(R, at(p0, 0, tab), level_grip)

    def rot(flip, tilt=0.0):
        # The grips are on the arm side of the posts, so the armrest's weight pitches the compliant wrists; they are
        # turned PRE the other way so it comes up square in its sockets.
        return _rot_y(tilt - PRE) @ level_grip(flip)

    def grab(arm, flip, target):
        yield from arm.move(target - back * ex, rot(flip), 2.0, grip=1.0)
        yield from arm.move(target, rot(flip), 1.2)
        yield from arm.set_grip(0.0, 0.8)
        yield from wait(0.3)

    # 1. Reach in level: right fingers across the red tab, left fingers across the pad's rear half; close.
    yield from both(grab(R, fr, at(p0, 0, tab)), grab(L, fl, at(p0, 0, pad_rear)))

    # 2. Right: push the tab toward the back of the chair, past its 7 mm stop (the arm is compliant): catch drawn in.
    yield from R.move(R.pos + np.array([0, T.LOCK_TRAVEL + 0.010, 0]), rot(fr), 1.0)
    yield from wait(0.3)

    # 3. Both: lift straight up together, 16 cm (the posts are 10 cm deep; the arms sag a few cm under the load).
    lift = 0.160
    yield from both(L.move(L.pos + lift * ez, rot(fl), 4.0), R.move(R.pos + lift * ez, rot(fr), 4.0))
    yield from wait(0.5)

    # 4. Both: carry it 5 cm toward the arms, then tip it over onto its side (top away from the arms, posts toward
    #    them) while carrying it on toward the arms and lowering it, so it lies on the pad's chair-side face, clear of
    #    the chair. The right hand keeps the tab pushed back (it no longer matters once the posts are out).
    push = np.array([0, T.LOCK_TRAVEL, 0])
    p1 = p0 + np.array([-0.05, 0, lift])
    yield from both(L.move(at(p1, 0, pad_rear), rot(fl), 1.5), R.move(at(p1, 0, tab) + push, rot(fr), 1.5))
    pf = np.array([0.020, 0.0, T.TOP + T.PAD_X / 2 + 0.012])   # its side 12 mm over the table: set down on release
    n = 24
    for k in range(1, n + 1):
        s = k / n
        tilt = math.radians(90) * s
        p = p1 + s * (pf - p1)
        low = min((p + _rot_y(tilt) @ c)[2] for c in T._CORNERS) - T.TOP
        if k < n - 3 and low < 0.040:
            p = p + np.array([0, 0, 0.040 - low])   # keep its lowest corner 4 cm off the table until the end
        yield from both(L.move(at(p, tilt, pad_rear), rot(fl, tilt), 0.3),
                        R.move(at(p, tilt, tab) + push, rot(fr, tilt), 0.3))

    # 5. Let go (it settles the last few degrees onto its side) and lift the hands away.
    yield from both(L.set_grip(1.0, 0.6), R.set_grip(1.0, 0.6))
    yield from both(L.move(L.pos + 0.05 * ez, L.rot, 1.0), R.move(R.pos + 0.05 * ez, R.rot, 1.0))


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
        if steps % 100 == 0 or (os.environ.get("DEMO_DEBUG") and steps % 20 == 0 and steps > int(os.environ.get("DEMO_DEBUG_FROM", 0))):
            msg = f"step {steps} score {info.get('score')}"
            if os.environ.get("DEMO_DEBUG"):
                st = ctx.states()
                a = st["armrest"]
                msg += (f" | armrest {np.round(1000 * (np.asarray(a.pos) - np.array(task.ARMREST.pos)), 1)}mm"
                        f" lock {1000 * a.sliders['lock']:.1f} tilt {task.tilt_deg(a):.1f}"
                        f" z-axis {np.round(a.rotation()[:, 2], 3)} low {1000 * task.lowest(a):.1f} out {task.posts_out(a)}"
                        f" yaw {math.degrees(math.atan2(a.rotation()[1, 0], a.rotation()[0, 0])):.2f}"
                        f" | L {np.round(ctx.left.pos, 3)} R {np.round(ctx.right.pos, 3)}")
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
    ap.add_argument("--max-steps", type=int, default=1600)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
