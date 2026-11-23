# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of
# the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""linkdemo: closed-loop scripted demos for yam_arena task files (full-pose IK, both arms, observable state only).

    python linkdemo.py TASK_FILE OUT_DIR [--max-steps 1200] [--variant NAME] [--no-record]

TASK_FILE defines ``demo_script(ctx)``, a generator that drives the arms with the primitives below and reads the
scene only through ``ctx.observe()`` (link and object poses as a camera could measure them, plus the robot's own
gripper state; no joint readouts, latch flags or brake states). Optional ``demo_report()`` adds a dict to the result.
Writes OUT_DIR/{top_cam,left_cam,right_cam}.mp4 (20 fps), OUT_DIR/setup.png and OUT_DIR/result.json.

Arm primitives (each a generator, one yield per control step; chain with ``yield from``):

    arm.move(pos, rot=None, width=None, duration=1.0)   # TCP to pos with any orientation (3x3), smooth
    arm.grip(width, duration=0.6)                     # pad opening in m (0 closed, 0.0939 open)
    arm.arc(centre, axis, deg, point=..., follow=1.0)  # swing a held point about a line (hinges, lids)
    arm.pinch_pivot(grasp, hinge, axis, deg, width=..)  # pinch with the pads closing along the hinge, then pivot
    arm.press(point, depth)                           # closed fingers straight down onto a button
    arm.wait(seconds)
    ctx.both(left_gen, right_gen)                     # run two arms' scripts together
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
OPEN_WIDTH = 2 * 0.04695  # m between the pads, fully open
DOWN = np.array([0.0, 0.0, -1.0])


# -- orientation helpers ---------------------------------------------------------------------------------------


def rot_about(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    t = math.radians(deg)
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * (k @ k)


def grip_frame(approach, closing, ref: np.ndarray | None = None) -> np.ndarray:
    """TCP rotation with the approach (gripper z) along ``approach`` and the pads closing along ``closing``
    (made perpendicular). A parallel gripper is the same turned 180 deg about the approach, so the sign nearer
    ``ref`` (default: the stock straight-down grip) is used, which keeps the wrist inside its limits."""
    from yam_arena.demo import grip_rotation

    z = np.asarray(approach, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, dtype=float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    ref = grip_rotation(0.0) if ref is None else ref
    best = None
    for sx in (1.0, -1.0):
        r = np.column_stack([sx * x, np.cross(z, sx * x), z])
        err = np.arccos(np.clip((np.trace(ref.T @ r) - 1) / 2, -1, 1))
        if best is None or err < best[0]:
            best = (err, r)
    return best[1]


def down(yaw_deg: float = 0.0) -> np.ndarray:
    """Straight-down grip, pads closing along world y turned ``yaw_deg`` about the vertical."""
    from yam_arena.demo import grip_rotation

    return grip_rotation(math.radians(yaw_deg))


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def _slerp(r0: np.ndarray, r1: np.ndarray, s: float) -> np.ndarray:
    from yam_arena import ik

    return ik.slerp_matrix(r0, r1, s)


# -- one arm ------------------------------------------------------------------------------------------------------


class Arm:
    """Cartesian moves for one arm, solved every control step with full-orientation IK from the last command."""

    def __init__(self, side: str, base):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.side = side
        self.base = np.asarray(base, dtype=float)
        self.q = np.array(READY, dtype=float)
        self.width = OPEN_WIDTH
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, 1.0]
        self.worst_mm = 0.0
        self.worst_deg = 0.0

    def _solve(self, pos, rot, width) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot, self.width = q, np.asarray(pos, float), rot, width
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, math.degrees(re))
        self.cmd = np.r_[q, float(np.clip(width / OPEN_WIDTH, 0.0, 1.0))]

    def follow(self, fn, duration: float):
        """Generator: ``fn(s) -> (pos, rot, width)`` for s from 0 to 1 over ``duration`` seconds."""
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(*fn(k / n))
            yield

    def move(self, pos, rot=None, *, width=None, duration: float = 1.0):
        p0, r0, w0 = self.pos.copy(), self.rot.copy(), self.width
        p1 = np.asarray(pos, dtype=float)
        r1 = r0 if rot is None else np.asarray(rot, dtype=float)
        w1 = w0 if width is None else float(width)

        def at(s):
            e = smooth(s)
            return p0 + e * (p1 - p0), _slerp(r0, r1, e), w0 + e * (w1 - w0)

        yield from self.follow(at, duration)

    def grip(self, width: float, duration: float = 0.6):
        yield from self.move(self.pos, self.rot, width=width, duration=duration)

    def open(self, duration: float = 0.6):
        yield from self.grip(OPEN_WIDTH, duration)

    def wait(self, seconds: float):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield

    def arc(self, centre, axis, deg: float, *, point=(0.0, 0.0, 0.0), follow: float = 1.0, duration: float = 3.0,
            width=None):
        """Swing the tool point ``point`` (TCP frame) on a circle about the line through ``centre`` along ``axis``
        by ``deg`` (right-handed); the wrist turns ``follow`` x ``deg`` with it (0 keeps its orientation, so a
        pinched link pivots between the pads)."""
        r0 = self.rot.copy()
        off = np.asarray(point, dtype=float)
        c0 = self.pos + r0 @ off
        centre = np.asarray(centre, dtype=float)
        w = self.width if width is None else float(width)

        def at(s):
            a = smooth(s) * deg
            c = centre + rot_about(axis, a) @ (c0 - centre)
            r = rot_about(axis, follow * a) @ r0
            return c - r @ off, r, w

        yield from self.follow(at, duration)

    def pinch_pivot(self, grasp, hinge, axis, deg: float, *, width: float, squeeze: float = 0.0006,
                    approach=DOWN, follow: float = 0.5, contact_depth: float | None = None, hover: float = 0.06,
                    pivot_s: float = 4.0, settle_s: float = 0.4):
        """Pinch a link at ``grasp`` (world point on it, e.g. its mid-plane near the free edge, ``width`` across)
        with the pads closing along the hinge ``axis``, then swing it ``deg`` about the hinge line through
        ``hinge``. The pads close to ``width - squeeze``; the contact centre sits ``contact_depth`` past the TCP.

        Keep the pinch light (``squeeze`` ~0.5 mm, about 1-2 N per pad): the link must turn between the pads by
        (1 - follow) x deg. A hard pinch locks it to the pads and the swing then lifts or drags the parent link.
        ``follow`` 0.5 halves that turn while keeping the wrist in reach; 1.0 is often out of the arm's reach."""
        from yam_arena.constants import PAD_TIP_PAST_TCP

        d = PAD_TIP_PAST_TCP - 0.002 if contact_depth is None else contact_depth
        self.pivot = {"depth": d, "follow": follow}
        app = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
        r = grip_frame(app, axis)
        tcp = np.asarray(grasp, dtype=float) - r @ np.array([0.0, 0.0, d])
        yield from self.move(tcp - app * hover, r, width=OPEN_WIDTH, duration=1.5)
        yield from self.move(tcp, r, duration=1.5)
        yield from self.grip(max(width - squeeze, 0.0), 0.8)
        yield from self.wait(settle_s)
        yield from self.arc(hinge, axis, deg, point=(0.0, 0.0, d), follow=follow, duration=pivot_s)

    def press(self, point, depth: float, *, yaw_deg: float = 0.0, hover: float = 0.06, duration: float = 1.5,
              hold: float = 0.6):
        """Closed fingers straight down: pad tips from ``hover`` above ``point`` to ``depth`` below it, then up."""
        from yam_arena.constants import PAD_TIP_PAST_TCP

        r = down(yaw_deg)
        top = np.asarray(point, dtype=float) + np.array([0.0, 0.0, PAD_TIP_PAST_TCP])
        yield from self.grip(0.0, 0.5)
        yield from self.move(top + [0, 0, hover], r, duration=2.0)
        yield from self.move(top - [0, 0, depth], r, duration=duration)
        yield from self.wait(hold)
        yield from self.move(top + [0, 0, hover], r, duration=1.0)

    def retreat(self, distance: float = 0.06, duration: float = 1.0):
        """Back off along the approach axis."""
        yield from self.move(self.pos - self.rot[:, 2] * distance, self.rot, duration=duration)


# -- what the demo may see --------------------------------------------------------------------------------------


class Seen:
    """Camera-observable scene: every link / object pose and the robot's own gripper state. No joint readouts."""

    def __init__(self, states: dict, tools: dict, specs: dict):
        self.tools = tools
        self._specs = specs
        self._poses = {}
        for name, s in states.items():
            if name in specs:
                self._poses[name] = {k: v.copy() for k, v in s.poses.items()}
            else:
                from linkkit import _pose, quat_to_matrix

                self._poses[name] = {"": _pose(s.pos, quat_to_matrix(s.quat_xyzw))}
        self.bounds = {n: (np.asarray(s.low).copy(), np.asarray(s.high).copy()) for n, s in states.items()}

    def pose(self, obj: str, link: str = "") -> np.ndarray:
        """4x4 world pose (``link`` of a linkage; plain objects: their centre)."""
        return self._poses[obj][link]

    def point(self, obj: str, link: str = "", local=(0.0, 0.0, 0.0)) -> np.ndarray:
        m = self.pose(obj, link)
        return m[:3, :3] @ np.asarray(local, dtype=float) + m[:3, 3]

    def axis(self, obj: str, joint: str) -> tuple[np.ndarray, np.ndarray]:
        """A joint's line as it is now (a point on it, unit direction), from its parent link's pose."""
        spec = self._specs[obj]
        j = spec.joint(joint)
        rel = self.pose(obj, j.parent) @ np.linalg.inv(spec.link(j.parent).design)
        a = rel[:3, :3] @ (np.asarray(j.axis, dtype=float) / np.linalg.norm(j.axis))
        return rel[:3, :3] @ np.asarray(j.anchor, dtype=float) + rel[:3, 3], a

    def angle_deg(self, obj: str, joint: str) -> float:
        """A hinge's angle measured from the two links' poses (what a camera sees), 0 at the design pose."""
        from linkkit import rotation_angle_about

        spec = self._specs[obj]
        j = spec.joint(joint)
        dp, dc = spec.link(j.parent).design, spec.link(j.child).design
        now = np.linalg.inv(self.pose(obj, j.parent)) @ self.pose(obj, j.child)
        design = np.linalg.inv(dp) @ dc
        if j.kind == "prismatic":
            a = dp[:3, :3].T @ np.asarray(j.axis, dtype=float) / np.linalg.norm(j.axis)
            return float((now[:3, 3] - design[:3, 3]) @ a)
        a_parent = dp[:3, :3].T @ np.asarray(j.axis, dtype=float)
        return math.degrees(rotation_angle_about(now[:3, :3] @ design[:3, :3].T, a_parent))


# -- context and runner -----------------------------------------------------------------------------------------


class Ctx:
    def __init__(self, emb, variant: str = ""):
        self.emb = emb
        self.variant = variant
        bases = emb.rig.bases
        self.left = Arm("left", bases["left"])
        self.right = Arm("right", bases["right"])
        self.t = 0.0

    def arm(self, side: str) -> Arm:
        return self.left if side == "left" else self.right

    def observe(self) -> Seen:
        import linkkit
        from yam_arena.scene import object_states

        return Seen(object_states(self.emb._env, self.emb.task.names), linkkit.grippers(self.emb._env),
                    linkkit._SPECS)

    def pinch_pivot(self, side: str, obj: str, joint: str, deg: float, *, local=(0.0, 0.0, 0.0), link=None,
                    width: float, tol_deg: float = 1.0, max_fixes: int = 3, max_fix_deg: float = 15.0, **kw):
        """Closed loop: pinch ``obj``'s link (default the joint's child) at ``local`` (its frame), pivot it ``deg``
        about ``joint`` with :meth:`Arm.pinch_pivot`, then nudge until the hinge angle seen from the link poses is
        within ``tol_deg`` of start + ``deg`` (arm compliance leaves a few degrees otherwise). Stays pinched.
        An error over ``max_fix_deg`` means the grasp was lost: it stops correcting (see ``ctx.pivot_log``)."""
        import linkkit

        arm = self.arm(side)
        seen = self.observe()
        start = seen.angle_deg(obj, joint)
        hinge, axis = seen.axis(obj, joint)
        grasp = seen.point(obj, link or linkkit._SPECS[obj].joint(joint).child, local)
        yield from arm.pinch_pivot(grasp, hinge, axis, deg, width=width, **kw)
        self.pivot_log = [round(self.observe().angle_deg(obj, joint) - start, 2)]
        for _ in range(max_fixes):
            yield from arm.wait(0.3)
            seen = self.observe()
            err = start + deg - seen.angle_deg(obj, joint)
            if abs(err) <= tol_deg or abs(err) > max_fix_deg:
                break
            hinge, axis = seen.axis(obj, joint)
            yield from arm.arc(hinge, axis, err, point=(0.0, 0.0, arm.pivot["depth"]), follow=arm.pivot["follow"],
                               duration=max(0.6, abs(err) / 8.0))
            self.pivot_log.append(round(self.observe().angle_deg(obj, joint) - start, 2))

    @staticmethod
    def both(*gens):
        """Run generators in lockstep (one control step each per yield) until all end."""
        live = [iter(g) for g in gens]
        while live:
            for g in list(live):
                try:
                    next(g)
                except StopIteration:
                    live.remove(g)
            if live:
                yield

    def wait(self, seconds: float):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _save_video(frames: list, path: Path) -> None:
    h, w = frames[0].shape[:2]
    ff = os.environ.get("FFMPEG") or shutil.which("ffmpeg")
    if ff:
        proc = subprocess.Popen([ff, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
                                 f"{w}x{h}", "-r", str(CONTROL_HZ), "-i", "-", "-c:v", "libx264", "-pix_fmt",
                                 "yuv420p", "-movflags", "+faststart", str(path)], stdin=subprocess.PIPE)
        for f in frames:
            proc.stdin.write(np.ascontiguousarray(f[..., :3], dtype=np.uint8).tobytes())
        proc.stdin.close()
        proc.wait()
    else:
        import imageio.v2 as imageio

        imageio.mimwrite(path, [f[..., :3] for f in frames], fps=CONTROL_HZ)


def _reset_check(emb, steps: int = 10) -> dict:
    """Reset again and hold the arms at ready for ``steps``: every linkage should be back at its initial joint
    positions and at rest, and every History cleared."""
    import linkkit
    from inspect_robots import Scene
    from yam_arena.scene import object_states

    emb.reset(Scene(id="linkdemo-reset", instruction="demo"))
    ctx = Ctx(emb)
    for _ in range(steps):
        emb.step(ctx.action())
    states = object_states(emb._env, emb.task.names)
    out = {"history_t": [round(h.t, 3) for h in linkkit._HISTORIES]}
    for name, spec in linkkit._SPECS.items():
        if name in states:
            s = states[name]
            out[name] = {"max_speed_mm_s": round(1000 * s.max_speed(), 3),
                         "q_err": {j.name: round(s.q[j.name] - j.init, 5) for j in spec.moving_joints},
                         "root_err_mm": round(1000 * float(np.linalg.norm(s.pos - spec.link_poses()[spec.links[0].name][:3, 3])), 3)}
    return out


def run(task_path: str, out_dir: str, max_steps: int = 1200, *, variant: str = "", record: bool = True,
        cam_width: int = 320, cam_height: int = 176, hold_s: float = 2.0, device: str = "cuda:0",
        check_reset: bool = False) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    os.environ["DEMO_VARIANT"] = variant
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height, device=device)
    obs = emb.reset(Scene(id="linkdemo", instruction="demo"))
    ctx = Ctx(emb, variant)
    script = emb.task.module.demo_script(ctx)
    frames: dict[str, list] = {}
    info: dict = {}
    steps, ended, success, t0 = 0, None, False, time.time()

    def keep(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    keep(obs)
    while steps < max_steps:
        if ended is None:
            try:
                next(script)
            except StopIteration:
                ended = steps
        res = emb.step(ctx.action())
        steps += 1
        ctx.t = steps / CONTROL_HZ
        info = res.info
        keep(res.observation)
        if steps % 100 == 0:
            print(f"step {steps} t {ctx.t:.1f} score {info.get('score')} {steps / (time.time() - t0):.1f} steps/s",
                  flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    module = emb.task.module
    report = module.demo_report() if hasattr(module, "demo_report") else None
    reset_check = _reset_check(emb) if check_reset else None
    result = {
        "task_score": score, "success": success, "steps": steps, "script_end_step": ended, "variant": variant,
        "ik_worst": {a.side: {"mm": round(a.worst_mm, 2), "deg": round(a.worst_deg, 2)} for a in (ctx.left, ctx.right)},
        "final_info": {k: v for k, v in info.items() if isinstance(v, int | float | bool)},
        "report": report,
        "reset_check": reset_check,
    }
    if record and frames:
        for cam, fr in frames.items():
            _save_video(fr, out / f"{cam}.mp4")
        import imageio.v2 as imageio

        imageio.imwrite(out / "setup.png", frames.get("top_cam", next(iter(frames.values())))[0])
    (out / "result.json").write_text(json.dumps(result, indent=1, default=float))
    print("RESULT", json.dumps({k: result[k] for k in ("task_score", "success", "steps", "ik_worst")}))
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--variant", default=os.environ.get("DEMO_VARIANT", ""))
    ap.add_argument("--no-record", action="store_true", help="skip videos (tiny camera frames, faster)")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--check-reset", action="store_true", help="after the run, reset once more and report the state")
    a = ap.parse_args()
    small = a.no_record
    run(a.task, a.out, a.max_steps, variant=a.variant, record=not a.no_record, device=a.device,
        cam_width=64 if small else 320, cam_height=36 if small else 176, check_reset=a.check_reset)
