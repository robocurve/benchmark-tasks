"""Scripted demo for the tilt-window-sash task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--mode full|still|gentle|oneside|onehand] [--max-steps 1200] [--no-record]

Each arm holds its open gripper like a fork round one end of the sash's top rail, its lower finger under the rail's
edge and its upper finger clear above. The two lower fingers raise the sash together about its pivot bars, from lying
flat up toward the frame, with nothing gripped: the rail turns between the fingers as the sash comes up. Near upright
the fingertips are on the sash's interior face, and they push it home: the ramped latch noses meet the jamb liners,
are cammed in, and click out into the slots once the sash is upright. The hands then draw back, clear of the sash.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.

``--mode gentle`` raises the sash the same way and stops where the noses meet the jamb liners, without the push;
``--mode oneside`` raises it with both hands and pushes home with the left hand only; ``--mode onehand`` does the whole
job with the left hand alone; ``--mode still`` does nothing.
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

    def joints(self, q1, duration: float, grip: float | None = None):
        """Generator: go to joint angles ``q1`` along a straight line in joint space."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.q = q0 + s * (np.asarray(q1, float) - q0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)

    def aim(self, pos, rot):
        """Go straight to a pose this step."""
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


PITCH = math.radians(60)     # the hands' lean from straight down toward the window, kept through the whole lift
HOLD_Y = 0.155               # where each lower finger bears on the sash, from its centre line (m)
BELOW_EDGE = 0.015           # the lower finger's tip this far past the rail's edge / below it on the interior face
TIPS = 0.013                 # PAD_TIP_PAST_TCP: the pad tips are this far past the tool point
OPEN_M = 0.0939              # pad-to-pad opening at grip 1.0
ROLL = {"left": -1, "right": 1}   # which way round each hand holds its closing axis in the vertical plane


def frame(approach, closing) -> np.ndarray:
    """A tool rotation (columns: closing axis, width axis, approach axis)."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def opening(tilt: float) -> float:
    """Pad-to-pad opening (m) for a sash tilt (rad): wide while the rail lies between the fingers, narrower near
    upright so that the upper finger stays over the lower sash's own top rail, clear of the upper sash."""
    d = math.degrees(tilt)
    return 0.075 if d >= 45 else 0.050 if d <= 15 else 0.050 + 0.025 * (d - 15) / 30


def fork(pivot, side: int, tilt: float, w: float, drop: float = 0.0, back: float = 0.0, phi: float = PITCH):
    """Tool point and rotation for a hand holding the top rail's end like a fork, the sash at ``tilt`` (rad from
    upright). Its lower finger's pad passes under the rail's interior top edge while the sash leans more than the hand
    (``tilt >= phi``), its tip 15 mm past the edge; nearer upright that tip bears on the interior face 15 mm below the
    edge. ``drop`` lowers the hand across its pads (m), ``back`` draws it back along its fingers."""
    a = np.array([math.sin(phi), 0.0, -math.cos(phi)])
    c = np.array([math.cos(phi), 0.0, math.sin(phi)])
    ry = _rot_y(-tilt)
    if tilt >= phi:
        edge = pivot + ry @ np.array([-0.0175, side * HOLD_Y, 0.275])
        tcp = edge + (BELOW_EDGE - TIPS) * a + (w / 2) * c
    else:
        spot = pivot + ry @ np.array([-0.0175, side * HOLD_Y, 0.275 - BELOW_EDGE])
        tcp = spot - TIPS * a + (w / 2) * c
    name = "left" if side > 0 else "right"
    return tcp - drop * c - back * a, frame(a, ROLL[name] * c)


def script(task, ctx, mode: str = "full"):
    """The demo as a generator: one yield per control step. Open loop: it uses only the published layout."""
    T = task
    L, R = ctx.left, ctx.right
    pivot = np.array(T.PIVOT)
    arms = ((L, 1), (R, -1)) if mode != "onehand" else ((L, 1),)
    if mode == "still":
        yield from wait(2.0)
        return

    def go(arm, side, tilt, duration, drop=0.0, back=0.0, w=None):
        w = opening(tilt) if w is None else w
        p, r = fork(pivot, side, tilt, w, drop, back)
        yield from arm.move(p, r, duration, grip=w / OPEN_M)

    t0 = T.START_TILT
    if not START_Q:
        START_Q.update(START_QS[T.LOT])
    ctx.sash_tilt = t0   # where the script expects the sash to be (its own bookkeeping, for the offline dry run)
    # 1. Both hands swing to a point behind the rail's ends, open, then slide their lower fingers under the rail's edge.
    yield from both(*(arm.joints(START_Q[arm.name], 3.0, grip=opening(t0) / OPEN_M) for arm, _ in arms))
    yield from both(*(go(arm, s, t0, 1.5, drop=0.006, back=0.06) for arm, s in arms))
    yield from both(*(go(arm, s, t0, 1.5, drop=0.006) for arm, s in arms))
    yield from wait(0.3)

    # 2. Raise the sash together about its pivot bars, 3 degrees a step, to where the latch noses are about to meet
    #    the jamb liners. Nothing is gripped: the rail lies on the lower fingers and turns between the two.
    stop = math.radians(LIFT_TO)
    n = int(round(math.degrees(t0 - stop) / 3))
    for k in range(1, n + 1):
        t = t0 + (stop - t0) * k / n
        ctx.sash_tilt = t
        yield from both(*(go(arm, s, t, 0.25) for arm, s in arms))
    yield from wait(0.3)

    if mode == "gentle":
        # Stop where the noses meet the liners (10 degrees), hold, and lay the sash back down on its block.
        yield from both(*(go(arm, s, math.radians(9.5), 0.6) for arm, s in arms))
        yield from wait(2.0)
        for k in range(1, n + 1):
            t = stop + (t0 - stop) * k / n
            ctx.sash_tilt = t
            yield from both(*(go(arm, s, t, 0.25) for arm, s in arms))
        yield from both(*(go(arm, s, t0, 1.0, drop=0.006) for arm, s in arms))
        yield from both(*(go(arm, s, t0, 1.2, drop=0.006, back=0.08) for arm, s in arms))
        yield from wait(1.0)
        return

    # 3. Push it home: the fingertips on the interior face are sent past upright (the arms are compliant and lag),
    #    which cams both latch noses in against their springs; they click out into the slots once the sash is upright.
    past = math.radians(PUSH_PAST)
    pushers = arms if mode != "oneside" else arms[:1]
    n_push = max(2, round(PUSH_TIME * CONTROL_HZ))
    for k in range(1, n_push + 1):
        t = stop + (past - stop) * k / n_push
        ctx.sash_tilt = max(t, 0.0)
        for arm, s in pushers:
            p, r = fork(pivot, s, t, 0.050)
            arm.grip = 0.050 / OPEN_M
            arm.aim(p, r)
        yield
    yield from wait(1.0)

    # 4. Ease off to the upright sash and draw the hands back and up, more than 3 cm clear of every part of it. The
    #    latched sash stands by itself; the full score needs it to stay so for 1 s with both hands away.
    yield from both(*(go(arm, s, 0.0, 0.5, w=0.050) for arm, s in arms))
    yield from both(*(go(arm, s, 0.0, 1.2, w=0.050, back=0.09) for arm, s in arms))

    def away(arm, side):   # then down and back toward the robot, well away from the window
        p, r = fork(pivot, side, 0.0, 0.050, back=0.09)
        yield from arm.move(p + np.array([AWAY[0], side * AWAY[1], AWAY[2]]), r, 1.2, grip=0.050 / OPEN_M)

    yield from both(*(away(arm, s) for arm, s in arms))
    yield from wait(1.5)


PUSH_PAST = -8.0   # degrees past upright that the push is sent to (the arms are compliant and lag)
PUSH_TIME = 1.5    # seconds the push takes
LIFT_TO = 12.0     # degrees from upright where the lift ends and the push begins
AWAY = (-0.10, 0.03, -0.12)   # the hands' last move from the drawn-back pose: toward the robot, outward, down (m)
# Joint angles of each hand's way in: the first fork pose drawn 10 cm back along the fingers with the hand leaning only
# 30 degrees, which puts it above and behind the rail's end (solved offline from the layout). The first move goes there
# in joint space, so that each wrist takes the turn that the lift needs while the hand is still clear of the sash.
WAY_IN = dict(drop=0.006, back=0.10, phi=math.radians(30))
START_Q: dict = {}   # filled from START_QS for the layout in use when the script starts
START_QS = {"A": {"left": [-1.3196, 1.0729, 0.9532, -1.3085, -0.5056, 0.2881],
                  "right": [1.3196, 1.0729, 0.9532, -1.3085, 0.5056, -0.2881]},
            "B": {"left": [-1.3551, 0.9904, 0.9162, -1.3736, -0.5103, 0.2478],
                  "right": [1.4047, 1.1575, 1.0002, -1.3184, 0.5157, -0.1912]}}


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.sash_tilt = None

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
        cam_height: int = 176, hold_s: float = 4.0, mode: str = "full") -> dict:
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
    gen = script(task, ctx, mode)
    trace, first = [], {}
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
        if steps % 5 == 0:   # a record of the run (read from the simulator for the log only; the script never sees it)
            sash = ctx.states()["sash"]
            hands = {}
            try:   # where each hand really is, from the robot's own joint angles (log only)
                from yam_arena import ik as _ik
                base = emb._env.unwrapped
                for nm, arm in (("left", ctx.left), ("right", ctx.right)):
                    rob = base.scene[f"{nm}_robot"]
                    qa = rob.data.joint_pos[0].detach().cpu().numpy()[:6]
                    pa, _ = _ik.tcp_pose(qa, arm.base)
                    hands[nm] = [round(float(v), 4) for v in pa - arm.pos] + [round(float(arm.pos[0]), 4), round(float(arm.pos[2] - task.TOP), 4)]
            except Exception as e:   # noqa: BLE001
                hands = {"err": str(e)[:80]}
            row = {"step": steps, "score": float(info.get("score", 0.0) or 0.0), "hands": hands,
                   "tilt_deg": round(task.tilt_deg(sash), 2), "pivot_mm": round(1000 * task.pivot_offset(sash), 1),
                   "latch_mm": {k: round(1000 * v, 2) for k, v in sash.sliders.items()},
                   "clear_mm": None if task._STATE.get("near") is None else round(1000 * task._STATE["near"], 1)}
            trace.append(row)
            for level in (0.25, 0.5, 0.75, 1.0):
                if row["score"] >= level:
                    first.setdefault(str(level), steps)
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
    result = {"lot": task.LOT, "mode": mode, "task_score": score, "success": success, "steps": steps, "script_steps": ended,
              "first_step_at_score": first, "end": trace[-1] if trace else None,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)},
              "trace": trace}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=["full", "still", "gentle", "oneside", "onehand"])
    ap.add_argument("--cam", default="320x176")
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam.split("x"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=cw, cam_height=ch, mode=a.mode)
