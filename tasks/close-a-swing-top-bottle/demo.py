"""Demo for the swing-top bottle task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2400] [--no-record] [--mode close|still|stopper|lever]

1. The left hand comes in level from the arms' side and holds the bottle's body, below the shoulder.
2. The right hand comes straight down on the stopper hanging beside the neck, takes it between its fingers, swings it
   up over the top onto the mouth (the stopper turning in its fingers about its hinge), and lets go.
3. The right hand takes the lever's paddle between its fingers, swings the lever back over the top and down the near
   side of the neck, through its centre, until it snaps shut, and lets go.
4. The left hand lets go of the bottle; both hands rise clear.

``--mode still`` keeps both arms still (a scene check); ``stopper`` stops after step 2 (the stopper on, not sealed);
``lever`` skips step 2 and shuts the lever on nothing (it does not seal); ``nohold`` does steps 2 and 3 without the
left hand holding the bottle.

Every target comes from the layout constants in the task file and the bottle's geometry in bottlekit; the only
feedback is the measured position of each hand (to correct an arm's sag before a placement). Writes one MP4 per
camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0
TIP = 0.013                        # the pads' tips reach this far past the tool point



def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])




def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.worst_deg = 0.0

    def _solve(self, pos, rot) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, duration: float, rot=None, grip: float | None = None):
        """Generator: go in a straight line to ``pos`` (world, tool point) and ``rot``; ``grip`` 1 open .. 0 closed."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        r1 = r0 if rot is None else rot
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, r1, s))
            yield

    def follow(self, path, duration: float, rot=None):
        """Generator: follow ``path(s)`` (world tool point for s from 0 to 1) at a smooth pace."""
        from yam_arena import ik

        r = self.rot if rot is None else rot
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(path(ik.smoothstep(k / n)), r)
            yield

    def joints_for(self, pos, rot, seed=None) -> np.ndarray:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(self.q if seed is None else seed, float),
                             base_pos=self.base, iters=400)
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        return q

    def set_grip(self, grip: float, duration: float = 0.5):
        yield from self.move(self.pos, duration, grip=grip)

    def move_joints(self, q_goal, duration: float, grip: float | None = None):
        """Generator: swing to joint angles ``q_goal``, every joint moving together."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (np.asarray(q_goal, float) - q0), g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def chain(*gens):
    for g in gens:
        yield from g


def together(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield



def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of a few seeds (the hand's own, and its wrist turned either way)."""
    from yam_arena import ik

    best = None
    for q6 in (None, -1.57, 1.57):
        seed = hand.q.copy()
        if q6 is not None:
            seed[5] = q6
        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=400)
        if best is None or pe < best[1]:
            best = (q, pe)
    return best[0]



TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)
DOWN = frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))        # straight down, fingers closing along x (the hinge's axis)
HOLD_AP = np.array([1.0, 0.0, -1.0]) / math.sqrt(2.0)   # the left hand comes in 45 degrees down, away from the arms,
LEVEL = frame(HOLD_AP, (0.0, 1.0, 0.0))                  # fingers closing along y round the body
HOLD_Z = 0.035                           # the left hand holds the body this far up
HOLD_IN = 0.015                          # its tool point this far short of the bottle's axis (along its approach)
HOLD_OPEN, HOLD_SHUT = 0.090, 0.030      # finger gap round the 56 mm body / commanded on it
PINCH_OPEN, PINCH_SHUT = 0.045, 0.006    # round the 23 mm stopper or the 18 mm paddle / commanded on it
STOPPER_OUT = 0.007                      # pinch the stopper this far out from its middle (fingertips clear of the lip)
PADDLE_OUT = 0.008                       # pinch the paddle this far out from its middle (fingers pass over the stopper)
SETTLE_MAX = 0.015                       # the most a placement is aimed off to make up for an arm's lag


def _arc(K, base, fn, a0, a1):
    """A path for the tool point: the stopper's or paddle's centre as its joint goes from a0 to a1."""
    quat = (0.0, 0.0, 0.0, 1.0)
    return lambda s: fn(base, quat, a0 + s * (a1 - a0))


def settle(hand, target, rounds: int = 3):
    """Get the tool point to ``target`` despite an arm's lag or sag: look where it is, and aim off by the miss."""
    target = np.asarray(target, float)
    aim = target.copy()
    for _ in range(rounds):
        aim = aim + (target - TOOL[hand.name])
        off = aim - target
        if np.linalg.norm(off) > SETTLE_MAX:                                # blocked: don't push harder and harder
            aim = target + off * SETTLE_MAX / np.linalg.norm(off)
        yield from chain(hand.move(aim, 0.4), wait(0.3))


def hold(T, left):
    b = T.bottle_start()
    at = b + [0.0, 0.0, HOLD_Z] - HOLD_AP * HOLD_IN
    clear = np.array([b[0] - 0.13, b[1] + 0.10, b[2] + 0.16])             # behind the bottle, clear of its top
    yield from left.move_joints(best_ik(left, clear, LEVEL), 2.0, grip=gap(HOLD_OPEN))
    yield from left.move(at - HOLD_AP * 0.08, 1.5)
    yield from chain(left.move(at, 1.2), wait(0.3))
    yield from chain(left.set_grip(gap(HOLD_SHUT), 0.8), wait(0.3))


def release_hold(left):
    yield from left.set_grip(gap(HOLD_OPEN), 0.5)
    yield from left.move(left.pos - HOLD_AP * 0.08, 1.0)


def stopper_on(T, right):
    import bottlekit as K

    b = T.bottle_start()
    path = _arc(K, b, lambda p, q, a: K.stopper_point(p, q, a, STOPPER_OUT), K.STOPPER_OPEN, -math.radians(4.0))
    start = path(0.0)
    yield from right.move_joints(best_ik(right, start + [0.0, 0.0, 0.06], DOWN), 2.5, grip=gap(PINCH_OPEN))
    yield from chain(right.move(start, 1.2), wait(0.3))
    yield from chain(right.set_grip(gap(PINCH_SHUT), 0.6), wait(0.3))
    yield from right.follow(path, 4.5)                                     # up and over onto the mouth
    yield from settle(right, path(1.0))
    yield from right.set_grip(gap(PINCH_OPEN), 0.4)
    yield from right.move(right.pos + [0.0, 0.0, 0.05], 0.8)


def lever_shut(T, right):
    import bottlekit as K

    b = T.bottle_start()
    path = _arc(K, b, lambda p, q, a: K.paddle_point(p, q, a, K.LEVER_L + PADDLE_OUT),
                K.LEVER_OPEN - math.radians(3.0), math.radians(4.0))
    start = path(0.0)
    yield from right.move(start + [0.0, 0.0, 0.03], 1.5, rot=DOWN, grip=gap(PINCH_OPEN))
    yield from chain(right.move(start, 1.2), wait(0.3))
    yield from chain(right.set_grip(gap(PINCH_SHUT), 0.6), wait(0.3))
    yield from right.follow(path, 5.0)                                     # back over the top and down: it snaps shut
    yield from settle(right, path(1.0))
    yield from right.set_grip(gap(PINCH_OPEN), 0.4)
    yield from right.move(right.pos + [0.0, -0.03, 0.05], 0.8)


def script(T, left, right, mode: str = "close"):
    yield from wait(0.3)
    if mode == "still":
        yield from wait(8.0)
        return
    if mode != "nohold":
        yield from hold(T, left)
    if mode != "lever":
        yield from stopper_on(T, right)
    if mode != "stopper":
        yield from lever_shut(T, right)
    if mode != "nohold":
        yield from release_hold(left)
    yield from wait(3.5)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "close", every: int = 5) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=320, cam_height=176,
                               headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="bottle-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode)
    frames: dict[str, list] = {}
    trace = []

    def snap(o) -> None:
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, [T.NAME])


    def hands() -> dict:
        from bottlekit import _np, finger_points

        out = {}
        for n, p in zip(("left", "right"), finger_points(emb._env)):
            robot = emb._env.unwrapped.scene[f"{n}_robot"]
            q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out[n] = {"tool_cm": [round(100 * float(v), 2) for v in p], "q6": round(float(q[names.index("joint6")]), 3),
                      "gap_mm": round(1000 * sum(abs(float(q[names.index(j)])) for j in ("joint7", "joint8")), 1)}
        return out

    snap(obs)
    start = T.report(states())
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        from bottlekit import finger_points
        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))
        info = res.info
        snap(res.observation)
        if steps % every == 0:
            r = T.report(states())
            trace.append({"step": steps, "score": info.get("score"), "task": r, "hands": hands(), "want_q6": round(float(right.q[5]), 3),
                          "want_cm": {h.name: [round(100 * float(v), 2) for v in h.pos] for h in (left, right)}})
        if steps % (10 if mode.endswith("only") else 40) == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {t['score']} {t['task']} {t['hands']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": T.report(states()), "trace": trace}
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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="close", choices=("close", "still", "stopper", "lever", "nohold"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every)
