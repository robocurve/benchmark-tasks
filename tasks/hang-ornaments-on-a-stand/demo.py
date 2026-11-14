"""Demo for the ornament task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1200] [--no-record] [--mode hang|still]

For each bauble (the left hand takes the one in the left box, the right hand the one in the right box):

1. The hand comes straight down on the bauble's metal cap and takes it between its fingers.
2. It lifts the bauble out of its box, carries it over its side's branch near the tip, its hook's crook curling
   over the branch, and lowers it until the crook rests on the branch.
3. It lets go, and rises clear. The bauble hangs from the branch by its hook.

``--mode still`` keeps both arms still (a scene check).

Open loop: every pose comes from the layout constants in the task file. Object states are read only to report
where things ended up. Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json.
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



CAP_IN = 0.016                           # the pads overlap the cap by this much (their tips clear the glass)
CAP_OPEN, CAP_SHUT = 0.045, 0.010         # finger gap around the 16 mm cap / commanded on it (a firm, not crushing, pinch)
LIFT = 0.070                             # up out of the box: the ball clear of the walls and the hook of the hand
ABOVE = 0.020                            # the crook comes this far above its resting place before being lowered
TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)
DOWN_L = frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))     # straight down, fingers closing across the hook's plane (y),
DOWN_R = DOWN_L                                          # so neither finger comes down on the crook


def hang(T, hand, rot, i: int, branch: str):
    import ornkit as K

    cap = np.array([0.0, 0.0, K.CAP_Z[1] + TIP - CAP_IN])                   # tool point above the ball's centre
    take = T.bauble_start(i) + cap
    put = T.hang_point(branch) + cap
    yield from hand.move_joints(best_ik(hand, take + [0.0, 0.0, 0.05], rot), 2.5, grip=gap(CAP_OPEN))
    yield from chain(hand.move(take, 1.2), wait(0.5))                      # settle before closing
    yield from chain(hand.set_grip(gap(CAP_SHUT), 1.0), wait(0.3))         # the cap between the fingers
    yield from hand.move(take + [0.0, 0.0, LIFT], 1.5)
    yield from hand.move(put + [0.0, 0.0, ABOVE + 0.03], 3.5)
    yield from chain(hand.move(put + [0.0, 0.0, ABOVE], 1.0), wait(1.0))   # let any swing die out
    aim = put.copy()
    for _ in range(2):                    # an arm holding a load out sags short of its command: look, and correct for it
        err = (put + [0.0, 0.0, ABOVE]) - TOOL[hand.name]
        aim = aim + [err[0], err[1], 0.0]
        yield from chain(hand.move(aim + [0.0, 0.0, ABOVE], 0.6), wait(0.6))
    put = aim
    yield from chain(hand.move(put + [0.0, 0.0, -0.002], 1.2), wait(0.4))  # the crook down onto the branch
    yield from chain(hand.set_grip(gap(CAP_OPEN), 0.4), wait(0.4))         # let go: it hangs
    yield from hand.move(hand.pos + [0.0, 0.0, 0.05], 0.8)


def script(T, left, right, mode: str = "hang"):
    yield from wait(0.3)
    if mode == "still":
        yield from wait(8.0)
        return
    yield from hang(T, left, DOWN_L, 0, "left")
    yield from hang(T, right, DOWN_R, 1, "right")
    yield from wait(6.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "hang", every: int = 5) -> dict:
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
    obs = emb.reset(Scene(id="ornament-demo", instruction=T.INSTRUCTION))
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

        return object_states(emb._env, list(T.NAMES))


    def hands() -> dict:
        from ornkit import _np, finger_points

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
        from ornkit import finger_points
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
    ap.add_argument("--mode", default="hang", choices=("hang", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every)
