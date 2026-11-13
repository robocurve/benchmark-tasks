"""Demo for the syringe-needle task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2400] [--no-record] [--mode pull|onehand]

For each syringe, nearest the arms first:

1. The left hand comes straight down over the syringe's barrel and takes it between its fingers, in the open
   space between the stand's two cradles, so the syringe cannot turn.
2. The right hand, level and pointing left along the syringe, takes the needle's cap between its fingers.
3. It turns its wrist 203 degrees, anticlockwise seen from the needle's tip: the cap is keyed to the
   hub, so the needle unscrews from the Luer-lock collar. It then lets go, winds its wrist back 110 degrees, takes the
   cap again and turns it 110 degrees more, since the cap slips a little in the fingers.
4. It draws the needle 2 cm straight off the collar, still capped, carries it over the sharps tray beyond the tips,
   lowers it and lets it drop in.

``--mode pull`` is a control: the right hand pulls the first cap straight off without turning it. ``--mode onehand``
is another: the same turn with the left hand kept away, so nothing holds the syringe.

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

CAP_AT = 0.084                     # where the right hand takes the cap: this far along the axis from the flange
HOLD_AT = 0.028                    # where the left hand holds the barrel: this far along the axis from the flange
HOLD_OPEN, HOLD_GRIP = 0.025, 0.002   # finger gap around the 10.5 mm barrel / commanded on it
OPEN, GRIP = 0.030, 0.002          # finger gap around the 8 mm cap / commanded on it
TWIST = math.pi + 0.40             # 203 degrees: half a turn and some, for the cap slipping a little in the fingers
PULL = 0.020                       # how far the needle is drawn off
DROP_Y = -0.065                    # the hand over the sharps tray (the needle's middle over the tray's middle) ...
DROP_Z = 0.075                     # ... this high above the table when it lets go
REGRIP = math.radians(110.0)       # the second turn


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


DOWN_L = frame((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0))     # left hand straight down, fingers closing along the barrel's x
LEVEL_UP = frame((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))     # right hand level, pointing left, fingers one above the other
LEVEL_DOWN = frame((0.0, 1.0, 0.0), (0.0, 0.0, -1.0))


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


def unscrew_plan(T, right, i: int):
    """The right hand's start rotation and wrist turn for syringe i: the one whose joint-6 turn unscrews the needle
    (turns the tool about world -y, the needle's direction, anticlockwise seen from its tip) within the joint's range."""
    from yam_arena import ik

    pinch = T.on_axis(i, CAP_AT)
    for rot in (LEVEL_UP, LEVEL_DOWN):
        q = best_ik(right, pinch, rot)
        r0 = ik.tcp_pose(q, right.base)[1]
        q1 = q.copy()
        q1[5] += 0.1
        r1 = ik.tcp_pose(q1, right.base)[1]
        rel = r1 @ r0.T
        axis = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
        sign = 1.0 if axis @ np.array([0.0, -1.0, 0.0]) > 0 else -1.0     # joint 6 direction that unscrews
        end = q[5] + sign * TWIST
        if ik.LOWER[5] + 0.05 <= end <= ik.UPPER[5] - 0.05:
            return rot, sign
    raise RuntimeError("no wrist turn of half a turn fits joint 6's range")


def one(T, left, right, i: int, mode: str):
    rot, sign = unscrew_plan(T, right, i)
    pinch = T.on_axis(i, CAP_AT)
    hold = T.on_axis(i, HOLD_AT)                                           # pads at the barrel's sides
    if mode != "onehand":
        yield from left.move(hold + [0.0, 0.0, 0.04], 1.2, rot=DOWN_L, grip=gap(HOLD_OPEN))
        yield from left.move(hold, 0.6)
        yield from chain(left.set_grip(gap(HOLD_GRIP), 0.4), wait(0.2))   # hold the barrel between the cradles
    yield from right.move(pinch + [0.0, -0.10, 0.0], 1.2, rot=rot, grip=gap(OPEN))
    yield from right.move(pinch + [0.0, -0.04, 0.0], 0.8)
    yield from right.move(pinch, 0.8)
    yield from chain(right.set_grip(gap(GRIP), 0.5), wait(0.2))           # the cap between the fingers
    if mode == "pull":
        yield from right.move(pinch + [0.0, -PULL, 0.0], 1.0)              # straight off, no turn
        return
    q = right.q.copy()
    q[5] += sign * TWIST
    yield from chain(right.move_joints(q, 3.5), wait(0.2))                # unscrew it
    # The cap slips a little in the fingers, so let go, wind the wrist back and give it a second, shorter turn.
    yield from right.set_grip(gap(OPEN), 0.3)
    q[5] -= sign * REGRIP
    yield from right.move_joints(q, 0.8)
    yield from chain(right.set_grip(gap(GRIP), 0.4), wait(0.1))
    q[5] += sign * REGRIP
    yield from chain(right.move_joints(q, 1.2), wait(0.2))
    yield from right.move(right.pos + [0.0, -PULL, 0.0], 0.8)              # draw it off the collar
    over = np.array([right.pos[0], DROP_Y, T.TOP + DROP_Z])
    yield from right.move(np.array([right.pos[0], DROP_Y, right.pos[2]]), 1.2)   # carry it over the sharps tray
    yield from right.move(over, 0.6)
    yield from chain(right.set_grip(1.0, 0.4), wait(0.4))                # let it go into the tray
    yield from right.move(right.pos + [0.0, 0.0, 0.03], 0.6)               # up, clear of the needle
    q = right.q.copy()
    q[5] -= sign * TWIST
    yield from right.move_joints(q, 1.2)                                  # turn the wrist back for the next one
    if mode != "onehand":
        yield from left.set_grip(gap(HOLD_OPEN), 0.3)
        yield from left.move(hold + [0.0, 0.0, 0.04], 0.6)


def script(T, left, right, mode: str = "remove"):
    yield from wait(0.3)
    first = T.on_axis(0, CAP_AT) + [0.0, -0.08, 0.02]
    rot, _ = unscrew_plan(T, right, 0)
    yield from right.move_joints(best_ik(right, first, rot), 2.5, grip=gap(OPEN))
    if mode != "onehand":
        top = T.on_axis(0, HOLD_AT) + [0.0, 0.0, 0.045]
        yield from left.move_joints(left.joints_for(top, DOWN_L), 2.0, grip=gap(HOLD_OPEN))
    for i in range(T.N if mode == "remove" else 1):
        yield from one(T, left, right, i, mode)
    yield from right.move(right.pos + [0.0, -0.04, 0.04], 1.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "remove", every: int = 5) -> dict:
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
    obs = emb.reset(Scene(id="syringe-demo", instruction=T.INSTRUCTION))
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

        return object_states(emb._env, T.NAMES)

    def hands() -> dict:
        from syringekit import finger_points

        from syringekit import _np

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
        info = res.info
        snap(res.observation)
        if steps % every == 0:
            r = T.report(states())
            trace.append({"step": steps, "score": info.get("score"), "syringes": r, "hands": hands(), "want_q6": round(float(right.q[5]), 3),
                          "want_cm": {h.name: [round(100 * float(v), 2) for v in h.pos] for h in (left, right)}})
        if steps % 40 == 0 and trace:
            t = trace[-1]
            brief = {n: (v["unscrewed_deg"], v["lift_mm"], v["cap_mm"], v["moved_cm"], int(v["off"]), int(v["in_tray"])) for n, v in t["syringes"].items()}
            print(f"step {steps} score {t['score']} {brief}", flush=True)
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
    ap.add_argument("--mode", default="remove", choices=("remove", "pull", "onehand"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every)
