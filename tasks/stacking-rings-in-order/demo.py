"""Scripted full-score demo for the ring-stacking task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--cam 640x352]

The rings go on largest first, each arm taking the rings on its own side of the table: the left arm the red (largest),
yellow and blue (smallest) rings, the right arm the orange and green. Each ring is taken by its wall (one finger in the
hole, one outside: four of the five are wider than the hand opens), carried over the post and let go just above its
top, so it drops down the tapered post onto the ring below. Both arms then go back to their ready poses and the toy stands on its own.
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


# The demo runs open loop: it reads no simulator state. Where each ring sits in the hand after it is lifted (the fingers
# close on it and it shifts a little) is measured once with DEMO_CALIBRATE=1 (which reads the simulator's poses at that
# moment and prints them) and fixed here. The layout is fixed (object_jitter=0) and the simulation deterministic.
CAL = {
    # ring index: (dx, dy, dz) of the ring's centre from where the plan puts it, once lifted (m)
    "ring_in_hand": {0: (-0.00565, -0.00331, -0.00179), 1: (0.00131, 0.00073, -0.0005), 2: (-0.00454, -0.00286, -0.00049),
                     3: (0.00216, 0.00118, -0.00221), 4: (-0.00011, -0.00342, -0.00085)},
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
_MEASURED: dict = {}

ARM_OF = {0: "left", 1: "right", 2: "left", 3: "right", 4: "left"}
WALL_GRIP = (0, 1, 2, 3, 4)    # taken by the wall, one finger in the hole (four of the five are wider than the hand opens)
FINGER_T = 0.0064              # a finger's thickness at its tip (closing direction)
INNER_DEPTH = 0.004            # the inner finger's pad stands this far inside the ring's hole when it closes
PICK_TCP_UP = {"wall": 0.028, "outside": 0.016}   # TCP above the table when taking a ring (pad tips 17 / 3 mm up)
DROP_GAP = 0.015               # wall-gripped rings are let go with their bottom this far above the post's top
LOWER_GAP = 0.010              # outside-gripped rings are let go with their bottom this far above their seat
CARRY_UP = 0.245               # carried over the toy with the TCP this high above the table
PITCH = -30.0                  # the grip is pitched 30 degrees, fingers leaning forward (the arms reach the post's
                               # height above the toy so)


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    T = task
    top = T.TOP
    ez = np.array([0.0, 0.0, 1.0])
    base_top = top + T.BASE_H
    axis = np.array([T.BASE_XY[0], T.BASE_XY[1], 0.0])

    def measure(key, value):
        if CALIBRATE:
            _MEASURED[key] = [round(float(v), 5) for v in value]
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)

    def plan(i):
        centre = np.array([*T.RING_START[i], top + T.RING_H / 2])
        rot = _rot_axis((0, 1, 0), math.radians(PITCH)) @ grip_rotation(0.0)
        if i in WALL_GRIP:
            # by the wall on the side toward the middle of the table, fingers closing across it (along y)
            rm = (T.RING_OD[i] + T.RING_ID[i]) / 4
            w = (T.RING_OD[i] - T.RING_ID[i]) / 2
            grip_open = (w + 2 * INNER_DEPTH + 0.004) / 0.094
            tcp = centre - np.array([0.0, math.copysign(rm, centre[1]), 0.0])
            tcp[2] = top + PICK_TCP_UP["wall"]
        else:
            grip_open = 1.0
            tcp = centre.copy()
            tcp[2] = top + PICK_TCP_UP["outside"]
        return rot, grip_open, tcp, centre - tcp        # the last: the ring's centre seen from the TCP

    def pick(i, arm):
        rot, grip_open, tcp, rel = plan(i)
        yield from arm.move(tcp + 0.08 * ez, rot, 2.5, grip=grip_open, label=f"over ring {i}")
        yield from arm.move(tcp, rot, 1.2, label=f"down to ring {i}")
        yield from arm.set_grip(0.0, 0.6)
        yield from wait(0.2)

    def place(i, arm):
        rot, grip_open, tcp, rel = plan(i)
        lift = tcp.copy()
        lift[2] = top + CARRY_UP
        yield from arm.move(lift, rot, 1.5, label=f"lift ring {i}")
        if CALIBRATE:
            yield from wait(0.5)
            s = ctx.states()[f"ring_{i}"]
            measure(f"ring_{i}", s.pos - (lift + rel))
        rel_held = rel + np.array(CAL["ring_in_hand"][i])
        if i in WALL_GRIP:      # let go just above the post's top: it drops down the post
            ring_z = base_top + T.POST_TOP + DROP_GAP + T.RING_H / 2
        else:                   # lowered over the post, let go just above the ring below
            ring_z = base_top + T.seat(i) + LOWER_GAP + T.RING_H / 2
        over = axis - rel_held
        over[2] = top + CARRY_UP
        yield from arm.move(over, rot, 2.5, label=f"carry ring {i} over the post")
        at = over.copy()
        at[2] = ring_z - rel_held[2]
        yield from arm.move(at, rot, 1.2 if i in WALL_GRIP else 2.0, label=f"lower ring {i}")
        yield from wait(0.3)
        yield from arm.set_grip(grip_open, 0.5)
        yield from wait(0.3)
        up = at.copy()
        up[2] = max(at[2], base_top + T.POST_TOP + 0.04)
        if up[2] > at[2] + 1e-6:
            yield from arm.move(up, rot, 1.0, label=f"up from ring {i}")

    back = None
    for i in range(T.N_RINGS):
        arm = ctx.left if ARM_OF[i] == "left" else ctx.right
        if back is None:
            yield from pick(i, arm)
        else:
            yield from both(back, pick(i, arm))
        yield from place(i, arm)
        back = arm.move(*arm.home, 2.0, grip=1.0, label="back")
    yield from back


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


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 640,
        cam_height: int = 352, hold_s: float = 6.0) -> dict:
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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam", default="640x352", help="camera width x height")
    a = ap.parse_args()
    w, h = (int(v) for v in a.cam.split("x"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=w, cam_height=h)
