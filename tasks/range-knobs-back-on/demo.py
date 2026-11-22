"""Scripted full-score demo for the range-knobs task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1600] [--no-record] [--cam 640x352]
    DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record

Each arm puts back the knob on its side: the left arm the left knob on the left stem, then the right arm the right
knob on the right stem (it picks its knob up while the left arm works, and waits for it to draw back). For each knob:
take it by its grip fin from above, carry it over its stem and set it on the stem's tip turned 30 degrees off OFF,
pressing lightly (well under the 10 N that unlocks the valve); turn it slowly toward OFF until its flat lines up
with the stem's and it drops onto the flat (and stops: the stem is locked at OFF); let go, close the fingers and push
the knob home with the fingertips on top of its fin, straight down without turning; draw back.
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

    def turn(self, pos, yaw0: float, yaw1: float, duration: float, label: str = ""):
        """Generator: hold ``pos`` and turn the straight-down grip from ``yaw0`` to ``yaw1`` (rad) at an even rate."""
        from yam_arena.demo import grip_rotation

        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            r = grip_rotation(yaw0 + (yaw1 - yaw0) * k / n)
            if self.solve:
                self._solve(pos, r)
            else:
                self.pos, self.rot = np.asarray(pos, float), r
            yield
        self.ends.append((np.asarray(pos, float).copy(), self.rot.copy(), label))

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


# The demo runs open loop: it reads no simulator state. Where each knob sits in the hand after it is picked up (its
# mouth's offset from the TCP in the gripper's frame, and its turn against the gripper's), and how far below where it
# stops each press is commanded (the arms are compliant, so a press pushes only as hard as it is commanded past where
# the part stops), are measured once with DEMO_CALIBRATE=1 (which reads the simulator's poses at those moments and
# prints them) and fixed here. The layout is fixed (object_jitter=0) and the simulation deterministic.
CAL = {
    # (x, y, z of the knob's mouth in the gripper frame (m), its turn against the gripper (deg)). Calibration reads
    # about (-1.8, -4.1, 36.0) mm and 1 deg here, but that is mostly the arm's own lag behind its commanded TCP, which
    # the move over the stem repeats; using it moved the knob off the stem, so the plan keeps the nominal grasp.
    "knob_in_hand_0": (0.0, 0.0, 0.035, 0.0),
    "knob_in_hand_1": (0.0, 0.0, 0.035, 0.0),
    "light_press": 0.008,     # while turning, the knob is commanded this far below the stem's tip (light: < 6 N)
    "push_past": 0.006,       # pushing home, the fingertips are commanded this far below the fin's top when home
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
if os.environ.get("DEMO_PRESS_MM"):   # a control run: turn the knobs while pressing this hard (mm past the stop)
    CAL["light_press"] = float(os.environ["DEMO_PRESS_MM"]) / 1000
_MEASURED: dict = {}

TIPS = 0.013          # the TCP sits this far short of the pad tips
PAD_UP = 0.022        # grasping, the pad tips are this far up the knob (the fin runs from 17 to 33 mm)
HUNT_FROM = 30.0      # the knob goes on the tip turned this far off OFF ...
HUNT_PAST = 2.0       # ... and is turned to this far past OFF
LIFT = 0.06


def knob_job(task, arm: "Arm", i: int, ctx, pick_only=False, place_only=False):
    """Generator for one knob: pick (``pick_only``), or place, find the flat and push home (``place_only``)."""
    from yam_arena.demo import grip_rotation

    T = task
    ez = np.array([0.0, 0.0, 1.0])
    x, y, a = T.KNOB_START[i]
    yaw_pick = math.radians(a)
    knob0 = np.array([x, y, T.PANEL_Z + 0.0005])
    pick = knob0 + (PAD_UP + TIPS) * ez
    sx, sy = T.STEMS[i]
    tip = np.array([sx, sy, T.PANEL_Z + T.STEM_OUT])
    side = -1.0 if a < 0 else 1.0                 # the knob starts this side of OFF and is turned back toward it

    def measure(key, value):
        if CALIBRATE:
            _MEASURED[key] = [round(float(v), 5) for v in value]
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)

    if not place_only:
        rot = grip_rotation(yaw_pick)
        yield from arm.move(pick + 0.06 * ez, rot, 2.5, grip=0.35, label=f"over knob {i}")
        yield from arm.move(pick, rot, 1.2, label="fingers round its fin")
        yield from arm.set_grip(0.0, 0.8)
        yield from wait(0.3)
        yield from arm.move(pick + LIFT * ez, rot, 1.2, label="lift it")
        if CALIBRATE:
            s = ctx.states()[f"knob_{i}"]
            off = arm.rot.T @ (s.pos - arm.pos)
            turn = math.degrees(math.atan2(s.rotation()[1, 0], s.rotation()[0, 0])) - a
            measure(f"knob_in_hand_{i}", (*off, turn))
        if pick_only:
            return
    ox, oy, oz, dturn = CAL[f"knob_in_hand_{i}"]
    off = np.array([ox, oy, oz])

    def tcp_for(mouth, knob_yaw_deg):
        """The TCP pose that puts the knob's mouth at ``mouth`` with its pointer at ``knob_yaw_deg`` from OFF."""
        yaw = math.radians(knob_yaw_deg - dturn)
        r = grip_rotation(yaw)
        return mouth - r @ off, r, yaw

    start = side * HUNT_FROM
    over, r_over, _ = tcp_for(tip + 0.03 * ez, start)
    yield from arm.move(over + 0.03 * ez, r_over, 2.5, label=f"carry knob {i} over its stem")
    yield from arm.move(over, r_over, 1.0, label="above the tip")
    # On the tip (the bore's round mouth over it, its flat resting on the tip) and pressing lightly.
    on, _, y0 = tcp_for(tip - T.FLAT_Z[0] * ez - CAL["light_press"] * ez, start)
    yield from arm.move(on, r_over, 1.5, label="on the tip, pressing lightly")
    _, _, y1 = tcp_for(tip, -side * HUNT_PAST)
    yield from arm.turn(on, y0, y1, 3.0, label="turn toward OFF until it drops onto the flat")
    yield from wait(0.5)
    if CALIBRATE:
        s, st = ctx.states()[f"knob_{i}"], ctx.states()[f"stem_{i}"]
        measure(f"after_turn_{i}", (1000 * (s.pos[2] - T.PANEL_Z), 1000 * (T.PANEL_Z - st.pos[2]),
                                    math.degrees(math.atan2(s.rotation()[1, 0], s.rotation()[0, 0]))))
    # Let go, close the fingers and push the knob home with the fingertips on top of its fin.
    yield from arm.set_grip(0.35, 0.6)
    yield from arm.move(arm.pos + 0.03 * ez, arm.rot, 0.8, label="fingers up off the fin")
    yield from arm.set_grip(0.0, 0.6)
    fin_top = np.array([sx, sy, T.PANEL_Z + T.SEATED_GAP + T.KNOB_H])
    yield from arm.move(fin_top + (TIPS + 0.004) * ez, arm.rot, 0.8, label="fingertips over the fin")
    yield from arm.move(fin_top + (TIPS - CAL["push_past"]) * ez, arm.rot, 1.5, label="push it home")
    yield from wait(0.6)
    if CALIBRATE:
        s, st = ctx.states()[f"knob_{i}"], ctx.states()[f"stem_{i}"]
        measure(f"pushed_{i}", (1000 * (s.pos[2] - T.PANEL_Z), 1000 * (T.PANEL_Z - st.pos[2])))
    yield from arm.move(fin_top + (TIPS + 0.05) * ez, arm.rot, 1.0, label="up off the knob")
    yield from arm.move(*arm.home, 2.5, grip=1.0, label="back")


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    L, R = ctx.left, ctx.right
    yield from both(knob_job(task, L, 0, ctx, pick_only=True), knob_job(task, R, 1, ctx, pick_only=True))
    yield from knob_job(task, L, 0, ctx, place_only=True)
    yield from knob_job(task, R, 1, ctx, place_only=True)


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
            print(f"step {steps} score {info.get('score')} max stem turn {task._EP.get('max_turn')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    # The trial's score is what the task file's score() reported on the last step (task_score gives 1.0 on success,
    # which score() also is then); it is never set to 1.0 here just because the episode ended.
    final = float(info.get("score", 0.0) or 0.0)
    result = {"task_score": final, "success": success, "steps": steps, "first_full_score_step": first_full,
              "script_ended_step": ended, "max_stem_turn_deg": round(float(task._EP.get("max_turn", 0.0)), 2),
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
