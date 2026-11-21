"""Scripted full-score demo for seat-switch-line-card-with-dual-ejector-levers."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
TIPS = 0.013          # TCP sits 13 mm above pad tips


class Arm:
    """One arm: smooth Cartesian moves (IK on URDF)."""

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
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def path(self, poses, duration: float):
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            i = min(len(poses) - 1, round(k / n * (len(poses) - 1)))
            self._solve(*poses[i])
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


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


def card_x(T, a_deg: float) -> float:
    a = math.radians(a_deg)
    px = T.CLAW_R * math.tan(a) - (0.003 + T.PIN_R) / math.cos(a)
    return (T.X_FACE + T.PIN_X) - T.PIVOT_X - px


def tab(T, side: int, a_deg: float, z: float, flip: float = 0.0, card_at: float = -1.0):
    from yam_arena.demo import grip_rotation

    a = math.radians(a_deg)
    d = np.array([-math.sin(a), -side * math.cos(a)])
    n = np.array([math.cos(a), -side * math.sin(a)])
    pivot = np.array([max(card_x(T, a_deg), card_at) + T.PIVOT_X, side * T.PIVOT_Y])
    c = pivot + (T.HANDLE_L + 0.001) * d
    yaw = math.atan2(-n[0], n[1]) + flip
    return np.array([c[0], c[1], z]), grip_rotation(yaw), yaw


CAL = {
    "push_y": 0.035,
    "push_gap": 0.010,
    "gap_after_push": 0.008,
    "shut_past": 10.0,
}


def script(task, ctx):
    from yam_arena.demo import grip_rotation

    T = task
    L, R = ctx.left, ctx.right
    top = T.TOP
    face_z = top + T.FLOOR + 0.008 + TIPS
    tab_z = top + T.FLOOR + 0.047 + TIPS
    high = top + 0.13
    push_l = grip_rotation(-math.pi / 2)
    push_r = grip_rotation(math.pi / 2)
    pad = 0.0064
    start_face = T.SEAT_X - T.START_OUT

    # 1. Both arms push card faceplate in synchronously
    ys = CAL["push_y"]
    x_push = T.SEAT_X - CAL["push_gap"] - pad
    yield from both(L.move(np.array([start_face - 0.04, ys, high]), push_l, 2.0, grip=0.0),
                    R.move(np.array([start_face - 0.04, -ys, high]), push_r, 2.0, grip=0.0))
    yield from both(L.move(np.array([start_face - 0.02 - pad, ys, face_z]), push_l, 1.5),
                    R.move(np.array([start_face - 0.02 - pad, -ys, face_z]), push_r, 1.5))
    yield from both(L.move(np.array([x_push, ys, face_z]), push_l, 3.0),
                    R.move(np.array([x_push, -ys, face_z]), push_r, 3.0))
    yield from wait(0.5)
    yield from both(L.move(np.array([x_push - 0.04, ys, high]), push_l, 1.2),
                    R.move(np.array([x_push - 0.04, -ys, high]), push_r, 1.2))

    # 2. Both arms grasp finger tabs
    a0 = T.OPEN_DEG
    at = T.SEAT_X - CAL["gap_after_push"]
    (pl, rl, _), (pr, rr, _) = tab(T, 1, a0, tab_z, math.pi, at), tab(T, -1, a0, tab_z, 0.0, at)
    up = np.array([0, 0, high - tab_z])
    yield from both(L.move(pl + up, rl, 1.5, grip=0.45), R.move(pr + up, rr, 1.5, grip=0.45))
    yield from both(L.move(pl, rl, 1.2), R.move(pr, rr, 1.2))
    yield from both(L.set_grip(0.0, 0.8), R.set_grip(0.0, 0.8))
    yield from wait(0.3)

    # 3. Swing both levers shut together along calculated arc into catches
    angles = np.linspace(a0, -CAL["shut_past"], 80)
    path_l = [tab(T, 1, a, tab_z, math.pi, at)[:2] for a in angles]
    path_r = [tab(T, -1, a, tab_z, 0.0, at)[:2] for a in angles]
    yield from both(L.path(path_l, 6.0), R.path(path_r, 6.0))
    yield from wait(1.0)

    # 4. Release grip and withdraw clear
    yield from both(L.set_grip(0.45, 0.6), R.set_grip(0.45, 0.6))
    yield from both(L.move(L.pos + np.array([-0.04, 0, 0.08]), L.rot, 1.2),
                    R.move(R.pos + np.array([-0.04, 0, 0.08]), R.rot, 1.2))


def waypoints(T):
    top = T.TOP
    out = []
    face_z = top + T.FLOOR + 0.008 + TIPS
    tab_z = top + T.FLOOR + 0.047 + TIPS
    from yam_arena.demo import grip_rotation

    for x in (T.SEAT_X - T.START_OUT - 0.02 - 0.0064, T.SEAT_X - CAL["push_gap"] - 0.0064):
        for side, arm in ((1, "left"), (-1, "right")):
            out.append((arm, np.array([x, side * CAL["push_y"], face_z]), grip_rotation(-side * math.pi / 2)))
    for side, arm in ((1, "left"), (-1, "right")):
        for a in np.linspace(T.OPEN_DEG, -CAL["shut_past"], 18):
            p, r, _ = tab(T, side, a, tab_z, math.pi if side > 0 else 0.0, T.SEAT_X - CAL["gap_after_push"])
            out.append((arm, p, r))
    return out


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


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, cam_width: int = 320,
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

    def snap(o, step):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs, 0)
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation, steps)
        if steps % 100 == 0:
            print(f"step {steps} score {info.get('score')}", flush=True)
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
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
