"""Scripted full-score demo for the locking pliers task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

The right arm grips both handles of the open pliers on their holder (the fingers across the handles, closed just
enough to hold them), lifts them and carries them, level, to the tube at the orange band, with the tube 13 mm in
front of the jaw tips. It corrects its aim once from the measured pliers pose, slides the jaws onto the tube, and
closes the gripper fully: the squeeze shuts the handles until the catch clicks. It opens the gripper and lifts away.
The plan uses only the published start layout and the pliers' pose (what a camera sees), never the catch or hinge.
Writes demo.mp4 (the three camera views side by side), setup.png (their first frames) and result.json.

DEMO_VARIANT=no_squeeze puts the jaws around the tube and lets go without squeezing; no_tube squeezes the pliers
shut in the air beside the tube and lets go; sweep pushes the pliers off their holder toward the tube with closed
fingers; idle does nothing. All must score 0.
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
GRIP_X = -0.035                 # along the handles from the pivot, where the fingers squeeze (pliers frame, m)
GRIP_Y = 0.00675                 # midway between the open handles' outer faces at the fingers (pliers frame, m)
OPEN_WIDTH = 0.0495              # across the open handles' outer faces at the fingers (m)
HOLD_WIDTH = 0.040              # gripper opening while carrying: just enough squeeze to hold them
SHUT_WIDTH = 0.036             # across the shut handles' outer faces
TUBE_IN_JAWS = np.array([0.032, -0.004])   # tube axis in the pliers frame when the jaws are slid on (open, 2 mm off the pad)
TUBE_LOCKED_Y = 0.002           # tube axis in the pliers frame once squeezed shut (pad pressed 4 mm)
LEAD = 0.045                    # how far short of the tube the jaws start their slide (m)


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
        before = self.worst_mm
        self.worst_mm = 0.0
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield
        if self.worst_mm > 1.0:
            print(f"IK {self.name}: {self.worst_mm:.1f} mm on the move to {np.round(pos, 3).tolist()}", flush=True)
        self.worst_mm = max(before, self.worst_mm)

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def width(w: float) -> float:
    """Gripper command for a finger opening of ``w`` m (1 = 94 mm open)."""
    return float(np.clip(w / 0.094, 0.0, 1.0))


def _rz(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _grip_yaw(points, base) -> float:
    """0 or pi: the turn of the straight-down grip that IK reaches best at the given TCP points."""
    from yam_arena import ik
    from yam_arena.constants import READY
    from yam_arena.demo import grip_rotation

    worst = {}
    for flip in (0.0, math.pi):
        errs = [ik.solve(p, ik.matrix_to_quat_xyzw(grip_rotation(flip)), np.array(READY, float), base_pos=base,
                         iters=200)[1] for p in points]
        worst[flip] = max(errs)
    return min(worst, key=worst.get)


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    A = ctx.right
    T = task
    variant = os.environ.get("DEMO_VARIANT", "")
    if variant == "idle":
        return
    p0 = ctx.states()["pliers"]
    yaw0 = math.atan2(p0.rotation()[1, 0], p0.rotation()[0, 0])
    tube = np.array([*T.TUBE_XY, T.TOP + sum(T.BAND) / 2])

    def tcp_for(pivot, yaw, y_off=GRIP_Y):
        """TCP over the grip point of pliers whose pivot is at ``pivot`` (world) turned ``yaw``."""
        return np.asarray(pivot) + _rz(yaw) @ np.array([GRIP_X, y_off, 0.002])

    # Where the pivot goes for the jaws to sit on the tube at the band, with the pliers square to the world.
    on_tube = tube - np.array([TUBE_IN_JAWS[0], TUBE_IN_JAWS[1], 0.0])
    approach = on_tube - np.array([LEAD, 0.0, 0.0])
    flip = _grip_yaw([tcp_for(p0.pos, yaw0), tcp_for(approach, 0.0), tcp_for(on_tube, 0.0)], A.base)
    down = _rz(yaw0 + T.OPEN / 2) @ grip_rotation(flip)
    level = _rz(T.OPEN / 2) @ grip_rotation(flip)

    if variant == "sweep":
        # Closed fingers behind the pliers' handle ends, pushed forward along the holder toward the tube.
        yield from A.set_grip(0.0, 0.6)
        start = p0.to_world((-0.150, 0.0, 0.0))
        yield from A.move(start + [0, 0, 0.06], down, 2.5)
        yield from A.move(start, down, 1.5)
        yield from A.move(start + [0.16, 0.06, 0.0], down, 4.0)
        yield from A.move(A.pos + [0, 0, 0.08], down, 1.5)
        return

    # 1. Fingers across both open handles; close just enough to hold them; lift off the holder.
    grasp = tcp_for(p0.pos, yaw0)
    yield from A.move(grasp + [0, 0, 0.08], down, 2.5, grip=1.0)
    yield from A.move(grasp, down, 1.5)
    yield from A.set_grip(width(HOLD_WIDTH), 1.0)
    yield from wait(0.3)
    yield from A.move(grasp + [0, 0, 0.06], down, 2.5)

    # 2. Carry them level to the band, the tube 13 mm in front of the jaw tips; correct once from the measured pose.
    yield from A.move(tcp_for(approach, 0.0) + [0, 0, 0.03], level, 4.0)
    yield from A.move(tcp_for(approach, 0.0), level, 1.0)
    yield from wait(0.5)
    # The pliers pitch a few degrees in the fingers while carried: turn the hand to bring them level, then correct
    # the aim, both from the measured pose of the pliers.
    p = ctx.states()["pliers"]
    pitch = math.asin(float(np.clip(p.rotation()[2, 0], -1.0, 1.0)))
    level = _ry(pitch) @ level
    yield from A.move(A.pos, level, 1.0)
    yield from wait(0.3)
    for _ in range(2):
        p = ctx.states()["pliers"]
        err = approach - p.pos
        yield from A.move(A.pos + err, level, 0.8)
        yield from wait(0.3)

    if variant == "no_tube":
        # Squeeze shut in the air beside the tube, then let go.
        yield from A.move(A.pos + [0, -0.06, 0.0], level, 1.5)
        yield from A.set_grip(0.0, 2.0)
        yield from wait(0.5)
        yield from A.set_grip(1.0, 1.0)
        yield from A.move(A.pos + [0, 0, 0.08], level, 1.5)
        return

    # 3. Slide the jaws onto the tube.
    yield from A.move(A.pos + [LEAD, 0, 0], level, 2.0)
    yield from wait(0.3)
    if variant == "no_squeeze":
        yield from A.set_grip(1.0, 1.0)
        yield from A.move(A.pos + [0, 0, 0.08], level, 1.5)
        return

    # 4. Squeeze shut. A parallel gripper moves the fixed handle 12 mm toward its centre while the jaws close, and
    # the pliers shift 6 mm as the pad meets the tube, so the hand slides 18 mm the other way as it squeezes.
    shift = (OPEN_WIDTH - SHUT_WIDTH) / 2 + (TUBE_LOCKED_Y - TUBE_IN_JAWS[1])
    if os.environ.get("DEMO_SHIFT_MM"):
        shift = float(os.environ["DEMO_SHIFT_MM"]) / 1000
    yield from A.move(A.pos - [0, shift, 0], _rz(-T.OPEN / 2) @ level, 3.0, grip=0.0)
    level = _rz(-T.OPEN / 2) @ level
    yield from wait(0.5)

    # 5. Let go and lift away.
    yield from A.set_grip(width(SHUT_WIDTH + 0.03), 0.8)
    yield from A.move(A.pos + [0, 0, 0.08], level, 1.5)
    yield from A.move(A.pos + [-0.06, -0.04, 0.03], level, 1.5)


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
    from types import SimpleNamespace

    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scoring import task_score

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height, device=os.environ.get("DEMO_DEVICE", "cpu"))
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}
    trace = (out / "trace.jsonl").open("w", buffering=1)

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended = 0, None
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        snap(res.observation)
        p = ctx.states()["pliers"]
        measure = task.measure(p)
        trace.write(json.dumps(dict(step=steps, score=res.info.get("score"), pos=p.pos.tolist(),
            quat=p.quat_xyzw.tolist(), hinges=p.hinges, sliders=p.sliders,
            measure={k: (v.tolist() if isinstance(v, np.ndarray) else v) for k,v in measure.items()},
            tcp=ctx.right.pos.tolist(), command=ctx.right.cmd.tolist())) + "\n")
        if steps % 50 == 0:
            p = ctx.states()["pliers"]
            print(f"step {steps} score {res.info.get('score')} pliers_mm {np.round(p.pos * 1000, 1).tolist()}",
                  flush=True)
        if res.terminated:
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    trace.close()
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps,
              "variant": os.environ.get("DEMO_VARIANT", ""),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        cams = sorted(frames, key=lambda c: ("left" not in c, "top" not in c))   # left, top, right
        _write_mp4([np.hstack([frames[c][i] for c in cams]) for i in range(len(frames[cams[0]]))], out / "demo.mp4")
        imageio.imwrite(out / "setup.png", np.hstack([frames[c][0] for c in cams]))
    if os.environ.get("DEMO_RESET_CHECK") == "1":
        reset_obs = emb.reset(Scene(id="reset-check", instruction=task.INSTRUCTION))
        reset_ctx = Ctx(emb)
        fresh = reset_ctx.states()["pliers"]
        reset_record = dict(pos=fresh.pos.tolist(), hinges=fresh.hinges, sliders=fresh.sliders,
                            recent_samples=len(task._STATE["recent"]), step=task._STATE["step"])
        for _ in range(20):
            reset_result = emb.step(reset_ctx.action())
        fresh = reset_ctx.states()["pliers"]
        reset_record.update(after20_pos=fresh.pos.tolist(), after20_hinges=fresh.hinges,
                            score=reset_result.info.get("score"), success=bool(reset_result.terminated))
        reset_record["passed"] = bool(
            np.linalg.norm(fresh.pos - np.asarray(task.PLIERS_START)) < 0.002
            and abs(fresh.hinges["moving_half"]) < .01
            and reset_record["step"] == 0 and reset_record["recent_samples"] == 0
            and reset_record["score"] == 0.0 and not reset_record["success"])
        (out / "reset-check.json").write_text(json.dumps(reset_record, indent=2))
        if not reset_record["passed"]:
            raise RuntimeError("Same-process reset check failed; see reset-check.json")
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
