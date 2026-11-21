"""Scripted full-score demo for toy-train-track-close, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]
    python demo.py task.py --ik          (no simulator: IK error of every target, step by step)

Open loop: every target comes from the task file's fixed layout and the clock; the only feedback is the arm's own joint
readings (an integral trim of the tool point on the way down). No object pose steers it. The right arm does the work.

1. The right hand comes straight down over the loose large curve's middle, its fingers open 66 mm across the 40 mm
   track, and pinches it across its width with the fingertips 3.5 mm above the table.
2. It lifts the curve 45 mm, carries it over the gap, turning it a quarter turn on the way (it lies across the way it
   has to go in), and lowers it in its place until its underside is 1 mm above the table: the socket at its near end
   comes down round the last piece's peg and its own peg into the next piece's socket, both at once.
3. It opens, lifts out and goes back to its ready pose.

Wrong attempts (DEMO_VARIANT=...):
  straight   the straight piece set in the gap the same way: it cannot reach both connectors (0)
  short      the short curve set in the gap: its ends fall short of the connectors (0)
  unturned   the right curve carried over the gap without the quarter turn and let go: it lies across the gap (0)
  offset     the right curve set down 6 mm outward of its line: it rests on the peg and the socket's rim (0.25)
  hold       the right curve set in place but never let go (0)
Writes one MP4 per camera, setup.png and result.json; DEMO_DEBUG=1 (default) prints a trace every 10 steps.
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
VARIANTS = ("", "straight", "short", "unturned", "offset", "hold")
KI, CORR_MAX = 0.2, 0.03        # joint-feedback trim: gain per step and limit (m)
XY = np.array([1.0, 1.0, 0.0])  # trim x and y only

TCP_Z = float(os.environ.get("DEMO_TCP_Z_MM", "16.5")) / 1000    # TCP above the table at the pinch (tips 3.5 mm up)
OPEN = float(os.environ.get("DEMO_OPEN", "0.70"))                  # about 66 mm between the fingers going in
CARRY = 0.045 + TCP_Z                                              # carried with its underside 45 mm up
SET_EXTRA = 0.001                                                  # let go with the underside 1 mm above the table
DOWN = np.array([0.0, 0.0, -1.0])
# The large curve turns 2.4 degrees counter-clockwise in the pinch and sits 0.9 mm forward of the tool point by the time
# it is over the gap, the same in every run (Isaac batches 7c7febf7 and 496e85ab, DEMO_VARIANT=hold); the set-down is
# turned and shifted back by that much. Fixed calibration constants, not read from the simulator.
PLACE_TRIM_DEG = float(os.environ.get("DEMO_TRIM_DEG", "-2.4"))
PLACE_TRIM_X = float(os.environ.get("DEMO_TRIM_X_MM", "-0.9")) / 1000


def cols(closing, approach):
    x, z = np.asarray(closing, float), np.asarray(approach, float)
    return np.column_stack([x, np.cross(z, x), z])


def rot_axis(axis, th):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each from where the last command left it, with an optional
    integral trim on the arm's own joint readings (forward kinematics) so the measured tool point follows the plan."""

    def __init__(self, name: str, base_pos, read=None):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.pos = np.asarray(self.pos, float)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.last_mm = 0.0
        self.read = read
        self.corr = np.zeros(3)

    def _solve(self, pos, rot, servo=None):
        from yam_arena import ik

        if servo is not None and self.read is not None:
            meas = self.read()
            if meas is not None:
                self.corr = np.clip(self.corr + KI * np.asarray(servo) * (self.pos - meas[0]), -CORR_MAX, CORR_MAX)
        q, pe, _ = ik.solve(np.asarray(pos, float) + self.corr, ik.matrix_to_quat_xyzw(rot), self.q,
                            base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.last_mm = pe * 1000
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, servo=None):
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s), servo)
            yield

    def path(self, fn, duration: float, servo=None):
        """Follow ``fn(s) -> (pos, rot)`` for s from 0 to 1 (smoothstep timing)."""
        from yam_arena import ik

        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            pos, rot = fn(ik.smoothstep(k / n))
            self._solve(pos, rot, servo)
            yield

    def hold(self, duration: float, grip: float | None = None, servo=None):
        yield from self.move(self.pos, self.rot, duration, grip, servo)

    def to_joints(self, q_target, duration: float, grip: float | None = None):
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        self.corr = np.zeros(3)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (np.asarray(q_target, float) - q0)
            self.grip = g0 + s * (g1 - g0)
            pos, rot = ik.tcp_pose(self.q, self.base)
            self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)
            self.cmd = np.r_[self.q, self.grip]
            yield


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


def chain(*gens):
    for g in gens:
        yield from g




def grip_for(closing) -> np.ndarray:
    """Pointing straight down, fingers closing along horizontal ``closing``."""
    c = np.asarray(closing, float)
    return cols(c / np.linalg.norm(c), DOWN)


def piece_targets(T, name: str, variant: str) -> dict:
    """Pick and place targets for one loose piece (world frame, relative to the env origin)."""
    kind, x, y, yaw = T.LOOSE_POSE[name]
    gx, gy, gyaw = T.loop_pose(T.GAP_K)
    place_yaw = (yaw if variant == "unturned" else gyaw) + math.radians(PLACE_TRIM_DEG)
    gx += PLACE_TRIM_X
    if variant == "offset":                      # 6 mm outward of the loop's centre line
        out = np.array([gx, gy]) - T.LOOP_C
        gx, gy = np.array([gx, gy]) + 0.006 * out / np.linalg.norm(out)
    top = T.TOP
    across = lambda a: np.array([-math.sin(a), math.cos(a), 0.0])    # noqa: E731 - the piece's own y, world frame
    return {"pick": np.array([x, y, top + TCP_Z]), "pick_hi": np.array([x, y, top + 0.09]),
            "lift": np.array([x, y, top + CARRY]),
            "over": np.array([gx, gy, top + CARRY]), "set": np.array([gx, gy, top + TCP_Z + SET_EXTRA]),
            "out": np.array([gx, gy, top + 0.09]),
            "R_pick": grip_for(across(yaw)), "R_place": grip_for(across(place_yaw))}


FLIP = np.diag([-1.0, -1.0, 1.0])       # the same parallel grip turned half a turn about its approach axis


def best_flip(arm, P) -> np.ndarray:
    """FLIP or identity, whichever gives the smaller worst IK error over the pick and the place, solved one after
    another from the arm's current command (the two grips are the same pinch with the fingers swapped)."""
    from yam_arena import ik

    out = []
    for F in (np.eye(3), FLIP):
        q, worst = arm.q.copy(), 0.0
        Rp, Rq = P["R_pick"] @ F, P["R_place"] @ F
        poses = [(P["pick_hi"], Rp), (P["pick"], Rp), (P["lift"], Rp)]
        poses += [(P["lift"] + s * (P["over"] - P["lift"]), ik.slerp_matrix(Rp, Rq, s)) for s in np.linspace(0, 1, 5)]
        poses += [(P["set"], Rq), (P["out"], Rq)]
        for pos, R in poses:
            q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(R), q, base_pos=arm.base, iters=200)
            worst = max(worst, pe + 0.05 * re + 0.01 * max(0.0, abs(float(q[5])) - 2.6))
        out.append((worst, F))
    return min(out, key=lambda t: t[0])[1]


def arm_script(T, arm, variant: str):
    name = {"straight": "track_b", "short": "track_c"}.get(variant, T.FITS)
    P = piece_targets(T, name, variant)
    F = best_flip(arm, P)
    Rp, Rq = P["R_pick"] @ F, P["R_place"] @ F
    # 1. down over the piece's middle, pinch it across its width
    yield from arm.move(P["pick_hi"], Rp, 3.0, OPEN)
    yield from arm.move(P["pick"], Rp, 1.5, servo=XY)
    yield from arm.hold(0.4, servo=XY)
    yield from arm.hold(0.8, 0.0)
    # 2. up, over the gap turning it, down into its place
    yield from arm.move(P["lift"], Rp, 1.2)
    yield from arm.move(P["over"], Rq, 3.0, servo=XY)
    yield from arm.hold(0.5, servo=XY)
    yield from arm.move(P["set"], Rq, 2.0, servo=XY)
    yield from arm.hold(0.5, servo=XY)
    if variant == "hold":
        yield from arm.hold(8.0, servo=XY)
        return
    # 3. let go, lift out, back to the ready pose
    yield from arm.hold(0.6, OPEN)
    yield from arm.move(P["out"], Rq, 1.2)
    from yam_arena.constants import READY as READY_Q

    yield from arm.to_joints(READY_Q, 2.0)


def script(task, ctx):
    variant = os.environ.get("DEMO_VARIANT", "")
    if variant not in VARIANTS:
        raise ValueError(f"unknown DEMO_VARIANT {variant!r}")
    yield from arm_script(task, ctx.right, variant)
    yield from wait(1.0)


def bases(task):
    from yam_arena import station

    up = np.array([0.0, 0.0, getattr(task, "PEDESTAL", 0.0)])
    return {k: np.asarray(v, float) + up for k, v in station.BASES.items()}


class Ctx:
    def __init__(self, emb, task):
        self.emb = emb
        self.bases = bases(task)
        self.left = Arm("left", self.bases["left"], read=lambda: self.joint_tcp("left"))
        self.right = Arm("right", self.bases["right"], read=lambda: self.joint_tcp("right"))

    def joint_tcp(self, name: str):
        """The arm's tool point and gripper rotation from its own joint positions (forward kinematics)."""
        from partkit import _np
        from yam_arena import ik
        from yam_arena.constants import ARM_JOINTS

        art = self.emb._env.unwrapped.scene[f"{name}_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        pos, rot = ik.tcp_pose(_np(art.data.joint_pos)[0][ids], self.bases[name])
        return np.asarray(pos, float), np.asarray(rot, float)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def summary(task, st) -> dict:
    """Logging only: where each loose piece is and what the scorer makes of the loop."""
    out = {}
    for n in task.LOOSE_POSE:
        p = st[n]
        out[n] = {"pos": np.round(np.asarray(p.pos) - [0, 0, task.TOP], 4).tolist(),
                  "yaw_deg": round(math.degrees(task.yaw_of(p)), 1), "tilt_deg": round(task.tilt_deg(p), 1)}
    out["joints"] = task.joints(st)
    out["near_gap"] = task.near_gap(st[task.FITS])
    gx, gy, _ = task.loop_pose(task.GAP_K)
    out["fits_off_mm"] = round(1000 * math.hypot(float(st[task.FITS].pos[0]) - gx, float(st[task.FITS].pos[1]) - gy), 1)
    out["loop_moved_mm"] = round(1000 * max(math.hypot(float(st[p.name].pos[0]) - p.pos[0],
                                                       float(st[p.name].pos[1]) - p.pos[1]) for p in task.LOOP), 1)
    return out


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
    ctx = Ctx(emb, task)
    gen = script(task, ctx)
    frames: dict[str, list] = {}
    debug = os.environ.get("DEMO_DEBUG", "1") != "0"
    min_gap = float("inf")
    import clearance

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    last_gap = {"mm": None}

    def row(steps, info):
        from yam_arena.scene import object_states

        st = object_states(emb._env, emb.task.names)
        return {"step": steps, "score": info.get("score"), "track": summary(task, st), "arm_gap_mm": last_gap["mm"],
                "violation": task._STATE["violation"],
                "tcp": {"L_cmd": np.round(ctx.left.pos - [0, 0, task.TOP], 3).tolist(),
                        "L_meas": np.round(ctx.joint_tcp("left")[0] - [0, 0, task.TOP], 3).tolist(),
                        "R_cmd": np.round(ctx.right.pos - [0, 0, task.TOP], 3).tolist(),
                        "R_meas": np.round(ctx.joint_tcp("right")[0] - [0, 0, task.TOP], 3).tolist(),
                        "grip": [round(float(ctx.left.grip), 2), round(float(ctx.right.grip), 2)]},
                "ik_mm": [round(ctx.left.last_mm, 1), round(ctx.right.last_mm, 1)]}

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
        if steps % 5 == 0:
            try:
                gap, _ = clearance.min_clearance(ctx.left.q, ctx.left.grip, ctx.left.base, ctx.right.q,
                                                 ctx.right.grip, ctx.right.base)
                min_gap = min(min_gap, gap)
                last_gap["mm"] = round(1000 * gap, 1)
            except Exception as exc:  # noqa: BLE001 - logging only
                if steps == 5:
                    print(f"clearance unavailable: {exc!r}", flush=True)
        if debug and steps % 10 == 0:
            try:
                print("TRACE " + json.dumps(row(steps, info), default=str), flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"trace unavailable: {exc!r}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "variant": os.environ.get("DEMO_VARIANT", ""),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)},
              "min_arm_link_gap_mm": round(1000 * min_gap, 1) if min_gap < float("inf") else None,
              "violation": task._STATE["violation"]}
    try:
        from yam_arena.scene import object_states

        result["track"] = summary(task, object_states(emb._env, emb.task.names))
    except Exception as exc:  # noqa: BLE001 - logging only
        result["track"] = repr(exc)
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, default=str), flush=True)
    return result


def ik_check(task_path: str) -> None:
    """No simulator: yam_arena IK error along each arm's whole script (both arms, default variant)."""
    import importlib

    sys.path.insert(0, str(Path(task_path).resolve().parent))
    task = importlib.import_module(Path(task_path).stem)
    B = bases(task)

    class C:
        pass

    ctx = C()
    ctx.left, ctx.right = Arm("left", B["left"]), Arm("right", B["right"])
    n = 0
    for _ in script(task, ctx):
        n += 1
    print(f"IK steps {n}: worst left {ctx.left.worst_mm:.1f} mm, right {ctx.right.worst_mm:.1f} mm", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out", nargs="?", default="demo_out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--ik", action="store_true", help="only print the IK error along the script")
    a = ap.parse_args()
    if a.ik:
        ik_check(a.task)
    else:
        run(a.task, a.out, a.max_steps, record=not a.no_record)
