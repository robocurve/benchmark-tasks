"""democore: the scripted-demo machinery shared by the kitchen tasks (yam_arena embodiment).

A task's demo.py writes a plan as timed segments for each arm (``Arm.add``: smoothstep moves of the gripper's TCP,
or ``path=`` for a curve), and calls ``run()``. The script closes the loop on the objects' poses, as any scripted
policy with full state may:

- ``Arm`` solves IK per step (with retries from fixed seeds) and adds a capped joint-space integral term that takes
  out the soft arm's steady sag in free motion.
- ``ObjectServo`` moves the TCP so that a held object (not the gripper) follows the plan: on the labels it is given,
  it integrates the error between where the plan puts the object and where it is.
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CONTROL_HZ = 20.0
DEBUG_EVERY = int(os.environ.get("DEMO_DEBUG", "0"))
# extra IK seeds around the YAM's ready pose (fixed, so plans are reproducible)
SEEDS = [np.array([0.0, 1.0, 0.8, 0.0, 0.0, 0.0]) + d for d in np.random.default_rng(11).normal(0.0, 0.7, (10, 6))]


def rot(axis, ang: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


@dataclass
class Seg:
    duration: float
    pos: np.ndarray | None
    R: np.ndarray | None
    grip: float | None
    label: str = ""
    path: object = None  # optional u -> (pos, R) in [0, 1] (smoothstep-timed): the segment follows it, not a line


class Arm:
    """Timed TCP segments for one arm, smoothly interpolated and solved with yam_arena's IK."""

    def __init__(self, side: str, base, q0, pos0, R0, grip0=1.0):
        self.side, self.base, self.q = side, np.asarray(base, dtype=float), np.array(q0, dtype=float)
        self.segs: list[Seg] = []
        self.start = (np.asarray(pos0, dtype=float), np.asarray(R0, dtype=float), grip0)
        self.worst: dict[str, tuple[float, float]] = {}
        self.correct = None  # optional (pos, R, label) -> (pos, R): adjusts the planned TCP target at run time
        self.integ = None  # optional joint-space integral term (rad), added to the IK solution
        self.q_ik = self.q.copy()

    INTEG_GAIN = 0.05
    INTEG_CLIP = np.array([0.03, 0.03, 0.06, 0.25, 0.25, 0.25])  # rad: small on the stiff shoulder joints

    def update_integral(self, q_actual) -> None:
        if self.integ is None:
            self.integ = np.zeros(6)
        self.integ = np.clip(self.integ + self.INTEG_GAIN * (self.q_ik - np.asarray(q_actual)), -self.INTEG_CLIP,
                             self.INTEG_CLIP)

    def add(self, duration, pos=None, R=None, grip=None, label="", path=None):
        if path is not None:
            pos, R = path(1.0)
        self.segs.append(Seg(duration, None if pos is None else np.asarray(pos, dtype=float),
                             None if R is None else np.asarray(R, dtype=float), grip, label, path))

    @property
    def duration(self) -> float:
        return sum(s.duration for s in self.segs)

    def target(self, t: float):
        from yam_arena import ik

        p0, r0, g0 = self.start
        for s in self.segs:
            p1 = p0 if s.pos is None else s.pos
            r1 = r0 if s.R is None else s.R
            g1 = g0 if s.grip is None else s.grip
            if t < s.duration:
                u = ik.smoothstep(t / s.duration)
                if s.path is not None:
                    pu, ru = s.path(u)
                    return np.asarray(pu, float), np.asarray(ru, float), g0 + u * (g1 - g0), s.label
                return p0 + u * (p1 - p0), ik.slerp_matrix(r0, r1, u), g0 + u * (g1 - g0), s.label
            t -= s.duration
            p0, r0, g0 = p1, r1, g1
        return p0, r0, g0, "done"

    def action7(self, t: float):
        from yam_arena import ik

        pos, R, grip, label = self.target(t)
        if self.correct is not None:
            pos, R = self.correct(pos, R, label)
        self.last_cmd = (np.asarray(pos, float), np.asarray(R, float))
        quat = ik.matrix_to_quat_xyzw(R)
        q, pe, re = ik.solve(pos, quat, self.q, base_pos=self.base, iters=80)
        if pe > 0.002 or re > math.radians(2):  # stuck on a poor branch: retry from other seeds, keep the nearest good
            from yam_arena.constants import READY

            best = (pe + 0.05 * re, q, pe, re)
            for seed in [np.array(READY, float), *SEEDS]:
                q2, pe2, re2 = ik.solve(pos, quat, seed, base_pos=self.base, iters=200)
                cost = pe2 + 0.05 * re2 + 0.002 * float(np.abs(q2 - self.q).max())
                if cost < best[0]:
                    best = (cost, q2, pe2, re2)
            _, q, pe, re = best
        self.q = q
        self.q_ik = q.copy()
        if self.integ is not None:  # joint-space integral term: removes the soft arm's steady sag
            from yam_arena.ik import LOWER, UPPER

            q = np.clip(q + self.integ, LOWER, UPPER)
        key = label.split(":")[-1].strip() if ":" in label else label
        w = self.worst.get(key, (0.0, 0.0))
        self.worst[key] = (max(w[0], pe * 1000), max(w[1], math.degrees(re)))
        return np.r_[q, float(np.clip(grip, 0.0, 1.0))], label




def R_approach(tilt_deg: float, az_deg: float, roll_deg: float = 0.0) -> np.ndarray:
    """A TCP rotation whose approach axis (z) points down, tilted ``tilt_deg`` from vertical toward azimuth ``az_deg``
    (the fingertips lean that way); ``roll_deg`` turns the closing axis (x) about the approach axis. At roll 0 the
    closing axis is horizontal (world x for tilt 0)."""
    tilt, az, roll = map(math.radians, (tilt_deg, az_deg, roll_deg))
    a = np.array([math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), -math.cos(tilt)])
    x0 = np.cross([0.0, 0.0, 1.0], a)
    x0 = x0 / np.linalg.norm(x0) if np.linalg.norm(x0) > 1e-6 else np.array([1.0, 0.0, 0.0])
    if tilt_deg == 0:
        x0 = np.array([math.cos(az), math.sin(az), 0.0])
    y0 = np.cross(a, x0)
    c, s = math.cos(roll), math.sin(roll)
    return np.column_stack([c * x0 + s * y0, -s * x0 + c * y0, a])


def best_rotation(side: str, points, cands, iters: int = 200) -> tuple[np.ndarray, float]:
    """The candidate TCP rotation whose worst IK error over ``points`` (TCP positions) is smallest (m + 0.05 rad)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    best = (None, np.inf)
    for R in cands:
        worst = 0.0
        for p in points:
            e = np.inf
            for sd in [np.array(READY, float), *SEEDS[:5]]:
                q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(R), sd,
                                     base_pos=station.BASES[side], iters=iters)
                e = min(e, pe + 0.05 * re)
                if e < 0.001:
                    break
            worst = max(worst, e)
            if worst >= best[1]:
                break
        if worst < best[1]:
            best = (R, worst)
    return best


def best_turn(side: str, R0, p_from, goal_points, yaws=range(-180, 180, 15), n_mid: int = 4,
              tol: float = 0.0015):
    """The smallest turn about the vertical, Rz(a) @ R0, that keeps the TCP on target on the way and at the goals.

    "On the way" is the planned transit: the TCP moving in a line from ``p_from`` to ``goal_points[0]`` while its
    rotation slerps from R0 to Rz(a) @ R0 (what ``Arm`` does between two segments). Returns (R, cost, a_deg); the cost
    is the worst IK error (m + 0.05 rad) over the transit and goal points, multi-seeded like ``best_rotation``.
    """
    from yam_arena import ik

    p_from = np.asarray(p_from, float)
    goals = [np.asarray(g, float) for g in goal_points]
    best = (None, np.inf, None)
    for a in sorted(yaws, key=abs):
        R1 = rot((0, 0, 1), math.radians(a)) @ R0
        checks = [(p_from + u * (goals[0] - p_from), ik.slerp_matrix(R0, R1, u))
                  for u in np.linspace(0.0, 1.0, n_mid + 2)[1:-1]] + [(g, R1) for g in goals]
        worst = 0.0
        for pt, R in checks:
            worst = max(worst, best_rotation(side, [pt], [R])[1])
            if worst >= min(best[1], 10 * tol):
                break
        if worst < tol:
            return R1, worst, a
        if worst < best[1]:
            best = (R1, worst, a)
    return best


class ObjectServo:
    """Correct an arm's TCP so a held object reaches where the plan puts it.

    The plan gives the TCP; ``offset`` is the object's reference point relative to the TCP, in the TCP frame, when
    held as planned. On segments whose label (the part after "name: ") is in ``xyz`` the full position error is
    integrated; in ``xy`` only the horizontal error (the object is pressed on something); in ``hold`` the last
    correction is kept. Elsewhere the correction is dropped."""

    def __init__(self, arm: Arm, obj: str, offset, xyz=(), xy=(), hold=(), gain=0.1, clip=0.04):
        self.arm, self.obj, self.offset = arm, obj, np.asarray(offset, float)
        self.xyz, self.xy, self.hold, self.gain, self.clip = set(xyz), set(xy), set(hold), gain, clip
        self.off = np.zeros(3)
        self.desired = None

    def update(self, kk, label: str) -> None:
        key = label.split(": ")[-1]
        if self.desired is None or self.obj not in kk.STATE.poses or key in self.hold:
            return
        if key in self.xyz or key in self.xy:
            e = self.desired - np.asarray(kk.STATE.poses[self.obj][0], float)
            if key in self.xy:
                e[2] = 0.0
            self.off = np.clip(self.off + self.gain * e, -self.clip, self.clip)
        else:
            self.off = np.zeros(3)

    def correct(self, pos, R, label):
        key = label.split(": ")[-1]
        self.desired = np.asarray(pos, float) + np.asarray(R, float) @ self.offset
        if key in self.xyz or key in self.xy or key in self.hold:
            return np.asarray(pos, float) + self.off, R
        return pos, R


def load_task(task_path: str):
    import importlib.util

    spec = importlib.util.spec_from_file_location("task_mod", task_path)
    t = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(t)
    return t


def plan_check(task_path: str, build_plan, variants) -> None:
    """Offline: solve IK along every variant's plan and report the worst errors (no simulator)."""
    t = load_task(task_path)
    for variant in variants:
        left, right, _ = build_plan(t, variant)
        T = max(left.duration, right.duration)
        for k in range(int(T * CONTROL_HZ) + 1):
            left.action7(k / CONTROL_HZ)
            right.action7(k / CONTROL_HZ)
        print(f"== {variant}: {T:.1f} s ({int(T * CONTROL_HZ)} steps)")
        for arm in (left, right):
            if not arm.worst:
                continue
            bad = {k: v for k, v in arm.worst.items() if v[0] > 3.0 or v[1] > 2.0}
            print(f"  {arm.side}: worst {max(v[0] for v in arm.worst.values()):.1f} mm "
                  f"{max(v[1] for v in arm.worst.values()):.1f} deg; over 3 mm / 2 deg: "
                  + (", ".join(f"{k} {v[0]:.0f}mm/{v[1]:.0f}d" for k, v in bad.items()) or "none"))


def run(task_path: str, out_dir: str, build_plan, variant: str = "full", max_steps: int = 2400, record: bool = True,
        servos_for=None, frozen=lambda label: False, debug=None) -> dict:
    """Run one scripted trial. ``servos_for(arm_name, arm) -> list[ObjectServo]``; ``frozen(label)``: labels on
    which the joint integral term is not updated (the arm pushes on something); ``debug(k, t, kk, labL, labR)``."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik as _ik
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    emb = yam_arena_embodiment(task=task_path, control_hz=CONTROL_HZ, cam_width=320, cam_height=176,
                               object_jitter=0.0)
    t = emb.task.module
    kk = sys.modules["kitchenkit"]
    obs = emb.reset(Scene(id=f"demo-{variant}", instruction=t.INSTRUCTION))
    kk.capture(emb._env)
    left, right, P = build_plan(t, variant)
    servos = {"left": [], "right": []}
    for name, arm in (("left", left), ("right", right)):
        servos[name] = list(servos_for(name, arm)) if servos_for else []
        if servos[name]:
            def corr(pos, R, label, ss=servos[name]):
                for s in ss:
                    pos, R = s.correct(pos, R, label)
                return pos, R
            arm.correct = corr
    frames = {k: [np.asarray(v)] for k, v in obs.images.items()}
    plan_s = max(left.duration, right.duration) + 2.0
    n_steps = min(max_steps, int(math.ceil(plan_s * CONTROL_HZ)))
    score, success, steps = 0.0, False, 0
    deviation = {"left": {}, "right": {}}
    for k in range(n_steps):
        tt = k / CONTROL_HZ
        for name, arm in (("left", left), ("right", right)):
            for s in servos[name]:
                s.update(kk, arm.target(tt)[3])
        aL, labL = left.action7(tt)
        aR, labR = right.action7(tt)
        res = emb.step(Action(data=pack(aL, aR)))
        steps = k + 1
        score = float(res.info.get("score", 0.0))
        for arm, name, lab in ((left, "left_robot", labL), (right, "right_robot", labR)):
            rob = emb._env.unwrapped.scene[name]
            jn, jp = list(rob.joint_names), kk._np(rob.data.joint_pos)[0]
            q6 = [jp[jn.index(f"joint{i}")] for i in range(1, 7)]
            tcp_a, _ = _ik.tcp_pose(np.asarray(q6), arm.base)
            dev = 1000 * float(np.linalg.norm(tcp_a - arm.target(tt)[0]))
            deviation[arm.side][lab] = max(deviation[arm.side].get(lab, 0.0), dev)
            if not frozen(lab):
                arm.update_integral(q6)
        if record:
            for key, v in res.observation.images.items():
                frames[key].append(np.asarray(v))
        if debug is not None and DEBUG_EVERY and k % DEBUG_EVERY == 0:
            debug(k, t, kk, labL, labR)
        if k % 50 == 0:
            print(f"step {k} t={tt:.1f}s score {score:.2f}  left: {labL} | right: {labR}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
    report = t.report() if hasattr(t, "report") else {}
    result = {"variant": variant, "task_score": 1.0 if success else score, "success": success, "steps": steps,
              "plan_s": round(plan_s, 1), "report": report,
              "worst_ik": {a.side: {k: [round(v[0], 1), round(v[1], 1)] for k, v in a.worst.items()} for a in (left, right)},
              "tracking_mm": {s: {k: round(v, 1) for k, v in d.items()} for s, d in deviation.items()}}
    (out / "result.json").write_text(json.dumps(result, indent=2, default=float))
    print(json.dumps({k: v for k, v in result.items() if k in ("variant", "task_score", "success", "steps", "report")},
                     default=float), flush=True)
    if record:
        import imageio
        from PIL import Image

        imageio.imwrite(out / "setup.png", frames["top_cam"][0])
        n = min(len(v) for v in frames.values())
        tiled = []
        for i in range(n):
            top = np.asarray(Image.fromarray(frames["top_cam"][i]).resize((640, 352), Image.BICUBIC))
            tiled.append(np.concatenate([top, np.concatenate([frames["left_cam"][i], frames["right_cam"][i]], 1)], 0))
        imageio.mimsave(out / "demo.mp4", tiled, fps=CONTROL_HZ, codec="libx264", quality=8,
                        ffmpeg_params=["-movflags", "+faststart"])
    emb.close()
    return result


def main(build_plan, variants, servos_for=None, frozen=lambda label: False, debug=None) -> None:
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out", nargs="?", default="out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--variant", default="full", choices=list(variants))
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--plan-only", action="store_true")
    a = ap.parse_args()
    if a.plan_only:
        plan_check(a.task, build_plan, variants)
    else:
        run(a.task, a.out, build_plan, a.variant, a.max_steps, not a.no_record, servos_for, frozen, debug)
