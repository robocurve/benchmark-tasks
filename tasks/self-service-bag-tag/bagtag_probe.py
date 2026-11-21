"""Shared helpers for bag-tag contact probes: 6D waypoint plans and a traced run.

Plans only command robot joint targets. Nothing here writes object poses.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.urdf import axis_angle_matrix

HERE = Path(__file__).resolve().parent
PANELS = [f"tag_panel_{i:02d}" for i in range(10)]


def frame(approach, closing):
    """TCP rotation whose approach (+z) and finger-closing (+x) axes point as given in the world."""
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x = x / np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def rot(axis, deg):
    return axis_angle_matrix(np.asarray(axis, float), math.radians(deg))


@dataclass
class W:
    duration: float
    pos: tuple
    R: np.ndarray
    finger: float
    label: str = ""


class Plan:
    """Smoothstep-interpolated 6D TCP waypoints solved by warm-started DLS IK."""

    def __init__(self, waypoints, base_pos, q_start=READY, finger_start=FINGER_OPEN):
        self.waypoints = list(waypoints)
        self.base = np.asarray(base_pos, float)
        self.q = np.array(q_start, float)
        self.start_pos, self.start_rot = ik.tcp_pose(self.q, self.base)
        self.finger_start = finger_start
        self.worst_ik_mm = 0.0
        self.worst_ik_deg = 0.0
        self.ik_log = []

    @property
    def duration(self):
        return sum(w.duration for w in self.waypoints)

    def target(self, t):
        prev_pos, prev_rot, prev_f = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            goal = np.asarray(w.pos, float)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, w.R, s),
                        prev_f + s * (w.finger - prev_f), w.label)
            t -= w.duration
            prev_pos, prev_rot, prev_f = goal, w.R, w.finger
        return prev_pos, prev_rot, prev_f, "done"

    def action7(self, t):
        pos, R, finger, label = self.target(t)
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(R), self.q, base_pos=self.base, iters=80)
        self.q = q
        self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
        self.worst_ik_deg = max(self.worst_ik_deg, math.degrees(re))
        grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))
        return np.r_[q, grip], label


def hold_ready():
    return lambda t: (np.r_[READY, 1.0], "ready")


class Run:
    """One continuous reset with per-step arm callables and a JSON/PNG trace."""

    def __init__(self, out: Path, run_id: str, cam_w=640, cam_h=352):
        from inspect_robots import Scene
        from yam_arena.inspect_embodiment import yam_arena_embodiment

        self.out = out
        out.mkdir(parents=True, exist_ok=False)
        import hashlib
        (out/'source_at_launch.json').write_text(json.dumps({n:hashlib.sha256((HERE/n).read_bytes()).hexdigest()
            for n in ('task.py','tag_closure.py','press_contacts.py','partkit.py','direct_thread_transport_probe.py')},indent=2))
        self.emb = yam_arena_embodiment(task=str(HERE / "task.py"), object_jitter=0, control_hz=20,
                                        cam_width=cam_w, cam_height=cam_h, top_cam="demo")
        self.obs = self.emb.reset(Scene(id=run_id, instruction=self.emb.task.module.INSTRUCTION,
                                        init_seed=42), seed=42)
        base = self.emb._env.unwrapped
        # Scripted motion reads poses, not camera feedback. Match rendering to
        # the saved 5 fps video while retaining the original 20 Hz physics/
        # control loop. This opt-in is not used for Dream observations.
        render_stride = int(os.environ.get('BAGTAG_RENDER_STRIDE', '1'))
        if render_stride > 1:
            base.cfg.sim.render_interval *= render_stride
        self.robots = {s: base.scene[s + "_robot"] for s in ("left", "right")}
        self.body_ids = {s: {n: r.body_names.index(n) for n in ("gripper", "tip_left", "tip_right")}
                         for s, r in self.robots.items()}
        self.trace = []
        self.step_i = 0
        self.last_info = {}

    def state(self, labels=("", "")):
        from yam_arena.scene import object_states

        names = PANELS + ["checked_suitcase"]
        st = object_states(self.emb._env, names)
        bodies = {}
        for s, r in self.robots.items():
            pose = r.data.body_pose_w.torch[0].detach().cpu().numpy()
            bodies[s] = {n: pose[i, :3].tolist() for n, i in self.body_ids[s].items()}
            bodies[s]["gripper_quat_xyzw"] = pose[self.body_ids[s]["gripper"], 3:7].tolist()
            bodies[s]["joints"] = r.data.joint_pos.torch[0].detach().cpu().numpy().tolist()
        return {
            "step": self.step_i, "left_phase": labels[0], "right_phase": labels[1],
            "panel_xyz": [st[n].pos.tolist() for n in PANELS],
            "panel_quat_xyzw": [np.asarray(st[n].quat_xyzw).tolist() for n in PANELS],
            "handle_slider_m": st["checked_suitcase"].sliders["carry_handle"],
            "robots": bodies,
            "info": {k: (float(v) if isinstance(v, (int, float, np.floating, bool)) else str(v))
                     for k, v in self.last_info.items()},
            "closure": (self.emb.task.module.closure_diagnostics()
                        if hasattr(self.emb.task.module, 'closure_diagnostics') else {}),
        }

    def snap(self, tag):
        import imageio.v2 as imageio

        for cam in ("top_cam", "left_cam", "right_cam"):
            imageio.imwrite(self.out / f"{tag}_{cam}.png", np.asarray(self.obs.images[cam]))

    def go(self, steps, left, right, *, every=5, snaps=(), video_every=0, hook=None):
        """Advance ``steps`` control steps; left/right map local time (s) -> (action7, label)."""
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        import imageio.v2 as imageio

        start = self.step_i
        for k in range(steps):
            t = (k + 1) / 20
            la, ll = left(t)
            ra, rl = right(t)
            res = self.emb.step(Action(data=pack(la, ra)))
            self.obs = res.observation
            self.last_info = dict(res.info or {})
            self.step_i += 1
            if self.step_i % every == 0 or k == steps - 1:
                self.trace.append(self.state((ll, rl)))
            if hook is not None:
                hook(self, k)
            if self.step_i in snaps:
                self.snap(f"step_{self.step_i:04d}")
            if video_every and self.step_i % video_every == 0:
                vdir = self.out / "frames"
                vdir.mkdir(exist_ok=True)
                imageio.imwrite(vdir / f"top_{self.step_i:05d}.png", np.asarray(self.obs.images["top_cam"]))
                # Opt-in wrist close-up frames for the video inset (recording only).
                if os.environ.get('BAGTAG_VIDEO_WRIST'):
                    for cam in ("right_cam", "left_cam"):
                        imageio.imwrite(vdir / f"{cam.split('_')[0]}wrist_{self.step_i:05d}.png",
                                        np.asarray(self.obs.images[cam]))
        return self.step_i - start

    def save(self, summary):
        result = dict(summary)
        result["trace"] = self.trace
        (self.out / "result.json").write_text(json.dumps(result, indent=1) + "\n")
        print("RESULT", json.dumps({k: v for k, v in summary.items()}))

    def close(self):
        self.emb.close()


def segment(plan: Plan, t0: float):
    """Callable for a plan that starts ``t0`` seconds into a phase (holds READY before)."""
    def f(t):
        if t < t0:
            return np.r_[plan.q, plan.finger_start / FINGER_OPEN], "wait"
        return plan.action7(t - t0)
    return f
