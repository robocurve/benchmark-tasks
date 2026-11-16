"""Embodiment ``yam_inspection_mirror``: the stock bimanual YAM station with a hidden-leak inspection.

Robot, cameras, 14-D joint action and observations are the stock adapter's (``yam_arena``), unchanged. This module
adds the fixed housing with its wall and cover strip, the leak trace on the hidden face (placed at reset at one of
five places, drawn from the episode seed), the Ullman HTC-2 inspection mirror and the paint marker (free tools), and
up to four paint dots. No joints. Measured each step, from the bodies' poses:

- the mirror head's clearance to the hidden face (touching it smears the trace);
- the marker tip: where it touches the cover strip, a paint mark stays (a dot is drawn there);
- the score (``scoring.LeakMarkScore``).

``-E leak=<0..4>`` fixes the leak place (for checks); by default it comes from the episode seed.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from . import design as d
from .mechanism import MechanismEmbodiment
from .scoring import LeakMarkScore

TASK_FILE = Path(__file__).resolve().parents[1] / "inspection_mirror_task.py"
MARK_GAP = 0.003  # a touch more than 3 mm from every earlier mark makes a new mark (a dragged tip leaves a streak)


class InspectionMirrorEmbodiment(MechanismEmbodiment):
    ENV_NAME = "yam_inspection_mirror"
    TASK_FILE = TASK_FILE

    def __init__(self, leak: int = -1, **kwargs):
        super().__init__(**kwargs)
        self.fixed_leak = int(leak)
        self.scorer = None
        self.leak_i = d.leak_index(None)

    def builders(self):
        from .assets import BUILDERS

        return BUILDERS

    def layout(self):
        return d.start_poses()

    def make_joints(self, js, origin):
        return None  # no joints: the housing is fixed and the mirror and marker are free tools

    def reset(self, scene, *, seed=None):
        self.leak_i = self.fixed_leak if 0 <= self.fixed_leak < len(d.LEAK_X) else d.leak_index(seed)
        self.start = d.start_poses()
        self.start["leak"] = d.leak_pose(self.leak_i)
        print(f"[inspect] reset seed={seed} leak place {self.leak_i} at x={d.LEAK_X[self.leak_i]:.3f}", flush=True)
        return super().reset(scene, seed=seed)

    def on_reset(self):
        self.scorer = LeakMarkScore(self.info.control_hz, d.LEAK_X[self.leak_i])
        self.marks: list[np.ndarray] = []
        self.smeared = False
        self.min_clear = 1.0
        self.state_cached = {"marks": [], "smeared": False}

    def _place_dot(self, i: int, p) -> None:
        torch, base = self._torch, self._base
        pose = torch.tensor([[float(p[0]), float(p[1]), d.TOP + d.CAP_TOP, 0.0, 0.0, 0.0, 1.0]],
                            dtype=torch.float32, device=base.device)
        pose[:, :3] += base.scene.env_origins
        base.scene[f"dot_{i}"].write_root_pose_to_sim_index(root_pose=pose)

    def measure(self):
        p = self.poses()
        clear = d.head_face_clearance(p["mirror"]) if d.head_in_gap(p["mirror"]) else 1.0
        self.min_clear = min(self.min_clear, clear)
        if clear <= d.TOUCH_MM / 1000.0:
            self.smeared = True
        tip = d.marker_tip(p["marker"])
        if d.on_strip(tip) and all(np.hypot(*(tip[:2] - m[:2])) > MARK_GAP for m in self.marks):
            if len(self.marks) < d.MARK_DOTS:
                self._place_dot(len(self.marks), tip)
            self.marks.append(tip.copy())
        speeds = [float(np.linalg.norm(self.velocity(n)[:3])) for n in ("mirror", "marker")]
        still = max(speeds) < d.REST_SPEED
        opening, _ = self.grippers()
        marks_x = [float(m[0]) for m in self.marks]
        score = self.scorer.update(marks_x, self.smeared, still, opening)
        head = d.head_center(p["mirror"])
        self.state_cached = {"marks": marks_x, "smeared": self.smeared}
        info = {"leak_place": self.leak_i, "leak_x": d.LEAK_X[self.leak_i], "marks_x": marks_x,
                "smeared": self.smeared, "head": head, "head_face_clear_mm": clear * 1000.0,
                "min_face_clear_mm": self.min_clear * 1000.0, "marker_tip": tip, "grip_open": opening,
                "score": score}
        info.update(self.scorer.info())
        if round(self._t * self.info.control_hz) % round(self.info.control_hz / 2) == 0:
            print(f"[inspect] t={self._t:6.2f} head={np.round(head - [0, 0, d.TOP], 3).tolist()} "
                  f"clear={clear * 1000:5.1f}mm tip={np.round(tip - [0, 0, d.TOP], 3).tolist()} "
                  f"marks={[round(m, 4) for m in marks_x]} smeared={int(self.smeared)} "
                  f"open={opening[0]:.2f}/{opening[1]:.2f} score={score:.3f} {self.scorer.reason}", flush=True)
        return info, self.scorer.success
