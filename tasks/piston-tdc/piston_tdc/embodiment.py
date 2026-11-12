"""Embodiment ``yam_piston_tdc``: the stock bimanual YAM station with a piston-stop engine trainer.

Robot, cameras, 14-D joint action and observations are the stock adapter's (``yam_arena``), unchanged. This module
adds the trainer's bodies and the native joints of its closed slider-crank chain:

    crank     revolute (z) between the world and the crankshaft, with a declared bearing drag (damping only)
    big end   revolute (z) between the crank pin and the connecting rod
    wrist pin spherical between the rod's small end and the piston (removes the loop's redundant constraints)
    bore      prismatic (y) between the world and the piston
    sleeve    revolute (z) between the handle pin and the handle sleeve

The stop is a loose rigid body; the piston meets it through contact only, and the cylinder head wall behind it
carries the load. Scoring reads measured part poses and the robot's measured joints (``scoring.TdcScore``).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from . import design as d
from .mechanism import MechanismEmbodiment
from .scoring import TdcScore

TASK_FILE = Path(__file__).resolve().parents[1] / "piston_tdc_task.py"
SEATED_TOL_M, SEATED_TILT_DEG = 0.002, 3.0
TRAY_MAX_Z = 0.08  # stop centre within this of the tray floor counts as in the tray (not held above it)
TRAY_STILL_MPS = 0.05


class PistonTdcEmbodiment(MechanismEmbodiment):
    ENV_NAME = "yam_piston_tdc"
    TASK_FILE = TASK_FILE

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.scorer = None
        self._unwrapped = None

    def builders(self):
        from .assets import BUILDERS

        return BUILDERS

    def layout(self):
        return d.start_poses()

    def make_joints(self, js, origin):
        ax = d.axis_point() + origin
        js.revolute("crank", None, "rotor", ax, axis="Z")
        js.drag("crank", d.CRANK_DAMPING)
        js.revolute("big_end", "rotor", "rod", (d.R, 0.0, 0.0), axis="Z")
        js.spherical("wrist_pin", "rod", "piston", (d.L, 0.0, 0.0))
        w0 = self.start["piston"][:3] + origin
        js.prismatic("bore", None, "piston", w0, axis="Y")
        js.revolute("sleeve", "rotor", "handle", d.sleeve_local(), axis="Z")

    def on_reset(self):
        self.scorer = TdcScore(self.info.control_hz, d.CONTACT_DEG, d.SIDE_TOL_DEG, d.FORCED_DEG, d.TDC_TOL_DEG)
        self._unwrapped = None
        self.peak = {k: 0.0 for k in d.check_layout(self.start)}

    # -- measured state -------------------------------------------------------------------------
    def state(self, p=None) -> dict:
        """Measured trainer state (scorer and telemetry; the demo policy does not read it)."""
        p = self.poses() if p is None else p
        theta = math.degrees(d.theta_of_phi(d.yaw_of(p["rotor"][3:])))
        if self._unwrapped is None:
            self._unwrapped = theta
        else:
            self._unwrapped += d.wrap_deg(theta - d.wrap_deg(self._unwrapped))
        omega = math.degrees(float(self.velocity("rotor")[5]))
        stop = p["stop"]
        seat = d.stop_seated_pos()
        seated = bool(np.linalg.norm(stop[:3] - seat) <= SEATED_TOL_M and
                      math.degrees(d.tilt_of(stop[3:])) <= SEATED_TILT_DEG)
        lo, hi = d.tray_inner_bounds()
        stop_v = float(np.linalg.norm(self.velocity("stop")[:3]))
        in_tray = bool(np.all(stop[:2] >= lo) and np.all(stop[:2] <= hi)
                       and stop[2] - (d.TOP + d.TRAY_FLOOR) <= TRAY_MAX_Z and stop_v < TRAY_STILL_MPS)
        crown_s = float(np.dot(p["piston"][:2] - d.CRANK_XY, d.U)) + d.CROWN
        face_s = float(np.dot(stop[:2] - d.CRANK_XY, d.U)) - d.STOP_T / 2
        return {"theta_deg": theta, "theta_unwrapped_deg": self._unwrapped, "omega_dps": omega,
                "stop_seated": seated, "stop_in_tray": in_tray, "stop_pos": stop[:3],
                "piston_stop_gap_mm": (face_s - crown_s) * 1000.0, "handle": p["handle"][:3]}

    def measure(self):
        p = self.poses()
        s = self.state(p)
        opening, tcp = self.grippers()
        clear = [float(np.linalg.norm(t - s["handle"])) for t in tcp]
        drift = d.check_layout(p)
        for k, v in drift.items():
            self.peak[k] = max(self.peak[k], v)
        info = {"theta_deg": s["theta_deg"], "theta_unwrapped_deg": s["theta_unwrapped_deg"],
                "omega_dps": s["omega_dps"], "stop_seated": s["stop_seated"], "stop_in_tray": s["stop_in_tray"],
                "piston_stop_gap_mm": s["piston_stop_gap_mm"], "grip_open": opening, "tcp_clear_m": clear,
                "joint_error_mm": {k: round(v * 1000, 3) for k, v in drift.items()},
                "joint_error_peak_mm": {k: round(v * 1000, 3) for k, v in self.peak.items()}}
        score = self.scorer.update(s["theta_deg"], s["omega_dps"], s["stop_seated"], s["stop_in_tray"], opening, clear,
                                   s["piston_stop_gap_mm"])
        info["score"] = score
        info.update(self.scorer.info())
        if self.telemetry and round(self._t * self.info.control_hz) % round(self.info.control_hz / 2) == 0:
            print(f"[piston] t={self._t:6.2f} theta={s['theta_deg']:+8.2f} unwrapped={s['theta_unwrapped_deg']:+8.2f} "
                  f"omega={s['omega_dps']:+7.1f} gap={s['piston_stop_gap_mm']:+6.2f}mm seated={int(s['stop_seated'])} "
                  f"tray={int(s['stop_in_tray'])} open={opening[0]:.2f}/{opening[1]:.2f} "
                  f"clear={clear[0]:.3f}/{clear[1]:.3f} err_peak_mm={max(self.peak.values()) * 1000:.3f} "
                  f"score={score:.2f} {self.scorer.reason}", flush=True)
        return info, self.scorer.success

