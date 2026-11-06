"""Embodiment ``yam_dividing_head``: the stock bimanual YAM station with a dividing head.

Robot, cameras, 14-D joint action and observations are the stock adapter's (``yam_arena``), unchanged. This module
adds the head's bodies and the native joints of its index crank:

    crank   revolute (z) between the world and the crank, with a declared bearing drag (damping only)
    slide   prismatic along the crank arm, between the 20-hole and the 21-hole circle radii (its limits), damped so
            it stays where it is put (the plunger's radial setting)
    pin     prismatic (z) between the slide and the plunger pin, with a spring drive pushing it down to its seated
            limit; the pin enters a hole only through contact with the plate's pockets
    knob    revolute (z) between the pin and the knob sleeve (spins freely, so a gripper keeps its yaw)

The workpiece spindle is kinematic: each control step its yaw is written from the crank's measured, unwrapped
rotation divided by 40, one way. Nothing the robot does to the workpiece can turn it (a worm drive is not
back-drivable). Scoring reads measured part poses and the robot's measured joints (``scoring.DivisionScore``).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from . import design as d
from .mechanism import MechanismEmbodiment
from .scoring import DivisionScore

TASK_FILE = Path(__file__).resolve().parents[1] / "dividing_head_task.py"
FORCED_DEG = 3.0  # crank travel with the pin seated that counts as forcing (slack in a pocket: about +-0.7 deg)
PIN_STILL_MPS = 0.01


class DividingHeadEmbodiment(MechanismEmbodiment):
    ENV_NAME = "yam_dividing_head"
    TASK_FILE = TASK_FILE

    def __init__(self, probe: str = "", **kwargs):
        # probe: diagnostic only (default off), not used for any submitted run. "lock": a torque-limited velocity drive
        # tries to turn the crank with the pin seated, then (after 6 s) with the pin lifted by its own drive.
        super().__init__(**kwargs)
        self.probe = str(probe)
        self.scorer = None
        self._crank_unwrapped = 0.0
        self._crank_prev = None
        self._spindle_prev = None
        self._spindle_unwrapped = 0.0

    def builders(self):
        from .assets import BUILDERS

        return BUILDERS

    def layout(self):
        return d.start_poses()

    def make_joints(self, js, origin):
        try:
            from pxr import PhysxSchema

            gear = [n for n in dir(PhysxSchema) if "Gear" in n]
        except Exception as e:  # noqa: BLE001 - diagnostic only
            gear = [f"error: {e}"]
        print(f"[dividing-head] PhysX gear joint classes in this build: {gear} (not used: a gear joint is a two-way "
              "constraint, so it could be back-driven; the spindle is kinematic instead)", flush=True)
        ax = self.start["rotor"][:3] + origin
        js.revolute("crank", None, "rotor", ax, axis="Z")
        js.drag("crank", d.CRANK_DAMPING)
        r20, r21 = d.CIRCLES[d.INNER_N], d.CIRCLES[d.OUTER_N]
        js.prismatic("slide", "rotor", "slide", (r20, 0.0, 0.0), axis="X", limits_m=(0.0, r21 - r20))
        js.drag("slide", d.SLIDE_DAMPING, kind="linear", stiffness=d.SLIDE_K, target=-d.SLIDE_PRELOAD)
        js.prismatic("pin", "slide", "pin", (0.0, 0.0, 0.0), axis="Z", limits_m=(0.0, d.PIN_LIFT_MAX))
        js.drag("pin", d.SPRING_DAMPING, kind="linear", stiffness=d.SPRING_K, target=-d.SPRING_PRELOAD)
        js.revolute("knob", "pin", "knob", (0.0, 0.0, d.KNOB_DZ), axis="Z")

    def on_reset(self):
        self.scorer = DivisionScore(self.info.control_hz, d.TARGET_DEG, d.TOL_DEG, d.TURNS_BAND_DEG, d.OUTER_N,
                                    FORCED_DEG)
        self._crank_unwrapped, self._crank_prev = 0.0, d.ZERO_DEG  # the start layout's crank angle and spindle
        self._spindle_unwrapped, self._spindle_prev = 0.0, 0.0
        self.peak = {k: 0.0 for k in d.check_layout(self.start)}
        self._probe_state = None
        self.state_cached = self.state()  # what the privileged demo reads before the first step

    # -- measured state -------------------------------------------------------------------------
    def state(self, p=None) -> dict:
        """Measured head state (scorer, telemetry and the privileged demo). Call once per control step."""
        p = self.poses() if p is None else p
        crank = math.degrees(d.yaw_of(p["rotor"][3:]))
        if self._crank_prev is not None:
            self._crank_unwrapped += d.wrap_deg(crank - self._crank_prev)
        self._crank_prev = crank
        work = math.degrees(d.yaw_of(p["work"][3:])) - d.POINTER_DEG
        if self._spindle_prev is not None:
            self._spindle_unwrapped += d.wrap_deg(work - self._spindle_prev)
        self._spindle_prev = work
        pin = p["pin"]
        rel = pin[:2] - d.CRANK_XY
        radius = float(np.hypot(*rel))
        circle = next((n for n, r in d.CIRCLES.items() if abs(radius - r) <= d.CIRCLE_TOL_M), None)
        lift = float(pin[2] - (d.TOP + d.PLATE_TOP))
        seated = lift <= d.SEATED_MAX_M
        pin_dir = math.degrees(math.atan2(rel[1], rel[0]))
        hole = None
        if circle is not None:
            hole = round(-d.wrap_deg(pin_dir - d.ZERO_DEG) / (360.0 / circle)) % circle
        return {"crank_deg": self._crank_unwrapped + d.ZERO_DEG, "crank_turns": self._crank_unwrapped / 360.0,
                "crank_dps": math.degrees(float(self.velocity("rotor")[5])),
                "spindle_deg": self._spindle_unwrapped, "radius_mm": radius * 1000.0, "circle": circle,
                "hole": hole, "pin_lift_mm": lift * 1000.0, "seated": bool(seated),
                "pin_speed": float(np.linalg.norm(self.velocity("pin")[:3])), "knob": p["knob"][:3]}

    def _drive_spindle(self):
        """Write the workpiece yaw from the crank's unwrapped rotation / 40 (one way: the spindle never reads back)."""
        torch, base = self._torch, self._base
        pose = d.work_pose(self._crank_unwrapped / d.RATIO)
        t = torch.tensor(pose[None], dtype=torch.float32, device=base.device)
        t[:, :3] += base.scene.env_origins
        base.scene["work"].write_root_pose_to_sim_index(root_pose=t)
        base.scene["work"].write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))

    def measure(self):
        p = self.poses()
        s = self.state(p)
        self.state_cached = s
        self._drive_spindle()
        opening, tcp = self.grippers()
        clear = [float(np.linalg.norm(t - s["knob"])) for t in tcp]
        drift = d.check_layout(p)
        for k, v in drift.items():
            self.peak[k] = max(self.peak[k], v)
        cmd_spindle = self._crank_unwrapped / d.RATIO
        info = {k: s[k] for k in ("crank_deg", "crank_turns", "crank_dps", "spindle_deg", "radius_mm", "circle",
                                  "hole", "pin_lift_mm", "seated")}
        info.update({"spindle_cmd_deg": cmd_spindle, "grip_open": opening, "tcp_clear_m": clear,
                     "joint_error_mm": {k: round(v * 1000, 3) for k, v in drift.items()},
                     "joint_error_peak_mm": {k: round(v * 1000, 3) for k, v in self.peak.items()}})
        still = abs(s["crank_dps"]) < 1.0 and s["pin_speed"] < PIN_STILL_MPS
        score = self.scorer.update(s["spindle_deg"], s["crank_deg"], s["circle"], s["seated"],
                                   s["crank_dps"] if still else 99.0, opening, clear)
        info["score"] = score
        info.update(self.scorer.info())
        if self.probe:
            self._probe_drive(s)
        if round(self._t * self.info.control_hz) % round(self.info.control_hz / 2) == 0:
            print(f"[dividing-head] t={self._t:6.2f} crank={s['crank_deg']:+9.2f} turns={s['crank_turns']:+6.3f} "
                  f"dps={s['crank_dps']:+6.1f} spindle={s['spindle_deg']:+8.3f} (cmd {cmd_spindle:+8.3f}) "
                  f"r={s['radius_mm']:5.1f}mm circle={s['circle']} hole={s['hole']} lift={s['pin_lift_mm']:+5.2f}mm "
                  f"seated={int(s['seated'])} open={opening[0]:.2f}/{opening[1]:.2f} clear={clear[0]:.3f}/"
                  f"{clear[1]:.3f} err_peak_mm={max(self.peak.values()) * 1000:.3f} score={score:.2f} "
                  f"{self.scorer.reason}", flush=True)
        return info, self.scorer.success

    def _probe_drive(self, s):
        """Diagnostic (probe runs only). 0-6 s: a velocity drive on the crank (30 deg/s target, 0.01 N m per deg/s:
        0.3 N m on a stalled crank, capped at 0.5 N m) pushes it with the pin seated. 6-14 s: the pin's own drive
        lifts it (target above the upper limit) while the crank drive keeps turning it. 14-22 s: the crank drive slows
        to 8 deg/s and the pin drive returns to its spring setting, so the pin drops into the next hole it meets.
        22 s: the crank drive stops."""
        crank, pin = self.js.drives["crank"], self.js.drives["pin"]
        if self._probe_state is None:
            self._probe_state = "push"
            crank.GetDampingAttr().Set(0.01)
            crank.GetMaxForceAttr().Set(0.5) if crank.GetMaxForceAttr() else crank.CreateMaxForceAttr(0.5)
            crank.GetTargetVelocityAttr().Set(-30.0)
        if self._t >= 6.0 and self._probe_state == "push":
            self._probe_state = "lift"
            print(f"[probe] seated push: crank moved {s['crank_deg'] - d.ZERO_DEG:+.3f} deg", flush=True)
            pin.GetTargetPositionAttr().Set(0.03)
        if self._t >= 14.0 and self._probe_state == "lift":
            self._probe_state = "drop"
            crank.GetTargetVelocityAttr().Set(-8.0)
            pin.GetTargetPositionAttr().Set(-d.SPRING_PRELOAD)
        if self._t >= 22.0 and self._probe_state == "drop":
            self._probe_state = "done"
            crank.GetTargetVelocityAttr().Set(0.0)
            crank.GetDampingAttr().Set(float(d.CRANK_DAMPING))
            print(f"[probe] after drop: crank {s['crank_deg']:+.3f} hole {s['hole']} seated {s['seated']} "
                  f"lift {s['pin_lift_mm']:+.2f} mm", flush=True)
