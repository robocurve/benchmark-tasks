"""Embodiment ``yam_lathe_centre_height``: the stock bimanual YAM station with a lathe tool to set to centre height.

Robot, cameras, 14-D joint action and observations are the stock adapter's (``yam_arena``), unchanged. This module
adds the lathe's bodies and native joints:

    xcrank  revolute (z) between the world and the cross-feed crank, with bearing drag
    xknob   revolute (z) between the cross-feed crank arm and its knob (spins freely, so a gripper keeps its yaw)
    zcrank  revolute (z) between the world and the height crank (on the tool holder's top), with bearing drag
    zknob   revolute (z) between the height crank arm and its knob
    xslide  prismatic (x) between the world and the carriage: a soft spring drive whose target is set every control
            step from the cross-feed crank's measured clockwise turns (0.100 in per turn), so the cutting edge pushed
            onto the steel rule pinches it lightly against the bar
    zslide  prismatic (z) between the carriage and the tool: a stiff spring drive whose target is set every step from
            the height crank's measured clockwise turns (24 threads per inch, lowering)

The cranks drive the slides one way (a screw is not back-drivable): pushing the tool does not turn a crank.
Scoring reads the tool's measured pose and the robot's measured joints (``scoring.CentreHeightScore``).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from . import design as d
from .mechanism import MechanismEmbodiment
from .scoring import CentreHeightScore

TASK_FILE = Path(__file__).resolve().parents[1] / "lathe_centre_height_task.py"
STILL_DPS = 2.0
STILL_MPS = 0.002


class LatheCentreHeightEmbodiment(MechanismEmbodiment):
    ENV_NAME = "yam_lathe_centre_height"
    TASK_FILE = TASK_FILE

    def __init__(self, debug_fingers: int = 0, **kwargs):
        super().__init__(**kwargs)
        self.debug_fingers = bool(int(debug_fingers))  # diagnostic only, off for every submitted run
        self.scorer = None
        self.z_bias = 0.0
        self._unwrapped = {"x": 0.0, "z": 0.0}
        self._prev = {"x": None, "z": None}

    def builders(self):
        from .assets import BUILDERS

        return BUILDERS

    def layout(self):
        return d.start_poses()

    def make_joints(self, js, origin):
        for pre in ("x", "z"):
            ax = self.start[f"{pre}crank"][:3] + origin
            js.revolute(f"{pre}crank", None, f"{pre}crank", ax, axis="Z")
            js.drag(f"{pre}crank", d.CRANK_DAMPING)
            js.revolute(f"{pre}knob", f"{pre}crank", f"{pre}knob", (d.CRANK_ARM_R, 0.0, d.KNOB_DZ), axis="Z")
        car = self.start["carriage"][:3] + origin
        js.prismatic("xslide", None, "carriage", car, axis="X", limits_m=d.X_TRAVEL)
        js.drag("xslide", d.X_DAMP, kind="linear", stiffness=d.X_K, target=0.0)
        js.prismatic("zslide", "carriage", "tool", (0.0, 0.0, 0.0), axis="Z", limits_m=d.Z_TRAVEL)
        js.drag("zslide", d.Z_DAMP, kind="linear", stiffness=d.Z_K, target=0.0)

    def on_reset(self):
        self.scorer = CentreHeightScore(self.info.control_hz, d.E0, d.FULL_M)
        self._unwrapped = {"x": 0.0, "z": 0.0}
        self._prev = {k: d.START_DEG[k] for k in ("x", "z")}
        self.peak = {k: 0.0 for k in d.check_layout(self.start)}
        # gravity sag of the height slide under its spring: measured once after the settle, then fed forward
        self.z_bias = 0.0
        self._set_targets()
        # let the height slide reach its start target before reading the sag (the drive still held the last
        # episode's target until now; reading at once carried that episode's turns into the next: +1.23 mm)
        for _ in range(max(1, round(0.5 / self._base.step_dt))):
            self._env.step(self._ready)
        self.z_bias = float(d.edge_z0() - self.poses()["tool"][2])
        print(f"[lathe] tool height sag {self.z_bias * 1000:+.4f} mm, fed forward", flush=True)
        self._set_targets()
        for _ in range(max(1, round(0.5 / self._base.step_dt))):
            self._env.step(self._ready)
        self.state_cached = self.state()
        # credit runs from the start error actually measured (the feed-forward leaves ~0.01 mm), so standing still
        # scores exactly 0
        e0 = max(abs(self.state_cached["err_mm"]) / 1000.0, d.FULL_M + 1e-4)
        self.scorer = CentreHeightScore(self.info.control_hz, e0, d.FULL_M)
        print(f"[lathe] start edge error {e0 * 1000:.4f} mm (credit 0 here, 1.0 at {d.FULL_M * 1000:.2f} mm)", flush=True)

    # -- measured state -----------------------------------------------------------------------------
    def state(self, p=None) -> dict:
        p = self.poses() if p is None else p
        for pre in ("x", "z"):
            a = math.degrees(d.yaw_of(p[f"{pre}crank"][3:]))
            if self._prev[pre] is not None:
                self._unwrapped[pre] += d.wrap_deg(a - self._prev[pre])
            self._prev[pre] = a
        tool, rule = p["tool"], p["rule"]
        return {"xcrank_deg": d.START_DEG["x"] + self._unwrapped["x"],
                "zcrank_deg": d.START_DEG["z"] + self._unwrapped["z"],
                "x_turns_cw": -self._unwrapped["x"] / 360.0, "z_turns_cw": -self._unwrapped["z"] / 360.0,
                "xcrank_dps": math.degrees(float(self.velocity("xcrank")[5])),
                "zcrank_dps": math.degrees(float(self.velocity("zcrank")[5])),
                "tool_x": float(tool[0]), "advance_mm": (float(tool[0]) - d.TIP_X0) * 1000.0,
                "err_mm": d.height_error(tool) * 1000.0, "tool_speed": float(np.linalg.norm(self.velocity("tool")[:3])),
                "rule_pos": rule[:3], "rule_quat": rule[3:], "rule_lean_deg": math.degrees(d.pitch_of(rule[3:])),
                "xknob": p["xknob"][:3], "zknob": p["zknob"][:3]}

    def _set_targets(self):
        xt = d.x_target(-self._unwrapped["x"] / 360.0)
        zt = d.z_target(-self._unwrapped["z"] / 360.0)
        self.js.drives["xslide"].GetTargetPositionAttr().Set(float(xt))
        self.js.drives["zslide"].GetTargetPositionAttr().Set(float(zt + self.z_bias))
        return xt, zt

    def measure(self):
        p = self.poses()
        s = self.state(p)
        self.state_cached = s
        xt, zt = self._set_targets()
        opening, tcp = self.grippers()
        clear = [float(min(np.linalg.norm(t - s["xknob"]), np.linalg.norm(t - s["zknob"]))) for t in tcp]
        drift = d.check_layout(p)
        for k, v in drift.items():
            self.peak[k] = max(self.peak[k], v)
        still = (abs(s["xcrank_dps"]) < STILL_DPS and abs(s["zcrank_dps"]) < STILL_DPS
                 and s["tool_speed"] < STILL_MPS)
        score = self.scorer.update(s["err_mm"] / 1000.0, still, opening, clear)
        info = {k: s[k] for k in ("xcrank_deg", "zcrank_deg", "x_turns_cw", "z_turns_cw", "advance_mm", "err_mm",
                                  "rule_lean_deg")}
        info.update({"x_target_mm": xt * 1000.0, "z_target_mm": zt * 1000.0, "grip_open": opening,
                     "tcp_clear_m": clear, "rule_pos": s["rule_pos"], "rule_quat": s["rule_quat"],
                     "joint_error_peak_mm": {k: round(v * 1000, 3) for k, v in self.peak.items()}, "score": score})
        info.update(self.scorer.info())
        if round(self._t * self.info.control_hz) % round(self.info.control_hz / 2) == 0:
            print(f"[lathe] t={self._t:6.2f} xturns={s['x_turns_cw']:+.3f} adv={s['advance_mm']:+.3f}mm "
                  f"(tgt {xt * 1000:+.3f}) zturns={s['z_turns_cw']:+.3f} err={s['err_mm']:+.3f}mm (tgt "
                  f"{zt * 1000:+.3f}) lean={s['rule_lean_deg']:+.2f}deg rule_z={s['rule_pos'][2] - d.TOP:.3f} "
                  f"open={opening[0]:.2f}/{opening[1]:.2f} clear={clear[0]:.3f}/{clear[1]:.3f} "
                  f"err_peak_mm={max(self.peak.values()) * 1000:.2f} score={score:.3f} {self.scorer.reason}",
                  flush=True)
        if self.debug_fingers and any(abs(self._t - t) < 0.026 for t in (10.0, 30.0, 60.0, 61.5, 62.5)):
            self._log_fingers(s)
        return info, self.scorer.success

    def _log_fingers(self, s):
        """Diagnostic (-E debug_fingers=1): world positions of the right arm's finger bodies next to the rule."""
        art = self._base.scene["right_arm"] if "right_arm" in self._base.scene.keys() else None
        if art is None:
            from yam_arena.inspect_embodiment import ARMS

            art = self._base.scene[ARMS[1]]
        names = art.body_names
        pos = art.data.body_pos_w.torch[0].cpu().numpy() - self.origin()
        rows = [f"{n}={np.round(pos[i], 4).tolist()}" for i, n in enumerate(names) if "finger" in n.lower()
                or "tip" in n.lower() or "link_6" in n.lower() or "link6" in n.lower()]
        print(f"[lathe-debug] t={self._t:.2f} rule={np.round(s['rule_pos'], 4).tolist()} {' '.join(rows)}",
              flush=True)
