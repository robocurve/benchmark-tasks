"""Embodiment ``yam_jack_stands``: the stock bimanual YAM station with a jack-and-stands trainer.

Robot, cameras, 14-D joint action and observations are the stock adapter's (``yam_arena``), unchanged. This module
adds the trainer's bodies and three native joints:

    saddle  prismatic (z) between the world and the jack saddle, with a stiff position drive whose target is the
            jack's hydraulics (design.Jack): pump strokes raise it, the open release valve lowers it
    lever   revolute (y) between the world and the pump lever, limits -5..25 degrees, a spring drive holding it up
    knob    revolute (z) between the world and the release knob, limits 0..90 degrees, a little damping

The beam, the stands and the wrench are loose rigid bodies: the saddle lifts the beam and the stands carry it only
through contact. Scoring reads measured part poses and the robot's measured joints (``scoring.JackStandsScore``).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from . import design as d
from .mechanism import MechanismEmbodiment
from .scoring import JackStandsScore

TASK_FILE = Path(__file__).resolve().parents[1] / "jack_stands_task.py"
PAD_PAST_TCP = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP
WRIST_BEHIND_TCP = 0.09  # a wrist point this far back from the tool point along the fingers (the housing's top is 7 cm)
GAP_OPEN = 0.0939  # finger gap (m) fully open


class JackStandsEmbodiment(MechanismEmbodiment):
    ENV_NAME = "yam_jack_stands"
    TASK_FILE = TASK_FILE

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.jack = d.Jack()
        self.scorer = None
        self._close_axis = None

    def builders(self):
        from .assets import BUILDERS

        return BUILDERS

    def layout(self):
        return d.start_poses()

    def make_joints(self, js, origin):
        s0 = self.start["saddle"][:3] + origin
        js.prismatic("saddle", None, "saddle", s0, axis="Z", limits_m=(0.0, d.JACK_TRAVEL))
        js.drag("saddle", d.SADDLE_C, kind="linear", stiffness=d.SADDLE_K, target=0.0, max_force=d.SADDLE_FMAX)
        p0 = self.start["lever"][:3] + origin
        js.revolute("lever", None, "lever", p0, axis="Y", limits_deg=(d.LEVER_DOWN_DEG, d.LEVER_UP_DEG))
        js.drag("lever", d.LEVER_C, stiffness=d.LEVER_K, target=d.LEVER_SPRING_TARGET_DEG)
        k0 = self.start["knob"][:3] + origin
        js.revolute("knob", None, "knob", k0, axis="Z", limits_deg=d.KNOB_LIMIT_DEG)
        js.drag("knob", d.KNOB_C)

    # The saddle's drive target lives on its joint: put it back to the start, and clear the hydraulics, before the
    # joints are switched back on, so a second episode never starts with the saddle driven to the last one's height.
    def reset(self, scene, *, seed=None):
        self.jack.reset()
        if self.js is not None:
            self._set_saddle(0.0)
        out = super().reset(scene, seed=seed)
        return out

    def _ensure_sim(self, goal):
        super()._ensure_sim(goal)
        self._set_saddle(0.0)

    def _set_saddle(self, target: float) -> None:
        self.js.drives["saddle"].GetTargetPositionAttr().Set(float(target))

    def on_reset(self):
        self.jack.reset()
        self._set_saddle(0.0)
        self.scorer = JackStandsScore(self.info.control_hz)
        self.peak_sag_mm = 0.0

    # -- measured state -------------------------------------------------------------------------------------------
    def _fingers(self, opening, tcp_rot):
        """Each gripper's checked points (env frame): tool point, two finger tips (from the measured opening) and a
        wrist point 9 cm back along the fingers, so a tilted forearm can't reach under the beam's edge unseen."""
        from yam_arena.demo import grip_rotation

        if self._close_axis is None:  # the TCP-frame axis the fingers close along: world y for a yaw-0 grip
            r0 = grip_rotation(0.0)
            self._close_axis = int(np.argmax(np.abs(r0[1, :])))
            self._approach_axis = 2
        out = []
        for g, (p, r) in zip(opening, tcp_rot):
            c, a = r[:, self._close_axis], r[:, self._approach_axis]
            tip = p + a * PAD_PAST_TCP
            half = 0.5 * GAP_OPEN * g
            out.append([p, tip + c * half, tip - c * half, p - a * WRIST_BEHIND_TCP])
        return out

    def _grippers_full(self):
        from yam_arena import ik, station
        from yam_arena.constants import FINGER_OPEN
        from yam_arena.inspect_embodiment import ARMS

        opening, poses = [], []
        for name, side in zip(ARMS, ("left", "right")):
            arm_ids, finger_ids = self._ids[name]
            q = self._base.scene[name].data.joint_pos.torch[0]
            opening.append(float(np.clip(q[finger_ids].mean().item() / FINGER_OPEN, 0.0, 1.0)))
            poses.append(ik.tcp_pose(q[arm_ids].cpu().numpy().astype(float), station.BASES[side]))
        return opening, poses

    def state(self, p=None) -> dict:
        """Measured trainer state (scorer, telemetry and the privileged demo)."""
        p = self.poses() if p is None else p
        load, saddle, wrench = p["load"], p["saddle"], p["wrench"]
        lv = self.velocity("load")
        lever_deg = math.degrees(d.pitch_y_of(p["lever"][3:]))
        knob_deg = math.degrees(d.yaw_of(p["knob"][3:]))
        under_lift = d.underside_z(load, 0.0, d.LIFT_Y)
        saddle_top = float(saddle[2] + d.SADDLE[2] / 2)
        stands = {}
        for n in d.STANDS:
            st = p[n]
            top = d.stand_top(st)
            lx, ly, _ = d.to_local(load, top)
            gap = d.underside_z(load, lx, ly) - top[2]
            sv = self.velocity(n)
            stands[n] = {
                "under": bool(abs(lx) <= d.UNDER_X_TOL and abs(ly - d.STAND_SIDE[n] * d.SUPPORT_Y) <= d.UNDER_Y_TOL),
                "upright": bool(math.degrees(d.tilt_of(st[3:])) <= d.STAND_TILT_DEG
                                and abs(st[2] - d.TOP) <= d.STAND_ON_TABLE),
                "gap_mm": gap * 1000.0, "still": bool(np.linalg.norm(sv[:3]) < 0.01),
                "offset_mm": [lx * 1000.0, (ly - d.STAND_SIDE[n] * d.SUPPORT_Y) * 1000.0],
            }
        wv = self.velocity("wrench")
        return {
            "lever_deg": lever_deg, "knob_deg": knob_deg, "jack_target_mm": self.jack.target * 1000.0,
            "strokes": self.jack.strokes, "saddle_rise_mm": (saddle[2] - self.start["saddle"][2]) * 1000.0,
            "under_lift_mm": (under_lift - d.TOP) * 1000.0,
            "raised_mm": (under_lift - d.TOP - d.STAND_H) * 1000.0,
            "carry_gap_mm": (under_lift - saddle_top) * 1000.0,
            "tilt_deg": math.degrees(d.tilt_of(load[3:])),
            "load_still": bool(np.linalg.norm(lv[:3]) < 0.01 and np.linalg.norm(lv[3:]) < 0.05),
            "load_vz": float(lv[2]),
            "load_shift_m": float(np.linalg.norm(load[:2] - self.start["load"][:2])),
            "stands": stands,
            "part_moved_m": float(np.linalg.norm(wrench[:2] - self.start["wrench"][:2])),
            "part_clear_m": d.wrench_clear(load, wrench),
            "part_still": bool(np.linalg.norm(wv[:3]) < 0.02),
            "load": load, "wrench": wrench,
        }

    def measure(self):
        p = self.poses()
        s = self.state(p)
        opening, tcp_rot = self._grippers_full()
        points = self._fingers(opening, tcp_rot)
        in_zone = [any(d.in_zone(p["load"], q) for q in pts) for pts in points]
        # drive the jack: hydraulics from the measured lever and knob angles
        target = self.jack.update(s["lever_deg"], s["knob_deg"], 1.0 / self.info.control_hz)
        self._set_saddle(target)
        st = [s["stands"][n] for n in d.STANDS]
        score = self.scorer.update(
            raised_mm=s["raised_mm"], carry_gap_mm=s["carry_gap_mm"], under=[x["under"] for x in st],
            upright=[x["upright"] for x in st], stand_gap_mm=[x["gap_mm"] for x in st],
            stand_still=[x["still"] for x in st], tilt_deg=s["tilt_deg"], load_still=s["load_still"],
            load_vz=s["load_vz"], load_shift_m=s["load_shift_m"], hand_in_zone=any(in_zone),
            part_xy=(float(p["wrench"][0]), float(p["wrench"][1])), part_clear_m=s["part_clear_m"],
            part_still=s["part_still"])
        info = {k: v for k, v in s.items() if k not in ("load", "wrench", "stands")}
        info.update({f"{n}_{k}": v for n, x in s["stands"].items() for k, v in x.items()})
        info.update({"grip_open": opening, "tcp": [t[0] for t in tcp_rot], "hand_in_zone": in_zone,
                     "load_pose": p["load"], "wrench_pose": p["wrench"], "score": score})
        info.update(self.scorer.info())
        if round(self._t * self.info.control_hz) % round(self.info.control_hz / 2) == 0:
            sl, sr = s["stands"]["stand_left"], s["stands"]["stand_right"]
            print(f"[jack] t={self._t:6.2f} lever={s['lever_deg']:+5.1f} knob={s['knob_deg']:5.1f} "
                  f"strokes={s['strokes']:2d} tgt={s['jack_target_mm']:5.1f} under={s['under_lift_mm']:6.1f} "
                  f"carry={s['carry_gap_mm']:+6.1f} tilt={s['tilt_deg']:4.2f} "
                  f"L[{int(sl['under'])}{int(sl['upright'])} gap={sl['gap_mm']:+6.1f}] "
                  f"R[{int(sr['under'])}{int(sr['upright'])} gap={sr['gap_mm']:+6.1f}] "
                  f"part_moved={s['part_moved_m'] * 100:4.1f}cm clear={s['part_clear_m'] * 100:+5.1f}cm "
                  f"zone={int(in_zone[0])}{int(in_zone[1])} score={score:.2f} {self.scorer.reason}", flush=True)
        return info, self.scorer.success
