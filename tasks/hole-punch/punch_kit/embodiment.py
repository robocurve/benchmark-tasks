"""The hole-punch scene as a yam_arena embodiment: the stock robot, cameras and actions, plus the punch, its lever and
sprung head, the foam stand, the card and its two pre-scored discs."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from . import geometry as g
from .mechanism import MechanismEmbodiment
from .scoring import PunchScore

TASK_FILE = Path(__file__).resolve().parents[1] / "hole_punch.py"
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def _pose(xyz, q=IDENT):
    return np.r_[np.asarray(xyz, float), q]


class PunchEmbodiment(MechanismEmbodiment):
    ENV_NAME = "yam_hole_punch"
    TASK_FILE = TASK_FILE

    def builders(self):
        from .assets import BUILDERS
        return BUILDERS

    def layout(self):
        card = g.CARD_START
        return {
            "punch": _pose((0.0, 0.0, 0.0)),
            "stand": _pose((0.0, 0.0, 0.0)),
            "head": _pose((g.PIN_X, 0.0, g.HEAD_REST_TIP)),
            "lever": _pose(g.LEVER_PIVOT),
            "card": _pose(card),
            "slug0": _pose(card + g.slug_offset(0)),
            "slug1": _pose(card + g.slug_offset(1)),
        }

    def make_joints(self, js, origin):
        head0 = origin + np.array([g.PIN_X, 0.0, g.HEAD_REST_TIP])
        js.prismatic("head_slide", None, "head", head0, axis="Z", limits_m=(-g.STROKE, 0.0))
        js.drag("head_slide", g.SPRING_DAMP, kind="linear", stiffness=g.SPRING_K, target=g.SPRING_PRELOAD)
        js.revolute("lever_pivot", None, "lever", origin + g.LEVER_PIVOT, axis="Y", limits_deg=(-g.LEVER_LIMIT_DEG, 0.0))
        js.drag("lever_pivot", 1e-4)
        for k in (0, 1):
            js.fixed(f"disc{k}", "card", f"slug{k}", g.slug_offset(k))

    SENSED = ("punch", "stand", "head", "lever")

    def configure_robot(self, robot):
        """A contact sensor on the card, split by what it touches (diagnostics only; scoring does not read it)."""
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.utils.configclass import make_configclass
        sensor = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/card", update_period=0.0, history_length=1,
                                  filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/{n}" for n in self.SENSED])
        robot.scene_config = make_configclass("PunchSceneCfg", [("card_contact", ContactSensorCfg, sensor)],
                                              bases=(type(robot.scene_config),))()

    def on_reset(self):
        self.dt = float(self._base.step_dt)  # one control step (0.05 s at 20 Hz)
        self.step0 = int(self._base.common_step_counter)
        self.scorer = PunchScore(hold_s=1.0)

    def measure(self):
        p = self.poses()
        tip_z = float(p["head"][2])
        t_now = (int(self._base.common_step_counter) - self.step0) * self.dt  # sim time since the start layout
        score, newly = self.scorer.update(t_now, p["card"], tip_z, [p["slug0"][:3], p["slug1"][:3]])
        for k in newly:  # the punch has cut disc k free: switch off its joint
            self.js.joints[f"disc{k}"].GetJointEnabledAttr().Set(False)
        lever = p["lever"][3:]
        lever_deg = float(np.degrees(2 * np.arctan2(lever[1], lever[3])))
        info = {"score": score, "best": self.scorer.best, "success": self.scorer.success,
                "released": list(self.scorer.released), "released_at": list(self.scorer.released_at),
                "head_travel_mm": (tip_z - g.HEAD_REST_TIP) * 1000.0, "lever_deg": lever_deg,
                "card": p["card"], "slug0": p["slug0"][:3], "slug1": p["slug1"][:3]}
        try:
            data = self._base.scene["card_contact"].data
            fm = data.force_matrix_w
            fm = (fm.torch if hasattr(fm, "torch") else fm)[0].cpu().numpy().sum(axis=0)  # (filters, 3)
            net = data.net_forces_w
            net = (net.torch if hasattr(net, "torch") else net)[0].cpu().numpy().sum(axis=0)
            info["card_contact_N"] = {n: fm[i].round(3).tolist() for i, n in enumerate(self.SENSED)}
            info["card_net_N"] = net.round(3).tolist()
        except Exception as e:  # noqa: BLE001
            info["card_contact_err"] = str(e)[:120]
        opening, tcp = self.grippers()  # measured, from the arms' joint angles
        info.update({"tcp_left": tcp[0], "tcp_right": tcp[1], "open_left": opening[0], "open_right": opening[1]})
        return info, self.scorer.success
