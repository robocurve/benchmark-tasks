"""On the station table, on a white absorbent bench pad: a white assembly mould holding an open CR2032 steel can; a petri
dish (A) lined with paper with a zinc disc, a glass-fibre separator and a second zinc disc in a row; a second dish (B) with
a steel spacer, a wave spring and a cap (open side down); plastic battery tweezers standing tips down in their stand; an
80 uL micropipette hanging in its holder; and a tube of 2 M ZnSO4 electrolyte in a rack. A phone on a clamp arm looks
straight down at the mould.

The task: with the tweezers for every part, stack zinc, separator, 80 uL of electrolyte (by pipette, two hands), zinc,
spacer, wave spring and cap in the can, each flat and centred; press the cap down once with the other hand; put the
tweezers and the pipette back in their stands and let go. Scored by cellrules.py, following task.json's `scoring`.
"""

from __future__ import annotations

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import cellparts as cp  # noqa: E402
import cellrules as cr  # noqa: E402
import mechkit as m  # noqa: E402
import pipette as pp  # noqa: E402
import steptime  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

PHYSICS_HZ = 480  # thin parts: a 0.1 mm zinc disc between plastic tips
m.set_physics_hz(PHYSICS_HZ)

SCENE = "station"
TOP = TABLE_TOP[SCENE]
Z0 = TOP + cp.PAD[2]  # the bench pad's top
CAN_XY = (0.32, -0.06)
MOULD_Z = Z0 + cp.MOULD[2] - cp.MOULD_POCKET[1]  # the pocket's floor: the can's underside
FLOOR = MOULD_Z + cp.CAN_FLOOR
RIM = MOULD_Z + cp.CAN_H
DISH_A, DISH_B = (0.32, -0.17), (0.22, -0.13)
LINER = Z0 + cp.DISH_GLASS + cp.DISH_LINER  # the paper's top in a dish
STAND_XY = (0.28, 0.02)
HOLDER_XY = (0.40, 0.10)
RACK_XY = (0.24, -0.22)
PIP_HOME = (*HOLDER_XY, Z0 + cp.HOLDER_CRADLE_Z - cp.PIP_COLLAR_Z + 0.0003)
LIQUID_Z = Z0 + cp.TUBE_SEAT + cp.LIQUID_TOP
TUBE_R = cp.TUBE_ID / 2
SCORE_CAM_Z = 1.20  # the phone over the mould

OBJECTS = [
    cp.bench_pad("bench_pad", (0.28, -0.07, TOP)),
    cp.mould("mould", (*CAN_XY, Z0)),
    cp.can("can", (*CAN_XY, MOULD_Z)),
    cp.dish("dish_a", (*DISH_A, Z0)),
    cp.dish("dish_b", (*DISH_B, Z0)),
    cp.zinc("zinc_1", (DISH_A[0] - 0.025, DISH_A[1], LINER)),
    cp.separator("separator", (DISH_A[0], DISH_A[1], LINER)),
    cp.zinc("zinc_2", (DISH_A[0] + 0.025, DISH_A[1], LINER)),
    cp.spacer("spacer", (DISH_B[0] - 0.025, DISH_B[1], LINER)),
    cp.wave_spring("spring", (DISH_B[0], DISH_B[1], LINER)),
    cp.cap("cap", (DISH_B[0] + 0.025, DISH_B[1], LINER)),
    cp.tweezer_stand("tw_stand", (*STAND_XY, Z0)),
    cp.tweezers("tweezers", (*STAND_XY, Z0 + cp.TW_STAND[2] - cp.TW_SLOT[2] + 0.0003)),
    cp.pipette_holder("pip_holder", (*HOLDER_XY, Z0)),
    cp.pipette("pipette", PIP_HOME),
    cp.tube_rack("tube_rack", (*RACK_XY, Z0)),
    cp.phone_rig("score_cam", (*CAN_XY, TOP), SCORE_CAM_Z - TOP),
]
PARTS = cr.ORDER

LAYOUT = cr.Layout(top=TOP, axis=CAN_XY, floor=FLOOR, rim=RIM, stand=(*STAND_XY, Z0 + cp.TW_STAND[2]), holder=PIP_HOME)


# -- the trial's record ------------------------------------------------------------------------------------------------


class Trial:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.pip = pp.Pipetting()
        self.judge = cr.CellJudge(LAYOUT)
        self.t = 0.0
        self.tier, self.why = 0.0, "nothing done"
        self.report = {}
        self._wet_shown = None
        self._tier_shown = None

    def snapshot(self, env) -> cr.Snapshot:
        parts = {}
        for n in PARTS:
            s = m.state(env, n)
            parts[n] = cr.Part(np.asarray(s.pos, float), s.rotation())
        tips = m.fingertips(env)
        tw = m.state(env, "tweezers")
        pip = m.state(env, "pipette")
        tip_end = np.asarray(pip.pos, float)
        travel = -pip.joint("plunger")
        depth = None
        if np.linalg.norm(tip_end[:2] - np.asarray(RACK_XY)) < TUBE_R and tip_end[2] < LIQUID_Z + 0.01:
            depth = LIQUID_Z - tip_end[2]
        sep = parts["separator"]
        window = (tip_end[2] - (sep.pos[2] + sep.up[2] * cr.THICK["separator"]), float(np.linalg.norm(tip_end[:2] - np.asarray(CAN_XY))))
        sep_in = self.judge.in_can("separator", sep) and sep.tilt_deg() < 5.0
        self.pip.update(self.t, travel, depth, window, sep_in)
        c = parts["cap"]
        cap_top = c.pos + c.up * cr.CAP_H
        local = (tips[12:] - c.pos) @ c.rot
        pressing = bool(np.any((np.linalg.norm(local[:, :2], axis=1) < cr.DIAM["cap"] / 2) & (local[:, 2] > cr.CAP_H - 0.001) & (local[:, 2] < cr.CAP_H + 0.003)))
        return cr.Snapshot(
            t=self.t, parts=parts, left=tips[:12], right=tips[12:],
            tweezers=(tw.to_world(np.array([[0.0, 0.0, 0.0]]))[0], tw.to_world(np.array([[0.0, 0.0, cp.TW_L]]))[0]), tweezers_tilt=tw.tilt_deg(),
            pipette=(tip_end, pip.to_world(np.array([[0.0, 0.0, cp.PIP_BUTTON_Z]]))[0]), pipette_tilt=pip.tilt_deg(),
            can_g=self.pip.can_g, spilled_g=self.pip.spilled_g,
            dispense_times=tuple(d[0] for d in self.pip.dispenses if d[1] == "separator"), cap_pressing=pressing and cap_top[2] > 0)

    def step(self, env, dt: float) -> None:
        self.t += dt
        s = self.snapshot(env)
        self.judge.update(s)
        self.tier, self.why = self.judge.tier(s)
        j = self.judge
        self.report = {"tier": self.tier, "why": self.why, "arrived": dict(j.arrived), "electrolyte_g": round(self.pip.can_g, 4),
                       "spilled_g": round(self.pip.spilled_g, 4), "tip_uL": self.pip.tip_ul, "pressed": j.pressed,
                       "zinc_1_offset_mm": round(j.frame_offset("zinc_1") * 1000, 3) if "zinc_1" in j.frame else None,
                       "zinc_2_offset_mm": round(j.frame_offset("zinc_2") * 1000, 3) if "zinc_2" in j.frame else None,
                       "zinc_to_zinc_mm": round(j.zinc_distance() * 1000, 3) if "zinc_2" in j.frame else None,
                       "spacer_offset_mm": round(j.frame_offset("spacer") * 1000, 3) if "spacer" in j.frame else None,
                       "sink_mm": j.sinks_mm(s), "gauge_mm": cr.gauge_mm(s.parts["spacer"], j.lay.floor) if "spacer" in s.parts else None,
                       "separator_between_zincs": j.pre_press_sep if j.press_started else j.separated(s), "left_can": list(j.left_can),
                       "finger_travel_mm": {n: round(v * 1000, 3) for n, v in j.finger_travel.items()},
                       "peak_since_arrival_mm": {n: [round(v * 1000, 2) for v in hs] for n, hs in j.peak.items()},
                       "seated": j.seated(s), "stowed": j.tools_stowed(s), "gripper_moved": list(j.gripper_moved[:3]),
                       "tweezers_tips_mm": np.round((np.asarray(s.tweezers[0]) - np.array([*STAND_XY, Z0])) * 1000, 1).tolist(),
                       "tweezers_tilt": round(s.tweezers_tilt, 1), "pipette_mm": np.round((np.asarray(s.pipette[0]) - np.asarray(PIP_HOME)) * 1000, 1).tolist(),
                       "clear_s": None if j.clear_since is None else round(self.t - j.clear_since, 2)}
        if self.tier != self._tier_shown:  # the log: each change of tier, with the final sinks and peaks
            self._tier_shown = self.tier
            print(f"[task] t={self.t:.2f} tier {self.tier}: {self.why}; sink_mm {self.report['sink_mm']}; gauge_mm {self.report['gauge_mm']}; peaks {self.report['peak_since_arrival_mm']}", flush=True)
        if self._wet_shown != self.pip.wet:
            self._wet_shown = self.pip.wet
            try:
                m.set_visible(f"{m.ENV_PATH}/separator/visual/wet", self.pip.wet)
                m.set_visible(f"{m.ENV_PATH}/separator/visual/disc", not self.pip.wet)
            except RuntimeError as err:
                print(f"[task] wet look: {err}", flush=True)


TRIAL = Trial()


@m.on_reset
def _reset(env) -> None:
    TRIAL.reset()


@m.on_step
def _step(env) -> None:
    TRIAL.step(env, 1.0 / float(os.environ.get("TASK_CONTROL_HZ", "20")))
    if os.environ.get("TASK_INSET"):  # the demo video's overhead inset of the can (inset.py); not a policy camera
        import inset

        inset.start(m.ENV_PATH, CAN_XY, SCORE_CAM_Z - 0.005, RIM)  # just under the phone
        inset.step()


def score(objects) -> float:
    """task.json `scoring`: 1.0, 0.5, 0.25 or 0 (cellrules.CellJudge.tier)."""
    return TRIAL.tier


def success(objects) -> bool:
    steptime.tick()  # the harness calls success() once per control step
    return TRIAL.tier == 1.0


# -- the scripted demo (closed loop; reads the poses of the parts and tools, which the cameras see) --------------------

def demo_script(ctx):
    import celldemo

    yield from celldemo.script(ctx, __import__("types").SimpleNamespace(**{k: v for k, v in globals().items() if k.isupper()}))
