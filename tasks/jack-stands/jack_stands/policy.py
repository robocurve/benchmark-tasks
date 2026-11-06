"""Policy ``jack_stands_scripted``: a privileged scripted demonstration (reads the trainer's measured state).

The procedure is the floor-jack manual's: pump the jack until the beam clears the stands, set both stands under
the support points, open the release so the beam settles onto the stands and the saddle drops clear, and only then
reach in for the work.

    left arm   presses the green pump pad down and lets it spring up, stroke after stroke, until the measured
               underside is 2 cm above the stands' tops; slides the left stand forward by its grip block; at the end
               pulls the wrench out by its grip block
    right arm  slides the right stand forward (at the same time as the left); grips the release knob's wing and
               turns it 60 degrees counterclockwise; holds it until the beam rests on both stands and the saddle is
               clear; lets go

It outputs ordinary 14-D joint targets through the stock IK; it never sets a part's pose. It reads the measured
underside height (to stop pumping) and the stand gaps (to let go of the knob), the way a mechanic watches the load.

``-P mode=demo`` (default) is the demonstration. Failure paths (for the README's scoring checks):
``-P mode=reach``     pumps the beam up, then reaches in for the wrench while the beam is on the jack only;
``-P mode=nostands``  pumps the beam up, then opens the release without setting the stands (lowers it back down);
``-P mode=idle``      keeps both arms at the ready pose.
"""

from __future__ import annotations

import math

import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import grip_rotation
from yam_arena.scripted import YamScriptedPolicy

from . import design as d
from .motion import Arm, Move

TOP = d.TOP
CLOSED = 0.0
HALF_OPEN = -0.020  # 40 mm finger gap, to keep the fingers clear of the neighbouring parts
BLOCK_SQUEEZE = CLOSED  # fingers driven fully closed on the 20 mm grip blocks: the full grip force (dragging a stand takes ~5 N)
WING_SQUEEZE = -0.0035  # 7 mm gap on the 10 mm knob wing
ABOVE_Z = 0.13  # travel height for the tool point above the table
RAISE_GOAL_MM = 20.0  # pump until the underside is this far above the stands' tops
MAX_STROKES = 16
KNOB_TURN_DEG = 60.0
PUMP_X = 0.195  # over the pad's centre through its stroke (19.0 .. 20.1 cm)
WRENCH_PULL = 0.10
STAND_GRIP_Z = d.STAND_SKID[1][2] + 0.025
SLIDE_MPS = 0.03  # commanded slide speed
SLIDE_GAIN, SLIDE_BIAS_MAX = 1.5, 0.04  # measured-position correction: the compliant arm lags the dragged stand
SLIDE_TOL, SLIDE_SETTLE_S, SLIDE_TIMEOUT_S = 0.003, 0.5, 6.0


def _p(x, y, z):
    return np.array([x, y, TOP + z], float)


def _at(pos, yaw=0.0):
    rot = grip_rotation(math.radians(yaw))
    pos = np.asarray(pos, float)
    return lambda sim, pos=pos, rot=rot: (pos, rot)


class JackStandsScriptedPolicy(YamScriptedPolicy):
    def __init__(self, mode: str = "demo", **kwargs):
        super().__init__(**kwargs)
        from dataclasses import replace

        try:
            self.info = replace(self.info, name="jack_stands_scripted")
        except Exception:  # noqa: BLE001 - the name is cosmetic
            pass
        self.mode = str(mode)
        assert self.mode in ("demo", "reach", "nostands", "idle"), self.mode

    def reset(self, scene):
        self._built = False
        self._steps: list = []
        self._end = None

    # -- plan -----------------------------------------------------------------------------------------------------
    def _park(self, arm):
        pose = self.park[arm]
        return lambda sim: pose

    def _stand_grip_moves(self, arm, side):
        (gx, _), grip = d.STAND_GRIP
        y = side * d.SUPPORT_Y
        x0 = d.STAND_START_X + gx
        return [Move(2.5, _at(_p(x0, y, ABOVE_Z)), FINGER_OPEN, f"{arm} stand approach"),
                Move(1.2, _at(_p(x0, y, STAND_GRIP_Z)), FINGER_OPEN, f"{arm} stand descend"),
                Move(0.6, _at(_p(x0, y, STAND_GRIP_Z)), BLOCK_SQUEEZE, f"{arm} stand grip")]

    def _stand_release_moves(self, arm, x, y):
        return [Move(0.5, _at(_p(x, y, STAND_GRIP_Z)), HALF_OPEN, f"{arm} stand release"),
                Move(1.0, _at(_p(x, y, ABOVE_Z)), HALF_OPEN, f"{arm} stand up"),
                Move(2.0, self._park(arm), FINGER_OPEN, f"{arm} park")]

    def _wrench_moves(self):
        x, y = (float(v) for v in d.WRENCH_XY)
        z = 0.030
        return [Move(2.5, _at(_p(x, y, ABOVE_Z)), HALF_OPEN, "wrench approach"),
                Move(1.2, _at(_p(x, y, z)), HALF_OPEN, "wrench descend"),
                Move(0.6, _at(_p(x, y, z)), BLOCK_SQUEEZE, "wrench grip"),
                Move(2.5, _at(_p(x - WRENCH_PULL, y, z)), BLOCK_SQUEEZE, "wrench pull"),
                Move(0.5, _at(_p(x - WRENCH_PULL, y, z)), HALF_OPEN, "wrench release"),
                Move(1.0, _at(_p(x - WRENCH_PULL, y, ABOVE_Z)), HALF_OPEN, "wrench up"),
                Move(2.0, self._park("left"), FINGER_OPEN, "left park")]

    def _knob_grip_moves(self):
        x, y = (float(v) for v in d.KNOB_XY)
        z = d.KNOB_Z + d.KNOB_HUB[1] + d.KNOB_WING[2] / 2 + 0.003
        return [Move(2.5, _at(_p(x, y, ABOVE_Z)), HALF_OPEN, "knob approach"),
                Move(1.2, _at(_p(x, y, z)), HALF_OPEN, "knob descend"),
                Move(0.6, _at(_p(x, y, z)), WING_SQUEEZE, "knob grip"),
                Move(2.5, _at(_p(x, y, z), KNOB_TURN_DEG), WING_SQUEEZE, "knob turn")]

    def _knob_release_moves(self):
        x, y = (float(v) for v in d.KNOB_XY)
        z = d.KNOB_Z + d.KNOB_HUB[1] + d.KNOB_WING[2] / 2 + 0.003
        return [Move(0.5, _at(_p(x, y, z), KNOB_TURN_DEG), HALF_OPEN, "knob let go"),
                Move(1.0, _at(_p(x, y, ABOVE_Z), KNOB_TURN_DEG), HALF_OPEN, "knob up"),
                Move(2.0, self._park("right"), FINGER_OPEN, "right park")]

    def _build(self, sim):
        self._built = True
        self.arms = {a: Arm(self._bases[a]) for a in ("left", "right")}
        self.park = {a: ik.tcp_pose(np.array(READY, float), np.asarray(self._bases[a], float)) for a in self.arms}
        if self.mode == "idle":
            self._steps = []
            return
        S = [("pump",)]
        if self.mode == "demo":
            S += [("moves", "left", self._stand_grip_moves("left", +1)),
                  ("moves", "right", self._stand_grip_moves("right", -1)), ("wait_idle", ("left", "right")),
                  ("slide_stands",)]
            S += [("moves", "right", self._knob_grip_moves()), ("wait_idle", ("right",)),
                  ("wait_until", self._settled, 20.0, "beam on both stands, saddle clear"),
                  ("moves", "right", self._knob_release_moves()), ("wait_idle", ("right",))]
            S += [("moves", "left", self._wrench_moves()), ("wait_idle", ("left",))]
        elif self.mode == "reach":
            S += [("moves", "left", self._wrench_moves()), ("wait_idle", ("left",))]
        elif self.mode == "nostands":
            S += [("moves", "right", self._knob_grip_moves()), ("wait_idle", ("right",)),
                  ("wait_until", self._lowered, 20.0, "saddle back down"),
                  ("moves", "right", self._knob_release_moves()), ("wait_idle", ("right",))]
        S += [("finish",)]
        self._steps = S

    @staticmethod
    def _settled(s) -> bool:
        st = s["stands"].values()
        return s["carry_gap_mm"] >= 8.0 and all(abs(x["gap_mm"]) <= 3.0 for x in st) and s["load_still"]

    @staticmethod
    def _lowered(s) -> bool:
        return s["jack_target_mm"] <= 0.0 and s["load_still"]

    # -- step machine -----------------------------------------------------------------------------------------------
    def _run(self, sim, t):
        if not self._steps:
            return
        step = self._steps[0]
        kind = step[0]
        st = self._st
        if kind == "pump":
            L = self.arms["left"]
            up = _p(PUMP_X, 0.0, d.lever_pad_top(d.LEVER_UP_DEG)[2] - TOP + 0.013 + 0.012)
            down = _p(PUMP_X, 0.0, d.lever_pad_top(d.LEVER_DOWN_DEG)[2] - TOP + 0.013 - 0.0065)
            if not st:
                st["n"] = 0
                L.push(Move(2.5, _at(up), CLOSED, "pump approach"))
            elif L.idle:
                s = sim.state()
                if s["raised_mm"] >= RAISE_GOAL_MM and s["jack_target_mm"] > 0 or st["n"] >= MAX_STROKES:
                    L.push(Move(1.5, self._park("left"), CLOSED, "left park"))
                    self._done(f"pumped {st['n']} strokes ({s['strokes']} counted), underside "
                               f"{s['under_lift_mm']:.1f} mm, {s['raised_mm']:+.1f} mm above the stands")
                else:
                    st["n"] += 1
                    L.push(Move(0.5, _at(down), CLOSED, f"pump down {st['n']}"),
                           Move(0.15, _at(down), CLOSED, "pump hold"),
                           Move(0.45, _at(up), CLOSED, f"pump up {st['n']}"),
                           Move(0.2, _at(up), CLOSED, "pump wait"))
        elif kind == "slide_stands":
            # Each arm drags its stand by the grip block; the command ramps to the support point and an integral of
            # the measured column-position error is added (the arm sags behind the stand's friction), so the stand
            # ends on its mark. Each arm lets go once its stand has stayed within 3 mm for 0.5 s.
            poses = sim.poses()
            goal = float(d.BEAM_XY[0])
            if not st:
                for arm, name in (("left", "stand_left"), ("right", "stand_right")):
                    st[arm] = {"cmd": float(poses[name][0]), "bias": 0.0, "bias_y": 0.0, "ok": 0.0, "t0": t,
                               "done": False}
                    y = (1 if arm == "left" else -1) * d.SUPPORT_Y
                    rot = grip_rotation(0.0)

                    def target(sim, arm=arm, y=y, rot=rot):
                        c = self._st[arm]
                        return _p(c["cmd"] + c["bias"] + d.STAND_GRIP[0][0], y + c["bias_y"], STAND_GRIP_Z), rot

                    self.arms[arm].push(Move(1e9, target, BLOCK_SQUEEZE, f"{arm} stand slide", track=True))
            for arm, name in (("left", "stand_left"), ("right", "stand_right")):
                c = st[arm]
                if c["done"]:
                    continue
                c["cmd"] = min(goal, c["cmd"] + SLIDE_MPS * self._dt)
                err = goal - float(poses[name][0])
                y_goal = (1 if arm == "left" else -1) * d.SUPPORT_Y
                err_y = y_goal - float(poses[name][1])  # the stand also drifts sideways in the grip
                c["bias_y"] = float(np.clip(c["bias_y"] + SLIDE_GAIN * err_y * self._dt, -0.02, 0.02))
                if c["cmd"] >= goal:
                    c["bias"] = float(np.clip(c["bias"] + SLIDE_GAIN * err * self._dt, -SLIDE_BIAS_MAX, SLIDE_BIAS_MAX))
                close = abs(err) < SLIDE_TOL and abs(err_y) < SLIDE_TOL
                c["ok"] = c["ok"] + self._dt if (c["cmd"] >= goal and close) else 0.0
                if c["ok"] >= SLIDE_SETTLE_S or t - c["t0"] > SLIDE_TIMEOUT_S + 0.15 / SLIDE_MPS:
                    c["done"] = True
                    A = self.arms[arm]
                    A.interrupt()
                    A.queue.clear()
                    A.push(*self._stand_release_moves(arm, float(poses[name][0]) + d.STAND_GRIP[0][0],
                                                      y_goal + c["bias_y"]))
                    print(f"[jack-demo] t={t:6.2f} {arm} stand at {err * 1000:+.1f} / {err_y * 1000:+.1f} mm (x / y) "
                          f"from its mark (bias {c['bias'] * 1000:+.1f} / {c['bias_y'] * 1000:+.1f} mm)", flush=True)
            if all(st[a]["done"] for a in ("left", "right")) and all(self.arms[a].idle for a in ("left", "right")):
                self._done("both stands set")
        elif kind == "moves":
            self.arms[step[1]].push(*step[2])
            self._done(f"{step[1]}: {step[2][0].label} ...")
        elif kind == "wait_idle":
            if all(self.arms[a].idle for a in step[1]):
                self._done(f"{'/'.join(step[1])} done")
        elif kind == "wait_until":
            if "t0" not in st:
                st["t0"] = t
            s = sim.state()
            if step[1](s) or t - st["t0"] > step[2]:
                self._done(f"{step[3]}: {'yes' if step[1](s) else 'timeout'} (carry {s['carry_gap_mm']:+.1f} mm, "
                           f"stand gaps {[round(x['gap_mm'], 1) for x in s['stands'].values()]})")
        elif kind == "finish":
            self._end = t + 3.0
            self._done("procedure finished")

    def _done(self, msg):
        print(f"[jack-demo] t={self._now:6.2f} {msg}", flush=True)
        self._steps.pop(0)
        self._st = {}

    def act(self, observation):
        from . import mechanism

        sim = mechanism.ACTIVE
        t = float(observation.state_time) + self._dt
        self._now = t
        if not self._built:
            self._st = {}
            self._build(sim)
        if self.mode != "idle":
            self._run(sim, t)
            out = [self.arms[a].act(sim, t) for a in ("left", "right")]
        else:
            out = [np.r_[READY, 1.0], np.r_[READY, 1.0]]
        meta = {"request_stop": True} if self._end is not None and t > self._end else {}
        return ActionChunk(actions=[Action(data=pack(*out), meta=meta)], control_hz=1.0 / self._dt)
