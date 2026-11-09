"""Policy ``lathe_centre_height_scripted``: a privileged scripted demonstration (reads the measured crank angles,
tool position and rule lean).

The shop procedure with a steel rule:
    1. right gripper: take the rule from its stand and hold it upright in the gap between the cutting edge and the
       bar, 100 mm from its top end level with the bar;
    2. left gripper: turn the cross-feed crank clockwise until the edge traps the rule lightly against the bar;
    3. right gripper: let go of the rule (it stays trapped) and move away;
    4. left gripper: turn the height crank until the rule stands vertical (the edge is on centre), then let go.
It outputs ordinary 14-D joint targets through the stock IK; it never sets a part's pose. Privileged inputs: the
measured crank angles and the rule's pose. It stops the height crank when the rule's measured lean reaches vertical
(what a machinist sees); it never reads the hidden edge height.

``-P mode=demo`` (default). Failure paths (for the README's scoring checks):
``-P mode=wrongway``  turns the height crank the wrong way (raises the edge 0.75 mm more);
``-P mode=half``      stops the height crank halfway (edge about 0.38 mm high);
``-P mode=norule``    skips the rule: turns the height crank by the computed amount, open loop;
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

KNOB_SQUEEZE = -0.007  # 14 mm finger gap on the 16 mm knob
KNOB_ABOVE = 0.045
RULE_SQUEEZE = -0.0002  # pads 0.4 mm apart on the 1 mm rule's faces: 0.3 mm of squeeze each, ~1.5 N per finger
# (pad gap = 2 x finger joint; gripping the rule's thin edges instead let the pads close through them and trap it)
RULE_YAW = math.pi / 2  # fingers close along world x, across the rule's thickness, pads flat on its faces
RULE_LOOSE = -0.0085  # 17 mm finger gap: just off the rule's edges
RULE_GRIP_BELOW_TOP = 0.040
RULE_X = d.TIP_X0 + d.GAP0 / 2  # the rule's trap point aimed at the middle of the gap (4 mm each side)
GAIN, RATE_MPS, CAP_M, FREEZE_M = 3.0, 0.02, 0.012, 0.02
FAST_DPS, SLOW_DPS, SLOW_SPAN = 40.0, 10.0, 25.0
LEAN_STOP_DEG = 0.5  # stop the height crank once the rule's lean falls to this (it stands vertical)


class LatheCentreHeightScriptedPolicy(YamScriptedPolicy):
    def __init__(self, mode: str = "demo", **kwargs):
        super().__init__(**kwargs)
        from dataclasses import replace

        try:
            self.info = replace(self.info, name="lathe_centre_height_scripted")
        except Exception:  # noqa: BLE001 - the name is cosmetic
            pass
        self.mode = str(mode)
        assert self.mode in ("demo", "wrongway", "half", "norule", "idle"), self.mode

    def reset(self, scene):
        self._built = False
        self._steps: list = []
        self._end = None

    def _build(self, sim):
        self._built = True
        self.arms = {a: Arm(self._bases[a]) for a in ("left", "right")}
        self.park = {a: ik.tcp_pose(np.array(READY, float), np.asarray(self._bases[a], float)) for a in self.arms}
        self.pbias = np.zeros(3)
        self.crank = None  # "x" or "z": the crank the left gripper works
        self.theta_d = 0.0
        self._servo_active = False
        x_turns = (d.x_target(99.0)) / d.FEED_PER_TURN + 0.06  # to the pinch limit, a little past it
        z_turns = d.turns_to_centre()
        S = []
        if self.mode in ("demo", "wrongway", "half"):
            S = [("rule_pick",), ("rule_place",), ("grip", "x"), ("turn", "x", x_turns), ("release", "x"),
                 ("rule_let_go",)]
        if self.mode == "demo":
            S += [("grip", "z"), ("turn", "z", z_turns, True), ("release", "z")]
        elif self.mode == "wrongway":
            S += [("grip", "z"), ("turn", "z", -z_turns), ("release", "z")]
        elif self.mode == "half":
            S += [("grip", "z"), ("turn", "z", z_turns / 2), ("release", "z")]
        elif self.mode == "norule":
            S = [("grip", "z"), ("turn", "z", z_turns), ("release", "z")]
        if S:
            S.append(("finish",))
        self._steps = S
        self._st = {}

    # -- knob control ---------------------------------------------------------------------------
    def _knob_target(self, sim):
        s = sim.state_cached
        want = d.knob_center(self.crank, self.theta_d)
        err = want - np.asarray(s[f"{self.crank}knob"], float)
        if self._servo_active and float(np.linalg.norm(err)) < FREEZE_M:
            step = np.clip(GAIN * err, -RATE_MPS, RATE_MPS) * self._dt
            step[2] = 0.0
            self.pbias = np.clip(self.pbias + step, -CAP_M, CAP_M)
        return want + self.pbias, grip_rotation(0.0)

    def _rule_moves(self, sim):
        rot = grip_rotation(RULE_YAW)
        r = np.asarray(sim.state_cached["rule_pos"], float)
        top = r[2] + d.RULE[2] / 2
        grip = np.array([r[0], r[1], top - RULE_GRIP_BELOW_TOP])
        c = lambda p: (lambda sim, p=np.asarray(p, float): (p, rot))  # noqa: E731
        return [Move(2.5, c(grip + [0, 0, 0.06]), RULE_LOOSE, "rule approach"),
                Move(1.5, c(grip), RULE_LOOSE, "rule descend"),
                Move(3.0, c(grip), RULE_SQUEEZE, "rule grip"),  # slow close: a fast one drove the edge into a finger
                Move(3.0, c(grip + [0, 0, 0.10]), RULE_SQUEEZE, "rule lift")]

    def _place_moves(self, sim):
        """Aim the rule's trap point (100 mm from its top) at the middle of the gap, level with the bar, using the
        measured offset between the commanded hand point and that point on the rule (the rule sits in the fingers
        as it was gripped, not necessarily square)."""
        rot = grip_rotation(RULE_YAW)
        s = sim.state_cached
        x, y, z, w = (float(v) for v in s["rule_quat"])
        zaxis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        trap = np.asarray(s["rule_pos"], float) + zaxis * (d.RULE[2] / 2 - d.RULE_PINCH_FROM_TOP)
        offset = trap - self.arms["right"].pos
        hold = np.array([RULE_X, d.CONTACT_Y, d.TOP + d.HC]) - offset
        print(f"[lathe-demo] rule trap point {np.round(offset * 1000, 1)} mm from the hand; hold at "
              f"{np.round(hold, 4)}", flush=True)
        c = lambda p: (lambda sim, p=np.asarray(p, float): (p, rot))  # noqa: E731
        # slide in edgewise, in the rule's own plane, past the bar's free end (lowering from above is out of reach)
        side = hold + np.array([0.0, -0.065, 0.0])
        return [Move(4.0, c(side + [0, 0, 0.02]), RULE_SQUEEZE, "rule beside the bar end"),
                Move(2.0, c(side), RULE_SQUEEZE, "rule down to height"),
                Move(5.0, c(hold), RULE_SQUEEZE, "rule into the gap"),
                Move(1.0, c(hold), RULE_SQUEEZE, "rule hold")]

    def _run(self, sim, t):
        if not self._steps:
            return
        step, st = self._steps[0], self._st
        kind = step[0]
        s = sim.state_cached
        L, R = self.arms["left"], self.arms["right"]
        if kind == "rule_pick":
            if "started" not in st:
                st["started"] = t
                R.push(*self._rule_moves(sim))
            elif R.idle:
                self._done(f"rule picked; its top at {s['rule_pos'][2] + d.RULE[2] / 2 - d.TOP:.3f} m")
        elif kind == "rule_place":
            if "started" not in st:
                st["started"] = t
                R.push(*self._place_moves(sim))
                hold = R.queue[-1]
                R.push(Move(1e9, hold.target, RULE_SQUEEZE, "rule holding", track=True))
            elif len(R.queue) == 0 and R.cur is not None and R.cur.track:
                self._done(f"rule in the gap at x={s['rule_pos'][0]:.4f}, lean {s['rule_lean_deg']:+.2f} deg")
        elif kind == "grip":
            pre = step[1]
            if "started" not in st:
                st["started"] = t
                self.crank = pre
                th = s[f"{pre}crank_deg"]
                self.theta_d, self.pbias = th, np.zeros(3)
                rot = grip_rotation(0.0)
                k = d.knob_center(pre, th)
                L.push(Move(2.5, lambda sim, k=k: (k + [0, 0, KNOB_ABOVE], rot), FINGER_OPEN, f"{pre}knob approach"),
                       Move(1.2, lambda sim, k=k: (k, rot), FINGER_OPEN, f"{pre}knob descend"),
                       Move(0.8, lambda sim, k=k: (k, rot), KNOB_SQUEEZE, f"{pre}knob grip"))
            elif L.idle:
                self._servo_active = True
                L.push(Move(1e9, self._knob_target, KNOB_SQUEEZE, f"{pre}knob", track=True))
                self._done(f"gripped the {pre} knob at {s[f'{pre}crank_deg']:+.1f} deg")
        elif kind == "turn":
            pre, turns = step[1], float(step[2])
            closed = len(step) > 3 and step[3]
            th = s[f"{pre}crank_deg"]
            if "started" not in st:
                st["started"] = t
                st["goal"] = th - 360.0 * turns  # clockwise = negative yaw
                st["limit"] = t + abs(360.0 * turns) / FAST_DPS * 2.0 + 20.0
                st["ok"] = 0.0
                self.theta_d = th
            if closed:
                # the machinist's stop: turn slowly while the rule's top leans away from the robot (edge high) and
                # stop as soon as it stands vertical. Reads the rule's lean, not the hidden edge height.
                if st.get("vertical") is None and s["rule_lean_deg"] <= LEAN_STOP_DEG:
                    st["vertical"] = th
                    st["goal"] = th
                    self.theta_d = th
                elif st.get("vertical") is None:
                    st["goal"] = self.theta_d - SLOW_DPS * self._dt
            goal = st["goal"]
            rate = FAST_DPS if (abs(goal - th) > SLOW_SPAN and not closed) else SLOW_DPS
            self.theta_d += float(np.clip(goal - self.theta_d, -rate * self._dt, rate * self._dt))
            if abs(goal - self.theta_d) < 1e-9:
                ready = abs(goal - th) < 1.5 and (not closed or st.get("vertical") is not None)
                st["ok"] = st["ok"] + self._dt if ready else 0.0
                if st["ok"] >= 0.5 or t > st["limit"]:
                    self._done(f"{pre} crank at {th:+.1f} (goal {goal:+.1f}); advance {s['advance_mm']:+.3f} mm, "
                               f"edge {s['err_mm']:+.3f} mm, rule lean {s['rule_lean_deg']:+.2f} deg")
        elif kind == "release":
            if "started" not in st:
                st["started"] = t
                self._servo_active = False
                L.interrupt()
                L.queue.clear()
                hold = L.pos.copy(), L.rot.copy()
                up = hold[0] + np.array([0.0, 0.0, 0.07])
                park = self.park["left"]
                L.push(Move(0.8, lambda sim: hold, FINGER_OPEN, "knob release"),
                       Move(1.2, lambda sim: (up, hold[1]), FINGER_OPEN, "knob withdraw"),
                       Move(2.0, lambda sim: park, FINGER_OPEN, "left park"))
            elif L.idle:
                self._done(f"released the {step[1]} knob; edge {s['err_mm']:+.3f} mm")
        elif kind == "rule_let_go":
            if "started" not in st:
                st["started"] = t
                R.interrupt()
                R.queue.clear()
                hold = R.pos.copy(), R.rot.copy()
                back = hold[0] + np.array([-0.030, 0.0, 0.0])  # toward the robot: the tool keeps the rule from following
                up = back + np.array([0.0, 0.0, 0.08])
                park = self.park["right"]
                # open slowly (a snap open drags the rule's thin edge sideways out of the trap), back the hand off
                # toward the robot, then lift and park (lifting straight up from the grip carried the rule with it)
                R.push(Move(3.0, lambda sim: hold, FINGER_OPEN, "rule release"),
                       Move(2.0, lambda sim: (back, hold[1]), FINGER_OPEN, "hand back off"),
                       Move(2.0, lambda sim: (up, hold[1]), FINGER_OPEN, "rule withdraw"),
                       Move(2.0, lambda sim: park, FINGER_OPEN, "right park"))
            elif R.idle:
                self._done(f"let go of the rule: it sits at z={s['rule_pos'][2] - d.TOP:.3f}, "
                           f"lean {s['rule_lean_deg']:+.2f} deg")
        elif kind == "finish":
            if "started" not in st:
                st["started"] = t
                self._end = t + 3.0
                self._done("procedure finished")

    def _done(self, msg):
        print(f"[lathe-demo] t={self._now:6.2f} {msg}", flush=True)
        self._steps.pop(0)
        self._st = {}

    def act(self, observation):
        from . import mechanism

        sim = mechanism.ACTIVE
        t = float(observation.state_time) + self._dt
        self._now = t
        if not self._built:
            self._build(sim)
        if self.mode != "idle":
            self._run(sim, t)
            out = [self.arms[a].act(sim, t) for a in ("left", "right")]
        else:
            out = [np.r_[READY, 1.0], np.r_[READY, 1.0]]
        meta = {"request_stop": True} if self._end is not None and t > self._end else {}
        return ActionChunk(actions=[Action(data=pack(*out), meta=meta)], control_hz=1.0 / self._dt)
