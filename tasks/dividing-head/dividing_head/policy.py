"""Policy ``dividing_head_scripted``: a privileged scripted demonstration (reads the head's measured crank angle).

The left gripper works the yellow knob (straight-down grip; the knob spins on its pin, so the gripper keeps its yaw
while it circles the crank axis). The procedure is the machinist's: 40 / 7 = 5 5/7 = 5 15/21 turns, so
    1. pull the plunger pin out of hole 0 of the 20-hole circle and slide it out to the 21-hole circle;
    2. turn the crank 5 whole turns plus 15 holes of the 21-hole circle, holding the pin out;
    3. lower the pin into that hole, open the gripper and withdraw.
It outputs ordinary 14-D joint targets through the stock IK; it never sets a part's pose. It reads the measured crank
angle (as a machinist watches the arm pass the zero line and counts holes) and corrects the compliant arm with an
integral of the measured error.

``-P mode=demo`` (default) is the demonstration. Failure paths (for the README's scoring checks):
``-P mode=wrongcircle``  stays on the 20-hole circle and turns 5 turns + 14 holes (5.70 turns: 0.13 deg short);
``-P mode=wronghole``    moves to the 21-hole circle but stops one hole short (5 turns + 14 holes);
``-P mode=plungeronly``  moves the plunger to hole 0 of the 21-hole circle and lets go without turning the crank;
``-P mode=force``        grips the knob and tries to turn the crank 30 degrees without pulling the pin;
``-P mode=twist``        the right gripper presses on the workpiece and drags it 4 cm along its rim;
``-P mode=idle``         keeps both arms at the ready pose.
"""

from __future__ import annotations

import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import grip_rotation
from yam_arena.scripted import YamScriptedPolicy

from . import design as d
from .motion import Arm, Move

KNOB_SQUEEZE = -0.009  # 18 mm finger gap on the 20 mm knob
KNOB_ABOVE = 0.045
LIFT_CMD = 0.007  # pin lift held while turning (pin bottom 3 mm above the face)
LIFT_OK = 0.0055  # measured lift needed before turning (pin bottom 1.5 mm above the face)
RADIAL_PUSH = 0.0002  # commanded past the 21-hole circle's limit, so the slide stays against it
RADIUS_OK = 0.0556  # measured radius needed before dropping on the 21-hole circle (its pocket allows +-0.5 mm)
# Knob position correction: the YAM arm is compliant, so the commanded knob point is the desired point plus an
# integral of the measured error (per axis, rate-limited, capped), frozen while the error is large (blocked crank).
GAIN, RATE_MPS, CAP_M, FREEZE_M = 3.0, 0.02, 0.015, 0.02
FAST_DPS, SLOW_DPS, SLOW_SPAN = 40.0, 8.0, 20.0
SETTLE_TOL_DEG = 0.25  # the pocket allows +-0.5 deg on the 21-hole circle
HANDLE_YAW = 0.0  # fingers close along world y


class DividingHeadScriptedPolicy(YamScriptedPolicy):
    def __init__(self, mode: str = "demo", **kwargs):
        super().__init__(**kwargs)
        from dataclasses import replace

        try:
            self.info = replace(self.info, name="dividing_head_scripted")
        except Exception:  # noqa: BLE001 - the name is cosmetic
            pass
        self.mode = str(mode)
        assert self.mode in ("demo", "wrongcircle", "wronghole", "plungeronly", "force", "twist",
                             "idle"), self.mode

    def reset(self, scene):
        self._built = False  # built on the first act(): the scene's own reset may run after this one
        self._steps: list = []
        self._end = None

    # -- plan ------------------------------------------------------------------------------
    def _build(self, sim):
        self._built = True
        self.arms = {a: Arm(self._bases[a]) for a in ("left", "right")}
        self.pbias = np.zeros(3)
        s = sim.state_cached
        self.theta_d = s["crank_deg"]
        self.radius_d = d.CIRCLES[d.START_N]
        self.lift_d = 0.0
        self._servo_active = False
        self._servo_z = True
        self.park = {a: ik.tcp_pose(np.array(READY, float), np.asarray(self._bases[a], float)) for a in self.arms}
        holes_15 = 360.0 * 15 / d.OUTER_N
        S = []
        if self.mode == "demo":
            S = [("grip",), ("lift",), ("slide", d.CIRCLES[d.OUTER_N] + RADIAL_PUSH),
                 ("turn", -(5 * 360.0 + holes_15)), ("drop",), ("release",)]
        elif self.mode == "wronghole":
            S = [("grip",), ("lift",), ("slide", d.CIRCLES[d.OUTER_N] + RADIAL_PUSH),
                 ("turn", -(5 * 360.0 + 360.0 * 14 / d.OUTER_N)), ("drop",), ("release",)]
        elif self.mode == "plungeronly":
            S = [("grip",), ("lift",), ("slide", d.CIRCLES[d.OUTER_N] + RADIAL_PUSH), ("turn", 0.0), ("drop",),
                 ("release",)]
        elif self.mode == "wrongcircle":
            S = [("grip",), ("lift",), ("turn", -(5 * 360.0 + 360.0 * 14 / d.INNER_N)), ("drop",), ("release",)]
        elif self.mode == "force":
            S = [("grip",), ("turn", -30.0), ("release",)]
        elif self.mode == "twist":
            S = [("arm", "right", self._twist_moves()), ("wait_idle", "right")]
        if S:
            S.append(("finish",))
        self._steps = S

    def _twist_moves(self):
        """Right gripper, closed: press on the workpiece's top face near the rim and drag it 4 cm along the rim."""
        rot = grip_rotation(0.0)
        top = d.TOP + d.WORK_Z0 + d.WORK_T
        y = d.SPINDLE_XY[1] - 0.035
        z = top + 0.013 - 0.003  # pad tips 3 mm into the face: the arm presses down
        a = np.array([d.SPINDLE_XY[0] - 0.02, y, z])
        b = np.array([d.SPINDLE_XY[0] + 0.02, y, z])
        c = lambda p: (lambda sim, p=np.asarray(p, float): (p, rot))  # noqa: E731
        park = self.park["right"]
        return [Move(2.5, c(a + [0, 0, 0.05]), 0.0, "twist approach"), Move(1.5, c(a), 0.0, "twist press"),
                Move(3.0, c(b), 0.0, "twist drag"), Move(1.0, c(b + [0, 0, 0.05]), 0.0, "twist lift"),
                Move(2.0, lambda sim: park, FINGER_OPEN, "right park")]

    # -- knob control -------------------------------------------------------------------------------
    def _knob_target(self, sim):
        s = sim.state_cached
        want = d.knob_center(self.theta_d, self.radius_d, self.lift_d)
        err = want - np.asarray(s["knob"], float)
        if self._servo_active and float(np.linalg.norm(err)) < FREEZE_M:
            step = np.clip(GAIN * err, -RATE_MPS, RATE_MPS) * self._dt
            if not self._servo_z:
                step[2] = 0.0
            self.pbias = np.clip(self.pbias + step, -CAP_M, CAP_M)
        return want + self.pbias, grip_rotation(HANDLE_YAW)

    def _track(self):
        L = self.arms["left"]
        L.interrupt()
        L.queue.clear()
        L.push(Move(1e9, self._knob_target, KNOB_SQUEEZE, "knob", track=True))

    def _run(self, sim, t):
        if not self._steps:
            return
        step, st = self._steps[0], self._state_of_step
        kind = step[0]
        s = sim.state_cached
        th = s["crank_deg"]
        L = self.arms["left"]
        if kind == "grip":
            if st.get("started") is None:
                st["started"] = t
                self.theta_d, self.pbias = th, np.zeros(3)
                r = s["radius_mm"] / 1000.0
                self.radius_d = r
                rot = grip_rotation(HANDLE_YAW)
                L.push(Move(2.5, lambda sim, th=th, r=r: (d.knob_center(th, r) + [0, 0, KNOB_ABOVE], rot),
                            FINGER_OPEN, "knob approach"),
                       Move(1.2, lambda sim, th=th, r=r: (d.knob_center(th, r), rot), FINGER_OPEN, "knob descend"),
                       Move(0.8, lambda sim, th=th, r=r: (d.knob_center(th, r), rot), KNOB_SQUEEZE, "knob grip"))
            elif L.idle:
                self._servo_active = True
                self._track()
                self._done(f"gripped the knob at crank {th:+.2f}, radius {s['radius_mm']:.1f} mm")
        elif kind == "lift":
            if st.get("started") is None:
                st["started"] = t
            self.lift_d = min(LIFT_CMD, self.lift_d + 0.01 * self._dt)  # 1 cm/s
            if self.lift_d >= LIFT_CMD:
                st["ok"] = st.get("ok", 0.0) + self._dt if s["pin_lift_mm"] / 1000.0 >= LIFT_OK else 0.0
                if st["ok"] >= 0.3 or t - st["started"] > 6.0:
                    self._done(f"pin out: lift {s['pin_lift_mm']:.2f} mm (z correction {self.pbias[2] * 1000:+.1f} mm)")
        elif kind == "slide":
            goal = float(step[1])
            self.radius_d += float(np.clip(goal - self.radius_d, -0.01 * self._dt, 0.01 * self._dt))
            st.setdefault("started", t)
            if abs(goal - self.radius_d) < 1e-9 and t - st["started"] > 0.5:
                st["ok"] = st.get("ok", 0.0) + self._dt if s["circle"] == d.OUTER_N else 0.0
                if st["ok"] >= 0.5 or t - st["started"] > 6.0:
                    self._done(f"slide at {s['radius_mm']:.2f} mm, circle {s['circle']}")
        elif kind == "turn":
            if st.get("started") is None:
                st["started"], st["goal"] = t, d.ZERO_DEG + float(step[1])  # absolute: hole k from the zero line
                st["limit"] = t + abs(st["goal"] - th) / FAST_DPS * 1.5 + 15.0
                st["ok"] = 0.0
                self.theta_d = th
            goal = st["goal"]
            rem = goal - self.theta_d
            rate = FAST_DPS if abs(goal - th) > SLOW_SPAN else SLOW_DPS
            self.theta_d += float(np.clip(rem, -rate * self._dt, rate * self._dt))
            if abs(goal - self.theta_d) < 1e-9:
                ready = abs(goal - th) < SETTLE_TOL_DEG
                if self.mode != "force":
                    ready = ready and s["pin_lift_mm"] / 1000.0 >= LIFT_OK
                if self.radius_d > d.CIRCLES[d.INNER_N] + 0.005:
                    ready = ready and s["radius_mm"] / 1000.0 >= RADIUS_OK
                st["ok"] = st["ok"] + self._dt if ready else 0.0
                if st["ok"] >= 0.5 or t > st["limit"]:
                    self._done(f"crank at {th:+.3f} (goal {goal:+.3f}, {(th - d.ZERO_DEG) / 360.0:+.4f} turns), "
                               f"radius {s['radius_mm']:.2f} mm, lift {s['pin_lift_mm']:.2f} mm, "
                               f"ready {st['ok'] >= 0.5}")
        elif kind == "drop":
            if st.get("started") is None:
                st["started"] = t
                self._servo_z = False  # keep correcting x-y (angle, radius); lower the knob by plain command
            self.lift_d = max(-0.001, self.lift_d - 0.01 * self._dt)
            if self.pbias[2] > 0.0:
                self.pbias[2] = max(0.0, self.pbias[2] - 0.01 * self._dt)
            if s["seated"]:
                st["ok"] = st.get("ok", 0.0) + self._dt
            if st.get("ok", 0.0) >= 0.5 or t - st["started"] > 5.0:
                self._done(f"pin seated={s['seated']} lift {s['pin_lift_mm']:+.2f} mm, circle {s['circle']} "
                           f"hole {s['hole']}, crank {th:+.3f}")
        elif kind == "release":
            if st.get("started") is None:
                st["started"] = t
                L.interrupt()
                L.queue.clear()
                hold = L.pos.copy(), L.rot.copy()
                up = hold[0] + np.array([0.0, 0.0, 0.07])
                park = self.park["left"]
                L.push(Move(0.8, lambda sim: hold, FINGER_OPEN, "knob release"),
                       Move(0.3, lambda sim: hold, FINGER_OPEN, "knob let go"),
                       Move(1.2, lambda sim: (up, hold[1]), FINGER_OPEN, "knob withdraw"),
                       Move(2.0, lambda sim: park, FINGER_OPEN, "left park"))
            elif L.idle:
                self._done(f"released the knob; crank {th:+.3f}, spindle {s['spindle_deg']:+.3f}")
        elif kind == "arm":
            self.arms[step[1]].push(*step[2])
            self._done(f"{step[1]} arm moves queued")
        elif kind == "wait_idle":
            if self.arms[step[1]].idle:
                self._done(f"{step[1]} arm done; spindle {s['spindle_deg']:+.3f}")
        elif kind == "finish":
            if st.get("started") is None:
                st["started"] = t
                self._end = t + 3.0
                self._done("procedure finished")

    def _done(self, msg):
        print(f"[dividing-head-demo] t={self._now:6.2f} {msg}", flush=True)
        self._steps.pop(0)
        self._state_of_step = {}

    def act(self, observation):
        from . import mechanism

        sim = mechanism.ACTIVE
        t = float(observation.state_time) + self._dt
        self._now = t
        if not self._built:
            self._state_of_step = {}
            self._build(sim)
        if self.mode != "idle":
            self._run(sim, t)
            out = [self.arms[a].act(sim, t) for a in ("left", "right")]
        else:
            out = [np.r_[READY, 1.0], np.r_[READY, 1.0]]
        meta = {"request_stop": True} if self._end is not None and t > self._end else {}
        return ActionChunk(actions=[Action(data=pack(*out), meta=meta)], control_hz=1.0 / self._dt)
