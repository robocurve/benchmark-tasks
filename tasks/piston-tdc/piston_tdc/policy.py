"""Policy ``piston_tdc_scripted``: a privileged scripted demonstration (reads the trainer's measured crank angle).

Left gripper: the orange handle sleeve (straight-down grip; the sleeve spins on its pin, so the gripper keeps its
yaw while it circles the crank axis). Right gripper: the red stop's grip block. The procedure is the degree-wheel
one: turn the crank backward until the piston meets the stop (the measured angle stops following the commanded one)
and note the angle; turn it forward the long way round until it meets the stop again and note that angle; release
the crank; pull the stop and set it in the tray; take the crank again and turn it to the midpoint of the arc the
stop blocked; release and withdraw. The demo never reads the true TDC: it bisects its own two readings.

Outputs ordinary 14-D joint targets through the stock IK; it never sets a part's pose.

``-P mode=demo`` (default) is the demonstration. Failure paths (for the README's scoring checks):
``-P mode=guess``   pulls the stop first, then turns the handle to point at the stop (a guess by eye) and releases;
``-P mode=wrongmid`` measures both sides, then stops at the midpoint of the arc it swept (bottom dead centre);
``-P mode=idle``    keeps both arms at the ready pose.
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
from .motion import AngleServo, Arm, Move

SLEEVE_SQUEEZE = -0.009  # 18 mm finger gap on the 20 mm sleeve
STOP_SQUEEZE = -0.008  # 16 mm gap on the 20 mm grip block
HANDLE_ABOVE = 0.045
STOP_ABOVE = 0.05
STOP_LIFT = 0.080  # the shank clears the lid by 2 cm
SEEK_FAST_DPS, SEEK_SLOW_DPS, MOVE_DPS = 30.0, 10.0, 15.0
FAST_SPAN_DEG = 220.0  # on the long sweep, slow down after this much travel
STALL_DEG = 6.0  # commanded ahead of measured by this much: the crank is blocked (backup to the speed test)
PRESS_DEG = 0.8  # held this far past the stall reading while marking
UNLOAD_DEG = 3.0  # backed off the stop by this much before letting go
SETTLE_TOL_DEG = 0.3
STOP_YAW = math.pi / 2  # fingers close along x across the 20 mm grip block
HANDLE_YAW = 0.0  # fingers close along y


def _handle_pose(theta_deg, dz=0.0):
    p = d.handle_center(math.radians(theta_deg)) + np.array([0.0, 0.0, dz])
    return p, grip_rotation(HANDLE_YAW)


class PistonTdcScriptedPolicy(YamScriptedPolicy):
    def __init__(self, mode: str = "demo", **kwargs):
        super().__init__(**kwargs)
        from dataclasses import replace

        try:
            self.info = replace(self.info, name="piston_tdc_scripted")
        except Exception:  # noqa: BLE001 - the name is cosmetic
            pass
        self.mode = str(mode)
        assert self.mode in ("demo", "guess", "wrongmid", "idle"), self.mode

    # -- plan ------------------------------------------------------------------------------
    def reset(self, scene):
        # built on the first act(): the scene's own reset may run after this one
        self._built = False
        self._steps: list = []
        self._marks: list[float] = []
        self._log: list[str] = []
        self._end = None

    def _build(self, sim):
        self._built = True
        self.arms = {a: Arm(self._bases[a]) for a in ("left", "right")}
        self.servo = AngleServo()
        self.theta_d = sim.state()["theta_unwrapped_deg"]
        self.crank_on = False
        self.park = {a: ik.tcp_pose(np.array(READY, float), np.asarray(self._bases[a], float)) for a in self.arms}
        S = []
        if self.mode == "idle":
            self._steps = []
            return
        if self.mode == "guess":
            S += [("arm", "right", self._pull_stop_moves()), ("wait_idle", "right")]
            S += [("grip_crank",), ("crank_to", lambda: self._handle_points_at_stop()), ("unload_release",)]
        else:
            S += [("grip_crank",), ("seek", -1, None), ("mark",), ("crank_to", lambda: self._marks[-1] + UNLOAD_DEG)]
            S += [("seek", +1, FAST_SPAN_DEG), ("mark",), ("crank_to", lambda: self._marks[-1] - UNLOAD_DEG)]
            S += [("release_crank",), ("arm", "right", self._pull_stop_moves()), ("wait_idle", "right")]
            target = (lambda: self._midpoint()) if self.mode == "demo" else (lambda: self._midpoint() - 180.0)
            S += [("grip_crank",), ("crank_to", target), ("unload_release",)]
        S += [("finish",)]
        self._steps = S

    def _midpoint(self) -> float:
        """Midpoint of the arc the stop blocked: between the backward mark (+360) and the forward mark."""
        a, b = self._marks
        while b > a + 360.0:
            a += 360.0
        while b < a:
            a -= 360.0
        return (a + 360.0 + b) / 2.0

    def _handle_points_at_stop(self) -> float:
        """The crank angle at which the handle points along the cylinder, toward the stop (a guess by eye)."""
        want = math.degrees(d.PHI_TDC)  # handle direction pointing at the stop
        theta = want - math.degrees(d.PHI_TDC) - d.HANDLE_OFFSET_DEG
        now = self.theta_d
        return now + d.wrap_deg(theta - d.wrap_deg(now))

    def _pull_stop_moves(self):
        seat = d.stop_seated_pos()
        grip_z = d.TOP + d.STOP_SHANK_Z[1] + d.STOP_PLATE[2] + d.STOP_GRIP[2] / 2
        rot = grip_rotation(STOP_YAW)
        at = np.array([seat[0], seat[1], grip_z])
        up = at + [0.0, 0.0, STOP_LIFT]
        tray_z = d.TOP + d.TRAY_FLOOR + 0.008 + (grip_z - d.TOP - d.STOP_SHANK_Z[0])
        tray = np.array([d.TRAY_XY[0], d.TRAY_XY[1], tray_z])
        tray_up = np.array([tray[0], tray[1], up[2]])
        c = lambda p: (lambda sim, p=np.asarray(p, float): (p, rot))  # noqa: E731
        park = self.park["right"]
        return [Move(2.5, c(at + [0, 0, STOP_ABOVE]), FINGER_OPEN, "stop approach"),
                Move(1.2, c(at), FINGER_OPEN, "stop descend"),
                Move(0.8, c(at), STOP_SQUEEZE, "stop grip"),
                Move(1.6, c(up), STOP_SQUEEZE, "stop pull"),
                Move(2.0, c(tray_up), STOP_SQUEEZE, "stop carry"),
                Move(1.2, c(tray), STOP_SQUEEZE, "stop lower"),
                Move(0.6, c(tray), FINGER_OPEN, "stop release"),
                Move(1.0, c(tray_up), FINGER_OPEN, "stop withdraw"),
                Move(2.0, lambda sim: park, FINGER_OPEN, "right park")]

    # -- crank control ------------------------------------------------------------------------------
    def _crank_target(self, sim):
        s = sim.state()
        cmd = self.servo.command(self.theta_d, s["theta_unwrapped_deg"], self._dt, active=self._servo_active)
        return _handle_pose(cmd)

    def _start_track(self):
        self.arms["left"].interrupt()
        self.arms["left"].queue.clear()
        self.arms["left"].push(Move(1e9, self._crank_target, SLEEVE_SQUEEZE, "crank", track=True))
        self.crank_on = True

    def _run(self, sim, t):
        """Advance the step list by one control step."""
        if not self._steps:
            return
        step = self._steps[0]
        kind = step[0]
        st = self._state_of_step
        s = sim.state()
        th = s["theta_unwrapped_deg"]
        if kind == "grip_crank":
            if st.get("started") is None:
                st["started"] = t
                self.theta_d = th
                self.servo.bias = 0.0
                L = self.arms["left"]
                L.push(Move(2.5, lambda sim, th=th: _handle_pose(th, HANDLE_ABOVE), FINGER_OPEN, "crank approach"),
                       Move(1.2, lambda sim, th=th: _handle_pose(th), FINGER_OPEN, "crank descend"),
                       Move(0.8, lambda sim, th=th: _handle_pose(th), SLEEVE_SQUEEZE, "crank grip"))
            elif self.arms["left"].idle:
                self._servo_active = True
                self._start_track()
                self._done(f"gripped the crank at {th:+.2f}")
        elif kind == "seek":
            direction, fast_span = step[1], step[2]
            if st.get("started") is None:
                st["started"], st["from"] = t, th
                self._servo_active = True
            travelled = abs(self.theta_d - st["from"])
            rate = SEEK_FAST_DPS if (fast_span is not None and travelled < fast_span) else SEEK_SLOW_DPS
            slow = abs(s["omega_dps"]) < 0.25 * rate and travelled > 5.0
            st["slow_n"] = st.get("slow_n", 0) + 1 if slow else 0
            if st["slow_n"] >= 3 or direction * (self.theta_d - th) > STALL_DEG:
                self._servo_active = False
                self.theta_d = th + direction * PRESS_DEG
                self._done(f"stalled at {th:+.2f} (seek {direction:+d}, lag {direction * (self.theta_d - th):.2f})")
            else:
                self.theta_d += direction * rate * self._dt
        elif kind == "mark":
            if st.get("started") is None:
                st["started"], st["readings"] = t, []
            if t - st["started"] > 0.5:
                st["readings"].append(th)
            if t - st["started"] >= 1.2:
                mark = float(np.mean(st["readings"]))
                self._marks.append(mark)
                self._done(f"mark {len(self._marks)}: {mark:+.2f} (wheel reading)")
        elif kind == "crank_to":
            if st.get("started") is None:
                st["started"], st["goal"] = t, float(step[1]())
                st["ok_s"], st["limit"] = 0.0, t + abs(st["goal"] - self.theta_d) / MOVE_DPS + 6.0
                self._servo_active = True
            goal = st["goal"]
            delta = goal - self.theta_d
            stepmax = MOVE_DPS * self._dt
            self.theta_d += float(np.clip(delta, -stepmax, stepmax))
            if abs(goal - self.theta_d) < 1e-9:
                st["ok_s"] = st["ok_s"] + self._dt if abs(goal - th) < SETTLE_TOL_DEG else 0.0
                if st["ok_s"] >= 0.5 or t > st["limit"]:
                    self._done(f"crank at {th:+.2f} (goal {goal:+.2f})")
        elif kind in ("release_crank", "unload_release"):
            L = self.arms["left"]
            if st.get("started") is None:
                st["started"] = t
                L.interrupt()
                L.queue.clear()
                self.crank_on = False
                hold = L.pos.copy(), L.rot.copy()
                up = hold[0] + np.array([0.0, 0.0, 0.07])
                park = self.park["left"]
                L.push(Move(0.8, lambda sim: hold, FINGER_OPEN, "crank release"),
                       Move(0.3, lambda sim: hold, FINGER_OPEN, "crank let go"),
                       Move(1.2, lambda sim: (up, hold[1]), FINGER_OPEN, "crank withdraw"),
                       Move(2.0, lambda sim: park, FINGER_OPEN, "left park"))
            elif L.idle:
                self._done(f"released the crank at {th:+.2f}")
        elif kind == "arm":
            self.arms[step[1]].push(*step[2])
            self._done(f"{step[1]} arm moves queued")
        elif kind == "wait_idle":
            if self.arms[step[1]].idle:
                self._done(f"{step[1]} arm done")
        elif kind == "finish":
            if st.get("started") is None:
                st["started"] = t
                self._end = t + 3.0
                self._done("procedure finished")

    def _done(self, msg):
        self._log.append(msg)
        print(f"[piston-demo] t={self._now:6.2f} {msg}", flush=True)
        self._steps.pop(0)
        self._state_of_step = {}

    def act(self, observation):
        from . import mechanism

        sim = mechanism.ACTIVE
        t = float(observation.state_time) + self._dt
        self._now = t
        if not self._built:
            self._state_of_step = {}
            self._servo_active = True
            self._build(sim)
        if self.mode != "idle":
            self._run(sim, t)
            out = [self.arms[a].act(sim, t) for a in ("left", "right")]
        else:
            out = [np.r_[READY, 1.0], np.r_[READY, 1.0]]
        meta = {"request_stop": True} if self._end is not None and t > self._end else {}
        return ActionChunk(actions=[Action(data=pack(*out), meta=meta)], control_hz=1.0 / self._dt)
