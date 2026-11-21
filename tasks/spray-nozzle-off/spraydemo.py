"""The scripted demo's planning and steady hands, for spray_nozzle_off.py.

On its first control step the ``yam_scripted`` policy asks the task file for a plan made from the bottle's start pose,
which it reads from the simulator (the scripted demo reads object poses; it moves the parts only through the grippers,
by contact). Over its place it shifts the rest of the carrying hand's moves once, by how far the bottle's base is from
that place (read from the simulator). The simulated arm's joints are soft, so under load each settles
short of its target; the demo closes that gap from the joint angles it observes (a bounded integral correction of the
TCP pose).
"""

from __future__ import annotations

import dataclasses
import math
import os

import numpy as np

_PLAN_LOG = open(os.environ["SPRAY_DEBUG"] + ".plan", "w") if os.environ.get("SPRAY_DEBUG") else None

SAG_GAIN = float(os.environ.get("SPRAY_SAG_GAIN", "0.25"))  # of the TCP pose error per control step
SAG_MAX = float(os.environ.get("SPRAY_SAG_MAX", "0.30"))  # rad per joint
UNWIND = ("back off up", "back to the ready pose")
FREEZE = ("let go of the bottle", "let go of the nozzle", "back off", "touch the door", "push the door shut")  # held, not grown or run down
UNWIND_RATE = 0.8
AIM_LABEL = "steady"


def install() -> None:
    """Hook yam_scripted (if it is importable): plan on the first step, re-aim once, and correct each arm's sag."""
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import ik, scripted
        from yam_arena.demo import Waypoint
    except Exception:  # the task file also loads where yam_scripted is not installed (the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_spraydemo", False):
        return
    plain_reset, plain_act = cls.reset, cls.act

    def tcp_jacobian(q, base):
        p0, r0 = ik.tcp_pose(q, base)
        jac = np.empty((6, 6))
        for k in range(6):
            qp = np.array(q, float)
            qp[k] += 1e-4
            p, r = ik.tcp_pose(qp, base)
            jac[:3, k] = (p - p0) / 1e-4
            w = (r - r0) / 1e-4 @ r0.T
            jac[3:, k] = 0.5 * np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]])
        return jac

    def log_rot(r):
        c = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
        th = math.acos(c)
        v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
        return 0.5 * v if th < 1e-7 else th / (2.0 * math.sin(th)) * v

    def _mine(self) -> bool:
        return self._task is not None and hasattr(self._task.module, "spraydemo_task")

    def reset(self, scene):
        plain_reset(self, scene)
        self._sd_planned = not _mine(self)
        self._sd_off = np.zeros(14)
        self._sd_prev = None
        self._sd_aim_t = None
        self._sd_aimed = False

    def _extend(self, plan: dict, t: float) -> None:
        for arm, more in plan.items():
            have = self._plans[arm]
            gap = t + self._dt - have.duration
            last = have.waypoints[-1]
            if gap > 1e-6:
                have.waypoints.append(Waypoint(gap, last.pos, last.finger, "look", last.yaw))
            have.waypoints.extend(more)
        self._end = max((p.duration for p in self._plans.values()), default=0.0)

    def _reaim(self, m, t) -> None:
        """Once the carrying hand has held "steady" over its place for 0.5 s, shift its remaining moves (not the
        park) by how far the bottle's base is from that place, at most 30 mm."""
        if self._sd_aim_t is None:
            self._sd_aim_t = t
        if t - self._sd_aim_t < 0.5:
            return
        self._sd_aimed = True
        corr = np.asarray(m.place_correction(m._EP["st"]), float)
        n = float(np.linalg.norm(corr))
        if n > 0.03:
            corr *= 0.03 / n
        wps = self._plans["right"].waypoints
        acc, start = 0.0, len(wps)
        for i, w in enumerate(wps):
            if acc + w.duration > t + 1e-9:
                start = i + 1
                break
            acc += w.duration
        for i in range(start, len(wps)):
            w = wps[i]
            if w.label == "back to the ready pose":
                continue
            wps[i] = dataclasses.replace(w, pos=tuple((np.asarray(w.pos, float) + corr).tolist()))
        print(f"spraydemo: re-aim at {t:.2f} s by ({corr[0] * 1000:.1f}, {corr[1] * 1000:.1f}) mm", flush=True)

    def act(self, observation):
        if not _mine(self):
            return plain_act(self, observation)
        m = self._task.module
        t = float(observation.state_time)
        if not self._sd_planned and t + 1e-6 >= self._end - 0.05:
            st = m._EP.get("st")
            if st is None:
                raise RuntimeError("spraydemo: no start poses recorded at reset")
            _extend(self, m.plan_from_state(st), t)
            self._sd_planned = True
        labels = {arm: plan.target(t)[3] for arm, plan in self._plans.items()}
        if labels.get("right") == AIM_LABEL and not self._sd_aimed:
            _reaim(self, m, t)
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        fields = [f"{int(round(t * 20)):4d}"]
        if self._sd_prev is not None:
            for arm, idx in (("left", np.r_[0:6]), ("right", np.r_[7:13])):
                base = self._bases[arm]
                p_plan, r_plan = ik.tcp_pose(planned[idx], base)
                p_meas, r_meas = ik.tcp_pose(measured[idx], base)
                if labels.get(arm) in UNWIND:
                    self._sd_off[idx] *= UNWIND_RATE
                elif labels.get(arm) not in FREEZE:
                    err = np.r_[p_plan - p_meas, log_rot(r_plan @ r_meas.T)]
                    jac = tcp_jacobian(measured[idx], base)
                    dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), err)
                    self._sd_off[idx] = np.clip(self._sd_off[idx] + SAG_GAIN * dq, -SAG_MAX, SAG_MAX)
                if _PLAN_LOG is not None:
                    fields.append(f"{arm}: plan=({p_plan[0]:.4f},{p_plan[1]:.4f},{p_plan[2]:.4f}) "
                                  f"meas=({p_meas[0]:.4f},{p_meas[1]:.4f},{p_meas[2]:.4f}) "
                                  f"rot_err={math.degrees(np.linalg.norm(log_rot(r_plan @ r_meas.T))):.1f} "
                                  f"grip={measured[idx[-1] + 1]:.4f} q6={measured[idx[-1]]:.2f} "
                                  f"off={np.linalg.norm(self._sd_off[idx]):.3f} [{labels.get(arm)}]")
        if _PLAN_LOG is not None:
            _PLAN_LOG.write(" ".join(fields) + "\n")
            _PLAN_LOG.flush()
        self._sd_prev = planned
        out = planned + self._sd_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._spraydemo = True
