"""The scripted demo's planning and steady hands, for drop_leaf_table.py.

On its first control step the ``yam_scripted`` policy asks the task file for a plan made from the start poses of the
wall section (the table moves with it), which it reads from the simulator (the scripted demo reads object poses; it moves the parts only
through the grippers, by contact). The simulated arm's joints are soft, so under load each settles short of its
target; the demo closes that gap from the joint angles it observes (a bounded integral correction of the TCP pose).
"""

from __future__ import annotations

import math
import os

import numpy as np

_PLAN_LOG = open(os.environ["LEAF_DEBUG"] + ".plan", "w") if os.environ.get("LEAF_DEBUG") else None

SAG_GAIN = float(os.environ.get("LEAF_SAG_GAIN", "0.25"))  # of the TCP pose error per control step
SAG_MAX = float(os.environ.get("LEAF_SAG_MAX", "0.30"))  # rad per joint
UNWIND = ("back off", "back off down", "back to the ready pose", "let go of the leaf", "let go of the bracket")
FREEZE = ("hold the leaf at level", "hold the bracket at its stop")  # pressing into a stop: the correction is held, not grown  # labels during which a hand's built-up correction runs down
UNWIND_RATE = 0.8


def install() -> None:
    """Hook yam_scripted (if it is importable): plan on the first step, and correct each arm's sag."""
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import ik, scripted
        from yam_arena.demo import Waypoint
    except Exception:  # the task file also loads where yam_scripted is not installed (the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_leafdemo", False):
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
        return self._task is not None and hasattr(self._task.module, "leafdemo_task")

    def reset(self, scene):
        plain_reset(self, scene)
        self._leaf_planned = not _mine(self)
        self._leaf_prev = None
        self._leaf_off = np.zeros(14)

    def _extend(self, plan: dict, t: float) -> None:
        for arm, more in plan.items():
            have = self._plans[arm]
            gap = t + self._dt - have.duration
            last = have.waypoints[-1]
            if gap > 1e-6:
                have.waypoints.append(Waypoint(gap, last.pos, last.finger, "look", last.yaw))
            have.waypoints.extend(more)
        self._end = max((p.duration for p in self._plans.values()), default=0.0)

    def act(self, observation):
        if not _mine(self):
            return plain_act(self, observation)
        m = self._task.module
        t = float(observation.state_time)
        if not self._leaf_planned and t + 1e-6 >= self._end - 0.05:
            st = m._EP.get("st")
            if st is None:
                raise RuntimeError("leafdemo: no start poses recorded at reset")
            _extend(self, m.plan_from_state(st), t)
            self._leaf_planned = True
        labels = {arm: plan.target(t)[3] for arm, plan in self._plans.items()}
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        fields = [f"{int(round(t * 20)):4d}"]
        if self._leaf_prev is not None:
            for arm, idx in (("left", np.r_[0:6]), ("right", np.r_[7:13])):
                base = self._bases[arm]
                p_plan, r_plan = ik.tcp_pose(planned[idx], base)
                p_meas, r_meas = ik.tcp_pose(measured[idx], base)
                if labels.get(arm) in UNWIND:
                    self._leaf_off[idx] *= UNWIND_RATE
                elif labels.get(arm) in FREEZE:
                    pass
                else:
                    err = np.r_[p_plan - p_meas, log_rot(r_plan @ r_meas.T)]
                    jac = tcp_jacobian(measured[idx], base)
                    dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), err)
                    self._leaf_off[idx] = np.clip(self._leaf_off[idx] + SAG_GAIN * dq, -SAG_MAX, SAG_MAX)
                if _PLAN_LOG is not None and True:
                    fields.append(f"{arm}: plan=({p_plan[0]:.4f},{p_plan[1]:.4f},{p_plan[2]:.4f}) "
                                  f"meas=({p_meas[0]:.4f},{p_meas[1]:.4f},{p_meas[2]:.4f}) "
                                  f"rot_err={math.degrees(np.linalg.norm(log_rot(r_plan @ r_meas.T))):.1f} "
                                  f"grip={measured[idx[-1] + 1]:.4f} off={np.linalg.norm(self._leaf_off[idx]):.3f} "
                                  f"[{labels.get(arm)}]")
        if _PLAN_LOG is not None:
            _PLAN_LOG.write(" ".join(fields) + "\n")
            _PLAN_LOG.flush()
        self._leaf_prev = planned
        out = planned + self._leaf_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._leafdemo = True
