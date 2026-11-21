"""The scripted demo's planner hook and steady hands, for glue_stick.py.

The ``yam_scripted`` policy plans each stretch of the demo when the previous one has run out, from the poses of the
stick and the cap read from the simulator (a scripted demo, not a policy). The parts move only through the grippers,
by contact.

The simulated arm's joints are soft, so under gravity and load each settles short of its target. The demo closes
that gap from the joint angles it observes (a bounded integral correction of the TCP pose).
"""

from __future__ import annotations

import math
import os

import numpy as np

_PLAN_LOG = open(os.environ["GLUE_DEBUG"] + ".plan", "w") if os.environ.get("GLUE_DEBUG") else None

SAG_GAIN = float(os.environ.get("GLUE_SAG_GAIN", "0.25"))  # of the TCP pose error per control step
SAG_MAX = float(os.environ.get("GLUE_SAG_MAX", "0.30"))  # rad per joint
UNWIND_RATE = 0.8
TRACK_M = float(os.environ.get("GLUE_TRACK_M", "0.02"))  # correct only when the hand is within this of its plan
# the correction runs down while a hand lets go, backs off, goes back to the ready pose or waits there (fast moves
# far from the parts, where it would only wind up); everywhere else it builds up
IDLE = ("let go", "back off", "ready pose", "wait", "plan", "look")


# while the left hand is on the knob both hands hold the stick: the corrections then hold still (neither builds up nor
# runs down), so the two hands cannot wind up against each other through the stick and spring apart when it lets go
ON_KNOB = ("grip the knob", "turn the knob")


def _active(label) -> bool:
    return bool(label) and not any(k in label for k in IDLE)


def install() -> None:
    """Hook yam_scripted (if it is importable)."""
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import ik, scripted
        from yam_arena.demo import Waypoint
    except Exception:  # the task file also loads where yam_scripted is not installed (the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_gluedemo", False):
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
        return self._task is not None and hasattr(self._task.module, "gluedemo_task")

    def reset(self, scene):
        plain_reset(self, scene)
        self._pl_stage = 0 if _mine(self) else -1
        self._pl_prev = None
        self._pl_off = np.zeros(14)

    def _extend(self, plan: dict, t: float) -> None:
        for arm, more in plan.items():
            have = self._plans[arm]
            gap = t + self._dt - have.duration
            last = have.waypoints[-1]
            if gap > 1e-6:
                have.waypoints.append(Waypoint(gap, last.pos, last.finger, "plan", last.yaw))
            have.waypoints.extend(more)
        self._end = max((p.duration for p in self._plans.values()), default=0.0)

    def act(self, observation):
        if not _mine(self):
            return plain_act(self, observation)
        m = self._task.module
        t = float(observation.state_time)
        if 0 <= self._pl_stage < m.LOOKS and t + 1e-6 >= self._end - 0.05:
            more = m.plan_stage(self._pl_stage, None, m)
            self._pl_stage += 1
            if more:
                _extend(self, more, t)
        labels = {arm: plan.target(t)[3] for arm, plan in self._plans.items()}
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        fields = [f"{int(round(t * 20)):4d}"]
        frozen = any(k in (labels.get("left") or "") for k in ON_KNOB) and "hold the stick" in (labels.get("right") or "")
        if self._pl_prev is not None:
            for arm, idx in (("left", np.r_[0:6]), ("right", np.r_[7:13])):
                base = self._bases[arm]
                p_plan, r_plan = ik.tcp_pose(planned[idx], base)
                p_meas, r_meas = ik.tcp_pose(measured[idx], base)
                err = np.r_[p_plan - p_meas, log_rot(r_plan @ r_meas.T)]
                if frozen:
                    pass
                elif not _active(labels.get(arm)) or np.linalg.norm(err[:3]) > TRACK_M:
                    self._pl_off[idx] *= UNWIND_RATE  # far behind (a fast move): do not wind up
                else:
                    jac = tcp_jacobian(measured[idx], base)
                    dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), err)
                    self._pl_off[idx] = np.clip(self._pl_off[idx] + SAG_GAIN * dq, -SAG_MAX, SAG_MAX)
                if _PLAN_LOG is not None:
                    fields.append(f"{arm}: plan=({p_plan[0]:.4f},{p_plan[1]:.4f},{p_plan[2]:.4f}) "
                                  f"meas=({p_meas[0]:.4f},{p_meas[1]:.4f},{p_meas[2]:.4f}) "
                                  f"grip={measured[idx[-1] + 1]:.4f} [{labels.get(arm)}]")
        if _PLAN_LOG is not None and int(round(t * 20)) % 2 == 0:
            _PLAN_LOG.write(" ".join(fields) + "\n")
            _PLAN_LOG.flush()
        self._pl_prev = planned
        out = planned + self._pl_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._gluedemo = True
