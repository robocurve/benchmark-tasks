"""The scripted demo's control hooks, for zip_lock.py.

Every trial starts from a different layout (case pose, each slider's distance from the lock, each puller's angle),
so a fixed plan cannot work. With both arms at the ready pose, the ``yam_scripted`` policy asks the task for the next
side's plan, which the task makes from the simulator's object poses (the case, the slider and the puller). The parts
are moved only by the grippers, through contact.

The simulated arm's joints are soft, so under gravity and load each settles short of its target. The demo closes
that gap from the joint angles it observes (a bounded integral correction of the TCP pose).
"""

from __future__ import annotations

import math
import os

import numpy as np

_PLAN_LOG = open(os.environ["ZIP_DEBUG"] + ".plan", "w") if os.environ.get("ZIP_DEBUG") else None

SAG_GAIN = float(os.environ.get("ZIP_SAG_GAIN", "0.25"))  # of the TCP pose error per control step
SAG_MAX = float(os.environ.get("ZIP_SAG_MAX", "0.30"))  # rad per joint
UNWIND = ("ease off", "let go")  # labels during which a hand's built-up correction runs down
UNWIND_RATE = 0.8


# Arm poses to restart the IK from when the plain warm-started solve misses (the level grip into the case's side
# needs the wrist rolled a quarter turn, a branch the plain solve does not find from the ready pose).
_LEFT_SEEDS = [[-k, 1.9, 0.3, 1.57, -k, w] for k in (0.3, 0.7, 1.1, 1.5) for w in (-1.57, 1.57)]
_LEFT_SEEDS += [[-0.5, 1.6, 0.6, 1.0, -0.5, -1.57], [-0.3, 2.2, 0.6, 1.2, -0.3, -1.57]]
IK_SEEDS = {"left": _LEFT_SEEDS,  # the left arm's base is at y +0.305; the right arm's seeds are its mirror image
            "right": [[-a, b, c, d, -e, -f] for a, b, c, d, e, f in _LEFT_SEEDS]}


def solve(ik, pos, quat, q_prev, base):
    """IK warm-started from ``q_prev``; if that misses by over 1 mm, restarted from IK_SEEDS too, keeping the solution
    that fits best and stays nearest the previous joint angles."""
    q, pe, re = ik.solve(pos, quat, q_prev, base_pos=base, iters=60)
    if pe < 0.001 and re < 0.01:
        return q, pe, re
    best = (pe + 0.05 * re, q, pe, re)
    for seed in (q, q_prev, *IK_SEEDS["left" if base[1] > 0 else "right"]):
        q2, pe2, re2 = ik.solve(pos, quat, np.asarray(seed, float), base_pos=base, iters=300)
        cost = pe2 + 0.05 * re2 + (0.002 * float(np.linalg.norm(q2 - q_prev)) if pe2 < 0.001 else 0.0)
        if cost < best[0]:
            best = (cost, q2, pe2, re2)
    return best[1], best[2], best[3]


def install() -> None:
    """Hook yam_scripted (if it is importable): plan each side when the task asks for it, and correct each arm's
    sag from its own joint readings."""
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import ik, scripted
        from yam_arena.demo import Waypoint
    except Exception:  # the task file also loads where yam_scripted is not installed (the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_zipdemo", False):
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
        return self._task is not None and hasattr(self._task.module, "zipdemo_task")

    def reset(self, scene):
        plain_reset(self, scene)
        self._zip_stage = 0 if _mine(self) and getattr(self._task.module, "LOOKS", 0) else -1
        self._zip_prev = None
        self._zip_off = np.zeros(14)

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
        if 0 <= self._zip_stage < m.LOOKS and t + 1e-6 >= self._end - 0.05:
            more = m.plan_stage(self._zip_stage)
            self._zip_stage += 1
            if more:
                _extend(self, more, t)
        labels = {arm: plan.target(t)[3] for arm, plan in self._plans.items()}
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        fields = [f"{int(round(t * 20)):4d}"]
        if self._zip_prev is not None:
            for arm, idx in (("left", np.r_[0:6]), ("right", np.r_[7:13])):
                base = self._bases[arm]
                p_plan, r_plan = ik.tcp_pose(planned[idx], base)
                p_meas, r_meas = ik.tcp_pose(measured[idx], base)
                if labels.get(arm) in UNWIND:
                    self._zip_off[idx] *= UNWIND_RATE
                else:
                    err = np.r_[p_plan - p_meas, log_rot(r_plan @ r_meas.T)]
                    jac = tcp_jacobian(measured[idx], base)
                    dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), err)
                    self._zip_off[idx] = np.clip(self._zip_off[idx] + SAG_GAIN * dq, -SAG_MAX, SAG_MAX)
                if _PLAN_LOG is not None:
                    fields.append(f"{arm}: plan=({p_plan[0]:.4f},{p_plan[1]:.4f},{p_plan[2]:.4f}) "
                                  f"meas=({p_meas[0]:.4f},{p_meas[1]:.4f},{p_meas[2]:.4f}) "
                                  f"grip={measured[idx[-1] + 1]:.3f} off={np.linalg.norm(self._zip_off[idx]):.3f} "
                                  f"[{labels.get(arm)}]")
        if _PLAN_LOG is not None:
            _PLAN_LOG.write(" ".join(fields) + "\n")
            _PLAN_LOG.flush()
        self._zip_prev = planned
        out = planned + self._zip_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._zipdemo = True
    install_ik()


def install_ik() -> None:
    """Make the scripted plans use :func:`solve` (the same targets, a better IK)."""
    from yam_arena import ik
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import ArmPlan

    if getattr(ArmPlan, "_zipdemo", False):
        return

    def action7(self, t: float):
        pos, rot, finger, label = self.target(t)
        q, pe, re = solve(ik, pos, ik.matrix_to_quat_xyzw(rot), np.asarray(self.q, float), self.base_pos)
        self.q = q
        self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
        self.worst_ik_deg = max(self.worst_ik_deg, float(np.degrees(re)))
        grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
        return np.r_[q, grip], label

    ArmPlan.action7 = action7
    ArmPlan._zipdemo = True
