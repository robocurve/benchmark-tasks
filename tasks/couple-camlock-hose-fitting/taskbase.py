"""taskbase: per-trial scoring memory for this task's score() and success(), plus small geometry helpers.

The embodiment calls success() and score() every control step. Trial keeps what a trial has done so far (milestones
reached, defects, how long the finished state has held), resets itself when a new trial starts (a new embodiment, or
the clock going backwards), and evaluates once per control step. Offline checks drive it with tick() and fake object
states, without the simulator.
"""

from __future__ import annotations

import json
import math
import os
import sys


import numpy as np


def _plain(v):
    """JSON fallback for numpy scalars and arrays in the trace."""
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, np.generic):
        return v.item()
    return str(v)

TCP_ALONG_FINGERS = 0.134  # tool point: 13.4 cm from the gripper body towards the finger tips


def _owner():
    try:
        frame = sys._getframe(3)
    except ValueError:
        return None
    for _ in range(20):
        if frame is None:
            return None
        owner = frame.f_locals.get("self")
        if owner is not None and hasattr(owner, "_t") and hasattr(owner, "task"):
            return owner
        frame = frame.f_back
    return None


def _tcps_sim(owner) -> list:
    env = getattr(owner, "_env", None)
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        g = pos[names.index("gripper")]
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]) / 2
        out.append(g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * TCP_ALONG_FINGERS)
    return out


class Trial:
    """Evaluate(objects, trial) -> (partial score in [0, 1), complete: bool). Defects are raised with trial.defect()."""

    def __init__(self, evaluate, hold_s=1.0, still_s=0.3, still_m=0.005, still_rad=0.03, trace_env="", hz=20.0):
        self.evaluate, self.hold_s, self.still_s, self.still_m, self.still_rad = evaluate, hold_s, still_s, still_m, still_rad
        self.trace_env, self.hz = trace_env, hz
        self.offline_tcps = None
        self.offline_t = 0.0
        self.new()

    # -- trial state ---------------------------------------------------------------------------------------------------
    def new(self, offline_tcps=None):
        self.key, self.t, self.bad, self.held_since = None, -1.0, False, None
        self.partial, self.complete, self.flags, self.hist, self.info = 0.0, False, {}, {}, {}
        if offline_tcps is not None:
            self.offline_tcps = offline_tcps
        self._owner = None

    def tick(self, dt=None):
        """Offline only: advance the trial clock by one control step."""
        self.offline_t += dt if dt is not None else 1.0 / self.hz

    def defect(self, why=""):
        if not self.bad:
            self.info["defect"] = why
        self.bad = True

    def once(self, flag, cond) -> bool:
        """A milestone that stays reached once it has been reached."""
        if cond:
            self.flags[flag] = True
        return bool(self.flags.get(flag, False))

    def tcps(self) -> list:
        if self._owner is None:
            return list(self.offline_tcps()) if self.offline_tcps is not None else []
        return _tcps_sim(self._owner)

    def clear_of(self, points, margin) -> bool:
        """No gripper tool point within ``margin`` of any of ``points``."""
        return all(np.linalg.norm(np.asarray(tc) - np.asarray(p, float)) > margin for p in points for tc in self.tcps())

    def still(self, name) -> bool:
        """The object (pose and joints) moved under still_m / still_rad over the last still_s."""
        h = self.hist.get(name, [])
        if not h or self.t - h[0][0] < self.still_s - 1e-9:
            return False
        ps = np.array([p for _, p, _ in h])
        if float(np.max(np.linalg.norm(ps - ps[-1], axis=1))) >= self.still_m:
            return False
        js = [j for _, _, j in h]
        if js[-1] is not None and len(js[-1]):
            arr = np.array(js)
            if float(np.max(np.abs(arr - arr[-1]))) >= self.still_rad:
                return False
        return True

    # -- the per-step update ------------------------------------------------------------------------------------------
    def update(self, objects):
        owner = _owner()
        key = id(owner) if owner is not None else "offline"
        t = float(owner._t) if owner is not None else float(self.offline_t)
        if key != self.key or (owner is not None and t < self.t):
            keep = self.offline_tcps if owner is None else None
            self.new(keep)
            self.key = key
        self._owner = owner
        if t == self.t:
            return
        self.t = t
        for name, obj in objects.items():
            if getattr(obj, "pos", None) is None:
                continue
            joints = getattr(obj, "joints", None)
            jv = np.array([joints[k] for k in sorted(joints)], float) if joints else None
            h = self.hist.setdefault(name, [])
            h.append((t, np.asarray(obj.pos, float).copy(), jv))
            while h and t - h[0][0] > self.still_s + 1e-9:
                h.pop(0)
        partial, complete = self.evaluate(objects, self)
        self.partial, self.complete = float(partial), bool(complete)
        if self.complete and not self.bad:
            if self.held_since is None:
                self.held_since = t
        else:
            self.held_since = None
        self._trace(objects)

    def _trace(self, objects):
        path = os.environ.get(self.trace_env) if self.trace_env else None
        if not path or self._owner is None:
            return
        rec = {"t": round(self.t, 3), "partial": round(self.partial, 3), "complete": self.complete,
               "defect": bool(self.bad), "flags": {k: bool(v) for k, v in self.flags.items()},
               "info": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.info.items()},
               "tcps": [[round(float(v), 4) for v in tc] for tc in self.tcps()]}
        for name, obj in objects.items():
            if getattr(obj, "pos", None) is None:
                continue
            rec[name] = [round(float(v), 4) for v in obj.pos]
            joints = getattr(obj, "joints", None)
            if joints:
                rec[name + "_joints"] = {k: round(float(v), 4) for k, v in joints.items()}
        with open(path, "a") as f:
            f.write(json.dumps(rec, default=_plain) + "\n")

    # -- what the task file exposes ------------------------------------------------------------------------------------
    def success(self, objects) -> bool:
        self.update(objects)
        return bool(self.complete and not self.bad)

    def score(self, objects) -> float:
        self.update(objects)
        if self.bad:
            return 0.0
        if self.held_since is not None and self.t - self.held_since >= self.hold_s - 1e-9:
            return 1.0
        return float(round(min(self.partial, 0.95), 3))


# -- geometry helpers ------------------------------------------------------------------------------------------------

AXES = {"X": np.array((1.0, 0, 0)), "Y": np.array((0, 1.0, 0)), "Z": np.array((0, 0, 1.0))}


def rot(axis, theta) -> np.ndarray:
    """Right-handed rotation by ``theta`` about ``axis`` ("X"/"Y"/"Z" or a vector)."""
    k = AXES[axis] if isinstance(axis, str) else np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * kx + (1 - math.cos(theta)) * kx @ kx


def joint_point(part_pos, anchor, axis, theta, point_at_zero) -> np.ndarray:
    """World position of a point on a joint body (given in the part frame at the joint's zero) at angle ``theta``;
    the part is not turned (yaw 0)."""
    a = np.asarray(anchor, float)
    return np.asarray(part_pos, float) + a + rot(axis, theta) @ (np.asarray(point_at_zero, float) - a)


def arc(part_pos, anchor, axis, point_at_zero, th0, th1, n, push_from=None, standoff=0.0, lift=0.0):
    """Points along the path of a joint-body point as the joint turns th0 -> th1, offset by ``standoff`` against the
    direction of motion (so closed finger tips trail behind the face they push) and raised by ``lift``."""
    out = []
    for k in range(n + 1):
        th = th0 + (th1 - th0) * k / n
        p = joint_point(part_pos, anchor, axis, th, point_at_zero)
        q = joint_point(part_pos, anchor, axis, th + 1e-3 * (1 if th1 > th0 else -1), point_at_zero)
        m = (q - p) / (np.linalg.norm(q - p) + 1e-12)
        out.append(p - m * standoff + np.array((0, 0, lift)))
    return out


# -- offline states --------------------------------------------------------------------------------------------------

def fake_state(pos, half=(0.02, 0.02, 0.02), joints=None, quat=(0, 0, 0, 1.0)):
    """An object state for offline checks: a box of half-extent ``half`` at ``pos``, optionally with joints."""
    from yam_arena.task_file import ObjectState
    pos = np.asarray(pos, float)
    h = np.asarray(half, float)
    if joints is None:
        return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                           low=pos - h, high=pos + h)
    from labkit import PartState
    return PartState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - h, high=pos + h, joints=dict(joints))
