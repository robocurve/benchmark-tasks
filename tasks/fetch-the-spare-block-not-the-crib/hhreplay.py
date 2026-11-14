"""Offline replay of a task's scripted plan, for its tests. No physics and no GPU.

Each arm's gripper follows its waypoints at the control rate (smoothstep between waypoints, as yam_scripted plays
them). An object inside closed fingers moves with the gripper; an object let go drops straight down onto whatever
is under its centre (a fixed object, another loose object or the table). Optionally every robot link's position is
computed from the plan's IK and the URDF, as the simulator reports link origins. Scoring is then run exactly as the
embodiment runs it: success(), distance() and score() once per control step, success held for 1 s.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from yam_arena import YAM_URDF, station
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan
from yam_arena.task_file import Asset, Box, Cylinder, ObjectState, Sphere
from yam_arena.urdf import forward_kinematics

import hhkit

HZ = 20


@dataclass
class Geom:
    """What the replay needs about an object: half-extents (box) or radius and height (upright cylinder/sphere)."""

    kind: str  # "box" | "cyl"
    half: np.ndarray  # box: x, y, z half edges; cyl: (r, r, h/2)
    top_off: float | None = None  # where a dropped object lands, from the centre (e.g. a bowl's floor)

    def top_at(self, pos, xy) -> float | None:
        """Height of this object's top over the point xy, or None if xy isn't over it."""
        dx, dy = xy[0] - pos[0], xy[1] - pos[1]
        if self.kind == "box":
            over = abs(dx) <= self.half[0] and abs(dy) <= self.half[1]
        else:
            over = math.hypot(dx, dy) <= self.half[0]
        if not over:
            return None
        return pos[2] + (self.half[2] if self.top_off is None else self.top_off)


def geom_of(spec, override=None) -> Geom:
    if override is not None:
        return override
    if isinstance(spec, Box):
        return Geom("box", np.asarray(spec.size, dtype=float) / 2)
    if isinstance(spec, Cylinder):
        return Geom("cyl", np.array([spec.radius, spec.radius, spec.height / 2]))
    if isinstance(spec, Sphere):
        return Geom("cyl", np.array([spec.radius] * 3))
    if isinstance(spec, Asset):
        r = spec.extent / 2
        return Geom("cyl", np.array([r, r, r]))
    raise TypeError(spec)


def spec_pos(spec, top):
    if isinstance(spec, Asset):
        return np.array([spec.xy[0], spec.xy[1], top + spec.extent / 2])
    return np.asarray(spec.pos, dtype=float)


def state(pos, quat=(0.0, 0.0, 0.0, 1.0), half=(0.02, 0.02, 0.02), vel=(0.0, 0.0, 0.0)) -> ObjectState:
    pos = np.asarray(pos, dtype=float)
    h = np.asarray(half, dtype=float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, dtype=float), lin_vel=np.asarray(vel, dtype=float),
                       ang_vel=np.zeros(3), low=pos - h, high=pos + h)


def gripper(tcp, opening=0.02) -> dict:
    tcp = np.asarray(tcp, dtype=float)
    return {"tcp": tcp, "tips": [tcp + [0, opening / 2, -PAD_TIP_PAST_TCP], tcp + [0, -opening / 2, -PAD_TIP_PAST_TCP]]}


PARKED = [gripper((0.12, 0.30, 0.95)), gripper((0.12, -0.30, 0.95))]


def initial_states(module, geoms=None) -> dict:
    geoms = geoms or {}
    out = {}
    for spec in module.OBJECTS:
        g = geom_of(spec, geoms.get(spec.name))
        q = hhkit.yaw_quat(getattr(spec, "yaw", 0.0))
        out[spec.name] = state(spec_pos(spec, module.TOP), q, g.half)
    return out


def _closed(finger) -> bool:
    return abs(finger - FINGER_CLOSED) < 0.3 * abs(FINGER_OPEN - FINGER_CLOSED)


def replay(module, plan, seconds=None, extra_s=2.0, links=False, geoms=None, grip_tol=0.012, hz=HZ):
    """A list of (objects, grippers, robot_points) frames for ``plan`` ({"left": [...], "right": [...]})."""
    geoms = geoms or {}
    specs = {s.name: s for s in module.OBJECTS}
    g = {n: geom_of(s, geoms.get(n)) for n, s in specs.items()}
    free = [n for n, s in specs.items() if s.mass is not None]
    pos = {n: spec_pos(s, module.TOP) for n, s in specs.items()}
    quat = {n: hhkit.yaw_quat(getattr(s, "yaw", 0.0)) for n, s in specs.items()}
    arms = {side: ArmPlan(ws, station.BASES[side]) for side, ws in plan.items()}
    total = (max(a.duration for a in arms.values()) + extra_s) if seconds is None else seconds
    attached, frames = {}, []
    prev = {n: pos[n].copy() for n in free}
    for k in range(1, int(round(total * hz)) + 1):
        t = k / hz
        grips, pts = [], []
        for idx, side in enumerate(("left", "right")):
            if side not in arms:
                grips.append(PARKED[idx])
                continue
            arm = arms[side]
            tcp, _r, finger, _ = arm.target(t)
            grips.append(gripper(tcp))
            if links:
                arm.action7(t)
                fk = forward_kinematics(YAM_URDF, {j: float(v) for j, v in zip(ARM_JOINTS, arm.q, strict=True)})
                pts.extend(m[:3, 3] + arm.base_pos for name, m in fk.items() if name not in ("base", "base_link"))
            if _closed(finger) and side not in attached:
                for n in free:
                    if n in attached.values():
                        continue
                    local = np.abs(np.asarray(tcp) - pos[n]) - g[n].half
                    if np.all(local <= grip_tol):
                        attached[side] = (n, pos[n] - np.asarray(tcp))
                        break
            if not _closed(finger) and side in attached:
                n, _off = attached.pop(side)
                pos[n][2] = support_z(n, pos, g, module.TOP)
            if side in attached:
                n, off = attached[side]
                pos[n] = np.asarray(tcp, dtype=float) + off
        for gr in grips:
            pts.extend([gr["tcp"], *gr["tips"]])
        objects = {}
        for n in specs:
            vel = (pos[n] - prev[n]) * hz if n in free else np.zeros(3)
            objects[n] = state(pos[n], quat[n], g[n].half, vel)
        prev = {n: pos[n].copy() for n in free}
        frames.append((objects, grips, np.asarray(pts, dtype=float)))
    return frames


def support_z(name, pos, g, top) -> float:
    """Centre height of ``name`` dropped onto whatever is under its centre."""
    xy = pos[name][:2]
    best = top
    for m, p in pos.items():
        if m == name or p[2] > pos[name][2]:
            continue
        h = g[m].top_at(p, xy)
        if h is not None:
            best = max(best, h)
    return best + g[name].half[2]


def run(module, trial, frames, hz=HZ):
    """task_score over frames, as the embodiment and scorer compute it."""
    hold = max(1, round(getattr(module, "SUCCESS_HOLD_S", 1.0) * hz))
    held, last, best = 0, 0.0, 0.0
    for objects, grips, pts in frames:
        trial.grippers_override, trial.points_override = grips, pts
        trial.new_step()
        ok = module.success(objects) if hasattr(module, "success") else False
        if hasattr(module, "distance"):
            module.distance(objects)
        last = min(max(float(module.score(objects)), 0.0), 1.0)
        best = max(best, last)
        held = held + 1 if ok else 0
        if held >= hold:
            return 1.0
    return best if getattr(module, "SCORE_OVER", "end") == "best" else last


def reachable(side, waypoints) -> bool:
    plan = ArmPlan(waypoints, station.BASES[side])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    return plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1
