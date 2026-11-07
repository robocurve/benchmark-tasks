"""Connect covered-case scoring to standard YAM task-file control-step hooks."""

from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path

import numpy as np

from scenekit import on_reset, on_step
from yam_arena.task_file import TABLE_TOP

from .design import make_design, MODULE_SIZE
from .geometry import lid_is_safe, mechanism_parts, obb_penetration
from .scoring import (CradleState, GripperState, ModuleState, Pose, ReturnScorer,
                      Snapshot, completely_outside_cradle, enters_occupied_cradle,
                      rotation, world_corners)

LAYOUT = make_design("covered", 2)
NAMES = ("incoming", "stored_0", "stored_1", "lid_0", "lid_1", "lid_2")
SCORE = ReturnScorer(LAYOUT.occupancy, max_steps=2400, covered_search=True)
_result = None
_violations: set[str] = set()
_steps = 0


def _module(state, size=MODULE_SIZE):
    x, y, z, w = state.quat_xyzw
    return ModuleState(Pose(tuple(state.pos), (w, x, y, z)), tuple(size),
                       tuple(state.lin_vel), tuple(state.ang_vel))


def _bounds(box):
    half = np.abs(rotation(box.pose)) @ (np.asarray(box.size) / 2)
    center = np.asarray(box.pose.position)
    return center - half, center + half


def _grippers(environment, targets):
    from yam_arena.analysis import world_hulls
    from yam_arena.constants import FINGER_JOINTS
    from yam_arena.runtime import link_poses

    corners = np.concatenate([world_corners(target) for target in targets])
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    result = []
    for side in ("left", "right"):
        robot = environment.unwrapped.scene[f"{side}_robot"]
        hulls = world_hulls(link_poses(robot), ["gripper", "tip_left", "tip_right"])
        if not hulls:
            raise RuntimeError("Gripper collision hulls unavailable")
        distances = []
        for _, hull in hulls:
            hull_lo, hull_hi = hull.vertices.min(axis=0), hull.vertices.max(axis=0)
            gap = np.maximum(np.maximum(lo - hull_hi, hull_lo - hi), 0)
            distances.append(float(np.linalg.norm(gap)))
        ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joint_pos = robot.data.joint_pos
        joint_pos = joint_pos.torch if hasattr(joint_pos, "torch") else joint_pos
        opening = -float(joint_pos[0, ids].sum().detach().cpu())
        result.append(GripperState(min(distances), opening))
    return tuple(result)


def _channels(lids):
    result = {}
    for i, origin in enumerate(LAYOUT.cradle_origins):
        clear = True
        for spec in LAYOUT.mechanisms:
            for _, part in mechanism_parts(spec, lids[spec.name]):
                corners = world_corners(part)
                lo, hi = corners.min(axis=0), corners.max(axis=0)
                blocked = (lo[0] < origin[0] + 0.025 and hi[0] > origin[0] - 0.025
                           and lo[1] < origin[1] + 0.025 and hi[1] > origin[1] - 0.025)
                clear &= not blocked
        result[f"cradle_{i}"] = bool(clear)
    return result


def _audit(modules, lids):
    fixed = [(p.name, ModuleState(Pose(p.position), p.size)) for p in LAYOUT.fixed]
    moving = [(f"{spec.name}:{name}", part)
              for spec in LAYOUT.mechanisms
              for name, part in mechanism_parts(spec, lids[spec.name])]

    def penetrate(name, item, others):
        lo, hi = _bounds(item)
        for other_name, other in others:
            other_lo, other_hi = _bounds(other)
            if np.all(np.minimum(hi, other_hi) - np.maximum(lo, other_lo) > 0.003):
                if obb_penetration(item, other) > 0.003:
                    _violations.add(f"solid_penetration:{name}:{other_name}")

    for name, item in modules.items():
        penetrate(name, item, fixed)
        penetrate(name, item, moving)
    for name, item in moving:
        penetrate(name, item, fixed)
    for spec in LAYOUT.mechanisms:
        p = np.asarray(lids[spec.name].pose.position)
        if not (-0.17 <= p[0] <= 0.89 and -0.63 <= p[1] <= 0.63
                and TABLE_TOP["station"] - 0.005 <= p[2] <= 1.8):
            _violations.add(f"discarded_lid:{spec.name}")


def _snapshot(environment, dt, states=None):
    if states is None:
        from yam_arena.scene import object_states
        states = object_states(environment, NAMES)
    modules = {name: _module(states[name]) for name in NAMES[:3]}
    lids = {name: _module(states[name], (1.0, 1.0, 1.0)) for name in NAMES[3:]}
    cradles = {f"cradle_{i}": CradleState(Pose(origin), LAYOUT.interior_min,
                                          LAYOUT.interior_max)
                for i, origin in enumerate(LAYOUT.cradle_origins)}
    if dt:
        _audit(modules, lids)
        for key, stored_name in LAYOUT.occupancy.items():
            if stored_name is None:
                continue
            if completely_outside_cradle(modules[stored_name], cradles[key], SCORE.t.expulsion_margin):
                _violations.add(f"stored_expelled:{stored_name}")
            if enters_occupied_cradle(modules["incoming"], cradles[key], SCORE.t):
                _violations.add(f"incoming_entered_occupied:{key}")
    seated = {}
    for i, spec in enumerate(LAYOUT.mechanisms):
        parts = [part for _, part in mechanism_parts(spec, lids[spec.name])]
        seated[f"cradle_{i}"] = lid_is_safe(
            spec, lids[spec.name], (LAYOUT.parking[i],), _grippers(environment, parts),
            TABLE_TOP["station"], require_case=True)
    return Snapshot(modules, cradles, _channels(lids),
                    _grippers(environment, [modules["incoming"]]),
                    not _violations, (-0.17, -0.64, TABLE_TOP["station"] - 0.01),
                    (0.89, 0.64, 1.8), float(dt), tuple(sorted(_violations)), seated)


@on_reset
def _reset(environment):
    global _result, _steps
    _violations.clear()
    _steps = 0
    snapshot = _snapshot(environment, 0.0)
    if not all(snapshot.lid_seated.values()):
        from yam_arena.scene import object_states
        states = object_states(environment, NAMES)
        detail = []
        for i, spec in enumerate(LAYOUT.mechanisms):
            body = _module(states[spec.name], (1.0, 1.0, 1.0))
            panel = dict(mechanism_parts(spec, body))["panel"]
            raw = environment.unwrapped.scene[spec.name].data
            poses = {}
            for key in ("root_pose_w", "root_link_pose_w", "root_com_pose_w"):
                value = getattr(raw, key, None)
                if value is not None:
                    value = value.torch if hasattr(value, "torch") else value
                    poses[key] = value[0].detach().cpu().numpy().tolist()
            detail.append((spec.name, tuple(body.pose.position),
                           tuple(body.linear_velocity), tuple(body.angular_velocity),
                           tuple(world_corners(panel)[::2, 2]),
                           _grippers(environment, [panel]), poses))
        raise RuntimeError(f"initial lid diagnostics: seated={snapshot.lid_seated}, "
                           f"detail={detail}")
    _result = SCORE.reset(snapshot)


@on_step
def _step(environment):
    global _result, _steps
    snapshot = _snapshot(environment, environment.unwrapped.step_dt)
    _result = SCORE.update(snapshot)
    _steps += 1
    output = os.environ.get("CASE_DEBUG_LOG")
    if output and (_steps % 20 == 0 or _result.success_ready):
        from yam_arena.scene import object_states
        lid_states = object_states(environment, NAMES[3:])
        row = {"step": _steps, "result": asdict(_result),
               "lid_seated": snapshot.lid_seated,
               "lids": {name: {"pos": [float(v) for v in lid_states[name].pos],
                               "quat_xyzw": [float(v) for v in lid_states[name].quat_xyzw],
                               "linear_velocity": [float(v) for v in lid_states[name].lin_vel],
                               "angular_velocity": [float(v) for v in lid_states[name].ang_vel]}
                        for name in NAMES[3:]},
               "open": snapshot.channel_open,
               "incoming": asdict(snapshot.modules["incoming"]),
               "grippers": [asdict(g) for g in snapshot.grippers]}
        with Path(output).open("a") as stream:
            stream.write(json.dumps(row) + "\n")


def evaluate(objects):
    if _result is None:
        raise RuntimeError("Covered-case scorer is not reset")
    return _result


def current_score():
    result = evaluate(None)
    return 1.0 if result.success_ready else result.score
