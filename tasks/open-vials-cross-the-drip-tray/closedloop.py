"""closedloop: closed-loop 6-DoF bimanual demo controller for yam_arena task files.

Standard ``yam_arena.demo.Waypoint`` plans are open-loop and constrain the gripper to point straight down. Multi-stage
mechanisms (deep shelves, spring clamps, recessed latches, angled inserts, sliding reliefs) often need either live
object positions after contact or a tilted/horizontal gripper approach.

Keep ``closedloop.py`` beside the task file and define ``demo_script(ctx)`` (plus ``demo = placeholder_demo`` so static
inspectors see a ``demo`` symbol):

    from closedloop import grip_rot, placeholder_demo
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    demo = placeholder_demo

    def demo_script(ctx):
        p = ctx.pos("cartridge")
        yield from ctx.right.move((p[0], p[1], p[2] + 0.14), 1.2, FINGER_OPEN, yaw=0.0, pitch=0.25)
        yield from ctx.right.move((p[0], p[1], p[2] + 0.01), 1.0, FINGER_OPEN, yaw=0.0, pitch=0.25)
        yield from ctx.right.grip(FINGER_CLOSED, 0.6)
        # Re-read live simulator state before the second stage:
        slot = ctx.pos("housing")
        yield from ctx.both(
            ctx.left.move((slot[0], slot[1] + 0.06, slot[2] + 0.04), 1.2, FINGER_CLOSED),
            ctx.right.move((slot[0] - 0.05, slot[1], slot[2] + 0.05), 1.2, FINGER_CLOSED, pitch=0.25),
        )
        yield from ctx.wait(1.5)

Importing this module hooks ``yam_arena.inspect_embodiment`` and ``yam_arena.scripted`` when they load, so the standard
``inspect-robots run --embodiment yam_arena --policy yam_scripted`` command runs ``demo_script(ctx)`` closed-loop at the
embodiment's control rate. ``dry_run(task_path)`` steps the generator offline in pure NumPy to verify reachability and
IK error without starting Isaac Sim.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import itertools
import math
import sys
from collections.abc import Callable, Generator
from pathlib import Path
from typing import Any

import numpy as np
from yam_arena import ik, station, task_file
from yam_arena.constants import (
    CONTROL_HZ,
    FINGER_CLOSED,
    FINGER_OPEN,
    LEFT_BASE_POS,
    READY,
    RIGHT_BASE_POS,
)
from yam_arena.demo import DOWN_GRIP
from yam_arena.task_file import ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

__all__ = [
    "ArmController",
    "ClosedLoopContext",
    "dry_run",
    "grip_rot",
    "placeholder_demo",
]

ARMS = ("left", "right")
_ACTIVE_EMBODIMENTS: dict[str, Any] = {}


def grip_rot(yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """3x3 world rotation matrix for the gripper TCP.

    - ``(0, 0, 0)`` is ``DOWN_GRIP``: approach axis pointing straight down (-Z), fingers closing along world Y.
    - ``pitch`` (rad) tilts the approach axis forward toward +X before ``yaw`` (e.g. ``pitch=math.pi/4`` angles 45 deg
      forward-and-down into a shelf; ``pitch=math.pi/2`` points horizontally along +X).
    - ``roll`` (rad) twists around the gripper's approach axis.
    - ``yaw`` (rad) turns the tilted approach around world +Z.
    """
    r = quat_xyzw_to_matrix(np.asarray(DOWN_GRIP, dtype=float))
    if roll != 0.0:
        r = r @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), float(roll))
    if pitch != 0.0:
        r = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -float(pitch)) @ r
    if yaw != 0.0:
        r = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), float(yaw)) @ r
    return r


def _as_rot_matrix(rot: Any, yaw: float, pitch: float, roll: float) -> np.ndarray:
    if rot is None:
        return grip_rot(yaw=yaw, pitch=pitch, roll=roll)
    arr = np.asarray(rot, dtype=float)
    if arr.shape == (3, 3):
        return arr
    if arr.shape == (4,):
        return quat_xyzw_to_matrix(arr)
    raise ValueError(f"rot must be a 3x3 matrix or 4-element xyzw quaternion, got shape {arr.shape}")


class ArmController:
    """Per-arm Cartesian + gripper trajectory generator with warm-started DLS IK."""

    def __init__(self, side: str, base_pos, dt: float) -> None:
        self.side = side
        self.base_pos = np.asarray(base_pos, dtype=float)
        self.dt = float(dt)
        self.q = np.array(READY, dtype=float)
        self.finger = float(FINGER_OPEN)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base_pos)
        self.measured_q = self.q.copy()
        self.measured_finger = self.finger
        self.measured_pos = self.pos.copy()
        self.measured_rot = self.rot.copy()
        self.worst_ik_mm = 0.0
        self.worst_ik_deg = 0.0

    def update_measured(self, q7: np.ndarray) -> None:
        """Update joint-encoder FK telemetry from the live 7-D arm state."""
        arr = np.asarray(q7, dtype=float)
        self.measured_q = arr[:6].copy()
        self.measured_finger = float(arr[6]) * FINGER_OPEN
        self.measured_pos, self.measured_rot = ik.tcp_pose(self.measured_q, self.base_pos)

    def _set_cartesian(self, pos: np.ndarray, rot: np.ndarray, finger: float) -> None:
        q, pe, re = ik.solve(
            pos,
            ik.matrix_to_quat_xyzw(rot),
            self.q,
            base_pos=self.base_pos,
            iters=80,
        )
        self.q = q
        self.pos = np.asarray(pos, dtype=float)
        self.rot = np.asarray(rot, dtype=float)
        self.finger = float(finger)
        self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000.0)
        self.worst_ik_deg = max(self.worst_ik_deg, float(np.degrees(re)))

    def action7(self) -> np.ndarray:
        grip = float(np.clip(self.finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
        return np.r_[self.q, grip]

    def move(
        self,
        pos: tuple[float, float, float] | np.ndarray,
        duration: float = 1.2,
        finger: float | None = None,
        *,
        yaw: float = 0.0,
        pitch: float = 0.0,
        roll: float = 0.0,
        rot: Any = None,
    ) -> Generator[None, None, None]:
        """Smoothstep move from the current TCP pose to ``pos`` and orientation over ``duration`` seconds."""
        p0 = self.pos.copy()
        r0 = self.rot.copy()
        f0 = self.finger
        p1 = np.asarray(pos, dtype=float)
        r1 = _as_rot_matrix(rot, yaw, pitch, roll)
        f1 = self.finger if finger is None else float(finger)
        steps = max(1, round(float(duration) / self.dt))
        for i in range(1, steps + 1):
            s = ik.smoothstep(i / steps)
            self._set_cartesian(
                p0 + s * (p1 - p0),
                ik.slerp_matrix(r0, r1, s),
                f0 + s * (f1 - f0),
            )
            yield

    def servo(
        self,
        target_fn: Callable[[], tuple[float, float, float] | np.ndarray | tuple[Any, Any]],
        duration: float = 1.2,
        finger: float | None = None,
        *,
        yaw: float = 0.0,
        pitch: float = 0.0,
        roll: float = 0.0,
        rot: Any = None,
        max_speed: float = 0.18,
    ) -> Generator[None, None, None]:
        """Closed-loop step-by-step tracking of ``target_fn()`` (re-evaluated every control step)."""
        f0 = self.finger
        f1 = self.finger if finger is None else float(finger)
        default_rot = _as_rot_matrix(rot, yaw, pitch, roll)
        steps = max(1, round(float(duration) / self.dt))
        max_step = float(max_speed) * self.dt
        for i in range(1, steps + 1):
            raw = target_fn()
            if isinstance(raw, tuple) and len(raw) == 2 and np.asarray(raw[0]).shape == (3,):
                goal_pos = np.asarray(raw[0], dtype=float)
                goal_rot = _as_rot_matrix(raw[1], yaw, pitch, roll)
            else:
                goal_pos = np.asarray(raw, dtype=float)
                goal_rot = default_rot
            delta = goal_pos - self.pos
            dist = float(np.linalg.norm(delta))
            next_pos = self.pos + (delta * (max_step / dist) if dist > max_step and dist > 1e-9 else delta)
            s = ik.smoothstep(i / steps)
            next_rot = ik.slerp_matrix(self.rot, goal_rot, min(1.0, 0.35 + 0.65 * s))
            self._set_cartesian(next_pos, next_rot, f0 + s * (f1 - f0))
            yield

    def grip(self, finger: float, duration: float = 0.6) -> Generator[None, None, None]:
        """Open or close the fingers over ``duration`` seconds while holding the current TCP pose."""
        yield from self.move(self.pos.copy(), duration=duration, finger=finger, rot=self.rot.copy())

    def hold(self, duration: float = 0.5) -> Generator[None, None, None]:
        """Hold the current joint target for ``duration`` seconds."""
        steps = max(1, round(float(duration) / self.dt))
        for _ in range(steps):
            yield

    def trim_to_measured(
        self,
        target: tuple[float, float, float] | np.ndarray,
        *,
        tol: float = 0.002,
        max_comp: float = 0.030,
        step_mps: float = 0.04,
        max_duration: float = 1.0,
    ) -> Generator[None, None, None]:
        """Compensate joint PD gravity/wrist sag so ``measured_pos`` converges to ``target`` (no-op in offline dry_run)."""
        goal = np.asarray(target, dtype=float)
        anchor = self.pos.copy()
        rot = self.rot.copy()
        max_step = float(step_mps) * self.dt
        steps = max(1, round(float(max_duration) / self.dt))
        for _ in range(steps):
            err = goal - self.measured_pos
            dist = float(np.linalg.norm(err))
            if dist <= tol:
                break
            step = err * min(0.5, max_step / dist)
            cand = self.pos + step
            off = cand - anchor
            off_norm = float(np.linalg.norm(off))
            if off_norm > max_comp:
                cand = anchor + off * (max_comp / off_norm)
            self._set_cartesian(cand, rot, self.finger)
            yield


class ClosedLoopContext:
    """Passed to ``demo_script(ctx)``; provides live ``ObjectState``s and both ``ArmController``s."""

    def __init__(
        self,
        bases: dict[str, Any],
        dt: float,
        state_reader: Callable[[], dict[str, ObjectState]],
    ) -> None:
        self.dt = float(dt)
        self.left = ArmController("left", bases["left"], self.dt)
        self.right = ArmController("right", bases["right"], self.dt)
        self._state_reader = state_reader
        self.step_count = 0

    def states(self) -> dict[str, ObjectState]:
        """Current simulator ``ObjectState``s keyed by object name."""
        return self._state_reader()

    def obj(self, name: str) -> ObjectState:
        return self.states()[name]

    def pos(self, name: str) -> np.ndarray:
        return np.array(self.states()[name].pos, dtype=float)

    def wait(self, duration: float) -> Generator[None, None, None]:
        steps = max(1, round(float(duration) / self.dt))
        for _ in range(steps):
            yield

    def both(
        self,
        left_gen: Generator[None, None, None],
        right_gen: Generator[None, None, None],
    ) -> Generator[None, None, None]:
        """Advance ``left_gen`` and ``right_gen`` together until both finish."""
        sentinel = object()
        for a, b in itertools.zip_longest(left_gen, right_gen, fillvalue=sentinel):
            _ = (a, b)
            yield


def placeholder_demo() -> dict[str, list]:
    """Assigned to ``demo`` in task files whose motion lives in ``demo_script(ctx)``."""
    return {"left": []}


def _mock_initial_states(task: task_file.TaskFile) -> dict[str, ObjectState]:
    out: dict[str, ObjectState] = {}
    for spec in task.objects:
        half_yaw = getattr(spec, "yaw", 0.0) / 2.0
        quat = np.array([0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)], dtype=float)
        if hasattr(spec, "pos"):
            pos = np.array(spec.pos, dtype=float)
        else:
            ext = float(getattr(spec, "extent", 0.06))
            pos = np.array([spec.xy[0], spec.xy[1], task.table_top + ext / 2.0], dtype=float)
        if hasattr(spec, "size"):
            half = np.array(spec.size, dtype=float) / 2.0
        elif hasattr(spec, "radius") and hasattr(spec, "height"):
            half = np.array([spec.radius, spec.radius, spec.height / 2.0], dtype=float)
        elif hasattr(spec, "radius"):
            half = np.array([spec.radius, spec.radius, spec.radius], dtype=float)
        else:
            ext = float(getattr(spec, "extent", 0.06))
            half = np.array([ext / 2.0, ext / 2.0, ext / 2.0], dtype=float)
        # If spec is a mechkit.Mech, populate joints=0 and origin=pos so score()/demo_script() can read .joints
        if hasattr(spec, "joints") and hasattr(spec, "shapes"):
            from mechkit import MechState  # local import only when Mech is used

            joints = {j.name: 0.0 for j in spec.joints}
            out[spec.name] = MechState(
                pos=pos,
                quat_xyzw=quat,
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
                low=pos - half,
                high=pos + half,
                joints=joints,
                origin=pos.copy(),
            )
        else:
            out[spec.name] = ObjectState(
                pos=pos,
                quat_xyzw=quat,
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
                low=pos - half,
                high=pos + half,
            )
    return out


def dry_run(
    task_or_path: str | Path | task_file.TaskFile,
    *,
    max_steps: int = 4000,
    dt: float = 1.0 / CONTROL_HZ,
) -> dict[str, Any]:
    """Step ``demo_script(ctx)`` offline against resting initial object poses to check IK and step budget."""
    t = task_or_path if isinstance(task_or_path, task_file.TaskFile) else task_file.load(task_or_path)
    fn = getattr(t.module, "demo_script", None)
    if fn is None:
        raise ValueError(f"{t.path.name} has no demo_script(ctx)")
    bases = dict(station.BASES) if t.scene == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    mock = _mock_initial_states(t)
    ctx = ClosedLoopContext(bases, dt, lambda: mock)
    gen = fn(ctx)
    traj: list[tuple[np.ndarray, np.ndarray]] = []
    steps = 0
    for _ in gen:
        steps += 1
        la, ra = ctx.left.action7(), ctx.right.action7()
        ctx.left.update_measured(la)
        ctx.right.update_measured(ra)
        traj.append((la, ra))
        if steps > max_steps:
            raise RuntimeError(f"{t.path.name}: demo_script exceeded max_steps={max_steps}")
    return {
        "steps": steps,
        "duration_s": round(steps * dt, 3),
        "worst_ik_mm": max(ctx.left.worst_ik_mm, ctx.right.worst_ik_mm),
        "worst_ik_deg": max(ctx.left.worst_ik_deg, ctx.right.worst_ik_deg),
        "trajectory": traj,
    }


def _patch_embodiment(mod: Any) -> None:
    cls = mod.YamArenaEmbodiment
    if getattr(cls, "_closedloop_patched", False):
        return
    orig_reset = cls.reset
    orig_close = cls.close

    def reset(self, scene, *, seed=None):
        obs = orig_reset(self, scene, seed=seed)
        _ACTIVE_EMBODIMENTS[self.info.environment_id] = self
        return obs

    def close(self):
        _ACTIVE_EMBODIMENTS.pop(self.info.environment_id, None)
        return orig_close(self)

    cls.reset = reset
    cls.close = close
    cls._closedloop_patched = True


def _print_states(ctx: ClosedLoopContext) -> None:
    """Diagnostic only: prints where each object is when demo_script ends."""
    try:
        for name, st in ctx.states().items():
            print(f"CLSTATE {name} pos={np.round(st.pos, 4).tolist()} low={np.round(st.low, 4).tolist()} "
                  f"high={np.round(st.high, 4).tolist()}", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"CLSTATE unavailable: {e}", flush=True)


def _patch_scripted(mod: Any) -> None:
    cls = mod.YamScriptedPolicy
    if getattr(cls, "_closedloop_patched", False):
        return
    orig_reset = cls.reset
    orig_act = cls.act
    pack = mod.pack
    Action = mod.Action
    ActionChunk = mod.ActionChunk

    def reset(self, scene):
        self._cl_ctx = None
        self._cl_gen = None
        self._cl_done_at = None
        fn = getattr(getattr(self._task, "module", None), "demo_script", None)
        if fn is not None:
            def reader() -> dict[str, ObjectState]:
                # Look the embodiment up here, not at policy reset: rollout.py resets the policy first, so on a
                # first trial it is not registered yet and every read would return start-of-run poses.
                emb = _ACTIVE_EMBODIMENTS.get(self._environment_id or "")
                if emb is not None and emb._env is not None and emb.task is not None:
                    from yam_arena.scene import object_states

                    return object_states(emb._env, emb.task.names)
                return _mock_initial_states(self._task)

            self._cl_ctx = ClosedLoopContext(self._bases, self._dt, reader)
            self._cl_gen = fn(self._cl_ctx)
            return
        return orig_reset(self, scene)

    def act(self, observation):
        if self._cl_ctx is None:
            return orig_act(self, observation)
        st = getattr(observation, "state", None)
        if isinstance(st, dict):
            raw14 = st.get("joint_pos")
            if raw14 is not None and len(raw14) == 14:
                self._cl_ctx.left.update_measured(raw14[:7])
                self._cl_ctx.right.update_measured(raw14[7:14])
        t = float(observation.state_time) + self._dt
        if self._cl_gen is not None:
            try:
                next(self._cl_gen)
                self._cl_ctx.step_count += 1
            except StopIteration:
                self._cl_gen = None
                self._cl_done_at = t
                _print_states(self._cl_ctx)
        meta = (
            {"request_stop": True}
            if (self._cl_done_at is not None and t > self._cl_done_at + self.hold_s)
            else {}
        )
        data = pack(self._cl_ctx.left.action7(), self._cl_ctx.right.action7())
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1.0 / self._dt)

    cls.reset = reset
    cls.act = act
    cls._closedloop_patched = True


class _AfterImport(importlib.abc.MetaPathFinder):
    """Hooks yam_arena.inspect_embodiment and yam_arena.scripted as soon as either is imported."""

    TARGETS = {
        "yam_arena.inspect_embodiment": _patch_embodiment,
        "yam_arena.scripted": _patch_scripted,
    }

    def __init__(self) -> None:
        self._pending = set(self.TARGETS)

    def find_spec(self, fullname, path=None, target=None):
        if fullname not in self._pending:
            return None
        self._pending.discard(fullname)
        if not self._pending and self in sys.meta_path:
            sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        if spec is None or spec.loader is None:
            return spec
        run = spec.loader.exec_module
        patcher = self.TARGETS[fullname]

        def exec_module(module):
            run(module)
            patcher(module)

        spec.loader.exec_module = exec_module
        return spec


def _install() -> None:
    finder = _AfterImport()
    for mod_name, patcher in list(finder.TARGETS.items()):
        if mod_name in sys.modules:
            patcher(sys.modules[mod_name])
            finder._pending.discard(mod_name)
    if finder._pending and not any(isinstance(f, _AfterImport) for f in sys.meta_path):
        sys.meta_path.insert(0, finder)


_install()
