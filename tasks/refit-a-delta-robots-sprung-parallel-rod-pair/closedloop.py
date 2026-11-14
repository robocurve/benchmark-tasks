"""closedloop: closed-loop 6-DoF bimanual demo controller for yam_arena task files.

Standard ``yam_arena.demo.Waypoint`` plans are open-loop and constrain the gripper to point straight down. Multi-stage
mechanisms (deep shelves, spring clamps, recessed latches, angled inserts, sliding reliefs) often need a tilted or
horizontal gripper approach, encoder feedback to cancel wrist sag, or a press that stops on contact.

Keep ``closedloop.py`` beside the task file, unchanged, and define ``demo_script(ctx)`` (plus ``demo = placeholder_demo``
so static inspectors see a ``demo`` symbol). This example takes every target from the task's constants (run with
object_jitter=0) and its feedback from the robot's own sensors, which keeps it robust; a demo may also read simulator
state (see below):

    from closedloop import placeholder_demo
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    CART = (0.26, -0.10, TOP + 0.05)      # fixed layout: run with -E object_jitter=0
    SEAT = (0.30, 0.08, TOP + 0.06)

    demo = placeholder_demo

    def demo_script(ctx):
        R = ctx.right
        yield from R.move((CART[0], CART[1], CART[2] + 0.12), 1.2, FINGER_OPEN, pitch=0.25)
        yield from R.trim_to_measured((CART[0], CART[1], CART[2] + 0.12))
        yield from R.move(CART, 1.0, FINGER_OPEN, pitch=0.25)
        yield from R.grip(FINGER_CLOSED, 0.6)
        yield from R.hold(0.3)
        if not R.holding(0.040):          # pads stopped at the 45 mm body: in hand
            return                         # a miss: stop rather than guess
        yield from R.move((SEAT[0], SEAT[1], SEAT[2] + 0.06), 1.5, pitch=0.25)
        res = yield from R.press_until((SEAT[0], SEAT[1], SEAT[2] + 0.01), max_depth=0.016, err_mm=4.0, min_depth=0.010)
        yield from R.home()

A scripted demo may read simulator state to plan its actions: Robocurve's review agent instructions (version 21,
2026-11-12, docs/REVIEW_AGENT_PROMPT_v21.txt) say a demo that reads object poses, joint states or latch flags must not
be failed, revised or flagged for it; only teleporting objects, editing the simulation or claiming a score it does not
reach fails feasibility. So the context is privileged by default: ``ctx.pos``, ``ctx.obj`` and ``ctx.states`` work and
are counted in ``ctx.state_reads``. Strict mode is opt-in: ``ClosedLoopContext(..., strict=True)``,
``dry_run(task, strict=True)``, or ``DEMO_STRICT = True`` in the task module (which also makes the live yam_scripted
run strict); there a state read raises ObservabilityError. The encoder helpers still make a demo robust whatever it
reads: ``arm.measured_pos`` / ``measured_rot`` (encoder forward kinematics), ``arm.trim_to_measured(...)``,
``arm.gap()``, ``arm.holding(min_gap)``, ``arm.tracking_error()``, ``arm.press_until(...)``, ``ctx.joint_pos`` and
``ctx.images`` (both None offline). ``ctx.look(name)`` is a one-time rounded pose read that the task must declare in
``DEMO_LOOKS``. ``kit/observable_lint.py`` reports state reads statically (advisory).

Importing this module hooks ``yam_arena.inspect_embodiment`` and ``yam_arena.scripted`` when they load, so the standard
``inspect-robots run --embodiment yam_arena --policy yam_scripted`` command runs ``demo_script(ctx)`` closed-loop at the
embodiment's control rate. ``dry_run(task_path)`` steps the generator offline in pure NumPy at 20 Hz (the rate the runs
use) to verify reachability and IK error without starting Isaac Sim. It follows the same default as a live run
(privileged unless strict is asked for).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import itertools
import math
import sys
from collections.abc import Callable, Generator, Mapping
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np
from yam_arena import ik, station, task_file
from yam_arena.constants import (  # noqa: F401  CONTROL_HZ stays importable for older check.py files
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
    "DEMO_HZ",
    "ArmController",
    "ClosedLoopContext",
    "LookPose",
    "ObservabilityError",
    "PressResult",
    "dry_run",
    "grip_rot",
    "placeholder_demo",
]

ARMS = ("left", "right")
_ACTIVE_EMBODIMENTS: dict[str, Any] = {}

# run_task.sh, sweep.py and gate_check.py all run at 20 Hz (yam_arena's CONTROL_HZ of 10 is the real rig's rate), so
# dry_run defaults to 20 Hz and its step counts match the step limit of a real run.
DEMO_HZ = 20.0


class ObservabilityError(RuntimeError):
    """A strict-mode demo_script read simulator object state that a policy never observes (images and joint_pos only)."""


def _resolve_privileged(privileged: bool | None, strict: bool | None, default: bool = True) -> bool:
    """privileged unless strict: ``strict=True`` (or ``privileged=False``) asks for strict mode; neither means ``default``."""
    if strict is not None and privileged is not None and bool(strict) == bool(privileged):
        raise ValueError(f"privileged={privileged!r} contradicts strict={strict!r}")
    if strict is not None:
        return not bool(strict)
    if privileged is not None:
        return bool(privileged)
    return bool(default)


_OBS_HINT = (
    "use task constants (object_jitter=0), arm.measured_pos, arm.gap(), arm.holding(), "
    "arm.tracking_error(), arm.press_until() or ctx.images"
)


class PressResult(NamedTuple):
    """What ``ArmController.press_until`` did: commanded depth reached (m), why it stopped, peak lag (mm)."""

    depth: float
    stopped_by: str  # "contact" or "max_depth"
    peak_lag_mm: float


class LookPose(NamedTuple):
    """A one-time pose read declared in ``DEMO_LOOKS``: position rounded to 1 mm, yaw rounded to 0.5 deg."""

    pos: np.ndarray
    yaw: float

    def to_world(self, local) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y, z = (float(v) for v in np.asarray(local, dtype=float))
        return np.array([c * x - s * y, s * x + c * y, z], dtype=float) + np.asarray(self.pos, dtype=float)


def _unit(v) -> np.ndarray:
    u = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(u))
    if n < 1e-12:
        raise ValueError("direction must be a non-zero 3-vector")
    return u / n


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
        # Set by the context: True in dry_run, where nothing can be felt (holding() assumes a grasp, press_until never
        # detects contact). The context also points press_log at its own list.
        self.offline = False
        # Set by dry_run's second, pessimistic pass: the first holding() reports a missed grasp and press_until feels
        # contact at min_depth, so retry and contact branches run offline too (under the guard in strict mode).
        self.pessimistic = False
        self._missed_once = False
        self.press_log: list[PressResult] = []

    def update_measured(self, q7: np.ndarray) -> None:
        """Update joint-encoder FK telemetry from the live 7-D arm state."""
        arr = np.asarray(q7, dtype=float)
        self.measured_q = arr[:6].copy()
        self.measured_finger = float(arr[6]) * FINGER_OPEN
        self.measured_pos, self.measured_rot = ik.tcp_pose(self.measured_q, self.base_pos)

    # -- encoder-only feedback -------------------------------------------------------------------------------------

    def gap(self) -> float:
        """Measured pad gap in metres from the gripper encoder: 0 closed, about 0.094 open. (FINGER_OPEN = -0.04695 is
        one finger joint; the pads are 94 mm apart open, so the gap is 2*|measured_finger|.)"""
        return 2.0 * abs(float(self.measured_finger))

    def commanded_gap(self) -> float:
        """Pad gap the current command asks for, in metres."""
        return 2.0 * abs(float(self.finger))

    def tracking_error(self, direction=None) -> float:
        """Commanded TCP minus measured TCP (m). With direction None, the Euclidean norm. With a 3-vector, the signed lag
        along it (positive means the measured TCP is behind the command, as when a press is blocked). Joint PD wrist sag
        makes the free-space value 15 to 20 mm, so compare it with a baseline, never with zero. In dry_run the measured
        pose is the FK of the IK solution, so the value is only the IK error."""
        d = np.asarray(self.pos, dtype=float) - np.asarray(self.measured_pos, dtype=float)
        return float(np.linalg.norm(d)) if direction is None else float(d @ _unit(direction))

    def holding(self, min_gap: float) -> bool:
        """After a commanded close, True when the encoder says the pads stopped at least min_gap apart (a part is between
        them). Returns False if the fingers were not commanded closer than min_gap. Call it after grip() plus hold(0.3):
        the observation lags one step. Pick min_gap about 3 to 5 mm under the part's width. In dry_run a commanded close
        always counts as a grasp, except in dry_run's pessimistic pass, where the first call on each arm reports a miss so
        the retry branch runs once."""
        if self.commanded_gap() >= float(min_gap):
            return False
        if self.offline and self.pessimistic and not self._missed_once:
            self._missed_once = True
            return False
        return True if self.offline else self.gap() >= float(min_gap)

    def press_until(
        self,
        target: tuple[float, float, float] | np.ndarray,
        max_depth: float,
        err_mm: float = 4.0,
        *,
        min_depth: float = 0.0,
        direction=(0.0, 0.0, -1.0),
        speed: float = 0.006,
        confirm_steps: int = 2,
        settle: float = 0.3,
        hold: float = 0.3,
        rot: Any = None,
    ) -> Generator[None, None, PressResult]:
        """Press from ``target`` (the start point just short of contact, from task constants) along ``direction`` at
        ``speed`` (m/s) until the lag along ``direction`` rises ``err_mm`` above its value at the start for
        ``confirm_steps`` consecutive steps, but only once depth >= ``min_depth`` (so a snap's detent cannot stop it
        early). Otherwise stop at ``max_depth``. Hold ``hold`` seconds at the stop point without backing off.

        Usage: ``res = yield from arm.press_until(start, max_depth=0.016, err_mm=4.0, min_depth=0.010)``.
        Set max_depth a few mm past the seated depth: dry_run's first pass never feels contact, so it runs to max_depth;
        its pessimistic pass feels contact at min_depth.
        """
        max_depth = float(max_depth)
        min_depth = float(min_depth)
        if max_depth <= 0.0:
            raise ValueError("press_until: max_depth must be positive")
        if min_depth > max_depth:
            raise ValueError("press_until: min_depth must not exceed max_depth")
        if speed <= 0.0 or confirm_steps < 1:
            raise ValueError("press_until: speed must be positive and confirm_steps at least 1")
        u = _unit(direction)
        start = np.asarray(target, dtype=float)
        r = self.rot.copy() if rot is None else _as_rot_matrix(rot, 0.0, 0.0, 0.0)
        if float(np.linalg.norm(self.pos - start)) > 0.001 or rot is not None:
            yield from self.move(start, 0.6, rot=r)
        yield from self.hold(settle)
        base = self.tracking_error(u)
        step = float(speed) * self.dt
        depth, streak, peak, why = 0.0, 0, 0.0, "max_depth"
        while depth < max_depth - 1e-12:
            depth = min(max_depth, depth + step)
            self._set_cartesian(start + depth * u, r, self.finger)
            yield
            lag = (self.tracking_error(u) - base) * 1000.0
            peak = max(peak, lag)
            if self.offline and self.pessimistic and depth >= min_depth:
                why = "contact"
                break
            if not self.offline and depth >= min_depth and lag >= float(err_mm):
                streak += 1
                if streak >= confirm_steps:
                    why = "contact"
                    break
            else:
                streak = 0
        yield from self.hold(hold)
        res = PressResult(round(depth, 6), why, round(peak, 3))
        self.press_log.append(res)
        return res

    # -- motion ------------------------------------------------------------------------------------------------------

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

    def home(self, duration: float = 3.0) -> Generator[None, None, None]:
        """Open the fingers, then swing back to the READY joint pose in joint space, so the final frame shows the
        arm well clear of whatever it released (a short Cartesian rise leaves open fingers straddling the part)."""
        yield from self.grip(FINGER_OPEN, 0.6)
        q0, q1 = self.q.copy(), np.array(READY, dtype=float)
        steps = max(1, round(float(duration) / self.dt))
        for i in range(1, steps + 1):
            self.q = q0 + ik.smoothstep(i / steps) * (q1 - q0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base_pos)
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
        """Closed-loop step-by-step tracking of ``target_fn()`` (re-evaluated every control step).

        target_fn must be built from task constants or measured values (``measured_pos``), never from object state."""
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
    """Passed to ``demo_script(ctx)``: both ``ArmController``s plus what the robot observes.

    Privileged by default (review agent v21 allows a scripted demo to read simulator state): ``states()``, ``obj()``
    and ``pos()`` return simulator state and count each read in ``state_reads``. Pass ``strict=True`` (or
    ``privileged=False``) for a demo that should use only what the robot observes; then they raise
    ``ObservabilityError``. ``images`` (camera name to HxWx3 uint8) and ``joint_pos`` (the raw 14-D observation,
    grippers 0 closed to 1 open) are refreshed every step in a live run and are None offline.
    """

    def __init__(
        self,
        bases: dict[str, Any],
        dt: float,
        state_reader: Callable[[], dict[str, ObjectState]],
        *,
        privileged: bool | None = None,
        strict: bool | None = None,
        offline: bool = False,
        looks: Mapping[str, str] | None = None,
        pessimistic: bool = False,
    ) -> None:
        self.dt = float(dt)
        self.left = ArmController("left", bases["left"], self.dt)
        self.right = ArmController("right", bases["right"], self.dt)
        self._state_reader = state_reader
        self.step_count = 0
        self.__privileged = _resolve_privileged(privileged, strict)  # read-only after construction
        self.offline = bool(offline)
        self.images: dict[str, np.ndarray] | None = None
        self.joint_pos: np.ndarray | None = None
        self.state_reads = 0
        self._looks = dict(looks or {})
        self.looks_taken: list[tuple[str, int]] = []
        self.press_log: list[PressResult] = []
        for arm in (self.left, self.right):
            arm.offline = self.offline
            arm.pessimistic = self.offline and bool(pessimistic)
            arm.press_log = self.press_log

    @property
    def privileged(self) -> bool:
        """True unless the context was built strict. It cannot be set afterwards."""
        return self.__privileged

    @property
    def strict(self) -> bool:
        """True for a context built with strict=True (or privileged=False): state reads raise ObservabilityError."""
        return not self.__privileged

    def _guard(self, what: str) -> None:
        if not self.privileged:
            raise ObservabilityError(f"strict demo_script read simulator object state ({what}); {_OBS_HINT}")
        self.state_reads += 1

    def states(self) -> dict[str, ObjectState]:
        """Current simulator ``ObjectState``s keyed by object name (raises in strict mode)."""
        self._guard("ctx.states()")
        return self._state_reader()

    def obj(self, name: str) -> ObjectState:
        """One object's simulator state (raises in strict mode)."""
        self._guard(f"ctx.obj({name!r})")
        return self._state_reader()[name]

    def pos(self, name: str) -> np.ndarray:
        """One object's simulator position (raises in strict mode)."""
        self._guard(f"ctx.pos({name!r})")
        return np.array(self._state_reader()[name].pos, dtype=float)

    def image(self, cam: str = "top_cam") -> np.ndarray | None:
        """The latest image from ``cam``, or None offline. A demo that uses images must fall back to constants."""
        if self.images is None:
            return None
        return self.images.get(cam)

    def look(self, name: str) -> LookPose:
        """One gated, documented look at ``name``'s pose (discouraged). The task module must declare
        ``DEMO_LOOKS = {"name": "why it is visible and why the demo needs it"}``, the README must describe the one look,
        and a second look at the same name raises. Position is rounded to 1 mm and yaw to 0.5 deg."""
        if name not in self._looks:
            raise ObservabilityError(
                f"ctx.look({name!r}) is not declared in the task's DEMO_LOOKS; {_OBS_HINT}"
            )
        if any(n == name for n, _ in self.looks_taken):
            raise ObservabilityError(f"ctx.look({name!r}) was already used once; a demo gets one look per name")
        st = self._state_reader()[name]
        x, y, z, w = (float(v) for v in np.asarray(st.quat_xyzw, dtype=float))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw = math.radians(round(math.degrees(yaw) * 2.0) / 2.0)
        pose = LookPose(np.round(np.asarray(st.pos, dtype=float), 3), yaw)
        self.looks_taken.append((name, self.step_count))
        if not self.offline:
            print(f"CLLOOK {name} {self.step_count}", flush=True)
        return pose

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
        # If spec is a mechkit.Mech, populate joints=0 and origin=pos so score()/demo_script() can read .joints.
        # Its box is the rest bounding box of its shapes, turned by yaw, as the simulator reports it (mechkit
        # _mech_state): pos is that box's centre, which is not spec.pos when the shapes sit off the origin.
        if hasattr(spec, "joints") and hasattr(spec, "shapes"):
            from mechkit import MechState, _local_corners, _quat_xyzw_matrix  # local import only when Mech is used

            joints = {j.name: 0.0 for j in spec.joints}
            c = _local_corners(spec, joints) @ _quat_xyzw_matrix(quat).T + pos
            low, high = c.min(0), c.max(0)
            out[spec.name] = MechState(
                pos=(low + high) / 2,
                quat_xyzw=quat,
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
                low=low,
                high=high,
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
    dt: float = 1.0 / DEMO_HZ,
    privileged: bool | None = None,
    strict: bool | None = None,
) -> dict[str, Any]:
    """Step ``demo_script(ctx)`` offline against resting initial object poses to check IK and step budget.

    Same default as a live run: privileged, so ``ctx.pos``, ``ctx.obj`` and ``ctx.states`` return the resting
    initial poses and are counted in ``state_reads``. ``strict=True`` (or ``privileged=False``, or ``DEMO_STRICT =
    True`` in the task module when neither is passed) makes any such read raise ``ObservabilityError``. The default
    ``dt`` is 1/20 s, the rate real runs use, so ``steps`` compares directly with the run's step limit. Nothing is felt
    offline: ``holding()`` is True for any commanded close and ``press_until`` runs to ``max_depth``.

    A second, pessimistic pass then runs the demo again: the first ``holding()`` on each arm reports a missed grasp
    and ``press_until`` feels contact at ``min_depth``, so retry and contact branches run offline too (and, in strict
    mode, a state read hidden in one raises ``ObservabilityError``). The returned numbers are the first pass's;
    ``result["pessimistic"]`` summarises the second (its ``error`` is any other exception it raised, or None) and
    ``result["strict"]`` says which mode ran.
    """
    t = task_or_path if isinstance(task_or_path, task_file.TaskFile) else task_file.load(task_or_path)
    fn = getattr(t.module, "demo_script", None)
    if fn is None:
        raise ValueError(f"{t.path.name} has no demo_script(ctx)")
    bases = dict(station.BASES) if t.scene == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    mock = _mock_initial_states(t)
    privileged = _resolve_privileged(privileged, strict, default=not bool(getattr(t.module, "DEMO_STRICT", False)))

    def run(pessimistic: bool) -> tuple[ClosedLoopContext, int, list[tuple[np.ndarray, np.ndarray]]]:
        ctx = ClosedLoopContext(
            bases, dt, lambda: mock, privileged=privileged, offline=True,
            looks=getattr(t.module, "DEMO_LOOKS", None), pessimistic=pessimistic,
        )
        traj: list[tuple[np.ndarray, np.ndarray]] = []
        steps = 0
        for _ in fn(ctx):
            steps += 1
            ctx.step_count = steps
            la, ra = ctx.left.action7(), ctx.right.action7()
            ctx.left.update_measured(la)
            ctx.right.update_measured(ra)
            traj.append((la, ra))
            if steps > max_steps:
                raise RuntimeError(f"{t.path.name}: demo_script exceeded max_steps={max_steps}")
        return ctx, steps, traj

    ctx, steps, traj = run(False)
    pess: dict[str, Any] | None = None
    try:
        pctx, psteps, _ = run(True)
        pess = {"steps": psteps, "state_reads": pctx.state_reads,
                "presses": [p._asdict() for p in pctx.press_log], "error": None}
    except ObservabilityError as e:
        raise ObservabilityError(f"{e} (in dry_run's pessimistic pass: the first holding() missed and "
                                 f"press_until felt contact at min_depth)") from e
    except Exception as e:  # noqa: BLE001  reported, not raised: the first pass is the step and IK check
        pess = {"steps": None, "state_reads": 0, "presses": [], "error": f"{type(e).__name__}: {e}"}
    return {
        "strict": not privileged,
        "steps": steps,
        "duration_s": round(steps * dt, 3),
        "worst_ik_mm": max(ctx.left.worst_ik_mm, ctx.right.worst_ik_mm),
        "worst_ik_deg": max(ctx.left.worst_ik_deg, ctx.right.worst_ik_deg),
        "state_reads": ctx.state_reads,
        "looks": list(ctx.looks_taken),
        "presses": [p._asdict() for p in ctx.press_log],
        "trajectory": traj,
        "pessimistic": pess,
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
    """Diagnostic only: prints where each object is when demo_script ends. It reads through the private state reader,
    so it works on a strict context too and never counts as a demo read."""
    try:
        for name, st in ctx._state_reader().items():
            line = (f"CLSTATE {name} pos={np.round(st.pos, 4).tolist()} low={np.round(st.low, 4).tolist()} "
                    f"high={np.round(st.high, 4).tolist()}")
            joints = getattr(st, "joints", None)
            if joints:
                line += f" joints={ {k: round(float(v), 4) for k, v in joints.items()} }"
            print(line, flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"CLSTATE unavailable: {e}", flush=True)
    for side in ARMS:
        try:
            arm = getattr(ctx, side)
            print(f"CLARM {side} cmd={np.round(arm.pos, 4).tolist()} meas={np.round(arm.measured_pos, 4).tolist()} "
                  f"gap_mm={1000 * arm.gap():.1f} terr_mm={1000 * arm.tracking_error():.1f}", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"CLARM {side} unavailable: {e}", flush=True)


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
        module = getattr(self._task, "module", None)
        fn = getattr(module, "demo_script", None)
        if fn is not None:
            def reader() -> dict[str, ObjectState]:
                # Look the embodiment up here, not at policy reset: rollout.py resets the policy first, so on a
                # first trial it is not registered yet and every read would return start-of-run poses.
                emb = _ACTIVE_EMBODIMENTS.get(self._environment_id or "")
                if emb is not None and emb._env is not None and emb.task is not None:
                    from yam_arena.scene import object_states

                    return object_states(emb._env, emb.task.names)
                return _mock_initial_states(self._task)

            # Privileged unless the task opts into strict mode with DEMO_STRICT = True (review agent v21 allows a
            # scripted demo to read simulator state).
            self._cl_ctx = ClosedLoopContext(
                self._bases, self._dt, reader, strict=bool(getattr(module, "DEMO_STRICT", False)), offline=False,
                looks=getattr(module, "DEMO_LOOKS", None),
            )
            self._cl_gen = fn(self._cl_ctx)
            return
        return orig_reset(self, scene)

    def act(self, observation):
        if self._cl_ctx is None:
            return orig_act(self, observation)
        st = getattr(observation, "state", None)
        raw14 = st.get("joint_pos") if isinstance(st, dict) else None
        if raw14 is not None and len(raw14) == 14:
            self._cl_ctx.left.update_measured(raw14[:7])
            self._cl_ctx.right.update_measured(raw14[7:14])
            self._cl_ctx.joint_pos = np.asarray(raw14, dtype=float)
        else:
            self._cl_ctx.joint_pos = None
        self._cl_ctx.images = getattr(observation, "images", None)
        t = float(observation.state_time) + self._dt
        if self._cl_gen is not None:
            try:
                next(self._cl_gen)
                self._cl_ctx.step_count += 1
            except StopIteration:
                self._cl_gen = None
                self._cl_done_at = t
                _print_states(self._cl_ctx)
            except ObservabilityError as e:
                print(f"CLOBS {e}", flush=True)
                _print_states(self._cl_ctx)
                raise
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
