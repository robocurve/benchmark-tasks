"""Inspect Robots embodiment ``yam_memory``: ``yam_arena`` playing a memory task file.

    inspect-robots run --embodiment yam_memory -E task=tasks/t01_box_retrieval/task.py --policy rta_oracle ...
    inspect-robots run --task rta/box_retrieval --embodiment yam_memory --policy rta_oracle ...

With ``--task``, the registered task's scenes carry the task file in ``metadata["task_file"]`` and the
embodiment loads it on the first ``reset()``. Every step reports ``info["q_score"]`` (latched milestones /
milestones, 1.0 on success), ``info["milestones"]``, ``info["success"]`` and ``info["phase"]``
(``"observe"`` while the arms are held, then ``"act"``).
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np
from inspect_robots import Scene
from inspect_robots.embodiment import PRIVILEGED_SUCCESS, RESETTABLE, SEEDABLE
from inspect_robots.types import StepResult
from inspect_robots_yam.packing import split
from yam_arena import station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import JOINT_GUIDE, ROOM_DOCS, YamArenaEmbodiment
from yam_arena.rig import TABLETOP

from rta import registry, spec

ARMS = ("left_robot", "right_robot")
SETTLE_S = 0.5
RESET_RENDERS = 4
OBSERVE_RULE = (
    "Memory task: for the first {act:.0f} seconds after reset the arms are held still in the ready pose while the "
    "scene is set up in front of the cameras (an object is hidden, a cue is shown and removed). Watch and "
    "remember; your actions take effect from {act:.0f} s. The information you need is no longer visible then."
)


def environment_id(rig_name: str, task: spec.MemoryTask | None) -> str:
    base = f"isaaclab-arena-0.3.1-yam-bimanual-{rig_name}-memory"
    return base if task is None else f"{base}-{task.name}"


class YamMemoryEmbodiment(YamArenaEmbodiment):
    """The bimanual YAM playing a memory task: scripted events, held arms, latched milestones."""

    def __init__(
        self, *, task: str | None = None, scene: str | None = None, object_jitter: float = 0.0, trace_dir: str | None = None, **kwargs
    ) -> None:
        memory = spec.load(task) if task else None
        self.trace_dir = Path(trace_dir).expanduser() if trace_dir else None
        self._trace: Any = None
        self._episode = 0
        self._last_command: list[float] | None = None
        self._last_snapshot: spec.Snapshot | None = None
        super().__init__(task=None, scene=(memory.scene if memory else (scene or "station")), object_jitter=object_jitter, **kwargs)
        self.memory: spec.MemoryTask | None = None
        self._ids_carried: set[str] = set()  # every environment_id this embodiment has reported (policies bind early)
        self._state: dict = {}
        self._latched: list[bool] = []
        self._world: Any = None
        self._describe(memory)

    # -- task binding ------------------------------------------------------------------

    def _describe(self, memory: spec.MemoryTask | None) -> None:
        """Adopt a task file: docs, environment id and capabilities follow it."""
        self.memory = memory
        if memory is None:
            docs = "\n\n".join((JOINT_GUIDE, ROOM_DOCS[self.scene]))
            caps = frozenset({SEEDABLE, RESETTABLE})
        else:
            rule = OBSERVE_RULE.format(act=memory.act_after_s) if memory.act_after_s > 0 and memory.memory else None
            docs = "\n\n".join(d for d in (JOINT_GUIDE, ROOM_DOCS[memory.scene], rule, memory.docs) if d)
            caps = frozenset({SEEDABLE, RESETTABLE, PRIVILEGED_SUCCESS})
        self.info = dataclasses.replace(
            self.info,
            name="yam_memory",
            capabilities=caps,
            supported_target_kinds=frozenset(),
            docs=docs,
            environment_id=environment_id(self.rig.name, memory),
            environment_revision=None if memory is None else memory.sha256[:12],
        )
        self._ids_carried.add(self.info.environment_id)
        if memory is not None:
            for env_id in self._ids_carried:
                registry.register_task(env_id, memory)

    def _bind_scene_task(self, scene: Scene) -> None:
        path = (scene.metadata or {}).get("task_file")
        if self.memory is None:
            if not path:
                raise ValueError("yam_memory needs a task: -E task=<file.py>, or a registered task whose scenes carry metadata['task_file']")
            memory = spec.load(path)
            if memory.scene != self.scene:
                if self._env is not None:
                    raise ValueError(f"{memory.path.name} sets SCENE={memory.scene!r}; the simulator is already built for {self.scene!r}")
                # Kit has not started: adopt the task file's room and rig.
                self.scene = memory.scene
                self.rig = station.rig("calibrated") if memory.scene == "station" else TABLETOP
            self._describe(memory)
        elif path and Path(path).resolve() != self.memory.path:
            raise ValueError(f"this embodiment plays {self.memory.path.name}, the scene asks for {path}")

    # -- Isaac lifecycle ---------------------------------------------------------------

    def _ensure_sim(self, goal=None) -> None:
        import yam_arena.inspect_embodiment as yam_mod

        if self._env is not None:
            return
        memory = self.memory
        assert memory is not None
        import os

        os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        if yam_mod._APP is None:
            from isaaclab.app import AppLauncher

            kwargs: dict[str, Any] = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                kwargs["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = yam_mod.window_display(os.environ)
            yam_mod._APP = AppLauncher(**kwargs).app
            import atexit

            atexit.register(yam_mod._close_app_at_exit)

        import torch

        from yam_arena.embodiment import BimanualYamEmbodiment

        from rta.scene import make_memory_env
        from rta.world import World

        emb = BimanualYamEmbodiment(enable_cameras=True, wrist_cam_mount=self.wrist_cam_mount, contact_sensors=False, rig=self.rig)
        self._env = make_memory_env(emb, memory, device=self.device, control_hz=self.info.control_hz)
        self._base = self._env.unwrapped
        self._torch = torch
        self._free_objects = memory.free_objects
        self._ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=self._base.device)
        self._ids = {
            name: (
                self._base.scene[name].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[name].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for name in ARMS
        }
        self._world = World(self._env, torch)
        self._world.objects = None

    # -- Embodiment contract -----------------------------------------------------------

    def reset(self, scene: Scene, *, seed: int | None = None) -> Any:
        self._bind_scene_task(scene)
        self._ensure_sim()
        memory = self.memory
        assert memory is not None
        base = self._base
        self._instruction = scene.instruction or memory.instruction
        self._held, self._latched = 0, [False] * len(memory.milestone_names)
        self._last_snapshot = None
        self._prev_pos = None
        self._env.reset()
        rng = np.random.default_rng(seed)
        trial = spec.Trial(scene_id=scene.id, init_seed=scene.init_seed, seed=seed)
        self._state = dict(memory.setup(rng, trial))
        for env_id in self._ids_carried:
            registry.publish_state(env_id, self._state)
        from rta.scene import memory_states

        placed = {}
        if memory.placements is not None:
            for name, pose in memory.placements(self._state).items():
                if name not in memory.free_objects:
                    raise ValueError(f"{memory.name}: placements() moves {name!r}, which is not a free object")
                pose = tuple(float(v) for v in pose)
                self._world.set_pose(name, pose[:3], pose[3] if len(pose) == 4 else 0.0)
                placed[name] = list(pose)
        if memory.forces is not None:  # hold the start forces through the settle (a magnet parked on a board)
            start = spec.Snapshot(memory_states(self._env, memory), t=0.0)
            for name, force in memory.forces(start, self._state).items():
                self._world.push(name, force)
        for _ in range(max(1, round(SETTLE_S / base.step_dt))):
            self._env.step(self._ready)
        for _ in range(RESET_RENDERS):
            base.sim.render()
        self._t = 0.0
        after_settle = {n: [round(float(v), 4) for v in st.pos] for n, st in memory_states(self._env, memory).items()}
        self._reset_log = {"placed": placed, "after_settle": after_settle,
                           "env_origin": [float(v) for v in base.scene.env_origins[0].tolist()],
                           "rigid_props": self._rigid_props(memory.free_objects)}
        self._open_trace(scene)
        return self._observation()

    _RIGID_ATTRS = ("solverPositionIterationCount", "solverVelocityIterationCount", "linearDamping", "angularDamping")
    _RIGID_ATTRS_SNAKE = ("solver_position_iteration_count", "solver_velocity_iteration_count", "linear_damping", "angular_damping")

    def _rigid_props(self, names) -> dict:
        """What reached each free object: the ``rigid_props`` on the spawn cfg Isaac Lab was given, and the
        PhysX rigid-body attributes authored on the object's prim (env 0) and on any descendant carrying
        ``UsdPhysics.RigidBodyAPI``. Shows whether ``scene.SOLVER_ITERS`` reached the bodies. A missing attribute
        reads ``None``; a probe error is recorded as text."""
        out = {}
        try:
            from isaaclab.sim.utils import find_matching_prims
            from pxr import Usd, UsdPhysics

            def authored(prim):
                return {k: (prim.GetAttribute(f"physxRigidBody:{k}").Get() if prim.HasAttribute(f"physxRigidBody:{k}") else None)
                        for k in self._RIGID_ATTRS}

            for name in names:
                cfg = self._base.scene[name].cfg
                spawn_rp = getattr(getattr(cfg, "spawn", None), "rigid_props", None)
                entry = {"cfg": {k: getattr(spawn_rp, k, None) for k in self._RIGID_ATTRS_SNAKE} if spawn_rp is not None else None}
                prims = find_matching_prims(cfg.prim_path)
                if not prims:
                    entry["prim"] = "no prim"
                else:
                    root = prims[0]
                    entry["prim"] = str(root.GetPath())
                    entry["root_has_api"] = bool(root.HasAPI(UsdPhysics.RigidBodyAPI))
                    entry["root"] = authored(root)
                    entry["bodies"] = {str(p.GetPath()): authored(p) for p in Usd.PrimRange(root)
                                       if p != root and p.HasAPI(UsdPhysics.RigidBodyAPI)}
                out[name] = entry
        except Exception as e:  # a probe must not end a run; the trace carries the reason instead
            out["error"] = f"{type(e).__name__}: {e}"
        return out

    def _snapshot(self, objects) -> spec.Snapshot:
        from dataclasses import replace

        from yam_arena.runtime import link_poses, tcp_world

        base = self._base
        # Object speed = displacement of the bounding-box centre over the control step. PhysX's reported root
        # velocity is not what "resting" means: a body pinned between fixtures read 0.24 m/s with its position
        # constant (S5), and solver rattle in pockets reads 4-7 cm/s. Predicates use lin_vel via resting().
        prev = getattr(self, "_prev_pos", None)
        dt = base.step_dt
        if prev is not None:
            objects = {n: (replace(st, lin_vel=(st.pos - prev[n]) / dt) if n in prev else st) for n, st in objects.items()}
        self._prev_pos = {n: np.array(st.pos, dtype=float) for n, st in objects.items()}
        tcp, finger, tcp_rot = {}, {}, {}
        for name in ARMS:
            arm = name.split("_")[0]
            tcp[arm] = tcp_world(base.scene[name])
            tcp_rot[arm] = link_poses(base.scene[name])["gripper"][:3, :3]
            _, finger_ids = self._ids[name]
            finger[arm] = float(base.scene[name].data.joint_pos.torch[0][finger_ids].mean().item())
        joints = {}
        for name, listed in getattr(getattr(self.memory, "module", None), "JOINTS", {}).items():
            joints[name] = {j: float(base.arena_world.get_joint_position(name, j)[0].item()) for j in listed}
        return spec.Snapshot(objects, t=self._t, tcp=tcp, finger=finger, joints=joints, tcp_rot=tcp_rot)

    # -- per-step trace (debugging and the task's auditable event log) ----------------

    def _open_trace(self, scene: Scene) -> None:
        if self._trace is not None:
            self._trace.close()
            self._trace = None
        if self.trace_dir is None:
            return
        import json

        self.trace_dir.mkdir(parents=True, exist_ok=True)
        self._episode += 1
        path = self.trace_dir / f"{self.memory.name}_{scene.id}_{self._episode:03d}.jsonl"
        self._trace = path.open("w")
        public = {k: v for k, v in self._state.items()}
        self._trace.write(json.dumps({"kind": "header", "task": self.memory.name, "scene": scene.id, "state": public,
                                      "act_after_s": self.memory.act_after_s, "control_hz": self.info.control_hz,
                                      "reset": getattr(self, "_reset_log", None),
                                      "joint_limits": {"low": [float(v) for v in self._low[:6]], "high": [float(v) for v in self._high[:6]]}}) + "\n")

    def _trace_step(self, states, info: dict) -> None:
        if self._trace is None:
            return
        import json

        grip = {f"{arm}_robot": v for arm, v in states.finger.items()}
        objects = {
            n: {"pos": [round(float(v), 4) for v in s.pos], "quat_xyzw": [round(float(v), 4) for v in s.quat_xyzw],
                "low_z": round(float(s.low[2]), 4), "speed": round(float(np.linalg.norm(s.lin_vel)), 4)}
            for n, s in states.items()
        }
        tcp = {f"{arm}_robot": [round(float(v), 4) for v in v3] for arm, v3 in states.tcp.items()}
        q = {}
        for name in ARMS:  # measured arm joints (rad), for reach/limit diagnosis
            arm_ids, _ = self._ids[name]
            q[name] = [round(float(v), 4) for v in self._base.scene[name].data.joint_pos.torch[0][arm_ids].tolist()]
        row = {"kind": "step", **{k: v for k, v in info.items()}, "objects": objects, "finger_m": grip, "tcp": tcp, "q": q,
               "command": self._last_command}
        if states.joints:  # articulated models' joints (m or rad)
            row["joints"] = {n: {j: round(float(x), 5) for j, x in js.items()} for n, js in states.joints.items()}
        scalars = {k: v for k, v in (self._state or {}).items() if isinstance(v, (bool, int, float, str)) or v is None}
        if scalars:  # the task's own record (times of its events), as it stands after this step
            row["task_state"] = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in scalars.items()}
        self._trace.write(json.dumps(row) + "\n")
        self._trace.flush()

    def step(self, action: Any) -> StepResult:
        from rta.scene import memory_states

        memory = self.memory
        assert memory is not None
        torch = self._torch
        t = self._t
        observing = t < memory.act_after_s
        interrupted = (
            not observing
            and memory.hold is not None
            and self._last_snapshot is not None
            and bool(memory.hold(self._last_snapshot, self._state))
        )
        if observing or interrupted:
            a = self._ready
            self._last_command = None
        else:
            target = np.clip(np.asarray(action.data, dtype=np.float64).reshape(-1), self._low, self._high)
            left, right = split(target)
            a = torch.tensor(np.concatenate([left, right]), dtype=torch.float32, device=self._base.device).unsqueeze(0)
            self._last_command = [round(float(v), 4) for v in target]
        self._world.objects = self._last_snapshot
        memory.events(t, self._state, self._world)
        if memory.forces is not None and self._last_snapshot is not None:
            for name, force in memory.forces(self._last_snapshot, self._state).items():
                self._world.push(name, force)
        self._env.step(a)
        self._t += self._base.step_dt
        states = self._snapshot(memory_states(self._env, memory))
        self._last_snapshot = states
        phase = "observe" if observing else ("hold" if interrupted else "act")
        info: dict[str, Any] = {"phase": phase, "t": self._t}
        success, reason = False, None
        if phase == "act":
            flags = list(memory.milestones(states, self._state))
            # events latch, end states are re-read (the task's LATCHED); the full score only after the success hold
            self._latched, self._held, success, _ = memory.score_step(
                self._latched, flags, self._held, bool(memory.success(states, self._state)), self.info.control_hz)
            if not success and memory.failure is not None:
                reason = memory.failure(states, self._state)
                if reason is not None and not isinstance(reason, str):
                    raise TypeError(f"{memory.name}: failure() must return a reason string or None")
        zeroed = reason is not None and reason in memory.zero_on
        q = 1.0 if success else (0.0 if zeroed else memory.q_score(self._latched))
        info.update(
            milestones=dict(zip(memory.milestone_names, self._latched, strict=True)),
            q_score=q,
            success=success,
            failure=reason,
            zeroed=zeroed,
        )
        self._trace_step(states, info)
        return StepResult(
            observation=self._observation(),
            terminated=success or reason is not None,
            termination_reason="success" if success else reason,
            info=info,
        )


def yam_memory_embodiment(**kwargs: Any) -> YamMemoryEmbodiment:
    """Embodiment ``yam_memory``; ``-E task=file.py`` picks the memory task (or a registered task supplies it)."""
    return YamMemoryEmbodiment(**kwargs)
