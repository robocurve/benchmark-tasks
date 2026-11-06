"""refitkit: the pulley, its screws, the stand, the rack and the mats for this task file, on yam_arena.

A `yam_arena` task file declares rigid shapes. This task needs a real part's mesh and bored colliders, so `RefitPart` subclasses `Box` so yam_arena's task-file checks accept
it, and importing this module hooks `yam_arena.scene` (it modifies nothing on disk) so each part spawns from its USD file and `object_states` reports the task's state.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import json
import math
import os
import pathlib
import sys
import tempfile
import time
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
from yam_arena.task_file import Box, ObjectState

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import refit_usd as u  # noqa: E402
from refit_score import NEW_MAT_XY, PULLEY_TOP, RACK_TOP, RACK_XY, SCREW_LENGTH, SCREW_NAME, STAND_TOP, STAND_XY, TOP, WORN_MAT_XY, Body, Scorer, hole_world, slot_world  # noqa: E402

SCENE = "station"
REST_GAP = 0.0003
ROBOTS = {"left": "left_robot", "right": "right_robot"}
FINGER_TIP = 0.147  # fingertips along the `gripper` link's z


@dataclass(frozen=True)
class RefitPart(Box):
    """One prim of the scene. A `Box` only so that yam_arena's checks accept it: `pos` and `size` are bounds."""

    kind: str = ""
    length: float = 0.0


@dataclass(frozen=True)
class RefitState(ObjectState):
    report: dict = field(default_factory=dict)


def parts() -> list[RefitPart]:
    """The task file's OBJECTS. `REFIT_START=open` starts with the pulley off the stand and the screws in the rack, for looking at the holes."""
    opened = os.environ.get("REFIT_START") == "open"
    worn_on_stand = (*STAND_XY, STAND_TOP + REST_GAP)
    worn_on_mat = (*WORN_MAT_XY, TOP + u.MAT_H + REST_GAP)
    out = [RefitPart("refit_stand", pos=(*STAND_XY, TOP + u.STAND[2] / 2), size=u.STAND, kind="stand"),
           RefitPart("refit_rack", pos=(*RACK_XY, TOP + u.RACK_TOP_H / 2), size=(0.03, 2 * u.RACK_END, u.RACK_TOP_H), kind="rack"),
           RefitPart("mat_worn", pos=(*WORN_MAT_XY, TOP + u.MAT_H / 2), size=(0.2, 0.2, u.MAT_H), kind="mat"),
           RefitPart("mat_new", pos=(*NEW_MAT_XY, TOP + u.MAT_H / 2), size=(0.2, 0.2, u.MAT_H), kind="mat"),
           RefitPart("pulley_worn", pos=worn_on_mat if opened else worn_on_stand, size=(0.16, 0.16, u.PULLEY_THICK), mass=u.PULLEY_MASS, kind="worn"),
           RefitPart("pulley_new", pos=(*NEW_MAT_XY, TOP + u.MAT_H + REST_GAP), size=(0.16, 0.16, u.PULLEY_THICK), mass=u.PULLEY_MASS, kind="new")]
    for i, hole in enumerate(u.SCREW_HOLES):
        name = SCREW_NAME[hole]
        if opened:
            pos = (*slot_world(i), RACK_TOP + REST_GAP)
        else:
            x, y = hole_world(hole)
            pos = (x, y, PULLEY_TOP + REST_GAP + REST_GAP)
        length = SCREW_LENGTH[name]
        out.append(RefitPart(name, pos=pos, size=(0.014, 0.014, length), mass=u.SCREW_MASS[length], kind="screw", length=length))
    return out


_NAMES = {p.name for p in parts()}
_usd: dict[str, str] = {}


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"refit_screws_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _arena_object(spec: RefitPart):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    d = _usd_dir()
    table = lambda xy: Pose(position_xyz=(float(xy[0]), float(xy[1]), float(TOP)))  # noqa: E731
    if spec.kind == "stand":
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=u.write_stand_usda(os.path.join(d, "stand.usda")), initial_pose=table(STAND_XY))
    if spec.kind == "rack":
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=u.write_rack_usda(os.path.join(d, "rack.usda")), initial_pose=table(RACK_XY))
    if spec.kind == "mat":
        root = "MatWorn" if spec.name == "mat_worn" else "MatNew"
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=u.write_mat_usda(os.path.join(d, f"{spec.name}.usda"), root), initial_pose=table(WORN_MAT_XY if spec.name == "mat_worn" else NEW_MAT_XY))
    pose = Pose(position_xyz=tuple(float(v) for v in spec.pos))
    if spec.kind in ("worn", "new"):
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=u.write_pulley_usda(os.path.join(d, f"{spec.name}.usda"), spec.kind == "worn"), initial_pose=pose)
    key = f"screw_{int(spec.length * 1000)}"
    if key not in _usd:
        _usd[key] = u.write_screw_usda(os.path.join(d, f"{key}.usda"), spec.length)
    return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=_usd[key], initial_pose=pose)


# -- the score --------------------------------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotate(q, v):
    """Rotate v by the unit quaternion q (x, y, z, w)."""
    a, w = q[:3], q[3]
    return v + 2.0 * np.cross(a, np.cross(a, v) + w * v)


class _Report:
    """Reads the scene once per control step, hands it to the scorer, and writes the task's state to refit_scores/ every 0.5 s and when the score changes, because the rollout's log keeps only the final score."""

    def __init__(self, env):
        self.base = env.unwrapped
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        root = os.environ.get("REFIT_SCORE_DIR", str(pathlib.Path(__file__).parent / "refit_scores"))
        os.makedirs(root, exist_ok=True)
        self.path = os.path.join(root, f"score_{stamp}.json")
        self.scorer = Scorer(self.base.step_dt)
        self.last_buf = -1
        self.reset()

    def reset(self) -> None:
        """A new trial: nothing kept from the last one."""
        self.scorer.reset()
        self.trace: list[dict] = []
        self.last: dict | None = None
        self.wall: list[float] = []
        self.worst_30 = 0.0
        self.deepest = {"shank_into_throat_mm": 0.0, "head_into_pulley_mm": 0.0}

    def _body(self, name: str) -> Body:
        s = self.base.scene[name]
        o = self.base.scene.env_origins[0]
        p = (_t(s.data.root_pos_w)[0] - o).cpu().numpy().astype(float)
        q = _t(s.data.root_quat_w)[0].cpu().numpy().astype(float)
        v = _t(s.data.root_lin_vel_w)[0].cpu().numpy().astype(float)
        return Body(p, _rotate(q, np.array([0.0, 0.0, 1.0])), float(np.linalg.norm(v)))

    def _fingertips(self, name: str) -> np.ndarray:
        robot = self.base.scene[name]
        o = self.base.scene.env_origins[0]
        i = robot.find_bodies("gripper")[0][0]
        gp = (_t(robot.data.body_pos_w)[0, i] - o).cpu().numpy().astype(float)
        gq = _t(robot.data.body_quat_w)[0, i].cpu().numpy().astype(float)
        f = abs(float(_t(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]].item()))
        return np.array([gp + _rotate(gq, np.array((s * f, 0.0, FINGER_TIP))) for s in (-1, 1)])

    def _check(self, screws: dict[str, Body], pulleys: list[Body]) -> None:
        """Physics check, run with `REFIT_CHECK=1` at `-E control_hz=240`: how far a shank goes past the flats of a hole in a pulley on the stand, and a head into that pulley, in mm so far."""
        for pulley in pulleys:
            base = pulley.pos
            if np.linalg.norm(base[:2] - np.array(STAND_XY)) > 0.01 or abs(base[2] - STAND_TOP) > 0.003:
                continue
            for name, b in screws.items():
                if b.up[2] < 0.7:
                    continue
                for hole in u.SCREW_HOLES:
                    x, y = u.hole_xy(hole)
                    for height in (base[2], base[2] + u.PULLEY_THICK):
                        d_along = (b.pos[2] - height) / b.up[2]
                        if not 0.0 <= d_along <= SCREW_LENGTH[name] - 0.003:  # the part of the shank that is straight
                            continue
                        at = b.pos - b.up * d_along
                        dx, dy = at[0] - base[0] - x, at[1] - base[1] - y
                        d = math.hypot(dx, dy)
                        if d < 0.006:
                            off = (math.degrees(math.atan2(dy, dx)) - u.HOLE_DEG[hole]) % 45.0
                            flats = d * math.cos(math.radians(min(off, 45.0 - off)))
                            self.deepest["shank_into_throat_mm"] = max(self.deepest["shank_into_throat_mm"], (flats + u.SHANK_R - u.THROAT) * 1000.0)
                    if np.linalg.norm(b.pos[:2] - base[:2] - np.array((x, y))) < 0.006:
                        self.deepest["head_into_pulley_mm"] = max(self.deepest["head_into_pulley_mm"], (base[2] + u.PULLEY_THICK - b.pos[2]) * 1000.0)

    def step(self) -> dict:
        buf = int(self.base.episode_length_buf[0])
        if buf < self.last_buf:
            self.reset()
        self.last_buf = buf
        now = time.monotonic()
        self.wall.append(now)
        if len(self.wall) > 30:
            span = now - self.wall.pop(0)
            if self.scorer.steps > 90:  # after the renderer's first frames
                self.worst_30 = max(self.worst_30, span)
        worn, new = self._body("pulley_worn"), self._body("pulley_new")
        screws = {n: self._body(n) for n in SCREW_NAME.values()}
        tips = {side: self._fingertips(name) for side, name in ROBOTS.items()}
        rep = self.scorer.update(worn, new, screws, np.vstack(list(tips.values())))
        new_q = _t(self.base.scene["pulley_new"].data.root_quat_w)[0].cpu().numpy().astype(float)
        across = _rotate(new_q, np.array([1.0, 0.0, 0.0]))
        rep.update({"worn": [round(float(v), 4) for v in worn.pos], "new": [round(float(v), 4) for v in new.pos],
                    "new_yaw_deg": round(math.degrees(math.atan2(across[1], across[0])), 2), "new_tilt_deg": round(math.degrees(math.acos(min(1.0, float(new.up[2])))), 2),
                    "screws": {n: {"pos": [round(float(v), 4) for v in b.pos], "up_z": round(float(b.up[2]), 4)} for n, b in screws.items()},
                    "tips": {s: np.round(t, 4).tolist() for s, t in tips.items()}, "wall_s_per_30_steps_worst": round(self.worst_30, 2)})
        if os.environ.get("REFIT_CHECK"):
            self._check(screws, [worn, new])
            rep["deepest"] = {k: round(v, 3) for k, v in self.deepest.items()}
        if os.environ.get("REFIT_STEP_FILE"):  # for a watcher outside the run: the control step, written every step
            with open(os.environ["REFIT_STEP_FILE"], "w") as f:
                f.write(str(rep["step"]))
        if rep["step"] % 10 == 0 or self.last is None or self.last["score"] != rep["score"]:
            rep["wall_s"] = round(now, 2)
            self.trace.append(rep)
            with open(self.path, "w") as f:
                json.dump({"final": rep, "trace": self.trace}, f, indent=0)
        self.last = rep
        return rep


_report: _Report | None = None


def _patch_scene(scene) -> None:
    if getattr(scene, "_refitkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, RefitPart) else orig_task_object(spec)

    def object_states(env, names):
        global _report
        if _report is None or _report.base is not env.unwrapped:
            _report = _Report(env)
        rep = _report.step()
        others = [n for n in names if n not in _NAMES]
        out = orig_states(env, others) if others else {}
        zero = np.zeros(3)
        for n in names:
            if n in _NAMES:
                out[n] = RefitState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=rep)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._refitkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, orig_exec = spec.loader, spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena.scene (now if already imported, else when it is). Idempotent."""
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
