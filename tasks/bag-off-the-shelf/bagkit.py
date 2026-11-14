"""bagkit: the shelf, stiff tote, soft strap and rake for this task file, on yam_arena.

A `yam_arena` task file declares rigid shapes. This task needs a tote and a rake of several shapes, a soft strap sewn to
them, and a scripted demo with level grips, so this module does what `cablekit`-style helpers do for cables: `BagPart`
subclasses `Box` so yam_arena's task-file checks accept it, and importing this module hooks `yam_arena.scene` (it modifies
nothing on disk) so each part spawns from its USD file and `object_states` reports the task's state. `plan()` makes the
`yam_scripted` policy play this task's own keyframes, which can hold any grip orientation and move freely in joint space;
yam_arena's own plans only grip straight down, which the arm cannot do at the height of the shelf.

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
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, ObjectState

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from parts.strap import ToteSpec, strap_legs, thicken  # noqa: E402
from parts.tote import ARCH_TUBE, WALL, handle_axis_points, write_arch_usda, write_tote_usda  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SHELF_TOP = TOP + 0.28  # the deck, 28 cm above the table
SHELF_X = (0.28, 0.64)
SHELF_Y = (-0.24, 0.24)  # between the arms' start poses: at READY their open fingers reach x 0.345, y +-0.26, below deck height
BOARD = 0.06  # depth of the deck's frame and of its collider
SPEC = ToteSpec()
BAG = np.array([0.35, 0.0, SHELF_TOP + 0.002])  # the tote's origin: the centre of its floor's underside
RAKE_IN_BAG_X = 0.12
RAKE_POS = (float(BAG[0] + RAKE_IN_BAG_X), 0.0, SHELF_TOP)
RAKE_YAW = -math.pi / 2  # the handle points to the robot's right, under the strap's right leg
# What is in the tote: `xy` is the item's centre in the tote's frame.
ITEMS = [
    {"name": "book", "shape": "box", "size": (0.02, 0.12, 0.15), "xy": (0.030, 0.0), "z0": 0.0, "color": (0.16, 0.36, 0.30), "mass": 0.10},
    {"name": "bottle", "shape": "cylinder", "radius": 0.024, "height": 0.22, "xy": (-0.015, -0.058), "z0": 0.0, "color": (0.60, 0.80, 0.92), "mass": 0.15},
    {"name": "can", "shape": "cylinder", "radius": 0.025, "height": 0.09, "xy": (-0.015, 0.058), "z0": 0.0, "color": (0.75, 0.10, 0.10), "mass": 0.10},
    {"name": "juice", "shape": "box", "size": (0.03, 0.04, 0.07), "xy": (-0.017, 0.0), "z0": 0.0, "color": (0.95, 0.65, 0.10), "mass": 0.10},
]
ROBOTS = {"left": "left_robot", "right": "right_robot"}
TCP_IN_GRIPPER = (0.0, 0.0, 0.134)


@dataclass(frozen=True)
class BagPart(Box):
    """One prim of the scene. A `Box` only so that yam_arena's checks accept it: `pos` and `size` are bounds."""

    kind: str = ""
    args: dict = field(default_factory=dict)


@dataclass(frozen=True)
class BagState(ObjectState):
    report: dict | None = None


def parts() -> list[BagPart]:
    """The task file's OBJECTS. The strap comes last: it is sewn to the tote and the arch."""
    x0, x1 = SHELF_X
    y0, y1 = SHELF_Y
    xc, yc, d, w = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    leg = 0.045
    under = SHELF_TOP - BOARD
    out = [BagPart("shelf_deck", pos=(xc, yc, SHELF_TOP - BOARD / 2), size=(d, w, BOARD), kind="collider")]
    for i, (x, y) in enumerate([(x0, y0), (x0, y1), (x1, y0), (x1, y1)]):
        px, py = x + (leg / 2 if x == x0 else -leg / 2), y + (leg / 2 if y == y0 else -leg / 2)
        out.append(BagPart(f"shelf_leg_{i}", pos=(px, py, (under + TOP) / 2), size=(leg, leg, under - TOP), kind="collider"))
    out.append(BagPart("shelf_steel", pos=(xc, yc, (SHELF_TOP + TOP) / 2), size=(d, w, SHELF_TOP - TOP), kind="steel"))
    out.append(BagPart("tote", pos=tuple(BAG + [0.0, 0.0, SPEC.height / 2]), size=(SPEC.depth, SPEC.width, SPEC.height), kind="tote"))
    out.append(BagPart("back_arch", pos=tuple(BAG + [0.17, 0.0, 0.006]), size=(0.08, 0.13, 0.012), kind="arch"))
    floor = BAG[2] + WALL + 0.002
    for it in ITEMS:
        h = it["size"][2] if it["shape"] == "box" else it["height"]
        out.append(BagPart(it["name"], pos=(BAG[0] + it["xy"][0], BAG[1] + it["xy"][1], floor + it["z0"] + h / 2), size=(0.05, 0.05, h),
                           kind="item", args=it))
    out.append(BagPart("rake", pos=RAKE_POS, size=(0.14, 0.14, 0.12), yaw=RAKE_YAW, kind="rake"))
    out.append(BagPart("straps", pos=tuple(BAG + [0.1, 0.0, 0.1]), size=(0.2, 0.15, 0.2), kind="straps"))
    return out


_NAMES = {p.name for p in parts()}
_usd_cache: dict[str, str] = {}


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"bag_off_the_shelf_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _arena_object(spec: BagPart):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from parts.items import write_item_usda
    from parts.rake import RakeSpec, write_rake_usda
    from parts.shelf import shelf_parts, write_shelf_usda

    d = _usd_dir()
    origin = Pose(position_xyz=tuple(float(v) for v in BAG))
    if spec.kind == "collider":  # invisible: the visual steel is `shelf_steel`
        cfg = sim_utils.CuboidCfg(size=tuple(spec.size), visible=False, collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.6),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5)))
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=Pose(position_xyz=tuple(spec.pos)))
    if spec.kind == "steel":
        usd = write_shelf_usda(os.path.join(d, "shelf.usda"), shelf_parts(SHELF_X, SHELF_Y, SHELF_TOP, BOARD, TOP))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0)))
    if spec.kind == "tote":
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=write_tote_usda(os.path.join(d, "tote.usda"), SPEC), initial_pose=origin)
    if spec.kind == "arch":
        _, _, _, (a, b) = strap_legs(SPEC, RAKE_IN_BAG_X)
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=write_arch_usda(os.path.join(d, "arch.usda"), "ArchBack", a, b, "out"), initial_pose=origin)
    if spec.kind == "item":
        usd = write_item_usda(os.path.join(d, f"item_{spec.name}.usda"), spec.args, spec.args["mass"])
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos)))
    if spec.kind == "rake":
        usd = write_rake_usda(os.path.join(d, "rake.usda"), RakeSpec())
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd,
                      initial_pose=Pose(position_xyz=RAKE_POS, rotation_xyzw=(0.0, 0.0, math.sin(RAKE_YAW / 2), math.cos(RAKE_YAW / 2))))
    return _straps(spec, origin, d)


def _straps(spec: BagPart, origin, d: str):
    """The strap's legs as a surface-deformable cloth, sewn to the tote's rim and to the arch handle's ends."""
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
    from parts.cloth import ClothUsdFileCfg
    from parts.usd_mesh import write_mesh_usda

    legs, rim, ends, _ = strap_legs(SPEC, RAKE_IN_BAG_X)
    v = np.asarray(legs.vertices)
    pos = lambda idx: tuple(tuple(float(c) for c in v[i]) for i in idx)
    sew = (("tote", tuple(int(i) for i in rim), pos(rim)), ("back_arch", tuple(int(i) for i in ends), pos(ends)))
    return DeformableObject(
        name=spec.name,
        spawner_cfg=ClothUsdFileCfg(
            usd_path=write_mesh_usda(legs, os.path.join(d, "straps.usda"), "Straps"),
            visual_usd_path=write_mesh_usda(thicken(legs), os.path.join(d, "straps_visual.usda"), "Straps"),
            cloth_rest_offset=0.003,
            cloth_contact_offset=0.01,
            deformable_props=PhysxDeformableBodyPropertiesCfg(solver_position_iteration_count=32, self_collision=True, self_collision_filter_distance=0.013),
            physics_material=PhysxSurfaceDeformableBodyMaterialCfg(density=300.0, youngs_modulus=5.0e8, poissons_ratio=0.3, surface_thickness=0.003,
                                                                   surface_bend_stiffness=1.0e4, bend_damping=2.0, elasticity_damping=0.05,
                                                                   static_friction=2.0, dynamic_friction=1.8),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.74, 0.66, 0.52), roughness=0.95),
            sew=sew,
        ),
        initial_pose=origin,
    )


# -- the score ---------------------------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotate(q, v):
    """Rotate v by the unit quaternion q (x, y, z, w)."""
    u, w = q[:3], q[3]
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _unrotate(q, v):
    return _rotate(np.array([-q[0], -q[1], -q[2], q[3]]), v)


def task_score(rake_off_shelf: bool, tote_on_shelf: bool, tote_on_table: bool, held: bool, inside: int) -> float:
    """The rake off the shelf: 0. The tote still on the shelf: 0. Off it, not resting on the table: 0.25. Resting on the
    table and not held: 0.5 and 0.125 per item inside."""
    if rake_off_shelf or tote_on_shelf:
        return 0.0
    if not tote_on_table or held:
        return 0.25
    return 0.5 + 0.125 * inside


class _Report:
    """Reads the scene once per control step and keeps the task's state; writes it to bag_scores/ every 0.5 s and when
    the score changes, because the rollout's log keeps only the final score."""

    def __init__(self, env):
        self.base = env.unwrapped
        self.step_count, self.last = 0, None
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        root = os.environ.get("BAG_SCORE_DIR", str(pathlib.Path(__file__).parent / "bag_scores"))
        os.makedirs(root, exist_ok=True)
        self.path = os.path.join(root, f"score_{stamp}.json")
        self.trace: list[dict] = []
        self.first_off = None

    def _pose(self, name):
        s = self.base.scene[name]
        o = self.base.scene.env_origins[0]
        p = (_t(s.data.root_pos_w)[0] - o).cpu().numpy().astype(float)
        q = _t(s.data.root_quat_w)[0].cpu().numpy().astype(float)
        return p, q

    def _held(self, tote_p, tote_q) -> dict:
        out = {}
        bars = {k: tote_p + _rotate(tote_q, v) for k, v in handle_axis_points(SPEC).items()}
        for side, name in ROBOTS.items():
            robot = self.base.scene[name]
            jid = robot.find_joints("joint7")[0]
            finger = abs(float(_t(robot.data.joint_pos)[0, jid].item() if len(jid) == 1 else _t(robot.data.joint_pos)[0, jid[0]].item()))
            bid = robot.find_bodies("gripper")[0][0]
            o = self.base.scene.env_origins[0]
            gp = (_t(robot.data.body_pos_w)[0, bid] - o).cpu().numpy().astype(float)
            gq = _t(robot.data.body_quat_w)[0, bid].cpu().numpy().astype(float)
            tcp = gp + _rotate(gq, np.array(TCP_IN_GRIPPER))
            near = min(float(np.linalg.norm(tcp - b)) for b in bars.values())
            out[side] = {"near_bar_m": round(near, 3), "finger_m": round(finger, 4), "holding": near < 0.07 and finger < 0.03}
        return out

    def _check_sink(self, rep: dict, rake_p) -> None:
        """Physics check, run with `BAG_CHECK=1`: the deepest the strap's nodes have gone into the deck and the table, and
        the rake's origin into the deck while it is on the deck, in mm so far."""
        o = self.base.scene.env_origins[0]
        n = (_t(self.base.scene["straps"].data.nodal_pos_w)[0] - o).cpu().numpy().reshape(-1, 3)
        over = (n[:, 0] > SHELF_X[0]) & (n[:, 0] < SHELF_X[1]) & (np.abs(n[:, 1]) < SHELF_Y[1]) & (n[:, 2] > SHELF_TOP - BOARD)
        now = {"strap_into_deck": float(np.max(SHELF_TOP - n[over, 2], initial=0.0)), "strap_into_table": float(TOP - n[:, 2].min()),
               "rake_into_deck": float(SHELF_TOP - rake_p[2]) if SHELF_X[0] < rake_p[0] < SHELF_X[1] and rake_p[2] > SHELF_TOP - BOARD else 0.0}
        self.sink = {k: max(getattr(self, "sink", {}).get(k, 0.0), v * 1000) for k, v in now.items()}
        rep["deepest_sink_mm"] = {k: round(v, 2) for k, v in self.sink.items()}

    def step(self) -> dict:
        self.step_count += 1
        tp, tq = self._pose("tote")
        half = np.array([SPEC.depth / 2, SPEC.width / 2])
        corners = np.array([tp + _rotate(tq, np.array([sx * half[0], sy * half[1], z])) for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, SPEC.height)])
        lo, hi = corners[:, 2].min(), corners[:, 2].max()
        in_x = corners[:, 0].max() > SHELF_X[0] and corners[:, 0].min() < SHELF_X[1]
        on_shelf = in_x and lo > SHELF_TOP - 0.03
        on_table = lo < TOP + 0.03 and hi < SHELF_TOP and corners[:, 0].max() < SHELF_X[1]
        inside = {}
        for it in ITEMS:
            p, _ = self._pose(it["name"])
            local = _unrotate(tq, p - tp)
            inside[it["name"]] = bool(abs(local[0]) <= half[0] + 0.005 and abs(local[1]) <= half[1] + 0.005 and -0.05 < local[2] < SPEC.height + 0.02)
        rake_p, rake_q = self._pose("rake")
        off = bool(rake_p[2] < SHELF_TOP - 0.05 or not (SHELF_X[0] - 0.02 < rake_p[0] < SHELF_X[1] + 0.02))
        held = self._held(tp, tq)
        any_held = any(h["holding"] for h in held.values())
        score = task_score(off, bool(on_shelf), bool(on_table), any_held, sum(inside.values()))
        rep = {"step": self.step_count, "score": score, "rake_off_shelf": off, "tote_on_shelf": bool(on_shelf), "tote_on_table": bool(on_table), "held": any_held,
               "items_inside": inside, "rake": [round(float(v), 3) for v in rake_p], "rake_up_z": round(float(1.0 - 2.0 * (rake_q[0] ** 2 + rake_q[1] ** 2)), 3),
               "tote": [round(float(v), 3) for v in tp], "tote_low_high": [round(float(lo), 3), round(float(hi), 3)], "grippers": held,
               "arch": [round(float(v), 3) for v in self._pose("back_arch")[0]]}
        if os.environ.get("BAG_CHECK"):
            self._check_sink(rep, rake_p)
        if os.environ.get("BAG_STEP_FILE"):  # for a watcher outside the run: the control step, written every step
            with open(os.environ["BAG_STEP_FILE"], "w") as f:
                f.write(str(self.step_count))
        if self.step_count % 10 == 0 or self.last is None or self.last["score"] != score:
            rep["wall_s"] = round(time.monotonic(), 2)
            self.trace.append(rep)
            with open(self.path, "w") as f:
                json.dump({"final": rep, "trace": self.trace}, f, indent=0)
        self.last = rep
        return rep


_report: _Report | None = None


def _patch_scene(scene) -> None:
    if getattr(scene, "_bagkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, BagPart) else orig_task_object(spec)

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
                out[n] = BagState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=rep)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._bagkit_patched = True


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


# -- the scripted demo's plan ---------------------------------------------------------------


def grip(closing, approach) -> np.ndarray:
    """A TCP rotation from the fingers' travel axis and the approach axis, the travel axis made perpendicular. In
    yam_arena's TCP frame the approach is +z and the fingers travel along x."""
    z = np.asarray(approach, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, dtype=float)
    x = x - x.dot(z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


@dataclass
class Key(Waypoint):
    """A `Waypoint` with its own grip. `seed`: a free move, solved from these joints and followed in a straight line in
    joint space (`exact`: these are the joints, unsolved, and `pos` only a record)."""

    rot: np.ndarray | None = None
    seed: tuple[float, ...] | None = None
    exact: bool = False


class Plan:
    """What `yam_scripted` plays for this task: `yam_arena.demo.ArmPlan` with `Key`s."""

    def __init__(self, waypoints, base_pos, q_start=READY, finger_start=FINGER_OPEN):
        from yam_arena import ik

        self.ik, self.base_pos = ik, np.asarray(base_pos, dtype=float)
        self.q = np.array(q_start, dtype=float)
        self.finger_start = finger_start
        self.waypoints = waypoints
        self.worst_ik_mm = self.worst_ik_deg = 0.0
        pos, rot = ik.tcp_pose(self.q, self.base_pos)
        self.segments, q = [], self.q.copy()
        for w in waypoints:
            goal = None
            if w.seed is not None:
                goal = np.array(w.seed, dtype=float)
                if not w.exact:
                    goal, _, _ = ik.solve(np.asarray(w.pos, dtype=float), ik.matrix_to_quat_xyzw(w.rot), goal, base_pos=self.base_pos, iters=300)
            self.segments.append(dict(key=w, q_goal=goal, q_from=None))

    @property
    def duration(self) -> float:
        return sum(w.duration for w in self.waypoints)

    def action7(self, t: float):
        ik = self.ik
        prev_pos, prev_rot = ik.tcp_pose(np.array(READY, dtype=float), self.base_pos)
        prev_finger = self.finger_start
        prev_q = np.array(READY, dtype=float)
        for seg in self.segments:
            w = seg["key"]
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                finger = prev_finger + s * (w.finger - prev_finger)
                if seg["q_goal"] is not None:
                    self.q = prev_q + s * (seg["q_goal"] - prev_q)
                else:
                    pos = prev_pos + s * (np.asarray(w.pos, dtype=float) - prev_pos)
                    rot = ik.slerp_matrix(prev_rot, w.rot, s)
                    self.q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base_pos, iters=60)
                    self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
                    self.worst_ik_deg = max(self.worst_ik_deg, float(np.degrees(re)))
                return np.r_[self.q, float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))], w.label
            t -= w.duration
            if seg["q_goal"] is not None:
                prev_q = seg["q_goal"]
                prev_pos, prev_rot = ik.tcp_pose(prev_q, self.base_pos)
            else:
                prev_pos, prev_rot = np.asarray(w.pos, dtype=float), w.rot
                prev_q = self.q.copy()
            prev_finger = w.finger
        return np.r_[self.q, float(np.clip(prev_finger / FINGER_OPEN, 0.0, 1.0))], "done"


def plan(keys: dict[str, list[Key]]) -> dict[str, list[Key]]:
    """Make the running `yam_scripted` policy play `keys` with `Plan`."""
    scripted = sys.modules.get("yam_arena.scripted")
    if scripted is not None:
        scripted.ArmPlan = Plan
    return keys


install()
