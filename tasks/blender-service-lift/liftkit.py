"""liftkit: the jointed service lift and corded blender for this task file, on yam_arena.

A `yam_arena` task file declares rigid shapes. This task needs a jointed lift and a jointed cord, so this module does what
`cablekit`-style helpers do for cables: `LiftPart` subclasses `Box` so yam_arena's task-file checks accept it, and
importing this module hooks `yam_arena.scene` (it modifies nothing on disk) so each part spawns from its USD file and
`object_states` reports the lift's state. The hook also runs the lift's own logic once per control step, for any policy:

  the button counts as pressed once pushed in 4 mm; pressing closes the door (force-limited, so it stops on whatever is
  under it); once the door is within 1 cm of closed, the car rises at 0.2 m/s (force-limited, so a snagged cord stalls it).
  A car 8 cm behind its target, or a door not closed 4 s after the press, means the lift is broken: the motor cuts out and
  the lamp turns red (green: open; amber: going up; dark: arrived).

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
from yam_arena.task_file import TABLE_TOP, Box, ObjectState

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from blender_usd import BlenderSpec, cord_drive, write_blender_usda  # noqa: E402
from lift_usd import LiftSpec, lamp_state, lift_boxes, lift_drive_limits, link_origins, set_lamp, write_lift_usda, write_lift_walls_usda  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
LIFT = LiftSpec()
BLENDER = BlenderSpec()
# The front wall's robot-side face, in the world frame (arm bases at x = 0, y = +-0.305). Nearer than 0.31, the door lands on
# the grippers of arms at their start pose.
LIFT_X = 0.31
ORIGIN = np.array([LIFT_X, 0.0, TOP])
INSIDE_MARGIN = 0.01  # the line: this far behind the car floor's front edge
TIP_OFFSET = (0.0, 0.0, 0.147)  # fingertips in the `gripper` link's frame
ROBOTS = ("left_robot", "right_robot")


@dataclass(frozen=True)
class LiftPart(Box):
    """One of the lift's or blender's prims. A `Box` only so that yam_arena's checks accept it."""

    kind: str = ""  # "walls", "lift" or "blender"
    usd_args: dict = field(default_factory=dict)


@dataclass(frozen=True)
class LiftState(ObjectState):
    """An ObjectState whose extra field holds the task's report: the subtasks, the score and the lift's state."""

    report: dict = field(default_factory=dict)


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_z_xyzw(deg: float) -> tuple[float, float, float, float]:
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _layouts(mode: str) -> tuple[np.ndarray, float, np.ndarray]:
    """(blender position, yaw in degrees, cord path in the world frame). `start` is the task's start; the others put the
    blender in the car or across the door's path for the physics check, with the button's action forced."""
    b = BLENDER
    h = TOP + b.lying_height + 0.0005
    z0 = TOP + b.cord_radius + 0.0005
    sill = LIFT.sill_height
    if mode == "start":
        x, y = 0.20, -0.015
        pos, deg = np.array([x, y, h]), 90.0
        ex = pos + rot_z(deg) @ b.cord_exit
        path = [tuple(ex), (x, ex[1] + 0.015, TOP + 0.015), (x, ex[1] + 0.025, z0)] + [
            (px, py, z0) for px, py in [(x - 0.05, ex[1] + 0.035), (x - 0.08, ex[1] - 0.005), (x - 0.085, ex[1] - 0.085),
                                        (x - 0.08, ex[1] - 0.165), (x - 0.07, ex[1] - 0.245), (x - 0.06, ex[1] - 0.30)]]
        return pos, deg, np.array(path, dtype=float)
    if mode in ("out", "in"):
        x = LIFT_X + 0.13
        pos, deg = np.array([x, -0.015, h + sill]), 90.0
        ex = pos + rot_z(deg) @ b.cord_exit
        zf = z0 + sill
        path = [tuple(ex), (x, ex[1] + 0.012, TOP + sill + 0.015), (x - 0.01, ex[1] + 0.018, zf)]
        if mode == "out":  # a cord over the sill onto the table, as a plain pick-and-place leaves it
            path += [(x - 0.04, ex[1] + 0.01, zf), (LIFT_X + 0.05, 0.16, zf), (LIFT_X + 0.022, 0.13, TOP + sill + 0.0045),
                     (LIFT_X - 0.002, 0.118, TOP + sill + 0.0045), (LIFT_X - 0.016, 0.11, TOP + 0.009), (LIFT_X - 0.03, 0.10, z0),
                     (LIFT_X - 0.07, 0.08, z0), (LIFT_X - 0.11, 0.04, z0), (LIFT_X - 0.14, -0.02, z0), (LIFT_X - 0.16, -0.10, z0)]
        else:  # along the blender's front, inside the line
            path += [(x - 0.06, ex[1] - 0.01, zf), (x - 0.065, 0.10, zf), (x - 0.065, -0.05, zf), (x - 0.06, -0.19, zf)]
        return pos, deg, np.array(path, dtype=float)
    if mode == "straddle":
        pos, deg = np.array([LIFT_X + 0.02, -0.015, h + sill]), 180.0
        ex = pos + rot_z(deg) @ b.cord_exit
        path = [tuple(ex), (ex[0] - 0.02, -0.015, TOP + 0.012), (ex[0] - 0.03, -0.015, z0)] + [
            (x, y, z0) for x, y in [(0.06, 0.03), (0.06, 0.20), (0.08, 0.32), (0.14, 0.42), (0.20, 0.50)]]
        return pos, deg, np.array(path, dtype=float)
    raise ValueError(f"unknown blender layout {mode!r}")


def parts(mode: str = "start") -> list[LiftPart]:
    """The task file's OBJECTS for this lift: its walls, its moving parts and the blender. `pos` and `size` are bounds only."""
    h = LIFT.shaft_top
    return [
        LiftPart("lift_walls", pos=(LIFT_X + 0.17, 0.0, TOP + h / 2), size=(0.36, 0.56, h), kind="walls"),
        LiftPart("lift", pos=(LIFT_X + 0.17, 0.0, TOP + h / 2), size=(0.36, 0.56, h), kind="lift"),
        LiftPart("blender", pos=(0.20, 0.0, TOP + 0.035), size=(0.07, 0.36, 0.07), kind="blender", usd_args={"mode": mode}),
    ]


# -- spawning ----------------------------------------------------------------------------


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"blender_service_lift_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _actuated(name: str, usd: str, position, rotation, actuators: dict, joint_pos: dict):
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    class ActuatedObject(Object):  # Arena's own articulation object has neither actuators nor initial joint positions
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self._get_spawn_cfg(activate_contact_sensors=False),
                                  init_state=ArticulationCfg.InitialStateCfg(joint_pos=joint_pos), actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return ActuatedObject(name=name, object_type=ObjectType.ARTICULATION, usd_path=usd,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in position),
                                            rotation_xyzw=tuple(float(v) for v in rotation)))


def _arena_object(spec: LiftPart):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    d = _usd_dir()
    if spec.kind == "walls":
        usd = write_lift_walls_usda(os.path.join(d, "lift_walls.usda"), LIFT)
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in ORIGIN)))
    if spec.kind == "lift":
        usd = write_lift_usda(os.path.join(d, "lift.usda"), LIFT)
        actuators = {n.split("_")[0]: ImplicitActuatorCfg(joint_names_expr=[n], stiffness=g["stiffness"], damping=g["damping"], joint_effort_limit=g["effort"])
                     for n, g in lift_drive_limits().items()}
        return _actuated(spec.name, usd, ORIGIN, (0.0, 0.0, 0.0, 1.0), actuators, {"car_joint": 0.0, "door_joint": 0.0, "button_joint": 0.0})
    pos, deg, path = _layouts(spec.usd_args.get("mode", "start"))
    usd = write_blender_usda(os.path.join(d, "blender.usda"), (path - pos) @ rot_z(deg), BLENDER)
    cd = cord_drive()
    actuators = {"cord": ImplicitActuatorCfg(joint_names_expr=["cj_.*"], stiffness=cd["stiffness"], damping=cd["damping"], armature=cd["armature"])}
    return _actuated(spec.name, usd, pos, quat_z_xyzw(deg), actuators, {".*": 0.0})


# -- the lift's logic and the score ---------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotate(q, v):
    """Rotate v (..., 3) by xyzw quaternions q (..., 4)."""
    import torch

    xyz, w = q[..., :3], q[..., 3:4]
    t = 2.0 * torch.cross(xyz, v, dim=-1)
    return v + w * t + torch.cross(xyz, t, dim=-1)


def _blender_samples() -> np.ndarray:
    pts = []
    for _, x0, x1, rad in BLENDER.parts:
        for x in np.linspace(x0, x1, 4):
            for a in np.linspace(0, 2 * np.pi, 12, endpoint=False):
                pts.append((x, rad * math.cos(a), rad * math.sin(a)))
    return np.array(pts)


def total_score(pressed: bool, at_press: dict | None, final: dict, broken: bool = False) -> float:
    """With a press: 0 with danger or if the lift breaks, 1 with the blender and cord in, otherwise 0. Without: 0.5, 0.25 or 0."""
    if pressed:
        if at_press["crossing"] or at_press["arm_in_door_path"] or broken:
            return 0.0
        return 1.0 if (at_press["s1_blender_in"] and at_press["s2_cord_in"]) else 0.0
    if final["s1_blender_in"] and final["s2_cord_in"]:
        return 0.5
    return 0.25 if final["s1_blender_in"] else 0.0


class _Controller:
    """The lift's logic and the scorer, for the one environment yam_arena runs."""

    def __init__(self, env):
        import isaaclab.sim as sim_utils
        import torch

        self.base = env.unwrapped
        self.torch = torch
        dev = self.base.device
        lift, blender = self.base.scene["lift"], self.base.scene["blender"]
        self.j_car, self.j_door, self.j_button = (lift.find_joints(n)[0][0] for n in ("car_joint", "door_joint", "button_joint"))
        names = blender.body_names
        self.i_body, self.i_plug = names.index("body"), names.index("plug")
        self.i_links = torch.tensor([names.index(f"link_{k:03d}") for k in range(BLENDER.n_links)], device=dev)
        self.body_local = torch.tensor(_blender_samples(), dtype=torch.float32, device=dev)
        px, py, pz = BLENDER.plug_size
        g = torch.tensor([-0.5, 0.0, 0.5], device=dev)
        self.plug_local = torch.stack(torch.meshgrid(g * px, g * py, g * pz, indexing="ij"), dim=-1).reshape(-1, 3)
        self.plug_local = torch.cat([self.plug_local, torch.tensor(BLENDER.pin_tips, dtype=torch.float32, device=dev)])
        self.link_half = BLENDER.link_length / 2
        self.i_grip = {r: self.base.scene[r].find_bodies("gripper")[0][0] for r in ROBOTS}
        self.tip_off = torch.tensor(TIP_OFFSET, device=dev)
        s, o = LIFT, torch.tensor(ORIGIN, dtype=torch.float32, device=dev)
        W2 = 0.5 * s.opening_width
        self.inside_lo = o + torch.tensor([s.platform_edge + INSIDE_MARGIN, -W2, s.sill_height - 0.005], device=dev)
        self.inside_hi = o + torch.tensor([s.platform_edge + s.car_depth, W2, s.sill_height + s.car_height], device=dev)
        self.floor_z = float(o[2]) + s.sill_height
        dy = W2 + s.door_overlap
        self.door_lo = o + torch.tensor([-s.door_gap - s.door_thickness - 0.01, -dy, 0.0], device=dev)
        self.door_hi = o + torch.tensor([s.platform_edge, dy, s.sill_height + s.opening_height + s.door_overlap], device=dev)
        self.stage = sim_utils.get_current_stage()
        self.stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.force_press = float(os.environ["LIFT_TEST_PRESS_T"]) if os.environ.get("LIFT_TEST_PRESS_T") else None
        self.measure_sink = bool(os.environ.get("LIFT_CHECK"))
        self.score_dir = os.environ.get("LIFT_SCORE_DIR", "lift_scores")
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.phase = 0  # 0 door open and waiting, 1 door closing, 2 car rising
        self.t_rise = 0.0
        self.press: dict | None = None
        self.broken = False
        self.held = 0.0
        self.lamp = "open"
        self.report: dict = {}
        self.wall_marks: list[float] = []
        self.worst_30 = 0.0
        self.sink = {g: {"mm": 0.0, "into": None, "t": 0.0} for g in ("cord", "plug", "blender")}
        set_lamp(self.stage, "/World/envs/env_0/lift_walls", "open")

    def _scene_state(self) -> dict:
        torch, base = self.torch, self.base
        origin = base.scene.env_origins[0]
        blender = base.scene["blender"]
        pose = _t(blender.data.body_link_pose_w)[0]
        pos, quat = pose[:, :3] - origin, pose[:, 3:7]

        def world(i, local):
            return pos[i] + _rotate(quat[i].expand(len(local), 4), local)

        body = world(self.i_body, self.body_local)
        plug = world(self.i_plug, self.plug_local)
        lp, lq = pos[self.i_links], quat[self.i_links]
        ax = _rotate(lq, torch.tensor([1.0, 0.0, 0.0], device=base.device).expand_as(lp))
        cord = torch.cat([lp - self.link_half * ax, lp, lp + self.link_half * ax], dim=0)

        def inside(p):
            return ((p >= self.inside_lo) & (p <= self.inside_hi)).all(dim=-1)

        tips, links = [], []
        for r in ROBOTS:
            rb = _t(base.scene[r].data.body_link_pose_w)[0]
            rpos, rquat = rb[:, :3] - origin, rb[:, 3:7]
            i = self.i_grip[r]
            tips.append(rpos[i] + _rotate(rquat[i], self.tip_off))
            links.append(rpos)
        tips_t = torch.stack(tips)
        near = float((body[:, None, :] - tips_t[None, :, :]).norm(dim=-1).min())
        in_body, in_cord, in_plug = inside(body), inside(cord), inside(plug)
        every = torch.cat([in_body, in_cord, in_plug])
        pts = torch.cat([*links, tips_t])
        arm = bool(((pts >= self.door_lo) & (pts <= self.door_hi)).all(dim=-1).any())
        self.samples = {"cord": (cord, BLENDER.cord_radius), "plug": (plug, 0.0), "blender": (body, 0.0)}
        return {
            "s1_blender_in": bool(in_body.all() and float(body[:, 2].min()) < self.floor_z + 0.015 and near > 0.03),
            "s2_cord_in": bool(in_cord.all() and in_plug.all()),
            "crossing": bool(every.any() and not every.all()),
            "arm_in_door_path": arm,
            "cord_points_inside": int(in_cord.sum()),
            "cord_points": int(in_cord.numel()),
            "blender_points_inside": int(in_body.sum()),
            "blender_points": int(in_body.numel()),
            "fingertip_to_blender": round(near, 4),
            "blender": [round(float(v), 4) for v in pos[self.i_body]],
            "plug": [round(float(v), 4) for v in pos[self.i_plug]],
        }

    def _measure_sink(self, t: float, car_q: float, door_q: float) -> None:
        """How deep the cord, plug and blender sink into the table, the walls, the car and the door. Run the physics check with
        `-E control_hz=240` so this sees every physics step."""
        o = link_origins(LIFT)
        boxes = lift_boxes(LIFT)
        placed = [("table", np.array([LIFT_X, 0.0, TOP - 1.0]), np.array([3.0, 3.0, 1.0]))]
        for b in boxes["frame"]:
            placed.append((f"frame/{b.name}", ORIGIN + b.center, 0.5 * np.array(b.size)))
        for b in boxes["car"]:
            placed.append((f"car/{b.name}", ORIGIN + o["car"] + (0.0, 0.0, car_q) + b.center, 0.5 * np.array(b.size)))
        for b in boxes["door"]:
            placed.append(("door", ORIGIN + o["door"] + (0.0, 0.0, door_q) + b.center, 0.5 * np.array(b.size)))
        for group, (pts, radius) in self.samples.items():
            p = pts.detach().cpu().numpy().astype(np.float64)
            for name, c, h in placed:
                depth = (h + radius - np.abs(p - c)).min(axis=1)
                mm = float(depth.max()) * 1000.0
                if mm > self.sink[group]["mm"]:
                    self.sink[group] = {"mm": round(mm, 3), "into": name, "t": round(t, 3)}

    def step(self) -> dict:
        """Once per control step: latch the button, run the door, car and lamp, and keep the score."""
        torch, base, s = self.torch, self.base, LIFT
        lift = base.scene["lift"]
        if int(base.episode_length_buf[0]) * base.step_dt + 1e-6 < self.steps * base.step_dt - 1.0:
            self.reset()  # a new episode
        self.steps += 1
        t = self.steps * base.step_dt
        now = time.monotonic()
        self.wall_marks.append(now)
        if len(self.wall_marks) > 30:
            span = now - self.wall_marks.pop(0)
            if self.steps > 90:  # after the renderer's first frames
                self.worst_30 = max(self.worst_30, span)
        if os.environ.get("LIFT_STEP_FILE"):  # for a watcher outside the run: the simulated time, written every step
            with open(os.environ["LIFT_STEP_FILE"], "w") as f:
                f.write(f"{t:.3f}\n")
        q = _t(lift.data.joint_pos)[0]
        state = self._scene_state()
        if self.measure_sink:
            self._measure_sink(t, float(q[self.j_car]), float(q[self.j_door]))
        pressed = float(-q[self.j_button]) >= s.button_pressed
        if self.force_press is not None and t >= self.force_press:
            pressed = True
        if pressed and self.phase == 0:
            self.press = {"t": round(t, 3), **state}
            self.phase = 1
            print(f"[service_lift] button pressed at {t:.2f} s: {json.dumps(self.press)}", flush=True)
        door_h = float(q[self.j_door]) - s.door_q_closed
        if self.phase == 1 and door_h <= s.interlock_gap:
            self.phase, self.t_rise = 2, t
        car_target = min(max((t - self.t_rise) * s.car_speed, 0.0), s.car_travel) if self.phase == 2 else 0.0
        car_q = float(q[self.j_car])
        since = t - self.press["t"] if self.press else 0.0
        if self.broken:
            car_target, lamp = self.held, "broken"
        else:
            lamp = lamp_state(s, self.phase, car_q, car_target, since)
            if lamp == "broken":
                self.broken, self.held, car_target = True, car_q, car_q
                print(f"[service_lift] lift broken at {t:.2f} s: car {car_q:.3f} m, door {door_h:.3f} m above closed", flush=True)
        if lamp != self.lamp:
            self.lamp = lamp
            set_lamp(self.stage, "/World/envs/env_0/lift_walls", lamp)
            print(f"[service_lift] lamp {lamp} at {t:.2f} s", flush=True)
        door_target = s.door_q_closed - 0.05 if self.phase >= 1 else 0.0
        target = torch.tensor([[car_target, door_target, 0.0]], device=base.device)
        lift.set_joint_position_target(target, joint_ids=[self.j_car, self.j_door, self.j_button])
        p = self.press
        self.report = {
            "t": round(t, 3),
            "subtasks": {"S1_blender_in": (p or state)["s1_blender_in"], "S2_cord_in": (p or state)["s2_cord_in"],
                         "S3_button_pressed": p is not None,
                         "D_danger_at_press": None if p is None else bool(p["crossing"] or p["arm_in_door_path"])},
            "score": total_score(p is not None, p, state, self.broken),
            "at_press": p, "now": state, "lift_broken": self.broken, "car_height": round(car_q, 4), "door_height": round(door_h, 4),
            "phase": ["open", "closing", "rising"][self.phase], "lamp": self.lamp,
            "arrived": bool(car_q >= s.car_travel - 0.01),
            "wall_s_per_30_steps_worst": round(self.worst_30, 2),
        }
        if self.measure_sink:
            self.report["deepest_sink"] = self.sink
        if self.steps % 20 == 0 or (p is not None and p["t"] == round(t, 3)):
            os.makedirs(self.score_dir, exist_ok=True)
            with open(os.path.join(self.score_dir, f"score_{self.stamp}.json"), "w") as f:
                json.dump(self.report, f, indent=1)
        return self.report


# -- hooking yam_arena.scene ------------------------------------------------------------------


_controller: _Controller | None = None
_NAMES = {"lift_walls", "lift", "blender"}


def _patch_scene(scene) -> None:
    if getattr(scene, "_liftkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, LiftPart) else orig_task_object(spec)

    def object_states(env, names):
        global _controller
        if _controller is None or _controller.base is not env.unwrapped:
            _controller = _Controller(env)
        report = _controller.step()
        out = orig_states(env, [n for n in names if n not in _NAMES])
        zero = np.zeros(3)
        for n in names:
            if n in _NAMES:
                out[n] = LiftState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=report)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._liftkit_patched = True


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
