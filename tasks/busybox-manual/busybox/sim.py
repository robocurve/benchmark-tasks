"""The busybox in Isaac Lab-Arena: its scene object, and its mechanics and state at run time.

Import after Isaac Sim has started.
"""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
import torch
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_arena.assets.object import Object
from isaaclab_arena.assets.object_type import ObjectType
from isaaclab_arena.utils.pose import Pose
from isaaclab_physx.sim.schemas import (
    PhysxArticulationRootPropertiesCfg,
    PhysxCollisionPropertiesCfg,
    PhysxRigidBodyPropertiesCfg,
)
from pxr import UsdGeom

from busybox import design as d
from busybox.state import BusyboxState, ButtonEvent
from busybox.usd import lamp_path

NAME = "busybox"
# Contacts start 2 mm out (the parts are small: buttons travel 4 mm) and rest at zero separation.
CONTACT_OFFSET = 0.002


def _actuator(joints: list[str], drive: dict) -> ImplicitActuatorCfg:
    return ImplicitActuatorCfg(joint_names_expr=joints, **drive)


class BusyboxObject(Object):
    """The busybox as an Arena articulation: a loose case with its moving parts, driven by weak joint drives."""

    def __init__(self, usd_path: str | Path, cfg: d.BusyboxConfig, pos: tuple[float, float, float]):
        self.busybox_cfg = cfg  # read by _generate_articulation_cfg, which Object.__init__ calls
        super().__init__(
            name=NAME,
            object_type=ObjectType.ARTICULATION,
            usd_path=str(usd_path),
            initial_pose=Pose(position_xyz=tuple(pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
        )

    def _generate_articulation_cfg(self) -> ArticulationCfg:
        assert self.object_type == ObjectType.ARTICULATION
        cfg = ArticulationCfg(
            prim_path=self.prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.usd_path,
                rigid_props=PhysxRigidBodyPropertiesCfg(max_depenetration_velocity=1.0),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=CONTACT_OFFSET, rest_offset=0.0),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    # The parts are housed in the case: they never collide with it (or each other).
                    enabled_self_collisions=False,
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos=self.busybox_cfg.initial_joint_pos(), joint_vel={".*": 0.0}
            ),
            actuators={
                "switches": _actuator(["switch_.*"], d.SWITCH_DRIVE),
                "sliders": _actuator(["slider_.*"], d.SLIDER_DRIVE),
                "dial": _actuator(["dial"], d.DIAL_DRIVE),
                "buttons": _actuator(["button_.*", "key_.*"], d.BUTTON_DRIVE),
                "big_button": _actuator(["big_button"], d.BIG_BUTTON_DRIVE),
            },
        )
        return self._add_initial_pose_to_cfg(cfg)


class BusyboxSim:
    """Runs one busybox (env 0): its detents, springs and lamps, and its :class:`BusyboxState`."""

    def __init__(self, env, cfg: d.BusyboxConfig):
        self.base = env.unwrapped
        self.cfg = cfg
        self.art = self.base.scene[NAME]
        self.device = self.base.device
        ids, names = self.art.find_joints(list(d.JOINTS), preserve_order=True)
        assert list(names) == list(d.JOINTS), f"busybox joints {names} != {d.JOINTS}"
        self.joint_ids = list(ids)
        self.state = BusyboxState(cfg)
        self._target_names = list(self.state.targets())
        self._target_ids = [self.joint_ids[d.JOINTS.index(n)] for n in self._target_names]
        root = f"{self.base.scene.env_prim_paths[0]}/{NAME}"
        stage = self.base.scene.stage
        self._lamps = [stage.GetPrimAtPath(f"{root}/{lamp_path(i, on=True)}") for i in range(d.NUM_SWITCHES)]
        missing = [str(p.GetPath()) for p in self._lamps if not p.IsValid()]
        if missing:
            raise RuntimeError(f"busybox lamps not found on the stage: {missing}")
        self._lit: list[bool | None] = [None] * d.NUM_SWITCHES

    def body_id(self, joint: str) -> int:
        """The rigid body of a moving part, by its joint's name (``"button_red"``, ``"dial"``, ...)."""
        return self.art.find_bodies(f"{joint}_link")[0][0]

    # -- simulator I/O ---------------------------------------------------------------------

    def joint_pos(self) -> dict[str, float]:
        q = self.art.data.joint_pos.torch[0, self.joint_ids].detach().cpu().tolist()
        return dict(zip(d.JOINTS, q, strict=True))

    def root_pos(self) -> torch.Tensor:
        """The case's origin (bottom centre) in the env frame."""
        return self.art.data.root_pose_w.torch[0, :3] - self.base.scene.env_origins[0]

    def _write_targets(self) -> None:
        targets = self.state.targets()
        value = torch.tensor([[targets[n] for n in self._target_names]], dtype=torch.float32, device=self.device)
        self.art.actuators.target_command.set_position_index(value=value, joint_ids=self._target_ids)

    def _update_lamps(self) -> None:
        for i, on in enumerate(self.state.switches):
            if on != self._lit[i]:
                img = UsdGeom.Imageable(self._lamps[i])
                if on:
                    img.MakeVisible()
                else:
                    img.MakeInvisible()
                self._lit[i] = on

    # -- episode -------------------------------------------------------------------------

    def reset(self, xy_offset: tuple[float, float] = (0.0, 0.0)) -> None:
        """Put the box back at its pose (shifted by ``xy_offset`` m) with every control as configured."""
        env_ids = torch.tensor([0], device=self.device)
        pose = self.art.data.default_root_pose.torch[:1].clone()
        pose[:, :3] += self.base.scene.env_origins[:1]
        pose[:, 0] += xy_offset[0]
        pose[:, 1] += xy_offset[1]
        self.art.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.art.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=self.device), env_ids=env_ids)
        q0 = self.cfg.initial_joint_pos()
        q = torch.tensor([[q0[n] for n in d.JOINTS]], dtype=torch.float32, device=self.device)
        self.art.write_joint_state_to_sim_index(
            position=q, velocity=torch.zeros_like(q), joint_ids=self.joint_ids, env_ids=env_ids
        )
        self.state.reset(q0)
        self._write_targets()
        self._update_lamps()

    def restart_clock(self) -> None:
        """Start the episode now: track from the current controls at time 0, with no events."""
        self.state.reset(self.joint_pos(), time=0.0)

    def after_step(self, time: float) -> list[ButtonEvent]:
        """Read the controls after a control step; set the next step's detent and spring targets and the lamps."""
        events = self.state.update(self.joint_pos(), time)
        self._write_targets()
        self._update_lamps()
        return events
