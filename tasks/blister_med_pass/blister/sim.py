"""The blister cards in Isaac Lab-Arena: their scene objects, the foil's tearing and the episode reset.

Each filled cell's foil is a flap on a spring drive (target 0, flat) holding the tablet; above the tablet, the bubble's
face sits on a stiff spring. Once the bubble has been pressed ``DOME_TEAR`` down (``TEAR_FORCE``), the foil is torn for
the rest of the episode and the drive swings the flap open, so the tablet falls out below.

Import after Isaac Sim has started.
"""

from __future__ import annotations

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

from blister import design as d
from blister.usd import build_card

CONTACT_OFFSET = 0.002  # thin parts: contacts start 2 mm out and rest at zero separation
# Pinched by its edges, the card is held against turning by friction over the pads' contact patches. PhysX models that
# torsional friction only with a patch radius, which otherwise shrinks with penetration depth (here near zero), so it
# gets a fixed minimum: a rubber fingertip pad on a cardboard edge, about 5 mm.
TORSIONAL_PATCH = 0.005  # m


class CardObject(Object):
    """A blister card as a floating-base Arena articulation; its flaps are held by their spring drives."""

    def __init__(self, card: d.Card, pos, yaw_xyzw=(0.0, 0.0, 0.0, 1.0)):
        super().__init__(name=card.name, object_type=ObjectType.ARTICULATION, usd_path=build_card(card),
                         initial_pose=Pose(position_xyz=tuple(pos), rotation_xyzw=tuple(yaw_xyzw)))  # fmt: skip

    def _generate_articulation_cfg(self) -> ArticulationCfg:
        cfg = ArticulationCfg(
            prim_path=self.prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.usd_path,
                rigid_props=PhysxRigidBodyPropertiesCfg(max_depenetration_velocity=0.5),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=CONTACT_OFFSET, rest_offset=0.0,
                                                            torsional_patch_radius=TORSIONAL_PATCH,
                                                            min_torsional_patch_radius=TORSIONAL_PATCH),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,  # the flaps sit in the card's openings
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos={".*": 0.0}, joint_vel={".*": 0.0}),
            actuators={
                "foil": ImplicitActuatorCfg(joint_names_expr=["flap_.*"], **d.FLAP_DRIVE),
                "bubbles": ImplicitActuatorCfg(joint_names_expr=["dome_.*"], **d.DOME_DRIVE),
            },
        )
        return self._add_initial_pose_to_cfg(cfg)


class Cards:
    """Runs every card (env 0): tears foil that has been pushed through, and resets cards and tablets."""

    def __init__(self, env):
        self.base = env.unwrapped
        self.device = self.base.device
        self.arts, self.cells, self.joint_ids, self.dome_ids = {}, {}, {}, {}
        for card in d.CARDS:
            art = self.base.scene[card.name]
            cells = list(card.full_cells())
            flaps, _ = art.find_joints([d.flap_joint(c) for c in cells], preserve_order=True)
            domes, _ = art.find_joints([d.dome_joint(c) for c in cells], preserve_order=True)
            self.arts[card.name], self.cells[card.name] = art, cells
            self.joint_ids[card.name], self.dome_ids[card.name] = list(flaps), list(domes)
        self.torn: dict[str, set[int]] = {c.name: set() for c in d.CARDS}

    def _write_targets(self, name: str) -> None:
        targets = [[-d.OPEN_ANGLE if c in self.torn[name] else 0.0 for c in self.cells[name]]]
        value = torch.tensor(targets, dtype=torch.float32, device=self.device)
        self.arts[name].actuators.target_command.set_position_index(value=value, joint_ids=self.joint_ids[name])

    def after_step(self) -> list[tuple[str, int]]:
        """Tear the foil under any bubble pressed with ``TEAR_FORCE``; returns the newly torn (card, cell)s."""
        new = []
        for name, art in self.arts.items():
            q = art.data.joint_pos.torch[0].detach().cpu()
            domes = q[self.dome_ids[name]].tolist()
            for cell, press in zip(self.cells[name], domes, strict=True):
                if press < -d.DOME_TEAR and cell not in self.torn[name]:
                    self.torn[name].add(cell)
                    new.append((name, cell))
            if any(n == name for n, _ in new):
                self._write_targets(name)
        return new

    def reset(self) -> None:
        """Every card at its starting pose with all foil intact, and every tablet back in its cell."""
        env_ids = torch.tensor([0], device=self.device)
        origin = self.base.scene.env_origins[:1]
        zero6 = torch.zeros(1, 6, device=self.device)
        for card in d.CARDS:
            art = self.arts[card.name]
            pose = art.data.default_root_pose.torch[:1].clone()
            pose[:, :3] += origin
            art.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
            art.write_root_velocity_to_sim_index(root_velocity=zero6, env_ids=env_ids)
            q = torch.zeros(1, art.num_joints, device=self.device)  # flaps flat, domes up
            art.write_joint_state_to_sim_index(position=q, velocity=q.clone(), env_ids=env_ids)
            self.torn[card.name].clear()
            self._write_targets(card.name)
            for cell in card.full_cells():
                pill = self.base.scene[card.pill_name(cell)]
                pose = pill.data.default_root_pose.torch[:1].clone()
                pose[:, :3] += origin
                pill.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
                pill.write_root_velocity_to_sim_index(root_velocity=zero6, env_ids=env_ids)
