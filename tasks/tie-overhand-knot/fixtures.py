"""Declared passive fixtures for the development knotting jig.

These are static contact shapes. They never attach to the rope or move in
response to the policy. The rounded free ends allow a loop to slide off.
"""
from dataclasses import dataclass
import math

from yam_arena.task_file import Box


@dataclass(frozen=True)
class Peg(Box):
    radius: float = .008
    cylinder_length: float = .14

    def _check(self):
        super()._check()
        if self.free or self.radius <= 0 or self.cylinder_length <= 0:
            raise ValueError('A jig peg must be a positive-sized static capsule')


def arena_peg(spec):
    import isaaclab.sim as sim_utils
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb

    spec._check()
    return Object(
        name=spec.name, object_type=ObjectType.BASE,
        spawner_cfg=sim_utils.CapsuleCfg(
            radius=spec.radius, height=spec.cylinder_length,
            collision_props=PhysxCollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=.5, dynamic_friction=.5, restitution=0.),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=linear_rgb(spec.color), roughness=.45)),
        initial_pose=Pose(position_xyz=spec.pos,
                          rotation_xyzw=(-math.sqrt(.5), 0., 0., math.sqrt(.5))))


def three_peg_jig():
    from config import CONFIG
    from jig_layout import geometry
    g = CONFIG.get('jig_geometry') or geometry()
    y0, y1 = g['peg_y_ends']
    radius = g['peg_radius']
    length = y1-y0
    pegs = [Peg(name, pos=(x, (y0+y1)/2, z),
                size=(2*radius, length+2*radius, 2*radius),
                radius=radius, cylinder_length=length,
                color=(.62, .62, .64)) for name, x, z in g['pegs']]
    # Two separate uprights leave the central threading corridor open from
    # both sides. A solid backplate would obstruct the opposing gripper.
    supports = [Box(name, pos=pos, size=size, color=(.32, .34, .36))
                for name, pos, size in g['supports']]
    return [*supports, *pegs]
