"""The cotter pin's prongs bend and stay bent: elastic up to the prong's plastic moment, plastic beyond it.

The cotter (assets/cotter.usda, work/cotter/make_cotter.py) is a floating-base articulation: the eye and the shank are
its root body; the two prongs lie side by side, each on a revolute joint about z at its root (``prong_p_joint``: the
+y prong, outwards a positive angle; ``prong_n_joint``: the -y prong, outwards negative); they are spread apart round
the nut. Each joint is driven back to a set angle by a
stiff spring, K = MY / 2 degrees, so a prong springs back about 2 degrees after being bent. When the spring's torque
would pass the plastic moment MY, the set angle follows the prong (it yields), keeping the torque at MY: a prong bent
past yield stays where it was pushed, less the 2 degree springback.

MY: plastic moment of a half-round prong 0.75 mm thick, 1.5 mm wide (a 1/16 in pin's wire, ASME B18.8.1) at a yield
strength of 300 MPa (low-carbon steel 1005-1010): about 0.045 N m, i.e. about 5.6 N pushed 8 mm from the root.

Isaac Lab writes every articulation's joint targets to PhysX before each physics substep (``write_data_to_sim``);
this wraps that call on the cotter, so it decides on the joint angles PhysX reported after the previous substep.
"""

from __future__ import annotations

import math

JOINTS = ("prong_p_joint", "prong_n_joint")
SIGN = {"prong_p_joint": 1.0, "prong_n_joint": -1.0}  # outwards: the +y prong a positive angle about z, the -y negative
MY = 0.045  # N m

# the scripted demo's live aim (task.py): the screwdriver's waypoints for each pass at the prongs, with their aim
# relative to where the pin should sit, which task.py's on_step hook moves to where it does sit (shared through this
# module, which the policy's and the environment's copies of the task file both import)
LIVE: dict = {}
SPRINGBACK = math.radians(2.0)
K = MY / SPRINGBACK  # N m / rad
DAMPING = K * 0.002
LIMIT = math.radians(98.0)  # the joints' own limits are 100 degrees outwards (the prong lies against the nut's flat)
INWARD = math.radians(2.0)  # a prong can't bend in past the other: its set angle stays within INWARD inwards (a
# stiff spring wall), whatever pushes it; only outwards does it yield


def outward(joint: str, angle: float) -> float:
    """A prong's bend outwards (rad) from its joint angle."""
    return SIGN[joint] * angle


class Plastic:
    def __init__(self, cotter):
        import torch

        self.cotter = cotter
        self.dev = cotter.device
        self.ids = dict(zip(JOINTS, cotter.find_joints(list(JOINTS), preserve_order=True)[0]))
        ids = [self.ids[j] for j in JOINTS]
        cotter.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[K, K]], device=self.dev), joint_ids=ids)
        cotter.write_joint_damping_to_sim_index(damping=torch.tensor([[DAMPING, DAMPING]], device=self.dev), joint_ids=ids)
        self.reset()
        write = cotter.write_data_to_sim

        def write_with_plastic():
            self.update()
            write()

        cotter.write_data_to_sim = write_with_plastic

    def q(self) -> dict:
        q = self.cotter.data.joint_pos
        q = getattr(q, "torch", q)[0]
        return {j: float(q[i]) for j, i in self.ids.items()}

    def _write(self) -> None:
        import torch

        self.cotter.set_joint_position_target_index(target=torch.tensor([[self.set[j] for j in JOINTS]], device=self.dev),
                                                    joint_ids=[self.ids[j] for j in JOINTS])

    def update(self) -> None:
        moved = False
        for j, a in self.q().items():
            stretch = a - self.set[j]
            if abs(K * stretch) > MY:  # yields: the set angle follows, keeping the spring at MY
                s = clamp(j, a - math.copysign(MY / K, stretch))
                if s != self.set[j]:
                    self.set[j] = s
                    moved = True
            self.most[j] = max(self.most[j], outward(j, a))
        if moved:
            self._write()

    def reset(self) -> None:
        self.set = {j: 0.0 for j in JOINTS}
        self.most = {j: 0.0 for j in JOINTS}
        self._write()


def clamp(joint: str, set_angle: float) -> float:
    """A prong's set angle kept between INWARD inwards and LIMIT outwards."""
    out = min(max(outward(joint, set_angle), -INWARD), LIMIT)
    return SIGN[joint] * out


def plastic_step(set_angle: float, angle: float, joint: str = "prong_p_joint") -> float:
    """The set angle after one update, for a prong now at ``angle`` (rad): the rule above, without Isaac Sim (for
    self_test)."""
    stretch = angle - set_angle
    if abs(K * stretch) > MY:
        return clamp(joint, angle - math.copysign(MY / K, stretch))
    return set_angle
