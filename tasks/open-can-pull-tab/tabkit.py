"""The ring-pull tab's resistance: the score line holds the tab shut until it is lifted hard enough, then the opening
is torn and the tab swings up freely.

The can (assets/can.usda) is a floating-base articulation: the can body is the root, the tab sits on a revolute joint
about x at its rivet (``tab_rivet``, 0 shut to 90 degrees up). Real stay-on tabs take about 6 lb (27 N) at the ring to
start the score line tearing (US patent literature on stay-on tabs; a lighter pull of 10-15 N once the score is
started), the ring's tip being about 20 mm from the rivet. So the joint is held by a stiff spring that yields at
BREAK (0.10 N m: 5 N at the ring's tip) while the tab is shut, and, once it has been lifted past TORN (8 degrees),
at TEAR (0.03 N m: 1.5 N at the tip, the torn flap being pushed down into the can).

Isaac Lab writes every articulation's joint targets to PhysX before each physics substep (``write_data_to_sim``);
this wraps that call on the can, so it decides on the joint angle PhysX reported after the previous substep.
"""

from __future__ import annotations

import math
import os

JOINT = "tab_rivet"
BREAK = float(os.environ.get("TAB_BREAK", 0.10))  # N m to start tearing the score line (the tab shut)
TEAR = 0.03  # N m once it is torn
TORN = math.radians(8.0)  # the tab's angle beyond which the score line counts as torn
SPRINGBACK = math.radians(1.0)
K = BREAK / SPRINGBACK  # N m / rad: the stiff spring
DAMPING = K * 0.002
LIMIT = math.radians(90.0)


def limit_for(most: float) -> float:
    return TEAR if most >= TORN else BREAK


def plastic_step(set_angle: float, angle: float, most: float = 0.0) -> float:
    """The set angle after one update for a tab now at ``angle`` (rad), having been lifted at most ``most`` (self_test)."""
    m = limit_for(most)
    stretch = angle - set_angle
    if abs(K * stretch) > m:
        return min(max(angle - math.copysign(m / K, stretch), 0.0), LIMIT)
    return set_angle


class Tab:
    def __init__(self, can):
        import torch

        self.can = can
        self.dev = can.device
        self.id = can.find_joints([JOINT], preserve_order=True)[0][0]
        can.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[K]], device=self.dev), joint_ids=[self.id])
        can.write_joint_damping_to_sim_index(damping=torch.tensor([[DAMPING]], device=self.dev), joint_ids=[self.id])
        self.reset()
        write = can.write_data_to_sim

        def write_with_tab():
            self.update()
            write()

        can.write_data_to_sim = write_with_tab

    def angle(self) -> float:
        q = self.can.data.joint_pos
        q = getattr(q, "torch", q)[0]
        return float(q[self.id])

    def _write(self) -> None:
        import torch

        self.can.set_joint_position_target_index(target=torch.tensor([[self.set]], device=self.dev), joint_ids=[self.id])

    def update(self) -> None:
        a = self.angle()
        s = plastic_step(self.set, a, self.most)
        self.most = max(self.most, a)
        if s != self.set:
            self.set = s
            self._write()

    def reset(self) -> None:
        self.set = 0.0
        self.most = 0.0
        self._write()
