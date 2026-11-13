"""The slot latches' over-centre spring, applied before every physics step (240 Hz).

A latch within SNAP (6) degrees of shut is pulled shut and held stiffly; otherwise it is pulled gently open, like the
socket's real spring: treading the latch's foot down with the module swings it past the centre and it snaps shut.
The latches are the board articulation's two revolute joints (0 rad = shut). Isaac Lab writes every articulation's
joint targets to PhysX before each physics substep (``write_data_to_sim``); the spring wraps that call on the board,
so it decides on the joint angles PhysX reported after the previous substep.
"""

from __future__ import annotations

import math

JOINTS = ("dimm1_neg_joint", "dimm1_pos_joint")
OPEN_SIGN = (1.0, -1.0)  # the open direction of each joint (assets/board.usda limits)
OPEN = math.radians(30.0)
SNAP = math.radians(6.0)  # keep equal to work/board/make_board.py LATCH_SNAP
PER_DEG = 180.0 / math.pi  # N m per degree -> N m per radian
STIFF_OPEN = 0.0001 * PER_DEG  # soft: treading a foot shut takes about 0.5 N
STIFF_SHUT = 0.005 * PER_DEG  # stiff: about 4-5 N at the arm's top to prise it open
DAMPING = 1e-6 * PER_DEG


class LatchSpring:
    def __init__(self, board):
        import torch

        self.board = board
        self.ids = board.find_joints(list(JOINTS), preserve_order=True)[0]
        self.sign = torch.tensor(OPEN_SIGN, device=board.device)
        self.shut = torch.zeros(len(JOINTS), dtype=torch.bool, device=board.device)
        board.write_joint_damping_to_sim_index(damping=torch.full((1, len(JOINTS)), DAMPING, device=board.device),
                                               joint_ids=self.ids)
        self._write(self.shut)
        write = board.write_data_to_sim

        def write_with_spring():
            self.update()
            write()

        board.write_data_to_sim = write_with_spring

    def angles(self):
        """Each latch's angle from shut (rad, signed as the joint)."""
        q = self.board.data.joint_pos
        return getattr(q, "torch", q)[0, self.ids]

    def update(self) -> None:
        now = self.angles().abs() < SNAP
        if bool((now != self.shut).any()):
            self._write(now)

    def reset(self) -> None:
        self.shut[:] = False
        self._write(self.shut)

    def _write(self, shut) -> None:
        import torch

        target = torch.where(shut, 0.0, self.sign * OPEN)[None]
        stiffness = torch.where(shut, STIFF_SHUT, STIFF_OPEN)[None]
        self.board.set_joint_position_target_index(target=target, joint_ids=self.ids)
        self.board.write_joint_stiffness_to_sim_index(stiffness=stiffness, joint_ids=self.ids)
        self.shut = shut.clone()
