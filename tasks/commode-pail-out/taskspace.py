"""taskspace: Isaac Lab's built-in task-space controller on one YAM arm, for demokit's `Arm.press`.

The harness takes joint position targets, 6 a arm, which its implicit PD actuators track with gravity compensation (yam_arena/actions.py).
So of Isaac Lab's two task-space controllers only the differential IK fits without touching the harness's action term:
- `DifferentialIKController` in relative pose mode turns a small TCP move (position and axis-angle) into joint position targets. Used here.
- `OperationalSpaceController` outputs joint torques. The harness has no torque action, and changing its action term is changing the harness,
  which a task may not do. Not used.

A force at the TCP then comes from the arm's own joint springs (80 N m/rad on joints 1 to 3, 10 on 4 to 6, `yam_arena.constants`): with the
command `lead` metres ahead of the TCP along `d`, the springs push with about `lead / (d . C d)` newtons, where C is the TCP's compliance
`J K^-1 J^T` (position rows). `compliance()` gives C, `lead_for()` the lead for a force. The Jacobian is the URDF's (as `yam_arena.ik` uses),
in the world frame (the arm bases are translated, not turned). With no Isaac Lab (a dry run), the same damped least-squares step runs in NumPy.
"""

from __future__ import annotations

import numpy as np

EPS = 1e-6


def jacobian(q, base) -> np.ndarray:
    """The 6 x 6 geometric Jacobian of the TCP (linear rows, then angular), world frame, by finite differences of the URDF kinematics."""
    from yam_arena import ik

    q = np.asarray(q, float)
    p, R = ik.tcp_pose(q, base)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += EPS
        p2, R2 = ik.tcp_pose(dq, base)
        J[:3, i] = (p2 - p) / EPS
        J[3:, i] = ik._rot_error(R2, R) / EPS
    return J


def stiffness() -> np.ndarray:
    from yam_arena.constants import SHOULDER, WRIST

    return np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3, float)


def compliance(q, base) -> np.ndarray:
    """3 x 3: how far (m) the TCP gives under a force (N) at it, held by the joint springs alone."""
    J = jacobian(q, base)
    return (J @ np.diag(1.0 / stiffness()) @ J.T)[:3, :3]


def lead_for(q, base, direction, force: float) -> float:
    """How far ahead of the TCP, along `direction`, the command must stand for the springs to push with `force` newtons."""
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    return float(force * (d @ compliance(q, base) @ d))


class DiffIK:
    """`isaaclab.controllers.DifferentialIKController`, relative pose mode, damped least squares, one arm, on the CPU."""

    def __init__(self, damping: float = 0.05):
        self.damping = damping
        self.ctrl = None
        try:
            import torch
            from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
            from yam_arena import ik

            cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls", ik_params={"lambda_val": damping})
            self.ctrl = DifferentialIKController(cfg, num_envs=1, device="cpu")
            if hasattr(self.ctrl, "set_joint_pos_limits"):
                self.ctrl.set_joint_pos_limits(torch.tensor(ik.LOWER, dtype=torch.float32)[None], torch.tensor(ik.UPPER, dtype=torch.float32)[None])
            self.torch = torch
        except Exception:  # noqa: BLE001  no Isaac Lab here: the NumPy step below is the same method
            self.ctrl = None

    def step(self, q, base, dpos, drot=(0.0, 0.0, 0.0)) -> np.ndarray:
        """Joint targets that move the TCP by `dpos` (m) and turn it by the rotation vector `drot` (rad, world frame), from joints `q`."""
        from yam_arena import ik

        q = np.asarray(q, float)
        p, R = ik.tcp_pose(q, base)
        J = jacobian(q, base)
        cmd = np.r_[np.asarray(dpos, float), np.asarray(drot, float)]
        if self.ctrl is not None:
            t = self.torch
            ee_pos = t.tensor(p, dtype=t.float32)[None]
            ee_quat = t.tensor(ik.matrix_to_quat_xyzw(R), dtype=t.float32)[None]  # x, y, z, w: Isaac Lab 3.0's order
            self.ctrl.set_command(t.tensor(cmd, dtype=t.float32)[None], ee_pos, ee_quat)
            out = self.ctrl.compute(ee_pos, ee_quat, t.tensor(J, dtype=t.float32)[None], t.tensor(q, dtype=t.float32)[None])
            return np.clip(out[0].double().numpy(), ik.LOWER, ik.UPPER)
        dq = J.T @ np.linalg.solve(J @ J.T + self.damping ** 2 * np.eye(6), cmd)
        return np.clip(q + dq, ik.LOWER, ik.UPPER)
