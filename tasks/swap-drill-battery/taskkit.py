"""taskkit: helpers shared by task.py's monitor and scorer (pure Python at import; Isaac imports are lazy).

- :func:`write_limits` / :func:`write_damping`: change an articulation joint at run time (latches, locks).
- :func:`fingertips`: both grippers' finger link positions, for keep-out zones.
- :class:`Still`: "at rest for N seconds", robust to PhysX putting settled bodies to sleep.
- :func:`tilt`: angle between a body's z axis and world up.
"""

from __future__ import annotations

import math

import numpy as np

CONTROL_HZ = 20

# measured (Isaac Lab-Arena 0.3.1): each arm's bodies are base, link1..link5, gripper, tip_left, tip_right. The
# tip_* link ORIGINS sit at the finger roots, about 7.7 cm short of the pads' tips, so fingertips() carries them out
# along the approach axis to the real tip depth.
FINGER_LINK_KEYS = ("tip_",)
TCP_OFFSET = 0.134        # yam_arena.constants.TCP_OFFSET: tool point on the gripper link's +z (approach) axis
FINGER_PAD_REACH = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP: the pads' tips lie this far past the tool point


def write_limits(environment, asset_name: str, joint: str, lo: float, hi: float) -> None:
    """Set one joint's position limits (rad or m) in the running simulation."""
    import torch

    asset = environment.unwrapped.scene[asset_name]
    ids = asset.find_joints([joint])[0]
    asset.write_joint_position_limit_to_sim_index(
        limits=torch.tensor([[[lo, hi]]], dtype=torch.float32, device=asset.device), joint_ids=ids,
        warn_limit_violation=False)


def write_damping(environment, asset_name: str, joint: str, damping: float) -> None:
    import torch

    asset = environment.unwrapped.scene[asset_name]
    ids = asset.find_joints([joint])[0]
    asset.write_joint_damping_to_sim_index(
        damping=torch.tensor([[damping]], dtype=torch.float32, device=asset.device), joint_ids=ids)


def apply_force(environment, asset_name: str, force_xyz, body: str | None = None) -> None:
    """A constant external force (N, world frame) on a rigid body (or one link of an articulation) until changed.
    Isaac Lab-Arena 0.3.1 has set_external_force_and_torque (it persists and is applied on every physics step)."""
    import torch

    asset = environment.unwrapped.scene[asset_name]
    if body is None:
        ids = [0]
    else:
        ids = asset.find_bodies([body])[0]
    f = torch.tensor([[list(force_xyz)] * len(ids)], dtype=torch.float32, device=asset.device)
    asset.set_external_force_and_torque(forces=f, torques=torch.zeros_like(f), body_ids=ids)


def body_poses(environment, asset_name: str) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Every link of an articulation (or the one body of a rigid object): name -> (pos, quat_xyzw), station frame.
    Isaac Lab stores quaternions as (w, x, y, z) in ``body_quat_w`` (checked: matches the root pose); this converts."""
    base = environment.unwrapped
    asset = base.scene[asset_name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    pos = asset.data.body_pos_w[0].detach().cpu().numpy() - origin
    quat_wxyz = asset.data.body_quat_w[0].detach().cpu().numpy()
    return {n: (pos[i], np.r_[quat_wxyz[i, 1:], quat_wxyz[i, 0]]) for i, n in enumerate(asset.body_names)}


def _hand(environment, side: str):
    """(tool point, approach axis, closing axis, pad half-separation) of one arm, from yam_arena's forward kinematics
    on the measured arm joints and the measured finger opening."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    base = environment.unwrapped
    robot = base.scene[f"{side}_robot"]
    q_all = robot.data.joint_pos[0].detach().cpu().numpy()
    ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
    tcp, rot = ik.tcp_pose(q_all[ids], station.BASES[side])
    approach = rot[:, 2]
    # the pads close along the tool frame's x axis (measured: with grip_rotation(0), whose x axis is world -y, the
    # open fingers straddled a block 55 mm across in y and landed on one 40 mm across in x). The tip_* links' origin
    # difference points along the tool frame's y instead: their origins are offset, not on the pads.
    close = rot[:, 0]
    f = robot.find_joints([FINGER_JOINTS[0]])[0]
    half = min(abs(float(q_all[f][0])), 0.047)                            # 0 closed .. 0.047 m open (pads 94 mm apart)
    return tcp, approach, close, half


def fingertips(environment) -> list[np.ndarray]:
    """The two pad tips of each gripper (station frame), left arm first: the tool point carried 13 mm along the
    approach axis, then half the measured finger opening either way along the closing axis. (The tip_* link origins
    sit on the finger slides, several cm from the pads: measured.)"""
    pts = []
    for side in ("left", "right"):
        tcp, a, c, half = _hand(environment, side)
        tip = tcp + FINGER_PAD_REACH * a
        pts += [tip + half * c, tip - half * c]
    return pts


def hand_points(environment, back: float = 0.10, step: float = 0.02) -> list[np.ndarray]:
    """Points covering both grippers' fingers (station frame): each pad tip and points every ``step`` back along the
    approach axis to ``back`` behind it (the finger's length, up to the gripper body). For "any part of the gripper"
    keep-out rules; pad a zone by about half a finger's thickness (6 mm)."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    tips = fingertips(environment)
    base = environment.unwrapped
    pts = []
    for k, side in enumerate(("left", "right")):
        robot = base.scene[f"{side}_robot"]
        ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
        q = robot.data.joint_pos[0, ids].detach().cpu().numpy()
        approach = ik.tcp_pose(q, station.BASES[side])[1][:, 2]
        for tip in tips[2 * k:2 * k + 2]:
            pts += [tip - d * approach for d in np.arange(0.0, back + 1e-9, step)]
    return pts


def in_box(p, lo, hi, margin: float = 0.0) -> bool:
    p, lo, hi = (np.asarray(v, float) for v in (p, lo, hi))
    return bool(np.all(p >= lo - margin) and np.all(p <= hi + margin))


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt(quat_xyzw) -> float:
    """Radians between the body's z axis and world up."""
    return math.acos(max(-1.0, min(1.0, rotation(quat_xyzw)[2, 2])))


class Still:
    """Counts how long a body has been at rest (under 1 cm/s and 0.5 rad/s, or not moved since the previous control
    step: PhysX can put a settled body to sleep while it still reports a small velocity)."""

    def __init__(self):
        self.prev: dict = {}
        self.held: dict = {}

    def reset(self) -> None:
        self.prev.clear()
        self.held.clear()

    def update(self, name: str, pos, quat_xyzw, lin_vel, ang_vel) -> float:
        now = np.r_[np.asarray(pos, float), np.asarray(quat_xyzw, float)]
        last = self.prev.get(name)
        self.prev[name] = now
        slow = np.linalg.norm(lin_vel) < 0.01 and np.linalg.norm(ang_vel) < 0.5
        unmoved = last is not None and np.linalg.norm(now[:3] - last[:3]) < 0.0005 and np.abs(now[3:] - last[3:]).max() < 0.01
        self.held[name] = self.held.get(name, 0.0) + 1.0 / CONTROL_HZ if (slow or unmoved) else 0.0
        return self.held[name]
