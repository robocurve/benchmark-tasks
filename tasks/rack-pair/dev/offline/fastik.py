"""Fast FK / geometric-Jacobian DLS IK for one YAM arm (same URDF chain as yam_arena.ik)."""
import sys, math
sys.path.insert(0, '/mnt/data/robocurve/yam-isaac-lab-arena')
import numpy as np
from yam_arena import YAM_URDF
from yam_arena.urdf import parse_joints, rpy_matrix
from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
from yam_arena import ik as yik

J = parse_joints(YAM_URDF)
CHAIN = [J[n] for n in ARM_JOINTS]
ORIG = []
for j in CHAIN:
    T = np.eye(4); T[:3, :3] = rpy_matrix(j.rpy); T[:3, 3] = j.xyz; ORIG.append(T)
AX = [j.axis / np.linalg.norm(j.axis) for j in CHAIN]
LO, HI = yik.LOWER, yik.UPPER
TCP = np.array(TCP_OFFSET)

def _rot(axis, a):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)

def fk(q, base):
    """Returns frames list (link1..gripper) as 4x4 in world (base translation only), joint axes/points, tcp pos, tcp rot."""
    T = np.eye(4); T[:3, 3] = base
    frames, axes, pts = [], [], []
    for i in range(6):
        T = T @ ORIG[i]
        axes.append(T[:3, :3] @ AX[i]); pts.append(T[:3, 3].copy())
        R = np.eye(4); R[:3, :3] = _rot(AX[i], q[i]); T = T @ R
        frames.append(T.copy())
    p = T[:3, :3] @ TCP + T[:3, 3]
    return frames, axes, pts, p, T[:3, :3]

def rot_err(Rt, R):
    return yik._rot_error(Rt, R)

def solve(p_t, R_t, base, q0, iters=300, damping=0.04, rot_w=0.3, tol=1e-5):
    q = np.array(q0, float)
    for _ in range(iters):
        frames, axes, pts, p, R = fk(q, base)
        e = np.concatenate([p_t - p, rot_w * rot_err(R_t, R)])
        if np.linalg.norm(e) < tol: break
        Jm = np.zeros((6, 6))
        for i in range(6):
            Jm[:3, i] = np.cross(axes[i], p - pts[i]); Jm[3:, i] = rot_w * axes[i]
        dq = Jm.T @ np.linalg.solve(Jm @ Jm.T + damping ** 2 * np.eye(6), e)
        q = np.clip(q + dq, LO, HI)
    frames, axes, pts, p, R = fk(q, base)
    return q, float(np.linalg.norm(p_t - p)), float(np.linalg.norm(rot_err(R_t, R)))

if __name__ == '__main__':
    # cross-check against yam_arena.ik.tcp_pose
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = rng.uniform(LO, HI)
        p1, R1 = yik.tcp_pose(q, (0, .305, .75)); _, _, _, p2, R2 = fk(q, np.array([0, .305, .75]))
        print(np.abs(p1 - p2).max(), np.abs(R1 - R2).max())
