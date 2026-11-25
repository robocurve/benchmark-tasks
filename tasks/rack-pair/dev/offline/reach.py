"""Offline reach, clearance and load checks for the four-arm server racking layout. NumPy only."""
import sys, math, itertools, xml.etree.ElementTree as ET
sys.path.insert(0, '/mnt/data/robocurve/yam-isaac-lab-arena')
import numpy as np
from yam_arena import YAM_URDF, ik
from yam_arena.urdf import forward_kinematics
from yam_arena.constants import ARM_JOINTS, READY, SHOULDER, WRIST
from yam_arena import station
import geometry as G

def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])

class Layout:
    def __init__(self, X, d, y_mid=0.0):
        self.X, self.d, self.y_mid = X, d, y_mid
        self.y_f = y_mid + d                      # rack front posts' front face
        self.origin = {'a': np.array([-X, y_mid, 0.]), 'b': np.array([X, y_mid, 0.])}
        self.R = {'a': rz(0.), 'b': rz(math.pi)}
    def base_world(self, agent, side):
        return self.origin[agent] + self.R[agent] @ np.array(station.BASES[side])
    def arm_for(self, agent, role):
        # role 'rear' = arm nearer the rack (+y), 'front' = arm at -y
        if agent == 'a':
            return 'left' if role == 'rear' else 'right'
        return 'right' if role == 'rear' else 'left'

def grip_rot(agent, approach_yaw_deg):
    """TCP rotation: approach horizontal toward the server's centre line, turned by yaw about z
    (positive yaw turns the approach toward +y, i.e. toward the rack); fingers close vertically."""
    inward = np.array([1., 0, 0]) if agent == 'a' else np.array([-1., 0, 0])
    sgn = 1 if agent == 'a' else -1
    a = rz(math.radians(sgn * approach_yaw_deg)) @ inward
    z_t = a / np.linalg.norm(a)
    y_t = np.array([0, 0, 1.])                    # finger axis vertical
    x_t = np.cross(y_t, z_t)
    return np.column_stack([x_t, y_t, z_t])

def push_rot(agent):
    """Approach +y (toward the rack) for pushing the front face; fingers closed, finger axis vertical."""
    z_t = np.array([0, 1., 0]); y_t = np.array([0, 0, 1.]); x_t = np.cross(y_t, z_t)
    return np.column_stack([x_t, y_t, z_t])

def solve_world(L, agent, side, p_w, R_w, q_init=None):
    Rl = L.R[agent].T
    p_l = Rl @ (np.asarray(p_w) - L.origin[agent])
    R_l = Rl @ R_w
    best = None
    seeds = [q_init] if q_init is not None else []
    seeds += [READY, [0.5, 1.2, 1.0, -0.3, 0, 0], [-0.5, 1.2, 1.0, -0.3, 0, 0], [0.0, 1.6, 1.6, 0.0, 0, 0],
              [0.8, 1.4, 1.2, 0.0, 0.5, 0], [-0.8, 1.4, 1.2, 0.0, -0.5, 0]]
    for s in seeds:
        q, pe, re = ik.solve(p_l, ik.matrix_to_quat_xyzw(R_l), q_init=s, base_pos=station.BASES[side], iters=400)
        score = pe + 0.05 * re
        if best is None or score < best[3]:
            best = (q, pe, re, score)
    return best[0], best[1], best[2]

def chain_points(L, agent, side, q):
    fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, q)})
    base = np.array(station.BASES[side])
    pts = []
    for link in ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper'):
        p_l = fk[link][:3, 3] + base
        pts.append(L.origin[agent] + L.R[agent] @ p_l)
    p_tcp, _ = ik.tcp_pose(q, base)
    pts.append(L.origin[agent] + L.R[agent] @ p_tcp)
    return np.array(pts)

def seg_box_dist(p0, p1, lo, hi, n=40):
    best = 1e9
    for t in np.linspace(0, 1, n):
        p = p0 + t * (p1 - p0)
        c = np.clip(p, lo, hi)
        best = min(best, np.linalg.norm(p - c))
    return best

def post_boxes(L):
    zlo, zhi = G.TABLE_TOP, G.TABLE_TOP + G.RACK_HEIGHT
    out = []
    for sx in (-1, 1):
        x0, x1 = sorted((sx * G.POST_X_IN, sx * G.POST_X_OUT))
        out.append((np.array([x0, L.y_f, zlo]), np.array([x1, L.y_f + G.POST_DEPTH, zhi])))
    return out

def clearance(L, agent, side, q, skip_last=2):
    pts = chain_points(L, agent, side, q)
    dmin = 1e9
    for lo, hi in post_boxes(L):
        for i in range(len(pts) - 1):
            dmin = min(dmin, seg_box_dist(pts[i], pts[i + 1], lo, hi))
    return dmin

def server_center(L, e=None, y_c=None):
    if e is not None:
        return L.y_f - G.D + e + G.D / 2      # e = engagement: rear face past the front posts' face
    return y_c

def grasp_point(L, agent, role, y_c, z_b):
    sx = -1 if agent == 'a' else 1
    x = sx * (G.W / 2 - G.EDGE_INSET)
    front_face = y_c - G.D / 2; rear_face = y_c + G.D / 2
    y = front_face + G.FRONT_GRASP_FROM_FRONT if role == 'front' else rear_face - G.REAR_GRASP_FROM_REAR
    return np.array([x, y, z_b + G.H / 2])

def load_torques(L, agent, side, q, load_kg):
    """Static joint torques (N m) from gravity on the arm's own links plus a payload at the TCP."""
    tree = ET.parse(YAM_URDF).getroot()
    inert = {}
    for link in tree.findall('link'):
        i = link.find('inertial')
        if i is None: continue
        m = float(i.find('mass').get('value'))
        o = i.find('origin'); xyz = np.array([float(v) for v in o.get('xyz').split()]) if o is not None else np.zeros(3)
        inert[link.get('name')] = (m, xyz)
    base = np.array(station.BASES[side])
    def potential(qq):
        fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, qq)})
        U = 0.
        for name, (m, c) in inert.items():
            if name == 'base' or name not in fk: continue
            p = fk[name][:3, :3] @ c + fk[name][:3, 3]
            U += m * 9.81 * p[2]
        ptcp, _ = ik.tcp_pose(qq, (0, 0, 0))
        U += load_kg * 9.81 * ptcp[2]
        return U
    tau = []
    for j in range(6):
        dq = np.zeros(6); dq[j] = 1e-5
        tau.append((potential(q + dq) - potential(q - dq)) / 2e-5)
    return np.array(tau)
