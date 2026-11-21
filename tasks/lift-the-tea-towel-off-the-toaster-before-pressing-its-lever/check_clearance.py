"""Offline: how close the arm's collision meshes come to the toaster body (and the lever's neighbours) along the demo.
Plays the demo's IK plan, poses every link's URDF collision meshes, and reports the nearest toaster-body point per
waypoint. The towel and lever are meant to be touched and are left out. Needs tools/toaster_body_surface_60k.npy
(60,000 points sampled on the SimReady body mesh). Run: python check_clearance.py"""
import re
import sys
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from yam_arena import YAM_URDF, station, task_file
from yam_arena.constants import ARM_JOINTS
from yam_arena.demo import ArmPlan
from yam_arena.urdf import forward_kinematics, rpy_matrix

T = task_file.load("task.py").module
HERE = __import__("pathlib").Path(__file__).parent
urdf = open(YAM_URDF).read()
root = __import__("pathlib").Path(YAM_URDF).parent
meshes = {}
for link in re.finditer(r'<link name="([^"]+)">(.*?)</link>', urdf, re.S):
    pts = []
    for c in re.finditer(r"<collision>.*?<origin xyz=\"([^\"]+)\" rpy=\"([^\"]+)\".*?filename=\"([^\"]+)\"", link.group(2), re.S):
        v = trimesh.load(root / c.group(3)).vertices
        M = np.eye(4); M[:3, :3] = rpy_matrix(np.array(c.group(2).split(), float)); M[:3, 3] = np.array(c.group(1).split(), float)
        pts.append((np.c_[v, np.ones(len(v))] @ M.T)[:, :3])
    if pts:
        meshes[link.group(1)] = np.vstack(pts)

body = np.load(HERE / "tools" / "toaster_body_surface_60k.npy").astype(float)
yaw = np.pi                                         # the toaster turned 180 deg
R = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
world = body @ R.T + np.array([*T.TOASTER_XY, T.TOP])
g = np.random.default_rng(0).uniform(-0.5, 0.5, (4000, 3))             # the two bread slices: points on their boxes
g[np.arange(4000), np.arange(4000) % 3] = np.sign(g[np.arange(4000), np.arange(4000) % 3]) * 0.5
bread = np.vstack([g * T.BREAD_SIZE + [T.TOASTER_XY[0], T.TOASTER_XY[1] + s, T.TOP + T.BREAD_TOP - T.BREAD_SIZE[2] / 2]
                   for s in (-T.BREAD_Y, T.BREAD_Y)])
world = np.vstack([world, bread])
tree = cKDTree(world)
CARRIED = {"lift it off the slots", "carry it off to the left, turning it", "over the table, clear of the toaster",
           "lower it to the table"}
tl, tw, th = T.TOWEL_SIZE
u = np.random.default_rng(1).uniform(-0.5, 0.5, (3000, 3)) * [tl, tw, th]   # points through the towel's box
for arm, wps in T.demo().items():
    plan = ArmPlan(wps, station.BASES[arm])
    base = np.asarray(station.BASES[arm])
    t, worst = 0.0, {}
    starts = np.cumsum([0] + [w.duration for w in wps])
    while t < plan.duration:
        q7, label = plan.action7(t)
        finger = float(q7[6]) * -0.04695                 # normalized grip -> finger joint (m)
        poses = forward_kinematics(YAM_URDF, {**dict(zip(ARM_JOINTS, q7[:6])), "joint7": finger, "joint8": finger})
        best = (np.inf, "")
        for link, pts in meshes.items():
            w = (np.c_[pts, np.ones(len(pts))] @ poses[link].T)[:, :3] + base
            d, _ = tree.query(w)
            if d.min() < best[0]:
                best = (float(d.min()), link)
        if label in CARRIED:                              # the towel held in the grip: long side across the fingers
            pos, rot, _, _ = plan.target(t)
            close = rot[:, 0] * [1, 1, 0]                 # the fingers close along the gripper's x axis
            close /= np.linalg.norm(close)
            M = np.array([[close[1], close[0], 0], [-close[0], close[1], 0], [0, 0, 1]])   # columns: long, across, up
            pts = u @ M.T + pos + [0, 0, -T.TIP_PAST - 0.0025 + th / 2]
            d = float(tree.query(pts)[0].min())
            if d < best[0]:
                best = (d, "towel")
        if label not in worst or best[0] < worst[label][0]:
            worst[label] = best
        t += 0.05
    print(arm)
    for w in wps:
        if w.label in worst:
            print(f"  {w.label:32s} nearest toaster/bread {worst[w.label][0] * 1000:6.1f} mm ({worst[w.label][1]})")
