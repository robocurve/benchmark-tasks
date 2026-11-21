"""Does a USD fixed joint with physics:breakTorque break where it should in the official sim's PhysX?

Each case is a cantilever: block A (kinematic) and block B (dynamic) joined at their touching faces by a
UsdPhysics.FixedJoint with breakTorque T. Gravity on B loads the joint with a known torque, a fraction of T. A case
breaks if B drops more than 5 mm. Unbroken cases report the largest gap that opened between the joint's two anchors,
which shows how much light pieces drift or jitter. The last case is a 4 x 3 grid of 3.6 g pieces lying on a static
plate, every neighbour pair joined, to check that a bar at rest doesn't come apart on its own.

    python check_break_joint.py --device cuda:0 --out result.json

Runs with the official simulator's venv (Isaac Sim 6.1, Isaac Lab 17), headless, 240 Hz physics as yam_arena uses.
"""

import argparse
import json

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--device", default="cuda:0")
parser.add_argument("--out", required=True)
parser.add_argument("--seconds", type=float, default=5.0)
args = parser.parse_args()
app = AppLauncher(headless=True).app

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import warp as wp  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

G = 9.81
T = 0.12  # N m: the planning estimate of one groove's bending strength
DT = 1.0 / 240.0  # yam_arena's physics step (scene.py: 240 Hz PhysX)
BAR = (0.036, 0.057, 0.007)  # one row of the bar: length along x (cantilever), width y, thickness z (m)

sim = SimulationContext(SimulationCfg(dt=DT, device=args.device))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")


def block(path, size, pos, mass=None, kinematic=False, com=None, static=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if static:
        return prim
    body = UsdPhysics.RigidBodyAPI.Apply(prim)
    body.CreateKinematicEnabledAttr(kinematic)
    if mass is not None:
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        if com is not None:  # in the cube's local (scaled) frame: divide by the scale
            m.CreateCenterOfMassAttr(Gf.Vec3f(*(c / s for c, s in zip(com, size))))
    return prim


def joint(path, a, b, local_a, local_b, size_a, size_b, break_torque):
    j = UsdPhysics.FixedJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([a.GetPath()])
    j.CreateBody1Rel().SetTargets([b.GetPath()])
    # Local positions are in each body's frame, which carries the cube's scale: divide by it.
    j.CreateLocalPos0Attr(Gf.Vec3f(*(p / s for p, s in zip(local_a, size_a))))
    j.CreateLocalPos1Attr(Gf.Vec3f(*(p / s for p, s in zip(local_b, size_b))))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateBreakTorqueAttr(break_torque)
    j.CreateBreakForceAttr(1.0e6)
    return j


def mass_for(frac, lever):
    return frac * T / (G * lever)


cases = []  # (name, expected torque / T, B path, anchor of A in world, B's local anchor, B's start position)
y = 0.0
specs = [
    ("flat 0.5 T", 0.5, BAR, None),
    ("flat 0.9 T", 0.9, BAR, None),
    ("flat 1.1 T", 1.1, BAR, None),
    ("flat 2.0 T", 2.0, BAR, None),
    ("on edge 0.9 T (in-plane bending)", 0.9, (BAR[0], BAR[2], BAR[1]), None),
    ("on edge 1.1 T (in-plane bending)", 1.1, (BAR[0], BAR[2], BAR[1]), None),
    ("twist 1.1 T", 1.1, (0.004, 0.057, 0.007), (0.0, 0.02, 0.0)),
    ("one real row (3 pieces, 10.8 g), own weight", None, BAR, None),
]
for name, frac, size, com in specs:
    y += 0.3
    root = f"/World/c{len(cases)}"
    UsdGeom.Xform.Define(stage, root)
    ax = (0.0, y, 1.0)
    a = block(f"{root}/A", size, ax, kinematic=True)
    bx = (ax[0] + size[0], y, 1.0)
    lever_x = size[0] / 2
    if frac is None:
        mass, frac = 0.0108, 0.0108 * G * lever_x / T
    elif com is None:
        mass = mass_for(frac, lever_x)
    else:
        mass = frac * T / (G * float(np.hypot(lever_x, com[1])))
    b = block(f"{root}/B", size, bx, mass=mass, com=com)
    anchor_a = (size[0] / 2, 0.0, 0.0)
    anchor_b = (-size[0] / 2, 0.0, 0.0)
    joint(f"{root}/joint", a, b, anchor_a, anchor_b, size, size, T)
    cases.append(dict(name=name, torque_over_T=round(frac, 3), mass_kg=round(mass, 4), b=f"{root}/B",
                      anchor_world=(ax[0] + anchor_a[0], ax[1], ax[2]), b_local=anchor_b, b_start=bx))

# The bar at rest: 12 pieces of 3.6 g in 4 rows (along x) of 3 (along y) on a static plate, every neighbour joined.
piece = (BAR[0], BAR[1] / 3, BAR[2])
plate_z = 0.5
block("/World/plate", (0.3, 0.3, 0.02), (0.0, -0.5, plate_z), static=True)
grid = {}
for i in range(4):
    for k in range(3):
        pos = ((i - 1.5) * piece[0], -0.5 + (k - 1) * piece[1], plate_z + 0.01 + piece[2] / 2)
        grid[(i, k)] = (block(f"/World/bar/p{i}{k}", piece, pos, mass=0.0036), pos)
n_grid_joints = 0
for (i, k), (p, pos) in grid.items():
    for di, dk in ((1, 0), (0, 1)):
        if (i + di, k + dk) in grid:
            q, _ = grid[(i + di, k + dk)]
            off = (di * piece[0] / 2, dk * piece[1] / 2, 0.0)
            joint(f"/World/bar/j{i}{k}_{i + di}{k + dk}", p, q, off, tuple(-o for o in off), piece, piece, T)
            n_grid_joints += 1

sim.reset()
view = sim.physics_manager.get_physics_sim_view()
bview = view.create_rigid_body_view("/World/c*/B")
gview = view.create_rigid_body_view("/World/bar/p*")


def poses(v):
    return wp.to_torch(v.get_transforms()).cpu().numpy()  # (N, 7): x y z, qx qy qz qw


def rotate(q, v):
    x, y_, z, w = q
    u = np.array([x, y_, z])
    v = np.asarray(v, dtype=float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


steps = int(args.seconds / DT)
start_b = poses(bview)
start_g = poses(gview)
max_gap = np.zeros(len(cases))
drop = np.zeros(len(cases))
max_grid_move = 0.0
for _ in range(steps):
    sim.step(render=False)
    P = poses(bview)
    for c, case in enumerate(cases):
        anchor_b = P[c, :3] + rotate(P[c, 3:], case["b_local"])
        max_gap[c] = max(max_gap[c], float(np.linalg.norm(anchor_b - np.array(case["anchor_world"]))))
        drop[c] = max(drop[c], float(start_b[c, 2] - P[c, 2]))
    Gp = poses(gview)
    max_grid_move = max(max_grid_move, float(np.max(np.linalg.norm(Gp[:, :3] - start_g[:, :3], axis=1))))

out = dict(device=args.device, dt=DT, seconds=args.seconds, break_torque=T, cases=[], grid={})
for c, case in enumerate(cases):
    broke = drop[c] > 0.005
    out["cases"].append(dict(name=case["name"], torque_over_T=case["torque_over_T"], mass_kg=case["mass_kg"],
                             broke=bool(broke), drop_mm=round(drop[c] * 1000, 2),
                             max_anchor_gap_mm=round(max_gap[c] * 1000, 3)))
Gp = poses(gview)
gaps = []
names = sorted(grid)
for (i, k) in names:
    for di, dk in ((1, 0), (0, 1)):
        if (i + di, k + dk) in grid:
            a, b = names.index((i, k)), names.index((i + di, k + dk))
            off = np.array((di * piece[0] / 2, dk * piece[1] / 2, 0.0))
            pa = Gp[a, :3] + rotate(Gp[a, 3:], off)
            pb = Gp[b, :3] + rotate(Gp[b, 3:], -off)
            gaps.append(float(np.linalg.norm(pa - pb)))
out["grid"] = dict(pieces=len(grid), joints=n_grid_joints, max_piece_move_mm=round(max_grid_move * 1000, 3),
                   max_joint_gap_mm=round(max(gaps) * 1000, 3), joints_open_over_1mm=int(sum(g > 0.001 for g in gaps)))
with open(args.out, "w") as f:
    json.dump(out, f, indent=1)
print(json.dumps(out, indent=1))
app.close()
