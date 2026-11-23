"""tasks/fit-bp-cuff/assets/arm.usdc: the visible training arm (skin-coloured mesh, no colliders: the task file's capsules
collide). A tube along x whose radius and centre height follow the capsules 1 mm proud: forearm 3.5 cm radius on its own
axis, a smooth elbow, upper arm 5.0 cm radius (31.4 cm round), a rounded shoulder end, and a flattened palm-up hand.
Run in the Isaac container (pxr):  python make_arm.py <out.usdc> <arm board top height over the table>  (positions are
relative to the table top; the task places the asset at the table top)."""
import math
import sys

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

out = sys.argv[1]
board, gap = 0.030, 0.004
R, RF = 0.050, 0.035
ZA, ZF = board + gap + R, board + gap + RF
X_WRIST, X_ELBOW, X_SHOULDER = 0.090, 0.300, 0.560


def smooth(a, b, t):
    t = min(max(t, 0.0), 1.0)
    t = t * t * (3 - 2 * t)
    return a + (b - a) * t


def section(x):
    """(radius, centre z) of the arm at x."""
    if x < X_WRIST:
        return 0.026, ZF - 0.009
    if x < X_ELBOW - 0.03:
        r = smooth(0.027, RF, (x - X_WRIST) / (X_ELBOW - 0.03 - X_WRIST))
        return r, ZF - (RF - r)  # the forearm rests on the board along its underside
    if x < X_ELBOW + 0.03:
        t = (x - (X_ELBOW - 0.03)) / 0.06
        return smooth(RF, R, t), smooth(ZF, ZA, t)
    if x < X_SHOULDER - 0.04:
        return R, ZA
    t = (x - (X_SHOULDER - 0.04)) / 0.06  # the shoulder end, rounded off
    return R * math.sqrt(max(1 - t * t, 0.02)), ZA


xs = np.concatenate([np.linspace(X_WRIST, X_SHOULDER + 0.02, 120)])
nseg = 40
pts, cols = [], []
for x in xs:
    r, zc = section(x)
    r += 0.001
    for k in range(nseg):
        a = 2 * math.pi * k / nseg
        pts.append(Gf.Vec3f(float(x), float(r * math.sin(a)), float(zc - r * math.cos(a))))
faces, counts = [], []
for i in range(len(xs) - 1):
    for k in range(nseg):
        a, b = i * nseg + k, i * nseg + (k + 1) % nseg
        faces += [a, b, b + nseg, a + nseg]
        counts.append(4)
# end caps
for ring, flip in ((0, True), (len(xs) - 1, False)):
    c = len(pts)
    r, zc = section(xs[ring])
    pts.append(Gf.Vec3f(float(xs[ring]), 0.0, float(zc)))
    for k in range(nseg):
        a, b = ring * nseg + k, ring * nseg + (k + 1) % nseg
        faces += [c, b, a] if flip else [c, a, b]
        counts.append(3)
# the hand: a flattened ellipsoid, palm up, fingers towards the robot
hx0, hz = X_WRIST - 0.055, board + 0.014
hand_start = len(pts)
nu, nv = 24, 16
for i in range(nv + 1):
    v = math.pi * i / nv
    for j in range(nu):
        u = 2 * math.pi * j / nu
        pts.append(Gf.Vec3f(float(hx0 + 0.062 * math.cos(v)), float(0.044 * math.sin(v) * math.cos(u)), float(hz + 0.014 * math.sin(v) * math.sin(u))))
for i in range(nv):
    for j in range(nu):
        a = hand_start + i * nu + j
        b = hand_start + i * nu + (j + 1) % nu
        faces += [a, b, b + nu, a + nu]
        counts.append(4)
stage = Usd.Stage.CreateNew(out)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/Asset")
stage.SetDefaultPrim(root.GetPrim())
mesh = UsdGeom.Mesh.Define(stage, "/Asset/arm")
mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
mesh.CreateFaceVertexCountsAttr(counts)
mesh.CreateFaceVertexIndicesAttr(faces)
mesh.CreateSubdivisionSchemeAttr("none")
skin = (0.86, 0.70, 0.60)
mesh.CreateDisplayColorAttr([Gf.Vec3f(*(c ** 2.2 for c in skin))])
rb = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())  # a kinematic body with no colliders: the task file's capsules collide
rb.CreateKinematicEnabledAttr(True)
UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(1.0)
stage.GetRootLayer().Save()
print("arm mesh", len(pts), "points", len(counts), "faces ->", out)
