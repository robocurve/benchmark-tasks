"""Writes assets/housing.usda (the light's bare steel housing with its four keyholes, a rigid body), assets/driver.usda
(a push-to-start cordless screwdriver, a rigid body) and assets/screws.usda (the four screws driven into the ceiling:
one articulation fixed to the ceiling, each screw on its own chain z (prismatic) and spin (revolute) from its pilot
hole). Needs pxr only (not the
simulator): python build_assets.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g
from geometry import subtract

OUT = Path(__file__).resolve().parent / "assets"
OFF = (0.0005, 0.0)

WHITE = (0.93, 0.93, 0.91)
STEEL_IN = (0.82, 0.83, 0.82)
LED_BOARD = (0.90, 0.90, 0.86)
LED = (0.98, 0.97, 0.85)
TRIM = (0.06, 0.06, 0.06)
CHANNEL = (0.75, 0.76, 0.75)
BOSCH = (0.02, 0.30, 0.52)
RUBBER = (0.10, 0.11, 0.12)
BIT = (0.55, 0.56, 0.58)
ZINC = (0.80, 0.80, 0.76)


# -- USD helpers -------------------------------------------------------------------------------------------------------
def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def offsets(prim, contact=0.0005, rest=0.0):
    prim = prim.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def box(stage, path, lo, hi, mat=None, collide=True, off=OFF):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    return c


def cylinder(stage, path, radius, z0, z1, mat=None, collide=False, off=OFF, xy=(0.0, 0.0)):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(z1 - z0))
    c.CreateAxisAttr("Z")
    c.AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), float((z0 + z1) / 2)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    return c


def capsule(stage, path, radius, z0, z1, mat=None, collide=False, off=OFF):
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(z1 - z0 - 2 * radius))
    c.CreateAxisAttr("Z")
    c.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float((z0 + z1) / 2)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    return c


def rigid(stage, path, mass, com, inertia, pos=(0.0, 0.0, 0.0)):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(float(mass))
    m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    return xf


def rigid_root(root, mass, com, inertia):
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(float(mass))
    m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))


# -- the housing ---------------------------------------------------------------------------------------------------------
def build_housing():
    """Origin: the middle of the back's top face (flush with the ceiling when mounted). x along the length (toward B),
    y across, z up; the open side faces down."""
    stage, root = stage_for("housing")
    r = "/housing"
    white = material(stage, f"{r}/Looks/white", WHITE, roughness=0.35, metallic=0.2)
    inner = material(stage, f"{r}/Looks/inner", STEEL_IN, roughness=0.4, metallic=0.3)
    board = material(stage, f"{r}/Looks/board", LED_BOARD, roughness=0.6)
    led = material(stage, f"{r}/Looks/led", LED, roughness=0.2)
    trim = material(stage, f"{r}/Looks/trim", TRIM, roughness=0.8)
    chan = material(stage, f"{r}/Looks/channel", CHANNEL, roughness=0.4, metallic=0.4)
    dark = material(stage, f"{r}/Looks/dark", (0.08, 0.08, 0.08), roughness=0.8)
    L, W, D, T, WT = g.HOUSING_L, g.HOUSING_W, g.HOUSING_D, g.SHEET_T, g.WALL_T
    m = g.HOUSING_MASS
    inertia = (m * (W * W + D * D) / 12, m * (L * L + D * D) / 12, m * (L * L + W * W) / 12)
    rigid_root(root, m, (0.0, 0.0, -0.018), inertia)
    b = r
    # the back plate, with its four keyholes cut out of the collider (and so seen through from below)
    rects = [(-L / 2, L / 2, -W / 2, W / 2)]
    for kx, ky in g.KEYS:
        for cut in g.keyhole_cuts(kx, ky):
            rects = subtract(rects, cut)
    for i, (x0, x1, y0, y1) in enumerate(rects):
        box(stage, f"{b}/back_{i}", (x0, y0, -T), (x1, y1, 0.0), white)
    # long walls (left at +y, right at -y, as seen from A) and end walls (A's at -x, B's at +x)
    box(stage, f"{b}/wall_pos", (-L / 2, W / 2 - WT, -D), (L / 2, W / 2, -T), white)
    box(stage, f"{b}/wall_neg", (-L / 2, -W / 2, -D), (L / 2, -W / 2 + WT, -T), white)
    box(stage, f"{b}/end_pos", (L / 2 - WT, -W / 2 + WT, -D), (L / 2, W / 2 - WT, -T), white)
    box(stage, f"{b}/end_neg", (-L / 2, -W / 2 + WT, -D), (-L / 2 + WT, W / 2 - WT, -T), white)
    # inside (visual): the inner face of the back around the keyholes, four LED boards, the driver channel in the middle
    rects_in = [(-L / 2 + WT, L / 2 - WT, -W / 2 + WT, W / 2 - WT)]
    for kx, ky in g.KEYS:
        for cut in g.keyhole_cuts(kx, ky):
            rects_in = subtract(rects_in, (cut[0] - 0.0015, cut[1] + 0.0015, cut[2] - 0.0015, cut[3] + 0.0015))
    for i, (x0, x1, y0, y1) in enumerate(rects_in):
        box(stage, f"{b}/inside_{i}", (x0, y0, -T - 0.0004), (x1, y1, -T - 0.0001), inner, collide=False)
    for i, y in enumerate((-0.130, -0.045, 0.045, 0.130)):
        box(stage, f"{b}/board_{i}", (-0.30, y - 0.012, -T - 0.004), (0.30, y + 0.012, -T - 0.0005), board, collide=False)
        for k in range(16):
            x = -0.28 + k * (0.56 / 15)
            box(stage, f"{b}/led_{i}_{k}", (x - 0.0025, y - 0.0025, -T - 0.0055), (x + 0.0025, y + 0.0025, -T - 0.004),
                led, collide=False)
    box(stage, f"{b}/channel", (-0.33, -0.020, -T - 0.030), (0.33, 0.020, -T - 0.0005), chan, collide=False)
    # round knockouts on the centre line (visual, the maker's callout 1)
    for i, x in enumerate((-0.48, 0.0, 0.48)):
        cylinder(stage, f"{b}/knockout_{i}", 0.011, -T - 0.0006, -T - 0.0002, dark, xy=(x, 0.0))
    # push-on rubber edge trim (U-channel, 8 mm across, 24 mm tall, 80 mm long) on the long walls' lower edges at the
    # four grip spots, 80 mm in from each end
    rubber = UsdShade.Material.Define(stage, f"{r}/Looks/rubber_phys")
    pm = UsdPhysics.MaterialAPI.Apply(rubber.GetPrim())
    pm.CreateStaticFrictionAttr(1.1)
    pm.CreateDynamicFrictionAttr(1.0)
    pm.CreateRestitutionAttr(0.0)
    for sx, xe in ((1.0, "pos"), (-1.0, "neg")):
        xc = sx * (L / 2 - g.GRIP_IN)
        for sy, ye in ((1.0, "pos"), (-1.0, "neg")):
            yw = sy * (W / 2 - WT / 2)
            c = box(stage, f"{b}/trim_{xe}_{ye}", (xc - g.TRIM_L / 2, yw - g.TRIM_T / 2, -D - 0.002),
                    (xc + g.TRIM_L / 2, yw + g.TRIM_T / 2, -D + g.TRIM_H - 0.002), trim)
            UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(rubber, UsdShade.Tokens.weakerThanDescendants, "physics")
    stage.GetRootLayer().Save()


# -- the screwdriver -----------------------------------------------------------------------------------------------------
def build_driver():
    """Origin: the middle of its bottom end; axis +z toward the bit."""
    stage, root = stage_for("driver")
    r = "/driver"
    blue = material(stage, f"{r}/Looks/blue", BOSCH, roughness=0.45)
    rubber = material(stage, f"{r}/Looks/rubber", RUBBER, roughness=0.9)
    steel = material(stage, f"{r}/Looks/steel", BIT, roughness=0.25, metallic=0.9)
    m, R, Lb = g.DRV_MASS, g.DRV_R, g.DRV_BODY_L
    inertia = (m * (3 * R * R + Lb * Lb) / 12, m * (3 * R * R + Lb * Lb) / 12, m * R * R / 2)
    rigid_root(root, m, (0.0, 0.0, 0.075), inertia)
    b = r
    cylinder(stage, f"{b}/grip", R, 0.0, Lb, rubber, collide=True)
    cylinder(stage, f"{b}/shell_top", R + 0.0008, 0.060, Lb, blue)
    cylinder(stage, f"{b}/end_cap", R + 0.0008, 0.0, 0.006, blue)
    box(stage, f"{b}/switch", (-0.006, R - 0.0005, 0.095), (0.006, R + 0.003, 0.125), rubber, collide=False)
    cylinder(stage, f"{b}/collar", g.DRV_COLLAR_R, Lb, Lb + g.DRV_COLLAR_L, steel, collide=True)
    h = g.DRV_BIT_HALF
    box(stage, f"{b}/bit", (-h, -h, Lb + g.DRV_COLLAR_L), (h, h, g.DRV_TIP), steel, collide=False)   # contact: task code
    stage.GetRootLayer().Save()


# -- the four screws -----------------------------------------------------------------------------------------------------
def build_screws():
    """One articulation, fixed to the world at its origin (place it at (0, 0, 0)). For each screw: base -> z (prismatic)
    -> spin (revolute), the frames at its pilot hole at the ceiling when the joints are at 0. The screw's frame origin is
    its tip; it points up (+z), its head below. The z joint is the thread (task code drives it from the spin)."""
    stage, root = stage_for("screws")
    r = "/screws"
    zinc = material(stage, f"{r}/Looks/zinc", ZINC, roughness=0.3, metallic=0.9)
    dark = material(stage, f"{r}/Looks/dark", (0.15, 0.15, 0.15), roughness=0.5)
    base = rigid(stage, r + "/base", 0.05, (0.0, 0.0, 0.0), (1e-5, 1e-5, 1e-5)).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    root.GetPrim().AddAppliedSchema("PhysxArticulationAPI")
    root.GetPrim().CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    root.GetPrim().CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    root.GetPrim().CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    root.GetPrim().CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    fixed = UsdPhysics.FixedJoint.Define(stage, r + "/fix")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])
    for s, (hx, hy) in g.SCREWS.items():
        hole = (hx, hy, g.CEIL)
        parent = base.GetPath()
        for name, kind in (("pz", "prismatic"), ("spin", "revolute")):
            last = name == "spin"
            path = f"{r}/{s}_screw" if last else f"{r}/{s}_link_pz"
            mass = g.SCREW_MASS if last else 0.01
            inertia = (2e-7, 2e-7, 5e-8) if last else (1e-6, 1e-6, 1e-6)
            link = rigid(stage, path, mass, (0.0, 0.0, -0.009 if last else 0.0), inertia, pos=hole)
            jt = (UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint).Define(
                stage, f"{r}/{s}_{name}")
            jt.CreateAxisAttr("Z")
            jt.CreateBody0Rel().SetTargets([parent])
            jt.CreateBody1Rel().SetTargets([link.GetPath()])
            pos0 = hole if parent == base.GetPath() else (0.0, 0.0, 0.0)
            jt.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
            jt.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            jt.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            jt.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            if kind == "prismatic":
                jt.CreateLowerLimitAttr(g.PZ0 - 0.002)
                jt.CreateUpperLimitAttr(g.TRAVEL + 0.004)
            drive = UsdPhysics.DriveAPI.Apply(jt.GetPrim(), "angular" if kind == "revolute" else "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(0.0)
            parent = link.GetPath()
        # The colliders ride on the z link, which does not turn: a turning head's contact friction on the light is a
        # couple, with no net force, but PhysX's contact patch on a fast-spinning collider pushes the light sideways.
        # The head's collider is a block as wide as the head across the slot (8 mm) and only as long as the shank is
        # thick along it (4.2 mm), so it never lies over the slot's closed end: clamped there, PhysX's contact pushes
        # the light along the slot. It holds the back on the slot's two edges as the round head does.
        zl = f"{r}/{s}_link_pz"
        capsule(stage, f"{zl}/shank_collider", g.SHANK_R, -g.SCREW_L, 0.0, None, collide=True)
        box(stage, f"{zl}/head_collider", (-g.SHANK_R, -g.HEAD_R, -g.SCREW_L - g.HEAD_H), (g.SHANK_R, g.HEAD_R, -g.SCREW_L),
            None, collide=True)
        for p in (f"{zl}/shank_collider", f"{zl}/head_collider"):
            UsdGeom.Imageable(stage.GetPrimAtPath(p)).MakeInvisible()
        sc = f"{r}/{s}_screw"
        capsule(stage, f"{sc}/shank", g.SHANK_R - 0.0001, -g.SCREW_L, 0.0, zinc, collide=False)
        cylinder(stage, f"{sc}/head", g.HEAD_R - 0.0001, -g.SCREW_L - g.HEAD_H, -g.SCREW_L, zinc, collide=False)
        h = g.DRV_BIT_HALF + 0.0003
        box(stage, f"{sc}/recess", (-h, -h, -g.SCREW_L - g.HEAD_H - 0.0002), (h, h, -g.SCREW_L - g.HEAD_H + 0.0002),
            dark, collide=False)
        box(stage, f"{sc}/mark", (h, -0.0005, -g.SCREW_L - g.HEAD_H - 0.0002),
            (g.HEAD_R - 0.0005, 0.0005, -g.SCREW_L - g.HEAD_H + 0.0002), dark, collide=False)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    build_housing()
    build_driver()
    build_screws()
    print("wrote", sorted(p.name for p in OUT.glob("*.usda")))
