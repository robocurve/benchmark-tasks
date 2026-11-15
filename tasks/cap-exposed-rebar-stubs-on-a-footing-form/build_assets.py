"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/form.usda: the footing form, a kinematic body (no mass): the 30 mm plywood floor, the five #4 bars standing in
  it (cylinders) and the three thin staging pads the loose caps stand on. Its origin is on the table top under the
  centre of the floor.
- assets/cap.usda: one impalement-rated safety cap, a floating articulation: the base (a 4 in square steel top plate with a raised grip boss on a
  polymer body with a blind bore and a 45 degree flared mouth, all box colliders) and two spring-loaded gripping fins
  (`jaw_p`, `jaw_n`: prismatic joints along x, pressed in toward the bar by a spring that task.py's drives set). Its origin
  is on the bore axis at the cap's mouth; z is up.
Colliders are boxes and cylinders; nothing is a mesh.
"""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"

PLYWOOD = (0.72, 0.58, 0.38)
STEEL = (0.30, 0.30, 0.32)
PAD_GREY = (0.45, 0.46, 0.48)
RED = (0.82, 0.10, 0.07)
ORANGE = (0.95, 0.45, 0.05)
DARK = (0.14, 0.14, 0.15)


def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def box(stage, path, center, size, color, mat=None, rot_deg=(0.0, 0.0, 0.0)):
    """A box collider: centre and size (x, y, z) in the parent's frame, optionally turned by ``rot_deg`` (x, y, z)."""
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    if any(rot_deg):
        c.AddRotateXYZOp().Set(Gf.Vec3f(*rot_deg))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        bind(c, mat)
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def cyl(stage, path, center, radius, height, color, mat=None):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr("Z")
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        bind(c, mat)
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def build_form():
    stage, root = stage_for("form")
    r = "/form"
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())      # task.py makes it kinematic (it has no mass)
    wood = material(stage, f"{r}/Looks/ply", PLYWOOD, 0.8)
    steel = material(stage, f"{r}/Looks/bar", STEEL, 0.4)
    grey = material(stage, f"{r}/Looks/pad", PAD_GREY, 0.6)
    fx, fy, fz = g.FORM_SIZE
    # the origin is on the table top at (0, 0); task.py places it at (FORM_X, 0)
    box(stage, f"{r}/floor", (0.0, 0.0, fz / 2), (fx, fy, fz), PLYWOOD, wood)
    for i, y in enumerate(g.STUB_Y):
        cyl(stage, f"{r}/bar_{i}", (0.0, y, fz + g.BAR_H / 2), g.BAR_R, g.BAR_H, STEEL, steel)
    for j, y in enumerate(g.CAP_START_Y):
        box(stage, f"{r}/pad_{j}", (g.PAD_X - g.FORM_X, y, g.PAD_SIZE[2] / 2), g.PAD_SIZE, PAD_GREY, grey)
    stage.GetRootLayer().Save()


def rigid(stage, path, mass, pos=(0.0, 0.0, 0.0)):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
    return xf


def prismatic(stage, path, body0, body1, pos0, rot0=(1, 0, 0, 0), limits=(0.0, 0.01)):
    """A prismatic joint along the local x axis of body0's frame turned by rot0 (w, x, y, z). A spring drive (stiffness, damping,
    target 0) is set by task.py's drives; the lower limit is the fin's rest stop."""
    j = UsdPhysics.PrismaticJoint.Define(stage, path)
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(*rot0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(*rot0))               # the fin is not turned, so both joint frames carry the same turn
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(g.JAW_K)
    drive.CreateDampingAttr(g.JAW_DAMP)
    drive.CreateTargetPositionAttr(0.0)
    return j


def build_cap():
    stage, root = stage_for("cap")
    r = "/cap"
    prim = rigid(stage, f"{r}/base", g.CAP_MASS - 2 * g.JAW_MASS).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    s = f"{r}/base"
    plate_m = material(stage, f"{r}/Looks/plate", ORANGE, 0.4)
    body_m = material(stage, f"{r}/Looks/body", DARK, 0.6)
    jaw_m = material(stage, f"{r}/Looks/jaw", STEEL, 0.3)
    B, by, hb = g.CAP_BODY, g.CAP_BORE_Y / 2, g.CAP_FUNNEL_HALF
    fh, bd, H = g.CAP_FUNNEL_H, g.CAP_BORE_DEPTH, g.CAP_BODY_H
    cav = 0.016                                         # half-width (x) of the cavity the fins slide in
    # the steel plate and the polymer roof over the bore
    box(stage, f"{s}/plate", (0.0, 0.0, H + g.CAP_PLATE_T / 2), (g.CAP_PLATE, g.CAP_PLATE, g.CAP_PLATE_T), ORANGE, plate_m)
    box(stage, f"{s}/roof", (0.0, 0.0, (bd + H) / 2), (B, B, H - bd), DARK, body_m)
    box(stage, f"{s}/boss", (0.0, 0.0, H + g.CAP_PLATE_T + g.CAP_BOSS_H / 2), (g.CAP_BOSS, g.CAP_BOSS, g.CAP_BOSS_H), DARK, body_m)
    # the walls round the fins' cavity (z from the funnel's top to the roof): +-y walls full width, +-x walls beyond the fins
    zc, hz = (fh + bd) / 2, bd - fh
    for sgn, name in ((1, "py"), (-1, "ny")):
        box(stage, f"{s}/wall_{name}", (0.0, sgn * (by + (B / 2 - by) / 2), zc), (B, B / 2 - by, hz), DARK, body_m)
    for sgn, name in ((1, "px"), (-1, "nx")):
        box(stage, f"{s}/wall_{name}", (sgn * (cav + (B / 2 - cav) / 2), 0.0, zc), (B / 2 - cav, 2 * by, hz), DARK, body_m)
    # the flared mouth below it: a 45 degree slab from the opening in to the bore's width, plus a filler block outside it
    hbf = by                                            # the funnel ends as wide as the bore across y
    t = 0.006
    run = hb - hbf
    length = math.hypot(run, fh)
    ang = math.degrees(math.atan2(run, fh))
    mx, mz = (hb + hbf) / 2, fh / 2
    nx, nz = fh / length, run / length
    cx, cz = mx + nx * t / 2, mz + nz * t / 2
    fill_in = hb + 0.001
    for sgn, name in ((1, "px"), (-1, "nx")):
        box(stage, f"{s}/funnel_{name}", (sgn * cx, 0.0, cz), (t, B, length), DARK, body_m, rot_deg=(0.0, -sgn * ang, 0.0))
        box(stage, f"{s}/fill_{name}", (sgn * (fill_in + B / 2) / 2, 0.0, fh / 2), (B / 2 - fill_in, B, fh), DARK, body_m)
    for sgn, name in ((1, "py"), (-1, "ny")):
        box(stage, f"{s}/funnel_{name}", (0.0, sgn * cx, cz), (B, t, length), DARK, body_m, rot_deg=(sgn * ang, 0.0, 0.0))
        box(stage, f"{s}/fill_{name}", (0.0, sgn * (fill_in + B / 2) / 2, fh / 2), (B, B / 2 - fill_in, fh), DARK, body_m)
    # the two fins: each a flat-faced block with a 45 degree chamfer at its lower inner corner, on a prismatic joint
    z0, z1 = g.JAW_Z
    chamfer = 0.003
    half_gap = g.JAW_REST_GAP / 2
    zj = (z0 + chamfer + z1) / 2                        # the fin's origin height (its main block's middle)
    for sgn, name, rot0 in ((1, "p", (1, 0, 0, 0)), (-1, "n", (0, 0, 0, 1))):
        ox = sgn * (half_gap + g.JAW_T / 2)
        jp = f"{r}/jaw_{name}"
        rigid(stage, jp, g.JAW_MASS, (ox, 0.0, zj))
        box(stage, f"{jp}/face", (0.0, 0.0, 0.0), (g.JAW_T, 0.013, z1 - z0 - chamfer), STEEL, jaw_m)
        # the chamfer: a slab from (half_gap + chamfer, z0) up and in to (half_gap, z0 + chamfer), in the fin's frame
        mid_x, mid_z = half_gap + chamfer / 2, z0 + chamfer / 2
        ln = math.hypot(chamfer, chamfer)
        tt = 0.003
        ccx, ccz = mid_x + tt / 2 * 0.7071, mid_z + tt / 2 * 0.7071
        box(stage, f"{jp}/chamfer", (sgn * ccx - ox, 0.0, ccz - zj), (tt, 0.013, ln), STEEL, jaw_m, rot_deg=(0.0, -sgn * 45.0, 0.0))
        box(stage, f"{jp}/chamfer_fill", (sgn * (half_gap + chamfer + (g.JAW_T - chamfer) / 2 + 0.0005) - ox, 0.0,
                                           z0 + chamfer / 2 - zj), (g.JAW_T - chamfer - 0.001, 0.013, chamfer), STEEL, jaw_m)
        prismatic(stage, f"{r}/joints/jaw_{name}", s, jp, (ox, 0.0, zj), rot0=rot0, limits=(0.0, g.JAW_TRAVEL))
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_form()
    build_cap()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
