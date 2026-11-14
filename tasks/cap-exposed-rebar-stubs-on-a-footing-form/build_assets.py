"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/form.usda: the footing form, a kinematic body (no mass): the 30 mm plywood floor, the five #4 bars standing in
  it (cylinders) and the three thin staging pads the loose caps stand on. Its origin is on the table top under the
  centre of the floor.
- assets/cap.usda: one safety cap, a free rigid body made of box colliders (a stem ring with a flared mouth, a head
  ring round the blind bore's upper end, and a solid roof). Its origin is on the bore axis at the cap's mouth; z is up.
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


def build_cap():
    stage, root = stage_for("cap")
    r = "/cap"
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(g.CAP_MASS)
    red = material(stage, f"{r}/Looks/plastic", RED, 0.45)
    hb, st, hd = g.CAP_BORE / 2, g.CAP_STEM, g.CAP_HEAD
    fh, sh_ = g.CAP_FUNNEL_H, g.CAP_STEM_H
    # stem ring above the funnel: z from the funnel's top to the head's underside
    z0, z1 = fh, sh_
    zc, hz = (z0 + z1) / 2, z1 - z0
    side = (st / 2 - hb) / 2 + hb                      # centre of a wall slab along its thickness axis
    thick = st / 2 - hb
    for sgn, name in ((1, "px"), (-1, "nx")):
        box(stage, f"{r}/stem_{name}", (sgn * side, 0.0, zc), (thick, st, hz), RED, red)
    for sgn, name in ((1, "py"), (-1, "ny")):
        box(stage, f"{r}/stem_{name}", (0.0, sgn * side, zc), (g.CAP_BORE, thick, hz), RED, red)
    # head ring round the bore's upper end (z from the head's underside to the bore's roof)
    z0, z1 = sh_, g.CAP_BORE_DEPTH
    zc, hz = (z0 + z1) / 2, z1 - z0
    side = (hd / 2 - hb) / 2 + hb
    thick = hd / 2 - hb
    for sgn, name in ((1, "px"), (-1, "nx")):
        box(stage, f"{r}/head_{name}", (sgn * side, 0.0, zc), (thick, hd, hz), RED, red)
    for sgn, name in ((1, "py"), (-1, "ny")):
        box(stage, f"{r}/head_{name}", (0.0, sgn * side, zc), (g.CAP_BORE, thick, hz), RED, red)
    # the roof
    box(stage, f"{r}/roof", (0.0, 0.0, (g.CAP_BORE_DEPTH + g.CAP_H) / 2), (hd, hd, g.CAP_H - g.CAP_BORE_DEPTH), RED, red)
    # the flared mouth: a 45 degree slab from the mouth's opening (half-width CAP_FUNNEL_HALF at z = 0) in to the bore
    # (half-width hb at z = CAP_FUNNEL_H), plus a filler block outside it
    t = 0.006
    run = g.CAP_FUNNEL_HALF - hb
    length = math.hypot(run, fh)
    ang = math.degrees(math.atan2(run, fh))             # the slab's lean from vertical (45 degrees here)
    mx, mz = (g.CAP_FUNNEL_HALF + hb) / 2, fh / 2
    nx, nz = fh / length, run / length                  # the outward normal in (r, z)
    cx, cz = mx + nx * t / 2, mz + nz * t / 2
    fill_in = g.CAP_FUNNEL_HALF + 0.001
    for sgn, name in ((1, "px"), (-1, "nx")):
        box(stage, f"{r}/funnel_{name}", (sgn * cx, 0.0, cz), (t, st, length), RED, red, rot_deg=(0.0, -sgn * ang, 0.0))
        box(stage, f"{r}/fill_{name}", (sgn * (fill_in + st / 2) / 2 + sgn * 0.0, 0.0, fh / 2),
            (st / 2 - fill_in, st, fh), RED, red)
    for sgn, name in ((1, "py"), (-1, "ny")):
        box(stage, f"{r}/funnel_{name}", (0.0, sgn * cx, cz), (st, t, length), RED, red, rot_deg=(sgn * ang, 0.0, 0.0))
        box(stage, f"{r}/fill_{name}", (0.0, sgn * (fill_in + st / 2) / 2, fh / 2), (st, st / 2 - fill_in, fh), RED, red)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_form()
    build_cap()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
