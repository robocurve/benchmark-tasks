"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/server.usda: the 1U chassis as one rigid body (origin: its ears' back face, centre, underside). Colliders: the
  body with its front panel, the two rack ears, and the two chassis members of the rail kit with their narrowed tips.
  Visuals: the front panel's ribs, power and reset buttons, LEDs and USB ports, the ears' mounting slots, the lid seam.
- assets/rack.usda: the 9U open-frame rack (kinematic, world frame): four L-section posts with their square holes,
  base and top frames, a top tray, and at U3 the two rack rails of the kit (outer and centre members as one channel
  with flared mouths, low friction), on their L brackets.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
BLACK = (0.035, 0.035, 0.04)
PANEL = (0.06, 0.06, 0.065)
RIB = (0.16, 0.16, 0.17)
ZINC = (0.70, 0.71, 0.73)
POST = (0.05, 0.05, 0.055)
HOLE = (0.005, 0.005, 0.005)
OFF = (0.001, 0.0)


def stage_for(name: str):
    OUT.mkdir(exist_ok=True)
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


def physics_material(stage, path: str, static: float, dynamic: float, combine: str = "min"):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    prim = mat.GetPrim()
    prim.AddAppliedSchema("PhysxMaterialAPI")
    prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set(combine)
    return mat


def bind(prim, mat, purpose=None) -> None:
    api = UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    if purpose:
        api.Bind(mat, UsdShade.Tokens.weakerThanDescendants, purpose)
    else:
        api.Bind(mat)


def offsets(prim, contact=0.001, rest=0.0):
    prim = prim.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def box(stage, path, lo, hi, mat=None, collide=True, off=OFF, phys=None):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
        if phys is not None:
            bind(c, phys, "physics")
    return c


def obox(stage, path, center, size, axis, angle, mat=None, collide=True, off=OFF, phys=None):
    """A box of ``size`` (x, y, z) at ``center``, turned ``angle`` rad about the world ``axis`` ('X' or 'Z')."""
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    h = angle / 2
    q = Gf.Quatf(math.cos(h), math.sin(h), 0, 0) if axis == "X" else Gf.Quatf(math.cos(h), 0, 0, math.sin(h))
    c.AddOrientOp().Set(q)
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
        if phys is not None:
            bind(c, phys, "physics")
    return c


def cylinder(stage, path, radius, center, length, axis, mat=None):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(length))
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if mat is not None:
        bind(c, mat)
    return c


def box_inertia(m, sx, sy, sz):
    return (m * (sy * sy + sz * sz) / 12, m * (sx * sx + sz * sz) / 12, m * (sx * sx + sy * sy) / 12)


# -- the server ----------------------------------------------------------------------------------------------------------
def build_server() -> None:
    stage, root = stage_for("server")
    r = "/server"
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(g.MASS)
    m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, g.COM)))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*box_inertia(g.MASS, g.BODY_W, g.DEPTH + 0.1, g.BODY_H)))
    black = material(stage, f"{r}/Looks/black", BLACK, 0.45, 0.3)
    panel = material(stage, f"{r}/Looks/panel", PANEL, 0.35, 0.6)
    rib = material(stage, f"{r}/Looks/rib", RIB, 0.3, 0.7)
    zinc = material(stage, f"{r}/Looks/zinc", ZINC, 0.35, 0.8)
    dark = material(stage, f"{r}/Looks/dark", HOLE, 0.9)
    blue = material(stage, f"{r}/Looks/usb", (0.05, 0.25, 0.85), 0.4)
    green = material(stage, f"{r}/Looks/led_green", (0.1, 0.9, 0.2), 0.2)
    amber = material(stage, f"{r}/Looks/led_amber", (1.0, 0.55, 0.05), 0.2)
    grey = material(stage, f"{r}/Looks/button", (0.55, 0.56, 0.58), 0.3, 0.6)
    hw, hh = g.BODY_W / 2, g.BODY_H
    y0, y1 = g.PANEL_Y[0], g.BODY_Y[1]
    box(stage, f"{r}/body", (-hw, 0.0, 0.0), (hw, y1, hh), black)
    box(stage, f"{r}/panel", (-hw, y0, 0.0), (hw, 0.0, hh), panel)
    for s, tag in ((-1, "a"), (1, "b")):
        x0, x1 = s * hw, s * (hw + g.EAR_W)
        box(stage, f"{r}/ear_{tag}", (x0, -g.EAR_T, 0.0), (x1, 0.0, g.EAR_H), panel)
        for k, z in enumerate((0.0095, 0.0349)):          # the ears' mounting slots
            xc = s * g.HOLE_PITCH_X
            box(stage, f"{r}/ear_{tag}_slot{k}", (xc - 0.005, -g.EAR_T - 0.0003, z - 0.0032),
                (xc + 0.005, -g.EAR_T + 0.0002, z + 0.0032), dark, collide=False)
        # the rail kit's chassis member, and its narrowed tip
        mx0, mx1 = s * g.MEMBER_X[0], s * g.MEMBER_X[1]
        zc, mh = g.MEMBER_ZC, g.MEMBER_H / 2
        box(stage, f"{r}/member_{tag}", (mx0, g.MEMBER_Y[0], zc - mh), (mx1, g.MEMBER_Y[1] - g.TIP_TAPER, zc + mh), zinc)
        box(stage, f"{r}/member_{tag}_tip", (mx0, g.MEMBER_Y[1] - g.TIP_TAPER, zc - mh + 0.002),
            (s * (g.MEMBER_X[1] - 0.001), g.MEMBER_Y[1], zc + mh - 0.002), zinc)
        for k in range(5):                                # screw heads holding the member to the chassis
            yk = g.MEMBER_Y[0] + 0.02 + k * 0.06
            if yk < y1 - 0.01:
                cylinder(stage, f"{r}/member_{tag}_screw{k}", 0.0035, (s * (g.MEMBER_X[1] + 0.0003), yk, zc), 0.0008,
                         "X", rib)
    # front panel: horizontal ribs, the control block at the right (+x from the front), lid seam
    yf = y0 - 0.0003
    for k in range(7):
        z = 0.006 + k * 0.0052
        box(stage, f"{r}/rib{k}", (-hw + 0.012, yf, z - 0.0009), (hw - 0.095, y0, z + 0.0009), rib, collide=False)
    cx = hw - 0.060
    box(stage, f"{r}/ctl_plate", (cx - 0.032, yf, 0.004), (cx + 0.032, y0, hh - 0.004), rib, collide=False)
    for k, x in enumerate((cx - 0.020, cx - 0.008)):       # two USB 2.0 ports
        box(stage, f"{r}/usb{k}", (x - 0.0035, yf - 0.0002, 0.012), (x + 0.0035, yf + 0.0001, 0.026), blue, False)
    cylinder(stage, f"{r}/power", 0.0045, (cx + 0.010, yf - 0.0006, 0.026), 0.0015, "Y", grey)
    cylinder(stage, f"{r}/reset", 0.0025, (cx + 0.010, yf - 0.0004, 0.012), 0.001, "Y", grey)
    box(stage, f"{r}/led_power", (cx + 0.021, yf - 0.0003, 0.025), (cx + 0.025, yf, 0.028), green, False)
    box(stage, f"{r}/led_hdd", (cx + 0.021, yf - 0.0003, 0.013), (cx + 0.025, yf, 0.016), amber, False)
    box(stage, f"{r}/lid_seam", (-hw, 0.006, hh), (hw, 0.008, hh + 0.0002), rib, collide=False)
    for k in range(10):                                    # lid vent slots near the front
        x = -0.15 + k * 0.012
        box(stage, f"{r}/vent{k}", (x, 0.03, hh), (x + 0.006, 0.07, hh + 0.0002), dark, collide=False)
    stage.Save()


# -- the rack --------------------------------------------------------------------------------------------------------------
def build_rack() -> None:
    stage, root = stage_for("rack")
    r = "/rack"
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.RigidBodyAPI(root.GetPrim()).CreateKinematicEnabledAttr(True)
    post = material(stage, f"{r}/Looks/post", POST, 0.6, 0.2)
    zinc = material(stage, f"{r}/Looks/zinc", ZINC, 0.35, 0.8)
    dark = material(stage, f"{r}/Looks/hole", HOLE, 0.9)
    slide = physics_material(stage, f"{r}/Looks/slide", g.RAIL_FRICTION, g.RAIL_FRICTION, "min")
    T = g.TABLE_TOP
    z_post0, z_post1 = T + 0.030, T + g.RACK_TOP + 0.010
    fx0, fx1 = g.FLANGE_X
    for s, tag in ((-1, "a"), (1, "b")):
        for yname, yface, leg_dir in (("front", g.Y_FRONT, 1), ("rear", g.Y_REAR, -1)):
            box(stage, f"{r}/{yname}_flange_{tag}", (s * fx0, yface, z_post0), (s * fx1, yface + g.FLANGE_T, z_post1), post)
            y_leg = (yface, yface + leg_dir * g.LEG_DEPTH)
            box(stage, f"{r}/{yname}_leg_{tag}", (s * (fx1 - g.LEG_T), y_leg[0], z_post0), (s * fx1, y_leg[1], z_post1), post)
            # base and top frames' corner blocks
        box(stage, f"{r}/base_side_{tag}", (s * (fx1 - 0.025), g.Y_FRONT - 0.010, T), (s * fx1, g.Y_REAR + 0.045, T + 0.030), post)
        box(stage, f"{r}/top_side_{tag}", (s * (fx1 - 0.025), g.Y_FRONT - 0.010, T + g.RACK_HEIGHT - 0.030),
            (s * fx1, g.Y_REAR + 0.045, T + g.RACK_HEIGHT), post)
        # square holes on the front faces of the front flanges (3 per U), and on the rear flanges
        for u in range(g.RACK_U):
            for k, dz in enumerate((0.00635, 0.02223, 0.0381)):
                z = T + g.RACK_BASE + u * g.U + dz
                xc = s * g.HOLE_PITCH_X
                box(stage, f"{r}/hole_{tag}_{u}_{k}", (xc - 0.0048, g.Y_FRONT - 0.0003, z - 0.0048),
                    (xc + 0.0048, g.Y_FRONT + 0.0001, z + 0.0048), dark, collide=False)
        # the kit's rack rail at U3: web, lips, flared mouth, L brackets
        zc, ph = g.RAIL_ZC, g.POCKET_H / 2
        ry0, ry1 = g.RAIL_Y
        web0, web1 = s * g.WEB_IN, s * (g.WEB_IN + g.WEB_T)
        lip0, lip1 = s * g.LIP_IN, s * g.WEB_IN
        yl0 = ry0 + g.FLARE_L
        box(stage, f"{r}/rail_{tag}_web", (web0, yl0, zc - ph - g.LIP_T - 0.002), (web1, ry1, zc + ph + g.LIP_T + 0.002),
            zinc, phys=slide)
        box(stage, f"{r}/rail_{tag}_top", (lip0, yl0, zc + ph), (lip1, ry1, zc + ph + g.LIP_T), zinc, phys=slide)
        box(stage, f"{r}/rail_{tag}_bottom", (lip0, yl0, zc - ph - g.LIP_T), (lip1, ry1, zc - ph), zinc, phys=slide)
        ang = math.atan2(g.FLARE, g.FLARE_L)
        L = math.hypot(g.FLARE, g.FLARE_L)
        ym = ry0 + g.FLARE_L / 2
        xl = s * (g.LIP_IN + g.WEB_IN) / 2
        wl = g.WEB_IN - g.LIP_IN
        # the top flare rises toward the mouth (-y): turned about +x by -ang; the bottom one by +ang
        obox(stage, f"{r}/rail_{tag}_flare_top", (xl, ym, zc + ph + g.LIP_T / 2 + g.FLARE / 2), (wl, L, g.LIP_T), "X",
             -ang, zinc, phys=slide)
        obox(stage, f"{r}/rail_{tag}_flare_bottom", (xl, ym, zc - ph - g.LIP_T / 2 - g.FLARE / 2), (wl, L, g.LIP_T), "X",
             ang, zinc, phys=slide)
        obox(stage, f"{r}/rail_{tag}_flare_web", (s * (g.WEB_IN + g.WEB_T / 2 + g.FLARE / 2), ym, zc),
             (g.WEB_T, L, g.POCKET_H + 2 * g.LIP_T + 0.004), "Z", s * ang, zinc, phys=slide)
        for yname, yb in (("front", g.Y_FRONT + g.FLANGE_T), ("rear", g.Y_REAR - 0.002)):
            box(stage, f"{r}/rail_{tag}_bracket_{yname}", (web1, yb, zc - 0.016), (s * (fx1 - g.LEG_T), yb + 0.002, zc + 0.016),
                zinc)
    for yname, y in (("front", g.Y_FRONT - 0.010), ("rear", g.Y_REAR + 0.020)):
        box(stage, f"{r}/base_{yname}", (-fx1, y, T), (fx1, y + 0.025, T + g.RACK_BASE - 0.004), post)
        box(stage, f"{r}/top_{yname}", (-fx1, y, T + g.RACK_HEIGHT - 0.045), (fx1, y + 0.025, T + g.RACK_HEIGHT), post)
    box(stage, f"{r}/top_tray", (-fx1, g.Y_FRONT - 0.010, T + g.RACK_HEIGHT - 0.004), (fx1, g.Y_REAR + 0.045, T + g.RACK_HEIGHT),
        post)
    stage.Save()


if __name__ == "__main__":
    build_server()
    build_rack()
    print("wrote", OUT / "server.usda", OUT / "rack.usda")
