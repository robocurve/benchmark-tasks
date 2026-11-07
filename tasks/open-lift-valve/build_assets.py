"""Write the scene's USD assets (assets/valve.usda, assets/lift.usda, assets/knife.usda) from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- valve: an articulation with a fixed brass body; the blue handwheel turns about the vertical (revolute ``wheel``,
  no limits) and carries a red spinner knob that turns freely on its own axis (revolute ``knob``).
- lift: an articulation with a fixed base (hinge brackets, cylinder barrel); the board turns on the hinge (revolute
  ``board``, about y) and the rod slides out of the barrel (prismatic ``rod``). The task drives both from the
  handwheel's turns. The rod has no collider, so it never fights the board it shows holding up.
- knife: one rigid body, a handle and a thin blade.

Each asset's origin is where task.py places it: the valve's on the table under the handwheel's axis, the lift's on
the table under the hinge axis, the knife's at the handle's centre.
"""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"

BRASS = (0.72, 0.55, 0.22)
BLUE = (0.16, 0.34, 0.78)
RED = (0.82, 0.12, 0.10)
STEEL = (0.62, 0.66, 0.70)
CHROME = (0.85, 0.88, 0.92)
DARK = (0.22, 0.24, 0.27)
YELLOW = (0.95, 0.74, 0.08)
BOARD = (0.93, 0.92, 0.89)
HANDLE = (0.08, 0.08, 0.09)
BLADE = (0.42, 0.45, 0.50)


def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def body(stage, path: str, mass: float, pos=(0.0, 0.0, 0.0)):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
    return xf


def _finish(prim, pos, color, collide: bool, rot_z_deg: float = 0.0, scale=None):
    x = UsdGeom.Xformable(prim)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rot_z_deg:
        x.AddRotateZOp().Set(rot_z_deg)
    if scale is not None:
        x.AddScaleOp().Set(Gf.Vec3f(*scale))
    UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())


def cube(stage, path, size, pos, color, collide=True, rot_z_deg=0.0):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    _finish(c, pos, color, collide, rot_z_deg, scale=size)
    return c


def cylinder(stage, path, r, h, pos, color, axis="Z", collide=True):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(h)
    c.CreateAxisAttr(axis)
    _finish(c, pos, color, collide)
    return c


def capsule(stage, path, r, h, pos, color, axis="X", collide=True, rot_z_deg=0.0):
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(h)
    c.CreateAxisAttr(axis)
    _finish(c, pos, color, collide, rot_z_deg)
    return c


def sphere(stage, path, r, pos, color, collide=True):
    s = UsdGeom.Sphere.Define(stage, path)
    s.CreateRadiusAttr(r)
    _finish(s, pos, color, collide)
    return s


def fix_to_world(stage, root: str, base: str):
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(root))
    j = UsdPhysics.FixedJoint.Define(stage, f"{root}/joints/root_joint")
    j.CreateBody1Rel().SetTargets([Sdf.Path(base)])


def joint(stage, kind, path, body0, body1, pos0, pos1, axis, limits=None):
    j = (UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    if limits is not None:
        j.CreateLowerLimitAttr(limits[0])
        j.CreateUpperLimitAttr(limits[1])
    # a force drive with zero gains; the task's actuator config sets the real ones
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


def build_valve():
    stage, _ = stage_for("valve")
    r = "/valve"
    base = body(stage, f"{r}/base", 3.0)
    cube(stage, f"{r}/base/body", (0.07, 0.07, 0.07), (0, 0, 0.035), BRASS)
    for s in (-1, 1):
        cube(stage, f"{r}/base/port{'ab'[s > 0]}", (0.05, 0.02, 0.05), (0, s * 0.045, 0.03), BRASS)
    cylinder(stage, f"{r}/base/bonnet", 0.02, 0.045, (0, 0, 0.0925), BRASS)
    cylinder(stage, f"{r}/base/stem", 0.007, 0.02, (0, 0, 0.12), STEEL)
    # a white arrow on the table pointing at the knob's start place, for counting turns
    cube(stage, f"{r}/base/mark", (0.04, 0.008, 0.001), (-(g.RIM_R + 0.045), 0, 0.0005), (0.95, 0.95, 0.93),
         collide=False)
    cube(stage, f"{r}/base/mark_tip", (0.014, 0.014, 0.001), (-(g.RIM_R + 0.022), 0, 0.0005), (0.95, 0.95, 0.93),
         collide=False, rot_z_deg=45.0)

    wheel = body(stage, f"{r}/wheel", 0.4, (0, 0, g.WHEEL_Z))
    n = 16
    seg = 2 * g.RIM_R * math.sin(math.pi / n)
    for k in range(n):
        a = (k + 0.5) * 2 * math.pi / n
        capsule(stage, f"{r}/wheel/rim{k:02d}", g.RIM_TUBE, seg, (g.RIM_R * math.cos(a), g.RIM_R * math.sin(a), 0),
                BLUE, axis="X", rot_z_deg=math.degrees(a) + 90.0)
    for k, a in enumerate((0.0, 2 * math.pi / 3, 4 * math.pi / 3)):
        cube(stage, f"{r}/wheel/spoke{k}", (g.RIM_R, 0.012, 0.008),
             (g.RIM_R / 2 * math.cos(a), g.RIM_R / 2 * math.sin(a), 0), BLUE, rot_z_deg=math.degrees(a))
    cylinder(stage, f"{r}/wheel/hub", 0.018, 0.022, (0, 0, 0), BLUE)
    kx, ky = g.RIM_R * math.cos(g.KNOB_ANGLE), g.RIM_R * math.sin(g.KNOB_ANGLE)
    cylinder(stage, f"{r}/wheel/boss", g.KNOB_R, g.KNOB_Z0 - g.WHEEL_Z, (kx, ky, (g.KNOB_Z0 - g.WHEEL_Z) / 2), BLUE)

    knob = body(stage, f"{r}/knob", 0.03, (kx, ky, g.KNOB_MID))
    cylinder(stage, f"{r}/knob/grip", g.KNOB_R, g.KNOB_H, (0, 0, 0), RED)

    fix_to_world(stage, r, f"{r}/base")
    joint(stage, "revolute", f"{r}/joints/wheel", f"{r}/base", f"{r}/wheel", (0, 0, g.WHEEL_Z), (0, 0, 0), "Z")
    joint(stage, "revolute", f"{r}/joints/knob", f"{r}/wheel", f"{r}/knob",
          (kx, ky, g.KNOB_MID - g.WHEEL_Z), (0, 0, 0), "Z")
    stage.GetRootLayer().Save()
    return base, wheel


def build_lift():
    stage, _ = stage_for("lift")
    r = "/lift"
    body(stage, f"{r}/base", 4.0)
    for s, tag in ((-1, "r"), (1, "l")):
        y = s * (g.BOARD_W / 2 + 0.016)
        cube(stage, f"{r}/base/bracket_{tag}", (0.04, 0.02, g.HINGE_Z), (-0.015, y, g.HINGE_Z / 2), DARK)
        cylinder(stage, f"{r}/base/knuckle_{tag}", 0.009, 0.02, (0, y, g.HINGE_Z), STEEL, axis="Y", collide=False)
    cylinder(stage, f"{r}/base/pin", 0.004, g.BOARD_W + 0.05, (0, 0, g.HINGE_Z), STEEL, axis="Y", collide=False)
    cx = -g.LEVER
    cy = g.CYL_Y
    cylinder(stage, f"{r}/base/barrel", g.CYL_R, g.CYL_TOP - 0.008, (cx, cy, 0.008 + (g.CYL_TOP - 0.008) / 2), YELLOW)
    cylinder(stage, f"{r}/base/foot", g.CYL_R + 0.008, 0.008, (cx, cy, 0.004), DARK)
    cylinder(stage, f"{r}/base/gland", g.CYL_R + 0.002, 0.006, (cx, cy, g.CYL_TOP - 0.003), DARK, collide=False)
    cylinder(stage, f"{r}/base/port", 0.007, 0.012, (cx - g.CYL_R - 0.004, cy, 0.02), DARK, axis="X", collide=False)

    body(stage, f"{r}/board", 0.6, (0, 0, g.HINGE_Z))
    cube(stage, f"{r}/board/top", (g.BOARD_L, g.BOARD_W, g.BOARD_T), (-g.BOARD_L / 2, 0, 0), BOARD)
    cube(stage, f"{r}/board/leaf", (0.02, g.BOARD_W - 0.02, 0.003), (-0.012, 0, -g.BOARD_T / 2 - 0.0015), STEEL,
         collide=False)

    rod_len = g.UNDER0 - 0.03
    body(stage, f"{r}/rod", 0.05, (cx, cy, 0))
    cylinder(stage, f"{r}/rod/shaft", g.ROD_R, rod_len - 0.006, (0, 0, 0.03 + (rod_len - 0.006) / 2), CHROME,
             collide=False)
    cylinder(stage, f"{r}/rod/pad", 0.018, 0.006, (0, 0, g.UNDER0 - 0.003), DARK, collide=False)

    fix_to_world(stage, r, f"{r}/base")
    joint(stage, "revolute", f"{r}/joints/board", f"{r}/base", f"{r}/board", (0, 0, g.HINGE_Z), (0, 0, 0), "Y",
          limits=(-2.0, 30.0))
    joint(stage, "prismatic", f"{r}/joints/rod", f"{r}/base", f"{r}/rod", (cx, cy, 0), (0, 0, 0), "Z",
          limits=(0.0, 0.1))
    stage.GetRootLayer().Save()


def build_knife():
    stage, root = stage_for("knife")
    r = "/knife"
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(g.KNIFE_MASS)
    cube(stage, f"{r}/handle", g.HANDLE, (0, 0, 0), HANDLE)
    cube(stage, f"{r}/bolster", (0.024, 0.006, 0.014), (0, -g.HANDLE[1] / 2 - 0.003, -0.001), STEEL, collide=False)
    cube(stage, f"{r}/blade", g.BLADE, (0, g.BLADE_Y, g.BLADE_Z), BLADE)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    build_valve()
    build_lift()
    build_knife()
    for p in sorted(OUT.glob("*.usda")):
        print(p, p.stat().st_size, "bytes")
