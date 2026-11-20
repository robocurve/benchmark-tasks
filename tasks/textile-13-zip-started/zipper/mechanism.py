"""Passive zipper hardware with a free insertion pin and a guarded slider."""

import numpy as np

LENGTH = 0.180
STARTED = 0.0
PITCH = 0.003
CONTROL_HZ = 20
MAX_STEPS = 60 * CONTROL_HZ
END_TOLERANCE = 0.003
PIN_SEATED_TOLERANCE = 0.0005
ORIGIN_XY = (0.145, 0.0)
INITIAL_PIN = (0.028, -0.025, 0.0)
PIN_LIMITS = ((-0.001, 0.042), (-0.040, 0.020), (-0.004, 0.020))
SOCKET_CENTER = (-0.0005, -0.010, 0.024)
# Published commercial repair set FIND-PH0028-52: 10 x 2.5 x 2.5 mm pin,
# 10 x 7 x 4.5 mm retaining box. Internal channel geometry is simplified.
PIN_BAR = ((0.0, -0.010, 0.024), (0.010, 0.0025, 0.0025))
PIN_GRASP = (0.018, -0.010, 0.026)
SLIDER_HANDLE = ((0.0, 0.009, 0.041), (0.018, 0.016, 0.024))
BRACE_POINT = (0.055, 0.060, 0.021)

TAPE = (0.08, 0.31, 0.43)
STEEL = (0.64, 0.68, 0.74)
ORANGE = (0.95, 0.31, 0.05)
PIN_COLOR = (0.68, 0.71, 0.76)


def pin_seated(offset):
    return bool(np.max(np.abs(offset)) <= PIN_SEATED_TOLERANCE)


def author_mechanism(stage, path, translation, orientation, initial_slider=0.0, **kwargs):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade
    from .visual import author_row

    root = UsdGeom.Xform.Define(stage, path)
    if translation is not None:
        root.AddTranslateOp().Set(Gf.Vec3d(*translation))
    if orientation is not None:
        root.AddOrientOp().Set(Gf.Quatf(orientation[3], Gf.Vec3f(*orientation[:3])))
    path = root.GetPath()

    grip = UsdShade.Material.Define(stage, path.AppendChild("GripMaterial"))
    physics = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    physics.CreateStaticFrictionAttr(0.9)
    physics.CreateDynamicFrictionAttr(0.75)

    def link(name, mass, xyz=(0.0, 0.0, 0.0)):
        body = UsdGeom.Xform.Define(stage, path.AppendChild(name))
        body.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(mass)
        return body.GetPath()

    def box(parent, name, center, size, color, *, hidden=False, collision=True):
        cube = UsdGeom.Cube.Define(stage, parent.AppendChild(name))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            prim = cube.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.AddAppliedSchema("PhysxCollisionAPI")
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.00015)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(grip, materialPurpose="physics")
        if hidden:
            cube.MakeInvisible()

    base = link("Base", 0.45)
    base_prim = stage.GetPrimAtPath(base)
    UsdPhysics.ArticulationRootAPI.Apply(base_prim)
    base_prim.AddAppliedSchema("PhysxArticulationAPI")
    base_prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(True)
    base_prim.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(16)
    base_prim.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(4)
    slider = link("Slider", 0.025)
    x_stage = link("PinAxial", 0.004, (INITIAL_PIN[0], 0, 0))
    y_stage = link("PinLateral", 0.004, (INITIAL_PIN[0], INITIAL_PIN[1], 0))
    pin = link("PinSide", 0.008, INITIAL_PIN)

    # Sewn supports stabilize the tape without anchoring the jacket to the table.
    for sign in (1, -1):
        box(base, f"RailSupport_{sign:+d}".replace("+", "plus").replace("-", "minus"),
            (0.104, sign * 0.014, 0.008), (0.184, 0.012, 0.009), TAPE, hidden=True)
        author_row(stage, base, sign, 0.0)
    # A 3.2 x 3.2 mm passage receives the 2.5 x 2.5 mm pin. The exterior
    # is exactly 10 x 7 x 4.5 mm, open toward the collar (+X).
    socket_x = SOCKET_CENTER[0]
    box(base, "RetainerInnerWall", (socket_x, -0.00745, 0.024), (0.010, 0.0019, 0.0045), STEEL)
    box(base, "RetainerOuterWall", (socket_x, -0.01255, 0.024), (0.010, 0.0019, 0.0045), STEEL)
    box(base, "RetainerFloor", (socket_x, -0.010, 0.022075), (0.010, 0.0032, 0.00065), STEEL)
    box(base, "RetainerRoof", (socket_x, -0.010, 0.025925), (0.010, 0.0032, 0.00065), STEEL)
    box(base, "RetainerEndStop", (-0.00525, -0.010, 0.024), (0.0005, 0.0032, 0.0032), STEEL)

    # The slider's pin-side passage lies beside the orange pull, and stays at the bottom until insertion.
    box(slider, "SliderHousing", (0, 0.009, 0.026), (0.018, 0.013, 0.010), STEEL)
    box(slider, "PullTab", *SLIDER_HANDLE, ORANGE)
    box(pin, "InsertionPin", *PIN_BAR, PIN_COLOR)
    # The reinforced polyester tape end is grasped behind the metal pin, leaving the tip exposed.
    box(pin, "ReinforcedTapeEnd", PIN_GRASP, (0.014, 0.010, 0.006), TAPE)
    box(pin, "TapeAttachment", (0.024, -0.018, 0.018), (0.020, 0.014, 0.003), TAPE, collision=False)

    def joint(name, parent, child, axis, limits, resistance):
        j = UsdPhysics.PrismaticJoint.Define(stage, path.AppendChild(name))
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([child])
        j.CreateAxisAttr(axis)
        j.CreateLowerLimitAttr(limits[0])
        j.CreateUpperLimitAttr(limits[1])
        j.CreateCollisionEnabledAttr(False)
        prim = j.GetPrim()
        prim.AddAppliedSchema("PhysxJointAxisAPI:linear")
        for setting, value in (("staticFrictionEffort", resistance), ("dynamicFrictionEffort", resistance * 0.6),
                               ("viscousFrictionCoefficient", 0.4)):
            prim.CreateAttribute(f"physxJointAxis:linear:{setting}", Sdf.ValueTypeNames.Float).Set(value)

    joint("slider_travel", base, slider, "X", (0.0, 0.002), 0.7)
    joint("pin_axial", base, x_stage, "X", PIN_LIMITS[0], 2.0)
    joint("pin_lateral", x_stage, y_stage, "Y", PIN_LIMITS[1], 2.0)
    joint("pin_height", y_stage, pin, "Z", PIN_LIMITS[2], 0.18)
    return root.GetPrim()
