"""Task-only USD joint and compound carriage authoring; also works without Kit."""

from . import layout as L


def cube(stage, path, position, size, color, collision=True):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*position))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    material = UsdShade.Material.Define(stage, path + '/material')
    shader = UsdShade.Shader.Define(stage, path + '/material/shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    # Match the station's sRGB-to-linear material convention.
    from yam_arena.station import linear_rgb
    from pxr import Sdf
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.6)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c.GetPrim()


def augment_slider(stage, root_path):
    """Attach the tab/index and a passive world-anchored X prismatic joint before PhysX parsing."""
    from pxr import Gf, Sdf, UsdPhysics
    root = stage.GetPrimAtPath(root_path)
    if not root.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError('Slider must be a real dynamic rigid body')
    for name, pos, size, color, collision in L.CHILDREN:
        cube(stage, root_path + '/' + name, pos, size, color, collision)
    # Compound colliders are siblings, so a material on the original mesh does
    # not reach the projecting handle. Bind identical physics explicitly.
    from pxr import Usd, UsdShade
    material = UsdShade.Material.Define(stage, root_path + '/slider_physics')
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(.20)
    physics.CreateDynamicFrictionAttr(.15)
    physics.CreateRestitutionAttr(0.0)
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            # PhysxSchema's Python module is loaded by Kit only. Author the same
            # USD schema/attributes so standalone geometry exports also work.
            prim.AddAppliedSchema('PhysxCollisionAPI')
            prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.001)
            prim.CreateAttribute('physxCollision:restOffset', Sdf.ValueTypeNames.Float).Set(0.0)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
    # body0 omitted = world. Local frame0 is the starting carriage world frame.
    # Scene has exactly one env, with origin zero, per the official task runner.
    joint = UsdPhysics.PrismaticJoint.Define(stage, root_path + '/guide_joint')
    joint.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
    joint.CreateAxisAttr('X')
    joint.CreateLocalPos0Attr(Gf.Vec3f(*L.HOME))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(L.TRAVEL)
    joint.CreateCollisionEnabledAttr(False)
    # No drive, target position or task motor; only robot contact moves the carriage.
    return joint


def build_diagnostic(path):
    """Export task geometry alone. This artifact is NOT a station render/physics result."""
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, '/ServiceFixture')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    for spec in L.FIXED:
        cube(stage, '/ServiceFixture/' + spec.name, spec.pos, spec.size, spec.color)
    moving = UsdGeom.Xform.Define(stage, '/ServiceFixture/slider')
    moving.AddTranslateOp().Set(L.HOME)
    UsdPhysics.RigidBodyAPI.Apply(moving.GetPrim())
    UsdPhysics.MassAPI.Apply(moving.GetPrim()).CreateMassAttr(L.MASS)
    cube(stage, '/ServiceFixture/slider/body', (0, 0, 0), L.SLIDER_SIZE, L.ORANGE)
    augment_slider(stage, '/ServiceFixture/slider')
    stage.GetRootLayer().Save()
    return stage
