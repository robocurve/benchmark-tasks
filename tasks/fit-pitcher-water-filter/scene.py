"""Pitcher reservoir, filter cartridge beside it, and an open hinged lid."""
import math
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec


def material(stage, path, color):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + '/surface')
    s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.55)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), 'surface')
    return m


def root(stage, path, pos, mass=None, yaw=0., quat=None):
    r = UsdGeom.Xform.Define(stage, path)
    r.AddTranslateOp().Set(Gf.Vec3d(*pos))
    r.AddOrientOp().Set(quat or Gf.Quatf(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim())
        UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim())
        api.CreateSolverPositionIterationCountAttr(48)
        api.CreateSolverVelocityIterationCountAttr(0)
        api.CreateMaxDepenetrationVelocityAttr(.15)
        api.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r


def box(stage, path, size, pos, color, mass=None, yaw=0., collision=True):
    r = root(stage, path, pos, mass, yaw)
    shape = UsdGeom.Cube.Define(stage, path + '/shape')
    shape.CreateSizeAttr(1.)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        p = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        p.CreateContactOffsetAttr(.00005)
        p.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(
            UsdShade.Material(stage.GetPrimAtPath('/World/contact')), materialPurpose='physics')
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material(stage, path + '/look', color))
    return r


def joint(stage, name, kind, b0, b1, p0, p1, axis='Y'):
    j = kind.Define(stage, '/World/joints/' + name)
    j.CreateBody0Rel().SetTargets([b0] if b0 else [])
    j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*p0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    j.CreateLocalRot0Attr(Gf.Quatf(1.))
    j.CreateLocalRot1Attr(Gf.Quatf(1.))
    j.CreateCollisionEnabledAttr(False)
    j.CreateExcludeFromArticulationAttr(True)
    if hasattr(j, 'CreateAxisAttr'):
        j.CreateAxisAttr(axis)
    return j


def build(stage):
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    ph = UsdPhysics.Scene.Define(stage, '/World/physics')
    ph.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ph.CreateGravityMagnitudeAttr(9.81)
    api = PhysxSchema.PhysxSceneAPI.Apply(ph.GetPrim())
    api.CreateTimeStepsPerSecondAttr(spec.PHYSICS_HZ)
    api.CreateEnableGPUDynamicsAttr(False)
    api.CreateBroadphaseTypeAttr('MBP')
    api.CreateEnableCCDAttr(True)
    for name, static, dynamic in [('contact', .70, .60), ('pad_contact', 1.2, 1.),
                                  ('cart_contact', 2.0, 1.8)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(0.)

    clear = (.78, .88, .92)
    grey = (.45, .48, .52)
    accent = (.18, .55, .42)
    filter_c = (.55, .72, .85)

    box(stage, '/World/floor', (1.8, 1.5, .06), (.40, 0, -.03), (.12, .13, .15))
    box(stage, '/World/table', (1.10, 1.30, .04), (.40, 0, spec.TABLE_Z - .02), (.62, .52, .38))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))
    # Raised stands so cartridge and pitcher sit in the reachable TCP band.
    for name, xy in [('pitcher_stand', (spec.PITCHER_START[0], spec.PITCHER_START[1])),
                     ('cart_stand', (spec.CART_START[0], spec.CART_START[1]))]:
        box(stage, f'/World/{name}', (.10, .10, spec.STAND_H),
            (xy[0], xy[1], spec.TABLE_Z + spec.STAND_H / 2), grey)

    # Pitcher body: kinematic shell with well floor for the cartridge.
    pitcher = root(stage, '/World/pitcher', spec.PITCHER_START, mass=spec.PITCHER_MASS)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath('/World/pitcher')).CreateKinematicEnabledAttr(True)
    wall = (spec.PITCHER_OD - spec.PITCHER_ID) / 2
    box(stage, '/World/pitcher/wall_n', (spec.PITCHER_OD, wall, spec.PITCHER_H),
        (0, spec.PITCHER_OD / 2 - wall / 2, 0), clear)
    box(stage, '/World/pitcher/wall_s', (spec.PITCHER_OD, wall, spec.PITCHER_H),
        (0, -spec.PITCHER_OD / 2 + wall / 2, 0), clear)
    box(stage, '/World/pitcher/wall_e', (wall, spec.PITCHER_ID, spec.PITCHER_H),
        (spec.PITCHER_OD / 2 - wall / 2, 0, 0), clear)
    box(stage, '/World/pitcher/wall_w', (wall, spec.PITCHER_ID, spec.PITCHER_H),
        (-spec.PITCHER_OD / 2 + wall / 2, 0, 0), clear)
    box(stage, '/World/pitcher/floor', (spec.PITCHER_ID, spec.PITCHER_ID, .008),
        (0, 0, -spec.PITCHER_H / 2 + .004), clear)
    # Upper well floor where the cartridge seats.
    box(stage, '/World/pitcher/well', (spec.PITCHER_ID - .010, spec.PITCHER_ID - .010, .006),
        (0, 0, spec.WELL_Z_LOCAL), grey)

    # Hinged lid, open toward the arms (+X).
    lid_world = (spec.PITCHER_START[0], spec.PITCHER_START[1],
                 spec.PITCHER_START[2] + spec.LID_HINGE_LOCAL[2])
    qy = Gf.Quatf(math.cos(spec.LID_OPEN / 2), 0., math.sin(spec.LID_OPEN / 2), 0.)
    lid = root(stage, '/World/lid', lid_world, mass=spec.LID_MASS, quat=qy)
    box(stage, '/World/lid/panel', spec.LID_SIZE, (spec.LID_SIZE[0] / 2 - .01, 0, 0), accent)
    j_lid = joint(stage, 'lid_hinge', UsdPhysics.RevoluteJoint, '/World/pitcher', '/World/lid',
                  spec.LID_HINGE_LOCAL, (0., 0., 0.), 'Y')
    j_lid.CreateLowerLimitAttr(0.)
    j_lid.CreateUpperLimitAttr(math.degrees(spec.LID_OPEN) + 5.)
    ld = UsdPhysics.DriveAPI.Apply(j_lid.GetPrim(), 'angular')
    ld.CreateTypeAttr('force')
    # Moderate open bias: holds the lid clear of the well during insert, still pushable shut.
    ld.CreateStiffnessAttr(.35)
    ld.CreateDampingAttr(.05)
    ld.CreateTargetPositionAttr(math.degrees(spec.LID_OPEN))
    ld.CreateMaxForceAttr(.45)

    # Filter cartridge on the counter (must be seated in the well).
    box(stage, '/World/cartridge', (spec.CART_OD, spec.CART_OD, spec.CART_H),
        spec.CART_START, filter_c, mass=spec.CART_MASS)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/World/cartridge/shape')).Bind(
        UsdShade.Material(stage.GetPrimAtPath('/World/cart_contact')), materialPurpose='physics')
    box(stage, '/World/cartridge/band', (spec.CART_OD + .004, spec.CART_OD + .004, .012),
        (0, 0, spec.CART_H / 2 - .020), accent, collision=False)

    return {
        'pitcher': '/World/pitcher',
        'lid': '/World/lid',
        'cartridge': '/World/cartridge',
        'table': '/World/table',
        'floor': '/World/floor',
    }
