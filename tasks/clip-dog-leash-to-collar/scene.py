"""Collar D-ring on a stand and a leash bolt-snap with spring gate."""
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


def joint(stage, name, kind, b0, b1, p0, p1, axis='Z'):
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
    for name, static, dynamic in [('contact', 1.10, .95), ('pad_contact', 1.4, 1.2)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(0.)


    leather = (.42, .28, .16)
    steel = (.62, .64, .66)
    accent = (.18, .42, .72)
    nylon = (.20, .22, .28)

    box(stage, '/World/floor', (1.8, 1.5, .06), (.40, 0, -.03), (.12, .13, .15))
    box(stage, '/World/table', (1.10, 1.30, .04), (.40, 0, spec.TABLE_Z - .02), (.62, .52, .38))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))
    # Collar on a stand; leash body on a wide tall pedestal (handle cantilevers off).
    box(stage, '/World/collar_stand', (.08, .08, spec.STAND_H),
        (spec.COLLAR_START[0], spec.COLLAR_START[1], spec.TABLE_Z + spec.STAND_H / 2), (.45, .48, .52))
    box(stage, '/World/snap_pedestal', spec.SNAP_PEDESTAL_SIZE,
        (spec.SNAP_START[0], spec.SNAP_START[1],
         spec.TABLE_Z + spec.SNAP_PEDESTAL_H / 2), (.45, .48, .52))
    # Shallow nest lips so the snap body cannot slide off before the pinch.
    nest_z = spec.TABLE_Z + spec.SNAP_PEDESTAL_H + .006
    for name, xy, size in [
        ('nest_x0', (spec.SNAP_START[0] - .028, spec.SNAP_START[1]), (.012, .050, .012)),
        ('nest_x1', (spec.SNAP_START[0] + .028, spec.SNAP_START[1]), (.012, .050, .012)),
        ('nest_y0', (spec.SNAP_START[0], spec.SNAP_START[1] + .028), (.050, .012, .012)),
    ]:
        box(stage, '/World/' + name, size, (xy[0], xy[1], nest_z), (.40, .43, .46))


    # Collar stand + D-ring (kinematic).
    collar = root(stage, '/World/collar', spec.COLLAR_START, mass=1.5)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath('/World/collar')).CreateKinematicEnabledAttr(True)
    PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath('/World/collar')).CreateEnableCCDAttr(False)
    box(stage, '/World/collar/band', (.080, .014, .010), (0, 0, 0), leather)
    # Approximate D-ring as a square frame (inner opening ~2*DRING_INNER).
    r = spec.DRING_INNER
    t = .004
    z = spec.DRING_LOCAL[2]
    box(stage, '/World/collar/ring_top', (2 * r + 2 * t, t, t), (0, 0, z + r), steel)
    box(stage, '/World/collar/ring_bot', (2 * r + 2 * t, t, t), (0, 0, z - r), steel)
    box(stage, '/World/collar/ring_l', (t, t, 2 * r), (-r, 0, z), steel)
    box(stage, '/World/collar/ring_r', (t, t, 2 * r), (r, 0, z), steel)
    # Invisible scoring marker at ring centre.
    box(stage, '/World/dring_centre', (.006, .006, .006),
        (spec.COLLAR_START[0], spec.COLLAR_START[1], spec.COLLAR_START[2] + z), accent, collision=False)

    # Single rigid bolt-snap / leash-handle body (one collider, no child-only mesh).
    snap = box(stage, '/World/snap', spec.SNAP_SIZE, spec.SNAP_START, steel, mass=spec.SNAP_MASS)
    # Spring gate (prismatic along +Z on the snap); starts closed (0).
    gate_pos = (spec.SNAP_START[0], spec.SNAP_START[1], spec.SNAP_START[2] + spec.SNAP_SIZE[2] / 2)
    gate = root(stage, '/World/gate', gate_pos, mass=.008)
    box(stage, '/World/gate/bar', (.024, .016, .022), (0, 0, .011), accent)
    j_gate = joint(stage, 'snap_gate', UsdPhysics.PrismaticJoint, '/World/snap', '/World/gate',
                   (0., 0., spec.SNAP_SIZE[2] / 2), (0., 0., 0.), 'Z')
    j_gate.CreateLowerLimitAttr(0.)
    j_gate.CreateUpperLimitAttr(spec.GATE_TRAVEL)
    gd = UsdPhysics.DriveAPI.Apply(j_gate.GetPrim(), 'linear')
    gd.CreateTypeAttr('force')
    gd.CreateStiffnessAttr(25.)
    gd.CreateDampingAttr(2.)
    gd.CreateTargetPositionAttr(0.)
    gd.CreateMaxForceAttr(4.)

    return {
        'collar': '/World/collar',
        'snap': '/World/snap',
        'gate': '/World/gate',
        'dring_centre': '/World/dring_centre',
        'table': '/World/table',
        'floor': '/World/floor',
    }
