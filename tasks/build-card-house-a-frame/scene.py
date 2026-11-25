"""Two playing cards standing in wooden place-card stands on a green felt mat on a table."""
import math
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

def material(stage, path, color, roughness=.45, metallic=0.):
    m = UsdShade.Material.Define(stage, path)
    s = UsdShade.Shader.Define(stage, path + '/surface')
    s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    s.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), 'surface')
    return m


def look(stage, prim, path, color, roughness=.45, metallic=0.):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material(stage, path, color, roughness, metallic))


def root(stage, path, pos, mass=None, quat=None):
    r = UsdGeom.Xform.Define(stage, path)
    r.AddTranslateOp().Set(Gf.Vec3d(*pos))
    r.AddOrientOp().Set(quat or Gf.Quatf(1.))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim())
        UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim())
        api.CreateSolverPositionIterationCountAttr(32)
        api.CreateSolverVelocityIterationCountAttr(4)
        api.CreateMaxDepenetrationVelocityAttr(.2)
        api.CreateEnableCCDAttr(True)
        api.CreateSleepThresholdAttr(0.)
        api.CreateStabilizationThresholdAttr(0.)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r


def collide(stage, prim, contact='/World/contact', offset=.001, rest=0.):
    UsdPhysics.CollisionAPI.Apply(prim)
    p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    p.CreateContactOffsetAttr(offset)
    p.CreateRestOffsetAttr(rest)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material(stage.GetPrimAtPath(contact)), materialPurpose='physics')


def cube(stage, path, size, pos, color, collision=True, contact='/World/contact', offset=.001, roughness=.45, rest=0.):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        collide(stage, shape.GetPrim(), contact, offset, rest)
    look(stage, shape.GetPrim(), path + '_look', color, roughness)
    return shape


def box(stage, path, size, pos, color, collision=True, contact='/World/contact'):
    root(stage, path, pos)
    return cube(stage, path + '/shape', size, (0., 0., 0.), color, collision, contact)


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
    api.CreateBounceThresholdAttr(.05)
    for name, (static, dynamic) in [('contact', (.65, .55)), ('pad_contact', (1.2, 1.)), ('felt', spec.FELT_FRICTION),
                                    ('card', spec.CARD_FRICTION), ('wood', spec.WOOD_FRICTION)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(0.)

    box(stage, '/World/floor', (3.2, 2.8, .06), (.80, -.20, -.03), (.62, .60, .56))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))
    tx, ty = spec.TABLE_CENTRE
    sx, sy = spec.TABLE_SIZE
    box(stage, '/World/table', (sx, sy, spec.TABLE_TOP_Z), (tx, ty, spec.TABLE_TOP_Z / 2), (.78, .74, .68))
    mx, my = spec.MAT_CENTRE
    box(stage, '/World/mat', spec.MAT_SIZE, (mx, my, spec.TABLE_TOP_Z + spec.MAT_SIZE[2] / 2), (.05, .33, .16),
        contact='/World/felt')

    # Place-card stands: a base under the slot and two jaws either side of it.
    sx, sy, sz = spec.STAND_SIZE
    jaw = (sx - spec.SLOT_W) / 2
    for side, y in spec.STAND_Y.items():
        root(stage, f'/World/stand_{side}', (spec.STAND_X, y, spec.MAT_TOP_Z), mass=spec.STAND_MASS)
        walnut = (.36, .22, .13)
        cube(stage, f'/World/stand_{side}/base', (sx, sy, sz - spec.SLOT_DEPTH), (0., 0., (sz - spec.SLOT_DEPTH) / 2),
             walnut, contact='/World/wood')
        for k, s in enumerate((-1, 1)):
            cube(stage, f'/World/stand_{side}/jaw_{k}', (jaw, sy, spec.SLOT_DEPTH),
                 (s * (spec.SLOT_W + jaw) / 2, 0., sz - spec.SLOT_DEPTH / 2), walnut, contact='/World/wood')

    # Cards: white paper, red back printed on the side facing away from the arms (+x at the start).
    for side in spec.STAND_Y:
        pos, q = spec.start_pose(side)
        root(stage, f'/World/card_{side}', pos, spec.CARD_MASS, quat=Gf.Quatf(*q))
        cube(stage, f'/World/card_{side}/shape', (spec.CARD_T, spec.CARD_W, spec.CARD_H), (0., 0., 0.),
             (.95, .95, .93), contact='/World/card', offset=spec.CARD_CONTACT, roughness=.35, rest=spec.CARD_REST)
        cube(stage, f'/World/card_{side}/back', (.00002, spec.CARD_W - .006, spec.CARD_H - .006),
             (spec.CARD_T / 2 + .00002, 0., 0.), (.70, .08, .10), collision=False, roughness=.35)
    return {f'card_{s}': f'/World/card_{s}' for s in spec.STAND_Y}


FIXED = ['floor', 'pedestal_left', 'pedestal_right', 'table', 'mat']
