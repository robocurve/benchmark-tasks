"""Countertop microwave with an open door, leftovers dish, and steel fork inside."""
import math
import numpy as np
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
    for name, static, dynamic in [('contact', .70, .60), ('pad_contact', 1.2, 1.), ('fork_contact', 2.0, 1.8)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(0.)

    white = (.90, .91, .93)
    grey = (.45, .48, .52)
    steel = (.62, .64, .66)
    food = (.55, .32, .18)
    mark = (.20, .75, .35)

    ox, oy, oz = spec.MW_ORIGIN
    ox_sz, oy_sz, oz_sz = spec.MW_OUTER
    cx, cy, cz = spec.CAVITY

    box(stage, '/World/floor', (1.8, 1.5, .06), (.40, 0, -.03), (.12, .13, .15))
    box(stage, '/World/table', (1.10, 1.30, .04), (.40, 0, spec.TABLE_Z - .02), (.62, .52, .38))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))

    # Microwave shell: kinematic rigid so door/button joints have a real body0.
    # Front opening faces -X (toward the arms).
    shell = root(stage, '/World/microwave', (ox, oy, oz + oz_sz / 2), mass=25.)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath('/World/microwave')).CreateKinematicEnabledAttr(True)
    PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath('/World/microwave')).CreateEnableCCDAttr(False)
    wall_t = .018
    box(stage, '/World/microwave/back', (wall_t, oy_sz, oz_sz),
        (ox_sz / 2 - wall_t / 2, 0, 0), white)
    box(stage, '/World/microwave/top', (ox_sz, oy_sz, wall_t),
        (0, 0, oz_sz / 2 - wall_t / 2), white)
    box(stage, '/World/microwave/bottom', (ox_sz, oy_sz, wall_t),
        (0, 0, -oz_sz / 2 + wall_t / 2), white)
    # Side walls stop short of the front lip so apron grasps clear the cheeks.
    side_depth = ox_sz - .100
    box(stage, '/World/microwave/side_l', (side_depth, wall_t, oz_sz),
        (.050, oy_sz / 2 - wall_t / 2, 0), white)
    box(stage, '/World/microwave/side_r', (side_depth, wall_t, oz_sz),
        (.050, -oy_sz / 2 + wall_t / 2, 0), white)
    # Control panel on the -Y cheek (away from the left-arm apron approach).
    box(stage, '/World/microwave/panel', (.040, .080, .180),
        (-ox_sz / 2 + .02, -oy_sz / 2 - .02, 0), grey)

    # Hinged door on the -X front face; hinge on the +Y stile, open swings toward -Y.
    hinge_world = (ox - ox_sz / 2, oy + oy_sz / 2 - wall_t, oz + oz_sz / 2)
    door = root(stage, '/World/door', hinge_world, .35, yaw=spec.DOOR_OPEN_YAW)
    # Visual-only open door (no collision): an open door must not block apron grasps.
    box(stage, '/World/door/panel', (spec.DOOR_THICK, oy_sz - 2 * wall_t, oz_sz - 2 * wall_t),
        (0., -(oy_sz / 2 - wall_t), 0.), (.75, .78, .82), collision=False)
    accent = (.18, .42, .72)
    box(stage, '/World/door/handle', (.018, .014, .080),
        (-spec.DOOR_THICK / 2 - .01, -(oy_sz - 2 * wall_t) + .04, 0.), accent, collision=False)
    j_door = joint(stage, 'door_hinge', UsdPhysics.RevoluteJoint, '/World/microwave', '/World/door',
                   (-ox_sz / 2, oy_sz / 2 - wall_t, 0.), (0., 0., 0.), 'Z')
    # Negative open angle: lower < upper in degrees.
    j_door.CreateLowerLimitAttr(math.degrees(spec.DOOR_OPEN_YAW) - 3.)
    j_door.CreateUpperLimitAttr(0.)
    dd = UsdPhysics.DriveAPI.Apply(j_door.GetPrim(), 'angular')
    dd.CreateTypeAttr('force')
    dd.CreateStiffnessAttr(.5)
    dd.CreateDampingAttr(.05)
    dd.CreateTargetPositionAttr(math.degrees(spec.DOOR_OPEN_YAW))
    dd.CreateMaxForceAttr(.8)

    # Latching start button on the front face (toward the arms / -X), slightly to -Y.
    # Pressing moves the button +X into the shell (travel = button_x - rest_x).
    # Large protruding start button: easy to find with a closed-pad push along +X.
    btn = root(stage, '/World/start_button', spec.BUTTON_POS, .04)
    box(stage, '/World/start_button/cap', (.040, .045, .030), (0, 0, 0), (.85, .20, .18))
    j_btn = joint(stage, 'start_button', UsdPhysics.PrismaticJoint, '/World/microwave', '/World/start_button',
                  (-ox_sz / 2 + .02, -0.050, -oz_sz / 2 + .12), (0., 0., 0.), 'X')
    j_btn.CreateLowerLimitAttr(0.)
    j_btn.CreateUpperLimitAttr(spec.BUTTON_TRAVEL)
    bd = UsdPhysics.DriveAPI.Apply(j_btn.GetPrim(), 'linear')
    bd.CreateTypeAttr('force')
    # Near-zero return spring so a pad push latches the short travel.
    bd.CreateStiffnessAttr(1.5)
    bd.CreateDampingAttr(.4)
    bd.CreateTargetPositionAttr(0.)
    bd.CreateMaxForceAttr(.4)

    # Narrow post + thin wide top: wrist clears the post; utensil won't tip off.
    for name, xy in [('apron_stand', (spec.DISH_START[0], spec.DISH_START[1])),
                     ('park_stand', (spec.FORK_SAFE[0], spec.FORK_SAFE[1]))]:
        box(stage, f'/World/{name}', (.035, .035, spec.STAND_H - .012),
            (xy[0], xy[1], spec.TABLE_Z + (spec.STAND_H - .012) / 2), grey)
        box(stage, f'/World/{name}_top', (.120, .080, .012),
            (xy[0], xy[1], spec.STAND_TOP - .006), grey)
    # Thin dish plate under the fork: smaller than the fork length so pads clear it.
    dish = box(stage, '/World/dish', (spec.DISH_OD * .45, spec.DISH_OD * .45, .008),
               (spec.DISH_START[0], spec.DISH_START[1], spec.STAND_TOP + .004), (.95, .95, .92),
               mass=spec.DISH_MASS)
    box(stage, '/World/dish/food', (.040, .040, .012), (0, 0, .010), food, collision=False)

    # Steel fork on the dish (must leave the metal keep-out zone before start).
    fork = box(stage, '/World/fork', spec.FORK_SIZE, spec.FORK_START, steel, mass=spec.FORK_MASS)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/World/fork/shape')).Bind(
        UsdShade.Material(stage.GetPrimAtPath('/World/fork_contact')), materialPurpose='physics')

    # Safe parking mark on the park stand (visual + scoring target).
    box(stage, '/World/fork_safe_mark', (.050, .050, .004),
        (spec.FORK_SAFE[0], spec.FORK_SAFE[1], spec.STAND_TOP + .002), mark, collision=False)

    return {
        'microwave': '/World/microwave',
        'door': '/World/door',
        'start_button': '/World/start_button',
        'dish': '/World/dish',
        'fork': '/World/fork',
        'fork_safe_mark': '/World/fork_safe_mark',
        'table': '/World/table',
        'floor': '/World/floor',
    }
