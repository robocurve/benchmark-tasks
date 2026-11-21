"""Installed pressure-mounted walk-through child safety gate with dual locks."""
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
        api.CreateMaxDepenetrationVelocityAttr(.2)
        api.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r


def box(stage, path, size, pos, color, mass=None, yaw=0., collision=True, mat=None):
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
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat or material(stage, path + '/look', color))
    return r


def joint(stage, name, kind, b0, b1, p0, p1, axis='Z', angle0=0.):
    j = kind.Define(stage, '/World/joints/' + name)
    j.CreateBody0Rel().SetTargets([b0] if b0 else [])
    j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*p0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    j.CreateLocalRot0Attr(Gf.Quatf(math.cos(angle0 / 2), 0, 0, math.sin(angle0 / 2)))
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
    for name, static, dynamic in [('contact', .65, .55), ('pad_contact', 1.2, 1.)]:
        m = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, '/World/' + name).GetPrim())
        m.CreateStaticFrictionAttr(static)
        m.CreateDynamicFrictionAttr(dynamic)
        m.CreateRestitutionAttr(0.)

    white = (.86, .88, .90)
    grey = (.55, .58, .62)
    accent = (.18, .42, .72)

    box(stage, '/World/floor', (1.8, 1.5, .06), (.40, 0, -.03), (.12, .13, .15))
    for side, sign in [('left', 1), ('right', -1)]:
        box(stage, '/World/pedestal_' + side, (.15, .15, .750), (0, sign * .305, .375), (.35, .39, .42))

    # Hallway walls / fixed pressure-mounted frame (already installed; not movable).
    half = spec.OPENING_WIDTH / 2
    box(stage, '/World/wall_left', (.080, .120, .900),
        (spec.HINGE_X, half + .060, .450), (.72, .70, .66))
    box(stage, '/World/wall_right', (.080, .120, .900),
        (spec.HINGE_X, -half - .060, .450), (.72, .70, .66))
    # Fixed frame uprights pressed into the opening.
    # Hinge upright is a kinematic rigid so the revolute joint has a real body0.
    box(stage, '/World/frame_hinge', (spec.FRAME_TUBE, spec.FRAME_TUBE, spec.GATE_HEIGHT),
        (spec.HINGE_X, half - spec.FRAME_TUBE, spec.GATE_HEIGHT / 2), white, mass=20.)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath('/World/frame_hinge')).CreateKinematicEnabledAttr(True)
    # Kinematic + CCD is unsupported; keep hinge upright as a plain kinematic anchor.
    PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath('/World/frame_hinge')).CreateEnableCCDAttr(False)
    box(stage, '/World/frame_latch', (spec.FRAME_TUBE, spec.FRAME_TUBE, spec.GATE_HEIGHT),
        (spec.HINGE_X, -half + spec.FRAME_TUBE, spec.GATE_HEIGHT / 2), white)
    # Fixed half-panel + centre stile on the child side (−X) so the spring latch,
    # which extends toward +X, does not jam the free stile short of yaw=0.
    free_stile_y = (half - spec.FRAME_TUBE) - spec.DOOR_WIDTH + spec.FRAME_TUBE
    centre_y = free_stile_y - spec.FRAME_TUBE - .006
    panel_x = spec.HINGE_X - .014
    box(stage, '/World/frame_centre', (spec.FRAME_TUBE, spec.FRAME_TUBE, spec.GATE_HEIGHT),
        (panel_x, centre_y, spec.GATE_HEIGHT / 2), white)
    fixed_panel_h = abs((-half + spec.FRAME_TUBE) - centre_y) - spec.FRAME_TUBE
    fixed_panel_y = (centre_y + (-half + spec.FRAME_TUBE)) / 2
    box(stage, '/World/fixed_panel', (spec.FRAME_TUBE, max(.040, fixed_panel_h), spec.GATE_HEIGHT - 2 * spec.FRAME_TUBE),
        (panel_x, fixed_panel_y, spec.GATE_HEIGHT / 2), white)
    # Receiving catch: visual + scoring volume only (no collision). Wide mouth so the
    # extended bolt overlaps once the door is inside the closed band (~5°), not only at 0°.
    catch_y = free_stile_y - spec.FRAME_TUBE / 2
    box(stage, '/World/catch', (.030, .016, .028),
        (spec.HINGE_X + .020, catch_y, spec.GATE_HEIGHT - .055), accent, collision=False)

    # Walk-through door: hinged rigid body with vertical bars and rectangular handle.
    door_yaw = spec.DOOR_START_YAW
    hinge_pos = (spec.HINGE_X, half - spec.FRAME_TUBE, 0.)
    door = root(stage, '/World/door', hinge_pos, spec.DOOR_MASS, yaw=door_yaw)
    # Local door geometry: hinge at origin; door extends in -Y when closed (yaw=0).
    box(stage, '/World/door/stile_hinge', (spec.FRAME_TUBE, spec.FRAME_TUBE, spec.GATE_HEIGHT),
        (0., 0., spec.GATE_HEIGHT / 2), white)
    box(stage, '/World/door/stile_free', (spec.FRAME_TUBE, spec.FRAME_TUBE, spec.GATE_HEIGHT),
        (0., -spec.DOOR_WIDTH + spec.FRAME_TUBE, spec.GATE_HEIGHT / 2), white)
    box(stage, '/World/door/rail_top', (spec.FRAME_TUBE, spec.DOOR_WIDTH, spec.FRAME_TUBE),
        (0., -spec.DOOR_WIDTH / 2 + spec.FRAME_TUBE / 2, spec.GATE_HEIGHT - spec.FRAME_TUBE / 2), white)
    box(stage, '/World/door/rail_bottom', (spec.FRAME_TUBE, spec.DOOR_WIDTH, spec.FRAME_TUBE),
        (0., -spec.DOOR_WIDTH / 2 + spec.FRAME_TUBE / 2, spec.FRAME_TUBE / 2), white)
    n_bars = max(3, int((spec.DOOR_WIDTH - 2 * spec.FRAME_TUBE) / spec.BAR_GAP) - 1)
    for i in range(n_bars):
        y = -spec.FRAME_TUBE - (i + 1) * (spec.DOOR_WIDTH - 2 * spec.FRAME_TUBE) / (n_bars + 1)
        box(stage, f'/World/door/bar_{i}', (.010, .010, spec.GATE_HEIGHT - 2 * spec.FRAME_TUBE),
            (0., y, spec.GATE_HEIGHT / 2), grey)
    # Rectangular grip preferred by the program notes.
    box(stage, '/World/door/handle', spec.HANDLE_SIZE, spec.HANDLE_LOCAL, accent)

    # Frame root is at mid-height; door root is at floor. Coincide joint frames in world.
    # Door body already carries DOOR_START_YAW, so the free revolute angle starts open.
    j_door = joint(stage, 'door_hinge', UsdPhysics.RevoluteJoint, '/World/frame_hinge', '/World/door',
                   (0., 0., 0.), (0., 0., spec.GATE_HEIGHT / 2), 'Z')
    # Revolute limits and angular drive targets are degrees in UsdPhysics.
    j_door.CreateLowerLimitAttr(-95.)
    j_door.CreateUpperLimitAttr(0.)
    d = UsdPhysics.DriveAPI.Apply(j_door.GetPrim(), 'angular')
    d.CreateTypeAttr('force')
    # Light damping so a mid-swing push can coast shut; authored pose holds a short hold.
    d.CreateStiffnessAttr(0.)
    d.CreateDampingAttr(0.35)
    d.CreateTargetPositionAttr(0.)
    d.CreateMaxForceAttr(0.)

    # Top lock lever (revolute about local Y on free stile).
    lever_pos_local = (0.0, -spec.DOOR_WIDTH + spec.FRAME_TUBE, spec.GATE_HEIGHT - .050)
    # Lever body as separate rigid attached by joint; start raised.
    # Parent in world with door's current pose so joint starts consistent.
    c, s = math.cos(door_yaw), math.sin(door_yaw)
    world_lever = (hinge_pos[0] + c * lever_pos_local[0] - s * lever_pos_local[1],
                   hinge_pos[1] + s * lever_pos_local[0] + c * lever_pos_local[1],
                   lever_pos_local[2])
    # Raised about door-local Y by LEVER_START, then yawed with the door.
    qy = Gf.Quatf(math.cos(spec.LEVER_START / 2), 0., math.sin(spec.LEVER_START / 2), 0.)
    qz = Gf.Quatf(math.cos(door_yaw / 2), 0., 0., math.sin(door_yaw / 2))
    lever = root(stage, '/World/lever', world_lever, spec.LEVER_MASS, quat=qz * qy)
    box(stage, '/World/lever/arm', (.016, .070, .014), (0., -.035, 0.), accent)
    box(stage, '/World/lever/pad', (.028, .022, .016), (0., -.070, 0.), (.12, .14, .18))
    j_lever = joint(stage, 'lever', UsdPhysics.RevoluteJoint, '/World/door', '/World/lever',
                    lever_pos_local, (0., 0., 0.), 'Y')
    # Orient joint so angle 0 is lever flat/down along door top.
    j_lever.CreateLowerLimitAttr(0.)
    j_lever.CreateUpperLimitAttr(70.)
    # Prefer locked (0°); starts raised via authored pose, then settles shut.
    ld = UsdPhysics.DriveAPI.Apply(j_lever.GetPrim(), 'angular')
    ld.CreateTypeAttr('force')
    ld.CreateStiffnessAttr(1.2)
    ld.CreateDampingAttr(.08)
    ld.CreateTargetPositionAttr(0.)
    ld.CreateMaxForceAttr(.35)

    # Spring latch bolt: prismatic along local +X from free stile; engages catch when extended.
    # Seat the joint outside the free-stile tube so the bolt cannot jam against its own stile.
    latch_local = (.014, -spec.DOOR_WIDTH + spec.FRAME_TUBE / 2, spec.GATE_HEIGHT - .055)
    world_latch = (hinge_pos[0] + c * latch_local[0] - s * latch_local[1],
                   hinge_pos[1] + s * latch_local[0] + c * latch_local[1],
                   latch_local[2])
    latch = root(stage, '/World/latch', world_latch, spec.SPRING_LATCH_MASS, yaw=door_yaw)
    box(stage, '/World/latch/bolt', (.022, .010, .016), (.011, 0., 0.), accent, collision=False)
    j_latch = joint(stage, 'latch_slide', UsdPhysics.PrismaticJoint, '/World/door', '/World/latch',
                    latch_local, (0., 0., 0.), 'X')
    j_latch.CreateLowerLimitAttr(0.)
    j_latch.CreateUpperLimitAttr(.020)
    # Spring toward extended; bolt clears the stile, then reaches the catch.
    sd = UsdPhysics.DriveAPI.Apply(j_latch.GetPrim(), 'linear')
    sd.CreateTypeAttr('force')
    sd.CreateStiffnessAttr(18.)
    sd.CreateDampingAttr(2.)
    sd.CreateTargetPositionAttr(.020)
    sd.CreateMaxForceAttr(4.)

    # Bottom base lock: rotate down to block the door swinging toward the child side.
    base_local = (0.0, -spec.DOOR_WIDTH / 2, .025)
    world_base = (hinge_pos[0] + c * base_local[0] - s * base_local[1],
                  hinge_pos[1] + s * base_local[0] + c * base_local[1],
                  base_local[2])
    qx = Gf.Quatf(math.cos(spec.BASE_START / 2), math.sin(spec.BASE_START / 2), 0., 0.)
    qz = Gf.Quatf(math.cos(door_yaw / 2), 0., 0., math.sin(door_yaw / 2))
    base = root(stage, '/World/base_lock', world_base, spec.BASE_LOCK_MASS, quat=qz * qx)
    # Raised slightly so an open door does not scrape the floor into closing.
    box(stage, '/World/base_lock/paddle', (.012, .055, .010), (0., .0, .012), grey)
    j_base = joint(stage, 'base_lock', UsdPhysics.RevoluteJoint, '/World/door', '/World/base_lock',
                   base_local, (0., 0., 0.), 'X')
    j_base.CreateLowerLimitAttr(0.)
    j_base.CreateUpperLimitAttr(90.)
    # Prefer locked (0°); starts horizontal via authored pose, then settles down.
    bd = UsdPhysics.DriveAPI.Apply(j_base.GetPrim(), 'angular')
    bd.CreateTypeAttr('force')
    bd.CreateStiffnessAttr(1.0)
    bd.CreateDampingAttr(.06)
    bd.CreateTargetPositionAttr(0.)
    bd.CreateMaxForceAttr(.3)

    # Soft floor stop so the locked base paddle can rest without penetrating.
    box(stage, '/World/base_stop', (.040, .040, .008),
        (spec.HINGE_X + .02, -half + .08, .004), (.20, .22, .24))

    return {
        'door': '/World/door',
        'lever': '/World/lever',
        'latch': '/World/latch',
        'base_lock': '/World/base_lock',
        'catch': '/World/catch',
        'frame_hinge': '/World/frame_hinge',
        'frame_latch': '/World/frame_latch',
        'wall_left': '/World/wall_left',
        'wall_right': '/World/wall_right',
        'base_stop': '/World/base_stop',
    }
