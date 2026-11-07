"""Author the actual passive ratchet and rigid inert cartridges in native USD.

All mechanism motion comes from joints, gravity, spring torque and contact.
No task-body motion is prescribed during an episode.
"""
from pathlib import Path
import json
import math


def create_assets(directory):
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    inventory = []

    def stage_at(filename, name):
        stage = Usd.Stage.CreateNew(str(directory / filename))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, '/' + name)
        stage.SetDefaultPrim(root.GetPrim())
        mat = UsdShade.Material.Define(stage, '/' + name + '/ContactMaterial')
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(.65)
        api.CreateDynamicFrictionAttr(.5)
        api.CreateRestitutionAttr(0)
        return stage, mat

    def body(stage, path, mass, position=(0, 0, 0), inertia=(.001, .001, .001)):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        m = UsdPhysics.MassAPI.Apply(x.GetPrim())
        m.CreateMassAttr(mass)
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        inventory.append(dict(asset=Path(stage.GetRootLayer().identifier).stem, body=path, mass_kg=mass, initial_local_position_m=list(position)))
        return x

    def solid(stage, mat, path, half, pos, color, orientation=None):
        box = UsdGeom.Cube.Define(stage, path)
        box.CreateSizeAttr(1)
        box.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if orientation is not None:
            box.AddOrientOp().Set(Gf.Quatf(math.cos(orientation / 2), Gf.Vec3f(0, math.sin(orientation / 2), 0)))
        box.AddScaleOp().Set(Gf.Vec3f(*[2*v for v in half]))
        box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(stage, mat, box.GetPrim())
        inventory.append(dict(asset=Path(stage.GetRootLayer().identifier).stem, solid=path, kind='box', half_extents_m=list(half), center_m=list(pos),
                              rotation_about_y_rad=orientation or 0))
        return box

    def contact(stage, mat, prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        p.CreateContactOffsetAttr(.0002)
        p.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose='physics')

    def cylinder(stage, mat, path, radius, length, pos, color):
        x = UsdGeom.Cylinder.Define(stage, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(length)
        x.CreateAxisAttr('Y')
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(stage, mat, x.GetPrim())
        inventory.append(dict(asset=Path(stage.GetRootLayer().identifier).stem, solid=path, kind='cylinder', radius_m=radius, length_m=length,
                              axis='Y', center_m=list(pos)))

    def tooth(stage, mat, path, y):
        # Ramp on +Y, vertical retaining face on -Y: advance +Y, prevent retraction -Y.
        points = [(x, yy, z) for x in (-.003, .003)
                  for yy, z in ((y-.003, -.004), (y-.003, -.008), (y+.003, -.004))]
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr([3, 3, 4, 4, 4])
        mesh.CreateFaceVertexIndicesAttr([0, 2, 1, 3, 4, 5, 0, 1, 4, 3, 1, 2, 5, 4, 2, 0, 3, 5])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.48, .5, .52)])
        contact(stage, mat, mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        inventory.append(dict(asset=Path(stage.GetRootLayer().identifier).stem, solid=path, kind='convex_prism', vertices_m=points))

    def joint(stage, name, cls, parent, child, anchor, axis=None, limits=None, spring=None):
        j = cls.Define(stage, '/Dispenser/' + name)
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([child])
        j.CreateLocalPos0Attr(Gf.Vec3f(*anchor))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateCollisionEnabledAttr(False)
        if axis:
            j.CreateAxisAttr(axis)
        if limits:
            j.CreateLowerLimitAttr(limits[0]); j.CreateUpperLimitAttr(limits[1])
        if spring:
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), 'angular')
            d.CreateTypeAttr('force')
            d.CreateTargetPositionAttr(0)
            d.CreateStiffnessAttr(spring[0] * math.pi / 180)
            d.CreateDampingAttr(spring[1] * math.pi / 180)
            d.CreateMaxForceAttr(.05)
        inventory.append(dict(asset="dispenser", joint=name, parent=parent, child=child,
                              parent_anchor_m=list(anchor), axis=axis, limits=limits, spring=spring))
        return j

    stage, mat = stage_at('dispenser.usda', 'Dispenser')
    root = stage.GetDefaultPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    api = PhysxSchema.PhysxArticulationAPI.Apply(root)
    api.CreateEnabledSelfCollisionsAttr(True)
    api.CreateSolverPositionIterationCountAttr(32)
    api.CreateSolverVelocityIterationCountAttr(8)
    body(stage, '/Dispenser/Frame', .42, inertia=(.002, .0005, .002))
    grey, dark = (.35, .38, .42), (.12, .14, .17)
    # Open-top frame, unobstructed cartridge loading. Box half sizes throughout.
    for name, half, pos in [
        ('LeftRail', (.004, .108, .004), (-.027, .108, -.026)),
        ('RightRail', (.004, .108, .004), (.027, .108, -.026)),
        ('RearLower', (.031, .006, .009), (0, -.006, -.019)),
        ('RearLeft', (.011, .006, .012), (-.020, -.006, .002)),
        ('RearRight', (.011, .006, .012), (.020, -.006, .002)),
        ('RearTop', (.031, .006, .004), (0, -.006, .018)),
        ('FrontLeft', (.011, .006, .026), (-.020, .220, 0)),
        ('FrontRight', (.011, .006, .026), (.020, .220, 0)),
        ('FrontBottom', (.031, .006, .006), (0, .220, -.023)),
        ('HandleBridge', (.013, .021, .012), (0, -.015, -.034)),
        ('StationaryHandle', (.012, .015, .0425), (0, -.025, -.0725)),
    ]:
        solid(stage, mat, '/Dispenser/Frame/' + name, half, pos, grey if 'Handle' not in name else dark)
    # The refill rests on the dispenser, with its axis aligned to the piston.
    # Undersides are flush with the rails; tops are one refill radius below
    # the rod axis. These bridges also separate the refill from the cradle.
    for n, y in enumerate((.055, .180)):
        solid(stage, mat, f'/Dispenser/Frame/RefillSupport{n}', (.025, .009, .00275),
              (0, y, -.02725), grey)
    # Reusable outlet belongs to the tool. It stays entirely ahead of the
    # loading bay, so the refill can be lowered vertically as a plain cylinder.
    solid(stage, mat, '/Dispenser/Frame/OutletBridge', (.011, .006, .004),
          (0, .220, -.012), grey)
    cylinder(stage, mat, '/Dispenser/Frame/OutletSeat', .008, .012,
             (0, .220, 0), dark)
    cylinder(stage, mat, '/Dispenser/Frame/Nozzle', .006, .029,
             (0, .2405, 0), (.85, .85, .8))
    # Carrier is a mathematical intermediate for two native single-DOF joints, with no collider.
    body(stage, '/Dispenser/Carrier', .001, inertia=(.000001,)*3)
    joint(stage, 'rod_slide', UsdPhysics.PrismaticJoint, '/Dispenser/Frame', '/Dispenser/Carrier',
          (0, 0, 0), 'Y', (-.210, .015))
    body(stage, '/Dispenser/Rod', .065, inertia=(.0005, .00002, .0005))
    cylinder(stage, mat, '/Dispenser/Rod/Shaft', .004, .264, (0, .060, 0), (.55, .57, .6))
    cylinder(stage, mat, '/Dispenser/Rod/Piston', .019, .006, (0, .192, 0), dark)
    solid(stage, mat, '/Dispenser/Rod/THandle', (.027, .008, .008), (0, -.070, 0), (.9, .7, .05))
    for n in range(31):
        tooth(stage, mat, f'/Dispenser/Rod/Tooth{n:02}', -.055+n*.008)
    j = joint(stage, 'rod_rotation', UsdPhysics.RevoluteJoint, '/Dispenser/Carrier', '/Dispenser/Rod',
              (0, 0, 0), 'Y')
    PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(.003)
    body(stage, '/Dispenser/Pawl', .008, (0, -.035, -.016), inertia=(.000001,)*3)
    solid(stage, mat, '/Dispenser/Pawl/Lever', (.006, .0115, .002), (0, .0115, .002), (.7, .35, .08))
    solid(stage, mat, '/Dispenser/Pawl/Toe', (.006, .001, .002), (0, .023, .008), (.7, .35, .08))
    joint(stage, 'pawl_angle', UsdPhysics.RevoluteJoint, '/Dispenser/Frame', '/Dispenser/Pawl',
          (0, -.035, -.016), 'X', (-35, 0), spring=(.025, .002))
    stage.GetRootLayer().Save()

    # Rigid, inert practice cartridges. The prepared one's back plate represents an immobile
    # full-cartridge piston; no extrusion, fluid pressure, foil or deformable tube is simulated.
    for kind, color, back in [('used', (.9, .3, .04), False), ('replacement', (.07, .35, .85), True)]:
        stage, mat = stage_at(kind+'.usda', 'Cartridge')
        body(stage, '/Cartridge/Body', .045 if not back else .16, inertia=(.0006, .00005, .0006))
        for n in range(20):
            angle = n*math.tau/20
            solid(stage, mat, f'/Cartridge/Body/Wall{n:02}', (.0015, .100, .0037),
                  (.023*math.cos(angle), 0, .023*math.sin(angle)), color, -angle)
        # Flush front face: neither refill has a projecting tip or end lip.
        cylinder(stage, mat, '/Cartridge/Body/Front', .024, .004, (0, .098, 0), color)
        if back:
            cylinder(stage, mat, '/Cartridge/Body/RearPiston', .0215, .003, (0, -.097, 0), dark)
        stage.GetRootLayer().Save()

    stage, mat = stage_at('fixtures.usda', 'Fixtures')
    top = .7475
    # Authored in world coordinates; frame's lower rails sit on these two cradle bars.
    for n, y in enumerate((.025, .150)):
        solid(stage, mat, f'/Fixtures/CradleBar{n}', (.040, .009, .006), (.32, y, .844), dark)
        for sign in (-1, 1):
            solid(stage, mat, f'/Fixtures/Post{n}_{sign+1}', (.006, .009, .04525),
                  (.32+sign*.034, y, top+.04525), grey)
            # Visible locating edges, 1 mm lateral clearance from each frame rail.
            # The tool remains free to lift out; nothing attaches it to the cradle.
            solid(stage, mat, f'/Fixtures/LocatingEdge{n}_{sign+1}', (.004, .012, .009),
                  (.32+sign*.036, y, .853), dark)
    solid(stage, mat, '/Fixtures/CradleFoot', (.056, .105, .003), (.32, .0875, top+.003), dark)
    # Spare horizontal cartridge lies directly on the table, between shallow anti-roll rails.
    for sign in (-1, 1):
        solid(stage, mat, f'/Fixtures/SpareGuide{sign+1}', (.004, .110, .006),
              (.18+sign*.030, .40, top+.006), (.2, .4, .65))
    solid(stage, mat, '/Fixtures/TrayFloor', (.055, .140, .002), (.43, .40, top+.002), (.2, .55, .28))
    for sign in (-1, 1):
        solid(stage, mat, f'/Fixtures/TraySide{sign+1}', (.004, .140, .008),
              (.43+sign*.055, .40, top+.012), (.2, .55, .28))
        solid(stage, mat, f'/Fixtures/TrayEnd{sign+1}', (.059, .004, .008),
              (.43, .40+sign*.140, top+.012), (.2, .55, .28))
    stage.GetRootLayer().Save()
    (directory/'geometry-inventory.json').write_text(json.dumps(inventory, indent=2)+'\n')
    return {name: directory/(name+'.usda') for name in ['dispenser', 'used', 'replacement', 'fixtures']}
