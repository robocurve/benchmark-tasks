"""Author a passive cam mount, a free camera/plate, and a receiving pad as native USD."""

import hashlib
import math
from pathlib import Path
import struct


CAMERA_SHA256 = "8a9a84dd9c9a67687e5ad0b37a7ed334dbfea6b0f995fef28524cc8da5f21ed2"
CAM_PIVOT = (0.025, -0.030, 0.017)


def create_assets(directory):
    """Write three USDs after Kit starts; all task motion remains native PhysX motion."""
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)

    def stage_at(filename, root_name):
        path = directory / filename
        stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, f"/{root_name}")
        stage.SetDefaultPrim(root.GetPrim())
        material = UsdShade.Material.Define(stage, f"/{root_name}/ContactMaterial")
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(0.6)
        physics.CreateDynamicFrictionAttr(0.5)
        physics.CreateRestitutionAttr(0.0)
        return stage, root, material, path

    def body(stage, path, mass, center, inertia, position=(0, 0, 0)):
        node = UsdGeom.Xform.Define(stage, path)
        node.AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdPhysics.RigidBodyAPI.Apply(node.GetPrim())
        props = UsdPhysics.MassAPI.Apply(node.GetPrim())
        props.CreateMassAttr(mass)
        props.CreateCenterOfMassAttr(Gf.Vec3f(*center))
        props.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        return node

    def shape(stage, path, material, kind, size, position, color, collision=True):
        if kind == "box":
            geom = UsdGeom.Cube.Define(stage, path)
            geom.CreateSizeAttr(1.0)
        elif kind == "cylinder":
            geom = UsdGeom.Cylinder.Define(stage, path)
            geom.CreateRadiusAttr(size[0])
            geom.CreateHeightAttr(2 * size[1])
            geom.CreateAxisAttr(UsdGeom.Tokens.z)
        else:
            raise ValueError(f"Unknown camera fixture shape {kind}")
        geom.AddTranslateOp().Set(Gf.Vec3d(*position))
        if kind == "box":
            geom.AddScaleOp().Set(Gf.Vec3f(*[2 * value for value in size]))
        geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            contacts = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
            contacts.CreateContactOffsetAttr(0.00005)
            contacts.CreateRestOffsetAttr(0.0)
            UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material, materialPurpose="physics")
        return geom

    stage, root, material, mount_path = stage_at("mount.usda", "CameraMount")
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    solver = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    solver.CreateEnabledSelfCollisionsAttr(False)
    solver.CreateSolverPositionIterationCountAttr(32)
    solver.CreateSolverVelocityIterationCountAttr(8)
    body(stage, "/CameraMount/Stand", 0.4, (0, 0, 0), (0.001, 0.001, 0.001))
    stand_boxes = (
        ("Foot", (.055, .055, .004), (0, 0, -.016), (.22, .23, .24)),
        ("Post", (.013, .013, .004), (0, 0, -.008), (.55, .57, .60)),
        ("Adapter", (.035, .026, .0045), (0, 0, .0045), (.13, .14, .15)),
        ("RearRail", (.027, .003, .003), (0, .023, .012), (.13, .14, .15)),
        ("RearLip", (.027, .005, .0015), (0, .021, .0165), (.13, .14, .15)),
        ("LeftGuide", (.002, .019, .002), (-.031, 0, .011), (.32, .34, .36)),
        ("RightGuide", (.002, .019, .002), (.031, 0, .011), (.32, .34, .36)),
    )
    for name, size, position, color in stand_boxes:
        shape(stage, f"/CameraMount/Stand/{name}", material, "box", size, position, color)
    for index, x in enumerate((-.045, .045)):
        shape(stage, f"/CameraMount/Stand/MountScrew{index}", material, "cylinder", (.003, .001),
              (x, 0, -.011), (.6, .62, .65))
    body(stage, "/CameraMount/Cam", .018, (-.01, -.01, 0), (.000004, .000004, .000006), CAM_PIVOT)
    for name, size, position in (
        ("Lever", (.012, .007, .0035), (-.020, -.018, 0)),
        ("Bridge", (.011, .004, .0025), (-.009, -.0081, 0)),
        ("Toe", (.014, .004, .002), (-.018, .010, 0)),
    ):
        color = (.9, .3, .04) if name == "Lever" else (.13, .14, .15)
        shape(stage, f"/CameraMount/Cam/{name}", material, "box", size, position, color)
    shape(stage, "/CameraMount/Cam/Pivot", material, "cylinder", (.004, .004), (0, 0, 0), (.5, .52, .55))
    fixed = UsdPhysics.FixedJoint.Define(stage, "/CameraMount/base_fixed")
    fixed.CreateBody1Rel().SetTargets(["/CameraMount/Stand"])
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/CameraMount/cam_angle")
    joint.CreateBody0Rel().SetTargets(["/CameraMount/Stand"])
    joint.CreateBody1Rel().SetTargets(["/CameraMount/Cam"])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(*CAM_PIVOT))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(math.degrees(.95))
    joint.CreateCollisionEnabledAttr(False)
    damping = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    damping.CreateTypeAttr("force")
    damping.CreateStiffnessAttr(0.0)
    damping.CreateDampingAttr(.001 * math.pi / 180)
    PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(.15)
    stage.GetRootLayer().Save()

    stage, root, material, payload_path = stage_at("payload.usda", "CameraPayload")
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(root.GetPrim())
    mass.CreateMassAttr(.083)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, .029))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.000025, .000025, .000020))
    shape(stage, "/CameraPayload/Plate", material, "box", (.025, .019, .0025), (0, 0, .0025), (.32, .34, .36))
    shell = shape(stage, "/CameraPayload/CameraHull", material, "box", (.0115, .021, .021),
                  (0, 0, .033), (.12, .13, .14))
    shell.MakeInvisible()
    shape(stage, "/CameraPayload/MountSpacer", material, "cylinder", (.006, .0035),
          (0, 0, .0085), (.6, .62, .65), collision=False)
    stl = Path(__file__).with_name("assets") / "d405.stl"
    contents = stl.read_bytes()
    if hashlib.sha256(contents).hexdigest() != CAMERA_SHA256:
        raise ValueError("D405 source mesh differs from its licensed source record")
    triangles = struct.unpack_from("<I", contents, 80)[0]
    if len(contents) != 84 + 50 * triangles:
        raise ValueError("D405 STL has an invalid binary triangle table")
    vertices = [struct.unpack_from("<3f", contents, 84 + 50 * i + 12 + 12 * j)
                for i in range(triangles) for j in range(3)]
    visual = UsdGeom.Mesh.Define(stage, "/CameraPayload/D405Visual")
    visual.CreatePointsAttr([Gf.Vec3f(*point) for point in vertices])
    visual.CreateFaceVertexCountsAttr([3] * triangles)
    visual.CreateFaceVertexIndicesAttr(list(range(3 * triangles)))
    visual.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    visual.CreateDisplayColorAttr([Gf.Vec3f(.12, .13, .14)])
    visual.AddTranslateOp().Set(Gf.Vec3d(.0115, 0, .033))
    visual.AddOrientOp().Set(Gf.Quatf(math.sqrt(.5), Gf.Vec3f(0, math.sqrt(.5), 0)))
    stage.GetRootLayer().Save()

    stage, root, material, pad_path = stage_at("pad.usda", "ReceivingPad")
    shape(stage, "/ReceivingPad/Surface", material, "box", (.065, .055, .002), (0, 0, 0), (.1, .6, .25))
    stage.GetRootLayer().Save()
    return mount_path, payload_path, pad_path
