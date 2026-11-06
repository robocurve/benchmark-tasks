"""Build reusable USD assets and a local YAM task scene. All lengths are metres."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct

import numpy as np
from PIL import Image, ImageFilter
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"
LAYOUT = json.loads((ASSETS / "base/station_layout.json").read_text())
TABLE_TOP = LAYOUT["table_top"]


def linear(rgb):
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb)


def new_stage(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, root).GetPrim())
    return stage


def transform(prim, translate=None, rotation=None, scale=None):
    xf = UsdGeom.Xformable(prim)
    if translate is not None:
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, translate)))
    if rotation is not None:
        xf.AddOrientOp().Set(Gf.Quatf(float(rotation[0]), Gf.Vec3f(*map(float, rotation[1:]))))
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))


def material(stage, path, color, roughness=0.5, metallic=0.0, texture=None):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear(color)))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    if texture:
        uv = UsdShade.Shader.Define(stage, path + "/UV")
        uv.CreateIdAttr("UsdPrimvarReader_float2")
        uv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, path + "/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), "result")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.GetInput("diffuseColor").ConnectToSource(tex.ConnectableAPI(), "rgb")
    return mat


def bind(prim, mat, strong=False):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.strongerThanDescendants if strong else UsdShade.Tokens.weakerThanDescendants
    )


def mesh(stage, path, points, faces, mat=None, uvs=None, smooth=False):
    points = np.asarray(points, dtype=float)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([int(i) for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(False)
    m.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))])
    if smooth:
        normals = np.zeros_like(points)
        for face in faces:
            p0 = points[face[0]]
            for i in range(1, len(face) - 1):
                n = np.cross(points[face[i]] - p0, points[face[i + 1]] - p0)
                normals[list((face[0], face[i], face[i + 1]))] += n
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals.astype(np.float32)))
        m.SetNormalsInterpolation("vertex")
    if uvs is not None:
        UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex").Set(
            Vt.Vec2fArray.FromNumpy(np.asarray(uvs, dtype=np.float32))
        )
    if mat:
        bind(m.GetPrim(), mat)
    return m


def collision(prim, convex=False, offset=0.0008):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    if convex:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    return prim


def physics_material(stage, root, friction=0.8):
    mat = UsdShade.Material.Define(stage, root + "/PhysicsMaterial")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction * 0.8)
    api.CreateRestitutionAttr(0.0)
    return mat


def physics_bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose="physics")


def body(stage, root, mass, com):
    prim = stage.GetPrimAtPath(root)
    UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(16)
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(4)
    return prim


def box(stage, path, size, center, mat=None, collide=False):
    sx, sy, sz = np.asarray(size) / 2
    points = [(x, y, z) for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    # A separate vertex per corner per face gives stable face UVs and flat wood surfaces.
    expanded, polygons, uvs = [], [], []
    for f in faces:
        start = len(expanded)
        expanded.extend(points[i] for i in f)
        polygons.append(tuple(range(start, start + 4)))
        uvs.extend([(0, 0), (1, 0), (1, 1), (0, 1)])
    m = mesh(stage, path, expanded, polygons, mat, uvs)
    transform(m.GetPrim(), center)
    if collide:
        collision(m.GetPrim(), convex=True)
    return m.GetPrim()


def ellipsoid(stage, path, radii, center, mat=None, clip_x=None, collider=False, segments=40, rings=24):
    pts, faces, uv = [], [], []
    for j in range(rings + 1):
        v = j / rings
        phi = math.pi * (1 - v)
        for i in range(segments + 1):
            u = i / segments
            theta = 2 * math.pi * u
            x = radii[0] * math.sin(phi) * math.cos(theta)
            if clip_x is not None:
                x = float(np.clip(x, -clip_x, clip_x))
            pts.append((x, radii[1] * math.sin(phi) * math.sin(theta), radii[2] * math.cos(phi)))
            uv.append((u, v))
    stride = segments + 1
    for j in range(rings):
        for i in range(segments):
            a = j * stride + i
            faces.append((a, a + 1, a + stride + 1, a + stride))
    m = mesh(stage, path, pts, faces, mat, uv, smooth=True)
    transform(m.GetPrim(), center)
    if collider:
        collision(m.GetPrim(), convex=True)
        m.CreatePurposeAttr("guide")
        m.CreateVisibilityAttr("invisible")
    return m.GetPrim()


def cylinder(stage, path, radius, height, center, mat, axis="Z"):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    transform(c.GetPrim(), center)
    bind(c.GetPrim(), mat)
    return c.GetPrim()


def curve(stage, path, points, width, mat):
    c = UsdGeom.BasisCurves.Define(stage, path)
    c.CreateTypeAttr("linear")
    c.CreateCurveVertexCountsAttr([len(points)])
    c.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    c.CreateWidthsAttr([width])
    c.SetWidthsInterpolation("constant")
    bind(c.GetPrim(), mat)


def matrix_transform(prim, matrix):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(matrix, dtype=float).T.reshape(-1).tolist()))


def quat_matrix(q):
    return np.asarray(Gf.Matrix3d(Gf.Rotation(q))).T


def pose_matrix(pos, quat):
    out = np.eye(4)
    out[:3, :3] = quat_matrix(quat)
    out[:3, 3] = np.asarray(pos)
    return out


def read_stl(path):
    data = Path(path).read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    records = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("p", "<f4", (3, 3)), ("attribute", "<u2")]), count=count, offset=84)
    return records["p"].astype(float)


def pose_robot(stage, root):
    joints = [p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.IsA(UsdPhysics.Joint) and p.GetName().startswith("joint")]
    joints.sort(key=lambda p: int(p.GetName()[5:]))
    values = [*LAYOUT["ready"], LAYOUT["finger_open"], LAYOUT["finger_open"]]
    for p in joints:
        j = UsdPhysics.Joint(p)
        parent = stage.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0])
        child = stage.GetPrimAtPath(j.GetBody1Rel().GetTargets()[0])
        q = values[int(p.GetName()[5:]) - 1]
        a = pose_matrix(j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get())
        b = pose_matrix(j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get())
        axis = p.GetAttribute("physics:axis").Get()
        v = np.eye(3)["XYZ".index(axis)]
        motion = np.eye(4)
        if p.IsA(UsdPhysics.RevoluteJoint):
            motion[:3, :3] = np.asarray(Gf.Matrix3d(Gf.Rotation(Gf.Vec3d(*v), math.degrees(q)))).T
            UsdPhysics.DriveAPI.Apply(p, "angular").CreateTargetPositionAttr(math.degrees(q))
        else:
            motion[:3, 3] = v * q
            UsdPhysics.DriveAPI.Apply(p, "linear").CreateTargetPositionAttr(q)
        parent_world = np.asarray(UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        actual_parent_world = np.asarray(UsdGeom.Xformable(child.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        matrix_transform(child, np.linalg.inv(actual_parent_world) @ parent_world @ a @ motion @ np.linalg.inv(b))


def base_scene():
    s = new_stage(ASSETS / "base_station.usda", "/World")
    floor_mat = material(s, "/World/Looks/Floor", (0.58, 0.59, 0.60), 0.85)
    oak = material(s, "/World/Looks/Oak", (0.82, 0.64, 0.43), 0.58, texture="textures/bamboo_grain.png")
    black = material(s, "/World/Looks/ArmBlack", tuple(LAYOUT["arm_black_srgb"]), 0.5)
    white = material(s, "/World/Looks/ArmWhite", tuple(LAYOUT["arm_white_srgb"]), 0.5)
    env = "/World/envs/env_0"
    UsdGeom.Xform.Define(s, env)
    box(s, env + "/Table", LAYOUT["table_size"], LAYOUT["table_center"], oak, collide=True)
    box(s, "/World/Floor", (4, 4, 0.03), (0, 0, LAYOUT["floor_z"] - 0.015), floor_mat, collide=True)
    source_rotation = np.array(LAYOUT["arm_part_rotation"])
    for side, prim_name in (("left", "LeftArm"), ("right", "RightArm")):
        path = f"{env}/{prim_name}"
        arm = UsdGeom.Xform.Define(s, path).GetPrim()
        arm.GetReferences().AddReference("base/yam_arm.usda")
        transform(arm, LAYOUT["rig"]["bases"][side])
        for p in list(Usd.PrimRange(arm)):
            if not p.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            visual = s.GetPrimAtPath(str(p.GetPath()) + "/" + p.GetName())
            if not visual:
                continue
            if p.GetName() in LAYOUT["arm_part_position"]:
                UsdGeom.Imageable(visual).MakeInvisible()
                for color, mat in (("black", black), ("white", white)):
                    triangles = read_stl(ASSETS / "base" / f"{p.GetName()}_{color}.stl")
                    transformed = triangles.reshape(-1, 3) @ source_rotation.T + np.array(LAYOUT["arm_part_position"][p.GetName()])
                    points, ids = np.unique(transformed.round(7), axis=0, return_inverse=True)
                    mesh(s, f"{p.GetPath()}/Cover_{color}", points, ids.reshape(-1, 3), mat, smooth=True)
            else:
                bind(visual, black, strong=True)
        pose_robot(s, path)
    dome = UsdLux.DomeLight.Define(s, "/World/Lighting/Dome")
    dome.CreateIntensityAttr(300)
    dome.CreateColorAttr(Gf.Vec3f(0.86, 0.90, 0.95))
    lamp = UsdLux.SphereLight.Define(s, "/World/Lighting/Key")
    lamp.CreateIntensityAttr(15000)
    lamp.CreateRadiusAttr(0.45)
    transform(lamp.GetPrim(), (-0.2, -0.55, 2.2))
    physics = UsdPhysics.Scene.Define(s, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(9.81)
    camera = UsdGeom.Camera.Define(s, "/World/PreviewCamera")
    camera.CreateFocalLengthAttr(43)
    camera.CreateHorizontalApertureAttr(36)
    camera.CreateVerticalApertureAttr(24)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.02, 30))
    look = Gf.Matrix4d().SetLookAt(Gf.Vec3d(-0.85, 0, 1.62), Gf.Vec3d(0.29, 0, 0.84), Gf.Vec3d(0, 0, 1)).GetInverse()
    camera.AddTransformOp().Set(look)
    s.GetRootLayer().Save()


def instance(stage, path, asset, position, rotation=None, fixed=False):
    p = UsdGeom.Xform.Define(stage, path).GetPrim()
    p.GetReferences().AddReference(asset)
    transform(p, position, rotation)
    if fixed:
        UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr(True)
    return p


