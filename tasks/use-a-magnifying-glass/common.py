"""Shared rendering and physical geometry for the magnifier task runners."""
from __future__ import annotations

import itertools
import numpy as np


def array(value):
    value = value.torch if hasattr(value, "torch") else value
    return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)


def author_card(stage, path, texture, width, height, center, rotation):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    from yam_arena.ik import matrix_to_quat_xyzw
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(-width/2, -height/2, 0), (width/2, -height/2, 0),
                          (width/2, height/2, 0), (-width/2, height/2, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex")
    uv.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    x = UsdGeom.Xformable(mesh)
    x.AddTranslateOp().Set(Gf.Vec3d(*center))
    q = matrix_to_quat_xyzw(rotation)
    x.AddOrientOp().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))
    material = UsdShade.Material.Define(stage, path+"/Material")
    shader = UsdShade.Shader.Define(stage, path+"/Material/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.)
    reader = UsdShade.Shader.Define(stage, path+"/Material/UV")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tex = UsdShade.Shader.Define(stage, path+"/Material/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture.resolve())))
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh.GetPrim()


def review_camera(embodiment):
    """The same fixed review view as the validated pickup demo; not a policy camera."""
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg
    from isaaclab_arena.utils.cameras import ArenaCameraCfg
    from isaaclab_arena.utils.configclass import make_configclass
    from pxr import Gf
    from yam_arena.embodiment import RENDERER
    q = Gf.Matrix4d().SetLookAt(Gf.Vec3d(.85, -.90, 1.65),
                               Gf.Vec3d(.30, 0., .87), Gf.Vec3d(0, 0, 1)).GetInverse().ExtractRotationQuat()
    config = CameraCfg(prim_path="{ENV_REGEX_NS}/MagnifierReviewCam", height=720, width=960,
        data_types=["rgb"], spawn=sim_utils.PinholeCameraCfg(focal_length=20., horizontal_aperture=20.955,
            clipping_range=(.01, 20.)), offset=CameraCfg.OffsetCfg(pos=(.85, -.90, 1.65),
            rot=(*q.GetImaginary(), q.GetReal()), convention="opengl"), renderer_cfg=RENDERER)
    fields = [(name, CameraCfg, value) for name, value in vars(embodiment.camera_config).items()
              if isinstance(value, CameraCfg)]
    fields.append(("review_cam", CameraCfg, config))
    embodiment.camera_config = make_configclass("MagnifierReviewCameraCfg", fields, bases=(ArenaCameraCfg,))()


def tool_vertices():
    """All collision-solid vertices, in the original handle's local frame."""
    if __package__:
        from . import tool
        from .lens import generate_lens
    else:
        import tool
        from lens import generate_lens
    corners = np.asarray(list(itertools.product(*[(-s/2, s/2) for s in tool.HANDLE_SIZE_M])))
    rim = np.asarray([v for i in range(tool.RIM_SECTORS) for v in tool.ring_sector_geometry(i)[0]])
    glass = np.asarray(generate_lens(curvature_radius_m=.100, glass_ior=1.5)["vertices"])
    return np.vstack((corners, rim, glass+np.asarray(tool.LENS_CENTER_M)))


def table_clearance(local_vertices, pose_xyzw, table_z=.7475):
    """Lowest point of the complete rigid tool above the horizontal table plane."""
    p = np.asarray(pose_xyzw, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all() or np.linalg.norm(p[3:]) < 1e-12:
        raise ValueError("Invalid tool pose")
    x, y, z, w = p[3:]/np.linalg.norm(p[3:])
    world_z_axis = np.array([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)])
    return float(np.min(np.asarray(local_vertices)@world_z_axis)+p[2]-table_z)
