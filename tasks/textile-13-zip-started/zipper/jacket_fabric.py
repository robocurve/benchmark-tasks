"""Build and spawn a padded jacket with a continuous cloth hem."""

from pathlib import Path

from .mechanism import INITIAL_PIN

import numpy as np


THICKNESS = 0.008
ASSETS = Path(__file__).resolve().parent.parent / "assets"
FABRIC_NAME = "jacket_fabric"

OUTLINE = (
    (-0.023, 0.009), (0.178, 0.009), (0.187, 0.018), (0.214, 0.035),
    (0.209, 0.066), (0.196, 0.093), (0.120, 0.176), (0.096, 0.184),
    (0.074, 0.157), (0.113, 0.092), (0.054, 0.087), (-0.018, 0.083), (-0.025, 0.074),
)
UV_ORIGIN = np.array([-0.026, 0.008])
UV_SIZE = np.array([0.242, 0.178])
TRIM = (
    (((-0.023, 0.009), (-0.010, 0.009), (-0.010, 0.083), (-0.018, 0.083), (-0.025, 0.074)), (33, 77, 116)),
    (((0.120, 0.176), (0.096, 0.184), (0.074, 0.157), (0.082, 0.144), (0.106, 0.169)), (33, 77, 116)),
    (((0.178, 0.009), (0.187, 0.018), (0.214, 0.035), (0.211, 0.053), (0.192, 0.040),
      (0.176, 0.018), (0.169, 0.009)), (33, 77, 116)),
    (((0.023, 0.034), (0.060, 0.043), (0.074, 0.061), (0.028, 0.068)), (65, 121, 159)),
    (((0.023, 0.034), (0.060, 0.043), (0.064, 0.048), (0.025, 0.039)), (127, 178, 205)),
    (((0.196, 0.093), (0.190, 0.100), (0.147, 0.096), (0.113, 0.092), (0.115, 0.090),
      (0.148, 0.094)), (127, 178, 205)),
)


def build_assets():
    """Bake the mesh and fabric texture offline. Triangle is only needed for this step."""
    import triangle
    from PIL import Image, ImageDraw

    ASSETS.mkdir(exist_ok=True)
    indices = np.arange(len(OUTLINE))
    shape = triangle.triangulate(
        {"vertices": OUTLINE, "segments": np.column_stack((indices, np.roll(indices, -1)))},
        "pq28a0.000021",
    )
    xy = shape["vertices"]
    # The seam is raised over the zipper rail; the front has a shallow padded crown.
    y = xy[:, 1]
    crown = np.sin(np.pi * np.clip((y - 0.012) / 0.083, 0, 1))
    z = 0.014 + 0.007 * crown
    np.savez_compressed(ASSETS / "jacket_panel.npz", vertices=np.column_stack((xy, z)), faces=shape["triangles"])

    width, height = 768, 564
    texture = Image.new("RGB", (width, height), (85, 144, 184))
    draw = ImageDraw.Draw(texture)
    for polygon, color in TRIM:
        uv = (np.asarray(polygon) - UV_ORIGIN) / UV_SIZE
        pixels = [(u * (width - 1), (1 - v) * (height - 1)) for u, v in uv]
        draw.polygon(pixels, fill=color)
    rows, cols = np.indices((height, width))
    grain = 3 * np.sin(cols * np.pi / 2) + 2 * np.cos(rows * np.pi / 2)
    pixels = np.asarray(texture, dtype=np.int16) + grain[:, :, None]
    Image.fromarray(np.clip(pixels, 0, 255).astype(np.uint8)).save(ASSETS / "jacket_fabric.png")


def make_jacket():
    import trimesh

    with np.load(ASSETS / "jacket_panel.npz") as asset:
        left = asset["vertices"]
        faces = asset["faces"]

    right = left * (1, -1, 1)
    count = len(left)
    # The jacket remains connected across the shoulders. Its separating hem is open.
    hem = np.flatnonzero(np.isclose(left[:, 1], 0.009) & (left[:, 0] > 0.145) & (left[:, 0] < 0.178))
    hem = hem[np.argsort(left[hem, 0])]
    start, end = hem[:-1], hem[1:]
    bridge = np.vstack((
        np.column_stack((start, start + count, end + count)),
        np.column_stack((start, end + count, end)),
    ))
    vertices = np.vstack((left, right))
    triangles = np.vstack((faces, faces[:, ::-1] + count, bridge))
    return trimesh.Trimesh(vertices=vertices, faces=triangles, process=False)


def initial_vertices():
    """Fold the lower corner while keeping the sewn cloth's flat rest shape."""
    vertices = np.asarray(make_jacket().vertices).copy()
    right = vertices[:, 1] < 0
    weight = np.clip((0.175 - vertices[:, 0]) / 0.150, 0, 1) * right
    vertices += weight[:, None] * np.asarray(INITIAL_PIN)
    vertices[:, 2] += 0.020 * np.sin(np.pi * weight)
    return vertices


def _finish_panel(stage, path, mesh):
    from pxr import Sdf, UsdGeom, UsdShade, Vt

    visual = UsdGeom.Mesh.Get(stage, f"{path}/geometry/mesh")
    vertex_count = len(mesh.vertices)
    offset = mesh.vertex_normals * (THICKNESS / 2)
    points = np.vstack((mesh.vertices + offset, mesh.vertices - offset))
    _, edge_ids, edge_counts = np.unique(mesh.edges_sorted, axis=0, return_inverse=True, return_counts=True)
    a, b = mesh.edges[edge_counts[edge_ids] == 1].T
    faces = np.vstack((
        mesh.faces,
        mesh.faces[:, ::-1] + vertex_count,
        np.column_stack((b, a, a + vertex_count)),
        np.column_stack((b, a + vertex_count, b + vertex_count)),
    ))
    usd_points = Vt.Vec3fArray.FromNumpy(points.astype(np.float32))
    visual.GetPointsAttr().Set(usd_points)
    visual.GetFaceVertexCountsAttr().Set([3] * len(faces))
    visual.GetFaceVertexIndicesAttr().Set(faces.ravel().tolist())
    visual.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    visual.GetPrim().GetAttribute("deformablePose:default:omniphysics:points").Set(usd_points)

    xy = np.tile(mesh.vertices[:, :2], (2, 1))
    xy[:, 1] = np.abs(xy[:, 1])
    uv = Vt.Vec2fArray.FromNumpy(((xy - UV_ORIGIN) / UV_SIZE).astype(np.float32))
    UsdGeom.PrimvarsAPI(visual).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex").Set(uv)
    material = UsdShade.Material.Define(stage, f"{path}/geometry/JacketMaterial")
    surface = UsdShade.Shader.Define(stage, f"{path}/geometry/JacketMaterial/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.91)
    reader = UsdShade.Shader.Define(stage, f"{path}/geometry/JacketMaterial/UV")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture = UsdShade.Shader.Define(stage, f"{path}/geometry/JacketMaterial/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(ASSETS / "jacket_fabric.png"))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(visual.GetPrim()).Bind(material)


def _sew_to_rails(stage, path):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    cloth = Sdf.Path(path)
    sim_prim = next(
        prim for prim in Usd.PrimRange(stage.GetPrimAtPath(cloth))
        if "OmniPhysicsSurfaceDeformableSimAPI" in prim.GetAppliedSchemas()
    )
    zipper = cloth.GetParentPath().AppendChild("zipper")
    points = UsdGeom.Mesh(sim_prim).GetPointsAttr().Get()
    excluded = [zipper.AppendChild(name) for name in ("Base", "Slider", "PinSide")]
    vertices = np.asarray(points)
    left = np.flatnonzero((vertices[:, 1] > 0) & (vertices[:, 1] < 0.018)
                          & (vertices[:, 0] > 0.005) & (vertices[:, 0] < 0.177))
    pin_local = vertices
    lower_right = np.flatnonzero((pin_local[:, 1] < -0.005) & (pin_local[:, 1] > -0.028)
                                 & (pin_local[:, 0] > 0.005) & (pin_local[:, 0] < 0.025))
    for name, link, indices, local in (
        ("LeftSeam", "Base", left, vertices),
        ("PinTapeSeam", "PinSide", lower_right, pin_local),
    ):
        if len(indices) < 4:
            raise RuntimeError(f"Too few seam nodes for {name}: {len(indices)}")
        attachment = stage.DefinePrim(cloth.AppendChild(name), "OmniPhysicsVtxXformAttachment")
        attachment.GetRelationship("omniphysics:src0").SetTargets([sim_prim.GetPath()])
        attachment.GetRelationship("omniphysics:src1").SetTargets([zipper.AppendChild(link)])
        attachment.GetAttribute("omniphysics:vtxIndicesSrc0").Set(indices.tolist())
        attachment.GetAttribute("omniphysics:localPositionsSrc1").Set([Gf.Vec3f(*map(float, local[i])) for i in indices])
    excluded = [
        prim.GetPath()
        for link in excluded
        for prim in Usd.PrimRange(stage.GetPrimAtPath(link))
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    UsdPhysics.FilteredPairsAPI.Apply(sim_prim).CreateFilteredPairsRel().SetTargets(excluded)



def jacket_spawner():
    from isaaclab.sim import MeshRectangleCfg
    from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
    from isaaclab.sim.utils import clone, get_current_stage
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg

    mesh = make_jacket()

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        _spawn_mesh_geom_from_mesh(prim_path, cfg, mesh, translation, orientation, **kwargs)
        stage = get_current_stage()
        _finish_panel(stage, prim_path, mesh)
        _sew_to_rails(stage, prim_path)
        return stage.GetPrimAtPath(prim_path)

    return MeshRectangleCfg(
        func=spawn,
        size=(0.24, 0.37),
        edge_refinement=1.0,
        deformable_props=PhysxDeformableBodyPropertiesCfg(),
        collision_props=[PhysxCollisionCfg(rest_offset=THICKNESS / 2, contact_offset=0.007)],
        physics_material=PhysxSurfaceDeformableBodyMaterialCfg(
            density=150.0,
            surface_thickness=THICKNESS,
            poissons_ratio=0.25,
            youngs_modulus=4.0e5,
            surface_bend_stiffness=4.0e4,
            elasticity_damping=0.1,
            bend_damping=0.1,
            static_friction=0.7,
            dynamic_friction=0.5,
        ),
    )


if __name__ == "__main__":
    build_assets()
