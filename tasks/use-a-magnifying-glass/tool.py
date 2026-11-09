"""Add a physical magnifier head to an existing dynamic Box handle.

Call before the first physics reset. The handle remains the only rigid body;
all added shapes are ordinary child colliders. Dimensions are metres.
"""

from __future__ import annotations

import json
import math

if __package__:
    from .lens import author_lens, generate_lens
else:
    from lens import author_lens, generate_lens


HANDLE_SIZE_M = (0.080, 0.020, 0.018)
LENS_CENTER_M = (-0.070, 0.0, 0.0)
TOOL_MASS_KG = 0.120
RIM_INNER_RADIUS_M = 0.030
RIM_OUTER_RADIUS_M = 0.034
RIM_THICKNESS_M = 0.018
RIM_SECTORS = 32
CONTACT_OFFSET_M = 0.0002
FRICTION = 0.7


def ring_sector_geometry(index: int) -> tuple[list[tuple], list[tuple]]:
    """One closed convex annular trapezoid extruded along local Z."""
    if not isinstance(index, int) or not 0 <= index < RIM_SECTORS:
        raise ValueError("Rim sector index must be in [0, 31]")
    a = 2 * math.pi * index / RIM_SECTORS
    b = 2 * math.pi * (index + 1) / RIM_SECTORS
    outline = [(RIM_INNER_RADIUS_M * math.cos(a), RIM_INNER_RADIUS_M * math.sin(a)),
               (RIM_OUTER_RADIUS_M * math.cos(a), RIM_OUTER_RADIUS_M * math.sin(a)),
               (RIM_OUTER_RADIUS_M * math.cos(b), RIM_OUTER_RADIUS_M * math.sin(b)),
               (RIM_INNER_RADIUS_M * math.cos(b), RIM_INNER_RADIUS_M * math.sin(b))]
    vertices = [(x + LENS_CENTER_M[0], y, z)
                for z in (-RIM_THICKNESS_M / 2, RIM_THICKNESS_M / 2)
                for x, y in outline]
    faces = [(3, 2, 1, 0), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return vertices, faces


def attach_tool(stage, root_path: str) -> dict:
    """Attach the 100 mm-curvature lens and rim to a dynamic Box root.

    The caller creates the 80 x 20 x 18 mm handle and sets its mass to 120 g.
    That mass is the complete tool's mass, not an additional handle mass.
    Existing handle colliders also receive the declared contact material.
    No rigid-body, mass, joint or force API is added to child prims.
    """
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    root_path = str(root_path).rstrip("/")
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError("root_path must name the existing handle rigid body")
    body = UsdPhysics.RigidBodyAPI(root)
    if body.GetRigidBodyEnabledAttr().Get() is False or body.GetKinematicEnabledAttr().Get():
        raise ValueError("The handle must be an enabled dynamic rigid body")
    mass = UsdPhysics.MassAPI(root).GetMassAttr().Get()
    if mass is None or not math.isclose(float(mass), TOOL_MASS_KG, rel_tol=1e-6):
        raise ValueError("Set the root's total tool mass to 0.120 kg before attaching")
    world = UsdGeom.XformCache().GetLocalToWorldTransform(root)
    basis = [world.TransformDir(Gf.Vec3d(*(float(i == j) for i in range(3)))) for j in range(3)]
    if any(abs(v.GetLength() - 1.0) > 1e-6 for v in basis):
        raise ValueError("Handle root and ancestors must have unit scale; scale its geometry instead")
    if any(abs(Gf.Dot(basis[i], basis[j])) > 1e-6 for i in range(3) for j in range(i)):
        raise ValueError("Handle root and ancestors must not shear the lens")
    head_path = root_path + "/MagnifierHead"
    if stage.GetPrimAtPath(head_path):
        raise ValueError("MagnifierHead already exists; attach once before physics initialization")
    existing_colliders = [prim for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.CollisionAPI)]
    if not existing_colliders:
        raise ValueError("The handle must already contain a collision shape")
    if any(prim != root and prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in Usd.PrimRange(root)):
        raise ValueError("The handle hierarchy must contain only its root rigid body")

    UsdGeom.Xform.Define(stage, head_path)
    physical = UsdShade.Material.Define(stage, head_path + "/ContactMaterial")
    material_api = UsdPhysics.MaterialAPI.Apply(physical.GetPrim())
    material_api.CreateStaticFrictionAttr().Set(FRICTION)
    material_api.CreateDynamicFrictionAttr().Set(FRICTION)
    material_api.CreateRestitutionAttr().Set(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(physical.GetPrim()).CreateFrictionCombineModeAttr().Set("average")

    def collision(prim, *, mesh=False):
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr().Set(True)
        if mesh:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexHull")
        contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        contact.CreateContactOffsetAttr().Set(CONTACT_OFFSET_M)
        contact.CreateRestOffsetAttr().Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            physical, UsdShade.Tokens.strongerThanDescendants, "physics")

    for prim in existing_colliders:
        collision(prim)

    look = UsdShade.Material.Define(stage, head_path + "/RimMaterial")
    shader = UsdShade.Shader.Define(stage, str(look.GetPath()) + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.012, 0.045, 0.12))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    lens = author_lens(stage, head_path + "/Lens", center=LENS_CENTER_M,
                       curvature_radius_m=0.100, glass_ior=1.5)
    collision(lens, mesh=True)
    for index in range(RIM_SECTORS):
        vertices, faces = ring_sector_geometry(index)
        mesh = UsdGeom.Mesh.Define(stage, f"{head_path}/Rim_{index:02d}")
        mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices])
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr([v for face in faces for v in face])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateOrientationAttr("rightHanded")
        mesh.CreateDoubleSidedAttr(False)
        mesh.CreateExtentAttr([Gf.Vec3f(*(min(v[i] for v in vertices) for i in range(3))),
                               Gf.Vec3f(*(max(v[i] for v in vertices) for i in range(3)))])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look)
        collision(mesh.GetPrim(), mesh=True)

    metadata = {
        "rigid_body_count": 1,
        "handle_size_m": HANDLE_SIZE_M,
        "lens_center_in_handle_m": LENS_CENTER_M,
        "lens_axis_in_handle": "+Z",
        "total_tool_mass_kg": float(mass),
        "lens": generate_lens(curvature_radius_m=0.100, glass_ior=1.5)["meta"],
        "rim": {"inner_vertex_radius_m": RIM_INNER_RADIUS_M,
                "minimum_clear_aperture_radius_m": RIM_INNER_RADIUS_M * math.cos(math.pi / RIM_SECTORS),
                "outer_vertex_radius_m": RIM_OUTER_RADIUS_M,
                "thickness_m": RIM_THICKNESS_M,
                "convex_sector_count": RIM_SECTORS},
        "existing_handle_collider_count": len(existing_colliders),
        "added_collider_count": 1 + RIM_SECTORS,
        "mesh_collision_approximation": "convexHull",
        "contact_offset_m": CONTACT_OFFSET_M,
        "rest_offset_m": 0.0,
        "static_friction": FRICTION,
        "dynamic_friction": FRICTION,
        "friction_combine_mode": "average",
        "restitution": 0.0,
        "assumptions": [
            "120 g is the complete tool mass; no mass is added to child shapes.",
            "Existing root mass/inertia settings are retained; otherwise PhysX computes compound mass properties.",
            "The frame is a 32-sector polygonal approximation to an annular plastic rim.",
            "Friction is an uncalibrated assumed value, shared by handle, rim and glass.",
            "Glass is rigid and unbreakable; no compliant mounting or material failure is modeled.",
            "All motion and gripping use ordinary physical contact; no joints or grasp assistance are added.",
        ],
    }
    root.CreateAttribute("magnifier:tool", Sdf.ValueTypeNames.String).Set(json.dumps(metadata, sort_keys=True))
    return metadata
