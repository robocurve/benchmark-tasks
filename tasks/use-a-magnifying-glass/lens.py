"""Closed spherical biconvex lens for an optical feasibility experiment.

Geometry is generated with the Python standard library; USD is imported only
inside ``author_lens`` after the caller has started Isaac Sim. Local +Z is the
optical axis. This builds glass, not a screen-space enlargement effect.
"""

from __future__ import annotations

import json
import math


OMNIGLASS_REFERENCE = (
    "https://docs.omniverse.nvidia.com/materials-and-rendering/latest/"
    "templates/OmniGlass.html"
)


def generate_lens(
    radius_m=0.03,
    curvature_radius_m=0.10,
    edge_thickness_m=0.001,
    rings=24,
    segments=128,
    *,
    glass_ior=1.5,
):
    """Return vertices, triangle faces, smooth normals and analytic metadata.

    The two spherical caps share their outer vertices with the connecting
    barrel, so the mesh is topologically closed. ``normals`` gives the spherical
    vertex normals; ``face_vertex_normals`` additionally gives the barrel its
    radial normals at the sharp rim. USD uses the latter with faceVarying
    interpolation: the optical surfaces remain smooth without rounding the rim.

    ``curvature_radius_m`` is the radius of each spherical surface, not an exact
    focal-length input. For glass IOR 1.5 its thin-lens focal length equals this
    radius. Metadata also reports the thick-lens paraxial effective focal length.
    Those approximations do not establish useful rendered magnification.
    """
    for name, value in (
        ("radius_m", radius_m),
        ("curvature_radius_m", curvature_radius_m),
        ("edge_thickness_m", edge_thickness_m),
    ):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if curvature_radius_m <= radius_m:
        raise ValueError("curvature_radius_m must exceed radius_m")
    if not math.isfinite(glass_ior) or glass_ior < 1:
        raise ValueError("glass_ior must be finite and at least 1")
    if not isinstance(rings, int) or isinstance(rings, bool) or rings < 2:
        raise ValueError("rings must be an integer of at least 2")
    if not isinstance(segments, int) or isinstance(segments, bool) or segments < 8:
        raise ValueError("segments must be an integer of at least 8")

    a, r = float(radius_m), float(curvature_radius_m)
    edge = float(edge_thickness_m)
    rim_root = math.sqrt(r * r - a * a)
    sag = r - rim_root
    thickness = edge + 2 * sag
    vertices, normals, faces = [], [], []
    for sign in (1, -1):
        vertices.append((0.0, 0.0, sign * thickness / 2))
        normals.append((0.0, 0.0, float(sign)))
        for ring in range(1, rings + 1):
            rho = a * ring / rings
            root = math.sqrt(r * r - rho * rho)
            z = sign * (edge / 2 + root - rim_root)
            for segment in range(segments):
                angle = 2 * math.pi * segment / segments
                x, y = rho * math.cos(angle), rho * math.sin(angle)
                vertices.append((x, y, z))
                normals.append((x / r, y / r, sign * root / r))

    cap_vertex_count = 1 + rings * segments

    def ring_vertex(cap, ring, segment):
        return cap * cap_vertex_count + 1 + (ring - 1) * segments + segment % segments

    for cap in (0, 1):
        cap_faces = []
        for segment in range(segments):
            cap_faces.append((cap * cap_vertex_count,
                              ring_vertex(cap, 1, segment),
                              ring_vertex(cap, 1, segment + 1)))
        for ring in range(1, rings):
            for segment in range(segments):
                inner = ring_vertex(cap, ring, segment)
                inner_next = ring_vertex(cap, ring, segment + 1)
                outer = ring_vertex(cap, ring + 1, segment)
                outer_next = ring_vertex(cap, ring + 1, segment + 1)
                cap_faces.extend(((inner, outer, outer_next),
                                  (inner, outer_next, inner_next)))
        faces.extend(cap_faces if cap == 0 else [(c, b, a_) for a_, b, c in cap_faces])

    cap_face_count = len(faces)
    for segment in range(segments):
        top = ring_vertex(0, rings, segment)
        top_next = ring_vertex(0, rings, segment + 1)
        bottom = ring_vertex(1, rings, segment)
        bottom_next = ring_vertex(1, rings, segment + 1)
        faces.extend(((top, bottom, bottom_next), (top, bottom_next, top_next)))

    face_vertex_normals = [normals[index] for face in faces[:cap_face_count] for index in face]
    for face in faces[cap_face_count:]:
        for index in face:
            x, y, _ = vertices[index]
            face_vertex_normals.append((x / a, y / a, 0.0))

    power = (glass_ior - 1) * (
        2 / r - (glass_ior - 1) * thickness / (glass_ior * r * r)
    )
    meta = {
        "shape": "closed symmetric spherical biconvex lens with cylindrical edge",
        "local_optical_axis": "+Z",
        "radius_m": a,
        "curvature_radius_m": r,
        "edge_thickness_m": edge,
        "center_thickness_m": thickness,
        "surface_sag_m": sag,
        "glass_ior": glass_ior,
        "thin_lens_focal_length_m": r / (2 * (glass_ior - 1)) if glass_ior > 1 else None,
        "thick_lens_effective_focal_length_m": 1 / power if power else None,
        "analytic_volume_m3": math.pi * a * a * edge + 2 * math.pi * sag * sag * (r - sag / 3),
        "rings": rings,
        "segments": segments,
        "vertex_count": len(vertices),
        "triangle_count": len(faces),
        "normals_interpolation": "faceVarying",
        "subdivision_scheme": "none",
        "assumptions": [
            "Uniform, nondispersive refractive index; air has refractive index 1.",
            "Two spherical caps, finite thickness and a sharp cylindrical edge.",
            "Focal lengths are paraxial lensmaker estimates, not a render result.",
            "The thick-lens effective focal length is measured from principal planes.",
            "Smooth analytic surface normals approximate a continuous optical surface.",
            "No scratch, coating, dispersion, surface roughness or manufacturing error.",
        ],
        "material_reference": OMNIGLASS_REFERENCE,
    }
    return {"vertices": vertices, "faces": faces, "normals": normals,
            "face_vertex_normals": face_vertex_normals, "meta": meta}


def _rotation_quaternion(rotation_wxyz, rotation_matrix):
    """Accept a conventional column-vector 3x3 rotation or normalized WXYZ."""
    if rotation_matrix is not None:
        if rotation_wxyz is not None:
            raise ValueError("Specify rotation_wxyz or rotation_matrix, not both")
        m = [[float(value) for value in row] for row in rotation_matrix]
        if len(m) != 3 or any(len(row) != 3 for row in m):
            raise ValueError("rotation_matrix must be 3x3")
        if not all(math.isfinite(value) for row in m for value in row):
            raise ValueError("rotation_matrix must be finite")
        for i in range(3):
            for j in range(3):
                if abs(sum(m[k][i] * m[k][j] for k in range(3)) - (i == j)) > 1e-6:
                    raise ValueError("rotation_matrix must be orthonormal")
        det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
               - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
               + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
        if abs(det - 1) > 1e-6:
            raise ValueError("rotation_matrix must preserve handedness")
        trace = sum(m[i][i] for i in range(3))
        if trace > 0:
            scale = math.sqrt(trace + 1) * 2
            q = (scale / 4, (m[2][1] - m[1][2]) / scale,
                 (m[0][2] - m[2][0]) / scale, (m[1][0] - m[0][1]) / scale)
        else:
            i = max(range(3), key=lambda index: m[index][index])
            j, k = (i + 1) % 3, (i + 2) % 3
            scale = math.sqrt(1 + m[i][i] - m[j][j] - m[k][k]) * 2
            vector = [0.0, 0.0, 0.0]
            vector[i] = scale / 4
            vector[j] = (m[j][i] + m[i][j]) / scale
            vector[k] = (m[k][i] + m[i][k]) / scale
            q = ((m[k][j] - m[j][k]) / scale, *vector)
    else:
        q = tuple(float(value) for value in (rotation_wxyz or (1, 0, 0, 0)))
        if len(q) != 4 or not all(math.isfinite(value) for value in q):
            raise ValueError("rotation_wxyz must have four finite values")
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12:
        raise ValueError("rotation quaternion must be nonzero")
    return tuple(value / norm for value in q)


def author_lens(
    stage,
    path,
    center=(0.0, 0.0, 0.0),
    rotation_wxyz=None,
    *,
    rotation_matrix=None,
    material_path=None,
    **geometry_kwargs,
):
    """Author a static thick-glass mesh; return its ``Usd.Prim``.

    Call only after Isaac Sim startup. Geometry arguments are forwarded to
    ``generate_lens``. No physics or collision API is added by this optical
    helper. Callers may later place this visual below a physical tool assembly.
    ``rotation_matrix`` uses the usual column-vector convention.
    """
    data = generate_lens(**geometry_kwargs)
    quaternion = _rotation_quaternion(rotation_wxyz, rotation_matrix)
    center = tuple(float(value) for value in center)
    if len(center) != 3 or not all(math.isfinite(value) for value in center):
        raise ValueError("center must have three finite values")
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in data["vertices"]])
    mesh.CreateFaceVertexCountsAttr([3] * len(data["faces"]))
    mesh.CreateFaceVertexIndicesAttr([index for face in data["faces"] for index in face])
    mesh.CreateNormalsAttr([Gf.Vec3f(*normal) for normal in data["face_vertex_normals"]])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr(False)
    a = data["meta"]["radius_m"]
    half_t = data["meta"]["center_thickness_m"] / 2
    mesh.CreateExtentAttr([Gf.Vec3f(-a, -a, -half_t), Gf.Vec3f(a, a, half_t)])
    transform = UsdGeom.Xformable(mesh.GetPrim())
    transform.ClearXformOpOrder()
    transform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*center))
    transform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(quaternion[0], Gf.Vec3d(*quaternion[1:])))

    material = UsdShade.Material.Define(stage, material_path or f"{path}/GlassMaterial")
    shader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/Shader")
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    shader.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1, 1, 1))
    shader.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(data["meta"]["glass_ior"])
    shader.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(False)
    shader.CreateInput("frosting_roughness", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(False)
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    mesh.GetPrim().CreateAttribute("magnifier:geometry", Sdf.ValueTypeNames.String).Set(
        json.dumps(data["meta"], sort_keys=True))
    return mesh.GetPrim()
