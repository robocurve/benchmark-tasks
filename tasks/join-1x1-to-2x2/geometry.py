"""Procedural, hollow, printable construction bricks at a 16 mm stud pitch.

These are custom Duplo-scale parts, not dimensionally certified LEGO parts.
The USD asset uses compound convex collisions. Socket interference is resolved
by a compliant contact material and friction, with no attachment state or joints.
The contact coefficients are explicit engineering assumptions, not measurements.

Pure geometry functions import no simulator modules. ``generate_brick_usd``
imports USD only when called, after the Isaac SimulationApp has started.
All geometric values are metres; the generated OpenSCAD file uses millimetres.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Sequence

PITCH = 0.016
BODY_HEIGHT = 0.0192
BODY_CLEARANCE = 0.0002
STUD_RADIUS = 0.0048
STUD_HEIGHT = 0.0048
WALL_THICKNESS = 0.0015
ROOF_THICKNESS = 0.0015
SOCKET_OUTER_RADIUS = 0.0062
SOCKET_MOUTH_RADIUS = 0.0053
SOCKET_CLEAR_RADIUS = 0.0049
HARD_LEAD_HEIGHT = 0.0007
SOFT_LEAD_HEIGHT = 0.0005
DEFAULT_INTERFERENCE = 0.0001
DEFAULT_SEGMENTS = 16
DEFAULT_STIFFNESS = 500.0
DEFAULT_DAMPING = 1.0
CONTACT_OFFSET = 0.0002


def _shape(shape: Sequence[int]) -> tuple[int, int]:
    if len(shape) != 2 or any(isinstance(n, bool) or int(n) != n or n < 1 for n in shape):
        raise ValueError("shape must be two positive integers, e.g. (1, 1) or (2, 2)")
    return int(shape[0]), int(shape[1])


def stud_centres(shape: Sequence[int]) -> list[tuple[float, float]]:
    """Stud/socket centres in body coordinates, ordered x first then y."""
    nx, ny = _shape(shape)
    return [((i - (nx - 1) / 2) * PITCH, (j - (ny - 1) / 2) * PITCH)
            for i in range(nx) for j in range(ny)]


def brick_dimensions(shape: Sequence[int]) -> tuple[float, float, float]:
    """The body dimensions, excluding the studs above its roof."""
    nx, ny = _shape(shape)
    return nx * PITCH - BODY_CLEARANCE, ny * PITCH - BODY_CLEARANCE, BODY_HEIGHT


def _ring_segment(cx, cy, z0, z1, inner0, inner1, outer, index, segments):
    """A convex tapered annular sector, with flat inner-face radii specified.

    The polygon's *inradius*, rather than its vertex radius, determines fit.
    This avoids silently adding polygon-approximation interference.
    """
    half = math.pi / segments
    angle = 2 * math.pi * index / segments
    angles = (angle - half, angle + half)
    scale = 1 / math.cos(half)
    vertices = []
    for z, inner in ((z0, inner0), (z1, inner1)):
        for radius, theta in ((inner, angles[0]), (outer, angles[0]),
                              (outer, angles[1]), (inner, angles[1])):
            vertices.append((cx + radius * scale * math.cos(theta),
                             cy + radius * scale * math.sin(theta), z))
    # Outward winding: bottom, top, two sides, outer face, inner face.
    faces = ((3, 2, 1, 0), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    return vertices, faces


def brick_geometry_specs(shape: Sequence[int], *,
                         interference: float = DEFAULT_INTERFERENCE,
                         segments: int = DEFAULT_SEGMENTS) -> dict:
    """Return JSON-serializable compound primitives and manufacturing dimensions.

    The root is the body centre, not the complete asset's bounding-box centre.
    A socket has a hard clearance lead-in, then a compliant tapered interference
    land. The straight land reaches the roof so every ring is connected in the
    printable part. Compliance is confined to the socket contact surfaces.
    """
    shape = _shape(shape)
    if not math.isfinite(interference) or not 0 < interference <= 0.0003:
        raise ValueError("interference must be finite and in (0, 0.0003] metres")
    if isinstance(segments, bool) or int(segments) != segments or segments < 8:
        raise ValueError("segments must be an integer >= 8")
    segments = int(segments)
    width, depth, height = brick_dimensions(shape)
    bottom = -height / 2
    roof_bottom = height / 2 - ROOF_THICKNESS
    wall_h = height - ROOF_THICKNESS
    wall_z = -ROOF_THICKNESS / 2
    parts = []

    def box(name, size, centre):
        parts.append({"name": name, "kind": "box", "size": list(size),
                      "centre": list(centre), "material": "hard"})

    box("roof", (width, depth, ROOF_THICKNESS), (0, 0, height / 2 - ROOF_THICKNESS / 2))
    box("wall_x_neg", (WALL_THICKNESS, depth, wall_h), (-(width - WALL_THICKNESS) / 2, 0, wall_z))
    box("wall_x_pos", (WALL_THICKNESS, depth, wall_h), ((width - WALL_THICKNESS) / 2, 0, wall_z))
    box("wall_y_neg", (width - 2 * WALL_THICKNESS, WALL_THICKNESS, wall_h), (0, -(depth - WALL_THICKNESS) / 2, wall_z))
    box("wall_y_pos", (width - 2 * WALL_THICKNESS, WALL_THICKNESS, wall_h), (0, (depth - WALL_THICKNESS) / 2, wall_z))

    land_radius = STUD_RADIUS - interference
    for stud_index, (x, y) in enumerate(stud_centres(shape)):
        parts.append({"name": f"stud_{stud_index}", "kind": "cylinder", "radius": STUD_RADIUS,
                      "height": STUD_HEIGHT, "centre": [x, y, height / 2 + STUD_HEIGHT / 2],
                      "material": "hard"})
        levels = (
            ("entrance", bottom, bottom + HARD_LEAD_HEIGHT,
             SOCKET_MOUTH_RADIUS, SOCKET_CLEAR_RADIUS, "hard"),
            ("taper", bottom + HARD_LEAD_HEIGHT, bottom + HARD_LEAD_HEIGHT + SOFT_LEAD_HEIGHT,
             SOCKET_CLEAR_RADIUS, land_radius, "compliant"),
            ("land", bottom + HARD_LEAD_HEIGHT + SOFT_LEAD_HEIGHT, roof_bottom,
             land_radius, land_radius, "compliant"),
        )
        for label, z0, z1, inner0, inner1, material in levels:
            for segment in range(segments):
                points, faces = _ring_segment(x, y, z0, z1, inner0, inner1,
                                             SOCKET_OUTER_RADIUS, segment, segments)
                parts.append({"name": f"socket_{stud_index}_{label}_{segment:02d}",
                              "kind": "mesh", "points": points, "faces": faces,
                              "material": material})
    return {
        "model": "custom-16mm-pitch-interference-brick-v1",
        "shape": list(shape), "body_size": [width, depth, height],
        "root": "body centre", "units": "metres", "pitch": PITCH,
        "stud_radius": STUD_RADIUS, "stud_height": STUD_HEIGHT,
        "wall_thickness": WALL_THICKNESS, "roof_thickness": ROOF_THICKNESS,
        "socket_segments": segments, "radial_interference": interference,
        "socket_land_inradius": land_radius, "socket_outer_inradius": SOCKET_OUTER_RADIUS,
        "socket_mouth_inradius": SOCKET_MOUTH_RADIUS,
        "socket_clear_inradius": SOCKET_CLEAR_RADIUS,
        "hard_lead_height": HARD_LEAD_HEIGHT, "soft_lead_height": SOFT_LEAD_HEIGHT,
        "parts": parts,
    }


def generate_brick_usd(path: str | Path, shape: Sequence[int], color: Sequence[float], mass: float, *,
                       stiffness: float = DEFAULT_STIFFNESS, damping: float = DEFAULT_DAMPING,
                       interference: float = DEFAULT_INTERFERENCE, segments: int = DEFAULT_SEGMENTS,
                       static_friction: float = 0.6, dynamic_friction: float = 0.5) -> Path:
    """Write a standalone USD asset with default prim ``/Brick``.

    Call after SimulationApp startup. ``color`` is sRGB; ``mass`` is kilograms.
    Friction and compliance are tunable assumptions. The generated asset carries
    their values as custom metadata. No runtime hooks are needed for attachment.
    """
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    if len(color) != 3 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in color):
        raise ValueError("color must contain three finite sRGB values in [0, 1]")
    for name, value in (("mass", mass), ("stiffness", stiffness), ("damping", damping),
                        ("static_friction", static_friction), ("dynamic_friction", dynamic_friction)):
        if not math.isfinite(value) or value < 0 or (name in ("mass", "stiffness") and value == 0):
            raise ValueError(f"invalid {name}")
    spec = brick_geometry_specs(shape, interference=interference, segments=segments)
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Brick")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateRigidBodyEnabledAttr().Set(True)
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(float(mass))
    body = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    body.CreateSolverPositionIterationCountAttr().Set(32)
    body.CreateSolverVelocityIterationCountAttr().Set(8)
    root.GetPrim().SetCustomDataByKey("brickModel", spec["model"])
    root.GetPrim().SetCustomDataByKey("bodyOrigin", "body centre; studs extend above +Z")
    root.GetPrim().SetCustomDataByKey("physicsAssumptions", json.dumps({
        "radial_interference_m": interference, "stiffness_N_per_m": stiffness,
        "damping_Ns_per_m": damping, "static_friction": static_friction,
        "dynamic_friction": dynamic_friction, "calibrated_to_physical_brick": False,
    }, sort_keys=True))

    materials = {}
    for label in ("hard", "compliant"):
        material = UsdShade.Material.Define(stage, f"/Brick/Materials/{label}")
        physical = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physical.CreateStaticFrictionAttr().Set(float(static_friction))
        physical.CreateDynamicFrictionAttr().Set(float(dynamic_friction))
        physical.CreateRestitutionAttr().Set(0.0)
        px = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        px.CreateFrictionCombineModeAttr().Set("average")
        if label == "compliant":
            px.CreateCompliantContactStiffnessAttr().Set(float(stiffness))
            px.CreateCompliantContactDampingAttr().Set(float(damping))
            px.CreateCompliantContactAccelerationSpringAttr().Set(False)
        materials[label] = material

    look = UsdShade.Material.Define(stage, "/Brick/Materials/look")
    shader = UsdShade.Shader.Define(stage, "/Brick/Materials/look/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    linear_color = tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in color)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(look)

    for part in spec["parts"]:
        prim_path = f"/Brick/Geometry/{part['name']}"
        if part["kind"] == "box":
            geom = UsdGeom.Cube.Define(stage, prim_path)
            geom.CreateSizeAttr().Set(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*part["centre"]))
            geom.AddScaleOp().Set(Gf.Vec3f(*part["size"]))
        elif part["kind"] == "cylinder":
            geom = UsdGeom.Cylinder.Define(stage, prim_path)
            geom.CreateRadiusAttr().Set(part["radius"])
            geom.CreateHeightAttr().Set(part["height"])
            geom.CreateAxisAttr().Set("Z")
            geom.AddTranslateOp().Set(Gf.Vec3d(*part["centre"]))
        else:
            geom = UsdGeom.Mesh.Define(stage, prim_path)
            geom.CreatePointsAttr().Set([Gf.Vec3f(*p) for p in part["points"]])
            geom.CreateFaceVertexCountsAttr().Set([len(face) for face in part["faces"]])
            geom.CreateFaceVertexIndicesAttr().Set([i for face in part["faces"] for i in face])
            geom.CreateSubdivisionSchemeAttr().Set("none")
            UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr().Set("convexHull")
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim()).CreateCollisionEnabledAttr().Set(True)
        collider = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
        collider.CreateContactOffsetAttr().Set(CONTACT_OFFSET)
        collider.CreateRestOffsetAttr().Set(0.0)
        geom.CreateDisplayColorAttr().Set([Gf.Vec3f(*linear_color)])
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(
            materials[part["material"]], UsdShade.Tokens.strongerThanDescendants, "physics"
        )
    stage.GetRootLayer().Save()
    return path


def generate_brick_scad(path: str | Path, shape: Sequence[int], *,
                        interference: float = DEFAULT_INTERFERENCE,
                        segments: int = DEFAULT_SEGMENTS) -> Path:
    """Write matching printable CAD as a union of the same component geometry.

    Cylinders use 96 facets for printing. The USD physics cylinders are analytic;
    ring facets are identical in both. Printer clearance/material behaviour must
    be tested before real use; matching CAD does not calibrate simulator friction.
    """
    spec = brick_geometry_specs(shape, interference=interference, segments=segments)

    def mm(values):
        return "[" + ", ".join(f"{1000 * v:.9g}" for v in values) + "]"

    lines = [
        "// Custom 16 mm pitch construction brick; units: millimetres.",
        "// Generated from geometry.py. Not a certified LEGO/Duplo part.",
        "// Test material, fit and printer tolerances before physical use.",
        f"// Shape: {spec['shape']}; nominal radial interference: {interference * 1000:.6g} mm.",
        "$fn = 96;", "union() {",
    ]
    for part in spec["parts"]:
        lines.append(f"  // {part['name']}; simulation contact: {part['material']}")
        if part["kind"] == "box":
            lines.append(f"  translate({mm(part['centre'])}) cube({mm(part['size'])}, center=true);")
        elif part["kind"] == "cylinder":
            lines.append(f"  translate({mm(part['centre'])}) cylinder(h={part['height'] * 1000:.9g}, r={part['radius'] * 1000:.9g}, center=true);")
        else:
            points = "[" + ", ".join(mm(point) for point in part["points"]) + "]"
            # OpenSCAD uses clockwise outward faces, opposite USD's convention.
            faces = json.dumps([list(reversed(face)) for face in part["faces"]])
            lines.append(f"  polyhedron(points={points}, faces={faces}, convexity=4);")
    lines.append("}")
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    return path
