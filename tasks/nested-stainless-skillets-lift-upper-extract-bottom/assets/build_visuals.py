"""Rebuild the three visual USDs with background Blender 5.2.

Run from the package root with the Arena checkout as the last argument:
    blender -b --factory-startup --python-exit-code 1 --python assets/build_visuals.py -- ARENA_CHECKOUT
If yam_arena is already importable in Blender, omit the last two arguments.
"""

from __future__ import annotations

from collections import Counter
from math import cos, pi, sin
from pathlib import Path
import struct
import sys
import zlib

import bpy
import numpy as np
from pxr import Sdf, Usd, UsdGeom

PACKAGE = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
if "--" in sys.argv and sys.argv.index("--") + 1 < len(sys.argv):
    sys.path.insert(0, sys.argv[sys.argv.index("--") + 1])
sys.path.insert(0, str(PACKAGE))
import nested_stainless_skillets_lift_upper_extract_bottom as task

SEGMENTS = 128
ANGLES = np.linspace(0, 2 * pi, SEGMENTS, endpoint=False)


def png(path, pixels):
    """Write an 8-bit RGB PNG without ambient color-management settings."""
    height, width, _ = pixels.shape

    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data)))

    rows = b"".join(b"\0" + row.tobytes() for row in pixels.astype(np.uint8))
    path.write_bytes(b"\x89PNG\r\n\x1a\n"
                     + chunk(b"IHDR", struct.pack(">2I5B", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(rows, 9))
                     + chunk(b"IEND", b""))


def make_textures(folder):
    folder.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(41205)
    rows, columns = 512, 768
    # Fine brush lines follow U, with a slow change of strength along each line.
    grain = rng.normal(0, 0.65, (rows, 1))
    knots = rng.normal(0, 0.93, (rows, 12))
    steps = np.arange(columns) * (knots.shape[1] / columns)
    i = steps.astype(int)
    phase = (1 - np.cos(pi * (steps - i))) / 2
    grain = grain + knots[:, i] * (1 - phase) + knots[:, (i + 1) % knots.shape[1]] * phase
    grain = np.clip(grain + rng.normal(0, 0.11, (rows, columns)), -2.3, 2.3)
    png(folder / "steel_grain.png", np.array([194, 197, 199]) + grain[..., None] * 2)
    png(folder / "steel_roughness.png", np.repeat((85 + grain[..., None] * 7), 3, axis=2))

    # Guides are printed into the non-slip surface, with no collision geometry.
    u, v = np.meshgrid(np.arange(512) / 512, np.arange(512) / 512)
    dots = np.maximum(0, np.cos(2 * pi * 104 * u) * np.cos(2 * pi * 140 * v)) ** 3
    surface = rng.normal(0, 0.7, dots.shape) - 5 * dots
    color = np.array([236, 237, 230]) + surface[..., None]
    x, y = (u - 0.5) * task.MAT_SIZE[0], (0.5 - v) * task.MAT_SIZE[1]
    for spot, radius in ((task.UPPER_SPOT, task.UPPER_RAD), (task.LOWER_SPOT, task.LOWER_RAD)):
        cx, cy = spot - task.MAT_CENTER
        dx, dy = x - cx, y - cy
        ring = np.abs(np.hypot(dx, dy) - radius) < 0.0025
        cross = (np.minimum(np.abs(dx), np.abs(dy)) < 0.0025) & (np.hypot(dx, dy) < 0.012)
        color[ring | cross] = np.array([45, 133, 122]) + surface[ring | cross, None]
    png(folder / "mat_grain.png", color)


def material(name, color, roughness, metal=0.0, color_file=None, roughness_file=None):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    shader = next(node for node in nodes if node.type == "BSDF_PRINCIPLED")
    shader.inputs["Base Color"].default_value = (*color, 1.0)
    shader.inputs["Metallic"].default_value = metal
    shader.inputs["Roughness"].default_value = roughness
    mat.diffuse_color = (*color, 1.0)
    for socket, path, colorspace in (
        ("Base Color", color_file, "sRGB"),
        ("Roughness", roughness_file, "Non-Color"),
    ):
        if path is None:
            continue
        image = bpy.data.images.load(str(path), check_existing=True)
        image.colorspace_settings.name = colorspace
        texture = nodes.new("ShaderNodeTexImage")
        texture.image = image
        uv = nodes.new("ShaderNodeUVMap")
        uv.uv_map = "st"
        links.new(uv.outputs["UV"], texture.inputs["Vector"])
        links.new(texture.outputs["Color"], shader.inputs[socket])
    return mat


def ring_mesh(name, rings, materials, bands, uv_path, flat_bands=(), longitudinal_uv=False):
    """Join closed rings from the first pole to the last, preserving the UV seam."""
    starts = np.cumsum([0, *(len(ring) for ring in rings[:-1])])
    points = [point for ring in rings for point in ring]
    faces, texcoords, face_bands = [], [], []
    for band, (a, b) in enumerate(zip(rings[:-1], rings[1:])):
        count = max(len(a), len(b))

        def entry(row, index):
            ring = rings[row]
            local = index % len(ring)
            angular = (index if len(ring) > 1 else index + 0.5) / count
            uv = (uv_path[row], angular) if longitudinal_uv else (angular, uv_path[row])
            return int(starts[row]) + local, uv

        for j in range(count):
            indices = [(band, j), (band, j + 1), (band + 1, j + 1), (band + 1, j)]
            if len(a) == 1:
                del indices[1]
            if len(b) == 1:
                del indices[2]
            face, uv = zip(*(entry(row, k) for row, k in indices))
            faces.append(face)
            texcoords.append(uv)
            face_bands.append(band)

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(points, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    for mat in materials:
        mesh.materials.append(mat)
    uv_layer = mesh.uv_layers.new(name="st")
    for poly, uv, band in zip(mesh.polygons, texcoords, face_bands):
        poly.use_smooth = band not in flat_bands
        poly.material_index = bands[band]
        for loop, pair in zip(poly.loop_indices, uv):
            uv_layer.uv[loop].vector = pair
    return obj


def collider_bounds(boxes):
    centers = np.array([box.at for box in boxes])
    half = np.array([np.abs(task.mk.quat_to_mat(box.quat)) @ (np.array(box.size) / 2)
                     for box in boxes])
    return (centers - half).min(axis=0), (centers + half).max(axis=0)


def bowl(part, steel, lip, foot):
    sole_boxes = [box for box in part.boxes[:-2] if np.allclose(box.quat, (0, 0, 0, 1))]
    floor_lo, floor_hi = collider_bounds(sole_boxes)
    body_lo, body_hi = collider_bounds(part.boxes[:-2])
    radius, ry = (body_hi[:2] - body_lo[:2]) / 2
    cx, cy = (body_hi[:2] + body_lo[:2]) / 2
    sole = (floor_hi[1] - floor_lo[1]) / 2
    bottom, floor, rim = body_lo[2], floor_hi[2], body_hi[2]
    flare, rise = radius - sole, rim - floor
    wall = min(0.0022, flare * 0.13, rise * 0.1)
    heel = min(0.0011, (floor - bottom) * 0.19)

    # A spun section with a flat heat spreader, a smooth flared wall, and a rolled lip.
    profile = [
        (0, bottom), (0.87 * sole, bottom), (0.972 * sole, bottom),
        (sole, bottom + heel * 0.35), (sole + heel * 0.7, bottom + heel),
        (sole + heel * 0.9, floor - heel * 1.4),
        (sole + heel, floor - heel * 0.85),
        (sole + 0.10 * flare, floor + 0.06 * rise),
        (sole + 0.26 * flare, floor + 0.23 * rise),
        (sole + 0.55 * flare, floor + 0.53 * rise),
        (sole + 0.83 * flare, floor + 0.81 * rise),
        (radius - 0.47 * wall, rim - 0.88 * wall),
        (radius - 0.13 * wall, rim - 0.70 * wall),
        (radius, rim - 0.43 * wall),
        (radius - 0.08 * wall, rim - 0.16 * wall),
        (radius - 0.33 * wall, rim),
        (radius - 0.60 * wall, rim - 0.04 * wall),
        (radius - 0.85 * wall, rim - 0.23 * wall),
        (radius - 1.12 * wall, rim - 0.56 * wall),
        (radius - 1.35 * wall, rim - 0.96 * wall),
        (sole + 0.83 * flare - wall, floor + 0.81 * rise),
        (sole + 0.55 * flare - wall, floor + 0.53 * rise),
        (sole + 0.26 * flare - wall, floor + 0.25 * rise),
        (sole + 0.12 * flare - wall, floor + 0.09 * rise),
        (sole + 0.01 * flare, floor + 0.021 * rise),
        (sole - 0.06 * flare, floor), (0.85 * sole, floor), (0, floor),
    ]
    section = np.array(profile)
    v = np.r_[0, np.cumsum(np.linalg.norm(np.diff(section, axis=0), axis=1))]
    rings = [[(cx, cy, z)] if radius_at_z == 0 else
             [(cx + radius_at_z * cos(a), cy + radius_at_z * ry / radius * sin(a), z)
              for a in ANGLES]
             for radius_at_z, z in profile]
    # Separate materials let the crest catch a highlight and the foot read as a clad disc.
    bands = [2] * 6 + [0] * 6 + [1] * 7 + [0] * 8
    obj = ring_mesh("spun_bowl", rings, (steel, lip, foot), bands, v * 7.0)
    outer = section[6:14]
    inner = section[18:26][::-1]
    return obj, radius, outer, inner, rim - 0.19 * rise, rise, wall


def grip(part, radius, outer, mount_z, wall, satin, fitting):
    bolster, bar = part.boxes[-2:]
    lo, hi = collider_bounds((bar,))
    (_, center_y, bar_z) = bar.at
    mid_z = bolster.at[2]
    half_y, half_z = np.array(bar.size)[1:] / 2
    mount_half, mount_thickness = np.array(bolster.size)[1:] / 2
    tip_round = min(half_y * 0.65, bar.size[0] * 0.12)
    joint = lo[0] + min(0.002, 0.15 * bar.size[0])
    # The first sections are offsets from the curved exterior wall at the rivets.
    # The straight sections carry those shoulders into a flattened, rounded grip.
    sections = [
        (-0.32 * wall, mount_z, 0, 0),
        (-0.32 * wall, mount_z, mount_half * 0.81, mount_thickness * 0.80),
        (0.78 * wall, mount_z + 0.001, mount_half * 0.86, mount_thickness * 0.85),
        (min(radius + 0.006, joint - 0.003), mid_z - 0.0015, mount_half * 0.74, mount_thickness * 0.93),
        (joint - 0.0014, (mid_z + bar_z) / 2, half_y * 0.86, half_z * 0.79),
        (joint + 0.002, bar_z, half_y * 0.94, half_z * 0.91),
        (joint + min(0.012, bar.size[0] * 0.14), bar_z, half_y, half_z),
        (hi[0] - 2 * tip_round, bar_z, half_y, half_z),
        (hi[0] - tip_round, bar_z, half_y * 0.95, half_z),
        (hi[0] - 0.32 * tip_round, bar_z, half_y * 0.83, half_z * 0.93),
        (hi[0] - 0.07 * tip_round, bar_z, half_y * 0.66, half_z * 0.77),
        (hi[0], bar_z, half_y * 0.51, half_z * 0.60),
        (hi[0], bar_z, 0, 0),
    ]
    # A round-rectangle section keeps the silhouette close to the handle collider.
    angles = ANGLES[::4]
    across = np.sign(np.cos(angles)) * np.sqrt(np.abs(np.cos(angles)))
    above = np.sign(np.sin(angles)) * np.sqrt(np.abs(np.sin(angles)))
    rings = []
    for index, (x, z, hy, hz) in enumerate(sections):
        offsets = zip(across, above) if hy else [(0, 0)]
        ring = []
        for dy, dz in offsets:
            y, height = center_y + dy * hy, z + dz * hz
            edge = 0.0
            if index < 3:
                contour = np.interp(height, outer[:, 1], outer[:, 0])
                edge = np.sqrt(contour**2 - y**2)
            ring.append((x + edge, y, height))
        rings.append(ring)
    start = rings[0][0][0]
    return ring_mesh("forged_handle", rings, (fitting, satin), [0] * 5 + [1] * 7,
                     [(np.mean([v[0] for v in ring]) - start) * 3.6 for ring in rings],
                     longitudinal_uv=True)


def rivet(name, y, inner, anchor_z, rise, wall, polish):
    mount_r = np.interp(anchor_z, inner[:, 1], inner[:, 0])
    radial_y = y / mount_r
    radial_x = np.sqrt(1 - radial_y**2)
    local_slope = (inner[-1, 0] - inner[0, 0]) / (inner[-1, 1] - inner[0, 1])
    axis = np.array([-radial_x, -radial_y, local_slope])
    axis /= np.linalg.norm(axis)
    center = np.array([mount_r * radial_x, y, anchor_z])
    side = np.array([radial_y, -radial_x, 0])
    up = np.cross(axis, side)
    r = min(0.0032, 0.125 * rise)
    thickness = min(0.0012, 0.60 * wall)
    profile = [(0, -0.0008), (0.93 * r, -0.0008), (r, -0.00005),
               (0.89 * r, 0.53 * thickness), (0.60 * r, 0.90 * thickness),
               (0.26 * r, thickness), (0, thickness)]
    rings = [[center + axis * h] if rad == 0 else
             [center + axis * h + rad * (cos(a) * side + sin(a) * up) for a in ANGLES[::4]]
             for rad, h in profile]
    return ring_mesh(name, rings, (polish,), [0] * 6, np.linspace(0, 1, len(rings)))


def prep_mat(part, aqua, white):
    cx, cy, cz = part.centre
    hx, hy, hz = (part.hi - part.lo) / 2
    corner = min(0.0075, 0.065 * min(hx, hy))
    bevel = min(0.00023, 0.40 * hz)
    margin = max(corner + bevel, 0.046 * min(hx, hy))
    sections = [
        (0, -hz, 0), (bevel, -hz, corner - bevel),
        (0.3 * bevel, -hz + 0.3 * bevel, corner - 0.3 * bevel),
        (0, -hz + bevel, corner), (0, hz - bevel, corner),
        (0.3 * bevel, hz - 0.3 * bevel, corner - 0.3 * bevel),
        (bevel, hz, corner - bevel), (margin, hz, corner),
        (0, hz, 0),
    ]

    def mat_ring(inset, z, radius):
        if not radius:
            return [(cx, cy, cz + z)]
        return [(cx + sx * (hx - inset - radius) + radius * cos(a),
                 cy + sy * (hy - inset - radius) + radius * sin(a),
                 cz + z)
                for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1)))
                for a in np.linspace(i * pi / 2, (i + 1) * pi / 2, 13)]

    rings = [mat_ring(*section) for section in sections]
    obj = ring_mesh("silicone_mat", rings, (aqua, white), [0] * 7 + [1],
                    np.zeros(len(rings)), flat_bands=(0, 6, 7))
    uv_layer = obj.data.uv_layers.active
    for loop in obj.data.loops:
        vertex = obj.data.vertices[loop.vertex_index].co
        uv_layer.uv[loop.index].vector = ((vertex.x - cx + hx) / (2 * hx),
                                          (vertex.y - cy + hy) / (2 * hy))
    return [obj]


def export(part, objects):
    for obj in bpy.context.selected_objects:
        obj.select_set(False)
    for obj in objects:
        obj.select_set(True)
        # Closed surface with no unconnected vertices used to pad its extent.
        edges = Counter(edge for poly in obj.data.polygons for edge in poly.edge_keys)
        if set(edges.values()) != {2} or len(obj.data.vertices) != len({v for e in edges for v in e}):
            raise ValueError(f"{obj.name} is not a closed surface")
    path = PACKAGE / part.visual
    bpy.ops.wm.usd_export(
        filepath=str(path), root_prim_path=f"/{part.name}", selected_objects_only=True,
        export_materials=True, export_mesh_colors=False, export_uvmaps=True, export_normals=True,
        export_textures_mode="KEEP", relative_paths=True, generate_preview_surface=True,
        export_animation=False, author_blender_name=False, export_custom_properties=False,
        convert_orientation=False, convert_scene_units="METERS",
    )
    order_specs(path)
    check_export(path, part)


def order_specs(path):
    """Make rebuilt crates independent of Blender's object traversal order."""
    source = Sdf.Layer.FindOrOpen(str(path))
    ordered = Sdf.Layer.CreateAnonymous(".usdc")

    def sorted_children(field, layer, location, exists, *_):
        if not exists or field not in ("primChildren", "propertyChildren"):
            return True
        spec = layer.GetObjectAtPath(location)
        children = spec.nameChildren if field == "primChildren" else spec.properties
        names = sorted(children.keys())
        return True, names, names

    root = Sdf.Path.absoluteRootPath
    if not Sdf.CopySpec(source, root, ordered, root, lambda *_: True, sorted_children) or not ordered.Export(str(path)):
        raise ValueError(f"could not order specs in {path.name}")


def check_export(path, part):
    stage = Usd.Stage.Open(str(path))
    root = stage.GetDefaultPrim()
    if root.GetName() != part.name or UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise ValueError(f"unexpected default prim or axis for {path.name}")
    cache = UsdGeom.XformCache()
    points = []
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            transform = cache.GetLocalToWorldTransform(prim)
            points.extend(tuple(transform.Transform(point))
                          for point in UsdGeom.Mesh(prim).GetPointsAttr().Get())
        for attr in prim.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                asset = attr.Get()
                if asset and (Path(asset.path).is_absolute() or not asset.resolvedPath):
                    raise ValueError(f"unresolved or absolute asset reference {asset.path}")
    points = np.array(points)
    lo, hi = points.min(axis=0), points.max(axis=0)
    error_center = np.max(np.abs((lo + hi) / 2 - part.centre)) * 1000
    error_size = np.max(np.abs((hi - lo) - (part.hi - part.lo))) * 1000
    if max(error_center, error_size) >= 0.0005:
        raise ValueError(f"{path.name} bounds differ by {error_center:.6f}, {error_size:.6f} mm")
    print(f"{path.name} center {error_center:.3f} mm, extents {error_size:.3f} mm")


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.scene.unit_settings.system = "METRIC"
    materials_dir = PACKAGE / Path(task.kit.parts[task.MAT].visual).parent / "textures"
    make_textures(materials_dir)
    grain = materials_dir / "steel_grain.png"
    rough = materials_dir / "steel_roughness.png"
    steel = material("circular_brushed_steel", (0.54, 0.56, 0.58), 0.34, 0.92, grain, rough)
    lip = material("polished_steel", (0.64, 0.66, 0.68), 0.20, 0.92)
    foot = material("bonded_steel_base", (0.43, 0.45, 0.47), 0.33, 0.86)
    satin = material("longitudinal_brushed_steel", (0.54, 0.56, 0.58), 0.34, 0.88, grain, rough)
    fitting = material("forged_steel", (0.37, 0.39, 0.41), 0.35, 0.85)
    aqua = material("turquoise_silicone", (0.023, 0.30, 0.26), 0.67)
    white = material("ivory_silicone", (0.84, 0.85, 0.79), 0.69,
                     color_file=materials_dir / "mat_grain.png")

    for name in (task.LOWER, task.UPPER):
        part = task.kit.parts[name]
        body, radius, outer, inner, mount_z, rise, wall = bowl(part, steel, lip, foot)
        handle = grip(part, radius, outer, mount_z, wall, satin, fitting)
        rivet_offset = part.boxes[-2].size[1] * 0.25
        rivets = [rivet(f"round_rivet_{side}", y, inner, mount_z, rise, wall, lip)
                  for side, y in (("left", -rivet_offset), ("right", rivet_offset))]
        export(part, [body, handle, *rivets])

    part = task.kit.parts[task.MAT]
    export(part, prep_mat(part, aqua, white))


if __name__ == "__main__":
    main()
