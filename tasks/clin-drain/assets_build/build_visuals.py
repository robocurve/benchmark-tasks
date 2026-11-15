"""Author the photoreal visual meshes of clin-drain in Blender and export one USD per rigid body.

Run: blender -b --factory-startup --python assets_build/build_visuals.py -- <pkg_dir> <pad_usd> [preview_dir]
  <pad_usd>: sm_consumable_absorbent_pad_stack_a01_01.usd of the NVIDIA SimReady package (README).

Every mesh is in its body's part frame (metres, Z up), so labkit can reference it straight under the body prim:
  drain_body.usda (part body), drain_mid.usda (bellows_mid joint), drain_top.usda (top_plate joint),
  plug.usda, mat.usda. Meshes whose name starts with "glass_" get labkit's glass material in Isaac (the inlet tube
  and the clear film wall) and cast no shadow.
Geometry follows the frozen constants of clin_drain.py (Ø112 mm discs, 76 mm expanded, 20 mm compressed, spout at
(+62, -2) mm, press pad at (-44, -5) mm, plug holder at (+77, -40) mm). The side wall follows the catalogue photo of
the real unit: a clear film, bowed slightly past the discs, with crumpled white film folds behind it. Each moving
body carries one band of it; the bands meet without overlap when expanded and nest when compressed.
The mat is the top surface of the SimReady absorbent-pad stack, resampled on a 2 mm grid to a 260 x 220 x 3 mm sheet.
"""
import math
import random
import sys
from pathlib import Path

import bpy
import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.interpolate import poly_3d_calc
from pxr import Sdf, Usd, UsdGeom

argv = sys.argv[sys.argv.index("--") + 1:]
PKG = Path(argv[0]).resolve()
PAD_USD = Path(argv[1])
PREVIEW = Path(argv[2]) if len(argv) > 2 else None
OUT = PKG / "assets" / "visual"
TEX = OUT / "textures"
MM = 0.001

H_EXP = 76.0
SPOUT = (62.0, -2.0)
PRESS = (-44.0, -5.0)
HOLDER = (77.0, -40.0)
SPRINGS = [(26.0 * math.cos(math.radians(a)), 26.0 * math.sin(math.radians(a))) for a in (60.0, 180.0, 300.0)]

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


# -- materials ------------------------------------------------------------------------------------------------------

def material(name, color, rough, metal=0.0, tex=None, normal=None, spec=0.5, coat=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if "Specular IOR Level" in bsdf.inputs:
        bsdf.inputs["Specular IOR Level"].default_value = spec
    if coat and "Coat Weight" in bsdf.inputs:
        bsdf.inputs["Coat Weight"].default_value = coat
    if tex:
        img = nt.nodes.new("ShaderNodeTexImage")
        img.image = bpy.data.images.load(str(TEX / tex))
        nt.links.new(img.outputs["Color"], bsdf.inputs["Base Color"])
    if normal:
        img = nt.nodes.new("ShaderNodeTexImage")
        img.image = bpy.data.images.load(str(TEX / normal))
        img.image.colorspace_settings.name = "Non-Color"
        nm = nt.nodes.new("ShaderNodeNormalMap")
        nt.links.new(img.outputs["Color"], nm.inputs["Color"])
        nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    return m


def srgb(c):
    return tuple(((x / 255.0 + 0.055) / 1.055) ** 2.4 for x in c)


PP_WHITE = material("pp_white", srgb((232, 232, 226)), 0.38, spec=0.5)
PP_CREAM = material("pp_cream", srgb((238, 236, 226)), 0.34)
TOP_PRINT = material("top_print", (1, 1, 1), 0.32, tex="drain_top_label.png", coat=0.2)
STEEL = material("spring_steel", srgb((205, 208, 212)), 0.22, metal=1.0)
BLUE = material("medical_blue", srgb((26, 92, 176)), 0.35, coat=0.3)
CLEAR = material("clear_pvc", srgb((225, 238, 244)), 0.05)  # replaced by Omniverse glass in Isaac (glass_ prefix)
# White crumpled film seen through the clear wall (catalogue photo). Opaque on purpose: behind thin glass an unlit
# interior rendered as a dark band under real-time RTX (v2 GPU run 20261114T150742Z).
FOLDS = material("film_folds", srgb((228, 232, 234)), 0.3, spec=0.5, coat=0.6)
PAD = material("absorbent_pad", (1, 1, 1), 0.85, tex="pad_albedo.jpg", normal="pad_normal.jpg", spec=0.3)


# -- mesh helpers ---------------------------------------------------------------------------------------------------

GROUPS: dict[str, list] = {k: [] for k in ("mat", "drain_body", "drain_mid", "drain_top", "plug")}


def finish(obj, group, mat, smooth=True, bevel=0.0):
    obj.data.name = obj.name
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new("bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 3
        mod.limit_method = "ANGLE"
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    if smooth:
        for p in obj.data.polygons:
            p.use_smooth = True
        if hasattr(obj.data, "set_sharp_from_angle"):
            obj.data.set_sharp_from_angle(angle=math.radians(40))
    GROUPS[group].append(obj)
    return obj


def lathe(name, profile, group, mat, seg=128, center=(0.0, 0.0), cap=False):
    """Revolve a (r_mm, z_mm) polyline about Z at center (mm)."""
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    rings = []
    for r, z in profile:
        ring = []
        for i in range(seg):
            a = 2 * math.pi * i / seg
            ring.append(bm.verts.new(((center[0] + r * math.cos(a)) * MM, (center[1] + r * math.sin(a)) * MM, z * MM)))
        rings.append(ring)
    for a, b in zip(rings, rings[1:]):
        for i in range(seg):
            j = (i + 1) % seg
            bm.faces.new((a[i], a[j], b[j], b[i]))
    if cap:
        bm.faces.new(list(reversed(rings[0])))
        bm.faces.new(rings[-1])
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    scene.collection.objects.link(obj)
    return finish(obj, group, mat)


def box(name, center, size, group, mat, bevel=0.6, yaw=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=tuple(c * MM for c in center))
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = tuple(s * MM for s in size)
    obj.rotation_euler[2] = yaw
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    return finish(obj, group, mat, smooth=False, bevel=bevel * MM)


def tube(name, pts, radius, group, mat, res=4):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = radius * MM
    cu.bevel_resolution = res
    cu.use_fill_caps = True
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for p, (x, y, z) in zip(sp.points, pts):
        p.co = (x * MM, y * MM, z * MM, 1.0)
    obj = bpy.data.objects.new(name, cu)
    scene.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.convert(target="MESH")
    obj.select_set(False)
    return finish(obj, group, mat)


def spring(name, cx, cy, z0, z1, coil_r, wire_r, turns, group):
    n = int(turns * 48)
    pts = [(cx + coil_r * math.cos(2 * math.pi * turns * i / n), cy + coil_r * math.sin(2 * math.pi * turns * i / n),
            z0 + (z1 - z0) * i / n) for i in range(n + 1)]
    return tube(name, pts, wire_r, group, STEEL, res=3)


def film(name, z0, z1, r_end, r_mid, crumple, group, mat, seed, seg=120, rows=28):
    """Open wall band about Z: radius bows from r_end (both ends) to r_mid (middle); crumple (mm) adds film creases."""
    rng = random.Random(seed)
    waves = [(rng.randint(3, 17), rng.uniform(1.0, 4.0), rng.uniform(0, 2 * math.pi), rng.uniform(0.4, 1.0))
             for _ in range(7)]
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    rings = []
    for k in range(rows + 1):
        t = k / rows
        z = z0 + (z1 - z0) * t
        bow = math.sin(math.pi * t)
        ring = []
        for i in range(seg):
            a = 2 * math.pi * i / seg
            c = sum(w * abs(math.sin(n * a + f * t * math.pi + ph)) for n, f, ph, w in waves) / len(waves)
            r = r_end + (r_mid - r_end) * bow + crumple * bow * (c - 0.45)
            ring.append(bm.verts.new((r * math.cos(a) * MM, r * math.sin(a) * MM, z * MM)))
        rings.append(ring)
    for a, b in zip(rings, rings[1:]):
        for i in range(seg):
            j = (i + 1) % seg
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    scene.collection.objects.link(obj)
    return finish(obj, group, mat)


def wall(tag, z0, z1, group, seed):
    """Clear film band (glass_ prefix) over a band of crumpled white film folds."""
    film(f"glass_{tag}_film", z0, z1, 56.2, 57.6, 0.8, group, CLEAR, seed)
    film(f"{tag}_film_folds", z0, z1, 53.4, 54.2, 3.0, group, FOLDS, seed + 1)


def planar_uv(obj, half_x, half_y, cx=0.0, cy=0.0):
    me = obj.data
    while me.uv_layers:
        me.uv_layers.remove(me.uv_layers[0])
    uv = me.uv_layers.new(name="st")
    for loop in me.loops:
        co = me.vertices[loop.vertex_index].co
        uv.data[loop.index].uv = ((co.x / MM - cx) / (2 * half_x) + 0.5, (co.y / MM - cy) / (2 * half_y) + 0.5)


def disc(name, r, z0, z1, group, mat, edge=1.0):
    prof = [(0.0, z0), (r - edge, z0), (r - edge * 0.3, z0 + edge * 0.3), (r, z0 + edge), (r, z1 - edge),
            (r - edge * 0.3, z1 - edge * 0.3), (r - edge, z1), (0.0, z1)]
    return lathe(name, prof, group, mat)


# -- mat (frame: mat centre, mid-thickness) ---------------------------------------------------------------------------

# The pad stack (407 x 477 x 62 mm) has its centre fold along its y axis at x = 0, a 5 mm deep V crease in the top
# sheet. The mat is the stack's top surface over pad x -30..190 mm, y -130..130 mm, turned +90 deg (mat x = -pad y,
# mat y = pad x - 80 mm): 260 x 220 mm with the crease 30 mm inside the mat's -y edge. It is resampled as a height field
# on a regular 2 mm grid (one downward ray per grid point, so no overhang, hole or cut edge) with the source's own UVs,
# its relief scaled by 0.4 about the median onto the collider top (+1.5 mm), over a flat underside at -1.5 mm.
bpy.ops.wm.usd_import(filepath=str(PAD_USD))
src = max((o for o in scene.objects if o.type == "MESH"), key=lambda o: len(o.data.vertices))
sbm = bmesh.new()
sbm.from_mesh(src.data)
sbm.transform(src.matrix_world)
sbm.faces.ensure_lookup_table()
src_uv = sbm.loops.layers.uv.active
tree = BVHTree.FromBMesh(sbm)
STEP, HX, HY = 2.0, 130.0, 110.0
NX, NY = int(2 * HX / STEP) + 1, int(2 * HY / STEP) + 1
samples = []
for j in range(NY):
    for i in range(NX):
        mx, my = -HX + i * STEP, -HY + j * STEP
        hit, _, idx, _ = tree.ray_cast(Vector(((my + 80.0) * MM, -mx * MM, 0.2)), Vector((0.0, 0.0, -1.0)))
        loops = sbm.faces[idx].loops
        w = poly_3d_calc([lp.vert.co for lp in loops], hit)
        uv = sum((lp[src_uv].uv * wk for lp, wk in zip(loops, w)), Vector((0.0, 0.0)))
        samples.append((mx * MM, my * MM, hit.z, uv.copy()))
sbm.free()
for o in list(scene.objects):
    bpy.data.objects.remove(o, do_unlink=True)
for c in list(bpy.data.collections):  # the importer files meshes under a hidden instancing "prototypes" collection
    bpy.data.collections.remove(c)
z_med = sorted(z for _, _, z, _ in samples)[len(samples) // 2]
bm = bmesh.new()
uvl = bm.loops.layers.uv.new("st")
top = [bm.verts.new((x, y, (z - z_med) * 0.4 + 0.0015)) for x, y, z, _ in samples]
uv_of = {v: s[3] for v, s in zip(top, samples)}
ring = ([j * NX + NX - 1 for j in range(NY)] + [(NY - 1) * NX + i for i in range(NX - 2, -1, -1)]
        + [j * NX for j in range(NY - 2, -1, -1)] + list(range(1, NX - 1)))  # counter-clockwise, starts at (+x, -y)
under = []
for k in ring:
    v = bm.verts.new((top[k].co.x, top[k].co.y, -0.0015))
    uv_of[v] = uv_of[top[k]]
    under.append(v)
faces = [[top[j * NX + i], top[j * NX + i + 1], top[(j + 1) * NX + i + 1], top[(j + 1) * NX + i]]
         for j in range(NY - 1) for i in range(NX - 1)]
faces += [[under[n], under[(n + 1) % len(ring)], top[ring[(n + 1) % len(ring)]], top[ring[n]]] for n in range(len(ring))]
faces.append(under[::-1])
for vs in faces:
    f = bm.faces.new(vs)
    for lp in f.loops:
        lp[uvl].uv = uv_of[lp.vert]
me = bpy.data.meshes.new("mat_absorbent_pad")
bm.to_mesh(me)
bm.free()
m = bpy.data.objects.new("mat_absorbent_pad", me)
scene.collection.objects.link(m)
lo, hi = [min(v.co[k] for v in me.vertices) for k in range(3)], [max(v.co[k] for v in me.vertices) for k in range(3)]
print("mat bounds mm", [round(v * 1000, 2) for v in lo], [round(v * 1000, 2) for v in hi])
finish(m, "mat", PAD, smooth=True)

# -- drain body (frame: bottom-disc centre on the mat) -------------------------------------------------------------

disc("body_bottom_disc", 56.0, 0.0, 4.5, "drain_body", PP_WHITE, edge=1.2)
lathe("body_bottom_rim", [(50.0, 4.5), (51.0, 6.0), (52.2, 6.0), (52.6, 4.5)], "drain_body", PP_WHITE)
box("body_foot", (56.0, -2.0, 2.25), (24.0, 24.0, 4.5), "drain_body", PP_WHITE, bevel=2.0)
box("body_cradle_bridge", (64.0, -26.0, 1.9), (42.0, 28.0, 3.8), "drain_body", PP_WHITE, bevel=1.5)
box("body_cradle_floor", (HOLDER[0], HOLDER[1], 2.0), (30.0, 28.0, 4.0), "drain_body", PP_WHITE, bevel=1.5)
for dx in (-12.0, 12.0):
    box(f"body_cradle_rib{int(dx)}", (HOLDER[0] + dx, HOLDER[1], 6.0), (4.0, 24.0, 4.0), "drain_body", PP_WHITE,
        bevel=1.0)
lathe("body_cradle_socket", [(9.0, 4.0), (9.0, 5.2), (6.0, 5.2), (6.0, 4.0)], "drain_body", PP_CREAM,
      center=HOLDER, seg=64)
box("body_tether_lug", (66.0, -18.0, 6.0), (6.0, 6.0, 4.0), "drain_body", BLUE, bevel=1.0)
wall("body", 4.5, 19.5, "drain_body", 11)
for k, (x, y) in enumerate(SPRINGS):
    lathe(f"body_spring_seat{k}", [(0.0, 4.5), (7.0, 4.5), (7.0, 6.5), (0.0, 6.5)], "drain_body", STEEL,
          center=(x, y), seg=48)
    spring(f"body_spring{k}", x, y, 6.5, 19.5, 5.6, 0.75, 2.5, "drain_body")

# -- bellows_mid (rides 0 .. -28 mm) ---------------------------------------------------------------------------------

wall("mid", 19.5, 47.5, "drain_mid", 21)
for k, (x, y) in enumerate(SPRINGS):
    spring(f"mid_spring{k}", x, y, 19.5, 47.5, 6.0, 0.75, 5.0, "drain_mid")

# -- top_plate (rides on bellows_mid, 0 .. -28 mm more) -----------------------------------------------------------

wall("top", 47.5, 71.5, "drain_top", 31)
for k, (x, y) in enumerate(SPRINGS):
    spring(f"top_spring{k}", x, y, 47.5, 71.5, 6.4, 0.75, 4.0, "drain_top")
top = disc("top_disc", 56.0, 71.5, 76.0, "drain_top", TOP_PRINT, edge=1.2)
planar_uv(top, 56.0, 56.0)
box("top_spout_shoulder", (54.0, SPOUT[1], 73.6), (24.0, 20.0, 4.0), "drain_top", PP_CREAM, bevel=1.5)
box("top_press_lug", (-52.0, -5.0, 73.6), (16.0, 24.0, 4.0), "drain_top", PP_CREAM, bevel=1.5)
lathe("top_spout", [(7.8, 76.0), (10.8, 76.0), (10.8, 83.0), (15.6, 89.2), (15.6, 90.0), (13.4, 90.0),
                    (7.8, 83.0), (7.8, 76.0)], "drain_top", PP_CREAM, center=SPOUT, seg=96)
lathe("top_inlet_port", [(0.0, 76.0), (6.0, 76.0), (6.0, 82.0), (4.2, 84.0), (3.2, 84.0), (3.2, 86.0), (0.0, 86.0)],
      "drain_top", BLUE, center=(0.0, 36.0), seg=64)
tube("glass_top_inlet_tube", [(0.0, 36.0, 84.0), (0.0, 36.0, 89.0), (3.0, 38.5, 92.0), (12.0, 43.0, 93.0),
                              (30.0, 46.0, 92.0)], 2.8, "drain_top", CLEAR)
lathe("top_tube_clamp", [(0.0, -2.0), (3.6, -2.0), (3.6, 2.0), (0.0, 2.0)], "drain_top", BLUE, seg=48)
GROUPS["drain_top"][-1].rotation_euler = (0.0, math.radians(90.0), math.radians(10.0))
GROUPS["drain_top"][-1].location = (22.0 * MM, 45.0 * MM, 92.5 * MM)
box("top_tether_lug", (56.0, -16.0, 74.0), (6.0, 5.0, 4.0), "drain_top", BLUE, bevel=1.0)

# -- plug (frame: stem tip bottom) -----------------------------------------------------------------------------------

lathe("plug_stem", [(0.0, 0.0), (3.0, 0.0), (5.2, 5.5), (5.2, 7.0), (5.6, 7.6), (5.2, 8.4), (5.2, 9.8), (5.6, 10.4),
                    (5.2, 11.2), (5.2, 12.5), (0.0, 12.5)], "plug", PP_CREAM, seg=64)
disc("plug_flange", 13.5, 12.5, 16.0, "plug", BLUE, edge=0.8)
box("plug_tab", (0.0, 0.0, 23.5), (20.0, 6.0, 15.0), "plug", BLUE, bevel=1.8)
for k in range(4):
    box(f"plug_tab_rib{k}", (-6.0 + 4.0 * k, 0.0, 26.0), (1.0, 7.0, 7.0), "plug", BLUE, bevel=0.4)
box("plug_strap_stub", (0.0, -15.5, 14.2), (5.0, 6.0, 1.2), "plug", BLUE, bevel=0.4)

# -- export -----------------------------------------------------------------------------------------------------------

for group, objs in GROUPS.items():
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.ops.wm.usd_export(filepath=str(OUT / f"{group}.usda"), selected_objects_only=True, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          export_uvmaps=True, export_normals=True, root_prim_path="/visual",
                          convert_orientation=False, export_animation=False)
    print("exported", group, len(objs))
    stage = Usd.Stage.Open(str(OUT / f"{group}.usda"))
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and prim.GetName().startswith("glass_"):
            UsdGeom.PrimvarsAPI(prim).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
    stage.GetRootLayer().Save()

if PREVIEW:
    # place the bodies as in the scene (expanded), light + camera, render two views
    for o in GROUPS["mat"]:
        o.location = (0.005, -0.010, -0.0015)
    for o in GROUPS["plug"]:
        o.location = (HOLDER[0] * MM, HOLDER[1] * MM, 0.0045)
    clear = next(n for n in CLEAR.node_tree.nodes if n.type == "BSDF_PRINCIPLED")  # glass, as Isaac draws it
    clear.inputs["Transmission Weight"].default_value = 1.0
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 48
    scene.cycles.device = "CPU"
    scene.render.resolution_x, scene.render.resolution_y = 1280, 704
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.55, 0.56, 0.58, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.8
    scene.world = world
    bpy.ops.mesh.primitive_plane_add(size=2.0, location=(0, 0, -0.0031))
    table = bpy.context.active_object
    table.data.materials.append(material("table", srgb((180, 180, 176)), 0.6))
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = (math.radians(35), math.radians(15), math.radians(30))
    scene.collection.objects.link(sun)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    PREVIEW.mkdir(parents=True, exist_ok=True)
    for tag, loc, rot, lens in (("top", (0.02, -0.01, 0.55), (0, 0, math.radians(-90)), 50),
                                ("persp", (0.33, -0.30, 0.26), (math.radians(62), 0, math.radians(48)), 50)):
        cam.location, cam.rotation_euler, cam.data.lens = loc, rot, lens
        scene.render.filepath = str(PREVIEW / f"preview_{tag}.png")
        bpy.ops.render.render(write_still=True)
