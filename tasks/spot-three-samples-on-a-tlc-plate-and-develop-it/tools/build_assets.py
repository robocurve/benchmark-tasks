"""Regenerate ``rdb_tlc_spotting/tlc.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs only the Python standard library.

``/Fixture/Actors`` holds the loose rigid bodies, each moved only by contact and gravity:
- ``plate``: the glass-backed TLC plate, one box collider (glass and layer); drawn: glass, white layer, the pencil
  baseline and three ticks, and hidden spot and gouge marks that the adapter shows as the rules make them;
- ``spotter_1`` .. ``spotter_3``: the capillaries, square 1.5 mm colliders 100 mm long; drawn: a glass tube and
  hidden sample columns at the tip, shown once a spotter is filled;
- ``tank``: the rectangular developing tank, a floor and four walls (boxes); drawn: flat glass panes and 5 mm of water;
- ``lid``: its flat glass lid, one box.
``/Fixture/Static`` holds fixed colliders (the bar, the capillary rack, the vial block and the three vials), drawn as
they are, with the samples drawn in the vials. Rigid colliders only (boxes and convex hulls); glass is drawn as smooth
surfaces of revolution with no collision.
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_tlc_spotting import fixed_scene as scene  # noqa: E402
from rdb_tlc_spotting import layout as L  # noqa: E402

# colour (linear RGB), roughness, metallic, opacity, friction
MATERIALS = {
    "glass": ((.90, .95, .97), .02, 0., .10, .5),           # clear borosilicate (two drawn surfaces)
    "plate_glass": ((.62, .72, .70), .10, 0., .85, .6),
    "silica": ((.93, .93, .90), .9, 0., 1., .6),
    "pencil": ((.16, .16, .17), .8, 0., 1., .6),
    "aluminium": ((.70, .71, .73), .35, 1., 1., .5),
    "foam": ((.10, .11, .12), .95, 0., 1., .8),
    "cap_glass": ((.80, .88, .92), .03, 0., .45, .6),
    "solvent": ((.55, .70, .85), .05, 0., .35, .5),
    "red": ((.70, .03, .04), .3, 0., .85, .5),
    "blue": ((.03, .10, .70), .3, 0., .85, .5),
    "green": ((.04, .50, .06), .3, 0., .85, .5),
    "mixed": ((.30, .18, .08), .3, 0., .9, .5),
    "spot_red": ((.85, .30, .32), .9, 0., 1., .6),
    "spot_blue": ((.30, .38, .85), .9, 0., 1., .6),
    "spot_green": ((.30, .70, .32), .9, 0., 1., .6),
    "spot_mixed": ((.45, .32, .22), .9, 0., 1., .6),
    "bare": ((.45, .55, .55), .1, 0., 1., .6),
}
COLOURS = (*L.DYES, "mixed")
SPOT_SIZES = (.00125, .00175, .00225, .00275, .00325, .00375)     # drawn spot diameters (the nearest is shown)
PREFIX = "</Fixture/Materials/"


def _f(v):
    return f"{v:.6g}"


def _v3(p):
    return f"({_f(p[0])}, {_f(p[1])}, {_f(p[2])})"


# -- prims --------------------------------------------------------------------------------------------------------
def shape(kind, name, material, centre, *, collide, visible=True, orient=None, scale_=None, attrs=(), pad="            ",
          offset=.001):
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    text = f'{pad}def {kind} "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
    text += f"{pad}    rel material:binding = {PREFIX}{material}>\n"
    if collide:
        text += (f"{pad}    rel material:binding:physics = {PREFIX}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = {_f(offset)}\n"
                 f"{pad}    float physxCollision:restOffset = 0\n")
    if not visible:
        text += f'{pad}    token visibility = "invisible"\n'
    for line in attrs:
        text += f"{pad}    {line}\n"
    ops = ['"xformOp:translate"']
    text += f"{pad}    double3 xformOp:translate = {_v3(centre)}\n"
    if orient is not None:
        text += f"{pad}    quatf xformOp:orient = ({', '.join(f'{v:.7g}' for v in orient)})\n"
        ops.append('"xformOp:orient"')
    if scale_ is not None:
        text += f"{pad}    float3 xformOp:scale = {_v3(scale_)}\n"
        ops.append('"xformOp:scale"')
    return text + f"{pad}    uniform token[] xformOpOrder = [{', '.join(ops)}]\n{pad}}}\n\n"


def box(name, material, centre, half, *, collide=True, **kw):
    return shape("Cube", name, material, centre, collide=collide, scale_=half, attrs=("double size = 2",), **kw)


def span(name, material, x, y, z, **kw):
    return box(name, material, tuple((a + b) / 2 for a, b in (x, y, z)), tuple((b - a) / 2 for a, b in (x, y, z)), **kw)


def cylinder(name, material, centre, radius, height, *, collide=False, **kw):
    return shape("Cylinder", name, material, centre, collide=collide,
                 attrs=('uniform token axis = "Z"', f"double height = {_f(height)}", f"double radius = {_f(radius)}"), **kw)


def hull(name, material, points, *, visible=False, pad="            ", offset=.001):
    """A convex-hull collider from points (its faces are a fan, it is not drawn)."""
    n = len(points)
    faces = [[0, i, i + 1] for i in range(1, n - 1)]
    vis = "" if visible else f'{pad}    token visibility = "invisible"\n'
    return (f'{pad}def Mesh "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {PREFIX}{material}>\n"
            f"{pad}    rel material:binding:physics = {PREFIX}{material}>\n"
            f'{pad}    uniform token physics:approximation = "convexHull"\n'
            f"{pad}    int physxConvexHullCollision:hullVertexLimit = 64\n"
            f"{pad}    float physxCollision:contactOffset = {_f(offset)}\n"
            f"{pad}    float physxCollision:restOffset = 0\n" + vis +
            f"{pad}    int[] faceVertexCounts = [{', '.join('3' for _ in faces)}]\n"
            f"{pad}    int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"{pad}    point3f[] points = [{', '.join(_v3(p) for p in points)}]\n"
            f'{pad}    uniform token subdivisionScheme = "none"\n{pad}}}\n\n')


def revolve(name, material, profile, *, segments=48, centre=(0., 0., 0.), pad="            "):
    """A drawn surface of revolution about z from an (r, z) polyline. No collision."""
    points, faces = [], []
    m = len(profile)
    for k in range(segments):
        a = 2 * math.pi * k / segments
        points += [(centre[0] + r * math.cos(a), centre[1] + r * math.sin(a), centre[2] + z) for r, z in profile]
    for k in range(segments):
        k2 = (k + 1) % segments
        faces += [[k * m + j, k2 * m + j, k2 * m + j + 1, k * m + j + 1] for j in range(m - 1)]
    return (f'{pad}def Mesh "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {PREFIX}{material}>\n{pad}    uniform bool doubleSided = 1\n"
            f"{pad}    int[] faceVertexCounts = [{', '.join('4' for _ in faces)}]\n"
            f"{pad}    int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"{pad}    point3f[] points = [{', '.join(_v3(p) for p in points)}]\n"
            f'{pad}    uniform token subdivisionScheme = "none"\n{pad}}}\n\n')


def ring(name, material, centre, r_in, r_out, z0, z1, segments, *, pad="            ", visible=False, offset=.001):
    """A ring of flat box segments (a cylinder wall), as colliders."""
    text = ""
    mid = (r_in + r_out) / 2
    width = 2 * math.pi * r_out / segments * 1.04
    for k in range(segments):
        a = 2 * math.pi * (k + .5) / segments
        c = (centre[0] + mid * math.cos(a), centre[1] + mid * math.sin(a), (z0 + z1) / 2)
        q = (math.cos(a / 2), 0., 0., math.sin(a / 2))                    # turn about z: local x radial
        text += shape("Cube", f"{name}_{k:02d}", material, c, collide=True, visible=visible, orient=q,
                      scale_=((r_out - r_in) / 2, width / 2, (z1 - z0) / 2), attrs=("double size = 2",), pad=pad,
                      offset=offset)
    return text


def polygon_prism(name, material, centre, radius, z0, z1, segments=16, **kw):
    pts = [(centre[0] + radius * math.cos(2 * math.pi * k / segments), centre[1] + radius * math.sin(2 * math.pi * k / segments), z)
           for z in (z0, z1) for k in range(segments)]
    return hull(name, material, pts, **kw)


def material(name):
    color, roughness, metallic, opacity, friction = MATERIALS[name]
    extra = f"                float inputs:opacity = {opacity}\n" if opacity < 1 else ""
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n" + extra +
            "                token outputs:surface\n            }\n        }\n\n")


GLASS_LOOK = """        def Material "glass_look"
        {
            token outputs:mdl:surface.connect = </Fixture/Materials/glass_look/Shader.outputs:out>
            token outputs:mdl:volume.connect = </Fixture/Materials/glass_look/Shader.outputs:out>

            def Shader "Shader"
            {
                uniform token info:implementationSource = "sourceAsset"
                uniform asset info:mdl:sourceAsset = @OmniGlass.mdl@
                uniform token info:mdl:sourceAsset:subIdentifier = "OmniGlass"
                color3f inputs:glass_color = (0.97, 0.99, 1)
                float inputs:glass_ior = 1.47
                bool inputs:thin_walled = 1
                token outputs:out
            }
        }

"""    # drawn glass only (Isaac Sim's own OmniGlass MDL, so the plate shows through the tank); colliders use "glass"


def rigid_body(name, mass, position, children, *, com=None):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    text = (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            f"            float physics:mass = {_f(mass)}\n"
            "            float physxContactReport:threshold = 0\n")
    if com is not None:
        text += f"            point3f physics:centerOfMass = {_v3(com)}\n"
    return (text + "            bool physics:rigidBodyEnabled = 1\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            "            quatf xformOp:orient = (1, 0, 0, 0)\n"
            f"            double3 xformOp:translate = {_v3(position)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


# -- the bodies ----------------------------------------------------------------------------------------------------
def plate_parts():
    hl, hw, ht = L.PLATE_LEN / 2, L.PLATE_WIDE / 2, L.PLATE_T / 2
    parts = box("collider", "plate_glass", (0., 0., 0.), (hl, hw, ht), visible=False)
    glass_top = ht - L.LAYER_T
    parts += span("glass", "plate_glass", (-hl, hl), (-hw, hw), (-ht, glass_top), collide=False)
    parts += span("layer", "silica", (-hl, hl), (-hw, hw), (glass_top, ht), collide=False)
    x = -hl + L.BASELINE_UP
    lift = ht + .00002
    parts += span("baseline", "pencil", (x - .00015, x + .00015), (-hw + .001, hw - .001), (ht, lift), collide=False)
    for k in range(3):
        tx, ty, _ = L.tick_point(k)
        parts += span(f"tick_{k + 1}", "pencil", (tx - L.TICK_LEN / 2, tx + L.TICK_LEN / 2), (ty - .00015, ty + .00015),
                      (ht, lift), collide=False)
        for c in COLOURS:
            for s, d in enumerate(SPOT_SIZES):
                parts += cylinder(f"spot_{k + 1}_{c}_{s}", f"spot_{c}", (tx, ty, ht + .00004), d / 2, .00004,
                                  visible=False)
        parts += span(f"bare_{k + 1}", "bare", (tx - .00075, tx + .00075), (ty - .0002, ty + .0002), (ht, ht + .00006),
                      collide=False, visible=False)
    return parts


def spotter_parts():
    h = L.CAP_LEN / 2
    parts = box("collider", "cap_glass", (0., 0., 0.), (L.CAP_OD / 2, L.CAP_OD / 2, h), visible=False, offset=.0005)
    parts += cylinder("tube", "cap_glass", (0., 0., 0.), L.CAP_OD / 2, L.CAP_LEN)
    for c in COLOURS:
        parts += cylinder(f"load_{c}", c, (0., 0., -h + .004), L.CAP_ID / 2 + .0001, .008, visible=False)
    return parts


def tank_parts():
    """The tank in its own frame (origin on the table under its middle): a floor and four glass walls, colliders and
    drawn alike (flat panes, so the plate shows through), and 5 mm of water (drawn only)."""
    (dx, dy, h), f = L.TANK, L.TANK_FLOOR
    ix, iy = L.TANK_IN
    pieces = (("floor", (-dx / 2, dx / 2), (-dy / 2, dy / 2), (0., f)),
              ("front_wall", (-dx / 2, -ix / 2), (-dy / 2, dy / 2), (f, h)),
              ("back_wall", (ix / 2, dx / 2), (-dy / 2, dy / 2), (f, h)),
              ("left_wall", (-ix / 2, ix / 2), (iy / 2, dy / 2), (f, h)),
              ("right_wall", (-ix / 2, ix / 2), (-dy / 2, -iy / 2), (f, h)))
    parts = "".join(span(name, "glass", x, y, z, visible=False) for name, x, y, z in pieces)
    parts += "".join(span(f"pane_{name}", "glass_look", x, y, z, collide=False) for name, x, y, z in pieces)
    parts += span("solvent", "solvent", (-ix / 2 + .0002, ix / 2 - .0002), (-iy / 2 + .0002, iy / 2 - .0002),
                  (f, L.SOLVENT_Z), collide=False)
    return parts


def lid_parts():
    lx, ly, lz = L.LID
    return (box("collider", "glass", (0., 0., 0.), (lx / 2, ly / 2, lz / 2), visible=False)
            + box("pane", "glass_look", (0., 0., 0.), (lx / 2, ly / 2, lz / 2), collide=False))


# -- the fixed parts -----------------------------------------------------------------------------------------------
def holed_block(prefix, material, cx, y0, y1, height, holes, hole, floor):
    """A block with square holes along y at ``holes``: two long slabs, spacers between the holes, a floor."""
    pad = "        "
    half_x = .015
    text = span(f"{prefix}_front", material, (cx - half_x, cx - hole / 2), (y0, y1), (0., height), pad=pad)
    text += span(f"{prefix}_back", material, (cx + hole / 2, cx + half_x), (y0, y1), (0., height), pad=pad)
    edges = [y0] + [v for y in sorted(holes) for v in (y - hole / 2, y + hole / 2)] + [y1]
    for i in range(0, len(edges), 2):
        text += span(f"{prefix}_spacer_{i // 2}", material, (cx - hole / 2, cx + hole / 2), (edges[i], edges[i + 1]),
                     (0., height), pad=pad)
    if floor:
        text += span(f"{prefix}_floor", material, (cx - hole / 2, cx + hole / 2), (y0, y1), (0., floor), pad=pad)
    return text


def vial(k):
    pad = "        "
    x, y = L.VIAL_X, L.VIAL_Y[k]
    neck = L.VIAL_H - L.VIAL_NECK_H
    text = ring(f"vial_{k + 1}_wall", "glass", (x, y, 0.), L.VIAL_ID / 2, L.VIAL_OD / 2, 0., neck, 12, pad=pad)
    text += ring(f"vial_{k + 1}_neck", "glass", (x, y, 0.), L.VIAL_NECK_ID / 2, L.VIAL_OD / 2 - .001, neck, L.VIAL_H, 12,
                 pad=pad)
    text += polygon_prism(f"vial_{k + 1}_floor", "glass", (x, y, 0.), L.VIAL_ID / 2 + .0005, 0., L.VIAL_FLOOR, pad=pad)
    ro, ri, rn = L.VIAL_OD / 2, L.VIAL_ID / 2, L.VIAL_NECK_ID / 2
    text += revolve(f"vial_{k + 1}_glass", "glass_look", [(0., L.VIAL_FLOOR), (ri, L.VIAL_FLOOR), (ri, neck - .002), (rn, neck),
                                                    (rn, L.VIAL_H), (ro - .001, L.VIAL_H), (ro - .001, neck), (ro, neck - .002),
                                                    (ro, .001), (ro - .001, 0.), (0., 0.)], segments=32, centre=(x, y, 0.),
                    pad=pad)
    text += cylinder(f"vial_{k + 1}_sample", L.DYES[k], (x, y, (L.VIAL_FLOOR + L.LIQUID_Z) / 2), ri - .0002,
                     L.LIQUID_Z - L.VIAL_FLOOR, pad=pad)
    return text


def usd():
    top = scene.TOP

    def world(p):
        return (p[0], p[1], p[2] + top)

    actors = rigid_body("plate", L.PLATE_MASS, world(scene.START["plate"][0]), plate_parts())
    for name in scene.SPOTTERS:
        actors += rigid_body(name, L.CAP_MASS, world(scene.START[name][0]), spotter_parts())
    water = L.SOLVENT_ML * 1e-3
    actors += rigid_body("tank", L.TANK_MASS + water, world(scene.START["tank"][0]), tank_parts(),
                         com=(0., 0., (L.TANK_MASS * .045 + water * L.SOLVENT_Z) / (L.TANK_MASS + water)))
    actors += rigid_body("lid", L.LID_MASS, world(scene.START["lid"][0]), lid_parts())

    pad = "        "
    static = span("bar", "aluminium", (L.PLATE_X - L.BAR[0] / 2, L.PLATE_X + L.BAR[0] / 2),
                  (L.PLATE_Y - L.BAR[1] / 2, L.PLATE_Y + L.BAR[1] / 2), (0., L.BAR[2]), pad=pad)
    static += holed_block("rack", "foam", L.RACK_X, L.RACK_Y - L.RACK[1] / 2, L.RACK_Y + L.RACK[1] / 2, L.RACK[2],
                          L.HOLE_Y, L.HOLE, L.RACK[2] - L.HOLE_DEPTH)
    static += holed_block("vial_block", "foam", L.VIAL_X, L.VIAL_BLOCK_Y - L.VIAL_BLOCK[1] / 2,
                          L.VIAL_BLOCK_Y + L.VIAL_BLOCK[1] / 2, L.VIAL_BLOCK[2], L.VIAL_Y, .016, 0.)
    for k in range(3):
        static += vial(k)

    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            '    def Scope "Actors"\n    {\n' + actors + "    }\n\n"
            '    def Xform "Static"\n    {\n'
            f'        double3 xformOp:translate = (0, 0, {_f(top)})\n'
            '        uniform token[] xformOpOrder = ["xformOp:translate"]\n\n' + static + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + GLASS_LOOK
            + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print("wrote", scene.USD.name, f"{scene.USD.stat().st_size / 1e3:.0f} kB")
