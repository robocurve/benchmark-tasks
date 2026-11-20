"""Write the drum kit scene ``rdb_drum_groove/kit.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Standard library only.

- ``/Fixture/Kit``: one kinematic rigid body per trigger zone (fixed: they never move): ``hihat`` (the rubber pad: its
  bow and its raised bell, two convex pieces), ``snare_head`` and ``tom_head`` (each the whole drum pad as one solid
  convex piece under the head, so a fast tip meets a thick collider; a grey mesh head is drawn on top), ``snare_rim``
  and ``tom_rim`` (the black hoop: 16 convex pieces round the head, its top 6 mm above the head). Kinematic bodies so
  that each zone is a contact filter of its own.
- ``/Fixture/Sticks``: two free rigid bodies (Vic Firth 5A), frame at the grip point (135 mm from the butt), +x toward
  the tip: a 7.2 mm shaft capsule, a 5 mm capsule for the taper and the 5 mm tip sphere; 50 g; contact offset 15 mm
  (speculative contacts: a tip at 3.6 m/s moves 15 mm in a physics step). CCD is requested on the sticks but the
  scene runs GPU dynamics with CCD off, so it has no effect (TECHNICAL.md).
- ``/Fixture/Drawn``: the kit's hardware, drawn only: a 38 mm rack tube along the table's far side held by two
  C-clamps, and each pad's clamp, L-arm and holder.

Materials: pads restitution 0.7 with combine "max"; sticks restitution 0 with combine "min" (a stick never bounces off
the finger pads) and friction 0.8 with combine "max" (the grip's friction is 0.8: the YAM's finger colliders have no
material of their own).
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_drum_groove import fixed_scene as scene  # noqa: E402
from rdb_drum_groove import layout as L  # noqa: E402

SEG = 32
STICK_OFFSET = L.CONTACT_OFFSET
PAD_OFFSET = .002
STICK_MU = L.STICK_MU


def _f(v):
    return f"{v:.6g}"


def _v3(p):
    return f"({_f(p[0])}, {_f(p[1])}, {_f(p[2])})"


def material(name, color, roughness, friction, restitution=0., fcombine="average", rcombine="average", extra=""):
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = {restitution}\n"
            f"            float physics:staticFriction = {friction}\n"
            f'            uniform token physxMaterial:frictionCombineMode = "{fcombine}"\n'
            f'            uniform token physxMaterial:restitutionCombineMode = "{rcombine}"\n{extra}\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


def materials():
    e = L.PAD_RESTITUTION
    pad_extra = ""
    # looks from the Nitro Max's product photos: grey mesh heads in black rims, black shells, a black rubber
    # cymbal-shaped hi-hat pad with a raised bell, chrome clamps and arms on a black rack tube
    text = material("mesh", (.16, .165, .17), .9, .5, e, "average", "max", pad_extra)
    text += material("rubber", (.035, .035, .04), .75, .8, e, "average", "max", pad_extra)
    text += material("hoop", (.03, .03, .035), .5, .4, e, "average", "max", pad_extra)
    text += material("shell", (.05, .05, .055), .55, .4, e, "average", "max", pad_extra)
    text += material("rack", (.02, .02, .022), .4, .4)
    text += material("hickory", (.80, .62, .40), .6, STICK_MU, 0., "max", "min")
    text += material("tape", (.95, .95, .95), .6, STICK_MU, 0., "max", "min")
    text += material("chrome", (.75, .75, .78), .25, .3)
    return text


PREFIX = "</Fixture/Materials/"


def mesh(name, mat, points, faces, *, collide=True, pad="            ", visible=True):
    apis = '"MaterialBindingAPI"' + (', "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", '
                                     '"PhysxConvexHullCollisionAPI"' if collide else "")
    text = (f'{pad}def Mesh "{name}" (\n{pad}    prepend apiSchemas = [{apis}]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {PREFIX}{mat}>\n")
    if collide:
        text += (f"{pad}    rel material:binding:physics = {PREFIX}{mat}>\n"
                 f'{pad}    uniform token physics:approximation = "convexHull"\n'
                 f"{pad}    int physxConvexHullCollision:hullVertexLimit = 64\n"
                 f"{pad}    float physxCollision:contactOffset = {_f(PAD_OFFSET)}\n"
                 f"{pad}    float physxCollision:restOffset = 0\n")
    if not visible:
        text += f'{pad}    token visibility = "invisible"\n'
    return (text + f"{pad}    uniform bool doubleSided = 1\n"
            f"{pad}    int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"{pad}    int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"{pad}    point3f[] points = [{', '.join(_v3(p) for p in points)}]\n"
            f'{pad}    uniform token subdivisionScheme = "none"\n{pad}}}\n\n')


def lathe(name, mat, rings, *, segments=SEG, collide=True, pad="            ", visible=True):
    pts, faces = [], []
    for r, z in rings:
        pts += [(r * math.cos(2 * math.pi * k / segments), r * math.sin(2 * math.pi * k / segments), z)
                for k in range(segments)]
    for j in range(len(rings) - 1):
        a, b = j * segments, (j + 1) * segments
        faces += [[a + k, a + (k + 1) % segments, b + (k + 1) % segments, b + k] for k in range(segments)]
    faces.append(list(range(segments - 1, -1, -1)))
    last = (len(rings) - 1) * segments
    faces.append(list(range(last, last + segments)))
    return mesh(name, mat, pts, faces, collide=collide, pad=pad, visible=visible)


def ring_pieces(prefix, mat, r_in, r_out, z0, z1, n=16):
    text = ""
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        pts = []
        for z in (z0, z1):
            pts += [(r_in * math.cos(a0), r_in * math.sin(a0), z), (r_out * math.cos(a0), r_out * math.sin(a0), z),
                    (r_out * math.cos(a1), r_out * math.sin(a1), z), (r_in * math.cos(a1), r_in * math.sin(a1), z)]
        faces = [[3, 2, 1, 0], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        text += mesh(f"{prefix}_{i:02d}", mat, pts, faces)
    return text


def kinematic(name, position, orient_wxyz, children):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"']
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            "            bool physics:rigidBodyEnabled = 1\n            bool physics:kinematicEnabled = 1\n"
            f"            quatf xformOp:orient = ({', '.join(f'{v:.7g}' for v in orient_wxyz)})\n"
            f"            double3 xformOp:translate = {_v3(position)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def capsule(name, mat, radius, x0, x1, pad="            "):
    """A capsule along the body's x axis whose cylinder runs from x0 to x1."""
    return (f'{pad}def Capsule "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI"]\n{pad})\n{pad}{{\n'
            f'{pad}    uniform token axis = "X"\n{pad}    double height = {_f(x1 - x0)}\n{pad}    double radius = {_f(radius)}\n'
            f"{pad}    rel material:binding = {PREFIX}{mat}>\n{pad}    rel material:binding:physics = {PREFIX}{mat}>\n"
            f"{pad}    float physxCollision:contactOffset = {_f(STICK_OFFSET)}\n{pad}    float physxCollision:restOffset = 0\n"
            f"{pad}    double3 xformOp:translate = ({_f((x0 + x1) / 2)}, 0, 0)\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate"]\n{pad}}}\n\n')


def sphere(name, mat, radius, x, pad="            "):
    return (f'{pad}def Sphere "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    double radius = {_f(radius)}\n"
            f"{pad}    rel material:binding = {PREFIX}{mat}>\n{pad}    rel material:binding:physics = {PREFIX}{mat}>\n"
            f"{pad}    float physxCollision:contactOffset = {_f(STICK_OFFSET)}\n{pad}    float physxCollision:restOffset = 0\n"
            f"{pad}    double3 xformOp:translate = ({_f(x)}, 0, 0)\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate"]\n{pad}}}\n\n')


def stick(name, position):
    butt = -L.GRIP_FROM_BUTT
    tip_c = L.TIP_OUT - L.TIP_R
    children = capsule("shaft", "hickory", L.STICK_R, butt + L.STICK_R, L.TAPER_FROM_GRIP)
    children += capsule("taper", "hickory", L.NECK_R, L.TAPER_FROM_GRIP, tip_c - L.TIP_R)
    children += sphere("tip", "hickory", L.TIP_R, tip_c)
    # the tape mark at the grip point (drawn: a thin band)
    children += (f'            def Cylinder "tape" (\n                prepend apiSchemas = ["MaterialBindingAPI"]\n            )\n'
                 "            {\n                uniform token axis = \"X\"\n                double height = 0.012\n"
                 f"                double radius = {_f(L.STICK_R + .0004)}\n"
                 f"                rel material:binding = {PREFIX}tape>\n            }}\n\n")
    com = L.BALANCE_FROM_BUTT - L.GRIP_FROM_BUTT
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            f"            float physics:mass = {_f(L.STICK_MASS)}\n"
            f"            point3f physics:centerOfMass = ({_f(com)}, 0, 0)\n"
            f"            float3 physics:diagonalInertia = ({_f(L.STICK_I_AXIAL)}, {_f(L.STICK_I_TRANSVERSE)}, {_f(L.STICK_I_TRANSVERSE)})\n"
            "            bool physics:rigidBodyEnabled = 1\n"
            "            bool physxRigidBody:enableCCD = 1\n"
            "            bool physxRigidBody:enableSpeculativeCCD = 1\n"
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            float physxRigidBody:stabilizationThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            "            quatf xformOp:orient = (1, 0, 0, 0)\n"
            f"            double3 xformOp:translate = {_v3(position)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def kit():
    top = scene.TOP
    text = ""
    for kind in scene.PADS:
        (x, y, z), _n = scene.pad_frame(kind)
        q = scene.pad_quat_wxyz(kind)
        pos = (x, y, z + top)
        if kind == "hihat":
            # a rubber cymbal: the bow, 12 mm thick at the edge rising 6 mm to the bell, and a raised bell, two convex
            # pieces (a cymbal's profile is not convex)
            r, bell = L.HIHAT_R, L.HIHAT_BELL_R
            rise, bell_top = L.HIHAT_BOW_RISE, L.HIHAT_BELL_TOP
            body = lathe("bow", "rubber", [(r, -L.HIHAT_T_EDGE), (r, 0.), (bell, rise), (0., rise)])
            body += lathe("bell", "rubber", [(bell - .003, rise - .003), (bell - .003, rise), (.030, rise + .006),
                                            (.012, bell_top), (0.002, bell_top)])
            body += lathe("stem", "chrome", [(.006, -.045), (.006, -L.HIHAT_T_EDGE)], collide=False)
            text += kinematic("hihat", pos, q, body)
            continue
        head_r = L.SNARE_HEAD_R if kind == "snare" else L.TOM_HEAD_R
        body = lathe("drum", "shell", [(head_r, -L.SHELL_DEPTH), (head_r, 0.)])
        # the grey mesh head, drawn just above the collider's top face, inside the black rim
        body += lathe("mesh_head", "mesh", [(head_r - .002, 0.), (head_r - .002, .0006)], collide=False)
        text += kinematic(f"{kind}_head", pos, q, body)
        text += kinematic(f"{kind}_rim", pos, q, ring_pieces("hoop", "hoop", head_r, head_r + L.RIM_W, -L.SHELL_DEPTH, L.RIM_H))
    return text


def cyl(name, mat, axis, length, radius, centre):
    return (f'        def Cylinder "{name}" (\n            prepend apiSchemas = ["MaterialBindingAPI"]\n        )\n        {{\n'
            f'            uniform token axis = "{axis}"\n            double height = {_f(length)}\n            double radius = {_f(radius)}\n'
            f"            rel material:binding = {PREFIX}{mat}>\n"
            f"            double3 xformOp:translate = {_v3(centre)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate"]\n        }\n\n')


def block(name, mat, centre, size):
    half = tuple(v / 2 for v in size)
    return (f'        def Cube "{name}" (\n            prepend apiSchemas = ["MaterialBindingAPI"]\n        )\n        {{\n'
            f"            rel material:binding = {PREFIX}{mat}>\n            double size = 2\n"
            f"            double3 xformOp:translate = {_v3(centre)}\n            float3 xformOp:scale = {_v3(half)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n        }\n\n')


RACK_X_PAST = .17                 # [E] the rack tube's axis this far past the farthest pad centre
RACK_R = .019                     # the kit's 38 mm rack tube


def drawn():
    """The kit's hardware, drawn only (no colliders): a 38 mm rack tube lying along the table's far side, held by two
    C-clamps, and for each pad a clamp on the tube, an L-arm (up from the clamp, then forward under the pad) and the
    pad's holder block under its centre."""
    top = scene.TOP
    text = ""
    xs = [scene.pad_frame(k)[0][0] for k in scene.PADS]
    xr = max(xs) + RACK_X_PAST
    ys = [scene.pad_frame(k)[0][1] for k in scene.PADS]
    y0, y1 = min(ys) - .16, max(ys) + .16
    zr = top + RACK_R
    text += cyl("rack_tube", "rack", "Y", y1 - y0, RACK_R, (xr, (y0 + y1) / 2, zr))
    for name, yc in (("c_clamp_left", y1 - .04), ("c_clamp_right", y0 + .04)):
        text += block(f"{name}_frame", "chrome", (xr, yc, top + .03), (.07, .018, .012))
        text += block(f"{name}_jaw", "chrome", (xr + .03, yc, top + .01), (.012, .018, .03))
    for kind in scene.PADS:
        (x, y, z), _ = scene.pad_frame(kind)
        under = z - (L.SHELL_DEPTH if kind != "hihat" else .045) - .012     # the arm runs this high under the pad
        text += block(f"{kind}_tube_clamp", "chrome", (xr, y, zr), (.05, .045, .05))
        rise = under - (zr - top) + .012
        text += cyl(f"{kind}_arm_up", "chrome", "Z", max(rise, .02), .0095, (xr, y, zr + max(rise, .02) / 2))
        text += cyl(f"{kind}_arm_out", "chrome", "X", xr - x, .0095, ((x + xr) / 2, y, top + under))
        text += block(f"{kind}_holder", "rack", (x, y, top + under + .006), (.05, .04, .024))
    return text


def usd():
    top = scene.TOP
    sticks = stick(scene.STICKS["left"], (-.25, .60, top + .30)) + stick(scene.STICKS["right"], (-.25, -.60, top + .30))
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            f'    def Scope "{scene.KIT}"\n    {{\n' + kit() + "    }\n\n"
            f'    def Scope "{scene.ACTORS}"\n    {{\n' + sticks + "    }\n\n"
            '    def Scope "Drawn"\n    {\n' + drawn() + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + materials() + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print("wrote", scene.USD.name, f"{scene.USD.stat().st_size / 1e3:.0f} kB")
