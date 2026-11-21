"""Regenerate ``rdb_tablets_cap/tablets.usda`` from ``layout.py``. Run from the task folder; needs only NumPy.

Each of the bottle, the cap and the cup is one free rigid body. What the cameras see is a smooth lathe-turned
mesh at the product's true outside shape with no collision; what the physics sees is an invisible ring of 16
convex staves plus a floor disc (see ``layout.py`` for where the collision walls are thicker than the real
ones). A tablet's visible mesh is also its convex collider.
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_tablets_cap import fixed_scene as scene  # noqa: E402
from rdb_tablets_cap import layout as L  # noqa: E402

S = L.SIDES
# colour, roughness, opacity, friction, combine mode
MATERIALS = {
    "hdpe": ((.93, .93, .91), .45, 1., .6, "average"), "cap_pp": ((.96, .96, .96), .35, 1., .6, "average"),
    "cup_pp": ((.86, .90, .92), .15, .38, .6, "average"), "tablet": ((.42, .20, .09), .35, 1., L.FRICTION, "min"),
    "label": ((.97, .97, .95), .6, 1., .6, "average"), "label_band": ((.10, .27, .55), .6, 1., .6, "average"),
    "ink": ((.15, .15, .15), .6, 1., .6, "average"),
}
_PREFIX = "</Fixture/Materials/"
_F = "{:.6g}".format


def _points(points):
    return "[" + ", ".join(f"({_F(x)}, {_F(y)}, {_F(z)})" for x, y, z in points) + "]"


def mesh(name, material, points, faces, *, collide=False, visible=True, offset=.001):
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"', '"PhysicsMeshCollisionAPI"']
                                       if collide else [])
    text = (f'            def Mesh "{name}" (\n                prepend apiSchemas = [{", ".join(apis)}]\n            )\n'
            "            {\n" f"                rel material:binding = {_PREFIX}{material}>\n")
    if collide:
        text += (f"                rel material:binding:physics = {_PREFIX}{material}>\n"
                 f"                float physxCollision:contactOffset = {offset}\n"
                 "                float physxCollision:restOffset = 0\n"
                 '                uniform token physics:approximation = "convexHull"\n')
    if not visible:
        text += '                token visibility = "invisible"\n'
    return (text + "                uniform bool doubleSided = 1\n"
            f"                int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"                int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"                point3f[] points = {_points(points)}\n"
            '                uniform token subdivisionScheme = "none"\n            }\n\n')


def lathe(name, material, profile, segments=48):
    """Display mesh: the (radius, height) profile turned about the z axis."""
    points, faces = [], []
    for r, z in profile:
        points += [(r * math.cos(2 * math.pi * k / segments), r * math.sin(2 * math.pi * k / segments), z)
                   for k in range(segments)]
    for i in range(len(profile) - 1):
        for k in range(segments):
            a, b = i * segments + k, i * segments + (k + 1) % segments
            faces.append([a, b, b + segments, a + segments])
    return mesh(name, material, points, faces)


def stave(name, material, k, r0, z0, r1, z1, thickness, inward):
    """Collision stave k of 16: a convex hexahedron between heights z0 and z1 whose true face (outside if
    ``inward``, else inside) runs from radius r0 to r1 at the corners."""
    a0, a1 = (2 * math.pi * (k - .5) / S, 2 * math.pi * (k + .5) / S)
    d = -thickness if inward else thickness
    points = [(r * math.cos(a), r * math.sin(a), z) for r, z in ((r0, z0), (r1, z1), (r1 + d, z1), (r0 + d, z0))
              for a in (a0, a1)]
    faces = [[0, 1, 3, 2], [2, 3, 5, 4], [4, 5, 7, 6], [6, 7, 1, 0], [0, 2, 4, 6], [1, 7, 5, 3]]
    return mesh(name, material, points, faces, collide=True, visible=False)


def disc(name, material, radius, z0, z1):
    ring = [(radius * math.cos(2 * math.pi * (k - .5) / S), radius * math.sin(2 * math.pi * (k - .5) / S)) for k in range(S)]
    points = [(x, y, z0) for x, y in ring] + [(x, y, z1) for x, y in ring]
    faces = [list(range(S - 1, -1, -1)), list(range(S, 2 * S))] + [[i, (i + 1) % S, S + (i + 1) % S, S + i] for i in range(S)]
    return mesh(name, material, points, faces, collide=True, visible=False)


def tablet_mesh():
    n = 12
    r, e, t, ri = L.TABLET_R, L.TABLET_EDGE / 2, L.TABLET_T / 2, .55 * L.TABLET_R
    rings = [(ri, -t + .0003), (r, -e), (r, e), (ri, t - .0003)]
    points = [(q * math.cos(2 * math.pi * k / n), q * math.sin(2 * math.pi * k / n), z) for q, z in rings for k in range(n)]
    points += [(0, 0, -t), (0, 0, t)]
    faces = [[i * n + k, i * n + (k + 1) % n, (i + 1) * n + (k + 1) % n, (i + 1) * n + k] for i in range(3) for k in range(n)]
    faces += [[4 * n, (k + 1) % n, k] for k in range(n)] + [[4 * n + 1, 3 * n + k, 3 * n + (k + 1) % n] for k in range(n)]
    return mesh("tablet", "tablet", points, faces, collide=True, offset=.0008)


def body(name, mass, position, children, *, small=False, orient="(1, 0, 0, 0)"):
    apis = '"PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI", "PhysicsMassAPI"'
    extra = ("            bool physxRigidBody:enableCCD = 1\n            float physxRigidBody:angularDamping = 0.5\n"
             "            float physxRigidBody:linearDamping = 0.05\n"
             "            float physxRigidBody:maxDepenetrationVelocity = 0.5\n"
             "            int physxRigidBody:solverPositionIterationCount = 16\n" if small else
             "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
             "            int physxRigidBody:solverPositionIterationCount = 32\n")
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{apis}]\n        )\n        {{\n'
            f"            float physics:mass = {_F(mass)}\n            bool physics:rigidBodyEnabled = 1\n" + extra +
            "            float physxContactReport:threshold = 0\n            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverVelocityIterationCount = 4\n"
            f"            quatf xformOp:orient = {orient}\n"
            f"            double3 xformOp:translate = ({_F(position[0])}, {_F(position[1])}, {_F(position[2])})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def material(name):
    color, roughness, opacity, friction, combine = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            f'            uniform token physxMaterial:frictionCombineMode = "{combine}"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:opacity = {opacity}\n                float inputs:metallic = 0\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


def sector(name, material, lo, hi, ri, ro, bottom, top, *, collide=True):
    """Convex annular sector; split long ledges into short facets."""
    points = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), z)
              for z in (bottom, top) for r, a in ((ri, lo), (ro, lo), (ro, hi), (ri, hi))]
    faces = [[3, 2, 1, 0], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return mesh(name, material, points, faces, collide=collide, offset=.0002)


def neck_lock():
    pieces = []
    for i, start in enumerate((0., 180.)):
        # Unbroken retaining ledge except at the two extraction windows.
        span = 180 + L.WINDOW_START - L.WINDOW_END
        for j in range(10):
            lo = start + L.WINDOW_END + j * span / 10
            pieces.append(sector(f"ledge_{i}_{j}", "hdpe", lo, lo + span / 10,
                                 .0170, .0195, L.LEDGE_Z, L.LEDGE_Z + .0014))
        pieces.append(sector(f"turn_stop_{i}", "hdpe", start + 8, start + 17,
                             .0170, .0195, L.STOP_Z, L.LEDGE_Z))
        pieces.append(sector(f"back_stop_{i}", "hdpe", start - 14, start - 8,
                             .0170, .0195, L.LUG_TOP - .0065, L.LEDGE_Z))
    return "".join(pieces)


def sprung_collar():
    """A bottle-mounted external annulus bears against the bottom edge of the cap."""
    ring = "".join(sector(f"ring_{i}", "label_band", i*22.5, (i+1)*22.5,
                          .0181, .0216, -.0018, 0.) for i in range(16))
    bx, by, _ = scene.START["bottle"]
    return body("spring_collar", L.COLLAR_MASS, (bx, by, scene.TOP + L.COLLAR_Z + .0003), ring)


def collar_joint():
    return f'''        def PhysicsPrismaticJoint "collar_slide" (
            prepend apiSchemas = ["PhysicsDriveAPI:linear"]
        )
        {{
            rel physics:body0 = </Fixture/Actors/bottle>
            rel physics:body1 = </Fixture/Closure/spring_collar>
            point3f physics:localPos0 = (0, 0, {L.COLLAR_Z})
            point3f physics:localPos1 = (0, 0, 0)
            uniform token physics:axis = "Z"
            float physics:lowerLimit = {-L.COLLAR_TRAVEL}
            float physics:upperLimit = 0.001
            uniform token drive:linear:physics:type = "force"
            float drive:linear:physics:targetPosition = 0.001
            float drive:linear:physics:stiffness = {L.COLLAR_STIFFNESS}
            float drive:linear:physics:damping = 0.35
        }}
'''


def bottle_parts():
    R, H, w = L.BOTTLE_R, L.BOTTLE_H, L.BOTTLE_WALL
    neck_wall = L.NECK_R_OUT - L.NECK_R_IN
    parts = [disc("floor", "hdpe", R, 0., L.BOTTLE_FLOOR), neck_lock()]
    for k in range(S):
        parts.append(stave(f"body_{k:02d}", "hdpe", k, R, L.BOTTLE_FLOOR, R, L.BODY_H, w, True))
        parts.append(stave(f"shoulder_{k:02d}", "hdpe", k, R, L.BODY_H, L.NECK_R_OUT, L.SHOULDER_H, neck_wall + .001, True))
        parts.append(stave(f"neck_{k:02d}", "hdpe", k, L.NECK_R_OUT, L.SHOULDER_H, L.NECK_R_OUT, H, neck_wall, True))
    n = L.NECK_R_OUT
    thread = [(n, L.SHOULDER_H + .0015), (n + .0012, L.SHOULDER_H + .0025), (n, L.SHOULDER_H + .0035), (n, L.SHOULDER_H + .006),
              (n + .0012, L.SHOULDER_H + .007), (n, L.SHOULDER_H + .008), (n, L.SHOULDER_H + .0105),
              (n + .0012, L.SHOULDER_H + .0115), (n, L.SHOULDER_H + .0125)]
    profile = [(0, 0), (R - .003, 0), (R, .003), (R, L.BODY_H - .002), (R - .002, L.BODY_H + .002), (n, L.SHOULDER_H), *thread,
               (n, H), (L.NECK_R_IN, H), (L.NECK_R_IN, L.SHOULDER_H), (L.BODY_R_IN, L.BODY_H), (L.BODY_R_IN, L.BOTTLE_FLOOR),
               (0, L.BOTTLE_FLOOR)]
    parts.append(lathe("shell", "hdpe", profile))
    parts.append(lathe("label", "label", [(R + .0003, .012), (R + .0003, .052)]))
    parts.append(lathe("label_band", "label_band", [(R + .0005, .040), (R + .0005, .050)]))
    parts.append(lathe("label_text", "ink", [(R + .0005, .020), (R + .0005, .0215)]))
    parts.append(sector("bottle_alignment_mark", "label_band", -5., 5., R, R + .0006,
                        .051, .060, collide=False))
    parts.append(lathe("label_text2", "ink", [(R + .0005, .026), (R + .0005, .0275)]))
    return "".join(parts)


def cap_parts():
    """The cap as it lies upside down: origin at the centre of its outer top face, open side toward +z."""
    R, H = L.CAP_R, L.CAP_H
    parts = [disc("top", "cap_pp", R, 0., L.CAP_FLOOR)]
    # Lower skirt clears the neck ledges; two inward-facing lugs run beneath them.
    for k in range(S):
        parts.append(stave(f"skirt_{k:02d}", "cap_pp", k, R, L.CAP_FLOOR, R, H,
                           R - .0203, True))
    for i, angle in enumerate((0., 180.)):
        parts.append(sector(f"lock_lug_{i}", "cap_pp", angle-L.LUG_HALF_ANGLE, angle+L.LUG_HALF_ANGLE,
                            .0179, .0210, L.CLOSED_Z-L.LUG_TOP,
                            L.CLOSED_Z-L.LUG_TOP+L.LUG_THICKNESS))
    parts.append(sector("alignment_mark", "label_band", -5., 5., .009, R-.001, -.00016, 0., collide=False))
    profile = [(0, 0), (R - .001, 0), (R, .001), (R, H), (.0203, H), (.0203, L.CAP_FLOOR), (0, L.CAP_FLOOR)]
    parts.append(lathe("shell", "cap_pp", profile))
    ribs, faces = [], []
    for k in range(36):                                       # the ribbed skirt
        a = 2 * math.pi * k / 36
        c, s, i = math.cos(a), math.sin(a), len(ribs)
        for dr, da in ((0, -.02), (.0007, 0.), (0, .02)):
            for z in (.003, H - .001):
                ribs.append(((R + dr) * math.cos(a + da), (R + dr) * math.sin(a + da), z))
        faces += [[i, i + 2, i + 3, i + 1], [i + 2, i + 4, i + 5, i + 3]]
    parts.append(mesh("ribs", "cap_pp", ribs, faces))
    return "".join(parts)


def cup_parts():
    wall = L.CUP_MODEL_WALL
    r0, r1 = L.cup_inner_radius(0.), L.cup_inner_radius(L.CUP_H)
    parts = [disc("floor", "cup_pp", r0 + wall, 0., L.CUP_FLOOR)]
    parts += [stave(f"wall_{k:02d}", "cup_pp", k, r0, L.CUP_FLOOR, r1, L.CUP_H, wall, False) for k in range(S)]
    profile = [(0, 0), (L.CUP_R_BOTTOM, 0), (L.CUP_R_TOP, L.CUP_H - .001), (L.CUP_R_TOP + .0008, L.CUP_H), (r1, L.CUP_H),
               (r0, .0008), (0, .0008)]
    parts.append(lathe("shell", "cup_pp", profile))
    for i, z in enumerate((.010, .017, .024, .030)):          # graduation lines
        parts.append(lathe(f"mark_{i}", "ink", [(L.cup_inner_radius(z) + L.CUP_WALL + .0002, z),
                                                 (L.cup_inner_radius(z + .0006) + L.CUP_WALL + .0002, z + .0006)], segments=24))
    return "".join(parts)


def usd():
    top = scene.TOP

    def place(name, lift=0.):
        x, y, z = scene.START[name]
        return (x, y, z + top + lift)

    bodies = body("bottle", L.BOTTLE_MASS, place("bottle", .0003), bottle_parts())
    bodies += body("cap", L.CAP_MASS, place("cap", .0003), cap_parts(), orient="(0, 1, 0, 0)")
    bodies += body("cup", L.CUP_MASS, place("cup", .0003), cup_parts())
    for name in scene.TABLETS:
        bodies += body(name, L.TABLET_MASS, place(name, .0003), tablet_mesh(), small=True)
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Scope "Closure"\n    {\n' + sprung_collar() + collar_joint() + '    }\n\n'
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    (ROOT / "rdb_tablets_cap/tablets.usda").write_text(usd())
    print("bodies", len(scene.BODIES), "bytes", len(usd()))
