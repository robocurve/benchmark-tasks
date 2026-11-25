"""Regenerate ``rdb_filter_clamp/filter_clamp.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs NumPy and SciPy (for the convex hulls).

Two loose rigid bodies and one floating articulation:
- ``Actors/holder``: the 1 L filtering flask, the stopper and the fritted glass base with its ground flange, as one
  body standing free on the table (the stopper's grip on the flask and the base is not simulated).
- ``Actors/funnel``: the 300 mL glass funnel, standing on its flange on the table.
- ``Clamp``: the hinge pin (knuckle) and two jaws on hinges about it, each with a pre-wound torsion spring that
  closes it. Each jaw is ten arc segments whose inner V faces bear on the flanges' bevels, plus its handle.
Collision is by convex hulls (at most 64 vertices) and is hidden on the glass, which is drawn instead as smooth
revolved glass surfaces with no collision. The clamp's colliders are drawn as they are (anodised blue).
Display-only parts (membrane, frit, side-arm, graduations, wipe, spring coil) have no collision.
"""

import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_filter_clamp import fixed_scene as scene  # noqa: E402
from rdb_filter_clamp import layout as L  # noqa: E402

# colour (linear RGB), roughness, metallic, opacity, static friction, dynamic friction
MATERIALS = {
    "glass": ((.86, .93, .96), .04, 0., .22, .40, .35),       # borosilicate; ground faces on a wet membrane
    "glass_edge": ((.70, .82, .86), .10, 0., .45, .40, .35),
    "anodised": ((.05, .22, .62), .35, .6, 1., .35, .30),     # blue anodised aluminium clamp
    "steel": ((.75, .76, .78), .3, 1., 1., .30, .25),
    "silicone": ((.55, .56, .58), .8, 0., 1., .90, .80),
    "membrane": ((.97, .97, .95), .9, 0., 1., .50, .45),
    "frit": ((.86, .84, .78), .9, 0., 1., .50, .45),
    "print": ((.98, .98, .98), .6, 0., 1., .40, .35),
    "wipe": ((.95, .95, .97), .95, 0., 1., .50, .45),
    "hose": ((.80, .55, .35), .7, 0., .85, .80, .70),
}
_M = "</Fixture/Materials/"
I = "    "


def fmt(v):
    return ", ".join(f"{float(x):.6g}" for x in v)


def hull_mesh(name, points, material, *, visible, offset=.001, indent=4):
    """A convex-hull collider. The hull's own triangles are written so that it can also be drawn."""
    pts = np.asarray(points, dtype=float)
    hull = ConvexHull(pts)
    used = sorted(set(hull.simplices.ravel()))
    index = {old: new for new, old in enumerate(used)}
    faces = []
    centre = pts[used].mean(axis=0)
    for tri in hull.simplices:
        a, b, c = (pts[i] for i in tri)
        if np.cross(b - a, c - a) @ (a - centre) < 0:
            tri = tri[::-1]
        faces.extend(index[i] for i in tri)
    p = I * indent
    return (f'{p}def Mesh "{name}" (\n{p}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"]\n{p})\n{p}{{\n'
            f"{p}    rel material:binding = {_M}{material}>\n"
            f"{p}    rel material:binding:physics = {_M}{material}>\n"
            f'{p}    uniform token physics:approximation = "convexHull"\n'
            f"{p}    int physxConvexHullCollision:hullVertexLimit = 64\n"
            f"{p}    float physxCollision:contactOffset = {offset:g}\n"
            f"{p}    float physxCollision:restOffset = 0\n"
            + ("" if visible else f'{p}    token visibility = "invisible"\n') +
            f"{p}    int[] faceVertexCounts = [{', '.join(['3'] * (len(faces) // 3))}]\n"
            f"{p}    int[] faceVertexIndices = [{', '.join(map(str, faces))}]\n"
            f"{p}    point3f[] points = [{', '.join('(' + fmt(pts[i]) + ')' for i in used)}]\n"
            f'{p}    uniform token subdivisionScheme = "none"\n'
            f"{p}}}\n\n")


def display_mesh(name, points, faces, material, *, indent=4, double_sided=True):
    p = I * indent
    counts = ", ".join(str(len(f)) for f in faces)
    flat = ", ".join(str(i) for f in faces for i in f)
    return (f'{p}def Mesh "{name}" (\n{p}    prepend apiSchemas = ["MaterialBindingAPI"]\n{p})\n{p}{{\n'
            f"{p}    rel material:binding = {_M}{material}>\n"
            f"{p}    uniform bool doubleSided = {1 if double_sided else 0}\n"
            f"{p}    int[] faceVertexCounts = [{counts}]\n"
            f"{p}    int[] faceVertexIndices = [{flat}]\n"
            f"{p}    point3f[] points = [{', '.join('(' + fmt(v) + ')' for v in points)}]\n"
            f'{p}    uniform token subdivisionScheme = "none"\n'
            f"{p}}}\n\n")


def revolve(name, profile, material, *, n=64, centre=(0., 0., 0.), indent=4):
    """A surface of revolution about z through ``centre`` from an (r, z) polyline. Display only."""
    profile = np.asarray(profile, dtype=float)
    pts, faces = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        for r, z in profile:
            pts.append((centre[0] + r * math.cos(a), centre[1] + r * math.sin(a), centre[2] + z))
    m = len(profile)
    for k in range(n):
        k2 = (k + 1) % n
        for j in range(m - 1):
            faces.append((k * m + j, k2 * m + j, k2 * m + j + 1, k * m + j + 1))
    return display_mesh(name, pts, faces, material, indent=indent)


def box_display(name, centre, half, material, *, indent=4, rotation=None):
    c = np.asarray(centre, dtype=float)
    h = np.asarray(half, dtype=float)
    r = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    corners = [c + r @ (h * np.array((sx, sy, sz))) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return display_mesh(name, corners, faces, material, indent=indent)


def tube_display(name, start, end, radius, material, *, n=16, indent=4):
    start, end = np.asarray(start, float), np.asarray(end, float)
    axis = end - start
    axis /= np.linalg.norm(axis)
    u = np.cross(axis, (0., 0., 1.) if abs(axis[2]) < .9 else (1., 0., 0.))
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    pts = [p + radius * (math.cos(a) * u + math.sin(a) * v) for p in (start, end)
           for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    faces = [(k, (k + 1) % n, n + (k + 1) % n, n + k) for k in range(n)]
    return display_mesh(name, pts, faces, material, indent=indent)


def rigid_body(name, mass, position, children, *, com=None, inertia=None, indent=2, contact_report=True):
    p = I * indent
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysicsMassAPI"']
    if contact_report:
        apis.append('"PhysxContactReportAPI"')
    text = (f'{p}def Xform "{name}" (\n{p}    prepend apiSchemas = [{", ".join(apis)}]\n{p})\n{p}{{\n'
            f"{p}    float physics:mass = {mass:.6g}\n")
    if com is not None:
        text += f"{p}    point3f physics:centerOfMass = ({fmt(com)})\n"
    if inertia is not None:
        text += f"{p}    float3 physics:diagonalInertia = ({fmt(inertia)})\n"
    text += (f"{p}    bool physics:rigidBodyEnabled = 1\n"
             + (f"{p}    float physxContactReport:threshold = 0\n" if contact_report else "") +
             f"{p}    float physxRigidBody:maxDepenetrationVelocity = 1\n"
             f"{p}    float physxRigidBody:sleepThreshold = 0\n"
             f"{p}    int physxRigidBody:solverPositionIterationCount = 32\n"
             f"{p}    int physxRigidBody:solverVelocityIterationCount = 8\n"
             f"{p}    quatf xformOp:orient = (1, 0, 0, 0)\n"
             f"{p}    double3 xformOp:translate = ({fmt(position)})\n"
             f'{p}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
             + children + f"{p}}}\n\n")
    return text


def hinge(name, body0, body1, anchor, lower, upper, *, stiffness, target, damping, armature):
    """Revolute joint about z. Limits and target in radians here; USD wants degrees and gains per degree."""
    p = I * 2
    return (f'{p}def PhysicsRevoluteJoint "{name}" (\n{p}    prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysxJointAPI"]\n{p})\n{p}{{\n'
            f"{p}    float drive:angular:physics:damping = {damping * math.pi / 180:.6g}\n"
            f"{p}    float drive:angular:physics:stiffness = {stiffness * math.pi / 180:.6g}\n"
            f"{p}    float drive:angular:physics:targetPosition = {math.degrees(target):.6g}\n"
            f"{p}    float drive:angular:physics:targetVelocity = 0\n"
            f'{p}    uniform token drive:angular:physics:type = "force"\n'
            f'{p}    uniform token physics:axis = "Z"\n'
            f"{p}    rel physics:body0 = </Fixture/Clamp/Bodies/{body0}>\n"
            f"{p}    rel physics:body1 = </Fixture/Clamp/Bodies/{body1}>\n"
            f"{p}    point3f physics:localPos0 = ({fmt(anchor)})\n"
            f"{p}    point3f physics:localPos1 = ({fmt(anchor)})\n"
            f"{p}    quatf physics:localRot0 = (1, 0, 0, 0)\n"
            f"{p}    quatf physics:localRot1 = (1, 0, 0, 0)\n"
            f"{p}    float physics:lowerLimit = {math.degrees(lower):.6g}\n"
            f"{p}    float physics:upperLimit = {math.degrees(upper):.6g}\n"
            f"{p}    float physxJoint:armature = {armature:.6g}\n"
            f"{p}    float physxJoint:maxJointVelocity = 720\n"
            f"{p}}}\n\n")


def material(name):
    color, roughness, metallic, opacity, static, dynamic = MATERIALS[name]
    extra = f"                float inputs:opacity = {opacity}\n" if opacity < 1 else ""
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {dynamic}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {static}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({fmt(color)})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n" + extra +
            ("                float inputs:ior = 1.47\n" if name.startswith("glass") else "") +
            "                token outputs:surface\n            }\n        }\n\n")


# ---- the parts ---------------------------------------------------------------------------------------------
def holder_children():
    out = "".join(hull_mesh(n, v, "glass" if n != "stopper" else "silicone", visible=False, offset=.001)
                  for n, v in L.holder_pieces().items())
    z = L.JOINT_Z
    # the flask: heavy-wall conical filtering flask, drawn as one glass surface with a rolled rim
    flask = [(0., .002), (L.FLASK_R - .006, 0.), (L.FLASK_R - .001, .002), (L.FLASK_R, .008), (L.FLASK_NECK_R + .002, L.FLASK_CONE_TOP - .006),
             (L.FLASK_NECK_R, L.FLASK_CONE_TOP + .004), (L.FLASK_NECK_R, L.FLASK_RIM - .003), (L.FLASK_NECK_R + .0018, L.FLASK_RIM - .0015),
             (L.FLASK_NECK_R + .0012, L.FLASK_RIM), (L.FLASK_NECK_R - .003, L.FLASK_RIM)]
    out += revolve("flask_glass", flask, "glass")
    # the side-arm with its hose barb, pointing away from the robot
    out += tube_display("sidearm", (L.FLASK_NECK_R - .002, 0., .186), (.060, 0., .186), .0055, "glass_edge")
    out += tube_display("hose_barb", (.058, 0., .186), (.072, 0., .186), .0035, "glass_edge")
    sr, sb, sh = L.STOPPER
    out += revolve("stopper_body", [(0., L.STOPPER_TOP), (sr, L.STOPPER_TOP), (sb, L.STOPPER_TOP - sh), (0., L.STOPPER_TOP - sh)],
                   "silicone", n=48)
    # the fritted base: drip stem, funnel-shaped body, the ground flange with its bevel; frit and membrane
    base = [(L.STEM_R - .001, L.STEM_TIP), (L.STEM_R, L.STEM_TIP + .003), (L.STEM_R, L.CUP_BOTTOM_Z),
            (L.CUP_BOTTOM_R + .002, L.CUP_BOTTOM_Z + .004), (L.CHAMFER_R - .002, z - L.FLANGE_T - .001),
            (L.CHAMFER_R, z - L.FLANGE_T), (L.FLANGE_R, z - L.CHAMFER_Z), (L.FLANGE_R, z), (L.FRIT_R + .001, z)]
    out += revolve("base_glass", base, "glass")
    out += revolve("flange_edge", [(L.FLANGE_R - .0004, z - L.CHAMFER_Z), (L.FLANGE_R - .0004, z)], "glass_edge")
    out += revolve("frit", [(0., z - .0035), (L.FRIT_R, z - .0035), (L.FRIT_R, z - .0002), (0., z - .0002)], "frit", n=48)
    out += revolve("membrane", [(0., z + .00005), (L.MEMBRANE_R, z + .00005), (L.MEMBRANE_R, z + .00012), (0., z + .00012)],
                   "membrane", n=64)
    return out


def funnel_children():
    out = "".join(hull_mesh(n, v, "glass", visible=False, offset=.001) for n, v in L.funnel_pieces().items())
    w = L.FUNNEL_WALL
    outer = [(L.FRIT_R + .001, 0.), (L.FLANGE_R, 0.), (L.FLANGE_R, L.CHAMFER_Z), (L.CHAMFER_R, L.FLANGE_T),
             (L.NECK_R, L.FLANGE_T), (L.NECK_R, L.NECK_TOP), (L.BODY_R, L.TAPER_TOP), (L.BODY_R, L.FUNNEL_H - .002),
             (L.BODY_R + .0012, L.FUNNEL_H - .0008), (L.BODY_R + .0008, L.FUNNEL_H), (L.BODY_R - w, L.FUNNEL_H),
             (L.BODY_R - w, L.TAPER_TOP + .002), (L.NECK_R - w, L.NECK_TOP + .001), (L.FRIT_R + .001, .003), (L.FRIT_R + .001, 0.)]
    out += revolve("funnel_glass", outer, "glass")
    out += revolve("funnel_flange_edge", [(L.FLANGE_R - .0004, 0.), (L.FLANGE_R - .0004, L.CHAMFER_Z)], "glass_edge")
    # printed graduations (100 to 250 mL, every 50) facing the robot, as on a graduated funnel
    area = math.pi * (L.BODY_R - w) ** 2
    for i, ml in enumerate((100, 150, 200, 250)):
        h = L.TAPER_TOP + ml * 1e-6 / area
        length = .012 if ml % 100 == 0 else .007
        a = math.pi                                           # on the -x side
        c = ((L.BODY_R + .0002) * math.cos(a), (L.BODY_R + .0002) * math.sin(a), h)
        out += box_display(f"grad_{ml}", c, (.0002, length / 2, .0004), "print")
    return out


def clamp_parts():
    bodies = ""
    knuckle = "".join(hull_mesh(n, v, "steel", visible=True) for n, v in L.knuckle_pieces().items())
    # display: the torsion spring's coil round the pin
    coil = revolve("coil", [(.0062, -.007), (.0068, -.006), (.0062, -.005), (.0068, -.004), (.0062, -.003), (.0068, -.002),
                            (.0062, .002), (.0068, .003), (.0062, .004), (.0068, .005), (.0062, .006), (.0068, .007)],
                   "steel", n=24, centre=(*L.PIVOT, 0.))
    bodies += rigid_body("knuckle", L.KNUCKLE_MASS, scene.CLAMP_START + (0., 0., scene.TOP), knuckle + coil, indent=3)
    for tag, side in (("L", 1), ("R", -1)):
        parts = "".join(hull_mesh(n, v, "anodised", visible=True) for n, v in L.jaw_pieces(side).items())
        bodies += rigid_body(f"jaw{tag}", L.JAW_MASS, scene.CLAMP_START + (0., 0., scene.TOP), parts, indent=3)
    joints = ""
    anchor = (*L.PIVOT, 0.)
    for tag, sign in (("L", 1), ("R", -1)):
        lo, hi = sorted((sign * L.THETA_MIN, sign * L.THETA_MAX))
        joints += hinge(f"hinge{tag}", "knuckle", f"jaw{tag}", anchor, lo, hi, stiffness=L.SPRING,
                        target=-sign * L.PRELOAD, damping=.002, armature=2e-5)
    return ('    def Xform "Clamp" (\n        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n    )\n    {\n'
            "        bool physxArticulation:enabledSelfCollisions = 0\n"
            "        int physxArticulation:solverPositionIterationCount = 32\n"
            "        int physxArticulation:solverVelocityIterationCount = 8\n"
            "        float physxArticulation:sleepThreshold = 0\n"
            "        float physxArticulation:stabilizationThreshold = 0\n\n"
            '        def Scope "Bodies"\n        {\n' + bodies + "        }\n\n"
            '        def Scope "Joints"\n        {\n' + joints + "        }\n    }\n\n")


def usd():
    top = np.array((0., 0., scene.TOP))
    holder_com = (0., 0., (L.FLASK_MASS * .060 + L.BASE_MASS * .240) / L.HOLDER_MASS)
    actors = rigid_body("holder", L.HOLDER_MASS, scene.HOLDER_START + top, holder_children(),
                        com=holder_com, inertia=(.0042, .0042, .0013))
    r, h = L.BODY_R, L.FUNNEL_H
    actors += rigid_body("funnel", L.FUNNEL_MASS, scene.FUNNEL_START + top, funnel_children(),
                         com=(0., 0., L.FUNNEL_COM), inertia=(L.FUNNEL_MASS * (3 * r * r + h * h) / 12,) * 2 + (L.FUNNEL_MASS * r * r,))
    # display only: the clean wipe the funnel stands on
    wipe = box_display("wipe", (*L.FUNNEL_START, .00025), (.055, .045, .00025), "wipe", indent=2)
    # A fixed side-view gauge beside the flask. The upper edge of the stripe is 20 mm above the bench.
    gauge = box_display("lift_gauge", (.330, -.109, scene.TOP + .013), (.012, .001, .013), "wipe", indent=2)
    gauge += box_display("lift_mark", (.330, -.1101, scene.TOP + .019), (.012, .0002, .001), "anodised", indent=2)
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            '    def Scope "Actors"\n    {\n' + actors + "    }\n\n"
            + clamp_parts() + wipe + gauge
            + '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    text = usd()
    scene.USD.write_text(text)
    print("wrote", scene.USD, len(text.splitlines()), "lines", len(text) // 1024, "KiB")
