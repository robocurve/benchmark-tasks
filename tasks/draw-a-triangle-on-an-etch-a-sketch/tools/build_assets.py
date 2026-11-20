"""Regenerate ``rdb_etch_triangle/etch_a_sketch.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs only NumPy.

The toy is one free articulation of three bodies: the frame (root) and two knobs on revolute joints about the
vertical, with no end stops. Collision shapes are plain boxes and cylinders and are not rendered; what the
cameras see are display meshes of the same size with no collision: the red frame with rounded corners, its
raised lip round the screen, the silver-grey screen and two ribbed white knobs. The drawn line is not in this
file: the adapter draws it at run time from the stylus position.
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_etch_triangle import fixed_scene as scene  # noqa: E402
from rdb_etch_triangle import layout as L  # noqa: E402

# colour, roughness, metallic, friction
MATERIALS = {
    "red_plastic": ((.72, .03, .03), .35, 0., .40),          # the frame; plastic on the wooden table
    "screen": ((.60, .60, .58), .55, .35, .40),               # aluminium powder behind glass
    "knob": ((.93, .93, .90), .45, 0., .90),                  # ribbed white plastic under rubber-like pads
    "stem": ((.85, .85, .82), .5, 0., .50),
}
_PREFIX = "</Fixture/Materials/"
CORNER_SEGMENTS = 6
KNOB_RIBS = 40
RING_SAMPLES = 96


def _f(v):
    return f"{v:.6g}"


def _points(points):
    return "[" + ", ".join(f"({_f(x)}, {_f(y)}, {_f(z)})" for x, y, z in points) + "]"


def _collide_lines(material):
    return (f"                rel material:binding:physics = {_PREFIX}{material}>\n"
            "                float physxCollision:contactOffset = 0.001\n"
            "                float physxCollision:restOffset = 0\n")


def box(name, material, centre, half):
    """An unrendered box collider."""
    return (f'            def Cube "{name}" (\n'
            '                prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]\n'
            "            )\n            {\n"
            f"                rel material:binding = {_PREFIX}{material}>\n" + _collide_lines(material) +
            '                token visibility = "invisible"\n'
            "                double size = 2\n"
            f"                float3 xformOp:scale = ({_f(half[0])}, {_f(half[1])}, {_f(half[2])})\n"
            f"                double3 xformOp:translate = ({_f(centre[0])}, {_f(centre[1])}, {_f(centre[2])})\n"
            '                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
            "            }\n\n")


def cylinder(name, material, centre, radius, height):
    """An unrendered vertical cylinder collider."""
    return (f'            def Cylinder "{name}" (\n'
            '                prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]\n'
            "            )\n            {\n"
            f"                rel material:binding = {_PREFIX}{material}>\n" + _collide_lines(material) +
            '                token visibility = "invisible"\n'
            '                uniform token axis = "Z"\n'
            f"                double height = {_f(height)}\n                double radius = {_f(radius)}\n"
            f"                double3 xformOp:translate = ({_f(centre[0])}, {_f(centre[1])}, {_f(centre[2])})\n"
            '                uniform token[] xformOpOrder = ["xformOp:translate"]\n'
            "            }\n\n")


def mesh(name, material, points, faces):
    """A rendered mesh with no collision."""
    return (f'            def Mesh "{name}" (\n                prepend apiSchemas = ["MaterialBindingAPI"]\n            )\n'
            "            {\n"
            f"                rel material:binding = {_PREFIX}{material}>\n"
            "                uniform bool doubleSided = 1\n"
            f"                int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"                int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"                point3f[] points = {_points(points)}\n"
            '                uniform token subdivisionScheme = "none"\n'
            "            }\n\n")


# ---------------------------------------------------------------------------------------------------------
# display shapes

def rounded_rect(half_x, half_y, radius, centre=(0., 0.)):
    """Counter-clockwise outline of a rectangle with rounded corners."""
    out = []
    for cx, cy, a0 in ((half_x - radius, half_y - radius, 0.), (-half_x + radius, half_y - radius, 90.),
                       (-half_x + radius, -half_y + radius, 180.), (half_x - radius, -half_y + radius, 270.)):
        for k in range(CORNER_SEGMENTS + 1):
            a = math.radians(a0 + 90. * k / CORNER_SEGMENTS)
            out.append((centre[0] + cx + radius * math.cos(a), centre[1] + cy + radius * math.sin(a)))
    return np.array(out)


def ray_outline(outline, centre, samples=RING_SAMPLES):
    """Where rays from ``centre`` at evenly spaced angles leave a star-shaped closed outline."""
    outline = np.asarray(outline)
    out = []
    for k in range(samples):
        a = 2 * math.pi * k / samples
        d = np.array((math.cos(a), math.sin(a)))
        best = None
        for p, q in zip(outline, np.roll(outline, -1, axis=0)):
            e = q - p
            m = np.array(((d[0], -e[0]), (d[1], -e[1])))
            if abs(np.linalg.det(m)) < 1e-12:
                continue
            t, u = np.linalg.solve(m, p - centre)
            if t > 0 and -1e-9 <= u <= 1 + 1e-9 and (best is None or t < best):
                best = t
        out.append(centre + best * d)
    return np.array(out)


def ring_mesh(outer, inner, z_outer, z_inner, bottom=None):
    """Band between two outlines with the same number of points (outer at z_outer, inner at z_inner), with
    outer walls down to ``bottom`` when given."""
    n = len(outer)
    points = [(x, y, z_outer) for x, y in outer] + [(x, y, z_inner) for x, y in inner]
    faces = [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    if bottom is not None:
        points += [(x, y, bottom) for x, y in outer]
        faces += [[2 * n + i, 2 * n + (i + 1) % n, (i + 1) % n, i] for i in range(n)]
    return points, faces


def frame_display():
    """Display meshes of the frame, in the frame body's coordinates (origin on the table under the middle)."""
    hx, hy = L.BODY[0] / 2, L.BODY[1] / 2
    sx0, sy0 = L.SCREEN_LL
    sx1, sy1 = sx0 + L.SCREEN[0], sy0 + L.SCREEN[1]
    centre = np.array(((sx0 + sx1) / 2, (sy0 + sy1) / 2))
    screen_rect = np.array(((sx1, sy1), (sx0, sy1), (sx0, sy0), (sx1, sy0)))
    outer = ray_outline(rounded_rect(hx, hy, L.CORNER_R), centre)
    shoulder = ray_outline(rounded_rect(hx - .003, hy - .003, L.CORNER_R - .002), centre)
    lip_out = ray_outline(rounded_rect(L.SCREEN[0] / 2 + L.RIDGE_W, L.SCREEN[1] / 2 + L.RIDGE_W, .004, centre), centre)
    hole = ray_outline(screen_rect, centre)
    parts = []
    # the frame's sides with a rounded-off top edge, then its face out to the lip
    points, faces = ring_mesh(shoulder, outer, L.BODY_H, L.BODY_H - .002, bottom=.0005)
    parts.append(mesh("frame_edge", "red_plastic", points, faces))
    points, faces = ring_mesh(shoulder, lip_out, L.BODY_H, L.BODY_H)
    parts.append(mesh("frame_face", "red_plastic", points, faces))
    # the raised lip round the screen: outer wall, top, and the inner wall down to the screen
    lip_in = ray_outline(rounded_rect(L.SCREEN[0] / 2 + .0008, L.SCREEN[1] / 2 + .0008, .001, centre), centre)
    points, faces = ring_mesh(lip_out, lip_out, L.BODY_H, L.RIDGE_H)
    parts.append(mesh("lip_outer_wall", "red_plastic", points, faces))
    points, faces = ring_mesh(lip_out, lip_in, L.RIDGE_H, L.RIDGE_H)
    parts.append(mesh("lip_top", "red_plastic", points, faces))
    points, faces = ring_mesh(lip_in, hole, L.RIDGE_H, L.SCREEN_Z)
    parts.append(mesh("lip_inner_wall", "red_plastic", points, faces))
    # the screen
    screen = [(sx0, sy0, L.SCREEN_Z), (sx1, sy0, L.SCREEN_Z), (sx1, sy1, L.SCREEN_Z), (sx0, sy1, L.SCREEN_Z)]
    parts.append(mesh("screen", "screen", screen, [[0, 1, 2, 3]]))
    # underside
    bottom = [(x, y, .0005) for x, y in outer]
    parts.append(mesh("frame_bottom", "red_plastic", bottom, [list(range(len(bottom) - 1, -1, -1))]))
    return "".join(parts)


def knob_display():
    """A ribbed white knob with a short stem, origin on the knob's axis at the frame face."""
    ribs = []
    for k in range(2 * KNOB_RIBS):
        a = 2 * math.pi * k / (2 * KNOB_RIBS)
        r = L.KNOB_R if k % 2 == 0 else L.KNOB_R - .0007
        ribs.append((r * math.cos(a), r * math.sin(a)))
    n = len(ribs)
    z0, z1 = L.KNOB_BOTTOM - L.BODY_H, L.KNOB_TOP - L.BODY_H
    inset = [(x * .93, y * .93) for x, y in ribs]
    points = ([(x, y, z0) for x, y in ribs] + [(x, y, z1 - .0012) for x, y in ribs] + [(x, y, z1) for x, y in inset]
              + [(0., 0., z1), (0., 0., z0)])
    faces = [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    faces += [[n + i, n + (i + 1) % n, 2 * n + (i + 1) % n, 2 * n + i] for i in range(n)]
    faces += [[2 * n + i, 2 * n + (i + 1) % n, 3 * n] for i in range(n)]
    faces += [[(i + 1) % n, i, 3 * n + 1] for i in range(n)]
    out = mesh("knob_look", "knob", points, faces)
    stem = []
    m = 16
    for k in range(m):
        a = 2 * math.pi * k / m
        stem.append((L.STEM_R * math.cos(a), L.STEM_R * math.sin(a)))
    points = [(x, y, 0.) for x, y in stem] + [(x, y, z0) for x, y in stem]
    faces = [[i, (i + 1) % m, m + (i + 1) % m, m + i] for i in range(m)]
    out += mesh("stem_look", "stem", points, faces)
    # a small dot on the knob's top so that its turning shows in the video
    a = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    dot = [(L.KNOB_R * .62 + .0018 * math.cos(t), .0018 * math.sin(t), z1 + .0002) for t in a]
    out += mesh("turn_mark", "stem", dot, [list(range(12))])
    return out


# ---------------------------------------------------------------------------------------------------------

def body(name, mass, position, yaw, children, *, root=False):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        extra = ("            bool physxArticulation:enabledSelfCollisions = 0\n"
                 "            int physxArticulation:solverPositionIterationCount = 32\n"
                 "            int physxArticulation:solverVelocityIterationCount = 8\n"
                 "            float physxArticulation:sleepThreshold = 0\n"
                 "            float physxArticulation:stabilizationThreshold = 0\n")
    w, z = math.cos(yaw / 2), math.sin(yaw / 2)
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            f"            float physics:mass = {_f(mass)}\n"
            "            bool physics:rigidBodyEnabled = 1\n" + extra +
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 0.5\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"            quatf xformOp:orient = ({w:.7g}, 0, 0, {z:.7g})\n"
            f"            double3 xformOp:translate = ({_f(position[0])}, {_f(position[1])}, {_f(position[2])})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def knob_joint(name, anchor):
    """Revolute joint about the toy's vertical through ``anchor`` (frame body coordinates), no end stops. The drag
    is a velocity drive toward rest whose torque is capped at the turning torque (a friction-like drag)."""
    return (f'        def PhysicsRevoluteJoint "{name}" (\n'
            '            prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysxJointAPI"]\n        )\n        {\n'
            f"            float drive:angular:physics:damping = {_f(L.KNOB_DAMPING)}\n"
            f"            float drive:angular:physics:maxForce = {_f(L.KNOB_FRICTION)}\n"
            "            float drive:angular:physics:stiffness = 0\n"
            "            float drive:angular:physics:targetPosition = 0\n"
            "            float drive:angular:physics:targetVelocity = 0\n"
            '            uniform token drive:angular:physics:type = "force"\n'
            '            uniform token physics:axis = "Z"\n'
            "            rel physics:body0 = </Fixture/Actors/frame>\n"
            f"            rel physics:body1 = </Fixture/Actors/{name}>\n"
            f"            point3f physics:localPos0 = ({_f(anchor[0])}, {_f(anchor[1])}, {_f(anchor[2])})\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            "            quatf physics:localRot0 = (1, 0, 0, 0)\n"
            "            quatf physics:localRot1 = (1, 0, 0, 0)\n"
            "            float physxJoint:armature = 0.00002\n"
            "            float physxJoint:maxJointVelocity = 1440\n"
            "        }\n\n")


def material(name):
    color, roughness, metallic, friction = MATERIALS[name]
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
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


FRAME_MASS = L.MASS - 2 * L.KNOB_MASS


def usd():
    top = scene.TOP
    yaw = scene.TOY_YAW

    def place(point_toy):
        p = scene.to_station(point_toy)
        return (p[0], p[1], p[2] + top)

    hx, hy = L.BODY[0] / 2, L.BODY[1] / 2
    sx0, sy0 = L.SCREEN_LL
    sx1, sy1 = sx0 + L.SCREEN[0], sy0 + L.SCREEN[1]
    lip_h = (L.RIDGE_H - L.BODY_H) / 2
    lip_z = (L.RIDGE_H + L.BODY_H) / 2
    w = L.RIDGE_W / 2
    parts = [box("core", "red_plastic", (0, 0, L.BODY_H / 2), (hx, hy, L.BODY_H / 2)),
             box("lip_near", "red_plastic", ((sx0 + sx1) / 2, sy0 - w, lip_z), (L.SCREEN[0] / 2 + L.RIDGE_W, w, lip_h)),
             box("lip_far", "red_plastic", ((sx0 + sx1) / 2, sy1 + w, lip_z), (L.SCREEN[0] / 2 + L.RIDGE_W, w, lip_h)),
             box("lip_left", "red_plastic", (sx0 - w, (sy0 + sy1) / 2, lip_z), (w, L.SCREEN[1] / 2, lip_h)),
             box("lip_right", "red_plastic", (sx1 + w, (sy0 + sy1) / 2, lip_z), (w, L.SCREEN[1] / 2, lip_h))]
    bodies = body("frame", FRAME_MASS, place((0, 0, 0)), yaw, "".join(parts) + frame_display(), root=True)
    joints = ""
    for name in scene.JOINTS:
        kx, ky = scene.KNOB_TOY[name]
        children = cylinder("grip", "knob", (0, 0, (L.KNOB_BOTTOM + L.KNOB_TOP) / 2 - L.BODY_H), L.KNOB_R, L.KNOB_H)
        children += cylinder("stem", "stem", (0, 0, (L.KNOB_BOTTOM - L.BODY_H) / 2), L.STEM_R, L.KNOB_BOTTOM - L.BODY_H)
        children += knob_display()
        bodies += body(name, L.KNOB_MASS, place((kx, ky, L.BODY_H)), yaw, children)
        joints += knob_joint(name, (kx, ky, L.BODY_H))
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    (ROOT / "rdb_etch_triangle/etch_a_sketch.usda").write_text(usd())
    print("written", len(usd()), "characters")
