"""Regenerate ``bag.usda`` from ``bag_model.py``.   Run:  python -m rdb_bag_load.build_scene

``/Fixture/Actors`` holds every body:

* ``base`` and the four walls and two handles are one floating articulation rooted on the loose base board. Each
  wall is two rigid panels: the lower one hinged along an edge of the base board, the upper one hinged to the lower
  one along the wall's crease. The large walls fold with the crease inside the bag, the end walls with the crease
  outside. Each large wall carries a rigid handle loop hinged along its rim.
* Four corner seams tie the top corners of neighbouring walls together: joints outside the articulation that are
  slack up to ``SEAM_SLACK`` and then go taut.
* ``carton`` and ``pack`` are free rigid boxes.

Every collider is a box. Print on the carton and the pack is drawn only.
"""

import math

import numpy as np

from . import bag_model as m
from . import fixed_scene as scene

TOP = scene.TOP
# colour, roughness, friction
MATERIALS = {
    "fabric": ((.10, .32, .30), .95, m.FRICTION["fabric"]),
    "fabric_end": ((.08, .27, .26), .95, m.FRICTION["fabric"]),
    "board": ((.07, .20, .19), .9, m.FRICTION["board"]),
    "strap": ((.85, .80, .62), .9, m.FRICTION["strap"]),
    "carton": ((.78, .16, .10), .6, m.FRICTION["carton"]),
    "carton_print": ((.95, .85, .45), .6, m.FRICTION["carton"]),
    "film": ((.93, .94, .96), .35, m.FRICTION["film"]),
    "film_print": ((.25, .45, .80), .35, m.FRICTION["film"]),
}
PREFIX = "</Fixture/Materials/"
ACTORS = "/Fixture/Actors/"
PAD = "            "


def quat(rotation):
    """(w, x, y, z) of a rotation matrix."""
    r = np.asarray(rotation, dtype=float)
    w = math.sqrt(max(0., 1. + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return (w, (r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w))
    i = int(np.argmax(np.diag(r)))                  # half a turn: take the axis from the largest diagonal term
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0., 1. + r[i, i] - r[j, j] - r[k, k])) * 2
    v = [0., 0., 0.]
    v[i], v[j], v[k] = s / 4, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s
    return ((r[k, j] - r[j, k]) / s, *v)


def cube(name, material, centre, half, *, collide=True, pad=PAD):
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    text = f'{pad}def Cube "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
    text += f"{pad}    rel material:binding = {PREFIX}{material}>\n"
    if collide:
        text += (f"{pad}    rel material:binding:physics = {PREFIX}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = 0.001\n{pad}    float physxCollision:restOffset = 0\n")
    text += (f"{pad}    double size = 2\n"
             f"{pad}    float3 xformOp:scale = ({half[0]:.6g}, {half[1]:.6g}, {half[2]:.6g})\n"
             f"{pad}    double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
             f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n{pad}}}\n\n')
    return text


def slab(name, material, x, y, z, **kwargs):
    """A box given by its (low, high) extents on each axis."""
    return cube(name, material, [(a + b) / 2 for a, b in (x, y, z)], [(b - a) / 2 for a, b in (x, y, z)], **kwargs)


def drawn_disc(name, material, centre, radius, pad=PAD):
    return (f'{pad}def Cylinder "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {PREFIX}{material}>\n"
            f'{pad}    uniform token axis = "Z"\n{pad}    double height = 0.0006\n'
            f"{pad}    double radius = {radius:.6g}\n"
            f"{pad}    double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate"]\n{pad}}}\n\n')


def body(name, mass, position, rotation, children, *, root=False, pad="        "):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        extra = (f"{pad}    bool physxArticulation:enabledSelfCollisions = 1\n"
                 f"{pad}    int physxArticulation:solverPositionIterationCount = 64\n"
                 f"{pad}    int physxArticulation:solverVelocityIterationCount = 8\n"
                 f"{pad}    float physxArticulation:sleepThreshold = 0\n"
                 f"{pad}    float physxArticulation:stabilizationThreshold = 0\n")
    w, x, y, z = quat(rotation)
    return (f'{pad}def Xform "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            f"{pad}    float physics:mass = {mass:.6g}\n{pad}    bool physics:rigidBodyEnabled = 1\n" + extra +
            f"{pad}    float physxContactReport:threshold = 0\n"
            f"{pad}    float physxRigidBody:maxDepenetrationVelocity = 1\n"
            f"{pad}    float physxRigidBody:sleepThreshold = 0\n"
            f"{pad}    float physxRigidBody:angularDamping = 0.5\n"
            f"{pad}    float physxRigidBody:linearDamping = 0.1\n"
            f"{pad}    int physxRigidBody:solverPositionIterationCount = 64\n"
            f"{pad}    int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"{pad}    quatf xformOp:orient = ({w:.8g}, {x:.8g}, {y:.8g}, {z:.8g})\n"
            f"{pad}    double3 xformOp:translate = ({position[0]:.7g}, {position[1]:.7g}, {position[2]:.7g})\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}{pad}}}\n\n")


def hinge(name, body0, body1, anchor0, anchor1, frame0, limit, stiffness, damping, armature, *, rest=0., start=0.,
          pad="        "):
    """A revolute joint about the joint frame's +x, sprung toward ``rest``, starting at ``start``. ``frame0`` is the
    joint frame in ``body0``; in ``body1`` it is the body's own frame. Angles in the file are degrees. PhysX starts
    an articulation's joints from the joint state written here, not from the bodies' poses."""
    per_deg = math.pi / 180.
    w, x, y, z = quat(frame0)
    return (f'{pad}def PhysicsRevoluteJoint "{name}" (\n'
            f'{pad}    prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysxJointAPI", "PhysicsJointStateAPI:angular"]\n{pad})\n{pad}{{\n'
            f"{pad}    float state:angular:physics:position = {math.degrees(start):.7g}\n"
            f"{pad}    float state:angular:physics:velocity = 0\n"
            f"{pad}    float drive:angular:physics:damping = {damping * per_deg:.7g}\n"
            f"{pad}    float drive:angular:physics:stiffness = {stiffness * per_deg:.7g}\n"
            f"{pad}    float drive:angular:physics:targetPosition = {math.degrees(rest):.7g}\n"
            f"{pad}    float drive:angular:physics:targetVelocity = 0\n"
            f'{pad}    uniform token drive:angular:physics:type = "force"\n'
            f'{pad}    uniform token physics:axis = "X"\n'
            f"{pad}    rel physics:body0 = <{ACTORS}{body0}>\n{pad}    rel physics:body1 = <{ACTORS}{body1}>\n"
            f"{pad}    point3f physics:localPos0 = ({anchor0[0]:.6g}, {anchor0[1]:.6g}, {anchor0[2]:.6g})\n"
            f"{pad}    point3f physics:localPos1 = ({anchor1[0]:.6g}, {anchor1[1]:.6g}, {anchor1[2]:.6g})\n"
            f"{pad}    quatf physics:localRot0 = ({w:.8g}, {x:.8g}, {y:.8g}, {z:.8g})\n"
            f"{pad}    quatf physics:localRot1 = (1, 0, 0, 0)\n"
            f"{pad}    float physics:lowerLimit = {math.degrees(limit[0]):.7g}\n"
            f"{pad}    float physics:upperLimit = {math.degrees(limit[1]):.7g}\n"
            f"{pad}    float physxJoint:armature = {armature:.7g}\n"
            f"{pad}    float physxJoint:maxJointVelocity = 720\n{pad}}}\n\n")


def seam(name, body0, body1, anchor0, anchor1, pad="        "):
    """A corner seam: free in every direction up to SEAM_SLACK between the two tied points, then a stiff spring.
    A D6 joint with only a distance limit; it is not part of the articulation."""
    return (f'{pad}def PhysicsJoint "{name}" (\n'
            f'{pad}    prepend apiSchemas = ["PhysicsLimitAPI:distance", "PhysxLimitAPI:distance"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel physics:body0 = <{ACTORS}{body0}>\n{pad}    rel physics:body1 = <{ACTORS}{body1}>\n"
            f"{pad}    bool physics:excludeFromArticulation = 1\n"
            f"{pad}    bool physics:collisionEnabled = 1\n"
            f"{pad}    point3f physics:localPos0 = ({anchor0[0]:.6g}, {anchor0[1]:.6g}, {anchor0[2]:.6g})\n"
            f"{pad}    point3f physics:localPos1 = ({anchor1[0]:.6g}, {anchor1[1]:.6g}, {anchor1[2]:.6g})\n"
            f"{pad}    quatf physics:localRot0 = (1, 0, 0, 0)\n{pad}    quatf physics:localRot1 = (1, 0, 0, 0)\n"
            f"{pad}    float limit:distance:physics:low = 0\n"
            f"{pad}    float limit:distance:physics:high = {m.SEAM_SLACK:.6g}\n"
            f"{pad}    float physxLimit:distance:stiffness = {m.SEAM_K:.6g}\n"
            f"{pad}    float physxLimit:distance:damping = {m.SEAM_DAMPING:.6g}\n{pad}}}\n\n")


def material(name):
    color, roughness, friction = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:metallic = 0\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


# ---------------------------------------------------------------------------------------------------------
def handle_shapes():
    """The handle loop in its own frame (origin on the rim, +z away from the rim, x along the rim)."""
    hs, t = m.HANDLE_SPAN / 2, m.LEG_T / 2
    reach = m.HANDLE_DROP - m.GRIP_W
    text = slab("leg_a", "strap", (-hs, -hs + m.LEG_W), (-t, t), (.002, reach))
    text += slab("leg_b", "strap", (hs - m.LEG_W, hs), (-t, t), (.002, reach))
    text += slab("grip", "strap", (-hs, hs), (-m.GRIP_T / 2, m.GRIP_T / 2), (reach, m.HANDLE_DROP))
    return text


def actors(angles=None):
    q = dict(m.START if angles is None else angles)
    origin = np.array((*m.BAG_XY, TOP))
    hx, hy = m.BASE_X / 2, m.BASE_Y / 2
    board = slab("board", "board", (-hx, hx), (-hy, hy), (0., m.BASE_T))
    text = body("base", m.BASE_MASS, origin, np.eye(3), board, root=True)
    half_t, gap = m.WALL_T / 2, .0005
    for wall in m.WALLS:
        look = "fabric" if wall in m.HANDLES else "fabric_end"
        pose = m.wall_pose(wall, q[m.LEAN[wall]], q[m.FOLD[wall]], q.get(m.SWING.get(wall), 0.))
        panel = cube("fabric", look, (0., 0., 0.), (m.HALF_LENGTH[wall], half_t, m.SEG / 2 - gap))
        for part, names in (("lower", m.LOWER), ("upper", m.UPPER)):
            position, rotation = pose[part]
            text += body(names[wall], m.PANEL_MASS[wall], origin + position, rotation, panel)
        if wall in m.HANDLES:
            position, rotation = pose["handle"]
            text += body(m.HANDLES[wall], m.HANDLE_MASS, origin + position, rotation, handle_shapes())
    # the carton: a box of crackers with a printed band, standing on its 60 x 132 end
    cx, cy, cz = (v / 2 for v in m.CARTON)
    shape = cube("box", "carton", (0., 0., 0.), (cx, cy, cz))
    for side in (-1., 1.):
        shape += slab(f"band_{'a' if side < 0 else 'b'}", "carton_print", sorted((side * cx, side * (cx + .0004))),
                      (-cy + .012, cy - .012), (-.030, .050), collide=False)
    text += body("carton", m.CARTON_MASS, (*m.CARTON_XY, TOP + cz + .0005), np.eye(3), shape)
    # the pack: four rolls in film, two across (along x) and two high; the rolls' ends and the film's print are drawn
    px, py, pz = (v / 2 for v in m.PACK)
    shape = cube("box", "film", (0., 0., 0.), (px, py, pz))
    for k, x in enumerate((-px / 2, px / 2)):
        shape += drawn_disc(f"roll_{k}", "carton_print", (x, 0., pz + .0003), .050)
        shape += drawn_disc(f"core_{k}", "board", (x, 0., pz + .0008), .020)
    for side in (-1., 1.):
        shape += slab(f"print_{'a' if side < 0 else 'b'}", "film_print", (-px + .020, px - .020),
                      sorted((side * py, side * (py + .0004))), (-.025, .025), collide=False)
    text += body("pack", m.PACK_MASS, (*m.PACK_XY, TOP + pz + .0005), np.eye(3), shape)
    return text


def joints(angles=None):
    q = dict(m.START if angles is None else angles)
    text = ""
    for wall in m.WALLS:
        frame = m.wall_frame(wall)
        text += hinge(m.LEAN[wall], "base", m.LOWER[wall], m.hinge_point(wall), (0., 0., -m.SEG / 2), frame,
                      m.LEAN_LIMIT[wall], m.CREASE_K, m.CREASE_DAMPING, m.CREASE_ARMATURE, rest=m.LEAN_REST[wall],
                      start=q[m.LEAN[wall]])
        text += hinge(m.FOLD[wall], m.LOWER[wall], m.UPPER[wall], (0., 0., m.SEG / 2), (0., 0., -m.SEG / 2), np.eye(3),
                      m.FOLD_LIMIT[wall], m.CREASE_K, m.CREASE_DAMPING, m.CREASE_ARMATURE, rest=m.FOLD_REST[wall],
                      start=q[m.FOLD[wall]])
        if wall in m.HANDLES:
            text += hinge(m.SWING[wall], m.UPPER[wall], m.HANDLES[wall], (0., 0., m.SEG / 2), (0., 0., 0.), np.eye(3),
                          m.HANDLE_LIMIT, 0., m.HANDLE_DAMPING, m.HANDLE_ARMATURE, start=q[m.SWING[wall]])
    for a, b in m.CORNERS:
        text += seam(f"seam_{a}_{b}", m.UPPER[a], m.UPPER[b], m.corner_anchor(a, b), m.corner_anchor(b, a))
    return text


def usd(angles=None):
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            '    def Scope "Actors"\n    {\n' + actors(angles) + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints(angles) + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print(f"wrote {scene.USD} ({len(usd().splitlines())} lines)")
