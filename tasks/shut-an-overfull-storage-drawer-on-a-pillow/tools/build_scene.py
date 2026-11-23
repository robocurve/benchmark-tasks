"""Regenerate ``rdb_overfull_drawer/drawer.usda`` from ``layout.py``.

Run from the task folder: ``python tools/build_scene.py``. Needs only NumPy.

The frame is a set of fixed boxes. The drawer is one rigid body on a slide along x whose friction is a
force-capped damper. The pillow is a base slab lying loose in the drawer with 16 pads on it, each pad a rigid
box on its own vertical slide with a spring and a damper. Every part is a box with full collision; the pads do
not collide with one another or with their base. Nothing is drawn without a collider.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_overfull_drawer import fixed_scene as scene  # noqa: E402
from rdb_overfull_drawer import layout as L  # noqa: E402

# colour (linear RGB), roughness, opacity, friction key
MATERIALS = {
    "frame": ((.93, .93, .91), .35, 1., "frame"),
    "drawer": ((.90, .93, .95), .08, .22, "drawer"),
    "fabric_a": ((.05, .16, .42), .9, 1., "fabric"),
    "fabric_b": ((.07, .21, .50), .9, 1., "fabric"),
}
_PREFIX = "</Fixture/Materials/"
DRAWER_DAMPING = 30000.          # N s/m: with the force cap this is dry friction down to 0.1 mm/s
SLIDE_TO_MINUS_X = (0., 0., .70710677, .70710677)     # (w, x, y, z): the joint's axis is -x, so its value is "open"
SLIDE_TO_Z = (0., .70710677, 0., .70710677)           # the joint's axis is +z


def box(name, material, low, high, indent=12):
    """A colliding box given by two opposite corners in its parent's frame."""
    pad = " " * indent
    centre = [(a + b) / 2 for a, b in zip(low, high)]
    half = [(b - a) / 2 for a, b in zip(low, high)]
    return (f'{pad}def Cube "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {_PREFIX}{material}>\n"
            f"{pad}    rel material:binding:physics = {_PREFIX}{material}>\n"
            f"{pad}    float physxCollision:contactOffset = 0.001\n"
            f"{pad}    float physxCollision:restOffset = 0\n"
            f"{pad}    double size = 2\n"
            f"{pad}    float3 xformOp:scale = ({half[0]:.6g}, {half[1]:.6g}, {half[2]:.6g})\n"
            f"{pad}    double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
            f"{pad}}}\n\n")


def body(name, mass, position, children, no_contact_with=()):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    pairs = ""
    if no_contact_with:
        apis.append('"PhysicsFilteredPairsAPI"')
        pairs = ("            rel physics:filteredPairs = [\n"
                 + "".join(f"                </Fixture/Actors/{other}>,\n" for other in no_contact_with)
                 + "            ]\n")
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            + pairs +
            f"            float physics:mass = {mass:.6g}\n"
            "            bool physics:rigidBodyEnabled = 1\n"
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            "            quatf xformOp:orient = (1, 0, 0, 0)\n"
            f"            double3 xformOp:translate = ({position[0]:.6g}, {position[1]:.6g}, {position[2]:.6g})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def slide(name, body0, body1, anchor0, rotation, low, high, *, stiffness, damping, target=0., max_force=None):
    """A slide between two bodies (``body0`` None: the world). ``anchor0`` is in body0's frame and body1's
    origin is on it when the joint reads 0."""
    w, x, y, z = rotation
    text = (f'        def PhysicsPrismaticJoint "{name}" (\n            prepend apiSchemas = ["PhysicsDriveAPI:linear"]\n'
            "        )\n        {\n"
            f"            float drive:linear:physics:damping = {damping:.6g}\n")
    if max_force is not None:
        text += f"            float drive:linear:physics:maxForce = {max_force:.6g}\n"
    text += (f"            float drive:linear:physics:stiffness = {stiffness:.6g}\n"
             f"            float drive:linear:physics:targetPosition = {target:.6g}\n"
             "            float drive:linear:physics:targetVelocity = 0\n"
             '            uniform token drive:linear:physics:type = "force"\n'
             '            uniform token physics:axis = "X"\n')
    if body0 is not None:
        text += f"            rel physics:body0 = </Fixture/Actors/{body0}>\n"
    text += (f"            rel physics:body1 = </Fixture/Actors/{body1}>\n"
             "            bool physics:collisionEnabled = 0\n"
             "            uniform bool physics:excludeFromArticulation = 1\n"
             f"            point3f physics:localPos0 = ({anchor0[0]:.6g}, {anchor0[1]:.6g}, {anchor0[2]:.6g})\n"
             "            point3f physics:localPos1 = (0, 0, 0)\n"
             f"            quatf physics:localRot0 = ({w:.9g}, {x:.9g}, {y:.9g}, {z:.9g})\n"
             f"            quatf physics:localRot1 = ({w:.9g}, {x:.9g}, {y:.9g}, {z:.9g})\n"
             f"            float physics:lowerLimit = {low:.6g}\n"
             f"            float physics:upperLimit = {high:.6g}\n"
             "        }\n\n")
    return text


def material(name):
    color, roughness, opacity, key = MATERIALS[name]
    friction = L.FRICTION[key]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:opacity = {opacity}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


def frame_boxes():
    """(name, low corner, high corner) of the frame's fixed boxes, tabletop frame."""
    x0, x1 = L.FRAME_FRONT_X, L.FRAME_FRONT_X + L.FRAME_SIZE[0]
    half, inner = L.FRAME_SIZE[1] / 2, L.FRAME_SIZE[1] / 2 - L.FRAME_WALL
    return [("top", (x0, -half, L.RAIL_Z), (x1, half, L.FRAME_SIZE[2])),
            ("side_left", (x0, inner, 0.), (x1, half, L.RAIL_Z)),
            ("side_right", (x0, -half, 0.), (x1, -inner, L.RAIL_Z)),
            ("back", (x1 - L.FRAME_BACK, -inner, 0.), (x1, inner, L.RAIL_Z)),
            ("base", (x0, -inner, 0.), (x1, inner, .010))]


def drawer_boxes():
    """(name, low, high) of the drawer's boxes in the drawer's frame (origin: outer face of its front, on the
    table plane)."""
    inner_x0, inner_x1 = L.FRONT_T, L.FRONT_T + L.DRAWER_INNER[0]
    inner, outer = L.DRAWER_INNER[1] / 2, L.DRAWER_INNER[1] / 2 + L.DRAWER_WALL
    floor0 = L.FLOOR_Z - L.DRAWER_FLOOR_T
    px, py, pz = L.PULL_SIZE
    return [("front", (0., -L.FRONT_WIDTH / 2, L.FRONT_Z[0]), (L.FRONT_T, L.FRONT_WIDTH / 2, L.FRONT_Z[1])),
            ("pull", (-px, -py / 2, L.PULL_Z - pz / 2), (0., py / 2, L.PULL_Z + pz / 2)),
            ("floor", (inner_x0, -outer, floor0), (inner_x1 + L.DRAWER_BACK, outer, L.FLOOR_Z)),
            ("wall_left", (inner_x0, inner, L.FLOOR_Z), (inner_x1 + L.DRAWER_BACK, outer, L.RIM_Z)),
            ("wall_right", (inner_x0, -outer, L.FLOOR_Z), (inner_x1 + L.DRAWER_BACK, -inner, L.RIM_Z)),
            ("wall_back", (inner_x1, -inner, L.FLOOR_Z), (inner_x1 + L.DRAWER_BACK, inner, L.RIM_Z))]


def usd():
    top = scene.TOP
    start = scene.WORLD_START
    pads = [L.pad_name(row, col) for row, col in L.PADS]
    bodies = body("drawer", L.DRAWER_MASS, start["drawer"],
                  "".join(box(name, "drawer", low, high) for name, low, high in drawer_boxes()))
    bx, by, bz = (v / 2 for v in L.BASE_SIZE)
    bodies += body("pillow_base", L.BASE_MASS, start["pillow_base"],
                   box("slab", "fabric_a", (-bx, -by, 0.), (bx, by, 2 * bz)), no_contact_with=pads)
    px, py, pt = L.PAD_SIZE[0] / 2, L.PAD_SIZE[1] / 2, L.PAD_SIZE[2]
    for i, (row, col) in enumerate(L.PADS):
        shade = "fabric_a" if (row + col) % 2 else "fabric_b"
        bodies += body(pads[i], L.PAD_MASS, start[pads[i]], box("pad", shade, (-px, -py, -pt), (px, py, 0.)),
                       no_contact_with=pads[i + 1:])
    fixed = "".join(box(name, "frame", (low[0], low[1], low[2] + top), (high[0], high[1], high[2] + top), indent=8)
                    for name, low, high in frame_boxes())
    joints = slide("drawer_slide", None, "drawer", (L.FRAME_FRONT_X, 0., top), SLIDE_TO_MINUS_X, 0., L.OPEN_LIMIT,
                   stiffness=0., damping=DRAWER_DAMPING, max_force=L.RUNNER_FRICTION_N)
    base_centre = L.FRONT_T + L.DRAWER_INNER[0] / 2
    for i, (row, col) in enumerate(L.PADS):
        anchor = (L.row_centre(row) - base_centre, L.PAD_Y[col], L.REST_TOP[row])
        joints += slide(f"spring_{row}_{'lr'[col]}", "pillow_base", pads[i], anchor, SLIDE_TO_Z, -L.PAD_TRAVEL, 0.,
                        stiffness=L.PAD_STIFFNESS, damping=L.PAD_DAMPING, target=L.PAD_PRELOAD_M)
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Scope "Static"\n    {\n' + fixed + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print(f"wrote {scene.USD.name}: {len(L.ACTORS)} bodies, {len(L.PADS) + 1} slides, {len(frame_boxes())} fixed boxes; "
          f"rows standing above the opening: {L.PROUD_ROWS}, by "
          f"{[round(1000 * L.ROW_PROUD[r]) for r in L.PROUD_ROWS]} mm; they catch at openings of "
          f"{[round(1000 * L.blocking_open(r), 1) for r in L.PROUD_ROWS if not L.under_frame_at_start(r)]} mm")
