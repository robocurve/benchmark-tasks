"""Regenerate ``rdb_desk_converter/desk_converter.usda`` from ``layout.py``. Run from the task folder:
``python tools/build_assets.py``. Needs only NumPy.

The converter is ONE fixed-base articulation (a tree, no loop joints):

    base --lift (revolute, the counterbalance spring)--> lift_arm --level (revolute, mimic of lift)--> top
    base --rear (revolute, mimic of lift; display only)--> rear_arm
    top --latch_left_slide / latch_right_slide (prismatic, sprung down)--> latch_left / latch_right

The parallelogram's second arm would close a loop, so the top's level is kept by the mimic coupling on ``level``
instead (the top turns back by exactly the arm's angle, which is what the second arm does). Each latch carries a
paddle (under its handle, squeezed by the fingers) and a pin that drops into one of 11 radial slots of a toothed
quadrant on its side of the lift arm. Monitor and keyboard are free rigid bodies resting on the top.

Colliders are plain boxes, invisible; what the cameras see are display meshes of the real unit drawn over them
(rounded black work surface and tray, handle housings, steel frame and arms, monitor, keyboard with its keys).
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_desk_converter import fixed_scene as scene  # noqa: E402
from rdb_desk_converter import layout as L  # noqa: E402

TOP_Z = scene.TOP
T4 = scene.START_THETA
# colour, roughness, metallic, friction, restitution
MATERIALS = {
    "laminate": ((.035, .035, .04), .45, 0., .5, 0.), "frame": ((.05, .05, .055), .35, .6, .4, 0.),
    "plastic": ((.03, .03, .03), .6, 0., .8, 0.), "steel": ((.35, .35, .37), .3, 1., .2, 0.),
    "rubber": ((.02, .02, .02), .8, 0., .8, 0.), "screen": ((.01, .012, .018), .08, 0., .4, 0.),
    "keycap": ((.17, .17, .18), .6, 0., .6, 0.), "silver": ((.62, .63, .65), .3, .8, .5, 0.),
    "green": ((.12, .62, .22), .6, 0., .5, 0.), "foam": ((.06, .06, .065), .9, 0., .6, 0.),
}
_M = "</Fixture/Materials/"
NAMES = set()


def w(p):
    """Station point to stage (world) coordinates."""
    return np.asarray(p, dtype=float) + (0., 0., TOP_Z)


def q_xz(a):
    """(w, x, y, z) of a rotation by ``a`` in the x-z plane from +x toward +z (about -y)."""
    return (math.cos(a / 2), 0., -math.sin(a / 2), 0.)


def _unique(name, scope):
    key = (scope, name)
    if key in NAMES:
        raise ValueError(f"duplicate prim name {name} in {scope}")
    NAMES.add(key)


def box(scope, name, material, centre, half, *, collide=True, quat=(1., 0., 0., 0.), visible=None, ind="                "):
    _unique(name, scope)
    visible = (not collide) if visible is None else visible
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    text = (f'{ind[:-4]}def Cube "{name}" (\n{ind}prepend apiSchemas = [{", ".join(apis)}]\n{ind[:-4]})\n{ind[:-4]}{{\n'
            f"{ind}rel material:binding = {_M}{material}>\n")
    if collide:
        text += (f"{ind}rel material:binding:physics = {_M}{material}>\n"
                 f"{ind}float physxCollision:contactOffset = 0.001\n{ind}float physxCollision:restOffset = 0\n")
    if not visible:
        text += f'{ind}token visibility = "invisible"\n'
    text += (f"{ind}double size = 2\n"
             f"{ind}quatf xformOp:orient = ({quat[0]:.7g}, {quat[1]:.7g}, {quat[2]:.7g}, {quat[3]:.7g})\n"
             f"{ind}float3 xformOp:scale = ({half[0]:.6g}, {half[1]:.6g}, {half[2]:.6g})\n"
             f"{ind}double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
             f'{ind}uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n'
             f"{ind[:-4]}}}\n\n")
    return text


def cylinder(scope, name, material, centre, radius, height, axis, ind="                "):
    _unique(name, scope)
    return (f'{ind[:-4]}def Cylinder "{name}" (\n{ind}prepend apiSchemas = ["MaterialBindingAPI"]\n{ind[:-4]})\n'
            f"{ind[:-4]}{{\n{ind}rel material:binding = {_M}{material}>\n{ind}uniform token axis = \"{axis}\"\n"
            f"{ind}double height = {height:.6g}\n{ind}double radius = {radius:.6g}\n"
            f"{ind}double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
            f'{ind}uniform token[] xformOpOrder = ["xformOp:translate"]\n{ind[:-4]}}}\n\n')


def rounded(x0, x1, y0, y1, r, n=6, cut=None):
    """Counter-clockwise outline of a rectangle with rounded corners; ``cut`` = (depth, half width) makes a shallow
    curved notch in the -x edge (the tray's front)."""
    pts = []
    for cx, cy, a0 in ((x1 - r, y1 - r, 0.), (x0 + r, y1 - r, 90.), (x0 + r, y0 + r, 180.), (x1 - r, y0 + r, 270.)):
        for k in range(n + 1):
            a = math.radians(a0 + 90. * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        if cut is not None and a0 == 180.:
            pass
    if cut is not None:
        depth, half = cut
        out = []
        for i, p in enumerate(pts):
            out.append(p)
            if i == 2 * (n + 1) - 1:                      # after the front-left corner: the curved front edge
                for k in range(1, 12):
                    y = half - 2 * half * k / 12
                    out.append((x0 + depth * math.cos(math.pi / 2 * y / half), y))
        pts = out
    return pts


def slab_mesh(scope, name, material, outline, z0, z1, ind="                "):
    """Display-only extrusion of a polygon (may be non-convex: drawn as a fan of quads plus caps)."""
    _unique(name, scope)
    n = len(outline)
    points = [(x, y, z0) for x, y in outline] + [(x, y, z1) for x, y in outline]
    cx, cy = np.mean(outline, axis=0)
    points += [(cx, cy, z0), (cx, cy, z1)]
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
        faces.append([2 * n, j, i])
        faces.append([2 * n + 1, n + i, n + j])
    return (f'{ind[:-4]}def Mesh "{name}" (\n{ind}prepend apiSchemas = ["MaterialBindingAPI"]\n{ind[:-4]})\n{ind[:-4]}{{\n'
            f"{ind}rel material:binding = {_M}{material}>\n{ind}uniform bool doubleSided = 1\n"
            f"{ind}int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"{ind}int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"{ind}point3f[] points = [" + ", ".join(f"({x:.6g}, {y:.6g}, {z:.6g})" for x, y, z in points) + "]\n"
            f'{ind}uniform token subdivisionScheme = "none"\n{ind[:-4]}}}\n\n')


def body(name, position, mass, com, inertia, children, *, ind="            "):
    _unique(name, "bodies")
    p = w(position)
    return (f'{ind[:-4]}def Xform "{name}" (\n{ind}prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", '
            f'"PhysxContactReportAPI", "PhysicsMassAPI"]\n{ind[:-4]})\n{ind[:-4]}{{\n'
            f"{ind}float physics:mass = {mass:.6g}\n"
            f"{ind}point3f physics:centerOfMass = ({com[0]:.6g}, {com[1]:.6g}, {com[2]:.6g})\n"
            f"{ind}float3 physics:diagonalInertia = ({inertia[0]:.6g}, {inertia[1]:.6g}, {inertia[2]:.6g})\n"
            f"{ind}quatf physics:principalAxes = (1, 0, 0, 0)\n"
            f"{ind}bool physics:rigidBodyEnabled = 1\n{ind}float physxContactReport:threshold = 0\n"
            f"{ind}float physxRigidBody:maxDepenetrationVelocity = 0.5\n{ind}float physxRigidBody:sleepThreshold = 0\n"
            f"{ind}int physxRigidBody:solverPositionIterationCount = 32\n"
            f"{ind}int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"{ind}quatf xformOp:orient = (1, 0, 0, 0)\n"
            f"{ind}double3 xformOp:translate = ({p[0]:.10g}, {p[1]:.10g}, {p[2]:.10g})\n"
            f'{ind}uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}{ind[:-4]}}}\n\n")


def box_inertia(mass, size):
    a, b, c = size
    return (mass * (b * b + c * c) / 12, mass * (a * a + c * c) / 12, mass * (a * a + b * b) / 12)


AXIS_MINUS_Y = (math.sqrt(.5), 0., 0., -math.sqrt(.5))      # joint frame whose X axis is the station's -y


def joint(kind, name, body0, body1, pos0, pos1, *, axis="X", rot=(1., 0., 0., 0.), limits=None, drive=None,
          mimic=None, ind="            "):
    """``drive`` = (kind 'angular'/'linear', stiffness, damping, target, max force) in USD units (deg for angles).
    ``mimic`` = (reference joint, gearing): this + gearing * reference = 0."""
    _unique(name, "joints")
    apis = []
    if drive:
        apis.append(f'"PhysicsDriveAPI:{drive[0]}"')
    apis.append('"PhysxJointAPI"')
    if mimic:
        apis.append('"PhysxMimicJointAPI:rotX"')
    text = f'{ind[:-4]}def {kind} "{name}" (\n{ind}prepend apiSchemas = [{", ".join(apis)}]\n{ind[:-4]})\n{ind[:-4]}{{\n'
    if drive:
        d, k, c, target, fmax = drive
        text += (f"{ind}float drive:{d}:physics:damping = {c:.6g}\n{ind}float drive:{d}:physics:stiffness = {k:.6g}\n"
                 f"{ind}float drive:{d}:physics:targetPosition = {target:.6g}\n"
                 f"{ind}float drive:{d}:physics:targetVelocity = 0\n"
                 f'{ind}uniform token drive:{d}:physics:type = "force"\n{ind}float drive:{d}:physics:maxForce = {fmax:.6g}\n')
    if kind != "PhysicsFixedJoint":
        text += f'{ind}uniform token physics:axis = "{axis}"\n'
    if body0:
        text += f"{ind}rel physics:body0 = </Fixture/{scene.ROOT_NAME}/Bodies/{body0}>\n"
    text += (f"{ind}rel physics:body1 = </Fixture/{scene.ROOT_NAME}/Bodies/{body1}>\n"
             f"{ind}point3f physics:localPos0 = ({pos0[0]:.10g}, {pos0[1]:.10g}, {pos0[2]:.10g})\n"
             f"{ind}point3f physics:localPos1 = ({pos1[0]:.10g}, {pos1[1]:.10g}, {pos1[2]:.10g})\n"
             f"{ind}quatf physics:localRot0 = ({rot[0]:.7g}, {rot[1]:.7g}, {rot[2]:.7g}, {rot[3]:.7g})\n"
             f"{ind}quatf physics:localRot1 = ({rot[0]:.7g}, {rot[1]:.7g}, {rot[2]:.7g}, {rot[3]:.7g})\n")
    if limits:
        text += f"{ind}float physics:lowerLimit = {limits[0]:.6g}\n{ind}float physics:upperLimit = {limits[1]:.6g}\n"
    if kind != "PhysicsFixedJoint":
        text += f"{ind}float physxJoint:armature = 0.001\n{ind}float physxJoint:maxJointVelocity = 720\n"
    if mimic:
        ref, gearing = mimic
        text += (f"{ind}float physxMimicJoint:rotX:gearing = {gearing:.9g}\n{ind}float physxMimicJoint:rotX:offset = 0\n"
                 f"{ind}rel physxMimicJoint:rotX:referenceJoint = </Fixture/{scene.ROOT_NAME}/Joints/{ref}>\n"
                 f'{ind}uniform token physxMimicJoint:rotX:referenceJointAxis = "rotX"\n')
    return text + f"{ind[:-4]}}}\n\n"


def material(name):
    color, roughness, metallic, friction, restitution = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = {restitution}\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


# ------------------------------------------------------------------------------------------------------------------
# Geometry at the start pose (setting 4), station frame.

def start_frames():
    top = L.top_origin(T4)
    pivot = L.PIVOT.copy()
    upper = L.upper_pivot(T4)
    base = np.array(((L.BASE_X[0] + L.BASE_X[1]) / 2, 0., 0.))
    rear = pivot + L.REAR_OFFSET
    return {"top": top, "lift_arm": pivot, "upper": upper, "base": base, "rear_arm": rear}


def latch_parts(sign):
    """(paddle centre, pin centre) of one latch in the TOP frame, released."""
    paddle = np.array((L.HANDLE_X, sign * L.HANDLE_Y, L.PADDLE_TOP_REST - L.PADDLE_HALF[2]))
    pin = L.UPPER_PIVOT + (0., 0., -(L.PIN_R[0] + L.PIN_R[1]) / 2)
    pin[1] = sign * (L.LOCK_Y[0] + L.LOCK_Y[1]) / 2
    return paddle, pin


def teeth(sign):
    """(name, centre in the ARM frame relative to the upper pivot, half sizes, quaternion) of one quadrant's teeth."""
    out = []
    r_mid = (L.NOTCH_R[0] + L.NOTCH_R[1]) / 2
    for j, a in enumerate(L.tooth_angles()):
        c = np.array((r_mid * math.cos(a), sign * (L.LOCK_Y[0] + L.LOCK_Y[1]) / 2, r_mid * math.sin(a)))
        half = ((L.NOTCH_R[1] - L.NOTCH_R[0]) / 2, (L.LOCK_Y[1] - L.LOCK_Y[0]) / 2, L.TOOTH_TANGENT / 2)
        out.append((f"tooth_{'l' if sign > 0 else 'r'}{j:02d}", c, half, q_xz(a)))
    return out


def usd():
    f = start_frames()
    top, pivot, upper, base, rear = f["top"], f["lift_arm"], f["upper"], f["base"], f["rear_arm"]

    # -- base: two flat steel side rails on rubber feet, cross bars, the arms' lower brackets -------------------------
    parts = ""
    bx0, bx1 = L.BASE_X[0] - base[0], L.BASE_X[1] - base[0]
    for sign, tag in ((1, "l"), (-1, "r")):
        yc = sign * sum(L.BASE_RAIL_Y) / 2
        half = ((bx1 - bx0) / 2, (L.BASE_RAIL_Y[1] - L.BASE_RAIL_Y[0]) / 2, L.BASE_RAIL_H / 2)
        parts += box("base", f"rail_{tag}", "frame", ((bx0 + bx1) / 2, yc, L.BASE_RAIL_H / 2), half, collide=True)
        parts += box("base", f"rail_{tag}_skin", "frame", ((bx0 + bx1) / 2, yc, L.BASE_RAIL_H / 2), half, collide=False)
        for k, xf in enumerate((bx0 + .03, bx1 - .03)):
            parts += box("base", f"foot_{tag}{k}", "rubber", (xf, yc, .002), (.025, .022, .002), collide=False)
        bracket = pivot - base + (0., sign * L.ARM_DISPLAY_Y, 0.)
        parts += box("base", f"bracket_{tag}", "frame", (bracket[0], bracket[1] + sign * .012, bracket[2] / 2 + .01),
                     (.03, .004, bracket[2] / 2 + .01), collide=False)
    for k, xc in enumerate((bx0 + .02, bx1 - .02)):
        parts += box("base", f"cross{k}", "frame", (xc, 0., .009), (.02, L.BASE_RAIL_Y[0], .009), collide=False)
    bodies = body("base", base, L.BASE_MASS, (0., 0., .01), box_inertia(L.BASE_MASS, (L.BASE_D, L.BASE_W, .02)), parts)

    # -- lift arm: the two primary arms (display), cross tube, and the two toothed lock quadrants (colliders) ---------
    parts = ""
    arm_vec = upper - pivot
    length = float(np.linalg.norm(arm_vec))
    for sign, tag in ((1, "l"), (-1, "r")):
        mid = arm_vec / 2 + (0., sign * L.ARM_DISPLAY_Y, 0.)
        parts += box("lift_arm", f"arm_{tag}", "frame", mid, (length / 2 + .015, .006, .016), collide=False, quat=q_xz(T4))
        parts += box("lift_arm", f"foam_{tag}", "foam", mid, (length / 2 - .03, .0075, .0185), collide=False, quat=q_xz(T4))
        for name, centre, half, quat in teeth(sign):
            parts += box("lift_arm", name, "steel", arm_vec + centre, half, collide=True, visible=True, quat=quat)
        # the quadrant plate the teeth stand on (display), and a strut from the arm to it
        parts += box("lift_arm", f"quadrant_{tag}", "steel", arm_vec + (0., sign * (L.LOCK_Y[1] + .004), -.07),
                     (.075, .004, .055), collide=False)
    parts += cylinder("lift_arm", "cross_tube", "frame", (0., 0., 0.), .014, 2 * L.ARM_DISPLAY_Y + .02, "Y")
    com = arm_vec / 2
    i_arm = (L.ARM_MASS * (.6 ** 2) / 12, L.ARM_MASS * length ** 2 / 12, L.ARM_MASS * (.6 ** 2 + length ** 2) / 12)
    bodies += body("lift_arm", pivot, L.ARM_MASS, com, i_arm, parts)

    # -- rear (secondary) arms: display only, kept parallel by a mimic coupling --------------------------------------
    parts = ""
    for sign, tag in ((1, "l"), (-1, "r")):
        mid = arm_vec / 2 + (0., sign * (L.ARM_DISPLAY_Y - .025), 0.)
        parts += box("rear_arm", f"rear_{tag}", "frame", mid, (length / 2 + .012, .005, .012), collide=False, quat=q_xz(T4))
    bodies += body("rear_arm", rear, L.REAR_ARM_MASS, com, (.01, .01, .01), parts)

    # -- top: work surface, tray, hangers (colliders); display surface, tray, handle housings, under-frame ------------
    parts = ""
    (x0, x1), (y0, y1), (z0, z1) = L.SLAB
    parts += box("top", "slab", "laminate", ((x0 + x1) / 2, 0., (z0 + z1) / 2), ((x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2))
    parts += slab_mesh("top", "surface_skin", "laminate", rounded(x0, x1, y0, y1, .02), z0, z1 + .0004)
    (tx0, tx1), _, (tz0, tz1) = L.TRAY
    parts += box("top", "tray", "laminate", ((tx0 + tx1) / 2, 0., (tz0 + tz1) / 2), ((tx1 - tx0) / 2, (y1 - y0) / 2, (tz1 - tz0) / 2))
    parts += slab_mesh("top", "tray_skin", "laminate", rounded(tx0, tx1, y0, y1, .02, cut=(.02, .2)), tz0, tz1 + .0004)
    for sign, tag in ((1, "l"), (-1, "r")):
        hy = sign * L.HANGER_Y
        parts += box("top", f"hanger_{tag}", "frame", (x0 - .006, hy, (tz1 + z0) / 2), (.006, .02, (z0 - tz1) / 2))
        parts += box("top", f"hanger_{tag}_skin", "frame", (x0 - .006, hy, (tz1 + z0) / 2), (.006, .02, (z0 - tz1) / 2),
                     collide=False)
        # handle housing: a black moulding under the edge, open at the bottom where the paddle sits
        hx, hyy = L.HANDLE_X, sign * L.HANDLE_Y
        parts += box("top", f"housing_{tag}_out", "plastic", (hx, sign * (L.WIDTH / 2 - .004), z0 - .016), (.045, .004, .016),
                     collide=False)
        parts += box("top", f"housing_{tag}_front", "plastic", (hx - .043, hyy, z0 - .012), (.003, .025, .012), collide=False)
        parts += box("top", f"housing_{tag}_back", "plastic", (hx + .043, hyy, z0 - .012), (.003, .025, .012), collide=False)
        # under-frame rails along each side (display)
        parts += box("top", f"subframe_{tag}", "frame", (.0, sign * .29, z0 - .012), (.15, .012, .012), collide=False)
    parts += box("top", "subframe_back", "frame", (.17, 0., z0 - .012), (.012, .29, .012), collide=False)
    i_top = box_inertia(L.TOP_MASS, (L.DEPTH, L.WIDTH, .1))
    bodies += body("top", top, L.TOP_MASS, L.TOP_COM, i_top, parts)

    # -- latches: paddle (collider, squeezed by the fingers) + pin (collider, into the teeth) + a link (display) ------
    for sign, side in ((1, "left"), (-1, "right")):
        paddle, pin = latch_parts(sign)
        origin = pin                                   # latch frame origin at the pin's centre (top frame)
        parts = ""
        parts += box(f"latch_{side}", "paddle", "plastic", paddle - origin, L.PADDLE_HALF)
        parts += box(f"latch_{side}", "paddle_skin", "plastic", paddle - origin, L.PADDLE_HALF, collide=False)
        pin_half = ((L.PIN_R[1] - L.PIN_R[0]) / 2 + 0., (L.LOCK_Y[1] - L.LOCK_Y[0]) / 2, L.PIN_TANGENT / 2)
        pin_half = (L.PIN_TANGENT / 2, (L.LOCK_Y[1] - L.LOCK_Y[0]) / 2, (L.PIN_R[1] - L.PIN_R[0]) / 2)
        parts += box(f"latch_{side}", "pin", "steel", (0., 0., 0.), pin_half, visible=True)
        # display link: from the paddle's inner end, along under the board, down to the pin
        y_link = sign * (L.LOCK_Y[1] + .012)
        z_link = L.PADDLE_TOP_REST - .006 - origin[2]
        x_a, x_b = paddle[0] - origin[0], 0.
        parts += box(f"latch_{side}", "link_bar", "steel", ((x_a + x_b) / 2, y_link - origin[1], z_link),
                     (abs(x_b - x_a) / 2 + .004, .003, .004), collide=False)
        parts += box(f"latch_{side}", "link_cross", "steel", (x_a, (paddle[1] + y_link) / 2 - origin[1], z_link),
                     (.004, abs(paddle[1] - y_link) / 2, .004), collide=False)
        parts += box(f"latch_{side}", "link_stem", "steel", (0., y_link - origin[1], z_link / 2),
                     (.004, .003, abs(z_link) / 2), collide=False)
        parts += box(f"latch_{side}", "link_foot", "steel", (0., (y_link - origin[1]) / 2, 0.),
                     (.004, abs(y_link - origin[1]) / 2, .003), collide=False)
        com = (paddle - origin) / 2
        bodies += body(f"latch_{side}", top + origin, L.LATCH_MASS, com, (2e-4, 2e-4, 2e-4), parts)

    # -- joints ----------------------------------------------------------------------------------------------------
    deg = math.degrees
    lo, hi = (deg(v - T4) for v in L.LIFT_LIMITS)
    joints = joint("PhysicsFixedJoint", "fix_base", None, "base", w(base), (0., 0., 0.))
    joints += joint("PhysicsRevoluteJoint", scene.LIFT, "base", "lift_arm", pivot - base, (0., 0., 0.), rot=AXIS_MINUS_Y,
                    limits=(lo, hi),
                    drive=("angular", L.SPRING_K * math.pi / 180, L.LIFT_DAMPING * math.pi / 180,
                           deg(scene.LIFT_TARGET), 5000.))
    joints += joint("PhysicsRevoluteJoint", scene.LEVEL, "lift_arm", "top", upper - pivot, upper - top, rot=AXIS_MINUS_Y,
                    limits=(-hi - 1., -lo + 1.), mimic=(scene.LIFT, 1.))
    joints += joint("PhysicsRevoluteJoint", scene.REAR, "base", "rear_arm", rear - base, (0., 0., 0.), rot=AXIS_MINUS_Y,
                    limits=(lo - 1., hi + 1.), mimic=(scene.LIFT, -1.))
    k_latch, preload = scene.LATCH_K, scene.LATCH_PRELOAD
    for sign, side in ((1, "left"), (-1, "right")):
        _, pin = latch_parts(sign)
        joints += joint("PhysicsPrismaticJoint", scene.LATCH[side], "top", f"latch_{side}", pin, (0., 0., 0.), axis="Z",
                        limits=(0., L.PADDLE_TRAVEL), drive=("linear", k_latch, 2.0, -preload, 200.))

    # -- the load: monitor on its stand, keyboard on the tray (free bodies) --------------------------------------------
    loads = ""
    m = scene.MONITOR_START
    fx, fy, fz = L.MONITOR_BASE
    pd, pw, ph = L.MONITOR_PANEL
    neck_x = .055
    parts = ""
    parts += box("monitor", "foot", "rubber", (0., 0., fz / 2), (fx / 2, fy / 2, fz / 2))
    parts += box("monitor", "neck", "plastic", (neck_x, 0., (fz + L.MONITOR_PANEL_BOTTOM + .16) / 2),
                 (.015, .035, (L.MONITOR_PANEL_BOTTOM + .16 - fz) / 2))
    parts += box("monitor", "panel", "plastic", (0., 0., L.MONITOR_PANEL_BOTTOM + ph / 2), (pd / 2, pw / 2, ph / 2))
    parts += slab_mesh("monitor", "foot_skin", "plastic", rounded(-fx / 2, fx / 2, -fy / 2, fy / 2, .03), 0., fz)
    parts += box("monitor", "neck_skin", "plastic", (neck_x, 0., (fz + L.MONITOR_PANEL_BOTTOM + .16) / 2),
                 (.015, .035, (L.MONITOR_PANEL_BOTTOM + .16 - fz) / 2), collide=False)
    parts += box("monitor", "panel_skin", "plastic", (0., 0., L.MONITOR_PANEL_BOTTOM + ph / 2), (pd / 2, pw / 2, ph / 2),
                 collide=False)
    parts += box("monitor", "screen", "screen", (-pd / 2 - .0006, 0., L.MONITOR_PANEL_BOTTOM + ph / 2 + .008),
                 (.0006, pw / 2 - .008, ph / 2 - .018), collide=False)
    m_com = (.01, 0., .2)
    loads += body("monitor", m, L.MONITOR_MASS, m_com, (.06, .03, .05), parts, ind="            ")
    k = scene.KEYBOARD_START
    kd, kw, kh = L.KEYBOARD_SIZE
    parts = box("keyboard", "case", "rubber", (0., 0., .006), (kd / 2, kw / 2, .006))
    parts += slab_mesh("keyboard", "case_skin", "plastic", rounded(-kd / 2, kd / 2, -kw / 2, kw / 2, .008), 0., .016)
    rows, cols = 6, 18
    for r in range(rows):
        for c in range(cols):
            if r == 0 and c in (1, 5, 10, 14):
                continue
            xk = -kd / 2 + .012 + (r + .5) * (kd - .024) / rows
            yk = -kw / 2 + .012 + (c + .5) * (kw - .024) / cols
            long = r == rows - 1 and 6 <= c <= 11            # space bar
            if r == rows - 1 and 7 <= c <= 11:
                continue
            half_y = (.0095 if not long else .0095 + 5 * (kw - .024) / cols / 2)
            yk = yk + (5 * (kw - .024) / cols / 2 if long else 0.)
            parts += box("keyboard", f"key{r}{c:02d}", "keycap", (xk, yk, .019), (.0075, half_y, .004), collide=False)
    parts += box("keyboard", "keys_collider", "keycap", (0., 0., .018), (kd / 2 - .012, kw / 2 - .012, .006))
    loads += body("keyboard", k, L.KEYBOARD_MASS, (0., 0., .01), box_inertia(L.KEYBOARD_MASS, L.KEYBOARD_SIZE), parts,
                  ind="            ")

    card = L.CARD_AT
    card_text = box("card", "card", "green", w((card[0], card[1], L.CARD_SIZE[2] / 2)), np.array(L.CARD_SIZE) / 2,
                    collide=True, visible=True, ind="        ")
    card_text += box("card", "card_foot", "green", w((card[0] + .02, card[1], .0015)), (.02, .03, .0015), collide=True,
                     visible=True, ind="        ")

    root = scene.ROOT_NAME
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            f'    def Xform "{root}" (\n        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n    )\n    {{\n'
            "        bool physxArticulation:enabledSelfCollisions = 1\n"
            "        int physxArticulation:solverPositionIterationCount = 32\n"
            "        int physxArticulation:solverVelocityIterationCount = 8\n"
            "        float physxArticulation:sleepThreshold = 0\n"
            "        float physxArticulation:stabilizationThreshold = 0\n\n"
            '        def Scope "Bodies"\n        {\n' + bodies + "        }\n\n"
            '        def Scope "Joints"\n        {\n' + joints + "        }\n    }\n\n"
            '    def Scope "Loads"\n    {\n' + loads + "    }\n\n"
            '    def Scope "Marks"\n    {\n' + card_text + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    text = usd()
    (ROOT / "rdb_desk_converter/desk_converter.usda").write_text(text)
    print("wrote", len(text), "bytes;", sum(1 for line in text.splitlines() if "def Cube" in line), "boxes")
