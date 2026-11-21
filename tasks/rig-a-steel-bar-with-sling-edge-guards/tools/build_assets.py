"""Write ``rdb_sling_guards/rigging.usda`` from ``layout.py`` and ``fixed_scene.py`` (text USD; the banknote and spark-plug
packages' way: box and capsule colliders only, a floating articulation for the flexible part, a fixed-base articulation
for the mechanism).

  /Rigging/Static/*     the two 2x4 blocks and the two trays (static colliders)
  /Rigging/Bar          the bar: a kinematic body (it does not move)
  /Rigging/Guards/*     four printed magnetic edge guards (free bodies; the pull is applied every substep at run time)
  /Rigging/Sling/*      fitting A, 33 web links, fitting B: one floating articulation, hinges across the web's width
  /Rigging/SlingJoints  its hinges
  /Rigging/HookRig/*    beam (welded) -> hoist (z) -> gimbal (x) -> chain and double hook (y) -> two latches (x)

Run from the task folder: ``python tools/build_assets.py`` (NumPy only). World coordinates: the station's (tabletop
x, y; z + the table top's height).
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_sling_guards import fixed_scene as F  # noqa: E402
from rdb_sling_guards import hook as HK  # noqa: E402
from rdb_sling_guards import layout as L  # noqa: E402

P = "/" + F.ROOT
TOP = F.TOP
CONTACT_OFFSET = .001
COLORS = {"steel": (.30, .31, .33), "web": (.86, .52, .06), "guard": (.08, .30, .75), "wood": (.60, .45, .27),
          "table": (.6, .6, .6), "hook": (.78, .62, .05), "tray": (.18, .19, .21), "chain": (.25, .25, .26),
          "fitting": (.55, .56, .58)}
# The fittings take the higher of the two friction coefficients in a contact: the gripper's rubber pads (yam_arena's
# GRIPPY_FRICTION, 1.2 static, 1.0 dynamic) grip steel at their own friction; against the hook, the web, the table and
# the guards this is within 0.08 of the average.
COMBINE = {"fitting": "max"}
MATERIAL_FRICTION = {"steel": L.FRICTION["steel"], "web": L.FRICTION["web"], "guard": L.FRICTION["guard"],
                     "wood": L.FRICTION["wood"], "hook": L.FRICTION["hook"], "tray": .50, "chain": .30,
                     "fitting": L.FRICTION["steel"]}


def W(p):
    """Tabletop point to the world."""
    return np.array((p[0], p[1], p[2] + TOP))


def quat(R):
    R = np.asarray(R, dtype=float)
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-3:
        q = np.array((w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)))
    else:                                    # half turns
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1e-12, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = .25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    return q / np.linalg.norm(q)


def aim_z(a, b):
    d = (np.asarray(b, float) - np.asarray(a, float))
    d /= np.linalg.norm(d)
    v = np.cross((0., 0., 1.), d)
    s, c = np.linalg.norm(v), d[2]
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag((1., -1., -1.))
    k = v / s
    K = np.array(((0., -k[2], k[1]), (k[2], 0., -k[0]), (-k[1], k[0], 0.)))
    return np.eye(3) + s * K + (1 - c) * K @ K


def fmt(v):
    return "(" + ", ".join(f"{float(x):.7g}" for x in v) + ")"


def collide_lines(material, pad):
    return (f"{pad}    rel material:binding = <{P}/Materials/{material}>\n"
            f"{pad}    rel material:binding:physics = <{P}/Materials/{material}>\n"
            f"{pad}    float physxCollision:contactOffset = {CONTACT_OFFSET}\n{pad}    float physxCollision:restOffset = 0\n")


def cube(name, material, centre, half, R=None, pad="            ", collide=True):
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    return (f'{pad}def Cube "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            + (collide_lines(material, pad) if collide else f"{pad}    rel material:binding = <{P}/Materials/{material}>\n")
            + f"{pad}    double size = 2\n{pad}    float3 xformOp:scale = {fmt(half)}\n"
            f"{pad}    double3 xformOp:translate = {fmt(centre)}\n{pad}    quatf xformOp:orient = {fmt(quat(R if R is not None else np.eye(3)))}\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n{pad}}}\n\n')


def capsule(name, material, a, b, r, pad="            ", collide=True):
    a, b = np.asarray(a, float), np.asarray(b, float)
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    return (f'{pad}def Capsule "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            + (collide_lines(material, pad) if collide else f"{pad}    rel material:binding = <{P}/Materials/{material}>\n")
            + f'{pad}    uniform token axis = "Z"\n{pad}    double height = {np.linalg.norm(b - a):.7g}\n{pad}    double radius = {r:.7g}\n'
            f"{pad}    double3 xformOp:translate = {fmt((a + b) / 2)}\n{pad}    quatf xformOp:orient = {fmt(quat(aim_z(a, b)))}\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n{pad}}}\n\n')


def body(name, mass, position, R, children, *, root=False, kinematic=False, self_collide=False, iters=(64, 4),
         inertia=None, pad="        ", filtered=(), damping=(.1, .5)):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    if filtered:
        apis.append('"PhysicsFilteredPairsAPI"')
    extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        extra = (f"{pad}    bool physxArticulation:enabledSelfCollisions = {int(self_collide)}\n"
                 f"{pad}    int physxArticulation:solverPositionIterationCount = {iters[0]}\n"
                 f"{pad}    int physxArticulation:solverVelocityIterationCount = {iters[1]}\n"
                 f"{pad}    float physxArticulation:sleepThreshold = 0\n"
                 f"{pad}    float physxArticulation:stabilizationThreshold = 0\n")
    if inertia is not None:
        extra += f"{pad}    float3 physics:diagonalInertia = {fmt(inertia)}\n"
    if filtered:
        extra += f"{pad}    rel physics:filteredPairs = [{', '.join(f'<{f}>' for f in filtered)}]\n"
    return (f'{pad}def Xform "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            f"{pad}    float physics:mass = {mass:.7g}\n{pad}    bool physics:rigidBodyEnabled = 1\n"
            f"{pad}    bool physics:kinematicEnabled = {int(kinematic)}\n" + extra +
            f"{pad}    float physxContactReport:threshold = 0\n"
            f"{pad}    float physxRigidBody:maxDepenetrationVelocity = 1\n"
            f"{pad}    float physxRigidBody:sleepThreshold = 0\n"
            f"{pad}    float physxRigidBody:angularDamping = {damping[1]}\n"
            f"{pad}    float physxRigidBody:linearDamping = {damping[0]}\n"
            f"{pad}    int physxRigidBody:solverPositionIterationCount = {iters[0]}\n"
            f"{pad}    int physxRigidBody:solverVelocityIterationCount = {iters[1]}\n"
            f"{pad}    double3 xformOp:translate = {fmt(position)}\n{pad}    quatf xformOp:orient = {fmt(quat(R))}\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}{pad}}}\n\n")


def joint(kind, name, b0, b1, p0, p1, axis, limits=None, *, stiffness=0., damping=0., target=0., max_force=None,
          armature=0., pad="        ", drive=True, r0=None, r1=None):
    """``kind`` "Revolute" (degrees) or "Prismatic" (metres)."""
    d = "angular" if kind == "Revolute" else "linear"
    apis = ([f'"PhysicsDriveAPI:{d}"'] if drive else []) + ['"PhysxJointAPI"']
    t = f'{pad}def Physics{kind}Joint "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
    if drive:
        t += (f"{pad}    float drive:{d}:physics:damping = {damping:.7g}\n{pad}    float drive:{d}:physics:stiffness = {stiffness:.7g}\n"
              f"{pad}    float drive:{d}:physics:targetPosition = {target:.7g}\n{pad}    float drive:{d}:physics:targetVelocity = 0\n"
              f'{pad}    uniform token drive:{d}:physics:type = "force"\n')
        if max_force is not None:
            t += f"{pad}    float drive:{d}:physics:maxForce = {max_force:.7g}\n"
    q0 = quat(r0) if r0 is not None else np.array((1., 0., 0., 0.))
    q1 = quat(r1) if r1 is not None else np.array((1., 0., 0., 0.))
    t += (f'{pad}    uniform token physics:axis = "{axis}"\n' + (f"{pad}    rel physics:body0 = <{b0}>\n" if b0 else "")
          + f"{pad}    rel physics:body1 = <{b1}>\n"
          f"{pad}    point3f physics:localPos0 = {fmt(p0)}\n{pad}    point3f physics:localPos1 = {fmt(p1)}\n"
          f"{pad}    quatf physics:localRot0 = {fmt(q0)}\n{pad}    quatf physics:localRot1 = {fmt(q1)}\n")
    if limits is not None:
        t += f"{pad}    float physics:lowerLimit = {limits[0]:.7g}\n{pad}    float physics:upperLimit = {limits[1]:.7g}\n"
    t += (f"{pad}    float physxJoint:armature = {armature:.7g}\n"
          f"{pad}    float physxJoint:maxJointVelocity = {720 if kind == 'Revolute' else 2}\n{pad}}}\n\n")
    return t


def material(name):
    mu = MATERIAL_FRICTION[name]
    c = COLORS[name]
    rough, metal = (.35, .9) if name in ("steel", "hook", "chain", "fitting") else (.6, 0.)
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n        )\n        {{\n'
            f"            token outputs:surface.connect = <{P}/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {mu}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {mu}\n"
            f'            uniform token physxMaterial:frictionCombineMode = "{COMBINE.get(name, "average")}"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = {fmt(c)}\n                float inputs:metallic = {metal}\n"
            f"                float inputs:roughness = {rough}\n                token outputs:surface\n            }}\n        }}\n\n")


# -- parts ------------------------------------------------------------------------------------------------------
def static_part():
    t = '    def Xform "Static"\n    {\n'
    for k, (x0, x1) in enumerate(L.BLOCK_X):
        t += cube(f"block_{k}", "wood", W(((x0 + x1) / 2, 0., L.BLOCK_H / 2)), ((x1 - x0) / 2, L.BLOCK_L / 2, L.BLOCK_H / 2),
                  pad="        ")
    sx, sy, floor = L.TRAY_SIZE
    wall, rim = L.TRAY_WALL
    for side, (x, y) in L.TRAY_XY.items():
        t += cube(f"tray_{side}_floor", "tray", W((x, y, floor / 2)), (sx / 2, sy / 2, floor / 2), pad="        ")
        c, h = F.tray_block(side)
        t += cube(f"tray_{side}_block", "tray", W(c), h, pad="        ")
        for k, (cx, cy, hx, hy) in enumerate(((x, y - sy / 2 + wall / 2, sx / 2, wall / 2), (x, y + sy / 2 - wall / 2, sx / 2, wall / 2),
                                              (x - sx / 2 + wall / 2, y, wall / 2, sy / 2 - wall), (x + sx / 2 - wall / 2, y, wall / 2, sy / 2 - wall))):
            t += cube(f"tray_{side}_wall_{k}", "tray", W((cx, cy, floor + rim / 2)), (hx, hy, rim / 2), pad="        ")
    return t + "    }\n\n"


def bar_part():
    c = W((L.XS, 0., (L.Z_BOT + L.Z_TOP) / 2))
    kids = cube("collider", "steel", (0., 0., 0.), (L.BAR_L / 2, L.HALF, L.HALF))
    return body(F.BAR, L.BAR_MASS, c, np.eye(3), kids, kinematic=True, pad="    ")


def guard_rotation(edge):
    """The guard's frame (inside corner on its x axis; face flange along -y on z = 0, side flange along -z on y = 0)
    turned onto ``edge``: local y to the side face's outward normal, local z to the horizontal face's."""
    _, n1z, n2y = L.EDGES[edge]
    n1, n2 = np.array((0., 0., n1z)), np.array((0., n2y, 0.))
    return np.column_stack((np.cross(n2, n1), n2, n1))


def guard_colliders():
    T, F_, H, R = L.GUARD_T, L.GUARD_FL, L.GUARD_LEN / 2, L.GUARD_NOSE_R
    kids = cube("face_flange", "guard", (0., (T - F_) / 2, T / 2), (H, (F_ + T) / 2, T / 2))
    kids += cube("side_flange", "guard", (0., T / 2, -F_ / 2), (H, T / 2, F_ / 2))
    kids += capsule("nose", "guard", (-H + R, R, R), (H - R, R, R), R)
    # a grip tab across each end, out along the outside bisector (outside the web's width)
    mid = (L.TAB_FROM + L.TAB_TO) / 2 / math.sqrt(2)
    turn = np.array(((1., 0., 0.), (0., math.cos(math.pi / 4), -math.sin(math.pi / 4)), (0., math.sin(math.pi / 4), math.cos(math.pi / 4))))
    for k, sx in enumerate((-1., 1.)):
        kids += cube(f"tab_{k}", "guard", (sx * (H - L.TAB_T / 2), mid, mid), (L.TAB_T / 2, (L.TAB_TO - L.TAB_FROM) / 2, L.TAB_W / 2), R=turn)
        kids += cube(f"lobe_{k}", "guard", (sx * (H - L.TAB_T / 2), L.LOBE_OUT / 2, -L.LOBE_DOWN / 2),
                     (L.TAB_T / 2, L.LOBE_OUT / 2, L.LOBE_DOWN / 2))
    return kids


def tray_pose(name):
    """World position and rotation of a guard in its tray: a top-edge guard as a tent (corner up), a bottom-edge guard
    as it will sit on the bottom edge of its side."""
    side, edge, corner = F.tray_slots()[name]
    if edge in ("BL", "BR"):
        R = guard_rotation(edge)
    else:                       # the top-edge orientation turned so the corner points straight up
        s = 1. if edge == "TL" else -1.
        ang = s * math.pi / 4
        Rx = np.array(((1., 0., 0.), (0., math.cos(ang), -math.sin(ang)), (0., math.sin(ang), math.cos(ang))))
        R = Rx @ guard_rotation(edge)
    return W(corner), R


def guard_part():
    t = '    def Xform "Guards"\n    {\n'
    for name in F.GUARD_NAMES:
        p, R = tray_pose(name)
        t += body(name, L.GUARD_MASS, p, R, guard_colliders(), iters=(64, 4))
    return t + "    }\n\n"


def side_bar(name, a, b):
    """A fitting's side bar as a box of square section (FIT_SIDE: a round bar in the drawing): between rigid flat
    pads a rigid round bar touches at single points and the fitting swings about them, where the gripper's rubber pads
    would wrap the bar; the flat faces give the pads the contact area they would have."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    along = (b - a) / np.linalg.norm(b - a)
    normal = np.array((0., 0., 1.))
    R = np.column_stack((np.cross(along, normal), along, normal))
    half = (L.FIT_SIDE_R, np.linalg.norm(b - a) / 2, L.FIT_SIDE_R)
    return cube(name, "fitting", (a + b) / 2, half, R=R)


def sling_part():
    bodies = F.sling_start()
    here = f"{P}/{F.SLING}/"
    root = L.N_LINKS // 2
    h = L.FIT_H
    t = f'    def Xform "{F.SLING}"\n    {{\n'
    pose = {}
    for name, p, cols in bodies:
        R = np.column_stack(cols)
        pose[name] = (W(p), R)
    for nm, sgn in (("fitA", -1.), ("fitB", 1.)):
        p, R = pose[nm]
        kids = (capsule("base_bar", "fitting", (-L.FIT_BASE_HALF, 0., 0.), (L.FIT_BASE_HALF, 0., 0.), L.FIT_BAR_R)
                + side_bar("side_p", (L.FIT_BASE_HALF, 0., 0.), (L.FIT_APEX_HALF, sgn * h, 0.))
                + side_bar("side_n", (-L.FIT_BASE_HALF, 0., 0.), (-L.FIT_APEX_HALF, sgn * h, 0.))
                + capsule("apex_bar", "fitting", (-L.FIT_APEX_HALF, sgn * h, 0.), (L.FIT_APEX_HALF, sgn * h, 0.), L.FIT_BAR_R))
        nb = [here + F.LINKS[1], here + F.LINKS[2]] if nm == "fitA" else [here + F.LINKS[-2], here + F.LINKS[-3]]
        t += body(nm, L.FIT_MASS, p, R, kids, filtered=nb)
    for i, name in enumerate(F.LINKS):
        p, R = pose[name]
        kids = cube("web", "web", (0., 0., 0.), (L.SLING_W / 2, L.LINK_L / 2, L.WEB_T / 2))
        filt = [here + F.LINKS[i + 2]] if i + 2 < L.N_LINKS else []
        t += body(name, L.LINK_MASS, p, R, kids, root=(i == root), self_collide=True, filtered=filt)
    t += "    }\n\n"
    t += f'    def Scope "SlingJoints"\n    {{\n'
    names = ["fitA", *F.LINKS, "fitB"]
    per_deg = math.pi / 180
    for i in range(len(names) - 1):
        a, b = names[i], names[i + 1]
        pa, Ra = pose[a]
        pb, Rb = pose[b]
        if a.startswith("link"):
            hinge = pa + Ra @ np.array((0., L.LINK_L / 2, 0.))      # the far end of link a
        else:
            hinge = pa                                               # fitting A's base bar
        loop = a == "fitA" or b == "fitB"        # the web's sewn loop turns freely round the fitting's base bar
        t += joint("Revolute", f"hinge_{i:02d}", here + a, here + b, Ra.T @ (hinge - pa), Rb.T @ (hinge - pb), "X",
                   (-179., 179.) if loop else (-L.HINGE_LIMIT_DEG, L.HINGE_LIMIT_DEG),
                   stiffness=0. if loop else L.HINGE_K * per_deg, damping=L.HINGE_D * per_deg,
                   armature=L.HINGE_ARMATURE)
    return t + "    }\n\n"


def hook_part():
    """beam (welded) -> hoist (z: the operator's lift) -> gimbal (x) -> hook (y; the load chain, the hoist's hook and the
    double hook as one pendulum body) -> latchL, latchR (each about x; 0 = closed on its tip, opening only into its
    bowl, spring-preloaded closed)."""
    here = f"{P}/{F.RIG}/"
    O = W(F.HOOK_O)
    pivot = W(F.PIVOT)
    top = pivot + np.array((0., 0., .06))
    t = (f'    def Xform "{F.RIG}" (\n        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n    )\n    {{\n'
         "        bool physxArticulation:enabledSelfCollisions = 0\n        int physxArticulation:solverPositionIterationCount = 64\n"
         "        int physxArticulation:solverVelocityIterationCount = 4\n        float physxArticulation:sleepThreshold = 0\n"
         "        float physxArticulation:stabilizationThreshold = 0\n\n")
    t += body("beam", 5., top, np.eye(3), cube("beam", "steel", (0., 0., .02), (.03, .14, .02), collide=False), inertia=(.05, .05, .05))
    t += body("hoist", L.HOIST_MASS, pivot, np.eye(3), cube("hoist_body", "hook", (0., 0., .03), (.035, .045, .03), collide=False),
              inertia=(1e-3, 1e-3, 1e-3))
    t += body("gimbal", L.GIMBAL_MASS, pivot, np.eye(3), "", inertia=(1e-4, 1e-4, 1e-4))
    eye_top = O + np.array((0., 0., L.EYE_TOP))

    def yz(p):
        return O + np.array((0., p[0], p[1]))

    kids = capsule("chain", "chain", (0., 0., -.005), eye_top + np.array((0., 0., .012)) - pivot, .005)
    kids += "".join(capsule(nm, "hook", yz(a) - pivot, yz(b) - pivot, r) for nm, a, b, r in HK.hook_capsules())
    t += body("hook", L.HOOK_MASS + L.CHAIN_MASS, pivot, np.eye(3), kids)
    for side in ("L", "R"):
        lp, le = HK.latch(side)
        t += body(f"latch{side}", L.LATCH_MASS, yz(lp), np.eye(3), capsule("plate", "hook", (0., 0., 0.), yz(le) - yz(lp), .003),
                  inertia=(2e-6, 2e-6, 2e-6))
    t += ('        def PhysicsFixedJoint "root_joint"\n        {\n'
          f"            rel physics:body1 = <{here}beam>\n"
          f"            point3f physics:localPos0 = {fmt(top)}\n            point3f physics:localPos1 = (0, 0, 0)\n"
          "            quatf physics:localRot0 = (1, 0, 0, 0)\n            quatf physics:localRot1 = (1, 0, 0, 0)\n        }\n\n")
    per_deg = math.pi / 180
    t += joint("Prismatic", "lift", here + "beam", here + "hoist", pivot - top, (0., 0., 0.), "Z", (-.10, .40),
               stiffness=L.LIFT_K, damping=400., max_force=250., armature=.05, target=L.HOIST_PRELOAD)
    t += joint("Revolute", "swing_x", here + "hoist", here + "gimbal", (0., 0., 0.), (0., 0., 0.), "X", (-60, 60),
               damping=L.SWING_DAMPING * per_deg, armature=1e-4)
    t += joint("Revolute", "swing_y", here + "gimbal", here + "hook", (0., 0., 0.), (0., 0., 0.), "Y", (-60, 60),
               damping=L.SWING_DAMPING * per_deg, armature=1e-4)
    pre = math.degrees(L.LATCH_PRELOAD / L.LATCH_K)
    # the left latch points +y from its pivot: a negative turn about +x lowers its free end into the bowl; the right
    # latch points -y: a positive turn lowers it. Each is preloaded against its closed stop at 0.
    for side, lim, target in (("L", (-80., 0.), pre), ("R", (0., 80.), -pre)):
        lp, _ = HK.latch(side)
        t += joint("Revolute", f"latch{side}_hinge", here + "hook", here + f"latch{side}", yz(lp) - pivot, (0., 0., 0.), "X", lim,
                   stiffness=L.LATCH_K * per_deg, damping=2e-4 * per_deg, target=target, armature=1e-5)
    return t + "    }\n\n"


def usd():
    return ('#usda 1.0\n(\n    defaultPrim = "' + F.ROOT + '"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            f'def Xform "{F.ROOT}"\n{{\n' + static_part() + bar_part() + guard_part() + sling_part() + hook_part()
            + '    def Scope "Materials"\n    {\n' + "".join(material(k) for k in MATERIAL_FRICTION) + "    }\n}\n")


if __name__ == "__main__":
    text = usd()
    F.USD.write_text(text)
    print(f"wrote {F.USD} ({len(text.splitlines())} lines)")
