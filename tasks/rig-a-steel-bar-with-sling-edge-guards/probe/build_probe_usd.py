"""Write sling_probe.usda from sling_model.py. Pattern: the banknote task's build_scene.py (text USD, box and
capsule colliders, a floating articulation for the flexible part, a fixed-base articulation for the mechanism).

  /Probe/Static/*      table and the two 2x4 blocks (static colliders)
  /Probe/Bar           the bar, a kinematic rigid body (fixed in place)
  /Probe/Guards/*      four magnetic L-shaped edge guards (free rigid bodies; the pull is applied each substep)
  /Probe/Sling/*       fitting A, 22 web links, fitting B: one floating articulation, hinges across the web's width
  /Probe/HookRig/*     frame (welded) -> lift (prismatic z) -> gimbal (x) -> hook (y) -> latch (x, sprung)

Run:  python build_probe_usd.py   (numpy only)
"""

import math
from pathlib import Path

import numpy as np

import sling_model as m

OUT = Path(__file__).with_name("sling_probe.usda")
HINGE_D, VEL_ITERS, BEAD, GUARD_MASS, GUARD_POS_ITERS = m.HINGE_D, 8, 0., m.GUARD_MASS, 64
GUARDS_KINEMATIC, CONTACT_OFFSET = False, .001
GUARD_INERTIA_SCALE, GUARD_DAMPING, GUARD_COMPLIANT = 1., (.1, .5), (0., 0.)
GUARD_FRICTION = None
P = "/Probe"
COLORS = {"steel": (.25, .26, .28), "web": (.85, .55, .05), "guard": (.05, .25, .75), "wood": (.55, .42, .25),
          "table": (.6, .6, .6), "hook": (.75, .1, .1)}


def quat_from_matrix(R):
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    q = np.array((w, x, y, z))
    return q / np.linalg.norm(q)


def aim_z(a, b):
    """Rotation taking +z to the direction a->b."""
    d = (np.asarray(b) - np.asarray(a)) / np.linalg.norm(np.asarray(b) - np.asarray(a))
    v = np.cross((0., 0., 1.), d)
    s, c = np.linalg.norm(v), d[2]
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag((1., -1., -1.))
    k = v / s
    K = np.array(((0., -k[2], k[1]), (k[2], 0., -k[0]), (-k[1], k[0], 0.)))
    return np.eye(3) + s * K + (1 - c) * K @ K


def fmt(v):
    return "(" + ", ".join(f"{float(x):.7g}" for x in v) + ")"


def collider_common(material, pad, offset=None):
    offset = CONTACT_OFFSET if offset is None else offset
    return (f"{pad}    rel material:binding = <{P}/Materials/{material}>\n"
            f"{pad}    rel material:binding:physics = <{P}/Materials/{material}>\n"
            f"{pad}    float physxCollision:contactOffset = {offset}\n{pad}    float physxCollision:restOffset = 0\n")


def cube(name, material, centre, half, R=None, pad="            ", collide=True):
    q = quat_from_matrix(R if R is not None else np.eye(3))
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    return (f'{pad}def Cube "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            + (collider_common(material, pad) if collide else f"{pad}    rel material:binding = <{P}/Materials/{material}>\n")
            + f"{pad}    double size = 2\n"
            f"{pad}    float3 xformOp:scale = {fmt(half)}\n{pad}    double3 xformOp:translate = {fmt(centre)}\n"
            f"{pad}    quatf xformOp:orient = {fmt(q)}\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n{pad}}}\n\n')


def capsule(name, material, a, b, r, pad="            "):
    a, b = np.asarray(a, float), np.asarray(b, float)
    q = quat_from_matrix(aim_z(a, b))
    return (f'{pad}def Capsule "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]\n{pad})\n{pad}{{\n'
            + collider_common(material, pad)
            + f'{pad}    uniform token axis = "Z"\n{pad}    double height = {np.linalg.norm(b - a):.7g}\n{pad}    double radius = {r:.7g}\n'
            f"{pad}    double3 xformOp:translate = {fmt((a + b) / 2)}\n{pad}    quatf xformOp:orient = {fmt(q)}\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n{pad}}}\n\n')


def body(name, mass, position, R, children, *, root=False, kinematic=False, self_collide=False, iters=(64, 8),
         inertia=None, extra_api="", pad="        ", filtered=(), damping=(.1, .5)):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    if filtered:
        apis.append('"PhysicsFilteredPairsAPI"')
    text_extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        text_extra = (f"{pad}    bool physxArticulation:enabledSelfCollisions = {int(self_collide)}\n"
                      f"{pad}    int physxArticulation:solverPositionIterationCount = {iters[0]}\n"
                      f"{pad}    int physxArticulation:solverVelocityIterationCount = {iters[1]}\n"
                      f"{pad}    float physxArticulation:sleepThreshold = 0\n"
                      f"{pad}    float physxArticulation:stabilizationThreshold = 0\n")
    if inertia is not None:
        text_extra += f"{pad}    float3 physics:diagonalInertia = {fmt(inertia)}\n"
    if filtered:
        text_extra += f"{pad}    rel physics:filteredPairs = [{', '.join(f'<{f}>' for f in filtered)}]\n"
    q = quat_from_matrix(R)
    return (f'{pad}def Xform "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            f"{pad}    float physics:mass = {mass:.7g}\n{pad}    bool physics:rigidBodyEnabled = 1\n"
            f"{pad}    bool physics:kinematicEnabled = {int(kinematic)}\n" + text_extra +
            f"{pad}    float physxContactReport:threshold = 0\n"
            f"{pad}    float physxRigidBody:maxDepenetrationVelocity = 1\n"
            f"{pad}    float physxRigidBody:sleepThreshold = 0\n"
            f"{pad}    float physxRigidBody:angularDamping = {damping[1]}\n"
            f"{pad}    float physxRigidBody:linearDamping = {damping[0]}\n"
            f"{pad}    int physxRigidBody:solverPositionIterationCount = {iters[0]}\n"
            f"{pad}    int physxRigidBody:solverVelocityIterationCount = {iters[1]}\n"
            f"{pad}    double3 xformOp:translate = {fmt(position)}\n{pad}    quatf xformOp:orient = {fmt(q)}\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}{pad}}}\n\n")


def joint(kind, name, b0, b1, p0, p1, axis, limits=None, *, r0=None, r1=None, stiffness=0., damping=0., target=0.,
          max_force=None, armature=0., pad="        ", drive=True):
    d = "angular" if kind == "Revolute" else "linear"
    apis = ([f'"PhysicsDriveAPI:{d}"'] if drive else []) + ['"PhysxJointAPI"']
    t = f'{pad}def Physics{kind}Joint "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
    if drive:
        t += (f"{pad}    float drive:{d}:physics:damping = {damping:.7g}\n"
              f"{pad}    float drive:{d}:physics:stiffness = {stiffness:.7g}\n"
              f"{pad}    float drive:{d}:physics:targetPosition = {target:.7g}\n"
              f"{pad}    float drive:{d}:physics:targetVelocity = 0\n"
              f'{pad}    uniform token drive:{d}:physics:type = "force"\n')
        if max_force is not None:
            t += f"{pad}    float drive:{d}:physics:maxForce = {max_force:.7g}\n"
    q0 = quat_from_matrix(r0) if r0 is not None else np.array((1., 0., 0., 0.))
    q1 = quat_from_matrix(r1) if r1 is not None else np.array((1., 0., 0., 0.))
    t += (f'{pad}    uniform token physics:axis = "{axis}"\n'
          + (f"{pad}    rel physics:body0 = <{b0}>\n" if b0 else "")
          + f"{pad}    rel physics:body1 = <{b1}>\n"
          f"{pad}    point3f physics:localPos0 = {fmt(p0)}\n{pad}    point3f physics:localPos1 = {fmt(p1)}\n"
          f"{pad}    quatf physics:localRot0 = {fmt(q0)}\n{pad}    quatf physics:localRot1 = {fmt(q1)}\n")
    if limits is not None:
        t += f"{pad}    float physics:lowerLimit = {limits[0]:.7g}\n{pad}    float physics:upperLimit = {limits[1]:.7g}\n"
    t += (f"{pad}    float physxJoint:armature = {armature:.7g}\n"
          f"{pad}    float physxJoint:maxJointVelocity = {720 if kind == 'Revolute' else 2}\n{pad}}}\n\n")
    return t


def material(name):
    mu = GUARD_FRICTION if (name == "guard" and GUARD_FRICTION is not None) else m.FRICTION[name]
    c = COLORS[name]
    comp = ""
    if name == "guard" and GUARD_COMPLIANT[0] > 0:
        comp = (f"            float physxMaterial:compliantContactStiffness = {GUARD_COMPLIANT[0]}\n"
                f"            float physxMaterial:compliantContactDamping = {GUARD_COMPLIANT[1]}\n")
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n        )\n        {{\n'
            f"            token outputs:surface.connect = <{P}/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {mu}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {mu}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n' + comp + '\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = {fmt(c)}\n"
            "                float inputs:roughness = 0.5\n                token outputs:surface\n            }\n        }\n\n")


# ------------------------------------------------------------------------------------------------------------
def to_local(R, p, world):
    return R.T @ (np.asarray(world) - p)


def static_part():
    t = '    def Xform "Static"\n    {\n'
    t += cube("table", "table", (.3, 0., -.02), (1.2, .8, .02), pad="        ")
    for k, (lo, hi) in enumerate(m.BLOCKS):
        lo, hi = np.array(lo), np.array(hi)
        t += cube(f"block_{k}", "wood", (lo + hi) / 2, (hi - lo) / 2, pad="        ")
    return t + "    }\n\n"


def bar_part():
    c = (m.BAR_LO + m.BAR_HI) / 2
    kids = cube("collider", "steel", (0., 0., 0.), (m.BAR_HI - m.BAR_LO) / 2)
    return body("Bar", 24.7, c, np.eye(3), kids, kinematic=True, pad="    ")


def guard_part():
    t = '    def Xform "Guards"\n    {\n'
    for name in m.EDGES:
        e, n1, n2, (f1c, f1h), (f2c, f2h) = m.guard_boxes(name)
        kids = cube("flange_face", "guard", f1c - e, f1h) + cube("flange_side", "guard", f2c - e, f2h)
        if BEAD > 0:      # a rounded nose along the guard's corner (radius BEAD), clear of the bar's faces
            c = BEAD * (n1 + n2)
            half = m.GUARD_LEN / 2 - BEAD
            kids += capsule("bead", "guard", c + np.array((-half, 0., 0.)), c + np.array((half, 0., 0.)), BEAD)
        inertia = None
        if GUARD_INERTIA_SCALE != 1.:   # flat-plate estimate of the L's inertia about its centre, scaled
            base = GUARD_MASS * np.array((.05 ** 2 + .05 ** 2, .1 ** 2 + .05 ** 2, .1 ** 2 + .05 ** 2)) / 12
            inertia = base * GUARD_INERTIA_SCALE
        t += body(f"guard_{name}", GUARD_MASS, e + 1e-4 * (n1 + n2), np.eye(3), kids, iters=(GUARD_POS_ITERS, 4),
                  kinematic=GUARDS_KINEMATIC, inertia=inertia, damping=GUARD_DAMPING)
    return t + "    }\n\n"


def link_frame(a, b):
    """Rotation of a web link from hinge a to hinge b ((y, z) points): local x = world x (across the web),
    local y along the web, local z the web's normal."""
    d = np.array((0., b[0] - a[0], b[1] - a[1]))
    d /= np.linalg.norm(d)
    x = np.array((1., 0., 0.))
    return np.column_stack((x, d, np.cross(x, d)))


def fitting_frame(base, apex):
    d = np.array((0., apex[0] - base[0], apex[1] - base[1]))
    d /= np.linalg.norm(d)
    x = np.array((1., 0., 0.))
    return np.column_stack((x, d, np.cross(x, d)))


def sling_part():
    bodies, H = m.start_bodies()
    _, bulge, gap = m.start_shape()
    names = [b[0] for b in bodies]
    pose = {b[0]: (b[1], b[2]) for b in bodies}
    root = m.N_LINKS // 2
    here = f"{P}/Sling/"
    t = '    def Xform "Sling"\n    {\n'
    h = m.FIT_H
    for nm, sgn in (("fitA", -1.), ("fitB", 1.)):
        p, R = pose[nm]
        kids = (capsule("base_bar", "steel", (-m.FIT_BASE_HALF, 0., 0.), (m.FIT_BASE_HALF, 0., 0.), m.FIT_BAR_R)
                + capsule("side_p", "steel", (m.FIT_BASE_HALF, 0., 0.), (m.FIT_APEX_HALF, sgn * h, 0.), m.FIT_SIDE_R)
                + capsule("side_n", "steel", (-m.FIT_BASE_HALF, 0., 0.), (-m.FIT_APEX_HALF, sgn * h, 0.), m.FIT_SIDE_R)
                + capsule("apex_bar", "steel", (-m.FIT_APEX_HALF, sgn * h, 0.), (m.FIT_APEX_HALF, sgn * h, 0.), m.FIT_BAR_R))
        nb = ([here + "link_01", here + "link_02"] if nm == "fitA"
              else [here + f"link_{m.N_LINKS - 2:02d}", here + f"link_{m.N_LINKS - 3:02d}"])
        t += body(nm, m.FIT_MASS, p, R, kids, filtered=nb)
    for i in range(m.N_LINKS):
        p, R = pose[f"link_{i:02d}"]
        kids = cube("web", "web", (0., 0., 0.), (m.SLING_W / 2, np.linalg.norm(H[i + 1] - H[i]) / 2, m.WEB_T / 2))
        filt = [here + f"link_{i + 2:02d}"] if i + 2 < m.N_LINKS else []
        t += body(f"link_{i:02d}", m.LINK_MASS, p, R, kids, root=(i == root), self_collide=True, filtered=filt,
                  iters=(64, VEL_ITERS))
    t += "    }\n\n"
    t += '    def Scope "SlingJoints"\n    {\n'
    per_deg = math.pi / 180
    for i in range(m.N_LINKS + 1):
        b0, b1 = names[i], names[i + 1]
        hinge = np.array((m.XS, H[i][0], H[i][1]))
        p0, R0 = pose[b0]
        p1, R1 = pose[b1]
        # joint frames on each body's own axes (x across the web), so the spring's rest shape is straight; the
        # start bend is written to the articulation at reset (sling_model.start_hinge_angles)
        t += joint("Revolute", f"hinge_{i:02d}", here + b0, here + b1, to_local(R0, p0, hinge), to_local(R1, p1, hinge), "X",
                   (-m.HINGE_LIMIT_DEG, m.HINGE_LIMIT_DEG),
                   stiffness=m.HINGE_K * per_deg, damping=HINGE_D * per_deg, armature=m.HINGE_ARMATURE)
    t += "    }\n\n"
    return t, bulge, gap


def hook_part():
    here = f"{P}/HookRig/"
    pivot = m.hp(0., m.PIVOT_V)
    t = '    def Xform "HookRig" (\n        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n    )\n    {\n'
    t += ("        bool physxArticulation:enabledSelfCollisions = 0\n"
          "        int physxArticulation:solverPositionIterationCount = 64\n"
          "        int physxArticulation:solverVelocityIterationCount = 8\n"
          "        float physxArticulation:sleepThreshold = 0\n        float physxArticulation:stabilizationThreshold = 0\n\n")
    top = pivot + np.array((0., 0., .25))
    t += body("frame", 5., top, np.eye(3), cube("beam", "steel", (0., 0., 0.), (.03, .2, .02), collide=False),
              inertia=(.05, .05, .05))
    t += body("carriage", m.CARRIAGE_MASS, pivot, np.eye(3),
              capsule("shackle", "hook", (0., 0., .012), (0., 0., .07), .012), inertia=(1e-3, 1e-3, 1e-3))
    t += body("gimbal", .05, pivot, np.eye(3), "", inertia=(1e-4, 1e-4, 1e-4))
    kids = "".join(capsule(nm, "hook", a - pivot, b - pivot, r) for nm, a, b, r in m.hook_capsules())
    t += body("hook", m.HOOK_MASS, pivot, np.eye(3), kids)
    lp, le = m.LATCH_PIVOT, m.LATCH_END
    t += body("latch", m.LATCH_MASS, lp, np.eye(3), capsule("plate", "hook", (0., 0., 0.), le - lp, .0025),
              inertia=(2e-6, 2e-6, 2e-6))
    t += ('        def PhysicsFixedJoint "root_joint"\n        {\n'
          f"            rel physics:body1 = <{here}frame>\n"
          f"            point3f physics:localPos0 = {fmt(top)}\n            point3f physics:localPos1 = (0, 0, 0)\n"
          "            quatf physics:localRot0 = (1, 0, 0, 0)\n            quatf physics:localRot1 = (1, 0, 0, 0)\n        }\n\n")
    t += joint("Prismatic", "lift", here + "frame", here + "carriage", pivot - top, (0., 0., 0.), "Z", (-.10, .20),
               stiffness=m.LIFT_K, damping=m.LIFT_D, max_force=m.LIFT_MAX_F, armature=.05)
    per_deg = math.pi / 180
    t += joint("Revolute", "swing_x", here + "carriage", here + "gimbal", (0., 0., 0.), (0., 0., 0.), "X", (-60, 60),
               damping=m.SWING_DAMPING * per_deg, armature=1e-4)
    t += joint("Revolute", "swing_y", here + "gimbal", here + "hook", (0., 0., 0.), (0., 0., 0.), "Y", (-60, 60),
               damping=m.SWING_DAMPING * per_deg, armature=1e-4)
    # latch: hinged about +x (the hook's plane is y-z); positive turns its free end down, into the bowl, so it can
    # only open inward; the spring is preloaded against the closed stop
    pre = m.LATCH_PRELOAD / m.LATCH_K
    t += joint("Revolute", "latch_hinge", here + "hook", here + "latch", lp - pivot, (0., 0., 0.), "X", (0., 80.),
               stiffness=m.LATCH_K * per_deg, damping=2e-4 * per_deg, target=-math.degrees(pre), armature=1e-5)
    return t + "    }\n\n"


def usd():
    sling, bulge, gap = sling_part()
    text = ('#usda 1.0\n(\n    defaultPrim = "Probe"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Probe"\n{\n' + static_part() + bar_part() + guard_part() + sling + hook_part()
            + '    def Scope "Materials"\n    {\n' + "".join(material(k) for k in m.FRICTION) + "    }\n}\n")
    return text, bulge, gap


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--hinge-d", type=float, default=m.HINGE_D)
    ap.add_argument("--vel-iters", type=int, default=8)
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--bead", type=float, default=0.)
    ap.add_argument("--guard-mass", type=float, default=m.GUARD_MASS)
    ap.add_argument("--guard-pos-iters", type=int, default=64)
    ap.add_argument("--guards-kinematic", action="store_true")
    ap.add_argument("--contact-offset", type=float, default=.001)
    ap.add_argument("--guard-inertia-scale", type=float, default=1.)
    ap.add_argument("--guard-damping", type=float, nargs=2, default=(.1, .5))
    ap.add_argument("--guard-compliant", type=float, nargs=2, default=(0., 0.), help="stiffness N/m, damping N s/m")
    ap.add_argument("--guard-friction", type=float, default=None)
    ap.add_argument("--guard-fl", type=float, default=m.GUARD_FL, help="flange width from the corner (m)")
    ap.add_argument("--latch-preload", type=float, default=m.LATCH_PRELOAD)
    a = ap.parse_args()
    m.GUARD_FL, m.LATCH_PRELOAD = a.guard_fl, a.latch_preload
    GUARD_FRICTION = a.guard_friction
    GUARD_INERTIA_SCALE, GUARD_DAMPING, GUARD_COMPLIANT = a.guard_inertia_scale, tuple(a.guard_damping), tuple(a.guard_compliant)
    GUARDS_KINEMATIC, CONTACT_OFFSET = a.guards_kinematic, a.contact_offset
    HINGE_D, VEL_ITERS, OUT, BEAD = a.hinge_d, a.vel_iters, Path(a.out), a.bead
    GUARD_MASS, GUARD_POS_ITERS = a.guard_mass, a.guard_pos_iters
    text, bulge, gap = usd()
    OUT.write_text(text)
    print(f"wrote {OUT} ({len(text.splitlines())} lines); start bulge {1000 * bulge:.1f} mm, end gap {1000 * gap:.2f} mm")
