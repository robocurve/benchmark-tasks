"""Writes the task's USD files into assets/. Run with any Python that has pxr (the Arena venv does):

    python make_assets.py

- head.usda: the care manikin's head, an articulation with a fixed base. Frame (all links): origin at the centre of
  the lips' front plane at the height of the upper incisor edge; +x into the head (away from the robot), +z up.
  ``mount`` is fixed to the world; ``skull`` sits on the sprung prismatic joint ``neck`` (along x); ``jaw`` (lower lip,
  lower teeth, chin) turns on the revolute joint ``jaw`` (hinge 9 cm behind the incisor edge, level with it; negative
  opens); ``tongue`` (a plate 4.5 cm long, 2.4 cm below the upper incisor edge) tips its back end down on the revolute
  joint ``swallow`` (hinge at its front edge; positive tips) and drops what lies on it into a cup inside the skull.
  Collision: boxes (each its own convex piece). Visual: an ellipsoid head with a mouth opening, the lower face on the
  jaw, nose, ears, eyes and hair.
- spoon.usda: a feeding spoon with a built-up handle (15 cm, 30 g). Origin: the centre of the bowl's floor (its
  underside), +x toward the tip. The bowl's front is a thin open lip, as on a spoon's tip.
- dish.usda: a rectangular scoop dish (melamine, 15 x 17 cm, flat floor, 3 cm straight walls), origin at the centre
  of its underside.
- peach.usda: a 1.5 cm dice of canned peach (3.5 g), origin at its centre.
"""

from __future__ import annotations

import math
from pathlib import Path

HERE = Path(__file__).resolve().parent

SKIN = (0.80, 0.62, 0.50)
LIP = (0.74, 0.42, 0.42)
TOOTH = (0.96, 0.95, 0.90)
TONGUE = (0.85, 0.45, 0.47)
CAVITY = (0.35, 0.10, 0.12)
HAIR = (0.42, 0.42, 0.44)
EYE = (0.12, 0.10, 0.10)
STEEL = (0.72, 0.73, 0.75)
GRIP = (0.20, 0.42, 0.75)
DISH = (0.96, 0.96, 0.94)
PEACH = (0.96, 0.62, 0.16)

# -- head geometry (m, head frame) ------------------------------------------------------------------------------------
MOUTH_W = 0.050                 # lips' width (the mouth opening is this wide)
JAW_HINGE = (0.09, 0.0)         # (x, z): level with the incisor edge, so the lower teeth drop almost straight down
JAW_OPEN = -0.375               # rad: 30 mm between the incisor edges
TONGUE_X = (0.018, 0.070)       # its front and back edges
TONGUE_TOP = -0.026
RIDGE = 0.002                   # the tongue's raised tip and sides (it cups a bite)
TONGUE_T = 0.006
SWALLOW_HINGE = (TONGUE_X[0], TONGUE_TOP - TONGUE_T / 2)
SWALLOW_TIP = 0.90              # rad the tongue tips to swallow
DEPTH_LIMIT = 0.050             # m behind the lips' plane
SKULL_MASS, JAW_MASS, TONGUE_MASS, MOUNT_MASS = 1.2, 0.15, 0.05, 1.0
ELLIPSOID_C = (0.085, 0.0, 0.03)
ELLIPSOID_R = (0.095, 0.075, 0.115)

# collision boxes: (name, lo, hi); lips are named "lip_*" (touching them is allowed), teeth "teeth_*"
W2 = MOUTH_W / 2
SKULL_BOXES = [
    ("lip_upper", (-0.006, -W2, 0.002), (0.008, W2, 0.018)),
    ("teeth_upper", (0.002, -0.020, 0.000), (0.009, 0.020, 0.010)),
    ("face_upper", (-0.008, -0.070, 0.018), (0.150, 0.070, 0.150)),
    ("palate", (0.012, -W2, 0.010), (0.150, W2, 0.018)),
    ("cheek_l_in", (0.000, W2, -0.075), (0.150, 0.035, 0.018)),
    ("cheek_r_in", (0.000, -0.035, -0.075), (0.150, -W2, 0.018)),
    ("cheek_l_out", (0.015, 0.035, -0.075), (0.150, 0.070, 0.018)),
    ("cheek_r_out", (0.015, -0.070, -0.075), (0.150, -0.035, 0.018)),
    ("throat_back", (0.092, -W2, -0.110), (0.150, W2, 0.010)),
    ("cup_floor", (0.040, -W2, -0.110), (0.092, W2, -0.100)),
    ("cup_front", (0.030, -0.070, -0.110), (0.040, 0.070, -0.065)),   # keeps a swallowed bite in the cup
    ("cheek_l_low", (0.030, W2, -0.110), (0.150, 0.070, -0.075)),
    ("cheek_r_low", (0.030, -0.070, -0.110), (0.150, -W2, -0.075)),
]
JAW_BOXES = [
    ("lip_lower", (-0.006, -W2, -0.020), (0.008, W2, -0.002)),
    ("teeth_lower", (0.004, -0.020, -0.010), (0.012, 0.020, 0.000)),
    ("chin_front", (-0.002, -0.050, -0.050), (0.012, 0.050, -0.020)),   # stops above the cup: an opening jaw
    ("chin_under", (0.012, -W2, -0.050), (0.030, W2, -0.034)),          # must not sweep a swallowed bite
]
TONGUE_BOXES = [("tongue_plate", (TONGUE_X[0], -0.020, TONGUE_TOP - TONGUE_T), (TONGUE_X[1], 0.020, TONGUE_TOP)),
                ("tongue_tip", (TONGUE_X[0], -0.020, TONGUE_TOP), (TONGUE_X[0] + 0.004, 0.020, TONGUE_TOP + RIDGE)),
                ("tongue_side_l", (TONGUE_X[0], 0.017, TONGUE_TOP), (TONGUE_X[1], 0.020, TONGUE_TOP + RIDGE)),
                ("tongue_side_r", (TONGUE_X[0], -0.020, TONGUE_TOP), (TONGUE_X[1], -0.017, TONGUE_TOP + RIDGE))]
NOSE = [(-0.030, 0.0, 0.030), (-0.008, 0.013, 0.030), (-0.008, -0.013, 0.030), (-0.006, 0.0, 0.075)]

# -- spoon --------------------------------------------------------------------------------------------------------------
SPOON_MASS = 0.030
BOWL_L, BOWL_W, FLOOR_T, RIM_H = 0.030, 0.022, 0.0018, 0.007
TIP_L = 0.005                                # the tip's bevel: 1.8 mm to a 0.2 mm edge over 5 mm (19 deg)
SIDE_H = 0.006                               # the bowl's sides
SPOON_FRICTION = (0.22, 0.20)
HANDLE = ((-0.135, -0.055), 0.016, 0.012)    # x range, width, height of the built-up handle
HANDLE_Z = 0.020                             # the handle's centre line above the bowl's floor
GRASP_X = -0.105                             # where the demo holds the handle
HANDLE_FRICTION = (1.5, 1.3)                 # the rubber grip

# -- dish -------------------------------------------------------------------------------------------------------------
DISH_L, DISH_W, DISH_RIM_H, DISH_T, DISH_WALL, DISH_MASS = 0.150, 0.170, 0.030, 0.004, 0.004, 0.45

# -- peach ------------------------------------------------------------------------------------------------------------
CUBE, CUBE_MASS = 0.015, 0.0035


def box_mesh(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1),
           (x0, y1, z1)]
    faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return pts, faces


def hull_mesh(pts):
    """A small convex solid from points: faces by brute force (fine for a handful of points)."""
    import itertools

    import numpy as np

    P = np.asarray(pts, float)
    faces = []
    for i, j, k in itertools.combinations(range(len(P)), 3):
        n = np.cross(P[j] - P[i], P[k] - P[i])
        if np.linalg.norm(n) < 1e-12:
            continue
        d = (P - P[i]) @ n
        if np.all(d <= 1e-12):
            faces.append([i, k, j])
        elif np.all(d >= -1e-12):
            faces.append([i, j, k])
    return [tuple(p) for p in P], faces


def ellipsoid(c, r, keep, n_u=96, n_v=64):
    """Triangles of an ellipsoid whose centroid passes ``keep(x, y, z)``."""
    pts, faces = [], []
    for i in range(n_v + 1):
        v = math.pi * i / n_v
        for j in range(n_u):
            u = 2 * math.pi * j / n_u
            pts.append((c[0] + r[0] * math.sin(v) * math.cos(u), c[1] + r[1] * math.sin(v) * math.sin(u),
                        c[2] + r[2] * math.cos(v)))
    for i in range(n_v):
        for j in range(n_u):
            a, b = i * n_u + j, i * n_u + (j + 1) % n_u
            cc, d = a + n_u, b + n_u
            for f in ([a, cc, d], [a, d, b]):
                cx = sum(pts[k][0] for k in f) / 3
                cy = sum(pts[k][1] for k in f) / 3
                cz = sum(pts[k][2] for k in f) / 3
                if keep(cx, cy, cz):
                    faces.append(f)
    return pts, faces


def cyl_mesh(r, z0, z1, n=32, c=(0.0, 0.0)):
    pts, faces = [], []
    for z in (z0, z1):
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append((c[0] + r * math.cos(a), c[1] + r * math.sin(a), z))
    for k in range(n):
        k2 = (k + 1) % n
        faces.append([k, k2, n + k2, n + k])
    faces.append(list(range(n))[::-1])
    faces.append(list(range(n, 2 * n)))
    return pts, faces


def mesh(stage, path, pts, faces, color, collide=True):
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def new_stage(name):
    from pxr import Usd, UsdGeom, UsdPhysics

    out = HERE / "assets" / f"{name}.usda"
    out.parent.mkdir(exist_ok=True)
    if out.exists():
        out.unlink()
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    return stage, out


def body(name, mass, parts, com=(0.0, 0.0, 0.0)):
    from pxr import Gf, UsdGeom, UsdPhysics

    stage, out = new_stage(name)
    root = UsdGeom.Xform.Define(stage, "/Body")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(root.GetPrim())
    mass_api.CreateMassAttr(mass)
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    for part, (pts, faces, color, collide) in parts.items():
        mesh(stage, f"/Body/{part}", pts, faces, color, collide)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


def _link(stage, path, mass, com=(0.05, 0.0, 0.0)):
    from pxr import Gf, UsdGeom, UsdPhysics

    x = UsdGeom.Xform.Define(stage, path)
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    m = UsdPhysics.MassAPI.Apply(x.GetPrim())
    m.CreateMassAttr(mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    return x


def _joint(stage, kind, name, b0, b1, pos, axis, lo=None, hi=None, drive=None):
    from pxr import Gf, Sdf, UsdPhysics

    J = UsdPhysics.PrismaticJoint if kind == "prismatic" else UsdPhysics.RevoluteJoint
    j = J.Define(stage, f"/Head/joints/{name}")
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))
    if lo is not None:
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear" if kind == "prismatic" else "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(0.0)
    return j


def head():
    from pxr import Sdf, UsdGeom, UsdPhysics

    stage, out = new_stage("head")
    root = UsdGeom.Xform.Define(stage, "/Head")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

    _link(stage, "/Head/mount", MOUNT_MASS, (0.09, 0.0, -0.15))
    mesh(stage, "/Head/mount/neck", *cyl_mesh(0.045, -0.20, -0.07, c=(0.095, 0.0)), SKIN, collide=False)

    _link(stage, "/Head/skull", SKULL_MASS, (0.08, 0.0, 0.04))
    for n, lo, hi in SKULL_BOXES:
        color = LIP if n.startswith("lip") else TOOTH if n.startswith("teeth") else CAVITY
        mesh(stage, f"/Head/skull/{n}", *box_mesh(lo, hi), color, collide=True)
        if not (n.startswith("lip") or n.startswith("teeth")):
            UsdGeom.Imageable(stage.GetPrimAtPath(f"/Head/skull/{n}")).MakeInvisible()

    def upper_keep(x, y, z):
        if x < 0.05 and abs(y) < 0.028 and -0.024 < z < 0.020:
            return False                          # the mouth opening
        return not (z < -0.020 and x < 0.075 and abs(y) < 0.062)   # the lower face is on the jaw

    mesh(stage, "/Head/skull/face", *ellipsoid(ELLIPSOID_C, ELLIPSOID_R, upper_keep), SKIN, collide=False)
    mesh(stage, "/Head/skull/nose", *hull_mesh(NOSE), SKIN, collide=False)
    mesh(stage, "/Head/skull/cavity", *box_mesh((0.06, -W2, -0.10), (0.064, W2, 0.010)), CAVITY, collide=False)
    # hides the swallow cup behind the chin when the jaw is open (visual only)
    mesh(stage, "/Head/skull/cup_screen", *box_mesh((0.030, -0.062, -0.115), (0.034, 0.062, -0.036)), SKIN, collide=False)
    hair_c = (ELLIPSOID_C[0] + 0.006, 0.0, ELLIPSOID_C[2] + 0.004)
    hair_r = (ELLIPSOID_R[0] + 0.004, ELLIPSOID_R[1] + 0.004, ELLIPSOID_R[2] + 0.004)
    mesh(stage, "/Head/skull/hair", *ellipsoid(hair_c, hair_r, lambda x, y, z: z > 0.075 or (x > 0.10 and z > -0.02)),
         HAIR, collide=False)
    for s in (1, -1):
        mesh(stage, f"/Head/skull/eye_{'l' if s > 0 else 'r'}",
             *ellipsoid((0.006, s * 0.030, 0.072), (0.006, 0.009, 0.005), lambda *a: True, 12, 8), EYE,
             collide=False)
        mesh(stage, f"/Head/skull/ear_{'l' if s > 0 else 'r'}",
             *ellipsoid((0.090, s * 0.076, 0.040), (0.016, 0.008, 0.028), lambda *a: True, 16, 10), SKIN,
             collide=False)

    _link(stage, "/Head/jaw", JAW_MASS, (0.03, 0.0, -0.04))
    for n, lo, hi in JAW_BOXES:
        color = LIP if n.startswith("lip") else TOOTH if n.startswith("teeth") else SKIN
        mesh(stage, f"/Head/jaw/{n}", *box_mesh(lo, hi), color, collide=True)
        if not (n.startswith("lip") or n.startswith("teeth")):
            UsdGeom.Imageable(stage.GetPrimAtPath(f"/Head/jaw/{n}")).MakeInvisible()

    def jaw_keep(x, y, z):
        return z < -0.020 and x < 0.075 and abs(y) < 0.062 and not (x < 0.05 and abs(y) < 0.028 and z > -0.024)

    mesh(stage, "/Head/jaw/chin", *ellipsoid(ELLIPSOID_C, ELLIPSOID_R, jaw_keep), SKIN, collide=False)
    mesh(stage, "/Head/jaw/floor", *box_mesh((0.006, -W2, -0.034), (0.030, W2, -0.032)), CAVITY, collide=False)

    _link(stage, "/Head/tongue", TONGUE_MASS, (0.04, 0.0, TONGUE_TOP - TONGUE_T / 2))
    for n, lo, hi in TONGUE_BOXES:
        mesh(stage, f"/Head/tongue/{n}", *box_mesh(lo, hi), TONGUE, collide=True)

    UsdGeom.Scope.Define(stage, "/Head/joints")
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Head/joints/root_joint")
    fixed.CreateBody1Rel().SetTargets([Sdf.Path("/Head/mount")])
    _joint(stage, "prismatic", "neck", "/Head/mount", "/Head/skull", (0.0, 0.0, 0.0), "X", -0.02, 0.02)
    _joint(stage, "revolute", "jaw", "/Head/skull", "/Head/jaw", (JAW_HINGE[0], 0.0, JAW_HINGE[1]), "Y",
           math.degrees(JAW_OPEN) - 3.0, 1.0)
    _joint(stage, "revolute", "swallow", "/Head/skull", "/Head/tongue", (SWALLOW_HINGE[0], 0.0, SWALLOW_HINGE[1]),
           "Y", -1.0, math.degrees(SWALLOW_TIP) + 3.0)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


def spoon_parts():
    """The spoon's collision pieces, as (name, points) in the spoon frame."""
    L, W = BOWL_L, BOWL_W
    x0, x1 = -L / 2, L / 2
    box = lambda lo, hi: box_mesh(lo, hi)[0]  # noqa: E731
    parts = [
        ("floor", box((x0, -W / 2, 0.0), (x1, W / 2, FLOOR_T))),
        ("tip", [(x1, -W / 2 + 0.002, 0.0), (x1, W / 2 - 0.002, 0.0), (x1, -W / 2 + 0.002, FLOOR_T),
                 (x1, W / 2 - 0.002, FLOOR_T), (x1 + TIP_L, -W / 2 + 0.004, 0.0),
                 (x1 + TIP_L, W / 2 - 0.004, 0.0), (x1 + TIP_L, -W / 2 + 0.004, 0.0002),
                 (x1 + TIP_L, W / 2 - 0.004, 0.0002)]),
        ("rim_l", box((x0, W / 2 - 0.0015, 0.0), (x1 - 0.006, W / 2, SIDE_H))),
        ("rim_r", box((x0, -W / 2, 0.0), (x1 - 0.006, -W / 2 + 0.0015, SIDE_H))),
        ("rim_back", box((x0 - 0.0015, -W / 2, 0.0), (x0, W / 2, RIM_H))),
        ("neck", [(x0 - 0.0015, -0.004, 0.002), (x0 - 0.0015, 0.004, 0.002), (x0 - 0.0015, -0.004, 0.006),
                  (x0 - 0.0015, 0.004, 0.006), (HANDLE[0][1], -0.005, HANDLE_Z - 0.003),
                  (HANDLE[0][1], 0.005, HANDLE_Z - 0.003), (HANDLE[0][1], -0.005, HANDLE_Z + 0.002),
                  (HANDLE[0][1], 0.005, HANDLE_Z + 0.002)]),
        ("handle", box((HANDLE[0][0], -HANDLE[1] / 2, HANDLE_Z - HANDLE[2] / 2),
                       (HANDLE[0][1], HANDLE[1] / 2, HANDLE_Z + HANDLE[2] / 2))),
    ]
    return parts


def spoon():
    parts = {}
    for n, p in spoon_parts():
        pts, faces = hull_mesh(p)
        parts[n] = (pts, faces, GRIP if n == "handle" else STEEL, True)
    body("spoon", SPOON_MASS, parts, com=(-0.07, 0.0, 0.012))
    for n, _ in spoon_parts():
        _bind_friction("spoon", f"/Body/{n}", HANDLE_FRICTION if n == "handle" else SPOON_FRICTION,
                       "grip" if n == "handle" else "steel")


def _bind_friction(name, prim, friction, mat_name):
    """Give one collision piece of an asset its own physics material (e.g. a rubber handle on a steel spoon)."""
    from pxr import Usd, UsdPhysics, UsdShade

    path = HERE / "assets" / f"{name}.usda"
    stage = Usd.Stage.Open(str(path))
    mat = UsdShade.Material.Define(stage, f"/Body/Looks/{mat_name}")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction[0])
    api.CreateDynamicFrictionAttr(friction[1])
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(prim)).Bind(
        mat, UsdShade.Tokens.strongerThanDescendants, "physics")
    stage.GetRootLayer().Save()


def dish():
    """A rectangular scoop dish: flat floor, four straight 3 cm walls (the far one is what food is pushed against)."""
    L, W, t, h = DISH_L / 2, DISH_W / 2, DISH_WALL, DISH_RIM_H
    parts = {"floor": (*box_mesh((-L, -W, 0.0), (L, W, DISH_T)), DISH, True),
             "wall_far": (*box_mesh((L - t, -W, 0.0), (L, W, h)), DISH, True),
             "wall_near": (*box_mesh((-L, -W, 0.0), (-L + t, W, h)), DISH, True),
             "wall_left": (*box_mesh((-L, W - t, 0.0), (L, W, h)), DISH, True),
             "wall_right": (*box_mesh((-L, -W, 0.0), (L, -W + t, h)), DISH, True)}
    body("dish", DISH_MASS, parts, com=(0.0, 0.0, 0.008))


CHAMFER = 0.0025                             # m: the cube's edges are bevelled (a soft cube's edges round off)


def peach():
    h, c = CUBE / 2, CUBE / 2 - CHAMFER
    pts = []
    for sx in (1, -1):
        for sy in (1, -1):
            for sz in (1, -1):
                pts += [(sx * h, sy * c, sz * c), (sx * c, sy * h, sz * c), (sx * c, sy * c, sz * h)]
    body("peach", CUBE_MASS, {"cube": (*hull_mesh(pts), PEACH, True)})


def main() -> None:
    head()
    spoon()
    dish()
    peach()


if __name__ == "__main__":
    main()
