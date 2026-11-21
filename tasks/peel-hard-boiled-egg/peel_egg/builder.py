"""Original millimetre-scale egg geometry with native shell and membrane bonds."""

import numpy as np

from . import config as C
from .usd_scene import Writer
from .motion import rotation


def profile(s, radius, half_length):
    """Measured ovoid proportions. The negative pole has the fuller shoulder."""
    return radius * np.sqrt(max(0., 1 - s * s)) * (1 - .14 * s), s * half_length


def ring(s, radius, length, angle):
    r, y = profile(s, radius, length)
    return r * np.cos(angle), y, r * np.sin(angle)


def rings_mesh(w, stage, path, axial, angles, outer, inner=None, collide=False,
               color=(1., 1., 1.), seam=None):
    """Surface or closed convex patch of a thin egg contour."""
    def outline(size):
        return [ring(seam(s, theta) if seam else s, *size, theta)
                for s in axial for theta in angles]
    points = outline(outer)
    n, m = len(angles), len(axial)
    faces = [[j * n + i, j * n + i + 1, (j + 1) * n + i + 1, (j + 1) * n + i]
             for j in range(m - 1) for i in range(n - 1)]
    if inner is not None:
        count = len(points)
        points += outline(inner)
        faces += [[i + count for i in reversed(face)] for face in faces.copy()]
        border = (list(range(n)) + [j * n + n - 1 for j in range(1, m)] +
                  list(reversed(range((m - 1) * n, m * n - 1))) +
                  [j * n for j in range(m - 2, 0, -1)])
        faces += [[a, b, b + count, a + count]
                  for a, b in zip(border, border[1:] + border[:1])]
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*map(float, p)) for p in points])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    if collide:
        w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(stage, mesh, color, collide, contact=.00025)
    return mesh


def kitchen(w):
    s = w.stage("kitchen")
    w.box(s, "/Asset/Bench", (1.1, 1.25, .060), (.38, 0, C.TABLE - .03), (.53, .45, .33))
    w.box(s, "/Asset/Floor", (8., 8., .020), (0., 0., -.010), (.31, .34, .32))
    w.rounded_box(s, "/Asset/WoodBoard", (.244, .225, .012), C.BOARD - [0., 0., .006],
                  (.65, .45, .23), .012, .002)
    plate = C.PLATE
    w.cylinder(s, "/Asset/BluePlateBase", .077, .0075, plate + [0., 0., .00375],
               (.16, .38, .58))
    for i, a in enumerate(np.linspace(0, 2 * np.pi, 32, endpoint=False)):
        radial = np.array([np.cos(a), np.sin(a), 0.])
        w.box(s, f"/Asset/PlateLip{i}", (.013, .016, .009),
              plate + radial * .073 + [0., 0., .008], (.20, .45, .66), rotation("Z", a))
    return s


def cracks(w, stage, parent, bounds, seam):
    """An impact reveals hairline cracks on the still-attached shell."""
    curve = w.Geom.BasisCurves.Define(stage, f"{parent}/Cracks")
    paths = []
    edge = bounds[1]
    if edge < .99:
        angles = np.linspace(0., 2 * np.pi, 73)
        paths.append([ring(seam(edge - .012, theta), .0218, .0292, theta)
                      for theta in angles])
    for theta in (.51, 2.37, 4.03):
        rows = np.linspace(bounds[0] * .21 + bounds[1] * .79,
                           bounds[0] * .87 + bounds[1] * .13, 12)
        angles = theta + .10 * np.sin(np.linspace(0., 7., len(rows))) + .22 * (rows - rows[-1])
        paths.append([ring(seam(row, phi), .0218, .0292, phi)
                      for row, phi in zip(rows, angles)])
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr([len(path) for path in paths])
    curve.CreatePointsAttr([w.Gf.Vec3f(*map(float, pt)) for path in paths for pt in path])
    curve.CreateWidthsAttr([.00061])
    curve.SetWidthsInterpolation("constant")
    curve.CreateDisplayColorAttr([w.Gf.Vec3f(.28, .135, .085)])
    curve.CreateVisibilityAttr("invisible")


def egg(w):
    s = w.stage("egg")
    material = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    material.CreateStaticFrictionAttr(.88)
    material.CreateDynamicFrictionAttr(.72)
    core = w.body(s, "/Asset/core", .054, C.START, ccd=True)
    w.Px.PhysxRigidBodyAPI.Get(s, core).CreateSolverPositionIterationCountAttr(32)
    rings_mesh(w, s, core + "/White", np.linspace(-1., 1., 21),
               np.linspace(0., 2 * np.pi, 33), (.02055, .02830), collide=True,
               color=(.96, .95, .88))

    shells = []
    for name, bounds in C.SHELL_BOUNDS.items():
        def seam(s, theta):
            amplitude = .09 * np.sin(5 * theta + .6) + .036 * np.cos(9 * theta)
            shift = max(0., 1. - abs(s + .55) / .40)
            shift -= max(0., 1. - abs(s - .45) / .46)
            return s + amplitude * shift
        color = (.72, .45, .27) if name == "blunt" else (.74, .47, .29)
        shell = w.body(s, f"/Asset/{name}", .0032, C.START, ccd=True)
        w.Px.PhysxRigidBodyAPI.Get(s, shell).CreateSolverPositionIterationCountAttr(32)
        shells.append(shell)
        outside = (.02155, .029)
        inside = (.02090, .02845)
        rings_mesh(w, s, shell + "/Shell", np.linspace(*bounds, 16),
                   np.linspace(0., 2 * np.pi, 49), outside, inside, color=color, seam=seam)
        cracks(w, s, shell, bounds, seam)
        for k, edge in enumerate(np.linspace(0., 2 * np.pi, 13)[:-1]):
            patch = rings_mesh(w, s, shell + f"/Hull{k}", np.linspace(*bounds, 8),
                               np.linspace(edge, edge + np.pi / 6, 4), outside, inside,
                               collide=True, color=color, seam=seam)
            patch.CreateVisibilityAttr("invisible")
        joint = w.Phy.FixedJoint.Define(s, f"/Asset/{name}Membrane")
        joint.CreateBody0Rel().SetTargets([core])
        joint.CreateBody1Rel().SetTargets([shell])
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        force, torque = {"blunt": (3.1, .051), "band": (8.0, .135),
                         "point": (9.0, .144)}[name]
        joint.CreateBreakForceAttr(force)
        joint.CreateBreakTorqueAttr(torque)
    # The two closed seam boundaries meet at reset. Filter their overlapping
    # convex approximations to avoid a fictitious contact between bonded shell pieces.
    for shell in shells:
        others = [other for other in shells if other != shell]
        w.Phy.FilteredPairsAPI.Apply(s.GetPrimAtPath(shell)).CreateFilteredPairsRel().SetTargets(others)
    core_filter = w.Phy.FilteredPairsAPI.Apply(s.GetPrimAtPath(core)).CreateFilteredPairsRel()
    core_filter.SetTargets(shells)
    return s


def create_assets(directory):
    w = Writer(directory)
    stages = (("kitchen", kitchen(w)), ("egg", egg(w)))
    assets = [dict(name=name, type="base", file=w.save(stage), position=(0., 0., 0.),
                   quaternion=(0., 0., 0., 1.)) for name, stage in stages]
    return dict(assets=assets, settle_actions=16)
