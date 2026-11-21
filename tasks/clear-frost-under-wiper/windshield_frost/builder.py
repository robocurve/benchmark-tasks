"""Native pane, spring wiper and scraper contacts, with a thin frost overlay."""

import numpy as np

from . import config as C
from .usd_scene import Writer

BENCH = (.47, .37, .24)
FRAME = (.105, .125, .14)
GLASS = (.14, .30, .36)
RUBBER = (.048, .054, .059)
STEEL = (.23, .27, .29)
PAINT = (.16, .22, .39)
GRIP = (.78, .33, .08)
EDGE = (.86, .71, .23)


def window(w):
    s = w.stage("window")
    w.box(s, "/Asset/Bench", (1.1, 1.25, .060), (.32, 0, C.TABLE - .03), BENCH)
    for i, x in enumerate((-.18, .81)):
        for j, y in enumerate((-.57, .57)):
            w.box(s, f"/Asset/Leg{i}{j}", (.055, .055, .68), (x, y, .345), FRAME)
    w.box(s, "/Asset/Floor", (8., 8., .020), (0., 0., -.010), (.31, .34, .32))

    pane = w.body(s, "/Asset/Pane", 8.5, C.GLASS_ORIGIN)
    # Use a kinematic body for the pane in the otherwise static scene.
    w.Phy.RigidBodyAPI.Get(s, pane).CreateKinematicEnabledAttr(True)
    local = lambda x, y, h: C.wind(x, y, h) - C.GLASS_ORIGIN
    w.box(s, pane + "/LaminatedGlass", (.671, .966, .008),
          local(.330, .111, -.004), GLASS, C.GLASS_ROT)
    for i, y in enumerate((-.383, .605)):
        w.box(s, f"/Asset/SideTrim{i}", (.677, .021, .033), C.wind(.333, y, -.008),
              FRAME, C.GLASS_ROT)
    w.box(s, "/Asset/TopSeal", (.023, .976, .015), C.wind(.667, .111, -.001),
          FRAME, C.GLASS_ROT)
    w.box(s, "/Asset/LowerSeal", (.023, .976, .016), C.wind(-.012, .111, -.002),
          FRAME, C.GLASS_ROT)
    w.rounded_box(s, "/Asset/Cowl", (.104, .997, .030), (.178, .111, .781),
                  FRAME, .012, .002)
    w.rounded_box(s, "/Asset/HoodEdge", (.095, 1.015, .032), (.089, .111, .783),
                  PAINT, .015, .003)
    for n, y in enumerate(np.linspace(-.338, .560, 31)):
        w.box(s, f"/Asset/Vent{n}", (.044, .006, .002), (.174, y, .797),
              (.048, .058, .066), collide=False)
    w.rounded_box(s, "/Asset/SpindleCap", (.047, .053, .030),
                  C.wind(-.033, .196, .011), STEEL, .010, .002, collide=False)

    dx = (C.BAND_X1 - C.BAND_X0) / C.CELLS_X
    dy = (C.BAND_Y1 - C.BAND_Y0) / C.CELLS_Y
    frost = []
    for ix in range(C.CELLS_X):
        for iy in range(C.CELLS_Y):
            shade = .80 + .025 * ((2 * ix + 3 * iy) % 5)
            name = f"Frost_{ix:02d}_{iy:02d}"
            frost.append(name)
            w.box(s, "/Asset/" + name, (dx * .996, dy * .997, .00020),
                  C.wind(C.BAND_X0 + (ix + .5) * dx, C.BAND_Y0 + (iy + .5) * dy, .00029),
                  (shade * .95, shade, shade), C.GLASS_ROT, collide=False)
    return s, frost


def fixture(w):
    s = w.stage("fixture")
    parent = w.body(s, "/Asset/Pivot", 2.0, fixed=True, articulation=True)
    w.cylinder(s, parent + "/Pin", .017, .037, C.HINGE, STEEL, axis="Y", collide=False)
    arm = w.body(s, "/Asset/Arm", .29, C.HINGE, ccd=True)
    # Relative link coordinates follow the slope of the glass at the lower stop.
    def part(name, size, center, color):
        w.box(s, arm + "/" + name, size, C.GLASS_ROT @ np.array(center), color,
              C.GLASS_ROT)
    part("SteelStem", (.207, .028, .016), (.121, 0., 0.), FRAME)
    part("SwanNeck", (.045, .061, .013), (.220, -.019, -.004), FRAME)
    part("Bridge", (.034, .123, .012), (.244, -.087, -.009), STEEL)
    part("BladeClip", (.030, .031, .020), (.250, C.BLADE_Y, -.015), STEEL)
    part("BladeSpine", (.022, C.BLADE_LENGTH, .010), (.250, C.BLADE_Y, -.022), FRAME)
    part("RubberLip", (.009, C.BLADE_LENGTH, .009),
         (.250, C.BLADE_Y, -.02625), RUBBER)

    j = w.Phy.RevoluteJoint.Define(s, "/Asset/Lift")
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([arm])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.HINGE)))
    j.CreateAxisAttr("Y")
    j.CreateCollisionEnabledAttr(False)
    j.CreateLowerLimitAttr(-24.)
    j.CreateUpperLimitAttr(2.)
    d = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(.082)
    d.CreateDampingAttr(.009)
    d.CreateMaxForceAttr(3.2)
    d.CreateTargetPositionAttr(9.)
    return s


def edge(w, s, path):
    points2 = [(.028, .006), (.071, .002), (C.EDGE_X, .000), (.081, .005),
               (.055, .016), (.028, .022)]
    points = [w.Gf.Vec3f(x, y, z) for y in (-C.EDGE_WIDTH / 2, C.EDGE_WIDTH / 2)
              for x, z in points2]
    count = len(points2)
    faces = [list(reversed(range(count))), list(range(count, 2 * count))]
    faces.extend([[i, (i + 1) % count, (i + 1) % count + count, i + count]
                  for i in range(count)])
    mesh = w.Geom.Mesh.Define(s, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(s, mesh, EDGE, True)


def scraper(w):
    s = w.stage("scraper")
    body = w.body(s, "/Asset/Scraper", .103, articulation=True, ccd=True)
    material = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    material.CreateStaticFrictionAttr(.82)
    material.CreateDynamicFrictionAttr(.65)
    w.rounded_box(s, body + "/Grip", (.111, .026, .023),
                  (-.036, 0., .0145), GRIP, .009, .002)
    w.rounded_box(s, body + "/FlaredHead", (.047, .093, .018),
                  (.035, 0., .013), GRIP, .008, .002)
    bevel = w.body(s, "/Asset/Blade", .011, ccd=True)
    edge(w, s, bevel + "/Bevel")
    # A few millimetres of compliance represent the actual ABS edge bending.
    # This axis is normal to the scraper handle. There is no activation latch.
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/Flex")
    joint.CreateBody0Rel().SetTargets([body])
    joint.CreateBody1Rel().SetTargets([bevel])
    joint.CreateAxisAttr("Z")
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateLowerLimitAttr(-.0008)
    joint.CreateUpperLimitAttr(.0070)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(C.FLEX_K)
    drive.CreateDampingAttr(32.)
    drive.CreateMaxForceAttr(65.)
    drive.CreateTargetPositionAttr(0.)
    return s


def create_assets(directory):
    w = Writer(directory)
    pane, frost = window(w)
    rows = (
        ("window", "base", pane, (0., 0., 0.)),
        ("fixture", "articulation", fixture(w), (0., 0., 0.)),
        ("scraper", "articulation", scraper(w), C.SCRAPER_START),
    )
    assets = [dict(name=name, type=kind, file=w.save(stage), position=list(pos),
                   quaternion=[0., 0., 0., 1.]) for name, kind, stage, pos in rows]
    w.inventory()
    return {"assets": assets, "settle_actions": 24, "frost": frost}
