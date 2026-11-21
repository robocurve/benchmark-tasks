"""Compound collisions for the loose veneered shelf, guides and thumb latch."""
import numpy as np

from . import config as C
from .usd_scene import Writer

VENEER = (.75, .61, .43)
LEAF = (.81, .67, .50)
FRAME = (.48, .38, .25)
GUIDE = (.20, .23, .24)
LOCK = (.24, .29, .31)
METAL = (.46, .48, .48)


def station(w):
    s = w.stage("station")
    for i, y in enumerate((-.305, .305)):
        w.box(s, f"/Asset/Pedestal{i}", (.21, .195, .72),
              (-.045, y, .360), (.20, .22, .24))
        w.box(s, f"/Asset/BasePlate{i}", (.22, .20, .030),
              (-.045, y, .735), (.29, .31, .32))
    w.box(s, "/Asset/Floor", (5., 6., .022), (.6, 0., -.011), (.36, .40, .42))
    w.box(s, "/Asset/BackWall", (.040, 4., 2.), (1.95, 0., 1.), (.67, .67, .62),
          collide=False)
    return s


def table(w):
    s = w.stage("table")
    root = w.body(s, "/Asset/Table", 19., fixed=True, articulation=True)
    w.rounded_box(s, root + "/MainTop", (.800, .700, C.THICKNESS),
                  (C.SEAM_X + .400, 0., C.SURFACE_Z - C.THICKNESS/2),
                  VENEER, .011, .0016)
    for i, y in enumerate((-.309, .309)):
        for j, x in enumerate((.332, 1.472)):
            w.box(s, f"{root}/Leg{i}_{j}", (.048, .048, .680),
                  (x, y, .340), FRAME)
        w.box(s, f"{root}/SideBeam{i}", (1.16, .046, .050),
              (.904, y, .684), FRAME)
    w.box(s, root + "/FarApron", (.035, .61, .042), (1.474, 0., .687), FRAME)
    w.box(s, root + "/FrontCrosspiece", (.032, .579, .024),
          (.325, 0., .649), FRAME)
    for i, y in enumerate((-.247, .247)):
        w.box(s, f"{root}/OpenSupportRail{i}", (.414, .047, .021),
              (.512, y, .7135), FRAME)
        # The assembly drawing depicts a receiver near each side below
        # the tabletop. Guide cheeks receive the leaf's two underside keys.
        for side, dy in enumerate((-.018, .018)):
            w.box(s, f"{root}/Guide{i}_{side}", (.051, .011, .017),
                  (.675, .205 * np.sign(y) + dy, .7195), GUIDE)
        w.box(s, f"{root}/SeamStop{i}", (.013, .039, .020),
              (.716, .205 * np.sign(y), .718), GUIDE)

    p = C.LATCH_PIVOT
    w.box(s, root + "/LatchMount", (.048, .043, .027),
          p + [.009, .027, -.012], METAL)
    child = w.body(s, "/Asset/Lever", .054, p, ccd=True)
    # Most latch mass sits in the steel hinge body, rather than the thumb tab.
    w.Phy.MassAPI(s.GetPrimAtPath(child)).CreateCenterOfMassAttr(w.Gf.Vec3f(-.004, 0., 0.))
    w.cylinder(s, child + "/Hinge", .008, .029, (0., 0., 0.), METAL, "Y")
    w.box(s, child + "/Arm", (.043, .026, .011), (-.025, 0., 0.), LOCK)
    w.cylinder(s, child + "/ThumbEnd", .008, .035, (-.047, 0., 0.), LOCK, "Y")
    w.box(s, child + "/CatchTop", C.CATCH_SIZE, C.CATCH_END, LOCK)

    j = w.Phy.RevoluteJoint.Define(s, "/Asset/FlipLever")
    j.CreateBody0Rel().SetTargets([root])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, p)))
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(0.)
    j.CreateUpperLimitAttr(C.LATCH_ANGLE)
    w.Px.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(.45)
    d = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateDampingAttr(.006)
    d.CreateStiffnessAttr(0.)
    return s


def leaf(w):
    s = w.stage("leaf")
    root = w.body(s, "/Asset/Leaf", C.LEAF_MASS, ccd=True)
    w.rounded_box(s, root + "/LeafPanel", (C.LEAF_DEPTH, C.LEAF_WIDTH, C.THICKNESS),
                  (0., 0., 0.), LEAF, .011, .0016)
    # Separate small low-friction shoes are the sliding support contacts.
    for i, y in enumerate((-.247, .247)):
        for j, x in enumerate((-.128, .156)):
            w.box(s, f"{root}/SlideShoe{i}_{j}", (.036, .024, .004),
                  (x, y, -.013), GUIDE)
        w.box(s, f"{root}/Locator{i}", (.030, .016, .013),
              (.182, .205 * np.sign(y), -.015), GUIDE)
    w.box(s, root + "/LatchReceiver", C.RECEIVER_SIZE, C.RECEIVER, METAL)
    return s


def create_assets(directory):
    w = Writer(directory)
    q = [0.0, 0.0, float(np.sin(C.START_YAW/2)), float(np.cos(C.START_YAW/2))]
    data = [
        {"name": "station", "type": "base", "file": w.save(station(w))},
        {"name": "table", "type": "articulation", "file": w.save(table(w))},
        {"name": "leaf", "type": "rigid", "file": w.save(leaf(w)),
         "position": (C.START_X, 0., C.SURFACE_Z - C.THICKNESS/2),
         "quaternion": q},
    ]
    w.inventory()
    return {"assets": data, "settle_actions": 16}
