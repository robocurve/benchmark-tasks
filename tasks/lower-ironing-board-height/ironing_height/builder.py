"""Native closed-loop scissor stand with a sliding pin and bearing latch."""

import numpy as np

from . import config as C
from .usd_scene import Writer

STEEL = (.53, .57, .59)
FRAME = (.24, .30, .32)
RUBBER = (.13, .16, .17)
PAD = (.43, .63, .63)
LOCK = (.88, .43, .10)
TRACK = (.36, .37, .39)
PIVOT = (.74, .76, .77)


def station(w):
    s = w.stage("station")
    w.box(s, "/Asset/Floor", (4.0, 5.0, .02), (.60, 0., -.01), (.36, .40, .42))
    w.box(s, "/Asset/BackWall", (.04, 4.0, 2.), (1.5, 0., 1.), (.66, .67, .63),
          collide=False)
    w.box(s, "/Asset/HeightScaleFoot", (.086, .07, .014), (.652, -.430, .007),
          FRAME)
    w.box(s, "/Asset/HeightScale", (.023, .042, .93), (.652, -.430, .465),
          STEEL)
    for i, z in enumerate(np.arange(.70, .93, .025)):
        w.box(s, f"/Asset/ScaleTick{i}", (.002, .023, .0025),
              (.6395, -.426, z), FRAME, collide=False)
    w.box(s, "/Asset/TargetBand", (.002, .041, .014),
          (.6395, -.430, C.LOWER_Z+.051), (.10, .56, .32), collide=False)
    for i, y in enumerate((-.305, .305)):
        w.box(s, f"/Asset/Pedestal{i}", (.215, .18, .75), (-.055, y, .375), FRAME)
    return s


def joint(w, s, name, kind, body_a, body_b, anchor, axis, *, limits=None,
          loop=False):
    primitive = {"hinge": w.Phy.RevoluteJoint, "slide": w.Phy.PrismaticJoint}[kind]
    j = primitive.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([body_a[0]])
    j.CreateBody1Rel().SetTargets([body_b[0]])
    anchor = np.asarray(anchor)
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, anchor - body_a[1])))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, anchor - body_b[1])))
    j.CreateAxisAttr(axis)
    if limits is not None:
        j.CreateLowerLimitAttr(limits[0])
        j.CreateUpperLimitAttr(limits[1])
    if loop:
        j.CreateExcludeFromArticulationAttr(True)
    return j


def support_tubes(w, s, body, top, tag, spacing):
    path, origin = body
    top = np.asarray(top)
    offset = top - origin
    direction = offset / np.linalg.norm(offset)
    orientation = np.column_stack(([1., 0., 0.], np.cross(direction, [1., 0., 0.]),
                                   direction))
    for i, x in enumerate((-spacing, spacing)):
        w.box(s, f"{path}/{tag}{i}", (.022, .022, np.linalg.norm(offset)),
              offset/2 + [x, 0, 0], STEEL, rotation=orientation, collide=False)
    w.cylinder(s, path + "/TopBar", .011, 2*spacing + .015, offset, STEEL, "X",
               collide=False)
    w.cylinder(s, path + "/FootBar", .014, 2*spacing + .058, (0., 0., -.007), STEEL,
               "X", collide=False)
    for i, x in enumerate((-spacing - .02, spacing + .02)):
        w.cylinder(s, f"{path}/Foot{i}", .017, .035, (x, 0., -.007), RUBBER, "X",
                   collide=False)


def board(w):
    s = w.stage("board")
    root = "/Asset/Base"
    w.body(s, root, 2.1, fixed=True, articulation=True)
    base = (root, np.zeros(3))
    x = C.CENTER_X
    p = np.array([x, C.FIXED_Y, C.BASE_Z])
    q = np.array([x, C.UPPER_Y, C.BASE_Z])
    a_top = np.array([x, C.UPPER_Y, C.UPPER_Z])
    b_top = np.array([x, C.FIXED_Y, C.UPPER_Z])
    mid = (p + a_top) / 2

    def body(name, mass, pos):
        path = w.body(s, "/Asset/" + name, mass, pos)
        return path, np.asarray(pos)

    frame_a = body("ScissorOne", 1.0, p)
    frame_b = body("ScissorTwo", 1.0, q)
    board_body = body("Worktop", 2.7, [0., 0., C.UPPER_Z])
    top_carriage = body("UpperRunner", .17, a_top)
    low_carriage = body("FootRunner", .24, q)
    latch = body("ReleaseLever", .16, C.HANDLE_PIVOT)

    support_tubes(w, s, frame_a, a_top, "Leg", .106)
    support_tubes(w, s, frame_b, b_top, "Leg", .133)
    w.cylinder(s, frame_a[0] + "/CenterPivot", .015, .285, mid-p, PIVOT, "X",
               collide=False)
    w.box(s, low_carriage[0] + "/RunnerShoe", (.20, .045, .017), (0., 0., -.013),
          STEEL, collide=False)
    w.box(s, base[0] + "/FloorGuide", (.17, .24, .008), (x, -.376, .004),
          FRAME, collide=False)
    top = board_body[0]
    w.rounded_box(s, top + "/PressedSteelDeck", (.30, 1.10, .029), (x, 0., .028),
                  TRACK, .135, .0025)
    w.rounded_box(s, top + "/CottonCover", (.302, 1.10, .010), (x, 0., .046),
                  PAD, .14, .0015, collide=False)
    for k, y in enumerate(np.linspace(-.42, .42, 13)):
        w.box(s, f"{top}/CoverStripe{k}", (.284, .004, .001),
              (x, y, .0513), (.59, .75, .72), collide=False)
    w.rounded_box(s, top + "/IronRestFrame", (.245, .12, .018), (x, .608, .027),
                  STEEL, .038, .002)
    w.rounded_box(s, top + "/IronRestSilicone", (.188, .086, .007), (x, .608, .040),
                  RUBBER, .024, .001, collide=False)
    for i, dx in enumerate((-.026, .026)):
        w.box(s, f"{top}/SlideGuide{i}", (.011, .646, .031), (x + dx, -.16, -.005),
              TRACK, collide=False)
    w.box(s, top + "/ReleaseMount", (.049, .380, .020), (.338, -.253, -.022),
          STEEL, collide=False)
    w.box(s, top_carriage[0] + "/RunnerBlock", (.035, .025, .031),
          (0., 0., -.013), STEEL, collide=False)
    w.box(s, top_carriage[0] + "/IndexShoulder", C.SHOULDER_SIZE,
          C.SHOULDER_CENTER, STEEL)
    s_latch = latch[0]
    w.cylinder(s, s_latch + "/Shaft", .005, .281, (0., .017, 0.), LOCK, "Y",
               collide=False)
    handle_x, handle_y, _ = C.HANDLE_END
    w.box(s, s_latch + "/Arm", (abs(handle_x), .033, .008),
          (handle_x/2, handle_y, 0.), LOCK)
    w.rounded_box(s, s_latch + "/ThumbGrip", (.032, .044, .016),
                  (handle_x, handle_y, 0.), LOCK, .009, .002)
    for i, target in enumerate((C.UPPER_Y, C.LOWER_Y)):
        notch = target - C.HANDLE_PIVOT[1] - .0205
        w.box(s, f"{s_latch}/CatchArm{i}", (.101, .012, .013), (.052, notch, -.011),
              LOCK, collide=False)
        w.box(s, f"{s_latch}/CatchTooth{i}", C.CATCH_SIZE, (.10, notch, -.017),
              LOCK)
    w.box(s, top+"/LowerHeightIndex", (.042, .014, .005), (x, C.LOWER_Y, -.031),
          (.19, .58, .42), collide=False)

    joint(w, s, "GroundHinge", "hinge", base, frame_a, p, "X",
          limits=(-30., 20.))
    joint(w, s, "CenterHinge", "hinge", frame_a, frame_b, mid, "X",
          limits=(-45., 45.))
    joint(w, s, "DeckHinge", "hinge", frame_b, board_body, b_top, "X",
          limits=(-28., 28.))
    joint(w, s, "TopGuide", "slide", board_body, top_carriage, a_top, "Y",
          limits=(-.135, .030))
    joint(w, s, "FootGuide", "slide", base, low_carriage, q, "Y",
          limits=(-.135, .030))
    joint(w, s, "UpperClosure", "hinge", top_carriage, frame_a, a_top, "X", loop=True)
    joint(w, s, "FootClosure", "hinge", low_carriage, frame_b, q, "X", loop=True)
    lever = joint(w, s, "HeightLever", "hinge", board_body, latch, C.HANDLE_PIVOT,
                  "Y", limits=(0., C.LEVER_STOP))
    drive = w.Phy.DriveAPI.Apply(lever.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(.035)
    drive.CreateDampingAttr(.006)
    drive.CreateTargetPositionAttr(0.)
    return s


def create_assets(directory):
    w = Writer(directory)
    data = [
        {"name": "station", "type": "base", "file": w.save(station(w))},
        {"name": "board", "type": "articulation", "file": w.save(board(w))},
    ]
    w.inventory()
    return {"assets": data, "settle_actions": 28}
