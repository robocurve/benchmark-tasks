"""Board and physically opposed stretch and tuck modes of the padded cover."""

from . import config as C
from .usd_scene import Writer

STEEL = (.32, .38, .42)
PAD = (.08, .47, .62)
EDGE = (.89, .43, .08)
DECK_OUTLINE = [
    (-.145, -.330), (-.131, -.355), (-.104, -.365), (.104, -.365),
    (.131, -.355), (.145, -.330), (.16, -.280), (.16, .100),
    (.142, .186), (.113, .264), (.071, .329), (.027, .359), (0, .365),
    (-.027, .359), (-.071, .329), (-.113, .264), (-.142, .186),
    (-.16, .100), (-.16, -.280),
]


def profile(w, stage, path, outline, low, high, color, collide=True):
    """A rounded footprint extruded as a thin convex shell."""
    solid = w.Geom.Mesh.Define(stage, path)
    size = len(outline)
    if sum(x0*y1 - x1*y0 for (x0,y0),(x1,y1) in zip(outline, outline[1:] + outline[:1])) < 0:
        outline.reverse()
    solid.CreatePointsAttr([(x, y, z) for z in (low, high) for x, y in outline])
    solid.CreateFaceVertexCountsAttr([size, size] + [4] * size)
    indices = list(reversed(range(size))) + list(range(size, 2 * size))
    for i in range(size):
        j = (i + 1) % size
        indices.extend((i, j, j + size, i + size))
    solid.CreateFaceVertexIndicesAttr(indices)
    solid.CreateSubdivisionSchemeAttr("none")
    w._finish(stage, solid, color, collide, .001)
    if collide:
        w.Phy.MeshCollisionAPI.Apply(solid.GetPrim()).CreateApproximationAttr("convexHull")


def board(w):
    stage = w.stage("board")
    profile(w, stage, "/Asset/SteelDeck", DECK_OUTLINE, -C.BOARD_THICK, 0., STEEL)
    for y in (-.224, .212):
        w.box(stage, f"/Asset/Crossbar{y}", (.246,.021,.025),
              (0,y,-.037), (.20,.21,.22))
        for x in (-.102, .102):
            w.cylinder(stage, f"/Asset/Leg{x}_{y}", .012, .073,
                       (x,y,-.065), (.28,.29,.32))
            w.box(stage, f"/Asset/Foot{x}_{y}", (.035,.032,.018),
                  (x,y,-.096), (.14,.14,.16))
    return w.save(stage)


def slider(w, stage, name, parent, child, anchor, axis, lower, upper, k, target):
    joint = w.Phy.PrismaticJoint.Define(stage, f"/Asset/{name}")
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*anchor))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0))
    joint.CreateLowerLimitAttr(lower)
    joint.CreateUpperLimitAttr(upper)
    if k:
        spring = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
        spring.CreateTypeAttr("force")
        spring.CreateTargetPositionAttr(target)
        spring.CreateStiffnessAttr(k)
        spring.CreateDampingAttr(C.DAMPING)
        spring.CreateMaxForceAttr(12.)


def cover(w):
    stage = w.stage("cover")
    pad = w.body(stage, "/Asset/Pad", .124, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, pad).CreateEnabledSelfCollisionsAttr(False)
    w.box(stage, pad + "/PaddedCentre", (.321, .500, C.TOP), (0.,-.030,0.), PAD)
    w.Geom.Imageable(stage.GetPrimAtPath(pad + "/PaddedCentre")).MakeInvisible()
    for sign, end, width in ((1,"Nose",.073),(-1,"Heel",.222)):
        end_y = sign * C.NEUTRAL_LENGTH
        panel = w.body(stage, f"/Asset/{end}Textile", .022, (0., end_y, 0.), ccd=True)
        lip = w.body(stage, f"/Asset/{end}Hem", .025, (0., end_y, .009), ccd=True)
        slider(w, stage, end + "Stretch", pad, panel, (0., end_y, 0.), "Y",
               min(0.,sign*C.TRAVEL), max(0.,sign*C.TRAVEL), C.SPRING, -sign*.010)
        slider(w, stage, end + "Tuck", panel, lip, (0., 0., .009), "Z", -C.DROP, .026, 22., .010)
        flare = .302 if sign == 1 else .319
        polygon = [(-width/2, .048), (-width/4,.061), (width/4,.061), (width/2,.048),
                   (flare/2,-.234),(-flare/2,-.234)]
        profile(w, stage, panel + "/PadEnd", [(x,y*sign) for x,y in (polygon if sign == 1 else reversed(polygon))],
                -C.TOP/2, C.TOP/2, PAD, collide=False)
        w.Geom.Imageable(stage.GetPrimAtPath(panel + "/PadEnd")).MakeInvisible()
        w.cylinder(stage, lip + "/SewnElastic", .006, width,
                   (0.,0.,0.), EDGE, axis="X")
        w.box(stage, lip+"/HemSleeve", (width,.011,.010), (0.,-.004*sign,.003), PAD)
    return w.save(stage)


def create_assets(directory):
    w = Writer(directory)
    assets = [dict(name="board", type="base", file=board(w), position=(C.BOARD_X,0.,C.BOARD_TOP)),
              dict(name="cover", type="articulation", file=cover(w), position=(C.BOARD_X,0.,C.START_Z))]
    w.inventory()
    return {"assets": assets, "settle_actions": 10}
