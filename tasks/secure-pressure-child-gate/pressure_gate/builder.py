"""Native cam, gate panels, compressible bumpers and timber doorway."""

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

IVORY = (.79, .75, .67)
WEB = (.72, .70, .64)
PLASTIC = (.32, .34, .37)
RUBBER = (.135, .143, .151)
WALL = (.74, .72, .67)
WOOD = (.44, .28, .16)
HANDLE = (.17, .25, .30)


def passage(w):
    s = w.stage("passage")
    floor = C.FLOOR_Z
    w.box(s, "/Asset/LabFloor", (6., 6., .02), (0, 0, -.01), (.35, .39, .42))
    for i, y in enumerate((-.305, .305)):
        w.box(s, f"/Asset/Pedestal{i}", (.208, .185, .72), (-.090, y, .36), PLASTIC)
        w.box(s, f"/Asset/BasePlate{i}", (.220, .192, .03), (-.090, y, .735), PLASTIC)

    w.box(s, "/Asset/Landing", (1.16, 1.52, .080),
          (.663, 0, floor - .04), WOOD)
    for i, y in enumerate((-1., 1.)):
        w.box(s, f"/Asset/Jamb{i}", (.150, .052, 1.28),
              (C.GATE_X, y * (C.WIDTH / 2 + .026), floor + .64), (.48, .34, .22))
        w.box(s, f"/Asset/ReturnWall{i}", (.320, .32, 1.28),
              (.55, y * .536, floor + .64), WALL)
        for j, height in enumerate(C.CUP_HEIGHTS):
            z = floor + height
            path = f"/Asset/Cup{i}{j}"
            # Restraining flanks and a U bottom, open upward.
            w.box(s, path + "_Seat", (.048, .005, .050),
                  (C.GATE_X, y*(C.CUP_BOTTOM+.0025), z), PLASTIC)
            for wing in (-1, 1):
                w.box(s, path + f"_Wing{wing}", (.004, .012, .050),
                      (C.GATE_X+wing*.022, y*.339, z), PLASTIC)
            w.box(s, path + "_Lip", (.046, .012, .004),
                  (C.GATE_X, y*.339, z-.022), PLASTIC)
    for i in range(4):
        height = .177 * (i + 1)
        w.box(s, f"/Asset/Step{i}", (.256, .700, height),
              (.644 + .256*i, 0., floor + height/2), (.52, .38, .27))
        w.box(s, f"/Asset/Nosing{i}", (.027, .700, .022),
              (.516 + .256*i, 0., floor + height-.011), (.38, .25, .17))
    # Pictorial room context is kept outside the physical manipulation envelope.
    w.box(s, "/Asset/TrimPanel", (.028, .70, .079), (.69, 0., floor+.021), WOOD,
          collide=False)
    return s


def panel(w, s, path, *, outer, inner, x):
    height = C.GATE_HEIGHT
    zcenter = height / 2
    outer_rail = .029
    for name, y in (("Stile", outer), ("OtherStile", inner)):
        w.box(s, path + "/" + name, (.038, outer_rail, height), (x, y, zcenter), IVORY)
    span = abs(outer-inner) + outer_rail
    center = (outer+inner) / 2
    for name, z in (("Bottom", .024), ("Top", height-.024)):
        w.box(s, path+"/"+name, (.038, span, .048), (x, center, z), IVORY)
    for i, y in enumerate(np.linspace(outer, inner, 9)[1:-1]):
        w.box(s, path+f"/Slat{i}", (.012, .013, height-.086),
              (x, y, zcenter), WEB)
    for i, z in enumerate(np.linspace(.055, height-.055, 15)[1:-1]):
        w.box(s, path+f"/Web{i}", (.012, span-.028, .011),
              (x, center, z), WEB)


def prismatic(w, s, name, parent, child, pos0, pos1, limits, stiffness, damping):
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, pos0)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, pos1)))
    joint.CreateAxisAttr("Y")
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateDampingAttr(damping)
    drive.CreateStiffnessAttr(stiffness)
    return joint


def gate(w):
    s = w.stage("child_gate")
    a = w.body(s, "/Asset/HandlePanel", 1.27, articulation=True, ccd=True)
    b = w.body(s, "/Asset/SlidingPanel", 1.26, ccd=True)
    panel(w, s, a, outer=-.303, inner=.121, x=-.013)
    panel(w, s, b, outer=.303, inner=-.121, x=.028)
    prismatic(w, s, "Width", a, b, (0., 0., 0.), (0., 0., 0.),
              (0., .062), 24., 6.)

    for side, sign, path in (("Left", 1, b), ("Right", -1, a)):
        for i, z in enumerate(C.CUP_HEIGHTS):
            pivot = np.array([.0, sign*C.PAD_CENTER, z])
            pad = w.body(s, f"/Asset/{side}Pad{i}", .026, pivot)
            w.cylinder(s, pad + "/Rubber", C.PAD_RADIUS, C.PAD_LENGTH,
                       (0, 0, 0), RUBBER, "Y")
            w.cylinder(s, path + f"/{side}PadSpindle{i}", .008, .033,
                       (0, sign*.310, z), PLASTIC, "Y", collide=False)
            direction = (-.009, 0.) if sign > 0 else (0., .009)
            prismatic(w, s, f"{side}Compression{i}", path, pad, pivot, (0, 0, 0),
                      direction, 2250., 11.)

    pivot = C.LEVER_PIVOT
    lever = w.body(s, "/Asset/Lever", .087, pivot, ccd=True)
    w.Phy.MassAPI(s.GetPrimAtPath(lever)).CreateCenterOfMassAttr(
        w.Gf.Vec3f(-.013, 0., 0.))
    w.box(s, lever+"/LeverBar", (.027, C.LEVER_LENGTH, .018),
          (-.019, -C.LEVER_LENGTH/2, 0), HANDLE)
    w.cylinder(s, lever+"/Grip", .015, .039,
               (-.028, -C.LEVER_LENGTH, 0), HANDLE, "X")
    w.cylinder(s, lever+"/Eccentric", C.CAM_RADIUS, .020,
               (.006, 0, -C.CAM_OFFSET), PLASTIC, "X")
    w.cylinder(s, a+"/AxleBoss", .020, .018, pivot + [-.001, 0, 0],
               IVORY, "X", collide=False)
    # Steel follower aligned to the factory locking handle. The eccentric
    # moves from its narrow open profile to its full pressure radius.
    w.box(s, b+"/Follower", (.016, .010, .066),
          pivot + [.006, C.CAM_RADIUS+.006, 0], (.41, .43, .44))
    j = w.Phy.RevoluteJoint.Define(s, "/Asset/Lock")
    j.CreateBody0Rel().SetTargets([a])
    j.CreateBody1Rel().SetTargets([lever])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, pivot)))
    j.CreateAxisAttr("X")
    j.CreateLowerLimitAttr(0.)
    j.CreateUpperLimitAttr(C.LEVER_STOP)
    w.Px.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(.075)
    d = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.)
    d.CreateDampingAttr(.0025)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "passage", "type": "base", "file": w.save(passage(w))},
        {"name": "child_gate", "type": "articulation", "file": w.save(gate(w)),
         "position": (C.GATE_START_X, 0., C.FLOOR_Z + .0015)},
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 14}
