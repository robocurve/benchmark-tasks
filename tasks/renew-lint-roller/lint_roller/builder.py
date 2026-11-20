"""Original lint roller, sheet edge and cloth geometry."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

HANDLE = (.11, .32, .28)
GRIP = (.09, .14, .13)
PAPER = (.97, .89, .63)
CLEAN = (.86, .88, .85)
CLOTH = (.065, .21, .43)


def roller_asset(w):
    s = w.stage("roller")
    handle = w.body(s, "/Asset/Handle", .060, articulation=True)
    w.cylinder(s, handle + "/Axle", .008, .049, (0, .064, 0), GRIP, axis="Y")
    w.cylinder(s, handle + "/Collar", .017, .014, (0, .085, 0), HANDLE, axis="Y")
    w.cylinder(s, handle + "/Grip", .014, .080, (0, .126, 0), HANDLE, axis="Y")
    w.cylinder(s, handle + "/Pommel", .017, .013, (0, .168, 0), GRIP, axis="Y")
    w.Px.PhysxArticulationAPI.Get(s, handle).CreateEnabledSelfCollisionsAttr(False)
    hub = w.body(s, "/Asset/Hub", .057)
    w.cylinder(s, hub + "/Roll", C.RADIUS, C.WIDTH, (0, 0, 0), CLEAN, axis="Y")
    for sign in (-1, 1):
        side = "Near" if sign < 0 else "Far"
        w.cylinder(s, hub + "/" + side, .0135, .002,
                   (0, sign * (.0007 + C.WIDTH / 2), 0), GRIP, axis="Y")
    joint = w.Phy.RevoluteJoint.Define(s, "/Asset/Spindle")
    joint.CreateBody0Rel().SetTargets([handle])
    joint.CreateBody1Rel().SetTargets([hub])
    joint.CreateAxisAttr("Y")
    joint.CreateCollisionEnabledAttr(False)
    return s


def tab_asset(w):
    s = w.stage("sheet_edge")
    tab = w.body(s, "/Asset/Edge", C.TAB_MASS, ccd=True)
    w.box(s, tab + "/PaperEdge", C.TAB_SIZE, (0, 0, 0), PAPER)
    m = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    m.CreateStaticFrictionAttr(.82)
    m.CreateDynamicFrictionAttr(.72)
    phys = w.Px.PhysxRigidBodyAPI.Get(s, tab)
    phys.CreateDisableGravityAttr(True)
    phys.CreateLinearDampingAttr(10.)
    phys.CreateAngularDampingAttr(8.)
    return s


def cloth_asset(w):
    s = w.stage("cloth")
    cloth = w.body(s, "/Asset/Cloth", .034, ccd=True)
    w.box(s, cloth + "/Weave", C.CLOTH_SIZE, (0, 0, 0), CLOTH, collide=False)
    backing = C.CLOTH_SIZE - [0., 0., .002]
    w.box(s, cloth + "/FeltBacking", backing, (0, 0, -.001), CLOTH)
    w.Geom.Imageable(s.GetPrimAtPath(cloth + "/FeltBacking")).MakeInvisible()
    for side in (-1, 1):
        w.box(s, cloth + ("/HemNear" if side < 0 else "/HemFar"), (C.CLOTH_SIZE[0], .005, .001),
              (0, side * (C.CLOTH_SIZE[1] / 2 - .004), C.CLOTH_SIZE[2] / 2), (.092, .30, .55),
              collide=False)
    m = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    m.CreateStaticFrictionAttr(.76)
    m.CreateDynamicFrictionAttr(.59)
    return s


def fixtures(w):
    s = w.stage("fixtures")
    w.box(s, "/Asset/RollRest", (.073, .110, C.REST),
          (C.ROLLER[0], C.ROLLER[1], C.TABLE + C.REST/2), (.27, .30, .31))
    w.box(s, "/Asset/PommelRest", (.057, .021, C.REST + .008),
          (C.ROLLER[0], C.ROLLER[1] + .169, C.TABLE + (C.REST + .008)/2), (.27, .30, .31))
    for sign, name in ((-1, "Front"), (1, "Rear")):
        w.box(s, "/Asset/RollStop" + name, (.006, .085, .007),
              (C.ROLLER[0] + .026*sign, C.ROLLER[1], C.TABLE + C.REST + .0035), (.27, .30, .31))
    return s


def create_assets(directory):
    writer = Writer(directory)
    objects = [
        ("roller", "articulation", C.ROLLER, roller_asset(writer)),
        ("sheet_edge", "rigid", C.TAB, tab_asset(writer)),
        ("cloth", "rigid", C.CLOTH, cloth_asset(writer)),
        ("fixtures", "base", np.zeros(3), fixtures(writer)),
    ]
    assets = [
        dict(name=name, type=kind, position=np.asarray(pose).tolist(), file=writer.save(stage))
        for name, kind, pose, stage in objects
    ]
    assets[1]["quaternion"] = [0., math.sin(C.TAB_TILT / 2), 0., math.cos(C.TAB_TILT / 2)]
    return {"assets": assets, "settle_actions": 18}
