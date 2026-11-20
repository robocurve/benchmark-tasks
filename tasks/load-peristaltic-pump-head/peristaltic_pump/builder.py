"""Procedural pump with separate native cam, occlusion shoes and tube links."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

HOUSING = (0.24, 0.34, 0.40)
EDGE = (0.48, 0.58, 0.63)
STEEL = (0.67, 0.73, 0.74)
LEVER = (0.97, 0.68, 0.15)


def joint(w, stage, name, kind, parent, child, axis, offset, lo, hi):
    joint = getattr(w.Phy, kind + "Joint").Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, offset)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateLowerLimitAttr(lo)
    joint.CreateUpperLimitAttr(hi)
    joint.CreateCollisionEnabledAttr(False)
    return joint


def drive(w, joint, kind, stiffness, damping, force):
    spring = w.Phy.DriveAPI.Apply(joint.GetPrim(), kind)
    spring.CreateTypeAttr("force")
    spring.CreateStiffnessAttr(stiffness)
    spring.CreateDampingAttr(damping)
    spring.CreateMaxForceAttr(force)


def pump(w):
    s = w.stage("pump")
    root = w.body(s, "/Asset/Case", 0.65, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, root).CreateEnabledSelfCollisionsAttr(False)
    w.box(s, root + "/Housing", (0.102, 0.101, 0.056), C.HEAD + (0, 0, 0.028), HOUSING)
    # Rounded four-roller rotor. The top roller extends across both parallel lanes.
    for i, (y, z) in enumerate(((-0.018, 0.049), (0, 0.068), (0.018, 0.049), (0, 0.030))):
        w.cylinder(s, root + f"/Roller{i}", 0.009, 0.047, C.HEAD + (0, y, z), STEEL, axis="X")
    for i, x in enumerate((-0.018, 0.0, 0.018)):
        w.cylinder(s, root + f"/RollerFlange{i}", 0.0125, 0.0037,
                   C.HEAD + (x, 0, 0.068), EDGE, axis="X")
    for side in (-1, 1):
        y = side * 0.046
        w.box(s, root + f"/RetainerBase{side}", (0.049, 0.011, 0.020), C.HEAD + (0, y, 0.067), EDGE)
        for i, x in enumerate((-0.018, 0.0, 0.018)):
            thickness = 0.005 if i == 1 else 0.004
            w.box(s, root + f"/RetainerPin{i}_{side}", (thickness, 0.011, 0.012),
                  C.HEAD + (x, y, 0.0830), EDGE)
    cam = w.body(s, "/Asset/Cam", 0.047, position=C.HEAD + C.PIVOT, ccd=True)
    w.box(s, cam + "/Bed", (0.058, 0.078, 0.008), (-0.044, 0, -0.009), HOUSING)
    w.cylinder(s, cam + "/Hinge", 0.008, 0.093, (0, 0, 0), EDGE, axis="Y")
    from .motion import rotation

    w.cylinder(s, cam + "/LeverAxle", 0.005, 0.110, (0, -0.055, 0), STEEL, axis="Y")
    w.box(s, cam + "/LeverArm", (0.010, 0.013, 0.030), (-0.010, C.LEVER[1], -0.010),
          LEVER, rotation=rotation("Y", math.pi / 4))
    w.cylinder(s, cam + "/LeverGrip", 0.0095, 0.019, C.LEVER, LEVER, axis="Y")
    hinge = joint(w, s, "CamClose", "Revolute", root, cam, "Y", C.HEAD + C.PIVOT, 0.0, 70.0)
    drive(w, hinge, "angular", 0.0, 0.025, 0.32)
    for i, x in enumerate(C.LANES):
        p = (x - C.PIVOT[0], 0, -0.0153)
        shoe = w.body(s, f"/Asset/Shoe{i}", 0.008, position=C.HEAD + C.PIVOT + p)
        w.box(s, shoe + "/OcclusionBed", (0.013, 0.096, 0.004), (0, 0, 0), STEEL)
        spring = joint(w, s, f"Compression{i}", "Prismatic", cam, shoe, "Z", p, 0.0, 0.008)
        drive(w, spring, "linear", 850.0, 9.0, 10.0)
    return s


def tube(w, name, color, stage_x):
    s = w.stage(name)
    # The inlet is captured in the outlet of a fixed feed bottle. The tube past
    # this real fitting can bend and shift, including its open discharge end.
    base = w.body(s, "/Asset/FeedBottle", 0.1, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, base).CreateEnabledSelfCollisionsAttr(False)
    z = C.TABLE + C.STAGE_Z
    w.cylinder(s, base + "/Bottle", 0.012, C.STAGE_Z - 0.013,
               (stage_x, C.PORT_Y - 0.011, C.TABLE + (C.STAGE_Z - 0.013) / 2), (0.62, 0.70, 0.75))
    w.box(s, base + "/Cap", (0.022, 0.026, 0.013), (stage_x, C.PORT_Y - 0.011, z - 0.0065), color)
    w.cylinder(s, base + "/Elbow", 0.0055, 0.009, (stage_x, C.PORT_Y - 0.001, z), color, axis="Y")
    for i in range(C.LINKS):
        p = (stage_x, C.PORT_Y + (i + 0.5) * C.TUBE_L, z)
        link = w.body(s, f"/Asset/Link{i}", 0.0020, position=p, ccd=True)
        geom = w.Geom.Capsule.Define(s, link + "/Silicone")
        geom.CreateRadiusAttr(C.TUBE_R)
        geom.CreateHeightAttr(C.TUBE_L)
        geom.CreateAxisAttr("Y")
        w._finish(s, geom, color, True)
        if i == 0:
            parent, anchor = base, (stage_x, C.PORT_Y, z)
        else:
            parent, anchor = f"/Asset/Link{i-1}", (0, C.TUBE_L / 2, 0)
        bend = joint(w, s, f"Bend{i}", "Revolute", parent, link,
                     "Z" if i % 2 == 0 else "X", anchor, -53.0, 53.0)
        bend.CreateLocalPos1Attr(w.Gf.Vec3f(0, -C.TUBE_L / 2, 0))
        drive(w, bend, "angular", 0.0012, 0.0010, 0.085)
    return s


def bench(w):
    from .motion import rotation

    s = w.stage("bench")
    for j, y in enumerate((-0.133, 0.130)):
        w.box(s, f"/Asset/Support{j}", (0.090, 0.016, 0.010), (0.345, y, C.STAGE_Z - C.TUBE_R - 0.005), EDGE)
        w.box(s, f"/Asset/SupportPost{j}", (0.022, 0.014, C.STAGE_Z - 0.013),
              (0.345, y, (C.STAGE_Z - 0.013) / 2), HOUSING)
    for i, (x, y) in enumerate(C.RECEIVERS):
        path = f"/Asset/Receiver{i}"
        w.cylinder(s, path + "Bottom", 0.0115, 0.003, (x, y, 0.0015), HOUSING)
        for j in range(16):
            angle = math.tau * j / 16
            w.box(s, path + f"Wall{j}", (0.001, 0.0044, 0.030),
                  (x + 0.011 * math.cos(angle), y + 0.011 * math.sin(angle), 0.0165),
                  (0.65, 0.74, 0.76), rotation=rotation("Z", angle), collide=False)
        w.cylinder(s, f"/Asset/Collected{i}", 0.010, 0.0001,
                   (x, y, 0.004), C.COLORS[i], collide=False).MakeInvisible()
    return s


def room(w):
    s = w.stage("workcell")
    w.box(s, "/Asset/Worktop", (1.20, 1.22, 0.065),
          (0.365, 0, C.TABLE - 0.0325), (0.57, 0.48, 0.33))
    for i, x in enumerate((-.19, .90)):
        for j, y in enumerate((-.55, .55)):
            w.box(s, f"/Asset/Leg{i}{j}", (.060, .060, .6825),
                  (x, y, .34125), HOUSING)
    w.box(s, "/Asset/Floor", (8., 8., .02), (0, 0, -.01),
          (.32, .37, .39))
    return s

def create_assets(directory):
    w = Writer(directory)
    assets = [{"name": "workcell", "type": "base", "file": w.save(room(w)), "position": (0, 0, 0)},
              {"name": "head", "type": "articulation", "file": w.save(pump(w)), "position": (0, 0, 0)},
              {"name": "bench", "type": "base", "file": w.save(bench(w)), "position": (0, 0, C.TABLE)}]
    for i, name in enumerate(C.TUBE_NAMES):
        assets.append({"name": name, "type": "articulation", "file": w.save(tube(w, name, C.COLORS[i], C.STAGING[i])),
                       "position": (0, 0, 0)})
    w.inventory()
    return {"assets": assets, "settle_actions": 18}
