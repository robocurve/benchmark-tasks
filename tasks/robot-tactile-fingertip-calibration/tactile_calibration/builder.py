"""Procedural free sensor, compliant optical window, and sphere indenter."""

from . import config as C
from .usd_scene import Writer

CARBON = (0.13, 0.18, 0.23)
GEL = (0.43, 0.70, 0.56)
ORANGE = (0.90, 0.29, 0.055)
STEEL = (0.70, 0.76, 0.82)
RACK = (0.17, 0.28, 0.33)


def sensor(w):
    stage = w.stage("sensor")
    body = w.body(stage, "/Asset/Housing", 0.072, articulation=True, ccd=True)
    pad = w.body(stage, "/Asset/Pad", 0.0044, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, body).CreateEnabledSelfCollisionsAttr(False)
    w.box(stage, body + "/NylonShell", (0.042, 0.036, 0.037), (0, 0, 0.0185), CARBON)
    w.box(stage, body + "/UsbBay", (0.016, 0.009, 0.009), (-0.008, 0.018, 0.009), STEEL, collide=False)
    for sign in (-1, 1):
        w.box(stage, body + f"/EndLip{sign > 0}", (0.006, 0.036, 0.003), (sign * 0.0185, 0, 0.0385), CARBON)
        w.box(stage, body + f"/SideLip{sign > 0}", (0.031, 0.003, 0.003), (0, sign * 0.0165, 0.0385), CARBON)
    w.box(stage, pad + "/ReflectiveGel", (0.026, 0.030, 0.004), (0, 0, C.PAD_Z - 0.002), GEL)
    for i, (x, y) in enumerate(C.REGIONS):
        w.cylinder(stage, pad + f"/Region{i}", 0.0012, 0.00010, (x, y, C.PAD_Z + 0.00006), STEEL, collide=False)
    for i, x in enumerate((-0.008, 0.0, 0.008)):
        w.cylinder(stage, body + f"/CaptureLamp{i}", 0.0017, 0.0004,
                   (x, -0.0165, 0.0402), CARBON, collide=False)
    joint = w.Phy.PrismaticJoint.Define(stage, "/Asset/Deflection")
    joint.CreateBody0Rel().SetTargets([body])
    joint.CreateBody1Rel().SetTargets([pad])
    joint.CreateAxisAttr("Z")
    joint.CreateLowerLimitAttr(-0.003)
    joint.CreateUpperLimitAttr(0.0002)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(C.SPRING)
    drive.CreateDampingAttr(C.DAMPING)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateTargetVelocityAttr(0.0)
    drive.CreateMaxForceAttr(9.0)
    return stage


def tool(w):
    stage = w.stage("probe")
    body = w.body(stage, "/Asset", 0.048, ccd=True)
    w.box(stage, body + "/Grip", (0.020, 0.021, 0.032), (0, 0, C.TOOL_GRIP), ORANGE)
    w.cylinder(stage, body + "/StainlessStem", 0.0030, 0.064, (0, 0, 0.039), STEEL)
    sphere = w.Geom.Sphere.Define(stage, body + "/MasterBall")
    sphere.CreateRadiusAttr(C.BALL_R)
    sphere.AddTranslateOp().Set(w.Gf.Vec3d(0, 0, C.BALL_Z))
    w._finish(stage, sphere, STEEL, True)
    return stage


def rack(w):
    stage = w.stage("holders")
    for sign in (-1, 1):
        x, y, _ = C.SENSOR_START
        w.box(stage, f"/Asset/Rest{sign > 0}", (0.010, 0.028, 0.100), (x + sign * 0.014, y, 0.050), RACK)
    x, y, _ = C.TOOL_START
    w.cylinder(stage, "/Asset/ProbeBase", 0.025, 0.008, (x, y, 0.004), RACK)
    # Three low pegs retain the tip of the upright free tool before pickup.
    import math
    for i in range(3):
        a = math.tau * i / 3
        w.cylinder(stage, f"/Asset/ProbeGuide{i}", 0.0025, 0.014, (x + 0.0068 * math.cos(a), y + 0.0068 * math.sin(a), 0.015), RACK)
    return stage


def create_assets(directory):
    writer = Writer(directory)
    return {
        "assets": [
            {"name": "sensor", "type": "articulation", "file": writer.save(sensor(writer)), "position": C.SENSOR_START, "quaternion": (0, 0, 0.3826834324, 0.9238795325)},
            {"name": "probe", "type": "rigid", "file": writer.save(tool(writer)), "position": C.TOOL_START},
            {"name": "holders", "type": "base", "file": writer.save(rack(writer)), "position": (0, 0, C.TABLE)},
        ],
        "settle_actions": 24,
    }
