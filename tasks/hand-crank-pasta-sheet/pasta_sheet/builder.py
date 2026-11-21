"""Native colliders around an unpowered pasta roller and a loose dough sheet."""

import math
from pathlib import Path

import numpy as np

from . import config as C
from .usd_scene import Writer

STEEL = (0.58, 0.62, 0.66)
DARK = (0.17, 0.21, 0.24)
WOOD = (0.60, 0.43, 0.26)
DOUGH = (0.91, 0.65, 0.25)


def revolute(w, stage, name, base, shaft, origin):
    joint = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([base])
    joint.CreateBody1Rel().SetTargets([shaft])
    joint.CreateAxisAttr("Y")
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateCollisionEnabledAttr(False)
    return joint


def machine(w):
    s = w.stage("machine")
    base = w.body(s, "/Asset/Housing", 2.3, fixed=True)
    root = s.GetDefaultPrim()
    w.Phy.ArticulationRootAPI.Apply(root)
    solver = w.Px.PhysxArticulationAPI.Apply(root)
    solver.CreateEnabledSelfCollisionsAttr(False)
    solver.CreateSolverPositionIterationCountAttr(32)
    solver.CreateSolverVelocityIterationCountAttr(8)
    w.box(
        s, base + "/Foot", (0.188, 0.202, 0.024), (*C.NIP[:2], C.TABLE + 0.012), STEEL
    )
    for k, sign in enumerate((-1, 1)):
        y = C.NIP[1] + sign * 0.091
        w.box(
            s,
            base + f"/Bearing{k}",
            (0.116, 0.027, 0.149),
            (C.NIP[0], y, C.NIP[2] - 0.068),
            STEEL,
        )
        w.cylinder(
            s,
            base + f"/Dial{k}",
            0.023,
            0.016,
            (C.NIP[0], y + sign * 0.018, C.NIP[2] - 0.051),
            DARK,
            "Y",
            False,
        )
    for name, origin in (("Driven", C.ROLL_NEAR), ("Opposed", C.ROLL_FAR)):
        shaft = w.body(s, "/Asset/" + name + "Shaft", 0.19, origin, ccd=True)
        w.Px.PhysxContactReportAPI.Apply(s.GetPrimAtPath(shaft)).CreateThresholdAttr(
            0.0
        )
        w.cylinder(
            s,
            shaft + "/SteelRoll",
            C.ROLL_RADIUS,
            C.ROLL_WIDTH,
            (0.0, 0.0, 0.0),
            STEEL,
            "Y",
        )
        if name == "Driven":
            w.cylinder(s, shaft + "/Axle", 0.006, 0.052, (0.0, -0.101, 0.0), STEEL, "Y")
            a = C.CRANK_START
            radial = np.array([math.cos(a), 0.0, -math.sin(a)])
            spin = np.array(
                [
                    [radial[0], 0.0, -radial[2]],
                    [0.0, 1.0, 0.0],
                    [radial[2], 0.0, radial[0]],
                ]
            )
            w.box(
                s,
                shaft + "/CrankArm",
                (0.080, 0.009, 0.013),
                0.5 * C.CRANK_RADIUS * radial + [0.0, -0.127, 0.0],
                STEEL,
                rotation=spin,
            )
            grip_offset = C.CRANK_RADIUS * radial + [0.0, C.CRANK_SPAN, 0.0]
            sleeve = w.body(
                s, "/Asset/CrankSleeve", 0.015, origin + grip_offset, ccd=True
            )
            w.cylinder(
                s,
                sleeve + "/PlasticGrip",
                C.CRANK_KNOB_RADIUS,
                0.063,
                (0.0, 0.0, 0.0),
                DARK,
                "Y",
            )
            revolute(w, s, "CrankBearing", shaft, sleeve, grip_offset)
        bearing = revolute(w, s, name, base, shaft, origin)
        drive(w, bearing, "angular", 0.0, 0.0021 if name == "Driven" else 0.0011, 0.35)
        w.Px.PhysxJointAPI.Apply(bearing.GetPrim()).CreateMaxJointVelocityAttr(330.0)
    return w.save(s)


def board(w):
    s = w.stage("board")
    p = "/Asset/Set"
    w.box(
        s,
        p + "/DoughBoard",
        (0.176, 0.170, 0.1655),
        (0.296, 0.038, C.TABLE + 0.08275),
        WOOD,
    )
    # An angled receiving board catches the sheet beneath the rolls and lets
    # its weight straighten the outfeed rather than pushing against a pile.
    catcher = p + "/CatchBoard"
    w.box(
        s,
        catcher,
        (C.CATCH_LENGTH, C.CATCH_WIDTH, 0.011),
        C.CATCH_TOP - 0.0055 * C.CATCH_FRAME[:, 2],
        (0.81, 0.82, 0.80),
        C.CATCH_FRAME,
    )
    finish = w.Shade.Material.Define(s, "/Asset/PolishedCatcher")
    contact = w.Phy.MaterialAPI.Apply(finish.GetPrim())
    contact.CreateStaticFrictionAttr(0.16)
    contact.CreateDynamicFrictionAttr(0.12)
    contact.CreateRestitutionAttr(0.0)
    w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(catcher)).Bind(
        finish, materialPurpose="physics"
    )
    lip = C.CATCH_TOP + C.CATCH_FRAME @ np.array(
        [0.5 * C.CATCH_LENGTH - 0.006, 0.0, 0.006]
    )
    w.box(
        s,
        p + "/LowerLip",
        (0.012, C.CATCH_WIDTH, 0.014),
        lip,
        (0.72, 0.74, 0.71),
        C.CATCH_FRAME,
    )
    for idx, station in enumerate((0.464, 0.598)):
        high = (
            C.CATCH_TOP[2] - (station - C.CATCH_TOP[0]) * math.tan(C.CATCH_TILT) - 0.011
        )
        w.box(
            s,
            p + f"/CatchSupport{idx}",
            (0.025, 0.128, high - C.TABLE),
            (station, C.NIP[1], 0.5 * (C.TABLE + high)),
            WOOD,
        )
    return w.save(s)


def drive(w, joint, mode, stiffness, damping, cap):
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), mode)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(cap)
    drive.CreateTargetPositionAttr(0.0)


def connect(w, stage, cls, name, base, child, axis, base_anchor, child_anchor, limits):
    joint = cls.Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([base])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*base_anchor))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(*child_anchor))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    return joint


def dough(w):
    stage = w.stage("dough")
    pitch, quarter = C.SECTION_PITCH, C.STOCK_THICKNESS / 4.0
    previous = None
    for index in range(C.SECTIONS):
        x = index * pitch
        root = w.body(
            stage,
            f"/Asset/Lower{index:02}",
            0.0036,
            (x, 0.0, 0.0),
            articulation=index == 0,
            ccd=True,
        )
        if index == 0:
            w.Px.PhysxArticulationAPI.Get(stage, root).CreateEnabledSelfCollisionsAttr(
                False
            )
        upper = w.body(
            stage, f"/Asset/Upper{index:02}", 0.0036, (x, 0.0, quarter), ccd=True
        )
        # Adjacent collision patches overlap to keep both faces in physical
        # contact as the sheet's material-coordinate spacing grows.
        backward = 0.5 * (pitch + (C.MAX_STRETCH if index else 0.0))
        forward = 0.5 * (pitch + (C.MAX_STRETCH if index + 1 < C.SECTIONS else 0.0))
        shape = (backward + forward, C.STOCK_WIDTH, 2.0 * quarter)
        cx = 0.5 * (forward - backward)
        for path, z in ((root, -quarter), (upper, 0.0)):
            w.box(stage, path + "/Dough", shape, (cx, 0.0, z), DOUGH)
            stage.GetPrimAtPath(path + "/Dough").GetAttribute("visibility").Set(
                "invisible"
            )
        compression = connect(
            w,
            stage,
            w.Phy.PrismaticJoint,
            f"Thickness{index:02}",
            root,
            upper,
            "Z",
            (0.0, 0.0, quarter),
            (0.0, 0.0, 0.0),
            (-0.00156, 0.0),
        )
        drive(w, compression, "linear", 4400.0, 1.4, 12.0)
        if previous is not None:
            pivot = w.body(
                stage, f"/Asset/Pivot{index:02}", 0.0012, (x - 0.5 * pitch, 0.0, 0.0)
            )
            stretch = connect(
                w,
                stage,
                w.Phy.PrismaticJoint,
                f"Stretch{index:02}",
                previous,
                pivot,
                "X",
                (0.5 * pitch, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (-0.00010, C.MAX_STRETCH + 0.0004),
            )
            drive(w, stretch, "linear", 850.0, 1.0, 4.0)
            hinge = connect(
                w,
                stage,
                w.Phy.RevoluteJoint,
                f"Bend{index:02}",
                pivot,
                root,
                "Y",
                (0.0, 0.0, 0.0),
                (-0.5 * pitch, 0.0, 0.0),
                (-74.0, 74.0),
            )
            drive(w, hinge, "angular", 0.00038, 0.00065, 0.012)
        previous = root
    return w.save(stage)


def create_assets(directory):
    w = Writer(Path(directory))
    return {
        "assets": [
            {"name": "machine", "type": "articulation", "file": machine(w)},
            {"name": "board", "type": "base", "file": board(w)},
            {
                "name": "dough",
                "type": "articulation",
                "file": dough(w),
                "position": C.STOCK
                - [0.5 * (C.SECTIONS - 1) * C.SECTION_PITCH, 0.0, 0.0],
            },
        ],
        "settle_actions": 10,
    }
