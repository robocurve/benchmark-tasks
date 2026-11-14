"""Original physical chain and headed-pin keeper. No runtime snaps or attachments."""

from pathlib import Path
import json
import math
import numpy as np


def create_assets(directory):
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade
    from chain_link.config import (
        DECK_TOP,
        NODES,
        PITCH,
        ROOT_CENTER,
        ROOT_YAW,
        CHAIN_POSITION,
    )

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    inventory = []

    def stage_at(name):
        stage = Usd.Stage.CreateNew(str(directory / (name + ".usda")))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, "/" + name.title())
        stage.SetDefaultPrim(root.GetPrim())
        mat = UsdShade.Material.Define(stage, str(root.GetPath()) + "/Material")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(0.65)
        api.CreateDynamicFrictionAttr(0.5)
        api.CreateRestitutionAttr(0)
        return stage, mat

    def body(stage, path, mass, pos=(0, 0, 0), yaw=0):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.AddOrientOp().Set(
            Gf.Quatf(math.cos(yaw / 2), Gf.Vec3f(0, 0, math.sin(yaw / 2)))
        )
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        m = UsdPhysics.MassAPI.Apply(x.GetPrim())
        m.CreateMassAttr(mass)
        inventory.append(
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                body=path,
                mass_kg=mass,
                initial_local_position_m=list(map(float, pos)),
                yaw_rad=yaw,
            )
        )

    def contact(mat, prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        p.CreateContactOffsetAttr(0.0002)
        p.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose="physics")

    def box(stage, mat, path, size, pos, color, yaw=0):
        x = UsdGeom.Cube.Define(stage, path)
        x.CreateSizeAttr(1)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.AddOrientOp().Set(
            Gf.Quatf(math.cos(yaw / 2), Gf.Vec3f(0, 0, math.sin(yaw / 2)))
        )
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        inventory.append(
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                solid=path,
                kind="box",
                display_color_rgb=list(color),
                size_m=list(map(float, size)),
                center_m=list(map(float, pos)),
                yaw_rad=yaw,
            )
        )

    def cylinder(stage, mat, path, radius, height, pos, color):
        x = UsdGeom.Cylinder.Define(stage, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(height)
        x.CreateAxisAttr("Z")
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        inventory.append(
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                solid=path,
                kind="cylinder",
                display_color_rgb=list(color),
                radius_m=radius,
                height_m=height,
                center_m=list(map(float, pos)),
                yaw_rad=0,
            )
        )

    def cone(stage, mat, path, radius, height, pos, color):
        x = UsdGeom.Cone.Define(stage, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(height)
        x.CreateAxisAttr("Z")
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        inventory.append(
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                solid=path,
                kind="cone",
                display_color_rgb=list(color),
                radius_m=radius,
                height_m=height,
                center_m=list(map(float, pos)),
                yaw_rad=0,
            )
        )

    grey, orange, blue, dark = (
        (0.38, 0.41, 0.45),
        (0.95, 0.48, 0.07),
        (0.08, 0.36, 0.85),
        (0.13, 0.16, 0.19),
    )
    chain_grey = (0.075, 0.09, 0.105)
    tooth_grey = (0.13, 0.15, 0.17)
    stage, mat = stage_at("chain")
    yaws = []
    for n, (a, b) in enumerate(zip(NODES[:-1], NODES[1:])):
        c, s = math.cos(ROOT_YAW), math.sin(ROOT_YAW)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        center = ((a + b) / 2 - ROOT_CENTER) @ rotation
        yaw = math.atan2(b[1] - a[1], b[0] - a[0]) - ROOT_YAW
        yaws.append(yaw)
        path = f"/Chain/Link{n:02}"
        body(stage, path, 0.035, center, yaw)
        if n == 0:
            # Explicit root body: otherwise PhysX selects a middle chain link,
            # making the configured spawn pose refer to the wrong body.
            UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(path))
            api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(path))
            api.CreateEnabledSelfCollisionsAttr(True)
            api.CreateSolverPositionIterationCountAttr(32)
            api.CreateSolverVelocityIterationCountAttr(8)
        layers = [(0.0, 0.008)] if n % 2 == 0 else [(-0.006, 0.004), (0.006, 0.004)]
        for layer, (z, height) in enumerate(layers):
            for end, x0 in enumerate((-PITCH / 2, PITCH / 2)):
                for j in range(20):
                    theta = j * math.tau / 20
                    box(
                        stage,
                        mat,
                        path + f"/Ring{layer}_{end}_{j}",
                        (0.007, 0.0075, height),
                        (x0 + 0.0195 * math.cos(theta), 0.0195 * math.sin(theta), z),
                        chain_grey,
                        theta,
                    )
            for side in (-1, 1):
                box(
                    stage,
                    mat,
                    path + f"/Rail{layer}_{side + 1}",
                    (PITCH, 0.006, height),
                    (0, side * 0.020, z),
                    chain_grey,
                )
        if n % 2:
            for j, x0 in enumerate((-PITCH / 2, PITCH / 2)):
                cylinder(
                    stage, mat, path + f"/HingePin{j}", 0.004, 0.020, (x0, 0, 0), dark
                )
        if n in (0, 10):
            # Taller central sections of the existing side rails provide enough
            # pad contact for a bare-link pinch, without separate grasp posts.
            # Keep the hinge and open end-hole geometry unchanged.
            for side in (-1, 1):
                box(
                    stage,
                    mat,
                    path + f"/RailThickening{side + 1}",
                    (0.030, 0.006, 0.012),
                    (0, side * 0.020, 0.010),
                    chain_grey,
                )
            # Raised shoulders support the keeper above a flat end link.
            # Their bore is identical to the original end hole.
            x0 = -PITCH / 2 if n == 0 else PITCH / 2
            for j in range(20):
                theta = j * math.tau / 20
                box(
                    stage,
                    mat,
                    path + f"/KeeperShoulder{j}",
                    (0.007, 0.0075, 0.005),
                    (x0 + 0.0195 * math.cos(theta), 0.0195 * math.sin(theta), 0.0065),
                    chain_grey,
                    theta,
                )
        if n:
            prev = f"/Chain/Link{n - 1:02}"
            j = UsdPhysics.RevoluteJoint.Define(stage, f"/Chain/Hinge{n:02}")
            j.CreateBody0Rel().SetTargets([prev])
            j.CreateBody1Rel().SetTargets([path])
            j.CreateAxisAttr("Z")
            j.CreateLocalPos0Attr(Gf.Vec3f(PITCH / 2, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(-PITCH / 2, 0, 0))
            for attr, angle in (
                (j.CreateLocalRot0Attr(), -yaws[n - 1]),
                (j.CreateLocalRot1Attr(), -yaw),
            ):
                attr.Set(
                    Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(0, 0, math.sin(angle / 2)))
                )
            j.CreateLowerLimitAttr(-150)
            j.CreateUpperLimitAttr(150)
            j.CreateCollisionEnabledAttr(False)
            PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(
                0.00005
            )
            inventory.append(
                dict(
                    asset="chain",
                    joint=f"Hinge{n:02}",
                    axis="Z",
                    limits_deg=[-150, 150],
                    parent=prev,
                    child=path,
                    initial_relative_angle_deg=0,
                )
            )
    stage.GetRootLayer().Save()

    stage, mat = stage_at("connector")
    body(stage, "/Connector/Body", 0.12)
    box(stage, mat, "/Connector/Body/Backing", (0.045, 0.140, 0.004), (0, 0, 0), orange)
    for n, y in enumerate((-PITCH / 2, PITCH / 2)):
        cylinder(
            stage, mat, f"/Connector/Body/Stem{n}", 0.004, 0.027, (0, y, 0.0155), grey
        )
        # Flat cylindrical shoulders permit a natural pin-head pinch grasp;
        # the short tapered lead retains the original insertion envelope.
        cylinder(
            stage, mat, f"/Connector/Body/Head{n}", 0.012, 0.006, (0, y, 0.032), orange
        )
        cone(
            stage, mat, f"/Connector/Body/Lead{n}", 0.012, 0.004, (0, y, 0.037), orange
        )
    stage.GetRootLayer().Save()

    stage, mat = stage_at("keeper")
    body(stage, "/Keeper/Body", 0.075)
    # Two side-entry forks admit the 8 mm stems below the 24 mm heads.
    # The narrow outer outline clears the raised bare end-link rails.
    xs = [-0.040, -0.026, 0.040]
    ys = [-0.064, -0.059, -0.041, 0.041, 0.059, 0.064]
    n = 0
    for xa, xb in zip(xs[:-1], xs[1:]):
        for ya, yb in zip(ys[:-1], ys[1:]):
            x, y = (xa + xb) / 2, (ya + yb) / 2
            empty = any(
                -0.026 < x and abs(y - center) < 0.009
                for center in (-PITCH / 2, PITCH / 2)
            )
            if not empty:
                box(
                    stage,
                    mat,
                    f"/Keeper/Body/Cell{n:02}",
                    (xb - xa, yb - ya, 0.008),
                    (x, y, 0),
                    blue,
                )
                n += 1
    box(stage, mat, "/Keeper/Body/Grip", (0.024, 0.030, 0.060), (0, 0, 0.030), blue)
    stage.GetRootLayer().Save()

    stage, mat = stage_at("fixtures")
    box(
        stage,
        mat,
        "/Fixtures/Deck",
        (0.60, 0.48, 0.012),
        (0.42, 0, DECK_TOP - 0.006),
        (0.55, 0.57, 0.60),
    )
    for i, x in enumerate((0.18, 0.60)):
        for j, y in enumerate((-0.19, 0.19)):
            box(
                stage,
                mat,
                f"/Fixtures/Post{i}{j}",
                (0.025, 0.025, DECK_TOP - 0.012 - 0.7475),
                (x, y, (DECK_TOP - 0.012 + 0.7475) / 2),
                dark,
            )
    # Visible base locating edges; the connector remains a free, liftable body.
    for n, y in enumerate((-0.079, 0.079)):
        box(
            stage,
            mat,
            f"/Fixtures/ConnectorEdge{n}",
            (0.055, 0.010, 0.004),
            (0.195, y, DECK_TOP + 0.002),
            dark,
        )
    # The front stays open for sliding the loose backing under the end plates.
    for n, x in enumerate((0.2225,)):
        box(
            stage,
            mat,
            f"/Fixtures/ConnectorSide{n}",
            (0.005, 0.140, 0.004),
            (x, 0, DECK_TOP + 0.002),
            dark,
        )
    # A fixed service former supports the loop during joining. Its low spokes
    # are recessed into the bed; positive teeth enter the open link windows.
    # The front service sector remains open for the two end links and keeper.
    cx, cy = CHAIN_POSITION[:2]
    cylinder(
        stage,
        mat,
        "/Fixtures/FormerHub",
        0.145,
        0.008,
        (cx, cy, DECK_TOP + 0.004),
        chain_grey,
    )
    cylinder(
        stage,
        mat,
        "/Fixtures/FormerBoss",
        0.025,
        0.014,
        (cx, cy, DECK_TOP + 0.007),
        dark,
    )
    cylinder(
        stage,
        mat,
        "/Fixtures/FormerCap",
        0.012,
        0.003,
        (cx, cy, DECK_TOP + 0.0155),
        chain_grey,
    )
    for n in range(1, 10):
        point = (NODES[n] + NODES[n + 1]) / 2 + CHAIN_POSITION
        delta = point[:2] - CHAIN_POSITION[:2]
        angle = math.atan2(delta[1], delta[0])
        radius = float(np.linalg.norm(delta))
        box(
            stage,
            mat,
            f"/Fixtures/FormerSpoke{n}",
            (radius + 0.012, 0.022, 0.004),
            (cx + delta[0] / 2, cy + delta[1] / 2, DECK_TOP),
            chain_grey,
            angle,
        )
        cylinder(
            stage,
            mat,
            f"/Fixtures/FormerTooth{n}",
            0.006,
            0.050,
            (point[0], point[1], DECK_TOP + 0.025),
            tooth_grey,
        )
        cone(
            stage,
            mat,
            f"/Fixtures/FormerToothLead{n}",
            0.006,
            0.003,
            (point[0], point[1], DECK_TOP + 0.0515),
            tooth_grey,
        )
    for n, angle in enumerate(np.deg2rad([45, 90, 135, 225, 270, 315])):
        cylinder(
            stage,
            mat,
            f"/Fixtures/FormerBolt{n}",
            0.0045,
            0.003,
            (
                cx + 0.116 * math.cos(angle),
                cy + 0.116 * math.sin(angle),
                DECK_TOP + 0.0095,
            ),
            dark,
        )
    stage.GetRootLayer().Save()
    (directory / "geometry-inventory.json").write_text(
        json.dumps(inventory, indent=2) + "\n"
    )
    return {
        n: directory / (n + ".usda")
        for n in ("chain", "connector", "keeper", "fixtures")
    }
