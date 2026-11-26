"""Fill the bag once and save its starting state; run once, never during an evaluation.

    OMNI_KIT_ACCEPT_EULA=YES python prepare_bag.py [--out DIR]

As a person fills a bag held open, the film (authored exactly as task.py authors it, by ``bagkit.author_film``) is
held open by its cut edge, with its two tabs held where they start. A thin rigid sleeve stands inside it, and the
grains (the radii from the previous ``mix_start.npz``, 0.80 kg) drop into the sleeve from a loose stack and settle.
The sleeve is drawn up and out, the mix spreads against the film, the cut edge is let go, and the film slumps onto
the mix. The settled film (``bag_start.npz``, in the bag frame) and the grains still inside it (``mix_start.npz``, in
the task's world frame) are saved; grains that ended outside the film are left out (the printout says how many).
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

os_env = __import__("os").environ
os_env.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")  # as yam_arena does for every evaluation

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402
from isaaclab.physics import PhysicsCfg  # noqa: E402

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, default=HERE)
add_launcher_args(ap)
args = ap.parse_args()
args.headless = True
args.physics = "isaacsim_physx"
args.require_kit = True
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402


def mat2quat(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return w, x, y, z


with launch_simulation(PhysicsCfg(), args) as physics_cfg:
    import isaaclab.sim as sim_utils
    import torch
    import warp as wp
    from isaaclab_physx.physics import PhysxManager
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    import bagkit
    import bagplan as p
    import clothbag as cb
    import soilkit

    DT = 1 / 240
    LIFT = np.array([0.0, 0.0, p.REST])  # the bag frame here: its floor REST above the ground at z = 0
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device="cuda:0", physics=physics_cfg))
    stage = sim.stage
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    pts, faces, info = cb.mesh()

    def body(path, pos, boxes, friction=0.5, kinematic=True):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim()).CreateKinematicEnabledAttr(kinematic)
        UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(0.05)
        mat = UsdShade.Material.Define(stage, path + "/mat")
        pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pm.CreateStaticFrictionAttr(friction)
        pm.CreateDynamicFrictionAttr(friction)
        for i, (c, s, q) in enumerate(boxes):
            g = UsdGeom.Cube.Define(stage, f"{path}/s{i}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            if q is not None:
                g.AddOrientOp().Set(Gf.Quatf(*q))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            pc = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            pc.CreateContactOffsetAttr(0.002)
            pc.CreateRestOffsetAttr(0.0)
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        return path

    # the tabs where they start, held there (kinematic); the film attaches to them as in the task
    tab_paths = []
    for tab in bagkit.TAB_BODIES:
        c, size, yaw = cb.tab_box(tab.name, pts, info)
        q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        tab_paths.append(body(f"/World/{tab.name}", c + LIFT, [((0, 0, 0), size, q)], friction=1.0))
    film_path = "/World/bag_film"
    bagkit.author_film(stage, film_path, pts + LIFT, pts + LIFT, tab_paths, looks=False)
    bagkit.set_deformable_contacts(stage)

    # the cut edge held open by a hand (a kinematic body, attached to the edge's vertices)
    rim = cb.mouth_rows(info)
    holder = body("/World/holder", np.array([0, 0, cb.HEIGHT]) + LIFT, [((0, 0, 0.05), (0.01, 0.01, 0.01), None)])
    att = stage.DefinePrim("/World/hold_edge", "OmniPhysicsVtxXformAttachment")
    att.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(film_path)])
    att.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(holder)])
    att.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set([int(v) for v in rim])
    att.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray).Set(
        [Gf.Vec3f(*map(float, q)) for q in pts[rim] - [0, 0, cb.HEIGHT]])
    att.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool).Set(True)

    # the sleeve: thin plates around the bag's outline, 6 mm inside it, standing on the floor, open at the top
    ring, _ = cb._outline(48)
    boxes = []
    for i in range(len(ring)):
        p0, p1 = ring[i], ring[(i + 1) % len(ring)]
        mid = (p0 + p1) / 2
        mid = mid - mid / np.linalg.norm(mid) * 0.006
        ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        boxes.append(((mid[0], mid[1], 0.26), (np.linalg.norm(p1 - p0) + 0.002, 0.002, 0.50),
                      (math.cos(ang / 2), 0, 0, math.sin(ang / 2))))
    sleeve = body("/World/sleeve", LIFT, boxes, friction=0.05)

    # the grains: a loose stack inside the sleeve
    prev = np.load(HERE / "mix_start.npz")
    radii = prev["radius"].astype(float)
    rng = np.random.default_rng(0)
    a_, b_ = cb.DEPTH / 2 - 0.013, cb.WIDTH / 2 - 0.013
    sp = 0.0125
    layer = [(x, y) for x in np.arange(-a_, a_ + 1e-9, sp) for y in np.arange(-b_, b_ + 1e-9, sp)
             if (abs(x) / a_) ** cb.SHAPE_N + (abs(y) / b_) ** cb.SHAPE_N <= 1]
    stack = []
    z = 0.012
    while len(stack) < len(radii):
        stack += [(x + rng.uniform(-1e-3, 1e-3), y + rng.uniform(-1e-3, 1e-3), z) for x, y in layer]
        z += sp
    gpos = np.array(stack[:len(radii)]) + LIFT
    quats = rng.normal(size=(len(radii), 4))
    quats /= np.linalg.norm(quats, axis=1, keepdims=True)
    mix = soilkit.Soil("prep", host="", low=(0, 0, 0), high=(0, 0, 0), grain_shape="icosahedron")
    v_ico, f_ico = soilkit._icosahedron()
    masses = p.GRAIN_DENSITY * soilkit.grain_volume(radii, "icosahedron")
    UsdGeom.Scope.Define(stage, "/World/mix")
    gm = UsdShade.Material.Define(stage, "/World/mix/mat")
    gpm = UsdPhysics.MaterialAPI.Apply(gm.GetPrim())
    gpm.CreateStaticFrictionAttr(p.MIX_FRICTION[0])
    gpm.CreateDynamicFrictionAttr(p.MIX_FRICTION[1])
    gpm.CreateRestitutionAttr(0.0)
    lay = stage.GetEditTarget().GetLayer()
    with Sdf.ChangeBlock():
        for i, (q, r) in enumerate(zip(gpos, radii)):
            ps = Sdf.CreatePrimInLayer(lay, f"/World/mix/g{i:05d}")
            ps.specifier = Sdf.SpecifierDef
            ps.typeName = "Mesh"
            ps.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(
                ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI",
                 "PhysxCollisionAPI", "MaterialBindingAPI", "PhysicsMeshCollisionAPI"]))

            def attr(n, t, v, ps=ps):
                Sdf.AttributeSpec(ps, n, t).default = v

            attr("points", Sdf.ValueTypeNames.Point3fArray, [Gf.Vec3f(*map(float, v)) for v in v_ico])
            attr("faceVertexCounts", Sdf.ValueTypeNames.IntArray, [3] * len(f_ico))
            attr("faceVertexIndices", Sdf.ValueTypeNames.IntArray, [v for f in f_ico for v in f])
            attr("physics:approximation", Sdf.ValueTypeNames.Token, "convexHull")
            attr("xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*map(float, q)))
            qx, qy, qz, qw = map(float, quats[i])
            attr("xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(qw, qx, qy, qz))
            attr("xformOp:scale", Sdf.ValueTypeNames.Float3, Gf.Vec3f(float(r), float(r), float(r)))
            attr("xformOpOrder", Sdf.ValueTypeNames.TokenArray, ["xformOp:translate", "xformOp:orient", "xformOp:scale"])
            attr("physics:mass", Sdf.ValueTypeNames.Float, float(masses[i]))
            attr("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, 0.5)
            attr("physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float, float(mix.linear_damping))
            attr("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float, 0.3)
            attr("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, int(mix.solver_iterations))
            attr("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1)
            attr("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, 0.0015)
            attr("physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.0)
            rel = Sdf.RelationshipSpec(ps, "material:binding:physics", False)
            rel.targetPathList.explicitItems = [Sdf.Path("/World/mix/mat")]

    sim.reset()
    view = PhysxManager.get_physics_sim_view()
    film = view.create_surface_deformable_body_view(film_path)
    grains = view.create_rigid_body_view("/World/mix/g*")
    sleeve_v = view.create_rigid_body_view(sleeve)
    one = wp.array(np.zeros(1, np.int32), dtype=wp.int32, device="cuda:0")

    def nodes():
        return wp.to_torch(film.get_simulation_nodal_positions()).cpu().numpy()[0].astype(float)

    def grain_tf():
        return wp.to_torch(grains.get_transforms()).cpu().numpy().astype(float)

    def report(tag):
        n, g = nodes(), grain_tf()[:, :3]
        ins = cb.inside(g, n, faces, rim)
        rim_z = n[rim, 2]
        print(json.dumps({"phase": tag, "t": round(t_sim, 2), "inside": int(ins.sum()), "outside": int((~ins).sum()),
                          "mix_top": round(float(np.percentile(g[ins, 2], 97)), 4) if ins.any() else None,
                          "edge_z": [round(float(rim_z.min()), 4), round(float(rim_z.max()), 4)],
                          "film_z": [round(float(n[:, 2].min()), 4), round(float(n[:, 2].max()), 4)],
                          "wall_s": round(time.time() - t0, 1)}), flush=True)

    t_sim, t0 = 0.0, time.time()

    def run(seconds, sleeve_z=None, tag="run"):
        global t_sim
        k = int(round(seconds / DT))
        for i in range(k):
            if sleeve_z is not None:
                z = sleeve_z((i + 1) / k)
                tf = torch.tensor([[0, 0, z + p.REST, 0, 0, 0, 1]], dtype=torch.float32, device="cuda:0")
                sleeve_v.set_transforms(wp.from_torch(tf), one)
            sim.step(render=False)
            t_sim += DT
        report(tag)

    run(2.0, tag="settled in the sleeve")
    run(3.0, lambda f: 0.55 * f, tag="sleeve drawn out")
    run(0.1, lambda f: 2.0, tag="sleeve away")
    att.GetAttribute("omniphysics:attachmentEnabled").Set(False)
    run(1.0, tag="edge let go")
    run(2.0, tag="settled")
    n, tf = nodes(), grain_tf()
    ins = cb.inside(tf[:, :3], n, faces, rim)
    speed = np.linalg.norm(wp.to_torch(grains.get_velocities()).cpu().numpy()[:, :3], axis=1)
    kg = float(masses[ins].sum())
    np.savez(args.out / "bag_start.npz", nodes=n - LIFT)
    np.savez(args.out / "mix_start.npz", pos=tf[ins, :3] - LIFT + p.BAG_AT, radius=radii[ins], quat=tf[ins, 3:])
    print(json.dumps({"saved": str(args.out), "grains": int(ins.sum()), "left_out": int((~ins).sum()),
                      "mix_kg": round(kg, 4), "grain_speed_max": round(float(speed[ins].max()), 4),
                      "mix_top_mm": round(float(np.percentile(tf[ins, 2] - p.REST, 97)) * 1000, 1)}), flush=True)
