"""Spawning a surface-deformable (cloth) object from our own USD mesh with usable collision offsets.

Isaac Lab's USD spawner writes `collision_props` onto the root prim before it creates the `sim_mesh` child, but the
simulation mesh is the collider. Left alone, it keeps PhysX's automatic offsets (about 2 cm for a bag-sized mesh):
the cloth then floats 2 cm above every surface, items float 2 cm above its floor, and nearby cloth faces at seams
push each other apart until the bag crumples. `spawn_cloth_from_usd` spawns as usual, then writes the offsets onto
every `sim_mesh`.

With `sew` set, chosen vertices of the cloth are pinned to rigid bodies (the stiff tote's straps to its rim and arch handles).

The simulated cloth is a surface. With `visual_usd_path` set, the mesh the cameras see is replaced after spawning by
the mesh in that file (the cloth drawn with thickness); PhysX carries it along with the simulated surface through
the bind pose that Isaac Lab sets up between the two meshes.
"""

from __future__ import annotations

from isaaclab.sim.spawners.from_files.from_files import spawn_from_usd
from isaaclab.utils import configclass

import isaaclab.sim as sim_utils


def _set_collision_offsets(prim, rest_offset: float, contact_offset: float) -> None:
    from pxr import Sdf

    if "PhysxCollisionAPI" not in prim.GetAppliedSchemas():
        prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest_offset)
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact_offset)


def _set_visual_mesh(stage, root_path: str, visual_usd_path: str) -> None:
    """Replace the visual mesh under `root_path` (the Mesh that is not `sim_mesh`) by the first Mesh in the file."""
    from pxr import Usd, UsdGeom

    src_stage = Usd.Stage.Open(visual_usd_path)  # keep a reference: its prims expire with the stage
    src = UsdGeom.Mesh(next(p for p in src_stage.Traverse() if p.IsA(UsdGeom.Mesh)))
    root = stage.GetPrimAtPath(root_path)
    vis = [p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Mesh) and p.GetName() != "sim_mesh"]
    assert len(vis) == 1, f"expected one visual mesh under {root_path}, found {[str(p.GetPath()) for p in vis]}"
    mesh = UsdGeom.Mesh(vis[0])
    points = src.GetPointsAttr().Get()
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexIndicesAttr().Set(src.GetFaceVertexIndicesAttr().Get())
    mesh.GetFaceVertexCountsAttr().Set(src.GetFaceVertexCountsAttr().Get())
    pose = vis[0].GetAttribute("deformablePose:default:omniphysics:points")
    if pose.IsValid():
        pose.Set(points)
    for name in ("normals", "primvars:normals"):
        if vis[0].HasAttribute(name):
            vis[0].RemoveProperty(name)


def _sew_to_body(stage, root_path: str, body_path: str, vertices, positions, name: str = "sewn") -> None:
    """Pin the cloth's simulation-mesh vertices `vertices` to the rigid body at `body_path`, at `positions` in the
    body's frame (an OmniPhysics vertex-to-xform attachment, infinitely stiff)."""
    from pxr import Sdf, Vt

    prim = stage.DefinePrim(f"{root_path}/{name}", "OmniPhysicsVtxXformAttachment")
    prim.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(f"{root_path}/sim_mesh")])
    prim.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(body_path)])
    prim.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set(Vt.IntArray([int(i) for i in vertices]))
    prim.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray).Set(
        Vt.Vec3fArray([tuple(float(c) for c in p) for p in positions]))
    prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool).Set(True)


def spawn_cloth_from_usd(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Spawn like `spawn_from_usd`, then set the cloth's simulation-mesh collision offsets and, if given, its visual
    mesh and its sewing to rigid bodies (`sew`)."""
    prim = spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    stage = prim.GetStage()
    for path in sim_utils.find_matching_prim_paths(f"{prim_path}/sim_mesh", stage=stage):
        _set_collision_offsets(stage.GetPrimAtPath(path), cfg.cloth_rest_offset, cfg.cloth_contact_offset)
    if cfg.visual_usd_path:
        for path in sim_utils.find_matching_prim_paths(prim_path, stage=stage):
            _set_visual_mesh(stage, path, cfg.visual_usd_path)
    for k, (body, vertices, positions) in enumerate(cfg.sew):
        for path in sim_utils.find_matching_prim_paths(prim_path, stage=stage):
            _sew_to_body(stage, path, path.rsplit("/", 1)[0] + "/" + body, vertices, positions, f"sewn_{k}")
    return prim


@configclass
class ClothUsdFileCfg(sim_utils.UsdFileCfg):
    """`UsdFileCfg` for a cloth mesh, with collision offsets applied to its simulation mesh."""

    func = spawn_cloth_from_usd
    cloth_rest_offset: float = 0.001
    cloth_contact_offset: float = 0.004
    visual_usd_path: str | None = None
    # Sewing to rigid bodies: (body prim name beside the cloth's prim, simulation-mesh vertex indices, their positions
    # in the body's frame), one entry per body.
    sew: tuple = ()
