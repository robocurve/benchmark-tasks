"""Collision-body bounds transformed by live physics poses, not USD animation."""
import itertools
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

def numpy(value):
    value = value.torch if hasattr(value, 'torch') else value
    return value.detach().cpu().numpy()

def hand_geometry():
    """Conservative link-local bounds of every collision hull on the three hand bodies.

    Nested child rigid bodies are excluded from the parent's bounds. The source
    USD is only used for immutable local shapes; runtime transforms come from
    the articulation's live body_link_pose_w data.
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    from yam_arena import require_usd
    stage = Usd.Stage.Open(str(require_usd()))
    prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
    wanted = {p.GetName() for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    links = {p.GetName(): p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName() in wanted}
    if set(links) != wanted:
        raise RuntimeError('Missing converted robot collision bodies')
    xforms = UsdGeom.XformCache()
    bboxes = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy', 'guide'], False, True)
    found = {name: [] for name in wanted}
    for prim in prims:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        parent = prim
        while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):
            parent = parent.GetParent()
        if not parent or parent.GetName() not in wanted or parent != links[parent.GetName()]:
            continue
        bounds = bboxes.ComputeWorldBound(prim).ComputeAlignedBox()
        if bounds.IsEmpty():
            raise RuntimeError('Empty hand collider bounds')
        inverse = xforms.GetLocalToWorldTransform(parent).GetInverse()
        low, high = np.array(bounds.GetMin()), np.array(bounds.GetMax())
        points = [inverse.Transform(Gf.Vec3d(*map(float, xyz)))
                  for xyz in itertools.product(*zip(low, high))]
        found[parent.GetName()].extend(points)
    out = {}
    for name, points in found.items():
        if not points:
            raise RuntimeError('No collision hulls for hand body ' + name)
        points = np.array(points)
        low, high = points.min(0), points.max(0)
        out[name] = np.array(list(itertools.product(*zip(low, high))))
    return out

def hand_bounds(base, geometry):
    out = {}
    for side, key in [('left', 'left_robot'), ('right', 'right_robot')]:
        robot = base.scene[key]
        poses = numpy(robot.data.body_link_pose_w)[0]
        for name, points in geometry.items():
            index = robot.body_names.index(name)
            pose = poses[index]
            rotation = quat_xyzw_to_matrix(pose[3:])
            world = points @ rotation.T + pose[:3]
            out[side + '_' + name] = (world.min(0), world.max(0))
    return out
