"""Release clearance from the full official palm, fingers and pads, at any aperture."""
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import scenekit
import meshclearance

LINKS = ('gripper', 'tip_left', 'tip_right')
_local_hulls = None
_pose_key = None
_world_hulls = None


def local_hulls():
    if _local_hulls is None:
        raise RuntimeError('Actual cooked hand geometry unavailable')
    return _local_hulls


def prepare(environment):
    """Read the official left hand's cooked convexes; both arms use the same USD.

    No collision settings or robot states are changed. Queries must all succeed;
    there is no fallback to the potentially different high-detail authoring mesh.
    """
    global _local_hulls, _pose_key, _world_hulls
    if _local_hulls is not None:
        return
    from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils, PhysicsSchemaTools, Gf
    from scipy.spatial import ConvexHull
    from omni.physx import get_physx_cooking_interface
    from omni.physx.bindings._physx import PhysxCollisionRepresentationResult
    from yam_arena.robot import asset_link_paths
    stage = environment.unwrapped.sim.stage
    paths = asset_link_paths()
    root = stage.GetPrimAtPath('/World/envs/env_0/LeftArm'+paths['gripper'])
    if not root:
        raise RuntimeError('Missing official gripper prim')
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    parts = {name: [] for name in LINKS}
    results = []
    count = 0
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        body = prim.GetParent()
        while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body or body.GetName() not in LINKS:
            raise RuntimeError('Unexpected hand collider owner')
        name = body.GetName()
        transform = cache.ComputeRelativeTransform(prim, body)[0]
        count += 1
        def receive(result, convexes, transform=transform, name=name):
            results.append((result, len(convexes)))
            if result != PhysxCollisionRepresentationResult.RESULT_VALID:
                return
            for convex in convexes:
                points = np.array([list(transform.Transform(Gf.Vec3d(*map(float, p))))
                                   for p in convex.vertices])
                if len(points) < 4 or not np.isfinite(points).all():
                    raise RuntimeError('Invalid cooked hand vertices')
                hull = ConvexHull(points)
                faces = hull.simplices.copy()
                tri = points[faces]
                normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
                reverse = np.einsum('ij,ij->i', normal, hull.equations[:, :3]) < 0
                faces[reverse] = faces[reverse, ::-1]
                parts[name].append(points[faces])
        get_physx_cooking_interface().request_convex_collision_representation(
            stage_id=stage_id,
            collision_prim_id=PhysicsSchemaTools.sdfPathToInt(str(prim.GetPath())),
            run_asynchronously=False, on_result=receive)
    if (not count or len(results) != count or any(not v for v in parts.values())
            or any(r != PhysxCollisionRepresentationResult.RESULT_VALID or n < 1 for r, n in results)):
        raise RuntimeError('Actual cooked hand query failed; refusing to score')
    _local_hulls = {name: tuple(v) for name, v in parts.items()}
    _pose_key = None
    _world_hulls = None
    print('ACTUAL_COOKED_HAND', 'colliders', count,
          'components', {n: len(v) for n, v in parts.items()},
          'max_vertices', max(len(np.unique(t.reshape(-1, 3), axis=0)) for v in parts.values() for t in v), flush=True)
    import os
    dump = os.environ.get('TASK_HAND_DUMP')
    if dump:
        arrays = [t for v in parts.values() for t in v]
        Path(dump).parent.mkdir(parents=True, exist_ok=True)
        np.savez(dump, triangles=np.concatenate(arrays), counts=np.array([len(t) for t in arrays]),
                 links=np.array([n for n, v in parts.items() for _ in v]))


def _hand_surfaces():
    """Six full link surfaces in world coordinates; aperture never determines score."""
    global _pose_key, _world_hulls
    from yam_arena.urdf import quat_xyzw_to_matrix
    base = scenekit.env().unwrapped
    origin = scenekit._np(base.scene.env_origins)[0]
    poses = []
    for side in ('left', 'right'):
        robot = base.scene[f'{side}_robot']
        body_poses = scenekit._np(robot.data.body_link_pose_w)[0]
        for name in LINKS:
            poses.append((name, body_poses[robot.body_names.index(name)].copy()))
    if not np.isfinite(origin).all() or any(not np.isfinite(pose).all() for _, pose in poses):
        raise RuntimeError('Nonfinite hand pose')
    key = origin.tobytes()+b''.join(pose.tobytes() for _, pose in poses)
    if key != _pose_key:
        hulls = local_hulls()
        _world_hulls = [tuple(t@quat_xyzw_to_matrix(pose[3:7]).T+pose[:3]-origin
                              for t in hulls[name]) for name, pose in poses]
        _pose_key = key
    return _world_hulls


def apertures():
    """Read-only diagnostic; closed hands away from objects are valid."""
    return [float(-np.sum(scenekit.arm_joints(scenekit.env(), side, ['joint7', 'joint8'])))
            for side in ('left', 'right')]


def _clear(hands, surfaces, clearance=.03, component_points=None):
    """Check full convex hand components against complete object boundaries/solids."""
    if not hands or any(not components for components in hands):
        return False
    if component_points is None:
        component_points = [None]*len(surfaces)
    if len(component_points) != len(surfaces):
        raise ValueError('Missing object component representatives')
    for surface, points in zip(surfaces, component_points):
        surface = np.asarray(surface, float).reshape(-1, 3, 3)
        if not len(surface) or not np.isfinite(surface).all():
            return False
        low, high = surface.min((0, 1)), surface.max((0, 1))
        tree = None
        for components in hands:
            nearby = []
            for hull in components:
                hull = np.asarray(hull, float)
                if not len(hull) or not np.isfinite(hull).all():
                    return False
                gap = np.maximum(np.maximum(low-hull.max((0, 1)), hull.min((0, 1))-high), 0.)
                if np.linalg.norm(gap) < clearance:
                    nearby.append(hull)
            if nearby:
                triangles = np.concatenate(nearby)
                # A nearby pair of real boundary vertices proves insufficient
                # clearance. This cheap rejection never substitutes for full
                # boundary/solid testing when it does not find such a pair.
                if tree is None:
                    tree = cKDTree(surface.reshape(-1, 3))
                distances, _ = tree.query(triangles.reshape(-1, 3), k=1)
                if np.any(distances < clearance-1e-12):
                    return False
                if not meshclearance.surfaces_separated(
                        triangles, surface, clearance,
                        [h[0, 0] for h in nearby], points):
                    return False
    return True
