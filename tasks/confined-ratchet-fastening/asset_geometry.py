"""Reproducible source-mesh convex slab colliders and restricted-access walls.

Primary can copy ``add_handle_colliders`` into its asset builder after the live
trial ends. This development script writes only into this operations folder.
"""
from pathlib import Path
import json
import sys
import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def rendered_triangles(visual_path):
    source=Usd.Stage.Open(str(visual_path));cache=UsdGeom.XformCache();triangles=[]
    for prim in source.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not str(prim.GetPath()).startswith('/RatchetVisual/Handle/'):
            continue
        mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
        points=(np.c_[points,np.ones(len(points))]@np.asarray(cache.GetLocalToWorldTransform(prim)))[:,:3]
        indices=list(mesh.GetFaceVertexIndicesAttr().Get());start=0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face=indices[start:start+count];start+=count
            triangles.extend([points[[face[0],face[i],face[i+1]]] for i in range(1,count-1)])
    return np.asarray(triangles)


def slab_surface_points(triangles,lower,upper):
    """Vertices plus exact triangle-edge intersections with both X clip planes.

    A triangle clipped against a slab has only these vertices. Their convex
    hull therefore contains every clipped surface triangle in that slab.
    """
    tri=triangles[(triangles[:,:,0].max(axis=1)>=lower)&(triangles[:,:,0].min(axis=1)<=upper)]
    points=tri.reshape(-1,3)
    result=[points[(points[:,0]>=lower)&(points[:,0]<=upper)]]
    for first,second in [(0,1),(1,2),(2,0)]:
        a,b=tri[:,first],tri[:,second]
        delta=b[:,0]-a[:,0]
        for plane in [lower,upper]:
            mask=(np.abs(delta)>1e-14)&((a[:,0]-plane)*(b[:,0]-plane)<=0)
            if mask.any():
                fraction=(plane-a[mask,0])/delta[mask]
                result.append(a[mask]+fraction[:,None]*(b[mask]-a[mask]))
    return np.unique(np.vstack(result).round(10),axis=0)


def conservative_hull(points,max_planes=128,tolerance=.00008):
    """Outer supporting-plane approximation: contains every original point.

    At most128planes imply at most252vertices in a nondegenerate3Dpolytope,
    meeting PhysX's255vertexlimit without relying on undocumented cookerdecimation.
    """
    original=ConvexHull(points);equations=original.equations
    if len(original.vertices)<=240:
        return points[original.vertices],0.
    lower,upper=points.min(0),points.max(0)
    planes=[]
    for axis in range(3):
        normal=np.zeros(3);normal[axis]=1
        planes.append(np.r_[normal,-upper[axis]])
        planes.append(np.r_[-normal,lower[axis]])
    interior=points[original.vertices].mean(0)
    for _ in range(max_planes-6+1):
        outer=HalfspaceIntersection(np.asarray(planes),interior).intersections
        violations=outer@equations[:,:3].T+equations[:,3]
        index=np.unravel_index(np.argmax(violations),violations.shape)
        error=float(violations[index])
        if error<=tolerance or len(planes)==max_planes:
            break
        plane=equations[index[1]]
        if any(np.allclose(plane,p,atol=1e-12) for p in planes):
            raise RuntimeError('Repeated supportplane during approximation')
        planes.append(plane)
    if len(outer)>255:
        raise RuntimeError(f'Unexpected hullvertexcount {len(outer)}')
    if np.max(points@np.asarray(planes)[:,:3].T+np.asarray(planes)[:,3])>1e-8:
        raise RuntimeError('Approximation failed containment')
    return outer,error


def add_handle_colliders(stage,visual_path,root='/Asset',slab_width=.005):
    """Add hidden SI-unit convex hulls in the visual's canonical output frame.

    Retains all source render geometry, including switch and release button.
    Slabs meet at exact planes; no body-level pose writes or magic attachments.
    The housing's internal bore is intentionally filled: handle/drive internal
    collision is disabled by the real joint; exterior contacts remain solid.
    """
    triangles=rendered_triangles(visual_path)
    lo,hi=triangles[:,:,0].min(),triangles[:,:,0].max()
    edges=np.arange(np.floor(lo/slab_width)*slab_width,np.ceil(hi/slab_width)*slab_width+slab_width/2,slab_width)
    summaries=[];hulls=[]
    for i,(lower,upper) in enumerate(zip(edges[:-1],edges[1:])):
        points=slab_surface_points(triangles,lower,upper)
        if len(points)<4:continue
        vertices,plane_error=conservative_hull(points)
        final=ConvexHull(vertices)
        name=f'{root}/SourceContact/Slab{i:02}'
        mesh=UsdGeom.Mesh.Define(stage,name)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in vertices])
        mesh.CreateFaceVertexCountsAttr([3]*len(final.simplices))
        mesh.CreateFaceVertexIndicesAttr(final.simplices.reshape(-1).tolist())
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        summaries.append(dict(path=name,vertices=len(vertices),faces=len(final.simplices),
            lower_x_m=float(vertices[:,0].min()),upper_x_m=float(vertices[:,0].max()),
            bounds_min_m=vertices.min(0).tolist(),bounds_max_m=vertices.max(0).tolist()))
        summaries[-1]['max_supportplane_violation_m']=plane_error
        hulls.append((vertices,final.simplices,final.equations))
    return dict(method='5mm source-mesh X slabs with exact edge/plane intersections and convex hulls',
                slab_width_m=slab_width,hulls=summaries),hulls


def add_neck_walls(stage,root='/Asset'):
    """Short45mm-X-span walls keep the blue grip outside the enclosure.

    Root XY coincides with the bolt axis; root Z is the tabletop, not the
    ratchet pivot. Actual rectangles are46.587mm long after their15deg yaw.
    """
    xmin,xmax,intercept=-.080,-.035,.015
    angle=np.deg2rad(15);x=(xmin+xmax)/2
    y=intercept-x*np.tan(angle);length=(xmax-xmin)/np.cos(angle)
    for sign in [-1,1]:
        cube=UsdGeom.Cube.Define(stage,root+('/NeckWallPositive' if sign>0 else '/NeckWallNegative'))
        cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(x,sign*y,.081))
        cube.AddRotateZOp().Set(float(-sign*15))
        cube.AddScaleOp().Set(Gf.Vec3f(length,.009,.162))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.17,.29,.40)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    return dict(center_x_m=x,center_y_magnitude_m=y,center_z_m=.081,
                length_m=length,thickness_m=.009,height_m=.162,
                positive_y_rotation_deg=-15,negative_y_rotation_deg=15)
