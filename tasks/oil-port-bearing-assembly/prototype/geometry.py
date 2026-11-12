"""Original compound colliders. Apertures are absent material, not painted marks."""
import math
import numpy as np
from scipy.spatial import ConvexHull,QhullError
from functools import lru_cache
from . import layout as L

VISUAL_REVISION='silver-inner-bore-v1'

def material(stage,path,color,roughness=.6,metallic=None):
    from pxr import Gf,Sdf,UsdShade
    from yam_arena.station import linear_rgb
    m=UsdShade.Material.Define(stage,path)
    s=UsdShade.Shader.Define(stage,path+'/shader');s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness)
    if metallic is not None:s.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def cube(stage,path,pos,size,color):
    from pxr import Gf,UsdGeom,UsdPhysics,UsdShade
    c=UsdGeom.Cube.Define(stage,path);c.CreateSizeAttr(1.)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material(stage,path+'/look',color))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim());return c.GetPrim()

def hull(points):
    points=np.unique(np.round(points,13),axis=0)
    # Repeated clipping can create duplicate vertices differing by roundoff.
    # Merge within 0.1 nm, well below the 10 nm numerical gauge allowance.
    keep=[]
    for point in points:
        if not keep or np.min(np.linalg.norm(np.asarray(keep)-point,axis=1))>1e-10:
            keep.append(point)
    points=np.asarray(keep)
    if len(points)<4:return None
    try:h=ConvexHull(points)
    except QhullError:return None
    if h.volume<1e-15:return None
    return points[h.vertices]

def split(points,normal,distance):
    """Return convex material inside/outside n.p <= d, with shared cut vertices."""
    values=points@normal-distance
    if values.max()<=1e-11:return points,None
    if values.min()>=-1e-11:return None,points
    h=ConvexHull(points);edges={tuple(sorted((int(i),int(j)))) for f in h.simplices for i,j in ((f[0],f[1]),(f[1],f[2]),(f[2],f[0]))}
    cuts=[]
    for i,j in edges:
        if values[i]*values[j]<0:
            t=values[i]/(values[i]-values[j]);cuts.append(points[i]+t*(points[j]-points[i]))
    return hull(np.vstack([points[values<=1e-11],cuts])),hull(np.vstack([points[values>=-1e-11],cuts]))

@lru_cache(None)
def wedges(inner,outer,height,port_radius,port_angle,segments=192,**kwargs):
    result=[];c,s=math.cos(port_angle),math.sin(port_angle)
    axial=np.array([c,s,0.]);transverse=np.array([-s,c,0.]);vertical=np.array([0.,0.,1.])
    # Tangent halfspaces contain the true circular drill: no nominal-radius intrusion.
    # Cut top/bottom first. This keeps the bearing faces broad instead of
    # partitioning bottom sectors into small coplanar contact slivers.
    # The set of tool halfspaces (and hence the union of retained material) is unchanged.
    angles=[2*math.pi*i/64 for i in [16,48]+[j for j in range(64) if j not in (16,48)]]
    cuts=[(-axial,0.)]+[(math.cos(t)*transverse+math.sin(t)*vertical,port_radius) for t in angles]
    for i in range(segments):
        a,b=2*math.pi*i/segments,2*math.pi*(i+1)/segments
        xy=[(inner*math.cos(a),inner*math.sin(a)),(outer*math.cos(a),outer*math.sin(a)),
            (outer*math.cos(b),outer*math.sin(b)),(inner*math.cos(b),inner*math.sin(b))]
        points=np.array([(x,y,z) for z in (-height/2,height/2) for x,y in xy])
        if not port_radius or np.max(points@axial)<=0 or np.min(points@transverse)>port_radius or np.max(points@transverse)<-port_radius:
            result.append((f'wedge_{i:03d}_solid',points.tolist()));continue
        remaining=points
        for j,(normal,distance) in enumerate(cuts):
            remaining,kept=split(remaining,normal,distance)
            if kept is not None:result.append((f'wedge_{i:03d}_cut_{j:02d}',kept.tolist()))
            if remaining is None:break
    return result

def convex_faces(points):
    p=np.asarray(points);h=ConvexHull(p);faces=h.simplices.copy()
    keep=[]
    for i,f in enumerate(faces):
        normal=np.cross(p[f[1]]-p[f[0]],p[f[2]]-p[f[0]])
        # Qhull may triangulate a collinear boundary with zero-area facets.
        if not np.any(normal):continue
        if np.dot(normal,h.equations[i,:3])<0:faces[i]=f[::-1]
        keep.append(i)
    return faces[keep]

def bore_faces(points,faces,sector,inner_radius,segments=192):
    """Faces on this sector's inward bore plane, not its exterior or drill wall."""
    angle=2*math.pi*(sector+.5)/segments
    normal=np.array([math.cos(angle),math.sin(angle),0.])
    tris=np.asarray(points)[faces]
    on_plane=np.all(np.abs(tris@normal-inner_radius*math.cos(math.pi/segments))<1e-8,axis=1)
    face_normals=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
    return np.flatnonzero(on_plane & (face_normals@normal<0)).tolist()

def mesh(stage,path,points,color,inner_material=None,sector=None):
    from pxr import UsdGeom,UsdPhysics,UsdShade
    m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(points)
    faces=convex_faces(points)
    m.CreateFaceVertexCountsAttr([3]*len(faces));m.CreateFaceVertexIndicesAttr([int(i) for face in faces for i in face])
    m.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material(stage,path+'/look',color))
    if inner_material is not None:
        selected=bore_faces(points,faces,sector,L.R_IN)
        if selected:
            subset=UsdGeom.Subset.Define(stage,path+'/silver_inner_faces')
            subset.CreateElementTypeAttr(UsdGeom.Tokens.face)
            subset.CreateFamilyNameAttr('materialBind')
            subset.CreateIndicesAttr(selected)
            UsdGeom.Subset.SetFamilyType(m,'materialBind',UsdGeom.Tokens.nonOverlapping)
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(inner_material)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
    return m.GetPrim()

def decorate(prim,name):
    from pxr import Usd,UsdPhysics,UsdShade,Sdf
    stage,root=prim.GetStage(),str(prim.GetPath())
    # Replace the bootstrap cuboid entirely: no invisible box fills a bore.
    for child in list(prim.GetChildren()):stage.RemovePrim(child.GetPath())
    for n,p,s,c in L.parts(name):cube(stage,root+'/'+n,p,s,c)
    ring=getattr(L,'ring',lambda name:None)(name)
    if ring:
        inner_material=(material(stage,root+'/silver_bore',(.90,.91,.93),roughness=.45,metallic=.25)
                        if name=='sleeve' else None)
        for n,points in wedges(**ring):
            mesh(stage,root+'/'+n,points,ring['color'],inner_material=inner_material,
                 sector=int(n.split('_')[1]) if inner_material is not None else None)
    for label,ring,z_offset in L.extra_rings(name):
        for n,points in wedges(**ring):
            shifted=[(x,y,z+z_offset) for x,y,z in points]
            mesh(stage,root+'/'+label+'_'+n,shifted,ring['color'])
    mat=UsdShade.Material.Define(stage,root+'/physical_material')
    physics=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    physics.CreateStaticFrictionAttr(.8);physics.CreateDynamicFrictionAttr(.6);physics.CreateRestitutionAttr(0.)
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            p.AddAppliedSchema('PhysxCollisionAPI')
            p.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0002)
            p.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
            UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,materialPurpose='physics')
    prim.AddAppliedSchema('PhysxContactReportAPI')
    prim.CreateAttribute('physxContactReport:threshold',Sdf.ValueTypeNames.Float).Set(0.)
    if name in L.MOVING:
        prim.AddAppliedSchema('PhysxRigidBodyAPI')
        # Resolve the compound sleeve's many contact constraints accurately;
        # no damping, pose pinning or scoring allowance is added.
        prim.CreateAttribute('physxRigidBody:solverPositionIterationCount',Sdf.ValueTypeNames.Int).Set(32)
        prim.CreateAttribute('physxRigidBody:solverVelocityIterationCount',Sdf.ValueTypeNames.Int).Set(16)
    for item in L.joints():
        if item['body']!=name:continue
        from pxr import Gf
        joint=UsdPhysics.FixedJoint.Define(stage,root+'/passive_interface')
        joint.CreateBody1Rel().SetTargets([root])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*item['anchor']))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*item['local']))
        joint.CreateBreakForceAttr(float(item['force']));joint.CreateBreakTorqueAttr(float(item['torque']))
        joint.CreateExcludeFromArticulationAttr(True)
