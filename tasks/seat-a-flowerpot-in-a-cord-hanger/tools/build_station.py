"""Native free pot with hollow grip lip, open rope network and fixed upper suspension hook."""
from pathlib import Path
import numpy as np
from pxr import Gf,Usd,UsdGeom,UsdPhysics, Sdf
from rdb_planter import fixed_scene as s
F=Path(__file__).resolve().parents[1]/'rdb_planter/station_prop.usda'
WOOD=(.47,.33,.20)
def gf(a):return tuple(map(float,a))
def main():
    st=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(st,1);UsdGeom.SetStageUpAxis(st,'Z')
    root=UsdGeom.Xform.Define(st,'/Set');st.SetDefaultPrim(root.GetPrim());UsdGeom.Xform.Define(st,'/Set/parts');UsdGeom.Scope.Define(st,'/Set/joints')
    bodypaths={}
    def body(n,p,m):
        x=UsdGeom.Xform.Define(st,'/Set/parts/'+n);x.AddTranslateOp().Set(gf(p));UsdPhysics.RigidBodyAPI.Apply(x.GetPrim());UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(m)
        prim=x.GetPrim()
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:solverPositionIterationCount",Sdf.ValueTypeNames.Int).Set(24)
        if n.startswith(("lace_","cord_")) or n=="bottom_knot":
            prim.CreateAttribute("physxRigidBody:angularDamping",Sdf.ValueTypeNames.Float).Set(.18)
            prim.CreateAttribute("physxRigidBody:linearDamping",Sdf.ValueTypeNames.Float).Set(.08)
        bodypaths[n]=(x.GetPath(),np.asarray(p));return x.GetPath()
    def geo(g,color):
        g.CreateDisplayColorAttr([color]);prim=g.GetPrim();UsdPhysics.CollisionAPI.Apply(prim)
        # Thin braided cord needs a submillimetre contact shell.
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset",Sdf.ValueTypeNames.Float).Set(.0008)
        prim.CreateAttribute("physxCollision:restOffset",Sdf.ValueTypeNames.Float).Set(0.)
    def sphere(path,r,col):
        g=UsdGeom.Sphere.Define(st,path);g.CreateRadiusAttr(r);geo(g,col);return g
    def rod(path,a,b,r):
        d=b-a;n=np.linalg.norm(d);q=Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*gf(d/n))).GetQuat()
        g=UsdGeom.Capsule.Define(st,path);g.CreateHeightAttr(n);g.CreateRadiusAttr(r);g.AddTranslateOp().Set(gf((a+b)/2))
        g.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q);geo(g,WOOD)
    def joint(name,a,b,point):
        j=UsdPhysics.SphericalJoint.Define(st,'/Set/joints/'+name)
        if a is not None:j.CreateBody0Rel().SetTargets([bodypaths[a][0]])
        j.CreateBody1Rel().SetTargets([bodypaths[b][0]])
        j.CreateLocalPos0Attr(gf(point-(bodypaths[a][1] if a is not None else 0.)))
        j.CreateLocalPos1Attr(gf(point-bodypaths[b][1]));j.CreateCollisionEnabledAttr(False)
    anchor=s.HOOK
    # Four strands join at a common upper ring and bottom knot. The empty harp
    # has slack and the pot starts entirely outside it.
    tip=anchor+[0,0,s.KNOT_Z]
    # Each rounded turn belongs to the same tied body. Outer contact follows
    # the four winding ridges. There are no gripper welds or pot attachments.
    turn_length = 2 * np.pi * (s.KNOT_RADIUS - s.CORD_RADIUS)
    core_length = 4 * 2 * s.KNOT_HALF_HEIGHT
    knot_mass = (s.WRAP_TURNS * turn_length + core_length) * (.018 / .41)
    k=body('bottom_knot',tip,knot_mass)
    for turn in range(s.WRAP_TURNS):
        z=(2*turn+1-s.WRAP_TURNS)*s.CORD_RADIUS
        shape=UsdGeom.Mesh.Define(st,k.AppendChild(f'wrap_{turn}'))
        points=[]; count=28
        for level in np.linspace(-np.pi/2,np.pi/2,9):
            radius=s.KNOT_RADIUS-s.CORD_RADIUS+s.CORD_RADIUS*np.cos(level)
            for angle in np.arange(count)*2*np.pi/count:
                points.append((radius*np.cos(angle),radius*np.sin(angle),z+s.CORD_RADIUS*np.sin(level)))
        faces=[tuple(reversed(range(count)))]
        faces.extend((j*count+i,j*count+(i+1)%count,(j+1)*count+(i+1)%count,(j+1)*count+i)
                     for j in range(8) for i in range(count))
        faces.append(tuple(8*count+i for i in range(count)))
        shape.CreatePointsAttr([gf(p) for p in points])
        shape.CreateFaceVertexCountsAttr([len(face) for face in faces])
        shape.CreateFaceVertexIndicesAttr([v for face in faces for v in face])
        shape.CreateSubdivisionSchemeAttr('none');geo(shape,WOOD)
        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexHull')
    for i in range(4):
        d=np.array([np.cos((i+.5)*np.pi/2),np.sin((i+.5)*np.pi/2),0])
        tip_attachment=tip+s.EYE_RADIUS*d+[0.,0.,s.EYE_HEIGHT]
        profile=s.REST_STATIONS
        points=np.concatenate([np.linspace(anchor+r1*d+[0,0,z1],anchor+r2*d+[0,0,z2],num+1)[:-1] for ((r1,z1),(r2,z2)),num in zip(zip(profile,profile[1:]),(1,9,5,3))]+[[tip_attachment]])
        prev=None
        for j,(a,b) in enumerate(zip(points[:-1],points[1:])):
            n=f'cord_{i}_{j:02d}';mid=(a+b)/2;p=body(n,mid,.018*np.linalg.norm(b-a)/.41)
            rod(p.AppendChild('fiber'),a-mid,b-mid,s.CORD_RADIUS)
            joint('start_'+n, prev,n,a);prev=n
        joint(f'base_{i}',prev,'bottom_knot',tip_attachment)
    from rdb_planter.lacing import routes, TOP_LINK, LOW_LINK, RADIUS
    for i,points,_ in routes():
        previous=f'cord_{i}_{TOP_LINK:02d}'
        for j,(a,b) in enumerate(zip(points[:-1],points[1:])):
            name=f'lace_{i}_{j:02d}'
            mid=(a+b)/2
            path=body(name,mid,.018*(RADIUS/s.CORD_RADIUS)**2*np.linalg.norm(b-a)/.41)
            rod(path.AppendChild('fiber'),a-mid,b-mid,RADIUS)
            joint('tied_'+name,previous,name,a)
            previous=name
        joint(f'lace_end_{i}',previous,f'cord_{(i+1)%4}_{LOW_LINK:02d}',points[-1])
    bodypath=body('pot',s.POT,.33)
    col=(.66,.31,.16)
    for n,r,h,z in [('base',.044,.005,.0025),('ballast',.049,.061,.033)]:
        g=UsdGeom.Cylinder.Define(st,bodypath.AppendChild(n));g.CreateRadiusAttr(r);g.CreateHeightAttr(h);g.AddTranslateOp().Set((0.,0.,z));geo(g,col if n=='base' else (.27,.22,.12))
    sectors=24
    for i in range(sectors):
        angles=[2*np.pi*j/sectors for j in (i,i+1)]
        for n,r0,r1,z0,z1,t in [('wall',.044,.060,.003,.096,.0023),('lip',.0615,.0615,.09,.096,.004)]:
            pts=[(r*np.cos(a),r*np.sin(a),z) for (r,z) in [(r0-t,z0),(r0,z0),(r1,z1),(r1-t,z1)] for a in angles]
            m=UsdGeom.Mesh.Define(st,bodypath.AppendChild(f'{n}_{i}'));m.CreatePointsAttr([gf(p) for p in pts])
            faces=[(0,2,3,1),(2,4,5,3),(4,6,7,5),(6,0,1,7),(0,6,4,2),(1,3,5,7)]
            m.CreateFaceVertexCountsAttr([4]*6);m.CreateFaceVertexIndicesAttr([v for face in faces for v in face])
            m.CreateSubdivisionSchemeAttr('none');geo(m,col);UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
    rod('/Set/support',anchor+[-.13,0,.018],anchor+[.025,0,.018],.005)
    eye=sphere('/Set/hook',.009,(.28,.29,.30));eye.AddTranslateOp().Set(gf(anchor));UsdPhysics.CollisionAPI(eye).CreateCollisionEnabledAttr(False)
    st.GetRootLayer().Export(str(F));assert Usd.Stage.Open(str(F))
if __name__=='__main__':main()
