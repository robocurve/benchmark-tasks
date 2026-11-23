"""Build the passive station from fixture.py. Run once, outside a trial."""
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics as P, UsdShade
import fixture as g

def build(file):
    stage=Usd.Stage.CreateInMemory()
    root="/station_fixture"
    base=UsdGeom.Xform.Define(stage,root)
    stage.SetDefaultPrim(base.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage,1)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    material=UsdShade.Material.Define(stage,root+"/surface")
    mat=P.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr(.85);mat.CreateDynamicFrictionAttr(.85);mat.CreateRestitutionAttr(0.)
    clear = UsdShade.Material.Define(stage, root+"/polycarbonate_visual")
    shader = UsdShade.Shader.Define(stage, clear.GetPath().AppendChild("surface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.55,.77,.84))
    shader.CreateInput("opacity",Sdf.ValueTypeNames.Float).Set(.19)
    shader.CreateInput("roughness",Sdf.ValueTypeNames.Float).Set(.27)
    clear.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
    names={}
    locations={}
    for name,parent,xyz,joint,kind,axis,bounds,mass,k,d in g.BODIES:
        path=root+"/"+name; names[name]=path
        parent_xyz=locations.get(parent,(0,0,0))
        absolute=tuple(a+b for a,b in zip(parent_xyz,xyz));locations[name]=absolute
        body=UsdGeom.Xform.Define(stage,path)
        body.AddTranslateOp().Set(Gf.Vec3d(*absolute))
        prim=body.GetPrim();P.RigidBodyAPI.Apply(prim)
        prim.AddAppliedSchema("PhysxContactReportAPI")
        prim.CreateAttribute("physxContactReport:threshold",Sdf.ValueTypeNames.Float,custom=False).Set(0.)
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:sleepThreshold",Sdf.ValueTypeNames.Float,custom=False).Set(0.)
        m=P.MassAPI.Apply(prim);m.CreateMassAttr(mass)
        center,diag=g.inertia(name);m.CreateCenterOfMassAttr(Gf.Vec3f(*center))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag));m.CreatePrincipalAxesAttr(Gf.Quatf(1))
        if parent is None:
            anchor=P.FixedJoint.Define(stage,root+"/mount")
            anchor.CreateBody1Rel().SetTargets([path]);anchor.CreateLocalPos0Attr(Gf.Vec3f(*g.WORLD))
            P.ArticulationRootAPI.Apply(prim)
            prim.AddAppliedSchema("PhysxArticulationAPI")
            prim.CreateAttribute("physxArticulation:enabledSelfCollisions",Sdf.ValueTypeNames.Bool,custom=False).Set(True)
            prim.CreateAttribute("physxArticulation:sleepThreshold",Sdf.ValueTypeNames.Float,custom=False).Set(0.)
            prim.CreateAttribute("physxArticulation:solverPositionIterationCount",Sdf.ValueTypeNames.Int,custom=False).Set(64)
            continue
        ctor=P.RevoluteJoint if kind=="hinge" else P.PrismaticJoint
        j=ctor.Define(stage,root+"/joints/"+joint)
        j.CreateBody0Rel().SetTargets([names[parent]]);j.CreateBody1Rel().SetTargets([path])
        j.CreateAxisAttr("Z" if axis==(0,0,1) else "Y" if axis==(0,1,0) else "X")
        rot=Gf.Quatf(1)
        j.CreateLocalPos0Attr(Gf.Vec3f(*xyz));j.CreateLocalPos1Attr(Gf.Vec3f(0))
        j.CreateLocalRot0Attr(rot);j.CreateLocalRot1Attr(rot)
        factor=180/3.141592653589793 if kind=="hinge" else 1
        j.CreateLowerLimitAttr(bounds[0]*factor);j.CreateUpperLimitAttr(bounds[1]*factor)
        drive=P.DriveAPI.Apply(j.GetPrim(),"angular" if kind=="hinge" else "linear")
        drive.CreateTypeAttr("force");drive.CreateStiffnessAttr(k/factor);drive.CreateDampingAttr(d/factor)
        drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
    from itertools import combinations
    # Filtering the articulation root against a child can suppress collisions for the entire subtree.
    for a,b in combinations(names,2):
        if a!="frame" and b!="frame" and not g.collision_allowed(a,b):
            P.FilteredPairsAPI.Apply(stage.GetPrimAtPath(names[a])).CreateFilteredPairsRel().AddTarget(names[b])
    for name,owner,xyz,size,color,kind in g.SHAPES:
        path=names[owner]+"/"+name
        if kind in ("box","guide_visual","track_visual","clear"):
            s=UsdGeom.Cube.Define(stage,path);s.CreateSizeAttr(1)
        elif kind in ("cylinder","band_visual"):
            s=UsdGeom.Cylinder.Define(stage,path);s.CreateAxisAttr("Y");s.CreateRadiusAttr(size[0]/2);s.CreateHeightAttr(size[1])
        elif kind=="ellipsoid":
            # PhysX spheres do not reliably retain nonuniform collision scaling.
            from math import pi, sin, cos
            s=UsdGeom.Mesh.Define(stage,path)
            rings,around=12,24
            pts=[(0,0,-size[2]/2)]
            for i in range(1,rings):
                a=pi*i/rings-pi/2
                pts.extend((size[0]*cos(a)*cos(2*pi*j/around)/2,size[1]*cos(a)*sin(2*pi*j/around)/2,size[2]*sin(a)/2) for j in range(around))
            pts.append((0,0,size[2]/2))
            faces=[]
            for j in range(around):
                n=(j+1)%around
                faces.append([0,1+n,1+j])
                for i in range(rings-2):
                    a=1+i*around
                    faces.append([a+j,a+n,a+around+n,a+around+j])
                a=1+(rings-2)*around
                faces.append([a+j,a+n,len(pts)-1])
            s.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);s.CreateFaceVertexCountsAttr([len(f) for f in faces])
            s.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);s.CreateSubdivisionSchemeAttr("none")
            P.MeshCollisionAPI.Apply(s.GetPrim()).CreateApproximationAttr("convexHull")
        else:
            raise ValueError(kind)
        s.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        if name in g.MARK_ANGLES:s.AddRotateYOp().Set(g.MARK_ANGLES[name])
        if kind in ("box","guide_visual","track_visual","clear"):s.AddScaleOp().Set(Gf.Vec3f(*size))
        s.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if kind=="clear":
            s.CreateDisplayOpacityAttr([.19])
            UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(clear)
        if kind.endswith("_visual"):continue
        P.CollisionAPI.Apply(s.GetPrim())
        s.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
        for key,value in (("contactOffset",.0008),("restOffset",0.)):
            s.GetPrim().CreateAttribute("physxCollision:"+key,Sdf.ValueTypeNames.Float,custom=False).Set(value)
        UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(material,materialPurpose="physics")
    file.parent.mkdir(parents=True,exist_ok=True)
    if not stage.GetRootLayer().Export(str(file)):raise RuntimeError("USD export failed")

def build_insert(file):
    stage=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    body=UsdGeom.Xform.Define(stage,"/insert");stage.SetDefaultPrim(body.GetPrim())
    P.RigidBodyAPI.Apply(body.GetPrim());m=P.MassAPI.Apply(body.GetPrim());m.CreateMassAttr(.070)
    body.GetPrim().AddAppliedSchema("PhysxContactReportAPI")
    body.GetPrim().CreateAttribute("physxContactReport:threshold",Sdf.ValueTypeNames.Float,custom=False).Set(0.)
    cube=UsdGeom.Cube.Define(stage,"/insert/yellow_insert")
    cube.CreateSizeAttr(1);cube.AddScaleOp().Set(Gf.Vec3f(*g.KEY_SIZE));cube.CreateDisplayColorAttr([Gf.Vec3f(*g.YELLOW)])
    P.CollisionAPI.Apply(cube.GetPrim())
    mat=UsdShade.Material.Define(stage,"/insert/contact_material")
    surf=P.MaterialAPI.Apply(mat.GetPrim());surf.CreateStaticFrictionAttr(.85);surf.CreateDynamicFrictionAttr(.85);surf.CreateRestitutionAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat,materialPurpose="physics")
    stage.GetRootLayer().Export(str(file))

if __name__=="__main__":
    build(Path(__file__).resolve().parent/"assets/station.usda")
    build_insert(Path(__file__).resolve().parent/"assets/service_insert.usda")
