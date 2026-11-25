"""Original cutaway catheter-removal training station, metres and Z up.

Free syringe, movable valve branch and contact-retained catheter.
Runtime pressure/volume surrogate is task-specific; no clinical validation.
"""
import math
import json
from pathlib import Path
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf
from geometry import Geometry, ROOT


def sphere(g,path,pos,scale,material,collide=False):
    obj=UsdGeom.Sphere.Define(g.stage,ROOT+'/'+path)
    obj.CreateRadiusAttr(1.)
    obj.AddTranslateOp().Set(Gf.Vec3d(*pos))
    obj.AddScaleOp().Set(Gf.Vec3f(*scale))
    return g.finish(obj,material,collide)


def ring_y(g,path,center,radius,thickness,material):
    x,y,z=center
    return g.tube(path,[(x+radius*math.cos(a*2*math.pi/96),y,
                         z+radius*math.sin(a*2*math.pi/96)) for a in range(97)],
                  thickness,material)


def cylinder_axis(g,path,pos,radius,length,axis,material,collide=False):
    obj=g.cylinder(path,pos,radius,length,material,collide)
    obj.GetAxisAttr().Set(axis)
    return obj


def rotate_local_before_scale(obj, axis, degrees):
    """Compose T*R*S: local box dimensions rotate with the annular sector.

    Appending R after nonuniform S composes T*S*R, deforming the intended bore.
    """
    rotation = getattr(obj, 'AddRotate'+axis+'Op')()
    rotation.Set(degrees)
    ops = obj.GetOrderedXformOps()
    obj.SetXformOpOrder([op for op in ops if op.GetOpType() != UsdGeom.XformOp.TypeScale]
                       + [op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeScale])


def build(path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    stage=Usd.Stage.CreateNew(str(path));root=UsdGeom.Xform.Define(stage,ROOT)
    stage.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    root.AddTranslateOp().Set(Gf.Vec3d(0,.10,0))
    g=Geometry(stage)
    for args in [('white',(.94,.96,.93),.55,0),('teal',(.10,.45,.47),.65,0),
                 ('glass',(.67,.86,.91),.16,0),('blue',(.18,.46,.82),.45,0),
                 ('balloon',(.96,.64,.22),.35,0),('silicone',(.88,.78,.61),.7,0),
                 ('steel',(.65,.72,.75),.24,.8),('ink',(.09,.16,.18),.8,0),
                 ('rubber',(.16,.22,.23),.85,0),('fluid',(.97,.84,.36),.35,0)]:g.material(*args)
    sh=UsdShade.Shader(stage.GetPrimAtPath(ROOT+'/Looks/glass/Shader'))
    sh.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.23)
    # Fixed phantom base and cradle on the existing surgical insert.
    g.box('Phantom/Base',(.375,.018,.794),(.183,.185,.009),'teal',True)
    for k,x in enumerate((.323,.427)):
        g.box(f'Phantom/Support{k}',(x,.025,.811),(.014,.105,.026),'white',True)
    # Hollow chamber physics retains the full wall except for the neck. The
    # separate visible shell has a broad cutaway so the retained balloon is
    # inspectable even when a renderer does not support translucent materials.
    cx,cy,cz=.375,.025,.851
    points=[];faces=[];rows=40;cols=96
    for j in range(rows+1):
        t=(math.pi-.16)*j/rows
        for k in range(cols):
            a=2*math.pi*k/cols
            points.append((cx+.075*math.sin(t)*math.cos(a),cy+.065*math.cos(t),
                           cz+.045*math.sin(t)*math.sin(a)))
    for j in range(rows):
        for k in range(cols):faces.append((j*cols+k,j*cols+(k+1)%cols,(j+1)*cols+(k+1)%cols,(j+1)*cols+k))
    collider=g.mesh('Phantom/ChamberCollision',points,faces,'glass')
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    collider.CreateVisibilityAttr('invisible')
    visible_faces=[f for f in faces
                   if sum(points[i][2] for i in f)/4 < cz-.025
                   or sum(points[i][1] for i in f)/4 > cy+.025]
    g.mesh('Phantom/CutawayChamber',points,visible_faces,'glass')
    # Sparse meridians show chamber shape even in views with weak translucency.
    for k,a in enumerate((0.,math.pi/2,math.pi,3*math.pi/2)):
        line=[]
        for j in range(65):
            t=(math.pi-.16)*j/64
            line.append((cx+.075*math.sin(t)*math.cos(a),cy+.065*math.cos(t),cz+.045*math.sin(t)*math.sin(a)))
        g.tube(f'Phantom/Outline{k}',line,.0008,'white',6)
    # A physical annular neck, axis Y, bore 12 mm. Individual convex sectors keep
    # the central channel open; the inflated balloon has a 30 mm diameter.
    for k in range(32):
        a=2*math.pi*k/32
        obj=g.box(f'Neck/Wall{k}',(.375+.009*math.cos(a),-.061,.851+.009*math.sin(a)),
                  (.006,.046,.0024),'silicone',True)
        rotate_local_before_scale(obj,'Y',-math.degrees(a))
    ring_y(g,'Neck/EntryRim',(.375,-.085,.851),.009,.002,'white')
    g.box('Neck/Anchor',(.375,-.061,.822),(.047,.048,.034),'white',True)
    # Catheter is a contact-retained rigid surrogate at the prototype stage.
    croot=(.375,-.145,.851)
    body=g.body('Catheter',croot,.040)
    cylinder_axis(g,'Catheter/Shaft',(0,.045,0),.003,.244,'Y','silicone',True)
    sphere(g,'Catheter/RoundedTip',(0,.168,0),(.003,.004,.003),'silicone')
    balloon=UsdGeom.Sphere.Define(stage,ROOT+'/Catheter/InflatedBalloon')
    balloon.CreateRadiusAttr(.015)
    balloon.AddTranslateOp().Set(Gf.Vec3d(0,.132,0))
    g.finish(balloon,'balloon',True)
    collapsed=UsdGeom.Sphere.Define(stage,ROOT+'/Catheter/CollapsedBalloon')
    collapsed.CreateRadiusAttr(.004)
    collapsed.AddTranslateOp().Set(Gf.Vec3d(0,.132,0))
    g.finish(collapsed,'balloon',True)
    UsdPhysics.CollisionAPI(collapsed.GetPrim()).CreateCollisionEnabledAttr(False)
    collapsed.CreateVisibilityAttr('invisible')
    # Eyelets are visual markers on the smooth training catheter, not tissue.
    sphere(g,'Catheter/EyeA',(.0027,.157,0),(.0008,.003,.0013),'ink')
    sphere(g,'Catheter/EyeB',(-.0027,.149,0),(.0008,.003,.0013),'ink')
    cylinder_axis(g,'Catheter/Hub',(0,-.087,0),.009,.030,'Y','white',True)
    g.box('Catheter/GraspTab',(0,-.087,.010),(.028,.027,.012),'blue',True)
    # Distinct drainage and balloon-valve branches; no human anatomy is depicted.
    ring_y(g,'Catheter/DrainOutletRim',(0,-.104,0),.007,.0015,'blue')
    valve=g.body('Valve',(.336,-.239,.851),.025)
    branch=g.tube('Valve/Branch',[(.039,.007,0),(.022,0,0),(0,0,0)],.004,'silicone')
    UsdPhysics.CollisionAPI.Apply(branch.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(branch.GetPrim()).CreateApproximationAttr('convexHull')
    g.box('Valve/GraspTab',(0,.015,.007),(.030,.024,.020),'blue',True)
    for k in range(24):
        a=2*math.pi*k/24
        obj=g.box(f'Valve/SocketWall{k}',(-.007,.0065*math.cos(a),.0065*math.sin(a)),
                  (.016,.0035,.0020),'white',True)
        rotate_local_before_scale(obj,'X',math.degrees(a))
    joint=UsdPhysics.SphericalJoint.Define(stage,ROOT+'/ValveBranchJoint')
    joint.CreateBody0Rel().SetTargets([body.GetPath()]);joint.CreateBody1Rel().SetTargets([valve.GetPath()])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0,-.087,0));joint.CreateLocalPos1Attr(Gf.Vec3f(.039,.007,0))
    joint.CreateCollisionEnabledAttr(False)
    # Empty, needle-free receiver syringe in two supports on a clean instrument pad.
    g.box('SyringePad',(.163,-.242,.782),(.160,.125,.010),'steel',True)
    for k,x in enumerate((.122,.202)):
        g.box(f'SyringeCradle/S{k}',(x,-.250,.797),(.013,.032,.021),'white',True)
    barrel=g.body('Syringe/Barrel',(.163,-.250,.820),.100)
    # Hollow barrel wall made from convex strips. Plunger never intersects a
    # solid collision cylinder used to impersonate transparent glass.
    for k in range(24):
        a=2*math.pi*k/24
        obj=g.box(f'Syringe/Barrel/Wall{k}',(0,.012*math.cos(a),.012*math.sin(a)),
                  (.090,.0023,.0023),'glass',True)
        rotate_local_before_scale(obj,'X',math.degrees(a))
    for name,x in [('Back',-.045),('Front',.045)]:
        cylinder_axis(g,f'Syringe/Barrel/{name}Ring',(x,0,0),.013,.004,'X','white')
    g.box('Syringe/Barrel/FingerFlange',(-.047,0,0),(.004,.043,.018),'white',True)
    cylinder_axis(g,'Syringe/Barrel/Nozzle',(.054,0,0),.004,.016,'X','white')
    for k in range(9):
        x=-.035+.008*k
        g.box(f'Syringe/Barrel/Mark{k}',(x,-.0123,.002),(.001,.001,.008 if k%2==0 else .004),'ink')
    plunger=g.body('Syringe/Plunger',(.163,-.250,.820),.015)
    cylinder_axis(g,'Syringe/Plunger/Piston',(.035,0,0),.0098,.007,'X','blue',True)
    g.box('Syringe/Plunger/Rod',(-.018,0,0),(.102,.005,.005),'white',True)
    g.box('Syringe/Plunger/ThumbPad',(-.072,0,0),(.006,.032,.025),'blue',True)
    slide=UsdPhysics.PrismaticJoint.Define(stage,ROOT+'/Syringe/PlungerJoint')
    slide.CreateBody0Rel().SetTargets([barrel.GetPath()]);slide.CreateBody1Rel().SetTargets([plunger.GetPath()])
    slide.CreateAxisAttr('X');slide.CreateLowerLimitAttr(-.055);slide.CreateUpperLimitAttr(0.)
    drive=UsdPhysics.DriveAPI.Apply(slide.GetPrim(),'linear')
    drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(.5)
    # No pre-connected extension: the loose syringe must be brought to the valve.
    # Kidney-shaped recovery dish, long axis Y; static, open and visibly empty.
    tc=(.310,-.361,.779);N=80;boundary=[]
    for k in range(N):
        a=2*math.pi*k/N
        boundary.append((tc[0]+.055*math.cos(a)+.012*math.cos(2*a),tc[1]+.128*math.sin(a)))
    pts=[(tc[0],tc[1],tc[2])]+[(x,y,tc[2]) for x,y in boundary]
    floor=g.mesh('RecoveryDish/Floor',pts,[(0,1+k,1+(k+1)%N) for k in range(N)],'steel')
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    wallpts=[(x,y,tc[2]+z) for z in (0.,.018) for x,y in boundary]
    wall=g.mesh('RecoveryDish/Rim',wallpts,[(k,(k+1)%N,N+(k+1)%N,N+k) for k in range(N)],'steel')
    UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
    for k,(x,y) in enumerate(((.295,-.32),(.295,-.40))):
        g.box(f'RecoveryDish/Rest{k}',(x,y,.783),(.020,.028,.008),'teal',True)
    stage.GetRootLayer().Save()
    meta={'units':'m','chamber_radii_m':[.075,.065,.045],'catheter_radius_m':.003,
          'inflated_balloon_radius_m':.015,'deflated_balloon_radius_m':.004,'neck_bore_radius_m':.006,
          'root_translation_m':[0,.10,0],'syringe_plunger_travel_m':[-.055,0],
          'rigid_bodies':['Catheter','Valve','Syringe/Barrel','Syringe/Plunger'],
          'physical':['annular neck and hollow syringe wall','free catheter retained by contacts','spherical valve branch','free syringe and prismatic plunger'],
          'visual_only':['cutaway chamber window'],
          'chamber_visualization':'cutaway display shell; invisible full hollow wall retains collisions',
          'approximations':['rigid main catheter','one spherical joint for flexible valve branch','runtime pressure-volume surrogate; no fluid solver'],
          'not_implemented':['clinical anatomy and forces','continuous catheter deformation']}
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2)+'\n')
    return meta


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('output',type=Path)
    print(json.dumps(build(p.parse_args().output),indent=2))
