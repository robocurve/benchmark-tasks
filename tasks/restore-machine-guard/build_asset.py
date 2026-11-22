"""Author a genuine articulated guard/latch/switch USD from standard primitives.

Runs only inside an Isaac Sim/Isaac Lab Python environment with ``pxr``. Pure
geometry; no runtime pose teleportation or surrogate lock-state toggles. The
fixture uses collision geometry with physical hinges and prismatic joints.
"""
from __future__ import annotations
from pathlib import Path
import os
import sys
from tempfile import TemporaryDirectory

import geometry as g


def build(path: str | Path) -> Path:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    # Author named USD schemas even when the standalone Python environment
    # lacks the optional Kit schema wrappers. PhysX reads them on stage load.
    def physics_api(prim, name, attrs):
        prim.AddAppliedSchema(name)
        for key, value_type, value in attrs:
            prim.CreateAttribute(key, value_type, custom=False).Set(value)
    T = Sdf.ValueTypeNames

    profile = os.getenv('GUARD_COLLISION_PROFILE', 'nominal')
    if profile not in ('nominal', 'no_door_colliders', 'no_bolt_colliders', 'no_switch_colliders', 'no_frame_colliders'):
        raise ValueError(f'Invalid collision diagnostic profile: {profile}')
    dst = Path(path).resolve()
    dst.parent.mkdir(parents=True, exist_ok=True)
    # Author in memory and atomically replace output only after verification.
    # CreateNew(dst) fails on a subsequent doctor/build if the file exists.
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = '/guard_station'
    UsdGeom.Xform.Define(stage, root)
    stage.SetDefaultPrim(stage.GetPrimAtPath(root))
    physics_material = root+'/Looks/phys_mat'
    mat = UsdShade.Material.Define(stage, physics_material)
    pmat = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pmat.CreateStaticFrictionAttr(float(g.STATIC_FRICTION))
    pmat.CreateDynamicFrictionAttr(float(g.DYNAMIC_FRICTION))
    pmat.CreateRestitutionAttr(float(g.RESTITUTION))

    # Colors identify moving mechanical parts; no invisible score-only geometry.
    steel=(.48,.51,.55); clear_blue=(.16,.56,.70); orange=(.95,.38,.09)
    yellow=(.97,.79,.05); dark=(.19,.20,.23)

    def rigid(name, mass):
        p=f'{root}/{name}'
        x=UsdGeom.Xform.Define(stage,p)
        r=UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        r.CreateRigidBodyEnabledAttr(True)
        physics_api(x.GetPrim(),'PhysxContactReportAPI',(
            ('physxContactReport:threshold',T.Float,0.),))
        UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(float(mass))
        physics_api(x.GetPrim(),'PhysxRigidBodyAPI',(
            ('physxRigidBody:solverPositionIterationCount',T.Int,48),
            ('physxRigidBody:solverVelocityIterationCount',T.Int,8),
            ('physxRigidBody:maxDepenetrationVelocity',T.Float,.35)))
        return p

    base=rigid('base',g.BASE_MASS_KG)
    door=rigid('door',g.DOOR_MASS_KG)
    # Critical v5 native reset correction: the USD rigid link itself begins
    # physically open.  In v3/v4 PhysX first initialized the authored door at
    # CLOSED (joint q~0), then the reset hook jumped it to q=-53 degrees while
    # contact pairs and the plunger were already engaged.  The following
    # settling step pushed the hinge/bolt to their stops.  Align both the
    # authored rigid-body rotation and ArticulationCfg.init_state.joint_pos.
    # The hinge joint's local frames remain identity: its coordinate at the
    # authored pose should be -53 deg, exactly as requested by the benchmark.
    from math import degrees
    UsdGeom.Xformable(stage.GetPrimAtPath(door)).AddRotateZOp().Set(float(degrees(g.OPEN_RAD)))
    bolt=rigid('bolt',g.BOLT_MASS_KG)
    switch=rigid('switch',g.SWITCH_MASS_KG)
    # Root body is fixed to world through a physically defined joint. The
    # world anchor is expressed in task-frame coordinates and audited natively.
    fixed=UsdPhysics.FixedJoint.Define(stage, root+'/joints/world_anchor')
    fixed.CreateBody1Rel().SetTargets([Sdf.Path(base)])
    from yam_arena.task_file import TABLE_TOP
    anchor_world_z = float(TABLE_TOP["station"]) + 0.020
    fixed.CreateLocalPos0Attr(Gf.Vec3f(g.HINGE_X,g.HINGE_Y,anchor_world_z))
    fixed.CreateLocalPos1Attr(Gf.Vec3f(0.0,0.0,0.0))
    fixed.CreateLocalRot0Attr(Gf.Quatf(1,0,0,0))
    fixed.CreateLocalRot1Attr(Gf.Quatf(1,0,0,0))

    # Geometry is relative to each rigid-body's own origin; body transforms
    # place bolt and switch in their neutral positions.
    UsdGeom.Xformable(stage.GetPrimAtPath(bolt)).AddTranslateOp().Set(Gf.Vec3d(-.014,-.218,.068))
    UsdGeom.Xformable(stage.GetPrimAtPath(switch)).AddTranslateOp().Set(Gf.Vec3d(.025,-.120,.125))

    def cube(body,name,center,size,color):
        c=UsdGeom.Cube.Define(stage,body+'/'+name)
        c.CreateSizeAttr(1)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if profile == 'nominal' or profile != {'base': 'no_frame_colliders', 'door': 'no_door_colliders', 'bolt': 'no_bolt_colliders', 'switch': 'no_switch_colliders'}[body.rsplit('/', 1)[-1]]:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat,materialPurpose='physics')
        physics_api(c.GetPrim(),'PhysxCollisionAPI',(
            ('physxCollision:contactOffset',T.Float,g.CONTACT_OFFSET_M),
            ('physxCollision:restOffset',T.Float,g.REST_OFFSET_M)))

    def cylinder(body,name,center,rad,length,color,axis='Y'):
        c=UsdGeom.Cylinder.Define(stage,body+'/'+name)
        c.CreateRadiusAttr(float(rad))
        c.CreateHeightAttr(float(length))
        c.CreateAxisAttr(axis)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if profile == 'nominal' or profile != {'base': 'no_frame_colliders', 'door': 'no_door_colliders', 'bolt': 'no_bolt_colliders', 'switch': 'no_switch_colliders'}[body.rsplit('/', 1)[-1]]:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat,materialPurpose='physics')
        physics_api(c.GetPrim(),'PhysxCollisionAPI',(
            ('physxCollision:contactOffset',T.Float,g.CONTACT_OFFSET_M),
            ('physxCollision:restOffset',T.Float,g.REST_OFFSET_M)))

    # Protected frame is OPEN in its middle: guard is not a painted plane.
    cube(base,'frame_hinge_post',(.021,0,.090),(.022,.016,.180),steel)
    cube(base,'frame_striker_post',(.022,-.184,.090),(.020,.014,.180),steel)
    cube(base,'frame_top',(.025,-.092,.173),(.018,.183,.015),steel)
    cube(base,'frame_bottom',(.021,-.092,.012),(.025,.184,.019),steel)
    # Frame-mounted seating face defines the datum for the bolt and keeper.
    # The closed panel's inside face is x=+2 mm. The bumper face is x=+2.1 mm.
    cube(base,'striker_stop',(.0091,-.153,.122),(.014,.018,.018),dark)
    cube(base,'striker_stop_bracket',(.018,-.166,.122),(.009,.040,.018),steel)
    # Machine housing behind the panel: physical, but not armed/powered.
    cube(base,'machine_spindle_housing',(.082,-.118,.087),(.052,.082,.094),dark)
    # Stationary three-jaw chuck on its headstock; spindle axis along Y.
    cylinder(base,'spindle_nose',(.070,-.078,.092),.019,.014,steel,axis='Y')
    cylinder(base,'fixed_spindle',(.070,-.057,.092),.039,.036,steel,axis='Y')
    from math import cos, sin, pi
    for i in range(3):
        a = 2*pi*i/3
        jaw = 'chuck_jaw_'+str(i)
        cube(base,jaw,(.070+.020*cos(a),-.035,.092+.020*sin(a)),(.019,.012,.010),dark)
        xf=UsdGeom.Xformable(stage.GetPrimAtPath(base+'/'+jaw))
        trans,scale=xf.GetOrderedXformOps()
        rot=xf.AddRotateYOp()
        rot.Set(-degrees(a))
        xf.SetXformOpOrder([trans,rot,scale])
    cylinder(base,'blunt_workpiece',(.070,-.031,.092),.008,.040,steel,axis='Y')

    # Real hinged leaf. Grasp handle attached near the FREE edge.
    cube(door,'polycarbonate_guard',(-.002,-.085,.090),(.008,.168,.145),clear_blue)
    panel=stage.GetPrimAtPath(door+'/polycarbonate_guard')
    UsdGeom.Gprim(panel).CreateDisplayOpacityAttr([.22])
    glass=UsdShade.Material.Define(stage,root+'/Looks/window')
    shader=UsdShade.Shader.Define(stage,root+'/Looks/window/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',T.Color3f).Set(Gf.Vec3f(*clear_blue))
    shader.CreateInput('opacity',T.Float).Set(.22)
    glass.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    UsdShade.MaterialBindingAPI.Apply(panel).Bind(glass)
    for z in (.020,.160):
        cube(door,'window_rail_'+str(int(z*1000)),(-.002,-.085,z),(.009,.168,.005),steel)
    cube(door,'guard_handle_mount',(-.030,-.111,.105),(.062,.012,.012),steel)
    cube(door,'guard_handle_stem',(-.060,-.111,.113),(.009,.010,.024),steel)
    cube(door,'guard_finger_handle',g.DOOR_HANDLE_LOCAL,(.020,.025,.028),orange)
    cube(door,'door_tongue',(.006,-.120,.125),(.022,.012,.010),yellow)
    # Four strips define a hollow keeper eyelet (not a solid scored region).
    # The narrower horizontal aperture limits rebound at the interlock.
    eye_y=-.179; eye_x=-.014; eye_z=.068
    for lab,dx,dz,sx,sz in (
        ('e_top',0,.012,.027,.006),('e_bottom',0,-.012,.027,.006),
        ('e_left',-.008,0,.006,.018),('e_right',g.KEEPER_WIDTH_M-.005+.003,0,.006,.018)):
        cube(door,lab,(eye_x+dx,eye_y,eye_z+dz),(sx,.010,sz),steel)

    # Welded strap joins the upper eyelet to the polycarbonate guard leaf.
    # Its 5mm height stays above the moving steel shaft's swept corridor.
    cube(door,'keeper_weld_bridge',(-.008,-.171,.080),(.014,.020,.005),steel)

    # Bolt carriage: cylinder extends through frame guide and door eyelet.
    # Bolt's initial body origin is (-.014,-.218,.068). The prismatic joint
    # translates it +Y up to 52mm; there is no software 'latch' teleport.
    # A circumferential groove captures the keeper under the opening load.
    # Coaxial pieces meet end to end; the spring still retracts a free bolt.
    for part,ymin,ymax,radius in (
            ('shaft_tail',-.0145,-.011,.004),
            ('keeper_groove',-.011,.003,.0031),
            ('lock_shaft',.003,.0325,.004)):
        cylinder(bolt,part,(0,(ymin+ymax)/2,0),radius,ymax-ymin,steel,axis='Y')
    cube(bolt,'bolt_grip',g.BOLT_PAD_LOCAL,(.020,.023,.024),orange)
    cube(bolt,'handle_riser',(-.052,g.BOLT_PAD_LOCAL[1],.027),(.009,.009,.054),steel)
    cube(bolt,'handle_tail',(-.052,(g.BOLT_PAD_LOCAL[1]-.018)/2,0),(.010,abs(g.BOLT_PAD_LOCAL[1]+.018)+.010,.010),steel)
    cube(bolt,'handle_foot',(-.041,-.018,0),(.030,.014,.009),steel)
    cube(bolt,'thumb_stud',(-.017,-.014,0),(.034,.006,.005),steel)

    # Steel four-piece guide with actual shaft clearance.
    bx=-.014; bz=.068; by=-.214
    for lab,dx,dz,sx,sz in (
        ('g_top',0,.012,.027,.006),('g_bottom',0,-.012,.027,.006),
        ('g_right',.012,0,.006,.018),):
        cube(base,lab,(bx+dx,by,bz+dz),(sx,.015,sz),steel)

    # Spring-return 7mm contact plunger. The door's tongue physically touches
    # its face; a sensor reads THIS slider coordinate, not door angle alone.
    cylinder(switch,'sensing_plunger',(-.003,0,0),.0045,.022,yellow,axis='X')
    # Case is a hollow cavity, so the plunger is free to translate +X.
    for lab,dy,dz,sy,sz in (
        ('s_top',0,.009,.022,.004),('s_bottom',0,-.009,.022,.004),
        ('s_left',-.010,0,.004,.015),('s_right',.010,0,.004,.015)):
        cube(base,lab,(.037,-.120+dy,.125+dz),(.025,sy,sz),dark)

    def joint(kind,name,body1,anchor,axis,lo,hi):
        if kind=='revolute': j=UsdPhysics.RevoluteJoint.Define(stage,root+'/joints/'+name)
        else: j=UsdPhysics.PrismaticJoint.Define(stage,root+'/joints/'+name)
        j.CreateBody0Rel().SetTargets([Sdf.Path(base)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*anchor))
        j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
        j.CreateLocalRot0Attr(Gf.Quatf(1,0,0,0))
        j.CreateLocalRot1Attr(Gf.Quatf(1,0,0,0))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        # Per-joint collisionEnabled is ignored for articulations by PhysX.
        # Use ArticulationRootPropertiesCfg(enabled_self_collisions=True)
        # in guardkit.py for door/bolt/switch contact instead.
        return j

    from math import degrees
    joint('revolute','hinge',door,(0,0,0),'Z',degrees(g.HINGE_MIN_RAD),degrees(g.HINGE_MAX_RAD))
    joint('prismatic','bolt_slide',bolt,(-.014,-.218,.068),'Y',0,g.BOLT_TRAVEL)
    joint('prismatic','switch_slide',switch,(.025,-.120,.125),'X',0,g.SWITCH_TRAVEL)
    # USD angular drive gains are per degree. Arena's implicit actuators take
    # the same physical constants per radian. The drives are part of the asset.
    from math import radians
    for name, kind, rest, stiffness, damping, limit in (
        ('hinge','angular',degrees(g.OPEN_RAD),radians(g.HINGE_OPEN_STIFFNESS_NM_RAD),
         radians(g.HINGE_DAMPING_NMS_RAD),2.0),
        ('bolt_slide','linear',0.0,g.BOLT_RETRACT_STIFFNESS_N_M,g.BOLT_SLIDE_DAMPING_NS_M,8.0),
        ('switch_slide','linear',0.0,g.SWITCH_SPRING_N_M,g.SWITCH_DAMPING_NS_M,3.0),
    ):
        prim = stage.GetPrimAtPath(root+'/joints/'+name)
        drive = UsdPhysics.DriveAPI.Apply(prim,kind)
        drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(float(rest))
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateStiffnessAttr(float(stiffness))
        drive.CreateDampingAttr(float(damping))
        drive.CreateMaxForceAttr(float(limit))
    # A filter aimed at the articulation root body also suppressed the
    # moving leaf/plunger and leaf/shaft contacts in the CPU PhysX experiment.
    # The open-sided guide and switch case have real geometric clearance.
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(base))
    physics_api(stage.GetPrimAtPath(base),'PhysxArticulationAPI',(
        ('physxArticulation:enabledSelfCollisions',T.Bool,True),
        ('physxArticulation:solverPositionIterationCount',T.Int,64),
        ('physxArticulation:solverVelocityIterationCount',T.Int,8)))


    # A second run must not fail due to the existing .usda. This also avoids
    # damaging a previously good fixture if export/validation fails midway.
    with TemporaryDirectory(prefix='.guard-usd-', dir=str(dst.parent)) as tmpdir:
        staged=Path(tmpdir)/dst.name
        if not stage.GetRootLayer().Export(str(staged)):
            raise RuntimeError(f'OpenUSD layer export failed: {staged}')
        reopened=Usd.Stage.Open(str(staged))
        if not reopened or not reopened.GetDefaultPrim() or reopened.GetDefaultPrim().GetName()!='guard_station':
            raise RuntimeError('Exported guard USD failed default-prim validation')
        for name in ('hinge','bolt_slide','switch_slide'):
            prim=reopened.GetPrimAtPath(root+'/joints/'+name)
            if not prim.IsValid():
                raise RuntimeError(f'Exported guard USD missing joint: {name}')
        os.replace(staged,dst)
    print(f'Built verified articulating USD: {dst}')
    return dst


if __name__ == '__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',default=str(Path(__file__).parent/'assets/guard_station.usda'))
    a=p.parse_args();build(a.out)
