"""Single-screw drawer-knob task with ideal 0.7 mm/rev feed."""
from gap_assembly_task import *
from single_screw_score import score_from_evidence
import numpy as np
from dataclasses import replace

INSTRUCTION='Inspect the loose drawer-knob screw. Fit the matching bit into the stopped powered screwdriver, tighten the screw with that tool, then release the stopped tool in its stand.'
MOTOR_RPM=15.0
MOTOR_TORQUE_NM=.10
SCREW_RUNNING_RESISTANCE_NM=.02
SCREW_SITES={'lower':-.06}
ASSEMBLY_HOLD_STEPS=20
FINAL_HOLD_STEPS=20
POWERED_FEED_REQUIRED_M=.0013
BIT_TIP_LOCAL_Z=.0955  # End of the collision mesh, not its midpoint.
TEST_LOAD_N=5.
MAX_LOAD_DEFLECTION_M=.00025
WORKPIECE_SHIFT_X=-.050
WORKPIECE_SHIFT_Y=.325
# A guided single-screw knob has 1.5 mm initial slack.
parts=[]
for o in OBJECTS:
    if o.name=='driver':
        extra=(Slider('pressure',(0,0,0),(),travel=.004,rate=600.,damping=2.,mass=.02,axis='Z',sign=-1),)
        # Rear grasp sleeve lets one arm hold the tool without entering the
        # other arm's front chuck corridor. It is one rigid body with the tool.
        rear_sleeve=((0,0,-.035),.018,.070,DARK)
        # A molded, flat-sided rubber grip gives the parallel jaws a stable
        # purchase on the tool body instead of the round housing rolling out.
        grip_panel=((0,0,.035),(.040,.040,.045),DARK)
        # Fixed molded grip shoulders, not controls: spread jaw contact along
        # the tool axis so the long nose cannot pivot about a single pad edge.
        grip_shoulders=tuple(((x,0,.035),(.006,.040,.045),DARK) for x in (-.023,.023))
        # Retire the inherited decorative selector plate as well as the trigger.
        housing_boxes=tuple(b for b in o.boxes if b[0] not in ((-.019,0,.077),(-.021,0,.077)))
        o=replace(o,sliders=tuple(o.sliders)+extra,
                  boxes=housing_boxes+(grip_panel,)+grip_shoulders,
                  cylinders=tuple(o.cylinders)+(rear_sleeve,),
                  size=(max(o.size[0],.052),o.size[1],o.size[2]+.070))
    elif o.name=='drawer_context':
        # Preserve indices consumed by upstream geometry wrappers, remove fused handle at spawn.
        # One screw holds a single drawer knob rather than a two-post handle.
        hb=(((FASTENER_X,-.06,.052),(.042,.042,.018),DARK),
            ((FASTENER_X,-.06,.070),(.018,.018,.026),DARK))
        # A real drawer fastener is exposed on the inner face. The earlier
        # raised guard blocked the much wider driver nose; physical use is
        # checked by the score rather than by an artificial access shroud.
        shallow_boxes=list(o.boxes)
        # A compact desk drawer leaves a 105 mm cavity behind the front
        # panel when stood inner-face-up for repair. The inherited 250 mm
        # cavity blocked the real driver body before its bit reached the
        # screw, although the screw itself was still 20 mm away.
        for idx in (0,2,3):
            center,size,*tail=shallow_boxes[idx]
            shallow_boxes[idx]=((center[0],center[1],.135),
                                (size[0],size[1],.110),*tail)
        center,size,*tail=shallow_boxes[4]
        shallow_boxes[4]=((center[0],center[1],.185),size,*tail)
        o=replace(o,boxes=tuple(shallow_boxes),
                  sliders=(Slider('handle',(0,0,0),hb,travel=.0015,rate=200.,damping=1.,mass=.05,axis='Z',sign=1),))
    elif o.name.startswith('tool_support'):
        # 58 mm clear width accommodates the 52 mm rubber grip without spawn overlap.
        o=replace(o,boxes=(((0,0,.0225),(.070,.012,.045),DARK),
                          ((-.032,0,.048),(.006,.012,.015),DARK),
                          ((.032,0,.048),(.006,.012,.015),DARK)),size=(.070,.012,.0555))
    parts.append(o)
# Keep the working cross-bit lane unchanged for the validated insertion path.
# Move only the slotted distractor and its fixed rack/shape plaque out of the
# right arm's initial top-camera silhouette.
OBJECTS=[replace(o,pos=(o.pos[0]+WORKPIECE_SHIFT_X,o.pos[1]+WORKPIECE_SHIFT_Y,o.pos[2])) if
         o.name=='drawer_context' or o.name.startswith('drawer_support') else
         replace(o,pos=(o.pos[0],.05,o.pos[2])) if o.name in
         ('bit_flat','rest_bit_flat','type_label_flat') else
         replace(o,pos=(o.pos[0],.063,o.pos[2])) if o.name=='tool_backstop' else
         replace(o,pos=(o.pos[0],-.030-o.pos[1],o.pos[2])) if o.name.startswith('tool_support') else o for o in parts]
_prev_spawn=partkit._spawn_part

def powered_spawn(path,cfg,*args,**kwargs):
    result=_prev_spawn(path,cfg,*args,**kwargs)
    if cfg.part_name=='tool_support_0':
        # Visual recessed bed joins the two already-collidable U saddles.
        # Keep it non-colliding so the known pickup and insertion contacts
        # remain unchanged; the original saddles guide the physical return.
        from pxr import Gf,UsdGeom
        from isaaclab.sim.utils.stage import get_current_stage
        bed=UsdGeom.Cube.Define(get_current_stage(),path+'/body/recessed_bed')
        bed.CreateSizeAttr(1.)
        bed.AddTranslateOp().Set(Gf.Vec3d(0.,-.041,.0215))
        bed.AddScaleOp().Set(Gf.Vec3f(.095,.070,.043))
        bed.CreateDisplayColorAttr([Gf.Vec3f(.35,.38,.41)])
        return result
    if cfg.part_name not in ('driver','drawer_context','bit'):return result
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema,Sdf
    from isaaclab.sim.utils.stage import get_current_stage
    stage=get_current_stage();layer=stage.GetRootLayer()
    if cfg.part_name=='bit':
        from isaaclab.sim.utils import bind_physics_material
        for i in (1,2):stage.RemovePrim(path+f'/body/box_{i}')
        for i in (0,1):
            points=[]
            for z,length,width in ((.0925,.007,.0016),(.0955,.0055,.001)):
                for x,y in ((-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)):
                    points.append(Gf.Vec3f(y if i else x,x if i else y,z))
            faces=[[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
            if i:faces=[list(reversed(f)) for f in faces]
            m=UsdGeom.Mesh.Define(stage,path+f'/body/tapered_cross_{i}')
            m.CreatePointsAttr(points);m.CreateFaceVertexCountsAttr([4]*6);m.CreateFaceVertexIndicesAttr([v for f in faces for v in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(*GREY)])
            UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
            pc=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());pc.CreateContactOffsetAttr(.0001);pc.CreateRestOffsetAttr(0.)
            bind_physics_material(str(m.GetPath()),path+'/Looks/physics',stage=stage)
        return result

    def body(name,mass,inertia):
        bp=path+'/'+name
        g=UsdGeom.Xform.Define(stage,bp)
        UsdPhysics.RigidBodyAPI.Apply(g.GetPrim())
        ma=UsdPhysics.MassAPI.Apply(g.GetPrim());ma.CreateMassAttr(mass);ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        ph=PhysxSchema.PhysxRigidBodyAPI.Apply(g.GetPrim());ph.CreateAngularDampingAttr(.1)
        ph.CreateSolverPositionIterationCountAttr(128);ph.CreateSolverVelocityIterationCountAttr(4)
        return bp
    def transfer(name,dest):
        src=path+'/body/'+name
        Sdf.CopySpec(layer,src,layer,dest+'/'+name);stage.RemovePrim(src)
    def hinge(name,parent,child,anchor):
        j=UsdPhysics.RevoluteJoint.Define(stage,path+'/'+name)
        j.CreateAxisAttr('Z');j.CreateBody0Rel().SetTargets([Sdf.Path(parent)]);j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*anchor));j.CreateLocalPos1Attr(Gf.Vec3f(*anchor))
        return j
    if cfg.part_name=='driver':
        # Rubber contact faces only; the bit/socket material stays unchanged.
        from pxr import UsdShade
        from isaaclab.sim.utils import bind_physics_material
        grip_material=UsdShade.Material.Define(stage,path+'/grip_rubber')
        physics_material=UsdPhysics.MaterialAPI.Apply(grip_material.GetPrim())
        physics_material.CreateStaticFrictionAttr(1.5)
        physics_material.CreateDynamicFrictionAttr(1.3)
        spec=next(o for o in OBJECTS if o.name=='driver')
        for index in range(len(spec.boxes)-3,len(spec.boxes)):
            bind_physics_material(path+f'/body/box_{index}',path+'/grip_rubber',stage=stage)
        spindle=body('spindle',.04,(1e-5,1e-5,2e-5))
        j=hinge('motor',path+'/pressure',spindle,(0,0,.11))
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateStiffnessAttr(0.);d.CreateDampingAttr(.02);d.CreateMaxForceAttr(MOTOR_TORQUE_NM)
        for k in range(4):transfer('box_'+str(k),spindle)
        transfer('cyl_3',spindle)
        floor=UsdGeom.Cylinder(stage.GetPrimAtPath(spindle+'/cyl_3'))
        for op in floor.GetOrderedXformOps():
            if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(0,0,.1007))
        # Fixed housing ends behind the 4 mm moving stroke; no parallel axial load path.
        housing=UsdGeom.Cylinder(stage.GetPrimAtPath(path+'/body/cyl_0'))
        housing.GetHeightAttr().Set(.09)
        for op in housing.GetOrderedXformOps():
            if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(0,0,.045))
        for k in range(6):transfer('lead_in_'+str(k),spindle)
        # 6.45 mm across-flats, 30 mm long machined hex guide; full support prevents tip orbit.
        for i,k in enumerate((1,2,4,5)):
            a=k*math.pi/3;g=UsdGeom.Cube(stage.GetPrimAtPath(spindle+f'/box_{i}'))
            for op in g.GetOrderedXformOps():
                if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(.004225*math.cos(a),.004225*math.sin(a),.115))
                if op.GetOpType()==UsdGeom.XformOp.TypeScale:op.Set(Gf.Vec3f(.002,.006,.030))
        # The two jaw faces have guide sections above and below their moving pad windows.
        for sign in (-1,1):
            for z,h in ((.103,.006),(.121,.018)):
                dest=spindle+f'/guide_{sign+1}_{int(z*1000)}'
                Sdf.CopySpec(layer,spindle+'/box_0',layer,dest)
                g=UsdGeom.Cube(stage.GetPrimAtPath(dest))
                for op in g.GetOrderedXformOps():
                    if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(sign*.004225,0,z))
                    if op.GetOpType()==UsdGeom.XformOp.TypeScale:op.Set(Gf.Vec3f(.002,.006,h))
                    if op.GetOpType()==UsdGeom.XformOp.TypeOrient:op.Set(Gf.Quatf(1,0,0,0))
        for k in range(6):
            mesh=UsdGeom.Mesh(stage.GetPrimAtPath(spindle+f'/lead_in_{k}'))
            pts=mesh.GetPointsAttr().Get();a=k*math.pi/3;n=np.array([math.cos(a),math.sin(a)])
            moved=[]
            for v in pts:
                xy=np.array([v[0],v[1]])
                if v[2]<.125 and np.dot(xy,n)<.005:xy-=n*.000275
                moved.append(Gf.Vec3f(float(xy[0]),float(xy[1]),float(v[2]+.010)))
            mesh.GetPointsAttr().Set(moved)
        for label in ('jaw_plus','jaw_minus'):
            joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(path+'/'+label+'/'+label+'_slide'))
            joint.GetBody0Rel().SetTargets([Sdf.Path(spindle)])
    else:
        for k in (5,6,7):stage.RemovePrim(path+'/body/box_'+str(k))
        # The inherited close-up builder authored two decorative screw heads.
        # Remove the unused upper head, leaving exactly one workpiece fastener.
        for sx in (0,2):
            for sy in (0,2):stage.RemovePrim(path+f'/body/round_screw_1_{sx}_{sy}')
        stage.RemovePrim(path+'/body/screw_floor_1')
        for index,(label,y) in enumerate(SCREW_SITES.items()):
            # A separate axial feed carriage makes each visible screw head
            # advance into its own hole as it turns. It is kinematically
            # coupled to the bearing at ideal M4 pitch, not a helical mesh.
            carriage=body('screw_feed_'+label,.003,(2e-7,2e-7,2e-7))
            feed=UsdPhysics.PrismaticJoint.Define(stage,path+'/screw_feed_'+label+'_slide')
            feed.CreateAxisAttr('Z')
            feed.CreateBody0Rel().SetTargets([Sdf.Path(path+'/body')])
            feed.CreateBody1Rel().SetTargets([Sdf.Path(carriage)])
            feed.CreateLocalPos0Attr(Gf.Vec3f(FASTENER_X,y,.099))
            feed.CreateLocalPos1Attr(Gf.Vec3f(FASTENER_X,y,.099))
            feed.CreateLowerLimitAttr(-.0015);feed.CreateUpperLimitAttr(0.)
            screw=body('screw_'+label,.012,(1e-6,1e-6,1e-6))
            j=hinge('screw_bearing_'+label,carriage,screw,(FASTENER_X,y,.099))
            d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateStiffnessAttr(0.);d.CreateDampingAttr(1.);d.CreateMaxForceAttr(SCREW_RUNNING_RESISTANCE_NM)
            for sx in (0,2):
                for sy in (0,2):transfer(f'round_screw_{index}_{sx}_{sy}',screw)
            transfer(f'screw_floor_{index}',screw)
            j.CreateLowerLimitAttr(-1800.);j.CreateUpperLimitAttr(0.)
            mimic=PhysxSchema.PhysxMimicJointAPI.Apply(feed.GetPrim(),'rotZ')
            mimic.CreateReferenceJointRel().SetTargets([j.GetPath()])
            mimic.CreateGearingAttr(-.0007/360.);mimic.CreateOffsetAttr(0.)
            follower=stage.GetPrimAtPath(path+'/handle/handle_slide')
            c=PhysxSchema.PhysxMimicJointAPI.Apply(follower,'rotZ')
            c.CreateReferenceJointRel().SetTargets([j.GetPath()])
            c.CreateGearingAttr(.0007/360.);c.CreateOffsetAttr(0.)
    return result
partkit._spawn_part=powered_spawn
_prev_object=partkit._arena_object
def powered_object(spec):
    obj=_prev_object(spec)
    from isaaclab.actuators import ImplicitActuatorCfg
    if spec.name=='driver':
        from isaaclab_arena.utils.pose import Pose
        obj.initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(0.,-2**-.5,2**-.5,0.))
        obj._pose_event_cfg=obj._build_reset_event()
        obj.object_cfg.actuators['motor']=ImplicitActuatorCfg(joint_names_expr=['motor'],stiffness=0.,damping=.02,effort_limit_sim=MOTOR_TORQUE_NM,velocity_limit_sim=10.)
    elif spec.name=='drawer_context':
        obj.object_cfg.actuators['brake']=ImplicitActuatorCfg(joint_names_expr=['screw_bearing_.*'],stiffness=0.,damping=1.,effort_limit_sim=SCREW_RUNNING_RESISTANCE_NM)
    return obj
partkit._arena_object=powered_object
MECHANISM={'motor_enabled':False,'stable_steps':0,'tool_assembled':False,'assembly_steps':0,'score':0.}
_STILL=[]
_ASSEMBLY=[]
_LOAD=[]

def mechanism_state(environment):
    base=environment.unwrapped
    driver=base.scene['driver'];drawer=base.scene['drawer_context']
    def joint(a,n):return float(partkit._np(a.data.joint_pos)[0,a.joint_names.index(n)])
    return dict(pressure_m=-joint(driver,'pressure_slide'),
                handle_gap_m=.0015-joint(drawer,'handle_slide'),
                screw_feed_lower_m=-joint(drawer,'screw_feed_lower_slide'),
                screw_angle_lower_rad=joint(drawer,'screw_bearing_lower'),
                motor_angle_rad=joint(driver,'motor'),
                motor_velocity_rad_s=float(partkit._np(driver.data.joint_vel)[0,driver.joint_names.index('motor')]))
@partkit.on_reset
def reset_power(env):
    MECHANISM.clear();MECHANISM.update(motor_enabled=False,stable_steps=0,tool_assembled=False,assembly_steps=0,
                                       powered_lower_m=0.,score=0.,load_deflection_m=None)
    _STILL.clear();_ASSEMBLY.clear();_LOAD.clear()
@partkit.on_step
def update_power(env):
    import torch
    v=mechanism_state(env);on=MECHANISM['motor_enabled']
    driver=partkit._part_state(env,next(o for o in OBJECTS if o.name=='driver'))
    stand_home=np.array([.225,-.015,TOP+.067])
    docked=bool(np.linalg.norm(driver.pos-stand_home)<.025 and
                np.dot(driver.rotation()[:,2],[0,-1,0])>math.cos(math.radians(15)))
    # Push-start mode: the nose spring must be compressed while the bit tip is
    # seated in the screw recess. This geometric tip-contact proxy excludes
    # spring compression from carrying the tool or changing the bit.
    tip_contact=any(MECHANISM.get('tip_engaged',{}).values())
    if v['pressure_m']>.0015 and tip_contact and not docked:on=True
    if v['pressure_m']<.0008 or not tip_contact or docked:on=False
    a=env.unwrapped.scene['driver'];idx=a.joint_names.index('motor')
    a.set_joint_velocity_target(torch.tensor([[2*np.pi*MOTOR_RPM/60 if on else 0.]],device=a.device),joint_ids=[idx])
    # Credit only measured screw advance while the mounted bit and running spindle
    # are physically aligned with the recessed screw. A free bit turning the screw,
    # or the driver spinning elsewhere, produces no powered progress.
    bit=partkit._part_state(env,next(o for o in OBJECTS if o.name=='bit'))
    drawer=partkit._part_state(env,next(o for o in OBJECTS if o.name=='drawer_context'))
    from yam_arena import ik,station
    from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
    hand_tcp={}
    hand_open={}
    for arm in ('left','right'):
        robot=env.unwrapped.scene[arm+'_robot']
        q=partkit._np(robot.data.joint_pos)[0]
        hand_tcp[arm],_=ik.tcp_pose(q[[robot.joint_names.index(n) for n in ARM_JOINTS]],np.array(station.BASES[arm]))
        hand_open[arm]=float(np.clip(np.mean(q[[robot.joint_names.index(n) for n in FINGER_JOINTS]])/FINGER_OPEN,0.,1.))
    rel=driver.rotation().T@(bit.pos-driver.pos)
    nominal_mount=bool(np.linalg.norm(rel[:2])<.002 and .097<rel[2]<.106 and
                       abs(float(np.dot(driver.rotation()[:,2],bit.rotation()[:,2])))>math.cos(math.radians(10)))
    # The physical retaining jaws can flex sub-millimetres while the tool is
    # carried. Require the tight window to earn assembly, then use a wider
    # release threshold so micrometre-scale solver jitter cannot erase it.
    retained=bool(np.linalg.norm(rel[:2])<.0025 and .0965<rel[2]<.107 and
                  abs(float(np.dot(driver.rotation()[:,2],bit.rotation()[:,2])))>math.cos(math.radians(12)))
    mounted=bool(nominal_mount or (MECHANISM['tool_assembled'] and retained))
    tip=bit.pos+bit.rotation()@np.array([0,0,BIT_TIP_LOCAL_Z])
    screw_centers={label:drawer.pos+drawer.rotation()@np.array([FASTENER_X,y,.099-v['screw_feed_'+label+'_m']])
                   for label,y in SCREW_SITES.items()}
    axis_aligned=np.dot(bit.rotation()[:,2],-drawer.rotation()[:,2])>math.cos(math.radians(8))
    # The cross-shaped recess is a volume, not a point. Its petal top is 2 mm
    # above the head centre and the floor is 3 mm below it. A physically
    # inserted bit therefore leaves the old 0.5 mm centre sphere. Require
    # radial centring, depth inside the recess, axial alignment and matching
    # cross orientation relative to the *rotating* screw head.
    bit_in_drawer=drawer.rotation().T@bit.rotation()
    bit_yaw=math.atan2(bit_in_drawer[1,0],bit_in_drawer[0,0])
    engaged={}
    for label,center in screw_centers.items():
        local=drawer.rotation().T@(tip-center)
        relative_yaw=bit_yaw-v['screw_angle_'+label+'_rad']
        cross_error=(relative_yaw+math.pi/4)%(math.pi/2)-math.pi/4
        engaged[label]=bool(np.linalg.norm(local[:2])<.001 and
                            -.0025<local[2]<.0023 and axis_aligned and
                            abs(cross_error)<math.radians(15))
    # Assembly is a prior subgoal: the bit must remain in the stopped
    # chuck after the inserting gripper releases the bit, away from the screw.
    # Holding the tool housing is allowed; neither arm needs to return to ready.
    bit_grip=bit.pos+bit.rotation()@np.array([0.,0.,.03])
    bit_released=all(np.linalg.norm(tcp-bit_grip)>.06 for tcp in hand_tcp.values())
    assembly_candidate=(nominal_mount and not on and bit_released and
                        all(np.linalg.norm(tip-c)>.02 for c in screw_centers.values()))
    if assembly_candidate:_ASSEMBLY.append(rel.copy())
    else:_ASSEMBLY.clear()
    if len(_ASSEMBLY)>ASSEMBLY_HOLD_STEPS:del _ASSEMBLY[:-ASSEMBLY_HOLD_STEPS]
    assembly_stable=(len(_ASSEMBLY)==ASSEMBLY_HOLD_STEPS and
                     max(np.linalg.norm(x-_ASSEMBLY[0]) for x in _ASSEMBLY)<=.0005)
    MECHANISM['assembly_steps']=len(_ASSEMBLY)
    if assembly_stable:MECHANISM['tool_assembled']=True
    if not mounted:
        MECHANISM['tool_assembled']=False
        MECHANISM['powered_lower_m']=0.
    MECHANISM['bit_released']=bool(bit_released)
    previous_gap=MECHANISM.get('handle_gap_m',.0015)
    previous_motor=MECHANISM.get('motor_angle_rad',v['motor_angle_rad'])
    signed_closure=previous_gap-v['handle_gap_m']
    closure=max(0.,signed_closure)
    # The reported revolute angle may wrap at a full turn; one control step
    # cannot physically traverse pi radians at the configured speed.
    motor_delta=(v['motor_angle_rad']-previous_motor+math.pi)%(2*math.pi)-math.pi
    motor_advance=max(0.,motor_delta)*.0007/(2*math.pi)
    for label in SCREW_SITES:
        angle_key='screw_angle_'+label+'_rad';credit_key='powered_'+label+'_m';feed_key='screw_feed_'+label+'_m'
        old_angle=MECHANISM.get(angle_key,0.)
        old_feed=MECHANISM.get(feed_key,0.)
        screw_advance=max(0.,old_angle-v[angle_key])*.0007/(2*math.pi)
        feed_advance=max(0.,v[feed_key]-old_feed)
        if (MECHANISM['tool_assembled'] and on and mounted and engaged[label]
                and screw_advance>0 and feed_advance>0 and motor_advance>0):
            # The entire required feed must occur after assembly through
            # corresponding spindle, screw, feed and knob closure motion.
            increment=min(feed_advance,screw_advance,motor_advance,closure)
            MECHANISM[credit_key]+=increment
        # Net physical feed and screw angle cap credit, removing gains from
        # undo/re-tighten cycles and unpowered axial pushes.
        net=min(max(0.,v[feed_key]),max(0.,-v[angle_key])*.0007/(2*math.pi))
        net=min(net,max(0.,.0015-v['handle_gap_m']))
        MECHANISM[credit_key]=min(MECHANISM[credit_key],net)
    MECHANISM.update(v,motor_enabled=on,dock_interlock=docked,bit_mounted=mounted,tip_engaged=engaged)
    # Final score needs measured closure and a released, stopped tool in the stand.
    st=driver
    home=np.array([.225,-.015,TOP+.067])
    home_quat=np.array([0.,-2**-.5,2**-.5,0.])
    tool_quat=np.asarray(st.quat_xyzw)
    orientation_error=2*math.acos(min(1.,abs(float(np.dot(tool_quat,home_quat)))))
    stand=bool(np.linalg.norm(st.pos-home)<.025 and orientation_error<math.radians(8.))
    velocity=partkit._np(a.data.root_vel_w)[0]
    speed=np.linalg.norm(velocity[:3]);angular_speed=np.linalg.norm(velocity[3:])
    hands_clear=True
    grip_center=st.pos+st.rotation()@np.array([0,0,.05])
    for tcp in hand_tcp.values():hands_clear &= np.linalg.norm(tcp-grip_center)>.06
    screw_center=screw_centers['lower']
    tip_clear=np.linalg.norm(tip-screw_center)>.010
    knob_center=drawer.pos+drawer.rotation()@np.array([FASTENER_X,-.06,.052])
    knob_clear=all(np.linalg.norm(tcp-knob_center)>.100 for tcp in hand_tcp.values())
    powered_complete=(MECHANISM['tool_assembled'] and mounted and
                      MECHANISM['powered_lower_m']>=POWERED_FEED_REQUIRED_M and
                      v['screw_feed_lower_m']>=POWERED_FEED_REQUIRED_M and
                      -v['screw_angle_lower_rad']*.0007/(2*math.pi)>=POWERED_FEED_REQUIRED_M and
                      v['handle_gap_m']<=.0002)
    # Apply a small opening load to the movable knob, after the tool and
    # hands withdraw. This tests the same joint that the screw must retain.
    load_candidate=powered_complete and tip_clear and knob_clear
    drawer_asset=env.unwrapped.scene['drawer_context']
    handle_ids=[i for i,n in enumerate(drawer_asset.body_names) if n=='handle']
    if len(handle_ids)!=1:raise RuntimeError(f'Expected one loadable knob body, got {drawer_asset.body_names}')
    load=torch.zeros((1,1,3),device=drawer_asset.device)
    if load_candidate:
        load[0,0,:]=torch.as_tensor(-TEST_LOAD_N*drawer.rotation()[:,2],device=drawer_asset.device)
    drawer_asset.set_external_force_and_torque(load,torch.zeros_like(load),body_ids=handle_ids,is_global=True)
    if load_candidate:
        if not _LOAD:MECHANISM['load_origin_gap_m']=v['handle_gap_m']
        _LOAD.append(v['handle_gap_m'])
    else:
        _LOAD.clear()
        MECHANISM.pop('load_origin_gap_m',None)
    if len(_LOAD)>FINAL_HOLD_STEPS:_LOAD.pop(0)
    deflection=max([0.]+[gap-MECHANISM['load_origin_gap_m'] for gap in _LOAD]) if _LOAD else None
    load_pass=(len(_LOAD)>=FINAL_HOLD_STEPS and deflection is not None and
               deflection<=MAX_LOAD_DEFLECTION_M and max(_LOAD)<=.0002)
    MECHANISM.update(load_steps=len(_LOAD),load_deflection_m=deflection,
                     load_pass=bool(load_pass),tip_clear=bool(tip_clear),knob_clear=bool(knob_clear))
    eligible=(load_pass and not on and stand and hands_clear and
              all(opening>=.8 for opening in hand_open.values()))
    MECHANISM.update(tool_in_stand=bool(stand),tool_orientation_error_deg=math.degrees(orientation_error),
                     hands_clear=bool(hands_clear),
                     hands_open=hand_open,tool_speed=float(speed),tool_angular_speed=float(angular_speed))
    if eligible:
        _STILL.append((st.pos.copy(),np.asarray(st.quat_xyzw).copy(),v['motor_angle_rad']))
    else:_STILL.clear()
    count=round(1./env.unwrapped.step_dt)+1
    if len(_STILL)>count:del _STILL[:-count]
    stable=False
    if len(_STILL)==count:
        displacement=max(np.linalg.norm(x[0]-_STILL[0][0]) for x in _STILL)
        rotation=max(2*math.acos(min(1.,abs(float(np.dot(x[1],_STILL[0][1]))))) for x in _STILL)
        spindle=np.unwrap([x[2] for x in _STILL])
        excursion=float(np.max(spindle)-np.min(spindle))
        stable=displacement<=.002 and rotation<=math.radians(1.) and excursion<=.02
        MECHANISM.update(rest_displacement_m=float(displacement),rest_rotation_deg=math.degrees(rotation),spindle_excursion_rad=excursion)
    MECHANISM['stable_steps']=len(_STILL)
    # Reward completed milestones, not distance, rotation or elapsed time.
    MECHANISM['score']=score_from_evidence(
        assembly_pass=bool(MECHANISM['tool_assembled'] and mounted),
        load_pass=bool(load_pass),return_pass=bool(stable and eligible))
    MECHANISM['valid_powered_progress']={'lower':float(MECHANISM['powered_lower_m'])}
    MECHANISM['assembly_pass']=bool(MECHANISM['tool_assembled'] and mounted)
def score(state):return MECHANISM['score']
def success(state):return score(state)==1.
