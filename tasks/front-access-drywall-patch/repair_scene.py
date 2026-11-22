"""Front-only repair prototype. All moving stock and both tools are native rigid bodies."""
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation as R
from surface_process_probe import WORLD, body, box
import rotary_driver as driver
from tool_profile import TIP as NOSE, TRIGGER
from wall_driver import build as build_driver, PITCH as CARRIER_PITCH, POCKETS
from grading import bite, backing_rows

FRONT, BOARD, DEPTH = .350, .0127, .089
from yam_arena.station import TABLE_TOP
MID = TABLE_TOP + .172
PATCH, LENGTH, WIDTH, THICK = .197, .251, .025, .016
STOCK = np.array([.248,-.066,TABLE_TOP+.126])
SECOND = np.array([.296,.158,TABLE_TOP+.070])
RACK_LIFT = .042
PATCH_START = np.array([.350,-.323,TABLE_TOP+.100+RACK_LIFT])
HANDLE = np.array([.180,.390,TABLE_TOP+.026])
HANDLE_Z = .010
GRIP_Y = -.065
ENTRY_BOTTOM,ENTRY_TOP,ENTRY_HALF_WIDTH=-.0145,-.0055,.0045
CARRIER_HOME = PATCH_START + [-.044,0,HANDLE_Z]
PICK = CARRIER_HOME + [0,GRIP_Y,0]
SCREW = .030
PITCH = .003175
BACKERS = ('strip','spare_strip')
BODIES = [*BACKERS,'patch','carrier','driver','spindle','shoe','trigger','screw_carrier']
FIXED = []
WOOD=(.70,.47,.26)
PAPER=(.80,.84,.82)
STAND=(.31,.34,.36)
CURRENT = None


def build(g,stage):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    FIXED.clear()
    def fixed(name,p,d,c):
        FIXED.append(name)
        box(stage,WORLD+'/'+name,p,d,c)
    def part(name,xyz,mass):
        path=WORLD+'/'+name
        body(stage,path,xyz,mass)
        return lambda n,p,d,c:box(stage,path+'/'+n,p,d,c)
    def panel(n,y0,y1,z0,z1):
        fixed(n,(FRONT+BOARD/2,(y0+y1)/2,(z0+z1)/2),(BOARD,y1-y0,z1-z0),PAPER)
    for n,a,b in [('jamb_minus',-.2025,-.10),('jamb_plus',.10,.2025)]:
        panel(n,a,b,TABLE_TOP,TABLE_TOP+.350)
    panel('header',-.10,.10,MID+.10,TABLE_TOP+.350)
    panel('sill',-.10,.10,TABLE_TOP,MID-.10)
    rear=FRONT+BOARD
    fixed('back',(rear+DEPTH+BOARD/2,0,TABLE_TOP+.175),(BOARD,.405,.35),(.69,.63,.55))
    for i,s in enumerate((-1,1)):
        fixed('stud'+str(i),(rear+DEPTH/2,s*.1935,TABLE_TOP+.175),(DEPTH,.018,.35),WOOD)
    for i,h in enumerate((.009,.341)):
        fixed('cap'+str(i),(rear+DEPTH/2,0,TABLE_TOP+h),(DEPTH,.369,.018),WOOD)
    for name,stock in zip(BACKERS,(STOCK,SECOND)):
        bench=(g in ('repair-tool','repair-brake','repair-lower','repair-lower-pair') and name=='strip') or g in ('repair-panel','repair-finish','repair-sites','repair-far')
        location=np.array([FRONT+BOARD+THICK/2,0,MID+(.066 if name=='strip' else -.044)]) if bench else stock
        wood=part(name,location,.049)
        if bench:
            clamp=UsdPhysics.FixedJoint.Define(stage,WORLD+'/test_only_clamp_'+name)
            clamp.CreateBody1Rel().SetTargets([WORLD+'/'+name])
            clamp.CreateLocalPos0Attr(Gf.Vec3f(*map(float,location)))
        wood('pine',(0,0,0),(THICK,LENGTH,WIDTH),WOOD)
        if name=='strip':
            # Load the loose timber into an open-ended cantilever rack. Two
            # bearing faces at the far end balance gravity. Sliding 31 mm along
            # the wood releases the upper lip before lifting from the shelf.
            h=stock[2]-TABLE_TOP-WIDTH/2
            fixed('strip_pedestal',(stock[0],stock[1]-.094,TABLE_TOP+h/2),(.022,.041,h),STAND)
            fixed('strip_shelf',(stock[0],stock[1]-.093,stock[2]-WIDTH/2-.002),(.030,.063,.004),STAND)
            fixed('strip_upright',(stock[0]+.0135,stock[1]-.114,stock[2]),(.006,.012,.038),STAND)
            fixed('strip_keeper',(stock[0],stock[1]-.114,stock[2]+WIDTH/2+.002),(.030,.009,.003),STAND)
            continue
        for i,s in enumerate((-1,1)):
            h=stock[2]-TABLE_TOP-WIDTH/2
            offset=s*.074 if name=='spare_strip' else (-.101,.044)[i]
            fixed(name+'rest'+str(i),(stock[0],stock[1]+offset,TABLE_TOP+h/2),(.022,.012,h),STAND)
            if name=='spare_strip':
                for side in (-1,1):
                    fixed(f'stock_saddle_{i}_{side+1}',(stock[0]+side*.0107,stock[1]+offset,stock[2]-WIDTH/2+.002),
                          (.003,.012,.006),STAND)
        if name=='spare_strip':
            for side in (-1,1):
                fixed('stock_end_'+str(side+1),(stock[0],stock[1]+side*.1283,stock[2]-WIDTH/2+.002),
                      (.014,.003,.006),STAND)
    # A detachable button-stud handle fits two physical keyhole slots.
    # Its engaged sliding guide releases when the heads enter both larger apertures.
    panel_at=np.array([FRONT+BOARD/2,0,MID]) if g in ('repair-sites','repair-far') else PATCH_START
    patch=part('patch',panel_at,.39)
    if g in ('repair-sites','repair-far'):
        clamp=UsdPhysics.FixedJoint.Define(stage,WORLD+'/test_only_clamp_patch')
        clamp.CreateBody1Rel().SetTargets([WORLD+'/patch'])
        clamp.CreateLocalPos0Attr(Gf.Vec3f(*map(float,panel_at)))
    color=(.91,.88,.82)
    for row,lo,hi,half_gap in [('bottom',-PATCH/2,HANDLE_Z+ENTRY_BOTTOM,0),('entry',HANDLE_Z+ENTRY_BOTTOM,HANDLE_Z+ENTRY_TOP,ENTRY_HALF_WIDTH),('slot',HANDLE_Z+ENTRY_TOP,HANDLE_Z+.0065,.00155),('top',HANDLE_Z+.0065,PATCH/2,0)]:
        bands=[(-PATCH/2,PATCH/2)] if half_gap==0 else [(-PATCH/2,-.038-half_gap),(-.038+half_gap,.038-half_gap),(.038+half_gap,PATCH/2)]
        for i,(a,b) in enumerate(bands):
            patch(row+str(i),(0,(a+b)/2,(lo+hi)/2),(BOARD,b-a,hi-lo),color)
    hook=part('carrier',panel_at+[-.044,0,HANDLE_Z],.072)
    grip_frame=R.from_euler('y',28,degrees=True)
    def grip_shape(n,offset,size):
        shape=hook(n,grip_frame.apply(offset)+[0,GRIP_Y,0],size,(.19,.37,.43))
        driver.orient(shape,grip_frame)
        # Same nominal rubber-on-pad coefficients used by the station for
        # graspable rubber objects. All forces still come from native contact.
        material=UsdPhysics.MaterialAPI.Get(stage,shape.GetPath().AppendChild('friction'))
        material.GetStaticFrictionAttr().Set(1.2)
        material.GetDynamicFrictionAttr().Set(1.)
    # A forked handling adapter supports both jaw pads on a central web.
    # Fingers enter the two mouths along their approach axis before closing.
    # The cap, stops and side walls are physical colliders on the carrier.
    grip_shape('grip',(-.006,0,0),(.062,.048,.036))
    for i,side in enumerate((-1,1)):
        grip_shape('pad_cap'+str(i),(-.003,0,side*.040),(.041,.056,.004))
        grip_shape('entry_rib'+str(i),(-.029,0,side*.0195),(.004,.046,.003))
        grip_shape('socket_side'+str(i),(-.002,side*.028,0),(.042,.004,.082))
    grip_shape('socket_end',(.020,0,0),(.004,.056,.082))
    hook('bar',(.020,0,0),(.012,.246,.008),driver.STEEL)
    for i,s in enumerate((-1,1)):
        hook('pin'+str(i),(.040,s*.038,.0032),(.024,.0018,.0018),driver.STEEL)
        hook('toe'+str(i),(.0519,s*.038,.0032),(.0018,.004,.004),driver.STEEL)
    for i,s in enumerate((-1,1)):
        fixed('patchfoot'+str(i),(PATCH_START[0],PATCH_START[1]+s*.074,TABLE_TOP+(.0013+RACK_LIFT)/2),(.031,.018,.0013+RACK_LIFT),STAND)
        fixed('patchback'+str(i),(PATCH_START[0]+.016,PATCH_START[1]+s*.074,TABLE_TOP+(.082+RACK_LIFT)/2),(.011,.014,.082+RACK_LIFT),STAND)
        fixed('patchlip'+str(i),(PATCH_START[0]-.014,PATCH_START[1]+s*.074,TABLE_TOP+(.01+RACK_LIFT)/2),(.007,.014,.01+RACK_LIFT),STAND)
    for i,sign in enumerate((-1,1)):
        h=CARRIER_HOME[2]-.004-TABLE_TOP
        fixed('carrierrest'+str(i),(CARRIER_HOME[0]+.02,CARRIER_HOME[1]+sign*.116,TABLE_TOP+h/2),(.011,.010,h),STAND)
        hook('pad'+str(i),(.034,sign*.027,0),(.006,.015,.054),driver.RUBBER)
        hook('strap'+str(i),(.028,sign*.027,0),(.014,.008,.009),driver.STEEL)
    # Captured button studs slide vertically in the two narrow keyhole necks.
    # Their common prismatic guide models the sub-millimetre flange clearance.
    # Lowering the carrier into both large apertures releases the guide.
    key=driver.joint(stage,UsdPhysics.PrismaticJoint,'paired_keyhole_guides','patch','carrier',(-.044,0,HANDLE_Z),(0,0,0))
    key.GetAxisAttr().Set('Z');key.CreateLowerLimitAttr(-.014);key.CreateUpperLimitAttr(.002)
    detent=UsdPhysics.DriveAPI.Apply(key.GetPrim(),'linear')
    detent.CreateTypeAttr('force');detent.CreateStiffnessAttr(170.);detent.CreateDampingAttr(1.2)
    detent.CreateMaxForceAttr(2.6);detent.CreateTargetPositionAttr(.0006)
    motor,feed_rail=build_driver(stage,HANDLE)
    global CURRENT
    CURRENT=Repair(stage,motor)
    CURRENT.rail=feed_rail
    CURRENT.braked=g=='repair-brake'
    CURRENT.key=key
    CURRENT.key_release=None
    return CURRENT


def local(pose,point):
    return R.from_quat(pose[3:]).inv().apply(point-pose[:3])


def embedded_shear(offset,velocity,normal,depth):
    """Lateral bearing of a rotating screw that has already entered the board."""
    lateral=offset-normal*(offset@normal)
    speed=velocity-normal*(velocity@normal)
    if depth<.0009 or np.linalg.norm(lateral)>.007:
        return np.zeros(3)
    stiffness=min(4100.,1500.+190000.*depth)
    limit=min(18.,4.+950.*depth)
    reaction=-(stiffness*lateral+3.5*speed)
    return reaction*min(1.,limit/max(1e-12,np.linalg.norm(reaction)))


class Repair:
    def __init__(self,stage,motor):
        from fastener_load import Connections
        self.connections=Connections()
        self.stage,self.motor=stage,motor
        self.fasteners=[]
        self.log=[]
        self.last_rotation=None
        self.stroke=None
        self.rounds=POCKETS
        self.pocket=0
        self.spent=False
        self.indexing=False
        self.magazine_events=[]
        self.powered=False
        self.score=0.
        self.hold=0.
        self.hold_samples=0
        self.panel_samples=0
        from putaway import Touchdown
        self.touchdown=Touchdown()
        self.park_samples=0
        self.park_reference=None
        self.park_indices=None
        self.loaded_steps=[]
        self.delivered_proof=[]
        self.cutting_torque=0.
        self.cutting_wood=None
        self.peak_torque=0.
        self.bearing_samples=0
        self.peak_bearing=0.
        self.proof_on=False
        self.proof_complete=False
        self.reference_corners=None
        self.last_metrics={}
        self.diagnostics={}

    def install_load(self,sim):
        import torch
        patch=sim.create_rigid_body_view(WORLD+'/patch')
        shoe=sim.create_rigid_body_view(WORLD+'/shoe')
        rotor=sim.create_rigid_body_view(WORLD+'/spindle')
        shell=sim.create_rigid_body_view(WORLD+'/driver')
        device=patch.get_transforms().device
        idx=torch.tensor([0],dtype=torch.int32,device=device)
        def external_wrench():
            wrench=np.zeros((3,6))
            if self.cutting_torque:
                axis=R.from_quat(rotor.get_transforms().cpu().numpy()[0,3:]).apply([1.,0.,0.])
                speed=(rotor.get_velocities()-shell.get_velocities()).cpu().numpy()[0,3:] @ axis
                torque=self.cutting_torque*np.clip(speed/12.,-1.,1.)*axis
                self.peak_torque=max(self.peak_torque,float(np.linalg.norm(torque)))
                t=torch.as_tensor((-torque)[None],device=device,dtype=torch.float32)
                rotor.apply_forces_and_torques_at_position(None,t,None,idx,True)
                if self.cutting_wood is not None:
                    work = max(0., float(torque @ axis) * speed) / 240
                    table = self.stroke.setdefault('wood_work_J', {})
                    table[self.cutting_wood] = table.get(self.cutting_wood, 0.) + work
                    wrench[BACKERS.index(self.cutting_wood),3:]+=torque
                elif self.stroke['front']=='patch':
                    wrench[2,3:]+=torque
            s=self.stroke
            if s is not None and self.powered and s['depth']>=.0009:
                sh=shoe.get_transforms().cpu().numpy()[0]
                v=shoe.get_velocities().cpu().numpy()[0]
                shoe_arm=R.from_quat(sh[3:]).apply(NOSE)
                at=sh[:3]+shoe_arm
                speed=v[:3]+np.cross(v[3:],shoe_arm)
                if s['front']=='patch':
                    panel=patch.get_transforms().cpu().numpy()[0]
                    pv=patch.get_velocities().cpu().numpy()[0]
                    pr=R.from_quat(panel[3:])
                    arm=pr.apply(s['local'])
                    anchor=panel[:3]+arm
                    normal=pr.apply([1.,0.,0.])
                    speed=speed-pv[:3]-np.cross(pv[3:],arm)
                else:
                    anchor=s['point']
                    normal=np.array([1.,0.,0.])
                force_vec=embedded_shear(at-anchor,speed,normal,s['depth'])
                if abs((at-anchor)@normal)<.002 and np.any(force_vec):
                    self.bearing_samples+=1
                    self.peak_bearing=max(self.peak_bearing,float(np.linalg.norm(force_vec)))
                    side=torch.as_tensor(force_vec[None],device=device,dtype=torch.float32)
                    point=torch.as_tensor(at[None],device=device,dtype=torch.float32)
                    shoe.apply_forces_and_torques_at_position(side,None,point,idx,True)
                    if s['front']=='patch':
                        wrench[2]+=np.r_[-force_vec,np.cross(at-panel[:3],-force_vec)]
            if self.proof_on:
                wrench[2,0]+=2.
            return wrench if np.any(wrench) else None

        def record_delivery(sample,internal,summed,submitted):
            if self.proof_on:
                # Record the exact float32 force row handed to the native body
                # view and the independently computed fastener reaction.
                self.loaded_steps.append(sample)
                self.delivered_proof.append(dict(sample=sample,
                    fasteners_N=internal[2,:3].tolist(),submitted_N=submitted.cpu().numpy().tolist()))
        self.connections.bind(sim,external_wrench,record_delivery)

    def consume(self,event,sample):
        from pxr import UsdGeom, UsdPhysics
        self.spent=True
        self.rounds-=1
        screw_shape = self.stage.GetPrimAtPath(WORLD+f'/screw_carrier/screw_{self.pocket}')
        UsdPhysics.CollisionAPI(screw_shape).GetCollisionEnabledAttr().Set(False)
        UsdGeom.Imageable(screw_shape).MakeInvisible()
        self.magazine_events.append(dict(event=event,pocket=self.pocket+1,sample=sample,remaining=self.rounds))

    def fasten(self,point,axis,wood,front,poses):
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
        i=len(self.fasteners)
        self.connections.insert(front,wood,point,poses)
        # Head flush with the front paper, with a visible cross-slot.
        anchor=WORLD+'/patch' if front=='patch' else WORLD
        c=local(poses['patch'],point) if front=='patch' else point
        head=UsdGeom.Cylinder.Define(self.stage,anchor+'/installed_head_'+str(i))
        head.CreateAxisAttr('X');head.CreateHeightAttr(.0004);head.CreateRadiusAttr(.00413)
        head.AddTranslateOp().Set(Gf.Vec3d(*map(float,c+[-.00018,0,0])))
        head.CreateDisplayColorAttr([(.19,.20,.22)])
        for k,d in enumerate(((.00016,.0048,.0008),(.00016,.0008,.0048))):
            box(self.stage,anchor+f'/ph2_{i}_{k}',c+[-.00045,0,0],d,(.08,.09,.10),collision=False)
        self.fasteners.append(dict(wood=wood,front=front,head=point.tolist(),axis=axis.tolist(),
                             wood_work_J=self.stroke.get('wood_work_J',{}).get(wood,0.),turns=self.stroke['turns'],travel_m=self.stroke['depth'],sample=self.sample))

    def update(self,rows,forces,paths,filters,sample,hz):
        self.sample=sample
        poses={name.rsplit('/',1)[-1]:p for name,p in zip(paths,rows)}
        carrier_offset=local(poses['patch'],poses['carrier'][:3])
        slide=carrier_offset[2]-HANDLE_Z
        if self.key_release is None and -.0148<slide<-.0112 and abs(carrier_offset[1])<.0018:
            self.key.GetJointEnabledAttr().Set(False)
            self.key_release=dict(sample=sample,slide_mm=1000*slide)
        p,sh,sp=(poses[n] for n in ('driver','shoe','spindle'))
        rot=R.from_quat(p[3:]);axis=rot.apply([1,0,0])
        face=sh[:3]+R.from_quat(sh[3:]).apply(NOSE)
        stroke=-float((sh[:3]-p[:3])@axis)
        relative=rot.inv()*R.from_quat(sp[3:])
        angle=0. if self.last_rotation is None else (self.last_rotation.inv()*relative).as_rotvec()[0]
        self.last_rotation=relative
        ids={s.rsplit('/',1)[-1]:i for i,s in enumerate(filters)}
        shoe=paths.index(WORLD+'/shoe')
        loads={n:max(0.,float(-np.mean([f[shoe,ids[n]]@axis for f in forces]))) for n in ('patch','jamb_minus','jamb_plus','header','sill')}
        front=max(loads,key=loads.get)
        load=loads[front]
        unloaded=stroke<.0008 and max(float(np.linalg.norm(f[shoe,ids[k]])) for f in forces
             for k in ('patch','jamb_minus','jamb_plus','header','sill'))<.03
        rail=-local(sh,poses['screw_carrier'][:3])[1]
        switch=TRIGGER[0]-local(p,poses['trigger'][:3])[0]
        if unloaded:
            if self.stroke is not None and self.stroke['depth']>=.0009 and not self.spent:
                self.consume('partial withdrawal',sample)
            self.stroke=None
            if self.spent and self.rounds and not self.indexing:
                self.pocket+=1
                self.rail.GetTargetPositionAttr().Set(-CARRIER_PITCH*self.pocket)
                self.indexing=True
                self.magazine_events.append(dict(event='index requested',pocket=self.pocket+1,sample=sample))
        aligned=abs(rail-CARRIER_PITCH*self.pocket)<.00045
        if self.indexing and aligned and unloaded:
            self.spent=False
            self.indexing=False
            self.magazine_events.append(dict(event='arrived',pocket=self.pocket+1,sample=sample,rail_mm=1000*rail))
        face_error=abs(local(poses['patch'],face)[0]+BOARD/2) if front=='patch' else abs(face[0]-FRONT)
        loaded=load>.35 and axis[0]>.98 and stroke>.001 and face_error<.0012
        running=loaded and switch>=.002 and aligned and not self.spent and not self.indexing and self.rounds>0 and not self.braked
        self.cutting_torque=0.
        self.cutting_wood=None
        if running and self.stroke is not None:
            depth=self.stroke['depth']
            if depth>BOARD:
                self.cutting_wood=next((n for n in BACKERS if bite(poses[n],face,axis)),None)
            fraction=np.clip((depth-BOARD)/THICK,0.,1.) if self.cutting_wood else 0.
            self.cutting_torque=.025+.30*fraction
        if running!=self.powered:
            self.motor.GetTargetVelocityAttr().Set(1980. if running else 0.)
            self.powered=running
        if running:
            if self.stroke is None:
                self.stroke=dict(point=face.copy(),local=local(poses['patch'],face) if front=='patch' else None,front=front,turns=0.,depth=0.)
            s=self.stroke
            entry=poses['patch'][:3]+R.from_quat(poses['patch'][3:]).apply(s['local']) if s['front']=='patch' else s['point']
            if s['front']==front and np.linalg.norm((entry-face)[1:])<.005:
                # A turn can cut only while the tip is fed. Spinning against
                # a stationary depth stop cannot bank turns for a later shove.
                s['turns']+=angle/(2*np.pi)
                s['depth']=float(np.clip(s['depth']+angle/(2*np.pi)*PITCH,0,max(0.,stroke+.0004)))
                if s['depth']>=SCREW-.0006:
                    self.consume('driven',sample)
                    self.motor.GetTargetVelocityAttr().Set(0.)
                    self.powered=False
                    self.cutting_torque=0.
                    point=face.copy()
                    wood=None
                    for name in BACKERS:
                        if bite(poses[name],point,axis):
                            wood=name;break
                    if wood and s.get('wood_work_J',{}).get(wood,0.)>=1.0 and all(np.linalg.norm(point-f['head'])>.011 for f,m in zip(self.fasteners,self.connections.mounts) if not m['broken']):
                        self.fasten(point,axis,wood,front,poses)
                    self.log.append(dict(sample=sample,front=front,wood=wood,point=point.tolist(),depth=s['depth'],turns=s['turns']))
        self.diagnostics=dict(pocket=self.pocket+1,rail_mm=1000*rail,switch_mm=1000*switch,aligned=bool(aligned),stroke_mm=round(stroke*1000,2),load=round(load,2),face=face.round(4).tolist(),axis=axis.round(3).tolist(),cut_mm=round(1000*self.stroke['depth'],2) if self.stroke else 0.,remaining=self.rounds,key_slide_mm=round(1000*slide,2),carrier_offset_x_mm=round(1000*carrier_offset[0],2),key_released=self.key_release is not None)
        # Evaluate all four exterior corners of the released patch.
        patch=poses['patch'];q=R.from_quat(patch[3:])
        corners=patch[:3]+q.apply([[-BOARD/2,y,z] for y in (-PATCH/2,PATCH/2) for z in (-PATCH/2,PATCH/2)])
        flush=float(np.max(np.abs(corners[:,0]-FRONT)))
        seam=float(max(np.max(np.abs(np.abs(corners[:,1])-.10)),np.max(np.abs(np.abs(corners[:,2]-MID)-.10))))
        inside=(np.max(np.abs(corners[:,1]))<=.1005 and np.max(np.abs(corners[:,2]-MID))<=.1005)
        rows, panel_rows, secure = backing_rows(self.fasteners, self.connections.mounts, MID)
        toolfree=not any(np.any(f[k,ids[n]]) for f in forces for k in [paths.index(WORLD+'/'+name) for name in (*BACKERS,'patch')] for n in ('driver','shoe','carrier','spindle','trigger','screw_carrier'))
        fingers=[i for i,n in enumerate(filters) if '/LeftArm/' in n or '/RightArm/' in n]
        unheld=all(not np.any(f[k,fingers]) for f in forces for k in [paths.index(WORLD+'/'+n) for n in (*BACKERS,'patch')])
        released = self.key_release is not None and toolfree and unheld
        seated = inside and flush < .002 and seam <= .0035
        self.panel_samples = self.panel_samples+len(forces) if rows==2 and panel_rows==2 and seated and released else 0
        good = secure and seated and released
        if good and self.reference_corners is None:
            self.reference_corners=corners.copy()
        drift=float(np.max(np.linalg.norm(corners-self.reference_corners,axis=1))) if self.reference_corners is not None else 0.
        if not good or drift>.001:
            self.hold=0.
            self.hold_samples=0
            self.proof_on=False
            self.proof_complete=False
            self.reference_corners=None
        else:
            self.hold_samples+=len(forces)
            self.hold=self.hold_samples/hz
            if hz//2<=self.hold_samples<3*hz//2:
                self.proof_on=True
            elif self.proof_on:
                self.proof_complete=True
                self.proof_on=False
        self.score = .25*rows
        if self.panel_samples >= hz:
            self.score = .75
        if self.hold_samples >= 5*hz//2 and self.proof_complete:
            self.score = 1.
        self.last_metrics=dict(fasteners=len(self.fasteners),flush_mm=1000*flush,max_joint_gap_mm=1000*seam,backing_rows=rows,panel_rows=panel_rows,released=bool(released),hold_s=self.hold,proof_complete=self.proof_complete,drift_mm=1000*drift,score=self.score)

    def measure_park(self,pose_batch,force_batch,paths,filters,hz):
        if self.park_indices is None:
            bodies=[paths.index(WORLD+'/'+name) for name in ('driver','shoe','spindle','trigger','screw_carrier')]
            table=filters.index(WORLD+'/table/geometry/mesh')
            robot=[i for i,n in enumerate(filters) if '/LeftArm/' in n or '/RightArm/' in n]
            right=[i for i in robot if '/RightArm/' in filters[i] and '/tip_' in filters[i]]
            self.park_indices=(bodies,table,robot,right,paths.index(WORLD+'/carrier'))
        bodies,table,robot,right,handle=self.park_indices
        f=np.asarray(force_batch)
        load=f[:,bodies,table,2].sum(axis=1)
        pads=np.linalg.norm(f[:,bodies[0],right],axis=-1)
        self.diagnostics['right_on_handle']=bool(np.any(f[:,handle,right]))
        self.diagnostics['right_lift_N']=float(f[:,bodies[0],right,2].sum(axis=1).mean())
        self.diagnostics.update(table=bool(np.max(load)>.15),table_load=float(load.mean()),
             right_pads=bool(np.all(pads>.1)),carrier_table=float(f[:,handle,table,2].mean()))
        if not self.fasteners or len(self.fasteners)<8:
            return
        pose=np.asarray(pose_batch)
        shells=pose[:,bodies[0]]
        rotation=R.from_quat(shells[:,3:])
        switch=TRIGGER[0]-rotation.inv().apply(pose[:,bodies[3],:3]-shells[:,:3])[:,0]
        fingers=[i for i in robot if '/tip_' in filters[i]]
        grip=np.any(np.linalg.norm(f[:,bodies[0],fingers],axis=-1)>.1,axis=1)
        self.touchdown.update(load,grip,switch<.002,hz,self.sample-len(f))
        good=self.score==1.
        if good and self.touchdown.seen and np.all(load>1.) and not np.any(f[:,bodies][:,:,robot]):
            if self.park_reference is None:self.park_reference=(shells[0,:3].copy(),rotation[0])
            anchor,att=self.park_reference
            steady=np.max(np.linalg.norm(shells[:,:3]-anchor,axis=1))<=.002
            steady=steady and np.max((att.inv()*rotation).magnitude())<=np.deg2rad(2.)
            if steady and np.all(switch<.002):
                self.park_samples+=len(f)
            else:
                self.park_samples=0
                self.park_reference=None
        else:
            self.park_samples=0
            self.park_reference=None
        if good and self.park_samples<hz:self.score=.75
        self.last_metrics.update(score=self.score,parked_s=self.park_samples/hz,controlled_touchdown=self.touchdown.seen,landing=self.touchdown.event)

    def save(self,out):
        (out/'repair-result.json').write_text(json.dumps(dict(metrics=self.last_metrics,magazine_events=self.magazine_events,remaining=self.rounds,shots=self.log,fasteners=self.fasteners,key_release=self.key_release,connections=[{k:v for k,v in m.items() if k in ('broken','gap','max_gap','peak_force','peak_moment')} for m in self.connections.mounts],peak_cutting_torque_Nm=self.peak_torque,embedded_screw_bearing=dict(samples=self.bearing_samples,peak_N=self.peak_bearing),proof_force_world_N=[2.,0.,0.],proof_substeps=self.loaded_steps,delivered_proof=self.delivered_proof),indent=2)+'\n')
