"""Read-only physical geometry for staged ordinary robot control."""
import math
import numpy as np
from trajectory_control import bounded_joint_pid_step
from yam_arena.urdf import quat_xyzw_to_matrix
from collision_geometry import minimum_separation
from geometry import object_obstacles,plug_boxes,box_triangles,wire_local_frame,GUIDE_BOTTOM
from config import *


def snapshot(read_pose):
    return ({i:np.asarray(read_pose(f'plug_{i}')) for i in USED},
            {i:np.asarray(read_pose(f'clip_{i}')) for i in USED},
            {(i,k):np.asarray(read_pose(f'wire_{i}_{k}')) for i in USED for k in range(LINKS)})


def gripper_sweep(geometry,fingers,tcp,rotation,*,end_fingers=None,translation=None):
    start=geometry.pieces(fingers)
    end=start if end_fingers is None else geometry.pieces(end_fingers)
    delta=np.zeros(3) if translation is None else np.asarray(translation)
    return [np.concatenate((a@rotation.T+tcp,b@rotation.T+tcp+delta)) for (_,a),(_,b) in zip(start,end)]


def capsule_gap(triangles,center,axis,height,radius,margin=.005):
    """Positive projection gap certifies distance from the exact capsule.

    Capsule support is c·d + |axis·d|h/2+r for unit d. No OBB corner can
    falsely become a cable contact. Zero only means absence is uncertified.
    """
    if not all(np.isfinite(value).all() for value in (triangles,center,axis,height,radius)):return 0.
    vertices=np.unique(triangles.reshape(-1,3),axis=0)
    extent=np.abs(axis)*height/2+radius
    gap=float(np.maximum(vertices.min(0)-(center+extent),(center-extent)-vertices.max(0)).max())
    if gap>margin:return gap
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    edges=np.concatenate((triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,1],triangles[:,0]-triangles[:,2]))
    nearest=center+np.clip((vertices-center)@axis,-height/2,height/2)[:,None]*axis
    axes=np.concatenate((np.eye(3),normals,np.cross(edges,axis),vertices-nearest))
    lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-12]/lengths[lengths>1e-12,None]
    axes=np.unique(np.round(axes,8),axis=0);axes/=np.linalg.norm(axes,axis=1)[:,None]
    for start in range(0,len(axes),128):
        directions=axes[start:start+128];project=vertices@directions.T
        middle=directions@center;support=np.abs(directions@axis)*height/2+radius
        gap=max(gap,float(np.maximum(project.min(0)-(middle+support),(middle-support)-project.max(0)).max()))
        if gap>margin:return gap
    return max(0.,gap)


def wire_clearance(parts,wire_poses,margin=.005):
    pmin=np.array([p.min((0,1)) for p in parts]);pmax=np.array([p.max((0,1)) for p in parts])
    centers=[];axes=[];heights=[]
    for (i,k),pose in wire_poses.items():
        local,height=wire_local_frame(i,k);rotation=quat_xyzw_to_matrix(pose[3:])
        centers.append(pose[:3]);axes.append((rotation@local)[:,2]);heights.append(height)
    centers=np.asarray(centers);axes=np.asarray(axes);heights=np.asarray(heights)
    extent=np.abs(axes)*heights[:,None]/2+CABLE_RADIUS
    gaps=np.maximum(pmin[:,None]-(centers+extent)[None,:],(centers-extent)[None,:]-pmax[:,None]).max(2)
    for i,j in np.argwhere(gaps<=margin):gaps[i,j]=capsule_gap(parts[i],centers[j],axes[j],heights[j],CABLE_RADIUS,margin)
    return float(gaps.min())


def clearance(parts,state,*,exclude=(),wire_margin=.005):
    plugs,clips,wires=state
    if not all(np.isfinite(value).all() for value in (*parts,*plugs.values(),*clips.values(),*wires.values())):
        return {'rigid_gap_m':0.,'wire_gap_m':0.,'minimum_gap_m':0.,'invalid':'nonfinite_geometry'}
    rigid=[triangles for owner,name,triangles in object_obstacles(plugs,clips,include_wires=False)
           if (owner,name) not in exclude]
    rg=minimum_separation(parts,rigid);wg=wire_clearance(parts,wires,wire_margin)
    return {'rigid_gap_m':rg,'wire_gap_m':wg,'minimum_gap_m':min(rg,wg)}


def boot_cylinder(plug,count=64):
    angles=np.arange(count)*2*np.pi/count;radius=.0035/np.cos(np.pi/count)
    rings=np.array([np.column_stack((np.full(count,x),radius*np.cos(angles),radius*np.sin(angles))) for x in (-.007,.007)])
    triangles=[]
    for i in range(count):
        j=(i+1)%count
        triangles.extend(((rings[0,i],rings[0,j],rings[1,j]),(rings[0,i],rings[1,j],rings[1,i]),
                          ((-.007,0,0),rings[0,j],rings[0,i]),((.007,0,0),rings[1,i],rings[1,j])))
    rotation=quat_xyzw_to_matrix(plug[3:])
    return (np.asarray(triangles)+[-.0205,0,-.0005])@rotation.T+plug[:3]


class BootJawPredictor:
    """First ordinary-pad/boot contact for each independent measured jaw."""
    def __init__(self,geometry):
        zero=geometry.pieces([0.,0.]);moved=geometry.pieces([-.001,-.001]);self.parts=[]
        for name in ('tip_left','tip_right'):
            self.parts.append([(a,(b[0,0]-a[0,0])/-.001) for (n,a),(_,b) in zip(zero,moved) if n==name])

    def predict(self,plug,tcp,rotation,actual_fingers,command_opening=.0015):
        boot=boot_cylinder(np.asarray(plug));stops=[];actual_gaps=[];possible=[]
        for i,pieces in enumerate(self.parts):
            def gap(opening):
                return minimum_separation([(a-opening*slope)@rotation.T+tcp for a,slope in pieces],[boot])
            low,high=0.,.0065
            possible.append(gap(command_opening)<=1e-6 and gap(.0065)>1e-6)
            for _ in range(14):
                middle=(low+high)/2
                if gap(middle)>1e-6:high=middle
                else:low=middle
            stops.append((low+high)/2);actual_gaps.append(gap(abs(actual_fingers[i])))
        return {'opening_m':stops,'actual_surface_gap_m':actual_gaps,'contact_possible':possible}

    def surface_gaps(self,plug,tcp,rotation,actual_fingers):
        boot=boot_cylinder(np.asarray(plug))
        return [minimum_separation([(a+float(actual_fingers[i])*slope)@rotation.T+tcp for a,slope in pieces],[boot])
                for i,pieces in enumerate(self.parts)]


class BootJawContactCache:
    """Cache first-contact roots, never the current measured surface check."""
    def __init__(self,predictor):
        self.predictor=predictor;self.last_key=None;self.cached=None;self.timestamp=None;self.full_evaluations=0

    @staticmethod
    def key(plug,tcp,rotation,command_opening):
        body=quat_xyzw_to_matrix(np.asarray(plug[3:]))
        return (body.T@(tcp-plug[:3]),body.T@rotation,float(command_opening))

    def unchanged(self,key):
        if self.last_key is None:return False
        p,r,command=key;old_p,old_r,old_command=self.last_key
        angle=math.acos(float(np.clip((np.trace(r.T@old_r)-1)/2,-1,1)))
        return bool(np.linalg.norm(p-old_p)<=.000025 and angle<=math.radians(.03) and command==old_command)

    def evaluate(self,plug,tcp,rotation,fingers,time,command_opening,*,fresh=False):
        key=self.key(plug,tcp,rotation,command_opening);hit=self.unchanged(key) and not fresh
        if not hit:
            self.cached=self.predictor.predict(plug,tcp,rotation,fingers,command_opening=command_opening)
            self.last_key=(key[0].copy(),key[1].copy(),key[2]);self.timestamp=float(time);self.full_evaluations+=1
        result=dict(self.cached)
        if hit:result['actual_surface_gap_m']=self.predictor.surface_gaps(plug,tcp,rotation,fingers)
        result.update(cache_hit=bool(hit),cache_timestamp_s=self.timestamp,full_evaluations=self.full_evaluations,
                      cache_key={'tcp_in_boot_m':self.last_key[0].tolist(),'hand_rotation_in_boot':self.last_key[1].tolist(),
                                 'command_opening_m':self.last_key[2]})
        return result


def closing_opening(age,entry_opening,hold_opening):
    """Reach the final millimetre gently, then retain one hold command."""
    def smooth(value):
        value=float(np.clip(value,0.,1.));return value*value*(3.-2.*value)
    near=min(entry_opening,hold_opening+.001)
    if age<2.:return entry_opening+(near-entry_opening)*smooth(age/2.)
    return near+(hold_opening-near)*smooth((age-2.)/2.)


def predicted_jaw_normal_forces(openings,command_opening,axis_vertical=1.):
    """Quasistatic PD+finger-gravity estimate, not a contact-force sensor."""
    gravity=.069657*9.81*axis_vertical
    spring=5000.*(np.asarray(openings)-command_opening)
    return [float(spring[0]-gravity),float(spring[1]+gravity)]


def boot_contact_force_gate(openings,command_opening,axis_vertical=1.):
    """The original 0.75 N obstruction margin, with each finger's gravity."""
    forces=predicted_jaw_normal_forces(openings,command_opening,axis_vertical)
    return {'normal_estimate_N':forces,'minimum_normal_N':.75,
            'blocked':bool(np.isfinite(forces).all() and min(forces)>.75)}


class EndpointCorrection:
    """Small Cartesian integral, enabled only at a measured settled endpoint."""
    def __init__(self):self.value=np.zeros(3)
    def reset(self):self.value[:]=0.
    def update(self,error,dt,enabled):
        if enabled:
            step=2.*dt*np.asarray(error);length=float(np.linalg.norm(step));limit=.00005*min(1.,dt/.05)
            if length>limit:step*=limit/length
            self.value+=step
            length=float(np.linalg.norm(self.value))
            if length>.0025:self.value*=.0025/length
        return self.value.copy()


class JointEndpointCorrection:
    """Fixed unbiased joint reference with a bounded outer integral."""
    def __init__(self):self.reset()
    def reset(self):
        self.reference=None;self.bias=np.zeros(6);self.frozen=False;self.invalid=None
        self.damping=np.zeros(6);self.integral_mask=np.ones(6,dtype=bool)
    def latch(self,reference,*,preserve_bias=False):
        reference=np.asarray(reference,dtype=float)
        if reference.shape!=(6,) or not np.isfinite(reference).all():raise ValueError('nonfinite or invalid joint endpoint reference')
        self.reference=reference.copy();self.reference.setflags(write=False)
        if not preserve_bias:self.bias[:]=0.
        self.damping[:]=0.
        self.frozen=False;self.invalid=None
    def freeze(self):self.frozen=True;self.damping[:]=0.
    def update(self,actual,dt,allowed=True):
        self.damping[:]=0.
        if self.reference is None or self.frozen or not allowed:return
        actual=np.asarray(actual,dtype=float)
        if not math.isfinite(dt) or dt<=0:
            self.invalid='nonfinite or invalid endpoint timestep';return
        if actual.shape!=(6,) or not np.isfinite(actual).all():
            self.invalid='nonfinite or invalid measured joint state';return
        step=.5*dt*(self.reference-actual);limit=.00025*min(1.,dt/.05)
        self.bias=np.clip(self.bias+np.clip(step,-limit,limit),-.02,.02)
    def command(self):
        return None if self.reference is None else self.reference+np.clip(self.bias+self.damping,-.02,.02)

    def track(self,reference,actual,dt,lower,upper,velocity=None):
        """Trajectory PI-D; native velocity damping remains outside I state."""
        reference=np.asarray(reference,dtype=float);actual=np.asarray(actual,dtype=float)
        if (reference.shape!=(6,) or actual.shape!=(6,) or not math.isfinite(dt) or dt<=0
                or not np.isfinite(reference).all() or not np.isfinite(actual).all()):
            self.invalid='nonfinite joint trajectory reference/state/timestep';return
        try:
            proposed,damping,total,mask=bounded_joint_pid_step(self.bias,reference,actual,velocity,dt)
        except ValueError as error:self.invalid=str(error);return
        step=proposed-self.bias
        candidate=reference+total
        # Do not integrate farther into joint or per-action saturation.
        saturated=((step>0)&(candidate>upper))|((step<0)&(candidate<lower))
        raw_total=proposed+damping
        saturated|=((step>0)&(raw_total>.02))|((step<0)&(raw_total<-.02))
        step=np.where(saturated,0.,step)
        self.reference=reference.copy();self.reference.setflags(write=False)
        self.bias=np.clip(self.bias+step,-.02,.02);self.frozen=False;self.invalid=None
        self.damping=damping;self.integral_mask=mask & ~saturated


def mesh_pose_distance(pieces,first,second):
    """Maximum displacement of every unmodified gripper mesh vertex."""
    return max(float(np.linalg.norm(a@first[1].T+first[0]-(a@second[1].T+second[0]),axis=2).max()) for _,a in pieces)


def plug_fit(plug,target):
    """Actual solid footprint predicted at engagement, excluding latch ramp."""
    rotation=quat_xyzw_to_matrix(np.asarray(plug[3:]));center=np.asarray(plug[:3]).copy();center[0]=port(target)[0]
    target_y,target_z=port(target)[1:];side=.0065;floor=target_z-.0045;roof=target_z+.0068
    gaps={}
    for name,local,size in plug_boxes():
        vertices=(box_triangles(local,size)@rotation.T+center).reshape(-1,3)
        gaps[name+'_side']=float(side-np.abs(vertices[:,1]-target_y).max())
        gaps[name+'_floor']=float(vertices[:,2].min()-floor)
        gaps[name+'_ceiling']=float((target_z+GUIDE_BOTTOM if name.startswith('shoulder') else roof)-vertices[:,2].max())
    return {'minimum_clearance_m':min(gaps.values()),'clearances_m':gaps,
            'axis_angle_deg':math.degrees(math.acos(float(np.clip(rotation[0,0],-1,1)))),
            'lateral_error_m':float(np.linalg.norm(np.asarray(plug[1:3])-np.asarray(port(target)[1:3])))}


def retaining_face(clip,target):
    """Actual clip-body retaining face, never reconstructed from the plug."""
    if not np.isfinite(clip).all():return {'retained':False,'vertices_w':None,'y_overlap_m':0.,'z_overlap_m':0.,'invalid':'nonfinite_clip'}
    rotation=quat_xyzw_to_matrix(np.asarray(clip[3:]));vertices=np.array([(.006,y,z) for y in (-.0025,.0025) for z in (.0005,.0025)])@rotation.T+clip[:3]
    _,y,z=port(target);lower=vertices.min(0);upper=vertices.max(0)
    y_overlap=float(min(upper[1],y+.00275)-max(lower[1],y-.00275))
    z_overlap=float(min(upper[2],z+.007)-max(lower[2],z+.005))
    retained=bool(lower[0]>.409-.0002 and upper[0]<.423 and abs(vertices[:,1].mean()-y)<.003
                  and z+.005<upper[2]<z+.0075 and y_overlap>0 and z_overlap>0)
    return {'retained':retained,'vertices_w':vertices.tolist(),'y_overlap_m':y_overlap,'z_overlap_m':z_overlap}


def plug_hand_target(position,rotation,actual_plug,tcp,hand_rotation):
    body=quat_xyzw_to_matrix(np.asarray(actual_plug[3:]));offset=body.T@(tcp-actual_plug[:3]);relative=body.T@hand_rotation
    return np.asarray(position)+rotation@offset,rotation@relative
