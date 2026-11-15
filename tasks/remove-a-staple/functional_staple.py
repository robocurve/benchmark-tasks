"""Quasistatic clinch-straightening model driven by the actual remover fork.

This replaces unstable microscopic steel joints. A fork must enter beneath
the crown, then lift it. As wire is drawn through the original holes, its
under-paper tails shorten and straighten, conserving centreline length. A
crosswise pull before both legs clear damages the paper; ordinary sliding
along the fork during leverage is permitted. The released staple
rests on the upward-facing fork until it tips off. Steel bending forces are
not fed back to the remover; direct finger extraction is not modelled.
"""
import math
import numpy as np
from materials import clinched_staple_centerline


def rotation(q):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(q).as_matrix()


def resample(points, distances):
    lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
    cumulative=np.r_[0,np.cumsum(lengths)]
    # Preserve the authored material coordinates, including the small bend
    # discretisation error, through a uniformly scaled arc-length parameter.
    u=np.asarray(distances)/distances[-1]*cumulative[-1]
    return np.column_stack([np.interp(u,cumulative,points[:,i]) for i in range(3)])


class Staple:
    def __init__(self,origin,stack=.0004):
        self.origin=np.array(origin,float);self.stack=stack
        initial=clinched_staple_centerline(paper_stack_m=stack)
        self.distances=np.r_[0,np.cumsum(np.linalg.norm(np.diff(initial,axis=0),axis=1))]
        self.height=0.;self.engaged=False;self.extracted=False;self.damaged=False
        self.crown=self.origin+[0,0,.00015]
        self.rotation=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
        self.local_crown=None;self.events=[];self.time=0.;self.velocity=np.zeros(3)

    def points(self):
        if not self.extracted:
            # Two equal6mm legs; bend travels down the material as it is drawn
            # through each hole. Radius0.35mm is an explicit coarse approximation.
            vertical=min(.006,self.stack+.0002+self.height)
            radius=min(.00035,max(0.,(.006-vertical)/(math.pi/2)))
            tail=max(0.,.006-vertical-radius*math.pi/2)
            leg=[[-.0064+radius+tail,0,-vertical-radius]]
            for angle in np.linspace(-math.pi/2,-math.pi,7):
                leg.append([-.0064+radius+radius*math.cos(angle),0,-vertical+radius*math.sin(angle)])
            leg.append([-.0064,0,0])
            right=[[-x,y,z] for x,y,z in reversed(leg)]
            line=np.array(leg+right)
            return resample(line,self.distances)@self.rotation.T+self.crown
        line=np.array([[-.0064,0,-.006],[-.0064,0,0],[.0064,0,0],[.0064,0,-.006]])
        return resample(line,self.distances)@self.rotation.T+self.crown

    def step(self,position,R,dt):
        self.time+=dt
        tip=position+R@np.array([0.,.024,0.000025])
        local=(self.crown-position)@R
        if not self.engaged and not self.extracted:
            under=(-.00065<=tip[2]-(self.crown[2]-.00015)<=.00012)
            aligned=abs(float(R[1,0]))>.96 and float(R[2,2])>.94
            within=abs(tip[1]-self.crown[1])<.0018 and abs(tip[0]-self.crown[0])<.0012
            if under and aligned and within:
                self.engaged=True;self.local_crown=local.copy()
                self.local_rotation=R.T@self.rotation
                self.events.append(dict(time_s=self.time,event='fork_under_crown'))
        if self.engaged:
            proposed=position+R@self.local_crown
            if not self.extracted:
                # The crown slides along the wedge as the heel pivots. It is
                # NOT glued to the first contact point. Intersect the authored
                # fork's top plane with the fixed needle-hole axis.
                slope=(.00005-.003)/.029
                intercept=.00005-slope*.025
                normal=R@np.array([0.,-slope,1.])
                if normal[2]<.35:
                    self.engaged=False
                    return
                z=(float(normal@position)+intercept-normal[0]*self.origin[0]-normal[1]*self.origin[1])/normal[2]
                support=np.array([self.origin[0],self.origin[1],z])
                at=(support-position)@R
                # Both tines must still overlap the12.8mm crown. Sliding along
                # their29mm length is ordinary lever contact, not paper damage.
                if abs(at[0])>.003:
                    self.damaged=True;self.engaged=False
                    self.events.append(dict(time_s=self.time,event='crosswise_pull',offset_m=float(at[0])))
                    return
                if not -.004<=at[1]<=.026 or z<self.crown[2]-.001:
                    self.engaged=False;self.events.append(dict(time_s=self.time,event='fork_withdrawn'))
                else:
                    lift=max(0.,z-self.origin[2])
                    self.height=max(self.height,lift)
                    self.crown=np.array([self.origin[0],self.origin[1],self.origin[2]+.00015+self.height])
                    if self.height>.0065:
                        self.extracted=True;self.local_crown=(self.crown-position)@R
                        self.events.append(dict(time_s=self.time,event='legs_clear'))
            else:
                self.crown=proposed
                # A crown seated across the two fork tines follows their frame.
                self.rotation=R@self.local_rotation
                if R[2,2]<.35:
                    self.engaged=False;self.events.append(dict(time_s=self.time,event='tipped_off_fork'))
        elif self.extracted:
            self.velocity[2]-=9.81*dt;self.crown+=self.velocity*dt
            from full_scene import TABLE_TOP
            from coordinate_frame import OFFSET_M
            floor=TABLE_TOP+OFFSET_M[2]+.00615
            if self.crown[2]<floor:self.crown[2]=floor;self.velocity[:]=0.

    def record(self):
        return dict(extracted=self.extracted,engaged=self.engaged,damaged=self.damaged,
            crown_m=self.crown.tolist(),lift_m=self.height,events=list(self.events))


def configure(stage,metadata):
    from pxr import UsdPhysics,PhysxSchema,Sdf
    paths=metadata['staple']['paths']
    for path in paths:
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr(True)
        PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path)).CreateEnableCCDAttr(False)
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(metadata['root']+'/staple/') and prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
    # Wire/paper and wire/fork contact are the quasistatic law, while paper,
    # remover, table and fingers retain native dynamics and collisions.
    for path in paths:
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path+'/steel')).CreateCollisionEnabledAttr(False)


def install(base,metadata):
    import torch,omni.physics.tensors as tensors
    from scipy.spatial.transform import Rotation
    from full_scene import STAPLE_XY
    from coordinate_frame import OFFSET_M
    import omni.usd
    from pxr import UsdUtils
    stage=omni.usd.get_context().get_stage();cache=UsdUtils.StageCache.Get()
    stage_id=cache.GetId(stage).ToLongInt()
    if stage_id<0:cache.Insert(stage);stage_id=cache.GetId(stage).ToLongInt()
    sim=tensors.create_simulation_view('torch',stage_id);sim.set_subspace_roots('/')
    paths=metadata['staple']['paths']
    view=sim.create_rigid_body_view(paths+[metadata['remover_path']]);ids={p:i for i,p in enumerate(view.prim_paths)}
    indices=torch.tensor([ids[p] for p in paths],dtype=torch.int32,device=base.device)
    origin=np.array([*STAPLE_XY,metadata['reset_layout']['crown_underside_z']])+OFFSET_M
    model=Staple(origin,2*metadata['paper_material'].thickness_m)
    model.simulation_view=sim
    original=base.scene.write_data_to_sim
    def step(*args,**kwargs):
        out=original(*args,**kwargs)
        poses=view.get_transforms();tool=poses[ids[metadata['remover_path']]].detach().cpu().numpy()
        model.step(tool[:3],rotation(tool[3:]),float(base.physics_dt))
        points=model.points();centres=(points[:-1]+points[1:])/2;delta=np.diff(points,axis=0)
        # The rod stays planar while passing through paper; out-of-plane
        # loading is a damage condition, not a simulated torsion mechanism.
        for i,path in enumerate(paths):
            x=delta[i]/np.linalg.norm(delta[i]);y=model.rotation[:,1].copy();y-=np.dot(y,x)*x;y/=np.linalg.norm(y)
            R=np.column_stack((x,y,np.cross(x,y)))
            poses[ids[path]]=torch.as_tensor(np.r_[centres[i],Rotation.from_matrix(R).as_quat()],device=poses.device,dtype=poses.dtype)
        view.set_kinematic_targets(poses,indices)
        return out
    base.scene.write_data_to_sim=step
    return model
