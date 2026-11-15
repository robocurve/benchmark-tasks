"""Quasistatic wheel and geometrically engaged spoke-key approximation.

Measured key rotation changes spoke rest length only while its slot surrounds
the nipple and its plane is aligned with the shaft. An elastic ring/spoke
energy solve determines the resulting rim and tensions. No target shape or
correct nipple setting is supplied. Thread torque, fast vibration and rim
plasticity are omitted; excessive deformation/turns invalidate the trial.
The robot and free wrench remain native rigid bodies. The wheel is rendered
and collided as kinematic sectors following this material model. A near rim
pinch drives its axle rotation; after release, viscous bearing drag slows it.
"""
import math
import numpy as np
from mechanics import WheelModel, starting_turn_errors
from wheel_scene import x_frame, quat, spoke_phase, wheel_world_frame, NIPPLE_CENTER_INSET_M
from thread_coupling import rotation, body_paths, TensorNativeIO


def configure(stage, metadata):
    from pxr import UsdPhysics, PhysxSchema, Sdf
    root=metadata['hub_path'].rsplit('/wheel/',1)[0]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if path.startswith(root+'/') and prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
    paths=body_paths(metadata)
    for path in paths:
        if path!=metadata['wrench_path']:
            UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr(True)
            PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path)).CreateEnableCCDAttr(False)
    # Slot contact is represented by the geometric engagement law below.
    # Keep native collisions with the rim, stand, table, and robot.
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(metadata['wrench_path'])).CreateFilteredPairsRel().SetTargets(
        [Sdf.Path(s['nipple_path']) for s in metadata['spokes']])
    metadata['material_model']='quasistatic-elastic-ring-and-geometric-thread-v1'


class Model:
    def __init__(self, metadata):
        self.meta=metadata
        self.elastic=WheelModel()
        self.reset()

    def reset(self):
        self.turns=starting_turn_errors(profile=self.meta['reset_profile'])
        self.initial=self.turns.copy()
        self.angle=self.meta['wheel_clocking_rad']
        self.omega=0.
        self.free_rotation=0.
        self.hand_angle={}
        self.engaged=None
        self.previous_tool_vector=None
        self.events=[]
        self.time=0.
        self.valid=True
        self.failure=None
        self.result=self.elastic.equilibrium(self.turns)

    def geometry(self):
        W=wheel_world_frame(self.angle)
        centre=np.array(self.meta['axle_position_m'])
        vertices=self.result['positions_m']
        frames={self.meta['hub_path']:(centre,W)}
        for i,s in enumerate(self.meta['spokes']):
            a=np.array(s['hub_anchor_local_m']); b=vertices[i]; d=b-a
            r=x_frame(d); rim=(b+vertices[(i+1)%32])/2
            rR=x_frame(vertices[(i+1)%32]-b)
            frames[s['rim_path']]=(centre+W@rim,W@rR)
            frames[s['inner_path']]=(centre+W@(a+d*.25),W@r)
            frames[s['outer_path']]=(centre+W@(a+d*.75),W@r)
            n=b-r[:,0]*NIPPLE_CENTER_INSET_M
            phase=self.meta['nipple_flat_phase_rad']-2*math.pi*(self.turns[i]-self.initial[i])
            frames[s['nipple_path']]=(centre+W@n,W@r@spoke_phase(phase))
        return frames

    def step(self, tool_position, tool_rotation, hands, dt):
        self.time+=dt
        centre=np.array(self.meta['axle_position_m'])
        holding=False
        for side,tcp,gap in hands:
            v=np.asarray(tcp)-centre
            radius=np.linalg.norm(v[1:])
            # Fingers meet the outer220mm rim surface; the TCP can sit
            # outside it while the16mm pad tips straddle the rim section.
            contact=(gap<.035 and abs(v[0])<.024 and abs(radius-.220)<.020)
            angle=math.atan2(v[2],v[1])
            if contact:
                holding=True
                if side in self.hand_angle:
                    delta=(angle-self.hand_angle[side]+math.pi)%(2*math.pi)-math.pi
                    self.angle+=delta
                    self.omega=.8*self.omega+.2*delta/dt
                else:self.omega=0.
                self.hand_angle[side]=angle
            else:self.hand_angle.pop(side,None)
        if not holding:
            delta=self.omega*dt
            self.angle+=delta
            self.free_rotation+=abs(delta)
            self.omega*=math.exp(-.08*dt)
        else:self.free_rotation=0.
        frames=self.geometry()
        candidate=None
        local=None
        for i,s in enumerate(self.meta['spokes']):
            p,R=frames[s['nipple_path']]
            q=tool_rotation.T@(p-tool_position)
            aligned=abs(float(np.dot(R[:,0],tool_rotation[:,2])))>.96
            if aligned and abs(q[0])<.0025 and abs(q[2])<.004 and -.0035<q[1]<.0035:
                candidate=i;local=q;break
        if self.engaged is not None:
            p,R=frames[self.meta['spokes'][self.engaged]['nipple_path']]
            q=tool_rotation.T@(p-tool_position)
            aligned=abs(float(np.dot(R[:,0],tool_rotation[:,2])))>.94
            if aligned and abs(q[0])<.004 and abs(q[2])<.006 and -.005<q[1]<.009:
                candidate=self.engaged;local=q
            else:
                self.events.append(dict(time_s=self.time,event='disengaged',spoke=self.engaged))
                self.engaged=None;self.previous_tool_vector=None
        if candidate is not None:
            p,R=frames[self.meta['spokes'][candidate]['nipple_path']]
            axis=R[:,0]
            vector=tool_rotation[:,0]-axis*np.dot(tool_rotation[:,0],axis)
            vector/=np.linalg.norm(vector)
            phase=self.meta['nipple_flat_phase_rad']-2*math.pi*(self.turns[candidate]-self.initial[candidate])
            reference=R@spoke_phase(-phase)
            material_vector=reference.T@vector
            if self.engaged==candidate:
                delta=math.atan2(np.cross(self.previous_tool_vector,material_vector)[0],
                                 np.dot(self.previous_tool_vector,material_vector))
                self.turns[candidate]-=delta/(2*math.pi)
            else:
                # The square flats must fit before rotation can transmit.
                alignment=max(abs(float(np.dot(vector,R[:,1]))),abs(float(np.dot(vector,R[:,2]))))
                if alignment<math.cos(math.radians(18)):
                    return
                self.engaged=candidate
                self.events.append(dict(time_s=self.time,event='engaged',spoke=candidate,slot_offset_m=local.tolist()))
            self.previous_tool_vector=material_vector.copy()
            if np.max(abs(self.turns))>3:
                self.valid=False;self.failure='Nipple exceeded three-turn thread travel'
                return
            self.result=self.elastic.equilibrium(self.turns)
            if self.result['maximum_displacement_ratio']>.02:
                self.valid=False;self.failure='Rim outside small-deformation material range'

    def record(self):
        r=self.result
        return dict(turns=self.turns.tolist(),tensions_n=r['tension_n'].tolist(),
            lateral_runout_m=r['lateral_peak_to_peak_m'],radial_runout_m=r['radial_peak_to_peak_m'],
            dish_m=r['dish_offset_m'],angle_rad=self.angle,omega_rad_s=self.omega,
            released_rotation_rad=self.free_rotation,engaged_spoke=self.engaged,
            valid=self.valid,failure=self.failure,events=list(self.events))


class Runtime:
    def __init__(self, base, metadata, tensor_view):
        import torch
        self.base=base;self.meta=metadata
        self.simulation_view=tensor_view
        self.io=TensorNativeIO(tensor_view,body_paths(metadata))
        self.model=Model(metadata)
        self.indices=torch.tensor([self.io.index[p] for p in self.model.geometry()],device=self.io.device,dtype=torch.int32)
        self.paths=list(self.model.geometry())
        from yam_arena.constants import FINGER_JOINTS
        self.hands=[(side,base.scene[side+'_robot'],list(base.scene[side+'_robot'].body_names).index('gripper'),
                     base.scene[side+'_robot'].find_joints(list(FINGER_JOINTS),preserve_order=True)[0]) for side in ('left','right')]

    def reset(self):self.model.reset()

    def step(self):
        import torch
        from yam_arena.constants import TCP_OFFSET
        self.io.begin_step()
        p,R=self.io.frame(self.meta['wrench_path'])
        hands=[]
        for side,robot,index,fingers in self.hands:
            pose=robot.data.body_link_pose_w.torch[0,index].detach().cpu().numpy()
            tcp=pose[:3]+rotation(pose[3:])@np.asarray(TCP_OFFSET)
            gap=float(robot.data.joint_pos.torch[0,fingers].abs().sum())
            hands.append((side,tcp,gap))
        self.model.step(p,R,hands,float(self.base.physics_dt))
        if not self.model.valid:raise RuntimeError(self.model.failure)
        frames=self.model.geometry()
        targets=self.io.view.get_transforms().clone()
        for path in self.paths:
            p,R=frames[path]
            targets[self.io.index[path]]=torch.as_tensor(np.r_[p,quat(R)],device=targets.device,dtype=targets.dtype)
        self.io.view.set_kinematic_targets(targets,self.indices)


def install(base,stage):
    from pxr import UsdUtils
    import omni.physics.tensors as tensors
    import arena_wheel
    cache=UsdUtils.StageCache.Get();stage_id=cache.GetId(stage).ToLongInt()
    if stage_id<0:cache.Insert(stage);stage_id=cache.GetId(stage).ToLongInt()
    view=tensors.create_simulation_view('torch',stage_id);view.set_subspace_roots('/')
    item=arena_wheel.ASSEMBLIES['wheel_station']
    runtime=Runtime(base,item['metadata'],view)
    original=base.scene.write_data_to_sim
    def before(*args,**kwargs):
        out=original(*args,**kwargs);runtime.step();return out
    base.scene.write_data_to_sim=before
    return runtime
