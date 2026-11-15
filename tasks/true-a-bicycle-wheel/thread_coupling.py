"""Measured-angle screw material coupling for the authored native wheel.

Only measured nipple rotation changes spoke rest length. The energy-conjugate
lead torque is applied equally/oppositely to nipple and threaded spoke. Native capped
implicit elastic Coulomb drive supplies thread friction; no pose writes,
proximity grasps or score triggers occur here. Short native holding is verified;
physical wrench sliding and robot manipulation remain untested.
"""
from dataclasses import dataclass
import math
import numpy as np


class ThreadTravelOutOfScope(RuntimeError):
    pass


def body_paths(metadata):
    """Exact tensor selection: native '*' matches one USD namespace level."""
    return [metadata['hub_path'],metadata['wrench_path'],
            *[s[key] for s in metadata['spokes']
              for key in ('rim_path','inner_path','outer_path','nipple_path')]]


@dataclass(frozen=True)
class ThreadSpec:
    pitch_m_per_turn: float = .000454
    friction_coefficient: float = .15
    thread_mean_radius_m: float = .001
    friction_stiffness_nm_per_rad: float = 20.
    viscous_friction_nm_s_per_rad: float = .002
    minimum_turns: float = -3.
    maximum_turns: float = 3.
    # +X points hub→rim. A right-hand nipple tightens clockwise when viewed
    # from the rim toward the hub, hence negative measured axial rotation.
    tightening_sign: int = -1


class ThreadFriction:
    """Elastic Coulomb friction history, driven only by measured relative angle.

    The native implicit drive supplies the actual torque. Its rest angle moves
    only when the measured elastic trial torque exceeds µ*T*r. This small
    preslip compliance holds a load at rest but never pulls an adjusted nipple
    all the way back to its starting angle. Loss of normal load releases it.
    """
    def __init__(self,count,stiffness):
        if not isinstance(count,int) or count<1 or not math.isfinite(stiffness) or stiffness<=0:
            raise ValueError('Positive count and friction stiffness required')
        self.stiffness=stiffness
        self.rest=np.zeros(count)
        self.dissipated=np.zeros(count)

    def advance(self,angle,limit):
        angle,limit=np.broadcast_arrays(np.asarray(angle,float),np.asarray(limit,float))
        if angle.shape!=self.rest.shape or not np.isfinite(angle).all() or not np.isfinite(limit).all() or np.any(limit<0):
            raise ValueError('Finite angle and nonnegative Coulomb limit required')
        elastic=np.clip(angle-self.rest,-limit/self.stiffness,limit/self.stiffness)
        new_rest=angle-elastic
        self.dissipated+=limit*np.abs(new_rest-self.rest)
        self.rest[:]=new_rest
        return dict(friction_target_rad=new_rest.copy(),friction_elastic_torque_nm=-self.stiffness*elastic,
                    friction_elastic_energy_j=.5*self.stiffness*elastic**2,
                    friction_dissipated_work_j=self.dissipated.copy())


class NativeFloatAttributes:
    """Skip only identical USD float values, never approximate changed forces.

    USD joint limits/drive attributes are float32. Keeping their already-authored
    values avoids unnecessary scene notices and repeated schema lookups. There
    is no position/force tolerance or time decimation. Invalidate after reset
    or any external attribute edit.
    """
    def __init__(self,attributes,*,batch_sdf_notices=False):
        self.attributes=attributes
        self.values=np.full((len(attributes),len(attributes[0])),np.nan,dtype=np.float32)
        self.attempts=0;self.writes=0
        self.batch_count=0;self.sdf_specs=None;self._change_block=None
        self._sdf_readback_verified=False
        if batch_sdf_notices:
            from pxr import Sdf
            self._change_block=Sdf.ChangeBlock
            specs=[];identifiers=set()
            # Cache all downstream USD resolutions BEFORE any ChangeBlock.
            # Inside it, touch only these authored Sdf specs. OpenUSD forbids
            # downstream Usd API calls while change notices are deferred:
            # https://openusd.org/dev/api/class_sdf_change_block.html
            for row in attributes:
                target=[]
                for attr in row:
                    stack=attr.GetPropertyStack()
                    if not stack or stack[0].typeName!=Sdf.ValueTypeNames.Float:
                        raise RuntimeError('Batched material updates require authored Float attributes')
                    spec=stack[0];identity=(spec.layer.identifier,str(spec.path))
                    if identity in identifiers or spec.default is None or not spec.layer.permissionToEdit:
                        raise RuntimeError('Material specs must be unique, writable and have authored defaults')
                    identifiers.add(identity);target.append(spec)
                specs.append(target)
            self.sdf_specs=specs

    def invalidate(self):self.values.fill(np.nan)

    def statistics(self):
        """Read back through USD only after any Sdf notice batch has closed."""
        authored=np.asarray([[attr.Get() for attr in row] for row in self.attributes],dtype=np.float32)
        return {'attempted':self.attempts,'written':self.writes,
            'sdf_notice_batches':self.batch_count,
            'first_sdf_readback_verified':self._sdf_readback_verified,
            'final_authored_values':authored.tolist(),
            'final_cached_values_match_authored':bool(np.array_equal(authored,self.values)),
            'skipping_rule':'Only float32-identical values; no material tolerance or update decimation'}

    def set(self,values):
        values=np.asarray(values,dtype=np.float32)
        if values.shape!=self.values.shape or not np.isfinite(values).all():
            raise ValueError('Finite native float attributes with matching shape required')
        self.attempts+=values.size
        changed=np.argwhere(values!=self.values)
        if self.sdf_specs is not None:
            edits=[(int(row),int(column),float(values[row,column]),self.sdf_specs[row][column]) for row,column in changed]
            if not edits:return
            with self._change_block():
                for _,_,value,spec in edits:spec.default=value
            # First batch verifies the native authored Float conversion. Later
            # writes use the same typed Sdf setter with identical float32 inputs.
            if not self._sdf_readback_verified:
                if any(float(spec.default)!=value for _,_,value,spec in edits):
                    raise RuntimeError('Sdf batch altered a native Float value')
                self._sdf_readback_verified=True
            for row,column,value,_ in edits:self.values[row,column]=value
            self.writes+=len(edits);self.batch_count+=1
            return
        for row,column in changed:
            if not self.attributes[row][column].Set(float(values[row,column])):
                raise RuntimeError('Native material attribute update failed')
            self.values[row,column]=values[row,column];self.writes+=1


def response(length_m, nominal_rest_m, turns, stiffness_n_per_m, spec=None):
    s = spec or ThreadSpec()
    if not all(math.isfinite(v) for v in (length_m,nominal_rest_m,turns,stiffness_n_per_m)):
        raise ValueError("Thread state must be finite")
    if not s.minimum_turns <= turns <= s.maximum_turns:
        raise ThreadTravelOutOfScope("Nipple exceeded the uncalibrated +/-3-turn thread model domain; invalidate the trial, never clamp the angle")
    if length_m <= 0 or nominal_rest_m <= 0 or stiffness_n_per_m <= 0:
        raise ValueError("Positive lengths/stiffness required")
    rest = nominal_rest_m-turns*s.pitch_m_per_turn
    if rest <= 0:
        raise ThreadTravelOutOfScope("Nonpositive spoke rest length")
    extension = max(length_m-rest,0.)
    tension = stiffness_n_per_m*extension
    return {"rest_length_m":rest,"tension_n":tension,
            "energy_j":.5*stiffness_n_per_m*extension**2,
            "lead_torque_nm":-s.tightening_sign*tension*s.pitch_m_per_turn/(2*math.pi),
            "friction_limit_nm":s.friction_coefficient*tension*s.thread_mean_radius_m}


def rotation(q):
    q = np.asarray(q,dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Invalid measured xyzw rotation")
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def rotation_batch(quaternions):
    """Vectorized xyzw conversion for a single native pose fetch per step."""
    q=np.asarray(quaternions,float)
    if q.ndim!=2 or q.shape[1]!=4 or not np.isfinite(q).all():raise ValueError('Invalid measured quaternion batch')
    norm=np.sum(q*q,axis=1)
    if np.any(norm<1e-24):raise ValueError('Zero measured quaternion')
    x,y,z,w=q.T;s=2/norm
    return np.stack((1-s*(y*y+z*z),s*(x*y-z*w),s*(x*z+y*w),
                     s*(x*y+z*w),1-s*(x*x+z*z),s*(y*z-x*w),
                     s*(x*z-y*w),s*(y*z+x*w),1-s*(x*x+y*y)),axis=-1).reshape((-1,3,3))


def relative_thread_angle(spoke_rotation,nipple_rotation,reference=None):
    """Swing/twist angle about the spoke axis and its world torque gradient.

    Common spoke/nipple rotation never advances the thread. The returned
    gradient is d(angle)/d(nipple world rotation); its negative is the spoke
    gradient, so the material applies equal/opposite, energy-conjugate torques.
    An antiparallel shaft has undefined twist and is outside this thread model.
    """
    reference=np.eye(3) if reference is None else np.asarray(reference,float)
    frame=np.asarray(spoke_rotation,float)@reference
    nipple=np.asarray(nipple_rotation,float);relative=frame.T@nipple
    a,b=frame[:,0],nipple[:,0];denominator=1.+float(np.dot(a,b))
    if denominator<1e-8:raise ThreadTravelOutOfScope('Antiparallel thread axes have undefined twist')
    angle=math.atan2(relative[2,1]-relative[1,2],relative[1,1]+relative[2,2])
    return angle,(a+b)/denominator


def relative_thread_angle_batch(spoke,nipple,reference):
    """The same energy-conjugate calculation for all measured spokes."""
    frame=np.asarray(spoke)@np.asarray(reference)
    relative=np.swapaxes(frame,1,2)@np.asarray(nipple)
    a,b=frame[:,:,0],np.asarray(nipple)[:,:,0]
    denominator=1.+np.sum(a*b,axis=1)
    if np.any(denominator<1e-8):
        raise ThreadTravelOutOfScope('Antiparallel thread axes have undefined twist')
    angle=np.arctan2(relative[:,2,1]-relative[:,1,2],relative[:,1,1]+relative[:,2,2])
    return angle,(a+b)/denominator[:,None]


def response_batch(length,nominal,turns,stiffness,spec):
    """Vector counterpart of response(); same unclamped validity boundary."""
    length,nominal,turns,stiffness=np.broadcast_arrays(length,nominal,turns,stiffness)
    if not all(np.isfinite(a).all() for a in (length,nominal,turns,stiffness)):
        raise ValueError('Thread state must be finite')
    if np.any((turns<spec.minimum_turns)|(turns>spec.maximum_turns)):
        raise ThreadTravelOutOfScope('Nipple exceeded the +/-3-turn thread model domain')
    if np.any((length<=0)|(nominal<=0)|(stiffness<=0)):
        raise ValueError('Positive lengths/stiffness required')
    rest=nominal-turns*spec.pitch_m_per_turn
    if np.any(rest<=0):raise ThreadTravelOutOfScope('Nonpositive spoke rest length')
    extension=np.maximum(length-rest,0.);tension=stiffness*extension
    return dict(rest_length_m=rest,tension_n=tension,energy_j=.5*stiffness*extension**2,
        lead_torque_nm=-spec.tightening_sign*tension*spec.pitch_m_per_turn/(2*math.pi),
        friction_limit_nm=spec.friction_coefficient*tension*spec.thread_mean_radius_m)


class NativeThreadCoupler:
    """Call before EVERY native physics substep after stage attachment.

    `metadata` comes from wheel_scene.build_scene. Missing live transforms or
    excess thread travel raises and invalidates the trial. The caller must
    record that error and must not resume with a saturated/clamped state.
    """
    def __init__(self, stage_id, metadata, spec=None, native_io=None,*,batch_sdf_notices=False,material_stride=1):
        from omni.physx import get_physx_interface, get_physx_simulation_interface
        from pxr import PhysicsSchemaTools, Sdf
        from integrity import RimDomainGuard
        self.integrity=RimDomainGuard()
        self.native_io=native_io
        if material_stride not in (1,5) or (material_stride!=1 and native_io is None):
            raise ValueError('Material stride must be1 or5; multirate mode requires native tensors')
        self.material_stride=material_stride
        self._material_step=0
        self._held_torques=None
        self.physics=get_physx_interface()
        self.sim=get_physx_simulation_interface()
        self.stage_id=stage_id
        self.meta=metadata
        self.spec=spec or ThreadSpec()
        self.ids={path:PhysicsSchemaTools.sdfPathToInt(Sdf.Path(path)) for path in
                  [path for s in metadata["spokes"] for path in (s["nipple_path"],s["outer_path"])]}
        self.previous=np.zeros(len(metadata["spokes"]))
        self.unwrapped=np.zeros(len(metadata["spokes"]))
        self.friction=ThreadFriction(len(metadata['spokes']),self.spec.friction_stiffness_nm_per_rad)
        self.parameters=NativeFloatAttributes([[s['rest_target_attribute'],
            s['friction_drive'].GetMaxForceAttr(),s['friction_drive'].GetTargetPositionAttr()]
            for s in metadata['spokes']],batch_sdf_notices=batch_sdf_notices)
        self.last=[]
        self._batch=None
        if native_io is not None:
            spokes=metadata['spokes']
            self._batch={key:np.asarray([s[key] for s in spokes],float) for key in
                ('reference_nipple_in_spoke_rotation','initial_turn_offset','hub_anchor_local_m',
                 'rim_anchor_local_m','half_rod_length_m','rest_target_origin_m','nominal_rest_m','stiffness_n_per_m')}
            self._batch.update({key:np.asarray([native_io.index[s[key]] for s in spokes],int)
                for key in ('rim_path','nipple_path','inner_path','outer_path')})
            self._batch['native_rest_bias_m']=np.asarray([s.get('native_rest_bias_m',0.) for s in spokes])

    @property
    def last(self):
        if self._last is None:
            self._last=[dict(spoke=i,**{key:float(value[i]) for key,value in self._state_arrays.items()})
                        for i in range(len(self.previous))]
        return self._last

    @last.setter
    def last(self,value):
        self._last=value

    def frame(self,path):
        if self.native_io is not None:
            return self.native_io.frame(path)
        tr=self.physics.get_rigidbody_transformation(path)
        if not tr.get("ret_val"):
            raise RuntimeError("Live PhysX transform unavailable: "+path)
        return np.asarray(tr["position"],dtype=float),rotation(tr["rotation"])

    def before_step(self, *, frozen_material_diagnostic=False):
        if self.native_io is not None:
            # Explicit sample-and-hold approximation: keep native constraints,
            # collisions and equal/opposite external torque active EVERY step.
            # Cache separately because assessor reads can clear io.torques.
            refresh=self._material_step%self.material_stride==0
            self._material_step+=1
            if not refresh:
                if not frozen_material_diagnostic:
                    self.native_io.torques[:]=self._held_torques
                    self.native_io.flush_torques()
                return
            self.native_io.begin_step()
            self.audit_integrity(refresh=False)
            self._before_tensor_step(frozen_material_diagnostic=frozen_material_diagnostic)
            self._held_torques=self.native_io.torques.copy()
            return
        import carb
        if frozen_material_diagnostic:raise ValueError('Frozen diagnostic requires tensor state')
        self.audit_integrity(refresh=False)
        hp,hR=self.frame(self.meta["hub_path"])
        self.last=[]
        for i,s in enumerate(self.meta["spokes"]):
            rp,rR=self.frame(s["rim_path"])
            _,nR=self.frame(s["nipple_path"])
            ip,iR=self.frame(s["inner_path"])
            op,oR=self.frame(s["outer_path"])
            wrapped,torque_gradient=relative_thread_angle(oR,nR,s['reference_nipple_in_spoke_rotation'])
            delta=(wrapped-self.previous[i]+math.pi)%(2*math.pi)-math.pi
            self.unwrapped[i]+=delta
            self.previous[i]=wrapped
            turns=s["initial_turn_offset"]+self.spec.tightening_sign*self.unwrapped[i]/(2*math.pi)
            anchor=hp+hR@np.asarray(s["hub_anchor_local_m"])
            endpoint=rp+rR@np.asarray(s["rim_anchor_local_m"])
            endpoint_length=float(np.linalg.norm(endpoint-anchor))
            half=s["half_rod_length_m"]
            inner_joint=ip+iR[:,0]*(half/2)
            outer_joint=op-oR[:,0]*(half/2)
            gap=float(np.dot(outer_joint-inner_joint,iR[:,0]))
            # Match the actual axial D6 coordinate, including spoke bending.
            # Endpoint distance alone would miss material stretch in a bowed
            # spoke and give the wrong thread reaction torque.
            length=s["rest_target_origin_m"]+(float(np.linalg.norm(op-ip)) if s.get('central_spring') else gap)
            state=response(length,s["nominal_rest_m"],turns,s["stiffness_n_per_m"],self.spec)
            torque=torque_gradient*state["lead_torque_nm"]
            if self.native_io is None:
                self.sim.apply_torque(self.stage_id,self.ids[s["nipple_path"]],carb.Float3(*map(float,torque)))
                self.sim.apply_torque(self.stage_id,self.ids[s["outer_path"]],carb.Float3(*map(float,-torque)))
            else:
                self.native_io.add_torque(s["nipple_path"],torque)
                self.native_io.add_torque(s["outer_path"],-torque)
            self.last.append({"spoke":i,"turns":turns,"length_m":length,"endpoint_distance_m":endpoint_length,"axial_joint_gap_m":gap,**state})
        friction=self.friction.advance(self.unwrapped,np.array([s['friction_limit_nm'] for s in self.last]))
        targets=np.degrees((friction['friction_target_rad']+math.pi)%(2*math.pi)-math.pi)
        self.parameters.set([[state['rest_length_m']-s['rest_target_origin_m']-s.get('native_rest_bias_m',0.),state['friction_limit_nm'],target]
            for s,state,target in zip(self.meta['spokes'],self.last,targets)])
        for i,s in enumerate(self.meta['spokes']):
            self.last[i].update({key:float(value[i]) for key,value in friction.items()})
        if self.native_io is not None:
            self.native_io.flush_torques()

    def _before_tensor_step(self, *, frozen_material_diagnostic=False):
        """Batch pose algebra; native attribute writes remain exact per joint."""
        io,b=self.native_io,self._batch
        p,R=io.transforms[:,:3],io.rotations
        rim,nipple,inner,outer=(b[key] for key in ('rim_path','nipple_path','inner_path','outer_path'))
        wrapped,gradient=relative_thread_angle_batch(R[outer],R[nipple],b['reference_nipple_in_spoke_rotation'])
        self.unwrapped+=(wrapped-self.previous+math.pi)%(2*math.pi)-math.pi
        self.previous[:]=wrapped
        turns=b['initial_turn_offset']+self.spec.tightening_sign*self.unwrapped/(2*math.pi)
        hp,hR=io.frame(self.meta['hub_path'])
        anchors=hp+b['hub_anchor_local_m']@hR.T
        endpoints=p[rim]+np.einsum('nij,nj->ni',R[rim],b['rim_anchor_local_m'])
        half=b['half_rod_length_m'][:,None]
        gap=np.sum((p[outer]-R[outer,:,0]*half/2-p[inner]-R[inner,:,0]*half/2)*R[inner,:,0],axis=1)
        coordinate=np.linalg.norm(p[outer]-p[inner],axis=1) if self.meta.get('central_spring_candidate') else gap
        length=b['rest_target_origin_m']+coordinate
        material_turns=b['initial_turn_offset'] if frozen_material_diagnostic else turns
        state=response_batch(length,b['nominal_rest_m'],material_turns,b['stiffness_n_per_m'],self.spec)
        if not frozen_material_diagnostic:
            friction=self.friction.advance(self.unwrapped,state['friction_limit_nm'])
            targets=np.degrees((friction['friction_target_rad']+math.pi)%(2*math.pi)-math.pi)
            self.parameters.set(np.column_stack((state['rest_length_m']-b['rest_target_origin_m']-b['native_rest_bias_m'],
                                                state['friction_limit_nm'],targets)))
            torque=gradient*state['lead_torque_nm'][:,None]
            # These are disjoint body indices, one nipple and outer half per spoke.
            io.torques[nipple]+=torque;io.torques[outer]-=torque
            self.custom_torque_audit={'net_world_torque_nm':io.torques.sum(0).tolist(),
                                     'maximum_single_body_torque_nm':float(np.linalg.norm(io.torques,axis=1).max())}
            io.flush_torques()
        self._state_arrays=dict(turns=turns,length_m=length,endpoint_distance_m=np.linalg.norm(endpoints-anchors,axis=1),
            axial_joint_gap_m=gap,**state)
        if not frozen_material_diagnostic:self._state_arrays.update(friction)
        if frozen_material_diagnostic:self._state_arrays['frozen_material_turns']=material_turns
        self._last=None

    def audit_integrity(self, *, refresh=True):
        """Latch unsupported rim deformation; no physics edits or score awards."""
        if self.native_io is not None:
            if refresh:self.native_io.begin_step()
            indices=self._batch['rim_path']
            centres=self.native_io.transforms[indices,:3]
            matrices=self.native_io.rotations[indices]
        else:
            frames=[self.frame(s['rim_path']) for s in self.meta['spokes']]
            centres=np.asarray([f[0] for f in frames]);matrices=np.asarray([f[1] for f in frames])
        state=self.integrity.update(centres,matrices)
        if not state['valid']:
            raise ThreadTravelOutOfScope('Native rim left elastic model domain: '+str(state['first_failure']))
        return state

    def observation_for_assessor(self):
        """Material telemetry only; never feed this into a robot policy/demo."""
        return self.last


class TensorNativeIO:
    """DirectGPU-safe pose and torque bridge; never writes body transforms.

    CPU PhysX transform access is not a valid source when readback is
    suppressed. This adapter reads native tensor state every substep and writes
    only the material's equal/opposite torques through the tensor force API.
    """
    def __init__(self,simulation_view,pattern):
        import torch
        self.view=simulation_view.create_rigid_body_view(pattern)
        self.paths=list(self.view.prim_paths)
        self.index={path:i for i,path in enumerate(self.paths)}
        values=self.view.get_transforms()
        self.device=values.device
        self.indices=torch.arange(self.view.count,dtype=torch.int32,device=self.device)
        self.torques=np.zeros((self.view.count,3))
        self.transforms=None
        self.rotations=None

    def begin_step(self):
        self.transforms=self.view.get_transforms().detach().cpu().numpy().copy()
        self.torques.fill(0.)
        if not np.isfinite(self.transforms).all():
            raise RuntimeError('Nonfinite native wheel tensor state')
        self.rotations=rotation_batch(self.transforms[:,3:])

    def frame(self,path):
        if path not in self.index:
            raise RuntimeError('Missing native tensor body: '+path)
        index=self.index[path]
        return self.transforms[index,:3],self.rotations[index]

    def add_torque(self,path,value):
        self.torques[self.index[path]]+=np.asarray(value,float)

    def flush_torques(self):
        import torch
        values=torch.as_tensor(self.torques,dtype=torch.float32,device=self.device)
        self.view.apply_forces_and_torques_at_position(force_data=None,torque_data=values,position_data=None,indices=self.indices,is_global=True)
