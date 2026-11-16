"""Passive two-well slider law and measured native-substep target selection.

This is a sampled approximation, not a claim of a continuous or calibrated
detent. Only the measured coordinate chooses a well. Native joint drives
provide spring/damping reaction; this module never writes poses or forces.
"""
from dataclasses import dataclass
from collections import deque
import math
from config import SWITCH_TRAVEL,SWITCH_DETENT_STIFFNESS,SWITCH_DETENT_DAMPING,rotate,conjugate


@dataclass(frozen=True)
class DetentLaw:
    travel: float=SWITCH_TRAVEL
    stiffness: float=SWITCH_DETENT_STIFFNESS
    damping: float=SWITCH_DETENT_DAMPING

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.travel,self.stiffness,self.damping)) or min(self.travel,self.stiffness)<=0 or self.damping<0:
            raise ValueError('invalid detent constitutive parameters')

    def target(self,q):
        if not math.isfinite(q):raise ValueError('nonfinite measured slider coordinate')
        return 0. if q<=self.travel/2 else self.travel

    def potential(self,q):
        target=self.target(q)
        return .5*self.stiffness*(q-target)**2

    def force(self,q,velocity=0.):
        if not math.isfinite(velocity):raise ValueError('nonfinite slider velocity')
        return -self.stiffness*(q-self.target(q))-self.damping*velocity

    @property
    def barrier(self):return self.stiffness*self.travel**2/8.


def slider_coordinate(dryer_pose,switch_pose):
    """Joint Z displacement from native body-link poses, quaternion XYZW."""
    if len(dryer_pose)!=7 or len(switch_pose)!=7 or not all(math.isfinite(v) for v in (*dryer_pose,*switch_pose)):
        raise ValueError('invalid native slider body poses')
    norm=math.sqrt(sum(v*v for v in dryer_pose[3:]))
    if norm<1e-12:raise ValueError('invalid native dryer orientation')
    orientation=tuple(v/norm for v in dryer_pose[3:])
    local=rotate(conjugate(orientation),tuple(b-a for a,b in zip(dryer_pose[:3],switch_pose[:3])))
    return float(local[2]+.045)


class SubstepDetent:
    """The only state is diagnostics and the last authored native target."""
    def __init__(self,read_coordinate,write_target,physics_dt,law=None,initial_target=0.,read_sample=None,read_target=None):
        self.law=DetentLaw() if law is None else law
        if not math.isfinite(physics_dt) or physics_dt<=0:raise ValueError('invalid physics timestep')
        if min(abs(initial_target),abs(initial_target-self.law.travel))>1e-9:raise ValueError('initial target is not a detent well')
        self.read_coordinate,self.write_target=read_coordinate,write_target
        self.read_sample,self.read_target=read_sample,read_target;self.trace=CrossingTrace()
        self.physics_dt=physics_dt;self.target=0. if abs(initial_target)<1e-9 else self.law.travel
        self.steps=0;self.sim_time=0.;self.last_q=None;self.last_potential=None;self.target_changes=0;self.retarget_energy=0.;self.max_retarget_increase=0.

    def on_physics_step(self,dt):
        if not math.isfinite(dt) or abs(dt-self.physics_dt)>max(1e-9,self.physics_dt*1e-5):
            raise RuntimeError('detent callback is not running at the configured physics timestep')
        q=float(self.read_coordinate());target=self.law.target(q);change=0.;crossing=target!=self.target
        if target!=self.target:
            change=.5*self.law.stiffness*((q-target)**2-(q-self.target)**2)
            if change>1e-12:raise RuntimeError('detent target switch would inject stored spring energy')
            if self.write_target(target) is False:raise RuntimeError('native slider target update failed')
            self.retarget_energy+=change;self.max_retarget_increase=max(self.max_retarget_increase,change)
            self.target_changes+=1;self.target=target
        authored=float(self.read_target()) if self.read_target else target
        if not math.isfinite(authored) or abs(authored-target)>1e-9:raise RuntimeError('native slider target attribute readback mismatch')
        potential=self.law.potential(q)
        sample={'callback_index':self.steps+1,'sim_time_s':self.sim_time,'q_m':q,
                'q_velocity_fd_m_s':None if self.last_q is None else (q-self.last_q)/dt,
                'selected_well_m':target,'authored_target_m':authored,
                'elastic_potential_J':potential,'potential_change_since_previous_callback_J':None if self.last_potential is None else potential-self.last_potential,
                'retarget_energy_change_J':change}
        if self.read_sample is not None:sample.update(self.read_sample())
        self.trace.record(sample,crossing)
        self.steps+=1;self.sim_time+=dt;self.last_q=q;self.last_potential=potential

    def check_interval(self,previous_steps,previous_time,control_dt):
        expected=round(control_dt/self.physics_dt)
        if self.steps-previous_steps!=expected or abs(self.sim_time-previous_time-control_dt)>1e-7:
            raise RuntimeError('passive detent did not receive one callback per native physics substep')

    def snapshot(self):
        return {'sampled_approximation':True,'physics_dt_s':self.physics_dt,'callback_steps':self.steps,
                'callback_sim_time_s':self.sim_time,'measurement_phase':'pre-physics substep',
                'last_measured_q_m':self.last_q,'last_authored_well_m':self.target,'well_target_changes':self.target_changes,
                'retarget_energy_change_J':self.retarget_energy,'max_retarget_energy_increase_J':self.max_retarget_increase,
                'travel_m':self.law.travel,'stiffness_N_per_m':self.law.stiffness,'damping_Ns_per_m':self.law.damping,
                'barrier_J':self.law.barrier,'native_target_application_latency_verified':False}


class CrossingTrace:
    """Bounded pre/post windows; long steady drying does not grow this trace."""
    def __init__(self,pre_steps=8,post_steps=24,max_events=16):
        if min(pre_steps,post_steps,max_events)<1:raise ValueError('trace bounds must be positive')
        self.pre_steps,self.post_steps,self.max_events=pre_steps,post_steps,max_events
        self.history=deque(maxlen=pre_steps);self.events=deque(maxlen=max_events);self.dropped=0

    def record(self,sample,crossing=False):
        for event in self.events:
            if event['post_remaining']>0:event['samples'].append(sample);event['post_remaining']-=1
        if crossing:
            if len(self.events)==self.max_events:self.dropped+=1
            self.events.append({'crossing_callback_index':sample['callback_index'],
                                'samples':[*self.history,sample],'post_remaining':self.post_steps})
        self.history.append(sample)

    def export(self):
        return {'pre_steps':self.pre_steps,'post_steps':self.post_steps,'max_events':self.max_events,
                'dropped_events':self.dropped,'events':list(self.events),
                'native_target_application_latency_verified':False,
                'sample_phase':'Native poses/velocities before physics; response appears in subsequent callback samples'}


class NativeDetentBinding:
    def __init__(self,base):
        # Import only after AppLauncher. IsaacLab3's PhysX manager catches and
        # propagates callback errors; no silent 20 Hz fallback is supported.
        from pxr import UsdPhysics
        from isaaclab_physx.physics.physx_manager import IsaacEvents
        self.manager=base.sim.backend_manager
        joints=[p for p in base.scene.stage.Traverse() if p.GetName()=='power_slider' and p.IsA(UsdPhysics.PrismaticJoint)]
        if len(joints)!=1:raise RuntimeError('expected one native dryer power slider')
        drive=UsdPhysics.DriveAPI(joints[0],'linear');law=DetentLaw()
        if (not drive or abs(drive.GetStiffnessAttr().Get()-law.stiffness)>1e-6
                or abs(drive.GetDampingAttr().Get()-law.damping)>1e-6):
            raise RuntimeError('native slider constitutive drive readback mismatch')
        self.target_attr=drive.GetTargetPositionAttr()
        views=[base.scene[name].root_view for name in ('dryer','switch')]
        def native_array(value):
            value=value.torch if hasattr(value,'torch') else value
            if hasattr(value,'detach'):value=value.detach().cpu().numpy()
            else:value=value.numpy()
            return value
        def sample_native():
            return {'dryer_pose':native_array(views[0].get_transforms()).reshape(-1,7)[0].tolist(),
                    'switch_pose':native_array(views[1].get_transforms()).reshape(-1,7)[0].tolist(),
                    'dryer_velocity_com_w':native_array(views[0].get_velocities()).reshape(-1,6)[0].tolist(),
                    'switch_velocity_com_w':native_array(views[1].get_velocities()).reshape(-1,6)[0].tolist()}
        self.sample_native=sample_native;latest={};self.latest_native_sample=latest
        def read_q():
            latest.clear();latest.update(sample_native())
            return slider_coordinate(latest['dryer_pose'],latest['switch_pose'])
        self.detent=SubstepDetent(read_q,self.target_attr.Set,float(base.sim.cfg.dt),law,float(self.target_attr.Get()),
                                 read_sample=lambda:dict(latest),read_target=self.target_attr.Get)
        self.handle=self.manager.register_callback(self.detent.on_physics_step,IsaacEvents.PRE_PHYSICS_STEP,
                                                   name='passive_dryer_slider_detent',wrap_weak_ref=False)

    def close(self):
        if self.handle is not None:self.manager.deregister_callback(self.handle);self.handle=None

    def snapshot(self):
        result=self.detent.snapshot();result['native_target_attribute_readback_m']=float(self.target_attr.Get())
        result['crossing_trace_events']=len(self.detent.trace.events);result['crossing_trace_dropped']=self.detent.trace.dropped
        return result

    def write_trace(self,path):
        import json
        path.write_text(json.dumps(self.detent.trace.export(),indent=2)+'\n')
