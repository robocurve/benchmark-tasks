"""Unscored, externally loaded native mechanism coupon; never a robot demo.

The ordinary dryer is clamped to the world only in this separate harness.
The switch, joint, detent, gravity, mass and collider geometry are unchanged.
No pose/state writes occur after initial scene construction.
"""
import math
from config import DRYER_START,SWITCH_START,SWITCH_TRAVEL,rotate,conjugate
from switch_detent import slider_coordinate

COUPON_DURATION=7.7
COUPON_FORCE=.35
REVERSE_FORCE=.10


def coupon_command(time_s):
    if not math.isfinite(time_s) or time_s<0:raise ValueError('invalid coupon time')
    if time_s<1.:return 'off_rest',0.
    if time_s<1.5:return 'push_on',COUPON_FORCE
    if time_s<3.5:return 'on_force_removed_hold',0.
    if time_s<3.7:return 'reverse_load_ramp_in',-REVERSE_FORCE*(time_s-3.5)/.2
    if time_s<4.:return 'reverse_load_hold',-REVERSE_FORCE
    if time_s<4.2:return 'reverse_load_ramp_out',-REVERSE_FORCE*(4.2-time_s)/.2
    if time_s<5.2:return 'on_zero_force_recovery',0.
    if time_s<5.7:return 'push_off',-COUPON_FORCE
    return 'off_force_removed_hold',0.


class CouponTravel:
    """Fixed-size extrema from every native substep, never just trace frames."""
    def __init__(self):
        self.windows={name:{'samples':0,'min_q_m':None,'max_q_m':None,'last_q_m':None}
                      for name in ('push_on','on_hold','reverse_load','on_recovery_final','push_off','off_hold')}

    def observe(self,time_s,phase,q):
        if not all(math.isfinite(v) for v in (time_s,q)):raise ValueError('nonfinite coupon travel sample')
        names=[]
        if phase in ('push_on','push_off'):names.append(phase)
        if phase=='on_force_removed_hold' and 2.5-1e-9<=time_s<=3.5+1e-9:names.append('on_hold')
        if phase.startswith('reverse_load_'):names.append('reverse_load')
        if phase=='on_zero_force_recovery' and time_s>=4.95-1e-9:names.append('on_recovery_final')
        if phase=='off_force_removed_hold' and time_s>=6.7-1e-9:names.append('off_hold')
        for name in names:
            window=self.windows[name];window['samples']+=1;window['last_q_m']=q
            window['min_q_m']=q if window['min_q_m'] is None else min(window['min_q_m'],q)
            window['max_q_m']=q if window['max_q_m'] is None else max(window['max_q_m'],q)

    def checks(self):
        def above(name,threshold):
            window=self.windows[name];return window['samples']>0 and window['min_q_m']>threshold
        def below(name,threshold):
            window=self.windows[name];return window['samples']>0 and window['max_q_m']<threshold
        on=self.windows['push_on'];off=self.windows['push_off']
        return {'on_reached_under_external_push':on['samples']>0 and on['max_q_m']>.9*SWITCH_TRAVEL,
                'on_retained_after_force_removed':above('on_hold',.8*SWITCH_TRAVEL),
                'on_well_retained_during_reverse_load':above('reverse_load',.5*SWITCH_TRAVEL),
                'on_recovered_after_reverse_force_removed':above('on_recovery_final',.8*SWITCH_TRAVEL),
                'off_reached_under_external_push':off['samples']>0 and off['min_q_m']<.1*SWITCH_TRAVEL,
                'off_retained_after_force_removed':below('off_hold',.1*SWITCH_TRAVEL)}


def attachment_sample(dryer,switch):
    q=slider_coordinate(dryer,switch)
    dnorm=math.sqrt(sum(v*v for v in dryer[3:]));snorm=math.sqrt(sum(v*v for v in switch[3:]))
    if min(dnorm,snorm)<1e-12:raise ValueError('invalid coupon orientation')
    dquat=tuple(v/dnorm for v in dryer[3:]);squat=tuple(v/snorm for v in switch[3:])
    local=rotate(conjugate(dquat),tuple(b-a for a,b in zip(dryer[:3],switch[:3])))
    angle=math.degrees(2.*math.acos(min(1.,abs(sum(a*b for a,b in zip(dquat,squat))))))
    return {'q_m':q,'lateral_anchor_error_m':math.hypot(local[0]+.037,local[1]),'relative_angle_deg':angle,
            'fixture_translation_error_m':math.dist(dryer[:3],DRYER_START),
            'fixture_angle_error_deg':math.degrees(2.*math.acos(min(1.,abs(dquat[3]))))}


def inferred_joint_reaction(pre_velocity,post_velocity,dt,mass,external_force_z,gravity_z=-9.81):
    """Newton residual on slider COM, including drive/limits reaction.

    This is inferred from native velocities and known loads, not a direct
    native constraint-force sensor or a calibrated detent-force measurement.
    """
    if len(pre_velocity)!=6 or len(post_velocity)!=6 or not all(math.isfinite(v) for v in (*pre_velocity,*post_velocity,dt,mass,external_force_z,gravity_z)) or dt<=0 or mass<=0:
        raise ValueError('invalid native reaction sample')
    return tuple(mass*(b-a)/dt-(external_force_z if i==2 else 0.)-(mass*gravity_z if i==2 else 0.)
                 for i,(a,b) in enumerate(zip(pre_velocity[:3],post_velocity[:3])))


def run_coupon(out):
    """Called only by run.py's explicit slider-coupon diagnostic mode."""
    import json,time
    from types import SimpleNamespace
    from isaaclab.app import AppLauncher
    started=time.monotonic();app=AppLauncher(headless=True,enable_cameras=False).app;binding=None
    try:
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import RigidObject,RigidObjectCfg
        from pxr import UsdPhysics,Gf
        from native import export,install_slider
        from switch_detent import NativeDetentBinding
        dt=1/480;simulation=sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt,device='cuda:0'))
        class Scene(dict):pass
        scene=Scene();scene.stage=simulation.stage
        for name,position in (('dryer',DRYER_START),('switch',SWITCH_START)):
            asset=export(out/(name+'.usda'),name)
            cfg=RigidObjectCfg(prim_path='/World/'+name,spawn=sim_utils.UsdFileCfg(usd_path=asset),
                               init_state=RigidObjectCfg.InitialStateCfg(pos=position))
            scene[name]=RigidObject(cfg)
        install_slider(simulation.stage,'/World')
        fixture=UsdPhysics.FixedJoint.Define(simulation.stage,'/World/coupon_fixture')
        fixture.CreateBody1Rel().SetTargets(['/World/dryer']);fixture.CreateLocalPos0Attr(Gf.Vec3f(*DRYER_START))
        fixture.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));fixture.CreateExcludeFromArticulationAttr(True)
        simulation.reset()
        base=SimpleNamespace(sim=simulation,scene=scene);binding=NativeDetentBinding(base)
        def array(value):
            value=value.torch if hasattr(value,'torch') else value
            return value.detach().cpu().numpy() if hasattr(value,'detach') else value.numpy()
        mass=float(array(scene['switch'].root_view.get_masses()).reshape(-1)[0])
        travel=CouponTravel();worst_lateral=0.;worst_angle=0.;worst_fixture=0.;worst_fixture_angle=0.;minimum_q=float('inf');maximum_q=-float('inf');peak_reaction=0.
        with (out/'trajectory.jsonl').open('w') as trace:
            for step in range(round(COUPON_DURATION/.05)):
                before_steps,before_time=binding.detent.steps,binding.detent.sim_time
                for substep in range(24):
                    phase,load=coupon_command((step*24+substep)*dt)
                    scene['switch'].permanent_wrench_composer.set_forces_and_torques_index(
                        forces=torch.tensor([[[0.,0.,load]]],device=simulation.device,dtype=torch.float32),
                        torques=torch.zeros((1,1,3),device=simulation.device),is_global=True)
                    for asset in scene.values():asset.write_data_to_sim()
                    simulation.step(render=False)
                    for asset in scene.values():asset.update(dt)
                    pre=binding.latest_native_sample;post=binding.sample_native()
                    state=attachment_sample(post['dryer_pose'],post['switch_pose'])
                    travel.observe((step*24+substep+1)*dt,phase,state['q_m'])
                    reaction=inferred_joint_reaction(pre['switch_velocity_com_w'],post['switch_velocity_com_w'],dt,mass,load)
                    worst_lateral=max(worst_lateral,state['lateral_anchor_error_m']);worst_angle=max(worst_angle,state['relative_angle_deg'])
                    worst_fixture=max(worst_fixture,state['fixture_translation_error_m']);worst_fixture_angle=max(worst_fixture_angle,state['fixture_angle_error_deg'])
                    minimum_q=min(minimum_q,state['q_m']);maximum_q=max(maximum_q,state['q_m'])
                    peak_reaction=max(peak_reaction,math.sqrt(sum(v*v for v in reaction)))
                binding.detent.check_interval(before_steps,before_time,.05)
                record={'time':(step+1)*.05,'phase':phase,'external_switch_force_N':[0.,0.,load],**state,**post,
                        'inferred_joint_reaction_N':reaction,'reaction_scope':'COM acceleration residual; drive + limits',
                        'slider_detent':binding.snapshot()}
                trace.write(json.dumps(record)+'\n')
        checks={**travel.checks(),
                'attachment_integrity':worst_lateral<.0002 and worst_angle<1. and worst_fixture<.0002 and worst_fixture_angle<1.,
                'travel_within_0p2mm_numerical_margin':minimum_q>=-.0002 and maximum_q<=SWITCH_TRAVEL+.0002,
                'native_substep_cadence':True}
        result={'diagnostic':True,'task_score':None,'robot_demo':False,'media_eligible':False,
                'probe_mode':'slider-coupon','external_bench_forces':True,'diagnostic_fixture':'native world-to-dryer fixed clamp',
                'switch_mass_readback_kg':mass,'elapsed_wall_s':time.monotonic()-started,'physics_dt_s':dt,
                'checks':checks,'mechanism_coupon_checks_pass':all(checks.values()),'min_q_m':minimum_q,'max_q_m':maximum_q,
                'max_lateral_anchor_error_m':worst_lateral,'max_relative_angle_deg':worst_angle,'max_fixture_translation_error_m':worst_fixture,
                'max_fixture_angle_error_deg':worst_fixture_angle,
                'native_substep_travel_windows':travel.windows,
                'retention_measurement_hz':480,'reverse_load_N':REVERSE_FORCE,
                'peak_inferred_joint_reaction_N':peak_reaction,'slider_detent':binding.snapshot(),
                'interpretation':'Unscored force/hold coupon only; target latency and ordinary robot tilt/lift retention remain unverified'}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    except BaseException as error:
        (out/'error.json').write_text(json.dumps({'error':str(error),'type':type(error).__name__},indent=2)+'\n');raise
    finally:
        if binding is not None:
            try:binding.write_trace(out/'slider-crossings.json')
            finally:binding.close()
        app.close()
