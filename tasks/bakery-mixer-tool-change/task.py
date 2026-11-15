"""Refit a commercial mixer tool and guard, then verify a low-speed start and stop."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile

import numpy as np
from resist_isaac.clearance_recovery import ClearanceRecovery
from resist_isaac.usd_assets import create_asset

CONDITIONS = ('recoverable',)


def ramp_fraction(start, enabled, ticks):
    fraction=min(max(ticks/40.,0.),1.)
    return start+(float(enabled)-start)*fraction*fraction*(3.-2.*fraction)


def export_assets(folder, directory):
    from pxr import PhysxSchema, Usd, UsdPhysics
    descriptions = json.loads((folder/'geometry.json').read_text())
    paths = {name: create_asset(desc, Path(directory)/(name+'.usda'), folder)
             for name, desc in descriptions.items()}
    for path in paths.values():
        stage=Usd.Stage.Open(str(path))
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        stage.GetRootLayer().Save()
    stage = Usd.Stage.Open(str(paths['mixer']))
    root = stage.GetDefaultPrim().GetPath()
    lift = stage.GetPrimAtPath(root.AppendPath('bowl_lift'))
    handle = root.AppendPath('handle_hinge')
    # Native two-way gear constraint: no per-frame bowl-position writes.
    mimic = PhysxSchema.PhysxMimicJointAPI.Apply(lift, UsdPhysics.Tokens.rotX)
    mimic.CreateReferenceJointRel().SetTargets([handle])
    # USD angular reference coordinates are degrees in this linear/angular pair.
    mimic.CreateGearingAttr(-.075/90.)
    mimic.CreateOffsetAttr(0.)
    stage.GetRootLayer().Save()
    return paths


class MixerTask(ClearanceRecovery):
    physics_hz = 960
    hold_s = 2.
    stop_when_success = True

    def __init__(self, folder):
        self.folder = Path(folder)
        self.cfg = json.loads((self.folder/'pilot.json').read_text())
        self.descriptions = json.loads((self.folder/'geometry.json').read_text())
        self.poses = self.cfg['poses']
        self._reset_counters()

    def _reset_counters(self):
        self._unsafe = self._motor = self._ran = False
        self._stopped_after_run = False
        self._drive_started = False
        self._start_pressed = self._stop_pressed = False
        self._retained_count = self._guard_count = self._running_count = 0
        self._previous = None
        self._drive_fraction = self._ramp_start = 0.
        self._ramp_tick = 0
        self._ramp_enabled = False

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.utils.pose import Pose
        self._temporary = tempfile.TemporaryDirectory(prefix='bakery-mixer-')
        paths = export_assets(self.folder, self._temporary.name)
        return [Object(name=name,
                object_type=ObjectType.ARTICULATION if name=='mixer' else ObjectType.BASE if name=='rack' else ObjectType.RIGID,
                usd_path=str(path), initial_pose=Pose(position_xyz=tuple(self.poses[name])))
                for name,path in paths.items()]

    def configure_native(self, cfg):
        from isaaclab.actuators import ImplicitActuatorCfg
        cfg.scene.mixer.actuators={
            'motor':ImplicitActuatorCfg(joint_names_expr=['orbit','spin'],
                stiffness=0.,damping=.4,joint_effort_limit=.10,joint_velocity_limit=30.),
            'retaining_spring':ImplicitActuatorCfg(joint_names_expr=['retaining_spring'],
                stiffness=400.,damping=4.,joint_effort_limit=20.,joint_velocity_limit=.2),
            'buttons':ImplicitActuatorCfg(joint_names_expr=['start_button','stop_button'],
                stiffness=300.,damping=1.,joint_effort_limit=5.,joint_velocity_limit=.1)}
        cfg.sim.physics.solver_type = 0
        cfg.sim.physics.enable_external_forces_every_iteration = False
        cfg.sim.physics.gpu_max_rigid_contact_count = 131072
        cfg.sim.physics.gpu_max_rigid_patch_count = 32768
        cfg.sim.physics.gpu_found_lost_pairs_capacity = 131072
        cfg.sim.physics.gpu_found_lost_aggregate_pairs_capacity = 131072
        cfg.sim.physics.gpu_total_aggregate_pairs_capacity = 262144

    def initial_action(self):
        from yam_arena.constants import READY
        return np.r_[READY,1.,READY,1.]

    def reset(self, base):
        super().reset(base)
        self._reset_counters()
        self._controller = None

    def validate_reset(self, base):
        state = self.states(base)
        for name in ('beater','guard'):
            p,r,v = state[name]
            if np.linalg.norm(p-self.poses[name])>.015 or np.linalg.norm(v[:3])>.04:
                raise RuntimeError('Unstable mixer reset: '+name)

    def begin_trial(self, base):
        self.validate_reset(base)
        print('MIXER_NATIVE_PHYSICS '+json.dumps(dict(device=str(base.device),dt=base.physics_dt,
              solver=base.sim.cfg.physics.solver_type,
              joint_names=base.scene['mixer'].joint_names,
              joint_damping=base.scene['mixer'].data.joint_damping[0].cpu().tolist())),flush=True)
        spec=importlib.util.spec_from_file_location('_mixer_contacts',self.folder/'contacts.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self._contacts=module.NativeContacts(base)
        self._contact_tick=0

    def mechanical_state(self, state):
        cfg = self.cfg
        p,r,v = state['beater'];sp,sr,sv = state['mixer/shaft']
        relative = sr.T@(p-sp)
        rr = sr.T@r
        yaw = math.atan2(rr[1,0],rr[0,0])
        yaw_error = math.atan2(math.sin(yaw-cfg['hub_yaw']),math.cos(yaw-cfg['hub_yaw']))
        retained = (np.linalg.norm(relative[:2])<.0005
                    and abs(relative[2]-cfg['hub_rest_offset'])<.0006
                    and rr[2,2]>math.cos(math.radians(3)) and abs(yaw_error)<math.radians(10))
        bp,br,bv = state['mixer/bowl'];gp,gr,gv = state['guard']
        guard_rel = br.T@(gp-bp)
        guard_r = br.T@gr
        guard_yaw = math.atan2(guard_r[1,0],guard_r[0,0])
        seated = (np.linalg.norm(guard_rel[:2])<.003 and abs(guard_rel[2]-cfg['guard_height'])<.0015
                  and guard_r[2,2]>math.cos(math.radians(2)) and abs(guard_yaw)<math.radians(5))
        lift = bp[2]-(cfg['poses']['mixer'][2]+.070)
        raised = abs(lift-cfg['lift_travel'])<.0015
        return dict(retained_geometry=bool(retained),guard_seated=bool(seated),bowl_raised=bool(raised),
                    hub_xy_error_m=float(np.linalg.norm(relative[:2])),hub_height_error_m=float(relative[2]-cfg['hub_rest_offset']),
                    hub_yaw_error_rad=float(yaw_error),guard_xy_error_m=float(np.linalg.norm(guard_rel[:2])),
                    guard_height_error_m=float(guard_rel[2]-cfg['guard_height']),lift_m=float(lift))

    def before_step(self, base):
        import torch
        if hasattr(self,'_contacts'):
            if self._contact_tick%10==0:self._contacts.sample(self._contact_tick)
            self._contact_tick+=1
        state = self.states(base);m = self.mechanical_state(state)
        start = state['mixer/start'][0][0]-self.cfg['button_rest_x']
        stop = state['mixer/stop'][0][0]-self.cfg['button_rest_x']
        interlock = m['guard_seated'] and m['bowl_raised']
        if not interlock:
            self._motor=False
        if start>=.003 and not self._start_pressed:
            self._motor=bool(interlock);self._start_pressed=True
            self._drive_started |= self._motor
        elif start<.001:self._start_pressed=False
        if stop>=.003:
            self._stopped_after_run |= self._ran and self._motor
            self._motor=False;self._stop_pressed=True
        elif stop<.001:self._stop_pressed=False
        player=base.scene['mixer']
        # Preserve fixed passive-drive rest targets when Lab submits all joint buffers.
        springs=[joint for joint in self.descriptions['mixer']['joints'] if 'target' in joint]
        player.actuators.target_command.set_position_index(
            value=torch.tensor([[joint['target'] for joint in springs]],device=base.device),
            joint_ids=[player.joint_names.index(joint['name']) for joint in springs])
        indices=[player.joint_names.index(x) for x in ('orbit','spin')]
        # Native implicit velocity drives run at the physics rate. The relative
        # spin target produces counterrotation in the world frame.
        self.update_drive_ramp(interlock)
        orbit=-self.cfg['orbit_rpm']*math.tau/60*self._drive_fraction
        spin=(-self.cfg['shaft_rpm']*math.tau/60+self.cfg['orbit_rpm']*math.tau/60)*self._drive_fraction
        player.actuators.target_command.set_velocity_index(
            value=torch.tensor([[orbit,spin]],device=base.device),joint_ids=indices)

    def update_drive_ramp(self, interlock):
        if not interlock:
            self._drive_fraction=self._ramp_start=0.
            self._ramp_tick=0
            self._ramp_enabled=False
            return
        if self._motor!=self._ramp_enabled:
            self._ramp_enabled=self._motor
            self._ramp_start=self._drive_fraction
            self._ramp_tick=0
        self._ramp_tick+=1
        self._drive_fraction=ramp_fraction(self._ramp_start,self._ramp_enabled,self._ramp_tick)

    def score_states(self, state, tcps, rpm, external_beater_contact=False):
        for p,r,v in state.values():
            if not np.isfinite(np.r_[p,r.ravel(),v]).all():raise RuntimeError('Non-finite native mixer state')
        m=self.mechanical_state(state);cfg=self.cfg
        p,r,v=state['beater'];gp,gr,gv=state['guard'];bp,br,bv=state['mixer/bowl']
        for point in (p,gp):
            self._unsafe |= bool(point[2]<cfg['table_top']-.02 or point[2]>1.50 or abs(point[1])>.65 or not -.18<point[0]<.90)
        moving=(self._motor or self._drive_started) and max(abs(rpm['shaft']),abs(rpm['carrier']))>5
        # Operational hand-clearance proxy: hands must stay outside the swept bowl volume.
        intrusion=any(np.linalg.norm(np.asarray(t)[:2]-bp[:2])<.145 and
                      bp[2]-.02<np.asarray(t)[2]<cfg['poses']['mixer'][2]+.350 for t in tcps)
        self._unsafe |= bool(moving and (intrusion or not m['guard_seated'] or not m['bowl_raised']))
        beater_clear=all(min(np.linalg.norm(np.asarray(t)-(p+r@np.array([0,0,z]))) for z in [0,-.04,-.105,-.17])>.075 for t in tcps)
        guard_clear=all(np.linalg.norm(np.asarray(t)-gp)>.19 for t in tcps)
        self._retained_count=self._retained_count+1 if m['retained_geometry'] and beater_clear and not external_beater_contact else 0
        retained=self._retained_count>=20
        self._guard_count=self._guard_count+1 if m['guard_seated'] and guard_clear else 0
        guard_released=self._guard_count>=20
        ready=retained and m['guard_seated'] and m['bowl_raised']
        if not ready:
            self._ran = self._stopped_after_run = False
        rotating=abs(rpm['beater']-cfg['shaft_rpm'])<8 and abs(rpm['shaft']-cfg['shaft_rpm'])<8 and abs(rpm['carrier']-cfg['orbit_rpm'])<5
        good_run=ready and self._motor and rotating and beater_clear and guard_clear and not self._unsafe
        self._running_count=self._running_count+1 if good_run else 0
        self._ran |= self._running_count>=round(cfg['run_seconds']*20)
        stopped=max(abs(x) for x in rpm.values())<1.
        buttons_released=all(state['mixer/'+key][0][0]-cfg['button_rest_x']<.001 for key in ['start','stop'])
        success=bool(ready and guard_released and self._ran and self._stopped_after_run and not self._motor and stopped and beater_clear and guard_clear and buttons_released and not self._unsafe)
        score=0. if self._unsafe else 1. if success else .75 if ready and guard_released else .5 if retained and guard_released else .25 if retained else 0.
        return dict(success=success,score=score,unsafe_event=bool(self._unsafe),beater_retained=bool(retained),
                    guard_released=bool(guard_released),motor_on=bool(self._motor),verified_run=bool(self._ran),run_controls=int(self._running_count),
                    stop_after_run=bool(self._stopped_after_run),
                    beater_rpm=float(rpm['beater']),shaft_rpm=float(rpm['shaft']),orbit_rpm=float(rpm['carrier']),
                    hands_clear=bool(beater_clear and guard_clear),external_beater_contact=bool(external_beater_contact),**m)

    def evaluate(self, base):
        from yam_arena.runtime import tcp_world
        state=self.states(base)
        current={key:state[key if key=='beater' else 'mixer/'+key][1].copy() for key in ['beater','shaft','carrier']}
        rpm={key:0. for key in current}
        if self._previous is not None:
            for key,r in current.items():
                delta=r@self._previous[key].T
                rpm[key]=-math.atan2(delta[1,0]-delta[0,1],delta[0,0]+delta[1,1])/.05*60/math.tau
        self._previous=current
        external=self._contacts.external_beater_contact() if hasattr(self,'_contacts') else False
        metrics=self.score_states(state,[tcp_world(base.scene[k+'_robot']) for k in ['left','right']],rpm,external)
        from resist_isaac.state import tensor
        asset=base.scene['mixer'];joints=tensor(asset.data.joint_pos)[0].cpu().numpy()
        for key in ['bowl_lift','handle_hinge','retaining_spring']:metrics['joint_'+key]=float(joints[asset.joint_names.index(key)])
        return metrics

    def scripted_action(self, action, phase, base):
        if phase == 'native reset hold':
            return action, {}
        if self._controller is None:
            spec=importlib.util.spec_from_file_location('_mixer_controller',self.folder/'scripted.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            self._controller=module.Controller(self)
        return self._controller.action(action,phase,base)


def make_task(condition):
    if condition not in CONDITIONS:raise ValueError(condition)
    return MixerTask(Path(__file__).parent)


def create_assets(directory, condition='recoverable'):
    return export_assets(Path(__file__).parent,directory)
