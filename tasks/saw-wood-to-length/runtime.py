"""Task-local mechanics hooks. Demo writes only public robot action targets."""
from __future__ import annotations
import math
from collections import deque
import numpy as np

ENGINE=None
RESET_SOURCE_ENV=None

def spawn_contact(stage,path,spec):
    from pxr import PhysxSchema,Usd,UsdPhysics
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
    if spec.name=='saw':
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material
        material=sim_utils.RigidBodyMaterialCfg(static_friction=1.6,dynamic_friction=1.4)
        material_path=path.rsplit('/body',1)[0]+'/Looks/grip_physics'
        material.func(material_path,material)
        for shape in ('box_0','box_3'):
            bind_physics_material(path+'/'+shape,material_path,stage=stage)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
        if prim.HasAPI(UsdPhysics.CollisionAPI) and (spec.name=='wood' or prim.GetName() in ['box_1','box_2']):
            PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(.000005)

def spawn_mechanism(stage,path,spec):
    from pxr import UsdPhysics,PhysxSchema,Gf,Sdf
    kind=UsdPhysics.PrismaticJoint if spec.name=='jaw' else UsdPhysics.RevoluteJoint
    joint=kind.Define(stage,path+'/mechanism_joint')
    joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
    joint.CreateAxisAttr('Y');joint.CreateLocalPos0Attr(Gf.Vec3f(*spec.pos));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    if spec.name=='jaw':joint.CreateLowerLimitAttr(-.020);joint.CreateUpperLimitAttr(.030)
    else:
        joint.CreateLowerLimitAttr(-10.);joint.CreateUpperLimitAttr(720.)
        # JointFrictionAttr applies only to articulations. This independent
        # screw uses a solver-level velocity drive with zero spring target.
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(.20*math.pi/180)
        drive.CreateTargetVelocityAttr(0.)
        drive.CreateMaxForceAttr(1.)
    base=path.rsplit('/',2)[0]
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path)).CreateFilteredPairsRel().SetTargets([Sdf.Path(base+'/vise/body')])

def install():
    # Task-local configuration wrapper; shared SDK files stay unchanged.
    # Scene imports Isaac modules; wait for its normal post-AppLauncher import.
    import partkit,sys
    previous=partkit._Finder.targets['yam_arena.scene']
    def patch(scene):
        previous(scene)
        old=scene.env_cfg_callback
        if getattr(old,'_saw480',False):return
        def cfg(control_hz=None):
            original=old(control_hz)
            def apply(c):
                c=original(c);c.sim.dt=1/480
                c.decimation=round(480/(control_hz or 20));c.sim.render_interval=c.decimation
                c.sim.use_fabric=False
                return c
            return apply
        cfg._saw480=True;scene.env_cfg_callback=cfg
    partkit._Finder.targets['yam_arena.scene']=patch
    if 'yam_arena.scene' in sys.modules:patch(sys.modules['yam_arena.scene'])

def reset(environment):
    global ENGINE,RESET_SOURCE_ENV
    if ENGINE is not None and ENGINE.base is environment.unwrapped and ENGINE.t==0:
        return
    same_scene=RESET_SOURCE_ENV is environment
    ENGINE=Mechanism(environment)
    RESET_SOURCE_ENV=None
    print('SAW_SAME_SCENE_RESET',same_scene,flush=True)
    if same_scene:ENGINE.settle_reset(environment)

def clear():
    global ENGINE,RESET_SOURCE_ENV
    if ENGINE is not None:
        RESET_SOURCE_ENV=ENGINE.environment
        ENGINE.cutting.teardown()
    ENGINE=None

def score():return 0. if ENGINE is None else ENGINE.task_score
def distance():return 1. if ENGINE is None else ENGINE.remaining

def advance_angle(previous_angle,previous_raw,current_raw):
    """Keep a revolute screw's turns when its quaternion crosses 2π."""
    return previous_angle+math.remainder(current_raw-previous_raw,2*math.pi)

class Mechanism:
    def __init__(self,environment):
        from pxr import UsdUtils,UsdPhysics,PhysxSchema,Gf,Sdf
        import omni.physics.tensors as tensors
        self.environment=environment
        self.base=environment.unwrapped;self.sim=self.base.sim
        self.stage=self.sim.stage;self.task_score=0.;self.remaining=1.
        self.sim.set_setting('/physics/disableContactProcessing',False)
        self.paths={}
        for prim in self.stage.Traverse():
            p=str(prim.GetPath())
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                for name in ['wood','saw','jaw','screw','vise']:
                    if '/'+name+'/' in p or p.endswith('/'+name):self.paths[name]=p
        print('SAW_PATHS',self.paths,flush=True)
        for name in ['wood','saw','jaw','screw']:assert name in self.paths
        self.sv=tensors.create_simulation_view('numpy',stage_id=UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt())
        self.sv.set_subspace_roots('/')
        self.views={k:self.sv.create_rigid_body_view(v) for k,v in self.paths.items()}
        self.contacts=self.sv.create_rigid_contact_view(self.paths['saw'],filter_patterns=[self.paths['wood']],max_contact_data_count=64)
        self.t=0.;self.last_fn=0.;self.rows=[]
        self.screw_angles=deque(maxlen=25)
        self.last_screw_raw=None;self.screw_angle=0.
        from cutting_runtime import Cutting
        self.cutting=Cutting(self)
        if not hasattr(self.sim,'_saw_original_step'):
            self.sim._saw_original_step=self.sim.step
            def step(*a,**k):
                result=self.sim._saw_original_step(*a,**k)
                if ENGINE is not None:ENGINE.update()
                return result
            self.sim.step=step
        print('SAW_RUNTIME_READY',flush=True)

    def settle_reset(self,environment):
        """Return the free jaw and wood to their authored start state.

        The embodiment's in-place reset settles before task on_reset hooks.
        The jaw has no task drive during that interval and can drift to its
        prismatic limit. Settle with the task drive active, then restore the
        intact free wood exactly and check that it remains supported.
        """
        import torch
        from yam_arena.constants import READY
        action=torch.tensor([[*READY,1.,*READY,1.]],device=self.base.device,dtype=torch.float32)
        for _ in range(20):environment.step(action)
        wood=self.base.scene['wood']
        pose=wood.data.default_root_pose.torch.clone()
        pose[:,:3]+=self.base.scene.env_origins
        wood.write_root_pose_to_sim_index(root_pose=pose)
        wood.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(wood.data.default_root_vel.torch))
        for _ in range(10):environment.step(action)
        self.t=0.;self.rows.clear();self.screw_angles.clear()
        self.last_screw_raw=None;self.screw_angle=0.;self.last_fn=0.
        from cutting_runtime import Cutting
        self.cutting=Cutting(self)

    def update(self):
        dt=1/480;self.t+=dt
        jaw=self.views['jaw'];screw=self.views['screw']
        q=screw.get_transforms()[0,3:7]
        raw=2*math.atan2(float(q[1]),float(q[3]))
        self.screw_angle=(raw if self.last_screw_raw is None else
                          advance_angle(self.screw_angle,self.last_screw_raw,raw))
        self.last_screw_raw=raw
        angle=self.screw_angle
        pos=jaw.get_transforms()[0,:3];v=jaw.get_velocities()[0,:3]
        # Positive rotation advances the jaw toward +Y, with self-locking lead.
        target=-.0575+(.006/(2*math.pi))*max(0,angle)
        force=float(np.clip(40000*(target-pos[1])-80*v[1],-100,100))
        jaw.apply_forces_and_torques_at_position(np.array([[0,force,.35*9.81]],np.float32),None,None,np.array([0],np.int32),True)
        # A real vise screw is self-locking under its jaw load. Cancel only
        # that back-drive torque (at most 0.12 N m), with passive damping on
        # measured shaft motion. A spring reference made the released knob
        # spin back; the held position must have zero restoring torque.
        # Quaternion estimates can oscillate between physics substeps. A
        # 50 ms pose difference gives the signed shaft rate of the visible
        # screw; a single-substep difference produced the wrong sign while
        # the released screw was moving.
        self.screw_angles.append(angle)
        span=len(self.screw_angles)-1
        rate=0. if span==0 else (((angle-self.screw_angles[0]+math.pi)%(2*math.pi)-math.pi)/(span*dt))
        load=-force*.006/(2*math.pi)
        self_lock=-float(np.clip(load,-.12,.12))
        torque=load+self_lock
        if abs(torque)>1e-8:
            screw.apply_forces_and_torques_at_position(None,np.array([[0,torque,0]],np.float32),None,np.array([0],np.int32),True)
        self.last_fn=max(0,float(self.contacts.get_contact_force_matrix(dt=dt).reshape(-1,3)[:,2].sum()))
        self.cutting.step(self.last_fn)
        self.task_score=self.cutting.value;self.remaining=self.cutting.distance
        if round(self.t/dt)%24==0:
            self.rows.append({'t':self.t,'screw_angle':angle,'screw_rate':rate,'jaw_y':float(pos[1]),
                'jaw_velocity_y':float(v[1]),'jaw_force':force,'screw_reaction_torque':torque,
                'screw_self_lock_torque':self_lock,
                'cut_force':self.last_fn,
                'wood_pose':self.views['wood'].get_transforms()[0].tolist(),'saw_pose':self.views['saw'].get_transforms()[0].tolist()})
