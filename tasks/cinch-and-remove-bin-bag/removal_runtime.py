"""Force callback and geometry observations shared by the evaluation adapter."""
import numpy as np
from drawstring_physics import SlidingDrawstring
from removal_assets import BLOCK_COUNT

EXIT_INDEX=5      # hem markers 5 and 15: the two opposite cut-outs
# Inertia used by the implicit string solve: hem bead plus the film it drags
# (about 2 g) and strap end plus the strap behind it (about 2 g). With the bare
# link masses the one-step solve lets the string stretch by several cm.
SOLVER_HEM_MASS=.0015
SOLVER_END_MASS=.002
STRING_COLOR=(.80,.80,.78)

def tensor(value):
    return value.torch if hasattr(value,"torch") else value

def array(value):
    if hasattr(value,'torch'):value=value.torch
    return value.detach().cpu().numpy()

class DrawstringRuntime:
    def __init__(self,env):
        import torch
        import warp as wp
        from isaaclab_physx.physics import PhysxManager
        import omni.usd
        from pxr import UsdGeom,Gf,Vt
        self.base=env.unwrapped;self.torch=torch;self.Vt=Vt;self.count=0
        from loop_chain import _SPECS as CHAINS
        hem=[self.base.scene.rigid_objects[f'hem_{j:02d}'] for j in range(20)]
        # A single PhysX view avoids separate wrench composers per substep.
        # Only forces are written; object poses and velocities are never set.
        paths=[o.root_view.prim_paths[0] for o in hem]
        root=paths[0].rsplit('/',1)[0]
        # The string acts on the two end links of each strap (its strand ends).
        for k in range(2):
            last=CHAINS[f'loop_{k}'].segments-1
            paths+=[f'{root}/loop_{k}/seg_000',f'{root}/loop_{k}/seg_{last:03d}']
        self.view=PhysxManager.get_physics_sim_view().create_rigid_body_view(paths)
        actual=list(self.view.prim_paths)
        if self.view.count!=len(paths) or set(actual)!=set(paths):raise RuntimeError('Cord body view mismatch')
        self.order=np.array([actual.index(path) for path in paths])
        self.wp=wp;self.indices=wp.array(np.arange(len(paths),dtype=np.int32),dtype=wp.int32,device=self.base.device)
        self.zero_torque=wp.zeros(len(paths)*3,dtype=wp.float32,device=self.base.device)
        self.cable=SlidingDrawstring(self.positions(),exit_index=EXIT_INDEX,hem_mass=SOLVER_HEM_MASS,end_mass=SOLVER_END_MASS)
        stage=omni.usd.get_context().get_stage()
        # Visual-only polyline showing the string path (hem, cut-outs, loops).
        c=UsdGeom.BasisCurves.Define(stage,'/World/drawstring')
        self.curves=[]
        for i,r in enumerate(self.cable.routes):
            c=UsdGeom.BasisCurves.Define(stage,f'/World/drawstring_{i}')
            c.CreateTypeAttr('linear');c.CreateCurveVertexCountsAttr([len(r)]);c.CreateWidthsAttr([.003]);c.SetWidthsInterpolation('constant');c.CreateDisplayColorAttr([Gf.Vec3f(*STRING_COLOR)]);self.curves.append(c)
        self.original=self.base.scene.write_data_to_sim
        self.base.scene.write_data_to_sim=self.write_forces
    def positions(self):return self.view.get_transforms().numpy()[self.order,:3]
    def detach(self):self.base.scene.write_data_to_sim=self.original
    def write_forces(self):
        pos=self.positions();vel=self.view.get_velocities().numpy()[self.order,:3];force=self.cable.forces(pos,vel,dt=float(self.base.cfg.sim.dt))
        if not np.isfinite(force).all() or np.abs(force).max()>10:raise RuntimeError('Invalid drawstring force')
        if self.count%48==0:
            for c,r in zip(self.curves,self.cable.routes):c.GetPointsAttr().Set(self.Vt.Vec3fArray.FromNumpy(pos[r].astype(np.float32)))
        raw_force=np.empty_like(force);raw_force[self.order]=force
        self.force_wp=self.wp.array(raw_force.astype(np.float32).ravel(),dtype=self.wp.float32,device=self.base.device)
        self.count+=1;self.original()
        self.view.apply_forces_and_torques_at_position(force_data=self.force_wp,torque_data=self.zero_torque,position_data=None,indices=self.indices,is_global=True)

def observation(env,step,mouth_ids):
    """Recorded geometry for the judge: film mesh, blocks, strap links, hem
    markers, and each gripper's pose and finger opening. String tension is
    recorded for diagnostics only; the judge does not use it."""
    base=env.unwrapped
    vertices=array(base.scene.deformable_objects['loaded_bag'].data.nodal_pos_w)[0].copy()
    neck=vertices[mouth_ids]
    state={'step':step,'mouth_diameter':float(np.linalg.norm(neck[:,None]-neck[None,:],axis=2).max())}
    for key,attribute in [('rubbish','root_pos_w'),('rubbish_quaternions','root_quat_w')]:
        state[key]=[array(getattr(base.scene.rigid_objects[f'rubbish_{i}'].data,attribute))[0].tolist() for i in range(BLOCK_COUNT)]
    from yam_arena.runtime import tcp_world,link_poses
    robots=[base.scene.articulations[name] for name in ['left_robot','right_robot']]
    state['tcp']=[tcp_world(r).tolist() for r in robots]
    state['gripper_poses']=[link_poses(r)['gripper'].tolist() for r in robots]
    state['finger_joints']=[finger_joints(r) for r in robots]
    state['finger_opening']=[finger_opening(r) for r in robots]
    from loop_chain import chain_points
    state['loop_points']=[chain_points(env,f'loop_{k}').tolist() for k in range(2)]
    state['hem_marker_positions']=[array(base.scene.rigid_objects[f'hem_{j:02d}'].data.root_pos_w)[0].tolist() for j in range(20)]
    state['rubbish_velocities']=[array(base.scene.rigid_objects[f'rubbish_{i}'].data.root_lin_vel_w)[0].tolist() for i in range(BLOCK_COUNT)]
    return state,vertices

def finger_joints(robot):
    ids=robot.find_joints(['joint7','joint8'],preserve_order=True)[0]
    return [float(x) for x in array(robot.data.joint_pos)[0][ids]]

def finger_opening(robot):
    """Distance between the two finger pads (m), from the two prismatic finger joints."""
    ids=robot.find_joints(['joint7','joint8'],preserve_order=True)[0]
    q=array(robot.data.joint_pos)[0][ids]
    # Each finger joint runs from 0 (closed, pads touching) to -0.04695 (open).
    return float(max(0.,-float(q[0])-float(q[1])))
