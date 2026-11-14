"""Rigid one-way pawl with finite take-up and dissipative return drag.

The native hard twist limit transmits one impulse sign. Its unloaded return
reference changes only about the free shaft axis, without moving either body.
Metal elasticity, tooth impacts, wear and wrench breakage are not modeled.
"""
import math
import numpy as np
from ratchet_task.mechanics import Ratchet
class NativeClutch:
    def __init__(self,stage):
        from pxr import UsdPhysics,Sdf,Gf
        import omni.physx
        from yam_arena.urdf import quat_xyzw_to_matrix
        self.law=Ratchet();self.native=omni.physx.get_physx_interface();self.enabled=True
        base='/World/envs/env_0/';self.path=base+'pawl'
        h=self.native.get_rigidbody_transformation(base+'handle');p=self.native.get_rigidbody_transformation(self.path)
        assert h['ret_val'] and p['ret_val'],'Ratchet bodies must be live before hardware initialization'
        hq=np.array(h['rotation']);pq=np.array(p['rotation']);m=quat_xyzw_to_matrix(hq).T@quat_xyzw_to_matrix(pq)
        self.last=math.atan2(m[1,0],m[0,0]);self.gate_relative=self.last
        self.gate_phase,_=self.law.update(self.last,0.)
        self.axis=Gf.Quatf(math.sqrt(.5),0.,-math.sqrt(.5),0.);self.previous=None
        gate=UsdPhysics.Joint.Define(stage,base+'PawlGate');gate.CreateBody0Rel().SetTargets([Sdf.Path(base+'handle')]);gate.CreateBody1Rel().SetTargets([Sdf.Path(self.path)]);gate.CreateLocalRot1Attr(self.axis);gate.CreateExcludeFromArticulationAttr(True)
        self.frame=gate.CreateLocalRot0Attr()
        limit=UsdPhysics.LimitAPI.Apply(gate.GetPrim(),'rotX');limit.CreateLowAttr(-179.);limit.CreateHighAttr(0.)
        self.set_frame(hq,pq)
    def set_frame(self,parent_q,child_q):
        from pxr import Gf
        q=Gf.Quatf(math.cos(self.gate_phase/2),0.,0.,math.sin(self.gate_phase/2))
        parent=Gf.Quatf(float(parent_q[3]),Gf.Vec3f(*map(float,parent_q[:3])));world=parent*q
        if np.dot(np.r_[list(world.GetImaginary()),world.GetReal()],child_q)<0:q=Gf.Quatf(-q.GetReal(),-q.GetImaginary())
        parts=np.r_[list(q.GetImaginary()),q.GetReal()]
        if self.previous is None or np.max(abs(parts-self.previous))>1.e-8:self.frame.Set(q*self.axis);self.previous=parts
    def step(self,hr,dr,hq,dq,dt):
        if not self.enabled:return 0.,0.
        from yam_arena.urdf import quat_xyzw_to_matrix
        p=self.native.get_rigidbody_transformation(self.path)
        if not p.get('ret_val'):raise RuntimeError('Missing live pawl body')
        pq=np.array(p['rotation']);m=hr.T@quat_xyzw_to_matrix(pq);raw=math.atan2(m[1,0],m[0,0]);delta=math.atan2(math.sin(raw-self.last),math.cos(raw-self.last))
        self.last=raw;self.gate_relative+=delta
        self.gate_phase,drag=self.law.update(self.gate_relative,delta/dt)
        self.set_frame(hq,pq)
        return 0.,float(drag)
