"""Disclosed no-slip pinch approximation, implemented as a compliant, force-limited native joint.

A material element must be between closing finger faces and inside their real
working area. The joint starts at the measured pose, without moving either body.
Its world anchor follows the observed hand, without changing task body poses.
It releases on opening and breaks at 18 N or 0.24 Nm. This approximates unresolved
finger friction; it does not move, fold, fit or score task material for the robot.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def candidate(point, radius, hand, thickness, half_width, z_range):
    local=(np.asarray(point)-hand['tcp'])@hand['rotation']
    q=np.asarray(hand.get('finger_joint_positions',[-hand['opening']*.04695]*2))
    left=q[0]+thickness;right=-q[1]-thickness
    # Opposed contact, allowing 1 mm solver/contact margin on either face.
    if abs((local[0]-radius)-left)>.001 or abs((local[0]+radius)-right)>.001:
        return None
    if abs(local[1])>half_width or not z_range[0]<=local[2]<=z_range[1]:return None
    return float(np.linalg.norm(local))


class FinitePinch:
    def __init__(self,model,names,radius,thickness,half_width,z_range):
        from pxr import UsdPhysics,PhysicsSchemaTools
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import SimulationEvent
        self.m=model;self.names=list(names);self.radii=radius if isinstance(radius,dict) else {name:radius for name in names};self.thickness=thickness
        self.half_width=half_width;self.z_range=z_range;self.held={};self.history={s:[] for s in ['left','right']};self.blocked=set();self.serial=0
        self.grippers={};self.body_paths={}
        for p in model.stage.Traverse():
            if not p.HasAPI(UsdPhysics.RigidBodyAPI):continue
            path=str(p.GetPath())
            if p.GetName()=='gripper':
                for side,arm in [('left','LeftArm'),('right','RightArm')]:
                    if arm in path:self.grippers[side]=path
            if '/materials/' in path and p.GetName() in self.names:self.body_paths[p.GetName()]=path
        assert set(self.grippers)=={'left','right'} and len(self.body_paths)==len(self.names)
        self.root=next(iter(self.body_paths.values())).rsplit('/',1)[0]
        def event(e):
            if e.type!=int(SimulationEvent.JOINT_BREAK):return
            path=str(PhysicsSchemaTools.decodeSdfPath(*e.payload['jointPath']))
            for side,g in list(self.held.items()):
                if path==g['path']:
                    self.m.events.append(dict(event='pinch_slipped',side=side,body=g['body'],time=self.m.time,force_limit_n=18.,torque_limit_nm=.24))
                    g['joint'].GetJointEnabledAttr().Set(False);self.held.pop(side);self.blocked.add(side)
        self.subscription=get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(event)

    def update(self,hands):
        from pxr import UsdPhysics,Sdf,Gf
        from yam_arena.constants import TCP_OFFSET
        for side,h in hands.items():
            history=self.history[side];history.append(float(h['opening']));del history[:-8]
            q=np.asarray(h.get('finger_joint_positions',[-h['opening']*.04695]*2))
            aperture=-q.sum()-2*self.thickness
            radius=self.radii[self.held[side]["body"]] if side in self.held else max(self.radii.values())
            if aperture>2*radius+.002:
                if side in self.held:
                    g=self.held.pop(side);g['joint'].GetJointEnabledAttr().Set(False)
                    self.m.events.append(dict(event='pinch_released',side=side,body=g['body'],time=self.m.time))
                if h['opening']>.30:self.blocked.discard(side)
                continue
            if side in self.held:
                g=self.held[side];now=(self.m.p(g['body'])-h['tcp'])@h['rotation']
                if np.linalg.norm(now-g['local_point'])>.008:
                    g['joint'].GetJointEnabledAttr().Set(False);self.held.pop(side);self.blocked.add(side)
                    self.m.events.append(dict(event='pinch_slipped',side=side,body=g['body'],time=self.m.time,reason='material moved 8 mm through the grip'))
            if side in self.held:
                g=self.held[side];target=np.asarray(h['tcp'])+h['rotation']@g['local_point']
                q=Rotation.from_matrix(h['rotation']@g['local_rotation']).as_quat()
                with Sdf.ChangeBlock():
                    g['joint'].GetLocalPos0Attr().Set(Gf.Vec3f(*target))
                    g['joint'].GetLocalRot0Attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
            if side in self.held or side in self.blocked or max(history)-h['opening']<.008:continue
            used={g['body'] for g in self.held.values()};choices=[]
            for name in self.names:
                if name in used:continue
                point=self.m.p(name)
                if name=='brush':
                    # The long rectangular handle crosses the pads even when
                    # the body origin at the brush head lies outside them.
                    axis=self.m.orientation(name)[:,1]
                    if abs(float(axis@h['rotation'][:,0]))>.2:continue
                    along=float(np.clip((h['tcp']-point)@axis,.0425,.1375))
                    point=point+along*axis
                distance=candidate(point,self.radii[name],h,self.thickness,self.half_width,self.z_range)
                if distance is not None:choices.append((distance,name))
            if not choices:continue
            _,name=min(choices);self.serial+=1;path=self.root+f'/pinch_constraint_{side}_{self.serial}'
            joint=UsdPhysics.Joint.Define(self.m.stage,path)
            joint.CreateJointEnabledAttr(False)
            with Sdf.ChangeBlock():
                joint.CreateExcludeFromArticulationAttr(True)
                joint.CreateBody1Rel().SetTargets([Sdf.Path(self.body_paths[name])])
                p=self.m.p(name);q=Rotation.from_matrix(self.m.orientation(name)).as_quat()
                joint.CreateLocalPos0Attr(Gf.Vec3f(*p));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
                joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])));joint.CreateLocalRot1Attr(Gf.Quatf(1.,Gf.Vec3f(0,0,0)))
                joint.CreateBreakForceAttr(18.);joint.CreateBreakTorqueAttr(.24);joint.CreateCollisionEnabledAttr(True)
                # A force-limited friction approximation, not an infinitely stiff weld.
                for axis in ('transX','transY','transZ','rotX','rotY','rotZ'):
                    linear=axis.startswith('trans')
                    if linear:
                        limit=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis);limit.CreateLowAttr(-.05);limit.CreateHighAttr(.05)
                    drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis);drive.CreateTypeAttr('force')
                    drive.CreateStiffnessAttr(1500. if linear else 2.0*np.pi/180.)
                    drive.CreateDampingAttr(.5 if linear else .02*np.pi/180.)
                    drive.CreateMaxForceAttr((18. if linear else 0.24)/np.sqrt(3.))
                    drive.CreateTargetPositionAttr(0.)
                joint.GetJointEnabledAttr().Set(True)
            self.held[side]=dict(body=name,path=path,joint=joint,local_point=((self.m.p(name)-h['tcp'])@h['rotation']).copy(),local_rotation=h['rotation'].T@self.m.orientation(name))
            self.m.events.append(dict(event='pinch_captured',side=side,body=name,time=self.m.time,force_limit_n=18.,torque_limit_nm=.24,local_point=((self.m.p(name)-h['tcp'])@h['rotation']).tolist(),opening=float(h['opening'])))
        self.m.grasp_state={side:dict(body=g['body'],force_limit_n=18.,torque_limit_nm=.24) for side,g in self.held.items()}
