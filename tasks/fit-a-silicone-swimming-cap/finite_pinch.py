"""Disclosed no-slip pinch approximation, implemented as a compliant, force-limited native joint.

A material element must be between closing finger faces and inside their real
working area. The joint starts at the measured pose, without moving either body.
The joint connects directly to the native gripper body, so it follows every physics substep and transmits reaction forces.
It releases on opening and breaks at 40 N or 0.08 Nm. This approximates unresolved
finger friction; it does not move, fold, fit or score task material for the robot.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def candidate(point, radius, hand, thickness, half_width, z_range, holding=False):
    local=(np.asarray(point)-hand['tcp'])@hand['rotation']
    q=np.asarray(hand.get('finger_joint_positions',[-hand['opening']*.04695]*2))
    left=q[0]+thickness;right=-q[1]-thickness
    # Opposed contact, allowing 1 mm solver/contact margin on either face.
    if holding:
        # Compression into a face is still contact. A rolled silicone rim
        # must continue spanning both pad faces; an air gap loses the grip.
        if local[0]-radius>left+.001 or local[0]+radius<right-.001:return None
    elif abs((local[0]-radius)-left)>.001 or abs((local[0]+radius)-right)>.001:
        return None
    if abs(local[1])>half_width or not z_range[0]<=local[2]<=z_range[1]:return None
    return float(np.linalg.norm(local))


class FinitePinch:
    def __init__(self,model,names,radius,thickness,half_width,z_range):
        from pxr import UsdPhysics,PhysicsSchemaTools
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import SimulationEvent
        self.m=model;self.names=list(names);self.radius=radius;self.thickness=thickness
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
                if any(path==item['path'] for item in g['items']):self.release(side,'native grip load limit')
        self.subscription=get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(event)

    def release(self,side,reason):
        g=self.held.pop(side)
        for item in g['items']:item['joint'].GetJointEnabledAttr().Set(False)
        if reason!='opened':self.blocked.add(side)
        self.m.events.append(dict(event='pinch_released' if reason=='opened' else 'pinch_slipped',side=side,body=g['body'],bodies=[i['body'] for i in g['items']],time=self.m.time,reason=reason))

    def update(self,hands):
        from pxr import UsdPhysics,Sdf,Gf
        from yam_arena.constants import TCP_OFFSET
        for side,h in hands.items():
            history=self.history[side];history.append(float(h['opening']));del history[:-8]
            q=np.asarray(h.get('finger_joint_positions',[-h['opening']*.04695]*2))
            aperture=-q.sum()-2*self.thickness
            if aperture>2*self.radius+.002:
                if side in self.held:self.release(side,'opened')
                if h['opening']>.30:self.blocked.discard(side)
                continue
            if side in self.held:
                g=self.held[side]
                for item in list(g['items']):
                    now=(self.m.p(item['body'])-h['tcp'])@h['rotation']
                    contact=candidate(self.m.p(item['body']),self.radius,h,self.thickness,self.half_width,self.z_range,holding=True)
                    if np.linalg.norm(now-item['local_point'])>.008 or contact is None:
                        item['joint'].GetJointEnabledAttr().Set(False);g['items'].remove(item)
                        self.m.events.append(dict(event='pinch_patch_lost',side=side,body=item['body'],time=self.m.time))
                # A pad can lose an edge contact while retaining the centre.
                # Surviving joints keep their original divided load limits.
                if not g['items']:self.release(side,'all rim contacts lost')
            if side in self.held or side in self.blocked or max(history)-h['opening']<.008:continue
            used={i['body'] for g in self.held.values() for i in g['items']};choices=[]
            for name in self.names:
                if name in used:continue
                distance=candidate(self.m.p(name),self.radius,h,self.thickness,self.half_width,self.z_range)
                if distance is not None:choices.append((distance,name))
            if not choices:continue
            # A broad real pad holds every rim point already inside its faces.
            # Split, rather than multiply, its finite force/torque and stiffness.
            choices.sort();count=len(choices);items=[]
            for _,name in choices:
                self.serial+=1;path=self.root+f'/pinch_constraint_{side}_{self.serial}'
                joint=UsdPhysics.Joint.Define(self.m.stage,path);joint.CreateJointEnabledAttr(False)
                local=(self.m.p(name)-h['tcp'])@h['rotation']
                with Sdf.ChangeBlock():
                    joint.CreateExcludeFromArticulationAttr(True)
                    joint.CreateBody0Rel().SetTargets([Sdf.Path(self.grippers[side])]);joint.CreateBody1Rel().SetTargets([Sdf.Path(self.body_paths[name])])
                    p=np.asarray(TCP_OFFSET)+local;q=Rotation.from_matrix(h['rotation'].T@self.m.orientation(name)).as_quat()
                    joint.CreateLocalPos0Attr(Gf.Vec3f(*p));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
                    joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])));joint.CreateLocalRot1Attr(Gf.Quatf(1.,Gf.Vec3f(0,0,0)))
                    joint.CreateBreakForceAttr(40./count);joint.CreateBreakTorqueAttr(.08/count);joint.CreateCollisionEnabledAttr(True)
                    for axis in ('transX','transY','transZ','rotX','rotY','rotZ'):
                        linear=axis.startswith('trans')
                        if linear:
                            limit=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis);limit.CreateLowAttr(-.05);limit.CreateHighAttr(.05)
                        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis);drive.CreateTypeAttr('force')
                        drive.CreateStiffnessAttr((40000. if linear else .25*np.pi/180.)/count)
                        drive.CreateDampingAttr((4. if linear else .005*np.pi/180.)/count)
                        drive.CreateMaxForceAttr((40. if linear else .08)/(count*np.sqrt(3.)));drive.CreateTargetPositionAttr(0.)
                    joint.GetJointEnabledAttr().Set(True)
                items.append(dict(body=name,path=path,joint=joint,local_point=local.copy()))
            self.held[side]=dict(body=items[0]['body'],items=items)
            self.m.events.append(dict(event='pinch_captured',side=side,body=items[0]['body'],bodies=[i['body'] for i in items],time=self.m.time,total_force_limit_n=40.,total_torque_limit_nm=.08,opening=float(h['opening'])))
        self.m.grasp_state={side:dict(body=g['body'],contact_bodies=[i['body'] for i in g['items']],force_limit_n=40.,torque_limit_nm=.08) for side,g in self.held.items()}
