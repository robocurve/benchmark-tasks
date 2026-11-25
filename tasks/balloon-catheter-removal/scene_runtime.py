"""Live PhysX mechanics. Never copies authored object poses to fake a grasp."""
from pathlib import Path
import numpy as np
from mechanics import BODY_PATHS, ROOT, DrainState, rotation, SYRINGE_TIP

class SceneRuntime:
    def __init__(self, stage, sim, root=ROOT):
        from pxr import UsdGeom, UsdPhysics
        self.stage,self.sim=stage,sim
        self.root=root
        self.paths={k:root+v.removeprefix(ROOT) for k,v in BODY_PATHS.items()}
        self.state=DrainState()
        self.views={k:sim.physics_manager._view.create_rigid_body_view(v) for k,v in self.paths.items()}
        self.balloon=UsdGeom.Sphere(stage.GetPrimAtPath(root+'/Catheter/InflatedBalloon'))
        self.inflated=self.balloon
        self.collapsed=UsdGeom.Sphere(stage.GetPrimAtPath(root+'/Catheter/CollapsedBalloon'))
        self.collapsed_mode=False
        self.drive=UsdPhysics.DriveAPI(stage.GetPrimAtPath(root+'/Syringe/PlungerJoint'),'linear')
        self.connection=None
        self.last_radius=.015
        self.last_metrics={}
        self._aligned_frames=0

    def poses(self):
        poses={k:np.array(v.get_transforms().numpy()[0],dtype=float) for k,v in self.views.items()}
        if not all(x.shape==(7,) and np.isfinite(x).all() for x in poses.values()):
            raise RuntimeError('Missing/nonfinite PhysX body transforms')
        return poses

    def update(self, dt):
        from mechanics import connector_metrics
        from pxr import UsdPhysics, Gf
        poses=self.poses()
        metrics=connector_metrics(poses)
        speed=max(float(np.linalg.norm(v.get_velocities().numpy()[0,:3]))
                  for k,v in self.views.items() if k in ('valve','barrel'))
        self._aligned_frames=self._aligned_frames+1 if metrics['aligned'] and speed<.10 else 0
        if not self.state.ever_connected and self._aligned_frames*dt >= .15 and self.state.connect(poses):
            # Join the currently aligned bodies at their measured pose. No snap,
            # pose teleport or hidden kinematic grasp is used.
            # Short, force-limited axial detent: the robot can pull the receiver
            # out. A fixed joint plus an XR/terminal release is not an action
            # available to the robot and cannot be used in a scored trial.
            joint=UsdPhysics.PrismaticJoint.Define(self.stage,self.root+'/ReceiverConnection')
            joint.CreateBody0Rel().SetTargets([self.paths['valve']])
            joint.CreateBody1Rel().SetTargets([self.paths['barrel']])
            v,b=poses['valve'],poses['barrel']
            local=rotation(v[3:7]).T@(b[:3]-v[:3])
            relative=Gf.Quatd(float(v[6]),Gf.Vec3d(*map(float,v[3:6]))).GetInverse()*Gf.Quatd(float(b[6]),Gf.Vec3d(*map(float,b[3:6])))
            joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,local)))
            joint.CreateLocalRot0Attr(Gf.Quatf(relative))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1,Gf.Vec3f(0,0,0)))
            joint.CreateCollisionEnabledAttr(False)
            joint.CreateAxisAttr('X')
            joint.CreateLowerLimitAttr(-.040)
            joint.CreateUpperLimitAttr(.006)
            detent=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear')
            detent.CreateTargetPositionAttr(0.)
            detent.CreateStiffnessAttr(80.)
            detent.CreateDampingAttr(.5)
            detent.CreateMaxForceAttr(1.)
            self.connection=joint
            # Pressure surrogate returns fluid into an empty free receiver.
            # The receiver moves under a finite-force joint drive; the balloon
            # volume is computed from its measured stroke, not elapsed time.
            self.drive.CreateTargetPositionAttr(-.054)
            self.drive.CreateStiffnessAttr(6.)
            self.drive.CreateDampingAttr(1.)
            self.drive.CreateMaxForceAttr(.35)
        self.last_metrics=self.state.update(poses)
        if self.connection is not None and not self.state.connected:
            # Topology release follows measured 12 mm pull-off travel, not a
            # timer/controller command; the object pose is never changed here.
            self.connection.CreateJointEnabledAttr(False)
            self.drive.CreateStiffnessAttr(0.)
            self.connection=None
        wants_collapsed=self.state.fraction>=.995
        if wants_collapsed!=self.collapsed_mode:
            # PhysX does not live-rebuild sphere geometry from a radius edit.
            # Switch two pre-authored render/collision states together instead.
            UsdPhysics.CollisionAPI(self.inflated.GetPrim()).CreateCollisionEnabledAttr(not wants_collapsed)
            self.inflated.CreateVisibilityAttr('invisible' if wants_collapsed else 'inherited')
            UsdPhysics.CollisionAPI(self.collapsed.GetPrim()).CreateCollisionEnabledAttr(wants_collapsed)
            self.collapsed.CreateVisibilityAttr('inherited' if wants_collapsed else 'invisible')
            self.balloon=self.collapsed if wants_collapsed else self.inflated
            self.collapsed_mode=wants_collapsed
        self.last_metrics['volume_equivalent_radius_m']=self.state.radius
        self.last_metrics['balloon_radius_m']=.004 if self.collapsed_mode else .015
        self.last_metrics['collapsed']=self.collapsed_mode
        self.last_metrics['collision_mode']='collapsed' if self.collapsed_mode else 'inflated'
        if self.last_metrics['balloon_center'][1]<.035 and not self.collapsed_mode:
            self.state.failed=True;self.state.reason='inflated_balloon_passed_throat'
            self.last_metrics.update(failed=True,failure=self.state.reason)
        return self.last_metrics

    def detach(self):
        # Compatibility for old operator frontends: a command cannot unlock the
        # connector. Pull the actual receiver axially by 12 mm instead.
        return False
