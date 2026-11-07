"""Live PhysX adapter. Only external forces/torques and passive constraints.

Direct state writes are intentionally absent. Diagnostic fixtures are separate
programs; policy actions cannot invoke the force API or change this hardware.
"""
import math
import numpy as np
from ratchet_task.mechanics import Screw
from ratchet_task.config import *
class Hardware:
    def __init__(self,stage,dt,pitch_sign=1,reverse_hinge=False):
        from pxr import UsdUtils,PhysicsSchemaTools
        import omni.physx
        self.stage=stage;self.dt=dt;self.pitch_sign=pitch_sign;self.reverse_hinge=reverse_hinge;self.native=omni.physx.get_physx_interface();self.sim=omni.physx.get_physx_simulation_interface()
        self.stageid=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.ids={n:PhysicsSchemaTools.sdfPathToInt('/World/envs/env_0/'+n) for n in ['handle','drive','bolt','pawl']}
        from ratchet_task.native_clutch import NativeClutch
        self.clutch=NativeClutch(stage);self.screw=Screw();self.last={};self.angles={};self.candidate=None;self.thread_joint=None
        self.cycles=0;self.return_start=None;self.drive_start=None;self.bolt_cycle_start=None;self.telemetry={};self.enabled=True;self.carrier_wait=0;self.seated_stop=False;self.contact_samples=[];self.contact_peak=0.;self.contact_header_count=0
        from pxr import PhysxSchema
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath('/World/envs/env_0/bolt')).CreateThresholdAttr(0.)
        self.contact_subscription=self.sim.subscribe_full_contact_report_events(lambda *args:None)
    def _contact_report(self,headers,contacts,anchors):
        self.contact_header_count+=len(headers)
        impulse=0.
        b=self.telemetry.get('positions',{}).get('bolt')
        if b is None:return
        for h in headers:
            if {int(h.actor0),int(h.actor1)}!={self.ids['bolt'],self.ids['drive']}:continue
            sign=1. if int(h.actor0)==self.ids['bolt'] else -1.
            pieces=list(contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data])
            pieces+=list(anchors[h.friction_anchors_offset:h.friction_anchors_offset+h.num_friction_anchors_data])
            for c in pieces:
                impulse+=sign*float(np.cross(np.asarray(c.position)-b,np.asarray(c.impulse))[2])
        torque=-impulse/self.dt
        self.contact_samples.append(torque);self.contact_peak=max(self.contact_peak,abs(torque))
        self.telemetry['socket_torque_nm']=float(torque)
    def contact_summary(self,clear=True):
        out=dict(socket_torque_nm=float(np.mean(self.contact_samples)) if self.contact_samples else 0.,socket_torque_peak_nm=self.contact_peak,samples=len(self.contact_samples),contact_headers=self.contact_header_count)
        if clear:self.contact_samples.clear();self.contact_peak=0.;self.contact_header_count=0
        return out
    def after_step(self):
        self._contact_report(*self.sim.get_full_contact_report())
    def install_seat_stop(self,theta,q):
        """Ideal hard seat, expressed as a native angular terminal limit.

        Rephase only the FREE angular coordinate; body poses stay untouched.
        This end stop carries seating torque; it does not estimate clamp preload.
        """
        from pxr import Gf,UsdPhysics,Sdf
        j=UsdPhysics.Joint.Define(self.stage,'/World/envs/env_0/SeatStop')
        frame=Gf.Quatf(math.sqrt(.5),0.,-math.sqrt(.5),0.)
        body=Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3])))
        j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/bolt')])
        j.CreateLocalRot0Attr(frame);j.CreateLocalRot1Attr(body.GetConjugate()*frame)
        j.CreateExcludeFromArticulationAttr(True)
        seat_theta=self.screw.angle0+(AXIS[2]-self.screw.z0)*2*math.pi/PITCH
        limit=UsdPhysics.LimitAPI.Apply(j.GetPrim(),'rotX')
        limit.CreateLowAttr(math.degrees(seat_theta-theta));limit.CreateHighAttr(179.)
        self.seated_stop=True
    def entry_contact(self,enabled):
        from pxr import UsdPhysics,Usd
        root=self.stage.GetPrimAtPath('/World/envs/env_0/entry_stop')
        if root:
            for p in Usd.PrimRange(root):
                if p.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(bool(enabled))
    def release_thread(self):
        for name in ['ThreadPitch','ThreadRotation','ThreadTranslation','ThreadAngleReference','SeatStop','ThreadCarrier']:
            self.stage.RemovePrim('/World/envs/env_0/'+name)
        self.screw.captured=False;self.seated_stop=False;self.thread_joint=None;self.candidate=None;self.carrier_wait=0
        self.last.pop('ThreadCarrier',None);self.angles.pop('ThreadCarrier',None);self.entry_contact(True)
    def pose(self,n):
        from yam_arena.urdf import quat_xyzw_to_matrix
        v=self.native.get_rigidbody_transformation('/World/envs/env_0/'+n)
        if not v.get('ret_val'):raise RuntimeError('Missing live body '+n)
        p=np.array(v['position']);q=np.array(v['rotation']);r=quat_xyzw_to_matrix(q)
        angle=math.atan2(r[1,0],r[0,0]);old=self.last.get(n)
        delta=0. if old is None else math.atan2(math.sin(angle-old[1]),math.cos(angle-old[1]))
        unwrapped=self.angles.get(n,angle)+delta
        velocity=np.zeros(3) if old is None else (p-old[0])/self.dt
        self.last[n]=(p,angle);self.angles[n]=unwrapped
        return p,r,q,unwrapped,delta/self.dt,velocity
    def torque(self,n,t):
        from pxr import Gf
        self.sim.apply_torque(self.stageid,self.ids[n],Gf.Vec3f(0,0,float(t)))
    def force(self,n,f,p):
        from pxr import Gf
        self.sim.apply_force_at_pos(self.stageid,self.ids[n],Gf.Vec3f(0,0,float(f)),Gf.Vec3f(*map(float,p)),'Force')
    def capture(self,p,q,theta):
        from pxr import UsdPhysics,UsdGeom,Gf,Sdf,PhysxSchema
        base='/World/envs/env_0/'
        carrier=UsdGeom.Xform.Define(self.stage,base+'ThreadCarrier')
        carrier.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)))
        UsdPhysics.RigidBodyAPI.Apply(carrier.GetPrim())
        marker=UsdGeom.Sphere.Define(self.stage,base+'ThreadCarrier/CarrierGeometry');marker.CreateRadiusAttr(.0005);marker.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(marker.GetPrim()).CreateCollisionEnabledAttr(False)
        mass=UsdPhysics.MassAPI.Apply(carrier.GetPrim());mass.CreateMassAttr(.014);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1.e-6));mass.CreateCenterOfMassAttr(Gf.Vec3f(0))
        carrier_props=PhysxSchema.PhysxRigidBodyAPI.Apply(carrier.GetPrim());carrier_props.CreateDisableGravityAttr(True);carrier_props.CreateLockedRotAxisAttr(7);carrier_props.CreateLockedPosAxisAttr(3);carrier_props.CreateSolverPositionIterationCountAttr(SOLVER_POSITION);carrier_props.CreateSolverVelocityIterationCountAttr(SOLVER_VELOCITY)
        linear=UsdPhysics.PrismaticJoint.Define(self.stage,base+'ThreadTranslation')
        linear.CreateBody1Rel().SetTargets([carrier.GetPath()]);linear.CreateAxisAttr('Z');linear.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));linear.CreateLocalPos1Attr(Gf.Vec3f(0));linear.CreateExcludeFromArticulationAttr(True)
        j=UsdPhysics.RevoluteJoint.Define(self.stage,base+'ThreadRotation')
        j.CreateBody0Rel().SetTargets([carrier.GetPath()]);j.CreateBody1Rel().SetTargets([Sdf.Path(base+'bolt')]);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(0));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateLocalRot1Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,-q[:3]))));j.CreateExcludeFromArticulationAttr(True);j.CreateLowerLimitAttr(-float('inf'));j.CreateUpperLimitAttr(float('inf'))
        if self.reverse_hinge:
            j.CreateBody0Rel().SetTargets([Sdf.Path(base+'bolt')]);j.CreateBody1Rel().SetTargets([carrier.GetPath()]);j.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,-q[:3]))));j.CreateLocalRot1Attr(Gf.Quatf(1.))
        coupling=PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(self.stage,base+'ThreadPitch')
        axis_frame=Gf.Quatf(float(math.sqrt(.5)),0.,-float(math.sqrt(.5)),0.)
        body_q=Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3])))
        # An enabled, unconstrained D6 supplies the world-relative twist angle.
        # It shares only the pinion actor with the meta joint. Referencing the
        # carrier/bolt hinge directly makes SDK positional-error signs ambiguous.
        reference=UsdPhysics.Joint.Define(self.stage,base+'ThreadAngleReference')
        reference.CreateBody1Rel().SetTargets([Sdf.Path(base+'bolt')]);reference.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));reference.CreateLocalPos1Attr(Gf.Vec3f(0));reference.CreateLocalRot0Attr(axis_frame);reference.CreateLocalRot1Attr(body_q.GetConjugate()*axis_frame);reference.CreateExcludeFromArticulationAttr(True)
        coupling.CreateBody0Rel().SetTargets([Sdf.Path(base+'bolt')]);coupling.CreateBody1Rel().SetTargets([carrier.GetPath()])
        coupling.CreateHingeRel().SetTargets([reference.GetPath()]);coupling.CreatePrismaticRel().SetTargets([linear.GetPath()]);coupling.CreateRatioAttr(self.pitch_sign*360./PITCH);coupling.CreateExcludeFromArticulationAttr(True)
        # A finite viscous angular bearing, capped at 0.015 N m. Angular
        # USD drive gains are per degree; the physical gain is 1 N m s/rad.
        drag=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');drag.CreateTypeAttr('force');drag.CreateStiffnessAttr(0.);drag.CreateDampingAttr(1.*math.pi/180);drag.CreateTargetVelocityAttr(0.);drag.CreateMaxForceAttr(.015)
        self.thread_joint=j;self.thread_linear=linear;self.thread_coupling=coupling;self.screw.capture(float(p[2]),theta);self.capture_pending=True
    def step(self):
        if not self.enabled:return
        h,hr,hq,ha,hw,hv=self.pose('handle');d,dr,dq,da,dw,dv=self.pose('drive');b,br,bq,ba,bw,bv=self.pose('bolt')
        # Relative drive-minus-handle rotation: positive extension is a clockwise handle drive.
        _,drag=self.clutch.step(hr,dr,hq,dq,self.dt)
        torque=float(self.telemetry.get('socket_torque_nm',0.))
        from pxr import Gf
        axis=hr[:,2]
        for name,sign in [('pawl',-1.),('handle',1.)]:
            if abs(drag)>1.e-15:self.sim.apply_torque(self.stageid,self.ids[name],Gf.Vec3f(*map(float,axis*(sign*drag))))
        tip=b-br[:,2]*BOLT_LENGTH
        radial=float(np.linalg.norm(tip[:2]-AXIS[:2]));tilt=float(math.acos(np.clip(br[2,2],-1,1)))
        depth=THREAD_ENTRY_Z-tip[2]
        inserted=radial<=CAPTURE_OFFSET and tilt<=CAPTURE_TILT and .001<=depth<=.003
        if not self.screw.captured:
            if inserted:
                if self.candidate is None:self.candidate=ba
                self.candidate=max(self.candidate,ba)
                if ba-self.candidate<=-THREAD_START:self.capture(b,bq,ba)
            else:self.candidate=None
        force=resistance=preload=0.
        gap=float(b[2]-AXIS[2])
        if self.screw.captured:
            if not self.seated_stop and gap<=.0005:self.install_seat_stop(ba,bq)
            if self.seated_stop and gap>.0006:
                self.stage.RemovePrim('/World/envs/env_0/SeatStop');self.seated_stop=False
            if tip[2]>THREAD_ENTRY_Z+.0001 and ba>self.screw.angle0+.15:self.release_thread()
            # Pitch and axial reactions are solved by PhysX. Contact between the
            # head and bracket supplies seating resistance; no progress animation.
            preload=0.
        bottom=d+dr@np.array([0.,0.,-.028])
        # Geometric contact eligibility supplements, never replaces, physical hex contact.
        mismatch=math.atan2(math.sin(6*(da-ba)),math.cos(6*(da-ba)))/6
        # The straight hex bore spans 1.5--9 mm above its mouth. A socket
        # that fully swallows the head remains engaged below the head's base.
        hex_overlap=min(bottom[2]+.009,b[2]+HEAD_HEIGHT)-max(bottom[2]+.0015,b[2])
        engagement=float(np.linalg.norm(bottom[:2]-b[:2]))<.0005 and hex_overlap>=.001 and abs(mismatch)<.07 and dr[2,2]>.996
        if engagement:
            if self.drive_start is None:self.drive_start=ha;self.bolt_cycle_start=ba
            if self.return_start is None and ha>self.drive_start:self.drive_start=ha;self.bolt_cycle_start=ba
            if ha<self.drive_start-.1 and ba<self.bolt_cycle_start-.02:
                self.return_start=ha if self.return_start is None else min(self.return_start,ha)
            if self.return_start is not None and ha>self.return_start+.1:
                self.cycles+=1;self.return_start=None;self.drive_start=ha;self.bolt_cycle_start=ba
        else:self.drive_start=None;self.return_start=None;self.bolt_cycle_start=None
        carrier_pose=None;pitch_error=0.
        if self.screw.captured:
            self.carrier_wait+=1
            try:
                cp,cr,cq,ca,cw,cv=self.pose('ThreadCarrier');carrier_pose=dict(position=cp.tolist(),yaw=ca,rotation=cr.tolist())
                if self.capture_pending:self.entry_contact(False);self.capture_pending=False
            except RuntimeError:
                if self.carrier_wait>5:raise
            pitch_error=float(b[2]-self.screw.expected_z(ba))
        bypass=not self.screw.captured and radial<.002 and tilt<.15 and depth>.0035
        valid=np.all(np.isfinite(np.r_[h,d,b,ha,da,ba,torque,drag,self.clutch.gate_relative])) and not bypass
        self.telemetry=dict(captured=self.screw.captured,capture_eligible=bool(inserted),entry_depth_m=float(depth),capture_rotation_rad=0. if self.candidate is None else self.candidate-ba,turns=0. if not self.screw.captured else -(ba-self.screw.angle0)/(2*math.pi),gap=gap,output_torque_nm=torque,preload=preload,interface_torque=max(0.,torque) if engagement else 0.,socket_engaged=bool(engagement),hex_overlap_m=float(hex_overlap),ratchet_cycles=self.cycles,valid=bool(valid),angles=dict(handle=ha,drive=da,bolt=ba),positions=dict(handle=h.tolist(),drive=d.tolist(),bolt=b.tolist()),thread_force_applied_by_python=force,seated_stop=self.seated_stop,radial_error=radial,tilt=tilt,carrier=carrier_pose,pitch_error_m=pitch_error)
        return self.telemetry
