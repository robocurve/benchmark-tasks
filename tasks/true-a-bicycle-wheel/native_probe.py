"""Bounded native PhysX ring/spoke probe; NOT a robot/task run.

Uses implicit D6 distance-limit springs for tension-only spokes and implicit D6
rim-joint drives. The isolated hub is fixed and gravity is disabled. One rest
length is changed by a diagnostic quarter turn after settling; no wrench is
claimed. At25µs the isolated ring remains bounded for two seconds. This is not the
full wheel assembly; complete=True alone is not a physical pass. See VALIDATION.json.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import signal
import time
import traceback
import numpy as np
from mechanics import WheelSpec
from thread_coupling import rotation


class ProbeInterrupted(RuntimeError):
    pass


def build(stage, turns=None, *, root="/Wheel", configure_physics=True, spoke_damping=100., rim_angular_damping=.3):
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, Vt
    from rim_geometry import sector_mesh
    s = WheelSpec()
    n = s.spokes
    turns = np.zeros(n) if turns is None else np.asarray(turns, dtype=float)
    if turns.shape != (n,) or not np.isfinite(turns).all():
        raise ValueError("32 finite initial turn offsets required")
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    if configure_physics:
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityMagnitudeAttr(0.)
        sx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        sx.CreateSolverTypeAttr("TGS")
        sx.CreateEnableGPUDynamicsAttr(False)
        sx.CreateBroadphaseTypeAttr("MBP")
    paths, rotations, centers = [], [], []
    theta = np.arange(n)*2*math.pi/n
    vertices = np.column_stack((s.rim_radius_m*np.cos(theta), s.rim_radius_m*np.sin(theta), np.zeros(n)))
    chord = 2*s.rim_radius_m*math.sin(math.pi/n)
    for i in range(n):
        j = (i+1)%n
        center = (vertices[i]+vertices[j])/2
        tangent = (vertices[j]-vertices[i])/chord
        yaw = math.atan2(tangent[1], tangent[0])
        R = np.array([[math.cos(yaw),-math.sin(yaw),0.],[math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
        path = f"{root}/rim_{i:02d}"
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
        body.AddRotateZOp().Set(math.degrees(yaw))
        p = body.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.42/n)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
        rb.CreateSolverPositionIterationCountAttr(128)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(.05)
        rb.CreateAngularDampingAttr(.05)
        rb.CreateSleepThresholdAttr(0.)
        points, faces = sector_mesh(radius=s.rim_radius_m, sectors=n, index=i)
        box = UsdGeom.Mesh.Define(stage, path+"/shape")
        box.CreatePointsAttr(Vt.Vec3fArray(points.tolist()))
        box.CreateFaceVertexCountsAttr([3]*len(faces))
        box.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
        box.CreateSubdivisionSchemeAttr("none")
        box.CreateDisplayColorAttr([Gf.Vec3f(.45,.47,.5)])
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(box.GetPrim()).CreateApproximationAttr("convexHull")
        PhysxSchema.PhysxCollisionAPI.Apply(box.GetPrim()).CreateContactOffsetAttr(.00001)
        UsdPhysics.FilteredPairsAPI.Apply(p).CreateFilteredPairsRel().SetTargets([
            Sdf.Path(f"{root}/rim_{(i-1)%n:02d}"), Sdf.Path(f"{root}/rim_{j:02d}")])
        paths.append(path)
        rotations.append(R)
        centers.append(center)
    for i in range(n):
        j = (i+1)%n
        joint = UsdPhysics.Joint.Define(stage, f"{root}/rim_joint_{i:02d}")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(paths[i])])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(paths[j])])
        joint.CreateLocalPos0Attr(Gf.Vec3f(chord/2,0.,0.))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-chord/2,0.,0.))
        # Half-angle frames coincide at the shared vertex in the reference ring.
        a = math.pi/n
        joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(a/2),0.,0.,math.sin(a/2)))
        joint.CreateLocalRot1Attr(Gf.Quatf(math.cos(a/2),0.,0.,-math.sin(a/2)))
        for axis in ("transY","transZ"):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr(1.)
            limit.CreateHighAttr(-1.)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "transX")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(s.rim_modulus_pa*s.rim_area_m2/chord)
        drive.CreateDampingAttr(10.)
        drive.CreateTargetPositionAttr(0.)
        for axis in ("rotX","rotY","rotZ"):
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(s.rim_modulus_pa*s.rim_second_moment_m4/chord*math.pi/180.)
            drive.CreateDampingAttr(rim_angular_damping*math.pi/180.)
            drive.CreateTargetPositionAttr(0.)
    spokes, nominal_rest = [], []
    for i in range(n):
        side = 1 if i%2 == 0 else -1
        a = theta[i]+side*s.cross_angle_rad
        anchor = np.array([s.hub_flange_radius_m*math.cos(a),s.hub_flange_radius_m*math.sin(a),side*s.hub_half_width_m])
        L = float(np.linalg.norm(vertices[i]-anchor))
        k = s.spoke_modulus_pa*math.pi*(s.spoke_diameter_m/2)**2/L
        rest = L-s.baseline_tension_n/k-turns[i]*s.nipple_pitch_m_per_turn
        joint = UsdPhysics.Joint.Define(stage, f"{root}/spoke_{i:02d}")
        # Missing body0 means fixed world anchor, explicitly an isolated-hub probe.
        joint.CreateBody1Rel().SetTargets([Sdf.Path(paths[i])])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,anchor)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-chord/2,0.,0.))
        # A D6 soft distance limit has no macroscopic DistanceJoint tolerance.
        # All linear/rotational axes remain free; only excessive length pulls.
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "distance")
        limit.CreateHighAttr(rest)
        spring = PhysxSchema.PhysxLimitAPI.Apply(joint.GetPrim(), "distance")
        spring.CreateStiffnessAttr(k)
        spring.CreateDampingAttr(spoke_damping)
        spokes.append(joint)
        nominal_rest.append(rest)
    return paths, spokes, nominal_rest


def constraint_diagnostics(positions,quaternions,rest_lengths):
    """Measured native-frame strain/constraint residuals, not policy inputs."""
    s=WheelSpec();n=s.spokes;chord=2*s.rim_radius_m*math.sin(math.pi/n)
    matrices=np.asarray([rotation(q) for q in quaternions]);a=math.pi/n
    Z=np.array(((math.cos(a),-math.sin(a),0.),(math.sin(a),math.cos(a),0.),(0.,0.,1.)))
    locked=[];axial=[];angles=[];spoke_lengths=[];spoke_tensions=[]
    for i in range(n):
        j=(i+1)%n
        f0=matrices[i]@Z;f1=matrices[j]@Z.T
        p0=positions[i]+matrices[i,:,0]*chord/2
        p1=positions[j]-matrices[j,:,0]*chord/2
        gap=f0.T@(p1-p0);axial.append(gap[0]);locked.append(np.linalg.norm(gap[1:]))
        R=f0.T@f1;angles.append(math.acos(float(np.clip((np.trace(R)-1)/2,-1.,1.))))
        theta=i*2*math.pi/n;side=1 if i%2==0 else -1
        anchor=np.array((s.hub_flange_radius_m*math.cos(theta+side*s.cross_angle_rad),s.hub_flange_radius_m*math.sin(theta+side*s.cross_angle_rad),side*s.hub_half_width_m))
        endpoint=positions[i]-matrices[i,:,0]*chord/2
        L=float(np.linalg.norm(endpoint-anchor));spoke_lengths.append(L)
        reference=np.array((s.rim_radius_m*math.cos(theta),s.rim_radius_m*math.sin(theta),0.))
        k=s.spoke_modulus_pa*math.pi*(s.spoke_diameter_m/2)**2/np.linalg.norm(reference-anchor)
        spoke_tensions.append(k*max(L-rest_lengths[i],0.))
    return {'maximum_locked_rim_joint_gap_m':float(max(locked)),
            'maximum_rim_joint_axial_gap_m':float(max(abs(x) for x in axial)),
            'maximum_rim_joint_bend_rad':float(max(angles)),
            'rim_hoop_energy_j':float(.5*s.rim_modulus_pa*s.rim_area_m2/chord*np.sum(np.square(axial))),
            'rim_bend_energy_j':float(.5*s.rim_modulus_pa*s.rim_second_moment_m4/chord*np.sum(np.square(angles))),
            'spoke_lengths_m':spoke_lengths,'elastic_spoke_tensions_n':list(map(float,spoke_tensions))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=.001)
    parser.add_argument("--seconds", type=float, default=8.)
    parser.add_argument('--adjust-at',type=float,default=None,help='Diagnostic adjustment time in seconds; defaults to halfway')
    parser.add_argument("--spoke-damping", type=float, default=100.)
    parser.add_argument("--rim-angular-damping", type=float, default=.3)
    parser.add_argument("--loosen-turns", type=float, default=0., help="Diagnostic initial offset of spoke 0")
    args = parser.parse_args()
    if not 0 < args.dt <= .005 or not .4 <= args.seconds <= 20:
        parser.error("Use 0 < dt <= 0.005 and0.4 <= seconds <=20; short runs only diagnose early mechanics")
    adjust_at=args.seconds/2 if args.adjust_at is None else args.adjust_at
    if not .05<=adjust_at<=args.seconds-.2:parser.error('Allow at least0.05s before and0.2s after adjustment')
    args.out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    app=sim=cache=stage=None
    attached=False;old_handlers={}
    record = {"schema":"native-wheel-probe-v1","complete":False,"calibrated":False,
              "evidence_type":"Isolated native spring-ring probe, not robot or wrench evidence",
              "dt":args.dt,"seconds":args.seconds,"spoke_damping_ns_m":args.spoke_damping,
              'adjustment_at_s':adjust_at,
              "rim_angular_damping_nm_s_rad":args.rim_angular_damping,"samples":[],"phase":"launch","completed_steps":0}
    def save():
        record['wall_seconds']=time.monotonic()-started
        tmp=args.out/'native-probe.json.tmp'
        tmp.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
        tmp.replace(args.out/'native-probe.json')
    def interrupted(signum,frame):
        raise ProbeInterrupted('Received '+signal.Signals(signum).name+'; saved partial measurements are not a physical pass')
    save()
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,enable_cameras=False).app
        for sig in (signal.SIGTERM,signal.SIGINT):
            old_handlers[sig]=signal.getsignal(sig);signal.signal(sig,interrupted)
        from pxr import Usd,UsdGeom,UsdUtils,UsdPhysics
        from omni.physx import get_physx_interface,get_physx_simulation_interface
        sim,physics=get_physx_simulation_interface(),get_physx_interface()
        cache=UsdUtils.StageCache.Get();stage=Usd.Stage.CreateInMemory();cache.Insert(stage)
        record['phase']='build'
        turns = np.zeros(32)
        turns[0] = -args.loosen_turns
        paths, spokes, rest = build(stage, turns, spoke_damping=args.spoke_damping, rim_angular_damping=args.rim_angular_damping)
        stage.GetRootLayer().Export(str(args.out/"initial.usda"))
        record["phase"] = "attach"
        sim.attach_stage(cache.GetId(stage).ToLongInt())
        attached = True
        steps = round(args.seconds/args.dt)
        adjust_step=round(adjust_at/args.dt)
        record["phase"] = "simulate"
        last_checkpoint=time.monotonic()
        for step in range(steps):
            if step == adjust_step:
                # Diagnostic only: replace by measured physical nipple angle in a task.
                UsdPhysics.LimitAPI(spokes[0].GetPrim(), "distance").GetHighAttr().Set(rest[0]-.25*WheelSpec().nipple_pitch_m_per_turn)
            sim.simulate(args.dt, step*args.dt)
            sim.fetch_results()
            record['completed_steps']=step+1
            if step % max(1,round(.05/args.dt)) == 0 or step in (adjust_step-1,steps-1):
                transforms = [physics.get_rigidbody_transformation(path) for path in paths]
                if not all(tr.get("ret_val") for tr in transforms):
                    raise RuntimeError("Live PhysX rim transformations unavailable")
                p = np.asarray([list(tr["position"]) for tr in transforms],dtype=float)
                if not np.isfinite(p).all() or np.max(np.linalg.norm(p,axis=1)) > 1.:
                    raise RuntimeError("Native ring became nonfinite or escaped the probe bounds")
                quats=[list(map(float,tr["rotation"])) for tr in transforms]
                actual_rest=[float(UsdPhysics.LimitAPI(j.GetPrim(),"distance").GetHighAttr().Get()) for j in spokes]
                diag=constraint_diagnostics(p,quats,actual_rest)
                record["samples"].append({"step":step+1,"rim_rotations_xyzw":quats,"constraint_diagnostics":diag,"spoke0_rest_attr_m":float(UsdPhysics.LimitAPI(spokes[0].GetPrim(), "distance").GetHighAttr().Get()),"rim_centres_m":p.tolist(),"lateral_peak_to_peak_m":float(np.ptp(p[:,2]))})
                if time.monotonic()-last_checkpoint>=5.:
                    save();last_checkpoint=time.monotonic()
        before = [r for r in record["samples"] if r["step"] <= adjust_step][-1]
        after = record["samples"][-1]
        change = np.asarray(after["rim_centres_m"])-np.asarray(before["rim_centres_m"])
        record["maximum_post_adjustment_displacement_m"] = float(np.max(np.linalg.norm(change,axis=1)))
        if record["maximum_post_adjustment_displacement_m"] < 1e-6:
            raise RuntimeError("Quarter-turn rest-length change produced no measured response; inspect native springs and USD readback")
        # Execution alone is not a physical pass: a sub-millimetre rest change
        # must not create centimetre-scale drift or an unsettled crooked ring.
        recent=[s for s in record["samples"] if s["step"]>=steps-round(.2/args.dt)]
        drift=max(np.max(np.linalg.norm(np.asarray(s["rim_centres_m"])-np.asarray(after["rim_centres_m"]),axis=1)) for s in recent)
        record["final_0p2s_maximum_drift_m"]=float(drift)
        record["bounded_small_response"]=bool(record["maximum_post_adjustment_displacement_m"]<.002 and drift<.0001 and after["lateral_peak_to_peak_m"]<.002)
        record["complete"] = True
        record["phase"] = "complete"
        stage.GetRootLayer().Export(str(args.out/"final-authored-parameters.usda"))
    except BaseException as error:
        record['interrupted']=isinstance(error,ProbeInterrupted)
        record["error"] = f"{type(error).__name__}: {error}"
        record["traceback"] = traceback.format_exc()
        raise
    finally:
        save()
        if attached: sim.detach_stage()
        if cache is not None and stage is not None:cache.Erase(stage)
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
        if app:app.close()


if __name__ == "__main__":
    main()
