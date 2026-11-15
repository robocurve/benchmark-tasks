"""Author the proposed wheel/stand/nipple/wrench scene in an existing USD stage.

Short native settling is verified; wrench contact and YAM manipulation remain
unrun. This module is not a demo, score or submission certificate. The caller must install
NativeThreadCoupler.before_step before every physics substep, preserve its
failure conditions, and reset all bodies/joints to their authored states.
"""
from __future__ import annotations
import math
import numpy as np
from mechanics import WheelSpec, starting_turn_errors
from native_probe import build as build_ring
from thread_coupling import ThreadSpec

# Six exposed millimetres of flats and a six-millimetre barrel into the rim.
NIPPLE_CENTER_INSET_M = .016
NIPPLE_FLAT_LENGTH_M = .006
WRENCH_ORIGIN_M = (.20,-.005,.7555)
WRENCH_HANDLE_CENTER_M = (0.,-.2075,0.)
WRENCH_HANDLE_SIZE_M = (.016,.055,.016)
WRENCH_NECK_CENTER_M = (0.,-.0925,0.)
WRENCH_NECK_SIZE_M = (.010,.175,.004)
WRENCH_TCP_M = (0.,-.195,.011)
WRENCH_MASS_KG = .090


def quat(R):
    R=np.asarray(R,dtype=float)
    tr=np.trace(R)
    if tr>0:
        s=2*math.sqrt(tr+1.)
        return np.array([(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s,.25*s])
    i=int(np.argmax(np.diag(R)))
    j,k=(i+1)%3,(i+2)%3
    s=2*math.sqrt(1+R[i,i]-R[j,j]-R[k,k])
    out=np.zeros(4)
    out[i]=.25*s
    out[j]=(R[j,i]+R[i,j])/s
    out[k]=(R[k,i]+R[i,k])/s
    out[3]=(R[k,j]-R[j,k])/s
    return out


def x_frame(direction):
    # Callers reuse the metric direction to place the two half-spokes.
    x=np.array(direction,dtype=float,copy=True)
    x/=np.linalg.norm(x)
    seed=np.array([0.,0.,1.]) if abs(x[2])<.9 else np.array([0.,1.,0.])
    y=np.cross(seed,x)
    y/=np.linalg.norm(y)
    return np.column_stack((x,y,np.cross(x,y)))


def spoke_phase(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array(((1.,0.,0.),(0.,c,-s),(0.,s,c)))


def wheel_world_frame(clocking_rad=0.):
    """Clock the whole wheel about its axle without changing internal geometry."""
    if not np.isfinite(clocking_rad):raise ValueError("Nonfinite wheel clocking")
    return spoke_phase(clocking_rad)@np.array(((0.,0.,1.),(0.,1.,0.),(-1.,0.,0.)))


def stand_boxes(fixture_layout='reference'):
    """Single-sided bench stand, clamped to the axle's far end.

    The former front upright intersected the standard open READY hand. This
    rigid cantilever fixture leaves the ordinary robot reset and rim access
    clear; it does not lock the rotating hub or remove external contacts.
    """
    from fixture_layout import stand_parts
    return stand_parts(fixture_layout)


def wrench_jaw_mesh(side):
    """A real flared mouth guides the3.23mm nipple into a3.25mm working slot."""
    if side not in (-1,1):raise ValueError('Jaw side must be -1 or1')
    # Centred nipple spans y=±1.615mm. Keep that entire square in the straight
    # working slot; beginning the flare at-2mm caused ~20° of geometric play.
    outline=np.array(((.001625,-.004),(.003625,-.004),(.003625,.008),
                      (.0025,.008),(.001625,.002)))
    outline[:,0]*=side
    if side<0:outline=outline[::-1]
    points=np.array([(x,y,z) for z in (-.002,.002) for x,y in outline])
    n=len(outline);faces=[]
    for i in range(1,n-1):faces.extend(((0,i+1,i),(n,n+i,n+i+1)))
    for i in range(n):
        j=(i+1)%n;faces.extend(((i,j,n+j),(i,n+j,n+i)))
    return points,np.array(faces,int)


def build_scene(stage, root="/WheelTask", *, reset_profile="two-defect-candidate", nipple_flat_phase_rad=0., wheel_clocking_rad=0., central_spring_candidate=False, distance_spring_tolerance_m=0., inextensible_rim_candidate=False, fixture_layout='reference'):
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, Vt
    from fixture_layout import axle_position, pointer_position
    axle=tuple(map(float,axle_position(fixture_layout)))
    spec,thread=WheelSpec(),ThreadSpec()
    if distance_spring_tolerance_m not in (0.,.025) or (distance_spring_tolerance_m and not central_spring_candidate):
        raise ValueError('Distance tolerance compensation is only a native central-spring diagnostic')
    flat_phase=spoke_phase(nipple_flat_phase_rad)
    wheel=root+"/wheel"
    n=spec.spokes
    UsdGeom.Xform.Define(stage,root)
    initial=starting_turn_errors(spec, profile=reset_profile)
    rim_paths,old_spokes,_=build_ring(stage,initial,root=wheel,configure_physics=False)
    if inextensible_rim_candidate:
        # Explicit inextensible-hoop approximation: native D6 axial spring
        # forces across non-collinear endpoint gaps produced a net moment in
        # the diagnostic. Keep every bending/torsion spring and all collisions.
        # The omitted measured hoop contraction was approximately67 micrometres.
        for i in range(n):
            prim=stage.GetPrimAtPath(f'{wheel}/rim_joint_{i:02d}')
            drive=UsdPhysics.DriveAPI.Apply(prim,'transX')
            drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(0.)
            limit=UsdPhysics.LimitAPI.Apply(prim,'transX')
            limit.CreateLowAttr(1.);limit.CreateHighAttr(-1.)
    wheel_xf=UsdGeom.Xform.Define(stage,wheel)
    wheel_xf.AddTranslateOp().Set(Gf.Vec3d(*axle))
    wheel_xf.AddRotateYOp().Set(90.)
    wheel_xf.AddRotateZOp().Set(math.degrees(wheel_clocking_rad))

    def qgf(R):
        x,y,z,w=map(float,quat(R))
        return Gf.Quatf(w,x,y,z)

    def shape(parent,name,center,size,kind="box",color=(.45,.47,.5)):
        path=parent+"/"+name
        scale=None
        if kind in ("cylinder","cylinder_x"):
            g=UsdGeom.Cylinder.Define(stage,path)
            g.CreateAxisAttr("X" if kind=="cylinder_x" else "Z")
            g.CreateRadiusAttr(size[0])
            g.CreateHeightAttr(size[1])
        elif kind=="capsule":
            g=UsdGeom.Capsule.Define(stage,path)
            g.CreateAxisAttr("X")
            g.CreateRadiusAttr(size[1])
            g.CreateHeightAttr(size[0])
        else:
            g=UsdGeom.Cube.Define(stage,path)
            g.CreateSizeAttr(1.)
            scale=g.AddScaleOp()
            scale.Set(Gf.Vec3f(*map(float,size)))
        translation=g.AddTranslateOp()
        translation.Set(Gf.Vec3d(*map(float,center)))
        if scale is not None:
            g.SetXformOpOrder([translation,scale])
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        c=PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
        c.CreateContactOffsetAttr(.000002)
        c.CreateRestOffsetAttr(0.)

    def body(path,pos,R,mass=None):
        g=UsdGeom.Xform.Define(stage,path)
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)))
        g.AddOrientOp().Set(qgf(R))
        if mass is not None:
            p=g.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(p)
            UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
            rb=PhysxSchema.PhysxRigidBodyAPI.Apply(p)
            rb.CreateSolverPositionIterationCountAttr(128)
            rb.CreateSolverVelocityIterationCountAttr(4)
            rb.CreateSleepThresholdAttr(0.)
            rb.CreateEnableCCDAttr(True)
        return g

    I=np.eye(3)
    stand=root+"/stand"
    body(stand,(0.,0.,0.),I)
    for name,center,size in stand_boxes(fixture_layout):
        shape(stand,name,center,size,color=(.18,.2,.22))
    shape(stand,"pointer",pointer_position(fixture_layout),(.020,.002,.002),color=(.85,.25,.12))

    hub=wheel+"/hub"
    body(hub,(0.,0.,0.),I,.18)
    # Single-sided stand: 10mm of axle beyond the front flange is enough.
    # The far end remains at world x=.415m, inside the cantilever clamp.
    shape(hub,"axle",(0.,0.,.015),(.010,.010,.100))
    shape(hub,"barrel",(0.,0.,0.),(.035,.035,.050))
    for side in (-1,1):
        shape(hub,f"flange_{'left' if side<0 else 'right'}",(0.,0.,side*.025),(.025,.003,0.),kind="cylinder")
    bearing=UsdPhysics.RevoluteJoint.Define(stage,wheel+"/hub_bearing")
    bearing.CreateAxisAttr("Z")
    bearing.CreateBody1Rel().SetTargets([Sdf.Path(hub)])
    bearing.CreateLocalPos0Attr(Gf.Vec3f(*axle))
    bearing.CreateLocalRot0Attr(qgf(wheel_world_frame(wheel_clocking_rad)))
    bearing.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,0.))
    visc=UsdPhysics.DriveAPI.Apply(bearing.GetPrim(),"angular")
    visc.CreateTypeAttr("force")
    visc.CreateStiffnessAttr(0.)
    visc.CreateDampingAttr(.001*math.pi/180.)
    visc.CreateTargetVelocityAttr(0.)
    # Axle/stand overlap is an authored bearing, not a collision support.
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(hub)).CreateFilteredPairsRel().SetTargets([Sdf.Path(stand)])

    theta=np.arange(n)*2*math.pi/n
    vertices=np.column_stack((spec.rim_radius_m*np.cos(theta),spec.rim_radius_m*np.sin(theta),np.zeros(n)))
    chord=2*spec.rim_radius_m*math.sin(math.pi/n)
    metadata={"hub_path":hub,"spokes":[],"wrench_path":root+"/wrench",
              "reset_profile":reset_profile,"initial_turn_offsets":initial.tolist(),
              "fixture_layout":fixture_layout,"axle_position_m":list(axle),
              "nipple_flat_phase_rad":float(nipple_flat_phase_rad),
              "wheel_clocking_rad":float(wheel_clocking_rad),
              "central_spring_candidate":bool(central_spring_candidate),
              "inextensible_rim_candidate":bool(inextensible_rim_candidate)}
    for i in range(n):
        old_spokes[i].CreateJointEnabledAttr(False)
        side=1 if i%2==0 else -1
        a=theta[i]+side*spec.cross_angle_rad
        anchor=np.array([spec.hub_flange_radius_m*math.cos(a),spec.hub_flange_radius_m*math.sin(a),side*spec.hub_half_width_m])
        endpoint=vertices[i]
        vec=endpoint-anchor
        L=float(np.linalg.norm(vec))
        R=x_frame(vec)
        k=spec.spoke_modulus_pa*math.pi*(spec.spoke_diameter_m/2)**2/L
        nominal=L-spec.baseline_tension_n/k
        rim_center=(vertices[i]+vertices[(i+1)%n])/2
        rim_R=x_frame(vertices[(i+1)%n]-vertices[i])
        # x_frame on a planar tangent produces the same z-up frame as the ring.
        inner=f"{wheel}/spoke_{i:02d}_inner"
        outer=f"{wheel}/spoke_{i:02d}_outer"
        mass=7850*math.pi*(spec.spoke_diameter_m/2)**2*L/2
        for path,fraction in ((inner,.25),(outer,.75)):
            body(path,anchor+vec*fraction,R,mass)
            shape(path,"steel",(0.,0.,0.),(L/2,.001,0.),kind="capsule")
            # The eyelet lies on the shared boundary of two rim sectors.
            # Both sectors overlap this jointed internal interface; filtering
            # only i would make i-1 eject the rod against its own rim joint.
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path)).CreateFilteredPairsRel().SetTargets([
                Sdf.Path(inner if path==outer else outer),Sdf.Path(hub),
                Sdf.Path(rim_paths[i]),Sdf.Path(rim_paths[(i-1)%n])])
        for label,path,other,p0,p1 in (
                ("hub",inner,hub,anchor,(-L/4,0.,0.)),
                ("rim",outer,rim_paths[i],(-chord/2,0.,0.),(L/4,0.,0.))):
            # A retained spoke head can tilt but cannot freely spin about the
            # shaft. This transmits the thread reaction through actual bodies.
            joint=(UsdPhysics.Joint if label=='hub' else UsdPhysics.SphericalJoint).Define(stage,f"{wheel}/spoke_{i:02d}_{label}_joint")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(other)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p0)))
            joint.CreateLocalPos1Attr(Gf.Vec3f(*map(float,p1)))
            if label=='hub':
                joint.CreateLocalRot0Attr(qgf(R));joint.CreateLocalRot1Attr(qgf(I))
                for axis in ('transX','transY','transZ','rotX'):
                    locked=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis)
                    locked.CreateLowAttr(1.);locked.CreateHighAttr(-1.)
        axial=UsdPhysics.Joint.Define(stage,f"{wheel}/spoke_{i:02d}_axial")
        axial.CreateBody0Rel().SetTargets([Sdf.Path(inner)])
        axial.CreateBody1Rel().SetTargets([Sdf.Path(outer)])
        axial.CreateLocalPos0Attr(Gf.Vec3f(L/4,0.,0.))
        axial.CreateLocalPos1Attr(Gf.Vec3f(-L/4,0.,0.))
        for axis in ("transY","transZ","rotX"):
            limit=UsdPhysics.LimitAPI.Apply(axial.GetPrim(),axis)
            limit.CreateLowAttr(1.)
            limit.CreateHighAttr(-1.)
        # Midspan bending is compliant: a perfectly rigid two-half rod would
        # make a physical spoke-tension gauge impossible to reproduce. For
        # pin-supported half rods K=3EI/L gives the standard 48EI/L^3 centre
        # load stiffness; axial pretension adds geometric stiffness naturally.
        bend_stiffness=3*spec.spoke_modulus_pa*(math.pi*spec.spoke_diameter_m**4/64)/L
        for axis in ("rotY","rotZ"):
            bend=UsdPhysics.DriveAPI.Apply(axial.GetPrim(),axis)
            bend.CreateTypeAttr("force");bend.CreateStiffnessAttr(bend_stiffness*math.pi/180.)
            bend.CreateDampingAttr(.002*math.pi/180.);bend.CreateTargetPositionAttr(0.)
        if central_spring_candidate:
            # Central tension acts along the line joining the two half-rod
            # centres. This uses a positive separation, unlike the reference
            # D6 soft limit at a negative midspan offset. The separate D6 still
            # supplies the rod's transverse, bending and torsional continuity.
            spring=UsdPhysics.DistanceJoint.Define(stage,f"{wheel}/spoke_{i:02d}_central_spring")
            spring.CreateBody0Rel().SetTargets([Sdf.Path(inner)])
            spring.CreateBody1Rel().SetTargets([Sdf.Path(outer)])
            spring.CreateMinDistanceAttr(-1.)
            rest_attr=spring.CreateMaxDistanceAttr(nominal-initial[i]*thread.pitch_m_per_turn-L/2-distance_spring_tolerance_m)
            soft=PhysxSchema.PhysxPhysicsDistanceJointAPI.Apply(spring.GetPrim())
            soft.CreateSpringEnabledAttr(True)
            soft.CreateSpringStiffnessAttr(k)
            soft.CreateSpringDampingAttr(1.)
            rest_origin=L/2
        else:
            limit=UsdPhysics.LimitAPI.Apply(axial.GetPrim(),"transX")
            limit.CreateLowAttr(-1.)
            rest_attr=limit.CreateHighAttr(nominal-initial[i]*thread.pitch_m_per_turn-L)
            soft=PhysxSchema.PhysxLimitAPI.Apply(axial.GetPrim(),"transX")
            soft.CreateStiffnessAttr(k)
            soft.CreateDampingAttr(1.)
            rest_origin=L

        nipple=f"{wheel}/nipple_{i:02d}"
        nipple_center=endpoint-R[:,0]*NIPPLE_CENTER_INSET_M
        nipple_R=R@flat_phase
        body(nipple,nipple_center,nipple_R,.0009)
        shape(nipple,"flats",(0.,0.,0.),(NIPPLE_FLAT_LENGTH_M,.00323,.00323),color=(.7,.52,.17))
        shape(nipple,"barrel",(.006,0.,0.),(.0015,.006,0.),kind="cylinder_x",color=(.7,.52,.17))
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(nipple)).CreateFilteredPairsRel().SetTargets([
            Sdf.Path(rim_paths[i]),Sdf.Path(rim_paths[(i-1)%n]),Sdf.Path(outer)])
        hinge=UsdPhysics.RevoluteJoint.Define(stage,f"{wheel}/nipple_{i:02d}_bearing")
        hinge.CreateAxisAttr("X")
        hinge.CreateBody0Rel().SetTargets([Sdf.Path(rim_paths[i])])
        hinge.CreateBody1Rel().SetTargets([Sdf.Path(nipple)])
        hinge.CreateLocalPos0Attr(Gf.Vec3f(*map(float,rim_R.T@(nipple_center-rim_center))))
        hinge.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,0.))
        hinge.CreateLocalRot0Attr(qgf(rim_R.T@nipple_R))
        # Thread friction acts between nipple and outer spoke, not between
        # nipple and rim. The separate rim bearing locates the nipple but does
        # not prescribe its threaded angle or transfer a fictitious torque.
        thread_joint=UsdPhysics.Joint.Define(stage,f'{wheel}/nipple_{i:02d}_thread_friction')
        thread_joint.CreateBody0Rel().SetTargets([Sdf.Path(outer)])
        thread_joint.CreateBody1Rel().SetTargets([Sdf.Path(nipple)])
        thread_joint.CreateLocalRot0Attr(qgf(flat_phase))
        thread_joint.CreateLocalRot1Attr(qgf(I))
        friction=UsdPhysics.DriveAPI.Apply(thread_joint.GetPrim(),"rotX")
        friction.CreateTypeAttr("force")
        friction.CreateStiffnessAttr(thread.friction_stiffness_nm_per_rad*math.pi/180.)
        friction.CreateDampingAttr(thread.viscous_friction_nm_s_per_rad*math.pi/180.)
        friction.CreateTargetPositionAttr(0.)
        friction.CreateTargetVelocityAttr(0.)
        friction.CreateMaxForceAttr(thread.friction_coefficient*spec.baseline_tension_n*thread.thread_mean_radius_m)
        metadata["spokes"].append({"nipple_path":nipple,"rim_path":rim_paths[i],
            "reference_nipple_in_spoke_rotation":flat_phase.tolist(),
            "initial_turn_offset":float(initial[i]),"hub_anchor_local_m":anchor.tolist(),
            "rim_anchor_local_m":[-chord/2,0.,0.],"nominal_rest_m":nominal,
            "stiffness_n_per_m":k,"rest_target_attribute":rest_attr,
            "inner_path":inner,"outer_path":outer,"half_rod_length_m":L/2,
            "rest_target_origin_m":rest_origin,"native_rest_bias_m":distance_spring_tolerance_m,
            "central_spring":bool(central_spring_candidate),"friction_drive":friction})

    wrench=metadata["wrench_path"]
    # A manufacturable extended spoke key keeps the bulky wrist in front of
    # the wheel. The validated jaw is unchanged; the neck remains collidable.
    body(wrench,WRENCH_ORIGIN_M,I,WRENCH_MASS_KG)
    shape(wrench,"handle",WRENCH_HANDLE_CENTER_M,WRENCH_HANDLE_SIZE_M,color=(.13,.28,.4))
    shape(wrench,"steel_neck",WRENCH_NECK_CENTER_M,WRENCH_NECK_SIZE_M,color=(.45,.47,.5))
    for side in (-1,1):
        points,faces=wrench_jaw_mesh(side)
        jaw=UsdGeom.Mesh.Define(stage,wrench+f"/jaw_{'left' if side<0 else 'right'}")
        jaw.CreatePointsAttr(Vt.Vec3fArray(points.tolist()))
        jaw.CreateFaceVertexCountsAttr([3]*len(faces));jaw.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
        jaw.CreateSubdivisionSchemeAttr('none');jaw.CreateDisplayColorAttr([Gf.Vec3f(.45,.47,.5)])
        UsdPhysics.CollisionAPI.Apply(jaw.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(jaw.GetPrim()).CreateApproximationAttr('convexHull')
        contact=PhysxSchema.PhysxCollisionAPI.Apply(jaw.GetPrim())
        contact.CreateContactOffsetAttr(.000002);contact.CreateRestOffsetAttr(0.)
    shape(wrench,"jaw_back",(0.,-.005,0.),(.00725,.002,.004))
    return metadata
