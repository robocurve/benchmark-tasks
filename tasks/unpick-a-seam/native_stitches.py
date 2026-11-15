"""Native flexible cloth joined only by contact with closed thread chains.

There are no cloth attachments, welds, proximity latches or vertex targets.
Breaking a thread joint leaves all fragments and their cloth contacts present.
Whether 0.4 mm thread/hole contact is stable is the purpose of the native probe.
"""
import math
import numpy as np
from geometry import patch,stitch_loop,STITCH_Y
from native_cloth import author_cloth,seam_fabric


def x_quat(direction):
    d=np.asarray(direction,float);d/=np.linalg.norm(d)
    if d[0]<-1+1e-10:return np.array((0.,0.,1.,0.))
    q=np.array((0.,-d[2],d[1],1+d[0]));return q/np.linalg.norm(q)


def author_thread(stage,path,nodes,*,radius=.0002,density=1400.,open_diagnostic=False,joint_kind='spherical',bending=None):
    from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema,UsdShade
    if joint_kind not in ('spherical','d6'):raise ValueError('Unknown native thread joint kind')
    if bending is not None and joint_kind!='d6':raise ValueError('Passive swing bending requires native D6 joints')
    nodes=np.asarray(nodes,float);d=np.roll(nodes,-1,axis=0)-nodes
    lengths=np.linalg.norm(d,axis=1);centres=(nodes+np.roll(nodes,-1,axis=0))/2
    if np.any(lengths<2*radius):raise ValueError('Thread links too short for their radius')
    if bending is not None:
        from thread_bending import planar_frames
        quaternions=planar_frames(nodes)
        bend_stiffness,bend_damping=bending.gains(lengths,radius,density)
    else:quaternions=np.asarray([x_quat(v) for v in d])
    n=len(nodes);bodies=[];joints=[]
    for i,(p,direction,length) in enumerate(zip(centres,d,lengths)):
        root=UsdGeom.Xform.Define(stage,f'{path}/link_{i:03d}')
        root.AddTranslateOp().Set(Gf.Vec3d(*p))
        q=quaternions[i];root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q[3],*q[:3]))
        prim=root.GetPrim();UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(math.pi*radius**2*length*density))
        rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(128)
        rb.CreateSolverVelocityIterationCountAttr(32)
        rb.CreateEnableCCDAttr(True)
        rb.CreateMaxDepenetrationVelocityAttr(.15)
        rb.CreateLinearDampingAttr(.04);rb.CreateAngularDampingAttr(.02)
        capsule=UsdGeom.Capsule.Define(stage,f'{path}/link_{i:03d}/Shape')
        capsule.CreateAxisAttr('X');capsule.CreateRadiusAttr(radius);capsule.CreateHeightAttr(float(length))
        capsule.CreateDisplayColorAttr([Gf.Vec3f(.8,.08,.03)])
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
        contact=PhysxSchema.PhysxCollisionAPI.Apply(capsule.GetPrim())
        contact.CreateContactOffsetAttr(.00005);contact.CreateRestOffsetAttr(0.)
        mat=UsdShade.Material.Define(stage,f'{path}/ThreadMaterial')
        pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pm.CreateStaticFrictionAttr(.35);pm.CreateDynamicFrictionAttr(.3)
        UsdShade.MaterialBindingAPI.Apply(capsule.GetPrim()).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
        # Only adjoining end-cap overlap is filtered. Every cloth, tool and
        # non-neighbouring thread contact remains active.
        UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
            [Sdf.Path(f'{path}/link_{j:03d}') for j in ((i-1)%n,(i+1)%n)])
        bodies.append(str(root.GetPath()))
    for i in range(n):
        before=(i-1)%n
        joint=(UsdPhysics.SphericalJoint if joint_kind=='spherical' else UsdPhysics.Joint).Define(stage,f'{path}/joint_{i:03d}')
        joint.CreateBody0Rel().SetTargets([Sdf.Path(bodies[before])])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(bodies[i])])
        joint.CreateLocalPos0Attr(Gf.Vec3f(float(lengths[before]/2),0.,0.))
        joint.CreateLocalPos1Attr(Gf.Vec3f(float(-lengths[i]/2),0.,0.))
        if joint_kind=='d6':
            # Equivalent free-angular connector using the translation-lock
            # mechanism of the validated overhand-knot rope. No angular drive.
            joint.CreateLocalRot0Attr(Gf.Quatf(1.))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.))
            for axis in ('transX','transY','transZ'):
                limit=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis)
                limit.CreateLowAttr(1.);limit.CreateHighAttr(-1.)
            if bending is not None:
                # Passive straight-rest bending, with free axial twist and no
                # hard angular limits. The drive never follows task state.
                for axis in ('rotY','rotZ'):
                    drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis)
                    drive.CreateTypeAttr('force')
                    drive.CreateStiffnessAttr(float(bend_stiffness[i]*math.pi/180.))
                    drive.CreateDampingAttr(float(bend_damping[i]*math.pi/180.))
                    drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateJointEnabledAttr(not (open_diagnostic and i==0))
        joints.append(str(joint.GetPath()))
    result={'bodies':bodies,'joints':joints,'nodes':nodes,'lengths':lengths,'radius_m':radius,
            'mass_kg':float(math.pi*radius**2*lengths.sum()*density)}
    if bending is not None:
        result['bending']={'rigidity_nm2':bending.rigidity_nm2,'damping_ratio':bending.damping_ratio,
                          'stiffness_nm_rad':bend_stiffness.tolist(),'damping_nms_rad':bend_damping.tolist(),
                          'straight_rest':True,'axial_twist_free':True,'hard_angular_limits':False}
    import os
    if os.environ.get('ROBOCURVE_SEAM_FUNCTIONAL_TACKS')=='1':
        from functional_tacks import prepare_thread
        prepare_thread(stage,result)
    if os.environ.get('ROBOCURVE_SEAM_CONVEX_CAPSULE_DIAGNOSTIC')=='1':
        from convex_capsule import replace_capsules
        result['convex_capsule_candidate']=replace_capsules(stage,[body+'/Shape' for body in bodies])
    if open_diagnostic:set_joint_enabled(stage,result,0,False)
    return result


def author_pair(stage,*,transform=lambda p:p,open_diagnostic=False,root_path='/World',
                thread_radius=.0002,hole_half=.0006,contact_refinement=None,hole_mesh_spacing=None,quality_mesh=False,
                cloth_contact_offset_m=.00025):
    cloth=[]
    material=seam_fabric(cloth_contact_offset_m)
    for i,left in enumerate((True,False)):
        if quality_mesh:
            if abs(hole_half-.0008)>1e-12 or hole_mesh_spacing is not None:
                raise ValueError("Quality mesh is the fixed 1.6 mm hole diagnostic")
            from quality_mesh import quality_patch
            mesh=quality_patch(left)
        else:
            mesh=patch(left,hole_half=hole_half,hole_mesh_spacing=hole_mesh_spacing)
        mesh.rest=transform(mesh.rest)
        author_cloth(stage,f'{root_path}/Cloth_{i}',mesh,material,
                     color=(.76,.68,.47) if left else (.25,.42,.66),
                     contact_refinement=contact_refinement)
        cloth.append(mesh)
    threads=[author_thread(stage,f'{root_path}/Stitch_{i}',transform(stitch_loop(y)),
                            open_diagnostic=open_diagnostic,radius=thread_radius) for i,y in enumerate(STITCH_Y)]
    return cloth,threads


def set_joint_enabled(stage,thread,index,enabled,*,restore_contact=True):
    """Change one thread connection and its adjoining end-cap pair together.

    Joined capsules share overlapping hemispheres, so that pair is filtered.
    A cut disables the mechanical connection immediately. With deferred contact,
    the original overlapping end-cap exclusion persists only until the finite
    capsules separate. Turning it on while the endcaps coincide injects contact
    correction energy. All cloth/tool/non-neighbour contact remains active.
    """
    from pxr import UsdPhysics
    n=len(thread['bodies'])
    if (type(enabled) is not bool or type(restore_contact) is not bool or
            not isinstance(index,(int,np.integer)) or not 0<=index<n or len(thread['joints'])!=n):
        raise ValueError('Invalid native thread connection')
    filtered=enabled or not restore_contact
    before=(index-1)%n
    paths=[thread['bodies'][before],thread['bodies'][index]]
    prims=[stage.GetPrimAtPath(path) for path in paths]
    for prim,other in zip(prims,reversed(prims)):
        relationship=UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
        targets=list(relationship.GetTargets());other_path=other.GetPath()
        targets=[target for target in targets if target!=other_path]
        if filtered:targets.append(other_path)
        if not relationship.SetTargets(targets):raise RuntimeError('Cannot update cut-end collision filter')
    joint=UsdPhysics.Joint(stage.GetPrimAtPath(thread['joints'][index]))
    if not joint.GetCollisionEnabledAttr().Set(not filtered):
        raise RuntimeError('Cannot update cut-end joint collision flag')
    if not joint.GetJointEnabledAttr().Set(enabled):raise RuntimeError('Cannot change native thread joint')
    pending=thread.setdefault('pending_cut_contacts',set())
    if not enabled and not restore_contact:pending.add(index)
    else:pending.discard(index)


def segment_distance(first,second):
    """Minimum distance of two finite axes, including interior and endpoints."""
    first,second=np.asarray(first,float),np.asarray(second,float)
    if first.shape!=(2,3) or second.shape!=(2,3) or not np.isfinite([first,second]).all():
        raise ValueError('Invalid capsule axes')
    a,b=first;c,d=second;u=b-a;v=d-c;w=a-c
    A=u@u;B=u@v;C=v@v;D=u@w;E=v@w
    if A<=0 or C<=0:raise ValueError('Capsule axes must have positive length')
    determinant=A*C-B*B;values=[]
    if determinant>1e-12*A*C:
        s=(B*E-C*D)/determinant;t=(A*E-B*D)/determinant
        if 0<=s<=1 and 0<=t<=1:values.append(np.linalg.norm(a+s*u-c-t*v))
    for p in (a,b):values.append(np.linalg.norm(p-c-np.clip((p-c)@v/C,0,1)*v))
    for p in (c,d):values.append(np.linalg.norm(p-a-np.clip((p-a)@u/A,0,1)*u))
    return float(min(values))


def restore_separated_cut_contacts(stage,threads,capsule_axes):
    """Re-enable only separated cut pairs; no joint, force or geometry repair.

    Separation includes both 50µm native contact offsets. Checking complete
    finite axes prevents folded links with distant endpoints from being treated
    as separated. A restored pair can never become excluded again during a run.
    """
    if not any(thread.get('pending_cut_contacts') for thread in threads):return []
    axes=np.asarray(capsule_axes,float)
    count=sum(len(thread['bodies']) for thread in threads)
    if axes.shape!=(count,2,3) or not np.isfinite(axes).all():
        raise ValueError('Missing finite capsule geometry for cut contacts')
    restored=[];start=0
    for thread in threads:
        n=len(thread['bodies']);radius=float(thread['radius_m'])
        if not np.isfinite(radius) or radius<=0:raise ValueError('Invalid thread radius')
        for index in sorted(thread.get('pending_cut_contacts',set())):
            if not 0<=index<n:raise ValueError('Unknown deferred cut contact')
            before=(index-1)%n
            if segment_distance(axes[start+before],axes[start+index])>=2*radius+.0001:
                set_joint_enabled(stage,thread,index,False,restore_contact=True)
                restored.append(start+index)
        start+=n
    return restored


def restore_thread_joints(stage,thread):
    """Reset-only restoration; cannot be called by the action policy."""
    import os
    if os.environ.get('ROBOCURVE_SEAM_FUNCTIONAL_TACKS')=='1':
        from functional_tacks import prepare_thread
        prepare_thread(stage,thread)
    else:
        for index in range(len(thread['joints'])):set_joint_enabled(stage,thread,index,True)


def sever_contacted_joints(stage,threads,global_indices,*,defer_end_contact=True):
    """Apply force-qualified events from ThreadCutter; never release cloth."""
    connections=[(thread,index) for thread in threads for index in range(len(thread['joints']))]
    for k in global_indices:
        if not 0<=int(k)<len(connections):raise ValueError('Unknown thread joint')
        thread,index=connections[int(k)]
        set_joint_enabled(stage,thread,index,False,restore_contact=not defer_end_contact)
