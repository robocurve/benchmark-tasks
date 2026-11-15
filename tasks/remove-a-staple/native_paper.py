"""Native 100 µm surface-FEM paper and contact-capable plastic staple.

No cloth attachment, task-phase release or animated extraction. Coupled probes
remain necessary: authoring a joint/material is not evidence of fidelity.
"""
import math
import numpy as np
from paper import PaperMaterial
from materials import clinched_staple_centerline, SteelSpec
from physx_material import PhysxPlasticHinge


def author_paper(stage, path, mesh, center=(0.,0.,0.), material=None):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade, Vt
    material=material or PaperMaterial();audit=mesh.audit(material)
    shape=UsdGeom.Mesh.Define(stage,path)
    points=mesh.rest+np.asarray(center)
    shape.CreatePointsAttr(Vt.Vec3fArray(points.tolist()))
    shape.CreateFaceVertexCountsAttr([3]*len(mesh.faces))
    shape.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
    shape.CreateSubdivisionSchemeAttr('none');shape.CreateDoubleSidedAttr(True)
    shape.CreateDisplayColorAttr([Gf.Vec3f(.94,.94,.91)])
    prim=shape.GetPrim()
    def set_attr(prim,name,value):
        attr=prim.GetAttribute(name)
        if not attr or not attr.Set(value):raise RuntimeError('Native paper schema missing: '+name)
    for api in ('OmniPhysicsDeformableBodyAPI','OmniPhysicsSurfaceDeformableSimAPI','PhysxSurfaceDeformableBodyAPI'):
        if not prim.ApplyAPI(api):raise RuntimeError('Required native surface API missing: '+api)
    set_attr(prim,'omniphysics:restShapePoints',Vt.Vec3fArray(mesh.rest.tolist()))
    set_attr(prim,'omniphysics:restTriVtxIndices',[Gf.Vec3i(*map(int,f)) for f in mesh.faces])
    set_attr(prim,'omniphysics:mass',audit['surface_area_m2']*material.areal_mass_kg_m2)
    set_attr(prim,'physxDeformableBody:selfCollision',True)
    set_attr(prim,'physxDeformableBody:solverPositionIterationCount',64)
    set_attr(prim,'physxDeformableBody:disableGravity',False)
    UsdPhysics.CollisionAPI.Apply(prim)
    collider=PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collider.CreateContactOffsetAttr(material.contact_offset_m)
    collider.CreateRestOffsetAttr(material.rest_offset_m)
    mat=UsdShade.Material.Define(stage,path+'/PaperMaterial');mp=mat.GetPrim()
    for api in ('OmniPhysicsBaseMaterialAPI','OmniPhysicsDeformableMaterialAPI',
                'OmniPhysicsSurfaceDeformableMaterialAPI','PhysxSurfaceDeformableMaterialAPI'):
        if not mp.ApplyAPI(api):raise RuntimeError('Required native material API missing: '+api)
    # PhysX multiplies the bend coefficient by thickness^3 internally.
    # Its documented shell relation is E/[12(1-nu^2)], not E itself:
    # https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/deformables/deformable_bodies.html
    for name,value in {'omniphysics:density':material.areal_mass_kg_m2/material.thickness_m,
                      'omniphysics:dynamicFriction':material.friction,
                      'omniphysics:youngsModulus':material.youngs_modulus_pa,
                      'omniphysics:poissonsRatio':material.poissons_ratio,
                      'omniphysics:surfaceThickness':material.thickness_m,
                      'omniphysics:surfaceBendStiffness':material.youngs_modulus_pa/(12*(1-material.poissons_ratio**2)),
                      'physxDeformableMaterial:elasticityDamping':.01,
                      'physxDeformableMaterial:bendDamping':.005}.items():set_attr(mp,name,value)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
    return shape


def author_staple(stage,root,origin=(0.,0.,0.),sample_m=.0005, fixed_crown=False, yaw=0., paper_stack_m=.0002):
    """Discrete rectangular wire; all hinge rotations are measured from physics.

    Bending around local Y follows elastoplastic steel. Twist and orthogonal
    bending are locked in this first coupon; disclose this limitation. The full
    task requires either an unrestricted rod model or validated planar loading.
    """
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from dataclasses import replace
    local=clinched_staple_centerline(sample_m=sample_m,paper_stack_m=paper_stack_m)
    delta=np.diff(local,axis=0);lengths=np.linalg.norm(delta,axis=1)
    angles=np.arctan2(-delta[:,2],delta[:,0])
    Rz=np.array(((math.cos(yaw),-math.sin(yaw),0.),(math.sin(yaw),math.cos(yaw),0.),(0.,0.,1.)))
    points=local@Rz.T+np.asarray(origin)
    centers=(points[:-1]+points[1:])/2
    def quat(angle):
        cy,sy=math.cos(angle/2),math.sin(angle/2)
        cz,sz=math.cos(yaw/2),math.sin(yaw/2)
        return Gf.Quatf(cz*cy,-sz*sy,cz*sy,sz*cy)
    paths=[];hinges=[]
    for i,(center,L,angle) in enumerate(zip(centers,lengths,angles)):
        path=f'{root}/wire_{i:03d}';paths.append(path)
        body=UsdGeom.Xform.Define(stage,path)
        body.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
        body.AddOrientOp().Set(quat(angle))
        prim=body.GetPrim();UsdPhysics.RigidBodyAPI.Apply(prim)
        steel=SteelSpec();UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(7850*L*steel.wire_width_m*steel.wire_thickness_m))
        rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(128);rb.CreateSolverVelocityIterationCountAttr(32)
        rb.CreateEnableCCDAttr(True);rb.CreateSleepThresholdAttr(0.)
        shape=UsdGeom.Cube.Define(stage,path+'/steel')
        shape.CreateSizeAttr(1.);shape.AddScaleOp().Set(Gf.Vec3f(L,steel.wire_width_m,steel.wire_thickness_m))
        shape.CreateDisplayColorAttr([Gf.Vec3f(.65,.67,.7)]);UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        cp=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());cp.CreateContactOffsetAttr(.000002);cp.CreateRestOffsetAttr(0.)
        neighbors=[Sdf.Path(f'{root}/wire_{j:03d}') for j in (i-1,i+1) if 0<=j<len(lengths)]
        UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(neighbors)
        if i:
            joint=UsdPhysics.RevoluteJoint.Define(stage,f'{root}/bend_{i:03d}')
            joint.CreateAxisAttr('Y');joint.CreateBody0Rel().SetTargets([Sdf.Path(paths[i-1])]);joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            joint.CreateLocalPos0Attr(Gf.Vec3f(lengths[i-1]/2,0.,0.));joint.CreateLocalPos1Attr(Gf.Vec3f(-L/2,0.,0.))
            for side,a in ((0,angles[i-1]),(1,angle)):
                getattr(joint,f'CreateLocalRot{side}Attr')(Gf.Quatf(math.cos(a/2),0.,-math.sin(a/2),0.))
            joint.CreateLowerLimitAttr(-170.);joint.CreateUpperLimitAttr(170.)
            material=PhysxPlasticHinge(joint.GetPrim(),replace(steel,hinge_length_m=float((L+lengths[i-1])/2)))
            hinges.append({'path0':paths[i-1],'path1':path,'reference_relative_angle':float(angle-angles[i-1]),'material':material})
    if fixed_crown:
        # Explicit diagnostic fixture: never enabled in a robot task.
        index=int(np.argmax(centers[:,2]));joint=UsdPhysics.FixedJoint.Define(stage,root+'/coupon_crown_fixture')
        joint.CreateBody1Rel().SetTargets([Sdf.Path(paths[index])]);joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,centers[index])))
        joint.CreateLocalRot0Attr(quat(angles[index]))
    return {'paths':paths,'hinges':hinges,'fixed_crown':fixed_crown,'initial_centers':centers.tolist(),'lengths_m':lengths.tolist()}


def update_staple(physics,metadata):
    transforms={path:physics.get_rigidbody_transformation(path) for path in metadata['paths']}
    if not all(tr.get('ret_val') for tr in transforms.values()):raise RuntimeError('Missing native staple transforms')
    def rotation(path):
        x,y,z,w=map(float,transforms[path]['rotation'])
        return np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                         (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                         (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))
    results=[]
    for hinge in metadata['hinges']:
        R=rotation(hinge['path0']).T@rotation(hinge['path1'])
        relative=math.atan2(R[0,2],R[0,0])-hinge['reference_relative_angle']
        relative=(relative+math.pi)%(2*math.pi)-math.pi
        results.append(hinge['material'].update_from_measured_angle(relative))
    return results


def measured_wire_segments(physics, metadata):
    segments=[]
    for path,length in zip(metadata['paths'],metadata['lengths_m']):
        tr=physics.get_rigidbody_transformation(path)
        if not tr.get('ret_val'):raise RuntimeError('Missing native wire body '+path)
        x,y,z,w=map(float,tr['rotation'])
        axis=np.array((1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)))
        centre=np.asarray(tr['position'],float)
        segments.append((centre-axis*length/2,centre+axis*length/2))
    return np.asarray(segments)
