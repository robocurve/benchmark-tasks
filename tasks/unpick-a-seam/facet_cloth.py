"""Finite rigid-facet shell candidate with native passive bending hinges.

This is an approximately inextensible cloth model, not surface FEM. Each
triangle is a free thin convex prism. Shared edges have ordinary revolute
joints and finite bending springs; no body is attached to the world. Adjacent
material facets share collision exclusions, while all thread contacts remain
enabled. Native deflection/contact evidence is required before using it.
"""
from collections import defaultdict
from dataclasses import dataclass
import math
import numpy as np
from geometry import Mesh,TABLE_TOP,STITCH_Y


@dataclass(frozen=True)
class ShellMaterial:
    thickness_m: float=.0004
    density_kg_m3: float=350.
    bend_modulus_pa: float=8e4
    poissons_ratio: float=.3
    damping_ratio: float=.15
    friction: float=.55
    minimum_panel_mass_kg: float=0.

    @property
    def rigidity_nm(self):
        return self.bend_modulus_pa*self.thickness_m**3/(12*(1-self.poissons_ratio**2))


def shell_mesh(upper=True):
    """Coarse shell with exactly the existing physical surface and holes."""
    xs=np.array((.235,.245,.255,.265,.275,.285,.2942,.2958,.3042,.3058,.310)
                if upper else (.290,.2942,.2958,.3042,.3058,.315,.325,.335,.345,.355,.365))
    ys=np.sort(np.r_[-.040,STITCH_Y-.0008,STITCH_Y+.0008,.040])
    points=[];faces=[];lookup={}
    def vertex(i,j):
        if (i,j) not in lookup:
            lookup[i,j]=len(points)
            points.append((xs[i],ys[j],TABLE_TOP+(.004 if upper else .0032)))
        return lookup[i,j]
    for i in range(len(xs)-1):
        for j in range(len(ys)-1):
            x,y=(xs[i]+xs[i+1])/2,(ys[j]+ys[j+1])/2
            if any(abs(x-hx)<.0008-1e-12 and abs(y-hy)<.0008-1e-12
                   for hx in (.295,.305) for hy in STITCH_Y):continue
            a,b,c,d=vertex(i,j),vertex(i+1,j),vertex(i+1,j+1),vertex(i,j+1)
            faces.extend(((a,b,c),(a,c,d)))
    mesh=Mesh(np.asarray(points),np.asarray(faces,int));mesh.audit();return mesh


def rotation_quaternion(matrix):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(matrix).as_quat()


def mechanics(mesh,material=ShellMaterial(),primitive_boxes=False):
    vertex_ids=mesh.faces
    if primitive_boxes:
        if len(mesh.faces)%2:raise ValueError('Box panels require paired rectangular triangles')
        ids=[]
        for first,second in zip(mesh.faces[::2],mesh.faces[1::2]):
            if first[0]!=second[0] or first[2]!=second[1]:
                raise ValueError('Triangle pair does not describe one rectangle')
            ids.append((first[0],first[1],first[2],second[2]))
        vertex_ids=np.asarray(ids,int)
    points=mesh.rest[vertex_ids];centres=points.mean(1)
    cross=np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0])
    areas=np.linalg.norm(cross,axis=1)/2;normals=cross/(2*areas[:,None])
    if primitive_boxes:
        edge1,edge2=points[:,1]-points[:,0],points[:,3]-points[:,0]
        if not (np.allclose(points[:,2],points[:,0]+edge1+edge2,atol=1e-12) and
                np.allclose(np.sum(edge1*edge2,axis=1),0.,atol=1e-14)):
            raise ValueError('Native box panels must be rectangles')
        areas*=2
    masses=np.maximum(areas*material.thickness_m*material.density_kg_m3,material.minimum_panel_mass_kg)
    edges=defaultdict(list);vertices=defaultdict(list)
    for face,ids in enumerate(vertex_ids):
        for vertex in ids:vertices[int(vertex)].append(face)
        for a,b in zip(ids,np.roll(ids,-1)):edges[tuple(sorted((int(a),int(b))))].append(face)
    neighbours=[set() for _ in points]
    for touching in vertices.values():
        for face in touching:neighbours[face].update(set(touching)-{face})
    hinges=[]
    for edge,owners in sorted(edges.items()):
        if len(owners)!=2:continue
        i,j=owners;a,b=mesh.rest[list(edge)];direction=b-a;length=np.linalg.norm(direction)
        axis=direction/length;normal=normals[i];frame=np.column_stack((axis,np.cross(normal,axis),normal))
        anchor=(a+b)/2
        width=sum(np.linalg.norm(np.cross(centres[k]-anchor,axis)) for k in owners)
        stiffness=material.rigidity_nm*length/width
        inertia=[]
        for k in owners:
            rel=points[k]-anchor;perpendicular=rel-(rel@axis)[:,None]*axis
            if primitive_boxes:
                offset=centres[k]-anchor;offset-=np.dot(offset,axis)*axis
                around_centre=perpendicular-offset
                inertia.append(masses[k]*(np.dot(offset,offset)+np.sum(around_centre**2)/12+
                                          material.thickness_m**2/12))
            else:
                inertia.append(masses[k]*((np.sum(perpendicular**2)+np.sum(perpendicular.sum(0)**2))/12
                                          +material.thickness_m**2/12))
        effective=inertia[0]*inertia[1]/sum(inertia)
        damping=2*material.damping_ratio*math.sqrt(stiffness*effective)
        hinges.append(dict(edge=edge,faces=owners,anchor=anchor,frame=frame,
                           stiffness_nm_rad=stiffness,damping_nms_rad=damping))
    endpoint_pairs=[]
    for hinge in hinges:
        i,j=hinge['faces']
        for vertex in hinge['edge']:
            endpoint_pairs.append((i,int(np.flatnonzero(vertex_ids[i]==vertex)[0]),
                                   j,int(np.flatnonzero(vertex_ids[j]==vertex)[0])))
    return dict(points=points,centres=centres,normals=normals,masses=masses,
                neighbours=neighbours,hinges=hinges,
                endpoint_pairs=np.asarray(endpoint_pairs,int).reshape(-1,4),
                vertex_ids=vertex_ids,primitive_boxes=primitive_boxes)


# Shared-edge geometry adapted from the team paper candidate; native poses only.
def geometric_edge_measurements(model,world,rotations,thickness_m=.0004):
    """Separate empty edge apertures from compression and tangential sliding.

    This is observation only. It does not change the native shell or decide
    whether a material fracture occurred; native break events report that.
    Thickness is the authored analytic box thickness, not a rendered skin.
    """
    if not np.isfinite(thickness_m) or thickness_m<=0:
        raise ValueError('Positive finite physical panel thickness required')
    pairs=model['endpoint_pairs'].reshape(-1,2,4)
    i,j=pairs[:,0,0],pairs[:,0,2]
    left=world[pairs[:,:,0],pairs[:,:,1]]
    right=world[pairs[:,:,2],pairs[:,:,3]]
    delta=right-left
    local_edge=model['points'][pairs[:,1,0],pairs[:,1,1]]-model['points'][pairs[:,0,0],pairs[:,0,1]]
    local_edge/=np.linalg.norm(local_edge,axis=1)[:,None]
    e0=np.einsum('nij,nj->ni',rotations[i],local_edge)
    e1=np.einsum('nij,nj->ni',rotations[j],local_edge)
    edge=e0+e1
    edge/=np.linalg.norm(edge,axis=1)[:,None]
    n0=np.einsum('nij,nj->ni',rotations[i],model['normals'][i])
    n1=np.einsum('nij,nj->ni',rotations[j],model['normals'][j])
    normal=n0+n1
    normal-=np.sum(normal*edge,axis=1)[:,None]*edge
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    across=np.cross(edge,normal)
    rest_across=model['centres'][j]-model['centres'][i]
    sign=np.sign(np.sum(np.cross(local_edge,model['normals'][i])*rest_across,axis=1))
    across*=sign[:,None]
    signed_opening=np.einsum('nej,nj->ne',delta,across)
    along=np.einsum('nej,nj->ne',delta,edge)
    normal_delta=np.einsum('nej,nj->ne',delta,normal)
    normal_thickness=thickness_m/2*(np.abs(np.sum(n0*normal,axis=1))+np.abs(np.sum(n1*normal,axis=1)))
    normal_clearance=np.maximum(np.abs(normal_delta)-normal_thickness[:,None],0.)
    opening=np.maximum(signed_opening,0.)
    aperture=np.hypot(opening,normal_clearance)
    left_projection=np.einsum('nej,nj->ne',left,edge)
    right_projection=np.einsum('nej,nj->ne',right,edge)
    overlap=np.minimum(left_projection.max(1),right_projection.max(1))-np.maximum(left_projection.min(1),right_projection.min(1))
    if not all(np.isfinite(x).all() for x in (opening,normal_clearance,aperture,overlap)):
        raise ValueError('Folded or nonfinite fabric edge geometry')
    worst=np.unravel_index(int(np.argmax(aperture)),aperture.shape)
    h,k=worst
    return dict(maximum_positive_edge_opening_m=float(opening.max()),
        maximum_empty_normal_clearance_m=float(normal_clearance.max()),
        maximum_empty_aperture_m=float(aperture.max()),
        maximum_across_edge_compression_m=float(np.maximum(-signed_opening,0.).max()),
        maximum_along_edge_slip_m=float(np.abs(along).max()),
        minimum_shared_edge_overlap_m=float(overlap.min()),
        edges_without_overlap=int(np.count_nonzero(overlap<=0.)),
        worst_aperture=dict(panel_indices=[int(i[h]),int(j[h])],endpoint=int(k),
            opening_m=float(opening[h,k]),normal_clearance_m=float(normal_clearance[h,k])),
        interpretation='Empty aperture geometry only. Compression and tangential slip are reported separately; native hinge fracture is a distinct event.')


def author_shell(stage,path,mesh,material=ShellMaterial(),color=(.76,.68,.47),*,primitive_boxes=False):
    """Author only before simulation. All shell facet bodies remain dynamic."""
    from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Vt
    model=mechanics(mesh,material,primitive_boxes);bodies=[];joints=[]
    mat=UsdShade.Material.Define(stage,path+'/Material')
    pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(material.friction);pm.CreateDynamicFrictionAttr(material.friction)
    pm.CreateRestitutionAttr(0.)
    prism_faces=((0,2,1),(3,4,5),(0,1,4),(0,4,3),(1,2,5),(1,5,4),(2,0,3),(2,3,5))
    for i,(points,centre,normal,mass) in enumerate(zip(model['points'],model['centres'],model['normals'],model['masses'])):
        root=UsdGeom.Xform.Define(stage,f'{path}/facet_{i:03d}')
        root.AddTranslateOp().Set(Gf.Vec3d(*centre));prim=root.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim);UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(128);rb.CreateSolverVelocityIterationCountAttr(32)
        rb.CreateMaxDepenetrationVelocityAttr(.15)
        rb.CreateLinearDampingAttr(0.);rb.CreateAngularDampingAttr(0.)
        local=points-centre
        if primitive_boxes:
            shape=UsdGeom.Cube.Define(stage,str(root.GetPath())+'/Shape')
            shape.CreateSizeAttr(1.)
            x,y=points[1]-points[0],points[3]-points[0]
            width,height=np.linalg.norm(x),np.linalg.norm(y)
            q=rotation_quaternion(np.column_stack((x/width,y/height,normal)))
            shape.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]),Gf.Vec3d(*q[:3])))
            shape.AddScaleOp().Set(Gf.Vec3d(width,height,material.thickness_m))
        else:
            shape=UsdGeom.Mesh.Define(stage,str(root.GetPath())+'/Shape')
            vertices=np.vstack((local-normal*material.thickness_m/2,local+normal*material.thickness_m/2))
            shape.CreatePointsAttr(Vt.Vec3fArray(vertices.tolist()))
            shape.CreateFaceVertexCountsAttr([3]*len(prism_faces));shape.CreateFaceVertexIndicesAttr(np.ravel(prism_faces).tolist())
            shape.CreateSubdivisionSchemeAttr('none');shape.CreateDoubleSidedAttr(True)
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        if not primitive_boxes:
            UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexHull')
            hull=PhysxSchema.PhysxConvexHullCollisionAPI.Apply(shape.GetPrim())
            hull.CreateMinThicknessAttr(float(material.thickness_m/4))
        cp=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        cp.CreateContactOffsetAttr(.00005);cp.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
        UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
            [Sdf.Path(f'{path}/facet_{j:03d}') for j in sorted(model['neighbours'][i])])
        bodies.append(str(root.GetPath()))
    for k,hinge in enumerate(model['hinges']):
        i,j=hinge['faces'];joint=UsdPhysics.RevoluteJoint.Define(stage,f'{path}/hinge_{k:03d}')
        joint.CreateBody0Rel().SetTargets([Sdf.Path(bodies[i])]);joint.CreateBody1Rel().SetTargets([Sdf.Path(bodies[j])])
        q=rotation_quaternion(hinge['frame']);rotation=Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3]))
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(hinge['anchor']-model['centres'][i])))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*(hinge['anchor']-model['centres'][j])))
        joint.CreateLocalRot0Attr(rotation);joint.CreateLocalRot1Attr(rotation)
        joint.CreateAxisAttr('X');joint.CreateCollisionEnabledAttr(False)
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular');drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
        drive.CreateStiffnessAttr(float(hinge['stiffness_nm_rad']*math.pi/180))
        drive.CreateDampingAttr(float(hinge['damping_nms_rad']*math.pi/180))
        drive.CreateMaxForceAttr(float(2*math.pi*hinge['stiffness_nm_rad']))
        joints.append(str(joint.GetPath()))
    return dict(bodies=bodies,joints=joints,mesh=mesh,model=model,material=material)


def measured_surface(shell,poses,*,geometry_aperture_m=None,require_edge_overlap=True):
    """Read native panel geometry, retaining the legacy endpoint telemetry.

    The optional diagnostic uses empty aperture rather than Euclidean endpoint
    error: compression and tangential slip are not holes. This is a numerical
    continuity guard, not a calibrated tear law or a no-damage certificate.
    """
    from scipy.spatial.transform import Rotation
    if geometry_aperture_m is not None and (not np.isfinite(geometry_aperture_m) or geometry_aperture_m<=0):
        raise ValueError('Positive finite empty-aperture limit required')
    poses=np.asarray(poses,float);mesh=shell['mesh'];model=shell['model']
    ids=model['vertex_ids']
    if poses.shape!=(len(ids),7) or not np.isfinite(poses).all():raise ValueError('Invalid facet poses')
    rotations=Rotation.from_quat(poses[:,3:]).as_matrix()
    world=np.einsum('fij,fvj->fvi',rotations,model['points']-model['centres'][:,None])+poses[:,:3,None].transpose(0,2,1)
    summed=np.zeros_like(mesh.rest);count=np.zeros(len(mesh.rest),int)
    np.add.at(summed,ids.ravel(),world.reshape(-1,3));np.add.at(count,ids.ravel(),1)
    points=summed/count[:,None]
    disagreement=np.linalg.norm(world-points[ids],axis=-1)
    pairs=model['endpoint_pairs']
    delta=world[pairs[:,0],pairs[:,1]]-world[pairs[:,2],pairs[:,3]]
    magnitudes=np.linalg.norm(delta,axis=1);worst=int(np.argmax(magnitudes))
    gap=float(magnitudes[worst]);pair=pairs[worst]
    result=dict(maximum_hinge_endpoint_gap_m=gap,
                maximum_vertex_average_error_m=float(disagreement.max()),
                legacy_endpoint_warning=gap>.0001,valid=gap<=.0001,
                worst_endpoint=dict(panel_indices=[int(pair[0]),int(pair[2])],
                    positions_m=[world[pair[0],pair[1]].tolist(),world[pair[2],pair[3]].tolist()],
                    difference_m=delta[worst].tolist()))
    if geometry_aperture_m is not None:
        geometry=geometric_edge_measurements(model,world,rotations,
            shell.get('material',ShellMaterial()).thickness_m)
        result.update(continuity_method='geometric_empty_aperture',
            endpoint_distance_is_stop_condition=False,
            maximum_allowed_empty_aperture_m=float(geometry_aperture_m),geometry=geometry,
            aperture_warning=geometry['maximum_empty_aperture_m']>.0002,
            require_edge_overlap=bool(require_edge_overlap),
            valid=bool(geometry['maximum_empty_aperture_m']<=geometry_aperture_m and
                       (geometry['edges_without_overlap']==0 if require_edge_overlap
                        else gap<=geometry_aperture_m)))
    return points,result


def cooked_hull_geometry(shell,stage):
    """Request native collision hulls; no geometry or pose writes.

    Use the stage-aware PhysX110 API demonstrated by ConvexMeshDataDemo. Failure
    must be reported rather than substituting authored mesh dimensions.
    """
    from omni.physx import get_physx_cooking_interface
    from omni.physx.bindings._physx import PhysxCollisionRepresentationResult
    from pxr import UsdUtils,PhysicsSchemaTools
    cooking=get_physx_cooking_interface();rows=[]
    stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    for i,body in enumerate(shell['bodies']):
        path=body+'/Shape';received=[]
        def on_result(result,convexes):received.append((result,convexes))
        cooking.request_convex_collision_representation(stage_id=stage_id,
            collision_prim_id=PhysicsSchemaTools.sdfPathToInt(path),
            run_asynchronously=False,on_result=on_result)
        if len(received)!=1 or received[0][0]!=PhysxCollisionRepresentationResult.RESULT_VALID:
            raise RuntimeError(f'Native collision representation unavailable for {path}: {received}')
        convexes=received[0][1]
        if len(convexes)!=1:raise RuntimeError(f'Expected one cooked hull for {path}, found {len(convexes)}')
        vertices=np.asarray([[v.x,v.y,v.z] for v in convexes[0].vertices],float)
        if vertices.ndim!=2 or vertices.shape[1]!=3 or len(vertices)<4:
            raise RuntimeError(f'Invalid cooked vertices for {path}')
        normal=shell['model']['normals'][i]
        rows.append(dict(body=i,vertices=len(vertices),
                         thickness_m=float(np.ptp(vertices@normal)),
                         axis_extents_m=np.ptp(vertices,axis=0).tolist()))
    return dict(source='native PhysX collision-representation vertices',facets=rows,
                minimum_thickness_m=min(row['thickness_m'] for row in rows),
                maximum_thickness_m=max(row['thickness_m'] for row in rows))
