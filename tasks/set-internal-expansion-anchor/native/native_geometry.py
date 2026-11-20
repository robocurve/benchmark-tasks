"""Task-local native USD primitives/joints. No native imports before SimulationApp.

Uses the unmodified MIT partkit scene/lifecycle plumbing alongside this file.
No object pose changes, robot attachments, catches, or task-conditioned forces.
"""
from dataclasses import dataclass
import sys,math,json,os,traceback
from pathlib import Path
import numpy as np
import partkit as pk

G=None
@dataclass(frozen=True)
class NativePart(pk.Part):
    native_spec: object=None

def objects(geometry):
    global G
    G=geometry;out=[]
    for p in G['parts']:
        # Centred conservative task metadata from the same physical shape vertices.
        from scipy.spatial.transform import Rotation
        points=[]
        for s in p['geoms']:
            if s['type']=='convex':v=np.asarray(s['vertices'],float)
            elif s['type']=='box':v=np.array([[x,y,z]for x in[-s['size'][0],s['size'][0]]for y in[-s['size'][1],s['size'][1]]for z in[-s['size'][2],s['size'][2]]])
            else:raise ValueError('Unsupported physical-bound shape '+s['type'])
            points.append(v@Rotation.from_quat(s['quat_xyzw']).as_matrix().T+s['pos'])
        ext=2*np.maximum(np.abs(np.vstack(points)).max(0),1e-6)
        out.append(NativePart(name=p['name'],pos=tuple(map(float,p['pos'])),size=tuple(ext),mass=p['mass_kg'],
            color=(.5,.5,.5),quat=tuple(map(float,p['quat_xyzw'])),fixed=p['fixed'],damping=0.,native_spec=p))
    return out

def _spawn_impl(prim_path,cfg,translation=None,orientation=None,**kwargs):
    import isaaclab.sim as su
    from isaaclab.sim.utils import bind_physics_material,bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf,PhysxSchema,UsdGeom,UsdPhysics
    from yam_arena.station import linear_rgb
    stage=get_current_stage();spec=pk._SPECS[cfg.part_name];p=spec.native_spec
    root=UsdGeom.Xform.Define(stage,prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float,translation or (0,0,0))))
    x,y,z,w=orientation if orientation is not None else (0,0,0,1)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(w),float(x),float(y),float(z)))
    body=UsdGeom.Xform.Define(stage,prim_path+'/body');bp=body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(bp)
    PhysxSchema.PhysxContactReportAPI.Apply(bp).CreateThresholdAttr(0.)
    if p['fixed']:UsdPhysics.RigidBodyAPI(bp).CreateKinematicEnabledAttr(True)
    mass=UsdPhysics.MassAPI.Apply(bp);mass.CreateMassAttr(float(p['mass_kg']))
    if 'inertia_kg_m2' in p:
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,p['inertia_kg_m2'])))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,p['inertia_pos'])))
        x,y,z,w=p['inertia_quat_xyzw'];mass.CreatePrincipalAxesAttr(Gf.Quatf(float(w),Gf.Vec3f(float(x),float(y),float(z))))
    rb=PhysxSchema.PhysxRigidBodyAPI.Apply(bp)
    rb.CreateMaxDepenetrationVelocityAttr(1.);rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(4);rb.CreateAngularDampingAttr(0.);rb.CreateLinearDampingAttr(0.)
    rb.CreateEnableCCDAttr(not p['fixed'])
    for i,s in enumerate(p['geoms']):
        path=f'{prim_path}/body/geom_{i}'
        _boundary('spawn_shape',part=cfg.part_name,path_repr=repr(path),source_geometry_name=s['name'])
        if s['type']=='box':
            sh=UsdGeom.Cube.Define(stage,path);sh.CreateSizeAttr(1.)
        elif s['type']=='capsule':
            sh=UsdGeom.Capsule.Define(stage,path);sh.CreateAxisAttr('Z');sh.CreateRadiusAttr(float(s['size'][0]));sh.CreateHeightAttr(float(2*s['size'][1]))
        elif s['type']=='convex':
            from scipy.spatial import ConvexHull
            verts=np.array(s['vertices'],float);ch=ConvexHull(verts);faces=ch.simplices.copy()
            for fi,tri in enumerate(faces):
                if np.dot(np.cross(verts[tri[1]]-verts[tri[0]],verts[tri[2]]-verts[tri[0]]),ch.equations[fi,:3])<0:faces[fi,[1,2]]=faces[fi,[2,1]]
            sh=UsdGeom.Mesh.Define(stage,path);sh.CreatePointsAttr([Gf.Vec3f(*map(float,x))for x in verts]);sh.CreateFaceVertexCountsAttr([3]*len(faces));sh.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist());sh.CreateSubdivisionSchemeAttr('none')
            UsdPhysics.MeshCollisionAPI.Apply(sh.GetPrim()).CreateApproximationAttr('convexHull')
        elif s['type']=='cylinder':
            #64vertices at32sides; authored convex mesh avoids implicit cylinder
            #cooking with an unknown GPU hull budget. Radial approximation disclosed.
            n=G['cylinder_polygon_sides'];r,h=s['size'][:2]
            pts=[(r*math.cos(a*2*math.pi/n),r*math.sin(a*2*math.pi/n),zz) for zz in (-h,h) for a in range(n)]
            counts=[n,n]+[4]*n;indices=list(reversed(range(n)))+list(range(n,2*n))
            for a in range(n):b=(a+1)%n;indices.extend([a,b,b+n,a+n])
            sh=UsdGeom.Mesh.Define(stage,path);sh.CreatePointsAttr([Gf.Vec3f(*map(float,x)) for x in pts]);sh.CreateFaceVertexCountsAttr(counts);sh.CreateFaceVertexIndicesAttr(indices);sh.CreateSubdivisionSchemeAttr('none')
            UsdPhysics.MeshCollisionAPI.Apply(sh.GetPrim()).CreateApproximationAttr('convexHull')
        else:raise ValueError(s['type'])
        sh.GetPrim().SetCustomDataByKey('source_geometry_name',s['name'])
        sh.AddTranslateOp().Set(Gf.Vec3d(*map(float,s['pos'])))
        x,y,z,w=s['quat_xyzw'];sh.AddOrientOp().Set(Gf.Quatf(float(w),float(x),float(y),float(z)))
        if s['type']=='box':sh.AddScaleOp().Set(Gf.Vec3f(*map(float,2*np.array(s['size']))))
        if s['collision_enabled']:
            UsdPhysics.CollisionAPI.Apply(sh.GetPrim());cp=PhysxSchema.PhysxCollisionAPI.Apply(sh.GetPrim())
            cp.CreateContactOffsetAttr(G['contact_offset_m']);cp.CreateRestOffsetAttr(0.)
            mat=f'{prim_path}/Looks/material_{i}';mc=su.RigidBodyMaterialCfg(static_friction=s['friction'],dynamic_friction=s.get('dynamic_friction',s['friction']),restitution=0.)
            mc.func(mat,mc);bind_physics_material(path,mat,stage=stage)
        vis=f'{prim_path}/Looks/visual_{i}';vc=su.PreviewSurfaceCfg(diffuse_color=linear_rgb(tuple(s['color'])),roughness=.45)
        vc.func(vis,vc);bind_visual_material(path,vis,stage=stage)
    return root.GetPrim()

def _boundary(event,**data):
    global _LAST_BOUNDARY
    _LAST_BOUNDARY=dict(event=event,**data)
    trace=Path(os.environ.get('ROBOCURVE_TRACE_DIR',str(Path(__file__).parent/'native-trace')))
    trace.mkdir(parents=True,exist_ok=True)
    with (trace/'adapter-boundaries.jsonl').open('a') as f:f.write(json.dumps(_LAST_BOUNDARY)+'\n')

def _failure(event,**data):
    _boundary(event,full_traceback=traceback.format_exc(),last_boundary=dict(globals().get('_LAST_BOUNDARY',{})),**data)
    print('SDS_ADAPTER_EXCEPTION '+json.dumps(_LAST_BOUNDARY),flush=True)

def spawn(prim_path,cfg,translation=None,orientation=None,**kwargs):
    _boundary('spawn_enter',prim_path_repr=repr(prim_path),part_name_repr=repr(cfg.part_name),translation_repr=repr(translation),orientation_repr=repr(orientation))
    try:return _spawn_impl(prim_path,cfg,translation,orientation,**kwargs)
    except Exception:
        _failure('spawn_error',prim_path_repr=repr(prim_path),part_name_repr=repr(cfg.part_name));raise

pk._spawn_part=spawn
_original_arena_object=pk._arena_object
def _arena_object(spec):
    _boundary('arena_object',name_repr=repr(spec.name),pos_repr=repr(spec.pos))
    try:return _original_arena_object(spec)
    except Exception:
        _failure('arena_object_error',name_repr=repr(spec.name));raise
pk._arena_object=_arena_object

def quat(R):
    from yam_arena.ik import matrix_to_quat_xyzw
    return matrix_to_quat_xyzw(R)
def frame_x(axis):
    x=np.asarray(axis,dtype=float);y=np.cross([0,0,1],x)
    if np.linalg.norm(y)<1e-8:y=np.cross([0,1,0],x)
    y/=np.linalg.norm(y);return np.column_stack([x,y,np.cross(x,y)])

def _author_joints_impl(stage,env_path):
    from pxr import Gf,Sdf,UsdGeom,UsdPhysics
    UsdGeom.Xform.Define(stage,env_path+'/dropin_material_joints')
    for j in G['joints']:
        path=env_path+'/dropin_material_joints/'+j['name']
        _boundary('author_joint',env_path_repr=repr(env_path),joint_path_repr=repr(path),parent_repr=repr(j['parent']),child_repr=repr(j['child']))
        if j['kind']=='hinge':joint=UsdPhysics.RevoluteJoint.Define(stage,path);F=frame_x(j['axes_child'][0])
        elif j['kind']=='slider':joint=UsdPhysics.PrismaticJoint.Define(stage,path);F=frame_x(j['axes_child'][0])
        else:joint=UsdPhysics.Joint.Define(stage,path);F=np.eye(3)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(env_path+'/'+j['parent']+'/body')]);joint.CreateBody1Rel().SetTargets([Sdf.Path(env_path+'/'+j['child']+'/body')])
        for side,pos,R in [(0,j['parent_local_pos'],np.array(j['parent_R']).T@np.array(j['child_R'])@F),(1,j['child_local_pos'],F)]:
            getattr(joint,f'CreateLocalPos{side}Attr')().Set(Gf.Vec3f(*map(float,pos)))
            x,y,z,w=quat(R);getattr(joint,f'CreateLocalRot{side}Attr')().Set(Gf.Quatf(float(w),Gf.Vec3f(float(x),float(y),float(z))))
        joint.CreateCollisionEnabledAttr(False)
        if j['kind']=='hinge':
            joint.CreateAxisAttr('X')
            if j['limits_rad']:
                joint.CreateLowerLimitAttr(math.degrees(j['limits_rad'][0]));joint.CreateUpperLimitAttr(math.degrees(j['limits_rad'][1]))
            axes=['angular']
        elif j['kind']=='slider':
            joint.CreateAxisAttr('X')
            joint.CreateLowerLimitAttr(float(j['limits_rad'][0]));joint.CreateUpperLimitAttr(float(j['limits_rad'][1]))
            drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(float(j['stiffness_N_m']));drive.CreateDampingAttr(float(j['damping_Ns_m']))
            drive.CreateTargetPositionAttr(float(j['springref_m']));drive.CreateTargetVelocityAttr(0.);drive.CreateMaxForceAttr(1e10)
            axes=[]
        else:
            for axis in ['transX','transY','transZ']:
                lim=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis);lim.CreateLowAttr(1.);lim.CreateHighAttr(-1.)
            axes=['rotX','rotY','rotZ']
        for k,axis in enumerate(axes):
            drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis);drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(j['stiffness_Nm_rad'][k]*math.pi/180)
            drive.CreateDampingAttr(j['damping_Nms_rad'][k]*math.pi/180)
            drive.CreateTargetPositionAttr(math.degrees(j['springref_rad'][k]));drive.CreateTargetVelocityAttr(0.);drive.CreateMaxForceAttr(float(j.get('max_force_Nm',1e10)))
    print('SDS_NATIVE_JOINTS '+str(len(G['joints'])),flush=True)

def author_joints(stage,env_path):
    _boundary('author_joints_enter',env_path_repr=repr(env_path))
    try:return _author_joints_impl(stage,env_path)
    except Exception:
        _failure('author_joints_error',env_path_repr=repr(env_path));raise

_original_patch_scene=pk._patch_scene

def patch_scene(mod):
    _original_patch_scene(mod)
    from isaaclab.scene import interactive_scene as isc
    if getattr(isc.InteractiveScene,'_sds_native_joints',False):return
    old=isc.InteractiveScene.__init__
    def init(self,*a,**k):
        old(self,*a,**k);author_joints(self.stage,self.env_prim_paths[0])
    isc.InteractiveScene.__init__=init;isc.InteractiveScene._sds_native_joints=True

pk._Finder.targets['yam_arena.scene']=patch_scene
if 'yam_arena.scene' in sys.modules:patch_scene(sys.modules['yam_arena.scene'])
