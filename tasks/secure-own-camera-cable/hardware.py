"""Scene construction: 74 cm capsule cable and hinged flexible fastening flap.
All initialization before stepping. No object pose writes during manipulation.
"""
from pathlib import Path
import json,sys,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
from prepare_geometry import fk,segments
from yam_arena.constants import READY
MOUNT=np.array([-.05,.007,-.16]);B=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
CREATED={}
def build(stage,links):
 from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema,UsdShade
 from cable_usd import build as cablebuild
 f=fk(READY);lay=json.loads((ROOT/'camera_mount.json').read_text());CR=np.array(lay['mount_rotation_in_gripper']);cp=np.array(lay['mount_position_in_gripper'])
 def vec(p):return Gf.Vec3f(*map(float,p))
 def quat(R):
  q=Rotation.from_matrix(R).as_quat();return Gf.Quatf(float(q[3]),vec(q[:3]))
 mat=UsdShade.Material.Define(stage,'/World/CableFriction');api=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());api.CreateStaticFrictionAttr(1.5);api.CreateDynamicFrictionAttr(1.2);api.CreateRestitutionAttr(0.)
 def collision(prim):
  UsdPhysics.CollisionAPI.Apply(prim);co=PhysxSchema.PhysxCollisionAPI.Apply(prim);co.CreateContactOffsetAttr(.0003);co.CreateRestOffsetAttr(0.);UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
 def box(path,p,sz,color,R=np.eye(3),collide=True):
  o=UsdGeom.Cube.Define(stage,path);o.CreateSizeAttr(1);o.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));o.AddOrientOp().Set(quat(R));o.AddScaleOp().Set(vec(sz));o.CreateDisplayColorAttr([color]);
  if collide:collision(o.GetPrim())
  return o
 gp=links['gripper'];lp=links['link3']
 PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(lp)).CreateThresholdAttr(0.)
 box(gp+'/camera_housing',cp,(.04,.036,.028),(.08,.1,.13),CR)
 box(gp+'/camera_lens',cp+CR@[0,.0183,0],(.012,.0005,.012),(.015,.02,.025),CR,False)
 port=cp+CR@[.024,0,0];box(gp+'/camera_plug',port,(.009,.008,.008),(.15,.15,.17),CR)
 # Permanent strain relief on the outside of the fixed gripper body.
 # The short port-to-tie section is secured; only the remaining cable is free.
 guide=port+CR@np.array([.018,0,0])
 tail_length=float(np.linalg.norm(port-guide))
 d=port-guide;zz=d/np.linalg.norm(d);xx=np.cross([0,0,1],zz);xx/=np.linalg.norm(xx);yy=np.cross(zz,xx);tailR=np.column_stack([xx,yy,zz])
 tail=UsdGeom.Cylinder.Define(stage,gp+'/secured_camera_pigtail');tail.CreateRadiusAttr(.003);tail.CreateHeightAttr(tail_length);tail.AddTranslateOp().Set(Gf.Vec3d(*map(float,(guide+port)/2)));tail.AddOrientOp().Set(quat(tailR));tail.CreateDisplayColorAttr([(.12,.18,.22)]);collision(tail.GetPrim())
 # No extra gripper tie or support. Cable exits directly from camera boot.
 # Preserve existing camera mounting rails and support struts.
 from camera_mount import MOUNT as RAILS,SHOE
 for name,p,sz in RAILS+SHOE:box(gp+'/mount_'+name,cp+CR@p,sz,(.18,.32,.48),CR)
 for s in [-1,1]:
  a=np.array([s*.017,-.02,.025]);z=cp+CR@[s*.015,-.03,-.025];d=z-a;zz=d/np.linalg.norm(d);xx=np.cross([0,0,1],zz);xx/=np.linalg.norm(xx);yy=np.cross(zz,xx)
  box(gp+'/strut'+str(s).replace('-','m'),(a+z)/2,(.007,.007,float(np.linalg.norm(d))),(.32,.36,.4),np.column_stack([xx,yy,zz]))
 def local(p):return MOUNT+B@p
 # Side-mounted mushroom peg: cable drapes over shaft, round cap retains it.
 box(lp+'/clip_foot',[-.05,-.003,-.16],(.016,.016,.018),(.23,.27,.30))
 def peg_cylinder(name,x,length,radius,color):
  o=UsdGeom.Cylinder.Define(stage,lp+'/'+name);o.CreateAxisAttr('X');o.CreateRadiusAttr(radius);o.CreateHeightAttr(length)
  o.AddTranslateOp().Set(Gf.Vec3d(*map(float,local([x,0,0]))));o.AddOrientOp().Set(quat(B));o.CreateDisplayColorAttr([color]);collision(o.GetPrim())
 peg_cylinder('peg_shaft',.010,.040,.010,(.30,.39,.46))
 peg_cylinder('peg_round_cap',.032,.005,.018,(.36,.47,.55))
 # Loop receiving surface on the outer cap; strap folds over cable and cap.
 peg_cylinder('loop_receiver_face',.035,.001,.017,(.10,.65,.70))
 # Original link2 port height and longitudinal position; opposite/right-facing shell.
 port_body=np.array([0,-.073,0]);box(links['link2']+'/cable_anchor',port_body,(.024,.014,.024),(.22,.24,.26))
 # Visible boot bridges the block surface to the physical cable attachment.
 anchor=port_body+np.array([0,-.019,0])
 boot=UsdGeom.Cylinder.Define(stage,links['link2']+'/cable_exit_boot');boot.CreateAxisAttr('Y');boot.CreateRadiusAttr(.0045);boot.CreateHeightAttr(.014);boot.AddTranslateOp().Set(Gf.Vec3d(*map(float,port_body+[0,-.012,0])));boot.CreateDisplayColorAttr([(.08,.10,.12)]);collision(boot.GetPrim())
 start=(f['link2']@np.r_[anchor,1])[:3];end=(f['gripper']@np.r_[guide,1])[:3]
 # Single convex Bezier arc: no intermediate knots or tangent overshoot.
 tt=np.linspace(0,1,1500);t=tt[:,None]
 out_start=f['link2'][:3,:3]@np.array([0.,-1.,0.])
 out_end=f['gripper'][:3,:3]@CR[:,0]
 def path(depth):
  p0=start;p1=start+depth*out_start;p2=end+depth*out_end;p3=end
  return (1-t)**3*p0+3*(1-t)**2*t*p1+3*(1-t)*t*t*p2+t**3*p3
 lo,hi=0.,.6
 for _ in range(40):
  mid=(lo+hi)/2;pts=path(mid);L=np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()
  if L<.74-tail_length:lo=mid
  else:hi=mid
 pts=path((lo+hi)/2);ss=np.r_[0,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))];samp=np.linspace(0,ss[-1],41);verts=np.column_stack([np.interp(samp,ss,pts[:,k]) for k in range(3)])
 centers,lens,rot=segments(verts);q=Rotation.from_matrix(rot).as_quat()
 g={'mass_kg':.12,'radius_m':.003,'centres_world':centers.tolist(),'vertices_world':verts.tolist(),'lengths_m':lens.tolist(),'rotations_xyzw':q.tolist(),'attachments':[{'local_xyz':anchor.tolist()},{'local_xyz':guide.tolist()}]}
 paths=cablebuild(stage,'/World/envs/env_0/service_cable',[links['link2'],gp],g)
 # Existing strain-relief boots resist arbitrary endpoint swivelling.
 for name,body0,body1,p0,p1,R0,R1 in [
  ('anchor_proximal',links['link2'],paths[0],anchor,[-lens[0]/2,0,0],f['link2'][:3,:3].T@rot[0],np.eye(3)),
  ('anchor_distal',paths[-1],gp,[lens[-1]/2,0,0],guide,np.eye(3),f['gripper'][:3,:3].T@rot[-1])]:
  jp='/World/envs/env_0/service_cable/'+name;stage.RemovePrim(jp);j=UsdPhysics.Joint.Define(stage,jp)
  j.CreateBody0Rel().SetTargets([Sdf.Path(body0)]);j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
  j.CreateLocalPos0Attr(vec(p0));j.CreateLocalPos1Attr(vec(p1));j.CreateLocalRot0Attr(quat(R0));j.CreateLocalRot1Attr(quat(R1))
  j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
  for axis in ['transX','transY','transZ']:
   lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(1.);lim.CreateHighAttr(-1.)
  for axis in ['rotX','rotY','rotZ']:
   lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(-30.);lim.CreateHighAttr(30.)
   d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force');d.CreateTargetPositionAttr(0.);d.CreateTargetVelocityAttr(0.);d.CreateStiffnessAttr(.008);d.CreateDampingAttr(.0002);d.CreateMaxForceAttr(.2)
 # Replace internal free ball joints with elastic D6 joints.
 # Joint frames coincide in the initial smooth curve: this is the rest shape.
 for i in range(len(paths)-1):
  jp='/World/envs/env_0/service_cable/bend_'+f'{i:03d}'
  stage.RemovePrim(jp);j=UsdPhysics.Joint.Define(stage,jp)
  j.CreateBody0Rel().SetTargets([Sdf.Path(paths[i])]);j.CreateBody1Rel().SetTargets([Sdf.Path(paths[i+1])])
  j.CreateLocalPos0Attr(vec([lens[i]/2,0,0]));j.CreateLocalPos1Attr(vec([-lens[i+1]/2,0,0]))
  j.CreateLocalRot0Attr(quat(np.eye(3)));j.CreateLocalRot1Attr(quat(rot[i+1].T@rot[i]))
  j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
  for axis in ['transX','transY','transZ']:
   lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(1.);lim.CreateHighAttr(-1.)
  for axis in ['rotX','rotY','rotZ']:
   lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(-30. if axis=='rotX' else -15.);lim.CreateHighAttr(30. if axis=='rotX' else 15.)
   drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);drive.CreateTypeAttr('force');drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
   drive.CreateStiffnessAttr(.001 if axis=='rotX' else .008);drive.CreateDampingAttr(.00006 if axis=='rotX' else .0002);drive.CreateMaxForceAttr(.2)
 for p in paths:
  pr=stage.GetPrimAtPath(p);ph=PhysxSchema.PhysxRigidBodyAPI.Apply(pr);ph.CreateSolverPositionIterationCountAttr(128);ph.CreateSolverVelocityIterationCountAttr(16);ph.CreateAngularDampingAttr(.15);ph.CreateLinearDampingAttr(.15);collision(stage.GetPrimAtPath(p+'/collision'))
 # At each connector only, suppress collision with its own attached housing.
 UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(links['link2'])).CreateFilteredPairsRel().SetTargets([Sdf.Path(paths[0]),Sdf.Path(paths[1])])
 UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(gp)).CreateFilteredPairsRel().SetTargets([Sdf.Path(paths[-1])])
 # Flexible strap and receiving patch.
 fit=json.loads(Path(__file__).with_name('shell_fit.json').read_text())
 def meshstrip(name,data,color):
  mesh=UsdGeom.Mesh.Define(stage,lp+'/'+name);mesh.CreatePointsAttr(data['points']);mesh.CreateFaceVertexCountsAttr(data['counts']);mesh.CreateFaceVertexIndicesAttr(data['indices']);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True);mesh.CreateDisplayColorAttr([color])
 meshstrip('strap_backing',fit['backing'],(.18,.23,.28));# Outward-facing receiving patch.
 patch=np.array(fit['landing']['centres']);patchnorm=[]
 for p in patch:
  n=p-np.array([-.05,-.0335,p[2]]);patchnorm.append((n/np.linalg.norm(n)).tolist())
 root='/World/envs/env_0/service_flap';UsdGeom.Xform.Define(stage,root);flap=[];flapposes=[];N=12;L=.014
 rootlocal=np.array([-.05+.044*np.cos(2.8),-.0335+.029*np.sin(2.8),-.16])
 direction=np.array([-1.,0.,0.]);direction/=np.linalg.norm(direction);yy=np.array([0.,0.,1.]);localR=np.column_stack([direction,yy,np.cross(direction,yy)]);initialR=f['link3'][:3,:3]@localR;attach=(f['link3']@np.r_[rootlocal,1])[:3]
 # Curved rest shape, toward the outside (+hook X), with compliant joints.
 edge=attach.copy();previousR=None
 for i in range(N):
  restR=localR@Rotation.from_euler('y',-(5+30*min(i,9)/9),degrees=True).as_matrix()
  initialR=f['link3'][:3,:3]@restR
  pos=edge+initialR[:,0]*L/2;edge=edge+initialR[:,0]*L
  p=root+f'/segment_{i:03d}';flap.append(p);xf=UsdGeom.Xform.Define(stage,p);xf.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));xf.AddOrientOp().Set(quat(initialR));UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());PhysxSchema.PhysxContactReportAPI.Apply(xf.GetPrim()).CreateThresholdAttr(0.);UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(.0056/10)
  ph=PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim());ph.CreateSolverPositionIterationCountAttr(128);ph.CreateSolverVelocityIterationCountAttr(16);ph.CreateAngularDampingAttr(2.)
  box(p+'/cloth',np.zeros(3),(L,.024 if i>=7 else .020,.0015),(.045,.045,.045) if i>=7 else (.025,.025,.025))
  box(p+'/single_hook_face',[0,0,.001],(L,.020,.0003),(.030,.030,.030),collide=False)
  j=UsdPhysics.RevoluteJoint.Define(stage,root+f'/hinge_{i}');j.CreateBody0Rel().SetTargets([Sdf.Path(lp if i==0 else flap[i-1])]);j.CreateBody1Rel().SetTargets([Sdf.Path(p)]);j.CreateAxisAttr('Y');j.CreateLocalPos0Attr(vec(rootlocal if i==0 else [L/2,0,0]));j.CreateLocalPos1Attr(vec([-L/2,0,0]));j.CreateLocalRot0Attr(quat(restR if i==0 else previousR.T@restR));j.CreateLocalRot1Attr(quat(np.eye(3)));j.CreateLowerLimitAttr(-150.);j.CreateUpperLimitAttr(150.);j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(False)
  drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(.20 if i==0 else .12);drive.CreateDampingAttr(.002);drive.CreateMaxForceAttr(.015);drive.CreateTargetPositionAttr(0.)
  previousR=restR.copy()
  flapposes.append(np.r_[pos,Rotation.from_matrix(initialR).as_quat()].tolist())
 UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(lp)).CreateFilteredPairsRel().SetTargets([Sdf.Path(flap[0])])
 CREATED.update(pull_tab_local=[0,0,0],pull_tab_segment=9,pull_tab_model='wide grasp region, no loop',rest_curve_degrees=[5,35],strap_length_m=N*L,strap_segment_length_m=L,strap_root_local=rootlocal.tolist(),landing_centres_local=patch.tolist(),landing_normals_local=patchnorm)
 CREATED.update(support_type='mushroom peg',peg_shaft_radius_m=.010,peg_shaft_length_m=.040,peg_cap_radius_m=.018,adhesive_enabled=False,hook_segments=list(range(N)),hook_face_local_normal=[0,0,1],loop_face_hook_local=[.0355,0,0],loop_normal_hook_local=[1,0,0],initialization_note='Settled cable state and elastic open strap.')
 CREATED.update(links=links,cable=g,cable_poses=np.column_stack([centers,q]).tolist(),flap_poses=flapposes,root=root,port_local=port.tolist(),strain_relief_local=guide.tolist(),secured_tail_length_m=tail_length,total_cable_length_m=.74,elastic_model=dict(rest_shape="initial smooth arc",bend_stiffness_Nm_per_degree=.008,bend_damping_Nm_per_degree_per_second=.0002,bend_limit_degrees=15,twist_limit_degrees=30))

 # Initialize from physically settled poses. Keep original joints/rest angles;
 # do not freeze bodies or re-anchor them during the episode.
 restfile=Path(__file__).with_name('rest_pose.json')
 if restfile.exists():
  rest=json.loads(restfile.read_text())
  if rest['max_2s_displacement_m']>=.002:raise ValueError('Rest snapshot still moving')
  for stem,key in [('service_cable','cable')]:
   saved=np.array(rest[key])
   for i,row in enumerate(saved):
    prim=stage.GetPrimAtPath(f'/World/envs/env_0/{stem}/segment_{i:03d}')
    prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*map(float,row[:3])))
    qq=row[3:];prim.GetAttribute('xformOp:orient').Set(Gf.Quatf(float(qq[3]),vec(qq[:3])))
   CREATED['flap_poses' if key=='flap' else 'cable_poses']=saved.tolist()
  CREATED['rest_snapshot_time_s']=rest['time_s']

def install():
 import yam_arena.embodiment as em
 from isaaclab.sim.utils.stage import get_current_stage
 from pxr import UsdPhysics
 orig=em.yam_arm_cfg
 def arm(*args,**kwargs):
  cfg=orig(*args,**kwargs);cfg.spawn.make_uninstanceable=True
  if 'RightArm' in cfg.prim_path:
   from yam_arena.constants import ARM_JOINTS
   snap=json.loads(Path(__file__).with_name('initial_robot_state.json').read_text())
   cfg.init_state.joint_pos.update(dict(zip(ARM_JOINTS,snap['right_actual_q'])))
  if 'RightArm' in cfg.prim_path:
   sp=cfg.spawn.func
   def spawn(*a,**kw):
    prim=sp(*a,**kw);stage=get_current_stage();paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and 'LeftArm' in str(p.GetPath())];links={n:next(x for x in paths if x.endswith('/'+n)) for n in ['link2','link3','gripper']};build(stage,links);return prim
   cfg.spawn.func=spawn
  return cfg
 em.yam_arm_cfg=arm
