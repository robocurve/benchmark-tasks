"""Native-camera-aligned protective window and surface contamination, no image compositing."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
DIRT=[]
WINDOW_BODY='/World/envs/env_0/camera_window_body'
def build(stage,gp,measurement=False):
 from pxr import UsdGeom,UsdShade,UsdPhysics,PhysxSchema,Gf,Sdf
 from yam_arena import station
 from camera_mount import MOUNT,SHOE
 layout=json.loads(Path(__file__).with_name('camera_mount.json').read_text());cp=np.array(layout['mount_position_in_gripper']);CR=np.array(layout['mount_rotation_in_gripper']);cam=station.wrist_camera('left');C=np.array(cam.pos);R=Rotation.from_quat(cam.rot_xyzw).as_matrix()
 def quat(rot):
  q=Rotation.from_matrix(rot).as_quat();return Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3])))
 def box(name,pos,sz,color,rot=CR,collision=False):
  o=UsdGeom.Cube.Define(stage,gp+'/'+name.replace('-','m'));o.CreateSizeAttr(1);o.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));o.AddOrientOp().Set(quat(rot));o.AddScaleOp().Set(Gf.Vec3f(*map(float,sz)));o.CreateDisplayColorAttr([color])
  if collision:
   UsdPhysics.CollisionAPI.Apply(o.GetPrim());pc=PhysxSchema.PhysxCollisionAPI.Apply(o.GetPrim());pc.CreateContactOffsetAttr(.0001);pc.CreateRestOffsetAttr(0.)
  return o
 box('camera_housing',cp,(.04,.036,.028),(.08,.10,.13),collision=True)
 for name,p,sz in MOUNT+SHOE:box('mount_'+name,cp+CR@p,sz,(.18,.32,.48),collision=True)
 # Complete wrist-to-rail mounting bridge; camera pose and optical axes unchanged.
 box('wrist_mount_base',[0,-.027,.025],(.038,.020,.006),(.18,.27,.34),np.eye(3),True)
 for side in [-1,1]:
  a=np.array([side*.013,-.030,.025]);b=cp+CR@np.array([side*.0122,-.0282,-.012])
  zz=(b-a)/np.linalg.norm(b-a);xx=np.array([1.,0,0]);xx-=zz*(xx@zz);xx/=np.linalg.norm(xx);yy=np.cross(zz,xx)
  box('wrist_to_rail_support_'+str(side),(a+b)/2,(.006,.006,float(np.linalg.norm(b-a))+.004),(.18,.32,.48),np.column_stack([xx,yy,zz]),True)
 # Visible fastener heads on the gripper-side mounting plate.
 for side in [-1,1]:
  box('mount_fastener_'+str(side),[side*.011,-.024,.029],(.0045,.0045,.002),(.48,.51,.55),np.eye(3))
 # Exactly native optical centre/axes; forward is OpenGL -Z.
 d=.003;W=.014;H=.012
 for x in [-1,1]:box('window_rim_x'+str(x),C+R@[x*(W/2+.0006),0,-(d-.0006)],(.0012,H+.0024,.0003),(.2,.24,.27),R,True)
 for y in [-1,1]:box('window_rim_y'+str(y),C+R@[0,y*(H/2+.0006),-(d-.0006)],(W,.0012,.0003),(.2,.24,.27),R,True)
 def material(name,alpha):
  m=UsdShade.Material.Define(stage,'/World/'+name);sh=UsdShade.Shader.Define(stage,'/World/'+name+'/shader');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set((.24,.15,.075));sh.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(float(alpha));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.95);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m,sh
 # The retaining bezel sits 0.6 mm behind the glass face, leaving it unobstructed.
 # Separate rigid body and fixed wrist joint provide pane-specific contact forces.
 from yam_arena import YAM_URDF
 from yam_arena.constants import ARM_JOINTS,READY
 from yam_arena.urdf import forward_kinematics
 q=READY if measurement else json.loads(Path(__file__).with_name('initial_state.json').read_text())['left_q']
 T=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,q)))['gripper'];T[:3,3]+=np.array(station.rig().bases['left'])
 local=C+R@np.array([0,0,-d]);world=T[:3,3]+T[:3,:3]@local;worldR=T[:3,:3]@R
 root=UsdGeom.Xform.Define(stage,WINDOW_BODY);root.AddTranslateOp().Set(Gf.Vec3d(*map(float,world)));root.AddOrientOp().Set(quat(worldR));UsdPhysics.RigidBodyAPI.Apply(root.GetPrim());UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(.001)
 rb=PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim());rb.CreateSolverPositionIterationCountAttr(128);rb.CreateSolverVelocityIterationCountAttr(16);PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.)
 pane=UsdGeom.Cube.Define(stage,WINDOW_BODY+'/glass');pane.CreateSizeAttr(1.);pane.AddScaleOp().Set(Gf.Vec3f(W,H,.0003));UsdPhysics.CollisionAPI.Apply(pane.GetPrim());pc=PhysxSchema.PhysxCollisionAPI.Apply(pane.GetPrim());pc.CreateContactOffsetAttr(.0001);pc.CreateRestOffsetAttr(0.);pane.GetVisibilityAttr().Set('invisible')
 fixed=UsdPhysics.FixedJoint.Define(stage,'/World/envs/env_0/camera_window_mount');fixed.CreateBody0Rel().SetTargets([Sdf.Path(gp)]);fixed.CreateBody1Rel().SetTargets([Sdf.Path(WINDOW_BODY)]);fixed.CreateLocalPos0Attr(Gf.Vec3f(*map(float,local)));fixed.CreateLocalRot0Attr(quat(R));fixed.CreateLocalPos1Attr(Gf.Vec3f(0));fixed.CreateLocalRot1Attr(Gf.Quatf(1));fixed.CreateCollisionEnabledAttr(False);fixed.CreateExcludeFromArticulationAttr(True)
 clear,sh=material('clear_window',0.);UsdShade.MaterialBindingAPI.Apply(pane.GetPrim()).Bind(clear)
 # One continuous RGBA texture on the physical window surface.
 mesh=UsdGeom.Mesh.Define(stage,gp+'/surface_contamination')
 points=[C+R@[x,y,-d-.00017] for x,y in [(-W/2,-H/2),(W/2,-H/2),(W/2,H/2),(-W/2,H/2)]]
 mesh.CreatePointsAttr([tuple(map(float,p)) for p in points]);mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateDoubleSidedAttr(True)
 st=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex);st.Set([(0,0),(1,0),(1,1),(0,1)])
 m,shader=material('surface_dirt',1.)
 shader.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.)
 shader.CreateInput('useSpecularWorkflow',Sdf.ValueTypeNames.Int).Set(1)
 shader.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set((0.,0.,0.))
 tex=UsdShade.Shader.Define(stage,'/World/surface_dirt/texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(Path(__file__).with_name('dirty.png')));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('raw')
 reader=UsdShade.Shader.Define(stage,'/World/surface_dirt/uv');reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result')
 shader.GetInput('opacity').ConnectToSource(tex.ConnectableAPI(),'a');shader.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(m)
 DIRT.append((mesh,shader,tex))
 return dict(native_optical_centre=C.tolist(),window_forward_distance_m=d,window_size_m=[W,H],continuous_texture=True)

def install(measurement=False):
 import yam_arena.embodiment as em
 from isaaclab.sim.utils.stage import get_current_stage
 from pxr import UsdPhysics
 original=em.yam_arm_cfg
 def arm(*args,**kwargs):
  cfg=original(*args,**kwargs);cfg.spawn.make_uninstanceable=True
  if 'LeftArm' in cfg.prim_path:
   from yam_arena.constants import ARM_JOINTS
   from yam_arena.constants import READY
   q=READY if measurement else json.loads(Path(__file__).with_name('initial_state.json').read_text())['left_q'];cfg.init_state.joint_pos.update(dict(zip(ARM_JOINTS,q)))
  if 'RightArm' in cfg.prim_path:
   spawn=cfg.spawn.func
   def do(*a,**kw):
    prim=spawn(*a,**kw);stage=get_current_stage();gp=next(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and 'LeftArm' in str(p.GetPath()) and str(p.GetPath()).endswith('/gripper'));build(stage,gp,measurement=measurement)
    from pxr import UsdGeom,Gf,PhysxSchema
    right=next(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and 'RightArm' in str(p.GetPath()) and str(p.GetPath()).endswith('/gripper'))
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(right)).CreateThresholdAttr(0.)
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(gp)).CreateThresholdAttr(0.)
    # A free brush in a small tabletop holder; no attachment to the hand.
    from brush_geometry import build_brush
    build_brush(stage)
    from test_card import build_card
    build_card(stage)
    return prim
   cfg.spawn.func=do
  return cfg
 em.yam_arm_cfg=arm
