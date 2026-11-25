"""The same printable 260x180 mm chart is present from the task's first frame."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
def geometry():
 from yam_arena import station,YAM_URDF
 from yam_arena.constants import READY,ARM_JOINTS
 from yam_arena.urdf import forward_kinematics
 T=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,READY)))['gripper'];T[:3,3]+=np.array(station.rig().bases['left']);cam=station.wrist_camera('left');C=T[:3,3]+T[:3,:3]@cam.pos;R=T[:3,:3]@Rotation.from_quat(cam.rot_xyzw).as_matrix();fwd=-R[:,2]
 ex=R[:,0].copy();ex[2]=0;ex/=np.linalg.norm(ex);ey=np.cross([0,0,1],ex)
 if ey@R[:,1]<0:ey=-ey
 z=station.TABLE_TOP+.003;centre=C+fwd*((z-C[2])/fwd[2]);return centre,ex,ey,.260,.180

def build_card(stage):
 from pxr import UsdGeom,UsdShade,UsdPhysics,Gf,Sdf
 from yam_arena import station
 centre,ex,ey,W,H=geometry();corners=[centre+ex*x+ey*y for x,y in [(-W/2,-H/2),(W/2,-H/2),(W/2,H/2),(-W/2,H/2)]]
 path='/World/envs/env_0/image_quality_test_card';card=UsdGeom.Mesh.Define(stage,path);card.CreatePointsAttr([tuple(map(float,c)) for c in corners]);card.CreateFaceVertexCountsAttr([4]);card.CreateFaceVertexIndicesAttr([0,1,2,3]);card.CreateDoubleSidedAttr(True);card.CreateSubdivisionSchemeAttr('none');uv=UsdGeom.PrimvarsAPI(card).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex);uv.Set([(0,0),(1,0),(1,1),(0,1)])
 # Thin static backing resting on the tabletop; no contact with the scripted brush path.
 backing=UsdGeom.Cube.Define(stage,'/World/envs/env_0/test_card_backing');backing.CreateSizeAttr(1.);backing.AddTranslateOp().Set(Gf.Vec3d(float(centre[0]),float(centre[1]),station.TABLE_TOP+.0014));backing.AddScaleOp().Set(Gf.Vec3f(H,W,.0028));backing.CreateDisplayColorAttr([(.7,.7,.7)]);UsdPhysics.CollisionAPI.Apply(backing.GetPrim())
 mat=UsdShade.Material.Define(stage,'/World/TestCardMaterial');sh=UsdShade.Shader.Define(stage,'/World/TestCardMaterial/shader');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.);sh.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set((0.,0.,0.));sh.CreateInput('useSpecularWorkflow',Sdf.ValueTypeNames.Int).Set(1)
 tex=UsdShade.Shader.Define(stage,'/World/TestCardMaterial/texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(Path(__file__).with_name('test_chart.png').resolve()));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');reader=UsdShade.Shader.Define(stage,'/World/TestCardMaterial/uv');reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(card.GetPrim()).Bind(mat)
 return {'centre_world':centre.tolist(),'width_m':W,'height_m':H,'corners_world':[p.tolist() for p in corners]}
