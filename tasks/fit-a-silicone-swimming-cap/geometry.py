"""Adult practice head and a separate thin elastic cap; SI units."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
HEAD=np.array([.36,0,TABLE+.125]);HEADROT=np.eye(3);RADII=np.array([.095,.080,.120]);CAP=HEAD+np.array([0.,0.,.105])
shapes=[sphere(RADII),sphere((.018,.012,.026),(-.095,0,-.015)),sphere((.012,.012,.026),(0,-.080,-.025)),sphere((.012,.012,.026),(0,.080,-.025)),box((.14,.13,.010),(0,0,-.120))]
shapes += [sphere((.003,.009,.005),(-.089,y,.017),(.13,.15,.18)) for y in [-.030,.030]]
shapes += [box((.001,.038,.004),(-.090,0,-.051),(.47,.17,.17))]
BODIES.append(body('headform',HEAD,shapes,fixed=True,mass=1.5,color=(.76,.63,.51),friction=.3,quat=Rotation.from_matrix(HEADROT).as_quat().tolist()))
# The loose cap straddles the crown, with its held rim15mm below the crown; it is not stretched into its final fit.
from cap_mesh import hemisphere
unit_points,TRIS,RIM=hemisphere()
M=len(RIM)
POINTS=CAP+unit_points*np.array([.095,.075,.090])
POINTS=HEAD+(POINTS-HEAD)@HEADROT.T
EDGES=np.unique(np.sort(np.concatenate([TRIS[:,[0,1]],TRIS[:,[1,2]],TRIS[:,[2,0]]]),axis=1),axis=0)
REST_LENGTHS=np.linalg.norm(POINTS[EDGES[:,1]]-POINTS[EDGES[:,0]],axis=1)
# Discrete native contact points represent the cap's 3 mm rolled rim.
# Each point couples to one elastic vertex, so the opening can still stretch.
for i,node in enumerate(RIM):
 BODIES.append(body(f'rim_contact_{i:03d}',POINTS[node],[sphere((.0015,)*3)],mass=.004/M,color=(.10,.44,.77),friction=.9,contact_offset=.00025,exclude=['cap']))
for side,y in [('left',.305),('right',-.305)]:
 BODIES.append(body(side+'_arm_riser',[0,y,TABLE+.05],[box((.16,.16,.10))],fixed=True,mass=3.,color=(.32,.34,.36)))
finalize(BODIES,JOINTS)

def spawn_surface(stage,root):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf,Vt,Sdf
 mesh=UsdGeom.Mesh.Define(stage,root+'/cap');mesh.CreatePointsAttr(Vt.Vec3fArray(POINTS.astype(np.float32).tolist()));mesh.CreateFaceVertexCountsAttr([3]*len(TRIS));mesh.CreateFaceVertexIndicesAttr(TRIS.ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
 p=mesh.GetPrim()
 for api in ['OmniPhysicsDeformableBodyAPI','OmniPhysicsSurfaceDeformableSimAPI','PhysxSurfaceDeformableBodyAPI']:
  if not p.ApplyAPI(api):raise RuntimeError('Required native surface API unavailable: '+api)
 p.GetAttribute('omniphysics:mass').Set(.040)
 p.GetAttribute('omniphysics:restShapePoints').Set(Vt.Vec3fArray(POINTS.astype(np.float32).tolist()))
 p.GetAttribute('omniphysics:restTriVtxIndices').Set(Vt.Vec3iArray(TRIS.tolist()))
 p.GetAttribute('omniphysics:restBendAnglesDefault').Set('restShapeDefault')
 p.GetAttribute('physxDeformableBody:selfCollision').Set(True)
 p.GetAttribute('physxDeformableBody:maxDepenetrationVelocity').Set(.5)
 p.GetAttribute('physxDeformableBody:solverPositionIterationCount').Set(128)
 p.GetAttribute('physxDeformableBody:enableSpeculativeCCD').Set(True)
 p.GetAttribute('physxDeformableBody:linearDamping').Set(.1)
 UsdPhysics.CollisionAPI.Apply(p);col=PhysxSchema.PhysxCollisionAPI.Apply(p);col.CreateContactOffsetAttr(.001);col.CreateRestOffsetAttr(.0004)
 mat=UsdShade.Material.Define(stage,root+'/capMaterial');q=mat.GetPrim()
 for api in ['OmniPhysicsBaseMaterialAPI','OmniPhysicsDeformableMaterialAPI','OmniPhysicsSurfaceDeformableMaterialAPI','PhysxSurfaceDeformableMaterialAPI']:q.ApplyAPI(api)
 for key,value in {'omniphysics:dynamicFriction':.35,'omniphysics:density':1100.,'omniphysics:youngsModulus':1e6,'omniphysics:poissonsRatio':.45,'omniphysics:surfaceThickness':.0008,'omniphysics:surfaceBendStiffness':1e6/(12*(1-.45**2)),'physxDeformableMaterial:elasticityDamping':.02,'physxDeformableMaterial:bendDamping':.01}.items():
  attr=q.GetAttribute(key)
  if not attr:raise RuntimeError('Required cap material attribute unavailable: '+key)
  attr.Set(value)
 shader=UsdShade.Shader.Define(stage,root+'/capMaterial/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.10,.44,.77));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
 bind=UsdShade.MaterialBindingAPI.Apply(p);bind.Bind(mat);bind.Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')

 for i,node in enumerate(RIM):
  attach=stage.DefinePrim(root+f'/rolled_rim_{i:03d}','OmniPhysicsVtxXformAttachment')
  attach.GetRelationship('omniphysics:src0').SetTargets([Sdf.Path(root+'/cap')])
  attach.GetRelationship('omniphysics:src1').SetTargets([Sdf.Path(root+f'/rim_contact_{i:03d}')])
  attach.GetAttribute('omniphysics:vtxIndicesSrc0').Set([int(node)])
  attach.GetAttribute('omniphysics:localPositionsSrc1').Set(Vt.Vec3fArray([[0.,0.,0.]]))
