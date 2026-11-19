"""Wet, independently flexible hair locks and a separate microfiber towel."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
HEAD=np.array([.49,0,TABLE+.305]);ELASTIC=np.array([.27,0,TABLE+.063])
head=[sphere((.090,.078,.115)),sphere((.015,.012,.023),(.091,0,-.015)),cylinder(.038,.19,(0,0,-.205)),box((.15,.15,.015),(0,0,-.2975))]
head += [sphere((.004,.008,.005),(.084,y,.020),(.12,.12,.13)) for y in [-.030,.030]]
BODIES.append(body('styling_head',HEAD,head,fixed=True,mass=1.5,color=(.75,.62,.50),friction=.5))
STRANDS=12;SEGMENTS=18
# Each flexible lock groups fine fibres into a 5 mm contact envelope.
# The 36 g total mass includes voids between fibres; no locks are joined together.
for s in range(STRANDS):
 y=(s-5.5)*.0065;z=.035+.015*np.cos(s);back=-.0915*np.sqrt(1-(y/.078)**2-(z/.115)**2);root=HEAD+[back,y,z]
 pts=[]
 for j in range(SEGMENTS+1):
  u=j/SEGMENTS;p=root+[-.035*u+.004*np.sin(2*np.pi*u+s*.25),.002*np.sin(3*np.pi*u+s),-.245*u]
  local=p-HEAD
  if (local[1]/.078)**2+(local[2]/.115)**2<1:
   scalp=HEAD[0]-.090*np.sqrt(1-(local[1]/.078)**2-(local[2]/.115)**2)
   p[0]=min(p[0],scalp-.003)
  pts.append(p)
 for i,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
  a,b=np.array(a),np.array(b);v=b-a;length=np.linalg.norm(v);rot=Rotation.align_vectors([v/length],[[0,0,1]])[0];name=f'hair_{s}_{i}'
  part=body(name,(a+b)/2,[cylinder(.0025,length)],mass=.0045/SEGMENTS,quat=rot.as_quat().tolist(),color=(.20+.015*(s%3),.095,.040),friction=.5,contact_offset=.0001,linear_damping=.2,angular_damping=.3)
  I=part['mass']*length*length/12;part['inertia']=[max(I,1e-10),max(I,1e-10),max(I/10,1e-10)];part['visible']=False;BODIES.append(part)
  if i:
   prev=next(b for b in BODIES if b['name']==f'hair_{s}_{i-1}')
   axes={axis:dict(limits=(-180,180),stiffness=.000015,damping=.000002) for axis in ('rotX','rotY','rotZ')}
   JOINTS.append(joint(f'hair_join_{s}_{i}',prev['name'],name,[0,0,prev['shapes'][0]['length']/2],[0,0,-length/2],axes=axes,break_force=15.))
  else:JOINTS.append(joint(f'root_{s}','styling_head',name,a-HEAD,[0,0,-length/2],axes=bend_axes(.0002,.00002),break_force=20.))

# 180 x 240 mm microfiber cloth starts held open by its two upper corners.
TOWEL=np.array([.32,0,TABLE+.305]);NX=13;NY=10
POINTS=np.array([TOWEL+[0,y,-x] for x in np.linspace(0,.24,NX) for y in np.linspace(-.09,.09,NY)])
TRIS=[]
for i in range(NX-1):
 for j in range(NY-1):
  a=i*NY+j;b=a+1;c=a+NY;d=c+1;TRIS.extend([[a,b,d],[a,d,c]])
TRIS=np.asarray(TRIS);EDGES=np.unique(np.sort(np.concatenate([TRIS[:,[0,1]],TRIS[:,[1,2]],TRIS[:,[2,0]]]),axis=1),axis=0)
REST_LENGTHS=np.linalg.norm(POINTS[EDGES[:,1]]-POINTS[EDGES[:,0]],axis=1)
GRIP_IDS=[NY-1,0];PATCHES=[]
for i,node in enumerate(GRIP_IDS):
 name=f'cloth_contact_{i}';PATCHES.append((name,node));BODIES.append(body(name,POINTS[node],[sphere((.0015,)*3)],mass=.0001,color=(.22,.55,.66),friction=.9,contact_offset=.00025,exclude=['towel']))
finalize(BODIES,JOINTS)

def spawn_surface(stage,root):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf,Vt,Sdf
 mesh=UsdGeom.Mesh.Define(stage,root+'/towel');mesh.CreatePointsAttr(Vt.Vec3fArray(POINTS.astype(np.float32).tolist()));mesh.CreateFaceVertexCountsAttr([3]*len(TRIS));mesh.CreateFaceVertexIndicesAttr(TRIS.ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
 p=mesh.GetPrim()
 for api in ['OmniPhysicsDeformableBodyAPI','OmniPhysicsSurfaceDeformableSimAPI','PhysxSurfaceDeformableBodyAPI']:
  if not p.ApplyAPI(api):raise RuntimeError('Required native surface API unavailable: '+api)
 p.GetAttribute('omniphysics:mass').Set(.0128)
 p.GetAttribute('omniphysics:restShapePoints').Set(Vt.Vec3fArray(POINTS.astype(np.float32).tolist()))
 p.GetAttribute('omniphysics:restTriVtxIndices').Set(Vt.Vec3iArray(TRIS.tolist()))
 p.GetAttribute('omniphysics:restBendAnglesDefault').Set('restShapeDefault')
 p.GetAttribute('physxDeformableBody:selfCollision').Set(True)
 p.GetAttribute('physxDeformableBody:solverPositionIterationCount').Set(64)
 p.GetAttribute('physxDeformableBody:enableSpeculativeCCD').Set(True)
 p.GetAttribute('physxDeformableBody:linearDamping').Set(.1)
 UsdPhysics.CollisionAPI.Apply(p);col=PhysxSchema.PhysxCollisionAPI.Apply(p);col.CreateContactOffsetAttr(.004);col.CreateRestOffsetAttr(.0004)
 mat=UsdShade.Material.Define(stage,root+'/towelMaterial');q=mat.GetPrim()
 for api in ['OmniPhysicsBaseMaterialAPI','OmniPhysicsDeformableMaterialAPI','OmniPhysicsSurfaceDeformableMaterialAPI','PhysxSurfaceDeformableMaterialAPI']:q.ApplyAPI(api)
 for key,value in {'omniphysics:dynamicFriction':.6,'omniphysics:density':1100.,'omniphysics:youngsModulus':2e6,'omniphysics:poissonsRatio':.3,'omniphysics:surfaceThickness':.0008,'omniphysics:surfaceBendStiffness':12000.,'physxDeformableMaterial:elasticityDamping':.02,'physxDeformableMaterial:bendDamping':.01}.items():
  attr=q.GetAttribute(key)
  if not attr:raise RuntimeError('Required cap material attribute unavailable: '+key)
  attr.Set(value)
 shader=UsdShade.Shader.Define(stage,root+'/towelMaterial/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.22,.55,.66));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
 bind=UsdShade.MaterialBindingAPI.Apply(p);bind.Bind(mat);bind.Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')


 for name,node in PATCHES:
  a=stage.DefinePrim(root+'/hem_'+name,'OmniPhysicsVtxXformAttachment')
  a.GetRelationship('omniphysics:src0').SetTargets([Sdf.Path(root+'/towel')]);a.GetRelationship('omniphysics:src1').SetTargets([Sdf.Path(root+'/'+name)])
  a.GetAttribute('omniphysics:vtxIndicesSrc0').Set([int(node)]);a.GetAttribute('omniphysics:localPositionsSrc1').Set(Vt.Vec3fArray([[0.,0.,0.]]))
