"""Loose synthetic hair and a small, plain elastic; no hair is pre-tied."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
ELASTIC_YOUNGS_MODULUS=6e5
HEAD=np.array([.39,0,TABLE+.305]);ELASTIC=np.array([.27,0,TABLE+.063])
head=[sphere((.090,.078,.115)),sphere((.015,.012,.023),(.091,0,-.015)),cylinder(.038,.19,(0,0,-.205)),box((.15,.15,.015),(0,0,-.2975))]
# A small foam-backed root patch supports the separately rooted section.
head += [box((.024,.040,.014),(-.098,0,.042),(.20,.095,.040))]
head += [sphere((.004,.008,.005),(.084,y,.020),(.12,.12,.13)) for y in [-.030,.030]]
BODIES.append(body('styling_head',HEAD,head,fixed=True,mass=1.5,color=(.75,.62,.50),friction=.5))
# A narrow native elastic strip provides continuous hair contact during recoil.
# Its 1.5 mm width plus 0.75 mm rest offset per side stays inside the
# rendered 3 mm cord envelope. Capsules also resolve round-cord contact.
STRANDS=12;SEGMENTS=18
# Each flexible lock groups fine fibres into a 5 mm contact envelope.
# The 36 g total mass includes voids between fibres; no locks are joined together.
for s in range(STRANDS):
 # A compact, untied section: twelve separately rooted locks in one straight
 # two-row section. This tests fastening, not gathering a whole head.
 y=(s%6-2.5)*.0054;back=-.101+(s//6-.5)*.0054
 root=HEAD+[back,y,.035]
 pts=[]
 for j in range(SEGMENTS+1):
  u=j/SEGMENTS;p=root+[0.,0.,-.245*u]
  local=p-HEAD
  if (local[1]/.078)**2+(local[2]/.115)**2<1:
   scalp=HEAD[0]-.090*np.sqrt(1-(local[1]/.078)**2-(local[2]/.115)**2)
   p[0]=min(p[0],scalp-.003)
  pts.append(p)
 for i,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
  a,b=np.array(a),np.array(b);v=b-a;length=np.linalg.norm(v);rot=Rotation.align_vectors([v/length],[[0,0,1]])[0];name=f'hair_{s}_{i}'
  part=body(name,(a+b)/2,[cylinder(.0025,length)],mass=.003/SEGMENTS,quat=rot.as_quat().tolist(),color=(.20+.015*(s%3),.095,.040),friction=.5,contact_offset=.0001,speculative_ccd=True,max_depenetration_velocity=1.,linear_damping=.2,angular_damping=.3)
  # Rounded overlapping ends keep a continuous contact envelope at hinges.
  part['shapes'][0]['kind']='capsule'
  I=part['mass']*length*length/12;part['inertia']=[max(I,1e-10),max(I,1e-10),max(I/10,1e-10)];part['visible']=False;BODIES.append(part)
  if i:
   prev=next(b for b in BODIES if b['name']==f'hair_{s}_{i-1}')
   axes={axis:dict(limits=(-180,180),stiffness=.000015,damping=.000002) for axis in ('rotX','rotY','rotZ')}
   JOINTS.append(joint(f'hair_join_{s}_{i}',prev['name'],name,[0,0,prev['shapes'][0]['length']/2],[0,0,-length/2],axes=axes,break_force=15.))
  else:JOINTS.append(joint(f'root_{s}','styling_head',name,a-HEAD,[0,0,-length/2],axes=bend_axes(.0002,.00002),break_force=20.))
# Two open-top saddles support the loose band without attaching it.
for sx in [-1,1]:
 BODIES.append(body(f'elastic_rest_{sx+1}',[ELASTIC[0]+sx*.0095,0,TABLE+.030],[box((.007,.006,.058),(0,0,-.001)),box((.0015,.007,.004),(-.00275,0,.030)),box((.0015,.007,.004),(.00275,0,.030)),box((.004,.0015,.004),(0,-.00275,.030)),box((.004,.0015,.004),(0,.00275,.030))],mass=.010,fixed=True,color=(.59,.44,.28)))
# A plain 3 mm cord elastic, 16 mm inside diameter. Native passive
# a continuous elastic surface joins contact-bearing patches into a closed loop.
M=32;radius=.0095;theta=np.arange(M)*2*np.pi/M
vertices=ELASTIC+np.c_[radius*np.cos(theta),radius*np.sin(theta),np.zeros(M)]
centres=(vertices+np.roll(vertices,-1,axis=0))/2
lengths=np.linalg.norm(np.roll(vertices,-1,axis=0)-vertices,axis=1)
BAND_CONTACT_INDICES=list(range(M))
for i in BAND_CONTACT_INDICES:
 tangent=(vertices[(i+1)%M]-vertices[i])/lengths[i];radial=centres[i]-ELASTIC;radial/=np.linalg.norm(radial);rot=Rotation.from_matrix(np.column_stack([radial,np.cross(tangent,radial),tangent]))
 shape=dict(kind='capsule',radius=.0015,length=float(lengths[i]),pos=[0,0,0],quat=[0,0,0,1])
 BODIES.append(body(f'band_segment_{i:03d}',centres[i],[shape],mass=.0004/len(BAND_CONTACT_INDICES),quat=rot.as_quat().tolist(),color=(.64,.12,.27),friction=1.2,contact_offset=.0002,speculative_ccd=True,max_depenetration_velocity=1.,torsional_patch_radius=.0015,min_torsional_patch_radius=.001,exclude=['elastic']+[f'band_segment_{(i+d)%M:03d}' for d in [-3,-2,-1,1,2,3]],visible=False,external_force=True))
radial=centres-ELASTIC;radial/=np.linalg.norm(radial,axis=1)[:,None]
POINTS=np.vstack([centres+offset*radial for offset in [-.00075,0,.00075]])
TRIS=[]
for row in range(2):
 for i in range(M):
  a=row*M+i;b=row*M+(i+1)%M;c=a+M;d=b+M;TRIS.extend([[a,b,c],[b,d,c]])
TRIS=np.asarray(TRIS);RIM=np.arange(M)
EDGES=np.unique(np.sort(np.concatenate([TRIS[:,[0,1]],TRIS[:,[1,2]],TRIS[:,[2,0]]]),axis=1),axis=0)
REST_LENGTHS=np.linalg.norm(POINTS[EDGES[:,1]]-POINTS[EDGES[:,0]],axis=1)
BAND_ATTACHMENTS=[]
for i in BAND_CONTACT_INDICES:
 part=next(b for b in BODIES if b['name']==f'band_segment_{i:03d}')
 ids=[M+i]
 local=Rotation.from_quat(part['quat']).inv().apply(POINTS[ids]-part['pos'])
 BAND_ATTACHMENTS.append((part['name'],ids,local))
finalize(BODIES,JOINTS)

def spawn_surface(stage,root):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf,Vt,Sdf
 mesh=UsdGeom.Mesh.Define(stage,root+'/elastic');mesh.CreatePointsAttr(Vt.Vec3fArray(POINTS.astype(np.float32).tolist()));mesh.CreateFaceVertexCountsAttr([3]*len(TRIS));mesh.CreateFaceVertexIndicesAttr(TRIS.ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
 p=mesh.GetPrim()
 for api in ['OmniPhysicsDeformableBodyAPI','OmniPhysicsSurfaceDeformableSimAPI','PhysxSurfaceDeformableBodyAPI']:
  if not p.ApplyAPI(api):raise RuntimeError('Required native surface API unavailable: '+api)
 p.GetAttribute('omniphysics:mass').Set(.0006)
 p.GetAttribute('omniphysics:restShapePoints').Set(Vt.Vec3fArray(POINTS.astype(np.float32).tolist()))
 p.GetAttribute('omniphysics:restTriVtxIndices').Set(Vt.Vec3iArray(TRIS.tolist()))
 p.GetAttribute('physxDeformableBody:selfCollision').Set(True)
 p.GetAttribute('physxDeformableBody:solverPositionIterationCount').Set(64)
 p.GetAttribute('physxDeformableBody:enableSpeculativeCCD').Set(True)
 p.GetAttribute('physxDeformableBody:linearDamping').Set(.1)
 UsdPhysics.CollisionAPI.Apply(p);col=PhysxSchema.PhysxCollisionAPI.Apply(p);col.CreateContactOffsetAttr(.00125);col.CreateRestOffsetAttr(.00075)
 mat=UsdShade.Material.Define(stage,root+'/elasticMaterial');q=mat.GetPrim()
 for api in ['OmniPhysicsBaseMaterialAPI','OmniPhysicsDeformableMaterialAPI','OmniPhysicsSurfaceDeformableMaterialAPI','PhysxSurfaceDeformableMaterialAPI']:q.ApplyAPI(api)
 for key,value in {'omniphysics:dynamicFriction':.9,'omniphysics:density':1100.,'omniphysics:youngsModulus':2*ELASTIC_YOUNGS_MODULUS,'omniphysics:poissonsRatio':.45,'omniphysics:surfaceThickness':.003,'omniphysics:surfaceBendStiffness':2*ELASTIC_YOUNGS_MODULUS/(12*(1-.45**2)),'physxDeformableMaterial:elasticityDamping':.15,'physxDeformableMaterial:bendDamping':.01}.items():
  attr=q.GetAttribute(key)
  if not attr:raise RuntimeError('Required elastic material attribute unavailable: '+key)
  attr.Set(value)
 shader=UsdShade.Shader.Define(stage,root+'/elasticMaterial/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.64,.12,.27));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
 bind=UsdShade.MaterialBindingAPI.Apply(p);bind.Bind(mat);bind.Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')

 for name,ids,local in BAND_ATTACHMENTS:
  attachment=stage.DefinePrim(root+'/elastic_contact_'+name,'OmniPhysicsVtxXformAttachment')
  attachment.GetRelationship('omniphysics:src0').SetTargets([Sdf.Path(root+'/elastic')])
  attachment.GetRelationship('omniphysics:src1').SetTargets([Sdf.Path(root+'/'+name)])
  attachment.GetAttribute('omniphysics:vtxIndicesSrc0').Set(ids)
  attachment.GetAttribute('omniphysics:localPositionsSrc1').Set(Vt.Vec3fArray(local.astype(np.float32).tolist()))
