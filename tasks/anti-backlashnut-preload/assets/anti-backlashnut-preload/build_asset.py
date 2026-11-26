"""Original full-size USDA generator, pure Python; no simulator or external mesh.
Run this file in place. Ramp vertices use mm mesh coordinates and a reciprocal
.001 scale to retain their exact SI geometry during native convex cooking.
"""
from pathlib import Path
import json, math
ROOT=Path(__file__).resolve().parent
P=json.loads((ROOT/'parameters.json').read_text());U=P['published'];E=P['estimates']
out=[]
def emit(s):out.append(s)
def vec(v):return '('+', '.join(f'{x:.12g}'for x in v)+')'
def mat(n,c,mu,md):
 emit(f'''def Material "{n}" (prepend apiSchemas = ["PhysicsMaterialAPI"]) {{
 float physics:staticFriction = {mu}
 float physics:dynamicFriction = {md}
 float physics:restitution = 0
 token outputs:surface.connect = </assembly/Materials/{n}/shader.outputs:surface>
 def Shader "shader" {{ uniform token info:id = "UsdPreviewSurface"
 color3f inputs:diffuseColor = {vec(c)}
 float inputs:roughness = 0.5
 token outputs:surface
 }}
 }}''')
def body(n,pos,mass,kin=False):
 emit(f'''def Xform "{n}" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI"]) {{
 double3 xformOp:translate = {vec(pos)}
 uniform token[] xformOpOrder = ["xformOp:translate"]
 bool physics:kinematicEnabled = {str(kin).lower()}
 float physics:mass = {mass}
 float physxContactReport:threshold = 0
 bool physxRigidBody:enableCCD = true
 float physxRigidBody:linearDamping = 0.01
 float physxRigidBody:angularDamping = 0.001
 ''')
def mesh(n,vs,faces,m):
 # all faces are planar; each sector is its own convex solid, preserving hollow bores.
 flat=[i for f in faces for i in f]
 emit(f'''def Mesh "{n}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]) {{
 point3f[] points = [{', '.join(vec([x*1000 for x in v]) for v in vs)}]
 int[] faceVertexCounts = [{', '.join(str(len(f))for f in faces)}]
 int[] faceVertexIndices = [{', '.join(map(str,flat))}]
 uniform token subdivisionScheme = "none"
 token physics:approximation = "convexHull"
 float physxCollision:contactOffset = 0.00001
 float physxCollision:restOffset = 0
 double3 xformOp:scale = (0.001, 0.001, 0.001)
 uniform token[] xformOpOrder = ["xformOp:scale"]
 rel material:binding = </assembly/Materials/{m}>
 }}''')
F=[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
def box(n,size,pos,m):
 vs=[(pos[0]+x*size[0]/2,pos[1]+y*size[1]/2,pos[2]+z*size[2]/2)for z in [-1,1]for x,y in [(-1,-1),(1,-1),(1,1),(-1,1)]];mesh(n,vs,F,m)
def ring(n,ri,ro,z,h,m,N=32,centre=(0,0)):
 for i in range(N):
  a,b=2*math.pi*i/N,2*math.pi*(i+1)/N
  vs=[(centre[0]+r*math.cos(t),centre[1]+r*math.sin(t),zz)for zz in [z,z+h]for r,t in [(ri,a),(ro,a),(ro,b),(ri,b)]];mesh(n+'_'+str(i),vs,F,m)

def build():
 global out;out=[]
 emit('#usda 1.0\n(defaultPrim = "assembly"\n metersPerUnit = 1\n upAxis = "Z")\ndef Xform "assembly" {\ndef Scope "Materials" {')
 mat('steel',(.50,.54,.58),E['cam_static_friction'],E['cam_dynamic_friction']);mat('acetal',(.83,.83,.76),E['cam_static_friction'],E['cam_dynamic_friction']);mat('mount',(.18,.21,.24),.4,.3);mat('mark',(.8,.22,.035),.12,.10);emit('}')
 body('base',(0,0,0),.35,True)
 box('stand',(.09,.09,.09),(0,0,-.045),'mount')
 # A bolted actuator carriage/front nut, already engaged on a stationary screw.
 ring('flange',.0049,.01905,0,.00508,'acetal')
 ring('front_nut',.0049,.0082,.00508,.031,'acetal')
 ring('stationary_screw',.0001,.00436,-.04,.2032,'steel')
 # Thread crest is visual only. The native axial lost-motion joint is the declared
 # local thread-flank model; it must not be mistaken for actual helical meshing.
 for i in range(35):ring('crest'+str(i),.00436,.0047625,-.039+i*.00508,.0006,'steel',16)
 for i in range(3):
  a=i*2*math.pi/3;ring('mount_bolt'+str(i),.0001,.0022,.0001,.0052,'steel',12,centre=(.0143*math.cos(a),.0143*math.sin(a)))
  # Original bolt representations are omitted from the flange holes: flange
  # interface is a fixed mount, outside this already-installed-nut suboperation.
 box('reference_mark',(.0015,.003,.003),(.009,-.012,.032),'mark');emit('}')
 camtop=.00508+E['cam_length_m']
 body('cam',(0,0,0),E['cam_mass_kg'])
 ring('sleeve',.0096,.0104,.00508,E['cam_length_m'],'steel')
 # Three literal wedge ramps: rising helical faces, not a drive or angle trigger.
 period=2*math.pi/3;N=30
 for r in range(3):
  start=r*period-E['ramp_phase_fraction_at_reset']*period
  for i in range(N):
   a=start+i*period/N;b=start+(i+1)*period/N
   za=camtop+E['ramp_rise_m']*i/N;zb=camtop+E['ramp_rise_m']*(i+1)/N
   vs=[(rr*math.cos(t),rr*math.sin(t),z)for z in [camtop-.001]for rr,t in [(.0072,a),(.0104,a),(.0104,b),(.0072,b)]]
   vs += [(rr*math.cos(t),rr*math.sin(t),zz)for rr,t,zz in [(.0072,a,za),(.0104,a,za),(.0104,b,zb),(.0072,b,zb)]]
   mesh('ramp'+str(r)+'_'+str(i),vs,F,'steel')
 box('cam_mark',(.001,.001,.012),(.01045,0,.021),'mark');emit('}')
 rearinit=camtop+E['ramp_rise_m']*E['ramp_phase_fraction_at_reset'];rearzero=rearinit-E['rear_initial_q_m']
 body('rear',(0,0,rearinit),E['rear_mass_kg'])
 ring('threaded_rear',.0049,.0068,-.012,.015,'acetal')
 ring('rear_flange',.0049,.0104,.002,.004,'acetal')
 for i in range(3):
  a=2*math.pi*i/3;rr=E['ramp_radius_m'];c=(rr*math.cos(a),rr*math.sin(a),.001)
  # Axis-aligned short followers at the three contact azimuths; dimensions are
  # declared internal approximations, not the published external envelope.
  vs=[]
  for z in [0,.002]:
   for rad,tan in [(-.0013,-.0004),(.0013,-.0004),(.0013,.0004),(-.0013,.0004)]:vs.append(((rr+rad)*math.cos(a)-tan*math.sin(a),(rr+rad)*math.sin(a)+tan*math.cos(a),z))
  mesh('follower'+str(i),vs,F,'acetal')
 emit('}')
 k=E['spring_Nm_rad']*math.pi/180;d=E['spring_damping_Nms_rad']*math.pi/180
 emit(f'''def PhysicsRevoluteJoint "cam_axis" (prepend apiSchemas = ["PhysicsDriveAPI:angular"]) {{
 rel physics:body0 = </assembly/base>
 rel physics:body1 = </assembly/cam>
 uniform token physics:axis = "Z"
 point3f physics:localPos0 = (0,0,0)
 point3f physics:localPos1 = (0,0,0)
 bool physics:collisionEnabled = false
 token drive:angular:physics:type = "force"
 float drive:angular:physics:stiffness = {k}
 float drive:angular:physics:damping = {d}
 float drive:angular:physics:targetPosition = {E['spring_rest_angle_deg']}
 float drive:angular:physics:maxForce = 0.2
 }}
 def PhysicsPrismaticJoint "engaged_thread_flanks" (prepend apiSchemas = ["PhysxLimitAPI:linear"]) {{
 rel physics:body0 = </assembly/base>
 rel physics:body1 = </assembly/rear>
 uniform token physics:axis = "Z"
 point3f physics:localPos0 = (0,0,{rearzero})
 point3f physics:localPos1 = (0,0,0)
 float physics:lowerLimit = {-E['thread_half_clearance_m']}
 float physics:upperLimit = {E['thread_half_clearance_m']}
 float physxLimit:linear:stiffness = {E['thread_axial_stiffness_N_m']}
 float physxLimit:linear:damping = {E['thread_axial_damping_Ns_m']}
 bool physics:collisionEnabled = false
 }}''')
 emit('}')
 (ROOT/'assembly.usda').write_text('\n'.join(out)+'\n')
 return {'rear_zero_z_m':rearzero,'cam_top_z_m':camtop,'ramp_slope_m_rad':E['ramp_rise_m']/period}
if __name__=='__main__':print(build())
