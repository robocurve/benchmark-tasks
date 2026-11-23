"""Secured 96-well plate, flexible adhesive film and hand-held padded sealing roller."""
from geometry_helpers import *
BODIES=[];JOINTS=[]
PLATE=np.array([.35,0,TABLE+.047]);TOP=PLATE[2]+.017
FILM=np.array([.20,0,TABLE+.120]);NX=18;NY=12;DX=.130/NX;DY=.080/NY;THICKNESS=.0001
WELLS=np.array([[(i-5.5)*.009,(j-3.5)*.009,.017] for i in range(12) for j in range(8)])
# A rectangular-well polypropylene 1 mL deepwell plate. Grid walls really collide.
shapes=[box((.12776,.08548,.002),(0,0,-.016))]
for i in range(13):shapes.append(box((.0015,.0735,.032),((i-6)*.009,0,0)))
for j in range(9):shapes.append(box((.1095,.0015,.032),(0,(j-4)*.009,0)))
shapes += [box((.12776,.002,.034),(0,sy*.04174,0)) for sy in [-1,1]]
shapes += [box((.002,.08148,.034),(sx*.06288,0,0)) for sx in [-1,1]]
# Top lands are at 17 mm above plate centre.
for shape in shapes[1:23]:shape['pos'][2]=.001
BODIES.append(body('plate',PLATE,shapes,mass=.104,fixed=True,color=(.88,.90,.91),friction=.9,contact_offset=.0001))
# The plate is secured in a reusable bench holder; four low clips show the fixture.
BODIES.append(body('plate_support',[.35,0,TABLE+.015],[box((.140,.100,.030))]+[box((.012,.004,.008),(x,y,.019)) for x in [-.055,.055] for y in [-.044,.044]],mass=.3,fixed=True,color=(.35,.43,.49),friction=.8))
# The film rests adhesive-side down on its non-stick backing, over a clean tray.
BODIES.append(body('release_tray',FILM+[0,0,-.0602],[box((.116,.044,.120))],mass=.2,fixed=True,color=(.60,.68,.72),friction=.05))
BODIES.append(body('release_liner',FILM+[0,0,-.00015],[box((.130,.044,.0001))],mass=.001,fixed=True,color=(.94,.94,.90),friction=.03))
CELLS=[]
for i in range(NX):
 for j in range(NY):
  name=f'film_{i}_{j}';center=FILM+[(i-(NX-1)/2)*DX,(j-(NY-1)/2)*DY,0];CELLS.append(name)
  BODIES.append(body(name,center,[box((DX-.00001,DY-.00001,THICKNESS))],mass=.00144/(NX*NY),color=(.65,.79,.84),visible=False,friction=.3,contact_offset=.00001,speculative_ccd=True,exclude=[f"film_{ii}_{jj}" for ii in range(max(0,i-1),min(NX,i+2)) for jj in range(max(0,j-1),min(NY,j+2)) if (ii,jj)!=(i,j)]))
  for di,dj in [(-1,0),(0,-1)]:
   if i+di<0 or j+dj<0:continue
   other=f'film_{i+di}_{j+dj}';delta=np.array([di*DX,dj*DY,0]);axes=bend_axes(.0002,.0000001,limit=170)
   JOINTS.append(joint(name+'_'+other,other,name,-delta/2,delta/2,axes=axes,break_force=12.,break_torque=.06))
TOOL=np.array([.18,-.17,TABLE+.012]);TOOL_REST_R=Rotation.from_euler('x',90,degrees=True);ROLLER=TOOL+TOOL_REST_R.apply([0,0,-.048])
# Loose roller lies on the bench; gravity settles its handle beside the wheel.
BODIES.append(body('roller_handle',TOOL,[box((.010,.016,.065)),box((.055,.012,.004),(0,0,-.0325))],quat=TOOL_REST_R.as_quat(),mass=.025,color=(.70,.12,.10),friction=.8))
BODIES.append(body('roller_head',ROLLER,[cylinder(.012,.050,quat=Rotation.from_euler('y',90,degrees=True).as_quat())],quat=TOOL_REST_R.as_quat(),mass=.015,color=(.16,.17,.18),friction=.6,contact_offset=.0001))
axes=fixed_axes();axes['rotX']=dict(limits=(-180,180));axes['transZ']=dict(limits=(-.0003,.003),stiffness=800.,damping=1.5)
JOINTS.append(joint('roller_compliance','roller_handle','roller_head',[0,0,-.048],[0,0,0],axes=axes,break_force=20.))
finalize(BODIES,JOINTS)
SAMPLES=np.array([[x,y,0] for x in np.linspace(-DX/2,DX/2,5) for y in np.linspace(-DY/2,DY/2,5)])
