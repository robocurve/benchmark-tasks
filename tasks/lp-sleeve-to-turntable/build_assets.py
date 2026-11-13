"""Original dimensioned CAD-style meshes; no downloaded models or physics shortcuts."""
import json, math
from pathlib import Path
P=Path(__file__).resolve().parent
(P/"assets").mkdir(exist_ok=True)
DARK=[.018,.021,.026,1];SILVER=[.62,.65,.69,1];PAPER=[.69,.58,.41,1]
def box(size,pos,color,**kw):return dict(type='box',size=[v/2 for v in size],position=pos,color=color,**kw)
def cyl(r,h,pos,color,**kw):return dict(type='cylinder',size=[r,h/2],position=pos,color=color,**kw)
def ring(ri,ro,h,n=128,start=0,end=2*math.pi):
 # Closed annular prism; hole remains a real hole. Sectors are convex collision pieces.
 points=[]
 for z in [-h/2,h/2]:
  for r in [ri,ro]:
   points.extend([[r*math.cos(start+(end-start)*i/n),r*math.sin(start+(end-start)*i/n),z] for i in range(n+1)])
 k=n+1; faces=[]
 for i in range(n):
  faces += [[i,i+1,k+i+1,k+i],[2*k+i,3*k+i,3*k+i+1,2*k+i+1],[i,2*k+i,2*k+i+1,i+1],[k+i,k+i+1,3*k+i+1,3*k+i]]
 faces += [[0,k,3*k,2*k],[n,2*k+n,3*k+n,k+n]]
 return dict(points=points,faces=faces)
def obj(name,mesh):
 f=P/'assets'/name
 f.write_text('# Original parametric mesh. Units: metres.\n'+''.join('v '+' '.join(map(str,p))+'\n' for p in mesh['points'])+''.join('f '+' '.join(str(i+1) for i in face)+'\n' for face in mesh['faces']))
 return str(f.relative_to(P))
def meshshape(name,mesh,color,collision=False,**kw):return dict(type='mesh',mesh_file=obj(name,mesh),color=color,collision=collision,**kw)
def body(name,shapes,mass=None,inertia=None):
 b=dict(name=name,shapes=shapes)
 if mass is None:b['static']=True
 else:b.update(mass=mass,inertia={'diagonal':inertia or [.001]*3})
 return b
def desc(name,b):return dict(name=name,bodies=[b],solver_position_iterations=64,solver_velocity_iterations=16)
# Generic consumer turntable; reference envelope 452 x 352 mm, not a product replica.
deck=[box([.352,.452,.085],[0,0,.0625],[.10,.065,.04,1],roughness=.32),box([.344,.444,.004],[0,0,.107],DARK)]
for x in [-.14,.14]:
 for y in [-.185,.185]:deck.append(cyl(.025,.025,[x,y,.0125],DARK))
pc=[-.02,.055,.117]
platter=[cyl(.15,.018,[0,0,0],SILVER,roughness=.22),cyl(.147,.002,[0,0,.010],[.035,.035,.035,1],friction=[.6,.5])]
platter += [cyl(.0034,.0086,[0,0,.0153],SILVER,contact_offset=.00002),dict(type='sphere',size=[.0034],position=[0,0,.0196],color=SILVER,contact_offset=.00002)]
for i in range(64):
 a=i*2*math.pi/64;platter.append(box([.002,.002,.002],[.149*math.cos(a),.149*math.sin(a),.005],[.15,.17,.19,1],collision=False))
# Actual two-axis, counterbalanced arm. The stylus sphere is a rigid contact proxy.
pivot=[.115,-.151,.154]
deck += [cyl(.024,.034,[.115,-.151,.125],SILVER),box([.012,.022,.034],[-.058,-.151,.126],DARK)]
gimbal=[cyl(.009,.014,[0,0,0],SILVER)]
arm=[dict(type='cylinder',size=[.004,0],fromto=[.065,0,0,-.202,0,0],color=SILVER),
 dict(type='cylinder',size=[.016,0],fromto=[.0315,0,0,.0585,0,0],color=DARK),box([.038,.018,.009],[-.220,0,-.004],DARK),
 box([.027,.016,.008],[-.225,0,-.013],[.68,.12,.09,1]),
 box([.022,.007,.004],[-.218,-.018,0],SILVER),
 dict(type='sphere',size=[.0007],position=[-.225,0,-.0235],color=[.7,.7,.75,1],friction=[.02,.01],contact_offset=.00002)]
deck += [cyl(.012,.009,[-.13,.12,.112],DARK),box([.095,.009,.002],[.005,-.198,.11],SILVER),box([.009,.018,.006],[.025,-.198,.114],DARK)]
button=[box([.025,.026,.008],[0,0,0],SILVER),box([.010,.002,.0002],[0,0,.0041],DARK,collision=False)]
# Lid removed as allowed by normal manual use; rear hinge sockets remain.
for y in [-.17,.17]:deck.append(box([.018,.025,.018],[.169,y,.114],DARK))
# Jacket is free, thin card, open toward local -Y. Symmetric access notches expose the rim.
sh=[];th=.0005;gap=.0034
for z in [-gap/2-th/2,gap/2+th/2]:
 sh += [box([.315,.28,th],[0,.0175,z],PAPER),box([.1175,.035,th],[-.09875,-.14,z],PAPER),box([.1175,.035,th],[.09875,-.14,z],PAPER)]
sh += [box([.001,.315,.0044],[-.157,0,0],PAPER),box([.001,.315,.0044],[.157,0,0],PAPER),box([.315,.001,.0044],[0,.157,0],PAPER)]
# Original, generic printed cover art built as thin visual-only shapes.
sh += [box([.23,.21,.00005],[0,.025,.00223],[.09,.22,.26,1],collision=False),cyl(.072,.00005,[0,.025,.00228],[.87,.49,.23,1],collision=False),box([.18,.003,.00005],[0,.132,.0023],[.9,.83,.65,1],collision=False)]
# Real annular record visual, with separate convex sectors retaining its physical centre hole.
record=[meshshape('lp-annulus.obj',ring(.00365,.15,.0018,256),DARK,roughness=.24)]
# One watertight native SDF collider avoids artificial radial seams under the stylus.
from collections import Counter
n=128; nr=2; ri=.00365; ro=.15; h=.0018
radii=[ri*(ro/ri)**(j/nr) for j in range(nr+1)]
points=[[r*math.cos(2*math.pi*i/n),r*math.sin(2*math.pi*i/n),z] for z in [-h/2,h/2] for r in radii for i in range(n)]
layer=(nr+1)*n; faces=[]
for j in range(nr):
 for i in range(n):
  k=(i+1)%n;a=j*n+i;b=(j+1)*n+i;c=(j+1)*n+k;d=j*n+k
  faces.extend([[d,c,b,a],[a+layer,b+layer,c+layer,d+layer]])
for i in range(n):
 k=(i+1)%n
 faces.extend([[i,i+layer,k+layer,k],[nr*n+i,nr*n+k,nr*n+k+layer,nr*n+i+layer]])
edges=Counter(tuple(sorted((a,b))) for f in faces for a,b in zip(f,f[1:]+f[:1]))
assert all(count==2 for count in edges.values()), 'Record collision mesh must be closed'
record.append(meshshape('lp-collision-annulus.obj',dict(points=points,faces=faces),DARK,collision=True,visible=False,collision_approximation='sdf',sdf_resolution=1024,contact_offset=.00002,friction=[.45,.35]))
for sign in [-1,1]:
 record.append(meshshape('label-'+str(sign)+'.obj',ring(.0038,.05,.00004,96),[.86,.73,.45,1],position=[0,0,sign*.00093]))
 for i in range(45):
  r=.058+i*.00185
  record.append(meshshape('groove-'+str(sign)+'-'+str(i)+'.obj',ring(r,r+.00012,.000015,128),[.045,.048,.054,1],position=[0,0,sign*.00092]))
 # Clear concentric label design, no copyrighted artwork, branding or identifying text.
 record.append(meshshape('label-line-'+str(sign)+'.obj',ring(.031,.0317,.00002,96),[.17,.25,.26,1],position=[0,0,sign*.00097]))
# Asymmetric generic label marks make actual rotation observable.
for i in range(3):
 record.append(box([.018-i*.003,.0012,.00002],[.009,.018+i*.004,.00099],[.17,.25,.26,1],collision=False))
m=.18;ro=.15;ri=.00365;h=.0018;ix=m*(3*(ro*ro+ri*ri)+h*h)/12;iz=m*(ro*ro+ri*ri)/2
rack=[box([.25,.018,.19],[.30,.225,.8425],[.34,.18,.08,1]),box([.25,.018,.19],[.30,.365,.8425],[.34,.18,.08,1])]
rack.append(box([.30,.20,.015],[.30,.295,.755],[.34,.18,.08,1]))
parts=[body('player',deck,5.,[.10,.10,.15]),body('platter',platter,1.1,[.0062,.0062,.0124]),body('gimbal',gimbal,.025,[.000003]*3),body('tonearm',arm,.18,[.00004,.0011,.0011]),body('button',button,.008,[.0000005]*3)]
parts[1]['position']=pc
parts[2]['position']=pivot
# Put the body frame at its balanced COM; hinge stays at the gimbal pivot.
# Geometry is rebased with the frame; the hinge and physical COM stay unchanged.
parts[3]['position']=[pivot[0]-.0025,pivot[1],pivot[2]]
for shape in arm:
 if 'fromto' in shape:
  shape['fromto'][0]+=.0025;shape['fromto'][3]+=.0025
 else:shape['position'][0]+=.0025
parts[4]['position']=[.130,.177,.117]
# Moving cap must clear the rotating platter throughout its vertical stroke.
assert math.hypot(abs(parts[4]['position'][0]-pc[0])-.0125, abs(parts[4]['position'][1]-pc[1])-.013) > .15+.006
joints=[dict(name='platter_axle',type='revolute',parent='player',child='platter',axis='Z',position_parent=pc),
 dict(name='arm_yaw',type='revolute',parent='player',child='gimbal',axis='Z',position_parent=pivot,lower=-.95,upper=.08,stiffness=.00114,target=0,damping=.005),
 dict(name='arm_pitch',type='revolute',parent='gimbal',child='tonearm',axis='Y',position_child=[.0025,0,0],lower=-.08,upper=.35,damping=.005),
 dict(name='start_button',type='prismatic',parent='player',child='button',axis='Z',position_parent=parts[4]['position'],lower=-.004,upper=0,stiffness=250,damping=1,target=.001)]
geo={'player':dict(name='Player',bodies=parts,joints=joints,articulation=True,fixed_base='player',solver_position_iterations=64,solver_velocity_iterations=16),'rack':desc('Rack',body('rack',rack)),'sleeve':desc('Sleeve',body('sleeve',sh,.075,[.00062,.00062,.00124])),'record':desc('LP',body('record',record,m,[ix,ix,iz]))}
# Smooth visual cylinders retain the original analytic PhysX cylinder colliders.
# USD's default cylinder tessellation is visibly polygonal at record-player scale.
for asset_name, description in geo.items():
 for part in description['bodies']:
  shapes=part['shapes']
  for i,shape in list(enumerate(shapes)):
   if shape['type']!='cylinder':continue
   r,h=shape['size'];h*=2
   if 'fromto' in shape:
    h=math.dist(shape['fromto'][:3],shape['fromto'][3:])
   n=128
   points=[[r*math.cos(2*math.pi*j/n),r*math.sin(2*math.pi*j/n),z] for z in [-h/2,h/2] for j in range(n)]
   faces=[list(reversed(range(n))),list(range(n,2*n))]+[[j,(j+1)%n,(j+1)%n+n,j+n] for j in range(n)]
   visual=meshshape(asset_name+'-'+part['name']+'-cylinder-'+str(i)+'.obj',dict(points=points,faces=faces),shape['color'],**{k:shape[k] for k in ('position','fromto','roughness') if k in shape})
   shape['visible']=False;shapes.append(visual)
cfg=dict(kind='lp_loading',instruction='Take the LP out of its sleeve and seat it on the turntable spindle. Set the sleeve aside. Lift the tonearm by its headshell, lower the stylus onto the outer recorded band, then press START to play the record. Leave both hands clear without dropping the LP or dragging the stylus.',table_top=.7475,max_steps=2400,control_hz=20,hold_s=1,poses={'player':[.40,-.22,.7475],'rack':[0,0,0],'sleeve':[.30,.30,.9397],'record':[.30,.30,.9389]},record_radius=.15,record_half_thickness=.0009,hole_radius=.00365,spindle_radius=.0034,spindle_xy=[.38,-.165],mat_top=.8755,spindle_top=.8875,sleeve_return=[.30,.30,.9397])
cfg.update(stylus_local=[-.2225,0,-.0242],headshell_local=[-.2175,0,0],button_rest_z=.8645,button_xy=[.530,-.043],play_rpm=100/3,playback_hold_s=5,lead_in_radius=[.135,.147],recorded_radius=[.058,.147])
for name,data in [('geometry.json',geo),('pilot.json',cfg)]: (P/name).write_text(json.dumps(data,indent=2)+'\n')
print('Authored',len(list((P/'assets').glob('*.obj'))),'original meshes; record 300 mm x 1.8 mm, 7.3 mm through-hole.')
