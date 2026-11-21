"""Fail-closed state and trajectory controls for the core-removal scorer."""
CASES=[
 ('unknown hands at start',{},0.,False),
 ('core pulled away without cutting',{'core':((.22,-.06,.7625),None,None)},0.,False),
 ('knife moved only',{'knife':((.15,.02,.88),None,None)},0.,False),
 ('whole quarter carried',{'leaf0':((.33,.07,.9),None,None),'core':((.33,.07,.9),None,None)},0.,False),
 ('core dropped below board',{'core':((.25,-.1,.70),None,None)},0.,False),
 ('quarter knocked over',{'leaf0':((.33,.07,.80),None,None,(.70710678,0,0,.70710678))},0.,False),
]

def run():
 import task as t
 import numpy as np
 objects={p.name:p.state() for p in t.OBJECTS}
 checks=[]
 def check(label,condition):
  checks.append((label,bool(condition)));print(('ok ' if condition else 'FAIL ')+label)
 # Knife-free core motion permanently fails, even if returned.
 t.REC=t.fresh()
 far={'left':np.array([[.1,.3,1.1]]),'right':np.array([[.1,-.3,1.1]])}
 t.observe(objects,far,.05)
 moved=dict(objects);moved['core']=t.CORE.state(pos=t.C+np.array([-.08,0,0]))
 t.observe(moved,far,.10);check('knife-free pull is permanent zero',t.REC['bad'] and t.score(moved)==0.)
 t.observe(objects,far,.15);check('returning core does not erase violation',t.REC['bad'])
 # Swept edge must cover a surface, not touch its tip or cross its endpoint.
 pts=t.CUT_SAMPLES[0]
 t.REC=t.fresh();tip=np.tile(pts[0],(3,1));t.record_incisions(tip,tip-np.array([0,0,.058]))
 check('tip-only plunge cannot sever full surface',t.REC['cuts'][0].mean()<.05)
 # An actual plane sweep intersects all samples of a boundary.
 t.REC=t.fresh();a=np.array([[-.07,-.025,.065],[-.03,0,.065],[-.03,0,.065]])
 b=a-np.array([0,0,.070]);t.record_incisions(a,b)
 check('full boundary sweep covers the surface',t.REC['cuts'][0].all() and not t.REC['cuts'][1].any())
 # The edge must lead: the demo's blade (rolled 60 degrees, lowered straight down) is 30 degrees off its plane.
 import math
 R=t.parts._rot((0,0,1),.5)@t.parts._rot((1,0,0),math.pi/3)
 normal=R[:,2];facing=-R[:,1];along=R[:,0]
 check('edge-first stroke 30 deg off the blade plane cuts',t.edge_leads(np.array([0,0,-.005]),R))
 check('draw along the edge cuts',t.edge_leads(along*.005,R))
 check('flat side first does not cut',not t.edge_leads(normal*.005,R))
 check('spine first does not cut',not t.edge_leads(-facing*.005,R))
 tilt50=facing*math.cos(math.radians(50))+normal*math.sin(math.radians(50))
 check('50 deg off the blade plane does not cut',not t.edge_leads(tilt50*.005,R))
 check('under 1 mm of motion in 0.5 s does not cut',not t.edge_leads(np.array([0,0,-.0009]),R))
 # Band: a parallel sweep counts from 2 mm inside the core outline to 10 mm outside it, never farther.
 import fixture.cut_geometry as cg
 for label,offset,want in [('9.9 mm outside',.0099,True),('10.1 mm outside',.0101,False),('1.9 mm inside',-.0019,True),('2.1 mm inside',-.0021,False)]:
  t.REC=t.fresh();n=cg.FRAMES[0][1][1];t.record_incisions(a+n*offset,b+n*offset)
  check('parallel sweep %s %s'%(label,'cuts' if want else 'does not cut'),bool(t.REC['cuts'][0].all())==want and bool(t.REC['cuts'][0].any())==want)
 # A transverse chop crosses the band along one line only.
 t.REC=t.fresh();o,B,L=cg.FRAMES[0];mid=o+B[0]*L/2
 chop=np.array([mid-B[1]*.06+[0,0,.065],mid+B[1]*.06+[0,0,.065],mid+B[1]*.06+[0,0,.065]])
 t.record_incisions(chop,chop-np.array([0,0,.070]))
 check('transverse chop severs at most one line of the surface',t.REC['cuts'][0].mean()<.05)

 # A geometric hit halo cannot turn a2 mm actual cut into the required5 mm incision.
 t.REC=t.fresh()
 shallow=np.array([[-.07,-.025,.011],[-.03,0.,.011],[-.03,0.,.011]])
 t.record_incisions(shallow,shallow-np.array([0,0,.002]))
 check('2mm shallow edge travel cannot qualify as5mm incision',not t.incision())
 # Exact thresholds use connected actual cut-surface extents, not a dilated point mask.
 def patch(length,depth,start=.008,normal=0.):
  u=np.array([.04,.025,0.]);u/=np.linalg.norm(u);v=np.array([-u[1],u[0],0.]);origin=np.array([-.07,-.025,.059])
  a=np.array([origin+u*start+v*normal,origin+u*(start+length)+v*normal,origin+u*(start+length)+v*normal])
  t.record_incisions(a,a-np.array([0,0,depth]))
 for label,length,depth,start,want in [('exact20x5',.020,.005,.008,True),('translated20x5',.020,.005,.01337,True),('short19.9x5',.0199,.005,.008,False),('shallow20x4.9',.020,.0049,.008,False)]:
  t.REC=t.fresh();patch(length,depth,start);check(label+' actual surface threshold',t.incision()==want)
 t.REC=t.fresh();patch(.010,.005,.003);patch(.010,.005,.014)
 check('disconnected10mm incisions cannot combine into20mm',not t.incision())
 patch(.002,.005,.0125)
 check('physically connected patches combine continuously',t.incision())
 t.REC=t.fresh();patch(.010,.005,.003,-.0019);patch(.010,.005,.013,.0019)
 check('opposite slab-side cuts3.8mm apart cannot merge in projection',not t.incision())
 tangent=np.array([.04,.025,0.]);tangent/=np.linalg.norm(tangent);normal=np.array([-tangent[1],tangent[0],0.]);origin=np.array([-.07,-.025,.059])
 top=origin+tangent*.013-normal*.0019
 bridge=np.array([top,top-np.array([0,0,.005]),top-np.array([0,0,.005])])
 t.record_incisions(bridge,bridge+normal*.0038)
 check('an actual3D bridge joins previously separate cuts',t.incision())
 t.REC=t.fresh()
 for start,offset in ((.003,-.001),(.013,-.003)):
  ends=[origin+tangent*u+normal*(.2*u+offset) for u in (start,start+.010)]
  edge=np.array([ends[0],ends[1],ends[1]])
  t.record_incisions(edge,edge-np.array([0,0,.005]))
 check('tilted parallel cuts with overlapping3D bounds stay disconnected',not t.incision())
 t.REC=t.fresh();patch(.020,.005,.003,.0021)
 check('a 20 x 5 mm cut 2.1 mm inside the core outline cannot qualify',not t.incision())
 t.REC=t.fresh();patch(.020,.005,.003,-.0101)
 check('a 20 x 5 mm cut 10.1 mm out in the leaves cannot qualify',not t.incision())
 t.REC=t.fresh();patch(.020,.005,.003,-.0099)
 check('a 20 x 5 mm cut 9.9 mm out in the leaves qualifies',t.incision())
 t.REC=t.fresh()
 top=np.array([[-.07,-.025,.065],[-.03,0,.065],[-.03,0,.065]])
 t.record_incisions(top,top-np.array([0,0,.010]))
 check('top-down4mm core penetration cannot qualify5mm',not t.incision())
 # Swept damage includes a moved leaf group and an unheld blade; returning pieces cannot erase it.
 t.REC=t.fresh();damaged=dict(objects)
 damaged['leaf2']=t.LEAVES[2].state(pos=t.C+np.array([0,.15,0]))
 damaged['knife']=t.KNIFE.state(pos=t.C+np.array([-.134,.15,.084]))
 t.observe(damaged,far,.05)
 damaged['knife']=t.KNIFE.state(pos=t.C+np.array([-.134,.15,.054]))
 t.observe(damaged,far,.10)
 check('unheld blade damage to a moved leaf is permanent zero',t.REC['bad'] and t.REC['overcut']>.025)
 t.observe(objects,far,.15)
 check('restoring leaf cannot erase the swept damage',t.REC['bad'])

 # Neither moving food nor an interval-only hand incursion may evade history checks.
 t.REC=t.fresh();before=dict(objects);after=dict(objects)
 stationary=t.KNIFE.state(pos=t.C+np.array([-.130,0.,.034]))
 before['knife']=after['knife']=stationary
 before['leaf1']=t.LEAVES[1].state(pos=t.C+np.array([0.,-.200,0.]))
 after['leaf1']=t.LEAVES[1].state(pos=t.C+np.array([0.,.200,0.]))
 t.observe(before,far,.05);t.observe(after,far,.10)
 check('food moving across stationary blade cannot evade damage',t.REC['bad'] and t.REC['overcut']>.025)
 t.REC=t.fresh();blade=t.KNIFE.shapes[2];kp=objects['knife']
 a=np.asarray(blade.at)+np.array([0.,blade.half[1]+.029,.020])
 b=a-np.array([0.,0.,.040])
 lefta=t.transform(kp,[a]);leftb=t.transform(kp,[b])
 t.observe(objects,{'left':lefta,'right':far['right']},.05)
 check('hand interval witness endpoints exceed guarded clearance',not t.REC['bad'])
 t.observe(objects,{'left':leftb,'right':far['right']},.10)
 check('between-frame hand path inside30mm is permanent zero',t.REC['bad'])
 # Slow accumulated motion must not earn a stillness hold just because each step is short.
 t.REC=t.fresh();drift=None
 for i in range(45):
  drift={o.name:o.state(pos=np.asarray(o.pos)+np.array([i*.0004,0,0])) for o in t.OBJECTS}
  t.observe(drift,far,(i+1)*.05)
 check('8mm/s accumulated creep cannot satisfy1s hold',t.REC['hold']<1.)
 for i in range(45,70):t.observe(drift,far,(i+1)*.05)
 check('stationary released scene satisfies1s anchored hold',t.REC['hold']>=1.)

 grip=t.HandClouds({'right':np.array([[0.,0.,1.]]),'left':np.array([[0.,0.,1.]])})
 sh=t.KNIFE.shapes[0];st=objects['knife'];R=t.rot(st)
 grip.pads['right']=tuple((np.array(st.pos)+R@np.array([0,sgn*.015,0]))[None,:] for sgn in (-1,1))
 grip.fingers['right']=np.array([-.015,-.015])
 check('bilateral native handle contact with closed fingers qualifies grasp',t.native_grasps(objects,grip)['right'])
 holder_contact=t.HandClouds({'right':t.transform(st,t.EDGE[:1]),'left':far['left']})
 holder_contact.pads=grip.pads;holder_contact.fingers=grip.fingers
 t.REC=t.fresh();t.observe(objects,holder_contact,.05)
 check('native handle grip never exempts blade contact',t.REC['bad'] and t.score(objects)==0.)

 same=t.HandClouds(grip);same.pads={'right':(grip.pads['right'][0],grip.pads['right'][0])};same.fingers={'right':np.array([-.015,-.015])}
 check('two pads pushing the same handle face cannot qualify a grasp',not t.native_grasps(objects,same)['right'])
 grip.fingers['right']=np.array([-.04,-.04])
 check('open fingers near handle cannot qualify a grasp',not t.native_grasps(objects,grip)['right'])
 grip.fingers['right']=np.array([-.015,-.015]);grip.pads['right']=(grip.pads['right'][0],np.array([[1.,1.,1.]]))
 check('one touching pad cannot qualify a grasp',not t.native_grasps(objects,grip)['right'])
 # Source-unit scoring boundaries, including exactly 5 and 10 mm, hold and release gates.
 t.REC=t.fresh();r=t.REC
 r.update(known=True,clear=True,hold=1.,bonds=False,layer_bonds=False)
 full=np.array([[-.07,-.025,.059],[-.03,0.,.059],[-.03,0.,.059]])
 t.record_incisions(full,full-np.array([0,0,.058]))
 r['cuts']=[np.ones(len(p),bool) for p in t.CUT_SAMPLES]
 final=dict(objects);final['core']=t.CORE.state(pos=t.C+np.array([0,-.1,0]))
 for value,want in [(0.,1.),(.005,1.),(.005002,.67),(.010,.67),(.010002,.33)]:
  r['overcut']=r['strip']=value
  check('overcut %.6f tier %.2f'%(value,want),t.score(final)==want)
 r['overcut']=r['strip']=0.
 r['layer_bonds']=True
 check('food compression bonds must be released before full credit',t.score(final)==0.)
 r['layer_bonds']=False
 edgecase=dict(final);edgecase['leaf2']=t.LEAVES[2].state(pos=t.C+np.array([.008,0,0]),quat_xyzw=(0,0,np.sin(.055),np.cos(.055)))
 check('corresponding layer edge over10mm caps credit despite close origins',t.score(edgecase)==.33)
 for shift,want in [(.010,1.),(.010002,.33)]:
  shifted=dict(final);shifted['leaf2']=t.LEAVES[2].state(pos=t.C+np.array([shift,0,0]))
  check('layer edge displacement %.6f boundary'%shift,t.score(shifted)==want)
 slid=dict(final)
 for k in range(3):slid['leaf%d'%k]=t.LEAVES[k].state(pos=t.C+np.array([0.,.012,0.]))
 check('whole stack slid 12 mm from its start caps at 0.33',t.score(slid)==.33)
 close=dict(final);close['core']=t.CORE.state(pos=t.C+np.array([0,-.050,0]))
 check('core face within10mm fails detached tier despite distant centres',t.score(close)==.33)
 for mask in r['cuts']:mask[:]=True
 r['cuts'][0][0]=r['cuts'][0][28]=False
 check('two disconnected sub10mm remnants do not combine into one58mm piece',t.score(final)==1.)
 r['cuts'][0][:]=True;r['cuts'][0][:5]=False
 check('connected residual span with grid guard over10mm caps full credit',t.score(final)==.67)
 for mask in r['cuts']:mask[:]=True
 r['hold']=.999;check('one-second hold has no early credit',t.score(final)==0.)
 r['hold']=1.;r['clear']=False;check('held core has no credit',t.score(final)==0.)
 r['clear']=True;r['known']=False;check('unknown hands fail closed',t.score(final)==0.)
 r['known']=True;r['bad']=True;check('whole-trial violation is zero',t.score(final)==0.)
 r['bad']=False
 separated=dict(final);separated['leaf2']=t.LEAVES[2].state(pos=t.C+np.array([.012,0,0]))
 check('separated leaf group capped at incision tier',t.score(separated)==.33)
 r['cuts']=[np.zeros(len(p),bool) for p in t.CUT_SAMPLES]
 r['incised']=False;r['incision_groups']=[[],[]]
 check('unearned separation with zero cuts earns zero',t.score(final)==0.)
 # Real face points must contain interior points as well as hull vertices.
 from yam_arena import analysis
 h=analysis.link_hulls()['gripper'][0]
 cloud=t.face_points('gripper')
 check('release test includes hull face interiors',len(cloud)>len(h.vertices))
 # A real collision-hull face, 0.5 mm from a2 mm cube, can have every hull vertex >5 mm away.
 allh=analysis.link_hulls()['gripper'];vertices=np.concatenate([h.vertices for h in allh]);witness=False
 for hh in allh:
  for plane in np.unique(np.round(hh.planes,7),axis=0):
   n,d=plane[:3],plane[3];on=hh.vertices[np.abs(hh.vertices@n-d)<1e-5]
   if len(on)<3:continue
   c=on.mean(axis=0);u=on[0]-c
   if np.linalg.norm(u)<1e-6:continue
   u/=np.linalg.norm(u);v=np.cross(n,u);R=np.column_stack([u,v,n]);centre=c+n*.0015
   vd=analysis.box_signed_distance(vertices,centre,R,np.array([.002,.002,.002])).min()
   fd=analysis.box_signed_distance(cloud,centre,R,np.array([.002,.002,.002])).min()
   if vd>.005 and fd<.005:witness=True;break
  if witness:break
 check('0.5 mm face contact blocked despite all vertices over5 mm',witness)
 t.REC=t.fresh()
 return sum(not ok for _,ok in checks)
if __name__=='__main__':raise SystemExit(run())
