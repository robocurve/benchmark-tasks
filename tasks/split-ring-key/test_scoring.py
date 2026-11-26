"""Numerical counterexamples only; these do not demonstrate physical success."""
import json,math
import numpy as np
import geometry as G
import task as T

def rubric(hits,**kw):
 defaults=dict(end_distance=.025,speed=0.,spin=0.,clear=True,closed=True,integrity=True,key_height=1.,splitter=0.,passages=(0,sum(h['sign'] for h in hits)),inspection_ready=True)
 defaults.update(kw);return T.evaluate(hits,**defaults)

def hit(arc,sign=1):return {'arc_rad':arc,'sign':sign}

def checks():
 two=[hit(.8),hit(.8+2*math.pi)]
 cases=[]
 def check(name,condition):
  assert condition,name;cases.append({'case':name,'passed':True})
 check('valid double turn',rubric(two)['complete'])
 check('mere proximity',rubric([])['partial_score']==0 and not rubric([])['complete'])
 check('near end single strand',rubric([hit(.05)],end_distance=.004)['partial_score']==.25)
 check('part-fed single strand',rubric([hit(3.)])['partial_score']==.5 and not rubric([hit(3.)])['complete'])
 check('held double turn awaits unloaded inspection',not rubric(two,clear=False)['complete'] and rubric(two,clear=False)['partial_score']==0.)
 check('spring still open',not rubric(two,closed=False)['complete'])
 check('splitter still pulled',not rubric(two,splitter=.004)['complete'])
 check('moving key',not rubric(two,speed=.03)['complete'])
 check('spinning key',not rubric(two,spin=.7)['complete'])
 check('moving ring is not settled retention',not rubric(two,ring_spin=.1)['complete'])
 check('at free ends',not rubric(two,end_distance=.002)['complete'])
 check('broken mechanism scores zero',rubric(two,integrity=False)['partial_score']==0)
 check('mid-wire forced placement lacks visible tip passage',rubric(two,passages=(0,0))['partial_score']==0)
 check('one passage cannot qualify two strands',rubric(two,passages=(0,1))['partial_score']==0)
 check('two passes of the same end qualify two strands',rubric(two,passages=(0,2))['complete'])
 check('wrong signed winding cannot qualify two strands',rubric(two,passages=(2,0))['partial_score']==0)
 check('loaded configuration awaits unloaded inspection',rubric(two,inspection_ready=False)['partial_score']==0)
 for credit_hits in ([hit(.05)],[hit(3.)],two):
  check(f'visible damage zeros tier with {len(credit_hits)} strands at {credit_hits[0]["arc_rad"]}',rubric(credit_hits,integrity=False)['partial_score']==0)
 check('same coil folded twice',rubric([hit(.8),hit(2.)])['partial_score']==0)
 check('opposite directions are not two retained turns',not rubric([hit(.8),hit(.8+2*math.pi,-1)])['complete'])
 check('table-supported proximity is not retained',not rubric(two,key_height=G.TABLE_Z+.02)['complete'])
 I=np.eye(3);O=np.zeros(3)
 # A true crossing split at a shared endpoint counts once.
 seg=[(np.array([0.,-.004,0.]),O),(O,np.array([0.,.004,0.]))]
 check('shared-vertex crossing deduplicated',len(T.crossings(seg,O,I))==1)
 seg=[(np.array([0.,-.004,0.]),O),(O,np.array([0.,-.004,0.]))]
 check('shared-vertex tangency cancels',len(T.crossings(seg,O,I))==0)
 seg=[(np.array([-.003,-.00001,0.]),np.array([.003,.00001,0.]))]
 check('near-tangent tube cannot fit',len(T.crossings(seg,O,I))==0)
 seg=[(np.array([G.HOLE_R,-.004,0.]),np.array([G.HOLE_R,.004,0.]))]
 check('wire surface grazing rim not counted',len(T.crossings(seg,O,I))==0)
 seg=[(np.array([-.011,-.004,0.]),np.array([.011,.004,0.]))]
 check('oblique mid-plane fit blocked by finite-thickness rim',len(T.crossings(seg,O,I))==0)
 angle=math.radians(30);hole=G.HOLE_R*math.cos(math.pi/32)
 offset=hole-.001*math.tan(angle)-G.WIRE_R/math.cos(angle)-.000025
 seg=[(np.array([offset-.004*math.tan(angle),-.004,0]),np.array([offset+.004*math.tan(angle),.004,0]))]
 check('valid oblique rim contact fits full thickness',len(T.crossings(seg,O,I))==1)
 v=[np.array(x)for x in[(0,-.004,0),(0,0,0),(.004,.0005,0),(0,.001,0),(0,.004,0)]]
 check('curved wire bulge inside slab rejected',len(T.crossings(list(zip(v[:-1],v[1:])),O,I))==0)
 seg=[(np.array([0.,-.004,0.]),np.array([0.,.0005,0.]))]
 check('tip ending inside plate is not a through crossing',len(T.crossings(seg,O,I))==0)
 # Independent polygon-wall/tolerance boundary cases. A vertex direction
 # has more space than the inscribed circle; face clearance remains exact.
 hole=G.HOLE_R*math.cos(math.pi/32);normal=np.array([math.cos(math.pi/32),math.sin(math.pi/32)])
 def upright(point):return [(np.array([point[0],-.004,point[1]]),np.array([point[0],.004,point[1]]))]
 point=np.array([(hole-G.WIRE_R+.000159)/math.cos(math.pi/32),0.])
 hh=T.crossings(upright(point),O,I)
 check('polygon vertex contact inside 0.16 mm allowance',len(hh)==1 and abs(hh[0]['slab_clearance_m']+.000159)<1e-12)
 point=normal*(hole-G.WIRE_R+.000161)
 check('polygon face contact outside 0.16 mm allowance',len(T.crossings(upright(point),O,I))==0)
 point=np.array([(hole-G.WIRE_R-.000001)/math.cos(math.pi/32),0.])
 hh=T.crossings(upright(point),O,I)
 check('actual polygon vertex clearance is not inscribed-circle clearance',len(hh)==1 and abs(hh[0]['slab_clearance_m']-.000001)<1e-12)
 def outside_piece(slope):
  out=[]
  for x in [.1,.9]:
   point=normal*G.WIRE_R*(3+slope*x);out.append(np.array([point[0],.001+G.WIRE_R*x,point[1]]))
  return out
 aa,bb=outside_piece(.75)
 check('capsule face-cap stationary support matches analytic value',abs(T.capsule_slab_support(aa,bb,.001,G.WIRE_R)-G.WIRE_R*(3+1.25))<1e-12)
 aa,bb=outside_piece(-.75)
 check('capsule face-cap clipped endpoint matches analytic value',abs(T.capsule_slab_support(aa,bb,.001,G.WIRE_R)-G.WIRE_R*(3-.075+math.sqrt(.99)))<1e-12)
 seg=[(np.array([-.5,-.004,0.]),np.array([.5,.004,0.]))]
 check('near-parallel full-depth wire exceeds polygon hole',len(T.crossings(seg,O,I))==0)
 spans=[]
 for i in range(G.N):
  p=G.segment_curve(i)@G.W.T+G.CENTER;spans.extend(zip(p[:-1],p[1:]))
 a=math.pi/2
 hh=T.crossings(spans,G.hole_target(a,.0004),G.key_rotation(a))
 check('actual authored curved two-turn geometry',len(hh)==2 and rubric(hh)['complete'])
 hh=T.crossings(spans,G.hole_target(a,.003),G.key_rotation(a))
 check('actual authored single-strand geometry',len(hh)==1 and rubric(hh)['partial_score']==.5)
 baseline={'diameters_m':[.038,.038],'roundness_m':0.,'tip_surface_gaps_m':[0.,0.],'visible_break_m':0.}
 check('intact unloaded measurements',T.visible_integrity(baseline,baseline))
 for field,value in [('diameters_m',[.0391,.038]),('roundness_m',.0011),('tip_surface_gaps_m',[.00051,0.]),('visible_break_m',.00026)]:
  damaged={**baseline,field:value}
  check(f'visible damage {field}',not T.visible_integrity(damaged,baseline))
 tracker=T.TipPassage()
 for y in np.linspace(-.004,.004,17):active=tracker.update([[0,y,0],[.02,.01,0]])
 check('free tip passes both faces',active==[1,0] and tracker.events==[1,0])
 for y in np.linspace(.004,-.004,17):active=tracker.update([[0,y,0],[.02,.01,0]])
 check('back-out cancels passage',active==[0,0] and tracker.events==[2,0])
 tracker=T.TipPassage()
 for y in np.linspace(-.004,.004,17):active=tracker.update([[.005,y,0],[.02,.01,0]])
 check('forced solid-bow traversal rejected',active==[0,0])
 tracker=T.TipPassage()
 point=normal*(hole-G.WIRE_R+.00010)
 for y in np.linspace(-.004,.004,17):
  active=tracker.update([[point[0],y,point[1]],[.02,.01,0]])
 check('tip passage keeps a stricter rim tolerance than final fit',active==[0,0])
 tracker=T.TipPassage()
 for y in np.linspace(-.004,.0005,17):active=tracker.update([[0,y,0],[.02,.01,0]])
 check('partial-depth tip does not count passage',active==[0,0])
 tracker=T.TipPassage()
 for repeat in range(2):
  for x,y in [(.02,.004),(.02,-.004),(0.,-.004)]:tracker.update([[.02,.01,0],[x,y,0]])
  for y in np.linspace(-.004,.004,17):active=tracker.update([[.02,.01,0],[0,y,0]])
 check('same marked tip can visibly thread twice',active==[0,2] and tracker.events==[0,2])
 lower=G.ring_curve(np.array([.2,1.,3.,5.]))
 upper=G.ring_curve(np.array([.2,1.,3.,5.])+2*math.pi)
 check('ordinary turns touch without authored entry gap',np.allclose(np.linalg.norm(upper-lower,axis=1),2*G.WIRE_R))
 return cases

if __name__=='__main__':
 cases=checks();print(json.dumps({'passed':len(cases),'cases':cases},indent=2))
