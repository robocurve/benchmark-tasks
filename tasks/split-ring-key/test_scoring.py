"""Numerical counterexamples only; these do not demonstrate physical success."""
import json,math
import numpy as np
import geometry as G
import task as T

def rubric(hits,**kw):
 defaults=dict(end_distance=.025,speed=0.,spin=0.,clear=True,closed=True,integrity=True,key_height=1.,splitter=0.)
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
 check('held double turn',not rubric(two,clear=False)['complete'] and rubric(two,clear=False)['partial_score']==.75)
 check('spring still open',not rubric(two,closed=False)['complete'])
 check('splitter still pulled',not rubric(two,splitter=.004)['complete'])
 check('moving key',not rubric(two,speed=.03)['complete'])
 check('spinning key',not rubric(two,spin=.7)['complete'])
 check('moving ring is not settled retention',not rubric(two,ring_spin=.1)['complete'])
 check('at free ends',not rubric(two,end_distance=.002)['complete'])
 check('broken mechanism scores zero',rubric(two,integrity=False)['partial_score']==0)
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
 point=np.array([(hole-G.WIRE_R+.000049)/math.cos(math.pi/32),0.])
 hh=T.crossings(upright(point),O,I)
 check('polygon vertex contact within49um allowance accepted',len(hh)==1 and abs(hh[0]['slab_clearance_m']+.000049)<1e-12)
 point=normal*(hole-G.WIRE_R+.000051)
 check('polygon face contact beyond51um allowance rejected',len(T.crossings(upright(point),O,I))==0)
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
 hh=T.crossings(spans,G.hole_target(a,.004),G.key_rotation(a))
 check('actual authored single-strand geometry',len(hh)==1 and rubric(hh)['partial_score']==.5)
 # Real-trial ring checks on the authored ring-frame centreline.
 line=np.vstack([G.segment_curve(i)[:-1] for i in range(G.N)]+[G.segment_curve(G.N-1)[-1:]]);arc=np.linspace(0,G.END_ANGLE,len(line))
 m=T.ring_measures(line)
 check('rest spiral measures 0.3 mm turn gap and 3.13 mm span',abs(m['turn_gap_m']-(G.PITCH-2*G.WIRE_R))<1e-9 and abs(m['axial_span_m']-.0031333)<1e-6 and m['radial_dev_m']<1e-12)
 check('rest spiral is intact and closed',T.ring_intact(m['turn_gap_m'],m['radial_dev_m']) and T.ring_closed(m['turn_gap_m'],m['axial_span_m']))
 def tip_lift(lift):
  l=line.copy();l[:,2]+=lift*np.clip((arc-math.radians(560))/(G.END_ANGLE-math.radians(560)),0,1);return T.ring_measures(l)
 m=tip_lift(G.SPREAD_LIMIT-G.PITCH+2*G.WIRE_R-.0002)
 check('tip spread just inside the video limit stays intact',abs(m['turn_gap_m']-(G.SPREAD_LIMIT-.0002))<1e-9 and T.ring_intact(m['turn_gap_m'],m['radial_dev_m']))
 m=tip_lift(G.SPREAD_LIMIT-G.PITCH+2*G.WIRE_R+.0002)
 check('tip spread beyond the video limit is damage',not T.ring_intact(m['turn_gap_m'],m['radial_dev_m']))
 check('earlier peak spread stays latched after the turns close',not T.ring_intact(G.SPREAD_LIMIT+.0001,0.))
 def bump(h):
  l=line.copy();l[(arc>=math.radians(480))&(arc<=math.radians(520)),2]+=h;return T.ring_measures(l)
 m=bump(G.FINAL_GAP_LIMIT-G.PITCH+2*G.WIRE_R-.0002)
 check('released ring within feeler-gauge gap is closed',T.ring_closed(m['turn_gap_m'],m['axial_span_m']))
 m=bump(G.FINAL_GAP_LIMIT-G.PITCH+2*G.WIRE_R+.0002)
 check('turn left spread fails feeler-gauge check while flat',m['axial_span_m']<=G.CLOSED_SPAN and not T.ring_closed(m['turn_gap_m'],m['axial_span_m']))
 check('ring not lying flat fails caliper thickness check',not T.ring_closed(G.PITCH-2*G.WIRE_R,G.CLOSED_SPAN+.0001))
 l=line.copy();l[:,:2]*=(G.R+G.RADIAL_LIMIT+.0005)/G.R;m=T.ring_measures(l)
 check('ring outline distorted beyond 4 mm is damage',not T.ring_intact(m['turn_gap_m'],m['radial_dev_m']))
 return cases

if __name__=='__main__':
 cases=checks();print(json.dumps({'passed':len(cases),'cases':cases},indent=2))
