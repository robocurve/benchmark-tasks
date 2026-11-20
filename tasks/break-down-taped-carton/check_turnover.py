"""Bounded CPU reach/full315 screen, not physical turnover proof.

All Cartesian poses below are predictions for planning; no actor is restored or
moved. Failed multi-start IK is reported, never replaced by an attained pose.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix
from turnover import rx,push_target,brace_target,solve_target
from turnover_safety import Geometry,BASES
from scene import layout
from config import CENTER,TOP,CUTTER_START,CUTTER_PAD_START
from model import WIDTH,HEIGHT

ANGLES=(0,10,20,25,30,45,60,75,88,90,100,120,140,160,178,180)


def parked_cutter():
 result={}
 for name,size,offset in [('handle',(.1,.025,.025),(0,0,0)),('guard',(.02,.03,.02),(.052,0,.002)),('blade',(.014,.0005,.008),(.062,0,-.016))]:
  result['cutter_'+name]=(np.array(CUTTER_START)+offset,np.eye(3),size)
 result['cutter_pad']=(np.array(CUTTER_PAD_START),np.eye(3),(.17,.07,.002))
 return result


def guard_pair(geometry,qs,fingers,obs,contact):
 guards=[]
 for i,side in enumerate(('left','right')):
  other=1-i;pieces=geometry.parts(('left','right')[other],qs[other],fingers[other])
  g=geometry.guard(side,qs[i],fingers[i],obs,pieces,intended=bool(contact[i]),intended_targets=contact[i]);guards.append(g)
 return guards


def check(center=None,left_roll=0.,right_roll=0.,starts=10):
 center=np.array(CENTER[:2] if center is None else center)
 geometry=Geometry();parts=layout()[0]
 raw={p['name']:(np.array(p['pos']),quat_xyzw_to_matrix([*p['quat'][1:],p['quat'][0]]),p['size']) for p in parts}
 corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
 vertices=np.concatenate([p+(corners*np.array(size)/2)@Q.T for p,Q,size in raw.values()])
 front=max(float(t[:,:,2].max()) for _,n,t in geometry.local_hand((.012,.012)) if n.startswith('tip_'))
 q=[np.array(READY),np.array(READY)];records=[];park=ik.tcp_pose(np.array(READY),BASES['left'])
 for angle in ANGLES:
  R=rx(-math.radians(angle));rotated=vertices@R.T;a=math.radians(angle)
  y=center[1]+WIDTH/2+.003-math.cos(a)*(WIDTH/2+.003)+math.sin(a)*(-.0015) if angle<=90 else center[1]+WIDTH/2+.003+HEIGHT-math.cos(a)*(WIDTH/2+.0045)-math.sin(a)*HEIGHT
  origin=np.array([center[0],y,TOP+.0002-rotated[:,2].min()])
  obs={n:(origin+R@p,R@Q,size) for n,(p,Q,size) in raw.items()};obs.update(parked_cutter())
  right=push_target(origin,R,'south' if angle<90 else 'bottom',front,.0002,roll_deg=right_roll)
  left=brace_target(origin,R,front,.0002,roll_deg=left_roll) if angle<=25 else park
  row=dict(angle_deg=angle,center_xy_m=center.tolist());records.append(row);sol=[]
  for i,(side,(p,Q)) in enumerate(zip(('left','right'),(left,right))):
   if side=='left' and angle>25:q[i]=np.array(READY);pe=re=0.
   else:q[i],pe,re=solve_target(p,Q,q[i],BASES[side],starts=starts)
   sol.append(dict(side=side,pe=float(pe),re=float(re),q=q[i].tolist(),target=p.tolist(),rotation=Q.tolist(),joint_margin_deg=float(np.degrees(np.minimum(q[i]-ik.LOWER,ik.UPPER-q[i]).min()))))
  row['IK']=sol
  if max(s['pe'] for s in sol)>.002 or max(s['re'] for s in sol)>.03:
   row.update(safe=False,reason='bounded_multistart_IK_failed');print(json.dumps({k:v for k,v in row.items() if k!='guard'}),flush=True);continue
  contact=[('north','north_top') if angle<=25 else (),('south','south_top') if angle<90 else ('north_bottom','south_bottom','east_bottom','west_bottom')]
  guards=guard_pair(geometry,q,[[-.012]*2]*2,obs,contact)
  row['guard']=guards;row['safe']=all(g['safe'] and g['target_gap_m']>=-.002 for g in guards)
  row['reason']='nominal_only_pass' if row['safe'] else 'full315_guard_failed'
  print(json.dumps({k:v for k,v in row.items() if k!='guard'}),flush=True)
 # A future path is not native-ready until every contact posture is reachable
 # and clear. Do not waste a full READY-path screen from a failed contact plan.
 if all(r['safe'] for r in records):
  start=q[1].copy();p,Q=ik.tcp_pose(start,BASES['right']);lifted=p-Q[:,2]*.12
  clear,pe,re=solve_target(lifted,Q,start,BASES['right'],starts=starts)
  checks=[]
  if pe<=.002 and re<=.03:
   for fraction in np.linspace(0,1,31):
    qq=[np.array(READY),clear+float(ik.smoothstep(fraction))*(np.array(READY)-clear)]
    fingers=[[-.04695]*2,[(-.012+(FINGER_OPEN+.012)*ik.smoothstep(fraction))]*2]
    gg=guard_pair(geometry,qq,fingers,obs,[(),()]);checks.append(all(g['safe'] for g in gg))
  records.append(dict(stage='READY_return_nominal',safe=bool(checks and all(checks)),right_clear_ik_m=pe,right_clear_ik_rad=re,samples=len(checks),reason='Not continuous-clearance or native proof'))
 return records

def self_test():
 """Focused control invariants; mocked guard is NOT a geometry pass."""
 import unittest
 from types import SimpleNamespace
 from turnover import CartonTurnoverProbe,roll_wrist
 class Tensor:
  def __init__(self,value):self.value=np.asarray(value)
  def detach(self):return self
  def cpu(self):return self
  def numpy(self):return self.value
 class Tests(unittest.TestCase):
  def test_multistart_keeps_exact_known_reachable_pose(self):
   p,R=ik.tcp_pose(READY,BASES['right']);q,pe,re=solve_target(p,R,np.array(READY),BASES['right'],starts=4)
   self.assertLess(pe,1e-8);self.assertLess(re,1e-8);np.testing.assert_array_equal(q,READY)
  def test_polarity_changes_only_tangent_axes_not_push_normal(self):
   R=rx(.3)
   for roll in (0.,90.,180.,-90.):
    Q=roll_wrist(R,roll);np.testing.assert_allclose(Q[:,2],R[:,2]);np.testing.assert_allclose(Q.T@Q,np.eye(3),atol=1e-14)
  def test_ready_entry_and_done_require_guarded_exact_issued_ready(self):
   probe=CartonTurnoverProbe.__new__(CartonTurnoverProbe);parts=layout()[0];probe.parts={p['name']:p for p in parts}
   R=rx(-math.pi);origin=np.array([CENTER[0],CENTER[1],TOP+HEIGHT+.0015]);poses={p['name']:(origin+R@np.array(p['pos']),R@quat_xyzw_to_matrix([*p['quat'][1:],p['quat'][0]])) for p in parts}
   ready=np.r_[READY,1.,READY,1.];start=ready.copy();start[[0,7]]+=.1;start[[6,13]]=.012/abs(FINGER_OPEN)
   probe.previous=start.copy();probe.failure=None;probe.done=False;probe.stage='return_ready';probe.started=0.;probe.starts=[(np.zeros(3),np.eye(3))]*2
   from turnover_targets import tip_vertices
   probe.tip_vertices=tip_vertices(Geometry());probe.report={};probe.last_raw=probe.angle=probe.best_angle=180.;probe.last_progress=0.;probe.bias=[np.zeros(3)]*2;probe.front=.014;probe.ready_start=None;probe.ready_stable=0
   probe.poses=lambda:poses
   def measure():
    return [(np.array(READY),np.array([FINGER_OPEN]*2),*ik.tcp_pose(READY,BASES[side])) for side in ('left','right')]
   probe.measure=measure
   class Guard:
    safe=True
    def parts(self,*a):return []
    def guard(self,*a,**kw):return dict(safe=self.safe,target_gap_m=10.,hard_gap_m=10.,other_arm_gap_m=10.)
   guard=Guard();probe.geometry=guard
   world=SimpleNamespace(get_pose_w=lambda name:Tensor([[*CUTTER_START,0,0,0,1]]))
   probe.env=SimpleNamespace(_ready=Tensor([ready]),rig=SimpleNamespace(bases=BASES),_low=np.full(14,-10.),_high=np.full(14,10.),_base=SimpleNamespace(arena_world=world))
   action,report=probe.command(0.);np.testing.assert_array_equal(action,start);self.assertFalse(probe.done)
   # Even if actual joints are already READY, completion needs both a valid
   # geometric check and the actual last-issued READY command.
   guard.safe=False;before=action.copy();action,report=probe.command(4.)
   np.testing.assert_array_equal(action,before);self.assertFalse(probe.done);self.assertIn('clearance',probe.failure)
   probe.failure=None;probe.done=False;probe.previous=start.copy();probe.ready_start=None;probe.ready_stable=0;guard.safe=True
   for i in range(90):
    action,report=probe.command(i*.05)
    if probe.done:break
   self.assertTrue(probe.done);np.testing.assert_array_equal(action,ready)
 return unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests)).wasSuccessful()


if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--self-test',action='store_true');parser.add_argument('--center',nargs=2,type=float);parser.add_argument('--left-roll',type=float,default=0.);parser.add_argument('--right-roll',type=float,default=0.);parser.add_argument('--starts',type=int,default=10);parser.add_argument('--out',type=Path,default=Path('turnover-cpu-screen.json'));args=parser.parse_args()
 if args.self_test:raise SystemExit(0 if self_test() else 1)
 if not 1<=args.starts<=20:parser.error('starts must be1..20')
 result=check(args.center,args.left_roll,args.right_roll,args.starts);args.out.write_text(json.dumps(result,indent=2)+'\n');print('PASS',sum(r['safe'] for r in result),'/',len(result))
