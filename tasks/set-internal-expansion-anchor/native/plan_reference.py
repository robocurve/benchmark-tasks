"""Builds geometry/full_plan.json: the scripted reference's nominal joint targets at 20 Hz.

Pure kinematics on the stock arm model (no simulation, no object motion). Run from the task folder:

    python native/plan_reference.py

The left hand steadies the punch with a level side grasp on its upper section. The right hand takes the hammer
handle from above, then rolls the tool a quarter turn about its handle so the striking face points down and the
two finger pads sit above and below the handle: a blow's reaction is then carried by a pad face instead of by pad
friction. Afterwards the hammer is rolled back and laid in its cradle, and the punch is lifted out and laid on its
two V blocks.
"""
from pathlib import Path
import sys,json,hashlib
import numpy as np
HERE=Path(__file__).resolve().parent
from yam_arena import ik
from yam_arena.constants import READY

GRASP_X=.225            # hammer grasp point along the handle from its free end (m); the head centre is at .265
HEAD_X=.265
FACE=np.array([HEAD_X,-.030,0.])   # centre of the striking face used, in the hammer frame
HAMMER_GRIP=.045        # normalised finger target on the 20 mm handle: 7.9 mm of squeeze, about 39 N per pad
LIFT=.080               # height the hammer is carried above its cradle (m)
PUNCH_Z=.100            # punch grasp height above the punch origin (m), on the 12 mm upper section
STRIKES=5               # most blows the reference may use
STROKE=.100             # height the head falls through in a blow (m)
FOLLOW=.003             # commanded travel past first contact (m)
STRIKE_CONTROLS=8
# Hand frames (columns: finger-closing axis, pad-width axis, approach axis) in the world.
DOWN=np.array([[0,-1,0],[-1,0,0],[0,0,-1]],float)       # straight down, fingers closing along world y
SIDE=np.array([[-1,0,0],[0,0,-1],[0,-1,0]],float)       # level, approaching along -y, fingers closing along x
TOOL=np.column_stack([[1.,0,0],[0,0,1.],[0,-1.,0]])     # hammer when striking: handle along +x, face (-y) down
STRIKE_HAND=TOOL@DOWN                                   # the same grasp after the quarter roll: approach along +y
PUNCH_TOP=.1444875
STOW=np.array([.186,.25,.790])                          # punch origin when it lies on the V blocks, axis along +x

def build(G):
 base={s:np.array(p,float)for s,p in G['robot_bases_m'].items()}
 q={s:np.array(READY,float)for s in base};f={'left':1.,'right':1.};rows=[]
 def record(phase):rows.append(dict(control=len(rows),phase=phase,left_q=q['left'].tolist(),right_q=q['right'].tolist(),left_grip=f['left'],right_grip=f['right']))
 def solve(side,p,R,seed,iters=300):
  end,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),seed,base_pos=base[side],iters=iters,tol=1e-6,damping=.015)
  if pe>.001 or re>.01:raise RuntimeError(('IK',side,np.asarray(p).tolist(),pe,re))
  return end
 def jointmove(side,endpoint,n,phase):
  start=q[side].copy()
  for i in range(1,n+1):q[side]=start+(endpoint-start)*ik.smoothstep(i/n);record(phase)
 def grip(side,end,n,phase):
  start=f[side]
  for i in range(1,n+1):f[side]=start+(end-start)*i/n;record(phase)
 def cart(side,target,R,n,phase):
  start,R0=ik.tcp_pose(q[side],base[side])
  for i in range(1,n+1):
   u=ik.smoothstep(i/n);q[side]=solve(side,start+(np.asarray(target)-start)*u,ik.slerp_matrix(R0,R,u),q[side],160);record(phase)
 def hold(n,phase):
  for _ in range(n):record(phase)
 origin=np.array(G['frame_origin_m'])
 punch=origin+[0,0,PUNCH_Z]
 seed=np.array([-.25,1.95,.62,1.3,1.3,0.])
 jointmove('left',solve('left',punch-SIDE[:,2]*.060,SIDE,seed),50,'punch_hover');cart('left',punch,SIDE,50,'punch_approach');grip('left',0.,30,'punch_close')
 hammer=next(p for p in G['parts']if p['name']=='hammer');Hg=np.array(hammer['pos'])+[GRASP_X,0,0]
 jointmove('right',solve('right',Hg+[0,0,.07],DOWN,q['right']),50,'hammer_hover');cart('right',Hg,DOWN,50,'hammer_approach')
 grip('right',HAMMER_GRIP,30,'hammer_close');cart('right',Hg+[0,0,LIFT],DOWN,50,'hammer_lift')
 contact=origin+[0,0,PUNCH_TOP]-TOOL@(FACE-[GRASP_X,0,0])
 hover=contact+[0,0,STROKE]
 cart('right',hover,STRIKE_HAND,110,'hammer_transit')
 for n in range(STRIKES):
  hold(30,f'align_{n}')
  cart('right',contact-[0,0,FOLLOW],STRIKE_HAND,STRIKE_CONTROLS,f'strike_{n}')
  hold(6,f'strike_settle_{n}')
  cart('right',hover,STRIKE_HAND,30,f'relift_{n}')
 hold(20,'seat_check')
 cart('right',Hg+[0,0,LIFT],DOWN,110,'hammer_return_high');hold(20,'hammer_place_align');cart('right',Hg+[0,0,.001],DOWN,50,'hammer_return')
 grip('right',1.,20,'hammer_release');cart('right',Hg+[0,0,LIFT],DOWN,40,'hammer_retreat')
 cart('left',punch+[0,0,.060],SIDE,60,'punch_withdraw')
 pst=STOW+[PUNCH_Z,0,0]
 jointmove('left',solve('left',pst+[0,0,.080],DOWN,np.array([-.2,1.6,1.3,-1.28,0.,-.2])),80,'punch_stow_high')
 cart('left',pst,DOWN,50,'punch_lower_to_vrest');grip('left',1.,25,'punch_release');cart('left',pst+[0,0,.080],DOWN,40,'punch_retreat')
 hold(40,'released_terminal_hold')
 return rows

if __name__=='__main__':
 root=HERE.parent/'geometry';G=json.load(open(root/'geometry.json'))
 rows=build(G)
 plan=dict(status='nominal joint targets; object motion comes only from simulated contact',rows=rows,
           source_geometry_sha256=hashlib.sha256((root/'geometry.json').read_bytes()).hexdigest())
 (root/'full_plan.json').write_text(json.dumps(plan,indent=1)+'\n')
 phases={}
 for r in rows:phases[r['phase']]=phases.get(r['phase'],0)+1
 print('rows',len(rows));print(phases)
