import argparse,json
from pathlib import Path
import numpy as np
from prepare_geometry import fk
from yam_arena import ik,station
from scorer import Scorer
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--output',type=Path);a=p.parse_args();run=Path(a.run)
rows=json.loads((run/'trace.json').read_text());policy=None
if (run/'policy_actions.npz').exists():policy=np.load(run/'policy_actions.npz')['measured_states']
s=Scorer();result=[];prev=-5
for r in rows:
 k=r['step']
 if 'left_actual_q' in r:
  lq=r['left_actual_q'];rq=r['right_actual_q'];gap=-sum(r['right_actual_fingers'])
 else:
  state=policy[min(k,len(policy)-1)];lq=state[:6];rq=state[7:13];gap=state[13]*.0939
 T=fk(lq)['link3'];tcp,_=ik.tcp_pose(rq,station.rig().bases['right'])
 result.append(dict(step=k,**s.update(r['cable'],r['flap'],T,tcp,gap,(k-prev)/20)));prev=k
summary={'score':result[-1]['score'],'final':result[-1],'steps_observed':rows[-1]['step'],'rubric':'routing/enclosure/released support-free hold'}
(a.output or run/'score_replay.json').write_text(json.dumps({'summary':summary,'samples':result},indent=2));print(json.dumps(summary,indent=2))
