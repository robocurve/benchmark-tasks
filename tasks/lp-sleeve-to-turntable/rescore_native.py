"""Replay the scorer on measured native states; never executes or replaces physics."""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

p=argparse.ArgumentParser();p.add_argument('task',type=Path);p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
spec=importlib.util.spec_from_file_location('lp_score',a.task/'task.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
task=module.LPTask(a.task);receipt=json.loads((a.run/'receipt.json').read_text());rows=[json.loads(s) for s in (a.run/'trace.jsonl').read_text().splitlines()]
def states(objects):
 return {k:(np.array(v[0]),quat_xyzw_to_matrix(np.array(v[1])),np.array(v[2])) for k,v in objects.items()}
prev=states(receipt['object_reset_state']);held=0;partial=0.;first_unsafe=None;first_full=None;last=None
for row in rows:
 travel=task.cfg['button_rest_z']-prev['player/button'][0][2]
 if travel>=.003 and not task._pressed:
  task._pressed=True;task._motor=not task._motor;task._qualified_start=bool(task._motor and task._cue_valid);task._start_radius=task._cue_radius if task._qualified_start else None
 elif travel<.001:task._pressed=False
 now=states(row['objects'])
 def tip(s):return s['player/tonearm'][0]+s['player/tonearm'][1]@np.array(task.cfg['stylus_local'])
 tv=(tip(now)-tip(prev))/.05
 motion=(-module.rotation_speed(prev['record'][1],now['record'][1],.05)*60/(2*math.pi),-module.rotation_speed(prev['player/platter'][1],now['player/platter'][1],.05)*60/(2*math.pi),float(np.linalg.norm(tv[:2])))
 last=task.score_states(now,list(row['tcp'].values()),motion,tv)
 assert last['motor_on']==row['metrics']['motor_on'],('motor replay differs',row['step'])
 held=held+1 if last['success'] else 0
 if last['score']<1:partial=last['score']
 if last['unsafe_event'] and first_unsafe is None:first_unsafe=row['step']
 if held>=math.ceil(task.hold_s*20) and first_full is None:first_full=row['step']
 prev=now
result=dict(scope='Scorer replay of unchanged measured native states; not a new physics run',run=a.run.name,native_revision=receipt['git_revision'],scorer_sha256=hashlib.sha256((a.task/'task.py').read_bytes()).hexdigest(),trace_sha256=hashlib.sha256((a.run/'trace.jsonl').read_bytes()).hexdigest(),controls=len(rows),first_unsafe_step=first_unsafe,held_success_controls_at_end=held,first_full_score_step=first_full,score=1. if held>=task.hold_s*20 else partial,final_metrics=last)
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='final_metrics'},indent=2))
