"""Cold policy integration check; does not instantiate an environment."""
import sys,json
from pathlib import Path
import numpy as np
from types import SimpleNamespace
sys.path.insert(0,'/home/freax/Documents/GitHub/yam-isaac-lab-arena')
from yam_arena.task_file import load
p=Path(__file__).parent
loaded=load(p/'task.py')
from yam_arena.scripted import YamScriptedPolicy
policy=YamScriptedPolicy();policy._task=loaded;policy._dt=.05
policy.reset(SimpleNamespace(instruction=loaded.instruction if hasattr(loaded,'instruction')else'',target=None))
actions=[]
for k in range(3):
 out=policy.act(SimpleNamespace(state_time=k*.05));data=np.asarray(out.actions[0].data)
 assert data.shape==(14,) and np.isfinite(data).all();actions.append(data.tolist())
early=[n for n in sys.modules if n=='pxr'or n.startswith(('pxr.','omni.','isaaclab.'))]
r=dict(stage='COLD_POLICY_PASS',task_objects=len(loaded.objects),normal_actions=len(actions),action_dimension=14,pre_app_native_modules=early,physics_executed=False,limits='Only first three normal target actions checked; no native contact or complete-path claim')
assert not early
(p/'CPU-POLICY-RESULT.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
