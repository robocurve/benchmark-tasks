"""One official Dream episode, capped at 200 get_action attempts including retries.

Invoke under the host GPU lock after confirming adequate free account credit.
This guard limits requests; it neither purchases credit nor changes billing.
"""
import json,os,sys
from pathlib import Path
import dream
from inspect_robots.cli import main
TASK=Path(__file__).resolve().parent
RUN=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else TASK/'runs/dream-01'
RUN.mkdir(parents=True,exist_ok=False)
count=0
original=dream.DreamClient.get_action

def limited(self,*args,**kwargs):
    global count
    if count>=200:
        raise RuntimeError('Task call budget exhausted: no more Dream requests permitted')
    count+=1
    (RUN/'request_budget.json').write_text(json.dumps({'attempts_reserved':count,'max_attempts':200,'nominal_usd_per_call':.0014,'estimated_usd_reserved':count*.0014,'includes_failed_attempts':True},indent=2)+'\n')
    return original(self,*args,**kwargs)
dream.DreamClient.get_action=limited
for name in ('BAGTAG_STAGE_STATE','BAGTAG_STAGE_STEP','BAGTAG_NEGATIVE','BAGTAG_RENDER_STRIDE','BAGTAG_HINGE_K','BAGTAG_HINGE_C'):
    os.environ.pop(name,None)
meta=json.loads((TASK/'task.json').read_text())
sys.argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream',
 '--policy-checkpoint','dreamlabs/dream-1-beta','-P','timeout_s=90',
 '-E',f'task={TASK/"task.py"}','-E','object_jitter=0','-E','control_hz=20',
 '-E','cam_width=320','-E','cam_height=176','-E','top_cam=demo',
 '--instruction',meta['instruction'],'--scorer','task_score','--max-steps','4800',
 '--store-frames','--seed','42','--no-prompt','--grader','none','--no-rerun','--log-dir',str(RUN)]
sys.exit(main())
